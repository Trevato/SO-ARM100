import type { FeetechBus } from "./bus.ts";
import { JOINTS, STS3215 } from "./sts3215.ts";
import { largestStep, type Trajectory } from "./trajectory.ts";

export interface SupervisorOptions {
  /** Torque cap while supervised, in percent of the servo's maximum. */
  torquePercent?: number;
  /** Targets are pulled this many ticks inside each servo's limits so a move never drives into a hard stop. */
  marginTicks?: number;
  /** How close a joint must get to count as arrived, in ticks. */
  tolerance?: number;
  /** Stops the motion where it is when aborted (wire this to Ctrl-C). */
  signal?: AbortSignal;
}

export interface Poll {
  readonly positions: ReadonlyMap<number, number>;
  /** Joints that pushed hard without net progress across the whole window: a collision, seen from the bus. */
  readonly stalled: readonly number[];
  /** Joints that stopped answering. */
  readonly missing: readonly number[];
  /** Every joint within tolerance of its target. */
  readonly arrived: boolean;
}

export class MotionError extends Error {
  readonly stalled: readonly number[];
  readonly missing: readonly number[];

  constructor(message: string, stalled: readonly number[] = [], missing: readonly number[] = []) {
    super(message);
    this.name = "MotionError";
    this.stalled = stalled;
    this.missing = missing;
  }
}

const STALL_WINDOW = 6;
// Net travel across the window below this is "not moving": a jammed joint chattering a few ticks against an obstacle still counts.
export const STALL_SPAN_TICKS = 8;

/**
 * Keeps a set of joints safe while something else decides where they go:
 * reads and sanity-checks their limits, caps torque, keeps targets inside a
 * margin, and watches load and progress so a jammed joint stops everything
 * instead of grinding.
 */
export class JointSupervisor {
  readonly ids: readonly number[];
  readonly #bus: FeetechBus;
  readonly #torquePercent: number;
  readonly #margin: number;
  readonly #tolerance: number;
  readonly #signal: AbortSignal | undefined;
  #limits: [number, number][] = [];
  /** Each servo's own ceiling (Max_Torque_Limit), put back afterward: a cap left by an earlier grip or hold is not "original". */
  readonly #ceilings = new Map<number, number>();
  /** The cap actually in force per servo: the requested percent, or less if the servo's Max_Torque_Limit is lower. */
  readonly #caps = new Map<number, number>();
  readonly #windows = new Map<number, number[]>();
  readonly #pushing = new Map<number, number>();

  constructor(bus: FeetechBus, ids: readonly number[], options: SupervisorOptions = {}) {
    this.#bus = bus;
    this.ids = ids;
    this.#torquePercent = options.torquePercent ?? 70;
    this.#margin = options.marginTicks ?? 30;
    this.#tolerance = options.tolerance ?? 15;
    this.#signal = options.signal;
  }

  get limits(): readonly (readonly [number, number])[] {
    return this.#limits;
  }

  /**
   * Read limits, cap torque, and enable torque without a lunge: the goal is
   * set to the present position and the velocity made gentle before the
   * servo is allowed to act, so a stale goal from an earlier run is harmless.
   * Call once before moving.
   */
  async arm(acceleration = 30): Promise<ReadonlyMap<number, number>> {
    if (!uniqueIds(this.ids)) throw new MotionError("servo ids must be distinct");
    this.#limits = [];
    for (const id of this.ids) {
      const min = await this.#bus.read(id, STS3215.Min_Position_Limit);
      const max = await this.#bus.read(id, STS3215.Max_Position_Limit);
      if (min >= max) throw new MotionError(`${name(id)} has limits ${min}-${max}: not calibrated, refusing to move it`);
      this.#limits.push([min, max]);
      const ceiling = await this.#bus.read(id, STS3215.Max_Torque_Limit);
      this.#ceilings.set(id, ceiling);
      this.#caps.set(id, Math.min(Math.round(this.#torquePercent * 10), ceiling));
    }
    const start = await this.#bus.syncRead(this.ids, STS3215.Present_Position);
    const missing = this.ids.filter((id) => !start.has(id));
    if (missing.length) throw new MotionError(`no response from ${missing.map(name).join(", ")}`, [], missing);
    for (const id of this.ids) {
      this.#windows.set(id, [start.get(id)!]);
      this.#pushing.set(id, 0);
    }
    await this.#bus.syncWrite(STS3215.Goal_Position, this.ids.map((id) => [id, start.get(id)!] as const));
    await this.#bus.syncWrite(STS3215.Goal_Velocity, this.ids.map((id) => [id, 20] as const));
    await this.#bus.syncWrite(STS3215.Acceleration, this.ids.map((id) => [id, acceleration] as const));
    await this.#bus.syncWrite(STS3215.Torque_Limit, this.ids.map((id) => [id, this.#caps.get(id)!] as const));
    await this.#bus.syncWrite(STS3215.Torque_Enable, this.ids.map((id) => [id, 1] as const));
    return start;
  }

  /** Pull targets inside the limit margin; a range too narrow for the margin aims at its middle. */
  clamp(targets: readonly number[]): { aims: number[]; adjusted: Map<number, number> } {
    if (targets.length !== this.ids.length || !targets.every(Number.isInteger)) {
      throw new MotionError(`expected ${this.ids.length} integer targets, got ${targets.length}`);
    }
    const adjusted = new Map<number, number>();
    const aims = targets.map((target, i) => {
      const [min, max] = this.#limits[i]!;
      const aim =
        max - min < 2 * this.#margin
          ? Math.round((min + max) / 2)
          : Math.min(max - this.#margin, Math.max(min + this.#margin, target));
      if (aim !== target) adjusted.set(this.ids[i]!, aim);
      return aim;
    });
    return { aims, adjusted };
  }

  /** Write one set of goals (already clamped) and per-joint velocities. */
  async goal(aims: readonly number[], velocities: readonly number[]): Promise<void> {
    await this.#bus.syncWrite(STS3215.Goal_Velocity, this.ids.map((id, i) => [id, velocities[i]!] as const));
    await this.#bus.syncWrite(STS3215.Goal_Position, this.ids.map((id, i) => [id, aims[i]!] as const));
  }

  /** One observation. Throws MotionError after stopping everything if a joint stalls, vanishes, or the signal aborts. */
  async poll(aims: readonly number[]): Promise<Poll> {
    const positions = await this.#bus.syncRead(this.ids, STS3215.Present_Position);
    const loads = await this.#bus.syncRead(this.ids, STS3215.Present_Load);
    const missing = this.ids.filter((id) => !positions.has(id) || !loads.has(id));
    if (missing.length) {
      await this.stop(positions);
      throw new MotionError(`${missing.map(name).join(", ")} stopped answering mid-move; stopped and released it`, [], missing);
    }
    const stalled: number[] = [];
    let arrived = true;
    for (const [i, id] of this.ids.entries()) {
      const pos = positions.get(id)!;
      const away = Math.abs(pos - aims[i]!);
      if (away > this.#tolerance) arrived = false;
      const window = this.#windows.get(id)!;
      window.push(pos);
      if (window.length > STALL_WINDOW) window.shift();
      const span = Math.max(...window) - Math.min(...window);
      const pushing = Math.abs(loads.get(id)!) >= this.#caps.get(id)! * 0.85; // Present_Load is in 0.1 %
      const count = away > this.#tolerance && pushing ? (this.#pushing.get(id) ?? 0) + 1 : 0;
      this.#pushing.set(id, count);
      if (count >= STALL_WINDOW && window.length >= STALL_WINDOW && span < STALL_SPAN_TICKS) stalled.push(id);
    }
    if (stalled.length) {
      await this.stop(positions);
      throw new MotionError(`${stalled.map(name).join(", ")} jammed; stopped everything where it is`, stalled);
    }
    if (this.#signal?.aborted) {
      await this.stop(positions);
      throw new MotionError("interrupted; stopped everything where it is");
    }
    return { positions, stalled, missing, arrived };
  }

  /** Make every answering joint's goal its present position, release any that do not answer, restore the torque cap. */
  async stop(positions: ReadonlyMap<number, number>): Promise<void> {
    const known = this.ids.filter((id) => positions.has(id));
    if (known.length) await this.#bus.syncWrite(STS3215.Goal_Position, known.map((id) => [id, positions.get(id)!] as const));
    const unknown = this.ids.filter((id) => !positions.has(id));
    if (unknown.length) await this.#bus.syncWrite(STS3215.Torque_Enable, unknown.map((id) => [id, 0] as const));
    await this.release();
  }

  /** Put each servo's own torque cap (its Max_Torque_Limit) back; torque itself stays on. */
  async release(): Promise<void> {
    await this.#bus.syncWrite(STS3215.Torque_Limit, this.ids.map((id) => [id, this.#ceilings.get(id) ?? 1000] as const));
  }
}

export interface MoveOptions extends SupervisorOptions {
  /** Ticks per second for the joint that has the farthest to go; the others are slowed to arrive with it. */
  speed?: number;
  /** Poll interval in ms. */
  pollMs?: number;
}

export interface MoveResult {
  readonly final: ReadonlyMap<number, number>;
  /** Joints whose target was pulled inside the limit margin, with the position actually aimed at. */
  readonly adjusted: ReadonlyMap<number, number>;
}

/**
 * Drive several joints to a pose at once under supervision. Velocities are
 * scaled so every joint arrives together. Resolves when all are within
 * tolerance; throws MotionError (after stopping) on a jam, a lost servo, an
 * abort, or a timeout. Torque stays on afterward.
 */
export async function moveToPose(
  bus: FeetechBus,
  ids: readonly number[],
  targets: readonly number[],
  { speed = 300, pollMs = 50, ...options }: MoveOptions = {},
): Promise<MoveResult> {
  if (!(speed >= 1 && Number.isFinite(speed))) throw new MotionError(`speed must be a positive number, got ${speed}`);
  if (targets.length !== ids.length || !targets.every(Number.isInteger)) {
    throw new MotionError(`expected ${ids.length} integer targets, got ${targets.length}`);
  }
  const supervisor = new JointSupervisor(bus, ids, options);
  const start = await supervisor.arm();
  const { aims, adjusted } = supervisor.clamp(targets);
  const deltas = ids.map((id, i) => Math.abs(aims[i]! - start.get(id)!));
  const farthest = Math.max(...deltas, 1);
  const seconds = farthest / speed;
  const velocities = deltas.map((d) => Math.max(20, Math.round(d / seconds)));
  await supervisor.goal(aims, velocities);

  const deadline = Date.now() + (seconds + 3) * 1000;
  for (;;) {
    await sleep(pollMs);
    const poll = await supervisor.poll(aims);
    if (poll.arrived) {
      // Settle on where it actually is, so a joint stopped a few ticks short by an obstacle is not pushed at full torque once the cap comes off.
      await supervisor.stop(poll.positions);
      return { final: poll.positions, adjusted };
    }
    if (Date.now() > deadline) {
      await supervisor.stop(poll.positions);
      throw new MotionError("timed out before arriving; stopped everything where it is");
    }
  }
}

export const name = (id: number): string => JOINTS[id] ?? `servo ${id}`;
export const uniqueIds = (ids: readonly number[]): boolean => new Set(ids).size === ids.length;
export const sleep = (ms: number): Promise<void> => new Promise((resolve) => setTimeout(resolve, ms));

/** A hand-made recording never jumps more than this between two frames; a file that does is refused. */
export const MAX_FRAME_STEP = 400;
/** Ceiling on the velocity asked of any joint while a recording plays, ticks/s. */
export const MAX_PLAY_VELOCITY = 1500;

export interface RecordOptions {
  hz?: number;
  /** Ends the recording. */
  signal: AbortSignal;
  onFrame?: (count: number) => void;
}

/**
 * Release torque and sample the joints until the signal fires. A missed reply
 * is filled from the previous frame so the recording stays dense; how many
 * frames needed that is reported alongside.
 */
export async function recordTrajectory(
  bus: FeetechBus,
  ids: readonly number[],
  { hz = 30, signal, onFrame }: RecordOptions,
): Promise<{ trajectory: Trajectory; filled: number }> {
  await bus.syncWrite(STS3215.Torque_Enable, ids.map((id) => [id, 0] as const));
  const frames: number[][] = [];
  let last: number[] | null = null;
  let filled = 0;
  const period = 1000 / hz;
  let next = performance.now();
  while (!signal.aborted) {
    const positions = await bus.syncRead(ids, STS3215.Present_Position);
    if (positions.size < ids.length && last) filled++;
    const frame = ids.map((id, i) => positions.get(id) ?? last?.[i] ?? Number.NaN);
    if (frame.every(Number.isFinite)) {
      frames.push(frame);
      last = frame;
      onFrame?.(frames.length);
    }
    next += period;
    const wait = next - performance.now();
    if (wait > 0) await sleep(wait);
  }
  return { trajectory: { hz, ids: [...ids], frames, recorded: new Date().toISOString() }, filled };
}

export interface PlayOptions {
  /** Playback speed multiplier, within 0-2. */
  rate?: number;
  torquePercent?: number;
  signal?: AbortSignal;
  onFrame?: (index: number) => void;
}

/**
 * Replay a recording under supervision: ease to its first frame, then stream
 * the frames with each joint given only the speed the recording asks of it,
 * so a dropped sample cannot become a lunge. Resolves with the number of
 * frames played; throws MotionError on a jam, a lost servo, an abort, or a
 * recording with an impossible jump. Torque stays on afterward.
 */
export async function playTrajectory(
  bus: FeetechBus,
  trajectory: Trajectory,
  { rate = 1, torquePercent = 70, signal, onFrame }: PlayOptions = {},
): Promise<number> {
  const step = largestStep(trajectory);
  if (step > MAX_FRAME_STEP) {
    throw new MotionError(`the recording jumps ${step} ticks between two frames; a hand-made recording never does that, so refusing to replay it`);
  }
  if (!(rate > 0 && rate <= 2)) throw new MotionError(`rate must be within 0-2, got ${rate}`);
  const ids = [...trajectory.ids];
  await moveToPose(bus, ids, trajectory.frames[0]!, { speed: 300, torquePercent, signal });
  const supervisor = new JointSupervisor(bus, ids, { torquePercent, signal, tolerance: 4096 });
  let previous = [...(await supervisor.arm(254)).values()];
  const period = 1000 / (trajectory.hz * rate);
  const started = performance.now();
  let index = 0;
  for (; index < trajectory.frames.length; index++) {
    const { aims } = supervisor.clamp(trajectory.frames[index]!);
    const velocities = aims.map((aim, i) => Math.min(MAX_PLAY_VELOCITY, Math.max(20, Math.round(Math.abs(aim - previous[i]!) * trajectory.hz * rate * 1.5))));
    await supervisor.goal(aims, velocities);
    previous = aims;
    await supervisor.poll(aims);
    onFrame?.(index + 1);
    const wait = started + (index + 1) * period - performance.now();
    if (wait > 0) await sleep(wait);
  }
  await supervisor.release();
  return index;
}
