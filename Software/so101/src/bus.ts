import { SerialPort } from "serialport";
import {
  BROADCAST_ID,
  FrameParser,
  Instruction,
  bytesEqual,
  encodeInstruction,
  type StatusPacket,
} from "./protocol.ts";
import { STS3215, decodeValue, encodeValue, type Register } from "./sts3215.ts";

export interface OpenOptions {
  /** STS3215s ship at 1 Mbaud; LeRobot keeps them there. */
  baudRate?: number;
}

export interface BusOptions {
  /** The adapter hands our own bytes back to us. `open()` detects this; tests set it. */
  echo?: boolean;
}

/**
 * The slice of a serialport `SerialPort` the bus needs. Anything with this
 * shape can stand in for the wire, which is how the tests run without servos.
 */
export interface Transport {
  readonly path: string;
  readonly baudRate: number;
  write(data: Uint8Array, callback: (error: Error | null | undefined) => void): unknown;
  drain(callback: (error: Error | null | undefined) => void): unknown;
  close(callback: (error: Error | null | undefined) => void): unknown;
  on(event: "data", listener: (chunk: Uint8Array) => void): unknown;
  on(event: "error", listener: (error: Error) => void): unknown;
}

export class NoResponseError extends Error {
  readonly id: number;

  constructor(id: number, what: string) {
    super(`no response from servo ${id} (${what})`);
    this.name = "NoResponseError";
    this.id = id;
  }
}

interface Pending {
  readonly remaining: Set<number>;
  readonly frames: StatusPacket[];
  /** How many replies complete the transaction; fewer than `remaining` when any one will do. */
  readonly need: number;
  readonly finish: (timedOut: boolean) => void;
  readonly fail: (error: Error) => void;
  /** The echo probe is the one transaction that wants to see its own packet. */
  readonly probe: boolean;
}

// A status packet is ~100 µs on the wire at 1 Mbaud, but USB CDC adapters
// hand bytes to the host in ~1 ms batches, so budget generously per motor.
export const BASE_TIMEOUT_MS = 15;
const PER_MOTOR_MS = 3;
// A servo that missed its deadline may still answer. Before addressing it
// again, wait this long so a stale frame cannot pass for the new reply.
const QUIET_MS = 2 * BASE_TIMEOUT_MS;
// An EEPROM write is acknowledged only after the cell is programmed, which
// on the bench has taken longer than a plain reply budget.
const EEPROM_TIMEOUT_MS = 100;

/**
 * One half-duplex Feetech bus. Every public method is a complete
 * request/response transaction; transactions are serialised internally
 * because the bus can only carry one conversation at a time.
 */
export class FeetechBus {
  readonly path: string;
  readonly baudRate: number;
  readonly #port: Transport;
  readonly #parser = new FrameParser();
  readonly #stragglers = new Set<number>();
  #queue: Promise<unknown> = Promise.resolve();
  #pending: Pending | null = null;
  #lastSent: Uint8Array | null = null;
  #echo: boolean;
  #echoPending = false;
  #failure: Error | null = null;

  constructor(transport: Transport, { echo = false }: BusOptions = {}) {
    this.#port = transport;
    this.path = transport.path;
    this.baudRate = transport.baudRate;
    this.#echo = echo;
    transport.on("data", (chunk) => this.#receive(new Uint8Array(chunk)));
    transport.on("error", (error) => {
      // The port is gone (unplugged mid-write, say): fail what is in flight and everything after it.
      this.#failure = error;
      this.#pending?.fail(error);
    });
  }

  static async open(path: string, { baudRate = 1_000_000 }: OpenOptions = {}): Promise<FeetechBus> {
    const port = new SerialPort({ path, baudRate, autoOpen: false });
    await new Promise<void>((resolve, reject) => port.open((err) => (err ? reject(err) : resolve())));
    const bus = new FeetechBus(port);
    await bus.detectEcho();
    return bus;
  }

  close(): Promise<void> {
    return new Promise((resolve) => this.#port.close(() => resolve()));
  }

  /** Whether the adapter echoes transmitted bytes back to the host. */
  get echo(): boolean {
    return this.#echo;
  }

  /**
   * Send a SYNC_READ that names no servo. Nothing answers it, and its
   * instruction byte (0x82) can never be a status packet's error byte, so
   * the only way those bytes come back is the adapter echoing us.
   */
  async detectEcho(): Promise<boolean> {
    const packet = encodeInstruction(BROADCAST_ID, Instruction.SyncRead, [
      STS3215.Present_Position.address,
      STS3215.Present_Position.size,
    ]);
    const [frame] = await this.#transact(packet, [BROADCAST_ID], BASE_TIMEOUT_MS, { probe: true });
    this.#echo = frame !== undefined;
    return this.#echo;
  }

  /** Resolves with the status packet, or null if nothing answered in time. */
  async ping(id: number, timeoutMs = BASE_TIMEOUT_MS): Promise<StatusPacket | null> {
    const [frame] = await this.#transact(encodeInstruction(id, Instruction.Ping), [id], timeoutMs);
    return frame ?? null;
  }

  /** Ids that answered a ping, in ascending order. */
  async scan(ids: Iterable<number> = range(0, BROADCAST_ID)): Promise<number[]> {
    const found: number[] = [];
    for (const id of ids) if (await this.ping(id)) found.push(id);
    return found;
  }

  async readBlock(id: number, address: number, length: number): Promise<Uint8Array> {
    const packet = encodeInstruction(id, Instruction.Read, [address, length]);
    const [frame] = await this.#transact(packet, [id], BASE_TIMEOUT_MS);
    if (!frame) throw new NoResponseError(id, `read ${length} bytes at ${address}`);
    if (frame.params.length !== length) {
      throw new Error(`servo ${id} returned ${frame.params.length} bytes, expected ${length}`);
    }
    return frame.params;
  }

  async read(id: number, register: Register): Promise<number> {
    return decodeValue(register, await this.readBlock(id, register.address, register.size));
  }

  /**
   * Write one register. EEPROM registers are wrapped in the sequence LeRobot
   * uses: torque off, Lock=0, write, Lock=1. A servo acknowledges an ID write
   * already wearing its new id, so that reply is accepted from either, and
   * the trailing Lock write goes to the new id.
   */
  async write(id: number, register: Register, value: number): Promise<void> {
    if (register.area !== "eeprom") {
      await this.#writeRaw(id, register, value);
      return;
    }
    await this.#writeRaw(id, STS3215.Torque_Enable, 0);
    await this.#writeRaw(id, STS3215.Lock, 0);
    const renaming = register === STS3215.ID;
    await this.#writeRaw(id, register, value, renaming ? [id, value] : [id]);
    await this.#writeRaw(renaming ? value : id, STS3215.Lock, 1);
  }

  /** Read the same register from many servos in one bus transaction. */
  async syncRead(ids: readonly number[], register: Register): Promise<Map<number, number>> {
    const packet = encodeInstruction(BROADCAST_ID, Instruction.SyncRead, [
      register.address,
      register.size,
      ...ids,
    ]);
    const frames = await this.#transact(packet, ids, BASE_TIMEOUT_MS + PER_MOTOR_MS * ids.length);
    const values = new Map<number, number>();
    for (const frame of frames) {
      if (frame.params.length === register.size) values.set(frame.id, decodeValue(register, frame.params));
    }
    return values;
  }

  /** Write the same register on many servos in one bus transaction. No status comes back. */
  async syncWrite(register: Register, values: Iterable<readonly [number, number]>): Promise<void> {
    const params = [register.address, register.size];
    for (const [id, value] of values) params.push(id, ...encodeValue(register, value));
    await this.#transact(encodeInstruction(BROADCAST_ID, Instruction.SyncWrite, params), [], 0);
  }

  async #writeRaw(id: number, register: Register, value: number, replyFrom: readonly number[] = [id]): Promise<void> {
    const packet = encodeInstruction(id, Instruction.Write, [
      register.address,
      ...encodeValue(register, value),
    ]);
    const timeout = register.area === "eeprom" ? EEPROM_TIMEOUT_MS : BASE_TIMEOUT_MS;
    const [frame] = await this.#transact(packet, replyFrom, timeout, { need: 1 });
    if (!frame) throw new NoResponseError(id, `write ${value} at ${register.address}`);
  }

  #receive(chunk: Uint8Array): void {
    for (const frame of this.#parser.push(chunk)) {
      const pending = this.#pending;
      if (!pending) continue;
      const ours = this.#lastSent !== null && bytesEqual(frame.raw, this.#lastSent);
      if (pending.probe) {
        if (ours) {
          pending.frames.push(frame);
          pending.finish(false);
        }
        continue;
      }
      // On an echoing adapter the first copy of our own packet is the echo;
      // a genuine reply can look identical (a ping answered with the voltage
      // bit set), so only one such frame per transaction is dropped.
      if (ours && this.#echoPending) {
        this.#echoPending = false;
        continue;
      }
      if (frame.error & 0x80) continue; // never a status packet: an echoed sync instruction
      if (!pending.remaining.delete(frame.id)) continue;
      pending.frames.push(frame);
      if (pending.frames.length >= pending.need) pending.finish(false);
    }
  }

  /**
   * Send one packet and collect replies from `expectIds`, finishing early
   * once `need` of them (default: all) have arrived or the timeout passes.
   */
  #transact(
    packet: Uint8Array,
    expectIds: readonly number[],
    timeoutMs: number,
    options: { probe?: boolean; need?: number } = {},
  ): Promise<StatusPacket[]> {
    const probe = options.probe ?? false;
    const run = async (): Promise<StatusPacket[]> => {
      if (this.#failure) throw this.#failure;
      if (expectIds.some((id) => this.#stragglers.has(id))) {
        await sleep(QUIET_MS);
        this.#stragglers.clear();
      }
      this.#parser.reset();
      this.#lastSent = packet;
      this.#echoPending = this.#echo;
      if (expectIds.length === 0) {
        return new Promise((resolve, reject) =>
          this.#port.write(packet, (err) => (err ? reject(err) : this.#port.drain(() => resolve([])))),
        );
      }
      return new Promise((resolve, reject) => {
        const remaining = new Set(expectIds);
        const frames: StatusPacket[] = [];
        let timer: ReturnType<typeof setTimeout> | undefined;
        const settle = () => {
          clearTimeout(timer);
          if (this.#pending === pending) this.#pending = null;
        };
        const finish = (timedOut: boolean) => {
          settle();
          if (timedOut && !probe) for (const id of remaining) this.#stragglers.add(id);
          resolve(frames);
        };
        const need = Math.min(options.need ?? expectIds.length, expectIds.length);
        const fail = (error: Error) => {
          settle();
          reject(error);
        };
        const pending: Pending = { remaining, frames, need, finish, fail, probe };
        this.#pending = pending;
        this.#port.write(packet, (err) => err && fail(err));
        timer = setTimeout(() => finish(true), timeoutMs);
      });
    };
    const result = this.#queue.then(run, run);
    this.#queue = result.then(noop, noop);
    return result;
  }
}

const noop = () => undefined;
const sleep = (ms: number) => new Promise<void>((resolve) => setTimeout(resolve, ms));

export function* range(start: number, end: number): Generator<number> {
  for (let i = start; i < end; i++) yield i;
}
