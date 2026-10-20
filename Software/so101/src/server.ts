/**
 * The network face of one arm: an HTTP API for state and supervised moves, and a WebSocket stream
 * for joint targets at teleoperation rate. Runs wherever the follower's board is plugged in.
 *
 *   GET  /                  the browser app: cameras, 3D model, live joints, poses, jog, record and play, metrics
 *   GET  /status            every joint: position, load, temperature, torque; what drives the arm; zone state
 *   GET  /limits            each servo's calibrated Min/Max_Position_Limit
 *   GET  /poses             the saved poses
 *   POST /pose/save         {"name": "he1", "ids"?: [1,2,3,4,5]}   remember where the joints are now
 *   POST /pose/delete       {"name": "he1"}
 *   POST /pose/go           {"name": "home", "speed"?: 300, "torque"?: 70, "release"?: false}
 *   POST /move              {"targets": {"3": 2048}, "speed"?, "torque"?, "release"?}   supervised, any joints
 *   POST /grip              {"position": 2000, "torque": 30}   close the gripper toward a position at a low torque cap and hold
 *   POST /torque            {"on": true|false}
 *   POST /stop              end whatever drives the arm: a move, a playback, a recording, or a stream; the arm holds
 *   GET  /recordings        the saved recordings with their length
 *   POST /record/start      {"name": "wave", "hz"?: 30}   torque off, sample the joints until /record/stop
 *   POST /record/stop       save the recording
 *   POST /recording/delete  {"name": "wave"}
 *   POST /play              {"name": "wave", "rate"?: 1, "torque"?: 70, "release"?: false}
 *   GET  /calibration       {"camera": {...}|null, "handeye": {...}|null, ...}   what the vision kit published
 *   PUT  /calibration/<kind>  store that JSON next to the poses (kind: camera, camera2, handeye, ...)
 *   GET  /settings          the safe zone: {"zone": "enforce"|"warn"|"off", "floorMarginMm", "baseMarginMm", "jointOffsetsDeg"}
 *   PUT  /settings          change any of those
 *   GET  /metrics           jams, blocks, stalls, peak load and temperature per joint, recent events
 *   GET  /model.json        the arm model: joints, the table plane, where the URDF is served
 *   GET  /model/<file>      the URDF and its meshes;  GET /vendor/<pkg>/<file>  three.js and the URDF loader
 *   GET  /printables        the sheets to print for the vision quest (board, cube markers, gripper marker); GET /printables/<file>
 *   WS   /follow            client sends {"1": ticks, "2": ticks, ...} frames; the server drives the arm
 *
 * One thing drives the arm at a time. Every target, whether a move or a stream frame, is run through
 * the arm's own geometry first: a pose that would push a part into the table or the base is refused
 * (or only reported, in "warn" mode). On the stream, a joint that pushes hard without moving is a jam:
 * its goal is parked where it is, and it follows again once the leader backs off.
 */

import { existsSync, mkdirSync, readdirSync, readFileSync, renameSync, statSync, unlinkSync, writeFileSync } from "node:fs";
import { createServer, type IncomingMessage, type ServerResponse } from "node:http";
import { dirname, extname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { WebSocketServer, type WebSocket } from "ws";
import type { FeetechBus } from "./bus.ts";
import { ArmModel, gripperTicksToRadians, ticksToRadians, type Violation } from "./kinematics.ts";
import { clamp, type Frame } from "./mapping.ts";
import { Metrics } from "./metrics.ts";
import { MotionError, moveToPose, playTrajectory, recordTrajectory, STALL_SPAN_TICKS } from "./motion.ts";
import { assertPoseName, parsePoseFile, serializePoseFile, type PoseFile } from "./poses.ts";
import { JOINTS, STS3215, ticksToDegrees } from "./sts3215.ts";
import { parseTrajectory, serializeTrajectory, type Trajectory } from "./trajectory.ts";

export interface ServerOptions {
  host?: string;
  port?: number;
  ids?: readonly number[];
  /** The poses file; recordings, calibration, settings and metrics live beside it. */
  posesFile?: string;
  /** Folder holding so101_new_calib.urdf and assets/; without it there is no safe zone and no 3D model. */
  modelDir?: string;
  /** Folder with the printable PNG sheets of the vision kit; found beside the package or in the repo layout. */
  printablesDir?: string;
  /** Torque cap while a stream drives the arm, percent. */
  streamTorque?: number;
  /** Largest per-frame change accepted on the stream, ticks. */
  maxStep?: number;
  /** With no frame for this long, the arm is told to hold where it is. */
  watchdogMs?: number;
  /** Speed used to catch up with the first frame of a stream, ticks/s. */
  easeSpeed?: number;
  /** How often the stream's jam watch samples position and load, ms. */
  jamSampleMs?: number;
}

export interface RunningServer {
  readonly port: number;
  close(): Promise<void>;
}

export type Driver = "stream" | "move" | "play" | "record";

export interface Settings {
  zone: "enforce" | "warn" | "off";
  floorMarginMm: number;
  baseMarginMm: number;
  /** Per joint, degrees added to the model's angle, to trim the model onto the real arm. */
  jointOffsetsDeg: Record<string, number>;
}

export const DEFAULT_SETTINGS: Settings = { zone: "warn", floorMarginMm: 15, baseMarginMm: 10, jointOffsetsDeg: {} };

class HttpError extends Error {
  readonly status: number;
  constructor(status: number, message: string) {
    super(message);
    this.status = status;
  }
}

const MAX_BODY = 1_000_000;
const URDF = "so101_new_calib.urdf";
/** Load, in percent of the servo's maximum, that counts as pushing against something. */
const JAM_LOAD = 45;
const JAM_WINDOW = 6;
/** Consecutive too-far frames for one joint before it is eased over to the leader instead of left behind. */
const RESUME_AFTER_DROPS = 20;
const VENDORED = new Set(["three", "urdf-loader"]);
const MIME: Record<string, string> = { ".js": "text/javascript", ".mjs": "text/javascript", ".json": "application/json", ".urdf": "application/xml", ".stl": "model/stl", ".dae": "model/vnd.collada+xml", ".png": "image/png", ".jpg": "image/jpeg" };

export async function startServer(bus: FeetechBus, options: ServerOptions = {}): Promise<RunningServer> {
  const ids = options.ids ?? [1, 2, 3, 4, 5, 6];
  const posesFile = resolve(options.posesFile ?? "poses.json");
  const dataDir = dirname(posesFile);
  const recordingsDir = join(dataDir, "recordings");
  const calibrationDir = join(dataDir, "calibration");
  const settingsFile = join(dataDir, "settings.json");
  const streamTorque = options.streamTorque ?? 70;
  const maxStep = options.maxStep ?? 400;
  const watchdogMs = options.watchdogMs ?? 500;
  const easeSpeed = options.easeSpeed ?? 300;
  const jamSampleMs = options.jamSampleMs ?? 100;
  const margin = 30;
  const metrics = new Metrics(join(dataDir, "metrics.jsonl"));

  const limits = new Map<number, { min: number; max: number }>();
  for (const id of ids) {
    limits.set(id, {
      min: await bus.read(id, STS3215.Min_Position_Limit),
      max: await bus.read(id, STS3215.Max_Position_Limit),
    });
  }

  // ------------------------------------------------------------------ the model and the zone
  const here = dirname(fileURLToPath(import.meta.url));
  const modelDir = options.modelDir ?? [join(here, "../../../Simulation/SO101"), join(here, "../model")].find((d) => existsSync(join(d, URDF)));
  const printablesDir = options.printablesDir ?? [join(here, "../../vision/printables"), join(here, "../printables")].find((d) => existsSync(d));
  let model: ArmModel | null = null;
  if (modelDir && existsSync(join(modelDir, URDF))) {
    try {
      model = ArmModel.load(join(modelDir, URDF));
    } catch (err) {
      console.error(`model: ${err instanceof Error ? err.message : String(err)}; no safe zone`);
    }
  } else console.error(`model: no ${URDF} in ${modelDir ?? "(no model dir)"}; no safe zone and no 3D view`);

  let settings: Settings = { ...DEFAULT_SETTINGS };
  if (existsSync(settingsFile)) {
    // Field by field: one bad value must not throw away the zone mode or the other trims.
    try {
      const stored = JSON.parse(readFileSync(settingsFile, "utf8")) as Record<string, unknown>;
      for (const [key, value] of Object.entries(stored)) {
        try {
          settings = validateSettings({ ...settings, [key]: value });
        } catch (err) {
          console.error(`settings: ignoring ${key} in ${settingsFile}: ${err instanceof Error ? err.message : String(err)}`);
        }
      }
    } catch (err) {
      console.error(`settings: ${settingsFile} is not readable JSON (${err instanceof Error ? err.message : String(err)}); using the defaults, zone "${settings.zone}"`);
    }
  }
  const zoneSettings = () => ({ floorMargin: settings.floorMarginMm / 1000, baseMargin: settings.baseMarginMm / 1000 });
  /** Joint angles for the model from servo ticks, with the trims applied. The gripper maps closed→0, open→its URDF limit. */
  const gripperOpen = model?.movable.find((j) => j.name === "gripper")?.upper ?? 1.745;
  const toQ = (positions: ReadonlyMap<number, number>) => {
    const q = new Map<string, number>();
    for (const [id, ticks] of positions) {
      const name = JOINTS[id];
      const lim = limits.get(id);
      if (!name || !lim) continue;
      const rad = name === "gripper" ? gripperTicksToRadians(ticks, lim.min, lim.max, gripperOpen) : ticksToRadians(ticks, lim.min, lim.max);
      q.set(name, rad + ((settings.jointOffsetsDeg[name] ?? 0) * Math.PI) / 180);
    }
    return q;
  };
  const evaluate = (positions: ReadonlyMap<number, number>) => (model && settings.zone !== "off" ? model.evaluate(toQ(positions), zoneSettings()) : null);
  /** Only poses the arm is in, or is sent to, count toward "closest to the table"; refused frames do not. */
  const approach = (r: ReturnType<typeof evaluate>) => {
    if (r && model) metrics.approach(r.lowest.z - model.floorZ, r.lowest.link);
  };
  const describeViolation = (v: Violation) => {
    const part = v.link.replace(/_link$/, "").replace(/_so101_v1$/, "");
    const marginMm = v.what === "table" ? settings.floorMarginMm : settings.baseMarginMm;
    const beyond = v.depth * 1000 - marginMm; // the depth includes the margin
    return beyond > 0
      ? `${part} would be ${beyond.toFixed(0)} mm ${v.what === "table" ? "below the table" : "inside the base"} (plus the ${marginMm} mm margin)`
      : `${part} would come within ${(-beyond).toFixed(0)} mm of the ${v.what}, inside the ${marginMm} mm margin`;
  };
  /** The worst violation on the straight joint-space path from one pose to another, or null. */
  const checkPath = (from: ReadonlyMap<number, number>, to: ReadonlyMap<number, number>, steps = 12): Violation | null => {
    if (!model || settings.zone === "off") return null;
    let worst: Violation | null = null;
    for (let s = 1; s <= steps; s++) {
      const mix = new Map<number, number>();
      for (const [id, a] of from) mix.set(id, Math.round(a + ((to.get(id) ?? a) - a) * (s / steps)));
      const v = model.check(toQ(mix), zoneSettings());
      if (v && (!worst || v.depth > worst.depth)) worst = v;
    }
    return worst;
  };

  // ------------------------------------------------------------------ who drives the arm
  let driver: Driver | null = null;
  let current: AbortController | null = null;
  const claim = (kind: Driver): AbortController => {
    if (driver) throw new HttpError(409, `${describe(driver)} is driving the arm`);
    driver = kind;
    current = new AbortController();
    return current;
  };
  const release = () => {
    driver = null;
    current = null;
  };
  const describe = (d: Driver) => ({ stream: "a stream", move: "a move", play: "a playback", record: "a recording" })[d];
  /** What the stream is up against right now, for /status. */
  let blocked: Violation | null = null;
  const jams = new Map<number, { position: number; direction: number }>();
  let streamStats: { frames: number; dropped: number; refused: number } | null = null;

  // ------------------------------------------------------------------ files
  const writeAtomic = (path: string, text: string) => {
    mkdirSync(dirname(path), { recursive: true });
    writeFileSync(`${path}.tmp`, text);
    renameSync(`${path}.tmp`, path);
  };
  const loadPoses = (): PoseFile => {
    try {
      return parsePoseFile(readFileSync(posesFile, "utf8"));
    } catch (err) {
      if ((err as NodeJS.ErrnoException).code === "ENOENT") return {};
      throw new HttpError(500, `${posesFile}: ${err instanceof Error ? err.message : String(err)}`);
    }
  };
  const poseNamed = (name: unknown): string => {
    if (typeof name !== "string") throw new HttpError(400, "name must be a string");
    try {
      assertPoseName(name);
    } catch (err) {
      throw new HttpError(400, (err as Error).message);
    }
    return name;
  };
  const recordingPath = (name: string) => join(recordingsDir, `${name}.json`);
  const loadRecording = (name: string): Trajectory => {
    try {
      return parseTrajectory(readFileSync(recordingPath(name), "utf8"));
    } catch (err) {
      if ((err as NodeJS.ErrnoException).code === "ENOENT") throw new HttpError(404, `no recording "${name}"`);
      throw new HttpError(500, `recording "${name}": ${err instanceof Error ? err.message : String(err)}`);
    }
  };
  const listRecordings = () => {
    const out: Record<string, { hz: number; frames: number; seconds: number; recorded?: string }> = {};
    let names: string[] = [];
    try {
      names = readdirSync(recordingsDir).filter((f) => f.endsWith(".json"));
    } catch {
      return out;
    }
    for (const file of names) {
      try {
        const t = parseTrajectory(readFileSync(join(recordingsDir, file), "utf8"));
        out[file.slice(0, -5)] = { hz: t.hz, frames: t.frames.length, seconds: Number((t.frames.length / t.hz).toFixed(1)), ...(t.recorded ? { recorded: t.recorded } : {}) };
      } catch {
        /* a file that is not a recording is not listed */
      }
    }
    return out;
  };
  const loadCalibration = (): Record<string, unknown> => {
    const out: Record<string, unknown> = { camera: null, handeye: null };
    try {
      for (const file of readdirSync(calibrationDir).filter((f) => f.endsWith(".json"))) {
        try {
          out[file.slice(0, -5)] = JSON.parse(readFileSync(join(calibrationDir, file), "utf8"));
        } catch {
          /* skip a broken file */
        }
      }
    } catch {
      /* none yet */
    }
    return out;
  };
  /** A file under a root, or null for anything that tries to leave it. */
  const safeFile = (root: string, rel: string): string | null => {
    if (rel.includes("\0") || rel.split("/").some((part) => part === "..")) return null;
    const path = resolve(root, rel);
    if (path !== root && !path.startsWith(root + "/")) return null;
    try {
      return statSync(path).isFile() ? path : null;
    } catch {
      return null;
    }
  };

  // ------------------------------------------------------------------ helpers
  const json = (res: ServerResponse, status: number, body: unknown) => {
    res.writeHead(status, { "content-type": "application/json" });
    res.end(JSON.stringify(body));
  };
  const file = (res: ServerResponse, path: string, cache: string) => {
    res.writeHead(200, { "content-type": MIME[extname(path).toLowerCase()] ?? "application/octet-stream", "cache-control": cache });
    res.end(readFileSync(path));
  };
  const readBody = (req: IncomingMessage) =>
    new Promise<Record<string, unknown>>((resolve, reject) => {
      let data = "";
      req.on("data", (c) => {
        data += c;
        if (data.length > MAX_BODY) {
          req.destroy();
          reject(new HttpError(413, "body too large"));
        }
      });
      req.on("end", () => {
        if (!data.trim()) return resolve({});
        try {
          const parsed: unknown = JSON.parse(data);
          if (typeof parsed !== "object" || parsed === null || Array.isArray(parsed)) throw new Error();
          resolve(parsed as Record<string, unknown>);
        } catch {
          reject(new HttpError(400, "body must be a JSON object"));
        }
      });
      req.on("error", reject);
    });
  const number = (value: unknown, fallback: number, min: number, max: number, what: string): number => {
    if (value === undefined) return fallback;
    if (typeof value !== "number" || !Number.isFinite(value)) throw new HttpError(400, `${what} must be a number`);
    return clamp(value, min, max);
  };
  const idList = (value: unknown): number[] => {
    if (value === undefined) return [...ids];
    if (!Array.isArray(value) || !value.length || !value.every((id) => Number.isInteger(id) && ids.includes(id as number))) {
      throw new HttpError(400, `ids must be a non-empty list drawn from ${ids.join(", ")}`);
    }
    return [...new Set(value as number[])];
  };
  function validateSettings(s: Partial<Settings>): Settings {
    const out: Settings = { ...DEFAULT_SETTINGS, jointOffsetsDeg: {} };
    if (s.zone !== undefined) {
      if (s.zone !== "enforce" && s.zone !== "warn" && s.zone !== "off") throw new HttpError(400, 'zone must be "enforce", "warn" or "off"');
      out.zone = s.zone;
    }
    for (const key of ["floorMarginMm", "baseMarginMm"] as const) {
      if (s[key] === undefined) continue;
      if (typeof s[key] !== "number" || !Number.isFinite(s[key]) || s[key]! < 0 || s[key]! > 100) throw new HttpError(400, `${key} must be 0-100`);
      out[key] = s[key]!;
    }
    if (s.jointOffsetsDeg !== undefined) {
      if (typeof s.jointOffsetsDeg !== "object" || s.jointOffsetsDeg === null || Array.isArray(s.jointOffsetsDeg)) throw new HttpError(400, "jointOffsetsDeg must be an object");
      for (const [name, deg] of Object.entries(s.jointOffsetsDeg)) {
        if (!Object.values(JOINTS).includes(name)) throw new HttpError(400, `no joint "${name}"`);
        if (typeof deg !== "number" || !Number.isFinite(deg) || Math.abs(deg) > 30) throw new HttpError(400, `offset for ${name} must be within ±30 degrees`);
        if (deg !== 0) out.jointOffsetsDeg[name] = deg;
      }
    }
    return out;
  }

  const status = async () => {
    const positions = await bus.syncRead(ids, STS3215.Present_Position);
    const loads = await bus.syncRead(ids, STS3215.Present_Load);
    const temps = await bus.syncRead(ids, STS3215.Present_Temperature);
    const torque = await bus.syncRead(ids, STS3215.Torque_Enable);
    const joints = ids.map((id) => ({
      id,
      joint: JOINTS[id] ?? null,
      position: positions.get(id) ?? null,
      degrees: positions.has(id) ? Number(ticksToDegrees(positions.get(id)!).toFixed(1)) : null,
      load: loads.has(id) ? loads.get(id)! / 10 : null,
      temperature: temps.get(id) ?? null,
      torque: torque.has(id) ? torque.get(id) === 1 : null,
    }));
    metrics.sample(joints);
    const zone = evaluate(positions);
    approach(zone);
    return {
      joints,
      busy: driver !== null,
      driving: driver,
      recording: recording?.name ?? null,
      zone: model ? settings.zone : "off",
      violation: zone?.violation ?? null,
      lowestMm: zone && model ? Number(((zone.lowest.z - model.floorZ) * 1000).toFixed(0)) : null,
      blocked,
      jammed: [...jams.keys()],
      stream: streamStats,
    };
  };

  /** A supervised move to explicit targets; the one way joints move over HTTP. */
  const move = async (targetIds: number[], positions: number[], body: Record<string, unknown>, source: string) => {
    const controller = claim("move");
    try {
      const from = await bus.syncRead(ids, STS3215.Present_Position);
      const to = new Map(from);
      targetIds.forEach((id, i) => to.set(id, positions[i]!));
      const violation = checkPath(from, to);
      if (violation) {
        metrics.event("block", { source, what: violation.what, link: violation.link, depthMm: Math.round(violation.depth * 1000), mode: settings.zone });
        if (settings.zone === "enforce") return { status: 409, body: { reached: false, error: `refused: ${describeViolation(violation)}`, violation } };
      }
      const result = await moveToPose(bus, targetIds, positions, {
        speed: number(body.speed, 300, 20, 1500, "speed"),
        torquePercent: number(body.torque, 70, 10, 100, "torque"),
        signal: controller.signal,
      });
      if (body.release === true) await bus.syncWrite(STS3215.Torque_Enable, targetIds.map((id) => [id, 0] as const));
      metrics.counters.move++;
      return { status: 200, body: { reached: true, final: Object.fromEntries(result.final), adjusted: Object.fromEntries(result.adjusted), ...(violation ? { warning: describeViolation(violation) } : {}) } };
    } catch (err) {
      if (err instanceof MotionError) {
        metrics.event(err.stalled.length ? "stall" : err.missing.length ? "missing" : "move", { source, error: err.message, stalled: err.stalled, missing: err.missing });
        return { status: 409, body: { reached: false, error: err.message, stalled: err.stalled, missing: err.missing } };
      }
      throw err;
    } finally {
      release();
    }
  };

  // ------------------------------------------------------------------ recording
  type Recorded = { trajectory: Trajectory; filled: number } | { error: string };
  let recording: { name: string; hz: number; done: Promise<Recorded> } | null = null;
  const finishRecording = async () => {
    if (!recording || !current) throw new HttpError(409, "nothing is being recorded");
    const { name, done } = recording;
    const controller = current;
    recording = null; // taken: a second caller, racing this one, finds nothing to stop
    controller.abort();
    const result = await done;
    release();
    if ("error" in result) throw new HttpError(500, `recording "${name}" failed: ${result.error}; nothing saved`);
    const { trajectory, filled } = result;
    if (!trajectory.frames.length) throw new HttpError(409, "nothing recorded; did the servos answer?");
    writeAtomic(recordingPath(name), serializeTrajectory(trajectory));
    metrics.event("record", { name, frames: trajectory.frames.length, seconds: trajectory.frames.length / trajectory.hz });
    return { saved: name, frames: trajectory.frames.length, seconds: Number((trajectory.frames.length / trajectory.hz).toFixed(1)), filled };
  };

  // ------------------------------------------------------------------ routes
  const viewerPath = join(here, "viewer.html");
  const nodeModules = resolve(here, "../node_modules");
  const http = createServer(async (req, res) => {
    try {
      const url = new URL(req.url ?? "/", "http://x");
      const route = `${req.method} ${url.pathname}`;
      const decode = (text: string) => {
        try {
          return decodeURIComponent(text);
        } catch {
          return "\0"; // never a file
        }
      };
      if (req.method === "GET" && url.pathname.startsWith("/model/") && modelDir) {
        const path = safeFile(resolve(modelDir), decode(url.pathname.slice("/model/".length)));
        if (!path || ![".urdf", ".stl", ".dae", ".png", ".jpg"].includes(extname(path).toLowerCase())) return json(res, 404, { error: "no such model file" });
        return file(res, path, "max-age=86400");
      }
      if (req.method === "GET" && url.pathname === "/printables") {
        let files: string[] = [];
        try {
          files = printablesDir ? readdirSync(printablesDir).filter((f) => f.endsWith(".png")).sort() : [];
        } catch {
          /* none */
        }
        return json(res, 200, files.map((f) => ({ file: f, url: `/printables/${f}`, bytes: statSync(join(printablesDir!, f)).size })));
      }
      if (req.method === "GET" && url.pathname.startsWith("/printables/") && printablesDir) {
        const path = safeFile(resolve(printablesDir), decode(url.pathname.slice("/printables/".length)));
        if (!path || extname(path).toLowerCase() !== ".png") return json(res, 404, { error: "no such printable" });
        return file(res, path, "max-age=3600");
      }
      if (req.method === "GET" && url.pathname.startsWith("/vendor/")) {
        const [pkg, ...rest] = decode(url.pathname.slice("/vendor/".length)).split("/");
        const path = pkg && VENDORED.has(pkg) && rest.length ? safeFile(join(nodeModules, pkg), rest.join("/")) : null;
        if (!path || ![".js", ".mjs", ".json"].includes(extname(path).toLowerCase())) return json(res, 404, { error: "no such vendored file" });
        return file(res, path, "max-age=86400");
      }
      if (req.method === "PUT" && url.pathname.startsWith("/calibration/")) {
        const kind = url.pathname.slice("/calibration/".length);
        if (!/^[a-z][a-z0-9_-]{0,31}$/.test(kind)) throw new HttpError(400, "calibration kind: lowercase letters, digits, - and _");
        const body = await readBody(req);
        writeAtomic(join(calibrationDir, `${kind}.json`), JSON.stringify({ ...body, saved: new Date().toISOString() }, null, 1));
        return json(res, 200, { stored: kind });
      }
      switch (route) {
        case "GET /":
        case "GET /index.html":
          res.writeHead(200, { "content-type": "text/html; charset=utf-8", "cache-control": "no-store" });
          return res.end(readFileSync(viewerPath)); // read per request: a viewer change deploys with rsync alone
        case "GET /status":
          return json(res, 200, await status());
        case "GET /limits":
          return json(res, 200, Object.fromEntries(limits));
        case "GET /poses":
          return json(res, 200, loadPoses());
        case "GET /recordings":
          return json(res, 200, listRecordings());
        case "GET /calibration":
          return json(res, 200, loadCalibration());
        case "GET /metrics":
          return json(res, 200, metrics.summary());
        case "GET /settings":
          return json(res, 200, { ...settings, model: model !== null });
        case "PUT /settings": {
          const body = await readBody(req);
          settings = validateSettings({ ...settings, ...body, jointOffsetsDeg: { ...settings.jointOffsetsDeg, ...((body.jointOffsetsDeg as Record<string, number> | undefined) ?? {}) } });
          writeAtomic(settingsFile, JSON.stringify(settings, null, 1));
          return json(res, 200, { ...settings, model: model !== null });
        }
        case "GET /model.json":
          if (!model) return json(res, 404, { error: "no model loaded on this server" });
          return json(res, 200, {
            urdf: `/model/${URDF}`,
            floorZ: model.floorZ,
            baseTop: model.baseTop,
            joints: model.movable.map((j) => ({ name: j.name, id: Number(Object.entries(JOINTS).find(([, n]) => n === j.name)?.[0] ?? 0), lower: j.lower, upper: j.upper })),
          });
        case "POST /pose/save": {
          const body = await readBody(req);
          const name = poseNamed(body.name);
          const wanted = idList(body.ids);
          const positions = await bus.syncRead(wanted, STS3215.Present_Position);
          const missing = wanted.filter((id) => !positions.has(id));
          if (missing.length) throw new HttpError(409, `no response from servo ${missing.join(", ")}`);
          const pose = { ids: wanted, positions: wanted.map((id) => positions.get(id)!), saved: new Date().toISOString() };
          writeAtomic(posesFile, serializePoseFile({ ...loadPoses(), [name]: pose }));
          const onStops = wanted.filter((id) => {
            const lim = limits.get(id)!;
            return positions.get(id)! - lim.min < 40 || lim.max - positions.get(id)! < 40;
          });
          return json(res, 200, { name, ...pose, onStops });
        }
        case "POST /pose/delete": {
          const name = poseNamed((await readBody(req)).name);
          const poses = loadPoses();
          if (!Object.hasOwn(poses, name)) throw new HttpError(404, `no pose "${name}"`);
          const { [name]: _, ...rest } = poses;
          writeAtomic(posesFile, serializePoseFile(rest));
          return json(res, 200, { deleted: name });
        }
        case "POST /pose/go": {
          const body = await readBody(req);
          const name = poseNamed(body.name);
          const poses = loadPoses();
          if (!Object.hasOwn(poses, name)) throw new HttpError(404, `no pose "${name}"`);
          const pose = poses[name]!;
          const result = await move([...pose.ids], [...pose.positions], body, `pose ${name}`);
          return json(res, result.status, result.body);
        }
        case "POST /move": {
          const body = await readBody(req);
          const targets = body.targets;
          if (typeof targets !== "object" || targets === null || Array.isArray(targets)) throw new HttpError(400, "targets must be an object of id: ticks");
          const targetIds: number[] = [];
          const positions: number[] = [];
          for (const [key, value] of Object.entries(targets as Record<string, unknown>)) {
            const id = Number(key);
            if (!/^\d+$/.test(key) || !ids.includes(id)) throw new HttpError(400, `no servo ${key} on this arm`);
            if (!Number.isInteger(value)) throw new HttpError(400, `target for servo ${key} must be an integer`);
            targetIds.push(id);
            positions.push(value as number);
          }
          if (!targetIds.length) throw new HttpError(400, "targets is empty");
          const result = await move(targetIds, positions, body, "move");
          return json(res, result.status, result.body);
        }
        case "POST /grip": {
          // Not supervised on purpose: a supervised close stops at the first contact and holds with no force.
          // The servo's own torque cap does the squeezing, and its overload protection holds a steady grip.
          const body = await readBody(req);
          const id = 6;
          const lim = limits.get(id);
          if (!lim) throw new HttpError(400, "this arm has no gripper (servo 6)");
          if (!Number.isInteger(body.position)) throw new HttpError(400, "position must be an integer");
          const position = clamp(body.position as number, lim.min + margin, lim.max - margin);
          const torque = number(body.torque, 30, 10, 60, "torque");
          claim("move");
          try {
            const present = await bus.syncRead([id], STS3215.Present_Position);
            if (!present.has(id)) throw new HttpError(409, "no response from the gripper");
            await bus.syncWrite(STS3215.Goal_Position, [[id, present.get(id)!]]);
            await bus.syncWrite(STS3215.Torque_Limit, [[id, Math.round(torque * 10)]]); // stays low while it holds; the next supervised gripper move puts the ceiling back
            await bus.syncWrite(STS3215.Goal_Velocity, [[id, 300]]);
            await bus.syncWrite(STS3215.Torque_Enable, [[id, 1]]);
            await bus.syncWrite(STS3215.Goal_Position, [[id, position]]);
          } finally {
            release();
          }
          return json(res, 200, { gripping: position, torque });
        }
        case "POST /torque": {
          const { on } = await readBody(req);
          if (typeof on !== "boolean") throw new HttpError(400, "on must be true or false");
          if (driver) throw new HttpError(409, `${describe(driver)} is driving the arm`);
          if (on) {
            const positions = await bus.syncRead(ids, STS3215.Present_Position);
            await bus.syncWrite(STS3215.Goal_Position, ids.filter((id) => positions.has(id)).map((id) => [id, positions.get(id)!] as const));
          }
          await bus.syncWrite(STS3215.Torque_Enable, ids.map((id) => [id, on ? 1 : 0] as const));
          return json(res, 200, { torque: on });
        }
        case "POST /stop": {
          const was = driver;
          if (was === "stream") for (const client of wss.clients) client.close(1000, "stopped from the viewer"); // its close handler holds the arm
          else if (was === "record") {
            const summary = await finishRecording();
            return json(res, 200, { stopped: was, ...summary });
          } else current?.abort(); // the supervisor stops the move where it is
          return json(res, 200, { stopped: was });
        }
        case "POST /record/start": {
          const body = await readBody(req);
          const name = poseNamed(body.name);
          const hz = number(body.hz, 30, 1, 100, "hz");
          const wanted = idList(body.ids);
          if (Object.hasOwn(listRecordings(), name)) throw new HttpError(409, `recording "${name}" already exists; pick another name`);
          const controller = claim("record");
          const done: Promise<Recorded> = recordTrajectory(bus, wanted, { hz, signal: controller.signal }).then(
            (r) => r,
            (err: unknown) => ({ error: err instanceof Error ? err.message : String(err) }), // reported by /record/stop, never unhandled
          );
          recording = { name, hz, done };
          return json(res, 200, { recording: name, hz, ids: wanted });
        }
        case "POST /record/stop":
          return json(res, 200, await finishRecording());
        case "POST /recording/delete": {
          const name = poseNamed((await readBody(req)).name);
          if (recording?.name === name) throw new HttpError(409, `"${name}" is being recorded right now`);
          try {
            unlinkSync(recordingPath(name));
          } catch (err) {
            if ((err as NodeJS.ErrnoException).code === "ENOENT") throw new HttpError(404, `no recording "${name}"`);
            throw err;
          }
          return json(res, 200, { deleted: name });
        }
        case "POST /play": {
          const body = await readBody(req);
          const name = poseNamed(body.name);
          const trajectory = loadRecording(name);
          const controller = claim("play");
          try {
            if (model && settings.zone !== "off") {
              // The way to the first frame, then every frame (sampled when the recording is long), each judged as the whole
              // pose with the joints the recording does not drive left where they are. Nothing moves before this passes.
              const present = await bus.syncRead(ids, STS3215.Present_Position);
              const poseAt = (frame: readonly number[]) => {
                const pose = new Map(present);
                trajectory.ids.forEach((id, i) => pose.set(id, frame[i]!));
                return pose;
              };
              const refuse = (v: Violation, where: string): boolean => {
                metrics.event("block", { source: `play ${name}`, where, what: v.what, link: v.link, depthMm: Math.round(v.depth * 1000), mode: settings.zone });
                if (settings.zone !== "enforce") return false;
                json(res, 409, { played: 0, error: `refused: ${where}: ${describeViolation(v)}`, violation: v });
                return true;
              };
              const onTheWay = checkPath(present, poseAt(trajectory.frames[0]!));
              if (onTheWay && refuse(onTheWay, "on the way to the first frame")) return;
              const stride = Math.max(1, Math.ceil(trajectory.frames.length / 3000));
              for (let index = 0; index < trajectory.frames.length; index += stride) {
                const v = model.check(toQ(poseAt(trajectory.frames[index]!)), zoneSettings());
                if (v) {
                  if (refuse(v, `frame ${index} of ${trajectory.frames.length}`)) return;
                  break;
                }
              }
            }
            const played = await playTrajectory(bus, trajectory, {
              rate: number(body.rate, 1, 0.1, 2, "rate"),
              torquePercent: number(body.torque, 70, 10, 100, "torque"),
              signal: controller.signal,
            });
            if (body.release === true) await bus.syncWrite(STS3215.Torque_Enable, trajectory.ids.map((id) => [id, 0] as const));
            metrics.event("play", { name, played, frames: trajectory.frames.length });
            return json(res, 200, { played, frames: trajectory.frames.length });
          } catch (err) {
            if (err instanceof MotionError) {
              metrics.event(err.stalled.length ? "stall" : err.missing.length ? "missing" : "play", { source: `play ${name}`, error: err.message, stalled: err.stalled, missing: err.missing });
              return json(res, 409, { played: 0, error: err.message, stalled: err.stalled, missing: err.missing });
            }
            throw err;
          } finally {
            release();
          }
        }
      }
      json(res, 404, { error: "no such route" });
    } catch (err) {
      if (err instanceof HttpError) return json(res, err.status, { error: err.message });
      json(res, 500, { error: err instanceof Error ? err.message : String(err) });
    }
  });

  // ------------------------------------------------------------------ the stream
  const wss = new WebSocketServer({ server: http, path: "/follow" });
  wss.on("connection", (ws: WebSocket) => {
    if (driver) {
      ws.close(1013, `${describe(driver)} is driving the arm`);
      return;
    }
    claim("stream");
    const startedAt = Date.now();
    const stats = { frames: 0, dropped: 0, refused: 0 };
    streamStats = stats;
    let last: Map<number, number> | null = null;
    /** Consecutive frames dropped per joint for jumping too far; after enough of them the joint is eased over instead of left behind. */
    const drops = new Map<number, number>();
    /** How deep in a violation the pose the arm was last sent to is; a frame may never go deeper, but may back out. */
    let heldDepth = 0;
    /** The latest sampled positions, so a frame that names only some joints is judged with the rest where they are. */
    let present = new Map<number, number>();
    let armed = false;
    let timer: ReturnType<typeof setTimeout> | undefined;
    let chain: Promise<void> = Promise.resolve();
    const run = (fn: () => Promise<void>) => (chain = chain.then(fn).catch((err: unknown) => console.error(`stream: ${err instanceof Error ? err.message : String(err)}`)));

    const hold = () =>
      run(async () => {
        const positions = await bus.syncRead(ids, STS3215.Present_Position);
        await bus.syncWrite(STS3215.Goal_Position, ids.filter((id) => positions.has(id)).map((id) => [id, positions.get(id)!] as const));
        last = null;
      });
    const arm = () =>
      run(async () => {
        const positions = await bus.syncRead(ids, STS3215.Present_Position);
        present = new Map(positions);
        const ready = ids.filter((id) => positions.has(id));
        await bus.syncWrite(STS3215.Goal_Position, ready.map((id) => [id, positions.get(id)!] as const));
        await bus.syncWrite(STS3215.Torque_Limit, ready.map((id) => [id, streamTorque * 10] as const));
        await bus.syncWrite(STS3215.Acceleration, ready.map((id) => [id, 254] as const));
        await bus.syncWrite(STS3215.Goal_Velocity, ready.map((id) => [id, 0] as const));
        await bus.syncWrite(STS3215.Torque_Enable, ready.map((id) => [id, 1] as const));
        last = null; // the first frame eases the arm over, however far away the leader is
        heldDepth = evaluate(positions)?.violation?.depth ?? 0;
        armed = true;
      });
    let easing = false;

    // The jam watch: a joint that pushes hard without moving toward its target is parked where it is.
    const windows = new Map<number, Array<{ p: number; l: number }>>();
    const observe = async (id: number, p: number, l: number, target: number | undefined): Promise<boolean> => {
      present.set(id, p);
      const w = windows.get(id) ?? [];
      w.push({ p, l });
      if (w.length > JAM_WINDOW) w.shift();
      windows.set(id, w);
      if (jams.has(id) || w.length < JAM_WINDOW || target === undefined) return jams.has(id);
      const pushing = w.every((s) => Math.abs(s.l) >= JAM_LOAD * 10);
      const travel = Math.max(...w.map((s) => s.p)) - Math.min(...w.map((s) => s.p));
      if (!(pushing && travel < STALL_SPAN_TICKS && Math.abs(target - p) > 30)) return false;
      await bus.syncWrite(STS3215.Goal_Position, [[id, p]]);
      jams.set(id, { position: p, direction: Math.sign(target - p) });
      last?.set(id, p);
      metrics.event("jam", { source: "stream", id, joint: JOINTS[id] ?? null, position: p, target, load: l / 10 });
      return true;
    };
    const watch = setInterval(
      () =>
        void run(async () => {
          if (!armed || last === null) return;
          const positions = await bus.syncRead(ids, STS3215.Present_Position);
          const loads = await bus.syncRead(ids, STS3215.Present_Load);
          for (const id of ids) {
            const p = positions.get(id), l = loads.get(id);
            if (p !== undefined && l !== undefined) await observe(id, p, l, last.get(id));
          }
        }),
      jamSampleMs,
    );

    ws.on("message", (data) => {
      let frame: Frame;
      try {
        frame = JSON.parse(String(data));
      } catch {
        return;
      }
      clearTimeout(timer);
      timer = setTimeout(() => void hold(), watchdogMs);
      if (!armed) void arm();
      if (easing) return; // frames that arrive while catching up are superseded by the next one
      run(async () => {
        const targets: Array<readonly [number, number]> = [];
        let ease = last === null; // after a hold, everything eases
        for (const [key, value] of Object.entries(frame)) {
          const id = Number(key);
          const lim = limits.get(id);
          if (!lim || !Number.isFinite(value)) continue;
          const target = clamp(Math.round(value), lim.min + margin, lim.max - margin);
          const jam = jams.get(id);
          if (jam) {
            if ((target - jam.position) * jam.direction > -10) continue; // still pushing into the obstacle: wait for the leader to back off
            jams.delete(id);
            windows.delete(id);
          }
          const previous = last?.get(id);
          if (previous === undefined) ease = true; // a joint this stream has not driven yet is eased over, never jumped
          else if (Math.abs(target - previous) > maxStep) {
            stats.dropped++;
            const n = (drops.get(id) ?? 0) + 1;
            drops.set(id, n);
            if (n < RESUME_AFTER_DROPS) continue; // no hand moves that far in one frame
            // The leader has stayed far away (after a block or a jam): ease over to it rather than leave the joint stranded.
            drops.delete(id);
            ease = true;
          } else drops.delete(id);
          targets.push([id, target]);
        }
        if (!targets.length) return;
        stats.frames++;
        // The zone: the frame is judged as the whole pose it would make, with uncommanded joints where they are.
        const pose = new Map(present);
        for (const [id, t] of last ?? []) pose.set(id, t);
        for (const [id, t] of targets) pose.set(id, t);
        const zone = evaluate(pose);
        if (zone?.violation) {
          if (!blocked) metrics.event("block", { source: "stream", what: zone.violation.what, link: zone.violation.link, depthMm: Math.round(zone.violation.depth * 1000), mode: settings.zone });
          blocked = zone.violation;
          if (settings.zone === "enforce" && zone.violation.depth > heldDepth + 0.001) {
            stats.refused++;
            return; // deeper in than the arm already is: it stays at the last good targets; a frame that backs out is let through
          }
        } else blocked = null;
        heldDepth = zone?.violation?.depth ?? 0;
        approach(zone);
        if (ease) {
          // Catch up slowly with every joint in this frame, then hand over to the full-rate stream.
          // A joint that jams on the way is parked and no longer waited for.
          easing = true;
          try {
            await bus.syncWrite(STS3215.Goal_Velocity, targets.map(([id]) => [id, easeSpeed] as const));
            await bus.syncWrite(STS3215.Goal_Position, targets);
            const deadline = Date.now() + 4000;
            const wanted = new Map(targets);
            while (Date.now() < deadline) {
              const positions = await bus.syncRead([...wanted.keys()], STS3215.Present_Position);
              const loads = await bus.syncRead([...wanted.keys()], STS3215.Present_Load);
              let worst = 0;
              for (const [id, t] of wanted) {
                const p = positions.get(id), l = loads.get(id);
                if (p === undefined || l === undefined) continue;
                if (await observe(id, p, l, t)) continue; // jammed: parked, not waited for
                worst = Math.max(worst, Math.abs(p - t));
              }
              if (worst <= 25) break;
              await new Promise((r) => setTimeout(r, 50));
            }
            await bus.syncWrite(STS3215.Goal_Velocity, targets.map(([id]) => [id, 0] as const));
          } finally {
            easing = false;
          }
          last = new Map([...(last ?? []), ...targets]);
          for (const [id, jam] of jams) last.set(id, jam.position);
          return;
        }
        await bus.syncWrite(STS3215.Goal_Position, targets);
        for (const [id, t] of targets) last!.set(id, t);
      });
    });
    ws.on("close", () => {
      clearTimeout(timer);
      clearInterval(watch);
      void run(async () => {
        try {
          if (armed) {
            const positions = await bus.syncRead(ids, STS3215.Present_Position);
            await bus.syncWrite(STS3215.Goal_Position, ids.filter((id) => positions.has(id)).map((id) => [id, positions.get(id)!] as const));
            await bus.syncWrite(STS3215.Torque_Limit, ids.map((id) => [id, 1000] as const));
          }
        } finally {
          blocked = null;
          jams.clear();
          streamStats = null;
          metrics.event("stream", { seconds: Number(((Date.now() - startedAt) / 1000).toFixed(1)), ...stats });
          release(); // whatever the bus did, the arm is free for the next driver
        }
      });
    });
  });

  await new Promise<void>((resolve) => http.listen(options.port ?? 8101, options.host ?? "0.0.0.0", resolve));
  const address = http.address();
  const port = typeof address === "object" && address ? address.port : options.port ?? 8101;
  return {
    port,
    close: () =>
      new Promise<void>((resolve) => {
        for (const client of wss.clients) client.terminate();
        wss.close();
        http.close(() => resolve());
        http.closeAllConnections(); // the viewer keeps idle keep-alive connections open; close() alone would wait for them
      }),
  };
}
