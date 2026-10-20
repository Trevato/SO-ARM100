import { closeSync, openSync, readFileSync, renameSync, writeFileSync } from "node:fs";
import { parseArgs } from "node:util";
import { SerialPort } from "serialport";
import { FeetechBus, range } from "./bus.ts";
import { describeError, hex } from "./protocol.ts";
import { MAX_FRAME_STEP, MotionError, moveToPose, name as jointName, playTrajectory, recordTrajectory, sleep } from "./motion.ts";
import { streamLeader } from "./leader.ts";
import { assertPoseName, parsePoseFile, serializePoseFile, type PoseFile } from "./poses.ts";
import { startServer } from "./server.ts";
import { durationSeconds, largestStep, parseTrajectory, serializeTrajectory } from "./trajectory.ts";
import {
  BAUD_RATES,
  JOINTS,
  MODEL_NUMBER,
  OperatingMode,
  RESOLUTION,
  STS3215,
  decodeAt,
  degreesToTicks,
  encodeValue,
  findRegister,
  ticksToDegrees,
  type Register,
} from "./sts3215.ts";

const USAGE = `so101 — talk to the Feetech STS3215 servos on an SO-101 bus

usage: so101 <command> [options]

  ports                           list serial ports; * marks USB adapters
  scan [--max 253]                ping every id and show what answers
  status [--ids 1-6]              one-shot telemetry per servo
  watch [--ids 1-6] [--hz 20]     live positions — move the arm by hand
  torque on|off [--ids 1-6]       hold or release the servos
  move <id> <target> [--deg] [--speed 500] [--acc 50] [--torque 100] [--hold]
                                  go to a position within the servo's limits: ticks, or degrees with --deg;
                                  --torque caps the effort and stays in force, --hold returns without waiting
  read <id> <register|address> [--len 1]
                                  read a register by its LeRobot name, or raw bytes
  set-id <current> <new>          re-address a servo (connect only that servo)
  record <file> [--ids 1-6] [--hz 30]
                                  release torque and sample the joints while you move the arm; ctrl-c saves
  play <file> [--rate 1] [--release]
                                  ease to the recording's first pose, then replay it at the recorded speed
  pose save <name> [--ids 1-6]    remember the current joint positions under a name (poses.json)
  pose go <name> [--via <name>] [--speed 300] [--torque 70] [--release]
                                  move every joint to a saved pose together; stops if any joint jams
  pose list | pose delete <name>
  serve [--listen 0.0.0.0:8101] [--file poses.json] [--model <dir with so101_new_calib.urdf>]
                                  HTTP API + WebSocket target stream for this arm (run where the follower is)
  leader --to ws://host:8101 [--hz 60]
                                  read this leader arm and drive a remote follower through its serve

options:
  --port <path>   serial port (default: $SO101_PORT, else the only USB adapter)
  --baud <rate>   bus baud rate (default: 1000000)
`;

const OPTIONS = {
  port: { type: "string" },
  baud: { type: "string" },
  ids: { type: "string" },
  max: { type: "string" },
  hz: { type: "string" },
  deg: { type: "boolean" },
  speed: { type: "string" },
  acc: { type: "string" },
  len: { type: "string" },
  rate: { type: "string" },
  release: { type: "boolean" },
  file: { type: "string" },
  via: { type: "string" },
  torque: { type: "string" },
  hold: { type: "boolean" },
  listen: { type: "string" },
  to: { type: "string" },
  model: { type: "string" },
} as const;

interface Values {
  port?: string;
  baud?: string;
  ids?: string;
  max?: string;
  hz?: string;
  deg?: boolean;
  speed?: string;
  acc?: string;
  len?: string;
  rate?: string;
  release?: boolean;
  file?: string;
  via?: string;
  torque?: string;
  hold?: boolean;
  listen?: string;
  model?: string;
  to?: string;
}

type Command = (values: Values, positionals: string[]) => Promise<void>;

export async function main(argv: string[]): Promise<number> {
  const [command, ...rest] = argv;
  if (command === undefined || ["help", "--help", "-h"].includes(command)) {
    process.stdout.write(USAGE);
    return 0;
  }
  const handler = COMMANDS[command];
  if (!handler) {
    process.stderr.write(`unknown command: ${command}\n\n${USAGE}`);
    return 2;
  }
  try {
    const { values, positionals } = parseArgs({ args: rest, options: OPTIONS, allowPositionals: true });
    await handler(values, positionals);
    return 0;
  } catch (err) {
    const hint = err instanceof MotionError ? " (torque is still on: so101 torque off releases the arm)" : "";
    process.stderr.write(`error: ${message(err)}${hint}\n`);
    return 1;
  }
}

function message(err: unknown): string {
  return (err instanceof Error ? err.message : String(err)).replace(/^Error: /, "");
}

const ADAPTER = /usbmodem|usbserial|ttyACM|ttyUSB/i;

const COMMANDS: Record<string, Command> = {
  ports: async () => {
    const list = await SerialPort.list();
    if (list.length === 0) {
      console.log("no serial ports found");
      return;
    }
    let adapters = 0;
    for (const port of list) {
      const adapter = ADAPTER.test(port.path);
      adapters += adapter ? 1 : 0;
      const vendor = port.vendorId ? `${port.vendorId}:${port.productId}` : "";
      const path = calloutDevice(port.path);
      console.log(`${adapter ? "*" : " "} ${[path, port.manufacturer, vendor].filter(Boolean).join("  ")}`);
    }
    console.log(
      adapters
        ? "\n* looks like a USB serial adapter; pass it with --port or SO101_PORT"
        : "\nnone of these look like a USB serial adapter — is the servo board plugged in?",
    );
  },

  scan: (values) => {
    const max = servoId(values.max ?? String(MAX_ID), "--max");
    return withBus(values, async (bus) => {
      console.log(`scanning ids 0-${max} on ${bus.path} at ${bus.baudRate} baud…`);
      const rows: string[][] = [];
      for (const id of range(0, max + 1)) {
        if (!(await bus.ping(id))) continue;
        try {
          const block = await bus.readBlock(id, 0, 7);
          const model = decodeAt(block, 0, STS3215.Model_Number);
          const position = await bus.read(id, STS3215.Present_Position);
          rows.push([
            String(id),
            JOINTS[id] ?? "",
            model === MODEL_NUMBER ? "STS3215" : `model ${model}`,
            `${block[0]}.${block[1]}`,
            String(BAUD_RATES[block[6]!] ?? `code ${block[6]}`),
            String(position),
          ]);
        } catch (err) {
          rows.push([String(id), JOINTS[id] ?? "", `answered the ping but not the read: ${message(err)}`]);
        }
      }
      if (rows.length === 0) {
        console.log("nothing answered — check power to the servo board and the cable to the servo");
        return;
      }
      console.log(table([["id", "joint", "model", "fw", "baud", "position"], ...rows]));
    });
  },

  status: (values) => {
    const ids = parseIds(values.ids);
    return withBus(values, async (bus) => {
      const rows: string[][] = [];
      const start = STS3215.Operating_Mode.address;
      const end = STS3215.Present_Current.address + STS3215.Present_Current.size;
      for (const id of ids) {
        const ping = await bus.ping(id);
        if (!ping) {
          rows.push([String(id), JOINTS[id] ?? "", "no response"]);
          continue;
        }
        let block: Uint8Array;
        try {
          block = await bus.readBlock(id, start, end - start);
        } catch (err) {
          rows.push([String(id), JOINTS[id] ?? "", `answered the ping but not the read: ${message(err)}`]);
          continue;
        }
        const at = (register: Register) => decodeAt(block, start, register);
        const position = at(STS3215.Present_Position);
        rows.push([
          String(id),
          JOINTS[id] ?? "",
          String(position),
          ticksToDegrees(position).toFixed(1),
          String(at(STS3215.Present_Velocity)),
          `${(at(STS3215.Present_Load) / 10).toFixed(1)}%`,
          `${Math.round(at(STS3215.Present_Current) * 6.5)}mA`,
          `${(at(STS3215.Present_Voltage) / 10).toFixed(1)}V`,
          `${at(STS3215.Present_Temperature)}°C`,
          at(STS3215.Torque_Enable) ? "on" : "off",
          modeName(at(STS3215.Operating_Mode)),
          describeError(at(STS3215.Status) | ping.error),
        ]);
      }
      console.log(
        table([
          ["id", "joint", "pos", "deg", "vel", "load", "current", "volt", "temp", "torque", "mode", "status"],
          ...rows,
        ]),
      );
    });
  },

  watch: (values) => {
    const ids = parseIds(values.ids);
    const hz = bounded(integer(values.hz ?? "20", "--hz"), 1, 100, "--hz");
    return withBus(values, async (bus) => {
      const period = 1000 / hz;
      let running = true;
      const stop = () => {
        running = false;
      };
      process.once("SIGINT", stop);
      console.log(`watching ${ids.join(",")} at ${hz} Hz — ctrl-c to stop\n`);
      let lines = 0;
      while (running) {
        const started = performance.now();
        const positions = await bus.syncRead(ids, STS3215.Present_Position);
        const rows = ids.map((id) => {
          const position = positions.get(id);
          const label = `${String(id).padStart(2)} ${(JOINTS[id] ?? "").padEnd(13)}`;
          if (position === undefined) return `${label} —`;
          const degrees = ticksToDegrees(position).toFixed(1).padStart(6);
          return `${label} ${String(position).padStart(4)}  ${degrees}°  ${bar(position)}`;
        });
        const up = lines ? `\x1b[${lines}A` : "";
        process.stdout.write(`${up}${rows.map((row) => `\x1b[2K${row}`).join("\n")}\n`);
        lines = rows.length;
        const remaining = period - (performance.now() - started);
        if (remaining > 0) await sleep(remaining);
      }
      process.off("SIGINT", stop);
    });
  },

  torque: (values, [state]) => {
    if (state !== "on" && state !== "off") throw new Error("usage: so101 torque on|off [--ids 1-6]");
    const ids = parseIds(values.ids);
    return withBus(values, async (bus) => {
      if (state === "on") {
        // Hold where each joint is now; a stale goal from an earlier run must not become a lunge.
        const positions = await bus.syncRead(ids, STS3215.Present_Position);
        const present = ids.filter((id) => positions.has(id));
        await bus.syncWrite(STS3215.Goal_Position, present.map((id) => [id, positions.get(id)!] as const));
      }
      await bus.syncWrite(
        STS3215.Torque_Enable,
        ids.map((id) => [id, state === "on" ? 1 : 0] as const),
      );
      console.log(`torque ${state} for ${ids.join(",")}`);
    });
  },

  move: (values, [idArg, targetArg]) => {
    if (idArg === undefined || targetArg === undefined) {
      throw new Error("usage: so101 move <id> <target> [--deg] [--speed 500] [--acc 50]");
    }
    const id = servoId(idArg, "id");
    const target = values.deg ? degreesToTicks(finite(targetArg, "target")) : integer(targetArg, "target");
    bounded(target, 0, RESOLUTION - 1, "target");
    // Goal_Velocity 0 means "as fast as the servo can", so it is not accepted here.
    const speed = bounded(integer(values.speed ?? "500", "--speed"), 1, 32767, "--speed");
    const acc = bounded(integer(values.acc ?? "50", "--acc"), 0, 255, "--acc");
    const torque = bounded(integer(values.torque ?? "100", "--torque"), 5, 100, "--torque");
    // Every value the move will write is encoded here, so nothing can fail after torque is on.
    encodeValue(STS3215.Goal_Velocity, speed);
    encodeValue(STS3215.Acceleration, acc);
    encodeValue(STS3215.Goal_Position, target);
    encodeValue(STS3215.Torque_Limit, torque * 10);
    return withBus(values, async (bus) => {
      const mode = await bus.read(id, STS3215.Operating_Mode);
      if (mode !== OperatingMode.Position) {
        throw new Error(`servo ${id} is in ${modeName(mode)} mode, not position mode`);
      }
      const min = await bus.read(id, STS3215.Min_Position_Limit);
      const max = await bus.read(id, STS3215.Max_Position_Limit);
      if (target < min || target > max) {
        throw new Error(`servo ${id} allows ${min}-${max} ticks (${ticksToDegrees(min).toFixed(1)}°-${ticksToDegrees(max).toFixed(1)}°); ${target} is outside`);
      }
      const from = await bus.read(id, STS3215.Present_Position);
      // Park the goal where the servo is before enabling torque, so a stale goal cannot become a lunge.
      await bus.write(id, STS3215.Goal_Position, from);
      await bus.write(id, STS3215.Torque_Limit, torque * 10);
      await bus.write(id, STS3215.Torque_Enable, 1);
      await bus.write(id, STS3215.Acceleration, acc);
      await bus.write(id, STS3215.Goal_Velocity, speed);
      await bus.write(id, STS3215.Goal_Position, target);
      console.log(`servo ${id}: ${from} → ${target} ticks (${ticksToDegrees(target).toFixed(1)}°) at ${speed} ticks/s${torque < 100 ? `, torque capped at ${torque}%` : ""}`);
      if (values.hold) return;

      const deadline = performance.now() + 10_000;
      let position = from;
      do {
        await sleep(50);
        position = await bus.read(id, STS3215.Present_Position);
        if (!(await bus.read(id, STS3215.Moving))) break;
      } while (performance.now() < deadline);
      console.log(`servo ${id}: now at ${position} ticks (${ticksToDegrees(position).toFixed(1)}°), torque still on`);
    });
  },

  read: (values, [idArg, what]) => {
    if (idArg === undefined || what === undefined) {
      throw new Error("usage: so101 read <id> <register|address> [--len 1]");
    }
    const id = servoId(idArg, "id");
    const named = findRegister(what);
    if (!named && !/^\d+$/.test(what)) {
      throw new Error(`unknown register "${what}"; names follow LeRobot's table (Present_Position, Goal_Position, Torque_Enable, …) or pass an address`);
    }
    const address = named ? named[1].address : bounded(integer(what, "register"), 0, 255, "register");
    const length = named ? named[1].size : bounded(integer(values.len ?? "1", "--len"), 1, 256 - address, "--len");
    return withBus(values, async (bus) => {
      const bytes = await bus.readBlock(id, address, length);
      if (named) {
        console.log(`${named[0]} @${address} = ${decodeAt(bytes, address, named[1])}  [${hex(bytes)}]`);
        return;
      }
      const value = Array.from(bytes).reduce((sum, byte, index) => sum + byte * 256 ** index, 0);
      console.log(`@${address}+${length} = ${value}  [${hex(bytes)}]`);
    });
  },

  record: (values, [file]) => {
    if (file === undefined) throw new Error("usage: so101 record <file> [--ids 1-6] [--hz 30]");
    const ids = parseIds(values.ids);
    const hz = bounded(integer(values.hz ?? "30", "--hz"), 1, 100, "--hz");
    // Claim the file now: refuse to overwrite anything, and find an unwritable path before the recording, not after.
    try {
      closeSync(openSync(file, "wx"));
    } catch (err) {
      throw new Error((err as NodeJS.ErrnoException).code === "EEXIST" ? `${file} already exists; pick another name` : `cannot create ${file}: ${message(err)}`);
    }
    return withBus(values, async (bus) => {
      console.log("releasing torque in 2 s — support the arm");
      await sleep(2000);
      const interrupt = onInterrupt();
      console.log(`torque off on ${ids.join(",")}; recording at ${hz} Hz — move the arm, ctrl-c when done`);
      const { trajectory, filled } = await recordTrajectory(bus, ids, {
        hz,
        signal: interrupt.signal,
        onFrame: (count) => {
          if (count % hz === 0) process.stdout.write(`\r${(count / hz).toFixed(0)} s, ${count} frames`);
        },
      });
      interrupt.dispose();
      if (trajectory.frames.length === 0) throw new Error("nothing recorded — did the servos answer?");
      writeFileSync(`${file}.tmp`, serializeTrajectory(trajectory));
      renameSync(`${file}.tmp`, file);
      console.log(`\nsaved ${trajectory.frames.length} frames (${(trajectory.frames.length / hz).toFixed(1)} s) to ${file}`);
      if (filled) console.log(`note: ${filled} frames had a missed reply filled from the previous frame`);
    });
  },

  play: (values, [file]) => {
    if (file === undefined) throw new Error("usage: so101 play <file> [--rate 1] [--release]");
    const rate = finite(values.rate ?? "1", "--rate");
    if (rate <= 0 || rate > 2) throw new Error("--rate must be within 0-2");
    const trajectory = parseTrajectory(readFileSync(file, "utf8"));
    const ids = [...trajectory.ids];
    const step = largestStep(trajectory);
    if (step > MAX_FRAME_STEP) {
      throw new Error(`${file} jumps ${step} ticks between two frames; a hand-made recording never does that, so refusing to replay it`);
    }
    const torque = bounded(integer(values.torque ?? "70", "--torque"), 10, 100, "--torque");
    return withBus(values, async (bus) => {
      const interrupt = onInterrupt();
      console.log(`easing to the first pose of ${file} (${durationSeconds(trajectory).toFixed(1)} s at rate ${rate})`);
      let played = 0;
      try {
        played = await playTrajectory(bus, trajectory, { rate, torquePercent: torque, signal: interrupt.signal, onFrame: (index) => (played = index) });
      } finally {
        interrupt.dispose();
      }
      if (values.release) {
        await bus.syncWrite(STS3215.Torque_Enable, ids.map((id) => [id, 0] as const));
        console.log(`played ${played} of ${trajectory.frames.length} frames; torque released`);
      } else {
        console.log(`played ${played} of ${trajectory.frames.length} frames; holding the final pose, torque on (so101 torque off to release)`);
      }
    });
  },

  pose: (values, [action, name]) => {
    const file = values.file ?? "poses.json";
    const load = (): PoseFile => {
      try {
        return parsePoseFile(readFileSync(file, "utf8"));
      } catch (err) {
        if ((err as NodeJS.ErrnoException).code === "ENOENT") return {};
        throw new Error(`${file}: ${message(err)}`);
      }
    };
    const need = (poses: PoseFile, wanted: string | undefined) => {
      if (wanted === undefined) throw new Error("which pose? give its name");
      assertPoseName(wanted);
      const pose = Object.hasOwn(poses, wanted) ? poses[wanted] : undefined;
      if (!pose) throw new Error(`no pose "${wanted}" in ${file}${Object.keys(poses).length ? ` (have: ${Object.keys(poses).join(", ")})` : ""}`);
      return pose;
    };

    switch (action) {
      case "list": {
        const poses = load();
        const names = Object.keys(poses);
        if (!names.length) return Promise.resolve(console.log(`no poses in ${file}`));
        console.log(table([["name", "ids", "positions", "saved"], ...names.map((n) => [n, poses[n]!.ids.join(","), poses[n]!.positions.join(" "), poses[n]!.saved ?? ""])]));
        return Promise.resolve();
      }
      case "delete": {
        const poses = load();
        need(poses, name);
        const { [name!]: _, ...rest } = poses;
        writeFileSync(file, serializePoseFile(rest));
        console.log(`deleted "${name}" from ${file}`);
        return Promise.resolve();
      }
      case "save": {
        if (name === undefined) throw new Error("usage: so101 pose save <name> [--ids 1-6]");
        assertPoseName(name);
        const ids = parseIds(values.ids);
        return withBus(values, async (bus) => {
          const positions = await bus.syncRead(ids, STS3215.Present_Position);
          const missing = ids.filter((id) => !positions.has(id));
          if (missing.length) throw new Error(`no response from servo ${missing.join(", ")}`);
          const poses = { ...load(), [name]: { ids, positions: ids.map((id) => positions.get(id)!), saved: new Date().toISOString() } };
          writeFileSync(file, serializePoseFile(poses));
          console.log(`saved "${name}": ${ids.map((id) => `${JOINTS[id] ?? id}=${positions.get(id)}`).join(" ")}`);
          const onStops: string[] = [];
          for (const id of ids) {
            const min = await bus.read(id, STS3215.Min_Position_Limit);
            const max = await bus.read(id, STS3215.Max_Position_Limit);
            const pos = positions.get(id)!;
            if (pos - min < 40 || max - pos < 40) onStops.push(JOINTS[id] ?? String(id));
          }
          if (onStops.length) console.log(`note: ${onStops.join(", ")} resting on a hard stop; "go" will aim 30 ticks inside the limit instead of pushing into it`);
        });
      }
      case "go": {
        const poses = load();
        const target = need(poses, name);
        const via = values.via === undefined ? undefined : need(poses, values.via);
        const speed = bounded(integer(values.speed ?? "300", "--speed"), 20, 1500, "--speed");
        const torque = bounded(integer(values.torque ?? "70", "--torque"), 10, 100, "--torque");
        return withBus(values, async (bus) => {
          const interrupt = onInterrupt();
          try {
            for (const [label, pose] of [...(via ? [["via " + values.via, via] as const] : []), [name!, target] as const]) {
              const result = await moveToPose(bus, pose.ids, pose.positions, { speed, torquePercent: torque, signal: interrupt.signal });
              if (result.adjusted.size) {
                console.log(`aimed inside the limits for ${[...result.adjusted].map(([id, aim]) => `${jointName(id)}→${aim}`).join(", ")}`);
              }
              console.log(`reached "${label}": ${pose.ids.map((id) => `${jointName(id)}=${result.final.get(id)}`).join(" ")}`);
            }
          } finally {
            interrupt.dispose();
          }
          if (values.release) {
            const all = [...new Set([...(via?.ids ?? []), ...target.ids])];
            await bus.syncWrite(STS3215.Torque_Enable, all.map((id) => [id, 0] as const));
            console.log("torque released");
          } else {
            console.log("holding, torque on (so101 torque off to release)");
          }
        });
      }
      default:
        throw new Error("usage: so101 pose save|go|list|delete <name>");
    }
  },

  serve: (values) => {
    const [host, portText] = (values.listen ?? "0.0.0.0:8101").split(":");
    const port = bounded(integer(portText ?? "8101", "--listen port"), 1, 65535, "--listen port");
    return withBus(values, async (bus) => {
      const server = await startServer(bus, { host, port, posesFile: values.file ?? "poses.json", modelDir: values.model });
      console.log(`serving ${bus.path} on http://${host}:${server.port} — open it in a browser; the API is listed in src/server.ts`);
      console.log("ctrl-c to stop");
      await new Promise<void>((resolve) => process.once("SIGINT", () => resolve()));
      await server.close();
      await bus.syncWrite(STS3215.Torque_Enable, [1, 2, 3, 4, 5, 6].map((id) => [id, 0] as const));
      console.log("stopped; torque released");
    });
  },

  leader: (values) => {
    if (!values.to) throw new Error("usage: so101 leader --to ws://host:8101 [--hz 60]");
    const hz = bounded(integer(values.hz ?? "60", "--hz"), 1, 120, "--hz");
    return withBus(values, async (bus) => {
      const interrupt = onInterrupt();
      console.log(`streaming this leader to ${values.to} at ${hz} Hz — ctrl-c to stop`);
      let frames = 0;
      const sent = await streamLeader(bus, values.to!, {
        hz,
        signal: interrupt.signal,
        onFrame: () => {
          if (++frames % hz === 0) process.stdout.write(`\r${(frames / hz).toFixed(0)} s`);
        },
      });
      interrupt.dispose();
      console.log(`\nstopped after ${sent} frames; the follower is holding`);
    });
  },

  "set-id": (values, [currentArg, nextArg]) => {
    if (currentArg === undefined || nextArg === undefined) throw new Error("usage: so101 set-id <current> <new>");
    const current = servoId(currentArg, "current id");
    const next = servoId(nextArg, "new id");
    return withBus(values, async (bus) => {
      if (!(await bus.ping(current))) throw new Error(`servo ${current} did not answer`);
      if (next !== current && (await bus.ping(next))) {
        throw new Error(`a servo already answers as ${next}; connect only the servo you are re-addressing`);
      }
      await bus.write(current, STS3215.ID, next);
      if (!(await bus.ping(next))) throw new Error(`servo did not come back as ${next}`);
      console.log(`servo ${current} is now ${next}${JOINTS[next] ? ` (${JOINTS[next]})` : ""}`);
    });
  },
};

/** A recording made by hand never moves a joint this far between two samples. */

/** Ctrl-C becomes an abort signal the supervisor checks on every poll, so the arm stops where it is instead of running on unsupervised. */
function onInterrupt(): { signal: AbortSignal; dispose: () => void } {
  const controller = new AbortController();
  const handler = () => controller.abort();
  process.once("SIGINT", handler);
  return { signal: controller.signal, dispose: () => process.off("SIGINT", handler) };
}

async function withBus<T>(values: Values, fn: (bus: FeetechBus) => Promise<T>): Promise<T> {
  const path = values.port ?? process.env.SO101_PORT ?? (await detectPort());
  const baudRate = values.baud ? integer(values.baud, "--baud") : 1_000_000;
  const bus = await FeetechBus.open(path, { baudRate });
  try {
    return await fn(bus);
  } finally {
    await bus.close();
  }
}

async function detectPort(): Promise<string> {
  const candidates = (await SerialPort.list())
    .map((port) => port.path)
    .filter((path) => ADAPTER.test(path))
    .map(calloutDevice);
  if (candidates.length === 1) return candidates[0]!;
  if (candidates.length === 0) {
    throw new Error("no USB serial adapter found — plug in the servo board, or pass --port");
  }
  throw new Error(`several adapters found, pick one with --port:\n  ${candidates.join("\n  ")}`);
}

/** macOS exposes each port twice; the cu.* node is the right one for a host-initiated link. */
function calloutDevice(path: string): string {
  return process.platform === "darwin" ? path.replace(/^\/dev\/tty\./, "/dev/cu.") : path;
}

/** Servo ids run 0-253; 254 is the broadcast address. */
const MAX_ID = 253;

/** "1-6", "1,3,5" or "1-3,6". */
function parseIds(spec = "1-6"): number[] {
  const ids = new Set<number>();
  for (const part of spec.split(",")) {
    const match = /^(\d+)(?:-(\d+))?$/.exec(part);
    if (!match) throw new Error(`--ids takes ids and ranges like 1-3,6, got "${spec}"`);
    const lo = servoId(match[1]!, "--ids");
    const hi = match[2] === undefined ? lo : servoId(match[2], "--ids");
    if (hi < lo) throw new Error(`--ids range ${part} runs backwards`);
    for (let id = lo; id <= hi; id++) ids.add(id);
  }
  return [...ids];
}

function finite(text: string, what: string): number {
  const value = text.trim() === "" ? Number.NaN : Number(text);
  if (!Number.isFinite(value)) throw new Error(`${what} must be a number, got "${text}"`);
  return value;
}

function integer(text: string, what: string): number {
  const value = finite(text, what);
  if (!Number.isInteger(value)) throw new Error(`${what} must be an integer, got "${text}"`);
  return value;
}

function bounded(value: number, min: number, max: number, what: string): number {
  if (value < min || value > max) throw new Error(`${what} must be within ${min}-${max}, got ${value}`);
  return value;
}

function servoId(text: string, what: string): number {
  return bounded(integer(text, what), 0, MAX_ID, what);
}

function modeName(mode: number): string {
  const entry = Object.entries(OperatingMode).find(([, value]) => value === mode);
  return entry ? entry[0].toLowerCase() : `mode ${mode}`;
}

function bar(position: number, width = 32): string {
  const filled = Math.round((position / (RESOLUTION - 1)) * width);
  return `${"█".repeat(filled)}${"·".repeat(width - filled)}`;
}

function table(rows: string[][]): string {
  const widths = rows[0]!.map((_, column) => Math.max(...rows.map((row) => (row[column] ?? "").length)));
  return rows
    .map((row) => widths.map((width, i) => (row[i] ?? "").padEnd(width)).join("  ").trimEnd())
    .join("\n");
}

