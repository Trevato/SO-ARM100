import { BROADCAST_ID, FrameParser, Instruction, checksum } from "./protocol.ts";
import { STS3215 } from "./sts3215.ts";
import type { Transport } from "./bus.ts";

export interface FakeOptions {
  /** Hand every transmitted packet back before any reply, like a looped-back adapter. */
  echo?: boolean;
  /** Ids that never answer. */
  drop?: Set<number>;
  /** Ids whose first reply is held back this many ms (later replies are prompt). */
  lateOnce?: Map<number, number>;
  /** Error byte every ping reply carries. */
  pingError?: number;
  /** Ids whose output never moves and that report heavy load when commanded, like a jammed joint. */
  stuck?: Set<number>;
  /** Stuck ids wobble by ±3 ticks between reads, like a joint chattering against an obstacle. */
  jitter?: boolean;
  /** Ids that go silent after answering this many more times (deterministic mid-move dropout). */
  dropAfter?: Map<number, number>;
}

/** Enough of an STS3215 to answer the bus: a memory map per id, replies by instruction. */
export class FakeBus implements Transport {
  readonly path = "fake";
  readonly baudRate = 1_000_000;
  readonly servos = new Map<number, Uint8Array>();
  readonly writes: Array<{ id: number; address: number; bytes: number[] }> = [];
  #listener: ((chunk: Uint8Array) => void) | undefined;
  readonly #parser = new FrameParser();
  readonly #options: FakeOptions;

  constructor(ids: number[], options: FakeOptions = {}) {
    this.#options = options;
    for (const id of ids) {
      const memory = new Uint8Array(100);
      memory[3] = 777 & 0xff;
      memory[4] = 777 >> 8;
      memory[5] = id;
      memory[11] = 0xff;
      memory[12] = 0x0f; // Max_Position_Limit 4095 (Min stays 0)
      memory[16] = 0xe8;
      memory[17] = 0x03; // Max_Torque_Limit 1000
      memory[48] = 0xe8;
      memory[49] = 0x03; // Torque_Limit 1000
      memory[56] = 0x00;
      memory[57] = 0x08; // Present_Position 2048
      memory[58] = 16; // Present_Velocity 16
      this.servos.set(id, memory);
    }
  }

  on(event: "data" | "error", listener: ((chunk: Uint8Array) => void) | ((error: Error) => void)): void {
    if (event === "data") this.#listener = listener as (chunk: Uint8Array) => void;
  }

  write(data: Uint8Array, callback: (error: Error | null | undefined) => void): void {
    callback(null);
    const replies: Array<[number, Uint8Array]> = this.#options.echo ? [[0, data]] : [];
    const replyFrom = (id: number, frame: Uint8Array) => {
      const left = this.#options.dropAfter?.get(id);
      if (left !== undefined) {
        if (left <= 0) {
          (this.#options.drop ??= new Set()).add(id);
          return;
        }
        this.#options.dropAfter!.set(id, left - 1);
      }
      const delay = this.#options.lateOnce?.get(id) ?? 0;
      this.#options.lateOnce?.delete(id);
      replies.push([delay, frame]);
    };
    // Instruction packets have the status-packet shape, so the parser reads them too.
    for (const packet of this.#parser.push(data)) {
      const instruction = packet.error;
      const targets = packet.id === BROADCAST_ID ? [...this.servos.keys()].sort((a, b) => a - b) : [packet.id];
      for (const id of targets) {
        const memory = this.servos.get(id);
        if (!memory) continue;
        // A dropped servo still hears writes (and acts on them); it just never answers.
        const reply = this.#options.drop?.has(id) ? () => undefined : replyFrom;
        if (instruction === Instruction.Ping) reply(id, status(id, [], this.#options.pingError ?? 0));
        if (instruction === Instruction.Read) {
          const [address, length] = packet.params;
          reply(id, status(id, [...memory.subarray(address!, address! + length!)]));
        }
        if (this.#options.jitter && this.#options.stuck?.has(id) && (instruction === Instruction.Read || instruction === Instruction.SyncRead)) {
          memory[56] = (memory[56]! + 3) % 7; // wobble within 0-6 ticks of zero without ever settling
        }
        if (instruction === Instruction.Write) {
          const [address, ...bytes] = packet.params;
          memory.set(bytes, address!);
          this.writes.push({ id, address: address!, bytes });
          this.#follow(id, address!);
          if (address === STS3215.ID.address) {
            this.#rekey(id, bytes[0]!);
            reply(bytes[0]!, status(bytes[0]!, [])); // a real STS3215 answers wearing its new id
          } else {
            reply(id, status(id, []));
          }
        }
        if (instruction === Instruction.SyncRead && packet.id === BROADCAST_ID) {
          const [address, length, ...ids] = packet.params;
          if (ids.includes(id)) reply(id, status(id, [...memory.subarray(address!, address! + length!)]));
        }
        if (instruction === Instruction.SyncWrite && packet.id === BROADCAST_ID) {
          const [address, length, ...rest] = packet.params;
          for (let i = 0; i < rest.length; i += 1 + length!) {
            if (rest[i] !== id) continue;
            const bytes = rest.slice(i + 1, i + 1 + length!);
            memory.set(bytes, address!);
            this.writes.push({ id, address: address!, bytes });
            this.#follow(id, address!);
          }
        }
      }
    }
    for (const [delay, frame] of replies) setTimeout(() => this.#listener?.(frame), delay);
  }

  drain(callback: (error: Error | null | undefined) => void): void {
    callback(null);
  }

  close(callback: (error: Error | null | undefined) => void): void {
    callback(null);
  }

  /** A written goal becomes the present position at once, unless the servo is stuck, which then reports 90 % load. */
  #follow(id: number, address: number): void {
    if (address !== STS3215.Goal_Position.address) return;
    const memory = this.servos.get(id)!;
    if (this.#options.stuck?.has(id)) {
      memory[60] = 900 & 0xff;
      memory[61] = 900 >> 8;
      return;
    }
    memory[56] = memory[42]!;
    memory[57] = memory[43]!;
  }

  #rekey(from: number, to: number): void {
    const memory = this.servos.get(from)!;
    this.servos.delete(from);
    this.servos.set(to, memory);
  }
}

function status(id: number, params: number[], error = 0): Uint8Array {
  const body = [id, params.length + 2, error, ...params];
  return Uint8Array.from([0xff, 0xff, ...body, checksum(body)]);
}
