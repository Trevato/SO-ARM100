/**
 * Feetech serial bus protocol (the "SCS/STS" half-duplex packet format).
 *
 * Every packet on the wire is:
 *
 *   0xFF 0xFF | ID | LENGTH | INSTRUCTION-or-ERROR | PARAMS... | CHECKSUM
 *
 * LENGTH counts everything after itself (so PARAMS.length + 2), and
 * CHECKSUM is the bitwise NOT of the low byte of ID + LENGTH + ... + PARAMS.
 * A status packet has the same shape, with the servo's error byte where the
 * instruction byte was. This module is pure: framing in, framing out.
 */

export const BROADCAST_ID = 0xfe;

export const Instruction = {
  Ping: 0x01,
  Read: 0x02,
  Write: 0x03,
  RegWrite: 0x04,
  Action: 0x05,
  SyncRead: 0x82,
  SyncWrite: 0x83,
} as const;
export type Instruction = (typeof Instruction)[keyof typeof Instruction];

/** Bits of the status packet's error byte. */
export const ErrorBit = {
  Voltage: 1 << 0,
  Angle: 1 << 1,
  Overheat: 1 << 2,
  Overcurrent: 1 << 3,
  Overload: 1 << 5,
} as const;

export function describeError(error: number): string {
  if (error === 0) return "ok";
  const names = (Object.keys(ErrorBit) as (keyof typeof ErrorBit)[])
    .filter((name) => error & ErrorBit[name])
    .map((name) => name.toLowerCase());
  const known = Object.values(ErrorBit).reduce((acc, bit) => acc | bit, 0);
  if (error & ~known) names.push(`0x${error.toString(16)}`);
  return names.join("+");
}

export function checksum(bytes: Iterable<number>): number {
  let sum = 0;
  for (const byte of bytes) sum += byte;
  return ~sum & 0xff;
}

/** The length byte counts instruction, params and checksum, and tops out at 250. */
export const MAX_PARAMS = 248;

export function encodeInstruction(
  id: number,
  instruction: Instruction,
  params: ArrayLike<number> = [],
): Uint8Array {
  if (params.length > MAX_PARAMS) throw new RangeError(`${params.length} params exceed the ${MAX_PARAMS}-byte packet limit`);
  const length = params.length + 2;
  const packet = new Uint8Array(4 + length);
  packet[0] = 0xff;
  packet[1] = 0xff;
  packet[2] = id;
  packet[3] = length;
  packet[4] = instruction;
  packet.set(params, 5);
  packet[packet.length - 1] = checksum(packet.subarray(2, packet.length - 1));
  return packet;
}

export interface StatusPacket {
  readonly id: number;
  readonly error: number;
  readonly params: Uint8Array;
  /** The exact bytes of the frame, for callers that need to recognise echoes. */
  readonly raw: Uint8Array;
}

/**
 * Incremental status-packet parser. Feed it whatever the serial port hands
 * you; it returns every complete, checksum-valid frame and keeps the rest.
 * Garbage between frames is skipped a byte at a time.
 */
export class FrameParser {
  #buffer = new Uint8Array(0);

  push(chunk: Uint8Array): StatusPacket[] {
    const joined = new Uint8Array(this.#buffer.length + chunk.length);
    joined.set(this.#buffer);
    joined.set(chunk, this.#buffer.length);

    const frames: StatusPacket[] = [];
    let offset = 0;
    while (joined.length - offset >= 4) {
      if (joined[offset] !== 0xff || joined[offset + 1] !== 0xff) {
        offset += 1;
        continue;
      }
      const id = joined[offset + 2]!;
      const length = joined[offset + 3]!;
      // 0xFE is kept: an echoed broadcast instruction is how the bus detects echo.
      if (id === 0xff || length < 2 || length > MAX_PARAMS + 2) {
        offset += 1;
        continue;
      }
      const end = offset + 4 + length;
      if (end > joined.length) break;
      const raw = joined.slice(offset, end);
      if (checksum(raw.subarray(2, raw.length - 1)) !== raw[raw.length - 1]) {
        offset += 1;
        continue;
      }
      frames.push({ id, error: raw[4]!, params: raw.slice(5, raw.length - 1), raw });
      offset = end;
    }
    this.#buffer = joined.slice(offset);
    return frames;
  }

  reset(): void {
    this.#buffer = new Uint8Array(0);
  }
}

export function bytesEqual(a: Uint8Array, b: Uint8Array): boolean {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) if (a[i] !== b[i]) return false;
  return true;
}

export function hex(bytes: Uint8Array): string {
  return Array.from(bytes, (b) => b.toString(16).padStart(2, "0")).join(" ");
}
