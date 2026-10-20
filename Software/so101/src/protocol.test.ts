import { describe, expect, test } from "bun:test";
import {
  FrameParser,
  Instruction,
  checksum,
  describeError,
  encodeInstruction,
  hex,
} from "./protocol.ts";

describe("checksum", () => {
  test("is the inverted low byte of the sum", () => {
    expect(checksum([0x01, 0x02, 0x01])).toBe(0xfb);
    expect(checksum([0xff, 0xff])).toBe(0x01);
    expect(checksum([])).toBe(0xff);
  });
});

describe("encodeInstruction", () => {
  test("ping id 1 matches the Feetech manual's example", () => {
    expect(hex(encodeInstruction(1, Instruction.Ping))).toBe("ff ff 01 02 01 fb");
  });

  test("read present position (addr 56, 2 bytes) from id 1", () => {
    expect(hex(encodeInstruction(1, Instruction.Read, [56, 2]))).toBe("ff ff 01 04 02 38 02 be");
  });

  test("sync write goal position for two servos", () => {
    const packet = encodeInstruction(0xfe, Instruction.SyncWrite, [42, 2, 1, 0x00, 0x08, 2, 0xff, 0x0f]);
    expect(hex(packet)).toBe("ff ff fe 0a 83 2a 02 01 00 08 02 ff 0f 2f");
    expect(packet[3]).toBe(packet.length - 4);
  });
});

describe("FrameParser", () => {
  test("parses a status packet with params", () => {
    const parser = new FrameParser();
    const frames = parser.push(Uint8Array.from([0xff, 0xff, 0x01, 0x04, 0x00, 0x00, 0x08, 0xf2]));
    expect(frames).toHaveLength(1);
    expect(frames[0]!.id).toBe(1);
    expect(frames[0]!.error).toBe(0);
    expect(Array.from(frames[0]!.params)).toEqual([0x00, 0x08]);
  });

  test("reassembles frames split across chunks and skips leading noise", () => {
    const parser = new FrameParser();
    expect(parser.push(Uint8Array.from([0x13, 0xff, 0xff, 0x01]))).toHaveLength(0);
    expect(parser.push(Uint8Array.from([0x02]))).toHaveLength(0);
    const frames = parser.push(Uint8Array.from([0x00, 0xfc, 0xff, 0xff, 0x02, 0x02, 0x00, 0xfb]));
    expect(frames.map((f) => f.id)).toEqual([1, 2]);
  });

  test("an absurd length byte does not swallow the frames behind it", () => {
    const parser = new FrameParser();
    const frames = parser.push(Uint8Array.from([0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0x03, 0x02, 0x00, 0xfa]));
    expect(frames.map((f) => f.id)).toEqual([3]);
  });

  test("drops a frame with a bad checksum without losing the next one", () => {
    const parser = new FrameParser();
    const frames = parser.push(
      Uint8Array.from([0xff, 0xff, 0x01, 0x02, 0x00, 0x00, 0xff, 0xff, 0x03, 0x02, 0x00, 0xfa]),
    );
    expect(frames.map((f) => f.id)).toEqual([3]);
  });
});

describe("describeError", () => {
  test("names set bits", () => {
    expect(describeError(0)).toBe("ok");
    expect(describeError(0b100101)).toBe("voltage+overheat+overload");
    expect(describeError(0x40)).toBe("0x40");
  });
});
