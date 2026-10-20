import { describe, expect, test } from "bun:test";
import {
  STS3215,
  decodeAt,
  decodeSignMagnitude,
  decodeValue,
  degreesToTicks,
  encodeSignMagnitude,
  encodeValue,
  findRegister,
  ticksToDegrees,
} from "./sts3215.ts";

describe("sign-magnitude", () => {
  test("round-trips through LeRobot's encoding", () => {
    expect(encodeSignMagnitude(-5, 15)).toBe(0x8005);
    expect(encodeSignMagnitude(5, 15)).toBe(5);
    expect(decodeSignMagnitude(0x8005, 15)).toBe(-5);
    expect(decodeSignMagnitude(0x0405, 10)).toBe(-5);
    expect(() => encodeSignMagnitude(-2048, 11)).toThrow(RangeError);
  });
});

describe("encodeValue / decodeValue", () => {
  test("two-byte values are little-endian", () => {
    expect(encodeValue(STS3215.Goal_Position, 2048)).toEqual([0x00, 0x08]);
    expect(decodeValue(STS3215.Present_Position, Uint8Array.from([0xff, 0x0f]))).toBe(4095);
  });

  test("single-byte values pass through", () => {
    expect(encodeValue(STS3215.Torque_Enable, 1)).toEqual([1]);
    expect(decodeValue(STS3215.Present_Temperature, Uint8Array.from([37]))).toBe(37);
  });

  test("signed registers apply their sign bit", () => {
    expect(encodeValue(STS3215.Homing_Offset, -100)).toEqual([0x64, 0x08]);
    expect(decodeValue(STS3215.Present_Load, Uint8Array.from([0x2c, 0x05]))).toBe(-300);
  });

  test("rejects values that do not fit", () => {
    expect(() => encodeValue(STS3215.ID, 256)).toThrow(RangeError);
    expect(() => encodeValue(STS3215.Goal_Position, 1.5)).toThrow(RangeError);
  });
});

describe("decodeAt", () => {
  test("pulls a register out of a block read", () => {
    const block = Uint8Array.from([0x00, 0x08, 0x10, 0x00]);
    expect(decodeAt(block, STS3215.Present_Position.address, STS3215.Present_Position)).toBe(2048);
    expect(decodeAt(block, STS3215.Present_Position.address, STS3215.Present_Velocity)).toBe(16);
  });
});

describe("conversions", () => {
  test("4096 ticks per revolution", () => {
    expect(ticksToDegrees(2048)).toBe(180);
    expect(degreesToTicks(90)).toBe(1024);
  });

  test("findRegister is case-insensitive", () => {
    expect(findRegister("present_position")?.[0]).toBe("Present_Position");
    expect(findRegister("nope")).toBeUndefined();
  });
});
