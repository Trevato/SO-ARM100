/**
 * Feetech STS3215 control table, named exactly as LeRobot names it
 * (lerobot/motors/feetech/tables.py) so `so101 read 1 Present_Position`
 * reads the same way the LeRobot docs do. Addresses are bytes into the
 * servo's memory map; multi-byte values are little-endian.
 */

export type Area = "eeprom" | "sram" | "factory";

export interface Register {
  readonly address: number;
  readonly size: 1 | 2;
  /** EEPROM registers persist across power cycles and need the Lock dance to write. */
  readonly area: Area;
  /** Bit index of the sign for sign-magnitude values (LeRobot's encoding table). */
  readonly signBit?: number;
}

const reg = (address: number, size: 1 | 2, area: Area, signBit?: number): Register =>
  signBit === undefined ? { address, size, area } : { address, size, area, signBit };

export const STS3215 = {
  // EEPROM
  Firmware_Major_Version: reg(0, 1, "eeprom"),
  Firmware_Minor_Version: reg(1, 1, "eeprom"),
  Model_Number: reg(3, 2, "eeprom"),
  ID: reg(5, 1, "eeprom"),
  Baud_Rate: reg(6, 1, "eeprom"),
  Return_Delay_Time: reg(7, 1, "eeprom"),
  Response_Status_Level: reg(8, 1, "eeprom"),
  Min_Position_Limit: reg(9, 2, "eeprom"),
  Max_Position_Limit: reg(11, 2, "eeprom"),
  Max_Temperature_Limit: reg(13, 1, "eeprom"),
  Max_Voltage_Limit: reg(14, 1, "eeprom"),
  Min_Voltage_Limit: reg(15, 1, "eeprom"),
  Max_Torque_Limit: reg(16, 2, "eeprom"),
  Phase: reg(18, 1, "eeprom"),
  Unloading_Condition: reg(19, 1, "eeprom"),
  LED_Alarm_Condition: reg(20, 1, "eeprom"),
  P_Coefficient: reg(21, 1, "eeprom"),
  D_Coefficient: reg(22, 1, "eeprom"),
  I_Coefficient: reg(23, 1, "eeprom"),
  Minimum_Startup_Force: reg(24, 2, "eeprom"),
  CW_Dead_Zone: reg(26, 1, "eeprom"),
  CCW_Dead_Zone: reg(27, 1, "eeprom"),
  Protection_Current: reg(28, 2, "eeprom"),
  Angular_Resolution: reg(30, 1, "eeprom"),
  Homing_Offset: reg(31, 2, "eeprom", 11),
  Operating_Mode: reg(33, 1, "eeprom"),
  Protective_Torque: reg(34, 1, "eeprom"),
  Protection_Time: reg(35, 1, "eeprom"),
  Overload_Torque: reg(36, 1, "eeprom"),
  Velocity_closed_loop_P_proportional_coefficient: reg(37, 1, "eeprom"),
  Over_Current_Protection_Time: reg(38, 1, "eeprom"),
  Velocity_closed_loop_I_integral_coefficient: reg(39, 1, "eeprom"),
  // SRAM
  Torque_Enable: reg(40, 1, "sram"),
  Acceleration: reg(41, 1, "sram"),
  Goal_Position: reg(42, 2, "sram", 15),
  Goal_Time: reg(44, 2, "sram"),
  Goal_Velocity: reg(46, 2, "sram", 15),
  Torque_Limit: reg(48, 2, "sram"),
  Lock: reg(55, 1, "sram"),
  Present_Position: reg(56, 2, "sram", 15),
  Present_Velocity: reg(58, 2, "sram", 15),
  Present_Load: reg(60, 2, "sram", 10),
  Present_Voltage: reg(62, 1, "sram"),
  Present_Temperature: reg(63, 1, "sram"),
  Status: reg(65, 1, "sram"),
  Moving: reg(66, 1, "sram"),
  Present_Current: reg(69, 2, "sram"),
  Goal_Position_2: reg(71, 2, "sram"),
  // Factory
  Moving_Velocity: reg(80, 1, "factory"),
  Moving_Velocity_Threshold: reg(80, 1, "factory"),
  DTs: reg(81, 1, "factory"),
  Velocity_Unit_factor: reg(82, 1, "factory"),
  Hts: reg(83, 1, "factory"),
  Maximum_Velocity_Limit: reg(84, 1, "factory"),
  Maximum_Acceleration: reg(85, 1, "factory"),
  Acceleration_Multiplier: reg(86, 1, "factory"),
} as const satisfies Record<string, Register>;

export type RegisterName = keyof typeof STS3215;

export const MODEL_NUMBER = 777;
export const RESOLUTION = 4096;

/** Baud_Rate register value → bits per second. */
export const BAUD_RATES: Readonly<Record<number, number>> = {
  0: 1_000_000,
  1: 500_000,
  2: 250_000,
  3: 128_000,
  4: 115_200,
  5: 57_600,
  6: 38_400,
  7: 19_200,
};

export const OperatingMode = { Position: 0, Velocity: 1, Pwm: 2, Step: 3 } as const;

/** SO-101 joint names by servo id, as LeRobot assigns them. */
export const JOINTS: Readonly<Record<number, string>> = {
  1: "shoulder_pan",
  2: "shoulder_lift",
  3: "elbow_flex",
  4: "wrist_flex",
  5: "wrist_roll",
  6: "gripper",
};

export function findRegister(name: string): [RegisterName, Register] | undefined {
  const wanted = name.toLowerCase();
  for (const key of Object.keys(STS3215) as RegisterName[]) {
    if (key.toLowerCase() === wanted) return [key, STS3215[key]];
  }
  return undefined;
}

export const ticksToDegrees = (ticks: number): number => (ticks * 360) / RESOLUTION;
export const degreesToTicks = (degrees: number): number => Math.round((degrees * RESOLUTION) / 360);

export function encodeSignMagnitude(value: number, signBit: number): number {
  const magnitude = Math.abs(value);
  const max = (1 << signBit) - 1;
  if (magnitude > max) throw new RangeError(`|${value}| exceeds ${max} for a sign bit at ${signBit}`);
  return (value < 0 ? 1 << signBit : 0) | magnitude;
}

export function decodeSignMagnitude(encoded: number, signBit: number): number {
  const magnitude = encoded & ((1 << signBit) - 1);
  return (encoded >> signBit) & 1 ? -magnitude : magnitude;
}

/** Register value → the bytes that go on the wire. */
export function encodeValue(register: Register, value: number): number[] {
  if (!Number.isInteger(value)) throw new RangeError(`register values are integers, got ${value}`);
  const raw = register.signBit === undefined ? value : encodeSignMagnitude(value, register.signBit);
  const max = register.size === 1 ? 0xff : 0xffff;
  if (raw < 0 || raw > max) throw new RangeError(`${value} does not fit in ${register.size} byte(s)`);
  return register.size === 1 ? [raw] : [raw & 0xff, raw >> 8];
}

/** Bytes off the wire → register value. */
export function decodeValue(register: Register, bytes: Uint8Array): number {
  if (bytes.length !== register.size) {
    throw new RangeError(`expected ${register.size} byte(s), got ${bytes.length}`);
  }
  const raw = register.size === 1 ? bytes[0]! : bytes[0]! | (bytes[1]! << 8);
  return register.signBit === undefined ? raw : decodeSignMagnitude(raw, register.signBit);
}

/** Decode a register out of a block read that started at `blockAddress`. */
export function decodeAt(block: Uint8Array, blockAddress: number, register: Register): number {
  const offset = register.address - blockAddress;
  return decodeValue(register, block.subarray(offset, offset + register.size));
}
