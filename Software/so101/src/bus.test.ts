import { describe, expect, test } from "bun:test";
import { BASE_TIMEOUT_MS, FeetechBus, NoResponseError } from "./bus.ts";
import { FakeBus } from "./fake-bus.ts";
import { STS3215 } from "./sts3215.ts";

describe("FeetechBus", () => {
  test("ping answers for present servos and null for absent ones", async () => {
    const bus = new FeetechBus(new FakeBus([1, 2]));
    expect((await bus.ping(1))?.id).toBe(1);
    expect(await bus.ping(7)).toBeNull();
  });

  test("scan lists responders in order", async () => {
    const bus = new FeetechBus(new FakeBus([5, 1, 3]));
    expect(await bus.scan([0, 1, 2, 3, 4, 5, 6])).toEqual([1, 3, 5]);
  });

  test("read decodes little-endian and write lands in memory", async () => {
    const fake = new FakeBus([1]);
    const bus = new FeetechBus(fake);
    expect(await bus.read(1, STS3215.Present_Position)).toBe(2048);
    expect(await bus.read(1, STS3215.Model_Number)).toBe(777);
    await bus.write(1, STS3215.Goal_Position, 4095);
    expect([...fake.servos.get(1)!.subarray(42, 44)]).toEqual([0xff, 0x0f]);
  });

  test("read of a missing servo throws NoResponseError", async () => {
    const bus = new FeetechBus(new FakeBus([1]));
    await expect(bus.read(9, STS3215.Present_Position)).rejects.toBeInstanceOf(NoResponseError);
  });

  test("syncRead returns what answered and omits what did not", async () => {
    const bus = new FeetechBus(new FakeBus([1, 2, 3], { drop: new Set([2]) }));
    const positions = await bus.syncRead([1, 2, 3], STS3215.Present_Position);
    expect([...positions.entries()]).toEqual([
      [1, 2048],
      [3, 2048],
    ]);
  });

  test("syncWrite sets every listed servo without waiting for replies", async () => {
    const fake = new FakeBus([1, 2]);
    const bus = new FeetechBus(fake);
    await bus.syncWrite(STS3215.Torque_Enable, [
      [1, 1],
      [2, 0],
    ]);
    expect(fake.writes).toEqual([
      { id: 1, address: 40, bytes: [1] },
      { id: 2, address: 40, bytes: [0] },
    ]);
  });

  test("EEPROM writes follow LeRobot's torque-off, unlock, write, lock sequence, and an ID write is acknowledged by the new id", async () => {
    const fake = new FakeBus([1]);
    const bus = new FeetechBus(fake);
    await bus.write(1, STS3215.ID, 4);
    expect(fake.writes).toEqual([
      { id: 1, address: 40, bytes: [0] },
      { id: 1, address: 55, bytes: [0] },
      { id: 1, address: 5, bytes: [4] },
      { id: 4, address: 55, bytes: [1] },
    ]);
    expect((await bus.ping(4))?.id).toBe(4);
    expect(await bus.ping(1)).toBeNull();
  });

  test("detectEcho tells an echoing adapter from a quiet one", async () => {
    expect(await new FeetechBus(new FakeBus([1], { echo: true })).detectEcho()).toBe(true);
    expect(await new FeetechBus(new FakeBus([1])).detectEcho()).toBe(false);
  });

  test("ignores the adapter echoing our own packet", async () => {
    const bus = new FeetechBus(new FakeBus([1], { echo: true }), { echo: true });
    expect(await bus.read(1, STS3215.Present_Position)).toBe(2048);
    expect((await bus.ping(1))?.error).toBe(0);
  });

  test("a ping reply that merely sets the voltage bit is not mistaken for an echo", async () => {
    // FF FF 01 02 01 FB is both the ping and a reply carrying error bit 0.
    const quiet = new FeetechBus(new FakeBus([1], { pingError: 1 }));
    expect((await quiet.ping(1))?.error).toBe(1);
    const echoing = new FeetechBus(new FakeBus([1], { pingError: 1, echo: true }), { echo: true });
    expect((await echoing.ping(1))?.error).toBe(1);
  });

  test("a reply that arrives after its deadline is not taken for the next answer", async () => {
    const bus = new FeetechBus(new FakeBus([1], { lateOnce: new Map([[1, BASE_TIMEOUT_MS + 10]]) }));
    await expect(bus.read(1, STS3215.Present_Position)).rejects.toBeInstanceOf(NoResponseError);
    expect(await bus.read(1, STS3215.Present_Velocity)).toBe(16);
  });

  test("concurrent calls are serialised on the bus", async () => {
    const bus = new FeetechBus(new FakeBus([1, 2]));
    const [a, b, c] = await Promise.all([
      bus.read(1, STS3215.ID),
      bus.read(2, STS3215.ID),
      bus.ping(3),
    ]);
    expect([a, b, c]).toEqual([1, 2, null]);
  });
});

describe("FeetechBus syncWrite ordering for playback", () => {
  test("writes goal positions for every servo in one packet per frame", async () => {
    const fake = new FakeBus([1, 2, 3]);
    const bus = new FeetechBus(fake);
    await bus.syncWrite(STS3215.Goal_Position, [
      [1, 1000],
      [2, 2000],
      [3, 3000],
    ]);
    expect(fake.writes.map((w) => [w.id, w.bytes])).toEqual([
      [1, [0xe8, 0x03]],
      [2, [0xd0, 0x07]],
      [3, [0xb8, 0x0b]],
    ]);
  });
});
