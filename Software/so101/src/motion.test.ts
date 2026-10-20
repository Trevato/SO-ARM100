import { describe, expect, test } from "bun:test";
import { FeetechBus } from "./bus.ts";
import { FakeBus } from "./fake-bus.ts";
import { MotionError, moveToPose } from "./motion.ts";

const fast = { speed: 500, pollMs: 2 };

describe("moveToPose", () => {
  test("reaches the pose, scales velocities to arrive together, and restores the torque cap", async () => {
    const fake = new FakeBus([1, 2]);
    const bus = new FeetechBus(fake);
    const result = await moveToPose(bus, [1, 2], [2548, 2148], fast);
    expect([...result.final.values()]).toEqual([2548, 2148]);
    const velocity = fake.writes.filter((w) => w.address === 46).map((w) => [w.id, w.bytes[0]! | (w.bytes[1]! << 8)]);
    expect(velocity.slice(-2)).toEqual([
      [1, 500],
      [2, 100],
    ]); // joint 2 has a fifth of the distance, so a fifth of the speed
    const caps = fake.writes.filter((w) => w.address === 48).map((w) => w.bytes[0]! | (w.bytes[1]! << 8));
    expect(caps).toEqual([700, 700, 1000, 1000]);
    // torque is enabled only after the goal is parked at the present position, so nothing can lunge
    const order = fake.writes.filter((w) => w.id === 1).map((w) => w.address);
    expect(order.indexOf(42)).toBeLessThan(order.indexOf(40));
  });

  test("respects a servo whose own ceiling is below the requested cap, and puts that ceiling back afterward", async () => {
    const fake = new FakeBus([1]);
    fake.servos.get(1)!.set([0xf4, 0x01], 16); // Max_Torque_Limit 500, like LeRobot's gripper config
    fake.servos.get(1)!.set([0x2c, 0x01], 48); // Torque_Limit 300 left behind by a grip: not the servo's own cap
    await moveToPose(new FeetechBus(fake), [1], [2100], fast);
    const caps = fake.writes.filter((w) => w.address === 48).map((w) => w.bytes[0]! | (w.bytes[1]! << 8));
    expect(caps).toEqual([500, 500]);
  });

  test("targets on a hard stop are pulled inside the margin; a narrow range aims at its middle", async () => {
    const fake = new FakeBus([1, 2]);
    fake.servos.get(2)!.set([0xd0, 0x07, 0xe4, 0x07], 9); // limits 2000-2020
    const bus = new FeetechBus(fake);
    const result = await moveToPose(bus, [1, 2], [4095, 2000], fast);
    expect([...result.adjusted]).toEqual([
      [1, 4065],
      [2, 2010],
    ]);
  });

  test("refuses a servo whose limits are not calibrated", async () => {
    const fake = new FakeBus([1]);
    fake.servos.get(1)!.set([0, 0, 0, 0], 9); // min = max = 0
    await expect(moveToPose(new FeetechBus(fake), [1], [100], fast)).rejects.toThrow("not calibrated");
  });

  test("a jammed joint stops the move and holds where it is", async () => {
    const fake = new FakeBus([1, 2], { stuck: new Set([2]) });
    const bus = new FeetechBus(fake);
    await expect(moveToPose(bus, [1, 2], [2548, 2548], fast)).rejects.toThrow("shoulder_lift jammed");
    const lastGoals = fake.writes.filter((w) => w.address === 42).slice(-2).map((w) => [w.id, w.bytes[0]! | (w.bytes[1]! << 8)]);
    expect(lastGoals).toEqual([
      [1, 2548],
      [2, 2048],
    ]); // goals reset to present positions
    const caps = fake.writes.filter((w) => w.address === 48).map((w) => w.bytes[0]! | (w.bytes[1]! << 8));
    expect(caps.slice(-2)).toEqual([1000, 1000]);
  });

  test("a jammed joint that chatters by a few ticks is still caught", async () => {
    const fake = new FakeBus([1], { stuck: new Set([1]), jitter: true });
    fake.servos.get(1)!.set([0, 0], 56); // start at 0 so the wobble stays within 0-6
    const started = performance.now();
    await expect(moveToPose(new FeetechBus(fake), [1], [2000], fast)).rejects.toThrow("jammed");
    expect(performance.now() - started).toBeLessThan(1000); // the 3 s timeout did not do the job
  });

  test("a servo that stops answering mid-move aborts and releases it", async () => {
    // arm() reads limits (2) and position (1); each poll reads position then load: go silent at the second poll's position read
    const fake = new FakeBus([1, 2], { stuck: new Set([2]), dropAfter: new Map([[2, 4]]) }); // arm() reads each servo 4 times; the first poll then finds 2 gone
    const bus = new FeetechBus(fake);
    const err = await moveToPose(bus, [1, 2], [2548, 2548], fast).catch((e) => e);
    expect(err).toBeInstanceOf(MotionError);
    expect(err.missing).toEqual([2]);
    expect(fake.writes.filter((w) => w.address === 40 && w.id === 2).at(-1)?.bytes).toEqual([0]); // torque released on the lost joint
  });

  test("an abort signal stops the move where it is", async () => {
    const fake = new FakeBus([1], { stuck: new Set([1]) });
    const controller = new AbortController();
    setTimeout(() => controller.abort(), 3);
    await expect(moveToPose(new FeetechBus(fake), [1], [3000], { ...fast, signal: controller.signal })).rejects.toThrow("interrupted");
  });
});
