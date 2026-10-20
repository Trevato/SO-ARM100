import { describe, expect, test } from "bun:test";
import { mapGripper, mapJoint } from "./mapping.ts";

describe("leader → follower mapping", () => {
  test("arm joints keep their angle from the middle of the range", () => {
    const leader = { min: 857, max: 3218 }; // mid 2037.5
    const follower = { min: 818, max: 3487 }; // mid 2152.5
    expect(mapJoint(2037.5, leader, follower)).toBe(2153);
    expect(mapJoint(2037.5 + 500, leader, follower)).toBe(2653);
  });

  test("arm joints never aim past the follower's limits minus the margin", () => {
    const leader = { min: 857, max: 3218 };
    const follower = { min: 818, max: 3487 };
    expect(mapJoint(0, leader, follower)).toBe(848);
    expect(mapJoint(4095, leader, follower)).toBe(3457);
  });

  test("the trigger's travel maps onto the jaw's travel", () => {
    const trigger = { min: 2047, max: 3515 };
    const jaw = { min: 1995, max: 3525 };
    expect(mapGripper(2047, trigger, jaw)).toBe(1995);
    expect(mapGripper(3515, trigger, jaw)).toBe(3525);
    expect(mapGripper(2781, trigger, jaw)).toBe(2760);
    expect(mapGripper(1000, trigger, jaw)).toBe(1995);
  });
});
