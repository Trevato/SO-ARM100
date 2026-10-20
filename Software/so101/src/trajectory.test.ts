import { describe, expect, test } from "bun:test";
import { clampFrame, durationSeconds, parseTrajectory, serializeTrajectory } from "./trajectory.ts";

describe("trajectory", () => {
  test("round-trips through JSON", () => {
    const original = { hz: 30, ids: [1, 2], frames: [[100, 200], [110, 210]], recorded: "2026-10-12T00:00:00Z" };
    expect(parseTrajectory(serializeTrajectory(original))).toEqual(original);
    expect(durationSeconds(original)).toBeCloseTo(2 / 30);
  });

  test("rejects malformed files with a reason", () => {
    expect(() => parseTrajectory("nope")).toThrow("not valid JSON");
    expect(() => parseTrajectory('{"hz":0,"ids":[1],"frames":[[1]]}')).toThrow("hz");
    expect(() => parseTrajectory('{"hz":30,"ids":[],"frames":[[1]]}')).toThrow("ids");
    expect(() => parseTrajectory('{"hz":30,"ids":[1],"frames":[]}')).toThrow("frames");
    expect(() => parseTrajectory('{"hz":30,"ids":[1,2],"frames":[[1]]}')).toThrow("frame 0");
    expect(() => parseTrajectory('{"hz":30,"ids":[1],"frames":[[1.5]]}')).toThrow("frame 0");
  });

  test("clampFrame keeps positions inside each servo's limits", () => {
    expect(clampFrame([50, 4000, 2048], [[100, 3000], [100, 3000], [0, 4095]])).toEqual([100, 3000, 2048]);
  });
});

describe("trajectory bounds", () => {
  test("rejects an infinite or absurd rate and duplicate ids", () => {
    expect(() => parseTrajectory('{"hz":1e999,"ids":[1],"frames":[[1]]}')).toThrow("hz");
    expect(() => parseTrajectory('{"hz":500,"ids":[1],"frames":[[1]]}')).toThrow("hz");
    expect(() => parseTrajectory('{"hz":30,"ids":[1,1],"frames":[[1,2]]}')).toThrow("distinct");
  });
});
