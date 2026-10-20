import { describe, expect, test } from "bun:test";
import { parsePoseFile, serializePoseFile } from "./poses.ts";

describe("pose file", () => {
  test("round-trips", () => {
    const poses = { home: { ids: [1, 2], positions: [2048, 2000], saved: "2026-10-13T00:00:00Z" } };
    expect(parsePoseFile(serializePoseFile(poses))).toEqual(poses);
  });

  test("rejects malformed entries with the pose name", () => {
    expect(() => parsePoseFile("[]")).toThrow("object");
    expect(() => parsePoseFile('{"x":{"ids":[1],"positions":[1,2]}}')).toThrow('pose "x"');
    expect(() => parsePoseFile('{"x":{"ids":[300],"positions":[1]}}')).toThrow('pose "x"');
  });
});

describe("pose names and ids", () => {
  test("rejects names that are also object properties, and duplicate ids", () => {
    expect(() => parsePoseFile('{"constructor":{"ids":[1],"positions":[1]}}')).toThrow("not a usable pose name");
    expect(() => parsePoseFile('{"__proto__":{"ids":[1],"positions":[1]}}')).toThrow("not a usable pose name");
    expect(() => parsePoseFile('{"a b":{"ids":[1],"positions":[1]}}')).toThrow("not a usable pose name");
    expect(() => parsePoseFile('{"x":{"ids":[1,1],"positions":[1,2]}}')).toThrow("distinct");
  });
});
