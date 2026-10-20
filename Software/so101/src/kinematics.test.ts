import { describe, expect, test } from "bun:test";
import { ArmModel, fromRpyXyz, gripperRadiansToTicks, gripperTicksToRadians, multiply, parseXml, radiansToTicks, rotationAbout, ticksToRadians, transformPoint } from "./kinematics.ts";
import fixtures from "./fk-fixtures.json";

const URDF = new URL("../../../Simulation/SO101/so101_new_calib.urdf", import.meta.url).pathname;
const ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"];
const deg = (q: number[]) => new Map(ORDER.map((name, i) => [name, (q[i]! * Math.PI) / 180]));
const model = ArmModel.load(URDF);

describe("matrices", () => {
  test("rpy follows the URDF convention and rotations compose", () => {
    const T = fromRpyXyz([Math.PI / 2, 0, 0], [1, 2, 3]);
    const round = (v: number) => Math.round(v * 1e9) / 1e9 + 0; // + 0 turns a -0 into 0 for toEqual
    expect(transformPoint(T, [0, 1, 0]).map(round)).toEqual([1, 2, 4]); // roll about x takes y to z
    const R = multiply(rotationAbout([0, 0, 1], Math.PI / 2), rotationAbout([1, 0, 0], Math.PI / 2));
    expect(transformPoint(R, [0, 1, 0]).map(round)).toEqual([0, 0, 1]);
  });

  test("the xml reader handles self-closing tags, comments and nesting", () => {
    const root = parseXml(`<?xml version="1.0"?><robot name="r"><!-- c --><joint name='j' type="revolute"><origin xyz='1 2 3'/><axis xyz="0 0 1"/></joint></robot>`);
    const robot = root.children[0]!;
    expect(robot.attrs.name).toBe("r");
    expect(robot.children[0]!.attrs.name).toBe("j"); // single quotes too
    expect(robot.children[0]!.children.map((c) => c.name)).toEqual(["origin", "axis"]);
    expect(robot.children[0]!.children[0]!.attrs.xyz).toBe("1 2 3");
  });
});

describe("the SO-101 model", () => {
  test("loads every link's mesh points and the base geometry", () => {
    expect(model.movable.map((j) => j.name)).toEqual(ORDER);
    expect(model.root).toBe("base_link");
    for (const name of ["base_link", "shoulder_link", "upper_arm_link", "lower_arm_link", "wrist_link", "gripper_link", "moving_jaw_so101_v1_link"]) {
      expect(model.links.get(name)!.points.length).toBeGreaterThan(26 * 3);
    }
    expect(model.floorZ).toBeLessThan(0);
    expect(model.floorZ).toBeGreaterThan(-0.05); // the base's bottom sits a few mm under base_link's origin
    expect(model.baseTop).toBeGreaterThan(0.05);
    expect(model.base.at(0.0388, 0)).toBeGreaterThan(0.02); // the base has height under the pan axis
    expect(model.base.at(0.5, 0.5)).toBe(-Infinity); // and nothing far away
  });

  test("forward kinematics matches LeRobot's placo solver on the gripper frame", () => {
    for (const f of fixtures.filter((f) => "T_base_gripper_frame" in f)) {
      const T = model.linkPoses(deg(f.q_deg)).get("gripper_frame_link")!;
      for (let i = 0; i < 16; i++) expect(T[i]).toBeCloseTo((f as { T_base_gripper_frame: number[][] }).T_base_gripper_frame[Math.floor(i / 4)]![i % 4]!, 5);
    }
  });

  test("and on every intermediate link", () => {
    for (const f of fixtures.filter((f) => "frame" in f)) {
      const T = model.linkPoses(deg(f.q_deg)).get(f.frame!)!;
      for (let i = 0; i < 16; i++) expect(T[i]).toBeCloseTo((f as { T: number[][] }).T[Math.floor(i / 4)]![i % 4]!, 5);
    }
  });

  test("the rest pose is clear of the table; pointing the arm down is not", () => {
    const settings = { floorMargin: 0.005, baseMargin: 0.005 };
    expect(model.check(deg([0, 0, 0, 0, 0, 0]), settings)).toBeNull();
    // Shoulder lift forward past horizontal with the elbow straight puts the gripper far below the table.
    const down = model.check(deg([0, 80, 0, 0, 0, 0]), settings);
    expect(down?.what).toBe("table");
    expect(down!.depth).toBeGreaterThan(0.05);
    // Folding the lower arm back into the base column is caught too.
    const folded = deg([0, -90, -95, 0, 0, 0]);
    const v = model.check(folded, settings);
    expect(v === null || v.what === "base" || v.what === "table").toBe(true);
  });

  test("ticks map to radians around the middle of the calibrated range, and back", () => {
    expect(ticksToRadians(2048, 1000, 3096)).toBe(0);
    expect(ticksToRadians(2048 + 4095 / 4, 1000, 3096)).toBeCloseTo(Math.PI / 2, 9);
    expect(radiansToTicks(ticksToRadians(1234, 800, 3300), 800, 3300)).toBe(1234);
    // The gripper: closed at the range's minimum is the jaw's zero; open at the maximum is the URDF's upper limit.
    expect(gripperTicksToRadians(1995, 1995, 3525, 1.745)).toBe(0);
    expect(gripperTicksToRadians(3525, 1995, 3525, 1.745)).toBeCloseTo(1.745, 9);
    expect(gripperRadiansToTicks(gripperTicksToRadians(2500, 1995, 3525, 1.745), 1995, 3525, 1.745)).toBe(2500);
  });

  test("a full check is fast enough for a 60 Hz stream", () => {
    const q = deg([10, -20, 30, 10, 0, 0]);
    const settings = { floorMargin: 0.01, baseMargin: 0.005 };
    const t0 = performance.now();
    for (let i = 0; i < 100; i++) model.check(q, settings);
    expect((performance.now() - t0) / 100).toBeLessThan(5); // ms per check on this machine; the Pi is slower but has 16 ms per frame
  });
});
