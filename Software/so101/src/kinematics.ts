/**
 * The arm's geometry, from its own URDF and meshes: forward kinematics for every link, and a
 * sampled point cloud per link so a joint target can be checked against the table and the base
 * before anything moves. No dependencies: a tiny XML reader and a binary STL reader are enough
 * for the files in Simulation/SO101.
 */

import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";

export type Vec3 = readonly [number, number, number];
/** Row-major 4x4. */
export type Mat4 = Float64Array;

export const identity = (): Mat4 => new Float64Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);

export function multiply(a: Mat4, b: Mat4): Mat4 {
  const out = new Float64Array(16);
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      out[r * 4 + c] = a[r * 4]! * b[c]! + a[r * 4 + 1]! * b[4 + c]! + a[r * 4 + 2]! * b[8 + c]! + a[r * 4 + 3]! * b[12 + c]!;
    }
  }
  return out;
}

/** URDF origin: rotation Rz(yaw)·Ry(pitch)·Rx(roll), then translation. */
export function fromRpyXyz(rpy: Vec3, xyz: Vec3): Mat4 {
  const [r, p, y] = rpy;
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  return new Float64Array([
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0],
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1],
    -sp, cp * sr, cp * cr, xyz[2],
    0, 0, 0, 1,
  ]);
}

/** Rotation about a unit axis (Rodrigues). */
export function rotationAbout(axis: Vec3, angle: number): Mat4 {
  const [x, y, z] = axis;
  const c = Math.cos(angle), s = Math.sin(angle), t = 1 - c;
  return new Float64Array([
    t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0,
    t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0,
    t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0,
    0, 0, 0, 1,
  ]);
}

export const transformPoint = (m: Mat4, p: Vec3): Vec3 => [
  m[0]! * p[0] + m[1]! * p[1] + m[2]! * p[2] + m[3]!,
  m[4]! * p[0] + m[5]! * p[1] + m[6]! * p[2] + m[7]!,
  m[8]! * p[0] + m[9]! * p[1] + m[10]! * p[2] + m[11]!,
];

// ---------------------------------------------------------------- a small XML reader
interface Node {
  name: string;
  attrs: Record<string, string>;
  children: Node[];
}

export function parseXml(text: string): Node {
  const root: Node = { name: "#root", attrs: {}, children: [] };
  const stack = [root];
  const clean = text.replace(/<\?[\s\S]*?\?>/g, "").replace(/<!--[\s\S]*?-->/g, "");
  const tag = /<(\/?)([A-Za-z_][\w:.-]*)([^>]*?)(\/?)>/g;
  for (let m = tag.exec(clean); m; m = tag.exec(clean)) {
    const [, closing, name, rawAttrs, selfClosing] = m;
    if (closing) {
      if (stack.length > 1 && stack[stack.length - 1]!.name === name) stack.pop();
      continue;
    }
    const attrs: Record<string, string> = {};
    const attr = /([\w:.-]+)\s*=\s*(?:"([^"]*)"|'([^']*)')/g;
    for (let a = attr.exec(rawAttrs!); a; a = attr.exec(rawAttrs!)) attrs[a[1]!] = a[2] ?? a[3] ?? "";
    const node: Node = { name: name!, attrs, children: [] };
    stack[stack.length - 1]!.children.push(node);
    if (!selfClosing) stack.push(node);
  }
  return root;
}

const child = (n: Node, name: string) => n.children.find((c) => c.name === name);
const vec3 = (text: string | undefined, fallback: Vec3): Vec3 => {
  if (!text) return fallback;
  const v = text.trim().split(/\s+/).map(Number);
  return v.length === 3 && v.every(Number.isFinite) ? [v[0]!, v[1]!, v[2]!] : fallback;
};

// ---------------------------------------------------------------- binary STL
/** Unique vertices of a binary STL, in its own units (meters here). */
export function stlVertices(data: Uint8Array): Float64Array {
  const view = new DataView(data.buffer, data.byteOffset, data.byteLength);
  const count = view.getUint32(80, true);
  if (84 + count * 50 > data.byteLength) throw new Error("not a binary STL (or truncated)"); // trailing padding is tolerated
  const seen = new Set<string>();
  const out: number[] = [];
  for (let i = 0; i < count; i++) {
    const base = 84 + i * 50 + 12; // skip the normal
    for (let v = 0; v < 3; v++) {
      const x = view.getFloat32(base + v * 12, true), y = view.getFloat32(base + v * 12 + 4, true), z = view.getFloat32(base + v * 12 + 8, true);
      const key = `${x},${y},${z}`;
      if (seen.has(key)) continue;
      seen.add(key);
      out.push(x, y, z);
    }
  }
  return Float64Array.from(out);
}

/**
 * A small set of points that stands in for a mesh in collision checks: its extreme vertex along
 * each of 26 directions (so the lowest point of any rotation is found to within a few mm) plus an
 * even sample of the rest.
 */
export function samplePoints(vertices: Float64Array, keep = 300): Float64Array {
  const n = vertices.length / 3;
  const picked = new Set<number>();
  for (let dx = -1; dx <= 1; dx++) {
    for (let dy = -1; dy <= 1; dy++) {
      for (let dz = -1; dz <= 1; dz++) {
        if (!dx && !dy && !dz) continue;
        let best = -Infinity, at = 0;
        for (let i = 0; i < n; i++) {
          const d = vertices[i * 3]! * dx + vertices[i * 3 + 1]! * dy + vertices[i * 3 + 2]! * dz;
          if (d > best) (best = d), (at = i);
        }
        picked.add(at);
      }
    }
  }
  const stride = Math.max(1, Math.floor(n / keep));
  for (let i = 0; i < n; i += stride) picked.add(i);
  const out = new Float64Array(picked.size * 3);
  let k = 0;
  for (const i of picked) {
    out[k++] = vertices[i * 3]!;
    out[k++] = vertices[i * 3 + 1]!;
    out[k++] = vertices[i * 3 + 2]!;
  }
  return out;
}

// ---------------------------------------------------------------- the model
export interface Joint {
  readonly name: string;
  readonly type: string;
  readonly parent: string;
  readonly child: string;
  readonly origin: Mat4;
  readonly axis: Vec3;
  readonly lower: number;
  readonly upper: number;
}

export interface Link {
  readonly name: string;
  /** Collision sample points in the link's own frame, xyz triples. */
  readonly points: Float64Array;
  /** Every mesh vertex, kept only for the root link, whose shape becomes the base height map. */
  readonly vertices?: Float64Array;
}

/**
 * The base as terrain: the top of the base at each cell of a grid over its footprint, so a part
 * hovering over the low front plate is fine while one pushed into the tall motor holder is not.
 */
export class HeightMap {
  readonly cell: number;
  readonly x0: number;
  readonly y0: number;
  readonly cols: number;
  readonly rows: number;
  readonly top: Float64Array; // -Infinity where the base has nothing

  constructor(points: Float64Array, cell = 0.005) {
    this.cell = cell;
    let xmin = Infinity, xmax = -Infinity, ymin = Infinity, ymax = -Infinity;
    for (let i = 0; i < points.length; i += 3) {
      xmin = Math.min(xmin, points[i]!); xmax = Math.max(xmax, points[i]!);
      ymin = Math.min(ymin, points[i + 1]!); ymax = Math.max(ymax, points[i + 1]!);
    }
    this.x0 = xmin - cell; this.y0 = ymin - cell;
    this.cols = Math.ceil((xmax - this.x0) / cell) + 2;
    this.rows = Math.ceil((ymax - this.y0) / cell) + 2;
    const top = new Float64Array(this.cols * this.rows).fill(-Infinity);
    for (let i = 0; i < points.length; i += 3) {
      const c = Math.floor((points[i]! - this.x0) / cell), r = Math.floor((points[i + 1]! - this.y0) / cell);
      top[r * this.cols + c] = Math.max(top[r * this.cols + c]!, points[i + 2]!);
    }
    // One dilation: a cell with no vertex inside a solid part takes its neighbours' height, so faces between vertices count.
    const dilated = Float64Array.from(top);
    for (let r = 0; r < this.rows; r++) {
      for (let c = 0; c < this.cols; c++) {
        if (top[r * this.cols + c]! > -Infinity) continue;
        let best = -Infinity;
        for (let dr = -1; dr <= 1; dr++) for (let dc = -1; dc <= 1; dc++) {
          const rr = r + dr, cc = c + dc;
          if (rr >= 0 && rr < this.rows && cc >= 0 && cc < this.cols) best = Math.max(best, top[rr * this.cols + cc]!);
        }
        dilated[r * this.cols + c] = best;
      }
    }
    this.top = dilated;
  }

  /** Height of the base under a point, or -Infinity outside its footprint. */
  at(x: number, y: number): number {
    const c = Math.floor((x - this.x0) / this.cell), r = Math.floor((y - this.y0) / this.cell);
    if (c < 0 || r < 0 || c >= this.cols || r >= this.rows) return -Infinity;
    return this.top[r * this.cols + c]!;
  }
}

export interface ZoneSettings {
  /** Moving parts stay this far above the table, meters. */
  floorMargin: number;
  /** Moving parts stay this far outside the base's column, meters. */
  baseMargin: number;
}

export interface Violation {
  /** "table": below the table plane. "base": inside the base's own shape. */
  readonly what: "table" | "base";
  readonly link: string;
  /** How far inside the forbidden region, meters. */
  readonly depth: number;
  readonly point: Vec3;
}

/** Links that sit on or around the base and may legitimately overlap its column. */
const NEAR_BASE = new Set(["base_link", "shoulder_link", "upper_arm_link"]);

export class ArmModel {
  readonly joints: readonly Joint[];
  readonly links: ReadonlyMap<string, Link>;
  readonly movable: readonly Joint[];
  readonly root: string;
  /** The table's plane: the bottom of the base, in base_link coordinates. */
  readonly floorZ: number;
  /** The base's shape as terrain. */
  readonly base: HeightMap;
  readonly baseTop: number;

  constructor(joints: Joint[], links: Map<string, Link>) {
    this.links = links;
    const children = new Set(joints.map((j) => j.child));
    const root = joints.map((j) => j.parent).find((p) => !children.has(p));
    if (!root) throw new Error("the URDF has no root link");
    this.root = root;
    // Chain order from the root, whatever order the file lists the joints in.
    const ordered: Joint[] = [];
    const queue = [root];
    while (queue.length) {
      const parent = queue.shift()!;
      for (const j of joints.filter((j) => j.parent === parent)) {
        ordered.push(j);
        queue.push(j.child);
      }
    }
    if (ordered.length !== joints.length) throw new Error("the URDF has joints that do not connect to the root");
    this.joints = ordered;
    this.movable = ordered.filter((j) => j.type === "revolute" || j.type === "continuous");
    const base = links.get(root)?.vertices ?? links.get(root)?.points;
    if (!base || !base.length) throw new Error(`the root link ${root} has no mesh to stand on`);
    let floor = Infinity, top = -Infinity;
    for (let i = 0; i < base.length; i += 3) {
      floor = Math.min(floor, base[i + 2]!);
      top = Math.max(top, base[i + 2]!);
    }
    this.floorZ = floor;
    this.baseTop = top;
    this.base = new HeightMap(base);
  }

  /** Read a URDF and the binary STL meshes it points at (paths relative to the URDF's folder). */
  static load(urdfPath: string): ArmModel {
    const dir = dirname(urdfPath);
    const robot = child(parseXml(readFileSync(urdfPath, "utf8")), "robot");
    if (!robot) throw new Error(`${urdfPath}: no <robot>`);
    const meshes = new Map<string, Float64Array>();
    const mesh = (file: string) => {
      let v = meshes.get(file);
      if (!v) meshes.set(file, (v = stlVertices(readFileSync(join(dir, file)))));
      return v;
    };
    const links = new Map<string, Link>();
    const joints: Joint[] = robot.children
      .filter((c) => c.name === "joint")
      .map((j) => {
        const o = child(j, "origin");
        const lim = child(j, "limit");
        return {
          name: j.attrs.name!,
          type: j.attrs.type ?? "fixed",
          parent: child(j, "parent")!.attrs.link!,
          child: child(j, "child")!.attrs.link!,
          origin: fromRpyXyz(vec3(o?.attrs.rpy, [0, 0, 0]), vec3(o?.attrs.xyz, [0, 0, 0])),
          axis: vec3(child(j, "axis")?.attrs.xyz, [0, 0, 1]),
          lower: Number(lim?.attrs.lower ?? -Infinity),
          upper: Number(lim?.attrs.upper ?? Infinity),
        };
      });
    const children = new Set(joints.map((j) => j.child));
    const root = joints.map((j) => j.parent).find((p) => !children.has(p));
    for (const l of robot.children.filter((c) => c.name === "link")) {
      const parts: number[] = [];
      const all: number[] = [];
      for (const visual of l.children.filter((c) => c.name === "visual")) {
        const file = child(child(visual, "geometry") ?? visual, "mesh")?.attrs.filename;
        if (!file) continue;
        const o = child(visual, "origin");
        const T = fromRpyXyz(vec3(o?.attrs.rpy, [0, 0, 0]), vec3(o?.attrs.xyz, [0, 0, 0]));
        const vertices = mesh(file);
        const pts = samplePoints(vertices);
        for (let i = 0; i < pts.length; i += 3) parts.push(...transformPoint(T, [pts[i]!, pts[i + 1]!, pts[i + 2]!]));
        if (l.attrs.name === root) for (let i = 0; i < vertices.length; i += 3) all.push(...transformPoint(T, [vertices[i]!, vertices[i + 1]!, vertices[i + 2]!]));
      }
      links.set(l.attrs.name!, { name: l.attrs.name!, points: Float64Array.from(parts), ...(all.length ? { vertices: Float64Array.from(all) } : {}) });
    }
    return new ArmModel(joints, links);
  }

  /** Every link's pose in root coordinates for the given joint angles (radians, by joint name; missing = 0). */
  linkPoses(q: ReadonlyMap<string, number>): Map<string, Mat4> {
    const poses = new Map<string, Mat4>([[this.root, identity()]]);
    let pending = [...this.joints];
    while (pending.length) {
      const rest: Joint[] = [];
      for (const j of pending) {
        const parent = poses.get(j.parent);
        if (!parent) {
          rest.push(j);
          continue;
        }
        const angle = j.type === "revolute" || j.type === "continuous" ? (q.get(j.name) ?? 0) : 0;
        poses.set(j.child, multiply(multiply(parent, j.origin), angle ? rotationAbout(j.axis, angle) : identity()));
      }
      if (rest.length === pending.length) throw new Error(`joints without a reachable parent: ${rest.map((j) => j.name).join(", ")}`);
      pending = rest;
    }
    return poses;
  }

  /**
   * One pass over every moving part: the worst way the pose breaks the zone (or null), and the
   * lowest point above the table, for the metrics and the viewer.
   */
  evaluate(q: ReadonlyMap<string, number>, settings: ZoneSettings): { violation: Violation | null; lowest: { z: number; link: string } } {
    const poses = this.linkPoses(q);
    let worst: Violation | null = null;
    let lowestZ = Infinity, lowestLink = "";
    const floor = this.floorZ + settings.floorMargin;
    for (const [name, link] of this.links) {
      if (name === this.root) continue;
      const T = poses.get(name);
      if (!T) continue;
      const pts = link.points;
      for (let i = 0; i < pts.length; i += 3) {
        const p = transformPoint(T, [pts[i]!, pts[i + 1]!, pts[i + 2]!]);
        if (p[2] < lowestZ) (lowestZ = p[2]), (lowestLink = name);
        if (p[2] < floor) {
          const depth = floor - p[2];
          if (!worst || depth > worst.depth) worst = { what: "table", link: name, depth, point: p };
        } else if (!NEAR_BASE.has(name)) {
          const top = this.base.at(p[0], p[1]) + settings.baseMargin;
          if (p[2] < top) {
            const depth = top - p[2];
            if (!worst || depth > worst.depth) worst = { what: "base", link: name, depth, point: p };
          }
        }
      }
    }
    return { violation: worst, lowest: { z: lowestZ, link: lowestLink } };
  }

  /** The worst way the pose breaks the zone, or null. */
  check(q: ReadonlyMap<string, number>, settings: ZoneSettings): Violation | null {
    return this.evaluate(q, settings).violation;
  }

  /** The lowest sampled point of the moving links. */
  lowest(q: ReadonlyMap<string, number>): { z: number; link: string } {
    return this.evaluate(q, { floorMargin: 0, baseMargin: 0 }).lowest;
  }
}

/** LeRobot's convention for the arm joints, which the *_new_calib URDF follows: zero at the middle of the calibrated range, 4095 ticks per turn. */
export const ticksToRadians = (ticks: number, min: number, max: number): number => ((ticks - (min + max) / 2) * 2 * Math.PI) / 4095;
export const radiansToTicks = (rad: number, min: number, max: number): number => Math.round((rad * 4095) / (2 * Math.PI) + (min + max) / 2);
/**
 * The gripper is different: LeRobot normalizes it 0-100 from closed (the range's minimum) to open (its maximum),
 * and the URDF's jaw is closed at zero and open at its upper limit. So the calibrated range maps onto [0, upper].
 */
export const gripperTicksToRadians = (ticks: number, min: number, max: number, upper: number): number => ((ticks - min) / (max - min)) * upper;
export const gripperRadiansToTicks = (rad: number, min: number, max: number, upper: number): number => Math.round(min + (rad / upper) * (max - min));
