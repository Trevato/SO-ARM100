import { afterAll, beforeAll, describe, expect, test } from "bun:test";
import { existsSync, mkdtempSync, readFileSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import WebSocket from "ws";
import { FeetechBus } from "./bus.ts";
import { FakeBus } from "./fake-bus.ts";
import { mapFrame } from "./leader.ts";
import { startServer, type RunningServer } from "./server.ts";

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
let fake: FakeBus;
let server: RunningServer;
let base: string;
let dataDir: string;
const post = (path: string, body: unknown, method = "POST") => fetch(`${base}${path}`, { method, body: JSON.stringify(body) });

beforeAll(async () => {
  dataDir = mkdtempSync(join(tmpdir(), "so101-serve-"));
  fake = new FakeBus([1, 2, 6]);
  fake.servos.get(1)!.set([0x32, 0x03, 0x9f, 0x0d], 9); // limits 818-3487
  fake.servos.get(2)!.set([0x2d, 0x03, 0x82, 0x0c], 9); // 813-3202
  fake.servos.get(6)!.set([0xcb, 0x07, 0xc5, 0x0d], 9); // 1995-3525
  server = await startServer(new FeetechBus(fake), { host: "127.0.0.1", port: 0, ids: [1, 2, 6], posesFile: join(dataDir, "poses.json"), watchdogMs: 60 });
  base = `http://127.0.0.1:${server.port}`;
});
// Under Node the close resolves at once in every scenario (probed). Bun's node:http keeps a connection the
// server closed itself from ending, so the test runner would wait on it for ever: cap the wait.
afterAll(() => Promise.race([server.close(), sleep(500)]));

describe("so101 serve", () => {
  test("reports status and limits", async () => {
    const status = (await (await fetch(`${base}/status`)).json()) as { joints: Array<{ id: number; position: number }> };
    expect(status.joints.map((j) => [j.id, j.position])).toEqual([
      [1, 2048],
      [2, 2048],
      [6, 2048],
    ]);
    const limits = (await (await fetch(`${base}/limits`)).json()) as Record<string, { min: number; max: number }>;
    expect(limits["6"]).toEqual({ min: 1995, max: 3525 });
  });

  test("a stream drives the arm inside the limits and drops impossible jumps", async () => {
    const ws = new WebSocket(`ws://127.0.0.1:${server.port}/follow`);
    await new Promise((r) => ws.once("open", r));
    ws.send(JSON.stringify({ "1": 2100, "2": 100, "6": 9999 })); // 2: below min → clamped; 6: above max → clamped
    await sleep(30);
    ws.send(JSON.stringify({ "1": 3000 })); // 900 ticks in one frame: dropped
    await sleep(30);
    ws.close();
    await sleep(30);
    const goals = fake.writes.filter((w) => w.address === 42).map((w) => [w.id, w.bytes[0]! | (w.bytes[1]! << 8)]);
    expect(goals).toContainEqual([1, 2100]);
    expect(goals).toContainEqual([2, 843]);
    expect(goals).toContainEqual([6, 3495]);
    expect(goals).not.toContainEqual([1, 3000]);
    // closing the stream holds at the present position and restores the torque cap
    const caps = fake.writes.filter((w) => w.address === 48).map((w) => w.bytes[0]! | (w.bytes[1]! << 8));
    expect(caps.at(-1)).toBe(1000);
  });

  test("a silent stream makes the arm hold", async () => {
    const before = fake.writes.length;
    const ws = new WebSocket(`ws://127.0.0.1:${server.port}/follow`);
    await new Promise((r) => ws.once("open", r));
    ws.send(JSON.stringify({ "1": 2200 }));
    await sleep(150); // past the 60 ms watchdog
    const holds = fake.writes.slice(before).filter((w) => w.address === 42 && w.id === 1).map((w) => w.bytes[0]! | (w.bytes[1]! << 8));
    expect(holds.at(-1)).toBe(2200); // present position (the fake follows goals), written again by the watchdog
    expect(holds.length).toBeGreaterThanOrEqual(2);
    ws.close();
    await sleep(30);
  });

  test("torque and unknown poses over HTTP", async () => {
    const t = await fetch(`${base}/torque`, { method: "POST", body: JSON.stringify({ on: false }) });
    expect(((await t.json()) as { torque: boolean }).torque).toBe(false);
    const p = await fetch(`${base}/pose/go`, { method: "POST", body: JSON.stringify({ name: "nope" }) });
    expect(p.status).toBe(404); // no poses file at all: nothing saved yet
  });
});

describe("leader mapping of a whole frame", () => {
  test("maps each joint by its own rule", () => {
    const leaderLimits = new Map([
      [1, { min: 857, max: 3218 }],
      [6, { min: 2047, max: 3515 }],
    ]);
    const followerLimits = { "1": { min: 818, max: 3487 }, "6": { min: 1995, max: 3525 } };
    const frame = mapFrame(new Map([[1, 2037.5], [6, 3515], [5, 100]]), leaderLimits, followerLimits, 6);
    expect(frame).toEqual({ "1": 2153, "6": 3525 }); // id 5 has no limits on either side and is skipped
  });
});

describe("viewer page", () => {
  test("is served at the root", async () => {
    const r = await fetch(`${base}/`);
    expect(r.status).toBe(200);
    expect(r.headers.get("content-type")).toContain("text/html");
    expect(await r.text()).toContain("Fullscreen");
  });
});

describe("moves over HTTP", () => {
  test("/move drives the joints it names under supervision and reports where they ended", async () => {
    const before = ((await (await fetch(`${base}/status`)).json()) as { joints: Array<{ id: number; position: number }> }).joints.find((j) => j.id === 2)!.position;
    const r = await post("/move", { targets: { "1": 2500, "6": 3000 } });
    expect(r.status).toBe(200);
    const d = (await r.json()) as { reached: boolean; final: Record<string, number> };
    expect(d.reached).toBe(true);
    expect(d.final).toEqual({ "1": 2500, "6": 3000 });
    const status = (await (await fetch(`${base}/status`)).json()) as { joints: Array<{ id: number; position: number }>; driving: string | null };
    expect(status.joints.find((j) => j.id === 2)!.position).toBe(before); // untouched
    expect(status.driving).toBeNull();
  });

  test("/move pulls a target inside the limit margin and refuses bad input", async () => {
    const r = await post("/move", { targets: { "6": 4000 } }); // above the gripper's 3525 max
    const d = (await r.json()) as { reached: boolean; final: Record<string, number>; adjusted: Record<string, number> };
    expect(d.reached).toBe(true);
    expect(d.final["6"]).toBe(3525 - 30);
    expect((await post("/move", { targets: { "9": 100 } })).status).toBe(400);
    expect((await post("/move", { targets: { "1": 10.5 } })).status).toBe(400);
    expect((await post("/move", { targets: {} })).status).toBe(400);
    expect((await post("/move", "nonsense")).status).toBe(400);
  });

  test("/grip closes the gripper toward a position at a low torque cap and keeps torque on", async () => {
    const r = await post("/grip", { position: 2000, torque: 30 });
    expect(await r.json()).toEqual({ gripping: 2025, torque: 30 }); // clamped to min + margin
    const memory = fake.servos.get(6)!;
    expect(memory[48]! + memory[49]! * 256).toBe(300); // Torque_Limit 30 %
    expect(memory[40]).toBe(1); // Torque_Enable
  });

  test("/stop with nothing running is harmless", async () => {
    expect(await (await post("/stop", {})).json()).toEqual({ stopped: null });
  });
});

describe("poses over HTTP", () => {
  test("save, list, go, delete", async () => {
    await post("/move", { targets: { "1": 2200, "2": 2300, "6": 2400 } });
    const saved = (await (await post("/pose/save", { name: "bench", ids: [1, 2] })).json()) as { name: string; positions: number[]; onStops: number[] };
    expect(saved.name).toBe("bench");
    expect(saved.positions).toEqual([2200, 2300]);
    expect(saved.onStops).toEqual([]);
    const file = JSON.parse(readFileSync(join(dataDir, "poses.json"), "utf8")) as Record<string, { ids: number[] }>;
    expect(file.bench!.ids).toEqual([1, 2]);
    expect(Object.keys((await (await fetch(`${base}/poses`)).json()) as object)).toEqual(["bench"]);

    await post("/move", { targets: { "1": 3000 } });
    const go = (await (await post("/pose/go", { name: "bench" })).json()) as { reached: boolean; final: Record<string, number> };
    expect(go.reached).toBe(true);
    expect(go.final).toEqual({ "1": 2200, "2": 2300 });

    expect((await post("/pose/delete", { name: "bench" })).status).toBe(200);
    expect((await post("/pose/delete", { name: "bench" })).status).toBe(404);
    expect((await post("/pose/go", { name: "bench" })).status).toBe(404);
  });

  test("a pose resting on a hard stop is reported, and names are checked", async () => {
    await post("/move", { targets: { "6": 3525 } }); // aims at max - 30, inside 40 of the stop
    const saved = (await (await post("/pose/save", { name: "edge", ids: [6] })).json()) as { onStops: number[] };
    expect(saved.onStops).toEqual([6]);
    expect((await post("/pose/save", { name: "__proto__" })).status).toBe(400);
    expect((await post("/pose/save", { name: "ok", ids: [9] })).status).toBe(400);
    await post("/pose/delete", { name: "edge" });
  });
});

describe("recordings over HTTP", () => {
  test("record, list, play, and stop a playback", async () => {
    const start = await post("/record/start", { name: "wave", hz: 50 });
    expect(start.status).toBe(200);
    expect(fake.servos.get(1)![40]).toBe(0); // torque released for the recording
    let status = (await (await fetch(`${base}/status`)).json()) as { driving: string | null; recording: string | null };
    expect(status.driving).toBe("record");
    expect(status.recording).toBe("wave");
    expect((await post("/move", { targets: { "1": 100 } })).status).toBe(409); // the recording owns the arm
    await sleep(120);
    const stop = (await (await post("/record/stop", {})).json()) as { saved: string; frames: number };
    expect(stop.saved).toBe("wave");
    expect(stop.frames).toBeGreaterThan(2);
    expect(existsSync(join(dataDir, "recordings", "wave.json"))).toBe(true);
    const list = (await (await fetch(`${base}/recordings`)).json()) as Record<string, { hz: number; frames: number }>;
    expect(list.wave!.hz).toBe(50);
    expect(list.wave!.frames).toBe(stop.frames);
    expect((await post("/record/start", { name: "wave" })).status).toBe(409); // no silent overwrite

    const play = (await (await post("/play", { name: "wave", rate: 2 })).json()) as { played: number; frames: number };
    expect(play.played).toBe(play.frames);
    status = (await (await fetch(`${base}/status`)).json()) as { driving: string | null; recording: string | null };
    expect(status.driving).toBeNull();
    expect((await post("/play", { name: "nothing" })).status).toBe(404);
    expect((await post("/record/stop", {})).status).toBe(409);
    expect((await post("/recording/delete", { name: "wave" })).status).toBe(200);
    expect((await post("/recording/delete", { name: "wave" })).status).toBe(404);
    expect(Object.keys((await (await fetch(`${base}/recordings`)).json()) as object)).toEqual([]);
  });

  test("/stop ends a recording and saves it", async () => {
    await post("/record/start", { name: "short", hz: 50 });
    await sleep(60);
    const stopped = (await (await post("/stop", {})).json()) as { stopped: string; saved: string };
    expect(stopped.stopped).toBe("record");
    expect(stopped.saved).toBe("short");
  });
});

describe("calibration store", () => {
  test("round trip", async () => {
    expect(await (await fetch(`${base}/calibration`)).json()).toEqual({ camera: null, handeye: null });
    expect((await post("/calibration/camera", { width: 1280, height: 720, rms: 0.4 }, "PUT")).status).toBe(200);
    const c = (await (await fetch(`${base}/calibration`)).json()) as { camera: { width: number; saved: string }; handeye: null };
    expect(c.camera.width).toBe(1280);
    expect(typeof c.camera.saved).toBe("string");
    expect(c.handeye).toBeNull();
  });
});

describe("/stop on a stream", () => {
  test("closes the socket and the arm holds", async () => {
    const ws = new WebSocket(`ws://127.0.0.1:${server.port}/follow`);
    await new Promise((r) => ws.once("open", r));
    ws.send(JSON.stringify({ "1": 2100 }));
    await sleep(30);
    let status = (await (await fetch(`${base}/status`)).json()) as { driving: string | null };
    expect(status.driving).toBe("stream");
    const closed = new Promise<number>((r) => ws.once("close", (code) => r(code)));
    expect(await (await post("/stop", {})).json()).toEqual({ stopped: "stream" });
    expect(await closed).toBe(1000);
    await sleep(30);
    status = (await (await fetch(`${base}/status`)).json()) as { driving: string | null };
    expect(status.driving).toBeNull();
  });
});

describe("a bus that fails mid-session", () => {
  // The same server on a bus whose reads can be made to throw, like a USB unplug.
  let faulty: FakeBus;
  let bus: FeetechBus;
  let srv: RunningServer;
  let url: string;
  let broken = false;
  const call = (path: string, body: unknown) => fetch(`${url}${path}`, { method: "POST", body: JSON.stringify(body) });
  beforeAll(async () => {
    faulty = new FakeBus([1, 2, 6]);
    bus = new FeetechBus(faulty);
    const real = bus.syncRead.bind(bus);
    bus.syncRead = (...args) => (broken ? Promise.reject(new Error("usb gone")) : real(...args));
    srv = await startServer(bus, { host: "127.0.0.1", port: 0, ids: [1, 2, 6], posesFile: join(mkdtempSync(join(tmpdir(), "so101-faulty-")), "poses.json"), watchdogMs: 60 });
    url = `http://127.0.0.1:${srv.port}`;
  });
  afterAll(() => Promise.race([srv.close(), sleep(500)]));

  test("a stream whose close hits a dead bus still frees the arm", async () => {
    const ws = new WebSocket(`ws://127.0.0.1:${srv.port}/follow`);
    await new Promise((r) => ws.once("open", r));
    ws.send(JSON.stringify({ "1": 2100 }));
    await sleep(30);
    broken = true;
    ws.close();
    await sleep(80);
    broken = false;
    const status = (await (await fetch(`${url}/status`)).json()) as { driving: string | null };
    expect(status.driving).toBeNull();
    expect((await call("/move", { targets: { "1": 2200 } })).status).toBe(200);
  });

  test("a recording whose bus dies reports the failure on stop and frees the arm", async () => {
    expect((await call("/record/start", { name: "dead", hz: 50 })).status).toBe(200);
    await sleep(30);
    broken = true;
    await sleep(60);
    broken = false;
    const stop = await call("/record/stop", {});
    expect(stop.status).toBe(500);
    expect(((await stop.json()) as { error: string }).error).toContain("usb gone");
    const status = (await (await fetch(`${url}/status`)).json()) as { driving: string | null; recording: string | null };
    expect(status.driving).toBeNull();
    expect(status.recording).toBeNull();
    const list = (await (await fetch(`${url}/recordings`)).json()) as Record<string, unknown>;
    expect(Object.keys(list)).toEqual([]);
  });

  test("/stop and /record/stop racing: one saves, the other finds nothing to stop", async () => {
    await call("/record/start", { name: "race", hz: 50 });
    await sleep(60);
    const [a, b] = await Promise.all([call("/stop", {}), call("/record/stop", {})]);
    expect([a.status, b.status].sort()).toEqual([200, 409]);
    const list = (await (await fetch(`${url}/recordings`)).json()) as Record<string, unknown>;
    expect(Object.keys(list)).toEqual(["race"]);
  });

  test("/grip owns the arm while it writes, and the next supervised gripper move puts the servo's own cap back", async () => {
    const ws = new WebSocket(`ws://127.0.0.1:${srv.port}/follow`);
    await new Promise((r) => ws.once("open", r));
    expect((await call("/grip", { position: 2500 })).status).toBe(409); // the stream drives
    const closed = new Promise((r) => ws.once("close", r));
    ws.close();
    await closed;
    await sleep(40);
    expect((await call("/grip", { position: 2500, torque: 30 })).status).toBe(200);
    const memory = faulty.servos.get(6)!;
    expect(memory[48]! + memory[49]! * 256).toBe(300);
    expect((await call("/move", { targets: { "6": 2600 } })).status).toBe(200);
    expect(memory[48]! + memory[49]! * 256).toBe(1000); // Max_Torque_Limit, not the cap a grip left behind
  });

  test("/move rejects ids that are not plain integers", async () => {
    expect((await call("/move", { targets: { "03": 2000 } })).status).toBe(400);
    expect((await call("/move", { targets: { " 1": 2000 } })).status).toBe(400);
    expect((await call("/move", { targets: { "1.0": 2000 } })).status).toBe(400);
  });
});

describe("the safe zone, the jam watch, and the model routes", () => {
  // A six-joint fake with the follower's real calibrated ranges, the repo's model, and a fast jam watch.
  let fake6: FakeBus;
  let zoneServer: RunningServer;
  let zurl: string;
  const zcall = (path: string, body: unknown, method = "POST") => fetch(`${zurl}${path}`, { method, body: JSON.stringify(body) });
  /** Wait until nothing drives the arm: a closed stream releases it a few bus operations later. */
  const idle = async () => {
    for (let i = 0; i < 150; i++) {
      const s = (await (await fetch(`${zurl}/status`)).json()) as { driving: string | null };
      if (s.driving === null) return;
      await sleep(20);
    }
    throw new Error("the arm never went idle");
  };
  const moved = async (targets: Record<string, number>) => {
    const r = await zcall("/move", { targets });
    if (r.status !== 200) throw new Error(`move ${JSON.stringify(targets)}: ${r.status} ${await r.text()}`);
  };
  const RANGES: Record<number, [number, number]> = { 1: [818, 3487], 2: [813, 3202], 3: [873, 3083], 4: [902, 2916], 5: [0, 4095], 6: [1995, 3525] };
  const mid = (id: number) => Math.round((RANGES[id]![0] + RANGES[id]![1]) / 2);
  const ticks = (id: number, deg: number) => Math.round(mid(id) + (deg * 4095) / 360);
  beforeAll(async () => {
    fake6 = new FakeBus([1, 2, 3, 4, 5, 6], { stuck: new Set([3]) });
    for (const [id, [min, max]] of Object.entries(RANGES)) {
      fake6.servos.get(Number(id))!.set([min & 0xff, min >> 8, max & 0xff, max >> 8], 9);
      fake6.servos.get(Number(id))!.set([mid(Number(id)) & 0xff, mid(Number(id)) >> 8], 56); // present = middle of range = the model's zero
    }
    zoneServer = await startServer(new FeetechBus(fake6), {
      host: "127.0.0.1",
      port: 0,
      posesFile: join(mkdtempSync(join(tmpdir(), "so101-zone-")), "poses.json"),
      modelDir: new URL("../../../Simulation/SO101", import.meta.url).pathname,
      watchdogMs: 60,
      jamSampleMs: 15,
    });
    zurl = `http://127.0.0.1:${zoneServer.port}`;
  });
  afterAll(() => Promise.race([zoneServer.close(), sleep(500)]));

  test("serves the model, its meshes and the vendored libraries, and nothing outside them", async () => {
    const info = (await (await fetch(`${zurl}/model.json`)).json()) as { urdf: string; joints: Array<{ name: string; id: number }>; floorZ: number };
    expect(info.joints.map((j) => j.id)).toEqual([1, 2, 3, 4, 5, 6]);
    expect(info.floorZ).toBeLessThan(0);
    expect((await fetch(`${zurl}${info.urdf}`)).headers.get("content-type")).toBe("application/xml");
    expect((await fetch(`${zurl}/model/assets/base_so101_v2.stl`)).headers.get("content-type")).toBe("model/stl");
    expect((await fetch(`${zurl}/model/../package.json`)).status).toBe(404);
    expect((await fetch(`${zurl}/model/README.md`)).status).toBe(404);
    expect((await fetch(`${zurl}/vendor/three/build/three.module.js`)).status).toBe(200);
    expect((await fetch(`${zurl}/vendor/urdf-loader/src/urdf-manipulator-element.js`)).status).toBe(200);
    expect((await fetch(`${zurl}/vendor/ws/index.js`)).status).toBe(404);
    expect((await fetch(`${zurl}/vendor/three/package.json`)).status).toBe(200);
    expect((await fetch(`${zurl}/vendor/three/../ws/index.js`)).status).toBe(404);
  });

  test("serves the printable sheets, and nothing else from that folder", async () => {
    const list = (await (await fetch(`${zurl}/printables`)).json()) as Array<{ file: string; url: string; bytes: number }>;
    expect(list.map((p) => p.file)).toEqual(["charuco_board.png", "cube_markers.png", "gripper_marker.png"]);
    const png = await fetch(`${zurl}${list[0]!.url}`);
    expect(png.headers.get("content-type")).toBe("image/png");
    expect((await png.arrayBuffer()).byteLength).toBe(list[0]!.bytes);
    expect((await fetch(`${zurl}/printables/../markers.py`)).status).toBe(404);
    expect((await fetch(`${zurl}/printables/nope.png`)).status).toBe(404);
  });

  test("status reports the zone, and the rest pose is clear", async () => {
    const s = (await (await fetch(`${zurl}/status`)).json()) as { zone: string; violation: unknown; lowestMm: number; blocked: unknown; jammed: number[] };
    expect(s.zone).toBe("warn");
    expect(s.violation).toBeNull();
    expect(s.lowestMm).toBeGreaterThan(0);
    expect(s.jammed).toEqual([]);
  });

  test("settings are validated and persisted", async () => {
    expect((await zcall("/settings", { zone: "sideways" }, "PUT")).status).toBe(400);
    expect((await zcall("/settings", { floorMarginMm: 500 }, "PUT")).status).toBe(400);
    expect((await zcall("/settings", { jointOffsetsDeg: { nope: 1 } }, "PUT")).status).toBe(400);
    const s = (await (await zcall("/settings", { zone: "enforce", floorMarginMm: 10, jointOffsetsDeg: { wrist_flex: 2 } }, "PUT")).json()) as { zone: string; floorMarginMm: number; jointOffsetsDeg: Record<string, number>; model: boolean };
    expect(s).toMatchObject({ zone: "enforce", floorMarginMm: 10, jointOffsetsDeg: { wrist_flex: 2 }, model: true });
    const again = (await (await fetch(`${zurl}/settings`)).json()) as { jointOffsetsDeg: Record<string, number> };
    expect(again.jointOffsetsDeg).toEqual({ wrist_flex: 2 });
  });

  test("a move into the table is refused in enforce mode and only reported in warn mode", async () => {
    const down = { targets: { "2": ticks(2, 80) } }; // shoulder lift far forward with the elbow straight: the gripper goes under the table
    const refused = await zcall("/move", down);
    expect(refused.status).toBe(409);
    const body = (await refused.json()) as { reached: boolean; error: string; violation: { what: string; link: string } };
    expect(body.reached).toBe(false);
    expect(body.error).toMatch(/^refused: .* below the table \(plus the \d+ mm margin\)$/);
    expect(body.violation.what).toBe("table");
    const status = (await (await fetch(`${zurl}/status`)).json()) as { joints: Array<{ id: number; position: number }>; driving: null };
    expect(status.joints.find((j) => j.id === 2)!.position).toBe(mid(2)); // nothing moved
    expect(status.driving).toBeNull();
    const m = (await (await fetch(`${zurl}/metrics`)).json()) as { counters: { block: number } };
    expect(m.counters.block).toBe(1);

    await zcall("/settings", { zone: "warn" }, "PUT");
    const warned = (await (await zcall("/move", down)).json()) as { reached: boolean; warning: string };
    expect(warned.reached).toBe(true);
    expect(warned.warning).toContain("below the table");
    await moved({ "2": mid(2) });
    await zcall("/settings", { zone: "enforce" }, "PUT");
  });

  test("a stream frame into the table holds the arm; a safe frame resumes it", async () => {
    const ws = new WebSocket(`ws://127.0.0.1:${zoneServer.port}/follow`);
    await new Promise((r) => ws.once("open", r));
    ws.send(JSON.stringify({ "1": mid(1) + 50 })); // first frame: safe, eases
    await sleep(80);
    ws.send(JSON.stringify({ "2": ticks(2, 80) })); // the leader dives at the table
    await sleep(60);
    let s = (await (await fetch(`${zurl}/status`)).json()) as { blocked: { what: string } | null; joints: Array<{ id: number; position: number }> };
    expect(s.blocked?.what).toBe("table");
    expect(s.joints.find((j) => j.id === 2)!.position).toBe(mid(2)); // held
    ws.send(JSON.stringify({ "2": mid(2) + 100 })); // the leader comes back up
    await sleep(60);
    s = (await (await fetch(`${zurl}/status`)).json()) as typeof s;
    expect(s.blocked).toBeNull();
    expect(s.joints.find((j) => j.id === 2)!.position).toBe(mid(2) + 100);
    const closed = new Promise((r) => ws.once("close", r));
    ws.close();
    await closed;
    await idle();
  });

  test("a joint the stream has not driven yet is eased over, never jumped", async () => {
    const ws = new WebSocket(`ws://127.0.0.1:${zoneServer.port}/follow`);
    await new Promise((r) => ws.once("open", r));
    ws.send(JSON.stringify({ "6": mid(6) + 100 }));
    await sleep(120);
    const before = fake6.writes.length;
    ws.send(JSON.stringify({ "6": mid(6) + 100, "1": mid(1) + 900 })); // the pan joins the stream 900 ticks away
    await sleep(150);
    const writes = fake6.writes.slice(before).filter((w) => w.id === 1);
    const velocity = writes.find((w) => w.address === 46);
    expect(velocity!.bytes[0]! | (velocity!.bytes[1]! << 8)).toBe(300); // eased at easeSpeed, not Goal_Velocity 0 (max)
    const s = (await (await fetch(`${zurl}/status`)).json()) as { joints: Array<{ id: number; position: number }>; stream: { frames: number; dropped: number } };
    expect(s.joints.find((j) => j.id === 1)!.position).toBe(mid(1) + 900);
    expect(s.stream.frames).toBe(2);
    expect(s.stream.dropped).toBe(0);
    const closed = new Promise((r) => ws.once("close", r));
    ws.close();
    await closed;
    await idle();
    await moved({ "1": mid(1), "6": mid(6) });
  });

  test("a joint left far behind after a block is eased over once the leader has stayed away long enough", async () => {
    const ws = new WebSocket(`ws://127.0.0.1:${zoneServer.port}/follow`);
    await new Promise((r) => ws.once("open", r));
    // Frames keep coming every 10 ms, as a leader's do, so the 60 ms watchdog of this server never turns this into a plain hold.
    const send = async (target: number, times: number) => {
      for (let i = 0; i < times; i++) {
        ws.send(JSON.stringify({ "2": target }));
        await sleep(10);
      }
    };
    await send(mid(2) + 200, 10); // safe, eases
    await send(ticks(2, 80), 4); // dives at the table: dropped as a jump, or refused by the zone
    await send(mid(2) - 500, 25); // safe, but 700 ticks from where the joint was held
    await sleep(150);
    const s = (await (await fetch(`${zurl}/status`)).json()) as { joints: Array<{ id: number; position: number }>; blocked: unknown; stream: { dropped: number } };
    expect(s.joints.find((j) => j.id === 2)!.position).toBe(mid(2) - 500);
    expect(s.blocked).toBeNull();
    expect(s.stream.dropped).toBeGreaterThanOrEqual(19);
    const closed = new Promise((r) => ws.once("close", r));
    ws.close();
    await closed;
    await idle();
    await moved({ "2": mid(2) });
  });

  test("from inside a violation, a stream frame may back out but not go deeper", async () => {
    await zcall("/settings", { zone: "warn" }, "PUT");
    await moved({ "2": ticks(2, 60) }); // warn mode lets the arm into the table
    await zcall("/settings", { zone: "enforce" }, "PUT");
    const ws = new WebSocket(`ws://127.0.0.1:${zoneServer.port}/follow`);
    await new Promise((r) => ws.once("open", r));
    ws.send(JSON.stringify({ "2": ticks(2, 70) })); // deeper: refused
    await sleep(120);
    let s = (await (await fetch(`${zurl}/status`)).json()) as { joints: Array<{ id: number; position: number }> };
    expect(s.joints.find((j) => j.id === 2)!.position).toBe(ticks(2, 60));
    ws.send(JSON.stringify({ "2": ticks(2, 50) })); // backing out: let through
    await sleep(120);
    s = (await (await fetch(`${zurl}/status`)).json()) as typeof s;
    expect(s.joints.find((j) => j.id === 2)!.position).toBe(ticks(2, 50));
    const closed = new Promise((r) => ws.once("close", r));
    ws.close();
    await closed;
    await idle();
    await zcall("/settings", { zone: "warn" }, "PUT");
    await moved({ "2": mid(2) });
    await zcall("/settings", { zone: "enforce" }, "PUT");
  });

  test("a recording is judged with the joints it does not drive where they are, and so is the way to its first frame", async () => {
    // Record the pan alone while the rest sits at the safe middle: safe to play.
    expect((await zcall("/record/start", { name: "pan", hz: 50, ids: [1] })).status).toBe(200);
    await sleep(80);
    expect((await zcall("/record/stop", {})).status).toBe(200);
    expect((await zcall("/play", { name: "pan", rate: 2 })).status).toBe(200);
    // The same recording with the lift pointing at the table is refused before anything moves.
    await zcall("/settings", { zone: "warn" }, "PUT");
    await moved({ "2": ticks(2, 60) });
    await zcall("/settings", { zone: "enforce" }, "PUT");
    const r = await zcall("/play", { name: "pan", rate: 2 });
    expect(r.status).toBe(409);
    expect(((await r.json()) as { error: string }).error).toContain("refused");
    await zcall("/settings", { zone: "warn" }, "PUT");
    await moved({ "2": mid(2) });
    await zcall("/settings", { zone: "enforce" }, "PUT");
    await zcall("/recording/delete", { name: "pan" });
  });

  test("a joint that pushes without moving on the stream is parked, and follows again once the leader backs off", async () => {
    const ws = new WebSocket(`ws://127.0.0.1:${zoneServer.port}/follow`);
    await new Promise((r) => ws.once("open", r));
    const push = setInterval(() => ws.send(JSON.stringify({ "3": mid(3) + 300 })), 20); // the elbow (stuck in this fake) is asked to move
    await sleep(500);
    let s = (await (await fetch(`${zurl}/status`)).json()) as { jammed: number[] };
    expect(s.jammed).toEqual([3]);
    const goal = fake6.writes.filter((w) => w.id === 3 && w.address === 42).at(-1)!;
    expect(goal.bytes[0]! | (goal.bytes[1]! << 8)).toBe(mid(3)); // parked where it is
    clearInterval(push);
    const m = (await (await fetch(`${zurl}/metrics`)).json()) as { counters: { jam: number } };
    expect(m.counters.jam).toBe(1);
    ws.send(JSON.stringify({ "3": mid(3) - 20 })); // the leader backs off (the fake is stuck both ways, so stay inside the 30-tick dead band)
    await sleep(60);
    s = (await (await fetch(`${zurl}/status`)).json()) as { jammed: number[] };
    expect(s.jammed).toEqual([]);
    const closed = new Promise((r) => ws.once("close", r));
    ws.close();
    await closed;
    await idle();
    const after = (await (await fetch(`${zurl}/metrics`)).json()) as { counters: { stream: number }; recent: Array<{ kind: string }> };
    expect(after.counters.stream).toBeGreaterThanOrEqual(2);
    expect(after.recent.at(-1)!.kind).toBe("stream");
  });
});

describe("settings on disk", () => {
  test("a bad value is skipped and the rest kept; unreadable JSON falls back to the defaults, with a log line", async () => {
    const dir = mkdtempSync(join(tmpdir(), "so101-settings-"));
    writeFileSync(join(dir, "settings.json"), JSON.stringify({ zone: "enforce", floorMarginMm: 40, jointOffsetsDeg: { wrist_roll: 45 } }));
    const fake = new FakeBus([1, 2, 6]);
    const s1 = await startServer(new FeetechBus(fake), { host: "127.0.0.1", port: 0, ids: [1, 2, 6], posesFile: join(dir, "poses.json") });
    const got = (await (await fetch(`http://127.0.0.1:${s1.port}/settings`)).json()) as { zone: string; floorMarginMm: number; jointOffsetsDeg: Record<string, number> };
    expect(got.zone).toBe("enforce");
    expect(got.floorMarginMm).toBe(40);
    expect(got.jointOffsetsDeg).toEqual({});
    await Promise.race([s1.close(), sleep(300)]);
    writeFileSync(join(dir, "settings.json"), "{ not json");
    const s2 = await startServer(new FeetechBus(new FakeBus([1, 2, 6])), { host: "127.0.0.1", port: 0, ids: [1, 2, 6], posesFile: join(dir, "poses.json") });
    const again = (await (await fetch(`http://127.0.0.1:${s2.port}/settings`)).json()) as { zone: string };
    expect(again.zone).toBe("warn");
    await Promise.race([s2.close(), sleep(300)]);
  });
});
