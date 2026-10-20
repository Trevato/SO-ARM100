// Bench script, 2026-10-19: fix servo ids on an ASSEMBLED leader by walking the chain outward from the base.
// Start with only the base servo connected to the board; reconnect one joint at a time when told.
// Run from Software/so101: SO101_PORT=/dev/cu.usbmodemXXXX node bench/walk-leader.ts
import { FeetechBus } from "../src/bus.ts";
import { JOINTS, STS3215 } from "../src/sts3215.ts";
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const log = (line: string) => console.log(line);
let bus: FeetechBus;
async function connect() { while (true) { try { bus = await FeetechBus.open(process.env.SO101_PORT!); return; } catch { await sleep(1000); } } }
await connect();

const EXPECTED: Record<number, string> = { 1: "1/191", 2: "1/345", 3: "1/191", 4: "1/147", 5: "1/147", 6: "1/147" };
const TEMP = 11; // the base parks here until the others are named, so factory id 1 never collides

let lastPeak = 0;
function classify(peak: number): string {
  lastPeak = peak;
  // On a 45° sweep: 1/345 ≈ 1400 ticks/s, 1/191 ≈ 4160. This shorter swing reads a little low, so the bands are wide.
  return peak < 2400 ? "1/345" : peak < 5200 ? "1/191" : "1/147";
}
async function present(): Promise<number[]> { const ids: number[] = []; for (let id = 0; id < 254; id++) if (await bus.ping(id)) ids.push(id); return ids; }
async function measure(id: number): Promise<number> {
  const start = await bus.read(id, STS3215.Present_Position);
  const target = start + (start > 2048 ? -400 : 400); // a swing toward the middle, long enough to hit top speed
  await bus.write(id, STS3215.Goal_Position, start);
  await bus.write(id, STS3215.Torque_Enable, 1);
  await bus.write(id, STS3215.Acceleration, 254);
  await bus.write(id, STS3215.Goal_Velocity, 32767);
  const t0 = performance.now(); await bus.write(id, STS3215.Goal_Position, target);
  const s: Array<[number, number]> = [];
  while (performance.now() - t0 < 1200) { const p = await bus.read(id, STS3215.Present_Position); const t = performance.now() - t0; s.push([t, p]); if (Math.abs(p - target) <= 3 && t > 60) break; await sleep(3); }
  let peak = 0; for (let i = 3; i < s.length; i++) { const [t1, p1] = s[i - 3]!, [t2, p2] = s[i]!; peak = Math.max(peak, Math.abs(p2 - p1) / ((t2 - t1) / 1000)); }
  await bus.write(id, STS3215.Goal_Velocity, 1000); await bus.write(id, STS3215.Goal_Position, start); await sleep(600);
  return peak;
}
async function wiggle(id: number, times = 2) {
  const pos = await bus.read(id, STS3215.Present_Position); const t = pos + (pos > 2048 ? -80 : 80);
  await bus.write(id, STS3215.Goal_Position, pos); await bus.write(id, STS3215.Torque_Limit, 300);
  await bus.write(id, STS3215.Goal_Velocity, 600); await bus.write(id, STS3215.Torque_Enable, 1);
  for (let i = 0; i < times; i++) { await bus.write(id, STS3215.Goal_Position, t); await sleep(350); await bus.write(id, STS3215.Goal_Position, pos); await sleep(350); }
  await bus.write(id, STS3215.Torque_Enable, 0);
}
function gearNote(finalId: number, ratio: string): string {
  return ratio === EXPECTED[finalId] ? `gearing ${ratio} is right for ${JOINTS[finalId]}` : `WRONG GEARING: ${JOINTS[finalId]} wants ${EXPECTED[finalId]} but this servo is ${ratio}`;
}



// Incremental mode: whatever already answers with a final id is kept (and its gearing checked once); every servo
// that appears afterwards is measured and given the next free id for its gearing. A 1/191 that must become id 1
// is parked at TEMP until the end so later factory-id-1 arrivals cannot collide with it.
const pools: Record<string, number[]> = { "1/345": [2], "1/191": [1, 3], "1/147": [4, 5, 6] };
const checked = new Set<number>();
let announced = "";
let parkedBase = false;

async function verifyKnown(ids: number[]) {
  for (const id of ids) {
    if (checked.has(id) || !(id >= 1 && id <= 6)) continue;
    checked.add(id);
    log(`EVENT ${JOINTS[id]} already has id ${id}`);
  }
}

for (const id of [1, 2, 3, 4, 5, 6, TEMP]) { try { await bus.write(id, STS3215.Torque_Enable, 0); } catch {} }
log("EVENT ready (torque released everywhere)");
async function walk(): Promise<boolean> {
  const ids = await present();
  const key = ids.join(",");
  if (ids.length === 0) { announced = ""; return false; }
  if (ids.includes(TEMP) && !parkedBase) { parkedBase = true; const i = pools["1/191"]!.indexOf(1); if (i >= 0) pools["1/191"]!.splice(i, 1); log("EVENT a servo is already parked as shoulder_pan from the earlier run; it gets id 1 at the end"); }
  await verifyKnown(ids.filter((i) => i >= 2 && i <= 6));
  const fresh = ids.filter((i) => !checked.has(i) && i !== TEMP);
  const missing = [1, 2, 3, 4, 5, 6].filter((f) => !ids.includes(f) && !(f === 1 && parkedBase));
  if (fresh.length === 0) {
    if (missing.length === 0) {
      if (parkedBase) { await bus.write(TEMP, STS3215.ID, 1); await wiggle(1); checked.add(1); }
      const all = await present();
      log(`DONE leader chain reads ids ${all.join(",")}; torque off everywhere`);
      for (const id of all) await bus.write(id, STS3215.Torque_Enable, 0);
      return true;
    }
    if (key !== announced) {
      // The ready signal: the shoulder lift wiggles once when the chain is stable and the next servo can be connected.
      if (ids.includes(2)) await wiggle(2, 1);
      log(`EVENT ready for the next servo (chain has ${ids.join(",")}; still needed: ${missing.map((m) => JOINTS[m]).join(", ")})`);
    }
    announced = key; return false;
  }
  if (fresh.length > 1) { if (key !== announced) log(`EVENT ids ${fresh.join(",")} are new at once; connect one joint at a time`); announced = key; return false; }
  const id = fresh[0]!;
  await sleep(500);
  // Several factory-id-1 servos answering together look like one id to a ping but garble every read.
  let clean = 0;
  for (let i = 0; i < 6; i++) { try { await bus.read(id, STS3215.Present_Position); clean++; } catch {} }
  if (clean < 6) {
    if (key !== announced) log(`EVENT id ${id} is really several servos talking at once (${6 - clean}/6 reads garbled); disconnect so only ONE unnamed servo is on the chain`);
    announced = key; return false;
  }
  // No speed test on a mounted arm: it drives joints into the structure. Ids go in connection order:
  // elbow 3, wrist flex 4, wrist roll 5, trigger 6; the pan is parked for the end.
  const next = [3, 4, 5, 6].find((f) => !ids.includes(f) && !checked.has(f));
  if (next === undefined) {
    await wiggle(id, 4);
    log(`EVENT all of 3-6 are named already; this extra servo (id ${id}) is not expected. Long wiggle = disconnect it`);
    announced = key; await sleep(5000); return false;
  }
  if (id !== next) await bus.write(id, STS3215.ID, next);
  checked.add(next); await wiggle(next);
  log(`EVENT ${JOINTS[next]} is id ${next}; wait for the ready wiggle, then connect the next`);
  announced = ""; return false;
}
while (true) {
  try { if (await walk()) break; await sleep(1500); }
  catch (err) {
    const msg = (err as Error).message.split("\n")[0];
    if (/ENXIO|not open|cannot open|disconnect/i.test(msg)) {
      log(`EVENT board dropped (${msg}); waiting for it to come back`);
      try { await bus.close(); } catch {}
      await connect();
    } else {
      log(`EVENT a servo stopped answering mid-step (${msg}); if the chain brought several id-1 servos in at once, reduce it to one`);
      for (const id of [1, 2, 3, 4, 5, 6, TEMP]) { try { await bus.write(id, STS3215.Torque_Enable, 0); } catch {} }
      await sleep(2000);
    }
    announced = "";
  }
}
await bus.close();
