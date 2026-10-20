// Bench script, 2026-10-19: leader servo sorting. Plug each leader servo in ALONE; it is found whatever id it has,
// its gearing is measured by a short full-speed sweep, it gets the next id for that gearing, is centered at 2048,
// and wiggles. Run from Software/so101: SO101_PORT=/dev/cu.usbmodemXXXX node bench/sort-leader.ts
import { FeetechBus } from "../src/bus.ts";
import { JOINTS, STS3215 } from "../src/sts3215.ts";
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const log = (line: string) => console.log(line);
let bus: FeetechBus;
while (true) { try { bus = await FeetechBus.open(process.env.SO101_PORT!); break; } catch { await sleep(1000); } }

// Measured on this bench: 1/345 ≈ 1400 ticks/s, 1/191 ≈ 4160. 1/147 is faster still.
function classify(peak: number): "1/345" | "1/191" | "1/147" | "unclear" {
  const rel = peak / 4160;
  return rel < 0.6 ? "1/345" : rel >= 0.8 && rel <= 1.15 ? "1/191" : rel > 1.2 ? "1/147" : "unclear";
}

async function present(): Promise<number[]> {
  const ids: number[] = [];
  for (let id = 0; id < 254; id++) if (await bus.ping(id)) ids.push(id);
  return ids;
}

async function measure(id: number): Promise<number> {
  const start = await bus.read(id, STS3215.Present_Position);
  const target = start > 3000 ? start - 512 : start + 512;
  await bus.write(id, STS3215.Goal_Position, start);
  await bus.write(id, STS3215.Torque_Enable, 1);
  await bus.write(id, STS3215.Acceleration, 254);
  await bus.write(id, STS3215.Goal_Velocity, 32767);
  const t0 = performance.now(); await bus.write(id, STS3215.Goal_Position, target);
  const samples: Array<[number, number]> = [];
  while (performance.now() - t0 < 1500) {
    const pos = await bus.read(id, STS3215.Present_Position); const t = performance.now() - t0;
    samples.push([t, pos]); if (Math.abs(pos - target) <= 3 && t > 100) break; await sleep(3);
  }
  let peak = 0;
  for (let i = 4; i < samples.length; i++) { const [t1, p1] = samples[i - 4]!, [t2, p2] = samples[i]!; peak = Math.max(peak, Math.abs(p2 - p1) / ((t2 - t1) / 1000)); }
  await bus.write(id, STS3215.Goal_Velocity, 1200);
  await bus.write(id, STS3215.Goal_Position, start); await sleep(700);
  return peak;
}

async function centerAndWiggle(id: number) {
  await bus.write(id, STS3215.Acceleration, 100);
  await bus.write(id, STS3215.Goal_Velocity, 1500);
  await bus.write(id, STS3215.Torque_Enable, 1);
  await bus.write(id, STS3215.Goal_Position, 2048); await sleep(1800);
  for (const p of [2300, 2048, 2300, 2048]) { await bus.write(id, STS3215.Goal_Position, p); await sleep(450); }
  await bus.write(id, STS3215.Torque_Enable, 0);
}

const pools: Record<string, number[]> = { "1/191": [1, 3], "1/345": [2], "1/147": [4, 5, 6] };
const done: string[] = [];
log("EVENT ready: plug in one leader servo by itself");
let lastSeen = "";
while (Object.values(pools).some((p) => p.length)) {
  const ids = await present();
  const key = ids.join(",");
  if (ids.length === 0) { lastSeen = ""; await sleep(1000); continue; }
  if (ids.length > 1) { if (key !== lastSeen) log(`EVENT more than one servo answers (ids ${key}); leave only one plugged in`); lastSeen = key; await sleep(1500); continue; }
  const [id] = ids;
  if (key === lastSeen) { await sleep(1000); continue; } // already handled; waiting for it to be unplugged
  await sleep(800);
  const peak = await measure(id!);
  const ratio = classify(peak);
  const next = pools[ratio]?.shift();
  if (next === undefined) {
    await centerAndWiggle(id!); await centerAndWiggle(id!);
    log(`EVENT servo (id ${id}) measured ${peak.toFixed(0)} ticks/s = ${ratio}: ${ratio === "unclear" ? "could not tell its gearing" : "every slot for that gearing is filled"}; double wiggle = set it aside, plug in the next`);
    lastSeen = key; continue;
  }
  if (next !== id) await bus.write(id!, STS3215.ID, next);
  await centerAndWiggle(next);
  done.push(`${ratio}→${next}`);
  const left = Object.entries(pools).filter(([, p]) => p.length).map(([r, p]) => `${r}: ${p.join(",")}`).join("; ");
  log(`EVENT ${peak.toFixed(0)} ticks/s = ${ratio} -> now id ${next} (${JOINTS[next]}), centered; mount it. ${left ? `Still needed: ${left}` : "That was the last one"}`);
  lastSeen = String(next);
}
log(`DONE leader set complete: ${done.join(", ")}`);
await bus.close();
