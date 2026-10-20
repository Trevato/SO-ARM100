// Bench script, 2026-10-12: hand servos off for mounting. Waits for all six, then wiggles id 1 until it is
// pulled, then id 2, and so on. Run from Software/so101: SO101_PORT=/dev/cu.usbmodemXXXX node bench/handoff.ts
import { FeetechBus } from "../src/bus.ts";
import { JOINTS, STS3215 } from "../src/sts3215.ts";
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const log = (line: string) => console.log(line);
let bus: FeetechBus;
while (true) { try { bus = await FeetechBus.open(process.env.SO101_PORT!); break; } catch { await sleep(1000); } }

const ids = [1, 2, 3, 4, 5, 6];
log("EVENT waiting for all six follower servos to answer on one chain");
while ((await bus.syncRead(ids, STS3215.Present_Position)).size < 6) await sleep(1500);
log("EVENT all six answer -> wiggling id 1 (shoulder_pan): pull the one that moves and mount it in the base");

async function wiggle(id: number) {
  const pos = await bus.read(id, STS3215.Present_Position);
  const target = pos > 3800 ? pos - 250 : pos + 250;
  await bus.write(id, STS3215.Torque_Enable, 1);
  await bus.write(id, STS3215.Goal_Velocity, 1500);
  await bus.write(id, STS3215.Goal_Position, target); await sleep(400);
  await bus.write(id, STS3215.Goal_Position, pos); await sleep(400);
  await bus.write(id, STS3215.Torque_Enable, 0);
}

for (const id of ids) {
  let seen = false, absent = 0;
  while (true) {
    const here = (await bus.ping(id)) !== null;
    if (here) { seen = true; absent = 0; try { await wiggle(id); } catch {} await sleep(1200); continue; }
    if (!seen) { await sleep(1000); continue; }           // not reachable yet (chain gap); wait for it
    if (++absent >= 3) break;                               // pulled
    await sleep(1000);
  }
  const next = ids[ids.indexOf(id) + 1];
  log(`EVENT id ${id} (${JOINTS[id]}) pulled${next ? ` -> now wiggling id ${next} (${JOINTS[next]})` : " -> all six handed off"}`);
}
log("DONE");
await bus.close();
