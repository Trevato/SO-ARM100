// Bench script, 2026-10-12: as each follower servo joins the chain, center it at 2048, wiggle it, and announce its id.
// Run from Software/so101: SO101_PORT=/dev/cu.usbmodemXXXX node bench/assemble-follower.ts
import { FeetechBus } from "../src/bus.ts";
import { JOINTS, STS3215 } from "../src/sts3215.ts";
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const log = (line: string) => console.log(line);

let bus: FeetechBus | null = null;
async function connect(): Promise<FeetechBus> {
  while (true) {
    try { return await FeetechBus.open(process.env.SO101_PORT!); } catch { await sleep(1000); }
  }
}

async function settle(id: number) {
  const b = bus!;
  await b.write(id, STS3215.Torque_Enable, 1);
  await b.write(id, STS3215.Acceleration, 100);
  await b.write(id, STS3215.Goal_Velocity, 1500);
  await b.write(id, STS3215.Goal_Position, 2048); await sleep(1800);
  await b.write(id, STS3215.Goal_Position, 2300); await sleep(450);
  await b.write(id, STS3215.Goal_Position, 2048); await sleep(450);
  await b.write(id, STS3215.Goal_Position, 2300); await sleep(450);
  await b.write(id, STS3215.Goal_Position, 2048); await sleep(600);
  await b.write(id, STS3215.Torque_Enable, 0);
}

log("EVENT waiting for the board");
bus = await connect();
const order: number[] = [5];
if (await bus.ping(5)) {
  log("EVENT board is up; the 345 is already id 5 and centered -> plug in an unlabeled servo");
} else {
  log("EVENT board is up -> plug in the labeled 345 by itself");
  while (!(await bus.ping(2))) await sleep(1000);
  await sleep(1000);
  await bus.write(2, STS3215.ID, 5);
  await settle(5);
  log("EVENT #1 on the chain: the 345 is now id 5 (wrist_roll), centered -> plug in an unlabeled servo");
}

const expected = new Set([1, 2, 3, 4, 6]);
while (expected.size > 0) {
  let found: number | null = null;
  for (const id of expected) if (await bus.ping(id)) { found = id; break; }
  if (found === null) { await sleep(1000); continue; }
  expected.delete(found);
  await sleep(500);
  await settle(found);
  order.push(found);
  log(`EVENT #${order.length} on the chain: id ${found} (${JOINTS[found]}), centered${expected.size ? " -> plug in the next unlabeled servo" : " -> that is all six"}`);
}
log(`ORDER plugged in: ${order.map((id, i) => `#${i + 1}=id ${id} ${JOINTS[id]}`).join(", ")}`);
log(`ORDER pull off in reverse: ${[...order].reverse().map((id) => `id ${id} ${JOINTS[id]}`).join(" -> ")}`);
log("DONE");
await bus.close();
