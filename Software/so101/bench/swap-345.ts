// Bench script, 2026-10-12: swap the borrowed labeled 1/345 (follower id 5) for the sixth unlabeled servo.
// Run from Software/so101: SO101_PORT=/dev/cu.usbmodemXXXX node bench/swap-345.ts
import { FeetechBus } from "../src/bus.ts";
import { STS3215 } from "../src/sts3215.ts";
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));
const log = (line: string) => console.log(line);
const bus = await FeetechBus.open(process.env.SO101_PORT!);

async function settle(id: number) {
  await bus.write(id, STS3215.Torque_Enable, 1);
  await bus.write(id, STS3215.Acceleration, 100);
  await bus.write(id, STS3215.Goal_Velocity, 1500);
  await bus.write(id, STS3215.Goal_Position, 2048); await sleep(1800);
  await bus.write(id, STS3215.Goal_Position, 2300); await sleep(450);
  await bus.write(id, STS3215.Goal_Position, 2048); await sleep(450);
  await bus.write(id, STS3215.Goal_Position, 2300); await sleep(450);
  await bus.write(id, STS3215.Goal_Position, 2048); await sleep(600);
  await bus.write(id, STS3215.Torque_Enable, 0);
}
const alone = async (id: number) => (await bus.ping(id)) && !(await bus.ping(id === 1 ? 2 : 1)) && !(await bus.ping(6));

log("EVENT waiting for the chain to be unplugged and the new servo (id 1) to be alone on the board");
while (!(await alone(1))) await sleep(1000);
await sleep(1000);
await bus.write(1, STS3215.ID, 5);
await settle(5);
log("EVENT new servo is now id 5 (follower wrist_roll), centered -> unplug it, then plug the 345 in alone");
while (await bus.ping(5)) await sleep(1000);
log("EVENT new servo unplugged -> waiting for the 345 (id 5) alone");
while (!(await alone(5))) await sleep(1000);
await sleep(1000);
await bus.write(5, STS3215.ID, 2);
await settle(2);
log("EVENT the 345 is back to id 2 (leader shoulder_lift), centered -> set it aside; chain the new servo with the five");
log("DONE");
await bus.close();
