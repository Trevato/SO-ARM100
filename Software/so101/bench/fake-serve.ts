/**
 * The arm server on a simulated bus, for working on the viewer without hardware:
 *   node bench/fake-serve.ts [port]      then open http://127.0.0.1:8102/?stream=<a camera stream url>
 * Goals become positions at once, so every move "arrives"; poses and recordings go to a temp dir.
 */
import { mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { FeetechBus } from "../src/bus.ts";
import { FakeBus } from "../src/fake-bus.ts";
import { startServer } from "../src/server.ts";

const fake = new FakeBus([1, 2, 3, 4, 5, 6]);
const ranges: Record<number, [number, number]> = { 1: [818, 3487], 2: [813, 3202], 3: [900, 3100], 4: [700, 3300], 5: [600, 3400], 6: [1995, 3525] };
for (const [id, [min, max]] of Object.entries(ranges)) {
  const m = fake.servos.get(Number(id))!;
  m.set([min & 0xff, min >> 8, max & 0xff, max >> 8], 9);
}
const dir = mkdtempSync(join(tmpdir(), "so101-fake-"));
const port = Number(process.argv[2] ?? 8102);
const server = await startServer(new FeetechBus(fake), { host: "127.0.0.1", port, posesFile: join(dir, "poses.json"), modelDir: new URL("../../../Simulation/SO101", import.meta.url).pathname });
console.log(`fake arm on http://127.0.0.1:${server.port}  (data in ${dir})`);
