import { describe, expect, test } from "bun:test";
import { mkdtempSync, readFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { Metrics } from "./metrics.ts";

describe("metrics", () => {
  test("counts events, keeps peaks, and persists across restarts", () => {
    const file = join(mkdtempSync(join(tmpdir(), "so101-metrics-")), "events.jsonl");
    const m = new Metrics(file);
    m.event("jam", { id: 3 });
    m.event("block", { what: "table" });
    m.sample([
      { id: 3, joint: "elbow_flex", load: -62.5, temperature: 41 },
      { id: 6, joint: "gripper", load: 10, temperature: 56 },
    ]);
    m.sample([{ id: 6, joint: "gripper", load: 5, temperature: 57 }]); // still hot: no second event
    m.approach(0.012, "gripper_link");
    m.approach(0.03, "wrist_link");
    expect(m.counters.jam).toBe(1);
    expect(m.counters.hot).toBe(1);
    expect(m.peaks.get(3)!.load).toBe(62.5);
    expect(m.peaks.get(6)!.temperature).toBe(57);
    expect(m.closest!.link).toBe("gripper_link");
    expect(readFileSync(file, "utf8").trim().split("\n")).toHaveLength(3);

    const again = new Metrics(file);
    expect(again.counters.jam).toBe(1);
    expect(again.counters.hot).toBe(1);
    expect(again.recent.map((e) => e.kind)).toEqual(["jam", "block", "hot"]);
  });

  test("works without a file", () => {
    const m = new Metrics();
    m.event("stream", { frames: 10 });
    expect(m.summary()).toMatchObject({ counters: { stream: 1 } });
  });
});
