/**
 * What the arm went through: jams, zone blocks, stalls, lost servos, sessions, and the peak load
 * and temperature of every joint. Events append to a JSONL file so nothing is lost across restarts;
 * the summary is what the viewer shows.
 */

import { appendFileSync, mkdirSync, readFileSync } from "node:fs";
import { dirname } from "node:path";

export type EventKind = "jam" | "block" | "stall" | "missing" | "stream" | "move" | "play" | "record" | "hot";

export interface MetricEvent {
  readonly t: string;
  readonly kind: EventKind;
  readonly detail: Record<string, unknown>;
}

export interface JointPeak {
  load: number;
  temperature: number;
  loadAt: string;
  temperatureAt: string;
}

const RECENT = 200;
const HOT = 55;
const COOL = 50;

export class Metrics {
  readonly counters: Record<EventKind, number> = { jam: 0, block: 0, stall: 0, missing: 0, stream: 0, move: 0, play: 0, record: 0, hot: 0 };
  readonly peaks = new Map<number, JointPeak>();
  readonly recent: MetricEvent[] = [];
  readonly since = new Date().toISOString();
  /** Closest any moving part came to the table, meters above it (negative = below), with the link and the time. */
  closest: { z: number; link: string; at: string } | null = null;
  readonly #file: string | undefined;
  readonly #hot = new Set<number>();

  constructor(file?: string) {
    this.#file = file;
    if (file) {
      try {
        // Carry the counters over from earlier runs, so "jams" means jams ever, not since the last restart.
        for (const line of readFileSync(file, "utf8").split("\n")) {
          if (!line.trim()) continue;
          const e = JSON.parse(line) as MetricEvent;
          if (e.kind in this.counters) this.counters[e.kind]++;
          this.recent.push(e);
        }
        this.recent.splice(0, Math.max(0, this.recent.length - RECENT));
      } catch {
        /* no file yet, or a line that is not an event */
      }
    }
  }

  event(kind: EventKind, detail: Record<string, unknown>): MetricEvent {
    const e: MetricEvent = { t: new Date().toISOString(), kind, detail };
    this.counters[kind]++;
    this.recent.push(e);
    if (this.recent.length > RECENT) this.recent.shift();
    if (this.#file) {
      try {
        mkdirSync(dirname(this.#file), { recursive: true });
        appendFileSync(this.#file, `${JSON.stringify(e)}\n`);
      } catch (err) {
        console.error(`metrics: cannot append to ${this.#file}: ${err instanceof Error ? err.message : String(err)}`);
      }
    }
    return e;
  }

  /** Fold a status reading into the peaks; a joint reaching 55 °C logs one "hot" event until it cools below 50. */
  sample(joints: ReadonlyArray<{ id: number; joint?: string | null; load: number | null; temperature: number | null }>): void {
    const now = new Date().toISOString();
    for (const j of joints) {
      const peak = this.peaks.get(j.id) ?? { load: 0, temperature: 0, loadAt: now, temperatureAt: now };
      if (j.load !== null && Math.abs(j.load) > peak.load) (peak.load = Math.abs(j.load)), (peak.loadAt = now);
      if (j.temperature !== null && j.temperature > peak.temperature) (peak.temperature = j.temperature), (peak.temperatureAt = now);
      this.peaks.set(j.id, peak);
      if (j.temperature !== null) {
        if (j.temperature >= HOT && !this.#hot.has(j.id)) {
          this.#hot.add(j.id);
          this.event("hot", { id: j.id, joint: j.joint ?? null, temperature: j.temperature });
        } else if (j.temperature < COOL) this.#hot.delete(j.id);
      }
    }
  }

  approach(z: number, link: string): void {
    if (!this.closest || z < this.closest.z) this.closest = { z, link, at: new Date().toISOString() };
  }

  summary(): Record<string, unknown> {
    return {
      since: this.since,
      counters: this.counters,
      peaks: Object.fromEntries(this.peaks),
      closest: this.closest,
      recent: this.recent.slice(-40),
    };
  }
}
