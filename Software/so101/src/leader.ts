/** Read a leader arm and stream follower targets to a `so101 serve` over a WebSocket. */

import WebSocket from "ws";
import type { FeetechBus } from "./bus.ts";
import { mapGripper, mapJoint, type Frame, type Range } from "./mapping.ts";
import { STS3215 } from "./sts3215.ts";

export interface LeaderOptions {
  hz?: number;
  ids?: readonly number[];
  gripperId?: number;
  signal?: AbortSignal;
  onFrame?: (frame: Frame) => void;
}

export async function fetchLimits(serverUrl: string): Promise<Record<string, Range>> {
  const http = serverUrl.replace(/^ws/, "http").replace(/\/follow$/, "");
  const res = await fetch(`${http}/limits`);
  if (!res.ok) throw new Error(`${http}/limits answered ${res.status}`);
  return (await res.json()) as Record<string, Range>;
}

/** Pure: leader ticks → follower frame, given both arms' calibrated ranges. */
export function mapFrame(leader: ReadonlyMap<number, number>, leaderLimits: Map<number, Range>, followerLimits: Record<string, Range>, gripperId: number): Frame {
  const frame: Frame = {};
  for (const [id, ticks] of leader) {
    const l = leaderLimits.get(id);
    const f = followerLimits[String(id)];
    if (!l || !f) continue;
    frame[String(id)] = id === gripperId ? mapGripper(ticks, l, f) : mapJoint(ticks, l, f);
  }
  return frame;
}

export async function streamLeader(bus: FeetechBus, serverUrl: string, { hz = 60, ids = [1, 2, 3, 4, 5, 6], gripperId = 6, signal, onFrame }: LeaderOptions = {}): Promise<number> {
  const followerLimits = await fetchLimits(serverUrl);
  const leaderLimits = new Map<number, Range>();
  for (const id of ids) {
    leaderLimits.set(id, { min: await bus.read(id, STS3215.Min_Position_Limit), max: await bus.read(id, STS3215.Max_Position_Limit) });
  }
  await bus.syncWrite(STS3215.Torque_Enable, ids.map((id) => [id, 0] as const)); // the leader is moved by hand
  const ws = new WebSocket(serverUrl.endsWith("/follow") ? serverUrl : `${serverUrl.replace(/\/$/, "")}/follow`);
  await new Promise<void>((resolve, reject) => {
    ws.once("open", resolve);
    ws.once("error", reject);
  });
  const period = 1000 / hz;
  let sent = 0;
  const started = performance.now();
  try {
    while (!signal?.aborted && ws.readyState === WebSocket.OPEN) {
      const leader = await bus.syncRead(ids, STS3215.Present_Position);
      const frame = mapFrame(leader, leaderLimits, followerLimits, gripperId);
      if (Object.keys(frame).length) {
        ws.send(JSON.stringify(frame));
        sent++;
        onFrame?.(frame);
      }
      const wait = started + (sent + 1) * period - performance.now();
      if (wait > 0) await new Promise((r) => setTimeout(r, wait));
    }
  } finally {
    ws.close();
  }
  return sent;
}
