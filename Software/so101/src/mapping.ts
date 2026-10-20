/**
 * Leader → follower joint mapping, the way LeRobot does it with both arms calibrated:
 * arm joints keep the same angle relative to the middle of each calibrated range (both servos have
 * 4096 ticks per turn, so a tick offset from the middle carries over unchanged); the gripper maps the
 * trigger's full travel onto the jaw's full travel.
 */

export interface Range {
  readonly min: number;
  readonly max: number;
}

export const mid = (r: Range): number => (r.min + r.max) / 2;

export function mapJoint(leaderTicks: number, leader: Range, follower: Range, margin = 30): number {
  const target = Math.round(leaderTicks - mid(leader) + mid(follower));
  return clamp(target, follower.min + margin, follower.max - margin);
}

export function mapGripper(leaderTicks: number, leader: Range, follower: Range, margin = 0): number {
  const span = Math.max(1, leader.max - leader.min);
  const fraction = clamp((leaderTicks - leader.min) / span, 0, 1);
  const target = Math.round(follower.min + fraction * (follower.max - follower.min));
  return clamp(target, follower.min + margin, follower.max - margin);
}

export const clamp = (v: number, lo: number, hi: number): number => Math.min(hi, Math.max(lo, v));

/** A target frame on the wire: servo id → ticks. */
export type Frame = Record<string, number>;
