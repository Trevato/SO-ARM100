/** Named joint poses, kept as plain JSON so they can be read and edited by hand. */
export interface Pose {
  readonly ids: readonly number[];
  readonly positions: readonly number[];
  readonly saved?: string;
}

export type PoseFile = Readonly<Record<string, Pose>>;

/** Letters, digits, dash, underscore, dot: nothing that is also a property of every JavaScript object. */
export const POSE_NAME = /^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$/;

export function assertPoseName(name: string): void {
  if (!POSE_NAME.test(name) || name in Object.prototype) {
    throw new Error(`"${name}" is not a usable pose name (letters, digits, - _ . only)`);
  }
}

export function uniqueIds(ids: readonly number[]): boolean {
  return new Set(ids).size === ids.length;
}

export function parsePoseFile(text: string): PoseFile {
  let data: unknown;
  try {
    data = JSON.parse(text);
  } catch {
    throw new Error("not valid JSON");
  }
  if (typeof data !== "object" || data === null || Array.isArray(data)) throw new Error("expected an object of poses");
  const poses: Record<string, Pose> = Object.create(null);
  for (const [name, value] of Object.entries(data as Record<string, unknown>)) {
    assertPoseName(name);
    if (typeof value !== "object" || value === null) throw new Error(`pose "${name}" must be an object`);
    const { ids, positions, saved } = value as Record<string, unknown>;
    if (!Array.isArray(ids) || ids.length === 0 || !ids.every((id) => Number.isInteger(id) && id >= 0 && id <= 253) || !uniqueIds(ids)) {
      throw new Error(`pose "${name}": ids must be a non-empty list of distinct servo ids`);
    }
    if (!Array.isArray(positions) || positions.length !== ids.length || !positions.every(Number.isInteger)) {
      throw new Error(`pose "${name}": positions must hold one integer per id`);
    }
    poses[name] = { ids, positions, ...(typeof saved === "string" ? { saved } : {}) };
  }
  return poses;
}

export function serializePoseFile(poses: PoseFile): string {
  return JSON.stringify(poses, null, 1);
}
