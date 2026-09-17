/**
 * Collision groups. Rapier packs them as (membership << 16) | filter.
 *
 * The rocker must not collide with the static frame: the real A-frame is open where the
 * rocker swings, and any approximation of it will jam the pocket against a panel. The
 * robot cannot reach the rocker either -- R102 caps expansion at 29 in and the pivot is
 * at 44 in -- so the rocker only ever needs to meet balls.
 */
export const GROUP = {
  field: 1 << 0,
  ball: 1 << 1,
  rocker: 1 << 2,
  robot: 1 << 3,
} as const;

const pack = (membership: number, filter: number) => (membership << 16) | filter;

export const GROUPS = {
  field: pack(GROUP.field, GROUP.ball | GROUP.robot),
  ball: pack(GROUP.ball, GROUP.field | GROUP.ball | GROUP.rocker | GROUP.robot),
  rocker: pack(GROUP.rocker, GROUP.ball),
  robot: pack(GROUP.robot, GROUP.field | GROUP.ball),
} as const;
