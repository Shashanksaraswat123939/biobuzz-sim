/**
 * The only place world coordinates become FTC field coordinates.
 *
 *   World (CAD/Three): Y up. X = pivot-axis direction (red hive at -X). Z toward the audience.
 *   FTC field:         Z up. X toward the audience.  Y to the left of an observer at the audience.
 *
 * So: ftcX = worldZ, ftcY = worldX, ftcZ = worldY. That is a right-handed rotation
 * (det = +1), which is why no sign flips are needed anywhere else.
 *
 * Heading: the robot's local forward is world +Z at yaw 0, and a right-handed yaw about
 * world +Y maps one-to-one onto FTC heading CCW from +X. worldYaw === ftcHeading.
 */
import { M_TO_IN, IN_TO_M, RAD, DEG } from '../units.js';
import type { Vec3 } from '../types.js';

/** World metres -> FTC inches. */
export function worldToFtc(p: Vec3): Vec3 {
  return [p[2] * M_TO_IN, p[0] * M_TO_IN, p[1] * M_TO_IN];
}

/** FTC inches -> world metres. */
export function ftcToWorld(p: Vec3): Vec3 {
  return [p[1] * IN_TO_M, p[2] * IN_TO_M, p[0] * IN_TO_M];
}

export const worldYawToFtcHeadingDeg = (yawRad: number) => yawRad * RAD;
export const ftcHeadingToWorldYaw = (headingDeg: number) => headingDeg * DEG;

/** A world-frame direction as an FTC bearing in degrees (CCW from FTC +X). */
export function worldDirToFtcBearingDeg(dx: number, dz: number): number {
  return Math.atan2(dx, dz) * RAD;
}
