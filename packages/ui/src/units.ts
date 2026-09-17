/**
 * Display units. One place, so "what the screen says" is a decision made once.
 *
 * The physics is already SI -- `packages/core` works in metres, kilograms, newtons and
 * seconds throughout, and nothing here changes a simulated number. Inches survive in two
 * places on purpose and neither is a unit of physics:
 *
 *   the FTC FIELD FRAME (`ftcFrame.ts`) is defined in inches by FIRST, and the AprilTag
 *   positions and the hub's own localizer speak it. Reinterpreting it would put the sim and
 *   the robot in different coordinate systems.
 *
 *   the SHOT TABLE's `range_in` column, which `genconstants.mjs` bakes into Java as
 *   `RANGE_IN`. It is an interface with the deliverable, not a readout.
 *
 * So both stay as they are, and everything a person READS is converted here instead.
 */

const IN_TO_M = 0.0254;

/** Inches -> metres, for distances that are naturally metres: positions, ranges, sweeps. */
export const m = (inches: number, dp = 2): string => `${(inches * IN_TO_M).toFixed(dp)} m`;

/**
 * Inches -> centimetres, for the small ones: misses, bias, group spread.
 *
 * A 6 in miss is 0.15 m, and a column of "0.15 m / 0.08 m / 0.23 m" is harder to compare at a
 * glance than "15 cm / 8 cm / 23 cm". Same number, fewer leading zeros.
 */
export const cm = (inches: number, dp = 1): string => `${(inches * IN_TO_M * 100).toFixed(dp)} cm`;

/** Signed centimetres, where the sign is the point: +long, -short. */
export const cmSigned = (inches: number, dp = 1): string =>
  `${inches >= 0 ? '+' : ''}${(inches * IN_TO_M * 100).toFixed(dp)} cm`;

/** Metres per second, from the SI value the snapshot already carries. */
export const mps = (metresPerSecond: number, dp = 2): string => `${metresPerSecond.toFixed(dp)} m/s`;

/** Metres, from a value that is already metres. */
export const metres = (v: number, dp = 2): string => `${v.toFixed(dp)} m`;

/** Millimetres, for the small mechanical dimensions in the tuning panel. */
export const mm = (metresValue: number, dp = 0): string => `${(metresValue * 1000).toFixed(dp)}`;
