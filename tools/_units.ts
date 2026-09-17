/**
 * Display units for the tools, matching the UI.
 *
 * The tools compute in inches in places because the shot table is keyed by `range_in` and the
 * FTC field frame is defined in inches -- both are interfaces, not readouts. What gets PRINTED
 * should still be metric, so it agrees with the app and with every number reported back.
 */
export const IN_M = 0.0254;

/** Inches -> metres, as a padded field for a table column. */
export const m = (inches: number, dp = 2, pad = 0): string => `${(inches * IN_M).toFixed(dp).padStart(pad)} m`;

/** Inches -> centimetres, for errors and clearances. */
export const cm = (inches: number, dp = 1, pad = 0): string => `${(inches * IN_M * 100).toFixed(dp).padStart(pad)} cm`;

/** Signed centimetres, where the sign carries the meaning. */
export const cmSigned = (inches: number, dp = 1, pad = 0): string =>
  `${inches >= 0 ? '+' : ''}${(inches * IN_M * 100).toFixed(dp)}`.padStart(pad) + ' cm';

/** Bare metres with no unit, for columns that carry the unit in the header. */
export const mBare = (inches: number, dp = 2, pad = 0): string => (inches * IN_M).toFixed(dp).padStart(pad);

/** Bare centimetres with no unit. */
export const cmBare = (inches: number, dp = 1, pad = 0): string => (inches * IN_M * 100).toFixed(dp).padStart(pad);
