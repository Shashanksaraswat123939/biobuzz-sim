/**
 * The measured land-probability calibration, as a value the brain can be handed.
 *
 * Kept separate from `entryModel.ts` so the core stays free of file reads: this module is
 * imported by the app and the tools, which have the JSON, and `BuiltinTeleOp` just takes
 * the object. `null` means "no calibration measured", and the brain then reports its
 * probability as UNCALIBRATED rather than quietly pretending.
 */
import landcal from '../../../../config/landcal.json' with { type: 'json' };
import { LandCalibration, type LandCalPoint } from './entryModel.js';

export function loadLandCal(): LandCalibration | null {
  const pts = (landcal as { points?: LandCalPoint[] }).points;
  return pts && pts.length ? new LandCalibration(pts) : null;
}
