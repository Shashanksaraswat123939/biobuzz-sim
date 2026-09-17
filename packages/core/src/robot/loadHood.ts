/**
 * The fixed-speed hood table, as a value the brain can be handed.
 *
 * Same split as loadCal.ts: the core stays free of file reads, and `null` means "not
 * generated", which makes the brain fall back to the speed-solving table rather than
 * pretending it has a fixed-speed solution.
 */
import hoodCsv from '../../../../java/teamcode/assets/hoodtable.csv?raw';
import { HoodTable } from './hoodTable.js';

export function loadHoodTable(): HoodTable | null {
  const t = HoodTable.fromCsv(hoodCsv as unknown as string);
  return t.isEmpty ? null : t;
}
