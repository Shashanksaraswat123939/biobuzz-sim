/**
 * The FLOWER table as a value the brain can be handed. Same split as loadCal.ts and
 * loadHood.ts: the core stays free of file reads, and `null` means "not generated", which
 * makes FLOWER mode unavailable rather than silently aimless.
 */
import flowerCsv from '../../../../java/teamcode/assets/flowertable.csv?raw';
import { FlowerTable } from './flowerTable.js';

export function loadFlowerTable(): FlowerTable | null {
  const t = FlowerTable.fromCsv(flowerCsv as unknown as string);
  return t.isEmpty ? null : t;
}
