import { readFileSync } from 'node:fs';
import { describe, it, expect } from 'vitest';
import entryJson from '../config/entry.json';
import robotJson from '../config/robot.json';
import { normalCdf, pThread } from '../packages/core/src/physics/ballistics.js';
import { EntryModel, LandCalibration, type EntryTable } from '../packages/core/src/robot/entryModel.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { buildZone } from '../tools/shotzone.js';
import { ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import type { RobotSpec } from '../packages/core/src/types.js';

describe('normalCdf', () => {
  it('matches the values everyone knows', () => {
    // A&S 26.2.17 is documented to ~1e-7, so assert to that and not further.
    expect(normalCdf(0)).toBeCloseTo(0.5, 7);
    expect(normalCdf(1)).toBeCloseTo(0.8413447, 6);
    expect(normalCdf(-1)).toBeCloseTo(0.1586553, 6);
    expect(normalCdf(1.959964)).toBeCloseTo(0.975, 5);
    expect(normalCdf(-3)).toBeCloseTo(0.0013499, 6);
  });

  it('is symmetric and monotone', () => {
    for (const z of [0.3, 1.1, 2.7, 4.5]) expect(normalCdf(z) + normalCdf(-z)).toBeCloseTo(1, 7);
    let prev = -1;
    for (let z = -5; z <= 5; z += 0.25) {
      const v = normalCdf(z);
      expect(v).toBeGreaterThanOrEqual(prev);
      prev = v;
    }
  });
});

describe('pThread — the chance the exit speed threads the mouth', () => {
  it('is highest dead centre and falls off either side', () => {
    const centre = pThread(9, 11, 10, 0.5);
    expect(centre).toBeGreaterThan(pThread(9, 11, 10.5, 0.5));
    expect(centre).toBeGreaterThan(pThread(9, 11, 9.5, 0.5));
    expect(pThread(9, 11, 10.5, 0.5)).toBeCloseTo(pThread(9, 11, 9.5, 0.5), 9);
  });

  it('is a two-sigma band when the band is two sigma wide', () => {
    // +-1 sigma either side of centre is the 68% interval.
    expect(pThread(9, 11, 10, 1)).toBeCloseTo(0.6827, 3);
    // +-2 sigma is 95%.
    expect(pThread(8, 12, 10, 1)).toBeCloseTo(0.9545, 3);
  });

  it('rewards a WIDER band and a TIGHTER wheel, which is the whole point', () => {
    expect(pThread(8, 12, 10, 1)).toBeGreaterThan(pThread(9, 11, 10, 1));
    expect(pThread(9, 11, 10, 0.3)).toBeGreaterThan(pThread(9, 11, 10, 1));
  });

  it('degenerates sensibly with no scatter at all', () => {
    expect(pThread(9, 11, 10, 0)).toBe(1);
    expect(pThread(9, 11, 12, 0)).toBe(0);
  });

  it('never returns a probability outside [0, 1]', () => {
    for (const mean of [0, 5, 10, 15, 100]) {
      for (const sig of [0.01, 0.5, 5]) {
        const v = pThread(9, 11, mean, sig);
        expect(v).toBeGreaterThanOrEqual(0);
        expect(v).toBeLessThanOrEqual(1);
      }
    }
  });
});

describe('the measured entry model', () => {
  const model = new EntryModel(entryJson as unknown as EntryTable);

  it('returns its own grid points', () => {
    const t = model.table;
    t.descents_deg.forEach((d, j) => {
      t.speeds_mps.forEach((s, i) => {
        expect(model.lookup(s, d)).toBeCloseTo(t.rate[j][i], 9);
      });
    });
  });

  it('interpolates between them and clamps outside', () => {
    const t = model.table;
    const [s0, s1] = t.speeds_mps;
    const d = t.descents_deg[0];
    expect(model.lookup((s0 + s1) / 2, d)).toBeCloseTo((t.rate[0][0] + t.rate[0][1]) / 2, 9);
    expect(model.lookup(-99, d)).toBeCloseTo(t.rate[0][0], 9);
    expect(model.lookup(9999, d)).toBeCloseTo(t.rate[0][t.speeds_mps.length - 1], 9);
  });

  it('every cell is a probability', () => {
    for (const row of model.table.rate) for (const v of row) {
      expect(v).toBeGreaterThanOrEqual(0);
      expect(v).toBeLessThanOrEqual(1);
    }
  });

  it('says a fast steep arrival is worse than a slow one — the finding this exists for', () => {
    // The margin-only table asked for ~6.2 m/s at 75 deg. The point of measuring entry at
    // all is that this is a bad arrival and nothing in the old objective could see it.
    const fastSteep = model.lookup(6.2, 75);
    const slowMid = model.lookup(3.0, 45);
    expect(fastSteep).toBeLessThan(slowMid);
    expect(fastSteep).toBeLessThan(0.55);
  });
});

describe('the shot table carries what the gate needs', () => {
  const csv = [
    'range_in,hoodPos,rpm,margin,hoodDeg,speedLo,speedHi,sigmaSpeed,pStay',
    '40.0,0.5000,2400,0.0340,60.00,5.0000,5.4000,0.06000,0.8000',
    '60.0,0.6000,2600,0.0320,62.00,5.6000,6.0000,0.07000,0.4000',
  ].join(String.fromCharCode(10));

  it('parses the added columns', () => {
    const t = ShotTable.fromCsv(csv);
    expect(t.rows[0].speedLo).toBeCloseTo(5.0, 6);
    expect(t.rows[0].pStay).toBeCloseTo(0.8, 6);
    expect(t.rows[1].sigmaSpeed).toBeCloseTo(0.07, 6);
  });

  it('still reads a table written before those columns existed', () => {
    const old = ['range_in,hoodPos,rpm,margin', '40.0,0.5000,2400,0.0340'].join(String.fromCharCode(10));
    const t = ShotTable.fromCsv(old);
    expect(t.rows[0].rpm).toBe(2400);
    expect(t.rows[0].pStay).toBeUndefined();
  });

  it('takes the WORSE entry probability when interpolating between rows', () => {
    // Averaging would claim a shot halfway between an 80% row and a 40% row is 60%
    // reliable. It is not reliable in any sense the robot can bank on, so take the floor.
    const t = ShotTable.fromCsv(csv);
    expect(t.lookup(50).pStay).toBeCloseTo(0.4, 6);
    // the other fields DO interpolate
    expect(t.lookup(50).speedLo).toBeCloseTo(5.3, 6);
  });
});

describe('the bearing term', () => {
  it('reads the mouth width out of the shipped table', () => {
    // If the table is ever regenerated by a builder that does not write this column, the
    // robot silently stops accounting for pointing error at all -- P(land) goes back to
    // being a statement about range only, and gets optimistic by exactly the amount that
    // tools/missmix.ts measured going wide (33% of shots at 70 in).
    const t = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
    expect(t.rows.length).toBeGreaterThan(0);
    for (const r of t.rows) expect(r.halfLat_m).toBeGreaterThan(0.05);
  });

  it('costs more probability at long range than at short, for the same pointing error', () => {
    // The whole point of doing this as a probability rather than a "within 3 degrees" gate.
    const halfLat = 0.2121;
    const sigma = (range_m: number) => range_m * Math.tan(1 * (Math.PI / 180));
    const p = (range_m: number, errDeg: number) =>
      pThread(-halfLat, halfLat, range_m * Math.tan(errDeg * (Math.PI / 180)), sigma(range_m));
    expect(p(1.0, 0)).toBeGreaterThan(0.99);
    expect(p(3.0, 3)).toBeLessThan(p(1.0, 3));
    // 5 deg at 3 m puts the mean arrival outside the mouth altogether.
    expect(p(3.0, 5)).toBeLessThan(0.5);
  });
});

describe('LandCalibration', () => {
  const cal = new LandCalibration([
    { score: 0.5, observed: 0.1, n: 10 },
    { score: 0.8, observed: 0.4, n: 20 },
    { score: 0.9, observed: 0.86, n: 14 },
  ]);

  it('interpolates between measured points and clamps outside them', () => {
    expect(cal.apply(0.65)).toBeCloseTo(0.25, 6);
    expect(cal.apply(0.2)).toBeCloseTo(0.1, 6);   // below the lowest bin measured
    expect(cal.apply(1)).toBeCloseTo(0.86, 6);    // a perfect score still only lands 86%
  });

  it('never returns more than the best frequency any bin actually achieved', () => {
    for (let s = 0; s <= 1.0001; s += 0.02) expect(cal.apply(s)).toBeLessThanOrEqual(cal.ceiling);
  });
});

describe('the shot zone map', () => {
  // Coarse on purpose: this is a guard on the map's shape, not a measurement.
  const { cells, threshold } = buildZone(9);

  it('covers ground and stays a probability', () => {
    expect(cells.length).toBeGreaterThan(20);
    for (const c of cells) {
      expect(c.p).toBeGreaterThanOrEqual(0);
      expect(c.p).toBeLessThanOrEqual(1);
    }
  });

  it('gets worse as the approach swings away from the mouth', () => {
    // The CELL mouth is a slot: its opening is fixed in the field frame, so what a shot has
    // to fit through is the opening seen edge-on, and that shrinks with the cosine. If this
    // ever inverts, the aperture is being built in the wrong frame.
    //
    // OVER THE CELLS THAT HAVE A SHOT AT ALL. Including the zeros makes this a comparison of
    // COVERAGE -- how many squares in each wedge happen to sit inside the shot table's range
    // -- which has nothing to do with the aperture and is not what the assertion above says.
    // The near wedge is small and sits mostly too close to the mouth to solve, so 12% of its
    // squares score at all against 20% of the far ones, and the two means crossed over on a
    // recalibration while the shootable squares still ordered correctly (0.83 against 0.73).
    const mean = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
    const near = cells.filter((c) => c.offAxisDeg < 30 && c.p > 0).map((c) => c.p);
    const far = cells.filter((c) => c.offAxisDeg > 45 && c.p > 0).map((c) => c.p);
    expect(near.length).toBeGreaterThan(3);
    expect(far.length).toBeGreaterThan(3);
    expect(mean(near)).toBeGreaterThan(mean(far));
  });

  it('does not claim the whole field is shootable', () => {
    // The point of the map. If everything clears the gate, either the gate is off or the
    // aperture is not being recomputed per spot, and the overlay is decoration.
    const live = cells.filter((c) => c.p >= threshold);
    expect(live.length).toBeGreaterThan(0);
    expect(live.length).toBeLessThan(cells.length * 0.75);
  });
});

describe('the configured threshold', () => {
  it('is set, and is a probability', () => {
    const f = (robotJson as unknown as RobotSpec).flywheel;
    expect(f.minLandProb).toBeDefined();
    expect(f.minLandProb).toBeGreaterThan(0);
    expect(f.minLandProb).toBeLessThanOrEqual(1);
  });

  it('is not above what the shooter has ever been measured doing', () => {
    // The failure this catches is silent and total: with the calibration applied, a
    // threshold above the ceiling can be met by NO shot, so the robot simply never fires
    // and the HUD reports a probability that reads like bad luck. tools/landcal.ts
    // measures the ceiling; this asserts the policy respects it.
    const cal = loadLandCal();
    if (!cal) return;
    expect((robotJson as unknown as RobotSpec).flywheel.minLandProb).toBeLessThanOrEqual(cal.ceiling);
  });
});
