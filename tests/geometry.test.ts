import { describe, it, expect } from 'vitest';
import params from '../config/params.json';
import {
  buildFieldGeometry, CAD, REST_ANGLE_DEG, MOUTH_TILT_DEG, LIP_LOW_IN, LIP_HIGH_IN,
  pointInCell, toCellLocal, fromCellLocal, cellMouthCentre,
} from '../packages/core/src/field/geometry.js';
import { mouthLips } from '../tools/shottable.js';
import { worldToFtc, ftcToWorld, worldYawToFtcHeadingDeg } from '../packages/core/src/field/ftcFrame.js';
import { inches, toInches, DEG } from '../packages/core/src/units.js';
import type { Params, Vec3 } from '../packages/core/src/types.js';

const P = params as unknown as Params;
const G = buildFieldGeometry(P);

describe('field sanity (PLAN.md section 6)', () => {
  it('POLLEN is 71.1 mm within 1%', () => {
    expect(P.ball.pollen.d_m).toBeCloseTo(0.0711, 3);
  });
  it('pivot is at Y = 1.1163 m within 1 mm', () => {
    expect(Math.abs(G.pivotY_m - 1.1163)).toBeLessThan(0.001);
  });
  it('inside width is 3.590 m within 5 mm', () => {
    expect(Math.abs(G.halfWidth_m * 2 - 3.59)).toBeLessThan(0.005);
  });
  it('four FLOWER openings are 0.1016 m diameter', () => {
    expect(G.flowers).toHaveLength(4);
    for (const f of G.flowers) expect(f.openingR_m * 2).toBeCloseTo(0.1016, 4);
  });
});

describe('ftcFrame', () => {
  const corners: Vec3[] = [
    [-G.halfWidth_m, 0, -G.halfWidth_m],
    [G.halfWidth_m, 0, -G.halfWidth_m],
    [-G.halfWidth_m, 0, G.halfWidth_m],
    [G.halfWidth_m, 0, G.halfWidth_m],
  ];
  it('round-trips the four field corners and both hive centres', () => {
    const pts: Vec3[] = [...corners, [G.hiveX_m.red, G.pivotY_m, 0], [G.hiveX_m.blue, G.pivotY_m, 0]];
    for (const p of pts) {
      const back = ftcToWorld(worldToFtc(p));
      for (let i = 0; i < 3; i++) expect(back[i]).toBeCloseTo(p[i], 9);
    }
  });
  it('is right-handed: world +Z (audience) is FTC +X, world +X is FTC +Y, world +Y is FTC +Z', () => {
    expect(worldToFtc([0, 0, 1])).toEqual([toInches(1), 0, 0]);
    expect(worldToFtc([1, 0, 0])).toEqual([0, toInches(1), 0]);
    expect(worldToFtc([0, 1, 0])).toEqual([0, 0, toInches(1)]);
  });
  it('the red hive is at FTC Y = -12.74 in, blue at +12.76 in', () => {
    expect(worldToFtc([G.hiveX_m.red, 0, 0])[1]).toBeCloseTo(CAD.redX_in, 2);
    expect(worldToFtc([G.hiveX_m.blue, 0, 0])[1]).toBeCloseTo(CAD.blueX_in, 2);
  });
  it('world yaw is FTC heading with no sign flip', () => {
    expect(worldYawToFtcHeadingDeg(Math.PI / 2)).toBeCloseTo(90, 9);
  });
});

describe('rocker geometry', () => {
  it('the rocker swings +-30 deg, the CAD arm tilt', () => {
    // cells.arm_tilt_deg in cad-summary.json. This is a STOP angle: it sets the holding
    // torque and therefore the tip threshold, and it is NOT the pocket's angle. It used to
    // be derived from the CELL centroid's bearing, which tangled the two together.
    expect(REST_ANGLE_DEG).toBeCloseTo(30.0, 6);
  });

  it('PUTS THE CELL MOUTH WHERE THE MANUAL DOES', () => {
    // The check that would have caught an 11 deg error in the pocket. Manual Fig 9-10: the
    // opening's base lip is 53.5 in off the tiles and its apex 65.6 in, and the difference
    // 12.1 = 14*cos(30) is the manual agreeing with itself. The tolerance is half an inch
    // because halfInterior is inset by the 0.25 in plate, so these are the INTERIOR lips.
    const lips = mouthLips(P);
    expect(toInches(lips.near.y)).toBeCloseTo(LIP_LOW_IN, 0);
    expect(toInches(lips.far.y)).toBeCloseTo(LIP_HIGH_IN, 0);
    expect(Math.abs(toInches(lips.near.y) - LIP_LOW_IN)).toBeLessThan(0.5);
    expect(Math.abs(toInches(lips.far.y) - LIP_HIGH_IN)).toBeLessThan(0.5);
  });

  it('points the pocket axis 30 deg above horizontal at rest, not 41', () => {
    // The pocket axis in the world = axisAngle - restAngle from +Y. The old model read the
    // arm's bearing (48.96 deg from vertical, i.e. 41 deg above horizontal) for both.
    const axisFromVertical = (G.cells[0].axisAngle_rad - G.restAngle_rad) / DEG;
    expect(90 - axisFromVertical).toBeCloseTo(MOUTH_TILT_DEG, 1);
    expect(MOUTH_TILT_DEG).toBeCloseTo(30.0, 1);
  });

  it('places the pocket on an arm 20 deg off its own axis', () => {
    // The two angles the simulator used to share. If they are ever equal again, the mouth
    // is wrong by exactly their difference.
    const arm = G.cells[0].bodyAngle_rad / DEG;
    const axis = G.cells[0].axisAngle_rad / DEG;
    expect(axis - arm).toBeGreaterThan(15);
    expect(toInches(G.cells[0].radius_m)).toBeCloseTo(16.44, 1);
  });

  it('CELL B is the mirror of CELL A through the Z = 0 plane', () => {
    const a = cellMouthCentre(G.cells[0]);
    const b = cellMouthCentre(G.cells[1]);
    expect(b[1]).toBeCloseTo(a[1], 9);
    expect(b[2]).toBeCloseTo(-a[2], 9);
  });

  it('the pocket box fits inside the CAD up-CELL bbox to better than an inch', () => {
    // CAD up-cell bbox relative to the pivot: Y 0.00..22.18 in, Z 1.57..20.06 in.
    const theta = -G.restAngle_rad;
    const cell = G.cells[0];
    let minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
    const h = cell.halfInterior;
    for (const su of [-1, 1]) {
      for (const st of [-1, 1]) {
        const q = fromCellLocal(cell, 0, su * h[1], st * h[2]);
        const y = q[1] * Math.cos(theta) - q[2] * Math.sin(theta);
        const z = q[1] * Math.sin(theta) + q[2] * Math.cos(theta);
        minY = Math.min(minY, toInches(y)); maxY = Math.max(maxY, toInches(y));
        minZ = Math.min(minZ, toInches(z)); maxZ = Math.max(maxZ, toInches(z));
      }
    }
    expect(minY).toBeGreaterThan(-1.0);
    expect(maxY).toBeLessThan(22.18 + 1.0);
    expect(minZ).toBeGreaterThan(1.57 - 1.0);
    expect(maxZ).toBeLessThan(20.06 + 1.0);
  });

  it('the CG sits directly above the pivot at theta = 0 (the over-centre point)', () => {
    const cg = P.hive.cgOffset_m;
    const angleFromUp = Math.atan2(cg[2], cg[1]) / DEG;
    expect(Math.abs(angleFromUp)).toBeLessThan(2.0);
  });

  it('gravity torque at the rest angle is ~0.6 N.m, matching the plan', () => {
    const r = Math.hypot(P.hive.cgOffset_m[1], P.hive.cgOffset_m[2]);
    const tau = P.hive.massKg * P.env.g * r * Math.sin(G.restAngle_rad);
    expect(tau).toBeGreaterThan(0.55);
    expect(tau).toBeLessThan(0.70);
  });

  it('pointInCell accepts the pocket centre and rejects a point outside the mouth', () => {
    const cell = G.cells[0];
    const centre: Vec3 = fromCellLocal(cell, 0, 0, 0);
    expect(pointInCell(cell, centre)).toBe(true);
    const outside: Vec3 = fromCellLocal(cell, 0, inches(20), 0);
    expect(pointInCell(cell, outside)).toBe(false);
    expect(toCellLocal(cell, centre)[1]).toBeCloseTo(0, 9);
  });
});
