import { describe, it, expect } from 'vitest';
import params from '../config/params.json';
import { buildFieldGeometry, CAD, REST_ANGLE_DEG, pointInCell, toCellLocal } from '../packages/core/src/field/geometry.js';
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
  it('the rest angle derived from the CAD is 30.04 deg, matching the plan', () => {
    expect(REST_ANGLE_DEG).toBeGreaterThan(29.5);
    expect(REST_ANGLE_DEG).toBeLessThan(30.5);
  });

  it('at the CAD rest angle CELL A sits where the CAD audience CELL sits', () => {
    // Rotating cell A's body angle by theta = -restAngle must reproduce the CAD centroid
    // (dY 9.82 in, dZ 11.28 in above/outboard of the pivot).
    const theta = -G.restAngle_rad;
    const phi = G.cells[0].bodyAngle_rad + theta;
    const r = toInches(G.cells[0].radius_m);
    expect(r * Math.cos(phi)).toBeCloseTo(53.77 - 43.95, 2);
    expect(r * Math.sin(phi)).toBeCloseTo(11.28, 2);
  });

  it('CELL B is the mirror of CELL A through the Z = 0 plane at the other rest angle', () => {
    const phi = G.cells[1].bodyAngle_rad + G.restAngle_rad;
    const r = toInches(G.cells[1].radius_m);
    expect(r * Math.cos(phi)).toBeCloseTo(53.77 - 43.95, 2);
    expect(r * Math.sin(phi)).toBeCloseTo(-11.28, 2);
  });

  it('the pocket box fits inside the CAD up-CELL bbox to better than an inch', () => {
    // CAD up-cell bbox relative to the pivot: Y 0.00..22.18 in, Z 1.57..20.06 in.
    const theta = -G.restAngle_rad;
    const cell = G.cells[0];
    let minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
    const h = cell.halfInterior;
    for (const su of [-1, 1]) {
      for (const st of [-1, 1]) {
        const phi = cell.bodyAngle_rad + theta;
        const rr = cell.radius_m + su * h[1];
        const y = rr * Math.cos(phi) - st * h[2] * Math.sin(phi);
        const z = rr * Math.sin(phi) + st * h[2] * Math.cos(phi);
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
    const c = Math.cos(cell.bodyAngle_rad), s = Math.sin(cell.bodyAngle_rad);
    const centre: Vec3 = [0, cell.radius_m * c, cell.radius_m * s];
    expect(pointInCell(cell, centre)).toBe(true);
    const outside: Vec3 = [0, (cell.radius_m + inches(20)) * c, (cell.radius_m + inches(20)) * s];
    expect(pointInCell(cell, outside)).toBe(false);
    expect(toCellLocal(cell, centre)[1]).toBeCloseTo(0, 9);
  });
});
