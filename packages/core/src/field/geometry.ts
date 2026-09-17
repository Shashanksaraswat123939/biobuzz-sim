/**
 * Field geometry, derived from the CAD numbers in cad/cad-summary.json.
 *
 * Everything here is SI and in the world frame (Y up, X = pivot axis, Z = audience).
 * The CELL pocket is built from convex boxes on purpose: a physics engine turns a
 * concave mesh into its hull, and a hull across the pocket mouth is the classic
 * "balls float on an invisible lid" bug (PLAN.md section 7).
 */
import { inches, DEG } from '../units.js';
import type { Params, Vec3 } from '../types.js';

/** A convex box collider: half extents in its own frame, placed and rotated about X only. */
export interface BoxPiece {
  name: string;
  half: Vec3; // half extents, m
  pos: Vec3; // centre in the parent frame, m
  rotX: number; // rotation about the X axis, rad
}

// ---- CAD constants (inches), kept here so every derived number has one origin ----
export const CAD = {
  pivotY_in: 43.95,
  redX_in: -12.74,
  blueX_in: 12.76,
  /**
   * Audience-CELL centroid relative to the pivot: (dY 9.82, dZ 11.28) -> radius and angle.
   *
   * Tried deriving the radius from `up_floor_bbox_in` instead, on the theory that the
   * assembly centroid sits too far inside the pocket. It does not work: that bbox is the
   * AXIS-ALIGNED box of a plate tilted 30 deg, so its centre is not the plate's radial
   * position, and the CAD's own staged balls end up below the floor it implies. The
   * centroid is the better estimator of the two. See docs/DECISIONS.md.
   */
  cellRadius_in: Math.hypot(53.77 - 43.95, 11.28), // 14.9556
  cellCadAngle_deg: Math.atan2(11.28, 53.77 - 43.95) / DEG, // 48.9565
  /** Nominal pocket, from the manual and confirmed against the CAD cell bbox to ~0.4 in. */
  mouthWidth_in: 20, // along the pivot axis
  mouthDepth_in: 14, // across the mouth (tangential)
  cellDepth_in: 12.04, // radial
  plateThick_in: 0.25,
  frameHalfX_in: 24.74,
  frameHalfZ_in: 19.48,
  frameTopBarY_in: 41.4,
  acmPanelY_in: [34.1, 40.0] as const,
  flowerXZ_in: [
    [-23.39, -69.77],
    [-69.77, 23.39],
    [69.77, -23.39],
    [23.39, 69.77],
  ] as const,
  flowerRingY_in: { lower: 0.2, middle: 4.34, top: 21.4 },
  flowerBackstopY_in: 22.65,
  flowerOpeningDia_in: 4.0,
};

/**
 * Body angle of CELL A in the rocker frame, measured from +Y.
 * At theta = 0 the CG sits directly above the pivot (the unstable mid-point), which is
 * what makes the rocker an over-centre see-saw held at its end stops by gravity.
 */
export const CELL_A_BODY_ANGLE_DEG = 79.0;
/** So the CAD's saved state (audience CELL up) is theta = -(79.0 - 48.9565) = -30.04 deg. */
export const REST_ANGLE_DEG = CELL_A_BODY_ANGLE_DEG - CAD.cellCadAngle_deg;

export interface CellGeometry {
  id: 'A' | 'B';
  /** Radial direction of the pocket axis in the rocker body frame, measured from +Y. */
  bodyAngle_rad: number;
  /** Pocket centre distance from the pivot, m. */
  radius_m: number;
  /** Half extents of the pocket interior: [alongAxis, radial, tangential], m. */
  halfInterior: Vec3;
  pieces: BoxPiece[];
}

export interface FlowerGeometry {
  index: number;
  x_m: number;
  z_m: number;
  openingR_m: number;
  scoreLow_m: number;
  scoreHigh_m: number;
  topY_m: number;
}

export interface ZoneGeometry {
  name: 'LOADING' | 'GARDEN';
  alliance: 'red' | 'blue';
  min: Vec3;
  max: Vec3;
}

export interface FieldGeometry {
  halfWidth_m: number;
  tileTopY_m: number;
  wallTopY_m: number;
  railTopY_m: number;
  pivotY_m: number;
  hiveX_m: { red: number; blue: number };
  restAngle_rad: number;
  cells: [CellGeometry, CellGeometry];
  cgRadius_m: number;
  frame: BoxPiece[];
  flowers: FlowerGeometry[];
  zones: ZoneGeometry[];
}

function pocketPieces(bodyAngle_rad: number, radius_m: number, tag: string): { pieces: BoxPiece[]; half: Vec3 } {
  const th = inches(CAD.plateThick_in);
  const hx = inches(CAD.mouthWidth_in) / 2; // along pivot axis
  const hu = inches(CAD.cellDepth_in) / 2; // radial (mouth -> floor)
  const ht = inches(CAD.mouthDepth_in) / 2; // tangential
  const phi = bodyAngle_rad;
  const c = Math.cos(phi);
  const s = Math.sin(phi);

  // (x, u, t) in pocket-local -> rocker body frame. u is radial-out from the pivot.
  const place = (x: number, u: number, t: number): Vec3 => [
    x,
    (radius_m + u) * c - t * s,
    (radius_m + u) * s + t * c,
  ];

  const pieces: BoxPiece[] = [
    { name: `${tag}-floor`, half: [hx, th / 2, ht], pos: place(0, -hu + th / 2, 0), rotX: phi },
    { name: `${tag}-wallT+`, half: [hx, hu, th / 2], pos: place(0, 0, ht - th / 2), rotX: phi },
    { name: `${tag}-wallT-`, half: [hx, hu, th / 2], pos: place(0, 0, -ht + th / 2), rotX: phi },
    { name: `${tag}-wallX+`, half: [th / 2, hu, ht], pos: place(hx - th / 2, 0, 0), rotX: phi },
    { name: `${tag}-wallX-`, half: [th / 2, hu, ht], pos: place(-hx + th / 2, 0, 0), rotX: phi },
  ];
  return { pieces, half: [hx - th, hu - th / 2, ht - th] };
}

export function buildFieldGeometry(params: Params): FieldGeometry {
  const halfWidth_m = params.env.fieldInside_m / 2;
  const phiA = CELL_A_BODY_ANGLE_DEG * DEG;
  const radius_m = inches(CAD.cellRadius_in);

  const a = pocketPieces(phiA, radius_m, 'A');
  const b = pocketPieces(-phiA, radius_m, 'B');

  const cells: [CellGeometry, CellGeometry] = [
    { id: 'A', bodyAngle_rad: phiA, radius_m, halfInterior: a.half, pieces: a.pieces },
    { id: 'B', bodyAngle_rad: -phiA, radius_m, halfInterior: b.half, pieces: b.pieces },
  ];

  // Frame: an open A-frame, approximated by four corner posts, a top bar and two ACM side
  // panels. Deliberately open so shots and balls pass through it as they do in reality.
  const fx = inches(CAD.frameHalfX_in);
  const fz = inches(CAD.frameHalfZ_in);
  const topY = inches(CAD.frameTopBarY_in);
  const post = inches(1.0);
  const frame: BoxPiece[] = [];
  for (const sx of [-1, 1]) {
    for (const sz of [-1, 1]) {
      frame.push({ name: `post${sx}${sz}`, half: [post, topY / 2, post], pos: [sx * fx, topY / 2, sz * fz], rotX: 0 });
    }
  }
  frame.push({ name: 'topbar', half: [fx, post, post], pos: [0, topY, 0], rotX: 0 });
  const acmLo = inches(CAD.acmPanelY_in[0]);
  const acmHi = inches(CAD.acmPanelY_in[1]);
  for (const sz of [-1, 1]) {
    frame.push({
      name: `acm${sz}`,
      half: [fx, (acmHi - acmLo) / 2, inches(0.2)],
      pos: [0, (acmLo + acmHi) / 2, sz * fz],
      rotX: 0,
    });
  }

  const flowers: FlowerGeometry[] = CAD.flowerXZ_in.map(([x, z], index) => ({
    index,
    x_m: inches(x),
    z_m: inches(z),
    openingR_m: inches(CAD.flowerOpeningDia_in / 2),
    scoreLow_m: inches(CAD.flowerRingY_in.middle),
    scoreHigh_m: inches(CAD.flowerRingY_in.top),
    topY_m: inches(CAD.flowerBackstopY_in),
  }));

  // LOADING ZONE 23 x 11 in and GARDEN 23 x 2 in, at the alliance stations on the +-X walls.
  // Which wall comes from the CAD: the STEP stages NECTAR at X = +-73.6..76.7 in with
  // |Z| < 4 in, and the eight loose POLLEN at X = +-73.0 in over Z = 26.4..46.0 in. Those
  // are the human player's hands, i.e. just OUTSIDE the glass, so the zones themselves are
  // placed on the tile inside the wall below them -- a robot has to be able to park in one.
  // The manual gives the sizes; the CAD gives the places. _source: derived.
  const zones: ZoneGeometry[] = [];
  const span = (a0: number, b0: number): [number, number] => [Math.min(a0, b0), Math.max(a0, b0)];
  for (const [alliance, sign] of [['red', -1] as const, ['blue', 1] as const]) {
    const wall = sign * halfWidth_m;
    const [lx0, lx1] = span(wall, wall - sign * inches(11));
    const [gx0, gx1] = span(wall, wall - sign * inches(2.5));
    const [gz0, gz1] = span(sign * inches(24.5), sign * inches(47.5));
    zones.push({ name: 'LOADING', alliance, min: [lx0, 0, inches(-11.5)], max: [lx1, inches(11), inches(11.5)] });
    zones.push({ name: 'GARDEN', alliance, min: [gx0, 0, gz0], max: [gx1, inches(4), gz1] });
  }

  return {
    halfWidth_m,
    tileTopY_m: 0,
    wallTopY_m: params.env.wallGlassTop_m,
    railTopY_m: params.env.railTop_m,
    pivotY_m: params.hive.pivotY_m,
    hiveX_m: { red: params.hive.redX_m, blue: params.hive.blueX_m },
    restAngle_rad: REST_ANGLE_DEG * DEG,
    cells,
    cgRadius_m: Math.hypot(params.hive.cgOffset_m[1], params.hive.cgOffset_m[2]),
    frame,
    flowers,
    zones,
  };
}

/** Transform a rocker-body-frame point into pocket-local (x, u, t) coordinates. */
export function toCellLocal(cell: CellGeometry, p: Vec3): Vec3 {
  const c = Math.cos(cell.bodyAngle_rad);
  const s = Math.sin(cell.bodyAngle_rad);
  return [p[0], p[1] * c + p[2] * s - cell.radius_m, -p[1] * s + p[2] * c];
}

/** Is a point inside a pocket? Point is in the rocker body frame. */
export function pointInCell(cell: CellGeometry, p: Vec3, margin = 0): boolean {
  const [x, u, t] = toCellLocal(cell, p);
  const h = cell.halfInterior;
  return Math.abs(x) < h[0] + margin && Math.abs(u) < h[1] + margin && Math.abs(t) < h[2] + margin;
}

/** Mouth centre of a cell, in the rocker body frame. */
export function cellMouthCentre(cell: CellGeometry): Vec3 {
  const c = Math.cos(cell.bodyAngle_rad);
  const s = Math.sin(cell.bodyAngle_rad);
  const r = cell.radius_m + cell.halfInterior[1];
  return [0, r * c, r * s];
}
