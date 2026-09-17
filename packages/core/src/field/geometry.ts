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
   * THE POCKET'S PLACEMENT AND ITS ORIENTATION ARE TWO DIFFERENT ANGLES, and conflating them
   * was an 11 deg error in the mouth that every shot solution rested on.
   *
   * Both come from `up_back_skin_bbox_in`, which is the pocket's FLOOR plate: Y 47.05..60.00,
   * Z 1.62..9.10. An axis-aligned bbox of a tilted plate does not give its centre position
   * directly -- but it does give the plate's own extent, 12.95 in of Y over 7.48 in of Z, so
   * the plate is hypot(12.95, 7.48) = 14.955 in long (the 14 in mouth plus its skin) and its
   * long axis is atan2(7.48, 12.95) = 30.01 deg off vertical. The pocket axis is the normal
   * to that plate: 59.99 deg from vertical, i.e. 30 deg above horizontal.
   *
   * Walk 12.04 in (cellDepth) up that normal from the plate's centre (53.525, 5.360) and the
   * mouth centre lands at Y 59.55; step +-7 in along the plate and the lips land at 53.49 and
   * 65.61. The manual's Fig 9-10 gives 53.5 and 65.6. Two independent sources, 0.02 in apart.
   *
   * The assembly CENTROID (53.77, 11.28) that this used to key off is a real CAD number and
   * still checks out -- it is just not the pocket's box centre, and using its bearing
   * (48.96 deg) as the pocket's ORIENTATION tilted the mouth 11 deg too steep: the lips came
   * out at 52.5 and 62.7, the apex 2.9 in low, and hoodsweep then chose a 30-85 deg hood for
   * a pocket that faces 41 deg above horizontal instead of 30. See docs/DECISIONS.md.
   */
  cellFloorY_in: (47.05 + 60.0) / 2, // 53.525
  cellFloorZ_in: (1.62 + 9.1) / 2, //  5.360
  cellFloorSpanY_in: 60.0 - 47.05, // 12.95
  cellFloorSpanZ_in: 9.1 - 1.62, //  7.48
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

/** Manual Fig 9-10: the mouth plane leans this far back from vertical. 65.6 - 53.5 = 12.1 = 14*cos(30). */
export const MOUTH_TILT_DEG = Math.atan2(CAD.cellFloorSpanZ_in, CAD.cellFloorSpanY_in) / DEG; // 30.01
export const LIP_LOW_IN = 53.5;
export const LIP_HIGH_IN = 65.6;

/**
 * The rocker's travel, from `cells.arm_tilt_deg` in the CAD: it swings +-30 deg between its
 * end stops. THIS IS A STOP ANGLE, not a pocket angle, and it used to be derived from the
 * CELL centroid's bearing instead -- which is why it read 30.04 and why moving the pocket
 * looked as though it had to move the stops too. It does not: the two are independent facts,
 * and the tip threshold (Hive.gravityTorque, sin(theta + cgBodyAngle)) hangs off this one.
 */
export const REST_ANGLE_DEG = 30.0;

/** Pocket axis in the ROCKER BODY frame, from +Y. 90 - 30 rest = 60 deg from vertical in the world. */
export const CELL_A_AXIS_BODY_ANGLE_DEG = (90 - MOUTH_TILT_DEG) + REST_ANGLE_DEG; // 89.99

/**
 * Direction from the pivot to the pocket's CENTRE, in the rocker body frame, from +Y -- the
 * arm, which is 20 deg off the pocket's own axis. Walk half a pocket depth back down the
 * axis from the mouth centre and measure where you land relative to the pivot.
 */
const cellCentre = (() => {
  const a = ((90 - MOUTH_TILT_DEG) * DEG); // pocket axis in the WORLD at rest
  const y = CAD.cellFloorY_in + (CAD.cellDepth_in / 2) * Math.cos(a) - CAD.pivotY_in;
  const z = CAD.cellFloorZ_in + (CAD.cellDepth_in / 2) * Math.sin(a);
  return { radius_in: Math.hypot(y, z), angle_deg: Math.atan2(z, y) / DEG };
})();
/** 16.438 in. WAS 14.956, the assembly centroid's distance -- a different point. */
export const CELL_RADIUS_IN = cellCentre.radius_in;
/** 70.03 deg. WAS 79.0, chosen so that 79.0 - 48.96 landed on a 30 deg stop. */
export const CELL_A_BODY_ANGLE_DEG = cellCentre.angle_deg + REST_ANGLE_DEG;

export interface CellGeometry {
  id: 'A' | 'B';
  /**
   * Direction from the pivot to the pocket's CENTRE, rocker body frame, from +Y. This places
   * the pocket; it does not orient it. The two differ by 20 deg on this rocker.
   */
  bodyAngle_rad: number;
  /** Orientation of the pocket's own axis (mouth-facing normal), rocker body frame, from +Y. */
  axisAngle_rad: number;
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

function pocketPieces(armAngle_rad: number, axisAngle_rad: number, radius_m: number, tag: string): { pieces: BoxPiece[]; half: Vec3 } {
  const th = inches(CAD.plateThick_in);
  const hx = inches(CAD.mouthWidth_in) / 2; // along pivot axis
  const hu = inches(CAD.cellDepth_in) / 2; // along the pocket axis (floor -> mouth)
  const ht = inches(CAD.mouthDepth_in) / 2; // across the mouth
  const phi = axisAngle_rad;
  const c = Math.cos(phi);
  const s = Math.sin(phi);
  // The pocket's centre sits along the ARM; its box is oriented along the AXIS. Using one
  // angle for both is what tilted the mouth 11 deg.
  const cy = radius_m * Math.cos(armAngle_rad);
  const cz = radius_m * Math.sin(armAngle_rad);

  // (x, u, t) in pocket-local -> rocker body frame. u runs out along the pocket axis.
  const place = (x: number, u: number, t: number): Vec3 => [x, cy + u * c - t * s, cz + u * s + t * c];

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
  const axA = CELL_A_AXIS_BODY_ANGLE_DEG * DEG;
  const radius_m = inches(CELL_RADIUS_IN);

  const a = pocketPieces(phiA, axA, radius_m, 'A');
  const b = pocketPieces(-phiA, -axA, radius_m, 'B');

  const cells: [CellGeometry, CellGeometry] = [
    { id: 'A', bodyAngle_rad: phiA, axisAngle_rad: axA, radius_m, halfInterior: a.half, pieces: a.pieces },
    { id: 'B', bodyAngle_rad: -phiA, axisAngle_rad: -axA, radius_m, halfInterior: b.half, pieces: b.pieces },
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

/**
 * Pocket-local (x, u, t) -> rocker body frame. u runs out along the pocket AXIS from the
 * pocket's centre, t across the mouth, x along the pivot axis.
 *
 * Every caller that needs this used to inline it, and every one of them inlined the same
 * mistake: `(radius + u) * cos(bodyAngle)`, which puts u along the ARM instead of along the
 * axis. They are 20 deg apart. One function now, so there is one place to be wrong.
 */
export function fromCellLocal(cell: CellGeometry, x: number, u: number, t: number): Vec3 {
  const c = Math.cos(cell.axisAngle_rad);
  const s = Math.sin(cell.axisAngle_rad);
  return [
    x,
    cell.radius_m * Math.cos(cell.bodyAngle_rad) + u * c - t * s,
    cell.radius_m * Math.sin(cell.bodyAngle_rad) + u * s + t * c,
  ];
}

/** Transform a rocker-body-frame point into pocket-local (x, u, t) coordinates. */
export function toCellLocal(cell: CellGeometry, p: Vec3): Vec3 {
  const c = Math.cos(cell.axisAngle_rad);
  const s = Math.sin(cell.axisAngle_rad);
  // Relative to the pocket's CENTRE (down the arm), resolved along the pocket's own AXIS.
  const dy = p[1] - cell.radius_m * Math.cos(cell.bodyAngle_rad);
  const dz = p[2] - cell.radius_m * Math.sin(cell.bodyAngle_rad);
  return [p[0], dy * c + dz * s, -dy * s + dz * c];
}

/** Is a point inside a pocket? Point is in the rocker body frame. */
export function pointInCell(cell: CellGeometry, p: Vec3, margin = 0): boolean {
  const [x, u, t] = toCellLocal(cell, p);
  const h = cell.halfInterior;
  return Math.abs(x) < h[0] + margin && Math.abs(u) < h[1] + margin && Math.abs(t) < h[2] + margin;
}

/** Mouth centre of a cell, in the rocker body frame: down the arm, then out along the axis. */
export function cellMouthCentre(cell: CellGeometry): Vec3 {
  return fromCellLocal(cell, 0, cell.halfInterior[1], 0);
}
