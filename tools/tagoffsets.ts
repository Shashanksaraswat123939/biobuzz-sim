/**
 * The rigid offset from the TAG the camera sees to the MOUTH the robot shoots into.
 *
 *   npm run tool -- tools/tagoffsets.ts
 *
 * WHY THIS EXISTS. The am-5888 panels are not on the CELL mouths. `tools/tagstudy.ts` pulls
 * them out of the CAD at a radius of 14.10 in and a body angle of -6.1 deg, while the mouth
 * of the CELL they belong to is at 22.07 in and 14.7 deg -- about 10 in apart, both bolted to
 * the same rocker. A robot that aims where the tag is aims 10 in short of the goal.
 *
 * Both are rigid on the rocker, so in the ROCKER BODY FRAME the vector between them is a
 * constant, and the rocker is bistable, so in the FIELD frame it has exactly two values. Which
 * one applies is told by the TAG ID -- which is the whole argument of docs/LOCALIZATION.md:
 * "the tag ID is the rocker state sensor", and "the pose of a tag you have identified is known
 * exactly". This turns that argument into the four numbers the hub needs.
 *
 * Writes config/tagoffsets.json, which tools/genconstants.mjs bakes into RobotConstants.
 */
import { writeFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import { buildFieldGeometry, cellMouthCentre, fromCellLocal, CAD, type CellGeometry } from '../packages/core/src/field/geometry.js';
import { M_TO_IN, RAD, DEG } from '../packages/core/src/units.js';
import { tagPoses } from './tagstudy.js';
import type { Params } from '../packages/core/src/types.js';

/** Rotate a rocker-body (y, z) through the rocker angle, exactly as Hive.toWorld does. */
function rotate(y: number, z: number, theta_rad: number): { y: number; z: number } {
  const c = Math.cos(theta_rad);
  const s = Math.sin(theta_rad);
  return { y: y * c - z * s, z: y * s + z * c };
}

export function main(): void {
  const p = params as unknown as Params;
  const geom = buildFieldGeometry(p);
  const rest = p.hive.restAngles_deg[1] * DEG; // +30, the magnitude of either stop

  // One hive is enough: the two CELLs of a rocker are mirror images, so the pair of offsets
  // below covers both states, and the red and blue rockers are the same part.
  const tags = tagPoses(p).filter((t) => t.alliance === 'red');
  const byCell: Record<string, { y: number; z: number }> = {};
  for (const t of tags) {
    const b = t.bodyAngle_deg * DEG;
    byCell[t.cell] = { y: t.radius_in * Math.sin(b), z: t.radius_in * Math.cos(b) };
  }

  // Pair each panel with its CELL by the sign of z: the two CELLs sit either side of the
  // pivot and so do the two panels. Matching on that rather than on a name means a rename in
  // the CAD cannot silently swap them.
  const cellOf = (z: number): CellGeometry => {
    const want = Math.sign(z);
    const m = geom.cells.filter((c) => Math.sign(cellMouthCentre(c)[2]) === want);
    if (m.length !== 1) throw new Error('cannot pair a tag panel with exactly one CELL');
    return m[0];
  };

  const states: Record<string, unknown> = {};
  for (const [cellName, tag] of Object.entries(byCell)) {
    const cell = cellOf(tag.z);
    const mouth = cellMouthCentre(cell);
    // Body-frame tag -> mouth. Constant: both parts are bolted to the same rocker.
    const dy = mouth[1] * M_TO_IN - tag.y;
    const dz = mouth[2] * M_TO_IN - tag.z;

    // THE ROCKER ANGLE THAT PUTS THIS CELL UP, taken from Hive's own rule rather than
    // guessed from a sign. `upCell` is whichever cell satisfies cos(bodyAngle + theta) > 0,
    // so ask that question of both stops instead of assuming which one answers it -- an
    // assumption here silently gives one of the two states the other one's normal.
    const theta = [-rest, rest].find((t) => Math.cos(cell.bodyAngle_rad + t) > 0);
    if (theta === undefined) throw new Error(`no stop puts CELL ${cell.id} up`);
    const w = rotate(dy, dz, theta);
    // The mouth's outward normal at that stop, built the way Hive.upCellMouthNormalWorld
    // builds it -- mouth centre minus pocket centre -- so the two cannot disagree.
    const base = fromCellLocal(cell, 0, 0, 0);
    const ny = mouth[1] - base[1];
    const nz = mouth[2] - base[2];
    const nn = Math.hypot(ny, nz) || 1;
    const axis = rotate(ny / nn, nz / nn, theta);

    // THE MOUTH'S ABSOLUTE POSITION, per alliance, in FTC field inches.
    //
    // The offsets above are enough while the camera can see a tag. They are not enough when it
    // cannot: a robot with odometry and no tag still knows where the HIVE is, because the HIVE
    // does not move around the field -- only which of its two CELLs is up changes. These are
    // the four numbers that let it aim anyway.
    //
    // ftcX = worldZ and ftcY = worldX (packages/core/src/field/ftcFrame.ts). The mouth sits on
    // the pivot axis, so its FTC Y is the hive's own X offset and nothing else.
    // cellMouthCentre is ALREADY pivot-relative -- it is the body-frame point Hive.toWorld
    // adds the pivot to. Subtracting the pivot height here as well put the mouth 22 in out.
    const mw = rotate(mouth[1] * M_TO_IN, mouth[2] * M_TO_IN, theta);
    const mouthField = {
      red: { x: +mw.z.toFixed(4), y: +(p.hive.redX_m * M_TO_IN).toFixed(4) },
      blue: { x: +mw.z.toFixed(4), y: +(p.hive.blueX_m * M_TO_IN).toFixed(4) },
    };
    // THE MOUTH RELATIVE TO THE PIVOT, which is the part that is a property of the HIVE
    // rather than of the field.
    //
    // The absolute positions above are a PRIOR -- a coordinate to start from before the robot
    // has seen anything. What the robot should actually track is the pivot, because it is the
    // one point on the HIVE that a TIP does not move: both CELLs swing about it. Estimate the
    // pivot from a sighting and the up CELL's mouth follows from it by this offset, for
    // whichever state the tag ID says the rocker is on.
    const mouthFromAnchor = { x: +mw.z.toFixed(4), y: 0 };

    states[cell.id] = {
      cell: cell.id,
      cadPanel: cellName,
      mouthField_in: mouthField,
      mouthFromAnchor_in: mouthFromAnchor,
      // The mouth's HEIGHT above the floor. A field constant like the rest of this file, and
      // the robot's only source for it -- the shot table encodes it implicitly in every hood
      // angle it was solved with. Drawing the robot's believed mouth needs it explicitly.
      mouthHeight_in: +(mw.y + p.hive.pivotY_m * M_TO_IN).toFixed(4),
      // GROUND PLANE ONLY. The pivot axis is world X and both parts sit on it, so there is no
      // X term; world Z is the field axis the offset actually moves along.
      groundOffset_in: +w.z.toFixed(4),
      heightOffset_in: +w.y.toFixed(4),
      mouthNormalZ: +axis.z.toFixed(6),
      mouthNormalY: +axis.y.toFixed(6),
      bodyOffset_in: { y: +dy.toFixed(4), z: +dz.toFixed(4) },
      rockerAngle_deg: +(theta * RAD).toFixed(2),
    };
  }

  const out = {
    _about:
      'GENERATED by tools/tagoffsets.ts from cad/parts.json + config/params.json. The rigid ' +
      'tag -> CELL mouth offset, per rocker state, in the FTC field frame. Keyed by CELL id, ' +
      'which the tag ID identifies. Re-run after any change to the pocket geometry.',
    _units: 'inches; ground offset is along the field Z axis (audience), height is world Y',
    // The pivot, in FTC inches. The ANCHOR the robot tracks the HIVE by: the only point on it
    // a TIP does not move. This is a PRIOR -- something to aim at before the first sighting --
    // not a constant the aim is built on.
    anchorPrior_in: {
      red: { x: 0, y: +(p.hive.redX_m * M_TO_IN).toFixed(4) },
      blue: { x: 0, y: +(p.hive.blueX_m * M_TO_IN).toFixed(4) },
    },
    // THE OTHER HIVE, as the obstacle it is. Its two CELLs swing about its pivot at a fixed
    // radius, so the volume they can occupy is a DISC -- the same shape whichever stop the
    // rocker is on, which is why a shot does not need to know the opponent's state.
    // packages/core/src/robot/clearShot.ts uses this; the shot solver never knew it existed.
    obstacle_in: {
      // For a RED robot the obstacle is the blue pivot, and the other way round.
      red: { x: 0, y: +(p.hive.blueX_m * M_TO_IN).toFixed(4) },
      blue: { x: 0, y: +(p.hive.redX_m * M_TO_IN).toFixed(4) },
      // The MOUTH's distance from the pivot, not the arm's length: the mouth is the furthest
      // point of the structure, so it is what sets the radius the CELLs sweep. CELL_RADIUS_IN
      // is the arm (16.4 in) and using it would have drawn the obstacle 5.6 in too small.
      radius_in: +(Math.hypot(
        cellMouthCentre(geom.cells[0])[1],
        cellMouthCentre(geom.cells[0])[2],
      ) * M_TO_IN).toFixed(4),
      halfWidth_in: +(CAD.mouthWidth_in / 2).toFixed(4),
    },
    states,
  };
  writeFileSync(new URL('../config/tagoffsets.json', import.meta.url), JSON.stringify(out, null, 2) + '\n');

  console.log('TAG -> MOUTH, the offset the robot has to apply to what it can see\n');
  console.log('  The am-5888 panel is NOT on the mouth. Both are bolted to the rocker, so the');
  console.log('  vector between them is rigid, and the rocker is bistable, so it has two values.');
  console.log('  Which one applies is told by the TAG ID (docs/LOCALIZATION.md).\n');
  console.log('  cell   panel        body dy,dz     rocker    ground offset   height offset');
  for (const s of Object.values(states) as Record<string, number | string | { y: number; z: number }>[]) {
    const b = s.bodyOffset_in as { y: number; z: number };
    console.log(
      `  ${String(s.cell).padEnd(6)} ${String(s.cadPanel).padEnd(11)} ` +
        `${b.y.toFixed(2).padStart(6)},${b.z.toFixed(2).padStart(7)} in ` +
        `${String(s.rockerAngle_deg).padStart(7)} deg ` +
        `${Number(s.groundOffset_in).toFixed(2).padStart(9)} in ` +
        `${Number(s.heightOffset_in).toFixed(2).padStart(12)} in`,
    );
  }
  console.log('\n  config/tagoffsets.json written. Run node tools/genconstants.mjs to bake it.');
  console.log('\n  The ground offset is what moves the aim; the height offset does not, because');
  console.log('  the shot table is indexed on GROUND range. Aiming at the tag instead of the');
  console.log('  mouth is worth the ground figure above against a mouth half-width of 8.3 in.');
}
