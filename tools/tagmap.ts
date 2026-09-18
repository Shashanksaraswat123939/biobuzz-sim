/**
 * Where on the floor can the robot SEE the goal?
 *
 *   npm run tool -- tools/tagmap.ts [--step 4] [--tipped]
 *
 * `tools/shotzone.ts` answers "where is a shot worth taking". That question used to be the
 * whole story, because the robot was handed the CELL's bearing and range for free from
 * anywhere on the field. It is not handed them any more (docs/PHYSICS.md 6): it reads an
 * AprilTag through a camera on the turret, and the camera is blind past its range, past its
 * grazing limit, and while the rocker is swinging.
 *
 * So there is now a SECOND map, and the usable field is the intersection of the two. A spot
 * the physics loves is worth nothing if no tag can be decoded from it, and this is the tool
 * that says which spots those are.
 *
 * WHAT IT DOES NOT MODEL, deliberately: the field of view. The lens rides the turret and the
 * turret tracks the CELL, so "is it pointed at the tag" is a question about whether the aim
 * has converged, not about where the robot is standing -- that is phase 4 of
 * docs/LOCALIZATION.md and the reason the camera is mounted there. What is left is the part
 * that IS a function of position: range and incidence on the panel's face.
 *
 * Writes config/tagmap.json for the overlay, and prints the map.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import { buildFieldGeometry } from '../packages/core/src/field/geometry.js';
import { M_TO_IN, RAD, DEG, inches } from '../packages/core/src/units.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

export interface TagCell {
  x_in: number;
  z_in: number;
  /** Ground range from this spot to the panel, inches. */
  range_in: number;
  /** Incidence on the panel's face, degrees. 0 is square on, 90 is edge-on. */
  incidence_deg: number;
  /** Can the pipeline decode it from here at all? */
  visible: boolean;
  /** Why not, when it cannot. '' when it can. */
  reason: 'range' | 'incidence' | '';
}

/** The up CELL's tag panel and its outward normal, in the world frame, at a rocker angle. */
function panelAt(p: Params, theta_rad: number, hiveX_in: number): { pos: Vec3; n: Vec3 } {
  const geom = buildFieldGeometry(p);
  // The up CELL is the one whose radial points up at this angle -- Hive's own rule.
  const cell = geom.cells.find((c) => Math.cos(c.bodyAngle_rad + theta_rad) > 0) ?? geom.cells[0];
  const t = cell.tagBody_m;
  const c = Math.cos(theta_rad);
  const s = Math.sin(theta_rad);
  const rot = (y: number, z: number): [number, number] => [y * c - z * s, y * s + z * c];
  const [py, pz] = rot(t[1], t[2]);
  const nn = Math.hypot(t[1], t[2]) || 1;
  const [ny, nz] = rot(t[1] / nn, t[2] / nn);
  return {
    pos: [inches(hiveX_in), inches(p.hive.pivotY_m * M_TO_IN) + py, pz],
    n: [0, ny, nz],
  };
}

export function main(args: string[] = []): void {
  const arg = (k: string, d: number): number => {
    const i = args.indexOf(`--${k}`);
    return i >= 0 ? Number(args[i + 1]) : d;
  };
  const step = arg('step', 4);
  const tipped = args.includes('--tipped');

  const p = params as unknown as Params;
  const spec = robotJson as unknown as RobotSpec;
  const cam = spec.sensors.tag;
  const geom = buildFieldGeometry(p);
  const half = geom.halfWidth_m * M_TO_IN;

  const rest = p.hive.restAngles_deg[1];
  // Red starts with CELL A up (Hive: side = -1), so the un-tipped state is -rest.
  const theta = (tipped ? rest : -rest) * DEG;
  const panel = panelAt(p, theta, p.hive.redX_m * M_TO_IN);

  // THE SAME GRID AS THE SHOT ZONE, taken from its own cells rather than rebuilt to match.
  // The overlay looks a square up by its "x,z" key, so a grid that differs by half a step --
  // or by a margin -- silently matches nothing and the whole field paints as blind. Reading
  // the coordinates back is the only version of this that cannot drift.
  let grid: { x: number; z: number }[];
  let gridStep = step;
  try {
    const zone = JSON.parse(readFileSync(new URL('../config/shotzone.json', import.meta.url), 'utf8'));
    grid = (zone.cells as { x_in: number; z_in: number }[]).map((c) => ({ x: c.x_in, z: c.z_in }));
    gridStep = zone.step_in ?? step;
    console.log(`  Grid taken from config/shotzone.json: ${grid.length} squares at ${gridStep} in.`);
  } catch {
    grid = [];
    const margin = 12; // keep the robot's own footprint off the wall
    for (let x = -half + margin; x <= half - margin; x += step) {
      for (let z = -half + margin; z <= half - margin; z += step) grid.push({ x: +x.toFixed(2), z: +z.toFixed(2) });
    }
    console.log('  No config/shotzone.json -- using an own grid, the overlay will not match it.');
  }

  const cells: TagCell[] = [];
  {
    for (const { x, z } of grid) {
      const dx = x - panel.pos[0] * M_TO_IN;
      const dz = z - panel.pos[2] * M_TO_IN;
      const d = Math.hypot(dx, dz);
      // Panel -> robot against the panel's outward normal: the angle it is being LOOKED AT
      // from, not the angle to it.
      const nn = Math.hypot(panel.n[0], panel.n[2]) || 1;
      const inc = Math.acos(Math.max(-1, Math.min(1, (dx * panel.n[0] + dz * panel.n[2]) / (d * nn)))) * RAD;
      const okRange = d <= cam.maxRange_in;
      const okInc = inc <= cam.maxIncidence_deg;
      cells.push({
        x_in: x,
        z_in: z,
        range_in: +d.toFixed(1),
        incidence_deg: +inc.toFixed(1),
        visible: okRange && okInc,
        reason: okRange ? (okInc ? '' : 'incidence') : 'range',
      });
    }
  }

  writeFileSync(
    new URL('../config/tagmap.json', import.meta.url),
    JSON.stringify(
      {
        _about:
          'GENERATED by tools/tagmap.ts. Where the tag on the up CELL can be decoded from, per ' +
          'field position, for the un-tipped rocker. Range and incidence only -- the lens rides ' +
          'the turret, so field of view is a question about the aim, not about where you stand.',
        generated: new Date().toISOString(),
        step_in: gridStep,
        tipped,
        maxRange_in: cam.maxRange_in,
        maxIncidence_deg: cam.maxIncidence_deg,
        cells,
      },
      null,
      2,
    ) + '\n',
  );

  const vis = cells.filter((c) => c.visible).length;
  const byRange = cells.filter((c) => c.reason === 'range').length;
  const byInc = cells.filter((c) => c.reason === 'incidence').length;

  console.log('WHERE THE ROBOT CAN SEE THE GOAL\n');
  console.log(`  ${cam.maxRange_in} in of range, ${cam.maxIncidence_deg} deg of grazing limit, rocker ${tipped ? 'TIPPED' : 'at its start stop'}.`);
  console.log(`  ${vis} of ${cells.length} squares (${((100 * vis) / cells.length).toFixed(0)}%) can decode the tag.`);
  console.log(`  Blind: ${byInc} too far round the side, ${byRange} too far away.\n`);

  // An ASCII map, because the answer is a shape and a table of numbers is not a shape.
  //   #  can see it        .  too far round the side        _  too far away
  const xs = [...new Set(cells.map((c) => c.x_in))].sort((a, b) => a - b);
  const zs = [...new Set(cells.map((c) => c.z_in))].sort((a, b) => b - a);
  const at = new Map(cells.map((c) => [`${c.x_in},${c.z_in}`, c]));
  console.log('        +X (red wall) ->                    # sees it   . grazing   _ too far');
  for (const z of zs) {
    let line = '';
    for (const x of xs) {
      const c = at.get(`${x},${z}`);
      line += !c ? ' ' : c.visible ? '#' : c.reason === 'incidence' ? '.' : '_';
    }
    console.log(`  ${z.toFixed(0).padStart(5)} ${line}`);
  }
  console.log('     ^ Z, audience at the top. The HIVE mouth opens toward +Z at this stop.\n');
  console.log('  config/tagmap.json written.');
  console.log('  Compare with tools/shotzone.ts: the field you can actually USE is the overlap --');
  console.log('  somewhere the shot is forgiving AND the tag is readable.');
}
