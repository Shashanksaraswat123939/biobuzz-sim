/**
 * Where do scored balls actually come to rest, and is that inside the CELL the CAD draws?
 *
 *   npm run tool -- tools/cellprobe.ts
 *
 * "The balls look stuck in the hive" is a claim about the gap between two different
 * descriptions of the same pocket: the convex boxes the solver collides with, and the
 * tessellated CAD skin drawn over them. This fires a handful of shots, lets them settle, and
 * reports each resting ball in the rocker's own polar frame (radius and angle from the pivot
 * axis) against both descriptions, so the disagreement is a number rather than an impression.
 *
 * The CAD numbers come from probing assets/field.glb directly:
 *   floor plate  ~9.5 in from the pivot
 *   mouth rim    ~21.0 in
 */
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { CAD } from '../packages/core/src/field/geometry.js';
import { inches, M_TO_IN, RAD } from '../packages/core/src/units.js';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import staging from '../assets/staging.json' with { type: 'json' };
import type { BallKind, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const stage = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));
const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

export async function main(): Promise<void> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotJson) as unknown as RobotSpec;
  const dt = p.sim.dt * p.sim.substepsPerFrame;

  const world = new World({ params: p, robot: spec, staging: stage, alliance: 'red', seed: 9, preload: 6 });
  const brain = new BuiltinTeleOp(spec, table, loadLandCal());

  const floor_in = (CAD.cellRadius_in - CAD.cellDepth_in / 2);
  const mouth_in = (CAD.cellRadius_in + CAD.cellDepth_in / 2);
  console.log('the pocket the SOLVER uses, from the pivot axis:');
  console.log(`   floor ${floor_in.toFixed(2)} in   rim ${mouth_in.toFixed(2)} in   half-width ${(CAD.mouthWidth_in / 2).toFixed(1)} in   half-depth ${(CAD.mouthDepth_in / 2).toFixed(1)} in`);
  console.log('the pocket the CAD DRAWS (probed from assets/field.glb):');
  console.log('   floor ~9.5 in   rim ~21.0 in   half-width 10.5 in');
  console.log('');

  // Fire until the hopper runs dry, topping up from the floor as a practice run would.
  const g = emptyGamepad();
  world.step(brain.update(world.sensors(), g, world.seq, dt));
  const fire = { ...emptyGamepad(), right_bumper: true };
  world.setGamepads(fire, emptyGamepad());
  world.step(brain.update(world.sensors(), fire, world.seq, dt));
  for (let i = 0; i < 60 * 45; i++) {
    world.setGamepads(g, emptyGamepad());
    world.step(brain.update(world.sensors(), g, world.seq, dt));
    if (world.seq % 20 === 0) topUp(world, spec);
    if (i % 600 === 0) {
      const rp = world.robot.pos;
      const where = world.robot.heldBalls().map((bb) => {
        const q = world.balls.pos(bb);
        const l = world.robot.toLocal([q[0] - rp[0], q[1] - rp[1], q[2] - rp[2]]);
        return `(${(l[0] * M_TO_IN).toFixed(1)},${(l[1] * M_TO_IN).toFixed(1)},${(l[2] * M_TO_IN).toFixed(1)})`;
      }).join(' ');
      console.log(`   i=${i} worldT=${world.t.toFixed(1)} shots ${world.robot.shots} bin ${world.robot.hopper.length} rpm ${world.robot.flywheelRpm.toFixed(0)} gate ${world.robot.servos.get('gate')!.pos.toFixed(2)} beltOmega ${world.robot.motors.get('transfer')!.omega.toFixed(0)} duty ${world.robot.motors.get('transfer')!.duty.toFixed(2)} lifted ${world.robot.debugFeed.lifted} idx ${world.robot.debugFeed.indexed} "${brain.state.note}"`);
      console.log(`        ${where}`);
      const sb = world.robot.heldBalls().find((bb) => {
        const q = world.balls.pos(bb);
        return world.robot.toLocal([q[0] - rp[0], q[1] - rp[1], q[2] - rp[2]])[1] > -0.09;
      });
      if (sb) {
        const bd = sb.body as unknown as { isSleeping(): boolean; bodyType(): number; mass(): number; linvel(): { x: number; y: number; z: number }; gravityScale(): number };
        const lv = bd.linvel();
        console.log(`        stuck ball: sleeping ${bd.isSleeping()} type ${bd.bodyType()} mass ${bd.mass().toFixed(4)} gScale ${bd.gravityScale()} v ${lv.x.toFixed(3)},${lv.y.toFixed(3)},${lv.z.toFixed(3)}`);
      }
    }
  }
  // What is the stuck ball actually touching? Rapier knows; arithmetic on collider extents
  // kept saying "nothing", and it kept being wrong.
  {
    const rp = world.robot.pos;
    const stuck = world.robot.heldBalls().find((bb) => {
      const q = world.balls.pos(bb);
      return world.robot.toLocal([q[0] - rp[0], q[1] - rp[1], q[2] - rp[2]])[1] > -0.09;
    });
    if (stuck) {
      console.log('');
      console.log('  contacts on the ball sitting in the tube:');
      const pw = world.physics as unknown as {
        contactPairsWith(c: unknown, f: (other: { handle: number; parent(): { handle: number } | null }, m: { numContacts(): number }) => void): void;
      };
      pw.contactPairsWith(stuck.collider, (other, manifold) => {
        const parent = other.parent();
        console.log(`    collider ${other.handle} (body ${parent ? parent.handle : 'static'}) — ${manifold.numContacts()} contact points`);
      });
      const bodyHandle = (world.robot.body as unknown as { handle: number }).handle;
      console.log(`    the robot's body handle is ${bodyHandle}`);
    }
  }

  // Let everything settle.
  for (let i = 0; i < 60 * 5; i++) world.step({ seq: 0, motors: {}, servos: {} });

  const hv = world.hives.red;
  const pivotY = world.geom.pivotY_m;
  const hiveX = world.geom.hiveX_m.red;
  console.log(`fired ${world.robot.shots} shots, rocker at ${(hv.angle * RAD).toFixed(1)} deg, tips ${hv.tips}`);
  console.log('');
  console.log('  ball   radius   angle    along-pivot   where it is');

  let inPocket = 0;
  let embedded = 0;
  for (const b of world.balls.balls) {
    if (!b.body.isEnabled()) continue;
    const q = world.balls.pos(b);
    if (q[1] < pivotY - inches(6)) continue;          // on the floor, not our problem here
    const dY = q[1] - pivotY;
    const dZ = q[2];
    const r = Math.hypot(dY, dZ) * M_TO_IN;
    const ang = Math.atan2(dZ, dY) * RAD;
    const x = (q[0] - hiveX) * M_TO_IN;

    // Which CELL is it near, in the rocker's BODY frame? Undo the live rocker angle.
    const body = ang - hv.angle * RAD;
    const nearA = Math.abs(body - 79.0) < 40;
    const nearB = Math.abs(body + 79.0) < 40;
    let where: string;
    if (!nearA && !nearB) where = 'not in either CELL sector';
    else if (r < floor_in - 1) where = 'INSIDE the floor plate — embedded';
    else if (r > mouth_in + 1) where = 'outside the rim';
    else if (Math.abs(x) > CAD.mouthWidth_in / 2) where = 'past the end wall';
    else where = `in the ${nearA ? 'A' : 'B'} pocket`;
    if (where.startsWith('in the')) inPocket++;
    if (where.startsWith('INSIDE')) embedded++;
    console.log(`  ${String(b.id).padStart(4)}   ${r.toFixed(1).padStart(6)}   ${ang.toFixed(1).padStart(6)}   ${x.toFixed(1).padStart(11)}   ${where}`);
  }
  console.log('');
  console.log(`  ${inPocket} resting properly in a pocket, ${embedded} embedded in geometry`);
  console.log(`  scorer says ${hv.ballsInUpCell} in the up CELL`);
}

function topUp(world: World, spec: RobotSpec): void {
  const r = world.robot;
  if (r.hopper.length >= spec.hopper.capacity) return;
  let best: (typeof world.balls.balls)[number] | null = null;
  let bestD = Infinity;
  for (const b of world.balls.balls) {
    if (b.kind !== 'pollen' || b.state !== 'free') continue;
    const q = world.balls.pos(b);
    if (q[1] > inches(8)) continue;
    const d = Math.hypot(q[0] - r.pos[0], q[2] - r.pos[2]);
    if (d < bestD) { bestD = d; best = b; }
  }
  if (best) r.preload(world.balls, best);
}
