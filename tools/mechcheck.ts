/**
 * Does the mechanism actually work as a mechanism?
 *
 *   npm run tool -- tools/mechcheck.ts
 *
 * Since the intake, the bin and the feed became real geometry driven by real contact
 * forces, none of them can be verified by reading the code -- a roller that grips too
 * weakly and a shaft 2 mm too narrow both look perfectly reasonable on the page and both
 * produce a robot that never fires. This drives each stage on its own and says where a
 * ball actually got to.
 */
import { readFileSync } from 'node:fs';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { M_TO_IN } from '../packages/core/src/units.js';
import params from '../config/params.json' with { type: 'json' };
import robotJson from '../config/robot.json' with { type: 'json' };
import staging from '../assets/staging.json' with { type: 'json' };
import type { ActuatorFrame, BallKind, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const stage = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));
const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));

const motor = (power: number): ActuatorFrame['motors'][string] => ({ mode: 'RUN_WITHOUT_ENCODER', power });

export async function main(): Promise<void> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = structuredClone(robotJson) as unknown as RobotSpec;
  const dt = p.sim.dt * p.sim.substepsPerFrame;

  const mk = (preload: number) =>
    new World({ params: p, robot: spec, staging: stage, alliance: 'red', seed: 3, preload });

  // ---------------------------------------------------- 1. does a preload stay put?
  {
    const w = mk(4);
    const before = w.robot.hopper.length;
    for (let i = 0; i < 240; i++) w.step({ seq: i, motors: {}, servos: {} });
    const local = w.robot.hopper.map((b) => {
      const q = w.balls.pos(b);
      const r = w.robot.pos;
      return w.robot.toLocal([q[0] - r[0], q[1] - r[1], q[2] - r[2]]).map((v) => +(v * M_TO_IN).toFixed(1));
    });
    console.log('1. four preloaded balls, 2 s of nothing');
    console.log(`   in the bin at start: ${before}, after: ${w.robot.hopper.length}, in the shaft: ${w.robot.heldBalls().length - w.robot.hopper.length}`);
    console.log(`   escaped: ${4 - w.robot.heldBalls().length}`);
    for (const l of local) console.log(`     local x ${l[0]}  y ${l[1]}  z ${l[2]} in`);
  }

  // ---------------------------------------------------- 2. does the feed lift a ball?
  {
    const w = mk(4);
    console.log('');
    console.log('2. transfer running, gate open, flywheel off — does a ball reach the nip?');
    let topped = -1;
    for (let i = 0; i < 600; i++) {
      w.step({ seq: i, motors: { transfer: motor(1) }, servos: { gate: 1 } });
      const held = w.robot.heldBalls();
      const r = w.robot.pos;
      const top = Math.max(-99, ...held.map((b) => {
        const q = w.balls.pos(b);
        return w.robot.toLocal([q[0] - r[0], q[1] - r[1], q[2] - r[2]])[1];
      }));
      if (top > spec.turret.muzzleHeight_m - spec.chassis.height_m / 2 - spec.chassis.clearance_m - 0.04 && topped < 0) topped = i;
      if (i % 120 === 0) console.log(`   t=${(i * dt).toFixed(1)}s  bin ${w.robot.hopper.length}  shaft ${held.length - w.robot.hopper.length}  highest ball local y ${(top * M_TO_IN).toFixed(1)} in`);
    }
    console.log(topped >= 0 ? `   reached the nip at t=${(topped * dt).toFixed(2)} s` : '   NEVER reached the nip');
  }

  // ---------------------------------------------------- 3. a full shot, end to end
  {
    const w = mk(4);
    const brain = new BuiltinTeleOp(spec, table, loadLandCal());
    console.log('');
    console.log('3. flywheel up and firing');
    const g = emptyGamepad();
    // toggle the fire latch on the second frame, the way an edge works
    w.step(brain.update(w.sensors(), g, w.seq, dt));
    const fire = { ...emptyGamepad(), right_bumper: true };
    w.setGamepads(fire, emptyGamepad());
    w.step(brain.update(w.sensors(), fire, w.seq, dt));
    for (let i = 0; i < 1200; i++) {
      w.setGamepads(g, emptyGamepad());
      w.step(brain.update(w.sensors(), g, w.seq, dt));
      if (i % 240 === 0) {
        const rp = w.robot.pos;
        for (const bb of w.robot.hopper) {
          const vv = bb.body.linvel();
          console.log(`      bin ball ${bb.id}: |v| ${Math.hypot(vv.x, vv.y, vv.z).toFixed(3)} m/s  sleeping ${bb.body.isSleeping()}`);
        }
        const where = w.robot.heldBalls().map((bb) => {
          const q = w.balls.pos(bb);
          const l = w.robot.toLocal([q[0] - rp[0], q[1] - rp[1], q[2] - rp[2]]);
          return `(${(l[0] * M_TO_IN).toFixed(1)},${(l[1] * M_TO_IN).toFixed(1)},${(l[2] * M_TO_IN).toFixed(1)})`;
        }).join(' ');
        console.log(`   t=${(i * dt).toFixed(1)}s  rpm ${w.robot.flywheelRpm.toFixed(0)}  bin ${w.robot.hopper.length}  shots ${w.robot.shots}  held ${where}  feed run=${w.robot.debugFeed.running} idx=${w.robot.debugFeed.indexed} lift=${w.robot.debugFeed.lifted}  "${brain.state.note}"`);
      }
    }
    console.log(`   fired ${w.robot.shots} of 4 preloaded`);
  }

  // ---------------------------------------------------- 3b. a FULL hopper
  {
    const w = mk(6);
    const brain = new BuiltinTeleOp(spec, table, loadLandCal());
    console.log('');
    console.log('3b. full hopper (6) and firing — does a crowded bin still index?');
    const g = emptyGamepad();
    w.step(brain.update(w.sensors(), g, w.seq, dt));
    const fire = { ...emptyGamepad(), right_bumper: true };
    w.setGamepads(fire, emptyGamepad());
    w.step(brain.update(w.sensors(), fire, w.seq, dt));
    for (let i = 0; i < 1800; i++) {
      w.setGamepads(g, emptyGamepad());
      w.step(brain.update(w.sensors(), g, w.seq, dt));
      if (i % 300 === 0) {
        const rp = w.robot.pos;
        const where = w.robot.heldBalls().map((bb) => {
          const q = w.balls.pos(bb);
          const l = w.robot.toLocal([q[0] - rp[0], q[1] - rp[1], q[2] - rp[2]]);
          return `(${(l[0] * M_TO_IN).toFixed(1)},${(l[1] * M_TO_IN).toFixed(1)},${(l[2] * M_TO_IN).toFixed(1)})`;
        }).join(' ');
        console.log(`   t=${(i * dt).toFixed(1)}s bin ${w.robot.hopper.length} shaft ${w.robot.heldBalls().length - w.robot.hopper.length} shots ${w.robot.shots} idx ${w.robot.debugFeed.indexed} lift ${w.robot.debugFeed.lifted}`);
        console.log(`        ${where}`);
      }
    }
    console.log(`   fired ${w.robot.shots} of 6`);
  }

  // ---------------------------------------------------- 4b. one ball, watched frame by frame
  {
    const w = mk(1);
    console.log('');
    console.log('4b. a single bin ball with the indexer running, frame by frame');
    const b = w.robot.hopper[0];
    // Put it in the REAR slot, behind the shaft: the front slot already works, and the
    // rear one is the position a preloaded ball got stuck in for a whole match.
    {
      const rp = w.robot.pos;
      const wv = w.robot.toWorld([0, -0.10, -0.1226]);
      w.balls.release(b, [rp[0] + wv[0], rp[1] + wv[1], rp[2] + wv[2]], [0, 0, 0], [0, 0, 0], 'hopper');
      for (let k = 0; k < 30; k++) w.step({ seq: k, motors: {}, servos: {} });
    }
    // What the robot is actually built out of, in its own frame. Guessing at collider
    // extents from the construction code is how three wrong fixes happened.
    {
      const body = w.robot.body as unknown as { numColliders(): number; collider(i: number): { halfExtents?: () => { x: number; y: number; z: number }; shape: { halfExtents?: { x: number; y: number; z: number } }; translationWrtParent?: () => { x: number; y: number; z: number } } };
      console.log('   robot colliders (local centre / half-extents, inches):');
      for (let ci = 0; ci < body.numColliders(); ci++) {
        const col = body.collider(ci) as unknown as { shape: { halfExtents?: { x: number; y: number; z: number } }; translationWrtParent?: () => { x: number; y: number; z: number } };
        const he = col.shape.halfExtents;
        const tr = col.translationWrtParent ? col.translationWrtParent() : null;
        if (!he || !tr) continue;
        console.log(`     c${ci} at ${(tr.x * M_TO_IN).toFixed(2)},${(tr.y * M_TO_IN).toFixed(2)},${(tr.z * M_TO_IN).toFixed(2)}  half ${(he.x * M_TO_IN).toFixed(2)},${(he.y * M_TO_IN).toFixed(2)},${(he.z * M_TO_IN).toFixed(2)}`);
      }
    }
    for (let i = 0; i < 90; i++) {
      w.step({ seq: i, motors: { transfer: motor(1) }, servos: { gate: 1 } });
      if (i % 6 === 0) {
        const q = w.balls.pos(b);
        const rp = w.robot.pos;
        const l = w.robot.toLocal([q[0] - rp[0], q[1] - rp[1], q[2] - rp[2]]);
        const v = b.body.linvel();
        console.log(`   f=${i}  local ${(l[0] * M_TO_IN).toFixed(2)}/${(l[1] * M_TO_IN).toFixed(2)}/${(l[2] * M_TO_IN).toFixed(2)} in   v ${v.x.toFixed(3)},${v.y.toFixed(3)},${v.z.toFixed(3)}   idx ${w.robot.debugFeed.indexed} lift ${w.robot.debugFeed.lifted}  ${b.state}`);
      }
    }
  }

  // ---------------------------------------------------- 4. can it pick a ball off the floor?
  {
    const w = mk(0);
    console.log('');
    console.log('4. intake: drive at a loose POLLEN and try to collect it');
    // Put one ball a foot in front of the robot.
    const b = w.balls.balls.find((x) => x.kind === 'pollen')!;
    const r = w.robot.pos;
    const fwd = w.robot.toWorld([0, 0, 0.45]);
    w.balls.release(b, [r[0] + fwd[0], b.radius + 0.005, r[2] + fwd[2]], [0, 0, 0], [0, 0, 0], 'free');
    for (let i = 0; i < 60; i++) w.step({ seq: i, motors: {}, servos: {} });
    // Drive STRAIGHT. The wire carries what each motor is actually told, so the per-wheel
    // reversals in robot.json have to be applied here exactly as the TeleOp mixer applies
    // them -- four equal powers makes the robot spin on the spot, which is what it did.
    const dm = spec.drivetrain.motors;
    const dir = (k: 'fl' | 'fr' | 'bl' | 'br') => (dm[k].reversed ? -1 : 1);
    const drive: ActuatorFrame = {
      seq: 0,
      motors: {
        fl: motor(0.3 * dir('fl')), fr: motor(0.3 * dir('fr')),
        bl: motor(0.3 * dir('bl')), br: motor(0.3 * dir('br')),
        intake: motor(1),
      },
      servos: {},
    };
    let got = -1;
    for (let i = 0; i < 360; i++) {
      w.step({ ...drive, seq: i });
      if (w.robot.hopper.length + (w.robot.heldBalls().length - w.robot.hopper.length) > 0 && got < 0) got = i;
      if (i % 45 === 0) {
        const q = w.balls.pos(b);
        const rp = w.robot.pos;
        const l = w.robot.toLocal([q[0] - rp[0], q[1] - rp[1], q[2] - rp[2]]);
        const bv = b.body.linvel();
        const rv = w.robot.toLocal([bv.x - w.robot.vel[0], bv.y - w.robot.vel[1], bv.z - w.robot.vel[2]]);
        const im = (w.robot as unknown as { motors: Map<string, { omega: number; duty: number }> }).motors.get('intake')!;
        console.log(`   t=${(i * dt).toFixed(1)}s  z ${(l[2] * M_TO_IN).toFixed(1)}  y ${(l[1] * M_TO_IN).toFixed(1)}  pulled-in speed ${(-rv[2]).toFixed(2)} m/s  roller ${im.omega.toFixed(0)} rad/s  gripped ${w.robot.debugIntake.touched}  ${b.state}`);
      }
    }
    console.log(got >= 0 ? `   collected at t=${(got * dt).toFixed(2)} s` : '   NOT collected');
  }
}
