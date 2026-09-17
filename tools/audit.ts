/**
 * Look for real misbehaviour rather than assuming it: balls that gain energy, tunnel, jitter
 * or go non-finite; a robot that drifts when told nothing; shots that scatter more than the
 * configured spread says they should.
 *
 *   npm run tool -- tools/audit.ts
 */
import { readFileSync } from 'node:fs';
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import staging from '../assets/staging.json' with { type: 'json' };
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { loadLandCal } from '../packages/core/src/robot/loadCal.js';
import { M_TO_IN, inches } from '../packages/core/src/units.js';
import type { BallKind, Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

const table = ShotTable.fromCsv(readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8'));
const balls = (staging.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));
const idle = { seq: 0, motors: {}, servos: {} };

export async function main(): Promise<void> {
  await initPhysics();
  const p = structuredClone(params) as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;

  // ---------------------------------------------------- 1. do the balls sit still?
  {
    const w = new World({ params: p, robot: spec, staging: balls, alliance: 'red', seed: 1 });
    for (let f = 0; f < 120; f++) w.step(idle); // settle
    const start = w.balls.balls.map((b) => w.balls.pos(b));
    let maxMove = 0;
    let moving = 0;
    let nonFinite = 0;
    let belowFloor = 0;
    let outside = 0;
    for (let f = 0; f < 600; f++) w.step(idle);
    w.balls.balls.forEach((b, i) => {
      if (!b.body.isEnabled()) return;
      const q = w.balls.pos(b);
      if (!q.every(Number.isFinite)) nonFinite++;
      const d = Math.hypot(q[0] - start[i][0], q[1] - start[i][1], q[2] - start[i][2]) * M_TO_IN;
      maxMove = Math.max(maxMove, d);
      if (d > 1) moving++;
      if (q[1] < -0.05) belowFloor++;
      if (Math.abs(q[0]) > w.geom.halfWidth_m + 0.1 || Math.abs(q[2]) > w.geom.halfWidth_m + 0.1) outside++;
    });
    const speeds = w.balls.balls.filter((b) => b.body.isEnabled()).map((b) => {
      const v = b.body.linvel();
      return Math.hypot(v.x, v.y, v.z);
    });
    console.log('1. settled balls after 10 s of nothing');
    console.log(`   moved >1in: ${moving}   max move ${maxMove.toFixed(1)} in   still moving: ${speeds.filter((s) => s > 0.05).length}`);
    console.log(`   non-finite: ${nonFinite}   below floor: ${belowFloor}   outside field: ${outside}`);
  }

  // ---------------------------------------------------- 2. does the robot hold still?
  {
    const w = new World({ params: p, robot: spec, staging: [], alliance: 'red', seed: 1 });
    w.robot.place([0, 0.17, -1.0], 0);
    for (let f = 0; f < 30; f++) w.step(idle);
    const a = w.robot.pos;
    const yaw0 = w.robot.yaw;
    for (let f = 0; f < 600; f++) w.step(idle);
    const b = w.robot.pos;
    console.log('2. robot told nothing for 10 s');
    console.log(`   drifted ${(Math.hypot(b[0] - a[0], b[2] - a[2]) * M_TO_IN).toFixed(2)} in, yaw ${(((w.robot.yaw - yaw0) * 180) / Math.PI).toFixed(2)} deg`);
  }

  // ---------------------------------------------------- 3. is the shot repeatable?
  {
    const land: number[] = [];
    let inCell = 0;
    for (let trial = 0; trial < 12; trial++) {
      const w = new World({ params: p, robot: spec, staging: Array.from({ length: 2 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 })), alliance: 'red', seed: 100 + trial });
      for (const b of w.balls.balls) w.balls.park(b);
      const mouth = w.hives.red.upCellMouthWorld();
      w.robot.place([mouth[0] + inches(40), 0.17, mouth[2] + inches(40)], 180 - 45);
      w.robot.preload(w.balls, w.balls.balls[0]);
      const brain = new BuiltinTeleOp(spec, table, loadLandCal());
      const step = (g = emptyGamepad()) => { w.setGamepads(g, emptyGamepad()); w.step(brain.update(w.sensors(), g, w.seq)); };
      step();
      const press = emptyGamepad(); press.dpad_up = true; step(press);
      for (let f = 0; f < 420; f++) step();
      // Holding right_bumper toggles the latch on its rising edge; setting state.firing as
      // well would toggle it straight back off.
      const fire = emptyGamepad(); fire.right_bumper = true;
      if (false) {
        const mz = w.robot.muzzle();
        const rp = w.robot.pos;
        const trueBear = (Math.atan2(mouth[0] - rp[0], mouth[2] - rp[2]) * 180) / Math.PI;
        console.log(`   [aim] robotYaw=${((w.robot.yaw * 180) / Math.PI).toFixed(1)} turret=${w.robot.turretAngle.toFixed(1)} -> muzzleAz=${((mz.azimuth * 180) / Math.PI).toFixed(1)}`);
        console.log(`   [aim] true bearing to mouth=${trueBear.toFixed(1)}  game.az=${w.sensors().game.upCellAzimuthDeg.toFixed(1)}`);
        console.log(`   [aim] robot at (${(rp[0] * M_TO_IN).toFixed(0)}, ${(rp[2] * M_TO_IN).toFixed(0)})  mouth at (${(mouth[0] * M_TO_IN).toFixed(0)}, ${(mouth[2] * M_TO_IN).toFixed(0)})`);
      }
      for (let f = 0; f < 200 && w.robot.shots === 0; f++) step(fire);
      const ball = w.balls.balls[0];
      for (let f = 0; f < 300; f++) step(fire);
      const q = w.balls.pos(ball);
      if (false) {
        const sf = w.sensors();
        console.log(`   [trial 0] shots=${w.robot.shots} note="${brain.state.note}" ready=${brain.state.ready}`);
        console.log(`   [trial 0] rpm=${sf.game.flywheelRpm.toFixed(0)} target=${brain.state.targetRpm.toFixed(0)} range=${sf.game.upCellRangeIn.toFixed(0)} az=${sf.game.upCellAzimuthDeg.toFixed(0)} turretErr=${brain.state.turretErrDeg.toFixed(1)} hopper=${sf.game.hopper}`);
      }
      if (w.robot.shots === 0) { land.push(NaN); continue; }
      inCell += ball.state === 'cell' ? 1 : 0;
      land.push(Math.hypot(q[0] - mouth[0], q[1] - mouth[1], q[2] - mouth[2]) * M_TO_IN);
    }
    const ok = land.filter(Number.isFinite);
    const mean = ok.reduce((a, b) => a + b, 0) / Math.max(1, ok.length);
    const sd = Math.sqrt(ok.reduce((a, b) => a + (b - mean) ** 2, 0) / Math.max(1, ok.length));
    console.log('3. twelve identical shots (same pose, same command)');
    console.log(`   fired ${ok.length}/12`);
    console.log(`   ended this far from the CELL mouth: mean ${mean.toFixed(1)} in, sd ${sd.toFixed(1)} in`);
    console.log(`   spread ${Math.min(...ok).toFixed(1)} .. ${Math.max(...ok).toFixed(1)} in`);
    console.log(`   landed in the CELL: ${inCell}/12`);
  }
}
