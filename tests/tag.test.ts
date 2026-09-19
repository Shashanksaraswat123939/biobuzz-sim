import { describe, it, expect, beforeAll } from 'vitest';
import { readFileSync } from 'node:fs';
import params from '../config/params.json';
import robotSpec from '../config/robot.json';
import offsets from '../config/tagoffsets.json';
import { World, initPhysics, emptyGamepad } from '../packages/core/src/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '../packages/core/src/robot/builtinTeleOp.js';
import { TagCamera } from '../packages/core/src/physics/tagCamera.js';
import { TagTarget } from '../packages/core/src/robot/tagTarget.js';
import { PoseFuser } from '../packages/core/src/robot/poseFuser.js';
import { shotIsBlocked } from '../packages/core/src/robot/clearShot.js';
import { Rng } from '../packages/core/src/io/rng.js';
import { inches } from '../packages/core/src/units.js';
import { buildFieldGeometry, cellMouthCentre } from '../packages/core/src/field/geometry.js';
import type { TagTruth } from '../packages/core/src/physics/tagCamera.js';
import type { Params, RobotSpec, Vec3 } from '../packages/core/src/types.js';

/**
 * The tests for the thing that replaced the oracle.
 *
 * `game.upCellAzimuthDeg` and friends used to be ground truth with no noise, no latency, no
 * field of view and no way to be invalid, and both brains aimed on them. Everything here
 * exists because taking that away has to be verifiable: that the camera really does go
 * blind, that going blind really does hold fire, and that the robot really does get itself
 * a fix back without being handed one.
 */

const csv = readFileSync(new URL('../java/teamcode/assets/shottable.csv', import.meta.url), 'utf8');
const table = ShotTable.fromCsv(csv);
const spec = robotSpec as unknown as RobotSpec;

beforeAll(async () => {
  await initPhysics();
});

const truth = (o: Partial<TagTruth> = {}): TagTruth => ({
  id: 1,
  bearingDeg: 0,
  rangeIn: 50,
  openDeg: 10,
  turretDeg: 0,
  tipping: false,
  ...o,
});

/** Run the camera for `seconds` against a fixed truth, at the world's own step rate. */
function soak(cam: TagCamera, seconds: number, t0: number, tr: TagTruth | ((t: number) => TagTruth)): number {
  const dt = 1 / 240;
  let t = t0;
  for (let i = 0; i < Math.round(seconds * 240); i++) {
    t += dt;
    cam.step(t, typeof tr === 'function' ? tr(t) : tr);
  }
  return t;
}

describe('the tag camera is a camera, not an oracle', () => {
  const mk = (over: Partial<RobotSpec['sensors']['tag']> = {}) =>
    new TagCamera({ ...spec.sensors.tag, ...over }, new Rng(11));

  it('sees the tag when the turret is pointed at it', () => {
    const cam = mk();
    soak(cam, 0.5, 0, truth());
    expect(cam.read()).not.toBeNull();
    expect(cam.hits).toBeGreaterThan(5);
  });

  it('sees nothing when the tag is outside the lens', () => {
    // The camera rides the TURRET, so this is the bearing against the turret's angle and not
    // against the chassis. A turret looking 90 deg away from a target dead ahead is blind.
    const cam = mk();
    soak(cam, 1.0, 0, truth({ bearingDeg: 0, turretDeg: 90 }));
    expect(cam.read()).toBeNull();
    expect(cam.frames).toBeGreaterThan(20); // it tried, it just could not decode
  });

  it('sees nothing past its range or too far round the side of the mouth', () => {
    expect(soakRead(mk(), truth({ rangeIn: spec.sensors.tag.maxRange_in + 10 }))).toBeNull();
    expect(soakRead(mk(), truth({ openDeg: spec.sensors.tag.maxIncidence_deg + 5 }))).toBeNull();
  });

  it('SEES NOTHING WHILE THE ROCKER IS SWINGING, which is the whole point', () => {
    // The oracle handed over `hiveTipping` as a boolean the instant it became true. A camera
    // is told nothing; it just stops decoding a tag that is moving. This is the fact the
    // robot now has to work from, and it is strictly less information than before.
    const cam = mk();
    soak(cam, 1.0, 0, truth({ tipping: true }));
    expect(cam.read()).toBeNull();
  });

  it('reports a detection that is already latencyMs old when it arrives', () => {
    const cam = mk();
    const t = soak(cam, 1.0, 0, (now: number) => truth({ rangeIn: 50 + now }));
    const s = cam.read()!;
    const age = t - s.sampleT;
    const lag = spec.sensors.tag.latencyMs / 1000;
    const framePeriod = 1 / spec.sensors.tag.frameRateHz;
    // Never fresher than the pipeline lag, never older than lag plus one frame.
    expect(age).toBeGreaterThanOrEqual(lag - 1e-6);
    expect(age).toBeLessThanOrEqual(lag + framePeriod + 1e-3);
  });

  it('runs at the camera frame rate, not the world rate', () => {
    const cam = mk();
    soak(cam, 2.0, 0, truth());
    expect(cam.frames).toBeGreaterThan(spec.sensors.tag.frameRateHz * 2 * 0.8);
    expect(cam.frames).toBeLessThan(spec.sensors.tag.frameRateHz * 2 * 1.2);
  });

  it('puts more error on range than on bearing, and most on the mouth angle', () => {
    // Not a style preference: it is the shape of an AprilTag pose. A run that flattened these
    // into one sigma would make the "is the mouth open" gate look better than it is.
    const cam = mk();
    const bear: number[] = [];
    const rng: number[] = [];
    let t = 0;
    for (let i = 0; i < 400; i++) {
      t = soak(cam, 1 / spec.sensors.tag.frameRateHz, t, truth());
      const s = cam.read()!;
      bear.push(Math.abs(s.bearingDeg - 0));
      rng.push(Math.abs(s.rangeIn - 50) / 50);
    }
    const mean = (a: number[]) => a.reduce((x, y) => x + y, 0) / a.length;
    expect(mean(bear)).toBeLessThan(spec.sensors.tag.noise.bearing_deg * 2);
    expect(mean(rng)).toBeGreaterThan(0);
    expect(spec.sensors.tag.noise.open_deg).toBeGreaterThan(spec.sensors.tag.noise.bearing_deg);
  });

  function soakRead(cam: TagCamera, tr: TagTruth) {
    soak(cam, 1.0, 0, tr);
    return cam.read();
  }
});

describe('the target fix: aim on it, hold on it, or go and find it', () => {
  const tg = spec.sensors.tag.target;
  const mk = () =>
    new TagTarget({
      mouthDx: { 1: offsets.states.A.groundOffset_in, 2: offsets.states.B.groundOffset_in },
      mouthFacingX: { 1: offsets.states.A.mouthNormalZ >= 0 ? 1 : -1, 2: offsets.states.B.mouthNormalZ >= 0 ? 1 : -1 },
      anchorPrior: offsets.anchorPrior_in.red,
      mouthFromAnchor: { 1: offsets.states.A.mouthFromAnchor_in, 2: offsets.states.B.mouthFromAnchor_in },
      anchorAlpha: 0.15,
      fireOnOdometry: spec.sensors.tag.target.fireOnOdometry !== false,
      maxStateAgeS: spec.sensors.tag.target.maxStateAgeS ?? Infinity,
      measAlpha: 1,
      startId: 1,
      holdS: tg.holdS,
      maxFireAgeS: tg.maxFireAgeS,
      scanRateDps: tg.scanRateDps,
      turretMinDeg: spec.turret.range_deg[0],
      turretMaxDeg: spec.turret.range_deg[1],
    });
  const at = { x: 0, y: 0, heading: 0 };
  /** Tag -> mouth along FTC +X, by tag id. The thing the provider has to add on. */
  const dxA = offsets.states.A.groundOffset_in;

  /** The same, but refusing to shoot on anything but a fresh detection. */
  const mkStrict = () =>
    new TagTarget({
      mouthDx: { 1: offsets.states.A.groundOffset_in, 2: offsets.states.B.groundOffset_in },
      mouthFacingX: { 1: offsets.states.A.mouthNormalZ >= 0 ? 1 : -1, 2: offsets.states.B.mouthNormalZ >= 0 ? 1 : -1 },
      anchorPrior: offsets.anchorPrior_in.red,
      mouthFromAnchor: { 1: offsets.states.A.mouthFromAnchor_in, 2: offsets.states.B.mouthFromAnchor_in },
      anchorAlpha: 0.15,
      fireOnOdometry: false,
      maxStateAgeS: Infinity,
      measAlpha: 1,
      startId: 1,
      holdS: tg.holdS,
      maxFireAgeS: tg.maxFireAgeS,
      scanRateDps: tg.scanRateDps,
      turretMinDeg: spec.turret.range_deg[0],
      turretMaxDeg: spec.turret.range_deg[1],
    });

  /** A robot with no localizer: nothing to dead-reckon with, so the sweep is all it has. */
  const mkBlind = () =>
    new TagTarget({
      mouthDx: { 1: offsets.states.A.groundOffset_in, 2: offsets.states.B.groundOffset_in },
      mouthFacingX: { 1: 1, 2: -1 },
      anchorPrior: offsets.anchorPrior_in.red,
      mouthFromAnchor: {},
      anchorAlpha: 0.15,
      fireOnOdometry: false,
      maxStateAgeS: Infinity,
      measAlpha: 1,
      startId: 1,
      holdS: tg.holdS,
      maxFireAgeS: tg.maxFireAgeS,
      scanRateDps: tg.scanRateDps,
      turretMinDeg: spec.turret.range_deg[0],
      turretMaxDeg: spec.turret.range_deg[1],
    });

  it('sweeps the turret when it has no pose and no tag, and stays inside its travel', () => {
    const t = mkBlind();
    let st = t.update(0, at, null, 1 / 60);
    expect(st.scanning).toBe(true);
    expect(st.valid).toBe(false);
    const seen: number[] = [];
    for (let i = 1; i < 60 * 8; i++) {
      st = t.update(i / 60, at, null, 1 / 60);
      seen.push(st.azimuthDeg);
    }
    expect(Math.min(...seen)).toBeGreaterThanOrEqual(spec.turret.range_deg[0]);
    expect(Math.max(...seen)).toBeLessThanOrEqual(spec.turret.range_deg[1]);
    // It actually covers ground rather than sitting still, or it would never find anything.
    expect(Math.max(...seen) - Math.min(...seen)).toBeGreaterThan(90);
  });

  it('never reports a shootable target before it has seen one, pose or no pose', () => {
    // THE INVARIANT, and it survived the odometry fallback: until a tag has been decoded the
    // rocker state is an assumption, so no shot may be taken however good the pose is. The
    // robot may AIM on odometry -- pointing at the wrong CELL costs nothing.
    const withPose = mk().update(0, at, null, 1 / 60);
    expect(withPose.fresh).toBe(false);
    expect(withPose.stateFromTag).toBe(false);
    expect(withPose.fromOdometry).toBe(true);
    expect(withPose.valid).toBe(true); // there IS something to point at

    const blind = mkBlind().update(0, at, null, 1 / 60);
    expect(blind.fresh).toBe(false);
    expect(blind.valid).toBe(false);
    expect(blind.openDeg).toBe(180); // unknown, and every mouth-open gate refuses above 75
  });

  it('falls back to odometry when the tag goes away, and aims at the baked HIVE', () => {
    const t = mk();
    // See a tag once, so the rocker state is observed rather than assumed.
    t.update(1.0, at, { id: 1, bearingDeg: 0, rangeIn: 50, openDeg: 8, sampleT: 1.0 }, 1 / 60);
    // Then lose it for longer than the carry allows.
    const st = t.update(1.0 + tg.holdS + 0.5, at, null, 1 / 60);
    expect(st.fromOdometry).toBe(true);
    expect(st.valid).toBe(true);
    expect(st.scanning).toBe(false);
    expect(st.stateFromTag).toBe(true);
    // It may fire on it, because the state is known and the policy allows it.
    expect(st.fresh).toBe(spec.sensors.tag.target.fireOnOdometry !== false);
    // AND IT AIMS AT THE HIVE IT MEASURED, not at the surveyed prior. The prior is 15.68 in
    // from the origin; the sighting put the mouth at 52.78, and that is what the fallback
    // keeps using -- the robot tracked the landmark rather than forgetting it.
    const measuredMouthX = 50 + offsets.states.A.groundOffset_in;
    expect(st.rangeIn).toBeCloseTo(measuredMouthX, 6);
    expect(st.anchorFromTag).toBe(true);
    // The anchor IS the pivot: mouth minus the rigid pivot->mouth offset.
    expect(st.anchor.x).toBeCloseTo(measuredMouthX - offsets.states.A.mouthFromAnchor_in.x, 6);
    expect(st.anchor.x).not.toBeCloseTo(offsets.anchorPrior_in.red.x, 3);
  });

  it('knows where the OTHER CELL is the moment the rocker goes over', () => {
    // This is what tracking the PIVOT buys that tracking a mouth does not. A TIP swings both
    // CELLs about the pivot, so one sighting of CELL A locates CELL B too -- without needing
    // to see its tag first, and without falling back on the surveyed prior.
    const t = mk();
    t.update(1.0, at, { id: 1, bearingDeg: 0, rangeIn: 50, openDeg: 8, sampleT: 1.0 }, 1 / 60);
    const anchorX = 50 + offsets.states.A.groundOffset_in - offsets.states.A.mouthFromAnchor_in.x;

    // Where CELL B's tag MUST be if the pivot is where the first sighting said it was:
    // pivot -> B's mouth, then back off the rigid tag -> mouth correction.
    const bTagRange = anchorX + offsets.states.B.mouthFromAnchor_in.x - offsets.states.B.groundOffset_in;
    const st = t.update(2.0, at, { id: 2, bearingDeg: 0, rangeIn: bTagRange, openDeg: 8, sampleT: 2.0 }, 1 / 60);
    expect(st.id).toBe(2);
    // The anchor does not budge: it is the same physical pivot, seen through the other CELL.
    expect(st.anchor.x).toBeCloseTo(anchorX, 6);
    // And the aim swung to the far side of it, which is what a TIP does.
    expect(st.rangeIn).toBeCloseTo(Math.abs(anchorX + offsets.states.B.mouthFromAnchor_in.x), 6);
  });

  it('latches on a detection, then ages through HELD into LOST', () => {
    const t = mk();
    const sight = { id: 1, bearingDeg: 20, rangeIn: 50, openDeg: 8, sampleT: 1.0 };
    let st = t.update(1.05, at, sight, 1 / 60);
    expect(st.fresh).toBe(true);
    // The fix is the MOUTH, not the tag: the panel is dxA inches from it along FTC +X, so
    // the reported range and bearing are the corrected ones, never the raw detection.
    const mx = 50 * Math.cos((20 * Math.PI) / 180) + dxA;
    const my = 50 * Math.sin((20 * Math.PI) / 180);
    expect(st.rangeIn).toBeCloseTo(Math.hypot(mx, my), 6);
    expect(st.azimuthDeg).toBeCloseTo((Math.atan2(my, mx) * 180) / Math.PI, 6);

    // A stale fix is still aimed with, and `lostLock` says the camera has stopped confirming
    // it -- the window a TIP lives in. Whether it may be FIRED on is now a policy question
    // (`fireOnOdometry`), not a property of the age, because past the carry the aim is coming
    // off surveyed geometry rather than off nothing at all.
    st = t.update(1.0 + tg.maxFireAgeS + 0.01, at, null, 1 / 60);
    expect(st.valid).toBe(true);
    expect(st.lostLock).toBe(true);

    // With the policy OFF, a stale fix is unshootable, which is the conservative contract.
    const strict = mkStrict();
    strict.update(1.0, at, sight, 1 / 60);
    const held = strict.update(1.0 + tg.maxFireAgeS + 0.01, at, null, 1 / 60);
    expect(held.valid).toBe(true);
    expect(held.fresh).toBe(false);

    // Past the carry it does NOT go blind any more: it hands over to odometry, which still
    // knows where the HIVE is. Blind is only for a robot with no pose at all.
    st = t.update(1.0 + tg.holdS + 0.01, at, null, 1 / 60);
    expect(st.valid).toBe(true);
    expect(st.fromOdometry).toBe(true);
    expect(mkBlind().update(99, at, null, 1 / 60).scanning).toBe(true);
  });

  it('carries the fix on odometry: the same field point, from a robot that has moved', () => {
    const t = mk();
    // Standing at the origin facing +X, a tag 50 in dead ahead puts the MOUTH at 50 + dxA.
    t.update(1.0, at, { id: 1, bearingDeg: 0, rangeIn: 50, openDeg: 8, sampleT: 1.0 }, 1 / 60);
    // Drive 10 in along +X and 0 in across, still facing +X, with no new detection.
    const st = t.update(1.1, { x: 10, y: 0, heading: 0 }, null, 1 / 60);
    expect(st.rangeIn).toBeCloseTo(40 + dxA, 6);
    expect(st.azimuthDeg).toBeCloseTo(0, 6);
    // And turning in place moves the bearing, not the range.
    const st2 = t.update(1.15, { x: 10, y: 0, heading: 30 }, null, 1 / 60);
    expect(st2.rangeIn).toBeCloseTo(40 + dxA, 6);
    expect(st2.azimuthDeg).toBeCloseTo(-30, 6);
  });

  it('takes the newest detection whole rather than blending two CELLs', () => {
    // After a TIP the up CELL is the other one, 14 in away on the far side of the pivot, and
    // it carries a different tag. An average of the two aims at the rocker between them.
    const t = mk();
    t.update(1.0, at, { id: 1, bearingDeg: 0, rangeIn: 50, openDeg: 8, sampleT: 1.0 }, 1 / 60);
    const st = t.update(2.0, at, { id: 2, bearingDeg: 40, rangeIn: 60, openDeg: 20, sampleT: 2.0 }, 1 / 60);
    expect(st.id).toBe(2);
    // CELL B's offset, not an average of the two -- and B's is the opposite sign to A's,
    // because the rocker is on the other stop, so a blend would land between the CELLs.
    const dxB = offsets.states.B.groundOffset_in;
    const mx = 60 * Math.cos((40 * Math.PI) / 180) + dxB;
    const my = 60 * Math.sin((40 * Math.PI) / 180);
    expect(st.rangeIn).toBeCloseTo(Math.hypot(mx, my), 6);
    expect(st.azimuthDeg).toBeCloseTo((Math.atan2(my, mx) * 180) / Math.PI, 6);
    expect(Math.sign(dxB)).toBe(-Math.sign(dxA));
  });
});

describe('odometry drifts, and the tag pulls it back', () => {
  const mkFuser = () => new PoseFuser({ gain: 0.15, headingGain: 0.05, rejectOver_in: 36 });

  it('integrates DELTAS, so a correction is not undone by the next reading', () => {
    // The whole reason this is an estimator and not an offset. Feed it odometry that is
    // bodily wrong by 10 in and then stops moving: the fix must stick, not be overwritten.
    const f = mkFuser();
    f.propagate(0, { x: 10, y: 0, heading: 0 });
    f.propagate(0.1, { x: 11, y: 0, heading: 0 });
    expect(f.pose().x).toBeCloseTo(11, 6);
    // A tag at x = 50 whose face points back down -X, seen square on (yaw 0) and dead ahead
    // (bearing 0) by a robot that is really at the origin. facingX = -1, because a panel
    // facing +X could not be seen square on from -X at all.
    f.observe(50, 0, 0, 50, -1, 0, 0.1);
    const afterFix = f.pose().x;
    expect(afterFix).toBeLessThan(11);
    // Odometry carries on reporting its own drifted numbers; the correction must survive.
    f.propagate(0.2, { x: 12, y: 0, heading: 0 });
    expect(f.pose().x).toBeCloseTo(afterFix + 1, 6);
  });

  it('corrects heading from the tag yaw, which is what makes the pose observable', () => {
    // Bearing and range are two constraints for three unknowns. Without the yaw the position
    // fix is measured along a line drawn at a heading nobody checked, and correcting under a
    // wrong heading moves the estimate AWAY from the truth -- measured, 0.8 in became 7.2.
    const f = mkFuser();
    // The robot really sits at (0, 0) facing +X. Its gyro has drifted and thinks 8 deg.
    f.propagate(0, { x: 0, y: 0, heading: 8 });
    // A tag at (50, 0), face pointing back down -X, seen square on. The camera measures
    // bearing against the TRUE heading, so a robot really pointing at it reports 0.
    const before = Math.abs(f.pose().heading);
    for (let i = 0; i < 60; i++) f.observe(50, 0, 0, 50, -1, 0, 0);
    // Pulled toward the truth, and most of the way there after enough fixes.
    expect(Math.abs(f.pose().heading)).toBeLessThan(before);
    expect(Math.abs(f.pose().heading)).toBeLessThan(1);
  });

  it('throws out a fix that disagrees by more than the gate', () => {
    const f = mkFuser();
    f.propagate(0, { x: 0, y: 0, heading: 0 });
    f.observe(500, 0, 0, 50, -1, 0, 0);   // implies the robot is 450 in away
    expect(f.rejected).toBe(1);
    expect(f.applied).toBe(0);
    expect(f.pose().x).toBeCloseTo(0, 6);
  });

  it('the world dead-reckons rather than reporting the truth', () => {
    // If this ever reads back as the truth again, every drift number in the project is a
    // fiction and the tag has nothing to correct.
    const d = spec.sensors.localizer.drift;
    expect(d).toBeDefined();
    expect(d?.enabled).not.toBe(false);
    expect(d!.gyroBias_dps).toBeGreaterThan(0);
    expect(d!.scaleErr).toBeGreaterThan(0);
    expect(spec.sensors.localizer.source).toBe('deadReckoning');
  });
});

describe('the other HIVE is a solid object', () => {
  const ob = {
    x: offsets.obstacle_in.red.x,
    y: offsets.obstacle_in.red.y,
    radius_in: offsets.obstacle_in.radius_in,
    halfWidth_in: offsets.obstacle_in.halfWidth_in,
  };

  it('refuses a shot whose path crosses the opposing rocker', () => {
    // From the far side of their hive, across it, to ours. The physics gives both rockers
    // colliders, so this ball really does hit something; the solver checks only our own lips.
    expect(shotIsBlocked(40, ob.y, -30, ob.y, ob)).toBe(true);
  });

  it('allows a shot that passes clear of it', () => {
    // Well downrange of their structure: same X travel, but 40 in along the pivot axis away.
    expect(shotIsBlocked(40, ob.y + 60, -30, ob.y + 60, ob)).toBe(false);
  });

  it('does not care which way the opponent rocker is lying', () => {
    // The reason this is four numbers and not a state machine: both of their CELLs swing
    // about their pivot at a fixed radius, so the swept volume is a DISC and a disc is the
    // same in both states. The footprint is symmetric about the pivot, which is that fact.
    expect(ob.x - ob.radius_in).toBeCloseTo(-(ob.x + ob.radius_in), 6);
    // And it reaches as far as the MOUTH does, not just as far as the arm.
    expect(ob.radius_in).toBeGreaterThan(20);
  });

  it('keeps the Java copy on the same four numbers', () => {
    const java = readFileSync(
      new URL('../java/teamcode/src/org/firstinspires/ftc/teamcode/config/RobotConstants.java', import.meta.url),
      'utf8',
    );
    const k = (name: string): number => {
      const m = java.match(new RegExp(`${name}\\s*=\\s*([-\\d.eE]+)`));
      if (!m) throw new Error(`${name} missing`);
      return Number(m[1]);
    };
    expect(k('OBSTACLE_X_IN')).toBeCloseTo(ob.x, 6);
    expect(k('OBSTACLE_Y_IN')).toBeCloseTo(ob.y, 6);
    expect(k('OBSTACLE_RADIUS_IN')).toBeCloseTo(ob.radius_in, 6);
    expect(k('OBSTACLE_HALF_WIDTH_IN')).toBeCloseTo(ob.halfWidth_in, 6);
  });
});

describe('the configuration has to be self-consistent or the robot is blind', () => {
  const tag = spec.sensors.tag;

  it('cannot sweep past a tag without ever seeing it', () => {
    // A sweep that advances further than the lens is wide between camera frames steps the tag
    // clean over. This is the one way the search can fail silently and never recover.
    const stepPerFrame = tag.target.scanRateDps / tag.frameRateHz;
    expect(stepPerFrame).toBeLessThan(tag.fov_deg);
  });

  it('does not set a fire-age gate the pipeline can never satisfy', () => {
    // A detection is already lag + up to one frame old when it arrives. Demand fresher than
    // that and the robot refuses every shot the camera is capable of supplying.
    const floor = tag.latencyMs / 1000 + 1 / tag.frameRateHz;
    expect(tag.target.maxFireAgeS).toBeGreaterThan(floor);
    expect(tag.target.holdS).toBeGreaterThanOrEqual(tag.target.maxFireAgeS);
  });

  it('models the tag where the CAD puts it, which is NOT on the mouth', () => {
    // The am-5888 panel is ~10 in from the CELL mouth on the same rocker (tools/tagoffsets.ts).
    // The first camera reported the MOUTH's bearing and range, i.e. a measurement of a thing
    // no camera can see, and silently deleted the offset the hub has to apply.
    const geom = buildFieldGeometry(params as unknown as Params);
    for (const cell of geom.cells) {
      const mouth = cellMouthCentre(cell);
      const tag = cell.tagBody_m;
      const apart = Math.hypot(mouth[1] - tag[1], mouth[2] - tag[2]) * 39.3700787;
      expect(apart).toBeGreaterThan(5);
    }
    // And the two states' corrections are equal and opposite, because the rocker is bistable.
    expect(offsets.states.A.groundOffset_in).toBeCloseTo(-offsets.states.B.groundOffset_in, 2);
    // The mouth faces opposite ways in the two states. This is the TIP, in one number.
    expect(Math.sign(offsets.states.A.mouthNormalZ)).toBe(-Math.sign(offsets.states.B.mouthNormalZ));
  });

  it('narrows the mouth off-axis, so the gate refuses what the zone map refuses', () => {
    // The zone map's whole point: the mouth is a slot, and from the side it is a narrower
    // one. If the robot scores its chances against the full head-on width from everywhere,
    // it will take shots the painted zone says are dead -- which is exactly the complaint.
    const src = readFileSync(
      new URL('../java/teamcode/src/org/firstinspires/ftc/teamcode/control/LandProbability.java', import.meta.url),
      'utf8',
    );
    // THE COSINE, AND NOT THE POCKET'S DEPTH. This test briefly demanded the opposite --
    // width*cos(beta) - depth*sin(beta) on both sides -- and locked in a bug: that formula is
    // the clear straight line THROUGH a slot, which is what a ball would need if it had to
    // reach the back wall untouched, and a ball only has to cross the mouth and stay in.
    // It cost 20 of the shot zone's 52 green squares, and tools/lostzone.ts fired 160 balls
    // from those squares with the gate forced open: 156 in, 98%, against 89% from the ones
    // it kept. So the depth term must NOT come back, and this is the side that says so.
    expect(src).toMatch(/HALF_LAT_M[\s\S]{0,120}Math\.cos\(beta\)/);
    expect(src).not.toMatch(/CELL_DEPTH_M/);
    const mirror = readFileSync(new URL('../packages/core/src/robot/builtinTeleOp.ts', import.meta.url), 'utf8');
    expect(mirror).toMatch(/row\.halfLat_m \* cosB/);
    expect(mirror).not.toMatch(/cellDepth_m/);

    // And the arithmetic it stands on: square on is full width, 60 deg round is half of it,
    // past 90 there is nothing left to aim at.
    const w = (deg: number) => Math.max(0, Math.cos((deg * Math.PI) / 180));
    expect(w(0)).toBeCloseTo(1, 9);
    expect(w(60)).toBeCloseTo(0.5, 9);
    expect(w(120)).toBe(0);
  });

  it('keeps the tag map on the same grid as the shot zone', () => {
    // The overlay looks a square up by its "x,z" key. A grid that differs by half a step, or
    // by a margin, matches NOTHING and paints the entire field as blind -- and it does it
    // silently, because a miss and a genuine blind spot look identical on screen.
    const zone = JSON.parse(readFileSync(new URL('../config/shotzone.json', import.meta.url), 'utf8'));
    const map = JSON.parse(readFileSync(new URL('../config/tagmap.json', import.meta.url), 'utf8'));
    const key = (c: { x_in: number; z_in: number }) => `${c.x_in},${c.z_in}`;
    const zk = new Set((zone.cells as { x_in: number; z_in: number }[]).map(key));
    const mk2 = new Set((map.cells as { x_in: number; z_in: number }[]).map(key));
    expect(mk2.size).toBe(zk.size);
    expect([...zk].every((k) => mk2.has(k))).toBe(true);
    // And it has to say something: an all-visible or all-blind map is a broken one.
    const vis = (map.cells as { visible: boolean }[]).filter((c) => c.visible).length;
    expect(vis).toBeGreaterThan(0);
    expect(vis).toBeLessThan(map.cells.length);
  });

  it('keeps the Java constants in step with the config they are generated from', () => {
    // RobotConstants.java is generated, and a stale generated file is how the flywheel encoder
    // ended up at 28 ticks a rev. These three are what the deliverable actually gates on.
    const java = readFileSync(
      new URL('../java/teamcode/src/org/firstinspires/ftc/teamcode/config/RobotConstants.java', import.meta.url),
      'utf8',
    );
    const k = (name: string): number => {
      const m = java.match(new RegExp(`${name}\\s*=\\s*([-\\d.eE]+)`));
      if (!m) throw new Error(`${name} missing from RobotConstants.java`);
      return Number(m[1]);
    };
    expect(k('TAG_HOLD_S')).toBeCloseTo(tag.target.holdS, 9);
    expect(k('TAG_MAX_FIRE_AGE_S')).toBeCloseTo(tag.target.maxFireAgeS, 9);
    expect(k('TAG_SCAN_RATE_DPS')).toBeCloseTo(tag.target.scanRateDps, 9);
    // And the generated tag -> mouth offsets, which are the ones a stale file would make
    // wrong by 2.8 in in a direction that flips with the rocker.
    expect(k('TAG_MOUTH_DX_A_IN')).toBeCloseTo(offsets.states.A.groundOffset_in, 9);
    expect(k('TAG_MOUTH_DX_B_IN')).toBeCloseTo(offsets.states.B.groundOffset_in, 9);
    expect(k('MOUTH_FACING_X_A')).toBe(offsets.states.A.mouthNormalZ >= 0 ? 1 : -1);
    expect(k('MOUTH_FACING_X_B')).toBe(offsets.states.B.mouthNormalZ >= 0 ? 1 : -1);
  });
});

describe('the robot in the world, with no oracle left to read', () => {
  /** Stand the robot square to the up CELL at a range the table has an answer for. */
  function rig(range_in: number) {
    const p = structuredClone(params) as unknown as Params;
    const s = structuredClone(robotSpec) as unknown as RobotSpec;
    s.flywheel.minLandProb = 0;
    const staging = Array.from({ length: 10 }, () => ({ kind: 'pollen' as const, pos: [0, -5, 0] as Vec3 }));
    const world = new World({ params: p, robot: s, staging, alliance: 'red', seed: 5 });
    for (const b of world.balls.balls) world.balls.park(b);
    const mouth = world.hives.red.upCellMouthWorld();
    const d = inches(range_in);
    const spot: Vec3 = [mouth[0], s.chassis.height_m / 2 + s.chassis.clearance_m, mouth[2] + d];
    const yaw = (Math.atan2(mouth[0] - spot[0], mouth[2] - spot[2]) * 180) / Math.PI;
    world.robot.place(spot, yaw);
    for (let i = 0; i < 4; i++) world.robot.preload(world.balls, world.balls.balls[i]);
    return { world, brain: new BuiltinTeleOp(s, table) };
  }

  /** `a` toggles the flywheel on (an edge), then the bumper latch asks for shots. */
  function sticks(frame: number) {
    const g = emptyGamepad();
    if (frame < 3) g.a = true;
    else g.right_bumper = true;
    return g;
  }

  it('acquires the tag by itself and then still scores', () => {
    // THE WHOLE FIX, END TO END. Nothing hands this robot a bearing: it sweeps, finds the
    // tag, latches, aims on a fix that is 75-108 ms old, and puts balls in the CELL anyway.
    const { world, brain } = rig(45);
    let sawFix = false;
    for (let f = 0; f < 60 * 22; f++) {
      const g = sticks(f);
      world.setGamepads(g, emptyGamepad());
      world.step(brain.update(world.sensors(), g, world.seq));
      const t = brain.target();
      if (t.fresh && !t.fromOdometry) sawFix = true;
    }
    // It finds the tag for itself. It no longer has to SWEEP to do it, because odometry
    // points the turret -- and so the camera -- at where the HIVE must be, which is the whole
    // reason the fallback earns its place: it is also the fastest way to reacquire.
    expect(sawFix).toBe(true);
    expect(world.robot.shots).toBeGreaterThan(0);
    expect(world.hives.red.ballsInUpCell + world.hives.red.tips * 4).toBeGreaterThan(0);
  }, 60000);

  it('holds fire while the HIVE is going over, without being told it is', () => {
    // The oracle gate was `!s.game.hiveTipping`, handed over free. Now the rocker swinging
    // simply blinds the camera, the fix ages out, and the shot is refused for staleness. No
    // shot may be taken on a fix older than the shooter can use -- in AUTO or anywhere else.
    const { world, brain } = rig(45);
    let firedStale = 0;
    let shots = world.robot.shots;
    for (let f = 0; f < 60 * 25; f++) {
      const g = sticks(f);
      world.setGamepads(g, emptyGamepad());
      world.step(brain.update(world.sensors(), g, world.seq));
      if (world.robot.shots > shots) {
        if (!brain.target().fresh) firedStale++;
        shots = world.robot.shots;
      }
      // Keep it fed so it keeps trying for the whole run.
      while (world.robot.heldBalls().length < 4) {
        const free = world.balls.balls.find((b) => b.state === 'parked');
        if (!free || !world.robot.preload(world.balls, free)) break;
      }
    }
    expect(world.robot.shots).toBeGreaterThan(0);
    expect(firedStale).toBe(0);
  }, 60000);

  it('never lets the brain read the truth block', () => {
    // The cheapest guard in the repo and the one most worth having: the four fields that were
    // the oracle now live behind `game.truth`, so a grep is a proof. If a brain ever reads one
    // again, this fails rather than the results quietly getting better.
    for (const f of ['builtinTeleOp.ts', 'autoRoutine.ts', 'tagTarget.ts', 'autoDriver.ts']) {
      const src = readFileSync(new URL(`../packages/core/src/robot/${f}`, import.meta.url), 'utf8');
      const reads = src.split('\n').filter((l) => /\.truth\./.test(l) && !/^\s*(\*|\/\/)/.test(l));
      expect(reads, `${f} reads ground truth: ${reads.join(' | ')}`).toEqual([]);
    }
  });
});
