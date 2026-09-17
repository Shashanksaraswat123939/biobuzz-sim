/**
 * Can this robot know where it is? An AprilTag feasibility study.
 *
 *   npm run tool -- tools/tagstudy.ts
 *
 * THE PROBLEM, stated precisely. Every AprilTag on this field is bolted to a ROCKER, and
 * the rocker moves. There are four of them (cad/parts.json, am-5888), two per hive, one on
 * the underside of each CELL. There is not a single static fiducial anywhere on the field:
 * no wall tags, no perimeter tags, nothing bolted to the A-frame. So the usual FTC answer —
 * "look at a tag, get your pose" — starts from a premise this game does not supply.
 *
 * WHAT SAVES IT is that the rocker is BISTABLE. params.json gives restAngles_deg [-30, +30]
 * and the thing sits on a hard stop at one end or the other; it is only in between during
 * the second or so of an actual tip. So a tag does not have a continuum of possible poses.
 * It has exactly TWO. And because each CELL carries its own tag with its own ID, the ID you
 * are looking at tells you which of the two the rocker is in.
 *
 * This works out what that is actually worth: where on the field a tag can be seen, how
 * accurate the resulting fix is, whether that accuracy clears what the SHOT needs, and how
 * long dead reckoning has to carry the robot between fixes.
 *
 * WHAT IS ASSUMED, because the repository does not record it. All three are flagged in the
 * output and none of them is a physics guess -- they are facts about hardware that someone
 * has to look up:
 *   - the printed tag's size (the am-5888 PANEL is 17 x 4.34 in; the tag on it is smaller)
 *   - the tag IDs
 *   - the camera
 */
import params from '../config/params.json' with { type: 'json' };
import robotSpec from '../config/robot.json' with { type: 'json' };
import cadParts from '../cad/parts.json' with { type: 'json' };
import { buildFieldGeometry } from '../packages/core/src/field/geometry.js';
import { M_TO_IN, inches, DEG, RAD } from '../packages/core/src/units.js';
import type { Params, RobotSpec } from '../packages/core/src/types.js';

interface CadPart { name: string; path: string; centroid_in: number[]; bbox_in: number[] }

/** ASSUMPTION 1: the printed tag inside the am-5888 panel. */
const TAG_IN = 4.0;
const TAG_SOURCE =
  'ASSUMED. The am-5888 panel measures 17.00 x 4.34 in in the CAD, so the printed tag fits ' +
  'inside 4.34 in and 4 in is the common FTC size. NOT confirmed against the manual.';

/** ASSUMPTION 2: a Logitech C920-class webcam at the resolution FTC usually runs tags at. */
const CAM = { widthPx: 640, hfov_deg: 60, heightAboveFloor_in: 11, pitchUp_deg: 15 };
const CAM_SOURCE =
  'ASSUMED. 640x480 is what the FTC SDK AprilTag pipeline usually runs at and 60 deg is a ' +
  'typical horizontal FOV for a C920-class webcam at 4:3. Measure the real one.';

/**
 * Corner localisation noise, in pixels, 1 sigma. This is the number every accuracy figure
 * below rests on, so it is worth being explicit: a well-lit, in-focus, square-on tag gives
 * roughly a quarter pixel; a small, blurred or oblique one is far worse. 0.5 px is a
 * deliberately unflattering middle.
 */
const CORNER_PX = 0.5;

export interface TagPose {
  name: string;
  alliance: 'red' | 'blue';
  cell: 'audience' | 'scoring';
  /** World position, metres, in the two rocker rest states. */
  world: { stateA: number[]; stateB: number[] };
  /** Radius from the pivot axis and angle in the rocker body frame. */
  radius_in: number;
  bodyAngle_deg: number;
}

/** Pull the four am-5888 panels out of the CAD and express them in the rocker body frame. */
export function tagPoses(p: Params): TagPose[] {
  const parts = cadParts as unknown as CadPart[];
  const tags = parts.filter((q) => q.name.toLowerCase().includes('april'));
  const pivotY = p.hive.pivotY_m * M_TO_IN;
  const rest = 30;

  return tags.map((t) => {
    const alliance: 'red' | 'blue' = t.name.toLowerCase().includes('red') ? 'red' : 'blue';
    const cell: 'audience' | 'scoring' = t.name.toLowerCase().includes('audience') ? 'audience' : 'scoring';
    const hiveX = (alliance === 'red' ? p.hive.redX_m : p.hive.blueX_m) * M_TO_IN;

    // The CAD is saved with the AUDIENCE cell up on red and down on blue: the two rockers
    // are mirrored, so they sit on opposite stops.
    //
    // SIGN. Hive.toWorld maps a body point through
    //     worldY = y*cos(t) - z*sin(t),  worldZ = y*sin(t) + z*cos(t)
    // which in (Z, Y) polar is worldAngle = bodyAngle - t, NOT + t. Getting that backwards
    // is not a cosmetic error: it rotates both tags the same way, so a "flip" moved the
    // audience tag from 49.7 in UP to 58.0 in and the scoring tag further down, instead of
    // swapping them. A rocker that does not swap its CELLs is not a rocker. The check that
    // catches it is below -- the two heights must exchange.
    const cadAngle = alliance === 'red' ? -rest : rest;
    const dy = t.centroid_in[1] - pivotY;
    const dz = t.centroid_in[2];
    const radius = Math.hypot(dy, dz);
    const worldAngle = Math.atan2(dy, dz) * RAD;
    const bodyAngle = worldAngle + cadAngle;

    const at = (theta_deg: number) => {
      const a = (bodyAngle - theta_deg) * DEG;
      return [inches(hiveX), inches(pivotY + radius * Math.sin(a)), inches(radius * Math.cos(a))];
    };
    return {
      name: t.name,
      alliance,
      cell,
      world: { stateA: at(-rest), stateB: at(rest) },
      radius_in: radius,
      bodyAngle_deg: bodyAngle,
    };
  });
}

/**
 * Bearing and range accuracy of a single tag fix, from corner pixel noise.
 *
 * A tag of side s at range d subtends s/d radians, which is (s/d) * (widthPx/hfov) pixels.
 * Bearing error is corner noise divided by the focal length in pixels. RANGE error is the
 * one that bites: range comes from the apparent SIZE, so it goes as d^2 -- double the
 * distance and the range error quadruples.
 */
export function tagAccuracy(range_in: number, tag_in = TAG_IN): { bearing_deg: number; range_in: number; px: number } {
  const fpx = CAM.widthPx / (2 * Math.tan((CAM.hfov_deg / 2) * DEG));
  const px = (tag_in / range_in) * fpx;                 // apparent tag size in pixels
  const bearing = (CORNER_PX / fpx) * RAD;              // rad -> deg
  // d = s*f/px, so dd/dpx = -s*f/px^2 = -d/px; with CORNER_PX/sqrt(2) effective on the size
  const dRange = (range_in / px) * CORNER_PX * Math.SQRT2;
  return { bearing_deg: bearing, range_in: dRange, px };
}

export async function main(): Promise<void> {
  const p = params as unknown as Params;
  const spec = robotSpec as unknown as RobotSpec;
  const geom = buildFieldGeometry(p);
  const poses = tagPoses(p);

  console.log('APRILTAG FEASIBILITY — can this robot know where it is?');
  console.log('');
  console.log('ASSUMPTIONS THAT ARE NOT PHYSICS — someone has to look these up');
  console.log(`  tag size ${TAG_IN} in: ${TAG_SOURCE}`);
  console.log(`  camera:   ${CAM_SOURCE}`);
  console.log('  tag IDs:  NOT RECORDED ANYWHERE IN THIS REPOSITORY. The whole scheme below turns on');
  console.log('            each CELL carrying a DISTINCT id; confirm that before building on it.');
  console.log('');
  console.log('THE FIELD GIVES YOU FOUR TAGS AND THEY ALL MOVE');
  console.log(`  ${poses.length} am-5888 panels in cad/parts.json, two per hive, one per CELL.`);
  console.log('  Static fiducials anywhere on the field: NONE. No wall tags, no A-frame tags.');
  console.log('');
  console.log('  tag                              radius   body angle   height in state A / B');
  for (const t of poses) {
    console.log(
      `  ${(t.alliance + ' ' + t.cell).padEnd(16)} ${t.radius_in.toFixed(2).padStart(6)} in  ` +
      `${t.bodyAngle_deg.toFixed(1).padStart(7)} deg   ` +
      `${(t.world.stateA[1] * M_TO_IN).toFixed(1).padStart(5)} / ${(t.world.stateB[1] * M_TO_IN).toFixed(1)} in`,
    );
  }
  console.log('');
  console.log('  The rocker is BISTABLE: params.json restAngles_deg = ' +
    `[${p.hive.restAngles_deg[0]}, ${p.hive.restAngles_deg[1]}], hard stops at either end. So each tag has`);
  console.log('  exactly TWO possible poses, not a continuum — and each CELL carries its own tag ID,');
  console.log('  so the ID you can see tells you which of the two the rocker is in. The pose of a tag');
  console.log('  you have identified is therefore KNOWN EXACTLY, except during a tip.');
  // A rocker that does not swap its CELLs is not a rocker. Assert it rather than trust it.
  for (const a of ['red', 'blue'] as const) {
    const pair = poses.filter((t) => t.alliance === a);
    const aUp = pair.map((t) => t.world.stateA[1] * M_TO_IN).sort((x, y) => x - y);
    const bUp = pair.map((t) => t.world.stateB[1] * M_TO_IN).sort((x, y) => x - y);
    const swapped = Math.abs(aUp[0] - bUp[0]) < 0.05 && Math.abs(aUp[1] - bUp[1]) < 0.05;
    const hi = pair.find((t) => t.world.stateA[1] > t.world.stateB[1])!;
    console.log(`  ${a}: state A puts the ${hi.cell} CELL up, state B the other one. ` +
      `heights swap: ${swapped ? 'yes' : 'NO — the transform is wrong'}`);
    if (!swapped) throw new Error('rocker flip does not swap the CELL heights; check the rotation sign');
  }
  console.log('');

  // ---- how far apart are the two states, i.e. how bad is it to guess wrong?
  for (const t of poses.filter((x) => x.alliance === 'red')) {
    const d = Math.hypot(
      (t.world.stateA[1] - t.world.stateB[1]) * M_TO_IN,
      (t.world.stateA[2] - t.world.stateB[2]) * M_TO_IN,
    );
    console.log(`  If you mistake the state, the ${t.cell} tag is ${d.toFixed(1)} in from where you think.`);
  }
  console.log('  That is the cost of getting it wrong, and it is why the ID matters more than the pose.');
  console.log('');

  // ---- accuracy vs range, against what the shot actually needs
  console.log('WHAT A FIX IS WORTH, AGAINST WHAT THE SHOT NEEDS');
  console.log(`  tag ${TAG_IN} in, camera ${CAM.widthPx} px across ${CAM.hfov_deg} deg, corner noise ${CORNER_PX} px 1-sigma`);
  console.log('');
  const mouthHalf = 10;
  console.log('  range   tag px   bearing err   range err    mouth half-angle   verdict');
  for (const r of [24, 36, 48, 54, 66, 78, 90, 110, 130]) {
    const a = tagAccuracy(r);
    const need = Math.atan2(mouthHalf, r) * RAD;
    // A bearing error puts the ball off by range*tan(err); a range error moves it downrange.
    const lateral = r * Math.tan(a.bearing_deg * DEG);
    const ok = a.px >= 12 && lateral < mouthHalf * 0.25 && a.range_in < mouthHalf * 0.5;
    console.log(
      `  ${String(r).padStart(4)} in ${a.px.toFixed(0).padStart(7)} ${a.bearing_deg.toFixed(2).padStart(11)} deg ` +
      `${a.range_in.toFixed(2).padStart(9)} in ${need.toFixed(1).padStart(15)} deg   ` +
      `${a.px < 12 ? 'TOO SMALL TO DETECT' : ok ? 'good' : 'usable, degrading'}`,
    );
  }
  console.log('');
  console.log('  Bearing is near-constant with range because corner noise is a fixed pixel error on a');
  console.log('  fixed focal length. RANGE is the one that falls apart: it comes from apparent SIZE, so');
  console.log('  its error grows as the SQUARE of distance. A tag is a good compass and a poor rangefinder.');
  console.log('');

  // ---- where on the field can you see one at all
  const halfW = geom.halfWidth_m * M_TO_IN;
  const redX = p.hive.redX_m * M_TO_IN;
  const up = poses.find((t) => t.alliance === 'red' && t.cell === 'audience')!;
  const tagPos = [redX, up.world.stateA[1] * M_TO_IN, up.world.stateA[2] * M_TO_IN];
  let seen = 0;
  let total = 0;
  const step = 4;
  for (let x = -halfW + 9; x <= halfW - 9; x += step) {
    for (let z = -halfW + 9; z <= halfW - 9; z += step) {
      total++;
      const d = Math.hypot(x - tagPos[0], z - tagPos[2]);
      if (tagAccuracy(d).px >= 12) seen++;
    }
  }
  console.log('COVERAGE');
  const fpx = CAM.widthPx / (2 * Math.tan((CAM.hfov_deg / 2) * DEG));
  console.log('  Range is set by how few pixels the decoder will accept, and that is not a sharp');
  console.log('  number — it depends on lighting, focus and motion blur as much as on the detector:');
  for (const floorPx of [10, 16, 24, 32]) {
    console.log(`    at a ${String(floorPx).padStart(2)} px floor, a ${TAG_IN} in tag is readable out to ${((TAG_IN * fpx) / floorPx).toFixed(0).padStart(3)} in`);
  }
  console.log('');
  console.log('  NOT MODELLED, and it will cut these numbers: INCIDENCE. The panel is on the pocket');
  console.log('  underside facing out of the mouth, so a robot far away sees it at a grazing angle,');
  console.log('  and a tag past about 60-70 deg off its normal stops decoding. cad/parts.json gives');
  console.log('  the panel a bounding box but not an orientation, so the normal cannot be recovered');
  console.log('  from it here — measure it off the STEP before trusting any long-range figure.');
  console.log('');
  console.log(`  On range alone that covers ${((seen / total) * 100).toFixed(0)}% of the field — IF the camera happens to be pointing`);
  console.log('  at it. A 60 deg FOV on a chassis that is free to face anywhere covers 60/360 = 17% of');
  console.log('  headings, so the honest figure for "a tag is in view right now" is far lower than that.');
  console.log('');
  console.log('  A TURRET-MOUNTED camera changes this completely: the turret already tracks the CELL,');
  console.log(`  so a camera bolted to it is pointing at the tag whenever the shooter is aimed, by`);
  console.log('  construction. That is the single highest-value hardware decision in this document.');
  console.log('');

  // ---- dead reckoning budget
  //
  // Odometry error is proportional to DISTANCE TRAVELLED, not to elapsed time. Quoting it
  // as "x inches per 30 seconds" flatters a robot that sits still and slanders one that
  // works hard, and it made dead wheels look like they solved the whole problem: 429 s to
  // 10 in of error is longer than a match, which is only true if the robot barely moves.
  //
  // So: estimate the distance actually covered, then apply a per-distance error rate.
  const cruise_in_s = 45;          // a fair average for a robot that also stops to shoot
  const dutyCycle = 0.55;          // fraction of the period actually driving
  const autoDist = cruise_in_s * dutyCycle * 30;
  const teleDist = cruise_in_s * dutyCycle * 120;

  console.log('THE DEAD-RECKONING BUDGET');
  console.log(`  A robot cruising ${cruise_in_s} in/s at a ${(dutyCycle * 100).toFixed(0)}% duty cycle covers ` +
    `${(autoDist / 12).toFixed(0)} ft in AUTO and ${(teleDist / 12).toFixed(0)} ft in TELEOP.`);
  console.log('  Odometry error scales with that distance, not with the clock.');
  console.log('');
  console.log('  method                                  error rate    after AUTO   after TELEOP');
  const drift = [
    { what: 'drive encoders on omni wheels', frac: 0.05, note: 'free rollers slip sideways BY DESIGN and the encoder cannot see it' },
    { what: 'two unpowered dead-wheel pods', frac: 0.01, note: 'the standard FTC answer' },
    { what: 'fused odo pod + IMU (Pinpoint etc.)', frac: 0.005, note: 'same pods, heading from a gyro instead of from the wheels' },
  ];
  for (const d of drift) {
    console.log(`  ${d.what.padEnd(38)} ${(d.frac * 100).toFixed(1).padStart(5)}% of dist  ` +
      `${(autoDist * d.frac).toFixed(1).padStart(8)} in ${(teleDist * d.frac).toFixed(1).padStart(12)} in`);
  }
  for (const d of drift) console.log(`    ${d.what}: ${d.note}`);
  console.log('');
  console.log('  Rates are ORDER-OF-MAGNITUDE from common FTC experience, not measured here. The');
  console.log('  conclusion does not hinge on the exact values: good pods carry a 30 s autonomous on');
  console.log('  their own and NOTHING carries a full match on its own.');
  console.log('');

  // ---- heading is the one that actually hurts a turret robot
  console.log('AND HEADING MATTERS MORE THAN POSITION HERE');
  const mouthHalfIn = mouthHalf;
  console.log('  A turret aimed from a global pose inherits the heading error directly. An FTC IMU');
  console.log('  drifts on the order of 1-3 deg per minute uncorrected, so over a 2:30 match:');
  console.log('');
  console.log('  yaw drift   lateral miss at 54 in   fraction of the mouth half-width');
  for (const deg of [1, 2, 3, 5, 7.5]) {
    const lat = 54 * Math.tan(deg * DEG);
    console.log(`  ${deg.toFixed(1).padStart(6)} deg ${lat.toFixed(1).padStart(16)} in ${((lat / mouthHalfIn) * 100).toFixed(0).padStart(24)}%`);
  }
  console.log('');
  console.log('  Half the mouth is gone by about 5 deg of yaw drift. That is squarely inside what an');
  console.log('  uncorrected IMU does over a match — and it is the strongest argument in this whole');
  console.log('  document for aiming off the TAG rather than off a global pose, because a relative');
  console.log('  bearing to the tag has no heading term in it at all.');
  console.log('');

  console.log('THE ANSWER');
  console.log('  No — "exact position at all times" is not available, and no FTC sensor suite gives it.');
  console.log('  But the robot does not need it, because the thing it is actually for is aiming, and');
  console.log('  aiming does not want a global position at all. It wants RANGE AND BEARING TO THE UP');
  console.log('  CELL MOUTH, and there is a tag bolted to that mouth. Measured directly, relative,');
  console.log('  with no global frame to drift and no accumulated error to correct.');
  console.log('');
  console.log(`  The mouth subtends ${(Math.atan2(mouthHalf, 54) * RAD).toFixed(1)} deg at 54 in, and a tag fix is good to ` +
    `${tagAccuracy(54).bearing_deg.toFixed(2)} deg of bearing there —`);
  console.log(`  ${(Math.atan2(mouthHalf, 54) * RAD / tagAccuracy(54).bearing_deg).toFixed(0)}x the margin it needs. The turret's own travel is ` +
    `${spec.turret.range_deg[0]} to ${spec.turret.range_deg[1]} deg, so it can hold the`);
  console.log('  CELL from any heading the chassis happens to be at.');
  console.log('');
  console.log('  Global pose is still needed — for driving to collection spots, for LEAVE, for PARK —');
  console.log('  but those want inches of accuracy, not tenths of a degree, and odometry supplies them.');
}
