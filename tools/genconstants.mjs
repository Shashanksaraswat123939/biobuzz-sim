/**
 * Bake config/robot.json and the generated shot table into Java constants.
 *
 *   node tools/genconstants.mjs
 *
 * TeamCode must not parse JSON on the hub: there is no guarantee the file is deployed, and
 * doing file I/O in init is how OpModes get killed by the watchdog. PLAN.md section 14.2
 * offers exactly this route, so both halves still have one origin for every number -- edit
 * robot.json, re-run this, and the Java follows.
 */
import { readFileSync, writeFileSync, mkdirSync, existsSync } from 'node:fs';

const here = (p) => new URL(p, import.meta.url);
const robot = JSON.parse(readFileSync(here('../config/robot.json'), 'utf8'));
const motors = JSON.parse(readFileSync(here('../config/motors.json'), 'utf8'));

const ticksPerRev = (spec) => {
  const v = motors.variants[spec.variant];
  if (!v) throw new Error(`unknown motor variant ${spec.variant}`);
  return v.ticksPerRev * (spec.gearRatio || 1);
};
const freeRpm = (spec) => {
  const v = motors.variants[spec.variant];
  return v.freeRpm / (spec.gearRatio || 1);
};

const fly = robot.flywheel;
const flyTpr = ticksPerRev(fly.motor);
const flyFree = freeRpm(fly.motor);

// FlywheelGate feedforward: power = (kS + kV*rpm)*(12/V) + kP*err.
//
// MEASURED, not derived. "One over the free speed" ignores the flywheel's own quadratic drag
// and coulomb friction, and at 2800 rpm it asks for 0.497 duty where the wheel needs 0.466 --
// about 190 rpm high, which is far enough outside a 60 rpm readiness window that the robot
// never fires. tools/flywheeltune.ts --ff fits them; robot.json carries them.
const kV = fly.kV ?? 1 / flyFree;
const kS = fly.kS ?? 0.03;
const kP = fly.kP ?? 0.00025;

const j = (v) => (typeof v === 'string' ? JSON.stringify(v) : String(v));
const d = (v) => `${Number(v)}`;

const lines = [];
const K = (type, name, value, comment) => {
  if (comment) lines.push(`    /** ${comment} */`);
  lines.push(`    public static final ${type} ${name} = ${value};`);
};

K('String', 'NAME', j(robot.name));
lines.push('');
lines.push('    // ---- hardware names (must match the Robot Controller configuration) ----');
for (const [k, v] of Object.entries(robot.hardware)) {
  if (k === 'battery') continue;
  K('String', `HW_${k.toUpperCase()}`, j(v));
}
lines.push('');
lines.push('    // ---- drivetrain ----');
K('double', 'WHEEL_RADIUS_M', d(robot.drivetrain.wheelRadius_m));
K('double', 'DRIVE_TICKS_PER_REV', d(ticksPerRev(robot.drivetrain.motors.fl)));
for (const w of ['fl', 'fr', 'bl', 'br']) {
  K('boolean', `${w.toUpperCase()}_REVERSED`, String(!!robot.drivetrain.motors[w].reversed));
}
lines.push('');
lines.push('    // ---- mechanisms ----');
K('double', 'CYCLE_TIME_S', d(robot.transfer.cycleTime_s), 'The knob this whole project exists to explore.');
K('double', 'FEED_PULSE_S', d(robot.transfer.feedPulse_s));
K('boolean', 'GATE_ENABLED', String(!!robot.transfer.gate.enabled));
K('double', 'GATE_OPEN', d(robot.transfer.gate.open));
K('double', 'GATE_CLOSED', d(robot.transfer.gate.closed));
K('int', 'HOPPER_CAPACITY', d(robot.hopper.capacity));
lines.push('');
K('double', 'TURRET_MIN_DEG', d(robot.turret.range_deg[0]));
K('double', 'TURRET_MAX_DEG', d(robot.turret.range_deg[1]));
K('double', 'TURRET_SPEED_DPS', d(robot.turret.speed_dps));
K('double', 'TURRET_TICKS_PER_DEG', d(robot.turret.motor.ticksPerDeg ?? ticksPerRev(robot.turret.motor) / 360));
K('double', 'MUZZLE_OFFSET_M', d(robot.turret.muzzleOffset_m),
  'Muzzle distance from the turret axis along the shot line, m. The lever arm of omega x r in ShotLead.muzzleVelocity().');
lines.push('');
K('double', 'HOOD_MIN_DEG', d(robot.hood.angleRange_deg[0]));
K('double', 'HOOD_MAX_DEG', d(robot.hood.angleRange_deg[1]));
K('double', 'HOOD_TOL_DEG', d(robot.hood.tolDeg ?? 2));
K('double', 'VEL_FILTER_ALPHA', d(robot.sensors.localizer.velFilterAlpha ?? 1));
lines.push('');
K('double', 'FLYWHEEL_TICKS_PER_REV', d(flyTpr));
K('double', 'FLYWHEEL_FREE_RPM', d(flyFree));
K('double', 'FLYWHEEL_MAX_RPM', d(fly.maxRpm));
K('double', 'FLYWHEEL_TOL_RPM', d(fly.tolRpm));
K('double', 'FLYWHEEL_MIN_RPM_FRAC', d(fly.minRpmFrac));
K('int', 'FLYWHEEL_READY_STEPS', d(fly.readySteps));
K('double', 'FLYWHEEL_KS', d(kS), 'MEASURED by tools/flywheeltune.ts --ff, not guessed');
K('double', 'FLYWHEEL_KV', d(kV.toPrecision(8)), 'MEASURED duty per RPM at 12 V');
K('double', 'FLYWHEEL_KP', d(kP), 'proportional trim on RPM error');
K('double', 'FLYWHEEL_K', d(fly.k), 'exit efficiency: v_exit = k * omega * r');
K('double', 'FLYWHEEL_R_M', d(fly.r_fly_m));
K('int', 'FLYWHEEL_RPM_FILTER_FRAMES', d(fly.rpmFilterFrames ?? 1),
  'loops of moving average over getVelocity(). One reading is quantised to about 107 RPM on a direct-driven 28-tick encoder');
K('double', 'FLYWHEEL_MIN_LAND_PROB', d(fly.minLandProb ?? 0),
  'refuse the shot below this CALIBRATED chance of landing. 0 disables the probability gate and leaves the RPM window alone');
K('double', 'FLYWHEEL_YAW_SCATTER_DEG', d(fly.scatter.yaw_deg), 'one sigma of launch bearing scatter');

// ---- the tag pipeline ----
// Only the three numbers the DELIVERABLE needs. The lens, the frame rate, the latency and the
// noise are the world's problem (packages/core/src/physics/tagCamera.ts); the robot never
// knows them, it only ever sees how old the last detection is.
const tag = robot.sensors.tag.target;
lines.push('');
lines.push('    // ---- tag target (TagTargetProvider) ----');
K('double', 'TAG_HOLD_S', d(tag.holdS),
  'how long a fix may be carried forward on odometry before the target counts as lost and the turret starts sweeping');
K('double', 'TAG_MAX_FIRE_AGE_S', d(tag.maxFireAgeS),
  'how old a fix may be and still be SHOT on. This is what replaced the oracle hiveTipping flag: a rocker going over blinds the camera, the fix ages past this, and the robot holds fire without ever being told the HIVE moved');
K('double', 'TAG_SCAN_RATE_DPS', d(tag.scanRateDps),
  'turret sweep rate while searching for the tag. No field geometry is baked in, so the robot re-acquires from wherever it actually is');

// ---- tag -> mouth, per rocker state (tools/tagoffsets.ts) ----
//
// THE PANEL IS NOT THE MOUTH. The am-5888 the camera decodes sits about 10 in from the CELL
// mouth on the same rocker, so a robot that aims at the tag aims at the wrong thing. Both are
// rigid on the rocker and the rocker is bistable, so in the field frame the correction has
// exactly two values -- and the TAG ID says which, which is what docs/LOCALIZATION.md means
// by "the tag ID is the rocker state sensor".
//
// ftcX = worldZ, so the offset falls entirely on FTC +X and the FTC +Y term is zero.
const offsets = JSON.parse(readFileSync(here('../config/tagoffsets.json'), 'utf8')).states;
const facing = (st) => (offsets[st].mouthNormalZ >= 0 ? 1 : -1);
lines.push('');
lines.push('    // ---- tag -> CELL mouth, per rocker state (GENERATED by tools/tagoffsets.ts) ----');
K('int', 'TAG_ID_CELL_A', '1', 'the tag on CELL A. Phase 0 of docs/LOCALIZATION.md: the REAL ids are not recorded anywhere and must be looked up');
K('int', 'TAG_ID_CELL_B', '2', 'the tag on CELL B');
K('double', 'TAG_MOUTH_DX_A_IN', d(offsets.A.groundOffset_in),
  'add to the tag field X to get the mouth, with CELL A up. The FTC Y term is zero: both parts sit on the pivot axis');
K('double', 'TAG_MOUTH_DX_B_IN', d(offsets.B.groundOffset_in), 'the same, with CELL B up. Opposite sign, because the rocker is on the other stop');
K('int', 'MOUTH_FACING_X_A', d(facing('A')),
  'which way the up CELL opens along FTC +X with CELL A up. A TIP flips this, which is why a robot that was square onto the goal is behind it afterwards');
K('int', 'MOUTH_FACING_X_B', d(facing('B')), 'the same, with CELL B up');

// ---- where the HIVE is, for when the camera cannot see it ----
//
// The HIVE does not move around the field; only which of its two CELLs is up changes. So a
// robot with odometry and no tag can still aim -- it just cannot tell, on its own, which of
// these two the rocker is currently on. That is the one thing only the tag ID can say.
const ally = (robot.alliance === 'blue' ? 'blue' : 'red');
K('boolean', 'ALLIANCE_IS_RED', String(ally === 'red'), 'set from robot.json -> alliance');
// A PRIOR AND A RIGID OFFSET, not a hardcoded goal. The robot measures the HIVE's pivot for
// itself from the first tag it decodes and aims off THAT; these are only what it starts from
// and the shape of the HIVE about its own pivot, which no amount of tipping changes.
const anchor = JSON.parse(readFileSync(here('../config/tagoffsets.json'), 'utf8')).anchorPrior_in[ally];
K('double', 'HIVE_ANCHOR_X_IN', d(anchor.x), 'surveyed pivot, FTC X. A PRIOR: replaced by the robot own estimate at the first sighting');
K('double', 'HIVE_ANCHOR_Y_IN', d(anchor.y), 'surveyed pivot, FTC Y');
K('double', 'MOUTH_FROM_ANCHOR_X_A_IN', d(offsets.A.mouthFromAnchor_in.x), 'pivot -> up CELL mouth with CELL A up. Rigid: a property of the HIVE, not of the field');
K('double', 'MOUTH_FROM_ANCHOR_X_B_IN', d(offsets.B.mouthFromAnchor_in.x), 'the same with CELL B up -- the far side of the same pivot');
K('double', 'HIVE_ANCHOR_ALPHA', d(tag.anchorAlpha ?? 0.15), 'one-pole on the tracked pivot across sightings');
K('boolean', 'FIRE_ON_ODOMETRY', String(tag.fireOnOdometry !== false),
  'may a shot be taken with no tag in view, on odometry alone? Never before a tag has been seen at least once, because until then the rocker state is an assumption');

// ---- the OTHER HIVE, as an obstacle ----
//
// Two rockers 25.5 in apart, each CELL 20 in wide: a shot from the far side of theirs crosses
// their structure, and the shot solver checks only the target's own lips. Their CELLs swing
// about their pivot at a fixed radius, so the volume is a DISC -- identical whichever stop
// their rocker is on, which is why none of this needs to know the opponent's state.
const obst = JSON.parse(readFileSync(here('../config/tagoffsets.json'), 'utf8')).obstacle_in;
K('double', 'OBSTACLE_X_IN', d(obst[ally].x), 'the opposing HIVE pivot, FTC X');
// The pocket's depth: the term that closes the CELL's opening as you go off its normal.
// LandProbability needs it to score an oblique shot honestly (tools/obliquity.ts).
K('double', 'CELL_DEPTH_M', d(12.04 * 0.0254), 'CELL pocket depth, metres (CAD cellDepth_in)');
K('double', 'OBSTACLE_Y_IN', d(obst[ally].y), 'the opposing HIVE pivot, FTC Y');
K('double', 'OBSTACLE_RADIUS_IN', d(obst.radius_in), 'how far their CELL mouths reach from that pivot');
K('double', 'OBSTACLE_HALF_WIDTH_IN', d(obst.halfWidth_in), 'half a CELL width, along the pivot axis');

// ---- tag fixes correcting the dead-reckoned pose (FusedLocalizer) ----
const fuse = robot.sensors.localizer.fuse ?? { gain: 0.15, headingGain: 0.05, rejectOver_in: 36 };
K('double', 'FUSE_GAIN', d(fuse.gain), 'fraction of each fix position disagreement absorbed. A gain, not a jump: a fix carries 4% of range');
K('double', 'FUSE_HEADING_GAIN', d(fuse.headingGain ?? 0.05), 'the same off the tag yaw, which is the worst axis of an AprilTag pose by an order');
K('double', 'FUSE_REJECT_OVER_IN', d(fuse.rejectOver_in), 'refuse a fix that disagrees by more than this: a misread, not drift');

const constants = `package org.firstinspires.ftc.teamcode.config;

/**
 * GENERATED by tools/genconstants.mjs from config/robot.json -- do not edit by hand.
 * Re-run after changing the robot description:  node tools/genconstants.mjs
 */
public final class RobotConstants {
    private RobotConstants() { }

${lines.join('\n')}
}
`;

const outDir = here('../java/teamcode/src/org/firstinspires/ftc/teamcode/config/');
mkdirSync(outDir, { recursive: true });
writeFileSync(new URL('RobotConstants.java', outDir), constants);

// ---- shot table ----
const csvPath = here('../java/teamcode/assets/shottable.csv');
let rows = [];
if (existsSync(csvPath)) {
  rows = readFileSync(csvPath, 'utf8')
    .trim()
    .split(/\r?\n/)
    .slice(1)
    .map((l) => l.split(',').map(Number))
    .filter((r) => Number.isFinite(r[0]));
}
const col = (i) => rows.map((r) => (Number.isFinite(r[i]) ? r[i] : 0)).join(', ');
const calPath = here('../config/landcal.json');
const calJson = existsSync(calPath) ? JSON.parse(readFileSync(calPath, 'utf8')) : { points: [] };
const cal = calJson.points ?? [];
const calCeiling = cal.length ? Math.max(...cal.map((c) => c.observed)).toFixed(6) : '0';
const table = `package org.firstinspires.ftc.teamcode.config;

/**
 * GENERATED by tools/genconstants.mjs from java/teamcode/assets/shottable.csv,
 * which tools/shottable.ts produces for the current robot. Do not edit by hand.
 *
 * ${rows.length} ranges. The solver runs on a laptop; the hub only interpolates.
 */
public final class ShotTableData {
    private ShotTableData() { }

    public static final double[] RANGE_IN = { ${col(0)} };
    public static final double[] HOOD_POS = { ${col(1)} };
    public static final double[] RPM      = { ${col(2)} };
    public static final double[] MARGIN   = { ${col(3)} };

    // What the land-probability gate needs. The hub still solves nothing: these are the
    // solver's own outputs per range, and LandProbability turns them into a number.
    public static final double[] SPEED_LO    = { ${col(5)} };
    public static final double[] SPEED_HI    = { ${col(6)} };
    public static final double[] SIGMA_SPEED = { ${col(7)} };
    public static final double[] P_STAY      = { ${col(8)} };
    /** Half-width of the mouth across the shot line, m, less the ball radius. */
    public static final double[] HALF_LAT_M  = { ${col(9)} };

    // MEASURED score -> frequency mapping from config/landcal.json (tools/landcal.ts).
    // Without it the threshold is a score with a percent sign on it.
    public static final double[] CAL_SCORE    = { ${cal.map((c) => c.score.toFixed(6)).join(', ')} };
    public static final double[] CAL_OBSERVED = { ${cal.map((c) => c.observed.toFixed(6)).join(', ')} };
    /** The best frequency any bin of shots actually achieved. No threshold above this can be met. */
    public static final double CAL_CEILING = ${calCeiling};
}
`;
writeFileSync(new URL('ShotTableData.java', outDir), table);

console.log(`RobotConstants.java: ${lines.filter((l) => l.includes('static final')).length} constants`);
console.log(`ShotTableData.java: ${rows.length} shot table rows`);
