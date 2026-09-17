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
