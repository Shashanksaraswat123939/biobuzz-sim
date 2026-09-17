/**
 * App shell: the loop, the input, and the panels.
 *
 * All physics lives in @core and all drawing in @render; this file is glue and layout.
 * Three modes share one world:
 *
 *   practice — you drive. Nothing is scripted.
 *   data     — AutoDriver drives, sweeping a range band and logging every shot, then
 *              @core/analysis/report says what went wrong.
 *   test     — sandbox. Every constant is a slider and @core/analysis/sensitivity shows
 *              what each one is doing to the shot before you take it.
 *
 * Every control on the page is listed in the Guide tab with what it does. If a button is
 * here it is wired to something; there is nothing that only looks like a control.
 */
import paramsJson from '../../../config/params.json';
import robotJson from '../../../config/robot.json';
import stagingJson from '../../../assets/staging.json';
import shotCsv from '../../../java/teamcode/assets/shottable.csv?raw';
import shotZoneJson from '../../../config/shotzone.json';

import { World, initPhysics, emptyGamepad } from '@core/physics/world.js';
import { BuiltinTeleOp, ShotTable } from '@core/robot/builtinTeleOp.js';
import { loadLandCal } from '@core/robot/loadCal.js';
import { AutoDriver, defaultPlan, type AutoPlan } from '@core/robot/autoDriver.js';
import { simulateShot, rpmToSpeed } from '@core/physics/ballistics.js';
import { knobs, predict, type Prediction } from '@core/analysis/sensitivity.js';
import { analyse, toCsv, type Report } from '@core/analysis/report.js';
import { Trace } from '@core/analysis/trace.js';
import { M_TO_IN, DEG, inches } from '@core/units.js';
import { Scene, type CameraMode, type ZonePayload } from '@render/scene.js';
import type { ActuatorFrame, Alliance, BallKind, GamepadState, Params, RobotSpec, Snapshot, Vec3 } from '@core/types.js';
import { readKeyboard, installKeyboard, type Keys } from './input.js';
import { m, cm, cmSigned, mps } from './units.js';
import { Joysticks } from './joystick.js';
import { buildTunePanel, type Tunable } from './tune.js';
import { WorldBridge } from './bridge.js';
import { CONTROLS, renderGuide } from './guide.js';
import { groupPlot, errorVsRange, histogram } from './charts.js';

const $ = <T extends HTMLElement = HTMLElement>(sel: string) => document.querySelector(sel) as T;
const $$ = <T extends HTMLElement = HTMLElement>(sel: string) => Array.from(document.querySelectorAll<T>(sel));

type Mode = 'practice' | 'collect' | 'test';

const baseParams = paramsJson as unknown as Params;
const baseRobot = robotJson as unknown as RobotSpec;
const staging = (stagingJson.balls as { kind: string; pos: number[] }[]).map((b) => ({ kind: b.kind as BallKind, pos: b.pos as Vec3 }));
const shotTable = ShotTable.fromCsv(shotCsv);

let params: Params = structuredClone(baseParams);
let robotSpec: RobotSpec = structuredClone(baseRobot);
let world: World;
let scene: Scene;
/** On-screen sticks, folded into the same gamepad frame as the keyboard and a real pad. */
let sticks: Joysticks;
let brain: BuiltinTeleOp;
let auto: AutoDriver | null = null;
let plan: AutoPlan = defaultPlan();
let alliance: Alliance = 'red';
let mode: Mode = 'practice';
let paused = false;
let autoLoad = false;
let useJavaBrain = false;
/**
 * Sim time per wall-clock second. A 40-shot sweep is several minutes at 1x, and nobody
 * needs to watch it; the physics is unchanged, it just gets more steps per frame.
 */
let turbo = 1;
let lastAct: ActuatorFrame = { seq: 0, motors: {}, servos: {} };
const keys: Keys = installKeyboard();
const bridge = new WorldBridge();
/**
 * Flight recorder. It survives a rebuild on purpose: comparing the run before a variable
 * changed with the run after is the whole point, and clearing it on Reset would throw the
 * "before" away every time.
 */
const trace = new Trace();

// ---------------------------------------------------------------- setup

async function boot(): Promise<void> {
  await initPhysics();
  build();
  buildTunePanel($('#tune-rows'), params, robotSpec, onTune);
  renderGuide($('#guide'));
  sticks = new Joysticks($('#app'));
  wireUi();
  setMode('practice');
  requestAnimationFrame(loop);
}

function build(): void {
  world = new World({ params, robot: robotSpec, staging, alliance, seed: params.sim.seed, preload: 4 });
  brain = new BuiltinTeleOp(robotSpec, shotTable, loadLandCal());
  auto = null;
  const specs = world.balls.balls.map((b) => ({ id: b.id, r: b.radius, kind: b.kind as string }));
  const mode0 = scene?.cameraMode ?? 'orbit';
  const traj = scene?.showTrajectory ?? true;
  const zone0 = scene?.showShotZone ?? false;
  scene = new Scene($<HTMLCanvasElement>('#view'), world.geom, specs);
  scene.cameraMode = mode0;
  scene.showTrajectory = traj;
  // Off until asked for: it is a model map, and a coloured floor that is always on reads
  // like a measurement of where the robot scores, which it is not.
  scene.setShotZone(shotZoneJson as unknown as ZonePayload);
  scene.showShotZone = zone0;
  scene.resize();
  prediction = null;
  // Handy from the console: sim.world.hives.red.angle, sim.world.robot.flywheelRpm, ...
  (globalThis as unknown as { sim: unknown }).sim = { world, scene, params, robotSpec, brain };
}

/**
 * A knob moved. Cheap ones are already written into `params`/`robotSpec` by the slider and
 * the world reads those objects live, so there is nothing more to do; expensive ones are
 * baked into rigid bodies at construction and need the rebuild the button offers.
 */
function onTune(t: Tunable): void {
  prediction = null;
  $<HTMLButtonElement>('#apply-reset').classList.toggle('on', !!t.rebuild);
}

// ---------------------------------------------------------------- input

function gamepadState(): GamepadState {
  const g = navigator.getGamepads?.().find((p) => p);
  const k = readKeyboard(keys);
  if (!g) return sticks.merge(k);
  const dz = (v: number) => (Math.abs(v) < 0.09 ? 0 : v);
  const merged: GamepadState = {
    left_stick_x: dz(g.axes[0] ?? 0) || k.left_stick_x,
    left_stick_y: dz(g.axes[1] ?? 0) || k.left_stick_y,
    right_stick_x: dz(g.axes[2] ?? 0) || k.right_stick_x,
    right_stick_y: dz(g.axes[3] ?? 0) || k.right_stick_y,
    left_trigger: Math.max(g.buttons[6]?.value ?? 0, k.left_trigger),
    right_trigger: Math.max(g.buttons[7]?.value ?? 0, k.right_trigger),
    a: (g.buttons[0]?.pressed ?? false) || k.a,
    b: (g.buttons[1]?.pressed ?? false) || k.b,
    x: (g.buttons[2]?.pressed ?? false) || k.x,
    y: (g.buttons[3]?.pressed ?? false) || k.y,
    left_bumper: (g.buttons[4]?.pressed ?? false) || k.left_bumper,
    right_bumper: (g.buttons[5]?.pressed ?? false) || k.right_bumper,
    dpad_up: (g.buttons[12]?.pressed ?? false) || k.dpad_up,
    dpad_down: (g.buttons[13]?.pressed ?? false) || k.dpad_down,
    dpad_left: (g.buttons[14]?.pressed ?? false) || k.dpad_left,
    dpad_right: (g.buttons[15]?.pressed ?? false) || k.dpad_right,
    start: (g.buttons[9]?.pressed ?? false) || k.start,
    back: (g.buttons[8]?.pressed ?? false) || k.back,
    left_stick_button: g.buttons[10]?.pressed ?? false,
    right_stick_button: g.buttons[11]?.pressed ?? false,
  };
  return sticks.merge(merged);
}

/** Same sticks, edge-triggered buttons released: a toggle fires once per animation frame. */
const neutralEdges = (g: GamepadState): GamepadState => ({ ...g, a: false, x: false, y: false, right_bumper: false });

/** The gamepad's own mode and utility buttons, read once per animation frame. */
let padPrev: GamepadState | null = null;
function padShortcuts(g: GamepadState): void {
  const p = padPrev;
  padPrev = { ...g };
  if (!p) return;
  const edge = (a: boolean, b: boolean) => a && !b;
  if (edge(g.y, p.y)) setAutoLoad(!autoLoad);                       // Y: auto-fill hopper
  if (edge(g.start, p.start)) primary();                            // Start: the green button
  if (edge(g.back, p.back)) cycleMode();                            // Back: next mode
  if (edge(g.dpad_up, p.dpad_up)) cycleCamera(1);                   // D-pad up/down: camera
  if (edge(g.dpad_down, p.dpad_down)) cycleCamera(-1);
  if (edge(g.left_stick_button, p.left_stick_button)) togglePause(); // L3: pause
}

// ---------------------------------------------------------------- loop

let frameMs = 0;
let drawTick = 0;
let acc = 0;
let last = performance.now();

function loop(now: number): void {
  requestAnimationFrame(loop);
  const dtReal = Math.min(0.1, (now - last) / 1000);
  last = now;

  const human = gamepadState();
  padShortcuts(human);

  const t0 = performance.now();
  if (!paused) {
    const frame = params.sim.dt * params.sim.substepsPerFrame;
    // The step size never changes with turbo -- only how many of them fit in one animation
    // frame -- so a fast run and a slow run produce the same trajectory.
    acc = Math.min(acc + dtReal * turbo, frame * 4 * turbo);
    let first = true;
    while (acc >= frame) {
      const sensors = world.sensors();
      // In data mode the AutoDriver replaces the human's hands, not the brain: it produces
      // a gamepad, and the same TeleOp code path flies the shot.
      const g = auto && auto.phase !== 'done'
        ? auto.update(sensors, frame, world.robot.shots)
        : first ? human : neutralEdges(human);
      world.setGamepads(g, emptyGamepad());
      lastAct = useJavaBrain ? bridge.exchange(sensors) : brain.update(sensors, g, world.seq, frame);
      world.telemetry = lastAct.telemetry ?? [];
      world.step(lastAct);
      acc -= frame;
      if (first) keys.pressed.clear();
      first = false;
      if (autoLoad && world.seq % 20 === 0) topUpHopper();
    }
  }
  frameMs = frameMs * 0.9 + (performance.now() - t0) * 0.1;

  // While fast-forwarding, draw one frame in four. The 675k-triangle CAD field is a real
  // cost, and during a collection run the physics is what you are here for.
  const snap = world.snapshot();
  if (!paused) trace.sample(snap, alliance);
  drawTick++;
  if (turbo === 1 || drawTick % 4 === 0) {
    scene.update(snap, world.aimPoint(), predictShot(snap));
    scene.render();
    paint(snap);
  }
}

let trajCache: Vec3[] | null = null;
let trajAge = 0;

/**
 * The arc the ball would fly if it were fired right now.
 * Recomputed every few frames at a coarse step: at 1/240 over a two-second lob this is a
 * ~500-step integration, and doing it every frame starved the physics loop badly enough
 * that the sim ran at well under real time.
 */
function predictShot(s: Snapshot): Vec3[] | null {
  if (!scene.showTrajectory || s.robot.flywheel.rpm < 200) return null;
  if (trajAge-- > 0 && trajCache) return trajCache;
  trajAge = 4;
  const mz = world.robot.muzzle();
  const speed = rpmToSpeed(s.robot.flywheel.rpm, robotSpec.flywheel.k, robotSpec.flywheel.r_fly_m);
  const traj = simulateShot(params, {
    from: mz.pos, azimuth: mz.azimuth, elevation: mz.elevation, speed,
    radius: params.ball.pollen.d_m / 2,
    mass: params.ball.pollen.m_kg,
    spin: robotSpec.flywheel.type === 'single' ? speed / (params.ball.pollen.d_m / 2) : 0,
  }, 8, 1 / 120);
  trajCache = traj.points;
  return trajCache;
}

// ---------------------------------------------------------------- paint

const row = (label: string, value: string, cls = '') => `<span>${label}</span><b class="${cls}">${value}</b>`;

function paint(s: Snapshot): void {
  const r = s.robot;
  const mins = Math.floor(s.remaining / 60);
  const secs = Math.floor(s.remaining % 60);
  $('#period').textContent = s.period;
  $('#time').textContent = `${mins}:${String(secs).padStart(2, '0')}`;
  $('#red-total').textContent = String(s.score.red.total);
  $('#blue-total').textContent = String(s.score.blue.total);

  const rangeIn = Math.hypot(world.aimPoint()[0] - r.p[0], world.aimPoint()[2] - r.p[2]) * M_TO_IN;
  set('#st-note', auto && auto.phase !== 'done' ? auto.note : brain.state.note, brain.state.ready ? 'on' : 'off');
  set('#st-range', m(rangeIn));
  set('#st-turret', `${r.turret.angleDeg.toFixed(0)}°`, r.turret.atLimit ? 'off' : '');
  set('#st-rpm', `${r.flywheel.rpm.toFixed(0)}`, brain.state.ready ? 'on' : '');
  set('#st-hopper', `${r.hopper.count}/${r.hopper.capacity}`, r.hopper.count ? '' : 'off');
  set('#st-batt', `${r.battery.volts.toFixed(1)} V`, r.battery.volts < 11.5 ? 'off' : '');

  paintDeck();

  const tab = $('.tabs button.on').dataset.tab;
  if (tab === 'robot') paintRobot(s, rangeIn);
  else if (tab === 'predict') paintPredict(rangeIn);
  else if (tab === 'analysis') paintAnalysis(s);

  $('#perf').textContent = `${frameMs.toFixed(1)} ms`;
}

function set(sel: string, text: string, cls = ''): void {
  const el = $(sel);
  el.textContent = text;
  el.className = cls;
}

function paintRobot(s: Snapshot, rangeIn: number): void {
  const r = s.robot;
  $('#r-pose').innerHTML = [
    row('field x, y', `${(r.ftc.x * 0.0254).toFixed(2)}, ${(r.ftc.y * 0.0254).toFixed(2)} m`),
    row('heading', `${r.ftc.heading.toFixed(1)}°`),
    row('speed', mps(r.speed)),
    row('range to CELL', m(rangeIn)),
    row('bearing to CELL', `${world.sensors().game.upCellAzimuthDeg.toFixed(1)}°`),
  ].join('');

  $('#r-shoot').innerHTML = [
    row('aim mode', brain.state.autoAim ? 'automatic' : 'manual'),
    row('turret', `${r.turret.angleDeg.toFixed(1)}° → ${r.turret.targetDeg.toFixed(1)}°`, r.turret.atLimit ? 'err' : ''),
    row('turret rate', `${r.turret.omegaDps.toFixed(0)} °/s`),
    row('motion lead', `${brain.state.leadDeg.toFixed(1)}°`),
    row('hood', `${r.hood.angleDeg.toFixed(1)}°`),
    row('flywheel', `${r.flywheel.rpm.toFixed(0)} / ${brain.state.targetRpm.toFixed(0)} rpm`, brain.state.ready ? 'good' : 'bad'),
    row('exit speed', r.lastShot ? `${r.lastShot.v_exit.toFixed(2)} m/s` : '—'),
    row('feed ready in', `${Math.max(0, r.transfer.cycleTime - r.transfer.sinceFeed).toFixed(2)} s`),
    row('shots fired', String(r.flywheel.shots)),
    row('balls out of play', String(s.outOfPlay), s.outOfPlay ? 'bad' : ''),
  ].join('');

  const hive = s.hives.find((h) => h.alliance === alliance)!;
  const restoring = Math.abs(hive.gravityTorque_Nm);
  // Only the component OPPOSING gravity counts: after a tip, balls in the now-down CELL
  // push the rocker further onto its stop, which is not progress toward the next one.
  const pushing = restoring > 1e-6 ? -hive.ballTorque_Nm * Math.sign(hive.gravityTorque_Nm) : 0;
  const progress = restoring > 1e-6 ? Math.max(0, Math.min(1.5, pushing / restoring)) : 0;
  const fill = $('#tip-fill');
  fill.style.width = `${Math.min(100, (progress / 1.5) * 100)}%`;
  fill.className = progress >= 1 ? 'over' : progress > 0.75 ? 'near' : '';
  $('#tip-label').textContent = hive.tipping
    ? 'Tipping now.'
    : progress <= 0
      ? 'Resting on its stop. The mark is where the balls beat gravity.'
      : `${(progress * 100).toFixed(0)}% of the torque needed to go over.`;

  $('#r-hive').innerHTML = [
    row('elements in up CELL', String(hive.ballsInUpCell)),
    row('ball torque', `${pushing.toFixed(3)} N·m`, pushing > restoring ? 'good' : ''),
    row('gravity holds', `${restoring.toFixed(3)} N·m`),
    row('angle', `${hive.angleDeg.toFixed(2)}°`),
    row('up CELL faces', hive.upCell),
    row('tips this match', String(hive.tips), hive.tips > 0 ? 'good' : ''),
  ].join('');

  $('#r-wheels').innerHTML = r.wheels
    .map((w) => row(w.name.toUpperCase(), `${w.cmd.toFixed(2)} cmd  ${w.force_N.toFixed(0)} N  slip ${w.slip.toFixed(2)}`, Math.abs(w.slip) > 0.25 ? 'bad' : ''))
    .join('');

  $('#r-telemetry').innerHTML = s.telemetry.length
    ? s.telemetry.map(([k, v]) => row(k, v)).join('')
    : row('—', 'no telemetry');
}

// ---------------------------------------------------------------- predictor

let prediction: Prediction | null = null;
let predictedAt = 0;
/** The solution being analysed, kept so the panel can show what it is reporting on. */
let predShot = { hood: 0, rpm: 0, speed: 0 };

/**
 * The sensitivity table. It costs about 20 trajectory integrations, so it is recomputed at
 * most twice a second and only while its own tab is open.
 */
function paintPredict(rangeIn: number): void {
  const now = performance.now();
  if (!prediction || now - predictedAt > 600) {
    predictedAt = now;
    // Analyse the shot the robot WOULD take from here -- the shot table's own solution for
    // this range -- not whatever the idle hardware happens to be sitting at. Otherwise the
    // panel reports on a 0 rpm shot that never leaves, which tells you nothing.
    const mz = world.robot.muzzle();
    const table = shotTable.lookup(rangeIn);
    const hood = robotSpec.hood.enabled
      ? robotSpec.hood.angleRange_deg[0] + table.hoodPos * (robotSpec.hood.angleRange_deg[1] - robotSpec.hood.angleRange_deg[0])
      : robotSpec.hood.fixedAngle_deg;
    const speed = rpmToSpeed(table.rpm, robotSpec.flywheel.k, robotSpec.flywheel.r_fly_m);
    predShot = { hood, rpm: table.rpm, speed };
    const radius = params.ball.pollen.d_m / 2;
    const mouth = world.hives[alliance].upCellMouthWorld();
    prediction = predict(
      params,
      {
        from: mz.pos, azimuth: mz.azimuth, elevation: hood * DEG, speed, radius,
        mass: params.ball.pollen.m_kg,
        spin: robotSpec.flywheel.type === 'single' ? speed / radius : 0,
      },
      mouth[1],
      rangeIn,
      knobs({
        tolRpm: robotSpec.flywheel.tolRpm,
        hoodSteps: 200,
        scatterDeg: robotSpec.flywheel.scatter.angle_deg,
        scatterSpeedFrac: robotSpec.flywheel.scatter.speedFrac,
        k: robotSpec.flywheel.k,
        rFly: robotSpec.flywheel.r_fly_m,
      }),
    );
  }
  const p = prediction;
  const ok = Number.isFinite(p.reach_in);

  $('#p-summary').innerHTML = [
    row('range to the CELL', m(p.target_in)),
    row('shot table solution', `${predShot.hood.toFixed(0)}° at ${predShot.rpm.toFixed(0)} rpm`),
    row('exit speed', `${predShot.speed.toFixed(2)} m/s`),
    row('this shot lands at', ok ? m(p.reach_in) : 'out of range'),
    row('miss', ok ? `${cmSigned(p.bias_in)} ${p.bias_in > 0 ? 'long' : 'short'}` : '—',
      ok ? (Math.abs(p.bias_in) < 6 ? 'good' : 'bad') : 'err'),
    row('predicted group, 1σ', `± ${cm(p.spread_in)}`, p.spread_in < 12 ? 'good' : 'bad'),
  ].join('');

  $('#p-rows').innerHTML = p.rows.map((k, i) => {
    const per = Number.isFinite(k.perStep_in) ? `${cmSigned(k.perStep_in)}/${k.step}${k.unit}` : '—';
    return `<div class="brow ${i === 0 ? 'lead' : ''}" data-key="${k.key}">
      <span class="n" title="${esc(k.why)}">${k.label} &nbsp;<span style="opacity:.6">${per}</span></span>
      <span class="t"><i style="width:${(k.share * 100).toFixed(0)}%"></i></span>
      <span class="v">${cm(k.contrib_in)}</span>
    </div>`;
  }).join('');

  const top = p.rows[0];
  $('#p-why').innerHTML = top
    ? `<b>${esc(top.label)}</b> owns ${(top.share * 100).toFixed(0)}% of the predicted spread. ${esc(top.why)}`
    : '';
}

const esc = (s: string) => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

// ---------------------------------------------------------------- analysis

let chartedAt = -1;

function paintAnalysis(s: Snapshot): void {
  const running = !!auto && auto.phase !== 'done';
  $('#a-plan').innerHTML = [
    row('shots to collect', String(plan.shots)),
    row('range sweep', `${m(plan.range_in[0])}–${m(plan.range_in[1])}`),
    row('bearing spread', `± ${plan.bearing_deg}°`),
    row('fire while moving', plan.onTheMove ? 'yes' : 'no'),
    row('progress', running ? `${auto!.shotsAsked} / ${plan.shots} — ${auto!.phase}` : s.shots.length ? 'finished' : 'not started'),
  ].join('');
  $<HTMLButtonElement>('#a-start').textContent = running ? 'Stop collection' : 'Start collection';
  $('#a-trace-note').textContent = `Flight recorder: ${trace.length} samples at ${trace.hz} Hz, ${(trace.length / trace.hz).toFixed(0)} s of run. It keeps recording across a reset so you can compare before and after a change.`;

  const rep: Report = analyse(s.shots, robotSpec.flywheel.tolRpm);
  $('#a-rows').innerHTML = [
    row('shots landed', `${rep.n}`),
    row('into the CELL', `${rep.landed} (${(rep.landRate * 100).toFixed(0)}%)`, rep.landRate > 0.6 ? 'good' : rep.landRate > 0.3 ? 'bad' : 'err'),
    row('downrange bias', cmSigned(rep.bias_in), Math.abs(rep.bias_in) < 6 ? 'good' : 'bad'),
    row('downrange spread 1σ', `± ${cm(rep.sd_in)}`, rep.sd_in < 18 ? 'good' : 'bad'),
    row('lateral bias', cmSigned(rep.latBias_in), Math.abs(rep.latBias_in) < 5 ? 'good' : 'bad'),
    row('lateral spread 1σ', `± ${cm(rep.latSd_in)}`, rep.latSd_in < 12 ? 'good' : 'bad'),
    row('rpm error at fire', `${rep.rpmErr_rpm.toFixed(0)} rpm`, rep.rpmErr_rpm < robotSpec.flywheel.tolRpm ? 'good' : 'bad'),
    row('fired off speed', String(rep.firedOffSpeed), rep.firedOffSpeed ? 'bad' : 'good'),
  ].join('');
  $('#a-verdict').textContent = rep.verdict;

  const cal = robotSpec.calibration;
  $('#a-cal').innerHTML = [
    row('range trim now', cmSigned(cal.rangeTrim_in), cal.rangeTrim_in ? 'good' : ''),
    row('turret trim now', `${cal.turretTrim_deg >= 0 ? '+' : ''}${cal.turretTrim_deg.toFixed(2)}°`, cal.turretTrim_deg ? 'good' : ''),
    row('this run suggests', rep.fix.worthIt
      ? `${cmSigned(rep.fix.rangeTrim_in)}, ${rep.fix.turretTrim_deg >= 0 ? '+' : ''}${rep.fix.turretTrim_deg.toFixed(2)}°`
      : 'no change', rep.fix.worthIt ? 'bad' : 'good'),
  ].join('');
  $('#a-fixwhy').textContent = rep.fix.why;
  $<HTMLButtonElement>('#a-apply').disabled = !rep.fix.worthIt;
  $('#a-faults').innerHTML = rep.faults.length
    ? rep.faults.map((f) => `<li>${esc(f)}</li>`).join('')
    : '<li class="cap">Nothing stood out.</li>';

  const worst = Math.max(1, ...rep.byRange.map((b) => b.meanMiss_in));
  $('#a-bins').innerHTML = rep.byRange.length
    ? rep.byRange.map((b) => `<div class="brow">
        <span class="n">${m(b.range_in)} &nbsp;<span style="opacity:.6">${b.landed}/${b.n} in</span></span>
        <span class="t"><i style="width:${((b.meanMiss_in / worst) * 100).toFixed(0)}%"></i></span>
        <span class="v">${b.meanMiss_in.toFixed(0)}"</span></div>`).join('')
    : '<p class="cap">No finished shots yet.</p>';

  // Charts are only redrawn when a shot actually finishes: redrawing three canvases at
  // 60 Hz to show the same dots is pure heat.
  const finished = s.shots.filter((x) => x.result !== 'flight').length;
  if (finished !== chartedAt) {
    chartedAt = finished;
    const mouthR = world.hives[alliance].upCell.halfInterior[2] * M_TO_IN;
    groupPlot($<HTMLCanvasElement>('#a-scatter'), s.shots, mouthR);
    errorVsRange($<HTMLCanvasElement>('#a-vsrange'), s.shots);
    histogram($<HTMLCanvasElement>('#a-hist'), s.shots);
  }

  $('#a-log').innerHTML = s.shots.slice(-12).reverse().map((x) =>
    row(`#${x.n} ${m(x.rangeIn, 1)} ${x.rpm.toFixed(0)}rpm`,
      x.result === 'flight' ? 'in flight' : `${x.result === 'cell' ? 'CELL' : 'miss'} ${cmSigned(x.long_in, 0)}/${cmSigned(x.lat_in, 0)}`,
      x.result === 'cell' ? 'good' : x.result === 'miss' ? 'bad' : ''),
  ).join('') || row('—', 'no shots yet');
}

// ---------------------------------------------------------------- modes and deck

/**
 * The action deck, per mode. Every entry runs real code and reports its own state; there is
 * nothing here that looks like a control and does nothing.
 */
interface Action { label: string; title: string; run: () => void; on?: () => boolean }

const DECK: Record<Mode, Action[]> = {
  practice: [
    { label: 'Auto-aim', title: 'Turret and hood solve for the CELL continuously, including a lead for the robot’s own motion. Off means the arrow keys aim it.', run: () => (brain.state.autoAim = !brain.state.autoAim), on: () => brain.state.autoAim },
    { label: 'Fire', title: 'Latch. Spins the flywheel, waits for it to be in tolerance and the turret to be on target, then feeds at the cycle time until you press it again.', run: () => (brain.state.firing = !brain.state.firing), on: () => brain.state.firing },
    { label: 'Auto-fill hopper', title: 'Practice aid, not a game rule: quietly picks up the nearest POLLEN off the floor whenever the hopper has room, so you can work on aiming without driving a collection lap.', run: () => setAutoLoad(!autoLoad), on: () => autoLoad },
    { label: 'Joystick', title: 'On-screen sticks: left translates, right turns. They feed the same gamepad frame the keyboard and a real controller do, so a phone or a trackpad can drive without either.', run: () => (sticks.visible = !sticks.visible), on: () => sticks.visible },
    { label: 'Shot zone', title: 'Green where a perfectly aimed shot clears the land-probability gate, red where it does not, using the hood and rpm the table commands at that range and the CELL mouth as seen from that spot. A MODEL map (tools/shotzone.ts), not a record of what this robot has hit.', run: () => (scene.showShotZone = !scene.showShotZone), on: () => scene.showShotZone },
    { label: 'Pause', title: 'Freeze the physics. The view still moves.', run: () => togglePause(), on: () => paused },
    { label: 'Reset', title: 'Rebuild the match: robot back on its start tile, balls re-staged, score and shot log cleared.', run: () => build() },
  ],
  collect: [
    { label: 'Shots: 20', title: 'How many samples this run collects. Below about 20 the statistics are noise.', run: () => { plan.shots = plan.shots >= 80 ? 20 : plan.shots + 20; }, },
    { label: 'Moving', title: 'Fire while still rolling instead of settling first. This is what the motion lead exists for, and the difference between the two runs tells you whether it works.', run: () => (plan.onTheMove = !plan.onTheMove), on: () => plan.onTheMove },
    { label: 'Auto-fill hopper', title: 'Required for a long run: the robot only carries a few balls and the sweep needs more.', run: () => setAutoLoad(!autoLoad), on: () => autoLoad },
    { label: 'Speed: 1x', title: 'Sim seconds per real second. The physics step never changes, so a 16x run gives exactly the same trajectories as a 1x one — it just does not make you watch.', run: () => cycleTurbo(), on: () => turbo > 1 },
    { label: 'Pause', title: 'Freeze the physics mid-run. Resuming continues the same plan.', run: () => togglePause(), on: () => paused },
    { label: 'Reset', title: 'Rebuild the match and clear the shot log.', run: () => build() },
  ],
  test: [
    { label: 'Auto-aim', title: 'Turret and hood solve for the CELL continuously. Off means the arrow keys aim it.', run: () => (brain.state.autoAim = !brain.state.autoAim), on: () => brain.state.autoAim },
    { label: 'Fire', title: 'Latch. Spins up and feeds at the cycle time until pressed again.', run: () => (brain.state.firing = !brain.state.firing), on: () => brain.state.firing },
    { label: 'Auto-fill hopper', title: 'Keeps the hopper topped up from the floor so a test run does not stop for ammunition.', run: () => setAutoLoad(!autoLoad), on: () => autoLoad },
    { label: 'Drop a POLLEN in the CELL', title: 'Places one POLLEN into your up CELL by hand. The quickest way to watch the HIVE tip: it takes 12.', run: () => dropBall() },
    { label: 'Shot arc', title: 'Draw the trajectory the ball would fly if it were fired this instant, using the same integrator the shot itself uses.', run: () => (scene.showTrajectory = !scene.showTrajectory), on: () => scene.showTrajectory },
    { label: 'Colliders', title: 'Show the convex shapes the solver actually collides with, instead of the CAD skin drawn over them.', run: () => (scene.showColliders = !scene.showColliders), on: () => scene.showColliders },
    { label: 'Joystick', title: 'On-screen sticks: left translates, right turns. They feed the same gamepad frame the keyboard and a real controller do, so a phone or a trackpad can drive without either.', run: () => (sticks.visible = !sticks.visible), on: () => sticks.visible },
    { label: 'Shot zone', title: 'Green where a perfectly aimed shot clears the land-probability gate, red where it does not, using the hood and rpm the table commands at that range and the CELL mouth as seen from that spot. A MODEL map (tools/shotzone.ts), not a record of what this robot has hit.', run: () => (scene.showShotZone = !scene.showShotZone), on: () => scene.showShotZone },
    { label: 'Speed: 1x', title: 'Sim seconds per real second. The physics step never changes, so the trajectories are identical — it just runs more of them per frame.', run: () => cycleTurbo(), on: () => turbo > 1 },
    { label: 'Pause', title: 'Freeze the physics.', run: () => togglePause(), on: () => paused },
    { label: 'Reset', title: 'Rebuild the match with the current variables.', run: () => build() },
  ],
};

function buildDeck(): void {
  const host = $('#deck');
  host.innerHTML = '';
  for (const [i, a] of DECK[mode].entries()) {
    const b = document.createElement('button');
    b.textContent = a.label;
    b.title = a.title;
    b.dataset.i = String(i);
    b.onclick = () => { a.run(); paintDeck(); };
    host.appendChild(b);
  }
}

function paintDeck(): void {
  const list = DECK[mode];
  for (const b of $$<HTMLButtonElement>('#deck button')) {
    const a = list[Number(b.dataset.i)];
    if (!a) continue;
    if (a.on) b.classList.toggle('on', a.on());
    if (a.label.startsWith('Shots:')) b.textContent = `Shots: ${plan.shots}`;
    if (a.label.startsWith('Speed:')) b.textContent = `Speed: ${turbo}x`;
  }
  const running = !!auto && auto.phase !== 'done';
  const p = $<HTMLButtonElement>('#primary');
  p.textContent = mode === 'collect'
    ? running ? 'Stop collection' : 'Run collection'
    : world.clock.period === 'STAGING' ? 'Start match' : paused ? 'Resume' : 'Pause';
  p.title = mode === 'collect'
    ? 'Start the autonomous sweep described in the Analysis tab.'
    : 'Start the 2:38 match clock, or pause it once it is running.';
}

function setMode(m: Mode): void {
  mode = m;
  for (const b of $$<HTMLButtonElement>('#mode button')) b.classList.toggle('on', b.dataset.mode === m);
  buildDeck();
  // Each mode opens on the tab it exists for.
  showTab(m === 'collect' ? 'analysis' : m === 'test' ? 'predict' : 'robot');
  if (m !== 'collect' && auto) auto = null;
  if (m !== 'collect') turbo = 1;   // never fast-forward while a human is driving
  if (m === 'collect') {
    autoLoad = true;                 // a 40-shot sweep needs more balls than the robot holds
    scene.cameraMode = 'follow';
    $<HTMLSelectElement>('#camera').value = 'follow';
  }
  paintDeck();
}

const TURBO = [1, 4, 16];
const cycleTurbo = () => { turbo = TURBO[(TURBO.indexOf(turbo) + 1) % TURBO.length]; };

const MODES: Mode[] = ['practice', 'collect', 'test'];
const cycleMode = () => setMode(MODES[(MODES.indexOf(mode) + 1) % MODES.length]);

const CAMERAS: CameraMode[] = ['orbit', 'follow', 'fpv', 'top', 'muzzle'];
function cycleCamera(d: number): void {
  const i = (CAMERAS.indexOf(scene.cameraMode) + d + CAMERAS.length) % CAMERAS.length;
  scene.cameraMode = CAMERAS[i];
  $<HTMLSelectElement>('#camera').value = CAMERAS[i];
}

/** The green button: whatever the current mode's main verb is. */
function primary(): void {
  if (mode === 'collect') {
    if (auto && auto.phase !== 'done') { auto = null; }
    else {
      const m = world.hives[alliance].upCellMouthWorld();
      auto = new AutoDriver(plan, [m[0], m[2]], world.geom.halfWidth_m);
      world.clock.startTeleOp();
      paused = false;
    }
  } else if (world.clock.period === 'STAGING') {
    world.clock.start();
    paused = false;
  } else {
    togglePause();
  }
  paintDeck();
}

// ---------------------------------------------------------------- wiring

function showTab(name: string): void {
  for (const b of $$('.tabs button')) b.classList.toggle('on', b.dataset.tab === name);
  for (const s of $$('#panel section')) s.classList.toggle('on', s.dataset.panel === name);
}

function wireUi(): void {
  for (const b of $$<HTMLButtonElement>('.tabs button')) b.onclick = () => showTab(b.dataset.tab!);
  for (const b of $$<HTMLButtonElement>('#mode button')) b.onclick = () => setMode(b.dataset.mode as Mode);
  $('#primary').onclick = primary;

  $<HTMLSelectElement>('#alliance').onchange = (e) => {
    alliance = (e.target as HTMLSelectElement).value as Alliance;
    build();
  };
  $<HTMLSelectElement>('#camera').onchange = (e) => { scene.cameraMode = (e.target as HTMLSelectElement).value as CameraMode; };
  $<HTMLSelectElement>('#brain-select').onchange = (e) => {
    useJavaBrain = (e.target as HTMLSelectElement).value === 'java';
    if (useJavaBrain) bridge.connect(paintBrain); else bridge.disconnect();
    paintBrain();
  };

  $('#apply-reset').onclick = () => { build(); $('#apply-reset').classList.remove('on'); };
  $('#tune-default').onclick = () => {
    params = structuredClone(baseParams);
    robotSpec = structuredClone(baseRobot);
    buildTunePanel($('#tune-rows'), params, robotSpec, onTune);
    build();
  };

  $('#a-start').onclick = primary;
  $('#a-clear').onclick = () => { world.shotLog.length = 0; chartedAt = -1; };
  $('#a-csv').onclick = () => download('shots', toCsv(world.shotLog));
  $('#a-apply').onclick = () => {
    const rep = analyse(world.shotLog, robotSpec.flywheel.tolRpm);
    if (!rep.fix.worthIt) return;
    // Trims ACCUMULATE. Each run measures the error that is left after the last trim, so
    // adding is what converges; replacing would undo the previous correction every time.
    robotSpec.calibration.rangeTrim_in += rep.fix.rangeTrim_in;
    robotSpec.calibration.turretTrim_deg += rep.fix.turretTrim_deg;
    world.shotLog.length = 0;
    chartedAt = -1;
  };
  $('#a-calclear').onclick = () => {
    robotSpec.calibration.rangeTrim_in = 0;
    robotSpec.calibration.turretTrim_deg = 0;
  };
  $('#a-trace').onclick = () => download('trace', trace.toCsv());

  installGrip();
  addEventListener('resize', () => { scene.resize(); chartedAt = -1; });
  addEventListener('keydown', (e) => {
    if ((e.target as HTMLElement).tagName === 'INPUT') return;
    const k = e.key.toLowerCase();
    if (e.key === 'Enter') primary();
    else if (k === 'p') togglePause();
    else if (k === 'r') build();
    else if (k === 'l') setAutoLoad(!autoLoad);
    else if (k === 'm') cycleMode();
    else if (k >= '1' && k <= '5') {
      scene.cameraMode = CAMERAS[Number(k) - 1];
      $<HTMLSelectElement>('#camera').value = CAMERAS[Number(k) - 1];
    }
  });
  paintBrain();
}

/**
 * Drag the panel's left edge to resize it. The width lives on :root so the grip, the panel
 * and the canvas all follow from one number, and it is remembered per browser because a
 * driver who widened it once should not have to do it again.
 */
function installGrip(): void {
  const grip = $('#grip');
  const apply = (px: number) => {
    const w = Math.max(300, Math.min(innerWidth * 0.7, px));
    document.documentElement.style.setProperty('--panel-w', `${w}px`);
    scene.resize();
    chartedAt = -1;
    try { localStorage.setItem('panelW', String(w)); } catch { /* private window */ }
  };
  try {
    const saved = Number(localStorage.getItem('panelW'));
    if (saved > 0) apply(saved);
  } catch { /* private window */ }
  grip.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    grip.classList.add('dragging');
    grip.setPointerCapture(e.pointerId);
    const move = (m: PointerEvent) => apply(innerWidth - m.clientX);
    const up = () => {
      grip.classList.remove('dragging');
      removeEventListener('pointermove', move);
      removeEventListener('pointerup', up);
    };
    addEventListener('pointermove', move);
    addEventListener('pointerup', up);
  });
}

/** Hand a CSV to the browser's downloader. */
function download(kind: string, text: string): void {
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([text], { type: 'text/csv' }));
  a.download = `biobuzz-${kind}-${new Date().toISOString().slice(0, 19).replace(/[:T-]/g, '')}.csv`;
  a.click();
  URL.revokeObjectURL(a.href);
}

function setAutoLoad(on: boolean): void {
  autoLoad = on;
}

/**
 * Practice aid: keep the hopper topped up from balls lying on the floor, so aiming can be
 * worked on without driving a collection lap first. Not a game rule — it takes real balls
 * off the real field, and the field runs out.
 */
function topUpHopper(): void {
  const r = world.robot;
  if (r.hopper.length >= robotSpec.hopper.capacity) return;
  const p = r.pos;
  let best: (typeof world.balls.balls)[number] | null = null;
  let bestD = Infinity;
  for (const b of world.balls.balls) {
    if (b.kind !== 'pollen' || b.state !== 'free') continue;
    const q = world.balls.pos(b);
    if (q[1] > inches(8)) continue; // only what is lying on the floor
    const d = Math.hypot(q[0] - p[0], q[2] - p[2]);
    if (d < bestD) { bestD = d; best = b; }
  }
  if (best) world.robot.preload(world.balls, best);
}

function paintBrain(): void {
  const detail: Record<string, string> = {
    off: 'relay down — run npm run relay',
    connecting: 'connecting',
    waiting: 'waiting for the Java runner',
    live: 'live',
    error: 'relay unreachable — run npm run relay',
  };
  $('#brain').textContent = useJavaBrain ? `java: ${detail[bridge.status]}` : 'built-in';
}

function togglePause(): void {
  paused = !paused;
  paintDeck();
}

/** Hand-drop a POLLEN into the up CELL: the quickest way to watch a tip happen. */
function dropBall(): void {
  const hive = world.hives[alliance];
  const free = world.balls.balls.find((b) => b.state === 'free' && b.kind === 'pollen' && world.balls.pos(b)[1] < inches(6));
  if (!free) return;
  const cell = hive.upCell;
  const phi = cell.bodyAngle_rad;
  const u = -cell.halfInterior[1] + free.radius + 0.02;
  const t = cell.halfInterior[2] - free.radius - 0.02 - Math.random() * 0.06;
  const local: Vec3 = [
    (Math.random() - 0.5) * inches(14),
    (cell.radius_m + u) * Math.cos(phi) - t * Math.sin(phi),
    (cell.radius_m + u) * Math.sin(phi) + t * Math.cos(phi),
  ];
  world.balls.release(free, hive.toWorld(local), [0, 0, 0], [0, 0, 0], 'cell');
}

void CONTROLS;
void DEG;
void lastAct;
boot();
