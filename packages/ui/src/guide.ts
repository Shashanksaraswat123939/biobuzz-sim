/**
 * Every control in the app, with what it actually does.
 *
 * This is the reference the Guide tab renders and the only place the control list is
 * written down, so it cannot drift from the keymap the way a hand-kept README does.
 */

export interface ControlDoc {
  /** What you press. */
  key: string;
  /** The same thing on a gamepad, if it has one. */
  pad?: string;
  what: string;
  /** What it does, in one or two sentences. */
  why: string;
}

export interface ControlGroup {
  title: string;
  blurb?: string;
  items: ControlDoc[];
}

export const CONTROLS: ControlGroup[] = [
  {
    title: 'Driving',
    blurb: 'Mecanum, and FIELD-CENTRIC: W is away from the driver station whatever way the robot is pointing. With a turret the chassis ends up facing wherever it was last going, so robot-centric means "forward" is a direction you have to keep track of and cannot see — which is what made this hard to drive. Hold Y for robot-centric, and tap Backspace to make "forward" whichever way the robot faces right now.',
    items: [
      { key: 'W S', pad: 'left stick ↕', what: 'Away from and toward the driver station', why: 'Field-relative. On a keyboard the stick ramps to full over about a fifth of a second rather than stepping, because a key has no middle and a 1.2 m/s drivetrain asked for in one step is undriveable. The wheels model slip, so flooring it on a low tile friction spins them instead of accelerating.' },
      { key: 'Y', pad: 'Y', what: 'Robot-centric while held', why: 'Falls back to W-is-where-the-nose-points, which is what you want if the heading estimate has drifted.' },
      { key: 'Backspace', pad: 'Back', what: 'Re-zero the field frame', why: 'Makes “forward” mean whichever way the robot is facing now. Square it up to the field and tap it; a real IMU drifts and every driver wants this button.' },
      { key: 'A D', pad: 'left stick ↔', what: 'Strafe left and right', why: 'Sideways without turning. Mecanum strafing is about 70% as efficient as driving straight, and the simulator charges you that.' },
      { key: 'Q E', pad: 'right stick ↔', what: 'Turn left and right', why: 'Rotates in place. Mixed with the left stick it arcs.' },
      { key: 'Shift', pad: 'LB', what: 'Slow mode', why: 'Scales every drive command to 35%. Use it for lining up close to the HIVE.' },
    ],
  },
  {
    title: 'Shooting',
    blurb: 'The intake always runs, like a real one. The gate opens itself when a shot is cleared, so there is nothing to arm by hand.',
    items: [
      { key: 'T', pad: 'X', what: 'Auto-aim on/off', why: 'On: the turret and hood solve for your up CELL every loop, including a lead for the robot’s own velocity, so you can shoot while driving across the shot. Off: the turret holds wherever you last pointed it.' },
      { key: '← →', pad: 'D-pad ↔', what: 'Aim the turret by hand', why: 'Only when auto-aim is off. Slews at 100 °/s; the axis has its own acceleration limit, so it does not snap.' },
      { key: 'Space', pad: 'RB', what: 'Fire — a latch, not a trigger', why: 'Press once and it stays on: the flywheel spins up, and every time the wheel is inside its tolerance band and the turret is on target, one ball goes. Press again to stop. The cycle time paces it, not your thumb.' },
      { key: 'F', pad: 'A', what: 'Pre-spin the flywheel', why: 'Optional. Brings the wheel up before you commit, so the first shot does not wait ~1.5 s for spin-up. Firing does this anyway.' },
      { key: 'L', pad: 'Y', what: 'Auto-fill the hopper', why: 'Practice aid: quietly picks up the nearest POLLEN lying on the floor whenever there is room. It takes real balls off the real field, so the field does run out.' },
    ],
  },
  {
    title: 'View',
    items: [
      { key: 'left-drag', what: 'Orbit the camera', why: 'Anywhere on the field. The camera never moves the robot.' },
      { key: 'right-drag', what: 'Pan', why: 'Slides the view across the field, the way every other 3D view does. Middle-drag and shift+drag do the same.' },
      { key: 'scroll', what: 'Zoom', why: '' },
      { key: '1 – 5', pad: 'D-pad ↕', what: 'Camera presets', why: 'Orbit, Follow (behind the robot), Driver (on the robot, looking where it points), Overhead, Muzzle (down the barrel — the turret is usually pointed somewhere the chassis is not).' },
    ],
  },
  {
    title: 'Match and modes',
    items: [
      { key: 'Enter', pad: 'Start', what: 'The green button', why: 'Starts the match clock in Practice and Test; runs or stops the autonomous routine in Auto; starts or stops the collection run in Data.' },
      { key: 'P', pad: 'L3', what: 'Pause', why: 'Freezes the physics. The camera still moves.' },
      { key: 'R', what: 'Reset', why: 'Rebuilds the match from the current variables: robot on its start tile, balls re-staged, score and shot log cleared.' },
      { key: 'M', pad: 'Back', what: 'Next mode', why: 'Cycles Practice → Auto → Data → Test.' },
    ],
  },
];

export const MODES: { name: string; what: string }[] = [
  {
    name: 'Practice',
    what: 'You drive, the clock runs, and the scoring is the real scoring. Nothing is scripted — the HIVE tips because the balls in it beat gravity, not because a counter reached a number.',
  },
  {
    name: 'Auto',
    what: 'The 30-second autonomous, played out. The routine LEAVEs the wall, drives round into the shooting sector — it has to, because the up CELL opens toward the audience and from the start tile the mouth is about 109° off its opening, so no launch from there can enter — empties the preload into the CELL, and PARKs before the buzzer. It drives through the same gamepad frames a human would produce, so nothing about the shot is privileged. `npm run tool -- tools/autocheck.ts` runs the same routine headlessly over several seeds, which is the honest way to judge it.',
  },
  {
    name: 'Data',
    what: 'The robot drives itself through a sweep of firing positions across a range band, settles (or keeps rolling, if you ask it to), and takes one shot per sample. It goes through the same aim solver and readiness gate you do, so the numbers mean something. When it finishes, the Analysis tab separates bias from spread and names the fault.',
  },
  {
    name: 'Test',
    what: 'Sandbox. Every physical constant is a slider in Variables, and the Predictor shows what each one is doing to the shot you are about to take — measured by re-running the trajectory with that one value nudged, not estimated.',
  },
];

export const PANELS: { name: string; what: string }[] = [
  { name: 'Robot', what: 'Live state: pose, shooter, the HIVE tip meter with the torque balance behind it, per-wheel slip, and whatever the brain is putting on telemetry.' },
  { name: 'Predictor', what: 'The error budget for the current shot. Each row is (inches the ball moves per unit of that variable) × (that variable’s own 1 σ uncertainty). The bar is its share of the total variance. Hover a row name for where its uncertainty comes from.' },
  { name: 'Analysis', what: 'The shot log and what it means. Bias is a table error that one offset fixes; spread is a repeatability error that no offset fixes. Export CSV to take the raw log elsewhere.' },
  { name: 'Variables', what: 'Every constant the simulator runs on, read from config/*.json. Rows marked * are baked into rigid bodies and need Apply & restart; everything else is live.' },
  { name: 'Guide', what: 'This page.' },
];

export function renderGuide(host: HTMLElement): void {
  const esc = (s: string) => s.replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' })[c]!);
  const parts: string[] = [];

  parts.push('<div class="guide-group"><h5>Modes</h5>');
  for (const m of MODES) parts.push(`<div class="guide-item"><kbd>${m.name}</kbd><p>${esc(m.what)}</p></div>`);
  parts.push('</div>');

  for (const g of CONTROLS) {
    parts.push(`<div class="guide-group"><h5>${esc(g.title)}</h5>`);
    if (g.blurb) parts.push(`<p class="cap">${esc(g.blurb)}</p>`);
    for (const c of g.items) {
      const pad = c.pad ? ` <span style="opacity:.6">/ ${esc(c.pad)}</span>` : '';
      parts.push(`<div class="guide-item"><kbd>${esc(c.key)}</kbd><p><b>${esc(c.what)}</b>${pad}${c.why ? `<br>${esc(c.why)}` : ''}</p></div>`);
    }
    parts.push('</div>');
  }

  parts.push('<div class="guide-group"><h5>Panels</h5>');
  for (const p of PANELS) parts.push(`<div class="guide-item"><kbd>${p.name}</kbd><p>${esc(p.what)}</p></div>`);
  parts.push('</div>');

  host.innerHTML = parts.join('');
}
