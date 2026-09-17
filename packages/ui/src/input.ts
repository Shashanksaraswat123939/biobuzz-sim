/** Keyboard mapped onto a Gamepad, so the brain only ever sees one input shape. */
import { emptyGamepad } from '@core/physics/world.js';
import type { GamepadState } from '@core/types.js';

export interface Keys {
  /** Keys currently held: for continuous controls like drive. */
  down: Set<string>;
  /**
   * Keys pressed since the last read, latched on keydown. Polling `down` would drop a tap
   * that starts and ends between two animation frames, which is most of them.
   */
  pressed: Set<string>;
}

export function installKeyboard(): Keys {
  const keys: Keys = { down: new Set(), pressed: new Set() };
  addEventListener('keydown', (e) => {
    if ((e.target as HTMLElement).tagName === 'INPUT') return;
    const k = e.key.toLowerCase();
    keys.down.add(k);
    if (!e.repeat) keys.pressed.add(k);
    // Space scrolls and the arrows scroll; neither should while driving.
    if (e.key === ' ' || e.key.startsWith('Arrow')) e.preventDefault();
  });
  addEventListener('keyup', (e) => keys.down.delete(e.key.toLowerCase()));
  addEventListener('blur', () => {
    keys.down.clear();
    keys.pressed.clear();
  });
  return keys;
}

/**
 * The two paddle buttons. `Gamepad.buttons` only standardises 17 entries, and M1/M2 on the
 * pads that have them sit past that in a vendor block, so they are read positionally and fall
 * back to L1/L2 on a pad without them. Carried beside the frame rather than added to
 * `GamepadState`, which is the wire the Java OpMode sees and has no such button.
 */
export interface Paddles { m1: boolean; m2: boolean }

/**
 * PHYSICAL keyboard state, in gamepad shape: which BUTTON is down, not what it does. The
 * mapping from button to action lives in one place (`remap` in main.ts) so the keyboard and a
 * real controller cannot drift apart.
 *
 * `pressed` is NOT cleared here: a frame may read the input and then run no physics step,
 * and a toggle consumed by nobody is a toggle the user has to press twice. The caller
 * clears it once a step has actually seen it.
 */
export function readKeyboard(keys: Keys): GamepadState & { paddles: Paddles } {
  const g = emptyGamepad() as GamepadState & { paddles: Paddles };
  const on = (k: string) => keys.down.has(k);
  // Left stick: drive. W/S forward and back, A/D strafe.
  g.left_stick_y = (on('s') ? 1 : 0) - (on('w') ? 1 : 0);
  g.left_stick_x = (on('d') ? 1 : 0) - (on('a') ? 1 : 0);
  // Right stick: the view. Arrows, because that is what a keyboard user reaches for to look
  // around, and the turret has moved to , and . to make room for it.
  g.right_stick_x = (on('arrowright') ? 1 : 0) - (on('arrowleft') ? 1 : 0);
  g.right_stick_y = (on('arrowdown') ? 1 : 0) - (on('arrowup') ? 1 : 0);
  // Face buttons: Q/E turn (X/B), R/F change gear (Y/A).
  g.x = on('q');
  g.b = on('e');
  g.y = keys.pressed.has('r');
  g.a = keys.pressed.has('f');
  g.right_bumper = on(' ');    // R1: fire
  g.left_bumper = on('shift'); // momentary crawl
  g.left_trigger = on('z') ? 1 : 0;  // reverse the intake
  g.dpad_left = on(',');       // manual turret slew
  g.dpad_right = on('.');
  g.left_stick_button = keys.pressed.has('backspace'); // re-zero the field frame
  g.right_stick_button = on('control');                // hold: robot-centric
  g.paddles = { m1: keys.pressed.has('t'), m2: keys.pressed.has('g') };
  return g;
}
