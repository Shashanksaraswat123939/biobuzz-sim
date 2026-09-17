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
 * `pressed` is NOT cleared here: a frame may read the input and then run no physics step,
 * and a toggle consumed by nobody is a toggle the user has to press twice. The caller
 * clears it once a step has actually seen it.
 */
export function readKeyboard(keys: Keys): GamepadState {
  const g = emptyGamepad();
  const on = (k: string) => keys.down.has(k);
  g.left_stick_y = (on('s') ? 1 : 0) - (on('w') ? 1 : 0);
  g.left_stick_x = (on('d') ? 1 : 0) - (on('a') ? 1 : 0);
  g.right_stick_x = (on('e') ? 1 : 0) - (on('q') ? 1 : 0);
  g.left_bumper = on('shift'); // slow mode
  // Held: fall back to robot-centric driving. The default is field-centric, and this is the
  // escape hatch for a drifted heading.
  g.y = on('y');
  // Manual turret slew. Arrow keys, because that is where a hand already is.
  g.dpad_left = on('arrowleft');
  g.dpad_right = on('arrowright');
  // Edge-triggered toggles, consumed on read.
  g.a = keys.pressed.has('f'); // pre-spin the flywheel
  g.x = keys.pressed.has('t'); // auto-aim
  g.right_bumper = keys.pressed.has(' '); // fire latch
  g.back = keys.pressed.has('backspace'); // re-zero the field frame
  return g;
}
