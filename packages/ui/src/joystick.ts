/**
 * On-screen sticks, mapped onto the same GamepadState everything else speaks.
 *
 * The keyboard already does this (`input.ts`), and the point is the same: the brain sees one
 * input shape and does not know or care where it came from. What this adds is a way to drive
 * with a thumb -- on a phone, on a laptop trackpad, or on the published page where there is
 * no gamepad to plug in.
 *
 * Two sticks because the drivetrain is holonomic and needs three numbers: the left one
 * translates (x, y) and the right one turns (x). One stick cannot express "strafe left while
 * spinning", which is most of what an X-drive is for.
 *
 * Pointer events, not touch events: the same code then works for a mouse, a finger and a
 * stylus, and `setPointerCapture` keeps a drag alive after the thumb slides off the pad --
 * without it the robot keeps driving because the stick never sees the release.
 */
import type { GamepadState } from '@core/types.js';

interface Stick {
  root: HTMLDivElement;
  knob: HTMLDivElement;
  x: number;
  y: number;
  pointer: number | null;
}

export class Joysticks {
  private readonly root: HTMLDivElement;
  private readonly left: Stick;
  private readonly right: Stick;

  constructor(parent: HTMLElement) {
    this.root = document.createElement('div');
    this.root.className = 'sticks';
    this.root.hidden = true;
    this.left = this.makeStick('drive');
    this.right = this.makeStick('turn');
    this.root.append(this.left.root, this.right.root);
    parent.append(this.root);
  }

  private makeStick(label: string): Stick {
    const root = document.createElement('div');
    root.className = 'stick';
    root.dataset.label = label;
    const knob = document.createElement('div');
    knob.className = 'knob';
    root.append(knob);
    const s: Stick = { root, knob, x: 0, y: 0, pointer: null };

    const move = (e: PointerEvent) => {
      const r = root.getBoundingClientRect();
      const reach = r.width / 2;
      let dx = (e.clientX - (r.left + reach)) / reach;
      let dy = (e.clientY - (r.top + r.height / 2)) / reach;
      // Clamp to the circle, not the square: a stick pushed into a corner must not read 1.41.
      const mag = Math.hypot(dx, dy);
      if (mag > 1) {
        dx /= mag;
        dy /= mag;
      }
      s.x = dx;
      s.y = dy;
      knob.style.transform = `translate(${dx * reach * 0.62}px, ${dy * reach * 0.62}px)`;
    };
    const release = () => {
      s.pointer = null;
      s.x = 0;
      s.y = 0;
      knob.style.transform = 'translate(0px, 0px)';
      root.classList.remove('live');
    };

    root.addEventListener('pointerdown', (e) => {
      s.pointer = e.pointerId;
      root.setPointerCapture(e.pointerId);
      root.classList.add('live');
      move(e);
      e.preventDefault();
    });
    root.addEventListener('pointermove', (e) => {
      if (s.pointer === e.pointerId) move(e);
    });
    for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture'] as const) {
      root.addEventListener(ev, (e) => {
        if (s.pointer === (e as PointerEvent).pointerId) release();
      });
    }
    return s;
  }

  get visible(): boolean {
    return !this.root.hidden;
  }

  set visible(on: boolean) {
    this.root.hidden = !on;
    // Let go of whatever was held: a stick hidden mid-drag would otherwise be stuck on.
    if (!on) {
      for (const s of [this.left, this.right]) {
        s.pointer = null;
        s.x = 0;
        s.y = 0;
        s.knob.style.transform = 'translate(0px, 0px)';
        s.root.classList.remove('live');
      }
    }
  }

  /**
   * Fold the sticks into a frame from the keyboard or a real gamepad.
   *
   * Largest magnitude wins per axis rather than summing: holding W while pushing the stick
   * back should not cancel to nothing, and it should not add up to 2 either.
   */
  merge(g: GamepadState): GamepadState {
    if (this.root.hidden) return g;
    const pick = (a: number, b: number) => (Math.abs(b) > Math.abs(a) ? b : a);
    return {
      ...g,
      left_stick_x: pick(g.left_stick_x, this.left.x),
      // Screen Y grows downward and a gamepad's stick Y does too, so this needs no flip --
      // pushing the knob up reads negative, which is forward, exactly like the hardware.
      left_stick_y: pick(g.left_stick_y, this.left.y),
      right_stick_x: pick(g.right_stick_x, this.right.x),
    };
  }
}
