/** The match clock: staging -> 30 s AUTO -> 8 s transition -> 2:00 TELEOP -> finished. */
import type { Params, Period } from '../types.js';

export class MatchClock {
  period: Period = 'STAGING';
  /** Seconds elapsed inside the current period. */
  elapsed = 0;
  running = false;

  constructor(private readonly m: Params['match']) {}

  private lengthOf(p: Period): number {
    switch (p) {
      case 'AUTO': return this.m.auto_s;
      case 'TRANSITION': return this.m.transition_s;
      case 'TELEOP': return this.m.teleop_s;
      default: return Infinity;
    }
  }

  /** Seconds left in the current period. */
  get remaining(): number {
    const len = this.lengthOf(this.period);
    return len === Infinity ? 0 : Math.max(0, len - this.elapsed);
  }

  /** True once the last minute of TELEOP has begun: the FLOWER and NECTAR gate (G410/G426). */
  get endgame(): boolean {
    return this.period === 'TELEOP' && this.remaining <= this.m.flowerUnlock_s;
  }

  get started(): boolean {
    return this.period === 'AUTO' || this.period === 'TELEOP';
  }

  get stopped(): boolean {
    return this.period === 'FINISHED' || this.period === 'TRANSITION' || !this.running;
  }

  start(): void {
    if (this.period === 'STAGING') {
      this.period = 'AUTO';
      this.elapsed = 0;
    }
    this.running = true;
  }

  /** Skip straight to driving, for practice. */
  startTeleOp(): void {
    this.period = 'TELEOP';
    this.elapsed = 0;
    this.running = true;
  }

  pause(): void {
    this.running = false;
  }

  /** Returns the period we just left, if the clock rolled over this step. */
  tick(dt: number): Period | null {
    if (!this.running || this.period === 'STAGING' || this.period === 'FINISHED') return null;
    this.elapsed += dt;
    if (this.elapsed < this.lengthOf(this.period)) return null;
    const was = this.period;
    this.elapsed = 0;
    this.period = was === 'AUTO' ? 'TRANSITION' : was === 'TRANSITION' ? 'TELEOP' : 'FINISHED';
    if (this.period === 'FINISHED') this.running = false;
    return was;
  }

  reset(): void {
    this.period = 'STAGING';
    this.elapsed = 0;
    this.running = false;
  }
}
