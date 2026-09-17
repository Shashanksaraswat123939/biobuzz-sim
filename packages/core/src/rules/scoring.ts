/**
 * Table 10-2, encoded.
 *
 * | Event                    | AUTO | TELEOP | assessed            |
 * | HIVE TIP                 |  20  |   20   | on the tip          |
 * | LEAVE                    |   3  |    -   | end of AUTO         |
 * | PARK                     |   5  |    5   | end of AUTO / match |
 * | element in the up-CELL   |   -  |    2   | end of match only   |
 * | bottom NECTAR bonus      |   -  |    5   | end of match        |
 * | element in an owned FLOWER |  - |    2   | end of match        |
 * | element in a GARDEN      |   -  |    1   | end of match        |
 *
 * RP: SWARM (LEAVE + PARK >= 16), POLLINATOR 1 (>= 4 tips), POLLINATOR 2 (>= 7 tips).
 *
 * Two readings of the bottom-NECTAR bonus are open (PLAN.md 10.1). Both are counted;
 * `bottomNectarPerFlower` picks which one the headline total uses.
 */
import type { Alliance, AllianceScore, BallKind, Period, ScoreState, Vec3 } from '../types.js';

export interface EndOfMatchCounts {
  upCell: number;
  garden: number;
  /** Per FLOWER: how many of each alliance's NECTAR, and the total element count. */
  flowers: { elements: number; redNectar: number; blueNectar: number }[];
  /** The lowest-sitting NECTAR in each FLOWER, by alliance. */
  bottomNectar: { flower: number; alliance: Alliance | null }[];
  parked: boolean;
  left: boolean;
}

const empty = (): AllianceScore => ({
  tips: 0, autoTips: 0, leave: false, park: false,
  upCell: 0, flower: 0, garden: 0, bottomNectar: 0,
  auto: 0, teleop: 0, total: 0,
  rp: { swarm: false, pollinator1: false, pollinator2: false },
});

export class Scorer {
  readonly state: ScoreState = { red: empty(), blue: empty(), fouls: [] };
  bottomNectarPerFlower = true;

  tip(alliance: Alliance, period: Period, t: number): void {
    const s = this.state[alliance];
    s.tips++;
    if (period === 'AUTO' || period === 'TRANSITION') s.autoTips++;
    this.recompute();
    void t;
  }

  foul(t: number, rule: string, note: string): void {
    this.state.fouls.push({ t, rule, note });
  }

  setAutoResult(alliance: Alliance, left: boolean, parked: boolean): void {
    const s = this.state[alliance];
    s.leave = left;
    s.park = parked;
    this.recompute();
  }

  /** Called once when the match ends; everything positional is assessed here, not before. */
  finalise(alliance: Alliance, c: EndOfMatchCounts): void {
    const s = this.state[alliance];
    s.upCell = c.upCell;
    s.garden = c.garden;
    s.park = c.parked;

    let flowerElements = 0;
    c.flowers.forEach((f, i) => {
      const mine = alliance === 'red' ? f.redNectar : f.blueNectar;
      const theirs = alliance === 'red' ? f.blueNectar : f.redNectar;
      if (mine > theirs) flowerElements += f.elements; // owner takes everything in it
      void i;
    });
    s.flower = flowerElements;

    const bottoms = c.bottomNectar.filter((b) => b.alliance === alliance).length;
    s.bottomNectar = this.bottomNectarPerFlower ? bottoms : Math.min(1, bottoms);
    this.recompute();
  }

  private recompute(): void {
    for (const a of ['red', 'blue'] as const) {
      const s = this.state[a];
      s.auto = s.autoTips * 20 + (s.leave ? 3 : 0) + (s.park ? 5 : 0);
      s.teleop = (s.tips - s.autoTips) * 20 + s.upCell * 2 + s.flower * 2 + s.garden * 1 + s.bottomNectar * 5;
      s.total = s.auto + s.teleop;
      s.rp.swarm = (s.leave ? 3 : 0) + (s.park ? 5 : 0) >= 16;
      s.rp.pollinator1 = s.tips >= 4;
      s.rp.pollinator2 = s.tips >= 7;
    }
  }

  reset(): void {
    Object.assign(this.state.red, empty());
    Object.assign(this.state.blue, empty());
    this.state.fouls.length = 0;
  }
}

/** Is a ball inside a FLOWER's scoring volume (between the middle and top rings)? */
export function inFlowerScoringVolume(p: Vec3, f: { x_m: number; z_m: number; openingR_m: number; scoreLow_m: number; scoreHigh_m: number }, r: number): boolean {
  if (p[1] < f.scoreLow_m || p[1] > f.scoreHigh_m) return false;
  return Math.hypot(p[0] - f.x_m, p[2] - f.z_m) < f.openingR_m + r;
}

export const allianceOfNectar = (k: BallKind): Alliance | null => (k === 'nectarRed' ? 'red' : k === 'nectarBlue' ? 'blue' : null);
