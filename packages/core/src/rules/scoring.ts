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
  flowers: { elements: number; redNectar: number; blueNectar: number; topNectar: Alliance | null }[];
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
    const p = positional(alliance, c);
    s.upCell = p.upCell;
    s.garden = p.garden;
    s.park = p.parked;
    s.flower = p.flower;
    s.bottomNectar = this.bottomNectarPerFlower ? p.bottoms : Math.min(1, p.bottoms);
    this.recompute();
  }

  /**
   * WHAT THE SCORE WOULD BE IF THE BUZZER WENT NOW, without touching the real one.
   *
   * Everything positional is assessed at the buzzer and not before, which is the rule -- and
   * which meant the scoreboard read 0 to 0 for two and a half minutes while balls piled up in
   * the CELLs. That reads as a broken scoreboard, not as a rule. The projection is the same
   * arithmetic on the same census, so it cannot disagree with the final answer; the UI labels
   * it as a projection until the clock stops.
   */
  project(alliance: Alliance, c: EndOfMatchCounts): number {
    const s = this.state[alliance];
    const p = positional(alliance, c);
    const bottoms = this.bottomNectarPerFlower ? p.bottoms : Math.min(1, p.bottoms);
    const auto = s.autoTips * 20 + (s.leave ? 3 : 0) + (s.park ? 5 : 0);
    const teleop = (s.tips - s.autoTips) * 20 + p.upCell * 2 + p.flower * 2 + p.garden + bottoms * 5
      + (p.parked ? 5 : 0);
    return auto + teleop;
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

/**
 * The positional half of the score, from one census: everything assessed by where things are
 * rather than by what happened. One function so the final answer and the live projection are
 * the same arithmetic and can never disagree.
 */
function positional(alliance: Alliance, c: EndOfMatchCounts) {
  // THE TOP-MOST NECTAR OWNS THE FLOWER, not the most of them.
  //
  // Manual 10.5.2, FLOWER Owner: "The ALLIANCE that has the TOP-MOST NECTAR of its color ...
  // owns that FLOWER." This used to compare counts, which gives the flower to whoever put
  // more in -- so an alliance that caps the opponent's three with one of its own was scored
  // as the LOSER of that flower. That made the whole endgame of STRATEGY.md section 8 --
  // plug early, cap late, saturate the tube -- invisible to the scorer, and capping looked
  // worthless. The FLOWER is a vertical tube; height is the criterion and it is the one
  // thing a late ball can still change.
  let flower = 0;
  for (const f of c.flowers) if (f.topNectar === alliance) flower += f.elements;
  return {
    upCell: c.upCell,
    garden: c.garden,
    parked: c.parked,
    flower,
    bottoms: c.bottomNectar.filter((b) => b.alliance === alliance).length,
  };
}

/**
 * Is a ball inside a FLOWER's scoring volume (between the middle and top rings)?
 *
 * ANY PART OF IT COUNTS -- manual 10.5.2, and PHYSICS_AND_SIMULATION.md 7.1 states the same
 * rule when it works out the stack capacity. This used the ball's RADIUS across the tube and
 * its CENTRE up it, which is two different rules on two axes and under-counts every flower:
 * with four POLLEN staged on the bottom ring the second ball's centre lands at 4.1-4.45 in
 * against a 4.34 in middle ring, so whether a flower scored 2 or 3 came down to a third of an
 * inch of settling noise. By the real rule its top is at 5.5 and it is unambiguously in.
 */
export function inFlowerScoringVolume(p: Vec3, f: { x_m: number; z_m: number; openingR_m: number; scoreLow_m: number; scoreHigh_m: number }, r: number): boolean {
  if (p[1] + r < f.scoreLow_m || p[1] - r > f.scoreHigh_m) return false;
  return Math.hypot(p[0] - f.x_m, p[2] - f.z_m) < f.openingR_m + r;
}

export const allianceOfNectar = (k: BallKind): Alliance | null => (k === 'nectarRed' ? 'red' : k === 'nectarBlue' ? 'blue' : null);
