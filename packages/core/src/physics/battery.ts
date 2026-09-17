/** 12 V NiMH pack. V = Voc(SoC) - Rint * sum(I);  SoC -= sum(I) dt / C. */
import type { Params } from '../types.js';

export class Battery {
  soc = 1.0;
  volts: number;
  amps = 0;
  private readonly curve: [number, number][];
  private readonly rint: number;
  private readonly capacity_As: number;

  constructor(private readonly p: Params['battery']) {
    this.curve = [...p.VocBySoC].sort((a, b) => a[0] - b[0]);
    this.rint = p.Rint_ohm;
    this.capacity_As = p.capacity_Ah * 3600;
    this.volts = this.voc(1);
  }

  private voc(soc: number): number {
    const c = this.curve;
    if (soc <= c[0][0]) return c[0][1];
    if (soc >= c[c.length - 1][0]) return c[c.length - 1][1];
    for (let i = 1; i < c.length; i++) {
      if (soc <= c[i][0]) {
        const t = (soc - c[i - 1][0]) / (c[i][0] - c[i - 1][0]);
        return c[i - 1][1] + t * (c[i][1] - c[i - 1][1]);
      }
    }
    return c[c.length - 1][1];
  }

  /** Call once per physics step with the total draw of every motor. */
  update(totalAmps: number, dt: number): void {
    this.amps = totalAmps;
    this.soc = Math.max(0, this.soc - (totalAmps * dt) / this.capacity_As);
    this.volts = Math.max(6, this.voc(this.soc) - this.rint * totalAmps);
  }

  reset(): void {
    this.soc = 1;
    this.amps = 0;
    this.volts = this.voc(1);
  }
}
