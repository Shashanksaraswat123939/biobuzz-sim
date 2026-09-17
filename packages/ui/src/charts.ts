/**
 * The three plots that say something a table of means cannot.
 *
 * Canvas 2D and no dependency: three charts do not justify a charting library, and the
 * axes here have to be in inches with a fixed aspect for the group plot, which most
 * libraries fight you about anyway.
 *
 * All three read the same ShotRecord list the report does, so a dot on the plot and a row
 * in the table are the same shot.
 */
import type { ShotRecord } from '@core/types.js';

const INK = '#e6e9ed';
const DIM = '#94a0b0';
const LINE = '#3d4653';
const GOOD = '#4a9e5c';
const BAD = '#d99a2b';
const ACCENT = '#f5821f';
const FONT = '11px ui-sans-serif, system-ui, "Segoe UI", sans-serif';

/** Set the backing store to the CSS size so nothing is blurry on a HiDPI screen. */
function fit(cv: HTMLCanvasElement): CanvasRenderingContext2D {
  const dpr = Math.min(devicePixelRatio || 1, 2);
  const w = cv.clientWidth || cv.width;
  const h = Number(cv.getAttribute('height')) || 300;
  cv.width = Math.round(w * dpr);
  cv.height = Math.round(h * dpr);
  cv.style.height = `${h}px`;
  const g = cv.getContext('2d')!;
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  g.font = FONT;
  return g;
}

function empty(g: CanvasRenderingContext2D, w: number, h: number): void {
  g.fillStyle = DIM;
  g.textAlign = 'center';
  g.fillText('no finished shots yet', w / 2, h / 2);
}

/**
 * Where the shots arrived, in the goal's own frame: x is left/right of the shot line,
 * y is long/short. Equal scale on both axes, because a group plot with stretched axes
 * makes a round group look like a line and tells you to fix the wrong thing.
 */
export function groupPlot(cv: HTMLCanvasElement, shots: ShotRecord[], mouthR_in: number): void {
  const w = cv.clientWidth || 320;
  const h = Number(cv.getAttribute('height')) || 400;
  const g = fit(cv);
  const done = shots.filter((s) => s.result !== 'flight' && Number.isFinite(s.long_in) && Number.isFinite(s.lat_in));
  if (!done.length) return empty(g, w, h);

  const pad = 26;
  const reach = Math.max(mouthR_in * 2.5, ...done.map((s) => Math.max(Math.abs(s.long_in), Math.abs(s.lat_in))));
  const lim = Math.ceil(reach / 10) * 10;
  const scale = Math.min((w - pad * 2) / (lim * 2), (h - pad * 2) / (lim * 2));
  const cx = w / 2;
  const cy = h / 2;
  const X = (v: number) => cx + v * scale;
  const Y = (v: number) => cy - v * scale;

  // grid every 12 in, which is a tile
  g.strokeStyle = LINE;
  g.lineWidth = 1;
  g.fillStyle = DIM;
  g.textAlign = 'center';
  for (let v = -lim; v <= lim; v += 12) {
    g.globalAlpha = v === 0 ? 1 : 0.4;
    g.beginPath();
    g.moveTo(X(v), Y(-lim)); g.lineTo(X(v), Y(lim));
    g.moveTo(X(-lim), Y(v)); g.lineTo(X(lim), Y(v));
    g.stroke();
  }
  g.globalAlpha = 1;

  // the CELL mouth
  g.strokeStyle = ACCENT;
  g.lineWidth = 1.5;
  g.beginPath();
  g.arc(cx, cy, mouthR_in * scale, 0, Math.PI * 2);
  g.stroke();

  for (const s of done) {
    const hit = s.result === 'cell';
    g.beginPath();
    g.arc(X(s.lat_in), Y(s.long_in), 3.2, 0, Math.PI * 2);
    g.fillStyle = hit ? GOOD : 'transparent';
    g.strokeStyle = hit ? GOOD : BAD;
    g.lineWidth = 1.4;
    if (hit) g.fill();
    g.stroke();
  }

  // the group's own centre and 1-sigma ellipse: the bias and the spread, drawn
  const mx = done.reduce((a, s) => a + s.lat_in, 0) / done.length;
  const my = done.reduce((a, s) => a + s.long_in, 0) / done.length;
  if (done.length > 2) {
    const sx = Math.sqrt(done.reduce((a, s) => a + (s.lat_in - mx) ** 2, 0) / (done.length - 1));
    const sy = Math.sqrt(done.reduce((a, s) => a + (s.long_in - my) ** 2, 0) / (done.length - 1));
    g.strokeStyle = INK;
    g.globalAlpha = 0.5;
    g.setLineDash([4, 3]);
    g.beginPath();
    g.ellipse(X(mx), Y(my), sx * scale, sy * scale, 0, 0, Math.PI * 2);
    g.stroke();
    g.setLineDash([]);
    g.globalAlpha = 1;
  }
  g.strokeStyle = INK;
  g.lineWidth = 1.2;
  g.beginPath();
  g.moveTo(X(mx) - 6, Y(my)); g.lineTo(X(mx) + 6, Y(my));
  g.moveTo(X(mx), Y(my) - 6); g.lineTo(X(mx), Y(my) + 6);
  g.stroke();

  g.fillStyle = DIM;
  g.textAlign = 'left';
  g.fillText('left', 4, cy - 5);
  g.textAlign = 'right';
  g.fillText('right', w - 4, cy - 5);
  g.textAlign = 'center';
  g.fillText(`long +${lim}"`, cx, 12);
  g.fillText(`short -${lim}"`, cx, h - 4);
}

/** Downrange error against the range it was fired from: does the table fail at one end? */
export function errorVsRange(cv: HTMLCanvasElement, shots: ShotRecord[]): void {
  const w = cv.clientWidth || 320;
  const h = Number(cv.getAttribute('height')) || 300;
  const g = fit(cv);
  const done = shots.filter((s) => s.result !== 'flight' && Number.isFinite(s.long_in));
  if (!done.length) return empty(g, w, h);

  const padL = 34;
  const padB = 20;
  const padT = 10;
  const rLo = Math.min(...done.map((s) => s.rangeIn)) - 4;
  const rHi = Math.max(...done.map((s) => s.rangeIn)) + 4;
  const eLim = Math.max(12, ...done.map((s) => Math.abs(s.long_in))) * 1.1;
  const X = (r: number) => padL + ((r - rLo) / (rHi - rLo || 1)) * (w - padL - 6);
  const Y = (e: number) => padT + (1 - (e + eLim) / (2 * eLim)) * (h - padT - padB);

  g.strokeStyle = LINE;
  g.lineWidth = 1;
  g.fillStyle = DIM;
  for (const e of [-eLim, -eLim / 2, 0, eLim / 2, eLim]) {
    g.globalAlpha = e === 0 ? 1 : 0.4;
    g.beginPath();
    g.moveTo(padL, Y(e));
    g.lineTo(w - 6, Y(e));
    g.stroke();
    g.globalAlpha = 1;
    g.textAlign = 'right';
    g.fillText(`${e >= 0 ? '+' : ''}${e.toFixed(0)}"`, padL - 4, Y(e) + 3.5);
  }
  g.textAlign = 'center';
  g.fillText(`${rLo.toFixed(0)} in`, padL + 14, h - 5);
  g.fillText(`${rHi.toFixed(0)} in`, w - 20, h - 5);

  for (const s of done) {
    g.beginPath();
    g.arc(X(s.rangeIn), Y(Math.max(-eLim, Math.min(eLim, s.long_in))), 3, 0, Math.PI * 2);
    const hit = s.result === 'cell';
    g.fillStyle = hit ? GOOD : 'transparent';
    g.strokeStyle = hit ? GOOD : BAD;
    g.lineWidth = 1.3;
    if (hit) g.fill();
    g.stroke();
  }

  // Least-squares fit. A visible slope is the table being wrong at one end of the band,
  // which is a different fix from a constant offset.
  if (done.length > 3) {
    const n = done.length;
    const sx = done.reduce((a, s) => a + s.rangeIn, 0) / n;
    const sy = done.reduce((a, s) => a + s.long_in, 0) / n;
    const num = done.reduce((a, s) => a + (s.rangeIn - sx) * (s.long_in - sy), 0);
    const den = done.reduce((a, s) => a + (s.rangeIn - sx) ** 2, 0);
    if (den > 1e-6) {
      const m = num / den;
      const c = sy - m * sx;
      g.strokeStyle = ACCENT;
      g.lineWidth = 1.5;
      g.beginPath();
      g.moveTo(X(rLo), Y(Math.max(-eLim, Math.min(eLim, m * rLo + c))));
      g.lineTo(X(rHi), Y(Math.max(-eLim, Math.min(eLim, m * rHi + c))));
      g.stroke();
      g.fillStyle = ACCENT;
      g.textAlign = 'left';
      g.fillText(`${m >= 0 ? '+' : ''}${(m * 12).toFixed(1)} in of error per foot of range`, padL + 4, padT + 11);
    }
  }
}

/** Histogram of the downrange error, with the mean marked. */
export function histogram(cv: HTMLCanvasElement, shots: ShotRecord[]): void {
  const w = cv.clientWidth || 320;
  const h = Number(cv.getAttribute('height')) || 220;
  const g = fit(cv);
  const vals = shots.filter((s) => s.result !== 'flight').map((s) => s.long_in).filter(Number.isFinite);
  if (vals.length < 2) return empty(g, w, h);

  const lim = Math.max(12, ...vals.map(Math.abs));
  const bins = 21;
  const counts = new Array(bins).fill(0);
  for (const v of vals) {
    const i = Math.min(bins - 1, Math.max(0, Math.floor(((v + lim) / (2 * lim)) * bins)));
    counts[i]++;
  }
  const peak = Math.max(...counts);
  const padB = 18;
  const bw = (w - 8) / bins;
  for (let i = 0; i < bins; i++) {
    const bh = (counts[i] / peak) * (h - padB - 8);
    g.fillStyle = ACCENT;
    g.globalAlpha = counts[i] ? 0.85 : 0;
    g.fillRect(4 + i * bw + 1, h - padB - bh, bw - 2, bh);
  }
  g.globalAlpha = 1;

  const mean = vals.reduce((a, v) => a + v, 0) / vals.length;
  const mx = 4 + ((mean + lim) / (2 * lim)) * (w - 8);
  g.strokeStyle = INK;
  g.lineWidth = 1.5;
  g.beginPath();
  g.moveTo(mx, 4);
  g.lineTo(mx, h - padB);
  g.stroke();

  g.strokeStyle = LINE;
  g.beginPath();
  g.moveTo(4, h - padB);
  g.lineTo(w - 4, h - padB);
  g.stroke();

  g.fillStyle = DIM;
  g.textAlign = 'left';
  g.fillText(`short -${lim.toFixed(0)}"`, 4, h - 5);
  g.textAlign = 'right';
  g.fillText(`long +${lim.toFixed(0)}"`, w - 4, h - 5);
  g.textAlign = 'center';
  g.fillStyle = INK;
  g.fillText(`mean ${mean >= 0 ? '+' : ''}${mean.toFixed(1)}"`, Math.max(40, Math.min(w - 40, mx)), h - 5);
}
