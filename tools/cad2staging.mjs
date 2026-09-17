/**
 * The part of the asset pipeline that actually changes the physics: pull the 56 staged ball
 * positions straight out of cad/parts.json (they are placed at their staged positions in the
 * STEP) and write assets/staging.json in metres, world frame (Y up).
 *
 * Run: node tools/cad2staging.mjs
 */
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';

const IN_TO_M = 0.0254;
const parts = JSON.parse(readFileSync(new URL('../cad/parts.json', import.meta.url), 'utf8'));

const KIND = {
  'am-5851: Pollen': 'pollen',
  'am-5852: Red Nectar': 'nectarRed',
  'am-5852: Blue Nectar': 'nectarBlue',
};

const balls = [];
for (const p of parts) {
  const name = p.name.replace(/\s*<\d+>\s*$/, '');
  const kind = KIND[name];
  if (!kind) continue;
  balls.push({
    kind,
    pos: p.centroid_in.map((v) => +(v * IN_TO_M).toFixed(6)),
    cad_in: p.centroid_in.map((v) => +v.toFixed(3)),
  });
}

// Stable order so a reset is deterministic regardless of CAD traversal order.
balls.sort((a, b) => a.kind.localeCompare(b.kind) || a.cad_in[0] - b.cad_in[0] || a.cad_in[2] - b.cad_in[2] || a.cad_in[1] - b.cad_in[1]);

const counts = balls.reduce((m, b) => ((m[b.kind] = (m[b.kind] || 0) + 1), m), {});
if (counts.pollen !== 40 || counts.nectarRed !== 8 || counts.nectarBlue !== 8) {
  throw new Error(`expected 40/8/8, got ${JSON.stringify(counts)}`);
}

mkdirSync(new URL('../assets/', import.meta.url), { recursive: true });
writeFileSync(
  new URL('../assets/staging.json', import.meta.url),
  JSON.stringify({ _source: 'cad/parts.json centroids (the STEP places balls at their staged positions)', units: 'm, world frame, Y up', counts, balls }, null, 1),
);
console.log('assets/staging.json', counts);
