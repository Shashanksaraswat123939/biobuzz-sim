/* Draws the stats report onto a canvas and leaves the PNG in window.__png. Tables only. */
const W = 1240, PAD = 34, GAP = 18;
const c = document.getElementById('c'), g = c.getContext('2d');
const F = (s, w) => (w || 400) + ' ' + s + 'px "Segoe UI", system-ui, sans-serif';
const COL = {
  bg: '#12151a', card: '#1a1f27', line: '#2a323d', row: '#20262f', fg: '#e8ecf2',
  dim: '#8b95a5', head: '#7dd3fc', good: '#4ade80', mid: '#fbbf24', bad: '#f87171', body: '#c3ccd8',
};
function rr(x, y, w, h, r) {
  g.beginPath(); g.moveTo(x + r, y);
  g.arcTo(x + w, y, x + w, y + h, r); g.arcTo(x + w, y + h, x, y + h, r);
  g.arcTo(x, y + h, x, y, r); g.arcTo(x, y, x + w, y, r); g.closePath();
}

const cards = [
  { t: 'DOES IT HIT THE GOAL?',
    head: ['Situation', 'Balls', 'Went in'],
    rows: [['Standing still, on a green square', '90', '98%', 'good'],
           ['Rolling slowly (0.4 m/s)', '25', '92%', 'good'],
           ['Medium speed (0.6 m/s)', '26', '88%', 'good'],
           ['Fast (0.8 m/s)', '25', '84%', 'good'],
           ['Driving past at 0.95 m/s', '103', '93%', 'good'],
           ['Swinging round the hive, 0.9 m/s', '49', '67%', 'mid'],
           ['Flat out in a straight line', '0', '—', 'dim']] },

  { t: 'HOW ACCURATE, BY DISTANCE',
    head: ['Distance to goal', 'Balls', 'Went in'],
    rows: [['40 inches', '150', '85%', 'good'],
           ['50 inches', '145', '92%', 'good'],
           ['60 inches', '143', '90%', 'good'],
           ['70 inches', '144', '85%', 'good'],
           ['80 inches', '109', '94%', 'good'],
           ['Best it can ever do', '708', '94%', 'mid']] },

  { t: 'DOES IT AIM AT THE RIGHT PLACE?',
    head: ['Measure', '', 'Value'],
    rows: [['How far off it points while driving', '', '0–3°', 'good'],
           ['Time pointing badly wrong (was 12.7%)', '', '2.1%', 'good'],
           ['Spinning on the spot at 66°/s', '', '94% in', 'good'],
           ['Turret can swing', '', '1½ turns', ''],
           ['Refuses the shot above this turn rate', '', '70°/s', ''],
           ['Refuses past this angle off the opening', '', '60°', '']] },

  { t: 'HOW FAST CAN IT SHOOT?',
    head: ['Gap between balls', 'Went in', 'Scored per sec'],
    rows: [['0.5 s (below the mechanism’s limit)', '61%', '1.19', 'dim'],
           ['0.6 s  — now shipping', '55%', '0.97', 'good'],
           ['0.8 s', '56%', '0.80', ''],
           ['1.0 s  — what it was before', '63%', '0.70', 'dim'],
           ['Improvement', '', '+39%', 'good']] },

  { t: 'DOES IT KNOW WHERE IT IS?',
    head: ['Measure', '', 'Value'],
    rows: [['Position error after 30 s of driving', '', '1.1 in', 'good'],
           ['Worst it gets', '', '3.3 in', ''],
           ['Camera fixes used / thrown away', '', '405 / 0', ''],
           ['Part of the field where it sees the tag', '', '26%', 'bad']] },

  { t: 'THE GREEN MAP — STAND ON IT AND SHOOT',
    head: ['Measure', '', 'Value'],
    rows: [['Green squares we stood on', '', '24', ''],
           ['Ones it actually fired from', '', '23', 'good'],
           ['Balls fired / went in', '', '90 / 88', 'good'],
           ['Grey squares that wrongly fired', '', '0', 'good'],
           ['Size of the shooting area', '', '13 sq ft', 'mid']] },

  { t: 'PRACTICE MODES',
    head: ['Mode', '', 'Result'],
    rows: [['Opponent, whole match', '', '60 pts', 'good'],
           ['   its shots / hive tips', '', '24.5 / 2.0', 'dim'],
           ['Auto, 30 s (5 runs)', '', '8 pts', 'good'],
           ['   leaves the line / parks', '', '5/5 · 5/5', 'good'],
           ['Points from FLOWERs', '', '0', 'bad']] },

  { t: 'WHAT IS LEFT',
    head: ['Item', '', 'State'],
    rows: [['FLOWER scoring behaviour', '', 'not built', 'bad'],
           ['   …but the lob into the tube', '', 'proven', 'good'],
           ['   …pushing a ball out of the intake', '', 'impossible', 'bad'],
           ['Robot model from ROBOT_BUILD.md', '', 'in progress', 'mid'],
           ['Rest of the interface work', '', 'not started', 'bad'],
           ['Physics constants still guessed', '', '6', 'mid']] },
];

function cardH(cd) { return 16 + 22 + 22 + cd.rows.length * 24 + 10; }
const colW = (W - PAD * 2 - GAP) / 2;
const hs = cards.map(cardH);
const y0 = PAD + 40 + 26;
const colY = [y0, y0];
const place = cards.map((cd, i) => {
  const col = i % 2, y = colY[col];
  colY[col] += hs[i] + GAP;
  return { x: PAD + col * (colW + GAP), y, w: colW, h: hs[i] };
});
const H = Math.max(colY[0], colY[1]) + 32;
c.width = W; c.height = H;

g.fillStyle = COL.bg; g.fillRect(0, 0, W, H);
g.fillStyle = COL.fg; g.font = F(27, 700);
g.fillText('BIOBUZZ Simulator — how well it works right now', PAD, PAD + 24);
g.fillStyle = COL.dim; g.font = F(13);
g.fillText('Every number measured by running the simulator.   19 September 2026 · 168 tests pass · live at shashanksaraswat123939.github.io/biobuzz-simulator/', PAD, PAD + 46);

function card(cd, b) {
  g.fillStyle = COL.card; rr(b.x, b.y, b.w, b.h, 9); g.fill();
  g.strokeStyle = COL.line; g.lineWidth = 1; rr(b.x + 0.5, b.y + 0.5, b.w - 1, b.h - 1, 9); g.stroke();
  let y = b.y + 26; const x = b.x + 18;
  g.fillStyle = COL.head; g.font = F(13, 700); g.fillText(cd.t, x, y); y += 20;

  g.fillStyle = COL.dim; g.font = F(11, 600);
  g.fillText(cd.head[0].toUpperCase(), x, y);
  g.textAlign = 'right';
  if (cd.head[1]) g.fillText(cd.head[1].toUpperCase(), b.x + b.w - 112, y);
  g.fillText(cd.head[2].toUpperCase(), b.x + b.w - 18, y);
  g.textAlign = 'left';
  y += 8;
  g.strokeStyle = COL.line; g.beginPath(); g.moveTo(x, y); g.lineTo(b.x + b.w - 18, y); g.stroke();

  cd.rows.forEach((r, i) => {
    y += 24;
    g.fillStyle = r[3] === 'dim' ? COL.dim : COL.body; g.font = F(13.5); g.fillText(r[0], x, y);
    g.textAlign = 'right'; g.font = F(13.5, 600);
    if (r[1]) { g.fillStyle = r[3] === 'dim' ? COL.dim : COL.fg; g.fillText(r[1], b.x + b.w - 112, y); }
    g.fillStyle = r[3] === 'good' ? COL.good : r[3] === 'mid' ? COL.mid : r[3] === 'bad' ? COL.bad : r[3] === 'dim' ? COL.dim : COL.fg;
    g.fillText(r[2], b.x + b.w - 18, y);
    g.textAlign = 'left';
    if (i < cd.rows.length - 1) {
      g.strokeStyle = COL.row; g.beginPath(); g.moveTo(x, y + 7); g.lineTo(b.x + b.w - 18, y + 7); g.stroke();
    }
  });
}
cards.forEach((cd, i) => card(cd, place[i]));

window.__png = c.toDataURL('image/png');
document.title = 'ready ' + window.__png.length;
