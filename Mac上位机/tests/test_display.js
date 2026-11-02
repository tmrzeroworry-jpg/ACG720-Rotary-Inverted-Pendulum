const fs = require('fs'),
  vm = require('vm'),
  assert = require('assert');
const app = require('path').resolve(__dirname, '..');
const source = fs.readFileSync(app + '/web/app.js', 'utf8');
const start = source.indexOf('// BEGIN DISPLAY MATH'),
  end = source.indexOf('// END DISPLAY MATH');
const context = vm.createContext({});
vm.runInContext(source.slice(start, end) + ';this.math=DisplayMath;', context);
const m = context.math,
  c = { up: 6251, down: 20039 };
assert(m.valid(c));
assert(!m.valid({ up: 6251, down: 6300 }));
assert(!m.valid({ up: 0, down: NaN }));
assert.equal(m.pole(c.up, c).inclination, 0);
assert.equal(m.pole(c.down, c).inclination, 180);
assert.equal(m.pole((c.up + c.down) / 2, c).inclination, 90);
assert.equal(m.pole(((c.up + c.down + 26400) / 2) % 26400, c).inclination, 90);
assert.equal(m.pole(c.up + 26400, c).inclination, 0);
assert(Math.abs(m.pole(c.down - 20, c).inclination - 180) < 0.3);
assert(Math.abs(m.pole(c.down + 20, c).inclination - 180) < 0.3);
for (let x = -52800; x <= 52800; x += 73) {
  const p = m.pole(x, c);
  assert(p.inclination >= 0 && p.inclination <= 180);
  assert(p.phase >= 0 && p.phase < 360);
}
assert.equal(m.arm(-10400, 1040).count, 0);
assert.equal(m.arm(-1, 1040).count, 1039);
assert.equal(m.arm(39494, 1040).count, 1014);
assert.equal(m.arm(1040 * 100000 + 130, 1040).angle, 45);
assert.equal(m.difference(359, 1), -2);
assert.equal(m.difference(1, 359), 2);
assert.equal(m.arm(null, 1040), null);
assert.equal(m.pole(0, { up: 0, down: 0 }), null);
// Execute the production view mapper; underlying received data must not mutate.
vm.runInContext(
  'let displayCalibration={up:6251,down:20039};' +
    source.slice(source.indexOf('function displayFrame('), source.search(/const \$\s*=/)) +
    ';this.view=displayFrame;',
  context,
);
const f = {
  build: 'WEXF',
  pose_valid: true,
  raw: 20039,
  pole: -172,
  encoder: 39494,
  counts_per_rev: 1040,
  target_counts: -169,
  arm: -9,
  target: -58.5,
  arm_error: 13729.5,
};
const before = JSON.stringify(f),
  v = context.view(f);
assert.equal(JSON.stringify(f), before);
assert.equal(v.pole, 180);
assert.equal(v.control_pole, -172);
assert.equal(v.arm, 351);
assert.equal(v.target, 301.5);
assert.equal(v.arm_error, 49.5);
assert.equal(v.encoder_mod, 1014);
assert.equal(v.target_mod, 871);
const h = { ...f, build: 'WEXH' },
  hv = context.view(h);
assert.equal(hv.pole, 180);
assert.equal(hv.arm, 351);
assert.equal(hv.encoder_mod, 1014);
assert.equal(h.pole, -172);

assert(!source.includes('let t=f.target*Math.PI/180'));
assert(!source.includes("armTrack?'arm_unwrapped'"));
console.log(
  'PASS: display anchors, both arcs, modulo/negative/multi-turn counts, circular error, immutable raw frames, no pose target.',
);

vm.runInContext(
  source.slice(
    source.indexOf('// BEGIN CONTINUOUS TRACE'),
    source.indexOf('// END CONTINUOUS TRACE'),
  ) + ';this.trace=continuousArm;this.bounds=traceBounds;',
  context,
);
const wave = [];
for (let i = 0; i <= 800; i++) {
  const angle = 350 + 115 * Math.sin((i / 800) * Math.PI * 8);
  wave.push({
    build: 'WEXF',
    epoch: 0,
    trial: 1,
    counts_per_rev: 1040,
    encoder: (angle * 1040) / 360,
    arm: angle % 360,
    target: null,
    target_counts: null,
    pose_valid: true,
    raw: 6251,
    pole: 0,
    t: i * 0.01,
    arm_rate: 0,
  });
}
const copy = JSON.stringify(wave),
  trace = context.trace(wave);
assert.equal(JSON.stringify(wave), copy);
assert(Math.max(...trace.slice(1).map((p, i) => Math.abs(p.arm_trace - trace[i].arm_trace))) < 4);
const values = trace.map((p) => p.arm_trace),
  range = context.bounds(values);
assert(range[0] < Math.min(...values) && range[1] > Math.max(...values));
const later = context.trace(wave.slice(100));
assert.equal(later[0].arm_trace, trace[100].arm_trace);
const spin = context.trace([
  { ...wave[0], trial: 2, encoder: -100 * 1040 },
  { ...wave[0], trial: 2, encoder: -97 * 1040 },
]);
assert.equal(spin[1].arm_trace - spin[0].arm_trace, 1080);
console.log(
  'PASS: continuous sine across0/360, auto-axis contains extrema, anchor persists, negative/multi-turn data remain real.',
);
