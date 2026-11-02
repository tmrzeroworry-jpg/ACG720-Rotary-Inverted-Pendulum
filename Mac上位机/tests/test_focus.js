const fs = require('fs'),
  vm = require('vm'),
  assert = require('assert'),
  path = require('path');
const app = path.resolve(__dirname, '..'),
  source = fs.readFileSync(app + '/web/app.js', 'utf8');
const c = vm.createContext({});
vm.runInContext(
  source.slice(source.indexOf('function traceBounds('), source.indexOf('// END CONTINUOUS TRACE')) +
    source.slice(source.indexOf('// BEGIN VISUAL FOCUS'), source.indexOf('// END VISUAL FOCUS')) +
    ';this.bounds=focusedArmBounds;this.direction=armScreenDirection;this.full=traceBounds;',
  c,
);
assert(Math.abs(c.direction(90).x) < 1e-12);
assert.equal(c.direction(90).y, 1);
assert.equal(c.direction(0).x, 1);
assert.equal(c.direction(270).y, -1);
const wave = [];
for (let i = 0; i <= 2000; i++) {
  const t = i / 100,
    angle = t < 6 ? 450 - t * 200 : -750 + 45 * Math.sin((2 * Math.PI * t) / 4);
  wave.push({ t, trace_key: 'trial1', arm_trace: angle });
}
const copy = JSON.stringify(wave),
  b = c.bounds(wave, 20),
  full = c.full(wave.map((p) => p.arm_trace));
assert.equal(JSON.stringify(wave), copy);
assert(b[1] - b[0] < (full[1] - full[0]) * 0.2);
for (const p of wave.filter((p) => p.t >= 12)) assert(p.arm_trace >= b[0] && p.arm_trace <= b[1]);
const still = c.bounds([{ t: 20, trace_key: 'trial1', arm_trace: 123 }], 20);
assert(still[0] < 123 && still[1] > 123 && still[1] - still[0] >= 10);
assert(c.bounds([], 20).every(Number.isFinite));
assert(c.bounds([{ t: 0, trace_key: 'a', arm_trace: 3 }], 20).every(Number.isFinite));
const reset = c.bounds([...wave, { t: 20, trace_key: 'trial2', arm_trace: 0 }], 20);
assert(reset[0] > -100 && reset[1] < 100);
assert(source.replace(/\s+/g, '').includes('if(j===2)text(ctx,fmt(start'));
console.log(
  'PASS: clockwise pose, recent8s focus, no sample mutation, current peaks retained, idle/missing/trial-reset cases, speed row retained.',
);
console.log('Synthetic regression axis:', full, '=>', b);
