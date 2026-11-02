const fs = require('fs'),
  vm = require('vm'),
  assert = require('assert'),
  path = require('path');
const near = (a, b) => assert(Math.abs(a - b) < 1e-9, `${a} != ${b}`);
const source = fs.readFileSync(path.resolve(__dirname, '../web/app.js'), 'utf8'),
  c = vm.createContext({});
vm.runInContext(
  source.slice(source.indexOf('// BEGIN DISPLAY MATH'), source.indexOf('// END DISPLAY MATH')) +
    ';let displayCalibration={up:6251,down:20039};' +
    source.slice(source.indexOf('function displayFrame('), source.search(/const \$\s*=/)) +
    ';this.view=displayFrame;',
  c,
);
vm.runInContext(
  source.slice(
    source.indexOf('// BEGIN CONTINUOUS TRACE'),
    source.indexOf('// END CONTINUOUS TRACE'),
  ) + ';this.trace=continuousArm;',
  c,
);
const f = {
  build: 'TR01',
  protocol_version: 5,
  epoch: 0,
  trial: 1,
  pose_valid: true,
  raw: 6251,
  pole: 0,
  encoder: -286,
  arm: -99,
  arm_home_counts: -286,
  arm_home_valid: true,
  target_counts: -286,
  target: -99,
  counts_per_rev: 1040,
};
const shift = vm.runInContext('DisplayMath.armOffsetDeg', c);
assert(shift > 18 && shift < 20);
const raw = JSON.stringify(f),
  v = c.view(f);
near(v.arm, shift);
assert.equal(v.encoder_mod, 0);
assert.equal(JSON.stringify(f), raw);
near(c.view({ ...f, encoder: -286 + 260 }).arm, 90 + shift);
near(c.view({ ...f, encoder: -286 - 260 }).arm, 270 + shift);
assert(Math.abs(c.view({ ...f, encoder: -286 + 1040 * 7 }).arm - shift) < 1e-9);
near(c.view({ ...f, target_counts: 100000 }).arm, shift); // Moving task target must never change the origin.
assert.equal(c.view({ ...f, arm_home_valid: false }).arm, null);
const values = c.trace([
  { ...f, encoder: -26 },
  { ...f, trial: 2, encoder: -26 },
  { ...f, trial: 2, encoder: 1014 },
]);
assert.equal(values[0].arm_trace, 90 + shift);
assert.equal(values[1].arm_trace, 90 + shift);
assert.equal(values[2].arm_trace, 450 + shift);
const meanFrame = { ...f, encoder: f.arm_home_counts - (shift * 1040) / 360 };
const centered = c.view(meanFrame).arm;
assert(centered < 1e-9 || centered > 360 - 1e-9);
const screenshotFrame = c.view({ ...f, encoder: -174, arm_home_counts: -118 });
assert(Math.abs(screenshotFrame.arm - 359.5357619502) < 1e-8);
assert.equal(screenshotFrame.encoder, -174);
assert.equal(screenshotFrame.encoder_mod, 984);
assert.equal(c.trace([{ ...f, arm_home_valid: false }])[0].arm_trace, null);
const legacy = c.view({ ...f, protocol_version: 3 });
assert.equal(legacy.arm, (754 * 360) / 1040);
console.log(
  'PASS HOME: fixed display bias centers the measured mean, screenshot340.6 becomes359.5, raw counts retained, ±quarter-turn, multi-turn continuity, task target independence, trial stability, invalid-home hidden, legacy unchanged, raw immutable.',
);

// Manual zero marks the measured position, overrides the fixed bias, and never transmits.
vm.runInContext("DisplayMath.setArmScope('live:record_a')", c);
const current = { ...f, sensor_ok: true, encoder: -174, arm_home_counts: -118 };
const saved = vm.runInContext('DisplayMath.markArmZero', c)(current);
near(c.view(current).arm, 0);
assert.equal(c.view(current).encoder_mod, 0);
near(c.view({ ...current, encoder: current.encoder + 260 }).arm, 90);
near(c.view({ ...current, trial: 99, encoder: current.encoder - 260 }).arm, 270);
assert.equal(c.view(current).arm_display_offset_deg, 0);
assert.equal(c.view(current).encoder, -174); // Received raw count is unchanged.
near(c.trace([current])[0].arm_trace, 0);
near(c.view({ ...current, arm_home_valid: false }).arm, 0);
assert.equal(c.view({ ...current, epoch: 1 }).arm_display_manual, false);
vm.runInContext("DisplayMath.setArmScope('live:record_b')", c);
assert.equal(c.view(current).arm_display_manual, false);
vm.runInContext("DisplayMath.setArmScope('live:record_a')", c);
vm.runInContext('DisplayMath.restoreArmZero', c)(JSON.parse(JSON.stringify(saved)));
near(c.view(current).arm, 0);
assert.equal(vm.runInContext('DisplayMath.markArmZero', c)({ ...current, sensor_ok: false }), null);
console.log(
  'PASS RESET: current=0; ±quarter turn, manual bias override, zero continuous displacement, refresh persistence, trial retention, session/reboot isolation, no raw mutation.',
);
