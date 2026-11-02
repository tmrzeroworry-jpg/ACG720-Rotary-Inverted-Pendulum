'use strict';

// BEGIN DISPLAY MATH: UI calibration only; never changes telemetry or controls.
const DisplayMath = (() => {
  const mod = (x, n) => ((x % n) + n) % n;
  function valid(c) {
    return (
      c &&
      Number.isFinite(c.up) &&
      Number.isFinite(c.down) &&
      c.up >= 0 &&
      c.up < 26400 &&
      c.down >= 0 &&
      c.down < 26400 &&
      mod(c.down - c.up, 26400) >= 6600 &&
      mod(c.down - c.up, 26400) <= 19800
    );
  }
  function pole(raw, c) {
    if (!valid(c) || !Number.isFinite(raw)) return null;
    const x = mod(raw - c.up, 26400),
      d = mod(c.down - c.up, 26400);
    const theta = x <= d ? (180 * x) / d : 180 + (180 * (x - d)) / (26400 - d);
    return { phase: theta, inclination: Math.min(theta, 360 - theta) };
  }
  function arm(count, rev) {
    return Number.isFinite(count) && Number.isFinite(rev) && rev > 0
      ? { count: mod(count, rev), angle: (mod(count, rev) * 360) / rev }
      : null;
  }
  const difference = (a, b) => mod(a - b + 180, 360) - 180;
  let manualZero = null,
    armScope = '';
  function setArmScope(scope) {
    armScope = String(scope);
  }
  function restoreArmZero(z) {
    if (
      z &&
      typeof z.scope === 'string' &&
      typeof z.build === 'string' &&
      Number.isInteger(z.epoch) &&
      Number.isFinite(z.encoder) &&
      Number.isFinite(z.rev) &&
      z.rev > 0
    )
      manualZero = {
        scope: z.scope,
        build: z.build,
        epoch: z.epoch,
        encoder: z.encoder,
        rev: z.rev,
      };
  }
  function armZero(frame) {
    return manualZero &&
      manualZero.scope === armScope &&
      manualZero.build === frame.build &&
      manualZero.epoch === frame.epoch &&
      manualZero.rev === frame.counts_per_rev
      ? manualZero.encoder
      : null;
  }
  function markArmZero(frame) {
    if (
      !frame ||
      !frame.sensor_ok ||
      !Number.isFinite(frame.encoder) ||
      !Number.isFinite(frame.counts_per_rev) ||
      frame.counts_per_rev <= 0
    )
      return null;
    manualZero = {
      scope: armScope,
      build: frame.build,
      epoch: frame.epoch,
      encoder: frame.encoder,
      rev: frame.counts_per_rev,
    };
    return { ...manualZero };
  }
  return {
    mod,
    valid,
    pole,
    arm,
    difference,
    armOffsetDeg: 18.920377334777,
    setArmScope,
    restoreArmZero,
    armZero,
    markArmZero,
  };
})();
// END DISPLAY MATH
// Down anchor: mean of the user's confirmed static-down record (2026-10-19).
// Up anchor: existing firmware reference, not an independently measured vertical.
const displayDefaults = { up: 6251, down: 20039, upMarked: false, downMarked: false };
const calibrationKey = 'WEXF_display_calibration_v2';
let displayCalibration = { ...displayDefaults };
try {
  const saved = JSON.parse(localStorage.getItem(calibrationKey));
  if (DisplayMath.valid(saved)) displayCalibration = saved;
} catch (e) {}
const armZeroStorageKey = 'WEXH_manual_arm_zero_v1';
try {
  DisplayMath.restoreArmZero(JSON.parse(localStorage.getItem(armZeroStorageKey)));
} catch (e) {}
function displayFrame(frame) {
  if (!frame) return null;
  const p =
    ['WEXF', 'WEXH', 'TR01', 'TR02', 'TR03', 'TR04', 'TR05', 'TR06', 'UI01', 'UI02'].includes(
      frame.build,
    ) && frame.pose_valid
      ? DisplayMath.pole(frame.raw, displayCalibration)
      : null;
  const hasHome =
    ['WEXH', 'TR01', 'TR02', 'TR03', 'TR04', 'TR05', 'TR06', 'UI01', 'UI02'].includes(
      frame.build,
    ) && frame.protocol_version >= 4;
  const manualOrigin = DisplayMath.armZero(frame);
  const ready =
    manualOrigin !== null ||
    !hasHome ||
    (frame.arm_home_valid && Number.isFinite(frame.arm_home_counts));
  const origin =
    manualOrigin !== null ? manualOrigin : hasHome && ready ? frame.arm_home_counts : 0;
  const a = ready ? DisplayMath.arm(frame.encoder - origin, frame.counts_per_rev) : null,
    t = ready ? DisplayMath.arm(frame.target_counts - origin, frame.counts_per_rev) : null;
  const shift = manualOrigin !== null ? 0 : hasHome && ready ? DisplayMath.armOffsetDeg : 0;
  if (a) a.angle = DisplayMath.mod(a.angle + shift, 360);
  if (t) t.angle = DisplayMath.mod(t.angle + shift, 360);
  return {
    ...frame,
    arm_display_manual: manualOrigin !== null,
    arm_display_offset_deg: shift,
    arm_display_origin: ready ? origin : null,
    control_pole: frame.pole,
    pole: p ? p.inclination : frame.pole === null ? null : Math.abs(frame.pole),
    pole_phase: p ? p.phase : frame.pole,
    arm: frame.arm === null ? null : (a?.angle ?? null),
    target: frame.target === null ? null : (t?.angle ?? null),
    arm_error: a && t ? DisplayMath.difference(a.angle, t.angle) : null,
    encoder_mod: a?.count ?? null,
    target_mod: t?.count ?? null,
  };
}
const $ = (id) => document.getElementById(id);
let snapshot = null,
  latest = null,
  points = [],
  cursor = 0,
  generation = -1,
  busy = false,
  dragging = false,
  frozen = false,
  frozenPoints = [],
  poleZoom = false,
  armFocus = true,
  hoverT = null,
  disconnected = false,
  initializedSelection = false;
const C = {
  green: '#66e2b4',
  cyan: '#53d5e8',
  amber: '#edb76b',
  muted: '#8199b5',
  line: '#26364d',
  red: '#fb8e9b',
};
const fmt = (v, n = 1) =>
  v === null || v === undefined || !Number.isFinite(v) ? '—' : Number(v).toFixed(n);
function timeFmt(v) {
  return `${Math.floor(v / 60)
    .toString()
    .padStart(2, '0')}:${Math.floor(v % 60)
    .toString()
    .padStart(2, '0')}`;
}
function showError(message) {
  $('error').textContent = message;
  $('error').classList.toggle('hidden', !message);
}
async function api(path, data) {
  let r = await fetch(
    path,
    data === undefined
      ? {}
      : {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(data),
        },
  );
  let value = await r.json();
  if (!r.ok) throw Error(value.error || '请求失败');
  return value;
}
async function action(fn) {
  if (busy) return;
  busy = true;
  document.querySelectorAll('.connection button').forEach((x) => (x.disabled = true));
  try {
    showError('');
    await fn();
  } catch (e) {
    showError(e.message);
  } finally {
    busy = false;
    document.querySelectorAll('.connection button').forEach((x) => (x.disabled = false));
  }
}
async function catalog() {
  const records = await api('/api/catalog'),
    sel = $('recordSelect'),
    old = sel.value;
  sel.replaceChildren();
  for (const r of records) {
    const opt = document.createElement('option');
    opt.value = r.id;
    opt.textContent = `${r.label} · ${Math.round(r.duration_s)}秒`;
    sel.append(opt);
  }
  if (records.some((x) => x.id === old)) sel.value = old;
}
async function refreshPorts() {
  const list = await api('/api/ports'),
    sel = $('portSelect'),
    old = sel.value;
  sel.replaceChildren();
  if (!list.length) {
    const opt = document.createElement('option');
    opt.value = '';
    opt.textContent = '未检测到板子，请接FPGA电源和USB';
    sel.append(opt);
  } else
    for (const p of list) {
      let opt = document.createElement('option');
      opt.value = p;
      opt.textContent = p;
      sel.append(opt);
    }
  if (list.includes(old)) sel.value = old;
}
function sourceSelection() {
  const live = $('sourceMode').value === 'live';
  for (const id of ['serialSelectBox', 'baudBox', 'refresh', 'connect', 'disconnect'])
    $(id).classList.toggle('hidden', !live);
  for (const id of ['replaySelectBox', 'loadRecord']) $(id).classList.toggle('hidden', live);
  $('importFile').parentElement.classList.toggle('hidden', live);
}
$('sourceMode').onchange = () => {
  sourceSelection();
  if ($('sourceMode').value === 'live') action(refreshPorts);
};
$('refresh').onclick = () => action(refreshPorts);
$('connect').onclick = () =>
  action(async () => {
    if (!$('portSelect').value) throw Error('没有可用USB串口。当前可继续回放实测记录。');
    await api('/api/connect', { port: $('portSelect').value, baud: Number($('baudSelect').value) });
    cursor = 0;
    generation = -1;
    points = [];
  });
$('disconnect').onclick = () =>
  action(async () => {
    await api('/api/disconnect', {});
    cursor = 0;
    generation = -1;
    points = [];
  });
$('loadRecord').onclick = () =>
  action(async () => {
    await api('/api/replay/load', { id: $('recordSelect').value });
    await api('/api/replay/action', { action: 'play' });
    cursor = 0;
    generation = -1;
    points = [];
  });
$('play').onclick = () =>
  action(() => api('/api/replay/action', { action: snapshot?.replay.playing ? 'pause' : 'play' }));
$('restart').onclick = () =>
  action(() => api('/api/replay/action', { action: 'seek', position: 0 }));
$('speed').onchange = () =>
  action(() => api('/api/replay/action', { action: 'speed', speed: Number($('speed').value) }));
$('seek').oninput = () => {
  dragging = true;
  $('replayTime').textContent = timeFmt(Number($('seek').value));
};
$('seek').onchange = () =>
  action(async () => {
    await api('/api/replay/action', { action: 'seek', position: Number($('seek').value) });
    dragging = false;
    cursor = 0;
    generation = -1;
    points = [];
  });
$('importFile').onchange = () =>
  action(async () => {
    const f = $('importFile').files[0];
    if (!f) return;
    if (f.size > 32 * 1024 * 1024) throw Error('单文件上限32MB。');
    const r = await fetch('/api/import?name=' + encodeURIComponent(f.name), {
      method: 'POST',
      headers: { 'Content-Type': 'application/octet-stream' },
      body: f,
    });
    const data = await r.json();
    if (!r.ok) throw Error(data.error);
    await catalog();
    $('recordSelect').value = data.id;
    await api('/api/replay/load', { id: data.id });
    cursor = 0;
    generation = -1;
    points = [];
    $('importFile').value = '';
  });
$('freeze').onclick = () => {
  frozen = !frozen;
  if (frozen) frozenPoints = points.slice();
  $('freeze').textContent = frozen ? '继续图表' : '冻结图表';
  if (snapshot) updateView(snapshot);
  else drawCharts();
};
$('zoomPole').onclick = () => {
  poleZoom = !poleZoom;
  $('zoomPole').textContent = poleZoom ? '摆角 自动' : '摆角 0–15°';
  drawCharts();
};
$('window').onchange = drawCharts;
$('zoomArm').onclick = () => {
  armFocus = !armFocus;
  $('zoomArm').textContent = armFocus ? '横臂：动态放大' : '横臂：完整范围';
  drawCharts();
};
$('resetArm').onclick = () => {
  if (!snapshot || disconnected || (snapshot.mode === 'live' && snapshot.stale)) return;
  const zero = DisplayMath.markArmZero(latest);
  if (!zero) return;
  try {
    localStorage.setItem(armZeroStorageKey, JSON.stringify(zero));
  } catch (e) {}
  traceAnchors.clear();
  hoverT = null;
  updateView(snapshot);
};
$('help').onclick = () => $('helpDialog').showModal();
$('closeHelp').onclick = () => $('helpDialog').close();
$('fullscreen').onclick = () => {
  document.body.classList.toggle('presentation');
  $('fullscreen').textContent = document.body.classList.contains('presentation')
    ? '⛶ 退出展示'
    : '⛶ 展示';
  requestAnimationFrame(renderCanvases);
};
function updateView(s) {
  snapshot = s;
  latest = s.latest;
  DisplayMath.setArmScope(
    s.mode === 'live'
      ? 'live:' + s.recording
      : s.mode === 'replay'
        ? 'replay:' + s.replay.id
        : 'idle',
  );
  const replay = s.mode === 'replay',
    live = s.mode === 'live',
    simulation = (s.record_kind || '').includes('仿真'),
    f = displayFrame(latest);
  if (!initializedSelection) {
    initializedSelection = true;
    $('sourceMode').value = replay ? 'replay' : 'live';
    if (live && s.baud) $('baudSelect').value = s.baud;
    if (replay && s.replay.id) $('recordSelect').value = s.replay.id;
    sourceSelection();
  }
  const stale = live && s.stale,
    valid = f && !stale && !disconnected;
  $('resetArm').disabled = !(
    valid &&
    f.sensor_ok &&
    Number.isFinite(f.encoder) &&
    f.counts_per_rev > 0
  );
  $('modeBadge').textContent = replay
    ? simulation
      ? '协议仿真 / 非实物'
      : '记录回放'
    : live
      ? 'USB 实时监测'
      : '未连接';
  $('modeBadge').className = 'badge ' + (replay ? 'amber' : live ? 'green' : 'muted');
  $('linkBadge').textContent = disconnected
    ? '服务中断'
    : replay
      ? s.replay.playing
        ? '正在播放'
        : '回放暂停'
      : live
        ? s.error
          ? '串口中断'
          : s.stale
            ? '等待有效数据'
            : '正在接收'
        : '未占用串口';
  $('linkBadge').className = 'badge ' + (s.error || stale || disconnected ? 'red' : 'muted');
  $('sourceLabel').textContent =
    (replay
      ? simulation
        ? '协议仿真 · '
        : s.record_kind === '历史真实实测'
          ? '历史真实数据 · '
          : '记录回放 · '
      : live
        ? '实时接收 · '
        : '') + s.label;
  const q = s.quality;
  $('quality').textContent =
    `CRC错误 ${q.bad_crc ?? q.bad_frames ?? 0} · 有效帧 ${(q.valid ?? q.valid_frames ?? 0).toLocaleString()} · 序号缺帧 ${q.missing ?? 0}`;
  $('replayBar').classList.toggle('hidden', !replay);
  $('play').textContent = s.replay.playing ? 'Ⅱ' : '▶';
  if (!dragging) {
    $('seek').max = Math.max(s.replay.duration, 0.01);
    $('seek').value = s.replay.position;
    $('replayTime').textContent = timeFmt(s.replay.position);
  }
  $('replayDuration').textContent = timeFmt(s.replay.duration);
  $('speed').value = s.replay.speed;
  const warning = disconnected
    ? '本地服务中断，请重新启动上位机。'
    : s.error ||
      (stale
        ? '串口数据已中断或尚无有效帧，实时数值已隐藏。'
        : ['WEXF', 'WEXH', 'TR01', 'TR02', 'TR03', 'TR04', 'TR05', 'TR06', 'UI01', 'UI02'].includes(
              f?.build,
            )
          ? ''
          : f?.warning || '');
  $('notice').textContent = warning;
  $('notice').classList.toggle('hidden', !warning);
  $('poleVal').textContent = valid ? fmt(f.pole) : '—';
  $('armVal').textContent = valid ? fmt(f.arm) : '—';
  $('tableNote').textContent = [
    'WEXF',
    'WEXH',
    'TR01',
    'TR02',
    'TR03',
    'TR04',
    'TR05',
    'TR06',
    'UI01',
    'UI02',
  ].includes(f?.build)
    ? '画面和横臂计数按一圈取模。试验表与CSV保留原控制偏角RMS、累计跟随范围，不受显示标定影响。≥为累计稳摆时间下界。'
    : f?.build === 'RF02'
      ? 'RF02摆角由原始ADC重建，RMS不等于内部控制误差；统计使用收到的1kHz样本，时间为已观察样本时长。'
      : 'RMS按已接收的稳摆遥测样本计算，使用修正后的软件零点；未收到的数据不补造。';
  $('balanceVal').textContent = valid ? (f.balance_lower_bound ? '≥' : '') + fmt(f.balance) : '—';
  $('durationVal').textContent = f?.duration === null ? 's' : '/ ' + (f?.duration || 20) + ' s';
  $('balanceProgress').parentElement.classList.toggle('hidden', f?.duration === null);
  $('balanceProgress').style.width =
    valid && f.duration ? Math.min(100, (f.balance / f.duration) * 100) + '%' : '0%';
  $('stage').textContent = stale ? '数据中断' : f?.stage || '待机';
  $('stage').style.color =
    stale || (f?.fault && f.fault !== 5) ? C.red : f?.stage === '正常完成' ? C.green : C.cyan;
  $('rawVal').textContent = valid ? '0x' + f.raw.toString(16).toUpperCase().padStart(4, '0') : '—';
  $('encoderVal').textContent = valid && f.encoder_mod !== null ? String(f.encoder_mod) : '—';
  $('upVal').textContent =
    valid && f.reference_ok ? '0x' + f.up.toString(16).toUpperCase().padStart(4, '0') : '—';
  $('poseWatermark').textContent = replay
    ? simulation
      ? '协议仿真 / 非实物'
      : '记录回放'
    : stale
      ? '数据中断'
      : live
        ? 'USB 实时遥测'
        : '等待连接';
  $('chartNote').textContent = frozen
    ? '图表已冻结 · 接收与原始记录仍继续'
    : replay
      ? '记录回放 · 不代表当前板子状态'
      : stale
        ? '数据已中断 · 轨迹仅保留历史数据'
        : '实时遥测 · 数据保存在本机';
  if (f?.build === 'RF02')
    $('chartNote').textContent += ' · 曲线取100Hz，原始记录保留1kHz，不能据图判断高频振动';
  $('recordingLabel').textContent =
    live && s.recording
      ? '原始记录 → recordings/' + s.recording
      : replay
        ? '本机回放 · 未占用串口'
        : '采集已断开 · 原始文件已保留';
  renderTrials(s.trials);
  renderCanvases();
}
function renderTrials(rows) {
  const tbody = $('trialRows');
  tbody.replaceChildren();
  if (!rows.length) {
    let tr = document.createElement('tr'),
      td = document.createElement('td');
    td.colSpan = 6;
    td.className = 'empty';
    td.textContent = '尚无试验记录，播放或接收启动后的数据即可显示';
    tr.append(td);
    tbody.append(tr);
    return;
  }
  for (const r of rows) {
    let tr = document.createElement('tr');
    const vals = [
      `#${r.trial} · ${r.build}`,
      r.route,
      (r.balance_lower_bound ? '≥' : '') + fmt(r.balance) + ' s',
      fmt(r.rms, 2) + '°',
      fmt(r.arm_range, 1) + '°',
      r.stage,
    ];
    vals.forEach((v, i) => {
      let td = document.createElement('td');
      td.textContent = v;
      if (i === 1) {
        let small = document.createElement('small');
        small.textContent = `摆杆 ${r.pole_filter} / 横臂 ${r.arm_filter}`;
        td.append(small);
      }
      if (i === 5) {
        td.className = r.stage === '正常完成' ? 'green' : r.stage === '故障' ? 'red' : '';
        let small = document.createElement('small');
        small.textContent = r.reason;
        td.append(small);
      }
      tr.append(td);
    });
    tbody.append(tr);
  }
}
function canvas(id) {
  const c = $(id),
    rect = c.getBoundingClientRect(),
    dpr = Math.min(window.devicePixelRatio || 1, 2);
  if (c.width !== Math.round(rect.width * dpr) || c.height !== Math.round(rect.height * dpr)) {
    c.width = Math.round(rect.width * dpr);
    c.height = Math.round(rect.height * dpr);
  }
  let ctx = c.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);
  return [ctx, rect.width, rect.height];
}
function line(ctx, x1, y1, x2, y2, color, width = 1, dash = []) {
  ctx.beginPath();
  ctx.setLineDash(dash);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.moveTo(x1, y1);
  ctx.lineTo(x2, y2);
  ctx.stroke();
  ctx.setLineDash([]);
}
function text(ctx, t, x, y, color = C.muted, size = 10, align = 'center') {
  ctx.fillStyle = color;
  ctx.font = `${size >= 18 ? '600 ' : ''}${size}px -apple-system, sans-serif`;
  ctx.textAlign = align;
  ctx.fillText(t, x, y);
}
function dot(ctx, x, y, r, color) {
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  ctx.fillStyle = color;
  ctx.fill();
}
function drawPose() {
  const [ctx, w, h] = canvas('pose');
  if (w < 1) return;
  const f = displayFrame(latest),
    good = f && !snapshot?.stale && !disconnected;
  const cx = w * 0.25,
    ax = w * 0.75,
    cy = h * 0.58,
    rad = Math.min(h * 0.29, w * 0.18);
  ctx.lineCap = 'round';
  text(ctx, '摆杆 / 侧视', cx, 30, '#b6c9df', w < 340 ? 18 : 22);
  text(ctx, '横臂 / 俯视', ax, 30, '#b6c9df', w < 340 ? 18 : 22);
  for (const x of [cx, ax]) {
    line(ctx, x - rad - 11, cy, x + rad + 11, cy, C.line);
    line(ctx, x, cy - rad - 11, x, cy + rad + 11, C.line);
    ctx.beginPath();
    ctx.arc(x, cy, rad + 6, 0, Math.PI * 2);
    ctx.strokeStyle = '#23374d';
    ctx.setLineDash([2, 6]);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  line(ctx, w / 2, 48, w / 2, h - 20, '#203149');
  if (good && f.pose_valid && f.pole !== null) {
    let a = (f.pole_phase * Math.PI) / 180,
      px = cx + Math.sin(a) * rad,
      py = cy - Math.cos(a) * rad;
    line(ctx, cx, cy, px, py, C.green, 4);
    dot(ctx, px, py, 5, C.green);
  } else {
    text(ctx, '参考未就绪', cx, cy + 29, '#617892', 10);
  }
  if (good && f.arm !== null) {
    const p = armScreenDirection(f.arm);
    line(ctx, ax, cy, ax + p.x * rad, cy + p.y * rad, C.cyan, 4);
    dot(ctx, ax + p.x * rad, cy + p.y * rad, 4, C.cyan);
  }
  dot(ctx, cx, cy, 5, '#e3eef9');
  dot(ctx, ax, cy, 5, '#e3eef9');
  text(ctx, good && f.pole !== null ? fmt(f.pole) + '°' : '—', cx, h - 9, C.green, 16);
  text(ctx, good && f.arm !== null ? fmt(f.arm) + '°' : '—', ax, h - 9, C.cyan, 16);
}

// BEGIN CONTINUOUS TRACE: raw accumulated counts stay continuous in charts.
const traceAnchors = new Map();
function continuousArm(rows, anchors = traceAnchors) {
  return rows.map((p) => {
    const key = `${p.build}:${p.epoch}:${p.trial}:${p.counts_per_rev}`;
    const view = displayFrame(p),
      home =
        ['WEXH', 'TR01', 'TR02', 'TR03', 'TR04', 'TR05', 'TR06', 'UI01', 'UI02'].includes(
          p.build,
        ) && p.protocol_version >= 4;
    const good = view.arm !== null && Number.isFinite(p.encoder) && p.counts_per_rev > 0;
    if (good && !anchors.has(key))
      anchors.set(key, Math.round(p.encoder / p.counts_per_rev) * p.counts_per_rev);
    return {
      ...view,
      trace_key: key,
      arm_trace: good
        ? ((p.encoder -
            (home || view.arm_display_manual ? view.arm_display_origin : anchors.get(key))) *
            360) /
            p.counts_per_rev +
          view.arm_display_offset_deg
        : null,
    };
  });
}
function traceBounds(values) {
  const finite = values.filter(Number.isFinite);
  if (!finite.length) return [-30, 30];
  const lo = Math.min(...finite),
    hi = Math.max(...finite),
    pad = Math.max(5, (hi - lo) * 0.12);
  const step = hi - lo > 1200 ? 180 : hi - lo > 180 ? 45 : hi - lo > 30 ? 15 : 5;
  return [Math.floor((lo - pad) / step) * step, Math.ceil((hi + pad) / step) * step];
}
// END CONTINUOUS TRACE

// BEGIN VISUAL FOCUS: screen orientation and axis scaling only.
function armScreenDirection(degrees) {
  const a = (degrees * Math.PI) / 180;
  return { x: Math.cos(a), y: Math.sin(a) };
}
function focusedArmBounds(rows, end, span = 8) {
  const last = rows
    .slice()
    .reverse()
    .find((p) => Number.isFinite(p.arm_trace));
  if (!last) return [-5, 5];
  const values = rows
    .filter(
      (p) => p.trace_key === last.trace_key && p.t >= end - span && Number.isFinite(p.arm_trace),
    )
    .map((p) => p.arm_trace);
  if (!values.length) return traceBounds(rows.map((p) => p.arm_trace));
  const lo = Math.min(...values),
    hi = Math.max(...values),
    center = (lo + hi) / 2;
  const width = Math.max(10, (hi - lo) * 1.16),
    step = width > 1200 ? 90 : width > 180 ? 15 : width > 40 ? 5 : 1;
  return [
    Math.floor((center - width / 2) / step) * step,
    Math.ceil((center + width / 2) / step) * step,
  ];
}
// END VISUAL FOCUS

function verticalText(ctx, label, x, centerY, color, size) {
  const chars = Array.from(label),
    step = size * 1.2;
  ctx.save();
  ctx.textBaseline = 'middle';
  chars.forEach((char, i) =>
    text(ctx, char, x, centerY + (i - (chars.length - 1) / 2) * step, color, size),
  );
  ctx.restore();
}

function drawCharts() {
  const [ctx, w, h] = canvas('charts');
  if (w < 1) return;
  const data = frozen ? frozenPoints : points,
    seconds = Number($('window').value),
    end = data.length ? data[data.length - 1].t : seconds,
    start = Math.max(0, end - seconds);
  const all = continuousArm(data),
    dataIn = all.filter((p) => p.t >= start),
    left = 96,
    right = 17,
    top = 12,
    row = (h - 28) / 3,
    plotW = w - left - right;
  const configs = [
    ['pole', '摆杆角度', '°', C.green],
    ['arm_trace', '横臂位移', '°', C.cyan],
    ['arm_rate', '横臂速度', '°/s', C.amber],
  ];
  configs.forEach(([key, label, unit, color], j) => {
    const y0 = top + j * row + 18,
      y1 = top + (j + 1) * row - 15;
    const [low, high] =
      j === 0
        ? [0, poleZoom ? 15 : 180]
        : j === 1 && armFocus
          ? focusedArmBounds(dataIn, end, Math.min(seconds, 8))
          : traceBounds(dataIn.map((p) => p[key]));
    verticalText(ctx, label, 16, (y0 + y1) / 2, color, w < 340 ? 18 : 22);
    text(ctx, unit, left - 10, top + j * row + 9, C.muted, 12, 'right');
    for (let k = 0; k < 5; k++) {
      const x = left + (k * plotW) / 4;
      line(ctx, x, y0, x, y1, '#203047', 1, [2, 5]);
      if (j === 2)
        text(ctx, fmt(start + (k * seconds) / 4, 1) + 's', x, h - 1, '#6d839e', w < 340 ? 10 : 12);
    }
    for (const v of [low, (low + high) / 2, high]) {
      const y = y1 - ((v - low) / (high - low)) * (y1 - y0);
      line(ctx, left, y, w - right, y, v === 0 ? '#344b65' : '#1d2d43', 1, v === 0 ? [4, 5] : []);
      text(
        ctx,
        fmt(v, high - low < 10 ? 1 : 0),
        left - 9,
        y + 4,
        '#6f86a1',
        w < 340 ? 11 : 13,
        'right',
      );
    }
    ctx.save();
    ctx.beginPath();
    ctx.rect(left, y0 - 1, plotW, y1 - y0 + 2);
    ctx.clip();
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.setLineDash([]);
    ctx.lineWidth = 1.6;
    let began = false,
      lastTime = null,
      lastKey = null;
    for (const p of dataIn) {
      const v = p[key];
      if (!Number.isFinite(v)) {
        began = false;
        continue;
      }
      const x = left + ((p.t - start) / seconds) * plotW,
        y = y1 - ((v - low) / (high - low)) * (y1 - y0);
      if (!began || p.t - lastTime > 0.2 || lastKey !== p.trace_key) {
        ctx.moveTo(x, y);
        began = true;
      } else ctx.lineTo(x, y);
      lastTime = p.t;
      lastKey = p.trace_key;
    }
    ctx.stroke();
    ctx.restore();
  });
  if (!dataIn.length) text(ctx, '等待有效遥测，或载入历史实测记录', w / 2, h / 2, '#6f86a1', 12);
  if (hoverT !== null) {
    const x = left + ((hoverT - start) / seconds) * plotW;
    if (x >= left && x <= w - right) line(ctx, x, 0, x, h - 16, '#bccddd88', 1, [3, 4]);
  }
}
$('charts').onmousemove = (e) => {
  const r = $('charts').getBoundingClientRect(),
    data = frozen ? frozenPoints : points;
  if (!data.length) return;
  const seconds = Number($('window').value),
    end = data[data.length - 1].t,
    t = Math.max(0, end - seconds) + ((e.clientX - r.left - 96) / (r.width - 113)) * seconds;
  hoverT = t;
  const visible = continuousArm(data).filter((p) => p.t >= end - seconds);
  if (!visible.length) return;
  const near = visible.reduce(
      (a, b) => (Math.abs(b.t - t) < Math.abs(a.t - t) ? b : a),
      visible[0],
    ),
    el = $('hover');
  el.replaceChildren();
  for (const s of [
    `记录 ${fmt(near.t, 2)} s`,
    `摆杆 ${fmt(near.pole, 2)}°`,
    `横臂位移 ${fmt(near.arm_trace, 2)}°`,
    `横臂速度 ${fmt(near.arm_rate, 2)}°/s`,
  ]) {
    const div = document.createElement('div');
    div.textContent = s;
    el.append(div);
  }
  el.style.left = Math.max(0, Math.min(r.width - 155, e.clientX - r.left + 12)) + 'px';
  el.style.top = '18px';
  el.classList.remove('hidden');
  drawCharts();
};
$('charts').onmouseleave = () => {
  hoverT = null;
  $('hover').classList.add('hidden');
  drawCharts();
};
function renderCanvases() {
  drawPose();
  drawCharts();
}
let polling = false;
async function poll() {
  if (polling) return;
  polling = true;
  try {
    let s = await api('/api/status?after=' + cursor);
    if (s.generation !== generation) {
      generation = s.generation;
      cursor = 0;
      points = [];
      traceAnchors.clear();
      frozen = false;
      $('freeze').textContent = '冻结图表';
      s = await api('/api/status?after=0');
    }
    for (const p of s.frames) {
      points.push(p);
      cursor = Math.max(cursor, p.i);
    }
    if (points.length > 6000) points = points.slice(-6000);
    disconnected = false;
    updateView(s);
  } catch (e) {
    disconnected = true;
    $('resetArm').disabled = true;
    $('linkBadge').textContent = '服务中断';
    $('linkBadge').className = 'badge red';
    $('notice').textContent = '本地服务中断，请重新启动上位机。';
    $('notice').classList.remove('hidden');
    for (const id of ['poleVal', 'armVal', 'balanceVal']) $(id).textContent = '—';
    renderCanvases();
  } finally {
    polling = false;
  }
}
window.addEventListener('resize', renderCanvases);
(async () => {
  try {
    await catalog();
    await refreshPorts();
    sourceSelection();
    await poll();
  } catch (e) {
    showError(e.message);
  }
  setInterval(poll, 120);
})();

// Historical exports read saved files only; never connect, stop, or transmit UART.
let historyIndexJob = null,
  historyBusy = false;
function historyReset() {
  historyIndexJob = null;
  $('historyTrial').replaceChildren();
  $('historyTrial').disabled = true;
  $('historyExport').disabled = true;
  $('historyDownload').classList.add('hidden');
  $('historyStatus').textContent = '选择记录后点击“分析记录”。';
}
async function historyRecords() {
  const old = $('historyRecord').value,
    rows = await api('/api/history'),
    select = $('historyRecord');
  select.replaceChildren();
  for (const r of rows) {
    const o = document.createElement('option');
    o.value = r.id;
    o.textContent = `${r.label} · ${(r.bytes / 1048576).toFixed(1)}MB · ${r.location}`;
    select.append(o);
  }
  if (rows.some((r) => r.id === old)) select.value = old;
  historyReset();
  if (!rows.length)
    $('historyStatus').textContent = '还没有可导出的串口记录。连接并记录后会自动保存。';
}
async function historyJob(job) {
  while (true) {
    const current = await api('/api/history/job?id=' + encodeURIComponent(job.id));
    $('historyProgress').value = current.progress || 0;
    $('historyStatus').textContent =
      (current.kind === 'index' ? '分析记录' : '生成数据包') + ` ${current.progress || 0}%`;
    if (current.status === 'error') throw Error(current.error || '导出失败');
    if (current.status === 'done') return current;
    await new Promise((resolve) => setTimeout(resolve, 500));
  }
}
async function historyAction(fn) {
  if (historyBusy) return;
  historyBusy = true;
  for (const id of [
    'historyRecord',
    'historyRefresh',
    'historyIndex',
    'historyTrial',
    'historyExport',
  ])
    $(id).disabled = true;
  $('historyDownload').classList.add('hidden');
  $('historyProgress').classList.remove('hidden');
  try {
    await fn();
  } catch (e) {
    $('historyStatus').textContent = e.message;
  } finally {
    historyBusy = false;
    for (const id of ['historyRecord', 'historyRefresh', 'historyIndex']) $(id).disabled = false;
    $('historyTrial').disabled = !historyIndexJob;
    $('historyExport').disabled = !historyIndexJob;
    $('historyProgress').classList.add('hidden');
  }
}
$('openHistory').onclick = async () => {
  $('historyDialog').showModal();
  if (historyBusy) return;
  try {
    await historyRecords();
  } catch (e) {
    $('historyStatus').textContent = e.message;
  }
};
$('historyClose').onclick = () => $('historyDialog').close();
$('historyRefresh').onclick = () => historyAction(historyRecords);
$('historyRecord').onchange = historyReset;
$('historyTrial').onchange = () => $('historyDownload').classList.add('hidden');
$('historyIndex').onclick = () =>
  historyAction(async () => {
    historyReset();
    if (!$('historyRecord').value) throw Error('没有可用记录。');
    const done = await historyJob(
      await api('/api/history/index', { id: $('historyRecord').value }),
    );
    historyIndexJob = done.id;
    const select = $('historyTrial');
    select.replaceChildren();
    let option = document.createElement('option');
    option.value = 'all';
    option.textContent = '全部样本（含待机）';
    select.append(option);
    for (const g of done.result.trials) {
      option = document.createElement('option');
      option.value = g.key;
      option.textContent = `${g.build} · 区间${g.epoch + 1} · ${g.trial ? '试验' + g.trial : '未启动/待机'} · ${g.duration_s.toFixed(1)}秒 · ${g.frames.toLocaleString()}帧`;
      select.append(option);
    }
    $('historyStatus').textContent =
      `分析完成：${done.result.quality.valid_frames.toLocaleString()}帧。选择试验，再生成数据包。`;
  });
$('historyExport').onclick = () =>
  historyAction(async () => {
    if (!historyIndexJob) throw Error('请先分析记录。');
    const done = await historyJob(
      await api('/api/history/export', {
        index_job: historyIndexJob,
        selection: $('historyTrial').value,
        calibration: displayCalibration,
      }),
    );
    const link = $('historyDownload');
    link.href = done.result.download;
    link.download = done.result.filename;
    link.classList.remove('hidden');
    $('historyStatus').textContent =
      `已生成：${done.result.rows.toLocaleString()}帧，${done.result.duration_s.toFixed(1)}秒。点击“下载ZIP”保存并发送给Windows电脑。`;
  });
