/* UI02 editor. Explicit START is the only entry into real-time command streaming. */
(() => {
  'use strict';
  const client = window.motionClient;
  const el = (id) => document.getElementById(id),
    canvas = el('trajectoryCanvas');
  let pts = TrajectoryMath.example(15),
    duration = 15,
    vmax = 180,
    amax = 360;
  let drag = -1,
    timer = null,
    valid = false,
    running = false,
    armedUI = false,
    status = null,
    invalid = false,
    pending = false,
    revision = 0,
    checkedRevision = -1;
  const phaseNames = {
    idle: '编辑预览',
    hello: '板端握手',
    arming: '等待确认',
    running: '曲线运行中',
    stopping: '正在减速',
    done: '曲线已结束',
    error: '任务已取消',
  };
  const body = () => ({ points: pts, vmax, amax, client, editor_revision: revision });
  function msg(text, bad = false) {
    invalid = bad;
    el('trajMessage').textContent = text;
    el('trajMessage').classList.toggle('invalid', bad);
  }
  async function request(path, data) {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(data || {}),
    });
    const d = await r.json();
    if (!r.ok) throw Error(d.error || '请求失败');
    return d;
  }
  function syncText() {
    el('trajPointCount').textContent = pts.length + ' / 16 个点';
    el('trajPoints').value = JSON.stringify(pts.map(([t, v]) => [+t.toFixed(2), +v.toFixed(2)]));
  }
  function validateLocal() {
    return TrajectoryMath.validate({ points: pts, vmax, amax }).peak_acceleration;
  }

  async function apply() {
    vmax = Number(el('trajVmax').value);
    amax = Number(el('trajAmax').value);
    const rev = revision;
    try {
      validateLocal();
      const c = await request('/api/trajectory/validate', body());
      if (rev !== revision) return;
      checkedRevision = rev;
      valid = true;
      msg(
        `已应用 · 最大加速度 ${c.peak_acceleration.toFixed(1)}°/s² · 板端上限 ${c.amax.toFixed(2)}°/s²${c.clipped_points ? ' · ' + c.clipped_points + '个点已速度限幅' : ''}${running ? ' · 按当前播放时刻继续' : ''}`,
      );
    } catch (e) {
      if (rev === revision) {
        valid = false;
        msg(e.message + (running ? ' 本次修改未应用，继续上一条有效曲线。' : ''), true);
      }
    }
    controls();
    draw();
  }
  function changed() {
    revision++;
    checkedRevision = -1;
    valid = false;
    syncText();
    clearTimeout(timer);
    try {
      let peak = validateLocal();
      msg(`待应用 · 最大加速度 ${peak.toFixed(1)}°/s²`);
      if (el('trajAuto').checked) timer = setTimeout(apply, 180);
    } catch (e) {
      msg(e.message, true);
    }
    controls();
    draw();
  }
  function geometry() {
    const r = canvas.getBoundingClientRect();
    return { w: r.width, h: r.height, l: 72, r: r.width - 24, t: 26, b: r.height - 48 };
  }
  function draw() {
    const g = geometry();
    if (g.w < 1) return;
    const d = Math.min(devicePixelRatio || 1, 2);
    if (canvas.width !== Math.round(g.w * d) || canvas.height !== Math.round(g.h * d)) {
      canvas.width = Math.round(g.w * d);
      canvas.height = Math.round(g.h * d);
    }
    const c = canvas.getContext('2d');
    c.setTransform(d, 0, 0, d, 0, 0);
    c.clearRect(0, 0, g.w, g.h);
    const span = Math.max(vmax, 10) * 1.15,
      x = (t) => g.l + (t / duration) * (g.r - g.l),
      y = (v) => (g.t + g.b) / 2 - ((v / span) * (g.b - g.t)) / 2;
    c.font = '16px -apple-system,sans-serif';
    c.textAlign = 'right';
    c.lineWidth = 1;
    for (const v of [-vmax, 0, vmax]) {
      c.strokeStyle = v === 0 ? '#49627e' : '#263a50';
      c.beginPath();
      c.moveTo(g.l, y(v));
      c.lineTo(g.r, y(v));
      c.stroke();
      c.fillStyle = '#a2b7cf';
      c.fillText(v.toFixed(0), g.l - 12, y(v) + 5);
    }
    c.textAlign = 'center';
    const ticks = g.w < 500 ? 2 : 4;
    for (let i = 0; i <= ticks; i++) {
      let t = (duration * i) / ticks;
      c.fillStyle = '#9db2cd';
      c.fillText(t.toFixed(1) + 's', x(t), g.b + 30);
    }
    c.fillStyle = '#bcd0e5';
    c.fillText('°/s', 37, 20);
    function trace(rows, color) {
      c.save();
      c.beginPath();
      c.rect(g.l, g.t, g.r - g.l, g.b - g.t);
      c.clip();
      c.strokeStyle = color;
      c.lineWidth = 2.5;
      c.beginPath();
      let first = true;
      for (const p of rows) {
        let v = p[1],
          t = p[0];
        if (!Number.isFinite(v)) {
          first = true;
          continue;
        }
        if (first) c.moveTo(x(t), y(v));
        else c.lineTo(x(t), y(v));
        first = false;
      }
      c.stroke();
      c.restore();
    }
    trace(
      pts.map(([t, v]) => [t, Math.max(-vmax, Math.min(vmax, v))]),
      invalid ? '#fb8e9b' : '#53d5e8',
    );
    if (running && status) {
      let t = status.elapsed;
      c.setLineDash([5, 5]);
      c.strokeStyle = '#e3edf9';
      c.beginPath();
      c.moveTo(x(t), g.t);
      c.lineTo(x(t), g.b);
      c.stroke();
      c.setLineDash([]);
    }
    for (const [t, v] of pts) {
      const yy = y(Math.max(-vmax, Math.min(vmax, v)));
      c.beginPath();
      c.arc(x(t), yy, 6, 0, Math.PI * 2);
      c.fillStyle = '#d8f7ff';
      c.fill();
    }
  }
  function controls() {
    el('trajVmax').disabled = running;
    el('trajAmax').disabled = running;
    el('trajStart').disabled =
      !valid || checkedRevision !== revision || running || pending || !status?.available;
    el('trajStop').disabled = !running;
  }
  canvas.onpointerdown = (e) => {
    const g = geometry(),
      r = canvas.getBoundingClientRect(),
      mx = e.clientX - r.left,
      my = e.clientY - r.top;
    let best = 16;
    drag = -1;
    pts.forEach(([t, v], i) => {
      const dx = g.l + (t / duration) * (g.r - g.l) - mx,
        dy =
          (g.t + g.b) / 2 -
          ((Math.max(-vmax, Math.min(vmax, v)) / (vmax * 1.15)) * (g.b - g.t)) / 2 -
          my,
        dist = Math.hypot(dx, dy);
      if (dist < best) {
        drag = i;
        best = dist;
      }
    });
    if (drag < 0) {
      if (pts.length >= 16) {
        msg('已达到16个点；可拖动现有点，或双击删除后添加。');
        return;
      }
      const t =
        Math.round(Math.max(0, Math.min(duration, ((mx - g.l) / (g.r - g.l)) * duration)) * 100) /
        100;
      if (t <= 0 || t >= duration || pts.some((p) => Math.abs(p[0] - t) < 0.1)) return;
      const v = Math.round(
        Math.max(-vmax, Math.min(vmax, (((g.t + g.b) / 2 - my) * vmax * 1.15 * 2) / (g.b - g.t))),
      );
      pts.push([t, v]);
      pts.sort((a, b) => a[0] - b[0]);
      drag = pts.findIndex((p) => p[0] === t);
      changed();
    }
    canvas.setPointerCapture(e.pointerId);
  };
  canvas.onpointermove = (e) => {
    if (drag < 0) return;
    const g = geometry(),
      r = canvas.getBoundingClientRect();
    if (drag === 0 || drag === pts.length - 1) return;
    let t = Math.round(((e.clientX - r.left - g.l) / (g.r - g.l)) * duration * 100) / 100;
    t = Math.max(pts[drag - 1][0] + 0.1, Math.min(pts[drag + 1][0] - 0.1, t));
    let v = Math.round((((g.t + g.b) / 2 - (e.clientY - r.top)) * vmax * 1.15 * 2) / (g.b - g.t));
    pts[drag] = [+t.toFixed(2), Math.max(-vmax, Math.min(vmax, v))];
    changed();
  };
  canvas.onpointerup = canvas.onpointercancel = () => {
    drag = -1;
  };
  canvas.ondblclick = (e) => {
    const g = geometry(),
      r = canvas.getBoundingClientRect();
    const t = ((e.clientX - r.left - g.l) / (g.r - g.l)) * duration;
    let i = pts.findIndex((p) => Math.abs(p[0] - t) < duration * 0.025);
    if (i > 0 && i < pts.length - 1) {
      pts.splice(i, 1);
      changed();
    }
  };
  el('trajVmax').onchange = () => {
    vmax = Number(el('trajVmax').value);
    changed();
  };
  el('trajAmax').onchange = () => {
    amax = Number(el('trajAmax').value);
    changed();
  };
  el('trajDuration').onchange = () => {
    let d = Number(el('trajDuration').value);
    if (!Number.isFinite(d) || d < Math.max(1, (pts.length - 1) * 0.1) || d > 120) {
      valid = false;
      checkedRevision = -1;
      controls();
      msg('请保证相邻点至少间隔0.1秒，总时长不超过120秒。', true);
      return;
    }
    let k = d / duration;
    pts = pts.map(([t, v]) => [+(t * k).toFixed(2), v]);
    duration = d;
    changed();
  };
  el('trajApply').onclick = apply;
  el('trajAuto').onchange = () => {
    if (el('trajAuto').checked) apply();
  };
  el('trajExample').onclick = () => {
    pts = TrajectoryMath.example(15);
    duration = 15;
    el('trajDuration').value = 15;
    changed();
  };
  el('trajClear').onclick = () => {
    duration = Math.max(1.5, duration);
    el('trajDuration').value = duration;
    pts = TrajectoryMath.example(duration).map(([t]) => [t, 0]);
    changed();
  };
  el('trajImport').onclick = () => {
    try {
      const parsed = JSON.parse(el('trajPoints').value),
        old = { pts, vmax, amax, duration };
      pts = Array.isArray(parsed) ? parsed : parsed.points;
      if (!Array.isArray(parsed)) {
        vmax = parsed.vmax ?? vmax;
        amax = parsed.amax ?? amax;
      }
      try {
        validateLocal();
      } catch (e) {
        ({ pts, vmax, amax, duration } = old);
        throw e;
      }
      duration = pts.at(-1)[0];
      el('trajDuration').value = duration;
      el('trajVmax').value = vmax;
      el('trajAmax').value = amax;
      changed();
    } catch (e) {
      valid = false;
      checkedRevision = -1;
      controls();
      msg(e.message, true);
    }
  };
  el('trajExport').onclick = () => {
    const blob = new Blob([JSON.stringify({ points: pts, vmax, amax }, null, 2)], {
        type: 'application/json',
      }),
      a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'UI02_velocity_curve_16points.json';
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 1000);
  };
  el('trajStart').onclick = async () => {
    pending = true;
    controls();
    try {
      await apply();
      if (!valid) return;
      await request('/api/trajectory/start', { client });
      armedUI = true;
      msg('等待FPGA确认…');
    } catch (e) {
      msg(e.message, true);
    } finally {
      pending = false;
      controls();
    }
  };
  el('trajStop').onclick = async () => {
    try {
      await request('/api/trajectory/stop');
    } catch (e) {
      msg(e.message, true);
    }
  };
  async function poll() {
    try {
      const r = await fetch('/api/status?after=2147483647'),
        s = await r.json();
      if (!r.ok) throw Error(s.error);
      status = s.trajectory;
      const flipBusy = ['hello', 'pending', 'running', 'aborting'].includes(s.flip?.phase);
      status.available =
        !flipBusy &&
        !(s.latest?.life_phase >= 1 && s.latest?.life_phase <= 8) &&
        window.webControlReady &&
        s.mode === 'live' &&
        !s.stale &&
        ['UI02'].includes(s.latest?.build) &&
        s.latest?.state === 4;
      running = ['hello', 'arming', 'running', 'stopping'].includes(status.phase);
      // Only the page that explicitly started a task renews its lease.
      if (armedUI && running) await request('/api/trajectory/heartbeat', { client });
      else if (!running) armedUI = false;
      el('trajectoryState').textContent = phaseNames[status.phase] || status.phase;
      el('trajAcceleration').textContent =
        '实际加速度估计 ' +
        (Number.isFinite(status.estimated_acceleration)
          ? status.estimated_acceleration.toFixed(1)
          : '—') +
        ' °/s²（100ms差分）';
      if (running && !invalid) el('trajMessage').textContent = status.message;
      else if (status.phase === 'error') msg(status.message, true);
      controls();
      draw();
    } catch (e) {
      controls();
    }
    setTimeout(poll, 150);
  }
  window.addEventListener('pagehide', () => {
    if (armedUI)
      fetch('/api/trajectory/stop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: '{}',
        keepalive: true,
      }).catch(() => {});
  });
  new ResizeObserver(draw).observe(canvas);
  syncText();
  apply();
  poll();
})();
