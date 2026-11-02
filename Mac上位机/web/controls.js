(() => {
  window.motionClient = crypto.randomUUID();
  window.webControlReady = false;
  const $ = (id) => document.getElementById(id),
    client = window.motionClient;
  let chosen = 'position',
    owned = false,
    active = false,
    localError = '',
    sending = false;
  async function post(path, data = {}) {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ...data, client }),
    });
    const v = await r.json();
    if (!r.ok) throw Error(v.error || '操作失败');
    return v;
  }
  async function perform(fn) {
    if (sending) return;
    sending = true;
    localError = '';
    try {
      await fn();
    } catch (e) {
      localError = e.message;
      $('controlMessage').textContent = localError;
    } finally {
      sending = false;
    }
  }
  $('controlClaim').onclick = () => perform(() => post('/api/control/claim'));
  $('controlStart').onclick = () =>
    perform(() =>
      post('/api/control/action', { action: 'start', value: Number($('startMethod').value) }),
    );
  $('controlStop').onclick = () => perform(() => post('/api/control/stop'));
  $('controlRelease').onclick = () => perform(() => post('/api/control/stop'));
  $('taskHold').onclick = () => perform(() => post('/api/control/action', { action: 'hold' }));
  $('taskExecute').onclick = () =>
    perform(() =>
      post('/api/control/action', {
        action: chosen,
        value:
          chosen === 'position'
            ? Number($('taskPosition').value)
            : chosen === 'spin'
              ? Number($('spinSpeed').value) + Number($('spinDirection').value) * 2
              : Number($('sinePattern').value),
      }),
    );
  document.querySelectorAll('[data-task-tab]').forEach(
    (b) =>
      (b.onclick = () => {
        chosen = b.dataset.taskTab;
        document
          .querySelectorAll('[data-task-tab]')
          .forEach((t) => t.setAttribute('aria-selected', String(t === b)));
        document
          .querySelectorAll('[data-task-fields]')
          .forEach((t) => (t.hidden = t.dataset.taskFields !== chosen));
        localError = '';
      }),
  );
  function taskName(f) {
    if (!f || f.state !== 4) return '横臂任务：未运行';
    if (f.life_phase >= 1 && f.life_phase <= 8) return '横臂任务：动作执行中';
    if (!(f.remote_flags & 1)) return '横臂任务：板上控制';
    if (f.remote_cfg === 128) return '横臂任务：减速 / 保持';
    if (f.remote_mode === 1) return '横臂任务：定点 ' + (f.remote_cfg & 7) * 45 + '°';
    if (f.remote_mode === 2) return '横臂任务：' + (f.remote_cfg & 16 ? '反向' : '正向') + '恒速';
    if (f.remote_mode === 3) return '横臂任务：正弦';
    return '横臂任务：稳摆保持';
  }
  async function poll() {
    try {
      const r = await fetch('/api/status?after=2147483647');
      if (!r.ok) throw Error('服务未就绪');
      const s = await r.json(),
        f = s.latest,
        m = s.control || {};
      const live =
        s.mode === 'live' && !s.stale && f?.build === 'UI02' && f?.protocol_version === 8;
      owned = m.owner === client;
      active = owned && m.phase === 'active' && live && Boolean(f.remote_flags & 1);
      window.webControlReady = active && !m.pending;
      const busy =
        m.pending ||
        (f && f.life_phase >= 1 && f.life_phase <= 8) ||
        ['hello', 'pending', 'running', 'aborting'].includes(s.flip?.phase) ||
        ['waiting_curve', 'hello', 'pending', 'running', 'aborting'].includes(
          s.return_motion?.phase,
        ) ||
        ['hello', 'arming', 'running', 'stopping'].includes(s.trajectory?.phase);
      const boardReady = live && (f.remote_flags & 12) === 12;
      $('controlClaim').disabled =
        !boardReady ||
        ['claiming', 'active', 'stopping'].includes(m.phase) ||
        Boolean(f?.remote_flags & 1) ||
        busy ||
        ![1, 4].includes(f?.state);
      $('controlClaim').textContent = active
        ? '网页控制已启用'
        : m.phase === 'claiming'
          ? '正在接管…'
          : '启用网页控制';
      $('controlStart').disabled =
        !active ||
        !boardReady ||
        busy ||
        f?.state !== 1 ||
        ($('startMethod').value === '0' && !f?.life_down_valid);
      $('startMethod').disabled = !active || busy || f?.state !== 1;
      $('controlStop').disabled = !live;
      $('controlRelease').disabled = !active || f?.state !== 1;
      $('taskExecute').disabled = $('taskHold').disabled =
        !active || !boardReady || busy || f?.state !== 4;
      document
        .querySelectorAll('[data-task-fields] select')
        .forEach((x) => (x.disabled = !active || busy));
      $('controlBadge').textContent = active
        ? '网页控制'
        : live && f?.remote_flags & 1
          ? '其他页面控制'
          : s.mode === 'replay'
            ? '回放 · 不控制'
            : live
              ? '板上控制'
              : '未连接';
      $('controlPanel').classList.toggle('web-enabled', active);
      $('controlCurrent').textContent = taskName(f);
      $('controlMessage').textContent = localError || m.message || '连接UI02后启用网页控制。';
      if (owned && ['claiming', 'active'].includes(m.phase)) await post('/api/control/heartbeat');
    } catch (e) {
      window.webControlReady = false;
      $('controlClaim').disabled =
        $('controlStart').disabled =
        $('taskExecute').disabled =
        $('taskHold').disabled =
          true;
      $('controlStop').disabled = true;
      $('controlBadge').textContent = '服务中断';
    }
    setTimeout(poll, 150);
  }
  window.addEventListener('pagehide', () => {
    if (owned)
      fetch('/api/control/stop', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ client }),
        keepalive: true,
      }).catch(() => {});
  });
  poll();
})();
