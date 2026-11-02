(() => {
  const $ = (id) => document.getElementById(id),
    client = window.motionClient;
  let own = false,
    active = false,
    localError = '';
  const phases = {
    0: '未启动',
    1: '平滑离开直立',
    2: '同向翻转',
    3: '顶部捕获',
    4: '确认稳摆',
    5: '翻转完成',
    6: '翻转未完成',
  };
  async function post(path, data = {}) {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ ...data, client }),
    });
    const j = await r.json();
    if (!r.ok) throw Error(j.error);
    return j;
  }
  async function start(direction) {
    localError = '';
    try {
      await post('/api/flip/start', { direction, variant: Number($('flipVariant').value) });
      own = true;
      $('flipMessage').textContent = '等待板端确认…';
    } catch (e) {
      localError = e.message;
      $('flipMessage').textContent = e.message;
    }
  }
  $('flipPositive').onclick = () => start(1);
  $('flipNegative').onclick = () => start(-1);
  $('flipCancel').onclick = () =>
    post('/api/flip/cancel').catch((e) => ($('flipMessage').textContent = e.message));
  async function poll() {
    try {
      const r = await fetch('/api/status?after=2147483647'),
        s = await r.json(),
        f = s.latest,
        m = s.flip || {};
      active = ['hello', 'pending', 'running', 'aborting'].includes(m.phase);
      const ready =
        window.webControlReady &&
        s.mode === 'live' &&
        !s.stale &&
        f?.build === 'UI02' &&
        f?.protocol_version === 8 &&
        !(1 <= f.life_phase && f.life_phase <= 8) &&
        f?.state === 4 &&
        f?.stream_velocity_q16 === 0 &&
        !active &&
        !['hello', 'arming', 'running', 'stopping'].includes(s.trajectory?.phase);
      $('flipPositive').disabled = $('flipNegative').disabled = !ready;
      $('flipCancel').disabled = !active;
      $('flipVariant').disabled = active;
      $('flipState').textContent = phases[f?.flip_phase] || '未启动';
      $('flipMessage').textContent =
        localError && !active && m.phase !== 'error' && m.phase !== 'done'
          ? localError
          : m.message || '先稳摆后开始。';
      $('flipProgress').textContent =
        '累计转角 ' +
        (Number.isFinite(f?.flip_progress) ? ((f.flip_progress * 360) / 26400).toFixed(1) : '—') +
        ' °';
      $('flipElapsed').textContent =
        '用时 ' +
        (Number.isFinite(f?.flip_elapsed_ms) ? (f.flip_elapsed_ms / 1000).toFixed(2) : '—') +
        ' s';
      if (own && active) await post('/api/flip/heartbeat');
      else if (!active) own = false;
    } catch (e) {}
    setTimeout(poll, 150);
  }
  window.addEventListener('pagehide', () => {
    if (own && active)
      fetch('/api/flip/cancel', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: '{}',
        keepalive: true,
      }).catch(() => {});
  });
  poll();
})();
