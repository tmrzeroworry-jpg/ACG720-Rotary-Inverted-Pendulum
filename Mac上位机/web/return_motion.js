(() => {
  const $ = (id) => document.getElementById(id),
    client = window.motionClient;
  let owner = false,
    active = false;
  const names = {
    0: '待机',
    1: '柔和启动',
    2: '能量起摆',
    3: '捕获与稳摆确认',
    4: '停止横臂任务',
    5: '引导离开直立',
    6: '主动收能下落',
    7: '下垂反馈消摆',
    8: '静止释放确认',
    9: '起摆完成',
    10: '已回到下垂',
    11: '动作未完成',
  };
  async function post(path) {
    const r = await fetch(path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ client }),
    });
    const d = await r.json();
    if (!r.ok) throw Error(d.error);
    return d;
  }
  $('returnStart').onclick = async () => {
    try {
      await post('/api/return/start');
      owner = true;
    } catch (e) {
      $('returnMessage').textContent = e.message;
    }
  };
  $('returnCancel').onclick = () =>
    post('/api/return/cancel').catch((e) => ($('returnMessage').textContent = e.message));
  async function poll() {
    try {
      const s = await (await fetch('/api/status?after=2147483647')).json(),
        f = s.latest,
        m = s.return_motion || {};
      active = ['waiting_curve', 'hello', 'pending', 'running', 'aborting'].includes(m.phase);
      const busy = f && f.life_phase >= 1 && f.life_phase <= 8;
      $('returnStart').disabled = !(
        window.webControlReady &&
        s.mode === 'live' &&
        !s.stale &&
        f?.build === 'UI02' &&
        f?.protocol_version === 8 &&
        f?.state === 4 &&
        f?.life_down_valid &&
        !active &&
        !busy &&
        !['hello', 'pending', 'running', 'aborting'].includes(s.flip?.phase)
      );
      $('returnCancel').disabled = !active;
      const legacyUp = f?.state === 2,
        catching = f?.state === 3,
        fastUp = busy && f?.life_mode === 0;
      $('lifeState').textContent = legacyUp
        ? '原版起摆'
        : catching
          ? '捕获与稳摆确认'
          : f?.state === 4 && !busy
            ? '稳摆'
            : names[f?.life_phase] || '待机';
      $('returnMessage').textContent = fastUp
        ? '快速起摆中：' + (names[f.life_phase] || '观察动作')
        : legacyUp
          ? '原版起摆中，本次保留原起摆逻辑。'
          : catching
            ? '正在捕获，等待稳摆确认。'
            : f?.state !== 4 && f?.life_phase === 11 && f?.life_sensor_fault === 1
              ? '端点观测等待超时，本次动作已停止。'
              : f?.state !== 4 && f?.life_phase === 11
                ? '动作已退出，原因码 ' + f.life_result + '；请保留记录。'
                : m.message || '待机时自然下垂静置约1秒，建立下垂参考。';
      $('lifeElapsed').textContent =
        '用时 ' +
        (Number.isFinite(f?.life_elapsed_ms) ? (f.life_elapsed_ms / 1000).toFixed(2) : '—') +
        ' s';
      $('lifeTurns').textContent = '有效换向 ' + (f?.life_turns ?? '—');
      $('downReference').textContent = f?.life_down_valid
        ? '下垂参考已就绪'
        : '下垂参考未就绪：待机时自然下垂静置';
      if (owner && active) await post('/api/return/heartbeat');
      else if (!active) owner = false;
    } catch (e) {}
    setTimeout(poll, 150);
  }
  window.addEventListener('pagehide', () => {
    if (owner && active)
      fetch('/api/return/cancel', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: '{}',
        keepalive: true,
      }).catch(() => {});
  });
  poll();
})();
