/* Pure curve validation: shared by the editor and offline Node tests. */
((root) => {
  'use strict';
  const Q = (65536 * 1040) / (360 * 1000);
  function example(duration = 15) {
    if (!Number.isFinite(duration) || duration < 1.5 || duration > 120)
      throw Error('16点示例总时长需为1.5～120秒。');
    return Array.from({ length: 16 }, (_, i) => [
      +((duration * i) / 15).toFixed(2),
      i === 0 || i === 15 ? 0 : +(150 * Math.sin((4 * Math.PI * i) / 15)).toFixed(2),
    ]);
  }
  function validate({ points: pts, vmax = 180, amax = 360 }) {
    if (!Number.isFinite(vmax) || vmax < 1 || vmax > 360) throw Error('速度限幅应在1～360°/s。');
    if (!Number.isFinite(amax) || amax < 6 || amax > 720) throw Error('加速度上限应在6～720°/s²。');
    if (!Array.isArray(pts) || pts.length < 2 || pts.length > 16)
      throw Error('控制点需要2～16个。');
    let peak = 0,
      clipped = 0;
    const raw = [];
    for (const p of pts) {
      if (!Array.isArray(p) || p.length !== 2 || !p.every(Number.isFinite))
        throw Error('控制点必须是有限数值[时间,速度]。');
      const [t, v] = p;
      if (t < 0 || t > 120 || Math.abs(v) > 10000)
        throw Error('时间范围0～120秒，速度输入不能异常。');
      const nv = Math.max(-vmax, Math.min(vmax, v));
      clipped += Number(nv !== v);
      if (raw.length) {
        const prev = raw.at(-1),
          dt = t - prev[0];
        if (dt < 0.099999999) throw Error('同一时刻只能有一个速度；时间递增且间隔至少0.1秒。');
        peak = Math.max(peak, Math.abs(nv - prev[1]) / dt);
      }
      raw.push([t, nv]);
    }
    if (raw[0][0] !== 0 || Math.abs(raw[0][1]) > 1e-9 || Math.abs(raw.at(-1)[1]) > 1e-9)
      throw Error('第一点从0秒、0速度开始，末点速度必须为0。');
    if (raw.at(-1)[0] < 1) throw Error('总时长至少1秒。');
    if (peak > amax + 1e-7)
      throw Error(`加速度过快：${peak.toFixed(2)}°/s²，上限${amax.toFixed(2)}°/s²。`);
    const step = Math.max(1, Math.min(136, Math.floor((amax * Q) / 1000))),
      actual = (step * 1000) / Q;
    if (peak > actual + 1e-7)
      throw Error(`受FPGA定点步长限制，实际加速度上限为${actual.toFixed(2)}°/s²，请稍微减缓曲线。`);
    return {
      points: raw,
      vmax,
      amax: actual,
      acc_step: step,
      peak_acceleration: peak,
      clipped_points: clipped,
      duration: raw.at(-1)[0],
    };
  }
  const api = Object.freeze({ example, validate });
  root.TrajectoryMath = api;
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
})(typeof window === 'undefined' ? globalThis : window);
