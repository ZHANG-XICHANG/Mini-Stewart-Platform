/*
 * 模擬曲線與單軸工作範圍：全部由 StewartIK 即時計算，不含任何量測資料。
 */
(function () {
  'use strict';
  const IK = window.StewartIK;
  if (!IK) return;
  const SERIES = ['#3987e5', '#d95926', '#199e70', '#c98500', '#d55181', '#008300'];
  const AXIS_NAMES = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'];
  const UNITS = ['mm', 'mm', 'mm', '°', '°', '°'];
  const NS = 'http://www.w3.org/2000/svg';

  // ---------- 單軸工作範圍（二分搜尋 IK 可行邊界） ----------
  const wsBody = document.querySelector('#workspace-table tbody');
  if (wsBody) {
    const limit = (axis, sign, matrix) => {
      let lo = 0, hi = axis < 3 ? 80 : 60;
      const ok = (v) => { const p = [0, 0, 0, 0, 0, 0]; p[axis] = sign * v; return IK.solve(p, matrix).ok; };
      if (ok(hi)) return '≥ ' + hi;
      for (let k = 0; k < 40; k++) { const m = (lo + hi) / 2; ok(m) ? (lo = m) : (hi = m); }
      return lo.toFixed(1);
    };
    const gui = ['±50', '±50', '±50', '±30', '±30', '±30'];
    for (let a = 0; a < 6; a++) {
      const tr = document.createElement('tr');
      tr.innerHTML = `<th scope="row">${AXIS_NAMES[a]}（${UNITS[a]}）</th><td>−${limit(a, -1, 'firmware')}</td><td>+${limit(a, 1, 'firmware')}</td><td>${gui[a]}</td>`;
      wsBody.appendChild(tr);
    }
  }

  // ---------- 正弦命令 → 伺服角模擬曲線 ----------
  const fig = document.getElementById('sim-chart');
  if (!fig) return;
  const svg = fig.querySelector('svg');
  const axisSel = fig.querySelector('[data-chart-axis]');
  const ampIn = fig.querySelector('[data-chart-amp]');
  const freqIn = fig.querySelector('[data-chart-freq]');
  const note = fig.querySelector('[data-chart-note]');
  const tableBody = fig.querySelector('table tbody');
  const tip = fig.querySelector('.chart-tip');
  const W = 720, H = 320, m = { l: 48, r: 16, t: 16, b: 36 };
  let data = [];

  function compute() {
    const axis = +axisSel.value, amp = +ampIn.value || 0, f = Math.max(0.05, +freqIn.value || 0.5);
    const T = Math.min(2 / f, 20), N = 200;
    data = [];
    let fails = 0;
    for (let k = 0; k <= N; k++) {
      const t = (k / N) * T;
      const p = [0, 0, 0, 0, 0, 0];
      p[axis] = amp * Math.sin(2 * Math.PI * f * t);
      const r = IK.solve(p, 'firmware');
      if (!r.ok) fails++;
      data.push({ t, cmd: p[axis], ang: r.ok ? r.angles : null });
    }
    // 對稱運動時部分伺服角度完全相同，曲線會重疊
    const same = [], used = new Set();
    for (let i = 0; i < 6; i++) {
      if (used.has(i)) continue;
      const group = [i];
      for (let k = i + 1; k < 6; k++) {
        if (data.every((d) => !d.ang || Math.abs(d.ang[i] - d.ang[k]) < 1e-6)) { group.push(k); used.add(k); }
      }
      if (group.length > 1) same.push(group.map((g) => 'S' + g).join('='));
    }
    note.textContent = (fails
      ? `有 ${fails} 個取樣點 IK 不可行（圖中斷開處），韌體會丟棄這些封包。`
      : `兩個週期全部 IK 可行（取樣 ${N + 1} 點，週期 ${(1 / f).toFixed(2)} s）。`) +
      (same.length ? ` 因幾何對稱，${same.join('、')} 的曲線重疊；滑過圖表可看到六軸數值。` : '');
    render(T);
  }

  function el(name, attrs, parent) {
    const e = document.createElementNS(NS, name);
    for (const k in attrs) e.setAttribute(k, attrs[k]);
    (parent || svg).appendChild(e);
    return e;
  }

  function render(T) {
    svg.innerHTML = '';
    const vals = data.flatMap((d) => d.ang || []);
    let ymax = Math.max(10, Math.ceil(Math.max(...vals.map(Math.abs), 0) / 10) * 10);
    const x = (t) => m.l + (t / T) * (W - m.l - m.r);
    const y = (v) => m.t + (1 - (v + ymax) / (2 * ymax)) * (H - m.t - m.b);
    // grid
    for (let v = -ymax; v <= ymax; v += ymax / 2) {
      el('line', { x1: m.l, x2: W - m.r, y1: y(v), y2: y(v), class: v === 0 ? 'axis' : 'grid' });
      el('text', { x: m.l - 8, y: y(v) + 4, class: 'tick', 'text-anchor': 'end' }).textContent = v + '°';
    }
    const steps = 4;
    for (let k = 0; k <= steps; k++) {
      const t = (T * k) / steps;
      el('text', { x: x(t), y: H - m.b + 18, class: 'tick', 'text-anchor': 'middle' }).textContent = t.toFixed(1) + ' s';
    }
    // series
    for (let i = 0; i < 6; i++) {
      let d = '', pen = false;
      data.forEach((p) => {
        if (!p.ang) { pen = false; return; }
        d += (pen ? 'L' : 'M') + x(p.t).toFixed(1) + ',' + y(p.ang[i]).toFixed(1);
        pen = true;
      });
      el('path', { d, fill: 'none', stroke: SERIES[i], 'stroke-width': 2, 'stroke-linejoin': 'round', 'stroke-linecap': 'round' });
    }
    const cross = el('line', { y1: m.t, y2: H - m.b, class: 'crosshair', visibility: 'hidden' });
    const hit = el('rect', { x: m.l, y: m.t, width: W - m.l - m.r, height: H - m.t - m.b, fill: 'transparent' });
    const move = (evt) => {
      const r = svg.getBoundingClientRect();
      const px = ((evt.clientX - r.left) / r.width) * W;
      const t = Math.max(0, Math.min(T, ((px - m.l) / (W - m.l - m.r)) * T));
      const p = data[Math.round((t / T) * (data.length - 1))];
      cross.setAttribute('x1', x(p.t)); cross.setAttribute('x2', x(p.t));
      cross.setAttribute('visibility', 'visible');
      const ax = +axisSel.value;
      tip.innerHTML = `<b>t = ${p.t.toFixed(2)} s</b><br>命令 ${AXIS_NAMES[ax]} = ${p.cmd.toFixed(2)} ${UNITS[ax]}` +
        (p.ang ? p.ang.map((a, i) => `<br><span class="swatch" style="--c:${SERIES[i]}"></span>S${i} ${a.toFixed(2)}°`).join('') : '<br>IK 不可行');
      tip.hidden = false;
      const fx = (x(p.t) / W) * r.width;
      tip.style.left = Math.min(fx + 12, r.width - 150) + 'px';
      tip.style.top = '8px';
    };
    hit.addEventListener('pointermove', move);
    hit.addEventListener('pointerdown', move);
    hit.addEventListener('pointerleave', () => { tip.hidden = true; cross.setAttribute('visibility', 'hidden'); });
    // table view (每 1/8 週期)
    tableBody.innerHTML = '';
    data.filter((_, k) => k % 25 === 0).forEach((p) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `<td>${p.t.toFixed(2)}</td><td>${p.cmd.toFixed(2)}</td>` +
        (p.ang ? p.ang.map((a) => `<td>${a.toFixed(2)}</td>`).join('') : '<td colspan="6">IK 不可行</td>');
      tableBody.appendChild(tr);
    });
  }

  [axisSel, ampIn, freqIn].forEach((e) => e.addEventListener('change', compute));
  compute();
})();
