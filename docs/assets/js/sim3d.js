/*
 * 六自由度互動模擬：Canvas 2D 透視投影，無外部相依。
 * 幾何全部來自 StewartIK（韌體參數），畫面為「模擬」，不是量測結果。
 */
(function () {
  'use strict';
  const IK = window.StewartIK;
  const root = document.getElementById('sim');
  if (!IK || !root) return;

  const canvas = root.querySelector('canvas');
  const ctx = canvas.getContext('2d');
  const SERIES = ['#3987e5', '#d95926', '#199e70', '#c98500', '#d55181', '#008300'];
  const AXES = [
    { key: 'x', label: 'X', unit: 'mm', min: -50, max: 50 },
    { key: 'y', label: 'Y', unit: 'mm', min: -50, max: 50 },
    { key: 'z', label: 'Z', unit: 'mm', min: -50, max: 50 },
    { key: 'roll', label: 'Roll', unit: '°', min: -30, max: 30 },
    { key: 'pitch', label: 'Pitch', unit: '°', min: -30, max: 30 },
    { key: 'yaw', label: 'Yaw', unit: '°', min: -30, max: 30 },
  ];
  const pose = [0, 0, 0, 0, 0, 0];
  let lastGood = IK.solve(pose, 'firmware');
  let matrix = 'firmware';
  let view = { az: -0.65, el: 0.42, dist: 520 };
  let anim = null;

  // ---------- 控制項 ----------
  const sliderBox = root.querySelector('[data-sliders]');
  const inputs = [];
  AXES.forEach((ax, i) => {
    const row = document.createElement('div');
    row.className = 'slider-row';
    const id = 'sim-' + ax.key;
    row.innerHTML =
      `<label for="${id}">${ax.label}</label>` +
      `<input type="range" id="${id}" min="${ax.min}" max="${ax.max}" step="0.1" value="0" aria-describedby="${id}-v">` +
      `<output id="${id}-v" for="${id}">0.0 ${ax.unit}</output>`;
    sliderBox.appendChild(row);
    const input = row.querySelector('input');
    const out = row.querySelector('output');
    input.addEventListener('input', () => {
      stopAnim();
      pose[i] = parseFloat(input.value);
      update();
    });
    inputs.push({ input, out, ax });
  });

  root.querySelectorAll('[name="sim-matrix"]').forEach((r) =>
    r.addEventListener('change', (e) => { matrix = e.target.value; update(); }));

  root.querySelector('[data-reset]').addEventListener('click', () => {
    stopAnim();
    pose.fill(0);
    update();
  });

  const animBtn = root.querySelector('[data-anim]');
  const animAxis = root.querySelector('[data-anim-axis]');
  const animAmp = root.querySelector('[data-anim-amp]');
  const animFreq = root.querySelector('[data-anim-freq]');
  animBtn.addEventListener('click', () => {
    if (anim) { stopAnim(); return; }
    const axis = parseInt(animAxis.value, 10);
    const amp = Math.abs(parseFloat(animAmp.value)) || 0;
    const freq = Math.min(Math.max(parseFloat(animFreq.value) || 0.5, 0.05), 3);
    const t0 = performance.now();
    const basePose = pose.slice();
    animBtn.textContent = '停止';
    animBtn.setAttribute('aria-pressed', 'true');
    const step = (now) => {
      if (!anim) return;
      const t = (now - t0) / 1000;
      // 與 sine_test_loop() 相同：啟用軸以 amp·sin(2πft) 覆寫
      pose.splice(0, 6, ...basePose);
      pose[axis] = amp * Math.sin(2 * Math.PI * freq * t);
      update();
      anim = requestAnimationFrame(step);
    };
    anim = requestAnimationFrame(step);
  });
  function stopAnim() {
    if (anim) cancelAnimationFrame(anim);
    anim = null;
    animBtn.textContent = '播放正弦';
    animBtn.setAttribute('aria-pressed', 'false');
  }

  // ---------- 狀態輸出 ----------
  const statusEl = root.querySelector('[data-status]');
  const tbody = root.querySelector('[data-table] tbody');
  const packetEl = root.querySelector('[data-packet]');
  for (let i = 0; i < 6; i++) {
    const tr = document.createElement('tr');
    tr.innerHTML = `<th scope="row"><span class="swatch" style="--c:${SERIES[i]}"></span>S${i}</th><td></td><td></td><td></td><td></td>`;
    tbody.appendChild(tr);
  }

  function update() {
    inputs.forEach(({ input, out, ax }, i) => {
      input.value = pose[i];
      out.textContent = pose[i].toFixed(1) + ' ' + ax.unit;
    });
    const res = IK.solve(pose, matrix);
    if (res.ok) lastGood = res;
    const shown = res.ok ? res : lastGood;
    const tips = IK.armTips(shown.angles);
    const pwm = IK.toPwm(shown.angles);
    statusEl.className = 'sim-status ' + (res.ok ? 'ok' : 'bad');
    statusEl.textContent = res.ok
      ? 'IK 成功：六支伺服角皆在 ±60° 內'
      : `IK 失敗（S${res.failIndex}：${res.reason}）——韌體會丟棄此封包並維持上一個姿態，畫面同樣保留上一個可行姿態`;
    [...tbody.rows].forEach((tr, i) => {
      const rod = dist(tips[i], shown.pivots[i]);
      tr.cells[1].textContent = shown.angles[i].toFixed(2) + '°';
      tr.cells[2].textContent = pwm[i].us + ' µs';
      tr.cells[3].textContent = pwm[i].count;
      tr.cells[4].textContent = rod.toFixed(2) + ' mm';
    });
    packetEl.textContent = IK.packet(pose);
    draw(shown, tips, res.ok);
  }

  const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

  // ---------- 繪圖 ----------
  function project(p) {
    // 以平台中立高度一半為視點中心
    const x = p[0], y = p[1], z = p[2] - 48;
    const ca = Math.cos(view.az), sa = Math.sin(view.az);
    const ce = Math.cos(view.el), se = Math.sin(view.el);
    const x1 = ca * x - sa * y;
    const y1 = sa * x + ca * y;
    const yz = ce * y1 - se * z;
    const zz = se * y1 + ce * z;
    const depth = view.dist + yz;
    const f = Math.min(canvas.width, canvas.height) / dpr * 1.9;
    const s = f / depth;
    return [canvas.width / dpr / 2 + x1 * s, canvas.height / dpr / 2 - zz * s, depth];
  }

  let dpr = 1;
  function resize() {
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = canvas.clientWidth, h = canvas.clientHeight;
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    update();
  }

  function line(a, b, color, width, dash) {
    const p = project(a), q = project(b);
    ctx.beginPath();
    ctx.setLineDash(dash || []);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.moveTo(p[0], p[1]);
    ctx.lineTo(q[0], q[1]);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  function dot(p3, r, color) {
    const p = project(p3);
    ctx.beginPath();
    ctx.fillStyle = color;
    ctx.arc(p[0], p[1], r, 0, Math.PI * 2);
    ctx.fill();
  }
  function poly(pts, fill, stroke) {
    ctx.beginPath();
    pts.forEach((p3, i) => { const p = project(p3); i ? ctx.lineTo(p[0], p[1]) : ctx.moveTo(p[0], p[1]); });
    ctx.closePath();
    if (fill) { ctx.fillStyle = fill; ctx.fill(); }
    if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1.2; ctx.stroke(); }
  }
  function label(p3, text, color) {
    const p = project(p3);
    ctx.fillStyle = color;
    ctx.font = '600 12px "Noto Sans TC", system-ui, sans-serif';
    ctx.fillText(text, p[0] + 4, p[1] - 4);
  }

  function draw(res, tips, ok) {
    const w = canvas.width / dpr, h = canvas.height / dpr;
    ctx.clearRect(0, 0, w, h);
    // 地面格線
    for (let g = -120; g <= 120; g += 40) {
      line([g, -120, 0], [g, 120, 0], 'rgba(230,234,240,0.06)', 1);
      line([-120, g, 0], [120, g, 0], 'rgba(230,234,240,0.06)', 1);
    }
    // 底座
    const ring = [];
    for (let k = 0; k < 48; k++) { const a = k / 48 * Math.PI * 2; ring.push([92 * Math.cos(a), 92 * Math.sin(a), 0]); }
    poly(ring, 'rgba(115,159,223,0.06)', 'rgba(115,159,223,0.35)');
    poly(IK.base.map((b) => [b[0], b[1], 0]), null, 'rgba(230,234,240,0.18)');
    // 世界座標軸 {B}
    line([0, 0, 0], [60, 0, 0], '#e66767', 2); label([62, 0, 0], 'X', '#e66767');
    line([0, 0, 0], [0, 60, 0], '#3fb27f', 2); label([0, 62, 0], 'Y', '#3fb27f');
    line([0, 0, 0], [0, 0, 60], '#739fdf', 2); label([0, 0, 62], 'Z', '#739fdf');

    // 腿：搖臂 + 連桿
    for (let i = 0; i < 6; i++) {
      const b = [IK.base[i][0], IK.base[i][1], 0];
      line(b, tips[i], SERIES[i], 4);
      line(tips[i], res.pivots[i], 'rgba(230,234,240,0.75)', 1.6);
      dot(b, 3.5, SERIES[i]);
      dot(tips[i], 3, '#e6eaf0');
    }
    // 平台
    poly(res.pivots, ok ? 'rgba(240,138,89,0.16)' : 'rgba(230,103,103,0.18)', ok ? '#f08a59' : '#e66767');
    res.pivots.forEach((p, i) => { dot(p, 3.5, '#f08a59'); label(p, 'P' + i, 'rgba(230,234,240,0.7)'); });
    // 平台座標軸 {P}：以中心與兩條邊中點估計
    const c = res.pivots.reduce((s, p) => [s[0] + p[0] / 6, s[1] + p[1] / 6, s[2] + p[2] / 6], [0, 0, 0]);
    const xr = IK.rotationColumns(pose[3], pose[4], pose[5], matrix);
    const ax = [c[0] + 40 * xr[0][0], c[1] + 40 * xr[1][0], c[2] + 40 * xr[2][0]];
    const ay = [c[0] + 40 * xr[0][1], c[1] + 40 * xr[1][1], c[2] + 40 * xr[2][1]];
    line(c, ax, '#e66767', 1.5, [4, 3]); label(ax, 'x′', '#e66767');
    line(c, ay, '#3fb27f', 1.5, [4, 3]); label(ay, 'y′', '#3fb27f');
    ctx.fillStyle = 'rgba(230,234,240,0.45)';
    ctx.font = '12px "Noto Sans TC", system-ui, sans-serif';
    ctx.fillText('模擬 · 依韌體幾何參數繪製（非 CAD 尺寸）', 12, h - 12);
  }

  // ---------- 視角操作 ----------
  let drag = null;
  canvas.addEventListener('pointerdown', (e) => { drag = { x: e.clientX, y: e.clientY, az: view.az, el: view.el }; canvas.setPointerCapture(e.pointerId); });
  canvas.addEventListener('pointermove', (e) => {
    if (!drag) return;
    view.az = drag.az - (e.clientX - drag.x) * 0.008;
    view.el = Math.max(-0.2, Math.min(1.45, drag.el + (e.clientY - drag.y) * 0.006));
    update();
  });
  const end = () => { drag = null; };
  canvas.addEventListener('pointerup', end);
  canvas.addEventListener('pointercancel', end);
  canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    view.dist = Math.max(300, Math.min(900, view.dist + e.deltaY * 0.4));
    update();
  }, { passive: false });
  canvas.addEventListener('keydown', (e) => {
    const k = { ArrowLeft: [-0.1, 0], ArrowRight: [0.1, 0], ArrowUp: [0, 0.08], ArrowDown: [0, -0.08] }[e.key];
    if (!k) return;
    e.preventDefault();
    view.az += k[0];
    view.el = Math.max(-0.2, Math.min(1.45, view.el + k[1]));
    update();
  });

  window.addEventListener('resize', resize);
  if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) animBtn.title = '已偵測減少動態偏好，可手動播放';
  resize();
})();
