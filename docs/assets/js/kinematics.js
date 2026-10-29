/*
 * Stewart 平台逆運動學 —— 逐行移植自 firmware/ESP32C6_PCA/stewart_kinematics.h
 * 以及 ESP32C6_PCA.ino 的角度→脈寬映射。僅供網頁模擬使用，不會控制硬體。
 *
 * matrix: 'firmware' = 韌體實際使用的展開式（預設，與實機一致）
 *         'zyx'      = 標準 Rz(ψ)·Ry(φ)·Rx(θ)，用來比較差異
 */
(function (global) {
  'use strict';

  const CFG = {
    BASE_RADIUS: 68.0,
    PLATFORM_RADIUS: 68.0,
    SERVO_ARM_LENGTH: 24.0,       // L1
    CONNECTING_ARM_LENGTH: 99.0,  // L2
    PLATFORM_HEIGHT: 96.04,       // h0
    THETA_R: 30.0,
    THETA_P: 30.0,
    THETA_S: [150, -90, 30, 150, -90, 30],
    SERVO_MIN_ANGLE: -60.0,
    SERVO_MAX_ANGLE: 60.0,
    DX_MULTIPLIER: [1, 1, 1, -1, -1, -1],
    ANGLE_MULTIPLIER: [1, -1, 1, 1, -1, 1],
    OFFSET_ANGLE: [Math.PI / 6, Math.PI / 6, -Math.PI / 2, -Math.PI / 2, Math.PI / 6, Math.PI / 6],
    // ESP32C6_PCA.ino
    SERVO_FREQ: 330.0,
    PCA_CALIBRATION_OFFSET: 1.7,
    PCA_CALIBRATION_SCALE: 0.935,
    CHANNEL_OFFSET: [80, 100, 100, 0, 100, 50],
    INVERT_SERVO: [true, false, true, false, true, false],
  };

  const deg2rad = (d) => d * Math.PI / 180;
  const rad2deg = (r) => r * 180 / Math.PI;

  // calculateGeometry()
  const platform = [], base = [];
  for (let i = 0; i < 6; i++) {
    const pa = CFG.OFFSET_ANGLE[i] + CFG.ANGLE_MULTIPLIER[i] * deg2rad(CFG.THETA_R);
    platform.push([CFG.DX_MULTIPLIER[i] * CFG.BASE_RADIUS * Math.cos(pa), CFG.BASE_RADIUS * Math.sin(pa)]);
    const ba = CFG.OFFSET_ANGLE[i] + CFG.ANGLE_MULTIPLIER[i] * deg2rad(CFG.THETA_P);
    base.push([CFG.DX_MULTIPLIER[i] * CFG.PLATFORM_RADIUS * Math.cos(ba), CFG.PLATFORM_RADIUS * Math.sin(ba)]);
  }

  // 回傳作用在平台座標 (px, py, 0) 的前兩欄 [[r00,r01],[r10,r11],[r20,r21]]
  function rotationColumns(rollDeg, pitchDeg, yawDeg, matrix) {
    const r = deg2rad(rollDeg), p = deg2rad(pitchDeg), y = deg2rad(yawDeg);
    const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p);
    const cy = Math.cos(y), sy = Math.sin(y);
    if (matrix === 'zyx') {
      return [
        [cr * cy, sp * sr * cy - cp * sy],
        [cr * sy, cp * cy + sr * sp * sy],
        [-sr, sp * cr],
      ];
    }
    return [
      [cr * cy, sp * sr * cy - cp * sy],
      [cp * sy, cr * cy + sr * sp * sy],
      [-sr, sp * cr],
    ];
  }

  // calculateServoAngles()；失敗時回傳原因，而非只有 false
  function solve(pose, matrix) {
    const [x, y, z, roll, pitch, yaw] = pose;
    const R = rotationColumns(roll, pitch, yaw, matrix || 'firmware');
    const L1 = CFG.SERVO_ARM_LENGTH, L2 = CFG.CONNECTING_ARM_LENGTH;
    const res = { ok: true, reason: null, failIndex: -1, angles: [], pivots: [], legLengths: [] };
    for (let i = 0; i < 6; i++) {
      const [px, py] = platform[i];
      const qx = px * R[0][0] + py * R[0][1] + x;
      const qy = px * R[1][0] + py * R[1][1] + y;
      const qz = px * R[2][0] + py * R[2][1] + CFG.PLATFORM_HEIGHT + z;
      res.pivots.push([qx, qy, qz]);
      const dLx = base[i][0] - qx, dLy = base[i][1] - qy, dLz = -qz;
      const l = Math.sqrt(dLx * dLx + dLy * dLy + dLz * dLz);
      res.legLengths.push(l);
      if (Math.abs(l - L1) > L2) return fail(res, i, '連桿長度不足');
      const L = l * l - (L2 * L2 - L1 * L1);
      const M = 2 * L1 * qz;
      const ts = deg2rad(CFG.THETA_S[i]);
      const N = 2 * L1 * (Math.cos(ts) * (qx - base[i][0]) + Math.sin(ts) * (qy - base[i][1]));
      const disc = M * M + N * N;
      if (disc <= 0) return fail(res, i, '無有效解');
      const k = L / Math.sqrt(disc);
      if (Math.abs(k) > 1) return fail(res, i, 'asin 定義域超出（幾何不可達）');
      const alpha = rad2deg(Math.asin(k) - Math.atan2(N, M));
      if (alpha < CFG.SERVO_MIN_ANGLE || alpha > CFG.SERVO_MAX_ANGLE) {
        res.angles.push(alpha);
        return fail(res, i, '伺服角超出 ±60°');
      }
      res.angles.push(alpha);
    }
    return res;
  }

  function fail(res, i, reason) {
    res.ok = false; res.failIndex = i; res.reason = reason;
    return res;
  }

  // 搖臂末端：b + L1 (cosα cosβ, cosα sinβ, sinα)
  function armTips(angles) {
    return angles.map((a, i) => {
      const al = deg2rad(a), be = deg2rad(CFG.THETA_S[i]);
      const L1 = CFG.SERVO_ARM_LENGTH;
      return [base[i][0] + L1 * Math.cos(al) * Math.cos(be), base[i][1] + L1 * Math.cos(al) * Math.sin(be), L1 * Math.sin(al)];
    });
  }

  // applyServoAngles() → angleToPwm() → writeServoUs()
  function toPwm(angles) {
    return angles.map((a, i) => {
      let ang = CFG.INVERT_SERVO[i] ? -a : a;
      ang = Math.max(-60, Math.min(60, ang));
      const us = Math.max(500, Math.min(2500, 1500 + Math.trunc(ang / 60 * 500)));
      let off = Math.max(0, Math.min(4000, us + CFG.CHANNEL_OFFSET[i]));
      const cal = Math.trunc((off + CFG.PCA_CALIBRATION_OFFSET) / CFG.PCA_CALIBRATION_SCALE + 0.5);
      const period = 1e6 / CFG.SERVO_FREQ;
      const count = Math.trunc(Math.max(0, Math.min(4095, cal * 4096 / period)) + 0.5);
      return { us, offsetUs: off, calibratedUs: cal, count };
    });
  }

  // 與 stewart_control_gui.py calculate_checksum() 相同
  function packet(pose) {
    const v = pose.map((n) => Math.round(n * 100) / 100);
    const cs = v.reduce((s, n) => s + Math.trunc(n * 10), 0) & 0xff;
    return '$P,' + v.map((n) => n.toFixed(2)).join(',') + ',' + cs;
  }

  global.StewartIK = { CFG, platform, base, solve, armTips, toPwm, packet, rotationColumns };
})(window);
