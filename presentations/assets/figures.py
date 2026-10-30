"""Generate slide figures from the firmware geometry and inverse kinematics.

Every number here comes from firmware/ESP32C6_PCA/stewart_kinematics.h.
The plots are computed (simulation), not hardware measurements.

Run from the repository root:
    python presentations/assets/figures.py
"""

from math import asin, atan2, cos, degrees, radians, sin, sqrt
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

OUT = Path(__file__).resolve().parent / "fig"
OUT.mkdir(exist_ok=True)

# ---- Firmware constants (stewart_kinematics.h) ----
BASE_RADIUS = 68.0
PLATFORM_RADIUS = 68.0
L1 = 24.0  # SERVO_ARM_LENGTH
L2 = 99.0  # CONNECTING_ARM_LENGTH
H0 = 96.04  # PLATFORM_HEIGHT
THETA_R = 30.0
THETA_P = 30.0
THETA_S = [150, -90, 30, 150, -90, 30]
SERVO_MIN, SERVO_MAX = -60.0, 60.0
DX_MULT = [1, 1, 1, -1, -1, -1]
ANG_MULT = [1, -1, 1, 1, -1, 1]
OFFSET = [radians(30), radians(30), radians(-90), radians(-90), radians(30), radians(30)]

# ---- Theme ----
BG = "#0b1020"
PANEL = "#121a2e"
GRID = "#25304a"
TEXT = "#e6edf7"
MUTED = "#8a9bb5"
TEAL = "#2bd4a0"
ORANGE = "#f28c38"
BLUE = "#4ea8de"
RED = "#ef5b6b"

for f in ("C:/Windows/Fonts/msjh.ttc", "C:/Windows/Fonts/msjhbd.ttc"):
    if Path(f).exists():
        font_manager.fontManager.addfont(f)
plt.rcParams.update({
    "font.family": ["Microsoft JhengHei", "DejaVu Sans"],
    "axes.unicode_minus": False,
    "figure.facecolor": BG,
    "axes.facecolor": PANEL,
    "axes.edgecolor": GRID,
    "axes.labelcolor": TEXT,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "text.color": TEXT,
    "grid.color": GRID,
    "font.size": 13,
})


def geometry():
    """calculateGeometry(): platform and base joint coordinates (xy)."""
    P, B = [], []
    for i in range(6):
        a = OFFSET[i] + ANG_MULT[i] * radians(THETA_R)
        P.append((DX_MULT[i] * BASE_RADIUS * cos(a), BASE_RADIUS * sin(a)))
        b = OFFSET[i] + ANG_MULT[i] * radians(THETA_P)
        B.append((DX_MULT[i] * PLATFORM_RADIUS * cos(b), PLATFORM_RADIUS * sin(b)))
    return P, B


P_XY, B_XY = geometry()


def ik(pos, rot, fixed=True):
    """calculateServoAngles(); fixed=False reproduces the pre-2376aef row 2."""
    roll, pitch, yaw = (radians(v) for v in rot)
    cr, sr, cp, sp, cy, sy = cos(roll), sin(roll), cos(pitch), sin(pitch), cos(yaw), sin(yaw)
    angles, pivots = [], []
    for i in range(6):
        px0, py0 = P_XY[i]
        bx, by = B_XY[i]
        x = px0 * cr * cy + py0 * (sp * sr * cy - cp * sy) + pos[0]
        if fixed:
            y = px0 * cr * sy + py0 * (cp * cy + sr * sp * sy) + pos[1]
        else:
            y = px0 * cp * sy + py0 * (cr * cy + sr * sp * sy) + pos[1]
        z = -px0 * sr + py0 * sp * cr + H0 + pos[2]
        d = sqrt((bx - x) ** 2 + (by - y) ** 2 + z ** 2)
        if abs(d - L1) > L2:
            return None, None
        l = d * d - (L2 * L2 - L1 * L1)
        m = 2 * L1 * z
        ts = radians(THETA_S[i])
        n = 2 * L1 * (cos(ts) * (x - bx) + sin(ts) * (y - by))
        disc = m * m + n * n
        if disc <= 0 or abs(l / sqrt(disc)) > 1:
            return None, None
        a = degrees(asin(l / sqrt(disc)) - atan2(n, m))
        if a < SERVO_MIN or a > SERVO_MAX:
            return None, None
        angles.append(a)
        pivots.append((x, y, z))
    return angles, pivots


def arm_end(i, alpha_deg):
    a, ts = radians(alpha_deg), radians(THETA_S[i])
    bx, by = B_XY[i]
    return (bx + L1 * cos(a) * cos(ts), by + L1 * cos(a) * sin(ts), L1 * sin(a))


def save(fig, name):
    fig.savefig(OUT / name, dpi=200, facecolor=BG, bbox_inches="tight", pad_inches=0.25)
    plt.close(fig)


# ---------- 1. Top view: joint layout and coordinate frame ----------
def fig_geometry():
    fig, ax = plt.subplots(figsize=(7.2, 6.4))
    ax.set_aspect("equal")
    ring = plt.Circle((0, 0), BASE_RADIUS, fill=False, ls="--", lw=1, color=GRID)
    ax.add_patch(ring)
    xs = [p[0] for p in B_XY] + [B_XY[0][0]]
    ys = [p[1] for p in B_XY] + [B_XY[0][1]]
    ax.plot(xs, ys, color=MUTED, lw=1, alpha=0.6)
    for i, (bx, by) in enumerate(B_XY):
        ts = radians(THETA_S[i])
        ax.annotate("", xy=(bx + L1 * 1.25 * cos(ts), by + L1 * 1.25 * sin(ts)), xytext=(bx, by),
                    arrowprops=dict(arrowstyle="-|>", color=ORANGE, lw=2))
        ax.scatter([bx], [by], s=90, color=TEAL, zorder=5, edgecolor=BG, linewidth=1.5)
        r = sqrt(bx * bx + by * by)
        ax.text(bx * 1.24, by * 1.24, f"S{i}\nθs={THETA_S[i]}°", ha="center", va="center",
                fontsize=11, color=TEXT)
    ax.annotate("", xy=(40, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2.2))
    ax.annotate("", xy=(0, 40), xytext=(0, 0), arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2.2))
    ax.text(43, -6, "X", color=BLUE, fontsize=14, weight="bold")
    ax.text(-8, 43, "Y", color=BLUE, fontsize=14, weight="bold")
    ax.scatter([0], [0], s=40, color=BLUE)
    ax.text(3, -9, "Z ⊙", color=BLUE, fontsize=12)
    ax.set_xlim(-105, 105)
    ax.set_ylim(-105, 105)
    ax.grid(True, lw=0.5)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.scatter([], [], s=60, color=TEAL, label="基座接點 Bᵢ（俯視與平台接點 Pᵢ 重合）")
    ax.plot([], [], color=ORANGE, lw=2, label="伺服搖臂方向 θs")
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, -0.27), frameon=False, fontsize=11, ncol=1)
    save(fig, "geometry-top.png")


# ---------- 2. 3D simulation of two poses ----------
def draw_pose(ax, pos, rot, title):
    angles, piv = ik(pos, rot)
    ax.set_facecolor(BG)
    for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
        pane.set_pane_color((0.07, 0.10, 0.18, 1))
        pane._axinfo["grid"]["color"] = GRID
    bx = [p[0] for p in B_XY] + [B_XY[0][0]]
    by = [p[1] for p in B_XY] + [B_XY[0][1]]
    ax.plot(bx, by, [0] * 7, color=MUTED, lw=2)
    px = [p[0] for p in piv] + [piv[0][0]]
    py = [p[1] for p in piv] + [piv[0][1]]
    pz = [p[2] for p in piv] + [piv[0][2]]
    ax.plot(px, py, pz, color=TEAL, lw=2.5)
    ax.plot_trisurf([p[0] for p in piv], [p[1] for p in piv], [p[2] for p in piv],
                    color=TEAL, alpha=0.18, linewidth=0)
    worst = 0.0
    for i in range(6):
        a = arm_end(i, angles[i])
        ax.plot([B_XY[i][0], a[0]], [B_XY[i][1], a[1]], [0, a[2]], color=ORANGE, lw=3)
        ax.plot([a[0], piv[i][0]], [a[1], piv[i][1]], [a[2], piv[i][2]], color=BLUE, lw=1.6)
        rod = sqrt(sum((piv[i][k] - a[k]) ** 2 for k in range(3)))
        worst = max(worst, abs(rod - L2))
    ax.set_xlim(-85, 85)
    ax.set_ylim(-85, 85)
    ax.set_zlim(0, 125)
    ax.set_box_aspect((1, 1, 0.75))
    ax.view_init(elev=18, azim=-62)
    ax.tick_params(labelsize=8, colors=MUTED)
    ax.set_title(title, color=TEXT, fontsize=14, pad=4)
    txt = "  ".join(f"S{i}:{a:+.1f}°" for i, a in enumerate(angles))
    ax.text2D(0.5, -0.02, txt, transform=ax.transAxes, ha="center", fontsize=9.5, color=MUTED)
    return worst


def fig_sim3d():
    fig = plt.figure(figsize=(12, 5.6))
    poses = [((0, 0, 0), (0, 0, 0), "中立姿態  (0, 0, 0 | 0°, 0°, 0°)"),
             ((0, 0, 0), (10, 0, 0), "Roll = 10°"),
             ((0, 0, 10), (0, 0, 15), "z = +10 mm，Yaw = 15°")]
    worst = 0.0
    for k, (p, r, t) in enumerate(poses):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d")
        worst = max(worst, draw_pose(ax, p, r, t))
    fig.subplots_adjust(wspace=0.02, left=0.0, right=1.0)
    save(fig, "sim-3d.png")
    return worst


# ---------- 3. Single-axis reachable range ----------
AXES = [("x", 0, "mm", 50), ("y", 1, "mm", 50), ("z", 2, "mm", 50),
        ("roll", 3, "°", 30), ("pitch", 4, "°", 30), ("yaw", 5, "°", 30)]


def reach(idx, sign, step=0.05, limit=120.0):
    v, last = 0.0, 0.0
    while v <= limit:
        q = [0.0] * 6
        q[idx] = sign * v
        if ik(q[:3], q[3:])[0] is None:
            break
        last = v
        v += step
    return sign * last


def fig_workspace():
    rows = [(n, u, g, reach(i, -1), reach(i, 1)) for n, i, u, g in AXES]
    fig, ax = plt.subplots(figsize=(9, 4.6))
    for k, (n, u, g, lo, hi) in enumerate(rows):
        y = len(rows) - 1 - k
        scale = 1.0 / g
        ax.barh(y, (hi - lo) * scale, left=lo * scale, height=0.5, color=TEAL, alpha=0.85)
        ax.plot([-1, 1], [y + 0.36, y + 0.36], color=MUTED, lw=1, ls=(0, (3, 3)))
        ax.text(1.62, y, f"{lo:+.1f} ～ {hi:+.1f} {u}", va="center", fontsize=15, color=TEXT)
    ax.set_yticks(range(len(rows)))
    ax.set_yticklabels([f"{n.upper()}  (GUI ±{g} {u})" for n, u, g, _, _ in reversed(rows)])
    ax.axvline(0, color=GRID, lw=1)
    ax.set_xlim(-1.6, 2.75)
    ax.set_xticks([-1, -0.5, 0, 0.5, 1])
    ax.set_xticklabels(["-GUI 上限", "", "0", "", "+GUI 上限"])
    ax.grid(True, axis="x", lw=0.5)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    save(fig, "workspace.png")
    return rows


# ---------- 4. Servo angle response to roll (shows the coupled solution) ----------
def fig_sweep():
    xs = [v / 10 for v in range(-200, 201)]
    series = {i: [] for i in range(6)}
    xv = []
    for r in xs:
        a, _ = ik((0, 0, 0), (r, 0, 0))
        if a is None:
            continue
        xv.append(r)
        for i in range(6):
            series[i].append(a[i])
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    colors = [TEAL, ORANGE, BLUE, "#c38cf2", "#f2d13a", RED]
    for i in range(6):
        ax.plot(xv, series[i], color=colors[i], lw=2, label=f"S{i}")
    ax.axhspan(60, 70, color=RED, alpha=0.12)
    ax.axhspan(-70, -60, color=RED, alpha=0.12)
    ax.set_ylim(-70, 70)
    ax.set_xlabel("Roll 命令 (°)")
    ax.set_ylabel("伺服角 α (°)")
    ax.grid(True, lw=0.5)
    ax.legend(ncol=6, loc="upper center", frameon=False, fontsize=10, bbox_to_anchor=(0.5, 1.12))
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    save(fig, "roll-sweep.png")


# ---------- 5. Rotation-matrix fix: old vs new ----------
def rotation_fix_delta():
    out = []
    for t in (5, 10, 15):
        worst = 0.0
        for rot in ((t, 0, 0), (0, t, 0)):
            new, _ = ik((0, 0, 0), rot, fixed=True)
            old, _ = ik((0, 0, 0), rot, fixed=False)
            worst = max(worst, max(abs(a - b) for a, b in zip(new, old)))
        out.append((t, worst))
    return out


if __name__ == "__main__":
    fig_geometry()
    rod_err = fig_sim3d()
    rows = fig_workspace()
    fig_sweep()
    print(f"neutral alpha: {ik((0, 0, 0), (0, 0, 0))[0][0]:.5f} deg")
    print(f"sqrt(L2^2-L1^2) = {sqrt(L2 * L2 - L1 * L1):.3f} mm")
    print(f"max rod-length residual in 3D sim: {rod_err:.2e} mm")
    for n, u, g, lo, hi in rows:
        print(f"reach {n}: {lo:+.2f} .. {hi:+.2f} {u}")
    for t, d in rotation_fix_delta():
        print(f"rotation fix, single-axis tilt {t} deg: max servo delta {d:.3f} deg")
