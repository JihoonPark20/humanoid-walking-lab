"""Forward Kinematics로 관절각에서 끝점 pose를 계산하고 검증한다.

Geometric FK (not DH — overkill for 2-link):
  x = L1*cos(θ1) + L2*cos(θ1 + θ2)
  y = L1*sin(θ1) + L2*sin(θ1 + θ2)

Verification: We compare our FK result with MuJoCo's site_xpos value."""
import numpy as np
import mujoco
import matplotlib.pyplot as plt

MODEL_PATH = "models/two_link.xml"
L1 = 0.3    # link1 uzunlugu (m)
L2 = 0.3    # link2 uzunlugu (m)
EE_OFFSET = 0.015  # end_effector site offset (model xml'deki pos="0.015 0 0")


# ── FK Functions ──────────────────────────────────────────────

def fk_endeffector(theta1: float, theta2: float, include_site_offset: bool = False) -> np.ndarray:
    """끝점의 (x, y) 위치를 계산한다.
    
    인자:
        theta1, theta2: 관절각 (rad).
        include_site_offset: MuJoCo와 비교할 때 끝점 site offset을 포함한다.
    
    반환:
        np.array([x, y]), 단위 m."""
    total_angle = theta1 + theta2
    L2_eff = L2 + (EE_OFFSET if include_site_offset else 0)
    x = L1 * np.cos(theta1) + L2_eff * np.cos(total_angle)
    y = L1 * np.sin(theta1) + L2_eff * np.sin(total_angle)
    return np.array([x, y])


def fk_all_joints(theta1: float, theta2: float) -> dict:
    """Base, 두 번째 관절, 끝점의 좌표를 반환한다. 키는 base, joint2, ee이다."""
    # Base (sabit)
    base = np.array([0.0, 0.0])

    # Joint2 position (tip of link1)
    j2 = np.array([L1 * np.cos(theta1), L1 * np.sin(theta1)])

    # End-effector
    ee = fk_endeffector(theta1, theta2)

    return {"base": base, "joint2": j2, "ee": ee}


def fk_homogeneous(theta1: float, theta2: float) -> np.ndarray:
    """T01 @ T12로 2차원 FK를 계산하고 3×3 동차 변환 행렬을 반환한다."""
    # T01: base -> joint2
    T01 = np.array([
        [np.cos(theta1), -np.sin(theta1), L1 * np.cos(theta1)],
        [np.sin(theta1),  np.cos(theta1), L1 * np.sin(theta1)],
        [0,               0,              1]
    ])

    # T12: joint2 -> end-effector
    T12 = np.array([
        [np.cos(theta2), -np.sin(theta2), L2 * np.cos(theta2)],
        [np.sin(theta2),  np.cos(theta2), L2 * np.sin(theta2)],
        [0,               0,              1]
    ])

    T02 = T01 @ T12
    return T02


# ── MuJoCo Verification ────────────────────────────────────────────

def mujoco_fk(theta1: float, theta2: float) -> np.ndarray:
    """MuJoCo에서 끝점 위치를 읽는다."""
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    data.qpos[0] = theta1
    data.qpos[1] = theta2
    mujoco.mj_forward(model, data)

    ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "end_effector")
    return data.site_xpos[ee_id][:2].copy()  # x, y only


# ── Comparison Table ─────────────────────────────────────────────

def comparison_table():
    """여러 관절각에서 해석 FK와 MuJoCo 위치를 비교한다."""
    test_angles = [
        (0, 0),
        (30, 0),
        (0, 45),
        (30, 45),
        (90, -45),
        (-45, 90),
        (60, 60),
        (180, 0),
        (45, -90),
        (-30, -60),
    ]

    print(f"\n{'='*75}")
    print("FK vs MuJoCo Comparison Table")
    print(f"{'='*75}")
    print(f"{'θ1':>6} {'θ2':>6} | {'FK_x':>8} {'FK_y':>8} | {'MJ_x':>8} {'MJ_y':>8} | {'Hata':>8}")
    print(f"{'-'*75}")

    max_error = 0.0
    for deg1, deg2 in test_angles:
        t1, t2 = np.radians(deg1), np.radians(deg2)

        fk_pos = fk_endeffector(t1, t2, include_site_offset=True)
        mj_pos = mujoco_fk(t1, t2)
        error = np.linalg.norm(fk_pos - mj_pos)
        max_error = max(max_error, error)

        # Geometric (offset'siz) ile Homogeneous ayni olmali
        fk_pure = fk_endeffector(t1, t2, include_site_offset=False)
        T = fk_homogeneous(t1, t2)
        hom_pos = T[:2, 2]
        hom_error = np.linalg.norm(fk_pure - hom_pos)
        assert hom_error < 1e-10, f"Geometric vs Homogeneous mismatch! error={hom_error}"

        print(
            f"{deg1:+6.0f}° {deg2:+6.0f}° | "
            f"{fk_pos[0]:+8.4f} {fk_pos[1]:+8.4f} | "
            f"{mj_pos[0]:+8.4f} {mj_pos[1]:+8.4f} | "
            f"{error:.6f}"
        )

    print(f"{'-'*75}")
    print(f"Max error: {max_error:.6f} m")

    if max_error < 0.02:
        print("SUCCESS: FK and MuJoCo match.")
    else:
        print(f"WARNING: Error is large ({max_error:.4f}m). Check site offset.")

    return max_error


# ── Matplotlib Plot ─────────────────────────────────────────────

def plot_robot(theta1_deg: float, theta2_deg: float, ax=None):
    """하나의 관절 자세를 link 선분으로 그린다."""
    t1 = np.radians(theta1_deg)
    t2 = np.radians(theta2_deg)
    pts = fk_all_joints(t1, t2)

    if ax is None:
        _, ax = plt.subplots(1, 1, figsize=(6, 6))

    # Linkler
    xs = [pts["base"][0], pts["joint2"][0], pts["ee"][0]]
    ys = [pts["base"][1], pts["joint2"][1], pts["ee"][1]]
    ax.plot(xs, ys, "o-", linewidth=3, markersize=8, color="steelblue", label="links")

    # Noktalar
    ax.plot(*pts["base"], "ks", markersize=12, label="base")
    ax.plot(*pts["ee"], "r^", markersize=10, label=f"EE ({pts['ee'][0]:.3f}, {pts['ee'][1]:.3f})")

    ax.set_xlim(-0.7, 0.7)
    ax.set_ylim(-0.7, 0.7)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title(f"θ1={theta1_deg:.0f}°, θ2={theta2_deg:.0f}°")


def plot_multiple_configs():
    """여러 자세를 나란히 그려 비교한다."""
    configs = [
        (0, 0),
        (30, 45),
        (90, -45),
        (-45, 90),
        (60, 60),
        (180, 0),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(14, 9))
    fig.suptitle("Forward Kinematics — Different Configurations", fontsize=14)

    for ax, (t1, t2) in zip(axes.flat, configs):
        plot_robot(t1, t2, ax)

    plt.tight_layout()
    plt.savefig("docs/a2_fk_configurations.png", dpi=120)
    print("\nPlot saved: docs/a2_fk_configurations.png")
    plt.show()


# ── Workspace Plot ─────────────────────────────────────────────

def plot_workspace():
    """로봇 끝점이 도달할 수 있는 workspace를 그린다."""
    points = []
    for t1 in np.linspace(-np.pi, np.pi, 200):
        for t2 in np.linspace(-np.pi, np.pi, 200):
            ee = fk_endeffector(t1, t2)
            points.append(ee)

    points = np.array(points)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.scatter(points[:, 0], points[:, 1], s=0.1, alpha=0.3, c="steelblue")
    ax.set_xlim(-0.7, 0.7)
    ax.set_ylim(-0.7, 0.7)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title("Robot Workspace (Reachable Area)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    # Ic ve dis daire
    theta = np.linspace(0, 2 * np.pi, 100)
    ax.plot((L1 + L2) * np.cos(theta), (L1 + L2) * np.sin(theta), "r--", alpha=0.5, label=f"Outer limit r={L1+L2:.1f}m")
    ax.plot(abs(L1 - L2) * np.cos(theta), abs(L1 - L2) * np.sin(theta), "g--", alpha=0.5, label=f"Inner limit r={abs(L1-L2):.1f}m")
    ax.legend()

    plt.tight_layout()
    plt.savefig("docs/a2_fk_workspace.png", dpi=120)
    print("\nWorkspace plot saved: docs/a2_fk_workspace.png")
    plt.show()


if __name__ == "__main__":
    print("=" * 60)
    print("A2: FORWARD KINEMATICS")
    print("=" * 60)

    # 1) Manual calculation verification
    print("\n--- Manual Calculation: θ1=30°, θ2=45° ---")
    t1, t2 = np.radians(30), np.radians(45)
    ee = fk_endeffector(t1, t2)
    print(f"EE position: x={ee[0]:.4f}, y={ee[1]:.4f}")
    print(f"  Expected: x = 0.3*cos(30°) + 0.3*cos(75°) = {0.3*np.cos(np.radians(30)) + 0.3*np.cos(np.radians(75)):.4f}")
    print(f"  Expected: y = 0.3*sin(30°) + 0.3*sin(75°) = {0.3*np.sin(np.radians(30)) + 0.3*np.sin(np.radians(75)):.4f}")

    # 2) Homogeneous transformation verification
    T = fk_homogeneous(t1, t2)
    print(f"\nHomogeneous T02:\n{T}")
    print(f"EE from T: ({T[0,2]:.4f}, {T[1,2]:.4f})")

    # 3) FK vs MuJoCo table
    comparison_table()

    # 4) Visualization
    print("\n--- Plot Drawing ---")
    plot_multiple_configs()
    plot_workspace()
