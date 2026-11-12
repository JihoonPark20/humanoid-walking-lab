"""W5: G1의 contact/impact shooting problem과 Box-FDDP.

Crocoddyl 공식 SimpleBipedGaitProblem의 action 생성기를 재사용한다.
https://github.com/loco-3d/crocoddyl (BSD-3-Clause)
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path
from time import perf_counter

import crocoddyl as croc
import mujoco
import numpy as np
import pinocchio as pin
from crocoddyl.utils.biped import SimpleBipedGaitProblem

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "lab-6-whole-body-control" / "src"))
from g1_torque_model import torque_limits
from lab6_common import (
    DT,
    dense_mass_matrix,
    load_g1_pinocchio,
    load_g1_torque_mujoco,
    mj_state_to_pin,
)
from wb_standing_controller import GravityMode, StandingController


@dataclass(frozen=True)
class OCPConfig:
    """짧은 보행 horizon과 G1 contact 모델 설정."""

    steps: int = 1
    step_length: float = 0.06
    step_height: float = 0.025
    dt: float = 0.01
    swing_knots: int = 60
    support_knots: int = 60
    max_iterations: int = 600
    control_weight: float = 3.0

    def __post_init__(self) -> None:
        """첫 실습은 한두 걸음으로 제한한다."""
        if self.steps not in (1, 2):
            raise ValueError("steps는 1 또는 2입니다.")
        if not all(np.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError("설정은 양수여야 합니다.")
        if any(
            not isinstance(v, int)
            for v in (self.swing_knots, self.support_knots, self.max_iterations)
        ):
            raise ValueError("knots/iterations는 정수입니다.")


class G1GaitProblem(SimpleBipedGaitProblem):
    """발바닥 크기, 관절 barrier, torque box를 G1에 맞춘 공식 gait problem."""

    def createSwingFootModel(
        self,
        timeStep: float,
        supportFootIds: list,
        comTask: np.ndarray | None = None,
        swingFootTask: list | None = None,
    ) -> croc.ActionModelAbstract:
        """공식 contact Dynamics에 G1의 실제 발 크기와 상태 범위를 적용한다."""
        model = super().createSwingFootModel(
            timeStep, supportFootIds, comTask, swingFootTask
        )
        # Pinocchio MJCF 로더가 model.armature를 이미 채운다.
        # differential.armature를 추가하면 rotor inertia가 이중 가산된다.
        costs = model.differential.costs
        # Torque 사용 비용을 높여 rigid-contact 해의 공격적인 피드백을 줄인다.
        # Torque box·접촉·목표 오차의 검증 기준은 유지한다.
        costs.costs["ctrlReg"].weight = self.control_weight
        state = self.state
        nu = self.actuation.nu
        for frame in supportFootIds:
            name = self.rmodel.frames[frame].name + "_wrenchCone"
            costs.removeCost(name)
            cone = croc.WrenchCone(np.eye(3), 0.6, np.array([0.10, 0.04]))
            residual = croc.ResidualModelContactWrenchCone(state, frame, cone, nu, True)
            activation = croc.ActivationModelQuadraticBarrier(
                croc.ActivationBounds(cone.lb, cone.ub)
            )
            costs.addCost(
                name, croc.CostModelResidual(state, activation, residual), 1e4
            )
        low = np.full(state.ndx, -np.inf)
        high = np.full(state.ndx, np.inf)
        q0 = self.rmodel.defaultState[: state.nq]
        low[6 : state.nv] = self.rmodel.lowerPositionLimit[7:] - q0[7:]
        high[6 : state.nv] = self.rmodel.upperPositionLimit[7:] - q0[7:]
        low[state.nv + 6 :] = -10.0
        high[state.nv + 6 :] = 10.0
        barrier = croc.ActivationModelQuadraticBarrier(croc.ActivationBounds(low, high))
        residual = croc.ResidualModelState(state, self.rmodel.defaultState, nu)
        costs.addCost(
            "jointBounds", croc.CostModelResidual(state, barrier, residual), 1e5
        )
        model.u_lb = -self.rmodel.effortLimit[6:].copy()
        model.u_ub = self.rmodel.effortLimit[6:].copy()
        return model


def prepare_g1() -> tuple:
    """MuJoCo에서 자세를 안정화하고 동일 상태로 contact optimal control을 시작한다."""
    if pin.__version__.startswith("4.1") and croc.__version__.startswith("3.2"):
        raise RuntimeError(
            "이 Pinocchio/Crocoddyl wheel 조합은 contact Dynamics에서 충돌합니다. Lab 10 requirements로 .venv-ocp를 사용하세요."
        )
    mj_model, mj_data = load_g1_torque_mujoco(timestep=DT)
    model, data = load_g1_pinocchio()
    standing = StandingController(
        mj_model, model, data, gravity_mode=GravityMode.CONTACT_CONSISTENT
    )
    for _ in range(round(1 / DT)):
        standing.step(mj_data)
    q, v = mj_state_to_pin(mj_data)
    # 발목 대신 실제 발바닥 중심을 contact/impulse frame으로 사용한다.
    for side in ("left", "right"):
        frame = model.frames[model.getFrameId(side + "_ankle_roll_link")]
        placement = frame.placement * pin.SE3(np.eye(3), np.array([0.035, 0.0, -0.035]))
        model.addFrame(
            pin.Frame(
                side + "_sole", frame.parentJoint, 0, placement, pin.FrameType.OP_FRAME
            )
        )
    model.effortLimit = np.r_[
        np.zeros(6),
        np.minimum(-torque_limits(mj_model)[:, 0], torque_limits(mj_model)[:, 1]),
    ]
    # Crocoddyl FloatingBase actuation은 root_joint 이름으로 비구동 DoF를 찾는다.
    # 이름만 바꾸며 joint index와 좌표 순서는 유지한다.
    model.names[1] = "root_joint"
    model.referenceConfigurations["half_sitting"] = q.copy()
    return mj_model, mj_data, model, np.r_[q, v]


def build_problem(config: OCPConfig) -> tuple:
    """DS → swing → impact → DS를 만들고 두 번째 걸음에서는 발을 모은다."""
    mj_model, mj_data, model, x0 = prepare_g1()
    gait = G1GaitProblem(model, "right_sole", "left_sole")
    gait.control_weight = config.control_weight
    if gait.actuation.nu != mj_model.nu:
        raise ValueError("floating-base actuation 차원이 일치하지 않습니다.")
    data = model.createData()
    pin.forwardKinematics(model, data, x0[: model.nq])
    pin.updateFramePlacements(model, data)
    com = pin.centerOfMass(model, data, x0[: model.nq]).copy()
    positions = {
        side: data.oMf[model.getFrameId(side + "_sole")].translation.copy()
        for side in ("left", "right")
    }
    running = []
    phases = []
    times = [0.0]
    targets = []
    both = [gait.lfId, gait.rfId]

    def add(action: croc.ActionModelAbstract, phase: str, duration: float) -> None:
        """각 action의 물리 시간과 접촉 label을 저장한다."""
        running.append(action)
        phases.append(phase)
        times.append(times[-1] + duration)

    for _ in range(config.support_knots):
        add(gait.createSwingFootModel(config.dt, both, com.copy()), "double", config.dt)
    for step in range(config.steps):
        side = "right" if step == 0 else "left"
        swing = gait.rfId if side == "right" else gait.lfId
        stance = gait.lfId if side == "right" else gait.rfId
        foot = positions[side].copy()
        models = gait.createFootstepModels(
            com,
            [foot],
            config.step_length,
            config.step_height,
            config.dt,
            config.swing_knots,
            [stance],
            [swing],
        )
        # 양발 착지 impulse는 기존 stance의 속도도 동시에 0으로 투영한다.
        target = pin.SE3(np.eye(3), positions[side] + [config.step_length, 0, 0])
        models[-1] = gait.createImpulseModel(both, [[swing, target]])
        for action in models[:-1]:
            add(action, side + "_swing", config.dt)
        add(models[-1], "impact", 0.0)
        positions[side] = target.translation.copy()
        targets.append((swing, target))
        for _ in range(config.support_knots):
            add(
                gait.createSwingFootModel(config.dt, both, com.copy()),
                "double",
                config.dt,
            )
    terminal = gait.createSwingFootModel(
        0.0, both, com.copy(), [[frame, target] for frame, target in targets]
    )
    problem = croc.ShootingProblem(x0, running, terminal)
    return problem, gait, mj_model, mj_data, np.asarray(times), phases, targets


def solve_problem(config: OCPConfig) -> tuple:
    """Box-FDDP를 실행한다. Convergence와 독립 rollout 판정을 따로 반환한다."""
    problem, gait, mj_model, mj_data, times, phases, targets = build_problem(config)
    mass_errors = audit_mass_matrices(problem, gait, mj_model, mj_data)
    if max(mass_errors.values()) > 1e-8:
        raise ValueError(f"MuJoCo/Pinocchio/Crocoddyl 질량 행렬 불일치: {mass_errors}")
    solver = croc.SolverBoxFDDP(problem)
    solver.th_stop = 1e-4
    xs = [problem.x0.copy() for _ in range(problem.T + 1)]
    us = problem.quasiStatic(xs[:-1])
    started = perf_counter()
    converged = solver.solve(xs, us, config.max_iterations, False)
    seconds = perf_counter() - started
    report = validate_solution(solver, gait, targets)
    report.update(
        converged=bool(converged),
        iterations=int(solver.iter),
        solve_seconds=seconds,
        objective=float(solver.cost),
        duration_s=float(times[-1]),
        nodes=problem.T,
        stopping_criterion=float(solver.stop),
        mass_matrix_errors=mass_errors,
    )
    return solver, gait, mj_model, mj_data, times, phases, report


def audit_mass_matrices(
    problem: croc.ShootingProblem,
    gait: G1GaitProblem,
    mj_model: mujoco.MjModel,
    mj_data: mujoco.MjData,
) -> dict:
    """Free-flyer 속도 좌표계를 맞춰 세 엔진의 contact/impact 관성을 비교한다."""
    mujoco.mj_forward(mj_model, mj_data)
    q, v = mj_state_to_pin(mj_data)
    x = np.r_[q, v]
    dense = dense_mass_matrix(mj_model, mj_data)
    transform = np.eye(mj_model.nv)
    transform[:3, :3] = pin.Quaternion(q[3:7]).matrix()
    expected = transform.T @ dense @ transform
    matrices = {"pinocchio": pin.crba(gait.rmodel, gait.rmodel.createData(), q)}
    for name, action in (
        ("contact", problem.runningModels[0]),
        ("impact", next(a for a in problem.runningModels if a.nu == 0)),
    ):
        data = action.createData()
        if action.nu:
            action.calc(data, x, np.zeros(action.nu))
            matrices[name] = data.differential.pinocchio.M.copy()
        else:
            action.calc(data, x)
            matrices[name] = data.pinocchio.M.copy()
    return {
        name: float(np.max(abs(np.triu(matrix) + np.triu(matrix, 1).T - expected)))
        for name, matrix in matrices.items()
    }


def validate_solution(
    solver: croc.SolverBoxFDDP, gait: G1GaitProblem, targets: list
) -> dict:
    """Manifold rollout gap, torque/관절/wrench 범위와 최종 발 위치를 검사한다."""
    problem = solver.problem
    model = gait.rmodel
    state = gait.state
    rollout = problem.rollout(solver.us)
    gap = max(
        float(np.linalg.norm(state.diff(x, y), np.inf))
        for x, y in zip(solver.xs, rollout)
    )
    torque = joint = wrench = velocity = impulse = impact_q = 0.0
    for action, data, x, u in zip(
        problem.runningModels, problem.runningDatas, solver.xs, solver.us
    ):
        if action.nu:
            torque = max(
                torque,
                float(
                    np.maximum(np.maximum(action.u_lb - u, u - action.u_ub), 0).max()
                ),
            )
            action.calc(data, x, u)
            costs = data.differential.costs.costs.todict()
            for key in costs:
                if key.endswith("_wrenchCone"):
                    item = action.differential.costs.costs[key]
                    cone = item.cost.residual.reference
                    residual = costs[key].residual.r
                    wrench = max(
                        wrench,
                        float(
                            np.maximum(
                                np.maximum(cone.lb - residual, residual - cone.ub), 0
                            ).max()
                        ),
                    )
        if not action.nu:
            action.calc(data, x)
            cone = croc.WrenchCone(np.eye(3), 0.6, np.array([0.10, 0.04]))
            for item in data.multibody.impulses.impulses.todict().values():
                value = cone.A @ item.f.vector
                impulse = max(
                    impulse,
                    float(
                        np.maximum(
                            np.maximum(cone.lb - value, value - cone.ub), 0
                        ).max()
                    ),
                )
            impact_q = max(
                impact_q,
                float(
                    np.linalg.norm(
                        pin.difference(model, x[: model.nq], data.xnext[: model.nq])
                    )
                ),
            )
    for x in solver.xs:
        joint = max(
            joint,
            float(
                np.maximum(
                    np.maximum(
                        model.lowerPositionLimit[7:] - x[7 : model.nq],
                        x[7 : model.nq] - model.upperPositionLimit[7:],
                    ),
                    0,
                ).max()
            ),
        )
        velocity = max(
            velocity, float(np.maximum(abs(x[model.nq + 6 :]) - 10.0, 0).max())
        )
    terminal = solver.xs[-1]
    data = model.createData()
    pin.forwardKinematics(model, data, terminal[: model.nq])
    pin.updateFramePlacements(model, data)
    foot_error = max(
        float(np.linalg.norm(data.oMf[frame].translation - target.translation))
        for frame, target in targets
    )
    quaternion = max(abs(float(np.linalg.norm(x[3:7])) - 1) for x in solver.xs)
    valid = (
        gap < 1e-5
        and torque < 1e-6
        and joint < 1e-3
        and wrench < 0.5
        and foot_error < 0.02
        and quaternion < 1e-9
        and velocity < 1e-3
        and impulse < 0.01
        and impact_q < 1e-9
    )
    return {
        "model_rollout_valid": bool(valid),
        "rollout_gap": gap,
        "torque_violation_nm": torque,
        "joint_violation_rad": joint,
        "wrench_violation": wrench,
        "final_foot_error_m": foot_error,
        "quaternion_norm_error": quaternion,
        "joint_velocity_violation": velocity,
        "impulse_cone_violation": impulse,
        "impact_position_jump": impact_q,
    }
