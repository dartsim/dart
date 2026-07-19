#!/usr/bin/env python3
# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the following "BSD-style" License:
#   Redistribution and use in source and binary forms, with or
#   without modification, are permitted provided that the following
#   conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
#   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
#   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
#   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
#   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

"""External-baseline harness for reconstructed FBF paper fixtures in MuJoCo.

The command-line interface prints sparse state rows that remain compatible
with ``fbf_paper_trace.cpp``'s tracked-body CSV.  The reusable
``simulate_small_fixture`` function additionally returns a complete,
per-step trajectory with synchronized ``mj_step`` wall times.  The durable
evidence wrapper in ``scripts/run_fbf_mujoco_evidence.py`` consumes that API.

This is a benchmark/example-only external-comparison artifact:
  - It is not part of the DART library and is not read by any DART library
    target.
  - It imports the optional "mujoco" pypi package lazily and exits 0 with a
    clear message if that package is not installed, so no default build or
    CTest gate depends on it. See pixi.toml's optional "mujoco-baseline"
    feature/environment and the "fbf-mujoco-baseline" task.
  - Published parameters are reproduced where available.  Initial
    penetrations and the turntable geometry/control are DART reconstruction
    choices because the paper does not publish the author scene files.  This
    harness must therefore not be presented as paper-parity evidence.
  - Appendix B's published MuJoCo solver settings are explicit: Newton,
    elliptic cone, and 500 maximum iterations.  The paper says "native
    tolerance" without publishing a number, so the version-specific MuJoCo
    default is intentionally left unset in XML and recorded after model
    compilation by the evidence wrapper.
  - Credits: masonry-arch geometry adapted from ipc-sim/rigid-ipc (MIT); see
    data/mjcf/rigid_ipc_arch/README.md and LICENSE.md.

Usage:
  python3 fbf_paper_mujoco_baseline.py [scenario ...] [--dt SECONDS]
      [--duration SECONDS] [--sample-stride N]
      [--arch-duration SECONDS] [--arch-sample-stride N]

Scenarios (default, no argument: every scenario except the Rigid-IPC arch):
  backspin, incline_mu_0_5, incline_mu_0_4,
  turntable_mu_0_2_omega_2, turntable_mu_0_2_omega_5,
  turntable_mu_0_5_omega_2, turntable_mu_0_5_omega_5,
  masonry_arch_101_rigid_ipc (opt-in: pass it explicitly, or pass "all")

Examples:
  python3 fbf_paper_mujoco_baseline.py
  python3 fbf_paper_mujoco_baseline.py backspin --dt 0.0005
  python3 fbf_paper_mujoco_baseline.py masonry_arch_101_rigid_ipc

CSV rows are written to stdout; human-readable summary/analysis notes (paper
analytic comparisons, qualitative arch-collapse notes, "unavailable" notices)
are written to stderr, so `... > trace.csv` captures only the data rows.
"""

import argparse
import math
import os
import sys
import time

CSV_HEADER = (
    "step,time,scenario,solver,body,x,y,z,vx,vy,vz,up_z,"
    "contacts,exact_solves,warm_starts,fallbacks,residual,status"
)

# --- Paper/fixture constants, mirrored from fbf_paper_trace.cpp. ---
PAPER_DT = 1.0 / 60.0
INCLINE_TAN = 0.5
BACKSPIN_RADIUS = 0.25
BACKSPIN_LINEAR_VELOCITY = 4.0
BACKSPIN_ANGULAR_VELOCITY = -200.0
BACKSPIN_FRICTION = 0.5
BACKSPIN_INITIAL_PENETRATION = 0.005
BACKSPIN_ANALYTIC_V_INF = -11.428571428571429
BACKSPIN_ANALYTIC_OMEGA_INF = -45.714285714285715
INCLINE_INITIAL_PENETRATION = 0.01
TURNTABLE_INITIAL_RADIUS = 1.0
TURNTABLE_RAMP_DURATION = 1.0
TURNTABLE_RIDER_INITIAL_PENETRATION = 0.005
TURNTABLE_RIDER_SIZE = 0.25

# Appendix B publishes these settings for MuJoCo.  It does not publish a
# numeric tolerance, so do not bake the local MuJoCo default into the XML.
PAPER_MUJOCO_SOLVER = "Newton"
PAPER_MUJOCO_CONE = "elliptic"
PAPER_MUJOCO_MAX_ITERATIONS = 500

PAPER_FIGURE_TIMES_S = {
    "backspin": [0.0, 10.0 / 60.0, 50.0 / 60.0, 2.0, 130.0 / 60.0],
    "incline_mu_0_5": [0.0, 2.0],
    "incline_mu_0_4": [0.0, 2.0],
}

SCENARIO_DEFAULT_DURATIONS_S = {
    # Figure 3's final published instant is step 130 at dt=1/60.
    "backspin": 130.0 / 60.0,
    # Section 5.1 explicitly publishes T=2 s.
    "incline_mu_0_5": 2.0,
    "incline_mu_0_4": 2.0,
    # Section 5.3 does not publish a duration. Four seconds matches the DART
    # reconstruction's evidence horizon and is labelled as such in metadata.
    "turntable_mu_0_2_omega_2": 4.0,
    "turntable_mu_0_2_omega_5": 4.0,
    "turntable_mu_0_5_omega_2": 4.0,
    "turntable_mu_0_5_omega_5": 4.0,
}

SMALL_FIXTURE_SCENARIOS = [
    "backspin",
    "incline_mu_0_5",
    "incline_mu_0_4",
    "turntable_mu_0_2_omega_2",
    "turntable_mu_0_2_omega_5",
    "turntable_mu_0_5_omega_2",
    "turntable_mu_0_5_omega_5",
]
ARCH_SCENARIO = "masonry_arch_101_rigid_ipc"
ALL_SCENARIOS = SMALL_FIXTURE_SCENARIOS + [ARCH_SCENARIO]

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
ARCH_XML_PATH = os.path.join(
    REPO_ROOT, "data", "mjcf", "rigid_ipc_arch", "arch-101-stones_mjc.xml"
)


def log(message):
    print(message, file=sys.stderr)


def csv_value(value):
    if isinstance(value, float) and not math.isfinite(value):
        return "nan"
    if isinstance(value, float):
        return f"{value:.17g}"
    return str(value)


def print_header():
    print(CSV_HEADER)


def print_row(step, time_s, scenario, body_name, pos, vel, up_z, contacts):
    fields = [
        step,
        time_s,
        scenario,
        "mujoco",
        body_name,
        pos[0],
        pos[1],
        pos[2],
        vel[0],
        vel[1],
        vel[2],
        up_z,
        contacts,
        0,
        0,
        0,
        float("nan"),
        "mujoco",
    ]
    print(",".join(csv_value(v) for v in fields))


def y_axis_quat(phi):
    """MuJoCo (w, x, y, z) quaternion for a rotation of `phi` about the Y axis,
    formatted as a MuJoCo attribute string."""
    return f"{math.cos(phi / 2.0)} 0 {math.sin(phi / 2.0)} 0"


# --------------------------------------------------------------------------
# Scenario builders: backspin, incline, turntable are procedurally generated
# to mirror fbf_paper_trace.cpp's createBackspinSphere / createInclineCube /
# createTurntableSupport+Rider exactly (same sizes, masses, frictions,
# initial penetration, initial velocities).
# --------------------------------------------------------------------------


def build_backspin_xml(dt):
    r = BACKSPIN_RADIUS
    return f"""
<mujoco>
  <option timestep="{dt}" gravity="0 0 -9.81"
          solver="{PAPER_MUJOCO_SOLVER}" cone="{PAPER_MUJOCO_CONE}"
          iterations="{PAPER_MUJOCO_MAX_ITERATIONS}"/>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 1"
          friction="{BACKSPIN_FRICTION} 0 0"/>
    <body name="backspin_sphere_body"
          pos="0 0 {r - BACKSPIN_INITIAL_PENETRATION}">
      <freejoint/>
      <geom type="sphere" size="{r}" mass="1"
            friction="{BACKSPIN_FRICTION} 0 0"/>
    </body>
  </worldbody>
</mujoco>
"""


def backspin_initial_velocity(data):
    data.qvel[0] = BACKSPIN_LINEAR_VELOCITY
    data.qvel[4] = BACKSPIN_ANGULAR_VELOCITY


def build_incline_xml(dt, mu):
    theta = math.atan(INCLINE_TAN)
    phi = -theta
    quat = y_axis_quat(phi)
    nx, nz = -math.sin(theta), math.cos(theta)
    size_z = 1.0
    offset = 0.5 * size_z - INCLINE_INITIAL_PENETRATION
    px, pz = nx * offset, nz * offset
    return f"""
<mujoco>
  <option timestep="{dt}" gravity="0 0 -9.81"
          solver="{PAPER_MUJOCO_SOLVER}" cone="{PAPER_MUJOCO_CONE}"
          iterations="{PAPER_MUJOCO_MAX_ITERATIONS}"/>
  <worldbody>
    <geom name="incline" type="plane" size="0 0 1" quat="{quat}"
          friction="{mu} 0 0"/>
    <body name="incline_cube_body" pos="{px} 0 {pz}" quat="{quat}">
      <freejoint/>
      <geom type="box" size="0.5 0.5 0.5" mass="1" friction="{mu} 0 0"/>
    </body>
  </worldbody>
</mujoco>
"""


def build_turntable_xml(dt, mu):
    thickness = 0.1
    rider_half = 0.5 * TURNTABLE_RIDER_SIZE
    rider_z = rider_half - TURNTABLE_RIDER_INITIAL_PENETRATION
    # The turntable is a real hinge-jointed body whose qvel we overwrite every
    # step (see run_small_fixture), not a MuJoCo "mocap" body: mocap bodies
    # have no DOFs, so the contact solver sees zero relative velocity at their
    # surface no matter how mocap_pos/mocap_quat are moved between steps, and
    # never drags a resting body via friction. A real, velocity-overridden
    # hinge joint mirrors what fbf_paper_trace.cpp actually does -- it calls
    # FreeJoint::setAngularVelocity(...) on an immobile-but-jointed skeleton
    # every step, which is a kinematic override backed by a genuine DOF.
    return f"""
<mujoco>
  <option timestep="{dt}" gravity="0 0 -9.81"
          solver="{PAPER_MUJOCO_SOLVER}" cone="{PAPER_MUJOCO_CONE}"
          iterations="{PAPER_MUJOCO_MAX_ITERATIONS}"/>
  <worldbody>
    <body name="turntable_body" pos="0 0 {-0.5 * thickness}">
      <joint name="turntable_hinge" type="hinge" axis="0 0 1"/>
      <geom type="box" size="2 2 {0.5 * thickness}" mass="1"
            friction="{mu} 0 0"/>
    </body>
    <body name="turntable_rider_body"
          pos="{TURNTABLE_INITIAL_RADIUS} 0 {rider_z}">
      <freejoint/>
      <geom type="box" size="{rider_half} {rider_half} {rider_half}" mass="1"
            friction="{mu} 0 0"/>
    </body>
  </worldbody>
</mujoco>
"""


def turntable_omega(scenario):
    return 2.0 if scenario.endswith("omega_2") else 5.0


def turntable_mu(scenario):
    return 0.2 if "mu_0_2" in scenario else 0.5


def small_fixture_setup(scenario, dt):
    """Return reconstructed XML and tracked-body information for a scenario."""
    if scenario == "backspin":
        xml = build_backspin_xml(dt)
        body_name = "backspin_sphere_body"
    elif scenario in ("incline_mu_0_5", "incline_mu_0_4"):
        mu = 0.5 if scenario == "incline_mu_0_5" else 0.4
        xml = build_incline_xml(dt, mu)
        body_name = "incline_cube_body"
    elif scenario.startswith("turntable_"):
        xml = build_turntable_xml(dt, turntable_mu(scenario))
        body_name = "turntable_rider_body"
    else:
        raise ValueError(f"unknown small-fixture scenario: {scenario}")
    return xml, body_name, SCENARIO_DEFAULT_DURATIONS_S[scenario]


def _warning_snapshot(mujoco, data):
    warnings = []
    for warning_id in range(int(mujoco.mjtWarning.mjNWARNING)):
        warning = data.warning[warning_id]
        if warning.number:
            warnings.append(
                {
                    "warning_id": warning_id,
                    "number": int(warning.number),
                    "last_info": int(warning.lastinfo),
                }
            )
    return warnings


def _contact_min_distance(data):
    if data.ncon == 0:
        return float("nan")
    return min(float(data.contact[i].dist) for i in range(data.ncon))


def simulate_small_fixture(
    mujoco,
    scenario,
    dt=PAPER_DT,
    duration=None,
    repetition=0,
    run_id=None,
):
    """Run one complete reconstructed small-fixture trajectory.

    Every completed call to ``mujoco.mj_step`` is timed with the monotonic
    high-resolution clock. Model compilation, initial ``mj_forward``, row
    extraction, and the turntable's per-step velocity override are outside
    the timed interval. The call is synchronous on the CPU, so the elapsed
    interval is the completed step rather than an enqueue time.
    """

    xml, body_name, default_duration = small_fixture_setup(scenario, dt)

    if duration is None:
        duration = default_duration
    steps = int(round(duration / dt))
    if steps <= 0:
        raise ValueError("duration must contain at least one simulation step")
    duration = steps * dt

    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    if scenario == "backspin":
        backspin_initial_velocity(data)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    body_dof_adr = int(model.body_dofadr[body_id])
    if body_dof_adr < 0 or int(model.body_dofnum[body_id]) != 6:
        raise RuntimeError(f"{body_name} is not backed by one free joint")

    turntable_dof = None
    turntable_qpos = None
    omega = 0.0
    if scenario.startswith("turntable_"):
        turntable_joint_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, "turntable_hinge"
        )
        turntable_dof = int(model.jnt_dofadr[turntable_joint_id])
        turntable_qpos = int(model.jnt_qposadr[turntable_joint_id])
        omega = turntable_omega(scenario)

    mujoco.mj_forward(model, data)
    initial_pos = [float(v) for v in data.xpos[body_id]]
    theta = math.atan(INCLINE_TAN)
    downhill = (-math.cos(theta), 0.0, -math.sin(theta))
    rows = []

    def capture(step, wall_ns):
        pos = [float(v) for v in data.xpos[body_id]]
        qvel = [float(v) for v in data.qvel[body_dof_adr : body_dof_adr + 6]]
        quat = [float(v) for v in data.xquat[body_id]]
        displacement = [pos[i] - initial_pos[i] for i in range(3)]
        tangential_displacement = float("nan")
        if scenario.startswith("incline_"):
            tangential_displacement = sum(
                displacement[i] * downhill[i] for i in range(3)
            )
        radial_distance = float("nan")
        azimuth = float("nan")
        turntable_angle = float("nan")
        relative_phase = float("nan")
        commanded_omega = float("nan")
        if scenario.startswith("turntable_"):
            radial_distance = math.hypot(pos[0], pos[1])
            azimuth = math.atan2(pos[1], pos[0])
            turntable_angle = float(data.qpos[turntable_qpos])
            relative_phase = math.atan2(
                math.sin(azimuth - turntable_angle),
                math.cos(azimuth - turntable_angle),
            )
            control_time = max((step - 1) * dt, 0.0)
            commanded_omega = min(control_time / TURNTABLE_RAMP_DURATION, 1.0) * omega

        solver_niter = [int(v) for v in data.solver_niter]
        solver_nnz = [int(v) for v in data.solver_nnz]
        rows.append(
            {
                "run_id": run_id or f"{scenario}-r{repetition + 1:03d}",
                "repetition": repetition + 1,
                "step": step,
                "sim_time_s": float(data.time),
                "scenario": scenario,
                "solver": "mujoco_newton_elliptic",
                "timed_step": int(step > 0),
                "step_wall_ns": int(wall_ns),
                "step_wall_ms": wall_ns / 1.0e6,
                "body": body_name,
                "x_m": pos[0],
                "y_m": pos[1],
                "z_m": pos[2],
                "vx_mps": qvel[0],
                "vy_mps": qvel[1],
                "vz_mps": qvel[2],
                "wx_rad_s": qvel[3],
                "wy_rad_s": qvel[4],
                "wz_rad_s": qvel[5],
                "quat_w": quat[0],
                "quat_x": quat[1],
                "quat_y": quat[2],
                "quat_z": quat[3],
                "up_z": float(data.xmat[body_id][8]),
                "speed_mps": math.sqrt(sum(v * v for v in qvel[:3])),
                "angular_speed_rad_s": math.sqrt(sum(v * v for v in qvel[3:])),
                "dx_m": displacement[0],
                "dy_m": displacement[1],
                "dz_m": displacement[2],
                "down_slope_displacement_m": tangential_displacement,
                "radial_distance_m": radial_distance,
                "azimuth_rad": azimuth,
                "turntable_angle_rad": turntable_angle,
                "turntable_relative_phase_rad": relative_phase,
                "turntable_command_omega_rad_s": commanded_omega,
                "contacts": int(data.ncon),
                "contact_constraints": int(data.nefc),
                "min_contact_distance_m": _contact_min_distance(data),
                "airborne": int(data.ncon == 0),
                "solver_iterations_sum": sum(solver_niter),
                "solver_iterations_max_island": max(solver_niter, default=0),
                "solver_nnz_sum": sum(solver_nnz),
            }
        )

    capture(0, 0)

    for step in range(steps):
        if turntable_dof is not None:
            t = step * dt
            ramp = min(t / TURNTABLE_RAMP_DURATION, 1.0)
            data.qvel[turntable_dof] = ramp * omega

        wall_start = time.perf_counter_ns()
        mujoco.mj_step(model, data)
        wall_end = time.perf_counter_ns()
        completed = step + 1
        capture(completed, wall_end - wall_start)

    model_options = {
        "timestep_s": float(model.opt.timestep),
        "solver_enum": int(model.opt.solver),
        "cone_enum": int(model.opt.cone),
        "iterations": int(model.opt.iterations),
        "tolerance": float(model.opt.tolerance),
        "ls_iterations": int(model.opt.ls_iterations),
        "ls_tolerance": float(model.opt.ls_tolerance),
        "noslip_iterations": int(model.opt.noslip_iterations),
        "integrator_enum": int(model.opt.integrator),
        "jacobian_enum": int(model.opt.jacobian),
        "impratio": float(model.opt.impratio),
        "disableflags": int(model.opt.disableflags),
        "enableflags": int(model.opt.enableflags),
        "numeric_dtype": str(data.qpos.dtype),
        "nq": int(model.nq),
        "nv": int(model.nv),
        "nbody": int(model.nbody),
        "ngeom": int(model.ngeom),
        "geom_friction": [[float(v) for v in row] for row in model.geom_friction],
        "geom_solref": [[float(v) for v in row] for row in model.geom_solref],
        "geom_solimp": [[float(v) for v in row] for row in model.geom_solimp],
    }

    return {
        "scenario": scenario,
        "body_name": body_name,
        "dt_s": dt,
        "duration_s": duration,
        "steps": steps,
        "xml": xml,
        "rows": rows,
        "model_options": model_options,
        "warnings": _warning_snapshot(mujoco, data),
    }


# --------------------------------------------------------------------------
# Small-fixture stdout runner (backspin / incline / turntable): one tracked
# body, with the legacy sparse CSV schema used by fbf_paper_trace.cpp.
# --------------------------------------------------------------------------


def run_small_fixture(mujoco, scenario, dt, duration, sample_stride):
    result = simulate_small_fixture(mujoco, scenario, dt, duration)
    for row in result["rows"]:
        if (
            row["step"] == 0
            or row["step"] % sample_stride == 0
            or row["step"] == result["steps"]
        ):
            print_row(
                row["step"],
                row["sim_time_s"],
                scenario,
                result["body_name"],
                (row["x_m"], row["y_m"], row["z_m"]),
                (row["vx_mps"], row["vy_mps"], row["vz_mps"]),
                row["up_z"],
                row["contacts"],
            )

    if scenario == "backspin":
        final = result["rows"][-1]
        vx_final = final["vx_mps"]
        wy_final = final["wy_rad_s"]
        dv_pct = (
            abs(vx_final - BACKSPIN_ANALYTIC_V_INF)
            / abs(BACKSPIN_ANALYTIC_V_INF)
            * 100.0
        )
        domega_pct = (
            abs(wy_final - BACKSPIN_ANALYTIC_OMEGA_INF)
            / abs(BACKSPIN_ANALYTIC_OMEGA_INF)
            * 100.0
        )
        log(
            f"# backspin (dt={dt:.6g}): MuJoCo final vx={vx_final:.6f} "
            f"wy={wy_final:.6f}; analytic v_inf={BACKSPIN_ANALYTIC_V_INF:.6f} "
            f"omega_inf={BACKSPIN_ANALYTIC_OMEGA_INF:.6f}; "
            f"rel diff dv={dv_pct:.3f}% domega={domega_pct:.3f}%"
        )


# --------------------------------------------------------------------------
# Optional Rigid-IPC 101-stone masonry-arch runner.
# --------------------------------------------------------------------------


def run_rigid_ipc_arch(mujoco, duration, sample_stride):
    if not os.path.isfile(ARCH_XML_PATH):
        log(
            "# masonry_arch_101_rigid_ipc: adapted Rigid-IPC MuJoCo asset not "
            f"found at {ARCH_XML_PATH}; skipping (see "
            "data/mjcf/rigid_ipc_arch/README.md for provenance/licensing; "
            "this scenario is opt-in and benchmark/example-only)."
        )
        return

    import numpy as np

    model = mujoco.MjModel.from_xml_path(ARCH_XML_PATH)
    # MuJoCo silently resets mjData back toward its initial state whenever a
    # step produces a non-finite QACC ("auto-reset"). Left enabled, a genuine
    # collapse that goes numerically unstable would keep getting reset back
    # near the initial pose, which would misreport as "held". Disable it so
    # divergence surfaces as an explicit, detectable non-finite state that we
    # catch and report honestly below instead.
    model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_AUTORESET
    data = mujoco.MjData(model)
    dt = model.opt.timestep
    steps = int(round(duration / dt))

    num_stones = 101
    stone_ids = [
        mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, f"rigid_ipc_arch_stone_{i:03d}"
        )
        for i in range(1, num_stones + 1)
    ]
    crown_id = stone_ids[num_stones // 2]
    crown_name = f"rigid_ipc_arch_stone_{num_stones // 2 + 1:03d}"

    mujoco.mj_forward(model, data)
    initial_crown_z = data.xpos[crown_id][2]
    initial_min_z = min(data.xpos[i][2] for i in stone_ids)
    initial_contacts = data.ncon
    if initial_contacts == 0:
        log(
            "# masonry_arch_101_rigid_ipc: MuJoCo's collision pipeline "
            "detects ZERO contacts anywhere in the 101-stone assembly at "
            "t=0 (default contact margin/gap). Rigid-IPC's own scene is "
            "built for its barrier-method contact formulation, which "
            "assumes exact face-to-face touching with no clearance; that "
            "falls just outside MuJoCo's discrete near-phase detection "
            "tolerance here, so every stone (including the two springer "
            "stones against the ground) starts this run with no contact "
            "support at all."
        )

    crown_dof_adr = model.body_dofadr[crown_id]

    def sample(step, time_s):
        pos = data.xpos[crown_id]
        vel = data.qvel[crown_dof_adr : crown_dof_adr + 3]
        up_z = data.xmat[crown_id][8]
        print_row(step, time_s, ARCH_SCENARIO, crown_name, pos, vel, up_z, data.ncon)

    sample(0, data.time)
    last_good_time = data.time
    last_good_crown_z = initial_crown_z
    last_good_min_z = initial_min_z
    diverged_at = None
    for step in range(steps):
        mujoco.mj_step(model, data)
        completed = step + 1
        # Beyond outright NaN/Inf, a blown-up-but-still-finite state (e.g. a
        # free-joint quaternion losing unit norm) is just as untrustworthy:
        # it shows up as an "up_z" outside [-1, 1], which is impossible for a
        # genuine rotation-matrix column. Treat huge finite qvel the same as
        # non-finite qacc/qpos/qvel -- both mean "stop trusting this state".
        finite_ok = (
            np.isfinite(data.qacc).all()
            and np.isfinite(data.qpos).all()
            and np.isfinite(data.qvel).all()
        )
        bounded_ok = finite_ok and float(np.abs(data.qvel).max()) < 1.0e3
        if not bounded_ok:
            diverged_at = completed * dt
            log(
                f"# masonry_arch_101_rigid_ipc: MuJoCo simulation diverged "
                f"({'non-finite' if not finite_ok else 'unbounded'} "
                f"QACC/QPOS/QVEL) at step {completed} (t={diverged_at:.4f} s); "
                "stopping early and reporting the last known-good state "
                "rather than trusting MuJoCo's default auto-reset-on-"
                "divergence recovery or a corrupted-but-finite state."
            )
            break
        last_good_time = data.time
        last_good_crown_z = data.xpos[crown_id][2]
        last_good_min_z = min(data.xpos[i][2] for i in stone_ids)
        if completed % sample_stride == 0 or completed == steps:
            sample(completed, data.time)

    crown_drop = initial_crown_z - last_good_crown_z
    min_drop = initial_min_z - last_good_min_z
    log(
        f"# masonry_arch_101_rigid_ipc: over {last_good_time:.3f} s of "
        f"{duration:.3f} s requested (dt={dt:.4g}), crown-stone origin "
        f"({crown_name}) body-frame z-displacement = {-crown_drop:+.4f} m "
        f"(drop={crown_drop:.4f} m); worst per-stone origin drop across all "
        f"{num_stones} stones = {min_drop:.4f} m. Body-frame displacement is "
        "relative to each stone's own zero pose (Rigid-IPC bakes absolute "
        "position into mesh vertices, not body pos/qpos), so this measures "
        "motion since t=0, not absolute height."
    )
    if diverged_at is not None:
        no_initial_contact_note = (
            " (root cause: MuJoCo detected zero initial contacts, so the "
            "assembly began this run in unsupported free-fall -- see the "
            "t=0 diagnostic line above)"
            if initial_contacts == 0
            else ""
        )
        log(
            "# masonry_arch_101_rigid_ipc: qualitative outcome = DIVERGED "
            f"(MuJoCo's contact solver went numerically unstable at "
            f"t={diverged_at:.4f} s under the default solver/timestep "
            f"settings in this adapted scene{no_initial_contact_note}; this "
            "itself corroborates the paper's qualitative MuJoCo-vs-"
            "FBF/Kamino masonry-arch table entry, but is not a controlled "
            "'it slumped by X' measurement)."
        )
    elif crown_drop > 0.05:
        log(
            "# masonry_arch_101_rigid_ipc: qualitative outcome = SLUMPED "
            "(crown stone moved down more than 5 cm), corroborating the "
            "paper's own reported qualitative MuJoCo-vs-FBF/Kamino table "
            "entry for the masonry-arch scenes."
        )
    else:
        log(
            "# masonry_arch_101_rigid_ipc: qualitative outcome = HELD "
            "(crown stone stayed within 5 cm of its initial pose)."
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "MuJoCo external-baseline harness for the FBF paper's small "
            "fixture scenes (benchmark/example-only; see module docstring)."
        )
    )
    parser.add_argument(
        "scenarios",
        nargs="*",
        help=(
            "Scenario name(s) to run, or 'all' for every scenario including "
            f"the opt-in {ARCH_SCENARIO}. Default: every scenario except "
            f"{ARCH_SCENARIO}. Choices: {', '.join(ALL_SCENARIOS)}, all"
        ),
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=PAPER_DT,
        help="Timestep for the small-fixture scenarios (default: paper 1/60).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Override duration (s) for all selected small-fixture scenarios.",
    )
    parser.add_argument(
        "--sample-stride",
        type=int,
        default=30,
        help="Print every Nth completed step for small fixtures (default 30).",
    )
    parser.add_argument(
        "--arch-duration",
        type=float,
        default=1.0,
        help="Sim duration (s) for masonry_arch_101_rigid_ipc (default 1.0).",
    )
    parser.add_argument(
        "--arch-sample-stride",
        type=int,
        default=20,
        help="Print every Nth completed step for the arch scenario (default 20).",
    )
    args = parser.parse_args(argv)

    if not math.isfinite(args.dt) or args.dt <= 0.0:
        parser.error("--dt must be finite and positive")
    if args.duration is not None and (
        not math.isfinite(args.duration) or args.duration <= 0.0
    ):
        parser.error("--duration must be finite and positive")
    if args.sample_stride <= 0:
        parser.error("--sample-stride must be positive")
    if not math.isfinite(args.arch_duration) or args.arch_duration <= 0.0:
        parser.error("--arch-duration must be finite and positive")
    if args.arch_sample_stride <= 0:
        parser.error("--arch-sample-stride must be positive")

    if not args.scenarios:
        args.scenarios = list(SMALL_FIXTURE_SCENARIOS)
    elif args.scenarios == ["all"]:
        args.scenarios = list(ALL_SCENARIOS)
    else:
        for name in args.scenarios:
            if name not in ALL_SCENARIOS:
                parser.error(
                    f"unknown scenario '{name}'; choices: "
                    f"{', '.join(ALL_SCENARIOS)}, all"
                )
    return args


def main(argv):
    args = parse_args(argv)

    try:
        import mujoco  # noqa: PLC0415 (intentionally lazy/optional)
    except ImportError:
        log(
            "fbf_paper_mujoco_baseline: the optional 'mujoco' pypi package is "
            "not installed in this environment; skipping the MuJoCo "
            "external-baseline comparison. This is not a DART library or "
            "default pixi-environment dependency -- install it with the "
            "optional 'mujoco-baseline' pixi feature/environment "
            "(`pixi run -e mujoco-baseline fbf-mujoco-baseline`) to enable "
            "this comparison."
        )
        return 0

    print_header()
    for scenario in args.scenarios:
        if scenario == ARCH_SCENARIO:
            run_rigid_ipc_arch(mujoco, args.arch_duration, args.arch_sample_stride)
        else:
            run_small_fixture(
                mujoco, scenario, args.dt, args.duration, args.sample_stride
            )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
