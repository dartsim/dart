"""Robot-agnostic SIMBICON biped locomotion controller (Python).

SIMBICON (Yin, Loken, van de Panne, SIGGRAPH 2007 -
https://www.cs.ubc.ca/~van/papers/simbicon.htm) is a simple, robot-independent
balance-and-locomotion control *law*:

* a symmetric finite state machine (FSM) with target poses per state,
* world-frame tracking of the torso and swing-hip orientation, with the stance
  hip applying the reaction torque (``tau_stance = -tau_torso - tau_swing``),
* a balance-feedback law ``theta_d = theta_d0 + c_d*d + c_v*v`` applied to the
  swing hip in the sagittal and coronal planes, where ``d`` and ``v`` are the
  horizontal position/velocity of the center of mass relative to the stance
  ankle.

Only the body/DOF *names* and the gait *parameters* (target angles, gains,
durations) are robot-specific; those live in :class:`SimbiconConfig`, so the
same controller drives any humanoid given a per-robot config. Concrete configs
are provided by :func:`make_atlas_config` and :func:`make_g1_config`.

Notes on the Python port (vs the C++ ``examples/demos/scenes/atlas_simbicon``):

* dartpy does not expose ``Skeleton::getCOM``; following the paper (3.3) we use
  the **midpoint of the hips** as the COM proxy (position and velocity), which
  the authors note "can be used as a simple and effective proxy".
* dartpy's ``CollisionResult`` has no per-body query, so swing-foot ground
  contact is detected with a **foot-height proxy** (sole near the ground while
  descending), falling back to a maximum swing duration.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class SimbiconConfig:
    """Per-robot role names and gait parameters for :class:`SimbiconController`."""

    name: str = "robot"

    # --- Body-node roles (skeleton body names) ---
    pelvis: str = "pelvis"
    left_foot: str = ""
    right_foot: str = ""
    left_thigh: str = ""
    right_thigh: str = ""

    # --- DOF roles (skeleton DOF names) ---
    torso: str = ""  # pelvis<->torso pitch DOF (empty -> skip torso target)
    left_hip_sagittal: str = ""
    right_hip_sagittal: str = ""
    left_hip_coronal: str = ""
    right_hip_coronal: str = ""
    left_knee: str = ""
    right_knee: str = ""
    left_ankle_sagittal: str = ""
    right_ankle_sagittal: str = ""

    # --- PD + balance gains ---
    kp: float = 600.0
    kd: float = 60.0
    cd: float = 0.5
    cv: float = 0.2
    # World-frame torso/swing-hip virtual PD gains.
    torso_kp: float = 1000.0
    torso_kd: float = 100.0
    # Stance-leg height-regulation gain (0 disables). Extends the stance knee
    # when the pelvis sinks below its standing height, counteracting the gradual
    # crouch-collapse the bare gait targets exhibit (the dominant fall mode).
    height_kp: float = 0.0

    # --- Emergency-stop pelvis-height window (world Z, metres). ---
    min_pelvis_height: float = -1.0
    max_pelvis_height: float = 2.0

    # --- Foot-contact proxy ---
    foot_contact_height: float = 0.05  # sole-height threshold above ground
    ground_height: float = 0.0

    # --- Gait: swing-up states (0,2) time-gated; swing-down (1,3) on contact ---
    swing_up_duration: float = 0.3
    swing_down_max_duration: float = 0.6

    # Target pelvis<->torso pitch held throughout the gait (world frame).
    walk_torso: float = 0.0
    # Desired global pelvis lean (sagittal/coronal) the torso control tracks.
    desired_pelvis_sagittal: float = 0.0
    desired_pelvis_coronal: float = 0.0

    # Swing-up phase (states 0,2) targets (radians, joint-local).
    swing_hip_up: float = -0.50
    swing_knee_up: float = 1.10
    swing_ankle_up: float = -0.60
    stance_knee_up: float = 0.05
    stance_ankle_up: float = 0.0

    # Swing-down phase (states 1,3) targets.
    swing_hip_down: float = 0.10
    swing_knee_down: float = 0.05
    swing_ankle_down: float = -0.15
    stance_knee_down: float = 0.10
    stance_ankle_down: float = 0.0

    # Per-joint sign multipliers (+1/-1) absorbing each robot's joint-axis
    # conventions so the same target magnitudes produce the same physical motion.
    hip_sign: float = 1.0
    knee_sign: float = 1.0
    ankle_sign: float = 1.0


_AXIS_EPS = 1e-9


def _normalized_or(vec: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(vec))
    if n <= _AXIS_EPS:
        return fallback
    return vec / n


class SimbiconController:
    """A SIMBICON FSM controller for one humanoid skeleton.

    Build it with a ``dartpy`` world + skeleton + :class:`SimbiconConfig`, then
    call :meth:`pre_step` once per simulation step (wire it as a scene's
    ``pre_step``).
    """

    def __init__(self, world, skeleton, config: SimbiconConfig) -> None:
        self.world = world
        self.skel = skeleton
        self.cfg = config

        self._pelvis = skeleton.get_body_node(config.pelvis)
        self._left_foot = skeleton.get_body_node(config.left_foot)
        self._right_foot = skeleton.get_body_node(config.right_foot)
        self._left_thigh = skeleton.get_body_node(config.left_thigh)
        self._right_thigh = skeleton.get_body_node(config.right_thigh)

        self._ndofs = skeleton.get_num_dofs()
        # dartpy's get_dof only takes an int index, so resolve names ourselves.
        name_to_index = {
            skeleton.get_dof(i).get_name(): i for i in range(self._ndofs)
        }
        self._dof = {}
        for role in (
            "torso",
            "left_hip_sagittal",
            "right_hip_sagittal",
            "left_hip_coronal",
            "right_hip_coronal",
            "left_knee",
            "right_knee",
            "left_ankle_sagittal",
            "right_ankle_sagittal",
        ):
            name = getattr(config, role)
            if name and name not in name_to_index:
                raise RuntimeError(
                    f"SIMBICON config role '{role}' references unknown DOF "
                    f"'{name}' on skeleton '{skeleton.get_name()}'"
                )
            self._dof[role] = name_to_index[name] if name else -1
        # FSM: 0,1 = left stance; 2,3 = right stance. 0,2 swing-up; 1,3 down.
        self._state = 0
        self._state_time = 0.0
        # Standing pelvis height captured at the spawn pose; the height-
        # regulation term drives the pelvis back toward it.
        self._target_pz = self._pelvis_world_z()

    # ----------------------------------------------------------------- COM ---
    def _pelvis_world_z(self) -> float:
        return float(np.asarray(self._pelvis.get_world_transform().matrix())[2, 3])

    def _hip_world(self, thigh):
        return np.asarray(thigh.get_world_transform().matrix())[:3, 3]

    def _com_proxy(self):
        """Hip-midpoint proxy for COM position and velocity (paper 3.3)."""
        pos = 0.5 * (
            self._hip_world(self._left_thigh) + self._hip_world(self._right_thigh)
        )
        vel = 0.5 * (
            np.asarray(self._left_thigh.get_linear_velocity())
            + np.asarray(self._right_thigh.get_linear_velocity())
        )
        return pos, vel

    def _com_frame(self):
        """Z-up frame at the COM: x=forward (pelvis x on ground), y=lateral."""
        z_axis = np.array([0.0, 0.0, 1.0])
        pelvis_x = np.asarray(self._pelvis.get_world_transform().matrix())[:3, 0]
        pelvis_x = pelvis_x - z_axis * float(z_axis @ pelvis_x)
        x_axis = _normalized_or(pelvis_x, np.array([1.0, 0.0, 0.0]))
        y_axis = _normalized_or(np.cross(z_axis, x_axis), np.array([0.0, 1.0, 0.0]))
        return np.column_stack((x_axis, y_axis, z_axis))

    def _stance_ankle_pos(self):
        foot = self._left_foot if self._state < 2 else self._right_foot
        return np.asarray(foot.get_world_transform().matrix())[:3, 3]

    def _balance_d_v(self):
        """Sagittal/coronal COM distance & velocity relative to stance ankle."""
        com_pos, com_vel = self._com_proxy()
        frame = self._com_frame()
        d_vec = com_pos - self._stance_ankle_pos()
        d_sag = float(d_vec @ frame[:, 0])
        d_cor = float(d_vec @ frame[:, 1])
        v_sag = float(com_vel @ frame[:, 0])
        v_cor = float(com_vel @ frame[:, 1])
        return d_sag, v_sag, d_cor, v_cor

    def _pelvis_angle(self, sagittal: bool) -> float:
        """Signed pelvis lean from vertical in the sagittal or coronal plane."""
        frame = self._com_frame()
        com_z = frame[:, 2]
        pelvis_z = np.asarray(self._pelvis.get_world_transform().matrix())[:3, 2]
        proj = frame.T @ pelvis_z
        proj[1 if sagittal else 0] = 0.0
        proj = _normalized_or(proj, np.array([0.0, 0.0, 1.0]))
        denom = float(np.linalg.norm(proj) * np.linalg.norm(com_z))
        angle = (
            math.acos(max(-1.0, min(1.0, float(proj @ com_z) / denom)))
            if denom > _AXIS_EPS
            else 0.0
        )
        cross = np.cross(com_z, proj)
        sign_component = cross[1 if sagittal else 0]
        return angle if sign_component > 0.0 else -angle

    def _pelvis_rates(self, frame: np.ndarray):
        """Sagittal/coronal pelvis-lean rates (d/dt of :meth:`_pelvis_angle`).

        ``getSpatialVelocity()`` returns the body-frame twist; rotating its
        angular part into the world gives the pelvis angular velocity. Its
        component about the lateral axis is the forward-tilt (sagittal) rate and
        the component about the forward axis is the lateral-tilt (coronal) rate
        -- the derivative terms the torso PD needs to damp the upright pose.
        """
        twist = np.asarray(self._pelvis.get_spatial_velocity())
        rot = np.asarray(self._pelvis.get_world_transform().matrix())[:3, :3]
        omega = rot @ twist[:3]
        rate_sag = float(omega @ frame[:, 1])
        rate_cor = float(omega @ frame[:, 0])
        return rate_sag, rate_cor

    # ------------------------------------------------------------- gait ------
    def _swing_is_right(self) -> bool:
        # States 0,1 = left stance -> right swing. States 2,3 -> left swing.
        return self._state < 2

    def _foot_in_contact(self, right: bool) -> bool:
        foot = self._right_foot if right else self._left_foot
        z = float(np.asarray(foot.get_world_transform().matrix())[2, 3])
        return z <= self.cfg.ground_height + self.cfg.foot_contact_height

    def _maybe_advance_state(self, dt: float) -> None:
        self._state_time += dt
        swing_up = self._state in (0, 2)
        if swing_up:
            if self._state_time >= self.cfg.swing_up_duration:
                self._state = (self._state + 1) % 4
                self._state_time = 0.0
        else:
            swing_right = self._swing_is_right()
            contact = self._foot_in_contact(swing_right)
            if (
                contact and self._state_time > 0.05
            ) or self._state_time >= self.cfg.swing_down_max_duration:
                self._state = (self._state + 1) % 4
                self._state_time = 0.0

    def _desired_positions(self) -> np.ndarray:
        """FSM target joint positions for the current state."""
        c = self.cfg
        q = np.zeros(self._ndofs)
        up = self._state in (0, 2)
        swing_right = self._swing_is_right()

        def setp(role: str, value: float) -> None:
            idx = self._dof[role]
            if idx >= 0:
                q[idx] = value

        if up:
            sh, sk, sa = c.swing_hip_up, c.swing_knee_up, c.swing_ankle_up
            tk, ta = c.stance_knee_up, c.stance_ankle_up
        else:
            sh, sk, sa = c.swing_hip_down, c.swing_knee_down, c.swing_ankle_down
            tk, ta = c.stance_knee_down, c.stance_ankle_down

        swing = "right" if swing_right else "left"
        stance = "left" if swing_right else "right"
        setp(f"{swing}_hip_sagittal", c.hip_sign * sh)
        setp(f"{swing}_knee", c.knee_sign * sk)
        setp(f"{swing}_ankle_sagittal", c.ankle_sign * sa)
        setp(f"{stance}_knee", c.knee_sign * tk)
        setp(f"{stance}_ankle_sagittal", c.ankle_sign * ta)
        setp("torso", c.walk_torso)
        return q

    # ------------------------------------------------------------ control ----
    def _allowing_control(self) -> bool:
        z = self._pelvis_world_z()
        return self.cfg.min_pelvis_height <= z <= self.cfg.max_pelvis_height

    def diagnostics(self) -> dict[str, float | int | bool | str]:
        """Return live controller state for demo UI panels."""
        d_sag, v_sag, d_cor, v_cor = self._balance_d_v()
        return {
            "name": self.cfg.name,
            "state": self._state,
            "state_time": self._state_time,
            "swing": "right" if self._swing_is_right() else "left",
            "pelvis_height": self._pelvis_world_z(),
            "balance_sagittal": d_sag,
            "balance_sagittal_velocity": v_sag,
            "balance_coronal": d_cor,
            "balance_coronal_velocity": v_cor,
            "control_enabled": self._allowing_control(),
        }

    def pre_step(self) -> None:
        dt = self.world.get_time_step()
        self._maybe_advance_state(dt)
        if not self._allowing_control():
            return

        c = self.cfg
        q = np.asarray(self.skel.get_positions())
        dq = np.asarray(self.skel.get_velocities())
        desired = self._desired_positions()

        # Balance feedback on the swing hip (sagittal + coronal). Eq. (1).
        d_sag, v_sag, d_cor, v_cor = self._balance_d_v()
        swing_right = self._swing_is_right()
        sag_role = f"{'right' if swing_right else 'left'}_hip_sagittal"
        cor_role = f"{'right' if swing_right else 'left'}_hip_coronal"
        sag_idx = self._dof[sag_role]
        cor_idx = self._dof[cor_role]
        if sag_idx >= 0:
            desired[sag_idx] += -(c.cd * d_sag + c.cv * v_sag)
        if cor_idx >= 0:
            desired[cor_idx] += -(c.cd * d_cor + c.cv * v_cor)

        # Stance-leg height regulation. The bare gait targets let the body
        # gradually crouch-collapse (traces show the pelvis sinking cycle over
        # cycle until it drops out of the control window -- the dominant fall
        # mode for both Atlas and G1). Extend the stance knee toward its
        # standing height when the pelvis sinks. Flexion is +knee_sign (the
        # swing knee bends with knee_sign*swing_knee_up), so extension is the
        # -knee_sign direction; the clip caps the per-step correction.
        if c.height_kp != 0.0:
            err_h = self._target_pz - self._pelvis_world_z()  # >0 when too low
            stance_knee = self._dof[f"{'left' if swing_right else 'right'}_knee"]
            if stance_knee >= 0:
                desired[stance_knee] += (
                    -c.knee_sign * c.height_kp * float(np.clip(err_h, -0.25, 0.25))
                )

        # Stable-PD (SPD, Tan et al. 2011) toward the feedback-adjusted targets.
        # SPD stays stable for stiff gains on low-inertia DOFs (e.g. G1's wrists)
        # where explicit PD explodes. Root (first 6 free-joint DOFs) is left
        # unactuated. (We omit the constraint-force term, which dartpy doesn't
        # expose; it is a minor refinement.)
        n = self._ndofs
        kp_vec = np.zeros(n)
        kd_vec = np.zeros(n)
        kp_vec[6:] = c.kp
        kd_vec[6:] = c.kd
        mass = np.asarray(self.skel.get_mass_matrix())
        cg = np.asarray(self.skel.get_coriolis_and_gravity_forces())
        prop = -kp_vec * (q + dq * dt - desired)
        deriv = -kd_vec * dq
        qddot = np.linalg.solve(
            mass + np.diag(kd_vec * dt), prop + deriv - cg
        )
        tau = prop + deriv - kd_vec * dt * qddot

        # World-frame torso control via the stance hip (SIMBICON 3.2). The
        # positive sagittal sign keeps the pelvis upright under the Z-up
        # convention (see the C++ atlas_simbicon backward-topple fix).
        pelvis_sag = self._pelvis_angle(sagittal=True)
        pelvis_cor = self._pelvis_angle(sagittal=False)
        rate_sag, rate_cor = self._pelvis_rates(self._com_frame())
        # PD on the world-frame pelvis lean. The paper's torso law is a PD, so
        # wire in the damping (torso_kd) term that opposes the lean rate; it
        # mirrors each proportional term's sign because rate_sag/rate_cor are
        # the exact time-derivatives of pelvis_sag/pelvis_cor. (It steadies the
        # torso but is not the dominant balance factor -- the gradual height
        # sink that height_kp addresses is; keep torso_kd modest.)
        tau_torso_sag = (
            self.cfg.torso_kp * (pelvis_sag + c.desired_pelvis_sagittal)
            + self.cfg.torso_kd * rate_sag
        )
        tau_torso_cor = (
            -self.cfg.torso_kp * (pelvis_cor - c.desired_pelvis_coronal)
            - self.cfg.torso_kd * rate_cor
        )
        stance_sag = self._dof[
            f"{'left' if swing_right else 'right'}_hip_sagittal"
        ]
        stance_cor = self._dof[
            f"{'left' if swing_right else 'right'}_hip_coronal"
        ]
        if stance_sag >= 0 and sag_idx >= 0:
            tau[stance_sag] = tau_torso_sag - tau[sag_idx]
        if stance_cor >= 0 and cor_idx >= 0:
            tau[stance_cor] = -tau_torso_cor - tau[cor_idx]

        tau[:6] = 0.0
        self.skel.set_forces(tau.tolist())
