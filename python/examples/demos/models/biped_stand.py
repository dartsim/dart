"""Standing-pose SPD tracking with the demo's sagittal ankle strategy."""

import math

import dartpy as dart
import numpy as np


class BipedStandController:
    def __init__(self, skeleton, timestep):
        if skeleton is None:
            raise ValueError("skeleton must not be None")
        if not isinstance(skeleton, dart.dynamics.Skeleton):
            raise TypeError("skeleton must be a Skeleton")
        self.timestep = timestep
        self._skeleton = skeleton
        self._desired_dofs = skeleton.getPositions().copy()
        self._kp = np.full(skeleton.getNumDofs(), 400.0)
        self._kd = np.full(skeleton.getNumDofs(), 40.0)
        self._kp[:6] = 0
        self._kd[:6] = 0
        self._left_heel = skeleton.getBodyNode("h_heel_left")
        self._ankle_dofs = []
        for name in (
            "j_heel_left_1",
            "j_toe_left",
            "j_heel_right_1",
            "j_toe_right",
        ):
            dof = skeleton.getDof(name)
            if dof is None:
                raise RuntimeError(f"fullbody1.skel: missing DOF {name}")
            self._ankle_dofs.append(dof.getIndexInSkeleton())
        self._previous_offset = 0.0

    @property
    def skeleton(self):
        return self._skeleton

    @property
    def timestep(self):
        return self._timestep

    @timestep.setter
    def timestep(self, value):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("timestep must be finite and positive")
        self._timestep = value

    def compute_torques(self):
        skeleton = self._skeleton
        if (
            skeleton.getNumDofs() != self._desired_dofs.size
            or skeleton.getBodyNode("h_heel_left") is not self._left_heel
        ):
            raise ValueError("controlled skeleton structure has changed")
        q = skeleton.getPositions()
        dq = skeleton.getVelocities()
        dt = self.timestep
        p = -self._kp * (q + dq * dt - self._desired_dofs)
        d = -self._kd * dq
        qddot = np.linalg.solve(
            skeleton.getMassMatrix() + np.diag(self._kd * dt),
            -skeleton.getCoriolisAndGravityForces()
            + p
            + d
            + skeleton.getConstraintForces(),
        )
        torques = p + d - self._kd * qddot * dt

        # World X survives the scene's Y-up to Z-up reorientation.
        if self._left_heel is not None:
            cop = self._left_heel.getTransform().multiply([0.05, 0, 0])
            offset = skeleton.getCOM()[0] - cop[0]
            if 0 < offset < 0.1:
                gains = (200.0, 100.0, 10.0)
            elif -0.2 < offset < -0.05:
                gains = (2000.0, 100.0, 100.0)
            else:
                gains = None
            if gains is not None:
                k1, k2, kd = gains
                delta = kd * (self._previous_offset - offset)
                for index, gain in zip(self._ankle_dofs, (k1, k2, k1, k2)):
                    torques[index] += -gain * offset + delta
                self._previous_offset = offset

        torques[:6] = 0
        return torques
