"""Full-body SPD pose tracking and the demo's sagittal ankle strategy."""

import math

import dartpy as dart
import numpy as np


class JointConstraintsController:
    def __init__(self, skeleton, timestep):
        if skeleton is None:
            raise ValueError("skeleton must have at least one DOF")
        if not isinstance(skeleton, dart.dynamics.Skeleton):
            raise TypeError("skeleton must be a Skeleton")
        if skeleton.getNumDofs() == 0:
            raise ValueError("skeleton must have at least one DOF")
        self.timestep = timestep
        self._desired_dofs = skeleton.getPositions().copy()
        self._kp = np.full(skeleton.getNumDofs(), 20.0)
        self._kd = np.full(skeleton.getNumDofs(), 10.0)
        self._kp[6:22] = 200
        self._kd[6:22] = 100
        self._kp[:6] = 0
        self._kd[:6] = 0
        self._previous_offset = 0.0

    @property
    def timestep(self):
        return self._timestep

    @timestep.setter
    def timestep(self, value):
        if not math.isfinite(value) or value <= 0:
            raise ValueError("timestep must be finite and positive")
        self._timestep = value

    def compute_torques(self, skeleton):
        if skeleton is None:
            raise ValueError("skeleton DOFs must match the initial pose")
        if not isinstance(skeleton, dart.dynamics.Skeleton):
            raise TypeError("skeleton must be a Skeleton")
        if skeleton.getNumDofs() != self._desired_dofs.size:
            raise ValueError("skeleton DOFs must match the initial pose")
        q = skeleton.getPositions()
        dq = skeleton.getVelocities()
        dt = self.timestep
        p = -self._kp * (q + dq * dt - self._desired_dofs)
        d = -self._kd * dq
        # The original demo intentionally leaves solver constraint forces out.
        qddot = np.linalg.solve(
            skeleton.getMassMatrix() + np.diag(self._kd * dt),
            -skeleton.getCoriolisAndGravityForces() + p + d,
        )
        torques = p + d - self._kd * qddot * dt

        heel = skeleton.getBodyNode("h_heel_left")
        if heel is not None and torques.size > 26:
            cop = heel.getTransform().multiply([0.05, 0, 0])
            offset = skeleton.getCOM()[0] - cop[0]
            if offset < 0.1:
                delta = 100 * (self._previous_offset - offset)
                torques[[17, 19]] += -20 * offset + delta
                torques[[25, 26]] += -10 * offset + delta
                self._previous_offset = offset

        torques[:6] = 0
        return torques
