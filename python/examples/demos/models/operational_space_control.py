"""Damped-pseudoinverse task-space control for the six-axis arm demo."""

import dartpy as dart
import numpy as np


def _finite_array(value, shape, name):
    array = np.array(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name} must be a finite array with shape {shape}")
    return array


class OperationalSpaceController:
    def __init__(self, robot, target):
        if not isinstance(robot, dart.dynamics.Skeleton):
            raise TypeError("robot must be a Skeleton")
        if robot.getNumBodyNodes() == 0 or robot.getNumDofs() == 0:
            raise ValueError("robot must have bodies and degrees of freedom")
        self._robot = robot
        self._end_effector = robot.getBodyNode(robot.getNumBodyNodes() - 1)
        dofs = robot.getNumDofs()
        if self._end_effector.getNumDependentGenCoords() != dofs:
            raise ValueError("end effector must depend on all robot DOFs")
        self._offset = np.array([0.05, 0, 0])
        self._kp = 50 * np.eye(3)
        self._kd = 5 * np.eye(dofs)
        self.target = target

    @property
    def robot(self):
        return self._robot

    @property
    def target(self):
        return self._target

    @target.setter
    def target(self, target):
        if not isinstance(target, dart.dynamics.SimpleFrame):
            raise TypeError("target must be a SimpleFrame")
        self._target = target

    @property
    def offset(self):
        return self._offset.copy()

    @offset.setter
    def offset(self, offset):
        self._offset = _finite_array(offset, (3,), "offset")

    @property
    def kp(self):
        return self._kp.copy()

    @kp.setter
    def kp(self, kp):
        self._kp = _finite_array(kp, (3, 3), "kp")

    @property
    def kd(self):
        return self._kd.copy()

    @kd.setter
    def kd(self, kd):
        dofs = self._robot.getNumDofs()
        self._kd = _finite_array(kd, (dofs, dofs), "kd")

    def compute_forces(self):
        robot = self._robot
        ee = self._end_effector
        if (
            robot.getNumBodyNodes() == 0
            or robot.getBodyNode(robot.getNumBodyNodes() - 1) is not ee
            or robot.getNumDofs() != self._kd.shape[0]
            or ee.getNumDependentGenCoords() != robot.getNumDofs()
        ):
            raise RuntimeError("robot structure changed after controller construction")

        mass = robot.getMassMatrix()
        jacobian = ee.getLinearJacobian(self._offset)
        pinv_j = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + 0.0025 * np.eye(3))
        derivative = ee.getLinearJacobianDeriv(self._offset)
        pinv_dj = derivative.T @ np.linalg.inv(
            derivative @ derivative.T + 0.0025 * np.eye(3)
        )
        tf = ee.getWorldTransform()
        error = self._target.getWorldTransform().translation() - (
            tf.rotation() @ self._offset + tf.translation()
        )
        velocity_error = -ee.getLinearVelocity(self._offset)
        return (
            mass @ (pinv_j @ self._kp @ velocity_error + pinv_dj @ self._kp @ error)
            + robot.getCoriolisAndGravityForces()
            + self._kd @ pinv_j @ self._kp @ error
        )

    def update(self):
        self._robot.setForces(self.compute_forces())
