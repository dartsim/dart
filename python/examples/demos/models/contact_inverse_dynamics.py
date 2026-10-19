"""Squat replay, force arrows, and histories around core contact dynamics."""

import math

import dartpy as dart
import numpy as np


def _control(value, lower, upper):
    if isinstance(value, (str, bytes)) or value is None:
        raise TypeError("control value must be numeric")
    try:
        value = float(value)
    except OverflowError as error:
        raise TypeError("control value must be numeric") from error
    if not math.isfinite(value) or abs(value) > float(np.finfo(np.float32).max):
        raise ValueError("control value must be finite")
    return float(np.clip(np.float32(value), np.float32(lower), np.float32(upper)))


class ContactInverseDynamicsModel:
    def __init__(self, world):
        if world is None:
            raise ValueError("world must not be None")
        if not isinstance(world, dart.simulation.World):
            raise TypeError("world must be a World")
        gravity = world.getGravity()
        if not np.isfinite(gravity).all() or np.linalg.norm(gravity) == 0:
            raise ValueError("world gravity must be finite and nonzero")
        self._world = world
        self._biped = world.getSkeleton("fullbody1")
        if self._biped is None:
            raise RuntimeError("fullbody1.skel: missing skeleton 'fullbody1'")
        for name, position in (
            ("j_pelvis_rot_y", -0.20),
            ("j_thigh_left_z", 0.15),
            ("j_shin_left", -0.40),
            ("j_heel_left_1", 0.25),
            ("j_thigh_right_z", 0.15),
            ("j_shin_right", -0.40),
            ("j_heel_right_1", 0.25),
            ("j_abdomen_2", 0.00),
        ):
            dof = self._biped.getDof(name)
            if dof is not None:
                dof.setPosition(position)

        self._solver = dart.dynamics.ContactInverseDynamics(self._biped)
        # Match the demo's steady force distribution and residual tolerance.
        self._solver.setRegularization(1e-4)
        self._solver.setResidualTolerance(1e-4)
        self._contact_bodies = [
            self._biped.getBodyNode(name)
            for name in ("h_heel_left", "h_toe_left", "h_heel_right", "h_toe_right")
        ]
        if any(body is None for body in self._contact_bodies):
            raise RuntimeError("fullbody1.skel: missing one or more foot body nodes")
        self._contact_frames = []
        self._arrows = []
        for _ in range(4):
            frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World())
            arrow = dart.dynamics.ArrowShape(
                [0, 0, 0],
                [0, 0, 0.1],
                dart.dynamics.ArrowShapeProperties(0.01, 2.0, 0.15),
                [0, 1, 0, 1],
            )
            frame.setShape(arrow)
            frame.getVisualAspect(True)
            world.addSimpleFrame(frame)
            self._contact_frames.append(frame)
            self._arrows.append(arrow)

        self._torque_names = []
        self._torque_dofs = []
        for name in (
            "j_thigh_left_z",
            "j_shin_left",
            "j_heel_left_1",
            "j_thigh_right_z",
            "j_shin_right",
            "j_heel_right_1",
        ):
            dof = self._biped.getDof(name)
            if dof is not None and len(self._torque_names) < 3:
                self._torque_names.append(name)
                self._torque_dofs.append(dof.getIndexInSkeleton())
        self._initial_positions = self._biped.getPositions().copy()
        self._time = 0.0
        self.playing = True
        self.playback_speed = 1.0
        self.squat_depth = 0.15
        self.friction_coeff = 0.8
        self.force_scale = 0.002
        self._regeneration_requested = False
        self._feasible = True
        self._residual_norm = 0.0
        self._contact_forces = []
        self._total_vertical_force = 0.0
        self._plot_samples = np.zeros(240, dtype=np.float32)
        self._torque_plots = np.zeros((3, 240), dtype=np.float32)
        self._sample_index = 0
        # The original constructor uses double 0.15, before float32 controls.
        self._generate_keyframes(0.15)

    @property
    def world(self):
        return self._world

    @property
    def biped(self):
        return self._biped

    @property
    def time(self):
        return self._time

    @property
    def keyframe_count(self):
        return len(self._keyframes)

    @property
    def regeneration_requested(self):
        return self._regeneration_requested

    @property
    def playing(self):
        return self._playing

    @playing.setter
    def playing(self, value):
        if type(value) is not bool:
            raise TypeError("playing must be a bool")
        self._playing = value

    @property
    def playback_speed(self):
        return self._playback_speed

    @playback_speed.setter
    def playback_speed(self, value):
        self._playback_speed = _control(value, 0.1, 2.0)

    @property
    def squat_depth(self):
        return self._squat_depth

    @squat_depth.setter
    def squat_depth(self, value):
        self._squat_depth = _control(value, 0.0, 0.25)

    @property
    def friction_coeff(self):
        return self._friction_coeff

    @friction_coeff.setter
    def friction_coeff(self, value):
        self._friction_coeff = _control(value, 0.0, 1.5)

    @property
    def force_scale(self):
        return self._force_scale

    @force_scale.setter
    def force_scale(self, value):
        self._force_scale = _control(value, 0.0001, 0.01)

    @property
    def feasible(self):
        return self._feasible

    @property
    def residual_norm(self):
        return self._residual_norm

    @property
    def contact_forces(self):
        return [force.copy() for force in self._contact_forces]

    @property
    def total_vertical_force(self):
        return self._total_vertical_force

    @property
    def torque_names(self):
        return self._torque_names.copy()

    @property
    def torque_plots(self):
        return self._torque_plots.tolist()

    @property
    def plot_samples(self):
        return self._plot_samples.tolist()

    @property
    def sample_index(self):
        return self._sample_index

    def request_regeneration(self):
        self._regeneration_requested = True

    def _generate_keyframes(self, depth):
        biped = self._biped
        biped.setPositions(self._initial_positions)
        for heel in (self._contact_bodies[0], self._contact_bodies[2]):
            ik = heel.getIK(True)
            ik.setTarget(
                dart.dynamics.SimpleFrame(
                    dart.dynamics.Frame.World(),
                    (
                        "leftHeelTarget"
                        if heel is self._contact_bodies[0]
                        else "rightHeelTarget"
                    ),
                    heel.getWorldTransform(),
                )
            )
            ik.setHierarchyLevel(0)
        pelvis = biped.getBodyNode(0)
        pelvis_ik = pelvis.getIK(True)
        pelvis_initial = pelvis.getWorldTransform()
        pelvis_target = dart.dynamics.SimpleFrame(
            dart.dynamics.Frame.World(),
            "pelvisTarget",
            pelvis_initial,
        )
        pelvis_ik.setTarget(pelvis_target)
        pelvis_ik.setHierarchyLevel(1)
        whole_body_ik = biped.getIK(True)
        solver = whole_body_ik.getSolver()
        if isinstance(solver, dart.optimizer.GradientDescentSolver):
            solver.setNumMaxIterations(200)
        self._keyframes = []
        for index in range(60):
            offset = depth * 0.5 * (1.0 - math.cos(2.0 * math.pi * (index / 60)))
            transform = dart.math.Isometry3(pelvis_initial.matrix())
            translation = transform.translation().copy()
            translation[2] -= offset
            transform.set_translation(translation)
            pelvis_target.setTransform(transform)
            whole_body_ik.solveAndApply(True)
            self._keyframes.append(biped.getPositions().copy())
            if index == 0 and isinstance(solver, dart.optimizer.GradientDescentSolver):
                solver.setNumMaxIterations(50)
        biped.setPositions(self._initial_positions)

    def _spline(self, phase):
        count = len(self._keyframes)
        if count == 0:
            zeros = np.zeros(self._biped.getNumDofs())
            return self._biped.getPositions().copy(), zeros, zeros.copy()
        scaled = phase * count
        index = int(scaled) % count
        u = scaled - math.floor(scaled)
        p0, p1, p2, p3 = (
            self._keyframes[i % count] for i in (index - 1, index, index + 1, index + 2)
        )
        a = -p0 + 3.0 * p1 - 3.0 * p2 + p3
        b = 2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3
        c = -p0 + p2
        u2, u3 = u * u, u * u * u
        return (
            0.5 * (a * u3 + b * u2 + c * u + 2.0 * p1),
            0.5 * count * ((3.0 * a) * u2 + (2.0 * b) * u + c),
            0.5 * (count * count) * ((6.0 * a) * u + 2.0 * b),
        )

    def update_frame(self):
        if self._regeneration_requested:
            self._generate_keyframes(self.squat_depth)
            self._regeneration_requested = False
        if self.playing:
            self._time += 1.0 / 60.0 * self.playback_speed
        phase = math.fmod(self._time, 3.0) / 3.0
        phase_rate = self.playback_speed / 3.0 if self.playing else 0.0
        q, dq, ddq = self._spline(phase)
        self._biped.setPositions(q)
        self._biped.setVelocities(dq * phase_rate)
        self._biped.setAccelerations(ddq * phase_rate * phase_rate)

        gravity = self._world.getGravity()
        up = -gravity / np.linalg.norm(gravity)
        contacts = []
        for body in self._contact_bodies:
            contact = dart.dynamics.ContactInverseDynamics.Contact()
            contact.bodyNode = body
            contact.localOffset = [0, 0, 0]
            contact.normal = up
            contact.frictionCoeff = self.friction_coeff
            contact.numBasis = 8
            contacts.append(contact)
        self._solver.setContacts(contacts)
        result = self._solver.compute()
        self._feasible = result.feasible
        self._residual_norm = float(
            np.max(np.abs(result.unactuatedResidual), initial=0.0)
        )
        self._contact_forces = [force.copy() for force in result.contactForces]
        self._total_vertical_force = sum(
            float(force.dot(up)) for force in self._contact_forces
        )
        self._plot_samples[self._sample_index] = np.float32(self._total_vertical_force)
        forces = result.jointForces
        for index, dof in enumerate(self._torque_dofs):
            if dof < len(forces):
                self._torque_plots[index, self._sample_index] = np.float32(forces[dof])
        self._sample_index = (self._sample_index + 1) % len(self._plot_samples)
        for index, force in enumerate(
            self._contact_forces[: len(self._contact_frames)]
        ):
            frame = self._contact_frames[index]
            visual = frame.getVisualAspect(True)
            hidden = np.linalg.norm(force) < 1e-8
            visual.setHidden(bool(hidden))
            if hidden:
                continue
            position = (
                self._contact_bodies[index]
                .getWorldTransform()
                .multiply(contacts[index].localOffset)
            )
            arrow = self._arrows[index]
            arrow.setPositions(position, position + self.force_scale * force)
            arrow.notifyColorUpdated([0, 1, 0, 1] if self.feasible else [1, 0, 0, 1])
