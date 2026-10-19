"""Y-up Atlas SIMBICON states, gait timing, balance feedback, and harnesses."""

import math

import dartpy as dart
import numpy as np

_BODIES = ("pelvis", "l_foot", "r_foot", "l_uleg", "r_uleg")
_DOFS = (
    "l_leg_hpx",
    "r_leg_hpx",
    "l_leg_hpy",
    "r_leg_hpy",
    "back_bky",
    "l_leg_kny",
    "r_leg_kny",
    "l_leg_aky",
    "r_leg_aky",
    "l_arm_shx",
    "r_arm_shx",
    "l_arm_shy",
    "r_arm_shy",
)


def _number(value):
    try:
        if issubclass(type(value), float):
            return float.__float__(value)
        for base in type.__getattribute__(type(value), "__mro__"):
            namespace = type.__getattribute__(base, "__dict__")
            if "__float__" in namespace or "__index__" in namespace:
                return float(value)
    except BaseException:
        pass
    raise TypeError("time must be a real number")


def _name(value):
    if not issubclass(type(value), str):
        raise TypeError("name must be a string")
    value = str.__str__(value)
    try:
        value.encode("utf-8")
    except BaseException:
        raise TypeError("name must be a string") from None
    return value


def _radians(degrees):
    # Native _deg literals use decimal long doubles before rounding to double.
    pi = np.longdouble(
        "3.141592653589793238462643383279502884197169399375105820974944592"
    )
    return float(np.longdouble(str(degrees)) * pi / 180.0)


def _message(text):
    print(f"\033[1;32mMsg\033[0m {text}")


def _dot(a, b):
    return (a[0] * b[0] + a[1] * b[1]) + a[2] * b[2]


def _norm(vector):
    return math.sqrt(float(_dot(vector, vector)))


def _normalized(vector):
    squared = _dot(vector, vector)
    return vector / math.sqrt(float(squared)) if squared > 0 else vector.copy()


def _angle(a, b):
    ratio = _dot(a, b) / np.float64(_norm(a) * _norm(b))
    try:
        return math.acos(float(ratio))
    except ValueError:
        return math.nan


def _mass_average(atlas, velocity=False):
    total = 0.0
    weighted = np.zeros(3)
    for body in atlas.getBodyNodes():
        # SoftBodyNode.getMass includes its points; native State uses link mass.
        mass = dart.dynamics.BodyNode.getMass(body)
        term = body.getCOMLinearVelocity() if velocity else body.getCOM()
        weighted += mass * term
        total += mass
        if issubclass(type(body), dart.dynamics.SoftBodyNode):
            for index in range(body.getNumPointMasses()):
                point = body.get_point_mass(index)
                term = point.world_velocity if velocity else point.world_position
                weighted += point.mass * term
                total += point.mass
    return weighted / total


class _TimerCondition:
    def __init__(self, state, duration):
        self.state = state
        self.duration = duration

    def is_satisfied(self):
        return self.state.elapsed > self.duration


class _BodyContactCondition:
    def __init__(self, body):
        self.body = body

    def is_satisfied(self):
        body = self.body
        if issubclass(type(body), dart.dynamics.SoftBodyNode):
            for index in range(body.getNumPointMasses()):
                if body.get_point_mass(index).is_colliding:
                    return True
        return body.is_colliding


class _State:
    def __init__(self, atlas, name, hips):
        self.atlas = atlas
        self.name = name
        self.hips = hips
        self.pelvis = atlas.getBodyNode("pelvis")
        self.left_foot = atlas.getBodyNode("l_foot")
        self.right_foot = atlas.getBodyNode("r_foot")
        self.stance = None
        self.next_state = self
        self.condition = None
        self.begin_time = self.end_time = self.elapsed = 0.0
        self.frame = 0
        dofs = atlas.getNumDofs()
        self.desired = np.zeros(dofs)
        self.sagital_cd = np.zeros(dofs)
        self.sagital_cv = np.zeros(dofs)
        self.coronal_cd = np.zeros(dofs)
        self.coronal_cv = np.zeros(dofs)

    def begin(self, time):
        self.begin_time = time
        self.elapsed = 0.0
        self.frame = 0

    def end(self, time):
        self.end_time = time

    def set_targets(self, targets):
        for name, value in targets.items():
            self.desired[self.atlas.getDof(name).getIndexInSkeleton()] = value

    def _pelvis_angle(self, rotation, sagittal):
        pelvis_z = self.pelvis.getTransform().rotation()[:, 2]
        projected = np.array([_dot(rotation[:, index], pelvis_z) for index in range(3)])
        projected[2 if sagittal else 0] = 0.0
        projected = _normalized(projected)
        angle = _angle(projected, rotation[:, 1])
        cross = np.cross(rotation[:, 1], projected)
        return angle if cross[2 if sagittal else 0] > 0.0 else -angle

    def compute_control_force(self, dt):
        with np.errstate(all="ignore"):
            com = _mass_average(self.atlas)
            velocity = _mass_average(self.atlas, velocity=True)
            y_axis = np.array([0.0, 1.0, 0.0])
            pelvis_x = self.pelvis.getTransform().rotation()[:, 0]
            x_axis = _normalized(pelvis_x - _dot(y_axis, pelvis_x) * y_axis)
            z_axis = np.cross(x_axis, y_axis)
            rotation = np.column_stack((x_axis, y_axis, z_axis))
            ankle = com
            if self.stance is not None:
                local = (
                    self.stance.getParentJoint()
                    .getTransformFromChildBodyNode()
                    .translation()
                )
                transform = self.stance.getTransform()
                ankle = transform.rotation() @ local + transform.translation()
            distance = com - ankle
            desired = (
                self.desired
                + _dot(distance, x_axis) * self.sagital_cd
                + _dot(velocity, x_axis) * self.sagital_cv
                + _dot(distance, z_axis) * self.coronal_cd
                + _dot(velocity, z_axis) * self.coronal_cv
            )
            q = self.atlas.getPositions()
            dq = self.atlas.getVelocities()
            torque = np.zeros(len(q))
            for index in range(6, len(q)):
                torque[index] = -1000.0 * (q[index] - desired[index]) - 1.0 * dq[index]
            coronal_left, coronal_right, sagital_left, sagital_right = self.hips
            if self.stance is not None:
                sagittal = -5000.0 * (self._pelvis_angle(rotation, True) + 0.0)
                coronal = -5000.0 * (self._pelvis_angle(rotation, False) - 0.0)
                if self.stance is self.left_foot:
                    torque[sagital_left] = sagittal - torque[sagital_right]
                    torque[coronal_left] = -coronal - torque[coronal_right]
                else:
                    torque[sagital_right] = sagittal - torque[sagital_left]
                    torque[coronal_right] = -coronal - torque[coronal_left]
            self.atlas.setForces(torque)
        self.elapsed += dt
        self.frame += 1


class _StateMachine:
    def __init__(self, name, states, initial):
        self.name = name
        self.states = states
        self.initial_state = states[initial]
        self.current_state = self.initial_state
        self.begin_time = self.end_time = self.elapsed = 0.0
        self.frame = 0

    def begin(self, time):
        self.begin_time = time
        self.elapsed = 0.0
        self.frame = 0

    def end(self, time):
        self.end_time = time

    def transition(self, state, time):
        self.current_state.end(time)
        self.current_state = state
        state.begin(time)

    def compute_control_force(self, dt):
        if self.current_state.condition.is_satisfied():
            self.transition(
                self.current_state.next_state, self.begin_time + self.elapsed
            )
        self.current_state.compute_control_force(dt)
        self.elapsed += dt
        self.frame += 1


def _standing(atlas, hips):
    state = _State(atlas, "0", hips)
    state.condition = _TimerCondition(state, 0.3)
    state.set_targets(
        {
            "back_bky": _radians(15.0),
            "l_leg_hpy": _radians(-10.0),
            "r_leg_hpy": _radians(-10.0),
            "l_leg_kny": _radians(30.0),
            "r_leg_kny": _radians(30.0),
            "l_leg_aky": _radians(-16.8),
            "r_leg_aky": _radians(-16.8),
            "l_arm_shx": _radians(-90.0),
            "r_arm_shx": _radians(90.0),
        }
    )
    return _StateMachine("standing", [state], 0)


def _gait(atlas, hips, name, running=False):
    states = [_State(atlas, str(index), hips) for index in range(2 if running else 4)]
    back = _radians(4.75 if name == "walking in place" else 10.0)
    for index, state in enumerate(states):
        state.next_state = states[(index + 1) % len(states)]
        left_stance = index == 0 if running else index < 2
        state.stance = state.left_foot if left_stance else state.right_foot
        if running or index % 2 == 0:
            swing = "r" if (index == 0) else "l"
            other = "l" if swing == "r" else "r"
            targets = {
                f"{swing}_leg_hpy": -0.50,
                f"{swing}_leg_kny": 1.10,
                f"{swing}_leg_aky": -0.60,
                f"{other}_leg_kny": 0.05,
                f"{other}_leg_aky": 0.0,
            }
        else:
            swing = "l" if index == 1 else "r"
            other = "r" if swing == "l" else "l"
            targets = {
                f"{swing}_leg_hpy": 0.10,
                f"{swing}_leg_kny": 0.05,
                f"{swing}_leg_aky": -0.15,
                f"{other}_leg_kny": 0.10,
                f"{other}_leg_aky": 0.0,
            }
        if running:
            left_arm, right_arm = (-45.0, 15.0) if index == 0 else (15.0, -45.0)
        else:
            left_arm, right_arm = (-20.0, 10.0) if index in (0, 3) else (10.0, -20.0)
        targets.update(
            {
                "back_bky": back,
                "l_arm_shy": _radians(left_arm),
                "r_arm_shy": _radians(right_arm),
                "l_arm_shx": _radians(-80.0),
                "r_arm_shx": _radians(80.0),
            }
        )
        state.set_targets(targets)
        for hip in hips[:2]:
            state.coronal_cd[hip] = -0.5
            state.coronal_cv[hip] = -0.2
        for hip in hips[2:]:
            state.sagital_cd[hip] = -0.5
            state.sagital_cv[hip] = -0.2
        if running or index % 2 == 0:
            state.condition = _TimerCondition(state, 0.15 if running else 0.3)
        else:
            state.condition = _BodyContactCondition(
                state.right_foot if index == 1 else state.left_foot
            )
    return _StateMachine(name, states, 0 if running else 1)


class Controller:
    """Retain Atlas and its World while applying the catalog's four gait machines."""

    def __init__(self, atlas, world):
        if not issubclass(type(atlas), dart.dynamics.Skeleton):
            raise TypeError("atlas must be a Skeleton")
        if not issubclass(type(world), dart.simulation.World):
            raise TypeError("world must be a World")
        for name in _BODIES:
            if atlas.getBodyNode(name) is None:
                raise ValueError(f"SIMBICON requires body node '{name}'")
        for name in _DOFS:
            if atlas.getDof(name) is None:
                raise ValueError(f"SIMBICON requires DOF '{name}'")
        self._atlas = atlas
        self._world = world
        self._pelvis_harness_on = False
        self._left_foot_harness_on = False
        self._right_foot_harness_on = False
        self._pelvis_constraint = self._left_foot_constraint = (
            self._right_foot_constraint
        ) = None
        self._verbosity = False
        hips = tuple(atlas.getDof(name).getIndexInSkeleton() for name in _DOFS[:4])
        machines = [
            _standing(atlas, hips),
            _gait(atlas, hips, "walking in place"),
            _gait(atlas, hips, "walking"),
            _gait(atlas, hips, "running", running=True),
        ]
        self._machines = {machine.name: machine for machine in machines}
        self._current = machines[1]
        self._current.begin(0.0)
        for index in range(1, atlas.getNumBodyNodes()):
            joint = atlas.getJoint(index)
            for dof in range(joint.getNumDofs()):
                joint.setDampingCoefficient(dof, 80.0)
        # Skeleton ConfigFlags: POSITIONS (1 << 1) | VELOCITIES (1 << 2).
        self._initial = atlas.getConfiguration(6)

    def __del__(self):
        if getattr(self, "_pelvis_harness_on", False):
            try:
                self.unharness_pelvis()
            except (AttributeError, ValueError):
                # A collected native World has already removed its constraints.
                pass

    @property
    def atlas(self):
        return self._atlas

    @property
    def world(self):
        return self._world

    @property
    def current_state_machine(self):
        return self._current.name

    @property
    def current_state(self):
        return self._current.current_state.name

    def update(self):
        if self.is_allowing_control():
            self._current.compute_control_force(self._atlas.getTimeStep())

    def change_state_machine(self, name, time):
        name = _name(name)
        time = _number(time)
        if name not in self._machines:
            raise ValueError(f"Unknown SIMBICON state machine '{name}'")
        machine = self._machines[name]
        if machine is self._current:
            return
        previous = self._current.name
        self._current.end(time)
        self._current = machine
        machine.begin(time)
        if self._verbosity:
            _message(f"State machine transition: from [{previous}] to [{name}].")

    def is_allowing_control(self):
        y = self._atlas.getBodyNode("pelvis").getTransform().translation()[1]
        return bool(not (y < -0.70 or 0.30 < y))

    def _harness(self, name, body):
        if getattr(self, f"_{name}_harness_on"):
            return
        constraint = dart.constraint.WeldJointConstraint(self._atlas.getBodyNode(body))
        setattr(self, f"_{name}_constraint", constraint)
        if name == "pelvis":
            self._world.getConstraintSolver().addConstraint(constraint)
        setattr(self, f"_{name}_harness_on", True)
        if self._verbosity:
            _message(f"{name.replace('_', ' ').capitalize()} is harnessed.")

    def _unharness(self, name):
        if not getattr(self, f"_{name}_harness_on"):
            return
        self._world.getConstraintSolver().removeConstraint(
            getattr(self, f"_{name}_constraint")
        )
        setattr(self, f"_{name}_harness_on", False)
        if self._verbosity:
            _message(f"{name.replace('_', ' ').capitalize()} is unharnessed.")

    def harness_pelvis(self):
        self._harness("pelvis", "pelvis")

    def unharness_pelvis(self):
        self._unharness("pelvis")

    def harness_left_foot(self):
        self._harness("left_foot", "l_foot")

    def unharness_left_foot(self):
        self._unharness("left_foot")

    def harness_right_foot(self):
        self._harness("right_foot", "r_foot")

    def unharness_right_foot(self):
        self._unharness("right_foot")

    def reset_robot(self):
        self._atlas.setConfiguration(self._initial)
        if self._verbosity:
            _message("Robot is reset.")

    def _restart_gait(self, time):
        time = _number(time)
        self._current.transition(self._current.initial_state, time)
        self._current.begin(time)

    def set_verbosity(self, enabled):
        if type(enabled) is not bool:
            raise TypeError("enabled must be a bool")
        self._verbosity = enabled

    def print_debug_info(self):
        atlas = self._atlas
        print("[ATLAS Robot]")
        print(f" NUM NODES : {atlas.getNumBodyNodes()}")
        print(f" NUM DOF   : {atlas.getNumDofs()}")
        print(f" NUM JOINTS: {atlas.getNumBodyNodes()}")
        for index in range(atlas.getNumBodyNodes()):
            joint, body = atlas.getJoint(index), atlas.getBodyNode(index)
            print(f"  Joint [{index}]: {joint.getName()} ({joint.getNumDofs()})")
            parent = body.getParentBodyNode()
            if parent is not None:
                print(f"    Parent body: {parent.getName()}")
            print(f"    Child body : {body.getName()}")
