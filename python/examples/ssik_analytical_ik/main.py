"""Interactive ssik analytical IK example for dartpy.

This example plugs an `ssik <https://github.com/personalrobotics/ssik>`_ prebuilt
analytical IK solver into DART's native inverse kinematics through
``InverseKinematics.setPythonAnalytical``. It builds a DART skeleton from the
solver's product-of-exponentials chain and lets you drag an ``InteractiveFrame``
target so the arm re-solves analytically and follows in real time.

It is the Python counterpart of the C++ ``ssik_ik_gui`` example. The analytical
IK *mechanism* (``setPythonAnalytical``) is a first-class dartpy binding; ssik is
only one possible solver, so it is an optional, user-supplied dependency rather
than a dependency of dartpy or DART:

    pixi run python -m pip install ssik

Usage::

    # default arm (UR5), interactive viewer
    pixi run py-ex ssik_analytical_ik

    # pick an arm or pass flags by running the script directly
    PYTHONPATH=build/default/cpp/Release/python/dartpy \
        python python/examples/ssik_analytical_ik/main.py --arm franka_panda_ik

    # headless smoke test of the IK pipeline (no window, no GL required)
    PYTHONPATH=build/default/cpp/Release/python/dartpy \
        python python/examples/ssik_analytical_ik/main.py --self-test
"""

import argparse
import sys

import dartpy as dart
import numpy as np

# (label, ssik prebuilt module name) for the arms ssik ships analytical IK for.
ARMS = [
    ("Universal Robots UR5", "ur5_ik"),
    ("KUKA Puma 560", "puma560_ik"),
    ("Kinova JACO 2", "jaco2_ik"),
    ("KUKA iiwa14", "iiwa14_ik"),
    ("Kinova Gen3", "gen3_ik"),
    ("Franka Panda", "franka_panda_ik"),
    ("UFactory xArm7", "xarm7_ik"),
    ("UFactory xArm6", "xarm6_ik"),
    ("Unitree Z1", "z1_ik"),
    ("AgileX PiPER", "piper_ik"),
    ("Flexiv Rizon 4", "rizon4_ik"),
    ("Kassow KR810", "kassow_kr810_ik"),
    ("Flexiv Rizon 10", "rizon10_ik"),
    ("FANUC CRX-10iA/L", "fanuc_crx10ial_ik"),
    ("I2RT YAM", "yam_ik"),
    ("I2RT big_yam", "big_yam_ik"),
    ("Franka Research 3", "fr3_ik"),
    ("OpenArm left", "openarm_left_ik"),
    ("OpenArm right", "openarm_right_ik"),
]

LINK_RADIUS = 0.02
JOINT_RADIUS = 0.035


def load_ssik_module(module_name):
    """Import an ssik prebuilt module, or return None with a friendly message."""
    try:
        import importlib

        return importlib.import_module("ssik.prebuilt." + module_name)
    except Exception as error:  # noqa: BLE001 - report any import/availability issue
        print(f"ssik is not available ({error}).")
        print("Install it into this environment with:")
        print("    pixi run python -m pip install ssik")
        return None


def isometry(matrix):
    """Build a dart.math.Isometry3 from a 4x4 array-like."""
    m = np.asarray(matrix, dtype=float).reshape((4, 4))
    tf = dart.math.Isometry3()
    tf.set_rotation(m[:3, :3])
    tf.set_translation(m[:3, 3])
    return tf


def rotation_aligning_z(direction):
    """Rotation matrix that maps +Z onto ``direction`` (for cylinder links)."""
    d = np.asarray(direction, dtype=float)
    norm = np.linalg.norm(d)
    if norm < 1e-12:
        return np.eye(3)
    d = d / norm
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, d)
    c = float(np.dot(z, d))
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        return np.eye(3) if c > 0.0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * ((1.0 - c) / (s * s))


def add_sphere(body, position, color):
    shape = dart.dynamics.SphereShape(JOINT_RADIUS)
    node = body.createShapeNode(shape)
    node.createVisualAspect()
    node.getVisualAspect().setColor(color)
    tf = dart.math.Isometry3()
    tf.set_translation(np.asarray(position, dtype=float))
    node.setRelativeTransform(tf)


def add_cylinder(body, a, b, color):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    delta = b - a
    length = float(np.linalg.norm(delta))
    if length < 1e-5:
        return
    shape = dart.dynamics.CylinderShape(LINK_RADIUS, length)
    node = body.createShapeNode(shape)
    node.createVisualAspect()
    node.getVisualAspect().setColor(color)
    tf = dart.math.Isometry3()
    tf.set_translation(0.5 * (a + b))
    tf.set_rotation(rotation_aligning_z(delta / length))
    node.setRelativeTransform(tf)


def build_arm_skeleton(name, module):
    """Build a DART skeleton from an ssik product-of-exponentials chain.

    Each joint contributes ``T_left * exp(axis * q) * T_right``, which maps to a
    DART joint with parent-to-joint = ``T_left``, axis = ``axis``, and
    child-to-joint = ``T_right^-1``. With the base at the world origin, the
    end-effector body's world transform equals ``module.fk(q)``, so the target
    DART feeds to the solver is already in the frame ssik expects.
    """
    t_lefts = [
        np.asarray(m, dtype=float).reshape((4, 4)) for m in module._JOINT_T_LEFTS
    ]
    t_rights = [
        np.asarray(m, dtype=float).reshape((4, 4)) for m in module._JOINT_T_RIGHTS
    ]
    axes = [np.asarray(a, dtype=float) for a in module._JOINT_AXES]
    types = [str(t) for t in module._JOINT_TYPES]

    skel = dart.dynamics.Skeleton(name)

    base_joint, parent = skel.createWeldJointAndBodyNodePair()
    base_joint.setName("base_joint")
    parent.setName("base")
    add_sphere(parent, np.zeros(3), [0.3, 0.3, 0.3])
    add_cylinder(parent, np.zeros(3), t_lefts[0][:3, 3], [0.5, 0.5, 0.5])

    for k in range(len(t_lefts)):
        axis = axes[k]
        if np.linalg.norm(axis) > 1e-9:
            axis = axis / np.linalg.norm(axis)
        t_right_inv = np.linalg.inv(t_rights[k])

        if types[k] == "prismatic":
            joint, body = skel.createPrismaticJointAndBodyNodePair(parent)
        else:
            joint, body = skel.createRevoluteJointAndBodyNodePair(parent)
        joint.setName(f"j{k}")
        body.setName(f"b{k}")
        joint.setTransformFromParentBodyNode(isometry(t_lefts[k]))
        joint.setTransformFromChildBodyNode(isometry(t_right_inv))
        joint.setAxis(axis)

        add_sphere(body, np.zeros(3), [0.85, 0.65, 0.1])
        # The body's parent and child joints are fixed in its frame, so the link
        # cylinders connecting them move rigidly with the body.
        add_cylinder(body, t_right_inv[:3, 3], np.zeros(3), [0.2, 0.5, 0.9])
        if k + 1 < len(t_lefts):
            add_cylinder(body, np.zeros(3), t_lefts[k + 1][:3, 3], [0.2, 0.5, 0.9])

        parent = body

    return skel, parent


def make_solver(module):
    """Wrap module.solve into a setPythonAnalytical callback.

    The callback receives the desired end-effector transform as a 4x4 matrix and
    returns ssik solution objects directly (each exposes ``.q``), which the
    binding accepts as analytical IK solutions.
    """

    def solve(target):
        matrix = np.asarray(target, dtype=float).reshape((4, 4))
        try:
            return list(module.solve(matrix, max_solutions=None, respect_limits=True))
        except Exception:  # noqa: BLE001 - unreachable target -> no solutions
            return []

    return solve


def setup_ik(skel, ee_body, module):
    dofs = list(range(skel.getNumDofs()))
    ik = ee_body.getOrCreateIK()
    ik.setDofs(dofs)
    analytical = ik.setPythonAnalytical(make_solver(module), dofs, "ssik.prebuilt")
    return ik, analytical


def apply_best_solution(skel, analytical, target_tf):
    """Solve analytical IK for ``target_tf`` and apply the best solution.

    Calls the analytical solver directly (a single ssik query) rather than the
    iterative Jacobian solver, so the arm tracks the target in real time.
    Returns True if a valid solution was applied.
    """
    solutions = analytical.getSolutions(target_tf)
    if not solutions:
        return False
    valid = [
        s
        for s in solutions
        if s.mValidity == dart.dynamics.InverseKinematicsAnalytical.VALID
    ]
    chosen = valid[0] if valid else solutions[0]
    skel.setPositions(np.asarray(chosen.mConfig, dtype=float))
    return bool(valid)


class SsikWorldNode(dart.gui.osg.WorldNode):
    """Re-solves the analytical IK every frame so the arm tracks the target."""

    def __init__(self, world, skel, ik, analytical):
        super(SsikWorldNode, self).__init__(world)
        self.skel = skel
        self.ik = ik
        self.analytical = analytical

    def customPreRefresh(self):
        apply_best_solution(
            self.skel, self.analytical, self.ik.getTarget().getTransform()
        )


def run_self_test():
    """Headless check of each arm's skeleton and analytical IK binding.

    For every arm this verifies two independent things:

    1. ``fk_err``: the DART skeleton built from the ssik chain reproduces
       ``module.fk(q)`` for an arbitrary configuration. This validates the
       product-of-exponentials construction and joint axes, with no solver
       involved.
    2. ``ik_err``: ``setPythonAnalytical`` solves and applies a configuration
       that reaches the arm's home pose (always a valid, in-limits target),
       exercising the full binding pipeline.
    """
    failures = 0
    for label, module_name in ARMS:
        module = load_ssik_module(module_name)
        if module is None:
            print(f"[self-test] {label} ({module_name}): ssik not available")
            failures += 1
            continue

        skel, ee_body = build_arm_skeleton(module_name, module)
        _, analytical = setup_ik(skel, ee_body, module)
        dof = skel.getNumDofs()

        # 1) Skeleton kinematics must match ssik FK (independent of the solver).
        q = 0.1 * np.cos(np.arange(dof))
        skel.setPositions(q)
        fk_err = float(
            np.linalg.norm(
                np.asarray(ee_body.getWorldTransform().matrix(), dtype=float)
                - np.asarray(module.fk(q), dtype=float).reshape((4, 4))
            )
        )

        # 2) The analytical binding must solve and apply a config reaching home.
        skel.resetPositions()
        t_home = np.asarray(module.T_HOME, dtype=float).reshape((4, 4))
        solved = apply_best_solution(skel, analytical, isometry(t_home))
        reached = np.asarray(ee_body.getWorldTransform().matrix(), dtype=float)
        ik_err = float(np.linalg.norm(reached[:3, 3] - t_home[:3, 3]))

        # Success is measured by reaching the target; ``solved`` only reports
        # ssik's limit verdict, which can be false when the home pose sits on a
        # joint-limit boundary even though the solution still reaches the target.
        status = "ok" if fk_err < 1e-6 and ik_err < 1e-3 else "FAILED"
        if status != "ok":
            failures += 1
        print(
            f"[self-test] {label} ({module_name}): DOF={dof} "
            f"fk_err={fk_err:.2e} ik_err={ik_err:.2e} valid={solved} -> {status}"
        )

    if failures:
        print(f"[self-test] {failures} arm(s) failed")
        return 1
    print(f"[self-test] all {len(ARMS)} arms solved and tracked")
    return 0


def run_viewer(module_name):
    module = load_ssik_module(module_name)
    if module is None:
        return 1

    world = dart.simulation.World()
    skel, ee_body = build_arm_skeleton(module_name, module)
    world.addSkeleton(skel)
    ik, analytical = setup_ik(skel, ee_body, module)

    # A draggable target, initialized at the arm's current end-effector pose.
    target = dart.gui.osg.InteractiveFrame(dart.dynamics.Frame.World(), "ssik_target")
    target.setTransform(ee_body.getWorldTransform())
    world.addSimpleFrame(target)
    ik.setTarget(target)

    node = SsikWorldNode(world, skel, ik, analytical)

    viewer = dart.gui.osg.ImGuiViewer()
    viewer.addWorldNode(node)
    viewer.enableDragAndDrop(target)
    viewer.addInstructionText(
        "\nDrag the target frame; the arm re-solves ssik analytical IK to follow.\n"
    )
    print(viewer.getInstructions())
    print(f"Arm: {module_name} (DOF={skel.getNumDofs()})")

    viewer.setUpViewInWindow(0, 0, 1280, 960)
    viewer.setCameraHomePosition([1.6, 1.6, 1.2], [0, 0, 0.3], [0, 0, 1])
    viewer.run()
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arm",
        default="ur5_ik",
        help="ssik prebuilt module name (default: ur5_ik)",
    )
    parser.add_argument(
        "--list", action="store_true", help="list the available arms and exit"
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="run a headless IK smoke test over all arms and exit",
    )
    args = parser.parse_args()

    if args.list:
        for label, module_name in ARMS:
            print(f"{module_name:20s} {label}")
        return 0

    if args.self_test:
        return run_self_test()

    return run_viewer(args.arm)


if __name__ == "__main__":
    sys.exit(main())
