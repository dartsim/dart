#!/usr/bin/env python3

"""Exercise nanobind ABI-sensitive dartpy binding paths."""

import gc


def main():
    import dartpy as dart

    def make_body_from_temporary_skeleton():
        skeleton = dart.Skeleton("nanobind_compatibility")
        _, body = skeleton.create_revolute_joint_and_body_node_pair()
        return body

    body = make_body_from_temporary_skeleton()
    gc.collect()
    if not body.get_name():
        raise AssertionError("BodyNode did not keep its Skeleton alive")

    child_frame = dart.SimpleFrame(body, "nanobind_compatibility_child")
    parent = child_frame.get_parent_frame()
    if not isinstance(parent, dart.BodyNode):
        raise AssertionError("Frame-to-BodyNode polymorphic cast failed")

    other_skeleton = dart.Skeleton("nanobind_compatibility_other")
    _, other_body = other_skeleton.create_revolute_joint_and_body_node_pair()
    constraint = dart.BallJointConstraint(body, other_body, [0.0, 0.0, 0.0])
    if constraint.get_type() != dart.BallJointConstraint.get_static_type():
        raise AssertionError("BodyNode handle cast failed")

    def make_joints_from_temporary_world():
        world = dart.World()
        world.multibody_options = dart.MultibodyOptions(
            integration_family=dart.MultibodyIntegrationFamily.VARIATIONAL
        )
        arm = world.add_multibody("nanobind_compatibility_arm")
        base = arm.add_link("base")
        child = arm.add_link(
            "child",
            parent=base,
            joint=dart.JointSpec(name="floating", type=dart.JointType.FLOATING),
        )
        world.add_joint(
            base,
            child,
            dart.JointSpec(
                name="nanobind_compatibility_hold",
                type=dart.JointType.FIXED,
            ),
        )
        return world.joints

    joints = make_joints_from_temporary_world()
    gc.collect()
    if len(joints) != 1 or joints[0].parent_link.name != "base":
        raise AssertionError("Joint handles did not keep their World alive")

    class CustomSolver(dart.Solver):
        def solve(self):
            return True

        def getType(self):
            return "NanobindCompatibilitySolver"

        def clone(self):
            return CustomSolver()

    solver = CustomSolver()
    get_solver_type = getattr(dart, "get_solver_type_wrapper", None)
    if get_solver_type is None:
        raise AssertionError("Solver trampoline test helper is unavailable")
    if get_solver_type(solver) != "NanobindCompatibilitySolver":
        raise AssertionError("Solver trampoline dispatch failed")
    if get_solver_type(solver) != "NanobindCompatibilitySolver":
        raise AssertionError("Solver trampoline cache failed")

    print("nanobind compatibility smoke passed")


if __name__ == "__main__":
    main()
