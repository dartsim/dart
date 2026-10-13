"""Ball, cylindrical and weld constraints with native visual guides."""

import dartpy as dart


def _transform(position):
    transform = dart.math.Isometry3()
    transform.set_translation(position)
    return transform


def _marker(world, name, shape, color, position=(0.0, 0.0, 0.0)):
    frame = dart.dynamics.SimpleFrame(
        dart.dynamics.Frame.World(), name, _transform(position)
    )
    frame.setShape(shape)
    frame.createVisualAspect()
    frame.getVisualAspect().setColor(color)
    world.addSimpleFrame(frame)


def _line(world, name, start, end, thickness, color):
    _marker(
        world,
        name,
        dart.dynamics.LineSegmentShape(start, end, thickness),
        color,
    )


def _free_box(world, name, position, size, color, velocity, offset=(0.0, 0.0, 0.0)):
    skeleton = dart.dynamics.Skeleton(name)
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    joint.setPositions(dart.dynamics.FreeJoint.convertToPositions(_transform(position)))
    body.setMass(1.0)
    for i in range(joint.getNumDofs()):
        joint.setDampingCoefficient(i, 0.02)
    node = body.createShapeNode(dart.dynamics.BoxShape(size))
    node.createVisualAspect()
    node.createCollisionAspect()
    node.createDynamicsAspect()
    node.setRelativeTranslation(offset)
    node.getVisualAspect().setColor(color)
    joint.setVelocities(velocity)
    world.addSkeleton(skeleton)
    return body


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    world.setGravity([0.0, 0.0, -4.0])
    world.setTimeStep(1e-3)

    for name, position, color in (
        ("ball_lane_pad", [-3.0, 0.0, 0.02], [0.60, 0.20, 0.16]),
        ("cylindrical_lane_pad", [0.0, 0.0, 0.02], [0.12, 0.42, 0.62]),
        ("weld_lane_pad", [3.0, 0.0, 0.02], [0.20, 0.52, 0.20]),
    ):
        _marker(world, name, dart.dynamics.BoxShape([1.9, 1.25, 0.04]), color, position)
    for name, start, end, color in (
        ("ball_rotation_cue", [-3.55, 0.0, 2.1], [-2.45, 0.0, 1.5], [1.0, 0.80, 0.20]),
        (
            "weld_lock_cue_a",
            [2.45, -0.35, 1.55],
            [3.55, 0.35, 0.75],
            [0.95, 0.95, 0.95],
        ),
        (
            "weld_lock_cue_b",
            [2.45, 0.35, 1.55],
            [3.55, -0.35, 0.75],
            [0.95, 0.95, 0.95],
        ),
    ):
        _line(world, name, start, end, 4.0, color)

    ball = _free_box(
        world,
        "ball_body",
        [-2.35, 0.0, 1.35],
        [0.75, 0.25, 0.25],
        [0.85, 0.25, 0.20],
        [0.0, 1.4, 0.0, 0.0, 0.0, 0.0],
    )
    _marker(
        world,
        "ball_anchor",
        dart.dynamics.SphereShape(0.08),
        [1.0, 1.0, 1.0],
        [-3.0, 0.0, 1.8],
    )
    _line(
        world,
        "ball_anchor_line",
        [-3.0, 0.0, 1.35],
        [-3.0, 0.0, 2.25],
        2.0,
        [0.75, 0.75, 0.75],
    )
    world.getConstraintSolver().addConstraint(
        dart.constraint.BallJointConstraint(ball, [-3.0, 0.0, 1.8])
    )

    cylindrical = _free_box(
        world,
        "cylindrical_body",
        [0.0, 0.0, 1.15],
        [0.28, 0.65, 0.25],
        [0.20, 0.55, 0.95],
        [0.0, 0.0, 4.0, 0.0, 0.0, 1.2],
        offset=[0.35, 0.0, 0.0],
    )
    axis_color = [0.20, 0.85, 1.0]
    _line(world, "cylindrical_axis", [0.0, 0.0, 0.0], [0.0, 0.0, 2.8], 4.0, axis_color)
    _marker(
        world,
        "cylindrical_axis_arrow",
        dart.dynamics.ArrowShape(
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 2.8],
            dart.dynamics.ArrowShapeProperties(0.025, 2.2, 0.18, 0.08, 0.18),
            [*axis_color, 1.0],
        ),
        axis_color,
    )
    world.getConstraintSolver().addConstraint(
        dart.constraint.CylindricalJointConstraint(
            cylindrical, [0.0, 0.0, 1.15], [0.0, 0.0, 1.0]
        )
    )

    weld = _free_box(
        world,
        "weld_body",
        [3.0, 0.0, 1.15],
        [0.70, 0.32, 0.32],
        [0.40, 0.80, 0.35],
        [0.7, 0.0, 0.9, 0.6, 0.0, 0.4],
    )
    _marker(
        world,
        "weld_reference",
        dart.dynamics.SphereShape(0.08),
        [0.40, 0.80, 0.35],
        [3.0, 0.0, 1.15],
    )
    _line(
        world,
        "weld_reference_line",
        [2.55, 0.0, 1.15],
        [3.45, 0.0, 1.15],
        3.0,
        [0.40, 0.80, 0.35],
    )
    world.getConstraintSolver().addConstraint(dart.constraint.WeldJointConstraint(weld))

    panel = dart.viewer.Panel()
    panel.add_text("Ball: fixed anchor / free rotation", wrap=True)
    panel.add_text("Cylindrical: fixed axis / slide + spin", wrap=True)
    panel.add_text("Weld: fixed pose", wrap=True)
    panel.add_separator()
    constraint_text = panel.add_text("Constraints: 3")

    def pre_refresh():
        constraint_text.label = (
            f"Constraints: {world.getConstraintSolver().getNumConstraints()}"
        )

    def on_activate(ctx):
        labels = dart.gui.DebugOverlay()
        labels.setTextOutlineWidth(1.0)
        labels.setTextOutlineColor([1.0, 1.0, 1.0, 1.0])
        scale = ctx.viewer.getGuiScale()
        for position, text, color, size in (
            (
                [0.0, -1.25, 3.35],
                "DART 6 DynamicJointConstraint catalog",
                [0.08, 0.08, 0.08, 1.0],
                22.0,
            ),
            (
                [-3.0, -1.05, 2.55],
                "BALL\nfixed point\nfree rotation",
                [0.75, 0.12, 0.08, 1.0],
                18.0,
            ),
            (
                [0.0, -1.05, 2.75],
                "CYLINDRICAL\nrail fixed\nslide + spin",
                [0.0, 0.45, 0.70, 1.0],
                18.0,
            ),
            (
                [3.0, -1.05, 2.55],
                "WELD\npose fixed\nno free DOF",
                [0.12, 0.45, 0.12, 1.0],
                18.0,
            ),
        ):
            labels.addLabelPixels(position, text, color, size * scale)
        ctx.add_attachment(labels)

    return dart.viewer.SceneSetup(
        world,
        panel=panel,
        pre_refresh=pre_refresh,
        on_activate=on_activate,
        camera_home=dart.viewer.CameraHome(
            eye=[5.0, -7.0, 4.0], center=[0.0, 0.0, 0.9], up=[0.0, 0.0, 1.0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "dynamic_joint_constraints",
            title="Dynamic Joint Constraints",
            category="Constraints & Joints",
            summary=(
                "Ball, cylindrical, and weld dynamic joint constraints on three "
                "free-flying bodies."
            ),
            factory=build,
        )
    ]
