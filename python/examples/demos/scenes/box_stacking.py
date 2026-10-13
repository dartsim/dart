"""Five free-falling boxes with Dantzig and PGS constraint solvers."""

import dartpy as dart


def _box(name, position, size, color, *, fixed=False):
    skeleton = dart.dynamics.Skeleton(name)
    if fixed:
        joint, body = skeleton.createWeldJointAndBodyNodePair()
    else:
        joint, body = skeleton.createFreeJointAndBodyNodePair()
    node = body.createShapeNode(dart.dynamics.BoxShape(size))
    node.createVisualAspect()
    node.createCollisionAspect()
    node.createDynamicsAspect()
    node.getVisualAspect().setColor(color)
    transform = dart.math.Isometry3()
    transform.set_translation(position)
    joint.setTransformFromParentBodyNode(transform)
    return skeleton


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    world.addSkeleton(
        _box(
            "floor",
            [0.0, 0.0, -0.005],
            [10.0, 10.0, 0.01],
            [0.9, 0.9, 0.9],
            fixed=True,
        )
    )
    for i in range(5):
        world.addSkeleton(
            _box(
                "box",
                [0.0, 0.0, 0.75 + i * 0.5],
                [1.0, 1.0, 0.5],
                [dart.math.Random.uniform(0.0, 1.0) for _ in range(3)],
            )
        )
    solver_kind = "dantzig"
    dart._demos.set_boxed_lcp_solver(world, solver_kind)

    panel = dart.viewer.Panel()
    time_text = panel.add_text("Time: 0.000")
    solver = panel.add_choice("LCP solver:", ["Dantzig", "PGS"])

    def change_solver():
        nonlocal solver_kind
        new_kind = "dantzig" if solver.value == 0 else "pgs"
        if new_kind != solver_kind:
            solver_kind = new_kind
            dart._demos.set_boxed_lcp_solver(world, solver_kind)

    solver.on_change = change_solver
    panel.add_text(
        "5 boxes free-fall onto a floor plane; switch the boxed-LCP "
        "solver to compare stacking stability.",
        wrap=True,
    )

    def pre_refresh():
        time_text.label = f"Time: {world.getTime():.3f}"

    return dart.viewer.SceneSetup(
        world,
        panel=panel,
        pre_refresh=pre_refresh,
        camera_home=dart.viewer.CameraHome(
            eye=[12.0, 12.0, 9.0], center=[0.0, 0.0, 2.0], up=[0.0, 0.0, 1.0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "box_stacking",
            title="Box Stacking",
            category="Constraints & Joints",
            summary="Stacked boxes comparing Dantzig and PGS LCP solvers.",
            factory=build,
        )
    ]
