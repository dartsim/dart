"""Rigid Body > sleeping: four box stacks and reusable wake-up projectiles."""

import colorsys
import math

import dartpy as dart
import numpy as np

_STACK_X = (-2.0, -0.7, 0.7, 2.0)
_SHOT_COLOR = (1.0, 0.84, 0.08, 1.0)
_DROP_COLOR = (0.0, 0.96, 1.0, 1.0)
_ISLAND_COLORS = (
    (0.20, 0.62, 1.00),
    (1.00, 0.48, 0.18),
    (0.16, 0.78, 0.50),
    (0.95, 0.30, 0.58),
    (1.00, 0.78, 0.18),
    (0.66, 0.42, 1.00),
    (0.00, 0.84, 0.86),
)


def _shape_node(body, shape):
    node = body.createShapeNode(shape)
    node.createVisualAspect()
    node.createCollisionAspect()
    node.createDynamicsAspect()
    return node


def _transform(position):
    transform = dart.math.Isometry3()
    transform.set_translation(position)
    return transform


def _free_body(name, shape, position):
    skeleton = dart.dynamics.Skeleton(name)
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    _shape_node(body, shape)
    mass = 1000.0 * shape.getVolume()
    inertia = dart.dynamics.Inertia()
    inertia.setMass(mass)
    inertia.setMoment(shape.computeInertia(mass))
    body.setInertia(inertia)
    joint.setPositions(dart.dynamics.FreeJoint.convertToPositions(_transform(position)))
    return skeleton


def _set_color(skeleton, color):
    for bi in range(skeleton.getNumBodyNodes()):
        body = skeleton.getBodyNode(bi)
        for si in range(body.getNumShapeNodes()):
            visual = body.getShapeNode(si).getVisualAspect()
            if visual is not None:
                visual.setRGBA(color)


def _set_projectile(skeleton, position, velocity, active):
    joint = skeleton.getJoint(0)
    joint.setPositions(dart.dynamics.FreeJoint.convertToPositions(_transform(position)))
    joint.setVelocities([0.0, 0.0, 0.0, *velocity])
    skeleton.getBodyNode(0).setCollidable(active)
    skeleton.setResting(not active)
    color = _SHOT_COLOR if skeleton.getName().startswith("proj_shot") else _DROP_COLOR
    _set_color(skeleton, color if active else (0.05, 0.07, 0.10, 1.0))


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    world.setTimeStep(1.0 / 60.0)
    world.setGravity([0.0, 0.0, -9.81])
    floor = dart.dynamics.Skeleton("floor")
    joint, body = floor.createWeldJointAndBodyNodePair()
    node = _shape_node(body, dart.dynamics.BoxShape([20.0, 20.0, 0.1]))
    node.getVisualAspect().setColor([0.40, 0.43, 0.47])
    joint.setTransformFromParentBodyNode(_transform([0.0, 0.0, -0.05]))
    floor.setMobile(False)
    world.addSkeleton(floor)

    options = world.getDeactivationOptions()
    options.mTimeUntilSleep = 0.3
    world.setDeactivationOptions(options)
    for stack, x in enumerate(_STACK_X):
        for level in range(4):
            world.addSkeleton(
                _free_body(
                    f"box{4 * stack + level}",
                    dart.dynamics.BoxShape([0.3, 0.3, 0.3]),
                    [x, 0.0, 0.15 + 0.01 + level * (0.3 + 1e-3)],
                )
            )

    for name, shape, position in (
        ("proj_shot0", dart.dynamics.EllipsoidShape([1.10] * 3), [60.0, 60.0, 10.0]),
        ("proj_drop0", dart.dynamics.BoxShape([0.62] * 3), [60.0, 61.0, 10.0]),
    ):
        projectile = _free_body(name, shape, position)
        projectile.getBodyNode(0).getShapeNode(0).getVisualAspect().setShadowed(False)
        _set_projectile(projectile, position, [0.0, 0.0, 0.0], False)
        world.addSkeleton(projectile)

    aim_frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "aim")
    aim_line = dart.dynamics.LineSegmentShape(3.0)
    for _ in range(32):
        aim_line.addVertex([0.0, 0.0, 0.0])
    for index in range(31):
        aim_line.addConnection(index, index + 1)
    aim_line.addDataVariance(dart.dynamics.Shape.DYNAMIC_VERTICES)
    aim_frame.setShape(aim_line)
    aim_frame.getVisualAspect(True).setColor([1.0, 0.92, 0.20])
    world.addSimpleFrame(aim_frame)

    target = 0
    color_by_island = False
    last_display_island = {}

    def sphere_launch():
        return np.array([_STACK_X[target % 4], 3.0, 2.0]), np.array([0.0, -4.2, 1.7])

    def launch(sphere):
        nonlocal target
        _set_projectile(
            world.getSkeleton("proj_shot0"), [60.0, 60.0, 10.0], [0.0] * 3, False
        )
        _set_projectile(
            world.getSkeleton("proj_drop0"), [60.0, 61.0, 10.0], [0.0] * 3, False
        )
        if sphere:
            position, velocity = sphere_launch()
            name = "proj_shot0"
        else:
            x = _STACK_X[target % 4]
            spawn_x = min(x - 0.45, -3.6)
            position = [spawn_x, 1.2, 2.7]
            velocity = [(x - spawn_x) / 0.65, -1.2 / 0.65, 0.0]
            name = "proj_drop0"
        _set_projectile(world.getSkeleton(name), position, velocity, True)
        target += 1

    def toggle_sleeping():
        options = world.getDeactivationOptions()
        options.mEnabled = not options.mEnabled
        world.setDeactivationOptions(options)

    def toggle_island_colors():
        nonlocal color_by_island
        color_by_island = not color_by_island

    panel = dart.viewer.Panel()
    stacks_text = panel.add_text("Stacks  16 awake / 0 asleep")
    projectiles_text = panel.add_text("Active  0 sphere(s) / 0 box(es)")
    panel.add_swatch([0.48, 0.76, 1.0, 1.0])
    panel.add_text("awake").same_line = True
    panel.add_swatch([0.16, 0.22, 0.30, 1.0]).same_line = True
    panel.add_text("asleep").same_line = True
    sleeping_control = panel.add_checkbox(
        "Automatic deactivation (sleeping)", options.mEnabled
    )
    island_control = panel.add_checkbox("Color by solver island", color_by_island)
    dwell_control = panel.add_slider("Dwell time", 0.3, min=0.05, max=2.0, decimals=3)
    panel.add_text("s").same_line = True
    panel.add_text(
        "The yellow arc previews the next sphere shot. Awake objects are "
        "bright; sleeping objects use a cooler muted tint.",
        wrap=True,
    )

    def set_sleeping():
        options = world.getDeactivationOptions()
        options.mEnabled = sleeping_control.value
        world.setDeactivationOptions(options)

    def set_island_colors():
        nonlocal color_by_island
        color_by_island = island_control.value

    def set_dwell():
        options = world.getDeactivationOptions()
        options.mTimeUntilSleep = float(np.float32(dwell_control.value))
        world.setDeactivationOptions(options)

    sleeping_control.on_change = set_sleeping
    island_control.on_change = set_island_colors
    dwell_control.on_change = set_dwell

    def pre_refresh():
        awake = asleep = shots = drops = 0
        for index in range(world.getNumSkeletons()):
            skeleton = world.getSkeleton(index)
            if not skeleton.isMobile():
                continue
            name = skeleton.getName()
            shot = name.startswith("proj_shot")
            dropped = name.startswith("proj_drop")
            projectile = shot or dropped
            active = projectile and skeleton.getBodyNode(0).isCollidable()
            if not projectile:
                asleep += skeleton.isResting()
                awake += not skeleton.isResting()
            elif active:
                shots += shot
                drops += dropped

            base = (
                _SHOT_COLOR[:3]
                if shot
                else (
                    _DROP_COLOR[:3]
                    if dropped
                    else colorsys.hsv_to_rgb((0.58 + 0.173 * index) % 1.0, 0.54, 0.92)
                )
            )
            if color_by_island and not projectile:
                solver_island = skeleton.getIslandIndex()
                if not skeleton.isResting() and solver_island >= 0:
                    island = solver_island
                    last_display_island[name] = island
                elif name.startswith("box") and name[3:].isdigit():
                    island = int(name[3:]) // 4
                else:
                    island = last_display_island.get(name, -1)
                if island >= 0:
                    base = _ISLAND_COLORS[island % len(_ISLAND_COLORS)]
            if not active and skeleton.isResting():
                color = np.clip(0.72 * np.array(base) + [0.015, 0.020, 0.030], 0.0, 1.0)
            else:
                color = np.clip(
                    0.84 * np.array(base) + 0.18 * np.array([0.98, 0.99, 1.0]), 0.0, 1.0
                )
            _set_color(skeleton, [*color, 1.0])

        position, velocity = sphere_launch()
        gravity = world.getGravity()
        t_end = 1.0
        a, b, c = 0.5 * gravity[2], velocity[2], position[2]
        if a < 0.0:
            discriminant = b * b - 4.0 * a * c
            if discriminant >= 0.0:
                t_end = (-b - math.sqrt(discriminant)) / (2.0 * a)
        if not t_end > 0.0:
            t_end = 1.0
        for index in range(32):
            t = t_end * index / 31
            point = position + velocity * t + 0.5 * gravity * (t * t)
            point[2] = max(point[2], 0.0)
            aim_line.setVertex(index, point)

        stacks_text.label = f"Stacks  {awake} awake / {asleep} asleep"
        projectiles_text.label = f"Active  {shots} sphere(s) / {drops} box(es)"
        options = world.getDeactivationOptions()
        sleeping_control.value = options.mEnabled
        island_control.value = color_by_island
        dwell_control.value = options.mTimeUntilSleep

    pre_refresh()
    return dart.viewer.SceneSetup(
        world,
        pre_refresh=pre_refresh,
        key_actions=[
            dart.viewer.KeyAction("f", "Shoot sphere", lambda: launch(True)),
            dart.viewer.KeyAction("d", "Drop box", lambda: launch(False)),
            dart.viewer.KeyAction("t", "Toggle sleeping", toggle_sleeping),
            dart.viewer.KeyAction("c", "Toggle island colors", toggle_island_colors),
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(eye=[6.0, 8.0, 4.0], center=[0.0, 0.0, 1.0]),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "sleeping",
            title="Sleeping",
            category="Rigid Body",
            summary="Automatic body deactivation across four independent box-stack solver islands.",
            factory=build,
        )
    ]
