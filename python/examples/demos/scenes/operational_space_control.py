"""Control & IK > operational_space_control.

Port of ``python/examples/operational_space_control/main.py``: a KR5 arm
chasing a fixed offset from its own end effector via Jacobian-pseudoinverse
task-space PD control.

Enhancement over the original: the target frame is now added to the world
and made draggable (``world.addSimpleFrame`` + ``viewer.enableDragAndDrop``
are both already-bound, already-used-elsewhere APIs -- no new bindings). The
original target was a bare ``SimpleFrame`` never added to the world, so it
was invisible and immovable; that didn't match the "chasing a draggable
target" behavior the C++ ``operational_space_control`` demo scene describes,
so this port gives the target a small visible marker shape and lets the user
drag it, which is what actually exercises the task-space controller
interactively.
"""

from __future__ import annotations

import dartpy as dart
import numpy as np


class OperationalSpaceController:
    def __init__(self, kr5, target):
        self.kr5 = kr5
        self.dofs = kr5.getNumDofs()
        self.ee = kr5.getBodyNode("palm")
        self.target = target
        self.Kp = np.eye(3) * 50.0
        self.Kd = np.eye(self.dofs) * 5.0

    def pre_step(self):
        m = self.kr5.getMassMatrix()

        j = self.ee.getLinearJacobian()
        jt = j.transpose()
        jjt = np.matmul(j, jt)
        k_i = 0.0025 * np.eye(3)
        inv_j = np.matmul(jt, np.linalg.inv(jjt + k_i))

        dj = self.ee.getLinearJacobianDeriv()
        djt = dj.transpose()
        djdjt = np.matmul(dj, djt)
        inv_dj = np.matmul(djt, np.linalg.inv(djdjt + k_i))

        e = (
            self.target.getTransform().translation()
            - self.ee.getTransform().translation()
        )
        de = -self.ee.getLinearVelocity()

        cg = self.kr5.getCoriolisAndGravityForces()

        term1 = np.matmul(np.matmul(inv_j, self.Kp), de)
        term2 = np.matmul(np.matmul(inv_dj, self.Kp), e)
        forces1 = np.matmul(m, term1 + term2)
        forces2 = cg
        forces3 = np.matmul(np.matmul(np.matmul(self.Kd, inv_j), self.Kp), e)

        self.kr5.setForces(forces1 + forces2 + forces3)


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()

    urdf_parser = dart.utils.DartLoader()
    kr5 = urdf_parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    ground = urdf_parser.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    world.addSkeleton(kr5)
    world.addSkeleton(ground)
    world.setGravity([0, -9.81, 0])

    ee = kr5.getBodyNode("palm")
    tf = ee.getTransform()
    tf.pretranslate([0.05, 0, 0])
    target = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "target", tf)
    target.setShape(dart.dynamics.SphereShape(0.03))
    target.createVisualAspect().setColor([0.9, 0.1, 0.1])
    world.addSimpleFrame(target)

    controller = OperationalSpaceController(kr5, target)

    def on_activate(ctx):
        grid = dart.gui.GridVisual()
        grid.setPlaneType(dart.gui.GridVisual.PlaneType.ZX)
        grid.setOffset([0, -0.55, 0])
        ctx.add_attachment(grid)
        ctx.enable_drag_and_drop(target)

    panel = dart.viewer.Panel()
    panel.add_text(
        "Target frame made visible + draggable (was an invisible, fixed "
        "SimpleFrame in the original standalone example).",
        wrap=True,
    )

    return dart.viewer.SceneSetup(
        world,
        pre_step=controller.pre_step,
        on_activate=on_activate,
        camera_home=dart.viewer.CameraHome(
            eye=[2.0, 1.0, 2.0], center=[0.0, 0.0, 0.0], up=[-0.24, 0.94, -0.25]
        ),
        shadows=False,
        panel=panel,
    )


def scenes():
    return [
        dart.viewer.Scene(
            "operational_space_control",
            title="Operational Space Control",
            category="Control & IK",
            summary="KR5 arm chasing a draggable target with task-space control.",
            factory=build,
        )
    ]
