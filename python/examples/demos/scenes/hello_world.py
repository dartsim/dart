"""Hello-world scene: a KR5 arm + ground loaded from URDF (legacy World)."""

from __future__ import annotations

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


def build() -> SceneSetup:
    world = dart.World()
    urdf = dart.io.UrdfParser()
    kr5 = urdf.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    ground = urdf.parseSkeleton("dart://sample/urdf/KR5/ground.urdf")
    world.addSkeleton(kr5)
    world.addSkeleton(ground)
    return SceneSetup(world=world, info={"robot": kr5.getName()})


SCENE = PythonDemoScene(
    id="hello_world",
    title="Hello World",
    category="Getting Started",
    summary="A KR5 arm and ground loaded from URDF on the legacy World.",
    build=build,
)
