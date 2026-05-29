"""KR5 arm scene: load the KR5 industrial arm + a ground via URDF.

Preserves the content of the legacy ``python/examples/hello_world`` example
that loaded the KR5 model from the bundled sample data. Python-only (no C++
demos counterpart).
"""

from __future__ import annotations

import dartpy as dart

from ..runner import PythonDemoScene, SceneSetup


def build() -> SceneSetup:
    world = dart.World("kr5_arm")
    urdf = dart.io.UrdfParser()
    kr5 = urdf.parse_skeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
    ground = urdf.parse_skeleton("dart://sample/urdf/KR5/ground.urdf")
    world.add_skeleton(kr5)
    world.add_skeleton(ground)
    return SceneSetup(world=world, info={"robot": kr5.get_name()})


SCENE = PythonDemoScene(
    id="kr5_arm",
    title="KR5 Arm",
    category="Robots",
    summary="A KR5 industrial arm and ground loaded from URDF.",
    build=build,
)
