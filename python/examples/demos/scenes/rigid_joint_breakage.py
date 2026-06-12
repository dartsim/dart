"""World rigid-body workflow row for AVBD break-force lifecycle debugging."""

from __future__ import annotations

from ..runner import PythonDemoScene, SceneSetup
from .avbd_rigid_breakable_joint import build_breakable_joint_scene


def build() -> SceneSetup:
    return build_breakable_joint_scene(
        panel_title="Rigid Joint Breakage",
        row_id="rigid_joint_breakage",
        related_source_row=None,
    )


SCENE = PythonDemoScene(
    id="rigid_joint_breakage",
    title="Rigid Joint Breakage",
    category="World Rigid Body",
    summary=(
        "AVBD-pinned fixed-joint break-force threshold, broken state, and reset "
        "lifecycle."
    ),
    build=build,
)
