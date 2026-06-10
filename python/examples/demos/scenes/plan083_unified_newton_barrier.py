"""PLAN-083 CPU corpus placeholders for unified Newton-barrier scenes."""

from __future__ import annotations

from dataclasses import dataclass

import dartpy as dart
import numpy as np

from ..runner import PythonDemoScene, ScenePanel, SceneSetup


@dataclass(frozen=True)
class Plan083SceneTarget:
    scene_id: str
    title: str
    row_ids: tuple[str, ...]
    category: str
    summary: str
    target: str
    smoke_command: str
    visual_command: str
    benchmark_command: str
    limitation: str


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _build_placeholder(target: Plan083SceneTarget) -> SceneSetup:
    world = dart.gui.DescriptorRenderScene(dart.World(), target.scene_id)
    world.set_time_step(1.0 / 60.0)

    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        f"{target.scene_id}_marker",
        _translation(0.0, 0.0, 0.1),
    )
    frame.set_shape(dart.BoxShape(np.array([0.68, 0.44, 0.2])))
    frame.create_visual_aspect().set_color([0.22, 0.45, 0.65])
    world.add_simple_frame(frame)

    row_ids = ", ".join(target.row_ids)

    def build_panel(builder: object, context: object) -> None:
        builder.text("status: planned PLAN-083 CPU corpus scene")
        builder.text(f"rows: {row_ids}")
        builder.separator()
        builder.text(f"target: {target.target}")
        builder.separator()
        builder.text(f"smoke: {target.smoke_command}")
        builder.text(f"visual: {target.visual_command}")
        builder.text(f"benchmark: {target.benchmark_command}")
        builder.separator()
        builder.text(f"limitation: {target.limitation}")

    return SceneSetup(
        world=world,
        panels=[ScenePanel(target.title, build_panel)],
        info={
            "plan083_cpu_corpus_scene": target.scene_id,
            "plan083_row_ids": target.row_ids,
            "plan083_smoke_command": target.smoke_command,
            "plan083_visual_command": target.visual_command,
            "plan083_benchmark_command": target.benchmark_command,
            "plan083_limitation": target.limitation,
        },
    )


def _scene(target: Plan083SceneTarget) -> PythonDemoScene:
    return PythonDemoScene(
        id=target.scene_id,
        title=target.title,
        category=target.category,
        summary=target.summary,
        build=lambda target=target: _build_placeholder(target),
    )


PLAN083_SCENE_TARGETS: tuple[Plan083SceneTarget, ...] = (
    Plan083SceneTarget(
        scene_id="plan083_lying_flat",
        title="PLAN-083 Lying Flat",
        row_ids=("unb-fig-01",),
        category="PLAN-083 Mixed Corpus",
        summary="Planned mixed rigid/deformable/rod/cloth/ragdoll corpus scene.",
        target="Paper Fig. 1 / Table 2 mixed-domain stress scene.",
        smoke_command="pixi run py-demos -- --scene plan083_lying_flat --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_lying_flat --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=lying_flat",
        limitation="Waiting for scene-level unified mixed-domain stepping.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_hanging_bridge",
        title="PLAN-083 Hanging Bridge",
        row_ids=("unb-fig-02",),
        category="PLAN-083 Mixed Corpus",
        summary="Planned bridge with rods, boards, point connections, and coupling.",
        target="Paper Fig. 2 / Table 2 hanging bridge scene.",
        smoke_command="pixi run py-demos -- --scene plan083_hanging_bridge --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_hanging_bridge --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=hanging_bridge",
        limitation="Waiting for rod/rigid/codimensional runtime coupling.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_pulley_system",
        title="PLAN-083 Pulley System",
        row_ids=("unb-fig-03",),
        category="PLAN-083 Constraints Corpus",
        summary="Planned pulley constraints and frictional lifting validation scene.",
        target="Paper Fig. 3/Fig. 21 pulley force-comparison scene.",
        smoke_command="pixi run py-demos -- --scene plan083_pulley_system --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_pulley_system --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=pulley_system",
        limitation="Waiting for runtime equality-constraint solve and force packet.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_umbrella",
        title="PLAN-083 Umbrella",
        row_ids=("unb-fig-04",),
        category="PLAN-083 Mixed Corpus",
        summary="Planned umbrella cloth/rod/hinge/sliding mixed scene.",
        target="Paper Fig. 4 umbrella visual-evidence scene.",
        smoke_command="pixi run py-demos -- --scene plan083_umbrella --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_umbrella --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=umbrella",
        limitation="Waiting for cloth/rod coupling and runtime sliding constraints.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_terrain_vehicle",
        title="PLAN-083 Terrain Vehicle",
        row_ids=("unb-fig-10",),
        category="PLAN-083 Robot Corpus",
        summary="Planned terrain vehicle with hinges, passive wheels, and friction.",
        target="Paper Fig. 10 / Table 2 terrain navigation scene.",
        smoke_command="pixi run py-demos -- --scene plan083_terrain_vehicle --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_terrain_vehicle --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=terrain_vehicle",
        limitation="Waiting for articulated rigid IPC scene assembly.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_ragdolls",
        title="PLAN-083 Ragdolls",
        row_ids=("unb-fig-11",),
        category="PLAN-083 Robot Corpus",
        summary="Planned many-ragdoll cone-twist stress scene.",
        target="Paper Fig. 11 / Table 2 60-ragdoll benchmark scene.",
        smoke_command="pixi run py-demos -- --scene plan083_ragdolls --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_ragdolls --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=ragdolls",
        limitation="Waiting for runtime cone-twist constraints and corpus assets.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_nunchaku",
        title="PLAN-083 Nunchaku",
        row_ids=("unb-fig-13", "unb-fig-25"),
        category="PLAN-083 Constraints Corpus",
        summary="Planned cone-twist range demo plus scalability packet seed.",
        target="Paper Fig. 13 and Fig. 25 nunchaku constraints/scaling rows.",
        smoke_command="pixi run py-demos -- --scene plan083_nunchaku --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_nunchaku --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=nunchaku",
        limitation="Waiting for sparse constraint solve and N-by-N scaling packet.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_windmill",
        title="PLAN-083 Windmill",
        row_ids=("unb-fig-20",),
        category="PLAN-083 Constraints Corpus",
        summary="Planned windmill comparison against a Bullet/reference baseline.",
        target="Paper Fig. 20 hinge/contact comparison scene.",
        smoke_command="pixi run py-demos -- --scene plan083_windmill --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_windmill --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=windmill",
        limitation="Waiting for Bullet/reference comparison packet approval.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_candy",
        title="PLAN-083 Candy",
        row_ids=("unb-fig-22",),
        category="PLAN-083 Mixed Corpus",
        summary="Planned affine/deformable/cloth coupling scene.",
        target="Paper Fig. 22 / Table 2 candy mixed-domain scene.",
        smoke_command="pixi run py-demos -- --scene plan083_candy --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_candy --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=candy",
        limitation="Waiting for ABD runtime stepping and cloth self-contact parity.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_precession",
        title="PLAN-083 Precession",
        row_ids=("unb-fig-23",),
        category="PLAN-083 Robot Corpus",
        summary="Planned rolling unicycle precession scene.",
        target="Paper Fig. 23 / Table 2 precession benchmark scene.",
        smoke_command="pixi run py-demos -- --scene plan083_precession --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_precession --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=precession",
        limitation="Waiting for rolling-contact runtime scene and matched angular-velocity sweep.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_abd_complex_geometry",
        title="PLAN-083 ABD Complex Geometry",
        row_ids=("abd-complex-geometry",),
        category="PLAN-083 ABD Corpus",
        summary="Planned ABD complex-geometry benchmark and py-demo.",
        target="ABD deck complex-geometry contact scene.",
        smoke_command="pixi run py-demos -- --scene plan083_abd_complex_geometry --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_abd_complex_geometry --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=abd_complex_geometry",
        limitation="Waiting for ABD runtime stepping beyond primitive/oracle rows.",
    ),
    Plan083SceneTarget(
        scene_id="plan083_abd_fem_coupling",
        title="PLAN-083 ABD FEM Coupling",
        row_ids=("abd-fem-coupling",),
        category="PLAN-083 ABD Corpus",
        summary="Planned ABD plus FEM deformable coupling scene.",
        target="ABD deck FEM-coupling benchmark and py-demo.",
        smoke_command="pixi run py-demos -- --scene plan083_abd_fem_coupling --headless --frames 4",
        visual_command="pixi run py-demo-capture -- --scene plan083_abd_fem_coupling --frames 240 --width 1280 --height 720",
        benchmark_command="pixi run bm bm_plan083_cpu_scene_corpus -- --benchmark_filter=abd_fem_coupling",
        limitation="Waiting for ABD/FEM mixed runtime coupling.",
    ),
)


_CATEGORY_ORDER = {
    "PLAN-083 Mixed Corpus": 0,
    "PLAN-083 Constraints Corpus": 1,
    "PLAN-083 Robot Corpus": 2,
    "PLAN-083 ABD Corpus": 3,
}


PLAN083_SCENES: tuple[PythonDemoScene, ...] = tuple(
    _scene(target)
    for target in sorted(
        PLAN083_SCENE_TARGETS,
        key=lambda target: (_CATEGORY_ORDER[target.category], target.scene_id),
    )
)
