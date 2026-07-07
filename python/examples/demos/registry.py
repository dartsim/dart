"""Scene catalog for the dartpy consolidated demos runner.

This mirrors the C++ ``examples/demos`` catalog's id/category/title scheme
where a Python port of the same demo exists (see
``docs/dev_tasks/dart6_consolidated_demos/PLAN.md`` for the frozen taxonomy).
A few scenes here have no C++ counterpart (``hello_world_gui``,
``contacts_pointcloud``); those get their own id/category/summary.

``SceneHandle`` is the Python analog of the C++ host's ``DemoSceneSetup``
(see ``examples/demos/DemoScene.hpp``), scaled down to what a plain
``dart.gui.osg.Viewer`` can actually do without ImGui (no per-scene panel,
no host-owned teardown registry -- the runner just re-derives everything a
scene needs from the fields below each time it switches).
"""

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple


@dataclass
class SceneHandle:
    """Everything a scene factory builds. The runner owns and drives all of
    this; the scene only supplies data, the world/node, and callbacks.
    """

    # The constructed world node (usually a RealTimeWorldNode subclass with
    # custom pre/post-step or pre/post-refresh hooks). Required.
    node: object

    # Camera pose to apply when this scene becomes active: (eye, center, up),
    # each a 3-element sequence. Optional.
    camera_home: Optional[Tuple[list, list, list]] = None

    # Whether the runner should attach a ZX GridVisual and the default shadow
    # technique. Most rigid-body scenes want both; kinematic/frame-only
    # scenes (drag_and_drop, atlas_puppet) typically want neither.
    grid: bool = False
    shadow: bool = False

    # Whether the viewer should allow physical simulation at all. Purely
    # kinematic scenes (atlas_puppet) set this False, matching the original
    # standalone example's ``viewer.allowSimulation(False)``.
    allow_simulation: bool = True

    # Objects to enable drag-and-drop on. Each entry is either a bare
    # SimpleFrame/InteractiveFrame (uses the single-argument
    # ``enableDragAndDrop`` overload) or a ``(BodyNode, useExternalIK,
    # useWholeBody)`` tuple (uses the three-argument BodyNode overload).
    drag_and_drop: List[object] = field(default_factory=list)

    # Simple, stateless keyboard shortcuts: {key_code: (label, callback)}.
    # Invoked once per KEYDOWN. Use ``extra_handler`` instead for anything
    # that needs KEYUP (held keys) or multi-key state.
    key_actions: Dict[int, Tuple[str, Callable[[], None]]] = field(default_factory=dict)

    # An optional object with a ``handle(ea, aa) -> bool`` method (typically
    # a ``dart.gui.osg.GUIEventHandler`` subclass instance, called directly
    # as a plain Python method rather than registered with the OSG event
    # system -- dartpy has no ``Viewer.removeEventHandler`` binding, so the
    # runner instead just forwards events to whichever scene is active and
    # drops the reference on switch).
    extra_handler: Optional[object] = None

    # Extra instruction lines shown via ``viewer.addInstructionText`` and
    # printed to stdout when this scene becomes active.
    instructions: List[str] = field(default_factory=list)

    # Short note about parity with the original standalone example or the
    # C++ demos scene (bindings dropped, keys remapped, etc). Printed to
    # stdout on activation and by ``--list -v``.
    notes: str = ""


@dataclass(frozen=True)
class PyDemoScene:
    """One entry in the demo catalog. ``build`` is only invoked lazily -- the
    first time the demo is selected (and again on rebuild) -- and may raise
    to signal a startup failure; the runner surfaces the reason and keeps
    whatever was running before.
    """

    id: str
    title: str
    category: str
    summary: str
    build: Callable[[], SceneHandle]


def make_demo_scenes() -> List[PyDemoScene]:
    """Returns the catalog in navigator order (categories group by
    first-appearance, matching the C++ host's convention).
    """

    from .scenes import (
        atlas_puppet,
        biped_stand,
        contacts_pointcloud,
        drag_and_drop,
        hello_world_gui,
        operational_space_control,
        rigid_chain,
        rigid_cubes,
        rigid_loop,
    )

    return [
        PyDemoScene(
            id="hello_world_gui",
            title="Hello World (GUI)",
            category="Getting Started",
            summary=(
                "KR5 arm and ground plane with no active controller -- the "
                "starting point for dartpy GUI apps."
            ),
            build=hello_world_gui.build,
        ),
        PyDemoScene(
            id="rigid_cubes",
            title="Rigid Cubes",
            category="Rigid Body",
            summary=(
                "A stack of cubes loaded from a skeleton file falls onto the "
                "ground plane under gravity, rendered with the default "
                "shadow technique."
            ),
            build=rigid_cubes.build,
        ),
        PyDemoScene(
            id="rigid_chain",
            title="Rigid Chain",
            category="Rigid Body",
            summary=(
                "A damped articulated chain loaded from a skeleton file with "
                "a randomized initial pose."
            ),
            build=rigid_chain.build,
        ),
        PyDemoScene(
            id="rigid_loop",
            title="Rigid Loop",
            category="Constraints & Joints",
            summary=(
                "A chain closed into a loop by a ball-joint constraint "
                "linking two non-adjacent links."
            ),
            build=rigid_loop.build,
        ),
        PyDemoScene(
            id="drag_and_drop",
            title="Drag and Drop",
            category="Visualization",
            summary=(
                "An interactive gizmo frame carrying a draggable child box, "
                "plus world-axis markers."
            ),
            build=drag_and_drop.build,
        ),
        PyDemoScene(
            id="contacts_pointcloud",
            title="Contacts Point Cloud",
            category="Visualization",
            summary=(
                "Live contact points between a transparent KR5 arm and the "
                "ground, rendered as a red point cloud."
            ),
            build=contacts_pointcloud.build,
        ),
        PyDemoScene(
            id="biped_stand",
            title="Biped Stand",
            category="Control & IK",
            summary=(
                "SPD-balanced standing biped with an ankle strategy and "
                "keyboard push tests (arrow keys)."
            ),
            build=biped_stand.build,
        ),
        PyDemoScene(
            id="operational_space_control",
            title="Operational Space Control",
            category="Control & IK",
            summary=(
                "KR5 arm chasing a draggable end-effector target with "
                "task-space (operational-space) PD control."
            ),
            build=operational_space_control.build,
        ),
        PyDemoScene(
            id="atlas_puppet",
            title="Atlas Puppet",
            category="Control & IK",
            summary=(
                "Purely kinematic Atlas whole-body IK puppet with "
                "draggable hand/foot targets and posture regularization."
            ),
            build=atlas_puppet.build,
        ),
    ]
