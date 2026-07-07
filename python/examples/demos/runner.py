"""CLI and keyboard-navigated viewer runtime for the dartpy consolidated
demos.

This is a plain ``dart.gui.osg.Viewer`` (no ImGui) driven by a single
``GUIEventHandler`` navigator instead of the C++ host's panel chrome because
dartpy has no Python-side ImGui panel API.

Modes:
  --list                 Print the catalog and exit.
  --cycle [--steps N]     Build + step every scene headlessly (no viewer, no
                          display needed) and exit nonzero on any failure.
  --scene ID --shot PATH [--steps N]
                          Build one scene, step it, capture a PNG, and exit.
                          Needs a real (or virtual, e.g. Xvfb) display.
  --scene ID (or nothing) Interactive mode: opens a window with the
                          keyboard navigator (n/p next/prev, 0-9 direct
                          select, r rebuild, h help, ESC quit).
"""

import argparse
import os
import sys
import time

import dartpy as dart

from .registry import make_demo_scenes

_DEFAULT_CYCLE_STEPS = 50
_DEFAULT_SHOT_STEPS = 60


def _print_catalog(scenes, *, active_index=-1):
    last_category = None
    for i, scene in enumerate(scenes):
        if scene.category != last_category:
            print(f"-- {scene.category} --")
            last_category = scene.category
        marker = "*" if i == active_index else " "
        print(f"{marker} [{i}] {scene.id:<28} {scene.title}")
        print(f"        {scene.summary}")


def _find_scene(scenes, scene_id):
    return next((s for s in scenes if s.id == scene_id), None)


def _install_scene(viewer, handle):
    """Wires a built SceneHandle into the given viewer. Returns state the
    caller must pass back to ``_uninstall_scene`` to tear it down.
    """

    viewer.addWorldNode(handle.node, True)

    if handle.shadow:
        shadow = dart.gui.osg.WorldNode.createDefaultShadowTechnique(viewer)
        handle.node.setShadowTechnique(shadow)

    grid = None
    if handle.grid:
        grid = dart.gui.osg.GridVisual()
        grid.setPlaneType(dart.gui.osg.GridVisual.PlaneType.ZX)
        grid.setOffset([0, -0.55, 0])
        viewer.addAttachment(grid)

    dnds = []
    for spec in handle.drag_and_drop:
        if isinstance(spec, tuple):
            obj, *extra = spec
            dnds.append(viewer.enableDragAndDrop(obj, *extra))
        else:
            dnds.append(viewer.enableDragAndDrop(spec))

    resume_simulation = viewer.isSimulating() and not handle.allow_simulation
    viewer.allowSimulation(handle.allow_simulation)

    for line in handle.instructions:
        viewer.addInstructionText(line)

    if handle.camera_home:
        eye, center, up = handle.camera_home
        viewer.setCameraHomePosition(eye, center, up)

    return {"grid": grid, "dnds": dnds, "resume_simulation": resume_simulation}


def _uninstall_scene(viewer, handle, state):
    for dnd in state["dnds"]:
        try:
            viewer.disableDragAndDrop(dnd)
        except Exception:
            pass
    if state["grid"] is not None:
        viewer.removeAttachment(state["grid"])
    viewer.removeWorldNode(handle.node)
    if not handle.allow_simulation:
        viewer.allowSimulation(True)
        if state["resume_simulation"]:
            viewer.simulate(True)


def _run_headless_steps(handle, world, steps):
    """Advances a scene deterministically, bypassing RealTimeWorldNode's
    wall-clock-gated stepping in refresh() (see dart/gui/osg/
    RealTimeWorldNode.cpp) so --cycle/--shot don't depend on real time.
    Physics is only stepped when the scene allows it (kinematic-only scenes
    like atlas_puppet set allow_simulation=False and animate purely through
    customPreRefresh).
    """

    node = handle.node
    step_physics = handle.allow_simulation
    for _ in range(steps):
        node.customPreRefresh()
        if step_physics:
            node.customPreStep()
            world.step()
            node.customPostStep()
        node.customPostRefresh()


def cmd_list(scenes):
    _print_catalog(scenes)
    return 0


def cmd_cycle(scenes, steps):
    ok_count = 0
    failed = []
    for scene in scenes:
        try:
            handle = scene.build()
            world = handle.node.getWorld()
            _run_headless_steps(handle, world, steps)
        except Exception as exc:  # noqa: BLE001 -- smoke test must never crash
            failed.append(scene.id)
            print(f"[FAIL] {scene.id}: {exc}", file=sys.stderr)
            continue
        ok_count += 1
        print(f"[OK]   {scene.id}")

    print(f"\n{ok_count}/{len(scenes)} scenes cycled cleanly ({steps} steps each).")
    if failed:
        print(f"Failed: {', '.join(failed)}", file=sys.stderr)
    return 0 if not failed else 1


def cmd_shot(scenes, scene_id, path, steps):
    if scene_id is not None:
        scene = _find_scene(scenes, scene_id)
        if scene is None:
            print(f"[demos] Unknown scene id '{scene_id}'.", file=sys.stderr)
            return 1
    else:
        scene = scenes[0]

    try:
        handle = scene.build()
    except Exception as exc:
        print(f"[demos] Failed to build demo '{scene.id}': {exc}", file=sys.stderr)
        return 1

    viewer = dart.gui.osg.Viewer()
    state = _install_scene(viewer, handle)
    viewer.setUpViewInWindow(0, 0, 640, 480)

    try:
        world = handle.node.getWorld()
        _run_headless_steps(handle, world, steps)
        # Two frames to settle the render state, then capture -- mirrors the
        # C++ demos host's headless-shot recipe (examples/demos/
        # DemoHost.cpp). The actual write happens inside SaveScreen's draw
        # callback (dart/gui/osg/Viewer.cpp), which may run on a separate
        # draw thread depending on the viewer's threading model. dartpy has
        # no setThreadingModel binding to force SingleThreaded, so keep
        # pumping frames for a short grace period until the file actually
        # shows up, rather than trust a single extra frame and risk a
        # silently-truncated/missing PNG.
        try:
            os.remove(path)
        except FileNotFoundError:
            pass
        except OSError as exc:
            print(
                f"[demos] Failed to remove stale screenshot '{path}': {exc}",
                file=sys.stderr,
            )
            return 1
        viewer.frame()
        viewer.frame()
        viewer.captureScreen(path)
        deadline = time.monotonic() + 2.0
        while not os.path.exists(path) and time.monotonic() < deadline:
            viewer.frame()
            time.sleep(0.02)
    except Exception as exc:
        print(f"[demos] Failed to capture demo '{scene.id}': {exc}", file=sys.stderr)
        return 1
    finally:
        _uninstall_scene(viewer, handle, state)

    if not os.path.exists(path):
        print(
            f"[demos] Screenshot for '{scene.id}' did not appear at {path} "
            "within the grace period.",
            file=sys.stderr,
        )
        return 1

    print(f"[demos] Wrote screenshot for '{scene.id}' to {path}")
    return 0


class Navigator(dart.gui.osg.GUIEventHandler):
    """Keyboard-driven scene switcher. Reserves n/p/h/r/0-9/ESC; forwards
    anything else to the active scene's key_actions and extra_handler (see
    registry.SceneHandle).
    """

    def __init__(self, viewer, scenes):
        super().__init__()
        self.viewer = viewer
        self.scenes = scenes
        self.index = -1
        self.running = True
        self._handle = None
        self._state = None

    def activate(self, index, *, rebuild=False):
        if not 0 <= index < len(self.scenes):
            return False
        scene = self.scenes[index]
        try:
            handle = scene.build()
        except Exception as exc:
            action = "rebuild" if rebuild else "start"
            print(
                f"[demos] Failed to {action} demo '{scene.id}': {exc}",
                file=sys.stderr,
            )
            return False

        if self._handle is not None:
            _uninstall_scene(self.viewer, self._handle, self._state)

        self._state = _install_scene(self.viewer, handle)
        self._handle = handle
        self.index = index

        print(
            f"\n[demos] Now showing '{scene.title}' ({scene.id}) -- "
            f"{scene.category}"
        )
        print(f"        {scene.summary}")
        if handle.notes:
            print(f"        Note: {handle.notes}")
        for line in handle.instructions:
            print(f"        {line}", end="")
        return True

    def select(self, delta):
        n = len(self.scenes)
        start = self.index if self.index >= 0 else 0
        for step in range(1, n + 1):
            candidate = (start + delta * step) % n
            if self.activate(candidate):
                return
        print("[demos] No other scene could be activated.", file=sys.stderr)

    def select_index(self, index):
        if not 0 <= index < len(self.scenes):
            print(f"[demos] No scene at index {index}.", file=sys.stderr)
            return
        if index != self.index:
            self.activate(index)

    def rebuild(self):
        if self.index >= 0:
            self.activate(self.index, rebuild=True)

    def print_help(self):
        _print_catalog(self.scenes, active_index=self.index)
        print(
            "\nKeys: n/p next/prev scene | 0-9 direct select | "
            "r rebuild | h help | ESC quit | space play/pause sim\n"
        )

    def handle(self, ea, aa):
        ea_cls = dart.gui.osg.GUIEventAdapter
        event_type = ea.getEventType()
        if event_type in (ea_cls.CLOSE_WINDOW, ea_cls.QUIT_APPLICATION):
            self.running = False
            return False
        if event_type == ea_cls.KEYDOWN:
            key = ea.getKey()
            if key in (ord("n"), ord("N")):
                self.select(1)
                return True
            if key in (ord("p"), ord("P")):
                self.select(-1)
                return True
            if key in (ord("h"), ord("H")):
                self.print_help()
                return True
            if key in (ord("r"), ord("R")):
                self.rebuild()
                return True
            if key == ea_cls.KEY_Escape:
                self.running = False
                return True
            if ord("0") <= key <= ord("9"):
                self.select_index(key - ord("0"))
                return True

            if self._handle is not None and key in self._handle.key_actions:
                label, callback = self._handle.key_actions[key]
                try:
                    callback()
                except Exception as exc:
                    print(
                        f"[demos] key action '{label}' failed: {exc}",
                        file=sys.stderr,
                    )
                return True

        if self._handle is not None and self._handle.extra_handler is not None:
            try:
                if self._handle.extra_handler.handle(ea, aa):
                    return True
            except Exception as exc:
                print(
                    f"[demos] scene input handler failed, disabling it: {exc}",
                    file=sys.stderr,
                )
                self._handle.extra_handler = None

        return False


def cmd_interactive(scenes, scene_id):
    viewer = dart.gui.osg.Viewer()
    navigator = Navigator(viewer, scenes)
    viewer.addEventHandler(navigator)

    initial_index = 0
    if scene_id is not None:
        found = next((i for i, s in enumerate(scenes) if s.id == scene_id), None)
        if found is None:
            print(
                f"[demos] Unknown scene id '{scene_id}', starting from the "
                "first scene.",
                file=sys.stderr,
            )
        else:
            initial_index = found

    if not navigator.activate(initial_index):
        started = any(navigator.activate(i) for i in range(len(scenes)))
        if not started:
            print("[demos] No scene in the catalog could be started.", file=sys.stderr)
            return 1

    navigator.print_help()
    viewer.setUpViewInWindow(0, 0, 1280, 960)

    while navigator.running:
        try:
            viewer.frame()
        except Exception as exc:
            print(f"[demos] Viewer loop stopped: {exc}", file=sys.stderr)
            break

    return 0


def build_arg_parser():
    parser = argparse.ArgumentParser(
        prog="python -m examples.demos",
        description="Consolidated dartpy demos runner.",
    )
    parser.add_argument(
        "--list", action="store_true", help="Print the demo catalog and exit."
    )
    parser.add_argument("--scene", metavar="ID", help="Scene id to show/capture.")
    parser.add_argument(
        "--shot", metavar="PATH", help="Capture a screenshot to PATH and exit."
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=None,
        help="Sim steps to advance for --shot/--cycle.",
    )
    parser.add_argument(
        "--cycle",
        action="store_true",
        help="Build and step every scene headlessly, then exit.",
    )
    return parser


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    scenes = make_demo_scenes()

    if args.list:
        return cmd_list(scenes)

    if args.cycle:
        steps = args.steps if args.steps is not None else _DEFAULT_CYCLE_STEPS
        return cmd_cycle(scenes, steps)

    if args.shot:
        steps = args.steps if args.steps is not None else _DEFAULT_SHOT_STEPS
        return cmd_shot(scenes, args.scene, args.shot, steps)

    return cmd_interactive(scenes, args.scene)


if __name__ == "__main__":
    sys.exit(main())
