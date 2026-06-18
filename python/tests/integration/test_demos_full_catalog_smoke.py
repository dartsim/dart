"""Full-catalog in-process smoke for registered Python demo scenes.

Builds every registered scene, steps two frames through the shared headless
runner contract, and exercises render/debug providers. The subprocess smoke in
``scripts/py_demos_smoke.py`` remains the crash-isolated catalog guard; this
pytest guard keeps the same contract visible in CI.
"""

from __future__ import annotations

import pathlib
import re
import sys

# Put python/ on sys.path so the demos package is importable.
_PYTHON_DIR = pathlib.Path(__file__).resolve().parents[2]
if str(_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_DIR))

import pytest


def _simulation_has(*names: str) -> bool:
    try:
        import dartpy as sx
    except Exception:  # pragma: no cover - dartpy import failure
        return False
    return all(hasattr(sx, name) for name in names)


def test_all_registered_scenes_build_step_and_render() -> None:
    if not _simulation_has("World"):
        pytest.skip("dartpy.World unavailable in this build")

    import dartpy as dart
    from examples.demos import runner
    from examples.demos._smoke_support import exercise_panels
    from examples.demos.registry import make_demo_scenes

    configure_gpu_compute = getattr(runner, "_configure_gpu_compute", None)
    if callable(configure_gpu_compute):
        configure_gpu_compute(dart, None)

    failures: list[tuple[str, str]] = []
    for scene in make_demo_scenes():
        try:
            setup = scene.build()
            runner._step(setup, 2)

            provider = setup.renderable_provider
            if provider is None and setup.world is not None:
                provider = getattr(setup.world, "renderable_provider", None)
            if callable(provider):
                provider()
            if callable(setup.debug_provider):
                setup.debug_provider()

            exercise_panels(setup)
        except Exception as exc:
            failures.append((scene.id, repr(exc)))

    details = "\n".join(f"{scene_id}: {error}" for scene_id, error in failures)
    assert not failures, f"demo scene smoke failures:\n{details}"


def test_every_scene_module_is_registered() -> None:
    """No orphan scene modules: every ``scenes/*.py`` is imported by the registry.

    Guards the scalable-contract criterion: a new scene file that is never wired
    into ``registry.py`` would otherwise silently escape all catalog coverage.
    Pure file parsing, so it runs even without a GUI/dartpy build.
    """

    demos_dir = _PYTHON_DIR / "examples" / "demos"
    modules = {
        path.stem
        for path in (demos_dir / "scenes").glob("*.py")
        if not path.stem.startswith("_") and path.stem != "__init__"
    }
    registry_src = (demos_dir / "registry.py").read_text(encoding="utf-8")
    imported = set(re.findall(r"from \.scenes\.(\w+) import", registry_src))
    orphans = sorted(modules - imported)
    assert not orphans, f"scene modules not imported by registry.py: {orphans}"
