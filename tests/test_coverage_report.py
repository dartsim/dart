"""Tests for scripts/coverage_report.py."""

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "coverage_report.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("coverage_report", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _touch_gcda(build_dir, relative_path):
    path = build_dir / relative_path
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("", encoding="utf-8")
    return path


def test_collect_gcda_directories_prunes_finally_excluded_sources(tmp_path):
    module = _load_module()
    build_dir = tmp_path / "build"

    kept_core = _touch_gcda(
        build_dir,
        "dart/CMakeFiles/dart.dir/dynamics/body.cpp.gcda",
    )
    kept_dartsim_ui = _touch_gcda(
        build_dir,
        "dartsim/ui/CMakeFiles/dartsim_ui.dir/src/panel.cpp.gcda",
    )
    _touch_gcda(
        build_dir,
        "dart/CMakeFiles/dart.dir/gui/osg.cpp.gcda",
    )
    _touch_gcda(
        build_dir,
        "dart/gui/CMakeFiles/dart-gui-core.dir/view.cpp.gcda",
    )
    _touch_gcda(
        build_dir,
        "tests/unit/CMakeFiles/test_world.dir/world/test_world.cpp.gcda",
    )
    _touch_gcda(
        build_dir,
        "examples/CMakeFiles/demo.dir/demo.cpp.gcda",
    )
    _touch_gcda(
        build_dir,
        "tutorials/CMakeFiles/tutorial.dir/tutorial.cpp.gcda",
    )

    directories = module.collect_gcda_directories(build_dir)

    assert directories == [kept_core.parent, kept_dartsim_ui.parent]
