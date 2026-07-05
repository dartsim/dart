import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
MODULE_PATH = ROOT / "scripts" / "coverage_report.py"
SPEC = importlib.util.spec_from_file_location("coverage_report", MODULE_PATH)
coverage_report = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(coverage_report)


def test_default_build_dir_uses_pixi_environment_and_build_type():
    assert coverage_report.default_build_dir(
        {"PIXI_ENVIRONMENT_NAME": "default", "BUILD_TYPE": "Debug"}
    ) == Path("build/default/cpp/Debug")


def test_default_build_dir_falls_back_to_ci_defaults():
    assert coverage_report.default_build_dir({}) == Path("build/default/cpp/Debug")


def test_delete_excluded_coverage_files_removes_examples_and_tutorials(tmp_path):
    build_dir = tmp_path / "build"
    keep = build_dir / "dart" / "CMakeFiles" / "lib.dir" / "World.cpp.gcda"
    remove_example = build_dir / "examples" / "demo.cpp.gcda"
    remove_tutorial = build_dir / "tutorials" / "tutorial.cpp.gcno"
    keep.parent.mkdir(parents=True)
    remove_example.parent.mkdir(parents=True)
    remove_tutorial.parent.mkdir(parents=True)
    keep.write_text("", encoding="utf-8")
    remove_example.write_text("", encoding="utf-8")
    remove_tutorial.write_text("", encoding="utf-8")

    coverage_report.delete_excluded_coverage_files(build_dir)

    assert keep.exists()
    assert not remove_example.exists()
    assert not remove_tutorial.exists()


def test_collect_gcda_directories_prunes_nested_directories(tmp_path):
    build_dir = tmp_path / "build"
    parent = build_dir / "dart" / "CMakeFiles" / "dart.dir"
    child = parent / "subdir"
    sibling = build_dir / "dartsim" / "engine" / "CMakeFiles" / "engine.dir"
    for directory in (parent, child, sibling):
        directory.mkdir(parents=True)
    (parent / "World.cpp.gcda").write_text("", encoding="utf-8")
    (child / "Nested.cpp.gcda").write_text("", encoding="utf-8")
    (sibling / "Engine.cpp.gcda").write_text("", encoding="utf-8")

    assert coverage_report.collect_gcda_directories(build_dir) == [parent, sibling]


def test_round_robin_chunks_matches_split_round_robin_distribution():
    items = [Path(str(index)) for index in range(7)]

    assert coverage_report.round_robin_chunks(items, 3) == [
        [Path("0"), Path("3"), Path("6")],
        [Path("1"), Path("4")],
        [Path("2"), Path("5")],
    ]


def test_lcov_capture_command_adds_each_directory_once(tmp_path):
    output_file = tmp_path / "chunk.info"
    command = coverage_report.lcov_capture_command(
        [Path("build/a"), Path("build/b")], output_file
    )

    assert command[:3] == ["lcov", "--quiet", "--capture"]
    assert command.count("--directory") == 2
    assert command[command.index("--output-file") + 1] == str(output_file)
    assert "geninfo_unexecuted_blocks=1" in command
