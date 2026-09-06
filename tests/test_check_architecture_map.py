"""Tests for scripts/check_architecture_map.py (PLAN-130 WP-130.2).

A synthetic repository is built under ``tmp_path`` so every rule is exercised
without depending on the real source tree; one test also runs the checker
against the real repository.
"""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_architecture_map.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_architecture_map", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


cam = _load_module()

SCHEDULE_HEADER = """
namespace dart::simulation::detail {
enum class BuiltInWorldStepStageSlot
{
  RigidBodyVelocity,
  RigidBodyContact, // comment
  Kinematics,
};
}
"""
OPTIONS_HEADER = """
namespace dart {
namespace simulation {
enum class RigidBodySolver
{
  SequentialImpulse,
  Ipc = 3,
};
enum class ContactSolverMethod { SequentialImpulse, BoxedLcp };
enum class ContactGradientMode;
} // namespace simulation
} // namespace dart
"""
MULTIBODY_HEADER = """
namespace dart::simulation {
enum class MultibodyIntegrationFamily
{
  SemiImplicit,
  Variational,
};
}
"""
STAGES_HEADER = """
namespace dart::simulation::compute {
class DART_SIMULATION_API RigidBodyVelocityStage final : public WorldStepStage
{
};
class DART_SIMULATION_API KinematicsStage final
  : public WorldStepStage
{
};
} // namespace dart::simulation::compute
"""
WORLD_HEADER = (
    "namespace dart::simulation {\nclass World\n{\n  void step();\n  StateSpace space;\n};\n}\n"
    + "\n" * 20
)


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _framework_view() -> dict:
    return {
        "schema_version": 1,
        "diagram_type": "architecture",
        "meta": {"title": "Framework"},
        "components": [
            {
                "id": "world",
                "type": "external",
                "label": "World facade",
                "sublabel": "dart::simulation::World · StateSpace",
                "tag": "Implemented",
                "pos": [0, 0],
                "size": [100, 50],
                "sources": [
                    {
                        "path": "dart/simulation/world.hpp",
                        "line": 2,
                        "end_line": 3,
                        "label": "World",
                    },
                    {"path": "dart/simulation/body/rigid_body.hpp"},
                    {"path": "dart/simulation/detail/world_step_schedule.hpp"},
                ],
            },
            {
                "id": "rigid",
                "type": "backend",
                "label": "Rigid solver families",
                "sublabel": "SequentialImpulse · Ipc · SemiImplicit · Variational",
                "tag": "Partial",
                "pos": [200, 0],
                "size": [100, 50],
                "sources": [
                    {"path": "dart/simulation/multibody/multibody_options.hpp"},
                    {"path": "dart/simulation/compute/multibody_dynamics.hpp"},
                ],
            },
        ],
        "boundaries": [{"kind": "region", "label": "Internals", "wraps": ["rigid"]}],
        "cards": [{"title": "Selectors", "items": ["ContactSolverMethod: BoxedLcp"]}],
        "connections": [
            {"id": "w-r", "from": "world", "to": "rigid", "label": "stages"}
        ],
    }


def _step_view() -> dict:
    return {
        "schema_version": 1,
        "diagram_type": "dataflow",
        "meta": {"title": "Step"},
        "stages": [
            {"label": "Velocity"},
            {"label": "Contact"},
            {"label": "Kinematics"},
        ],
        "nodes": [
            {
                "id": "rigid_body_velocity",
                "type": "backend",
                "label": "Rigid velocity",
                "tag": "Implemented",
                "stage": 0,
                "row": 0,
            },
            {
                "id": "rigid_body_contact",
                "type": "backend",
                "label": "Rigid contact",
                "tag": "Partial",
                "stage": 1,
                "row": 0,
            },
            {
                "id": "kinematics",
                "type": "backend",
                "label": "Kinematics",
                "tag": "Implemented",
                "stage": 2,
                "row": 0,
            },
        ],
        "flows": [
            {
                "from": "rigid_body_velocity",
                "to": "rigid_body_contact",
                "label": "velocities",
            },
            {"from": "rigid_body_contact", "to": "kinematics", "label": "poses"},
        ],
    }


def _compute_view() -> dict:
    return {
        "schema_version": 1,
        "diagram_type": "architecture",
        "meta": {"title": "Compute"},
        "components": [
            {
                "id": "graph",
                "type": "cloud",
                "label": "Compute graph",
                "tag": "Implemented",
                "pos": [0, 0],
                "size": [100, 50],
                "sources": [
                    {"path": "dart/simulation/compute/compute_graph.hpp"},
                    {"path": "dart/simulation/compute/cuda/kernel.cuh"},
                    {"path": "dart/simulation/compute/detail/world_step_stages.hpp"},
                ],
            }
        ],
        "connections": [],
    }


def _library_view() -> dict:
    return {
        "schema_version": 1,
        "diagram_type": "architecture",
        "meta": {"title": "Library"},
        "components": [
            {
                "id": "modules",
                "type": "backend",
                "label": "Modules",
                "tag": "Implemented",
                "pos": [0, 0],
                "size": [100, 50],
                "sources": [
                    {"path": "dart/simulation/world.hpp"},
                    {"path": "dart/math/lie.hpp"},
                    {"path": "python/dartpy/module.cpp"},
                ],
            },
            {
                "id": "app",
                "type": "frontend",
                "label": "dartsim app",
                "tag": "Partial",
                "pos": [200, 0],
                "size": [100, 50],
                "sources": [{"path": "dartsim/main.cpp"}],
            },
        ],
        "connections": [],
    }


PAGE = """# Architecture

<iframe src="architecture-map/simulation-framework.html"></iframe>
Source: docs/assets/architecture/simulation-framework.architecture.json
<iframe src="architecture-map/world-step.html"></iframe> world-step.dataflow.json
<iframe src="architecture-map/compute-graph.html"></iframe> compute-graph.architecture.json
<iframe src="architecture-map/library-context.html"></iframe> library-context.architecture.json
"""


@pytest.fixture
def repo(tmp_path: Path) -> Path:
    root = tmp_path / "repo"
    _write(root / "dart/simulation/world.hpp", WORLD_HEADER)
    _write(root / "dart/simulation/world_options.hpp", OPTIONS_HEADER)
    _write(root / "dart/simulation/detail/world_step_schedule.hpp", SCHEDULE_HEADER)
    _write(root / "dart/simulation/multibody/multibody_options.hpp", MULTIBODY_HEADER)
    _write(root / "dart/simulation/compute/detail/world_step_stages.hpp", STAGES_HEADER)
    _write(
        root / "dart/simulation/compute/compute_graph.hpp", "class ComputeGraph {};\n"
    )
    _write(
        root / "dart/simulation/compute/multibody_dynamics.hpp",
        "class UnifiedConstraint {};\n",
    )
    _write(root / "dart/simulation/compute/cuda/kernel.cuh", "// cuda\n")
    _write(root / "dart/simulation/body/rigid_body.hpp", "class RigidBody {};\n")
    _write(root / "dart/simulation/ecs/component_access.hpp", "// ecs\n")
    _write(root / "dart/math/lie.hpp", "class SE3 {};\n")
    _write(root / "python/dartpy/module.cpp", "// bindings\n")
    _write(root / "dartsim/main.cpp", "// app\n")
    views = root / "docs/assets/architecture"
    _write(
        views / "simulation-framework.architecture.json", json.dumps(_framework_view())
    )
    _write(views / "world-step.dataflow.json", json.dumps(_step_view()))
    _write(views / "compute-graph.architecture.json", json.dumps(_compute_view()))
    _write(views / "library-context.architecture.json", json.dumps(_library_view()))
    _write(views / "compute-graph.runtime.json", "{}")
    _write(root / "docs/readthedocs/architecture.md", PAGE)
    return root


def _run(root: Path) -> list[str]:
    return cam.Checker(repo_root=root).run()


def _rewrite(root: Path, name: str, mutate) -> None:
    path = root / "docs/assets/architecture" / name
    ir = json.loads(path.read_text(encoding="utf-8"))
    mutate(ir)
    path.write_text(json.dumps(ir), encoding="utf-8")


def test_snake_case_matches_stage_slot_naming() -> None:
    assert cam.snake_case("RigidBodyVelocity") == "rigid_body_velocity"
    assert cam.snake_case("RigidIpcContact") == "rigid_ipc_contact"
    assert (
        cam.snake_case("MultibodyVariationalIntegration")
        == "multibody_variational_integration"
    )


def test_parse_enumerators_strips_comments_and_values() -> None:
    assert cam.parse_enumerators(SCHEDULE_HEADER, "BuiltInWorldStepStageSlot") == [
        "RigidBodyVelocity",
        "RigidBodyContact",
        "Kinematics",
    ]
    assert cam.parse_enumerators(OPTIONS_HEADER, "RigidBodySolver") == [
        "SequentialImpulse",
        "Ipc",
    ]
    assert cam.parse_enumerators(OPTIONS_HEADER, "Missing") == []


def test_clean_fixture_passes(repo: Path) -> None:
    assert _run(repo) == []


def test_missing_required_view_is_reported(repo: Path) -> None:
    (repo / "docs/assets/architecture/library-context.architecture.json").unlink()
    errors = _run(repo)
    assert any("required view `library-context` is missing" in e for e in errors)


def test_missing_path_and_bad_lines(repo: Path) -> None:
    def mutate(ir):
        ir["components"][0]["sources"][0]["line"] = 999
        ir["components"][0]["sources"][1]["path"] = "dart/simulation/body/nope.hpp"

    _rewrite(repo, "simulation-framework.architecture.json", mutate)
    errors = _run(repo)
    assert any("line 999" in e for e in errors)
    assert any("missing path `dart/simulation/body/nope.hpp`" in e for e in errors)


def test_end_line_before_line_is_reported(repo: Path) -> None:
    def mutate(ir):
        ir["components"][0]["sources"][0]["end_line"] = 1

    _rewrite(repo, "simulation-framework.architecture.json", mutate)
    assert any("end_line 1" in e for e in _run(repo))


def test_line_cited_source_must_hold_its_symbol(repo: Path) -> None:
    def moved(ir):
        ir["components"][0]["sources"][0]["label"] = "StateSpace"

    _rewrite(repo, "simulation-framework.architecture.json", moved)
    errors = _run(repo)
    assert any(
        "lines 2..3 for `StateSpace`, but they no longer contain `StateSpace`" in e
        for e in errors
    )

    def prose(ir):
        ir["components"][0]["sources"][0]["label"] = "loading bridge"

    _rewrite(repo, "simulation-framework.architecture.json", prose)
    assert any("needs the symbol declared there as its label" in e for e in _run(repo))

    def qualified(ir):
        ir["components"][0]["sources"][0] = {
            "path": "dart/simulation/world.hpp",
            "line": 4,
            "label": "World::step",
        }

    _rewrite(repo, "simulation-framework.architecture.json", qualified)
    assert _run(repo) == []


def test_unknown_symbol_in_sublabel(repo: Path) -> None:
    def mutate(ir):
        ir["components"][0][
            "sublabel"
        ] = "dart::simulation::Nonexistent · GhostClassName"

    _rewrite(repo, "simulation-framework.architecture.json", mutate)
    errors = _run(repo)
    assert any("`dart::simulation::Nonexistent`" in e for e in errors)
    assert any("`GhostClassName`" in e for e in errors)


def test_qualified_symbols_resolve_in_their_namespace_directory(repo: Path) -> None:
    checker = cam.Checker(repo_root=repo)
    assert checker.symbol_resolves("dart::simulation::World")
    assert checker.symbol_resolves("dart::simulation::RigidBodySolver")
    assert not checker.symbol_resolves("dart::collision::World")
    assert not checker.symbol_resolves("dart::nowhere::World")
    assert not checker.symbol_resolves("totally::wrong::World")
    assert checker.symbol_resolves("StateSpace")
    # The enclosing namespace must match, not just the module directory.
    assert checker.symbol_resolves("dart::simulation::compute::KinematicsStage")
    assert not checker.symbol_resolves("dart::simulation::KinematicsStage")
    assert checker.symbol_resolves(
        "dart::simulation::detail::BuiltInWorldStepStageSlot"
    )
    assert not checker.symbol_resolves("dart::simulation::BuiltInWorldStepStageSlot")
    # Namespaces resolve as symbols, in both nested and compact forms.
    assert checker.symbol_resolves("dart::simulation")
    assert checker.symbol_resolves("dart::simulation::compute")


def test_enclosing_namespace_tracks_nested_and_compact_forms() -> None:
    text = (
        "namespace dart {\nnamespace collision {\nclass A {};\n}\n"
        "namespace math { struct B { void f() {} }; }\n}\n"
        "namespace dart::simulation::compute {\nclass C {};\n}\nclass D {};\n"
    )
    assert cam.enclosing_namespace(text, text.index("class A")) == "dart::collision"
    assert cam.enclosing_namespace(text, text.index("struct B")) == "dart::math"
    assert (
        cam.enclosing_namespace(text, text.index("class C"))
        == "dart::simulation::compute"
    )
    assert cam.enclosing_namespace(text, text.index("class D")) == ""


def test_dartpy_dotted_names_are_not_symbols() -> None:
    assert cam.Checker._symbols("dartpy.World · dart::simulation::World") == [
        "dart::simulation::World"
    ]
    assert cam.Checker._symbols("Taskflow executors and Ipc") == []


def test_bad_tag_and_dangling_edge(repo: Path) -> None:
    def mutate(ir):
        ir["components"][0]["tag"] = "Done"
        ir["connections"].append({"from": "world", "to": "ghost", "label": "x"})
        ir["boundaries"][0]["wraps"].append("phantom")

    _rewrite(repo, "simulation-framework.architecture.json", mutate)
    errors = _run(repo)
    assert any("tag 'Done'" in e for e in errors)
    assert any("to=`ghost`" in e for e in errors)
    assert any("wraps unknown id `phantom`" in e for e in errors)


def test_missing_tag_is_reported(repo: Path) -> None:
    def mutate(ir):
        ir["components"][0].pop("tag")

    _rewrite(repo, "simulation-framework.architecture.json", mutate)
    assert any("`world` has no status tag" in e for e in _run(repo))


def test_duplicate_view_names_across_suffixes_are_rejected(repo: Path) -> None:
    (repo / "docs/assets/architecture/compute-graph.dataflow.json").write_text(
        json.dumps(_step_view()), encoding="utf-8"
    )
    errors = _run(repo)
    assert any("view name `compute-graph` is already used" in e for e in errors)


def test_dataflow_needs_labels_and_valid_stage(repo: Path) -> None:
    def mutate(ir):
        ir["flows"][0].pop("label")
        ir["nodes"][0]["stage"] = 7

    _rewrite(repo, "world-step.dataflow.json", mutate)
    errors = _run(repo)
    assert any("has no label" in e for e in errors)
    assert any("stage 7" in e for e in errors)


def test_missing_stage_slot_node(repo: Path) -> None:
    def mutate(ir):
        ir["nodes"] = [n for n in ir["nodes"] if n["id"] != "rigid_body_contact"]
        ir["flows"] = []

    _rewrite(repo, "world-step.dataflow.json", mutate)
    errors = _run(repo)
    assert any("stage slot `RigidBodyContact` has no node" in e for e in errors)


def test_stage_id_outside_step_view_must_exist_there(repo: Path) -> None:
    def mutate(ir):
        ir["components"].append(
            {
                "id": "kinematics",
                "type": "backend",
                "label": "K",
                "tag": "Implemented",
                "pos": [0, 0],
                "size": [1, 1],
            }
        )

    _rewrite(repo, "compute-graph.architecture.json", mutate)
    assert _run(repo) == []  # kinematics exists in the step view

    def rename(ir):
        for node in ir["nodes"]:
            if node["id"] == "kinematics":
                node["id"] = "kinematics_stage"
        for flow in ir["flows"]:
            if flow["to"] == "kinematics":
                flow["to"] = "kinematics_stage"

    _rewrite(repo, "world-step.dataflow.json", rename)
    errors = _run(repo)
    assert any("stage id `kinematics` is not a node" in e for e in errors)


def test_unmapped_stage_class_is_reported(repo: Path) -> None:
    header = repo / "dart/simulation/compute/detail/world_step_stages.hpp"
    header.write_text(
        header.read_text(encoding="utf-8")
        + "class DART_SIMULATION_API GhostStage final : public WorldStepStage {};\n",
        encoding="utf-8",
    )
    errors = _run(repo)
    assert any("`GhostStage` derives from WorldStepStage" in e for e in errors)


def test_allowlisted_stage_class_passes(
    repo: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    header = repo / "dart/simulation/compute/detail/world_step_stages.hpp"
    header.write_text(
        header.read_text(encoding="utf-8")
        + "class DART_SIMULATION_API GhostStage final : public WorldStepStage {};\n",
        encoding="utf-8",
    )
    monkeypatch.setitem(cam.STAGE_CLASS_ALLOWLIST, "GhostStage", "test reason")
    assert _run(repo) == []


def test_simulation_directory_coverage(repo: Path) -> None:
    (repo / "dart/simulation/newdomain").mkdir()
    (repo / "dart/simulation/newdomain/thing.hpp").write_text("//\n", encoding="utf-8")
    errors = _run(repo)
    assert any("no source cites `dart/simulation/newdomain/`" in e for e in errors)


def test_exempt_directory_is_not_required(repo: Path) -> None:
    assert "dart/simulation/ecs" in cam.SIMULATION_COVERAGE_EXEMPTIONS
    assert _run(repo) == []


def test_enumerator_vocabulary_coverage(repo: Path) -> None:
    def mutate(ir):
        ir["components"][1][
            "sublabel"
        ] = "SequentialImpulse · SemiImplicit · Variational"

    _rewrite(repo, "simulation-framework.architecture.json", mutate)
    errors = _run(repo)
    assert any("`RigidBodySolver::Ipc` does not appear" in e for e in errors)


def test_every_public_selector_enum_is_swept(repo: Path) -> None:
    header = repo / "dart/simulation/world_options.hpp"
    text = header.read_text(encoding="utf-8")
    assert cam.parse_enum_names(text) == ["RigidBodySolver", "ContactSolverMethod"]
    header.write_text(
        text.replace(
            "enum class ContactGradientMode;",
            "enum class ContactGradientMode;\n"
            "enum class ComputeAcceleratorPolicy { CpuOnly, PreferAccelerated };",
        ),
        encoding="utf-8",
    )
    errors = _run(repo)
    assert any(
        "`ComputeAcceleratorPolicy::CpuOnly` does not appear" in e for e in errors
    )
    assert not any("ContactGradientMode" in e for e in errors)


def test_pixi_lint_aggregates_wire_the_map_gate_on_every_target() -> None:
    import tomllib

    config = tomllib.loads(
        (Path(__file__).resolve().parents[1] / "pixi.toml").read_text(encoding="utf-8")
    )

    def dependencies(tasks: dict, name: str) -> list[str]:
        entries = tasks.get(name, {}).get("depends-on", [])
        return [e["task"] if isinstance(e, dict) else e for e in entries]

    task_tables = {"tasks": config["tasks"]}
    for target, table in config.get("target", {}).items():
        if "tasks" in table:
            task_tables[f"target.{target}.tasks"] = table["tasks"]
    for scope, tasks in task_tables.items():
        for aggregate, gate in (
            ("lint", "lint-architecture-map"),
            ("check-lint", "check-architecture-map"),
        ):
            if aggregate in tasks:
                assert gate in dependencies(tasks, aggregate), f"{scope}.{aggregate}"


def test_compute_and_library_coverage(repo: Path) -> None:
    def drop_cuda(ir):
        ir["components"][0]["sources"] = ir["components"][0]["sources"][:1]

    _rewrite(repo, "compute-graph.architecture.json", drop_cuda)

    def drop_math(ir):
        ir["components"][0]["sources"] = [ir["components"][0]["sources"][0]]

    _rewrite(repo, "library-context.architecture.json", drop_math)
    errors = _run(repo)
    assert any(
        "compute-graph" in e and "`dart/simulation/compute/cuda/`" in e for e in errors
    )
    assert any("library-context" in e and "`dart/math/`" in e for e in errors)
    assert any("library-context" in e and "`python/dartpy/`" in e for e in errors)


def test_page_must_embed_and_name_every_view(repo: Path) -> None:
    page = repo / "docs/readthedocs/architecture.md"
    page.write_text("# Architecture\n\nnothing here\n", encoding="utf-8")
    errors = _run(repo)
    assert any("does not embed `architecture-map/world-step.html`" in e for e in errors)
    assert any("does not name the source" in e for e in errors)


def test_suffix_and_type_must_agree(repo: Path) -> None:
    def mutate(ir):
        ir["diagram_type"] = "dataflow"

    _rewrite(repo, "compute-graph.architecture.json", mutate)
    assert any("does not match the file suffix" in e for e in _run(repo))


def test_main_reports_and_exits(repo: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert cam.main(["--repo-root", str(repo)]) == 0
    assert "Validated 4 architecture map view(s)" in capsys.readouterr().out
    (repo / "docs/readthedocs/architecture.md").unlink()
    assert cam.main(["--repo-root", str(repo)]) == 1
    assert "ERROR:" in capsys.readouterr().out


def test_real_repository_passes() -> None:
    errors = cam.Checker().run()
    assert errors == [], "\n".join(errors)
