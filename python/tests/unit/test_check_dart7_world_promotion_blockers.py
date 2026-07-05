import importlib.util
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_dart7_world_promotion_blockers.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_dart7_world_promotion_blockers", SCRIPT
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _messages(violations) -> list[str]:
    return [violation.message for violation in violations]


def test_current_repo_blockers_are_classified():
    module = _load_module()

    assert module.find_violations(ROOT) == []


def test_known_loader_reference_is_classified(tmp_path):
    module = _load_module()
    _write(
        tmp_path / "dart" / "io" / "legacy_world_loader.cpp",
        """
#include <dart/simulation/world.hpp>

dart::simulation::WorldPtr loadWorld();
""",
    )

    findings = module.collect_findings(tmp_path)

    assert {finding.category for finding in findings} == {"classic_loader_api"}
    assert module.find_violations(tmp_path) == []


def test_main_tree_parity_reference_is_classified_and_fails_by_default(tmp_path):
    module = _load_module()
    _write(
        tmp_path
        / "tests"
        / "unit"
        / "simulation"
        / "experimental"
        / "world"
        / "test_world_parity.cpp",
        """
#include <dart/simulation/world.hpp>

auto legacy = dart::simulation::World::create();
""",
    )

    findings = module.collect_findings(tmp_path)

    assert {finding.category for finding in findings} == {"main_tree_parity_reference"}

    messages = _messages(module.find_violations(tmp_path))

    assert any(
        "main_tree_parity_reference blocker count grew" in message
        for message in messages
    )


def test_category_ceiling_growth_fails(tmp_path):
    module = _load_module()
    _write(
        tmp_path
        / "tests"
        / "unit"
        / "simulation"
        / "experimental"
        / "world"
        / "test_world_parity.cpp",
        """
#include <dart/simulation/world.hpp>

auto legacy = dart::simulation::World::create();
""",
    )

    messages = _messages(
        module.find_violations(
            tmp_path,
            category_ceilings={"main_tree_parity_reference": 0},
            category_allowed_paths=None,
        )
    )

    assert any(
        "main_tree_parity_reference blocker count grew" in message
        for message in messages
    )


def test_main_tree_parity_reference_new_file_fails(tmp_path):
    module = _load_module()
    _write(
        tmp_path / "tests" / "unit" / "simulation" / "experimental" / "new.cpp",
        """
#include <dart/simulation/world.hpp>

auto legacy = dart::simulation::World::create();
""",
    )

    messages = _messages(module.find_violations(tmp_path))

    assert any(
        "main_tree_parity_reference blocker moved into an unapproved file" in message
        for message in messages
    )


def test_retired_contact_parity_file_fails_if_classic_world_returns(tmp_path):
    module = _load_module()
    _write(
        tmp_path
        / "tests"
        / "unit"
        / "simulation"
        / "experimental"
        / "world"
        / "test_world_contact_parity.cpp",
        """
#include <dart/simulation/world.hpp>

auto legacy = dart::simulation::World::create();
""",
    )

    messages = _messages(module.find_violations(tmp_path))

    assert any(
        "main_tree_parity_reference blocker moved into an unapproved file" in message
        for message in messages
    )


def test_retired_multibody_parity_file_fails_if_classic_world_returns(tmp_path):
    module = _load_module()
    _write(
        tmp_path
        / "tests"
        / "unit"
        / "simulation"
        / "experimental"
        / "multibody"
        / "test_skeleton_to_multibody.cpp",
        """
#include <dart/simulation/world.hpp>

auto legacy = dart::simulation::World::create();
""",
    )

    messages = _messages(module.find_violations(tmp_path))

    assert any(
        "main_tree_parity_reference blocker moved into an unapproved file" in message
        for message in messages
    )


def test_retired_skeleton_loader_file_fails_if_classic_world_returns(tmp_path):
    module = _load_module()
    _write(
        tmp_path
        / "tests"
        / "unit"
        / "simulation"
        / "experimental"
        / "world"
        / "test_skeleton_loader.cpp",
        """
#include <dart/simulation/world.hpp>

auto legacy = dart::simulation::World::create();
""",
    )

    messages = _messages(module.find_violations(tmp_path))

    assert any(
        "main_tree_parity_reference blocker moved into an unapproved file" in message
        for message in messages
    )


def test_release6_branch_ref_filter_accepts_only_branches():
    module = _load_module()

    refs = module.release6_branch_refs(
        (
            "refs/heads/main",
            "refs/heads/release-6.17",
            "refs/remotes/origin/release-6.16",
            "refs/remotes/origin/feature/release-6.17",
            "refs/tags/v6.17.1",
        )
    )

    assert refs == (
        "refs/heads/release-6.17",
        "refs/remotes/origin/release-6.16",
    )


def test_release6_branch_ref_requirement_fails_without_branch():
    module = _load_module()

    messages = _messages(module.release6_branch_violations(("refs/tags/v6.17.1",)))

    assert any(
        "requires a local release-6.* branch ref" in message for message in messages
    )


def test_release6_branch_ref_requirement_passes_with_branch():
    module = _load_module()

    assert (
        module.release6_branch_violations(("refs/remotes/origin/release-6.17",)) == []
    )


def test_unknown_classic_world_reference_fails(tmp_path):
    module = _load_module()
    _write(
        tmp_path / "new_surface" / "world_api.cpp",
        """
#include <dart/simulation/world.hpp>

auto world = dart::simulation::World::create();
""",
    )

    messages = _messages(module.find_violations(tmp_path))

    assert any("unclassified classic-world blocker" in message for message in messages)


def test_unknown_public_classic_world_loader_fails(tmp_path):
    module = _load_module()
    _write(
        tmp_path / "new_surface" / "world_loader.cpp",
        """
auto world = dart::io::readWorld(uri);
""",
    )

    messages = _messages(module.find_violations(tmp_path))

    assert any("unclassified classic-world blocker" in message for message in messages)


def test_unknown_temporary_world_facade_fails(tmp_path):
    module = _load_module()
    _write(
        tmp_path / "new_surface" / "world7.cpp",
        """
#include <dart/simulation/world7.hpp>

auto* world = new dart::simulation::v7::World();
""",
    )

    messages = _messages(module.find_violations(tmp_path))

    assert any(
        "unclassified temporary-world-facade blocker" in message for message in messages
    )


def test_unknown_experimental_reference_fails(tmp_path):
    module = _load_module()
    _write(
        tmp_path / "new_surface" / "world_api.cpp",
        """
#include <dart/simulation/experimental/world.hpp>
namespace sx = dart::simulation::experimental;
""",
    )

    messages = _messages(module.find_violations(tmp_path))

    assert any(
        "unclassified experimental-staging blocker" in message for message in messages
    )


def test_deleted_tracked_files_are_skipped(tmp_path):
    module = _load_module()
    deleted = tmp_path / "tests" / "unit" / "deleted.cpp"
    _write(deleted, "#include <dart/simulation/world.hpp>\n")

    subprocess.run(["git", "init"], cwd=tmp_path, check=True, stdout=subprocess.PIPE)
    subprocess.run(["git", "add", "."], cwd=tmp_path, check=True)
    deleted.unlink()

    assert deleted.relative_to(tmp_path) not in module._tracked_files(tmp_path)
    assert module.find_violations(tmp_path) == []


def test_strict_final_fails_on_classified_code_debt(tmp_path):
    module = _load_module()
    _write(
        tmp_path / "dart" / "io" / "legacy_world_loader.cpp",
        """
#include <dart/simulation/world.hpp>

dart::simulation::WorldPtr loadWorld();
""",
    )

    messages = _messages(module.find_final_violations(tmp_path))

    assert any(
        "final DART 7 promotion still has classic-world debt" in message
        for message in messages
    )


def test_strict_final_allows_transition_docs(tmp_path):
    module = _load_module()
    _write(
        tmp_path / "docs" / "plans" / "041-official-simulation-api-promotion.md",
        """
Retire DART_BUILD_SIMULATION_EXPERIMENTAL and dart-simulation-experimental.
""",
    )

    assert module.find_final_violations(tmp_path) == []
