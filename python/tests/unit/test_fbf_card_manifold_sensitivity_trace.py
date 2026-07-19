import hashlib
import os
import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"
DEFAULT_TRACE_BINARY = (
    ROOT / "build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace"
)


def _source() -> str:
    return TRACE_SOURCE.read_text(encoding="utf-8")


def _string_literals(fragment: str) -> str:
    literals = re.findall(r'"((?:\\.|[^"\\])*)"', fragment)
    return bytes("".join(literals), "utf-8").decode("unicode_escape")


def _performance_headers(source: str) -> tuple[str, str]:
    base_match = re.search(
        r"void printPerformanceHeader\(SolverContract contract\)\s*\{\s*"
        r"std::printf\((.*?)\);\s*if \(contract",
        source,
        re.DOTALL,
    )
    assert base_match is not None
    base = _string_literals(base_match.group(1)).strip()

    sensitivity_match = re.search(
        r"if \(nativeManifoldSensitivityEnabled\(\)\) \{\s*"
        r"std::printf\((.*?)\);\s*\}",
        source,
        re.DOTALL,
    )
    assert sensitivity_match is not None
    sensitivity = _string_literals(sensitivity_match.group(1))
    return base, sensitivity


def _binary() -> Path:
    binary = Path(os.environ.get("FBF_PAPER_TRACE_BINARY", DEFAULT_TRACE_BINARY))
    if not binary.is_file():
        pytest.skip(f"trace binary not built: {binary}")
    return binary


def _run(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(_binary()), *args],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )


def test_default_performance_header_remains_byte_identical():
    base, sensitivity = _performance_headers(_source())

    assert len(base.split(",")) == 83
    assert hashlib.sha256((base + "\n").encode()).hexdigest() == (
        "396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50"
    )
    assert sensitivity.startswith(",")
    assert len((base + sensitivity).split(",")) == 94
    assert hashlib.sha256((base + sensitivity + "\n").encode()).hexdigest() == (
        "007311fb28062377dd6a0d26cad1ab4f7e2c99f359afd33554651f3cc0929ef5"
    )
    assert sensitivity.split(",")[1:] == [
        "manifold_sensitivity_contract",
        "requested_native_contact_manifold_mode",
        "actual_native_contact_manifold_mode",
        "collision_max_contacts",
        "collision_max_contacts_per_pair",
        "step_exact_max_iterations_accepted",
        "step_internal_fbf_status",
        "step_internal_fbf_best_iteration",
        "step_internal_fbf_best_residual",
        "colliding_body_pair_labels",
        "contact_multiplicity_by_body_pair",
    ]


def test_selector_parser_and_frozen_command_guard_are_exact():
    source = _source()

    assert "argc > 16" in source
    assert "argc > 15 ? argv[15] : nullptr" in source
    for token in ("default", "compact", "four_point_planar"):
        assert f'"{token}"' in source
    frozen_guard = re.search(
        r"if \(nativeManifoldSensitivityEnabled\(\)\) \{\s*"
        r"const bool frozenContract(.*?)\n  \}",
        source,
        re.DOTALL,
    )
    assert frozen_guard is not None
    guard = frozen_guard.group(1)
    for index, value in (
        (1, "card_house_26_settle_projectile_full"),
        (2, "exact_fbf"),
        (3, "1"),
        (4, "600"),
        (5, "nan"),
        (6, "performance"),
        (7, "default"),
        (8, "default"),
        (9, "1"),
        (10, "paper_cpu"),
        (11, "native"),
        (12, "default"),
        (13, "0"),
        (14, "0"),
    ):
        assert re.search(rf"argv\[{index}\]\)\s*==\s*\"{value}\"", guard)


def test_requested_mode_is_installed_and_actual_native_mode_is_read_back():
    source = _source()

    detector_builder = re.search(
        r"createFbfPaperNativeCollisionDetector\(Scenario scenario\)\s*\{" r"(.*?)\n\}",
        source,
        re.DOTALL,
    )
    assert detector_builder is not None
    builder = detector_builder.group(1)
    assert "NativeManifoldSensitivitySelector::Compact" in builder
    assert "NativeManifoldSensitivitySelector::FourPointPlanar" in builder
    assert "detector->setContactManifoldMode(mode);" in builder

    assert re.search(
        r"dynamic_pointer_cast<\s*dart::collision::NativeCollisionDetector>"
        r".*?detector->getContactManifoldMode\(\)",
        source,
        re.DOTALL,
    )
    assert "collisionOption.maxNumContacts != kCardHouseReducedMaxContacts" in source
    assert (
        "collisionOption.maxNumContactsPerPair\n"
        "               != kCardHouseReducedMaxContactsPerPair" in source
    )


def test_sensitivity_rows_export_internal_fbf_and_sorted_pair_multiplicities():
    source = _source()

    for token in (
        "getLastExactCoulombFbfStatus()",
        "getLastExactCoulombBestIteration()",
        "getLastExactCoulombBestResidual()",
        "card_house_native_manifold_sensitivity_v1",
        "pairMetrics.pairLabels",
        "pairMetrics.multiplicities",
    ):
        assert token in source
    assert "std::map<std::string, std::size_t> multiplicities;" in source
    assert "if (name2 < name1)" in source
    assert '++multiplicities[name1 + "|" + name2];' in source
    assert "counts << label << '=' << count;" in source


def test_subprocess_default_selector_matches_omitted_selector_byte_for_byte():
    common = [
        "backspin",
        "exact_fbf",
        "1",
        "0",
        "nan",
        "performance",
        "default",
        "default",
        "1",
        "dart_best",
        "dart",
        "default",
        "0",
        "0",
    ]
    omitted = _run(common)
    explicit_default = _run([*common, "default"])

    assert omitted.returncode == explicit_default.returncode == 1
    assert omitted.stdout == explicit_default.stdout
    assert omitted.stderr == explicit_default.stderr
    assert len(omitted.stdout.strip().split(",")) == 83


@pytest.mark.parametrize("selector", ["compact", "four_point_planar"])
def test_subprocess_explicit_selector_rejects_nonfrozen_command_before_stepping(
    selector: str,
):
    result = _run(
        [
            "card_house_26_settle_projectile_full",
            "exact_fbf",
            "1",
            "599",
            "nan",
            "performance",
            "default",
            "default",
            "1",
            "paper_cpu",
            "native",
            "default",
            "0",
            "0",
            selector,
        ]
    )

    assert result.returncode == 2
    assert result.stdout == ""
    assert "requires the frozen card-house v1" in result.stderr


def test_subprocess_rejects_unknown_selector():
    result = _run(
        [
            "backspin",
            "exact_fbf",
            "1",
            "0",
            "nan",
            "performance",
            "default",
            "default",
            "1",
            "dart_best",
            "dart",
            "default",
            "0",
            "0",
            "not_a_mode",
        ]
    )

    assert result.returncode == 2
    assert result.stdout == ""
    assert "Usage: fbf_paper_trace" in result.stderr
