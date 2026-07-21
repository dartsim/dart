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

BASE_HEADER_SHA256 = "396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50"
MANIFOLD_HEADER_SHA256 = (
    "007311fb28062377dd6a0d26cad1ab4f7e2c99f359afd33554651f3cc0929ef5"
)
POLICY_HEADER_SHA256 = (
    "b8590420ebcbf62c522fb88a5cad06f0c5ebd917400cf578c4c63f2d76dc1a36"
)
CROSS_STEP_POLICY_COLUMNS = [
    "cross_step_policy_contract",
    "requested_cross_step_policy",
    "actual_cross_step_policy",
    "requested_native_contact_manifold_mode",
    "actual_native_contact_manifold_mode",
    "collision_max_contacts",
    "collision_max_contacts_per_pair",
    "step_exact_attempts",
    "step_exact_max_iterations_accepted",
    "step_warm_start_gamma_caps",
    "step_unconverged_warm_start_cache_skips",
    "worst_exact_residual_to_date",
    "last_exact_diagnostics_contract",
    "last_exact_initial_natural_map_residual",
    "last_exact_final_natural_map_residual",
    "last_exact_uncapped_initial_gamma",
    "last_exact_warm_start_gamma_cap_applied",
    "warm_start_match_mode",
    "warm_start_match_distance",
    "warm_start_normal_cosine",
    "strict_warm_start_match_distance",
    "warm_start_max_age",
    "persistent_gamma_safe_bound_scale",
    "minimum_adaptive_gamma",
    "maximum_adaptive_gamma",
    "warm_start_gamma_natural_residual_threshold",
    "warm_start_gamma_cap",
    "persist_uncapped_gamma_after_warm_cap",
    "require_residual_improvement_for_unconverged_cache_save",
    "coupling_variation_tolerance",
    "shrink_factor",
    "max_step_shrink_iterations",
]


def _source() -> str:
    return TRACE_SOURCE.read_text(encoding="utf-8")


def _string_literals(fragment: str) -> str:
    literals = re.findall(r'"((?:\\.|[^"\\])*)"', fragment)
    return bytes("".join(literals), "utf-8").decode("unicode_escape")


def _performance_header_fragments(source: str) -> tuple[str, str, str]:
    base_match = re.search(
        r"void printPerformanceHeader\(SolverContract contract\)\s*\{\s*"
        r"std::printf\((.*?)\);\s*if \(contract",
        source,
        re.DOTALL,
    )
    assert base_match is not None

    manifold_match = re.search(
        r"if \(nativeManifoldSensitivityEnabled\(\)\) \{\s*"
        r"std::printf\((.*?)\);\s*\}",
        source,
        re.DOTALL,
    )
    assert manifold_match is not None

    policy_match = re.search(
        r"if \(crossStepPolicyEvidenceEnabled\(\)\) \{\s*"
        r"std::printf\((.*?)\);\s*\}",
        source,
        re.DOTALL,
    )
    assert policy_match is not None

    return (
        _string_literals(base_match.group(1)).strip(),
        _string_literals(manifold_match.group(1)),
        _string_literals(policy_match.group(1)),
    )


def _function_body(source: str, signature: str, next_signature: str) -> str:
    match = re.search(
        rf"{signature}\s*\{{(.*?)(?=\n{next_signature})",
        source,
        re.DOTALL,
    )
    assert match is not None
    return match.group(1)


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


def test_existing_headers_remain_byte_identical_and_policy_suffix_is_exact():
    base, manifold, policy = _performance_header_fragments(_source())

    assert len(base.split(",")) == 83
    assert hashlib.sha256((base + "\n").encode()).hexdigest() == BASE_HEADER_SHA256
    assert len((base + manifold).split(",")) == 94
    assert (
        hashlib.sha256((base + manifold + "\n").encode()).hexdigest()
        == MANIFOLD_HEADER_SHA256
    )
    assert policy.startswith(",")
    assert policy.split(",")[1:] == CROSS_STEP_POLICY_COLUMNS
    assert len(CROSS_STEP_POLICY_COLUMNS) == 32
    assert len((base + policy).split(",")) == 115
    assert (
        hashlib.sha256((base + policy + "\n").encode()).hexdigest()
        == POLICY_HEADER_SHA256
    )


def test_policy_selector_and_frozen_90_step_guard_are_exact():
    source = _source()
    parser = _function_body(
        source,
        r"bool parseCrossStepPolicySelector\(.*?\)",
        r"bool parseLocalSolverOverride",
    )
    for selector in (
        "default",
        "dart_current",
        "author_policy_inspired_b3f3c5c",
    ):
        assert f'"{selector}"' in parser
    assert re.search(
        r"parseCrossStepPolicySelector\(\s*"
        r"argc > 16 \? argv\[16\] : nullptr, gCrossStepPolicySelector\)",
        source,
    )

    guard_match = re.search(
        r"if \(crossStepPolicyEvidenceEnabled\(\)\) \{\s*"
        r"const bool frozenContract(.*?)\n  \}",
        source,
        re.DOTALL,
    )
    assert guard_match is not None
    guard = guard_match.group(1)
    assert "argc == 17" in guard
    for index, value in (
        (1, "card_house_26_settle_projectile_full"),
        (2, "exact_fbf"),
        (3, "1"),
        (4, "90"),
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
        (15, "default"),
    ):
        assert re.search(rf'argv\[{index}\]\)\s*==\s*"{value}"', guard)

    assert source.index("if (crossStepPolicyEvidenceEnabled())") < source.index(
        "auto world = createTraceWorld("
    )


def test_policy_is_installed_then_classified_from_solver_readbacks():
    source = _source()
    author_options = _function_body(
        source,
        r"dart::constraint::ExactCoulombFbfCrossStepPolicyOptions\s*"
        r"authorPolicyInspiredCrossStepOptions\(\)",
        r"bool matchesAuthorPolicyInspiredCrossStepPolicy",
    )
    for statement in (
        "policy.warmStartMatchMode = MatchMode::OrderedBodyBLocalFeature;",
        "policy.warmStartNormalCosine = 0.9;",
        "policy.useStrictWarmStartMatchDistance = true;",
        "policy.warmStartMaxAge = 3;",
        "policy.persistentStepSizeSafeBoundScale = 10.0;",
        "policy.minimumStepSize = 1e-6;",
        "policy.maximumStepSize = 1e6;",
        "policy.warmStartResidualThreshold = 1e-4;",
        "policy.warmStartStepSizeCap = 1e4;",
        "policy.persistUncappedStepSizeOnWarmStartCap = true;",
        "policy.requireResidualImprovementForUnconvergedCacheSave = true;",
    ):
        assert statement in author_options

    classifier = _function_body(
        source,
        r"const char\* classifyCrossStepPolicy\(.*?\)",
        r"void installCrossStepPolicy",
    )
    assert "solver.getExactCoulombOptions()" in classifier
    assert "solver.getExactCoulombCrossStepPolicyOptions()" in classifier
    assert "matchesDartCurrentCrossStepPolicy(options, policy)" in classifier
    assert "matchesAuthorPolicyInspiredCrossStepPolicy(options, policy)" in classifier
    assert 'return "unclassified";' in classifier

    installer = _function_body(
        source,
        r"void installCrossStepPolicy\(.*?\)",
        r"std::shared_ptr<dart::collision::NativeCollisionDetector>",
    )
    assert "solver.setExactCoulombCrossStepPolicyOptions({});" in installer
    assert "solver.setExactCoulombOptions(options);" in installer
    assert (
        "solver.setExactCoulombCrossStepPolicyOptions(\n"
        "      authorPolicyInspiredCrossStepOptions());" in installer
    )

    install_call = source.index("installCrossStepPolicy(*exactSolver);")
    readback_check = source.index(
        "std::string(classifyCrossStepPolicy(*exactSolver))", install_call
    )
    header_call = source.index("printPerformanceHeader(contract);", readback_check)
    assert install_call < readback_check < header_call
    for token in (
        "detector->getContactManifoldMode()",
        "ContactManifoldMode::Compact",
        "collisionOption.maxNumContacts != kCardHouseReducedMaxContacts",
        "collisionOption.maxNumContactsPerPair",
        "cross-step policy failed installed solver, detector, collision-",
    ):
        assert token in source[install_call:header_call]


def test_subprocess_omitted_and_default_argv16_are_byte_identical():
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
        "default",
    ]
    omitted = _run(common)
    explicit_default = _run([*common, "default"])

    assert omitted.returncode == explicit_default.returncode == 1
    assert omitted.stdout == explicit_default.stdout
    assert omitted.stderr == explicit_default.stderr == ""
    assert len(omitted.stdout.strip().split(",")) == 83


def test_subprocess_rejects_unknown_argv16_before_csv():
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
            "default",
            "not_a_policy",
        ]
    )

    assert result.returncode == 2
    assert result.stdout == ""
    assert "Usage: fbf_paper_trace" in result.stderr


@pytest.mark.parametrize("selector", ["dart_current", "author_policy_inspired_b3f3c5c"])
def test_subprocess_rejects_89_step_explicit_arm_before_csv(selector: str):
    result = _run(
        [
            "card_house_26_settle_projectile_full",
            "exact_fbf",
            "1",
            "89",
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
            "default",
            selector,
        ]
    )

    assert result.returncode == 2
    assert result.stdout == ""
    assert "requires the frozen 90-step card-house A/B v1" in result.stderr
