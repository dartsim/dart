import hashlib
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"
PREREGISTRATION = (
    ROOT / "docs/dev_tasks/fbf_exact_coulomb_friction/LITERAL_CROWN_IMPACT_V1.md"
)


def _source() -> str:
    return TRACE_SOURCE.read_text(encoding="utf-8")


def _performance_header_parts(source: str) -> tuple[str, str]:
    base_match = re.search(
        r"void printPerformanceHeader\(SolverContract contract\)\s*\{\s*"
        r"std::printf\((.*?)\);\s*if \(contract",
        source,
        re.DOTALL,
    )
    assert base_match is not None
    base_literals = re.findall(r'"((?:\\.|[^"\\])*)"', base_match.group(1))
    base = bytes("".join(base_literals), "utf-8").decode("unicode_escape")

    colored_match = re.search(
        r"if \(contract == SolverContract::DartBestColoredBgs\)\s*\{\s*"
        r"std::printf\((.*?)\);\s*\}",
        source,
        re.DOTALL,
    )
    assert colored_match is not None
    colored_literals = re.findall(r'"((?:\\.|[^"\\])*)"', colored_match.group(1))
    colored = bytes("".join(colored_literals), "utf-8").decode("unicode_escape")
    return base, colored


def test_impact_scenario_parser_and_default_720_step_contract_are_explicit():
    source = _source()

    assert "MasonryArch25LiteralWedgeCrownImpactV1" in source
    assert 'name == "masonry_arch_25_literal_wedge_crown_impact_v1"' in source
    assert "constexpr std::size_t kLiteralCrownImpactLaunchAfterSteps = 600u;" in source
    assert "constexpr std::size_t kLiteralCrownImpactSteps = 120u;" in source
    assert "kLiteralCrownImpactLaunchAfterSteps + kLiteralCrownImpactSteps" in source
    assert (
        "case Scenario::MasonryArch25LiteralWedgeCrownImpactV1:\n"
        "      return kLiteralCrownImpactDefaultSteps;" in source
    )


def test_impact_projectile_construction_matches_frozen_preregistration():
    source = _source()
    preregistration = PREREGISTRATION.read_text(encoding="utf-8")

    for token in (
        "kLiteralCrownImpactProjectileCount = 3u",
        "kLiteralCrownImpactProjectileEdgeLength = 0.035",
        "kLiteralCrownImpactProjectileMass",
        "kLiteralCrownImpactProjectileSpacing = 0.045",
        "kLiteralCrownImpactProjectileDropHeight = 0.95",
        "kLiteralCrownImpactProjectileSpeed = 3.0",
        "joint->setAngularVelocity(Eigen::Vector3d::Zero())",
    ):
        assert token in source

    assert "`0.042875 kg`" in preregistration
    assert "x coordinates are `-0.045`, `0`, and `0.045 m`" in preregistration


def test_impact_world_starts_without_projectiles_and_launches_only_after_step_600():
    source = _source()
    world_case = re.search(
        r"case Scenario::MasonryArch25LiteralWedge:\s*"
        r"case Scenario::MasonryArch25LiteralWedgeCrownImpactV1:\s*"
        r"addLiteralMasonryArch25\(world\);\s*break;",
        source,
    )
    assert world_case is not None

    launch_guard = re.search(
        r"scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1\s*"
        r"&& step == kLiteralCrownImpactLaunchAfterSteps\)\s*\{\s*"
        r"launchLiteralCrownImpactProjectiles\(world\);",
        source,
    )
    assert launch_guard is not None
    assert re.search(
        r"captureLiteralCrownImpactPreImpactPoses\(\s*"
        r"world, previousCounters, literalCrownImpactState\);",
        source,
    )
    capture_offset = source.index(
        "captureLiteralCrownImpactPreImpactPoses(", source.index("int main")
    )
    apply_offset = source.index(
        "applyScenarioControl(world, scenario, step);", capture_offset
    )
    assert capture_offset < apply_offset
    assert "state.exactSolvesAtImpactLaunch = counters.exactSolves;" in source
    assert "counters.exactSolves > state.exactSolvesAtImpactLaunch" in source


def test_standing_performance_schemas_remain_byte_identical():
    base, colored = _performance_header_parts(_source())
    base = base.strip()
    assert len(base.split(",")) == 83
    assert hashlib.sha256((base + "\n").encode()).hexdigest() == (
        "396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50"
    )

    assert colored.startswith(",")
    colored_header = base + colored
    assert len(colored_header.split(",")) == 95
    assert hashlib.sha256((colored_header + "\n").encode()).hexdigest() == (
        "424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5"
    )


def test_impact_extension_is_opt_in_and_preregistered_gates_are_frozen():
    source = _source()
    preregistration = PREREGISTRATION.read_text(encoding="utf-8")

    assert "if (gAppendLiteralCrownImpactColumns)" in source
    assert (
        "gAppendLiteralCrownImpactColumns\n"
        "      = scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1;"
        in source
    )
    for token in (
        "kLiteralCrownImpactMinimumCrownResponse = 0.0001",
        "kLiteralCrownImpactMaximumBodyDisplacement = 0.07",
        "kLiteralCrownImpactMinimumOrientationAlignment\n" "    = 0.8660254037844386",
        "kLiteralCrownImpactMaximumFarFieldDisplacement = 0.007",
        "kLiteralCrownImpactSpringerTolerance = 1e-12",
        "kLiteralCrownImpactFarFieldAdjacentPairCount = 16u",
    ):
        assert token in source

    assert "does not establish paper parity" in preregistration
    assert "Do not change these parameters or the thresholds" in preregistration
    assert "scientific negative" in preregistration
