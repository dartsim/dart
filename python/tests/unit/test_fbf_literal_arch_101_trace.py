import hashlib
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TRACE_SOURCE = ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp"


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


def test_literal_arch_101_parser_default_and_fail_closed_cli_contract():
    source = _source()

    assert "MasonryArch101LiteralWedge" in source
    assert 'name == "masonry_arch_101_literal_wedge"' in source
    assert (
        "case Scenario::MasonryArch101LiteralWedge:\n"
        "      return kLiteralArchDefaultSteps;" in source
    )
    assert "constexpr std::size_t kLiteralArchDefaultSteps = 600u;" in source
    assert re.search(
        r"isLiteralWedgeScenario\(scenario\)\s*"
        r"&& collisionFrontend != CollisionFrontend::Native",
        source,
    )
    assert re.search(
        r"scenario == Scenario::MasonryArch101LiteralWedge\s*"
        r"&& solverMode != SolverMode::ExactFbf",
        source,
    )


def test_literal_arch_builder_is_shared_and_preserves_source_geometry_contract():
    source = _source()
    builder_match = re.search(
        r"void addLiteralMasonryArch\(.*?\n\}\n\n" r"void addLiteralMasonryArch25",
        source,
        re.DOTALL,
    )
    assert builder_match is not None
    builder = builder_match.group(0)

    assert "generateMasonryArchStoneWedges(\n      stoneCount," in builder
    assert "MasonryArchBarrierGapPolicy::OmitSourceOffsets" in builder
    assert "kLiteralArchEndFaceExpansion" in builder
    assert "if (i == 0u || i + 1u == geometries.size())" in builder
    assert "stone->setMobile(false);" in builder
    assert re.search(
        r"case Scenario::MasonryArch101LiteralWedge:\s*"
        r"addLiteralMasonryArch\(world, kArch101StoneCount\);",
        source,
    )

    assert "const double mass = kArchDensity * geometry.volume;" in source
    assert "inertia.setMoment(mass * geometry.momentPerUnitMass);" in source
    assert "kLiteralArchEndFaceExpansion = 1e-6" in source
    assert "kLiteralArchDownwardShift = 0.001001" in source
    assert "kArchFriction = 0.8" in source


def test_literal_arch_101_colored_profile_rejects_outer_iteration_caps():
    source = _source()
    colored_profile = re.search(
        r"if \(isLiteralWedgeScenario\(scenario\)\) \{(.*?)\n    \}",
        source,
        re.DOTALL,
    )
    assert colored_profile is not None
    profile = colored_profile.group(1)

    assert "options.maxOuterIterations = 5000;" in profile
    assert "options.innerMaxSweeps = 30;" in profile
    assert "options.enableWarmStart = true;" in profile
    assert "options.enableStepSizePersistence = false;" in profile
    assert re.search(
        r"if \(scenario == Scenario::MasonryArch101LiteralWedge\) \{.*?"
        r"options.acceptOuterMaxIterations = false;",
        profile,
        re.DOTALL,
    )


def test_literal_arch_101_has_all_body_metrics_without_impact_extension():
    source = _source()

    assert re.search(
        r"scenario == Scenario::MasonryArch101LiteralWedge\s*"
        r"\? kArch101StoneCount\s*:\s*kArchStoneCount",
        source,
    )
    assert "collectArchOutcomeMetrics(initialArchPoses)" in source
    assert "max_arch_body_displacement_from_initial" in source
    assert "min_arch_body_orientation_alignment_from_initial" in source
    assert (
        "gAppendLiteralCrownImpactColumns\n"
        "      = scenario == Scenario::MasonryArch25LiteralWedgeCrownImpactV1;"
        in source
    )


def test_literal_arch_101_keeps_standing_performance_schemas_byte_identical():
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
