import importlib.util
import sys
from pathlib import Path


def _load_script(name):
    script = Path(__file__).resolve().parents[1] / "scripts" / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, script)
    module = importlib.util.module_from_spec(spec)
    assert spec is not None
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_rigid_ipc_classifier_marks_paper_figures_as_visual_examples():
    module = _load_script("generate_rigid_ipc_fixture_manifest")

    row = module.classify_entry(
        "fixtures/paper-figures/01-expanding-lock-box.json", "fixture"
    )

    assert row["family"] == "paper-figure"
    assert row["dart_target_type"] == "example"
    assert row["visual_evidence_requirement"] == "headless-filament-required"


def test_rigid_ipc_classifier_marks_ccd_data_as_tests():
    module = _load_script("generate_rigid_ipc_fixture_manifest")

    row = module.classify_entry(
        "tests/data/wrecking-ball/ccd-test-001.json", "test-data"
    )

    assert row["family"] == "ccd-data-wrecking-ball"
    assert row["dart_target_type"] == "test"
    assert "time of impact" in row["expected_invariant"]


def test_rigid_ipc_classifier_marks_large_hashgrid_data_as_benchmarks():
    module = _load_script("generate_rigid_ipc_fixture_manifest")

    row = module.classify_entry(
        "tests/data/large-rb-hashgrid/large-rb-hashgrid-000.json", "test-data"
    )

    assert row["family"] == "ccd-data-large-rb-hashgrid"
    assert row["dart_target_type"] == "benchmark"
    assert row["benchmark_profile_artifact"] == "hash-grid benchmark JSON packet"
    assert "time of impact" not in row["expected_invariant"]
    assert "broad-phase" in row["topic"]


def test_rigid_ipc_manifest_marks_audited_root_ccd_rows_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "tests" / "data" / "ccd-test-000.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"type":"ee"}')

    row = module.row_for_path("tests/data/ccd-test-000.json", "test-data", tmp_path)

    assert row["status"] == "implemented"
    assert "EvaluatesAuditedUpstreamRootCcdRowsAsMisses" in row["dart_artifact"]
    assert "full-step miss outcome" in row["expected_invariant"]
    assert "corpus-scale interval-root parity remains tracked" in row["notes_or_gap"]


def test_rigid_ipc_manifest_leaves_uncovered_ccd_rows_planned(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "tests" / "data" / "kinematic" / "ccd-test-013.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"type":"ee"}')

    row = module.row_for_path(
        "tests/data/kinematic/ccd-test-013.json", "test-data", tmp_path
    )

    assert row["status"] == "planned"


def test_rigid_ipc_manifest_marks_large_hashgrid_rows_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path / "tests" / "data" / "large-rb-hashgrid" / "large-rb-hashgrid-000.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"bodies":[]}')

    row = module.row_for_path(
        "tests/data/large-rb-hashgrid/large-rb-hashgrid-000.json",
        "test-data",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert row["dart_target_type"] == "benchmark"
    assert "BM_RigidIpcLargeHashgridSceneBounds" in row["dart_artifact"]
    assert "bm_rigid_ipc_solver" in row["dart_command_or_ctest_or_benchmark"]
    assert "upstream exact scene bounds" in row["expected_invariant"]
    assert "source hashes" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_first_kinematic_ccd_rows_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "tests" / "data" / "kinematic" / "ccd-test-000.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"type":"ee"}')

    row = module.row_for_path(
        "tests/data/kinematic/ccd-test-000.json", "test-data", tmp_path
    )

    assert row["status"] == "implemented"
    assert "EvaluatesAuditedKinematicRowsWithoutZeroTimeHits" in row["dart_artifact"]
    assert "zero-time hits" in row["expected_invariant"]


def test_rigid_ipc_manifest_marks_first_wrecking_ball_ccd_rows_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "tests" / "data" / "wrecking-ball" / "ccd-test-000.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"type":"ee"}')

    row = module.row_for_path(
        "tests/data/wrecking-ball/ccd-test-000.json", "test-data", tmp_path
    )

    assert row["status"] == "implemented"
    assert "EvaluatesAuditedWreckingBallCorpusConservatively" in row["dart_artifact"]
    assert "truncated interval" in row["expected_invariant"]


def test_rigid_ipc_manifest_marks_below_threshold_friction_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path
        / "fixtures"
        / "3D"
        / "friction"
        / "incline-plane"
        / "slopeTest_highSchoolPhysics_mu=0.49.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"coefficient_friction":0.49}}')

    row = module.row_for_path(
        "fixtures/3D/friction/incline-plane/slopeTest_highSchoolPhysics_mu=0.49.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "FrictionThresholdBelowFixtureRowSlides" in row["dart_artifact"]
    assert "mu=0.49" in row["expected_invariant"]
    assert "mu=0.5 row" in row["notes_or_gap"]


def test_rigid_ipc_manifest_leaves_at_threshold_friction_row_planned(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path
        / "fixtures"
        / "3D"
        / "friction"
        / "incline-plane"
        / "slopeTest_highSchoolPhysics_mu=0.5.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"coefficient_friction":0.5}}')

    row = module.row_for_path(
        "fixtures/3D/friction/incline-plane/slopeTest_highSchoolPhysics_mu=0.5.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "planned"


def test_rigid_ipc_manifest_marks_high_friction_row_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path
        / "fixtures"
        / "3D"
        / "friction"
        / "incline-plane"
        / "slopeTest_highSchoolPhysics_mu=1.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"coefficient_friction":1.0}}')

    row = module.row_for_path(
        "fixtures/3D/friction/incline-plane/slopeTest_highSchoolPhysics_mu=1.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "FrictionThresholdHighFixtureRowSticks" in row["dart_artifact"]
    assert "mu=1.0" in row["expected_invariant"]
    assert "mu=0.5 row" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_sliding_friction_row_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "fixtures" / "3D" / "friction" / "sliding.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"coefficient_friction":0.05}}')

    row = module.row_for_path(
        "fixtures/3D/friction/sliding.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "SlidingCubeFixtureRowIsBrakedByFriction" in row["dart_artifact"]
    assert "mu=0.05" in row["expected_invariant"]
    assert "differential sliding-cube" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_spolling_coin_row_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "fixtures" / "3D" / "friction" / "spolling-coin.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"coefficient_friction":0.2}}')

    row = module.row_for_path(
        "fixtures/3D/friction/spolling-coin.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "SpinningCoinIsBrakedByFrictionWithoutPenetration" in row["dart_artifact"]
    assert "angular velocity" in row["expected_invariant"]
    assert "visual alias remains planned" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_high_friction_turntable_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path
        / "fixtures"
        / "3D"
        / "friction"
        / "turntable"
        / "turntable-mu=1.0.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"coefficient_friction":1.0}}')

    row = module.row_for_path(
        "fixtures/3D/friction/turntable/turntable-mu=1.0.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "TurntableHighFrictionFixtureRowCarriesRider" in row["dart_artifact"]
    assert "mu=1.0" in row["expected_invariant"]
    assert "mu=0.0" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_moderate_friction_turntable_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path
        / "fixtures"
        / "3D"
        / "friction"
        / "turntable"
        / "turntable-mu=0.5.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"coefficient_friction":0.5}}')

    row = module.row_for_path(
        "fixtures/3D/friction/turntable/turntable-mu=0.5.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "TurntableModerateFrictionFixtureRowCarriesRider" in row["dart_artifact"]
    assert "mu=0.5" in row["expected_invariant"]
    assert "mu=0.0" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_low_friction_turntable_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path
        / "fixtures"
        / "3D"
        / "friction"
        / "turntable"
        / "turntable-mu=0.1.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"coefficient_friction":0.1}}')

    row = module.row_for_path(
        "fixtures/3D/friction/turntable/turntable-mu=0.1.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "TurntableLowFrictionFixtureRowCarriesRider" in row["dart_artifact"]
    assert "mu=0.1" in row["expected_invariant"]
    assert "mu=0.0" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_tunneling_unit_row_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "fixtures" / "3D" / "unit-tests" / "tunneling.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/tunneling.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "HighSpeedCubeDoesNotTunnelThroughWall" in row["dart_artifact"]
    assert "line-search hit" in row["expected_invariant"]
    assert "3D unit-test fixture mechanism" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_erleben_cliff_edges_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path / "fixtures" / "3D" / "unit-tests" / "erleben" / "cliff-edges.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/erleben/cliff-edges.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "ErlebenCliffEdgesFixtureRowStaysSeparated" in row["dart_artifact"]
    assert "Erleben cliff-edges row" in row["expected_invariant"]
    assert "Other Erleben rows remain planned" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_erleben_internal_edges_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path / "fixtures" / "3D" / "unit-tests" / "erleben" / "internal-edges.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/erleben/internal-edges.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "ErlebenInternalEdgesFixtureRowStaysSeparated" in row["dart_artifact"]
    assert "Erleben internal-edges row" in row["expected_invariant"]
    assert "Other Erleben rows remain planned" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_erleben_sliding_spike_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path / "fixtures" / "3D" / "unit-tests" / "erleben" / "sliding-spike.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/erleben/sliding-spike.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "ErlebenSlidingSpikeFixtureRowStaysSeparated" in row["dart_artifact"]
    assert "Erleben sliding-spike row" in row["expected_invariant"]
    assert "Other Erleben rows remain planned" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_erleben_sliding_wedge_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path / "fixtures" / "3D" / "unit-tests" / "erleben" / "sliding-wedge.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/erleben/sliding-wedge.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "ErlebenSlidingWedgeFixtureRowStaysSeparated" in row["dart_artifact"]
    assert "Erleben sliding-wedge row" in row["expected_invariant"]
    assert "Other Erleben rows remain planned" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_erleben_spikes_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "fixtures" / "3D" / "unit-tests" / "erleben" / "spikes.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/erleben/spikes.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "ErlebenSpikesFixtureRowStaysSeparated" in row["dart_artifact"]
    assert "Erleben spikes row" in row["expected_invariant"]
    assert "Other Erleben rows remain planned" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_erleben_wedges_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "fixtures" / "3D" / "unit-tests" / "erleben" / "wedges.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/erleben/wedges.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "ErlebenWedgesFixtureRowStaysSeparated" in row["dart_artifact"]
    assert "Erleben wedges row" in row["expected_invariant"]
    assert "Other Erleben rows remain planned" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_erleben_spike_and_wedge_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path / "fixtures" / "3D" / "unit-tests" / "erleben" / "spike-and-wedge.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/erleben/spike-and-wedge.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "ErlebenSpikeAndWedgeFixtureRowStaysSeparated" in row["dart_artifact"]
    assert "Erleben spike-and-wedge row" in row["expected_invariant"]
    assert "Crack and hole Erleben rows remain planned" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_two_triangle_plane_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path
        / "fixtures"
        / "3D"
        / "unit-tests"
        / "tessellated-plane"
        / "two-triangles.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/tessellated-plane/two-triangles.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "CubeSettlesOnTwoTrianglePlaneFixtureRow" in row["dart_artifact"]
    assert "two-triangle mesh plane" in row["expected_invariant"]
    assert "8K tessellated-plane row remains planned" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_two_triangle_tet_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path
        / "fixtures"
        / "3D"
        / "unit-tests"
        / "tessellated-plane"
        / "two-triangles-tet.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/tessellated-plane/two-triangles-tet.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "TetCornerFallsOnTwoTrianglePlaneFixtureRow" in row["dart_artifact"]
    assert "two-triangle tet row" in row["expected_invariant"]
    assert "8K tessellated-plane row remains planned" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_large_mass_ratio_rows_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    cases = {
        "fixtures/3D/unit-tests/large-mass-ratio.json": (
            "3D large-mass-ratio row",
            "non-visual Fig. 16 paper-unit alias",
        ),
        "fixtures/paper-figures/16-unit-tests/large-mass-ratio.json": (
            "Fig. 16 large-mass-ratio unit-test alias",
            "Visual paper-figure rows still require",
        ),
    }

    for path, (invariant_text, notes_text) in cases.items():
        data_path = tmp_path / path
        data_path.parent.mkdir(parents=True, exist_ok=True)
        data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

        row = module.row_for_path(path, "fixture", tmp_path)

        assert row["status"] == "implemented"
        assert "LargeMassRatioFixtureRowStaysSeparated" in row["dart_artifact"]
        assert invariant_text in row["expected_invariant"]
        assert notes_text in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_rotating_cube_unit_row_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path / "fixtures" / "3D" / "unit-tests" / "rotation" / "rotating-cube.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/rotation/rotating-cube.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "RotatingCubeFixtureRowAdvancesWithoutContact" in row["dart_artifact"]
    assert "does not translate" in row["expected_invariant"]
    assert "rotating-cube runtime coverage" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_spinning_cube_over_plane_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path / "fixtures" / "3D" / "unit-tests" / "spinning-cube-over-plane.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/spinning-cube-over-plane.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "SpinningCubeOverPlaneFixtureRowAdvancesSafely" in row["dart_artifact"]
    assert "preserves nonnegative clearance" in row["expected_invariant"]
    assert "spinning-cube runtime coverage" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_ellipsoid_rotation_rows_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    cases = {
        "fixtures/3D/unit-tests/rotation/rotating-sphere.json": (
            "RotatingScaledSphereFixtureRowAdvancesWithoutContact",
            "scaled-sphere",
        ),
        "fixtures/3D/unit-tests/rotation/rotating-ellipsoid-major.json": (
            "RotatingEllipsoidMajorFixtureRowAdvancesWithoutContact",
            "major-axis",
        ),
        "fixtures/3D/unit-tests/rotation/rotating-ellipsoid-intermediate.json": (
            "RotatingEllipsoidIntermediateFixtureRowAdvancesWithoutContact",
            "intermediate-axis",
        ),
        "fixtures/3D/unit-tests/rotation/rotating-ellipsoid-minor.json": (
            "RotatingEllipsoidMinorFixtureRowAdvancesWithoutContact",
            "minor-axis",
        ),
    }

    for path, (test_name, invariant_text) in cases.items():
        data_path = tmp_path / path
        data_path.parent.mkdir(parents=True, exist_ok=True)
        data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

        row = module.row_for_path(path, "fixture", tmp_path)

        assert row["status"] == "implemented"
        assert test_name in row["dart_artifact"]
        assert invariant_text in row["expected_invariant"]
        assert "3D rotation fixture mechanism" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_torque_rotation_row_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path / "fixtures" / "3D" / "unit-tests" / "rotation" / "torque-test.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/rotation/torque-test.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "TorqueFixtureRowAcceleratesFreeBody" in row["dart_artifact"]
    assert "gains angular velocity" in row["expected_invariant"]
    assert "Contacting 3D unit-test fixture rows" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_dzhanibekov_rotation_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path / "fixtures" / "3D" / "unit-tests" / "rotation" / "dzhanibekov.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"rigid_bodies":[]}}')

    row = module.row_for_path(
        "fixtures/3D/unit-tests/rotation/dzhanibekov.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "DzhanibekovWingNutFixtureRowAdvancesSafely" in row["dart_artifact"]
    assert "wing-nut-like rigid mesh" in row["expected_invariant"]
    assert "wing-nut runtime coverage" in row["notes_or_gap"]


def test_rigid_ipc_manifest_marks_below_threshold_paper_alias_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = (
        tmp_path
        / "fixtures"
        / "paper-figures"
        / "18-high-school-physics-friction-test-mu=0.49.json"
    )
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"rigid_body_problem":{"coefficient_friction":0.49}}')

    row = module.row_for_path(
        "fixtures/paper-figures/18-high-school-physics-friction-test-mu=0.49.json",
        "fixture",
        tmp_path,
    )

    assert row["status"] == "implemented"
    assert "FrictionThresholdBelowFixtureRowSlides" in row["dart_artifact"]


def test_rigid_ipc_manifest_marks_barrier_source_row_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")

    row = module.row_for_path(
        "tests/barrier/test_barriers.cpp", "test-source", tmp_path
    )

    assert row["status"] == "implemented"
    assert "IpcBarrierKernel" in row["dart_artifact"]
    assert "finite-difference derivatives" in row["expected_invariant"]


def test_rigid_ipc_manifest_marks_rigid_toi_source_row_implemented(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")

    row = module.row_for_path(
        "tests/ccd/test_rigid_body_time_of_impact.cpp", "test-source", tmp_path
    )

    assert row["status"] == "implemented"
    assert "RigidIpcCcdCase" in row["dart_artifact"]
    assert "rotational trajectory" in row["expected_invariant"]


def test_rigid_ipc_manifest_marks_rigid_hash_grid_source_row_implemented(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")

    row = module.row_for_path(
        "tests/ccd/test_rigid_body_hash_grid.cpp", "test-source", tmp_path
    )

    assert row["status"] == "implemented"
    assert "BM_RigidIpcLargeHashgridSceneBounds" in row["dart_artifact"]
    assert "large rigid-body hash-grid corpus" in row["expected_invariant"]
    assert "test_hash_grid.cpp row" in row["notes_or_gap"]


def test_rigid_ipc_manifest_leaves_synthetic_wrecking_ball_ccd_rows_planned(
    tmp_path,
):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    data_path = tmp_path / "tests" / "data" / "wrecking-ball" / "ccd-test-386.json"
    data_path.parent.mkdir(parents=True)
    data_path.write_text('{"type":"ee"}')

    row = module.row_for_path(
        "tests/data/wrecking-ball/ccd-test-386.json", "test-data", tmp_path
    )

    assert row["status"] == "planned"


def test_rigid_ipc_asset_normalization_prefers_mesh_root(tmp_path):
    module = _load_script("generate_rigid_ipc_fixture_manifest")
    (tmp_path / "meshes").mkdir()
    (tmp_path / "meshes" / "cube.obj").write_text("o cube\n")

    assert module.normalize_asset("cube.obj", tmp_path) == "meshes/cube.obj"


def test_rigid_ipc_validator_rejects_unclassified_rows():
    module = _load_script("check_rigid_ipc_fixture_manifest")
    manifest = {
        "schema_version": 1,
        "source": {"commit": module.EXPECTED_COMMIT},
        "summary": {
            "source_kind_counts": {"fixture": 1},
            "family_counts": {"unclassified": 1},
            "target_type_counts": {"unclassified": 1},
        },
        "entries": [
            {
                "upstream_path": "fixtures/bad.json",
                "upstream_commit": module.EXPECTED_COMMIT,
                "source_kind": "fixture",
                "alias_of": "",
                "family": "unclassified",
                "topic": "missing",
                "priority": "P0",
                "dart_target_type": "unclassified",
                "status": "planned",
                "dart_artifact": "missing",
                "required_assets_or_importer": [],
                "expected_invariant": "missing",
                "dart_command_or_ctest_or_benchmark": "missing",
                "visual_evidence_requirement": "not-required",
                "benchmark_profile_artifact": "not-required",
                "notes_or_gap": "test row",
            }
        ],
    }

    errors = module.validate_manifest(manifest, strict_counts=False)

    assert any("dart_target_type is unclassified" in error for error in errors)
    assert any("missing family" in error for error in errors)
