"""Tests for scripts/check_plan104_paper_parity.py."""

import copy
import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_plan104_paper_parity.py"
CONTRACT_DIR = ROOT / "docs" / "plans" / "104-vertex-block-descent-solver"
VBD_CONTRACT = CONTRACT_DIR / "vbd-paper-coverage-contract.json"
AVBD_CONTRACT = CONTRACT_DIR / "avbd-paper-coverage-contract.json"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_plan104_paper_parity", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_contract(path):
    return json.loads(path.read_text())


def _write_contract(tmp_path, contract):
    path = tmp_path / f"{contract['solver_family']}-contract.json"
    path.write_text(json.dumps(contract, indent=2) + "\n")
    return path


def _errors_for_mutation(tmp_path, path, mutate):
    module = _load_module()
    contract = _load_contract(path)
    mutate(contract)
    return module.contract_errors(_write_contract(tmp_path, contract))


def test_committed_contracts_pass_and_cover_176_rows(capsys):
    module = _load_module()
    assert module.main([]) == 0
    output = capsys.readouterr().out
    assert "VBD: 88 rows" in output
    assert "AVBD: 88 rows" in output
    assert "176 canonical rows" in output


def test_removing_a_canonical_requirement_is_rejected(tmp_path):
    errors = _errors_for_mutation(
        tmp_path,
        VBD_CONTRACT,
        lambda contract: contract["coverage_groups"][2]["requirements"].pop(),
    )
    assert any("requirement ids/order" in error for error in errors), errors


@pytest.mark.parametrize("path", (VBD_CONTRACT, AVBD_CONTRACT))
def test_every_canonical_requirement_is_guarded_against_deletion(path):
    module = _load_module()
    baseline = _load_contract(path)
    for group_index, group in enumerate(baseline["coverage_groups"]):
        for requirement_index, requirement in enumerate(group["requirements"]):
            contract = copy.deepcopy(baseline)
            contract["coverage_groups"][group_index]["requirements"].pop(
                requirement_index
            )
            errors = module.validate_contract(
                contract, source_name=f"without {requirement['id']}"
            )
            assert any("requirement ids/order" in error for error in errors), (
                requirement["id"],
                errors,
            )


def test_mutating_an_authoritative_source_pin_is_rejected(tmp_path):
    def mutate(contract):
        contract["sources"]["paper"]["sha256"] = "0" * 64

    errors = _errors_for_mutation(tmp_path, AVBD_CONTRACT, mutate)
    assert any("source paper.sha256" in error for error in errors), errors


@pytest.mark.parametrize("path", (VBD_CONTRACT, AVBD_CONTRACT))
def test_every_authoritative_source_value_is_guarded(path):
    module = _load_module()
    baseline = _load_contract(path)
    family = baseline["solver_family"]
    for source_path in module.EXPECTED_SOURCE_VALUES[family]:
        contract = copy.deepcopy(baseline)
        value = contract["sources"]
        for key in source_path[:-1]:
            value = value[key]
        value[source_path[-1]] = None
        errors = module.validate_contract(
            contract, source_name=f"mutated {'.'.join(source_path)}"
        )
        expected_fragment = f"source {'.'.join(source_path)}"
        assert any(expected_fragment in error for error in errors), (
            source_path,
            errors,
        )


@pytest.mark.parametrize("path", (VBD_CONTRACT, AVBD_CONTRACT))
def test_every_predicate_definition_is_guarded(path):
    module = _load_module()
    baseline = _load_contract(path)
    for predicate in baseline["predicate_definitions"]:
        contract = copy.deepcopy(baseline)
        contract["predicate_definitions"][predicate] = "weakened"
        errors = module.validate_contract(contract, source_name=f"mutated {predicate}")
        assert any(
            f"predicate definition {predicate!r} must be" in error for error in errors
        ), (predicate, errors)


@pytest.mark.parametrize("path", (VBD_CONTRACT, AVBD_CONTRACT))
def test_every_authoritative_source_locator_is_guarded(path):
    module = _load_module()
    baseline = _load_contract(path)
    for group_index, group in enumerate(baseline["coverage_groups"]):
        for requirement_index, requirement in enumerate(group["requirements"]):
            contract = copy.deepcopy(baseline)
            contract["coverage_groups"][group_index]["requirements"][requirement_index][
                "source_locator"
            ] += " drift"
            errors = module.validate_contract(
                contract, source_name=f"mutated {requirement['id']}"
            )
            assert any(
                "source-locator inventory digest must be" in error for error in errors
            ), (requirement["id"], errors)


def test_missing_row_cannot_claim_evidence(tmp_path):
    def mutate(contract):
        row = contract["coverage_groups"][0]["requirements"][12]
        assert row["status"] == "missing"
        row["evidence"] = ["docs/plans/104-vertex-block-descent-solver/README.md"]

    errors = _errors_for_mutation(tmp_path, VBD_CONTRACT, mutate)
    assert any("missing rows cannot claim evidence" in error for error in errors)


def test_partial_row_requires_evidence(tmp_path):
    def mutate(contract):
        row = contract["coverage_groups"][0]["requirements"][0]
        assert row["status"] == "partial"
        row["evidence"] = []

    errors = _errors_for_mutation(tmp_path, AVBD_CONTRACT, mutate)
    assert any("partial rows require evidence" in error for error in errors)


def test_complete_row_requires_all_predicates_and_backends(tmp_path):
    def mutate(contract):
        row = contract["coverage_groups"][0]["requirements"][0]
        row["status"] = "complete"
        row["blockers"] = []

    errors = _errors_for_mutation(tmp_path, AVBD_CONTRACT, mutate)
    assert any("predicate_results is required" in error for error in errors), errors
    assert any("backend_results is required" in error for error in errors), errors


def test_incomplete_row_cannot_set_claim_valid_true(tmp_path):
    def mutate(contract):
        row = contract["coverage_groups"][0]["requirements"][0]
        row["predicate_results"] = {"claim_valid": True}

    errors = _errors_for_mutation(tmp_path, VBD_CONTRACT, mutate)
    assert any("cannot set claim_valid true" in error for error in errors)


def test_video_timecode_discontinuity_is_rejected(tmp_path):
    def mutate(contract):
        contract["coverage_groups"][3]["requirements"][1]["source_seconds"][0] += 1

    errors = _errors_for_mutation(tmp_path, AVBD_CONTRACT, mutate)
    assert any("source_seconds must be" in error for error in errors), errors


def test_project_media_duration_drift_is_rejected(tmp_path):
    def mutate(contract):
        contract["coverage_groups"][-1]["requirements"][0]["source_seconds"][1] = 1

    errors = _errors_for_mutation(tmp_path, VBD_CONTRACT, mutate)
    assert any(
        "vbd.project.teaser: source_seconds must be" in error for error in errors
    )


def test_nonexistent_evidence_path_is_rejected(tmp_path):
    def mutate(contract):
        row = contract["coverage_groups"][0]["requirements"][0]
        row["evidence"] = ["this/path/does/not/exist.md"]

    errors = _errors_for_mutation(tmp_path, VBD_CONTRACT, mutate)
    assert any("evidence path does not exist" in error for error in errors)


def test_overall_complete_cannot_be_asserted_while_rows_are_incomplete(tmp_path):
    def mutate(contract):
        contract["overall_status"] = "complete"

    errors = _errors_for_mutation(tmp_path, AVBD_CONTRACT, mutate)
    assert any("overall_status must be 'incomplete'" in error for error in errors)


def test_prose_matrix_count_drift_is_rejected(tmp_path):
    module = _load_module()
    matrix = tmp_path / "paper-parity-matrix.md"
    matrix.write_text("| VBD method | 1 | 0 | 0 | 1 | 0 |\n")
    contracts = (_load_contract(VBD_CONTRACT), _load_contract(AVBD_CONTRACT))
    errors = module.matrix_errors(contracts, matrix_path=matrix)
    assert any("VBD method" in error for error in errors), errors
    assert any("Grand total" in error for error in errors), errors


def test_contract_mutations_do_not_change_committed_fixture():
    before = _load_contract(VBD_CONTRACT)
    mutated = copy.deepcopy(before)
    mutated["overall_status"] = "complete"
    assert _load_contract(VBD_CONTRACT) == before
