"""Tests for scripts/check_plan104_paper_parity.py."""

import copy
import hashlib
import importlib.util
import json
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_plan104_paper_parity.py"
CONTRACT_DIR = ROOT / "docs" / "plans" / "104-vertex-block-descent-solver"
VBD_CONTRACT = CONTRACT_DIR / "vbd-paper-coverage-contract.json"
AVBD_CONTRACT = CONTRACT_DIR / "avbd-paper-coverage-contract.json"

CLOSURE_PACKET_KEYS = ("benchmark", "source_provenance", "visual_evidence")
CLOSURE_SOURCE_PATHS = (
    "scripts/check_plan104_paper_parity.py",
    "tests/test_check_plan104_paper_parity.py",
)
CLOSURE_CAPTURE_ROLES = ("outcome", "semantic_review")
CLOSURE_BENCHMARK_METHODS = ("BM_ClosureTestStep",)
SOURCE_PROVENANCE_ALGORITHM = "sha256-length-prefixed-path-and-content-v1"


def _closure_source_provenance():
    """Build a source_provenance block the generic packet validator accepts."""
    combined = hashlib.sha256()
    files = []
    for source_path in CLOSURE_SOURCE_PATHS:
        payload = (ROOT / source_path).read_bytes()
        encoded_path = source_path.encode("utf-8")
        combined.update(struct.pack("<Q", len(encoded_path)))
        combined.update(encoded_path)
        combined.update(struct.pack("<Q", len(payload)))
        combined.update(payload)
        files.append(
            {"path": source_path, "sha256": hashlib.sha256(payload).hexdigest()}
        )
    return {
        "algorithm": SOURCE_PROVENANCE_ALGORITHM,
        "digest": combined.hexdigest(),
        "files": files,
    }


def _load_module(*, validate_packets=True):
    spec = importlib.util.spec_from_file_location("check_plan104_paper_parity", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    if not validate_packets:
        module.EVIDENCE_VALIDATORS[module.AVBD_PACKET_VALIDATOR] = (
            lambda _path, _context: []
        )
    return module


def _load_contract(path):
    return json.loads(path.read_text())


def _write_contract(tmp_path, contract):
    path = tmp_path / f"{contract['solver_family']}-contract.json"
    path.write_text(json.dumps(contract, indent=2) + "\n")
    return path


def _errors_for_mutation(tmp_path, path, mutate):
    module = _load_module(validate_packets=False)
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
    module = _load_module(validate_packets=False)
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
    module = _load_module(validate_packets=False)
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
    module = _load_module(validate_packets=False)
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
    module = _load_module(validate_packets=False)
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
    assert any("complete rows require closure_evidence" in error for error in errors)


def test_non_string_status_fails_closed():
    module = _load_module(validate_packets=False)
    requirement = {
        "id": "avbd.method.cpu_solver",
        "status": [],
        "source_locator": "test source",
        "evidence": [],
        "blockers": ["malformed status"],
    }

    errors = module._requirement_errors(
        requirement,
        expected_id=requirement["id"],
        required_predicates=module.METHOD_PREDICATES,
        context=module._ValidationContext(),
    )

    assert any("invalid status" in error for error in errors), errors


def test_non_string_result_map_key_fails_closed():
    module = _load_module(validate_packets=False)

    errors = module._result_map_errors(
        {0: True},
        allowed_keys=module.METHOD_PREDICATES,
        label="test results",
        require_all_true=False,
    )

    assert any("has unknown keys" in error for error in errors), errors


def test_non_string_solver_family_fails_closed():
    module = _load_module(validate_packets=False)

    errors = module.validate_contract({"solver_family": []})

    assert any("solver_family must be one of" in error for error in errors), errors


@pytest.mark.parametrize(
    "payload",
    (
        b'{"solver_family": ' + b"1" * 5000 + b"}\n",
        b"\xff\xfe\x00",
    ),
)
def test_contract_parser_failures_are_reported_as_invalid_json(tmp_path, payload):
    module = _load_module(validate_packets=False)
    path = tmp_path / "malformed-contract.json"
    path.write_bytes(payload)

    errors = module.contract_errors(path)

    assert any("invalid JSON" in error for error in errors), errors


@pytest.mark.parametrize(
    "payload",
    (
        b'{"solver_family":"vbd","solver_family":"avbd"}\n',
        b'{"solver_family":"vbd","source":NaN}\n',
        b'{"solver_family":"vbd","source":Infinity}\n',
    ),
)
def test_contract_parser_rejects_ambiguous_or_nonstandard_json(tmp_path, payload):
    module = _load_module(validate_packets=False)
    path = tmp_path / "ambiguous-contract.json"
    path.write_bytes(payload)

    errors = module.contract_errors(path)

    assert any("invalid JSON" in error for error in errors), errors


@pytest.mark.parametrize("hostile_path", ("bad\x00path", "bad\ud800path"))
def test_contract_hostile_evidence_paths_fail_closed(tmp_path, hostile_path):
    module = _load_module(validate_packets=False)
    contract = _load_contract(VBD_CONTRACT)
    row = contract["coverage_groups"][0]["requirements"][0]
    row["evidence"] = [hostile_path]

    errors = module.contract_errors(_write_contract(tmp_path, contract))

    assert any("evidence path must be repository-relative" in error for error in errors)


def test_contract_huge_video_number_fails_closed(tmp_path):
    module = _load_module(validate_packets=False)
    contract = _load_contract(VBD_CONTRACT)
    video_row = next(
        row
        for group in contract["coverage_groups"]
        for row in group["requirements"]
        if "source_seconds" in row
    )
    video_row["source_seconds"][0] = 10**309

    errors = module.contract_errors(_write_contract(tmp_path, contract))

    assert any("source_seconds must be two finite numbers" in error for error in errors)


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("required_backends", None, "required_backends must be"),
        ("required_predicates", 1, "required_predicates must be"),
        ("requirements", None, "requirements must be a list"),
    ),
)
def test_malformed_group_collection_fails_closed(field, value, message):
    module = _load_module(validate_packets=False)
    contract = _load_contract(AVBD_CONTRACT)
    contract["coverage_groups"][0][field] = value

    errors = module.validate_contract(contract)

    assert any(message in error for error in errors), errors


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


def test_prose_matrix_duplicate_expected_row_is_rejected(tmp_path):
    module = _load_module()
    matrix = tmp_path / "paper-parity-matrix.md"
    matrix.write_text(
        "| Grand total | 1 | 0 | 0 | 1 | 0 |\n" + module.MATRIX_PATH.read_text()
    )
    contracts = (_load_contract(VBD_CONTRACT), _load_contract(AVBD_CONTRACT))

    errors = module.matrix_errors(contracts, matrix_path=matrix)

    assert any(
        "Grand total" in error and "must appear exactly once" in error
        for error in errors
    ), errors


def test_prose_matrix_unexpected_numeric_row_is_rejected(tmp_path):
    module = _load_module()
    matrix = tmp_path / "paper-parity-matrix.md"
    matrix.write_text(
        module.MATRIX_PATH.read_text() + "\n| Invented total | 1 | 0 | 0 | 1 | 0 |\n"
    )
    contracts = (_load_contract(VBD_CONTRACT), _load_contract(AVBD_CONTRACT))

    errors = module.matrix_errors(contracts, matrix_path=matrix)

    assert any(
        "unexpected numeric table row 'Invented total'" in error for error in errors
    ), errors


def test_contract_mutations_do_not_change_committed_fixture():
    before = _load_contract(VBD_CONTRACT)
    mutated = copy.deepcopy(before)
    mutated["overall_status"] = "complete"
    assert _load_contract(VBD_CONTRACT) == before


def _write_closure_packet(
    module,
    tmp_path,
    requirement_id,
    predicates,
    backends,
    *,
    solver_family="avbd",
):
    relative = (
        module.CONTRACT_DIR_RELATIVE / f"{solver_family}-test-closure-packet.json"
    )
    path = tmp_path / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    artifact = tmp_path / "artifacts" / "closure-sentinel.txt"
    artifact.parent.mkdir(parents=True, exist_ok=True)
    artifact.write_text("substantive closure artifact\n")
    packet = {
        "schema_version": module.AVBD_PACKET_SCHEMA_VERSION,
        "packet": f"{solver_family}_test_closure",
        "resolved_solver_identity": {
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "synthetic closure test resolved report",
            "rigid_contact_selection": "world_solver_family",
            "rigid_contact_solver": solver_family,
            "rigid_point_joint_solver": solver_family,
            "multibody_integration_family": "none",
        },
        "substantive_artifact": {
            "path": artifact.relative_to(tmp_path).as_posix(),
            "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
        },
        "benchmark": {
            "benchmark": CLOSURE_BENCHMARK_METHODS[0],
            "rows": [
                {
                    "name": f"{CLOSURE_BENCHMARK_METHODS[0]}/iterations:8_mean",
                    "run_name": f"{CLOSURE_BENCHMARK_METHODS[0]}/iterations:8",
                    "run_type": "aggregate",
                }
            ],
        },
        "source_provenance": _closure_source_provenance(),
        "visual_evidence": {role: {"label": role} for role in CLOSURE_CAPTURE_ROLES},
        "target": {"contract_rows": [requirement_id]},
        "plan104_claims": {
            requirement_id: {
                "status": "complete",
                "predicate_results": dict(predicates),
                "backend_results": dict(backends),
            }
        },
    }
    path.write_text(json.dumps(packet, indent=2) + "\n")
    return relative, path, packet, artifact


def _profile_declarations(profile):
    return {
        "required_packet_keys": profile.required_packet_keys,
        "required_source_paths": profile.required_source_paths,
        "required_capture_roles": profile.required_capture_roles,
        "required_benchmark_methods": profile.required_benchmark_methods,
    }


def _complete_requirement(
    module,
    tmp_path,
    monkeypatch,
    *,
    solver_family="avbd",
    requirement_id=None,
):
    requirement_id = requirement_id or f"{solver_family}.method.cpu_solver"
    predicates = {key: True for key in module.METHOD_PREDICATES}
    backends = {key: True for key in module.REQUIRED_BACKENDS}
    relative, path, packet, artifact = _write_closure_packet(
        module,
        tmp_path,
        requirement_id,
        predicates,
        backends,
        solver_family=solver_family,
    )
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    calls = {"packet": 0, "row": 0}

    def validate_packet(_path, candidate):
        calls["packet"] += 1
        record = candidate.get("substantive_artifact")
        if not isinstance(record, dict):
            return ["required substantive artifact record is missing"]
        relative_artifact = module._safe_relative_path(record.get("path"))
        if relative_artifact is None:
            return ["substantive artifact path is unsafe"]
        resolved_artifact = module._repository_path(relative_artifact)
        if resolved_artifact is None or not resolved_artifact.is_file():
            return ["substantive artifact is missing"]
        payload = resolved_artifact.read_bytes()
        if hashlib.sha256(payload).hexdigest() != record.get("sha256"):
            return ["substantive artifact hash drifted"]
        if payload != b"substantive closure artifact\n":
            return ["substantive artifact sentinel is invalid"]
        return []

    def authorize_row(
        _path,
        _candidate,
        claim_id,
        solver_family,
        expected_predicates,
        expected_backends,
    ):
        calls["row"] += 1
        if claim_id != requirement_id or solver_family != profile_family:
            return ["synthetic profile received the wrong row identity"]
        if tuple(expected_predicates) != module.METHOD_PREDICATES:
            return ["synthetic profile received the wrong predicate order"]
        if tuple(expected_backends) != module.REQUIRED_BACKENDS:
            return ["synthetic profile received the wrong backend order"]
        return []

    profile_family = solver_family
    profile = module.ClosureProfile(
        solver_family=profile_family,
        claim_ids=(requirement_id,),
        required_packet_keys=CLOSURE_PACKET_KEYS,
        required_source_paths=CLOSURE_SOURCE_PATHS,
        required_capture_roles=CLOSURE_CAPTURE_ROLES,
        required_benchmark_methods=CLOSURE_BENCHMARK_METHODS,
        validate_packet=validate_packet,
        authorize_row=authorize_row,
    )
    monkeypatch.setitem(module.CLOSURE_PROFILES, path.resolve(), profile)
    requirement = {
        "id": requirement_id,
        "status": "complete",
        "source_locator": "test source",
        "evidence": [relative.as_posix()],
        "blockers": [],
        "predicate_results": predicates,
        "backend_results": backends,
        "closure_evidence": {
            "path": relative.as_posix(),
            "validator": module.AVBD_PACKET_VALIDATOR,
            "sha256": module.hashlib.sha256(path.read_bytes()).hexdigest(),
            "claim_id": requirement_id,
        },
    }
    return requirement, path, packet, artifact, calls


def _complete_requirement_errors(module, requirement):
    return module._requirement_errors(
        requirement,
        expected_id=requirement["id"],
        required_predicates=module.METHOD_PREDICATES,
        context=module._ValidationContext(),
    )


def _rewrite_closure_packet(requirement, path, packet):
    path.write_text(json.dumps(packet, indent=2) + "\n")
    requirement["closure_evidence"]["sha256"] = hashlib.sha256(
        path.read_bytes()
    ).hexdigest()


def test_complete_row_accepts_typed_current_packet_closure(tmp_path, monkeypatch):
    module = _load_module()
    requirement, _, _, _, calls = _complete_requirement(module, tmp_path, monkeypatch)
    assert _complete_requirement_errors(module, requirement) == []
    assert calls == {"packet": 1, "row": 1}


def test_complete_vbd_row_accepts_vbd_native_packet_closure(tmp_path, monkeypatch):
    module = _load_module()
    requirement, path, packet, _, calls = _complete_requirement(
        module,
        tmp_path,
        monkeypatch,
        solver_family="vbd",
        requirement_id="vbd.method.local_vertex_block_newton",
    )

    assert path.name == "vbd-test-closure-packet.json"
    assert packet["resolved_solver_identity"]["rigid_contact_solver"] == "vbd"
    assert _complete_requirement_errors(module, requirement) == []
    assert calls == {"packet": 1, "row": 1}


def test_all_true_assertion_packet_without_profile_cannot_close(tmp_path, monkeypatch):
    module = _load_module()
    requirement, path, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    monkeypatch.delitem(module.CLOSURE_PROFILES, path.resolve(), raising=False)

    errors = _complete_requirement_errors(module, requirement)

    assert any("no registered row-closing profile" in error for error in errors)


def test_closure_profile_substantive_error_fails_closed(tmp_path, monkeypatch):
    module = _load_module()
    requirement, _, _, artifact, calls = _complete_requirement(
        module, tmp_path, monkeypatch
    )
    artifact.write_text("assertions only\n")

    errors = _complete_requirement_errors(module, requirement)

    assert any("substantive artifact hash drifted" in error for error in errors)
    assert calls == {"packet": 1, "row": 1}


@pytest.mark.parametrize("layer", ("packet", "row"))
def test_closure_profile_invalid_callback_result_fails_closed(
    tmp_path, monkeypatch, layer
):
    module = _load_module()
    requirement, path, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    profile = module.CLOSURE_PROFILES[path.resolve()]
    replacement = module.ClosureProfile(
        solver_family=profile.solver_family,
        claim_ids=profile.claim_ids,
        **_profile_declarations(profile),
        validate_packet=(
            (lambda _path, _packet: "")
            if layer == "packet"
            else profile.validate_packet
        ),
        authorize_row=(
            (lambda *_args: "") if layer == "row" else profile.authorize_row
        ),
    )
    monkeypatch.setitem(module.CLOSURE_PROFILES, path.resolve(), replacement)

    errors = _complete_requirement_errors(module, requirement)

    callback_name = "validate_packet" if layer == "packet" else "authorize_row"
    assert any(
        f"profile {callback_name} must return a list of non-empty strings" in error
        for error in errors
    ), errors


def test_hook_only_closure_profile_cannot_close_a_declaration_free_packet(
    tmp_path, monkeypatch
):
    module = _load_module()
    requirement, path, packet, _, _ = _complete_requirement(
        module, tmp_path, monkeypatch
    )
    profile = module.CLOSURE_PROFILES[path.resolve()]
    permissive = module.ClosureProfile(
        solver_family=profile.solver_family,
        claim_ids=profile.claim_ids,
        **_profile_declarations(profile),
        validate_packet=lambda _path, _packet: [],
        authorize_row=lambda *_args: [],
    )
    monkeypatch.setitem(module.CLOSURE_PROFILES, path.resolve(), permissive)
    for key in CLOSURE_PACKET_KEYS:
        packet.pop(key)
    _rewrite_closure_packet(requirement, path, packet)

    expected = (
        [f"packet must contain object member {key!r}" for key in CLOSURE_PACKET_KEYS]
        + [
            f"packet source_provenance.files must cover {source_path!r}"
            for source_path in CLOSURE_SOURCE_PATHS
        ]
        + [
            f"packet visual_evidence must contain capture role {role!r}"
            for role in CLOSURE_CAPTURE_ROLES
        ]
        + [
            f"packet benchmark must record method {method!r}"
            for method in CLOSURE_BENCHMARK_METHODS
        ]
    )

    packet_errors = module._closure_packet_profile_errors(
        path.resolve(), packet, permissive, module._ValidationContext()
    )
    row_errors = _complete_requirement_errors(module, requirement)

    assert packet_errors == expected
    for message in expected:
        assert any(message in error for error in row_errors), (message, row_errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda packet: packet.pop("benchmark"),
            "packet must contain object member 'benchmark'",
        ),
        (
            lambda packet: packet.pop("source_provenance"),
            "packet must contain object member 'source_provenance'",
        ),
        (
            lambda packet: packet.pop("visual_evidence"),
            "packet must contain object member 'visual_evidence'",
        ),
        (
            lambda packet: packet["source_provenance"]["files"].pop(0),
            "packet source_provenance.files must cover "
            "'scripts/check_plan104_paper_parity.py'",
        ),
        (
            lambda packet: packet["visual_evidence"].pop("outcome"),
            "packet visual_evidence must contain capture role 'outcome'",
        ),
        (
            lambda packet: packet["benchmark"].update(
                benchmark="BM_UnrelatedClosureStep", rows=[]
            ),
            "packet benchmark must record method 'BM_ClosureTestStep'",
        ),
    ],
)
def test_declared_closure_packet_element_cannot_be_dropped(
    tmp_path, monkeypatch, mutation, message
):
    module = _load_module()
    requirement, path, packet, _, _ = _complete_requirement(
        module, tmp_path, monkeypatch
    )
    assert _complete_requirement_errors(module, requirement) == []
    mutation(packet)
    _rewrite_closure_packet(requirement, path, packet)

    errors = _complete_requirement_errors(module, requirement)

    assert any(message in error for error in errors), errors


@pytest.mark.parametrize(
    "field",
    (
        "required_packet_keys",
        "required_source_paths",
        "required_capture_roles",
        "required_benchmark_methods",
    ),
)
@pytest.mark.parametrize(
    "declaration", ((), "outcome", ["outcome"], ("",), ("outcome", "outcome"))
)
def test_closure_profile_declaration_fields_must_be_unique_string_tuples(
    tmp_path, monkeypatch, field, declaration
):
    module = _load_module()
    requirement, path, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    profile = module.CLOSURE_PROFILES[path.resolve()]
    declarations = _profile_declarations(profile)
    declarations[field] = declaration
    monkeypatch.setitem(
        module.CLOSURE_PROFILES,
        path.resolve(),
        module.ClosureProfile(
            solver_family=profile.solver_family,
            claim_ids=profile.claim_ids,
            **declarations,
            validate_packet=profile.validate_packet,
            authorize_row=profile.authorize_row,
        ),
    )

    errors = _complete_requirement_errors(module, requirement)

    assert any(
        f"profile {field} must be a non-empty unique string tuple" in error
        for error in errors
    ), errors


def _add_second_closure_row(module, requirement, path, packet, monkeypatch, calls):
    second_id = "avbd.method.cuda_solver"
    second = copy.deepcopy(requirement)
    second["id"] = second_id
    second["closure_evidence"]["claim_id"] = second_id
    packet["target"]["contract_rows"].append(second_id)
    packet["plan104_claims"][second_id] = {
        "status": "complete",
        "predicate_results": dict(second["predicate_results"]),
        "backend_results": dict(second["backend_results"]),
    }
    _rewrite_closure_packet(requirement, path, packet)
    second["closure_evidence"]["sha256"] = requirement["closure_evidence"]["sha256"]
    existing = module.CLOSURE_PROFILES[path.resolve()]

    def authorize_row(
        _path,
        _packet,
        claim_id,
        solver_family,
        expected_predicates,
        expected_backends,
    ):
        calls["row"] += 1
        if claim_id not in (requirement["id"], second_id):
            return ["unexpected claim id"]
        if solver_family != "avbd":
            return ["unexpected solver family"]
        if tuple(expected_predicates) != module.METHOD_PREDICATES:
            return ["unexpected predicates"]
        if tuple(expected_backends) != module.REQUIRED_BACKENDS:
            return ["unexpected backends"]
        return []

    monkeypatch.setitem(
        module.CLOSURE_PROFILES,
        path.resolve(),
        module.ClosureProfile(
            solver_family="avbd",
            claim_ids=(requirement["id"], second_id),
            **_profile_declarations(existing),
            validate_packet=existing.validate_packet,
            authorize_row=authorize_row,
        ),
    )
    return second


def test_one_packet_two_rows_caches_substantive_work_and_authorizes_each_row(
    tmp_path, monkeypatch
):
    module = _load_module()
    requirement, path, packet, _, calls = _complete_requirement(
        module, tmp_path, monkeypatch
    )
    second = _add_second_closure_row(
        module, requirement, path, packet, monkeypatch, calls
    )
    context = module._ValidationContext()

    first_errors = module._requirement_errors(
        requirement,
        expected_id=requirement["id"],
        required_predicates=module.METHOD_PREDICATES,
        context=context,
    )
    second_errors = module._requirement_errors(
        second,
        expected_id=second["id"],
        required_predicates=module.METHOD_PREDICATES,
        context=context,
    )

    assert first_errors == []
    assert second_errors == []
    assert calls == {"packet": 1, "row": 2}


def test_one_packet_two_rows_rejects_bad_second_claim(tmp_path, monkeypatch):
    module = _load_module()
    requirement, path, packet, _, calls = _complete_requirement(
        module, tmp_path, monkeypatch
    )
    second = _add_second_closure_row(
        module, requirement, path, packet, monkeypatch, calls
    )
    packet["plan104_claims"][second["id"]]["predicate_results"] = {
        "artifact_valid": True,
        "claim_valid": True,
    }
    _rewrite_closure_packet(requirement, path, packet)
    second["closure_evidence"]["sha256"] = requirement["closure_evidence"]["sha256"]
    context = module._ValidationContext()

    first_errors = module._requirement_errors(
        requirement,
        expected_id=requirement["id"],
        required_predicates=module.METHOD_PREDICATES,
        context=context,
    )
    second_errors = module._requirement_errors(
        second,
        expected_id=second["id"],
        required_predicates=module.METHOD_PREDICATES,
        context=context,
    )

    assert first_errors == []
    assert any(
        "predicate_results must exactly match" in error for error in second_errors
    )
    assert calls == {"packet": 1, "row": 2}


@pytest.mark.parametrize(
    ("row_family", "profile_family"), (("vbd", "avbd"), ("avbd", "vbd"))
)
def test_closure_profile_cannot_cross_solver_families(
    tmp_path, monkeypatch, row_family, profile_family
):
    module = _load_module()
    requirement, path, packet, _, _ = _complete_requirement(
        module, tmp_path, monkeypatch
    )
    old_id = requirement["id"]
    row_id = f"{row_family}.method.cpu_solver"
    profile_claim_id = f"{profile_family}.method.cpu_solver"
    requirement["id"] = row_id
    requirement["closure_evidence"]["claim_id"] = row_id
    packet["target"]["contract_rows"] = [row_id]
    packet["plan104_claims"] = {row_id: packet["plan104_claims"].pop(old_id)}
    _rewrite_closure_packet(requirement, path, packet)
    existing = module.CLOSURE_PROFILES[path.resolve()]
    monkeypatch.setitem(
        module.CLOSURE_PROFILES,
        path.resolve(),
        module.ClosureProfile(
            solver_family=profile_family,
            claim_ids=(profile_claim_id,),
            **_profile_declarations(existing),
            validate_packet=existing.validate_packet,
            authorize_row=existing.authorize_row,
        ),
    )

    errors = _complete_requirement_errors(module, requirement)

    assert any("cannot authorize" in error for error in errors), errors


@pytest.mark.parametrize("legacy_version", (1, 3, 4, 5))
def test_readable_legacy_packet_cannot_close_a_row(
    tmp_path, monkeypatch, legacy_version
):
    module = _load_module()
    requirement, path, packet, _, _ = _complete_requirement(
        module, tmp_path, monkeypatch
    )
    packet_validator_module = sys.modules["check_avbd_packets"]
    monkeypatch.setitem(
        packet_validator_module.LEGACY_PACKET_SCHEMA_VERSIONS,
        path.name,
        legacy_version,
    )
    packet["schema_version"] = legacy_version
    packet.pop("plan104_claims")
    if legacy_version == 1:
        packet.pop("resolved_solver_identity")
    elif legacy_version == 3:
        packet["resolved_solver_identity"] = {
            "avbd_rigid_contact_config_emplaced": False,
            "recorded_from": "legacy synthetic closure packet",
            "rigid_contact_selection": "not_applicable",
            "rigid_contact_solver": "none",
            "rigid_point_joint_solver": "avbd",
        }
    _rewrite_closure_packet(requirement, path, packet)
    context = module._ValidationContext()

    generic_errors = module._validator_result(
        module.AVBD_PACKET_VALIDATOR, path.resolve(), context
    )
    closure_errors = module._requirement_errors(
        requirement,
        expected_id=requirement["id"],
        required_predicates=module.METHOD_PREDICATES,
        context=context,
    )

    assert generic_errors == ()
    assert any("schema_version 6" in error for error in closure_errors)
    assert any("must contain plan104_claims" in error for error in closure_errors)


def test_incomplete_row_cannot_carry_closure_evidence(tmp_path, monkeypatch):
    module = _load_module()
    requirement, _, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    requirement["status"] = "partial"
    requirement["blockers"] = ["not closed"]
    requirement["predicate_results"]["claim_valid"] = False

    errors = _complete_requirement_errors(module, requirement)

    assert any("incomplete rows cannot carry closure_evidence" in e for e in errors)


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda row, _path, _packet: row["closure_evidence"].update(
                path="../avbd-test-closure-packet.json"
            ),
            "path must be a safe repository-relative path",
        ),
        (
            lambda row, _path, _packet: row["evidence"].clear(),
            "path must also be present in evidence",
        ),
        (
            lambda row, _path, _packet: row["closure_evidence"].update(sha256="0" * 64),
            "sha256 does not match current packet contents",
        ),
        (
            lambda row, _path, _packet: row["closure_evidence"].update(
                validator="unknown/v1"
            ),
            "validator is unknown",
        ),
        (
            lambda row, _path, _packet: row["closure_evidence"].update(
                claim_id="avbd.method.cuda_solver"
            ),
            "claim_id must equal",
        ),
    ],
)
def test_closure_reference_mutations_are_rejected(
    tmp_path, monkeypatch, mutation, message
):
    module = _load_module()
    requirement, path, packet, _, _ = _complete_requirement(
        module, tmp_path, monkeypatch
    )
    mutation(requirement, path, packet)

    errors = _complete_requirement_errors(module, requirement)

    assert any(message in error for error in errors), errors


@pytest.mark.parametrize("validator", (None, [], {}))
def test_closure_validator_must_be_a_string(tmp_path, monkeypatch, validator):
    module = _load_module()
    requirement, _, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    requirement["closure_evidence"]["validator"] = validator

    errors = _complete_requirement_errors(module, requirement)

    assert any("validator must be a string" in error for error in errors), errors


def test_closure_descriptor_same_content_order_mutation_is_rejected(
    tmp_path, monkeypatch
):
    module = _load_module()
    requirement, _, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    closure = requirement["closure_evidence"]
    requirement["closure_evidence"] = {
        "validator": closure["validator"],
        "path": closure["path"],
        "sha256": closure["sha256"],
        "claim_id": closure["claim_id"],
    }

    errors = _complete_requirement_errors(module, requirement)

    assert any("must contain exactly, in order" in error for error in errors), errors


@pytest.mark.parametrize("map_name", ("predicate_results", "backend_results"))
def test_complete_row_same_content_map_order_mutation_is_rejected(
    tmp_path, monkeypatch, map_name
):
    module = _load_module()
    requirement, _, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    requirement[map_name] = dict(reversed(tuple(requirement[map_name].items())))

    errors = _complete_requirement_errors(module, requirement)

    assert any(
        f"{map_name} must contain exactly, in order" in error for error in errors
    ), errors


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda row, packet: packet.update(schema_version=4),
            "current nonlegacy PLAN-104 packet schema_version 6",
        ),
        (
            lambda row, packet: packet.pop("packet"),
            "packet identity must be a non-empty string",
        ),
        (
            lambda row, packet: packet["target"].update(contract_rows=[]),
            "target.contract_rows must contain",
        ),
        (
            lambda row, packet: packet.pop("plan104_claims"),
            "must contain plan104_claims",
        ),
        (
            lambda row, packet: packet["plan104_claims"].update(
                {"avbd.method.cuda_solver": packet["plan104_claims"].pop(row["id"])}
            ),
            "must contain plan104_claims",
        ),
        (
            lambda row, packet: packet["plan104_claims"][row["id"]][
                "predicate_results"
            ].update(claim_valid=False),
            "predicate_results must exactly match the row",
        ),
        (
            lambda row, packet: packet["plan104_claims"][row["id"]][
                "backend_results"
            ].update(cuda=False),
            "backend_results must exactly match the row",
        ),
    ],
)
def test_closure_packet_mutations_are_rejected(
    tmp_path, monkeypatch, mutation, message
):
    module = _load_module()
    requirement, path, packet, _, _ = _complete_requirement(
        module, tmp_path, monkeypatch
    )
    mutation(requirement, packet)
    _rewrite_closure_packet(requirement, path, packet)

    errors = _complete_requirement_errors(module, requirement)

    assert any(message in error for error in errors), errors


@pytest.mark.parametrize(
    "mutation",
    [
        lambda claim: {
            "predicate_results": claim["predicate_results"],
            "status": claim["status"],
            "backend_results": claim["backend_results"],
        },
        lambda claim: {
            **claim,
            "predicate_results": dict(
                reversed(tuple(claim["predicate_results"].items()))
            ),
        },
        lambda claim: {
            **claim,
            "backend_results": dict(reversed(tuple(claim["backend_results"].items()))),
        },
    ],
)
def test_closure_claim_same_content_order_mutation_is_rejected(
    tmp_path, monkeypatch, mutation
):
    module = _load_module()
    requirement, path, packet, _, _ = _complete_requirement(
        module, tmp_path, monkeypatch
    )
    claims = packet["plan104_claims"]
    claims[requirement["id"]] = mutation(claims[requirement["id"]])
    _rewrite_closure_packet(requirement, path, packet)

    errors = _complete_requirement_errors(module, requirement)

    assert any("order" in error for error in errors), errors


def test_closure_packet_must_contain_valid_json(tmp_path, monkeypatch):
    module = _load_module()
    requirement, path, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    path.write_text("{not-json\n")
    requirement["closure_evidence"]["sha256"] = hashlib.sha256(
        path.read_bytes()
    ).hexdigest()

    errors = _complete_requirement_errors(module, requirement)

    assert any("invalid JSON" in error for error in errors), errors


def test_closure_packet_symbolic_link_is_rejected(tmp_path, monkeypatch):
    module = _load_module()
    requirement, path, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    target = path.with_name("avbd-closure-target-packet.json")
    path.rename(target)
    path.symlink_to(target.name)

    errors = _complete_requirement_errors(module, requirement)

    assert any("cannot be a symbolic link" in error for error in errors), errors


def test_closure_packet_directory_is_rejected(tmp_path, monkeypatch):
    module = _load_module()
    requirement, path, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    path.unlink()
    path.mkdir()

    errors = _complete_requirement_errors(module, requirement)

    assert any("must be a regular file" in error for error in errors), errors


def test_closure_packet_hash_read_error_fails_closed(tmp_path, monkeypatch):
    module = _load_module()
    requirement, path, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    real_read_bytes = Path.read_bytes

    def failing_read_bytes(candidate):
        if candidate.resolve() == path.resolve():
            raise PermissionError("denied")
        return real_read_bytes(candidate)

    monkeypatch.setattr(Path, "read_bytes", failing_read_bytes)

    errors = _complete_requirement_errors(module, requirement)

    assert any("cannot be read" in error and "denied" in error for error in errors)
    assert any("could not be hashed" in error for error in errors)


def test_referenced_packet_validator_runs_at_partial_status_and_is_deduplicated(
    tmp_path, monkeypatch
):
    module = _load_module()
    requirement, path, _, _, _ = _complete_requirement(module, tmp_path, monkeypatch)
    calls = []

    def validator(packet_path, _context):
        calls.append(packet_path)
        return ["stale packet"]

    monkeypatch.setitem(
        module.EVIDENCE_VALIDATORS,
        module.AVBD_PACKET_VALIDATOR,
        validator,
    )
    context = module._ValidationContext()
    first = module._evidence_path_errors(
        requirement["id"], requirement["evidence"], context
    )
    second = module._evidence_path_errors(
        requirement["id"], requirement["evidence"], context
    )

    assert first == ["stale packet"]
    assert second == []
    assert calls == [path]


def test_packet_validator_dispatch_is_limited_to_plan104_contract_directory(
    tmp_path, monkeypatch
):
    module = _load_module()
    monkeypatch.setattr(module, "REPO_ROOT", tmp_path)
    outside = tmp_path / "other" / "avbd-outside-packet.json"
    outside.parent.mkdir()
    outside.write_text("{}\n")
    calls = []
    monkeypatch.setitem(
        module.EVIDENCE_VALIDATORS,
        module.AVBD_PACKET_VALIDATOR,
        lambda path, _context: calls.append(path) or [],
    )

    errors = module._evidence_path_errors(
        "avbd.method.cpu_solver",
        ["other/avbd-outside-packet.json"],
        module._ValidationContext(),
    )

    assert errors == []
    assert calls == []
