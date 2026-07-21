import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_fbf_card_cross_step_policy_ab_v2.py"


def _load_module():
    scripts = str(ROOT / "scripts")
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    sys.modules.pop("run_fbf_card_cross_step_policy_ab_v1", None)
    spec = importlib.util.spec_from_file_location("card_policy_ab_runner_v2", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _row(module, attempts):
    return {
        "step_exact_attempts": str(attempts),
        "exact_diagnostics_contract": (
            module.MULTI_GROUP_CONTRACT
            if attempts > 1
            else module.SINGLE_GROUP_CONTRACT
        ),
        "last_exact_diagnostics_contract": (
            module.MULTI_GROUP_CONTRACT
            if attempts > 1
            else "last_exact_group_only_single_group"
        ),
    }


def test_frozen_v2_protocol_hash_is_exact():
    module = _load_module()

    assert module._protocol_contract_sha256() == (
        "084731bea140d8911570155ec41f15d11eb47047ed8e4c03d5dc04df729fdd21"
    )


@pytest.mark.parametrize("attempts", [1, 2, 7])
def test_v2_validates_group_count_label_and_delegates_an_isolated_copy(
    monkeypatch, attempts
):
    module = _load_module()
    raw = _row(module, attempts)
    original = dict(raw)
    delegated = []
    monkeypatch.setattr(
        module,
        "_validate_common_row_v1",
        lambda row, policy, step: delegated.append((row, policy, step)),
    )

    module._validate_common_row(raw, "dart_current", 4)

    assert raw == original
    assert delegated[0][0] is not raw
    assert delegated[0][0]["exact_diagnostics_contract"] == (
        module.SINGLE_GROUP_CONTRACT
    )
    assert delegated[0][1:] == ("dart_current", 4)


@pytest.mark.parametrize(
    ("attempts", "field", "value", "message"),
    [
        (1, "exact_diagnostics_contract", "wrong", "v2 exact diagnostics"),
        (
            1,
            "exact_diagnostics_contract",
            "last_exact_group_only_multi_group_noncomparable",
            "v2 exact diagnostics",
        ),
        (1, "last_exact_diagnostics_contract", "wrong", "v2 additive diagnostics"),
        (
            1,
            "last_exact_diagnostics_contract",
            "last_exact_group_only_multi_group_noncomparable",
            "v2 additive diagnostics",
        ),
        (2, "exact_diagnostics_contract", "wrong", "v2 exact diagnostics"),
        (
            2,
            "exact_diagnostics_contract",
            "last_exact_group_public_getters_contact_row_no_dense_snapshot_"
            "warm_fraction_over_step_contacts",
            "v2 exact diagnostics",
        ),
        (2, "last_exact_diagnostics_contract", "wrong", "v2 additive diagnostics"),
        (
            2,
            "last_exact_diagnostics_contract",
            "last_exact_group_only_single_group",
            "v2 additive diagnostics",
        ),
    ],
)
def test_v2_rejects_wrong_group_count_labels(
    monkeypatch, attempts, field, value, message
):
    module = _load_module()
    row = _row(module, attempts)
    row[field] = value
    monkeypatch.setattr(module, "_validate_common_row_v1", lambda *_args: None)

    with pytest.raises(module.v1.EvidenceError, match=message):
        module._validate_common_row(row, "author_policy_inspired_b3f3c5c", 81)


def test_v2_main_installs_only_validator_metadata_overrides(monkeypatch):
    module = _load_module()
    observed = {}

    def fake_main(argv):
        observed.update(
            {
                "argv": argv,
                "protocol": module.v1.PROTOCOL,
                "schema": module.v1.SCHEMA_VERSION,
                "runner_file": module.v1.__file__,
                "validator": module.v1._validate_common_row,
                "execution_identity": module.v1._execution_identity,
                "command": module.v1._command,
                "arm_summary": module.v1._arm_summary,
            }
        )
        return 7

    monkeypatch.setattr(module.v1, "main", fake_main)

    assert module.main(["--output-dir", "unused"]) == 7
    assert observed == {
        "argv": ["--output-dir", "unused"],
        "protocol": module.PROTOCOL,
        "schema": module.SCHEMA_VERSION,
        "runner_file": str(SCRIPT),
        "validator": module._validate_common_row,
        "execution_identity": module._execution_identity,
        "command": module.v1._command,
        "arm_summary": module.v1._arm_summary,
    }


def test_v2_keeps_trace_command_and_schema_tokens_unchanged():
    module = _load_module()
    source = (ROOT / "tests/benchmark/integration/fbf_paper_trace.cpp").read_text(
        encoding="utf-8"
    )
    protocol = module.PROTOCOL.read_text(encoding="utf-8")

    assert "card_house_cross_step_policy_ab_v1" in source
    assert "card_house_cross_step_policy_ab_v1" in protocol
    assert "No scene, solver, collision" in protocol
    assert "No scene, solver, collision" not in module.v1.PROTOCOL.read_text(
        encoding="utf-8"
    )


def test_v2_commands_order_header_and_gate_functions_are_inherited_unchanged():
    module = _load_module()

    assert module.v1.POLICIES == (
        "dart_current",
        "author_policy_inspired_b3f3c5c",
    )
    assert module.v1.EXPECTED_STEPS == 90
    assert module.v1.EXPECTED_CPU == 8
    assert module.v1.EXPECTED_HEADER_SHA256 == (
        "b8590420ebcbf62c522fb88a5cad06f0c5ebd917400cf578c4c63f2d76dc1a36"
    )
    expected_prefix = [
        "taskset",
        "--cpu-list",
        "8",
        "/tmp/fbf_paper_trace",
        "card_house_26_settle_projectile_full",
        "exact_fbf",
        "1",
        "90",
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
    ]
    for policy in module.v1.POLICIES:
        assert module.v1._command(Path("/tmp/fbf_paper_trace"), policy) == [
            *expected_prefix,
            policy,
        ]
    assert module.v1._arm_summary is not module._validate_common_row
    assert module.v1._comparison is not module._validate_common_row


def test_v2_pins_inherited_runner_and_complete_v1_execution_identity(monkeypatch):
    module = _load_module()
    assert module.v1._sha256_file(module.V1_RUNNER_SOURCE) == (
        module.EXPECTED_V1_RUNNER_SOURCE_SHA256
    )

    frozen_components = {
        "identity_helper_source_sha256": "helper",
        "trace_source_sha256": "trace",
        "trace_executable": {"sha256": "binary", "libraries": ["libdart"]},
        "taskset_tool": {"sha256": "taskset"},
    }
    payload = json.dumps(
        frozen_components,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode()
    expected = module.hashlib.sha256(payload).hexdigest()
    identity = {
        "protocol_contract_sha256": "v2",
        "runner_source_sha256": "wrapper",
        **frozen_components,
    }
    monkeypatch.setattr(module, "_execution_identity_v1", lambda _binary: identity)
    monkeypatch.setattr(
        module, "EXPECTED_FROZEN_V1_EXECUTION_COMPONENTS_SHA256", expected
    )

    observed = module._execution_identity(Path("unused"))

    assert observed["inherited_v1_runner_source_sha256"] == (
        module.EXPECTED_V1_RUNNER_SOURCE_SHA256
    )
    assert observed["frozen_v1_execution_components_sha256"] == expected


def test_v2_rejects_predecessor_execution_identity_drift(monkeypatch):
    module = _load_module()
    identity = {
        "identity_helper_source_sha256": "changed",
        "trace_source_sha256": "trace",
        "trace_executable": {},
        "taskset_tool": {},
    }
    monkeypatch.setattr(module, "_execution_identity_v1", lambda _binary: identity)

    with pytest.raises(module.v1.EvidenceError, match="component identity drifted"):
        module._execution_identity(Path("unused"))


def test_v2_preserves_fresh_output_requirement(tmp_path):
    module = _load_module()

    with pytest.raises(module.v1.EvidenceError, match="must be fresh"):
        module.v1._prepare_output(tmp_path)
