#!/usr/bin/env python3
"""Run the frozen v2 validator for the 90-step card-house policy A/B."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Sequence

import run_fbf_card_cross_step_policy_ab_v1 as v1

ROOT = Path(__file__).resolve().parents[1]
PROTOCOL = (
    ROOT / "docs/dev_tasks/fbf_exact_coulomb_friction/"
    "CARD_HOUSE_CROSS_STEP_POLICY_AB_V2.md"
)
SCHEMA_VERSION = "dart.fbf_card_cross_step_policy_ab_v2/v2"
EXPECTED_PROTOCOL_CONTRACT_SHA256 = (
    "084731bea140d8911570155ec41f15d11eb47047ed8e4c03d5dc04df729fdd21"
)
V1_RUNNER_SOURCE = Path(v1.__file__).resolve()
EXPECTED_V1_RUNNER_SOURCE_SHA256 = (
    "47f32dc5cdab8457ec92438200bfd39fd3f78240b3ce9faa926a2b56fd6c25d8"
)
FROZEN_V1_EXECUTION_COMPONENT_KEYS = (
    "identity_helper_source_sha256",
    "trace_source_sha256",
    "trace_executable",
    "taskset_tool",
)
EXPECTED_FROZEN_V1_EXECUTION_COMPONENTS_SHA256 = (
    "43984b7c42f6968877948f80ccd7dd199a62cc323e8323b0aee4b7ac2095a813"
)
SINGLE_GROUP_CONTRACT = (
    "last_exact_group_public_getters_contact_row_no_dense_snapshot_"
    "warm_fraction_over_step_contacts"
)
MULTI_GROUP_CONTRACT = "last_exact_group_only_multi_group_noncomparable"

_validate_common_row_v1 = v1._validate_common_row
_report_v1 = v1._report
_execution_identity_v1 = v1._execution_identity


def _protocol_contract_sha256() -> str:
    text = PROTOCOL.read_text(encoding="utf-8")
    marker = "## Frozen v2 result"
    if marker not in text:
        raise v1.EvidenceError("v2 protocol result marker is missing")
    frozen = (text.split(marker, 1)[0].rstrip() + "\n").encode()
    digest = hashlib.sha256(frozen).hexdigest()
    if digest != EXPECTED_PROTOCOL_CONTRACT_SHA256:
        raise v1.EvidenceError("frozen v2 protocol contract hash drifted")
    return digest


def _validate_common_row(row: dict[str, str], policy: str, step: int) -> None:
    attempts = v1._int(row, "step_exact_attempts")
    expected = MULTI_GROUP_CONTRACT if attempts > 1 else SINGLE_GROUP_CONTRACT
    if row.get("exact_diagnostics_contract") != expected:
        raise v1.EvidenceError("v2 exact diagnostics contract drifted")

    expected_last = (
        MULTI_GROUP_CONTRACT if attempts > 1 else "last_exact_group_only_single_group"
    )
    if row.get("last_exact_diagnostics_contract") != expected_last:
        raise v1.EvidenceError("v2 additive diagnostics contract drifted")

    inherited_row = dict(row)
    inherited_row["exact_diagnostics_contract"] = SINGLE_GROUP_CONTRACT
    _validate_common_row_v1(inherited_row, policy, step)


def _frozen_v1_execution_components_sha256(identity: dict[str, Any]) -> str:
    try:
        components = {key: identity[key] for key in FROZEN_V1_EXECUTION_COMPONENT_KEYS}
    except KeyError as error:
        raise v1.EvidenceError(
            f"v2 execution identity is missing {error.args[0]!r}"
        ) from error
    payload = json.dumps(
        components,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode()
    return hashlib.sha256(payload).hexdigest()


def _execution_identity(binary: Path) -> dict[str, Any]:
    identity = _execution_identity_v1(binary)
    inherited_runner_sha256 = v1._sha256_file(V1_RUNNER_SOURCE)
    if inherited_runner_sha256 != EXPECTED_V1_RUNNER_SOURCE_SHA256:
        raise v1.EvidenceError("inherited v1 runner source identity drifted")

    component_sha256 = _frozen_v1_execution_components_sha256(identity)
    if component_sha256 != EXPECTED_FROZEN_V1_EXECUTION_COMPONENTS_SHA256:
        raise v1.EvidenceError("frozen v1 execution component identity drifted")

    identity["inherited_v1_runner_source_sha256"] = inherited_runner_sha256
    identity["frozen_v1_execution_components_sha256"] = component_sha256
    return identity


def _report(summaries: dict, comparison: dict) -> str:
    return _report_v1(summaries, comparison).replace(
        "# Card-House Cross-Step Policy A/B v1",
        "# Card-House Cross-Step Policy A/B v2",
        1,
    )


def main(argv: Sequence[str] | None = None) -> int:
    v1.PROTOCOL = PROTOCOL
    v1.SCHEMA_VERSION = SCHEMA_VERSION
    v1.EXPECTED_PROTOCOL_CONTRACT_SHA256 = EXPECTED_PROTOCOL_CONTRACT_SHA256
    v1._protocol_contract_sha256 = _protocol_contract_sha256
    v1._validate_common_row = _validate_common_row
    v1._execution_identity = _execution_identity
    v1._report = _report
    v1.__file__ = __file__
    return v1.main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
