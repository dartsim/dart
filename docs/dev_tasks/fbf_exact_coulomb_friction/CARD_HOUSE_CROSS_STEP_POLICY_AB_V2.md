# Card-house cross-step policy A/B v2

Status: frozen before either v2 evidence child is executed.

This protocol is the one-defect successor to
`CARD_HOUSE_CROSS_STEP_POLICY_AB_V1.md`. It incorporates the entire frozen v1
execution, policy, schema, scientific-negative, strict-gate, promotion, and
no-retuning contract whose prefix SHA-256 is
`a2babf6d28f802fce85d49abf4231c5ac94f3db62747f73a093cbba5ee3db4b5`.
The v1 run remains invalid and is not reinterpreted as v2 evidence.

## Why v2 exists

The committed v1 runner required the legacy `exact_diagnostics_contract`
field to equal the single-last-group contact-row label on every row. In the
first frozen run, the author-inspired arm produced two exact groups at step
81. The trace correctly changed that field to
`last_exact_group_only_multi_group_noncomparable`, but the validator rejected
the row before reaching its already-correct additive
`last_exact_diagnostics_contract` check.

V2 corrects only that structural parser rule. No scene, solver, collision,
affinity, command, order, policy, threshold, exit class, scientific gate, or
promotion rule changes.

## Frozen execution

The two child commands remain byte-for-byte identical to v1 and run in this
order:

```text
taskset --cpu-list 8 <fbf_paper_trace> \
  card_house_26_settle_projectile_full exact_fbf 1 90 nan performance \
  default default 1 paper_cpu native default 0 0 default dart_current

taskset --cpu-list 8 <fbf_paper_trace> \
  card_house_26_settle_projectile_full exact_fbf 1 90 nan performance \
  default default 1 paper_cpu native default 0 0 default \
  author_policy_inspired_b3f3c5c
```

V2 uses the same 115-column trace header SHA-256
`b8590420ebcbf62c522fb88a5cad06f0c5ebd917400cf578c4c63f2d76dc1a36`.
The trace still reports the v1 C++ schema token
`card_house_cross_step_policy_ab_v1`; v2 is an evidence-validator revision,
not a trace-schema or executable revision.

## Corrected structural rule

For each row, let `A = step_exact_attempts`. V2 requires:

- if `A == 1`, `exact_diagnostics_contract` is
  `last_exact_group_public_getters_contact_row_no_dense_snapshot_warm_fraction_over_step_contacts`
  and `last_exact_diagnostics_contract` is
  `last_exact_group_only_single_group`;
- if `A > 1`, both fields are
  `last_exact_group_only_multi_group_noncomparable`; and
- `A` remains positive under the inherited frozen card-house contract.

The v2 wrapper validates the observed label before substituting the inherited
single-group token into an isolated row copy passed through every remaining
v1 structural, policy, negative-result, and strict-gate check. The raw child
row is never modified.

## Identity and output

The v2 packet binds the v2 protocol prefix, v2 wrapper source, unchanged trace
source and executable, identity-helper source, `taskset`, `ldd`, dependency
resolution, and resolved shared libraries before and after both children. It
also binds the inherited v1 runner that supplies all unchanged execution and
scientific-gate behavior.

The following predecessor identities are frozen from the sole v1 execution:

- inherited v1 runner SHA-256:
  `47f32dc5cdab8457ec92438200bfd39fd3f78240b3ce9faa926a2b56fd6c25d8`;
- trace source SHA-256:
  `797c74753946ca7cf7f8e3680863b1acb13f53779c8c550f3427b76e337990db`;
- identity-helper source SHA-256:
  `7155b9bc6082e79aca317be6626fea587b58538df2f34e94f865cd54c15eb993`;
- trace executable SHA-256:
  `3ba276a8791b5e3ee74012772032c6ac879d78691db7a609c668117d15d618c7`;
  and
- canonical frozen execution-component SHA-256:
  `43984b7c42f6968877948f80ccd7dd199a62cc323e8323b0aee4b7ac2095a813`.

The last digest is over compact, key-sorted JSON containing the complete
`identity_helper_source_sha256`, `trace_source_sha256`, `trace_executable`,
and `taskset_tool` objects from v1. It therefore covers `ldd`, the resolved
DART library, and every resolved regular shared-library identity, not only
the executable file. V2 refuses to execute if either this component digest or
the inherited v1 runner digest differs. It must use a fresh output directory.
Timing remains diagnostic-only.

The inherited no-retuning rule applies. Any further correction requires v3
and another fresh output directory. The runner never launches a 600-step child
automatically.

## Frozen v2 result

Not executed yet. Fill this section only after the committed v2 protocol and
wrapper produce a provenance-complete bundle.

The frozen v2 protocol-contract SHA-256 is
`084731bea140d8911570155ec41f15d11eb47047ed8e4c03d5dc04df729fdd21`.
