# ABI Minor Policy (Issue #1026)

## Status

- Policy chosen: best-effort ABI stability within a major series.
- Implementation pending: documentation and release-process updates.

## Problem Statement

Downstream plugin ecosystems (notably Gazebo) need clarity on whether minor or
patch updates can break ABI. Today the policy is implicit and can surprise
plugin users when ABI breaks occur.

## Scope

- Define and document a best-effort ABI policy for minor and patch releases.
- Add ABI review expectations to the release process.
- Align with the ABI check tooling task to make the policy actionable.

## Out of Scope

- Inline namespaces or symbol versioning.
- Side-by-side installation of multiple minor versions.
- Hard guarantees of ABI stability.

## Policy

- Best-effort ABI stability across minor and patch releases within a major line.
- ABI breaks should be rare, explicitly reviewed, and clearly documented.

## Plan

- Add a short ABI policy note to existing onboarding/contribution docs.
- Add a release checklist item to run ABI checks and record outcomes.
- Ensure release notes call out any intentional ABI breaks.
- Confirm downstream expectations (Gazebo) match the policy.

## Deliverables

- Updated policy docs (no new files).
- Release checklist updated to include ABI review.
- Pointer to the ABI check workflow from the policy note.

## Validation / Success Criteria

- Policy is discoverable in onboarding/contributing guidance.
- Release workflow includes an ABI review step.
- ABI-affecting changes are either avoided or explicitly documented.

## Risks / Trade-offs

- Best-effort policy may still allow ABI breaks that impact plugins.
- Current SOVERSION behavior may signal ABI changes on minor bumps; revisit if
  policy expectations change.

## Open Questions

- Keep SOVERSION at major.minor or move to major-only?
- Who signs off on intentional ABI breaks?
- How are exceptions tracked for downstream communication?
