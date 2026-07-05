---
name: dart-benchmark-packet
description: "DART Benchmark Packet: author or refresh a benchmark evidence packet for an owning plan"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-benchmark-packet.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-benchmark-packet

Use this skill in Codex to run the DART `dart-benchmark-packet` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a generated Codex adapter entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-benchmark-packet <arguments>`
- Codex: `$dart-benchmark-packet <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Author or refresh a benchmark evidence packet: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/profiling.md

Also read the owning plan file named by the packet (for example
`docs/plans/<NNN>-<slug>.md`) and its packet convention.

## Workflow

1. Identify the owning plan and its packet convention: the packet checker (for
   example `pixi run check-avbd-packets`) and the packet generator (for example
   `scripts/write_*_packet.py`) that the plan names.
2. Build the benchmark target and run the benchmark that feeds the packet,
   following `docs/onboarding/profiling.md` for a stable measurement setup.
3. Run the packet writer to record the machine-generated evidence packet with
   its provenance and resolved configuration.
4. Validate the packet with the plan's packet checker; treat a failing checker
   as incomplete evidence and fix the packet, not the checker.
5. Prepare the owning plan's row or link update that task-specific gates
   require, but leave editing the plan file to the plan's own workflow — this
   command prepares and validates the packet. This is a local task; do not push
   or open PRs without explicit maintainer/user approval.

## Output

- Owning plan, packet ID, and the checker/generator used
- Benchmark command run and the measurement setup
- Packet file written and checker result
- Whether the packet is new or refreshed, and any remaining gap
