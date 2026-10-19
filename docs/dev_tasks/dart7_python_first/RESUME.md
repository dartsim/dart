# Resume

## Current state (2026-10-19)

M0 exited on 2026-10-14, when the scope artifacts
([#3728](https://github.com/dartsim/dart/pull/3728)) and the gz API inventory
([#3729](https://github.com/dartsim/dart/pull/3729)) merged;
[#3787](https://github.com/dartsim/dart/pull/3787) later added the inventory's
missing construction and callable dependencies. 6.20.1 was packaged in
[#3806](https://github.com/dartsim/dart/pull/3806) and tagged, with the backports in
[#3805](https://github.com/dartsim/dart/pull/3805), which include #3722's
analytical-IK copy fix. Main also gained the shape and query capability matrix
(#3788), sdformat-based SDF parsing (#3789), delegation by default for AI
agents (#3799, #3802) and lower SDF loading memory (#3803).

#3789 changed the contact benchmark's workload, so comparisons against
`dart7-base` withhold the `contact_benchmark` rows as behavior changes; the
portable and micro rows still compare. The milestone-exit baseline for those
rows is an open decision (see [README](README.md#blockers-and-next-actions)).

M1 is active. M1a merged in #3775. The first M1b, M1c and M1d packets,
developed on unpublished branches, were re-cut from current `main` and
published one per pull request; the rest of the native chain is still
unpublished. No Python 7.0 API has landed yet: every merged M1 packet is C++
with no Python binding, and none of the 44 target-shape tests has turned
green.

| Packet | Milestone | Pull request | State |
| --- | --- | --- | --- |
| World-local preparation | M1b | [#3818](https://github.com/dartsim/dart/pull/3818) | Merged |
| Void signal raises with zero or one subscriber avoid snapshot allocation | M1 zero-allocation | [#3815](https://github.com/dartsim/dart/pull/3815) | Merged |
| Default Shape constructor initializes its lazy caches | Fix | [#3819](https://github.com/dartsim/dart/pull/3819) | Merged |
| Detector and contact-handler replacement order | Fix | [#3820](https://github.com/dartsim/dart/pull/3820) | Merged |
| Soft point-mass state on add and remove | Fix | [#3813](https://github.com/dartsim/dart/pull/3813) | Merged; found by the snapshot tests |
| World dynamic state snapshots | M1c | [#3816](https://github.com/dartsim/dart/pull/3816) | Merged |
| Manual joint constraint state | M1c | [#3817](https://github.com/dartsim/dart/pull/3817) | Merged |
| Restore without pre-restore contact, solver and sleep history | M1c | [#3827](https://github.com/dartsim/dart/pull/3827) | Merged |
| Executor consolidation | M1d prerequisite | [#3825](https://github.com/dartsim/dart/pull/3825) | Merged |
| Line segment and point cloud bounds after edits | Fix | [#3826](https://github.com/dartsim/dart/pull/3826) | Merged |
| Clearing Python raycast filters with `None` | M5 fix | [#3810](https://github.com/dartsim/dart/pull/3810) | Merged |
| dartpy version in CMake builds | M5 fix | [#3809](https://github.com/dartsim/dart/pull/3809) | Merged |
| Generated headers in headers-only installs | Build fix | [#3811](https://github.com/dartsim/dart/pull/3811) | Merged |
| Inventory identities and raw signatures | M5a tooling | [#3812](https://github.com/dartsim/dart/pull/3812) | Merged; feeds the rename table |
| Locale-independent Fuel cache key in the Gazebo lanes | Tooling fix | [#3814](https://github.com/dartsim/dart/pull/3814) | Merged |

Codex wrote the original packet code; Claude re-cut, integrated and published
it. Reviews came from engines other than the code's writer.

## Proposed re-prioritization (pending maintainer decisions)

On 2026-10-18 the maintainer restated the program's goals: a Python-first
DART 7; a Pythonic dartpy rather than a parity binding; best-effort C++ that
stays compatible with gz-physics and gz-sim; and multi-physics and
multi-solver support in `World` and the constraint solver. The proposal runs
these tracks in parallel, Python first:

1. Python API (M5): binding fixes, an API-surface ratchet, snapshot bindings,
   the package layout, read-only arrays, errors and logging, `Transform` and
   `Rotation`, a vertical slice of the 7.0 facade, then the naming rules and
   one old-to-new mapping that drives the codemod, stubs and migration guide
   (which revisits M5a's single rename-table approval).
   Every native feature ships its Python binding and acceptance test.
2. Native core (M1): compile and edit, counted stepping, per-solver
   parameters, the remapping clone, `WorldBatch` with its own executor (which
   revisits the shared-executor plan under D14), then actuators and servos,
   sensors and checkpoints.
3. Multi-solver and multi-physics: a typed solver policy, a written `World`
   and solver contract, and the step's rigid code behind a private domain
   boundary; the coupling seam is built together with a first experimental
   domain, which revisits D2 and D7.
4. Gazebo gate: pinned lane environments, baselines from the last 6.20.x, and
   lanes triggered by step, solver, collision and packaging changes.

Python packets are prepared on unpublished branches, reviewed by two engines
other than each writer and re-qualified on current `main` before publication:
`None` frame arguments no longer crash; the API-surface ratchet;
`capture_state()` and `restore_state()`; the package layout with cp312-abi3
wheels; read-only array returns; and immutable `Transform` and `Rotation`,
expected to un-xfail the transform and rotation target-shape tests.
The decisions they wait on are listed in the
[README](README.md#blockers-and-next-actions).

## Next steps

1. Publish the prepared Python packets as the pending decisions allow.
2. Compile and edit (M1b): its fix round follows the decision on
   never-compiled C++ Worlds; then counted stepping, per-solver parameters and
   the remapping clone. Each packet is re-qualified on current `main`.
3. Add the macOS and Windows Python 3.15 wheels to 6.20.x once conda-forge's
   `bullet-cpp` builds for Python 3.15.

## Risks and recovery

- Main moves quickly. Before every approved push to a published branch, merge
  the latest `origin/main` (never rebase a published branch) and re-run the
  packet's gates.
- Gazebo lane results drift with the environment: after an update, gz-sim's
  `INTEGRATION_reset_sensors` failed for every DART version, including 6.20.0,
  because the test reads a freed component pointer. Classify failures against
  a base variant; pinning the lane environments is part of the proposal.
- The rollover integrates #3696's Filament AI-checker and test edits while
  retaining the DART 7 branch profile and release routing.
- dartpy has no Python 3.15 CI lane yet. Re-run the shutdown regression test
  against a 3.15 build when touching finalization code.
- #3741 fixed the Windows `ContactSurfaceProbe` failure seen at `722f28ddd`.
  Preserve its platform-specific dialects: Unix probes C++14 with existing
  compiler extensions; MSVC uses DART's required C++17 and exported
  template-parsing options.

## Verification commands

- `pixi run lint`, then `pixi run check-lint`.
- `pixi run test-ai-infra` and `pixi run exercise-agent-scenarios` for AI-policy
  changes.
- `pixi run build`, then `pixi run test`.
- `pixi run build-py-dev`, then `pixi run test-py`.
