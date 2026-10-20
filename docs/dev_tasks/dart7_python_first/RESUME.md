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
portable and micro rows still compare. Milestone exits compare those rows
against `812310ffb` instead (decided 2026-10-19).

M1 is active. M1a merged in #3775. The first M1b, M1c and M1d packets,
developed on unpublished branches, were re-cut from current `main` and
published one per pull request; the rest of the native chain is still
unpublished. Every merged M1 packet is C++ with no Python binding. The first
Python 7.0 API on `main` is the package layout (#3847) with its empty
`dartpy.experimental` namespace; none of the 44 target-shape tests has turned
green yet.

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
| Inventory identities and raw signatures | M5a tooling | [#3812](https://github.com/dartsim/dart/pull/3812) | Merged; feeds the API-surface ratchet and the old-to-new mapping |
| Locale-independent Fuel cache key in the Gazebo lanes | Tooling fix | [#3814](https://github.com/dartsim/dart/pull/3814) | Merged |
| `None` frame arguments no longer crash dartpy | M5 fix | [#3841](https://github.com/dartsim/dart/pull/3841) | Merged |
| API-surface ratchet | M5a tooling | [#3846](https://github.com/dartsim/dart/pull/3846) | Merged |
| Package layout and cp312-abi3 wheels | M5a | [#3847](https://github.com/dartsim/dart/pull/3847) | Merged |

Codex wrote the original packet code; Claude re-cut, integrated and published
it. Reviews came from engines other than the code's writer.

## Priorities (decided 2026-10-19)

On 2026-10-18 the maintainer restated the program's goals: a Python-first
DART 7; a Pythonic dartpy rather than a parity binding; best-effort C++ that
stays compatible with gz-physics and gz-sim; and multi-physics and
multi-solver support in `World` and the constraint solver. The maintainer
then decided the program's open questions; see
[Re-prioritization](decisions.md#re-prioritization-2026-10-19). Work runs in
parallel tracks, Python first:

1. Python API (M5), designed first under one naming rulebook: binding fixes,
   an API-surface ratchet, snapshot bindings, the package layout, read-only
   arrays, errors and logging, `Transform` and `Rotation`, then the naming
   rulebook, a vertical slice of the facade and the remaining API areas, with
   the codemod, stubs, migration guide and API reference generated from the
   old-to-new mapping. Approved decisions fix the central public names of the
   snapshot, package, arrays, errors and transform packets: the `dartpy`
   package (D4, D6), `dartpy.experimental` (D22), `capture_state` and
   `restore_state` (the design doc's `World` surface, under D11), the `dartpy` logger and the error classes the design
   doc names (D22), and `Transform` and `Rotation` (D28) with their named
   quaternion orders (D29). The rulebook adopts those names. Every other public
   name these packets add, including `WorldState`, is provisional and follows
   the rulebook once it is approved.
   Every native feature ships its Python binding and acceptance test.
2. Native core (M1): compile and edit, counted stepping, the remapping clone
   and `WorldBatch` with its own executor, then actuators and servos, sensors
   and same-World checkpoints.
3. Multi-solver and multi-physics: the typed solver policy with its per-solver
   parameters (M1e), the written `World` and solver contract, and the rigid
   step code behind an internal domain boundary; the coupling seam is built
   together with a first experimental domain (rods), in 7.0 if ready before
   `F` and otherwise in 7.1.
4. Gazebo gate: the Ionic and Jetty lanes, run for every step, solver,
   collision, packaging or Gazebo-lock change, against baselines from the last
   6.20.x. Building `dart-io` against each distribution's sdformat in those
   lanes is pending; the Ionic lane's DART still uses sdformat 16.

Python packets in review:

| Packet | Pull request |
| --- | --- |
| `capture_state()` / `restore_state()` with `WorldState` | [#3842](https://github.com/dartsim/dart/pull/3842) |
| Read-only array returns | [#3843](https://github.com/dartsim/dart/pull/3843) |
| `dartpy.errors` and the logging bridge | [#3844](https://github.com/dartsim/dart/pull/3844) |
| Immutable `Transform` and `Rotation` | [#3845](https://github.com/dartsim/dart/pull/3845) |

Once they merge, the transform, rotation and logging-bridge target-shape tests
pass, 3 of 44.

## Next steps

1. Land the Python packets still in review (#3842 to #3845).
2. Rename the private extension from `dartpy._dartpy`, which #3847 shipped, to
   `dartpy._core` (decided 2026-10-19) in one mechanical follow-up: `NB_MODULE`
   in `python/dartpy/module.cpp`, the CMake `OUTPUT_NAME`,
   `python/dartpy/package/__init__.py`, the wheel scripts and their tests, the
   binding tests that locate the extension,
   and `docs/onboarding/python-bindings.md`. The design doc already names
   `dartpy._core`.
3. Compile and edit (M1b): its fix round keeps never-compiled C++ Worlds on
   6.20 behavior and starts ownership tracking at `compile()`; then counted
   stepping and the remapping clone. Each packet is re-qualified on current
   `main`.
4. The naming rulebook, then the facade slice.
5. The typed solver policy with its per-solver parameters (M1e), and the
   written `World` and solver contract.
6. Add the macOS and Windows Python 3.15 wheels to 6.20.x once conda-forge's
   `bullet-cpp` builds for Python 3.15.

## Risks and recovery

- Main moves quickly. Before every approved push to a published branch, merge
  the latest `origin/main` (never rebase a published branch) and re-run the
  packet's gates.
- Gazebo lane results drift with the environment: after an update, gz-sim's
  `INTEGRATION_reset_sensors` failed for every DART version, including 6.20.0,
  because the test reads a freed component pointer. Classify failures against
  a base variant.
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
