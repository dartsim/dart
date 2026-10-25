# Resume

## Current state (2026-10-24)

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

M1 is active. M1a merged in #3775 and M1b's compile and edit in #3869; the
first M1c and M1d packets merged earlier, one per pull request, and counted
stepping is next. The merged M1 packets are C++; of them, only the dynamic
snapshots have a Python binding (#3842). The Python 7.0 API on `main` has the
package layout (#3847), read-only arrays, `dartpy.errors` with the logging
bridge, and `Transform` and `Rotation`. Three of the 44 target-shape tests
pass: the logging bridge, transform composition and the rotation value.

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

1. Python API (M5), designed first under one naming rulebook: binding fixes, an
   API-surface ratchet, snapshot bindings, the package layout, read-only arrays,
   errors and logging, `Transform` and `Rotation`, then the naming rulebook
   (approved on 2026-10-24), a vertical slice of the facade and the remaining
   API areas, with the codemod, stubs, migration guide and API reference
   generated from the old-to-new mapping. Approved decisions fix the central
   public names of the snapshot, package, arrays, errors and transform packets:
   the `dartpy` package (D4, D6), `dartpy.experimental` (D22), `capture_state`
   and `restore_state` (the design doc's `World` surface, under D11), the
   `dartpy` logger (D22), and `Transform` and `Rotation` (D28) with their named
   quaternion orders (D29). The rulebook adopts those names, and builtin errors
   replace the errors packet's classes. Every other public name these packets
   add, including `WorldState`, follows the rulebook.
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

Merged since 2026-10-19:

| Packet | Milestone | Pull request |
| --- | --- | --- |
| `capture_state()` / `restore_state()` with `WorldState` | M5a | [#3842](https://github.com/dartsim/dart/pull/3842); budget in [#3906](https://github.com/dartsim/dart/pull/3906) |
| Read-only array returns | M5a | [#3843](https://github.com/dartsim/dart/pull/3843) |
| `dartpy.errors` and the logging bridge | M5a | [#3844](https://github.com/dartsim/dart/pull/3844) |
| Immutable `Transform` and `Rotation` | M5a | [#3845](https://github.com/dartsim/dart/pull/3845) |
| Private extension renamed to `dartpy._core` | M5a | [#3873](https://github.com/dartsim/dart/pull/3873) |
| API snapshot imports every public subpackage | M5a tooling | [#3872](https://github.com/dartsim/dart/pull/3872) |
| Opt-in `World::compile()` and scoped `World::edit()` | M1b | [#3869](https://github.com/dartsim/dart/pull/3869) |
| Changelog fragments on `main` | Process | [#3874](https://github.com/dartsim/dart/pull/3874) |
| Project-home decisions of 2026-10-19 | Process | [#3867](https://github.com/dartsim/dart/pull/3867) |

## Next steps

1. Close the post-merge review findings: #3869's constraint-solver workers
   must carry the stepping World's busy context, and a soft body's mesh
   rebuild must bump its shape version so detectors refresh their geometry;
   #3846's growth ratchet must count aliases, public exports of private
   classes, members shared across unrelated classes, whole optional feature
   groups and the tracked special methods.
2. Counted stepping (M1d), re-qualified on current `main`, then the
   remapping clone and `WorldBatch`.
3. The facade slice, under the approved naming rulebook.
4. The typed solver policy with its per-solver parameters (M1e), and the
   written `World` and solver contract.
5. Add the macOS and Windows Python 3.15 wheels to 6.20.x once conda-forge's
   `bullet-cpp` builds for Python 3.15.

## Risks and recovery

- Main moves quickly. Before every approved push to a published branch, merge
  the latest `origin/main` (never rebase a published branch) and re-run the
  packet's gates.
- Other sessions also merge `main` into program branches and merge program
  pull requests. Fetch a branch's remote head before pushing to it, and push an
  API-budget update together with the API it covers: #3842 merged without its
  budget update, which #3906 then supplied.
- `BodyNode` ids come from a process-wide counter that is not atomic, so
  building models on several threads at once is a data race. Tests that build
  models concurrently serialize construction; fixing the counter changes an
  installed header and needs approval.
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
