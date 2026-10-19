# Resume

## Current state (2026-10-19)

- The project home lives on the local, unpublished branch
  `docs/dart7-dependency-plan`. Publishing it needs approval.
- Wave 1 is committed locally, one branch per packet, each rebased onto
  `origin/main` `0d7f6d77a` with two clean reviews from engines other than
  its writer:

  | Packet | Branch |
  | --- | --- |
  | DEP-0 | `test/gz-contact-surface-probe-gate` |
  | DEP-1 | `build/remove-boost` |
  | DEP-11 PR1 | `refactor/cxx20-clean-code` |
  | DEP-12(a) | `build/ubuntu-2404-toolchain-floor` |
  | DEP-14 | `refactor/dart-log-sink` |

  The Harmonic, Ionic and Jetty lanes pass on their combined tree, and the
  local integration branch `int/dart7-deps-wave1` passes the default gates
  ([verification.md](verification.md)). Pushing the branches and opening
  draft PRs needs approval; each CHANGELOG entry gets its PR link in a
  follow-up commit once the PR exists.
- Wave 2 is committed locally, one branch per packet on `0d7f6d77a`
  (DEP-27a on DEP-28), each with two clean reviews
  ([verification.md](verification.md#wave-2-2026-10-19)):
  `refactor/delete-dead-files`, `build/enforce-filament-pin`,
  `fix/gui-texture-decode-limits`, `fix/gui-empty-overlay-font`,
  `refactor/urdf-glue-cleanup`, `fix/voxel-ray-key-limit`,
  `build/bullet-collision-libraries-only`, `build/third-party-notices`,
  `build/assimp-5.3-floor`. The Harmonic, Ionic and Jetty lanes pass on
  their combined tree against `0d7f6d77a` base variants.
- DEP-15 (fmt replaced by a standard-library formatter, the Q1 default) is
  committed locally on `refactor/remove-fmt`, stacked on DEP-14's branch;
  it lands after DEP-12(a).
- The balance review under DD-9 ([balance-review.md](balance-review.md))
  re-scoped the packet plan. Its second decision batch (Q1 to Q5 and the
  approvals) is open in [decisions.md](decisions.md). After the answers,
  apply the review's §3 "Project-home corrections" to `decisions.md`,
  `inventory.md`, `README.md` and `packets.md`.
- Two local spike branches hold experiment edits only and will not be
  published: `spike/cxx20-standard` and `spike/cxx20-private`. Their
  worktrees also hold the Gazebo lane builds and the `base548` variants;
  remove both once DEP-21's own evidence replaces them.
- gz-sim's `INTEGRATION_reset_sensors` is an external test defect; the DART 7
  program session documents it and drafts the upstream report.

## Next steps

1. Record the maintainer's answers; publish Wave 1 once approved.
2. Run the decision-free packets in the review's §3 "Order", item 2,
   starting with those that do not depend on an unmerged Wave 1 branch.
3. Owed spikes: the Werror C++20 budget on every toolchain, `abidiff` between
   the C++17 and C++20 builds, and the stock-noble Tier 2 build. The distro
   FCL assertion is triaged ([verification.md](verification.md)); filing the
   upstream FCL and Debian/Ubuntu reports needs approval.
4. After M1d: DEP-21, the `World.cpp` captures from DEP-11, and any
   quiet-window hunk the PLAN-624 owner did not clear earlier.

## Risks and recovery

- `main` moves quickly. Re-check file and line references in
  [packets.md](packets.md) and the review on current `main` before each
  packet.
- The step-path quiet window is open until M1d merges: `World.cpp` and
  `lcpsolver` hunks wait or go through the PLAN-624 owner.
- The merging PR regenerates `pixi.lock`; never hand-merge it.
- Gazebo lanes: compare against a base variant built from the packet's base
  (`GZ_COMPAT_BASE_VARIANT`); `reset_sensors` fails on every base today.
- Never edit a lane wrapper script while a lane that runs it is still going.

## Verification commands

- `pixi run lint`, then `pixi run check-lint`.
- `pixi run build`, `pixi run test`, `pixi run test-py`, and
  `pixi run test-gui` for GUI changes.
- `pixi run -e gazebo test-gz`, `pixi run -e gazebo gz-compat-ionic`,
  `pixi run -e gazebo-harmonic gz-compat-harmonic`,
  `pixi run -e gazebo-jetty gz-compat-jetty`.
- `pixi run test-package-core`; `pixi run test-build-requirements` when floors
  change.
