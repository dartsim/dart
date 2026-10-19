# Resume

## Current state (2026-10-19)

- The project home lives on the local, unpublished branch
  `docs/dart7-dependency-plan`. Publishing it needs approval.
- The maintainer answered the first decision batch (DD-3 to DD-8 in
  [decisions.md](decisions.md)). A follow-up study is running for asset
  networking, SDF packaging with sdformat kept, the urdfdom floor, the
  fetch-first acquisition matrix and in-tree bundled code.
- Wave 1 is running, each packet in its own worktree on an engine branch
  based on `origin/main` `548ff24ad` (#3829 merged, which DEP-12 builds on):

  | Packet | Engine branch | Writer |
  | --- | --- | --- |
  | DEP-0 | `claude/dep0-gz-probe-gate` | Claude Sonnet |
  | DEP-11 PRs 1-2 | `codex/dep11-cxx20-clean` | Codex (high) |
  | DEP-14 | `codex/dep14-log-sink` | Codex (ultra) |

- Two local spike branches hold experiment edits only and will not be
  published: `spike/cxx20-standard` (built and exported as C++20) and
  `spike/cxx20-private` (option B). Their worktrees also hold Gazebo lane
  builds and the C++17 `base` variants; remove both worktrees once DEP-21's
  own evidence replaces them.
- gz-sim's `INTEGRATION_reset_sensors` is an external test defect; the DART 7
  program session documents it and drafts the upstream report.

## Paused state (2026-10-19, usage limit)

- DD-9 (balance, not blind removal) is recorded but uncommitted in this
  branch's working tree. The balance review and the follow-up study's final
  revision were stopped; resume each from its workflow run ID in the same
  session, or rerun them. The follow-up study's verified results, draft and
  critique were extracted to the orchestrator's local scratch files.
- DEP-1 is committed locally on `build/remove-boost`, with two clean reviews
  (Codex, Muse). Its Gazebo lane run is still owed.
- DEP-0 has fixes from both reviews applied and uncommitted in its worktree.
  It needs a second review round, a `config-gz` re-check, and a decision on
  `test-gz`'s single configure, which tools/gazebo/README.md documents.
- DEP-14's fix round (Codex) and DEP-11 (Codex) may still be running in their
  worktrees; read their reports before reviewing.
- DEP-12(a)'s Sonnet worker finished; its report is unread.
- DEP-15 (in-house fmt formatter) is on hold until the balance review rules
  under DD-9.

## Next steps

1. Review each Wave 1 report against its done-when list, get two reviews from
   engines other than the writer, run the Gazebo lanes against a base variant
   where the packet touches installed headers or package config, and commit
   on a packet branch named for DART's conventions.
2. Present the follow-up study to the maintainer and record the answers.
3. Wave 2: DEP-1, DEP-12(a), DEP-13, DEP-3, DEP-4(a)(c)(d)(e), DEP-5, DEP-8,
   DEP-9, DEP-10; then the packets the study unblocks.
4. Owed spikes: the Werror C++20 budget on every toolchain, `abidiff` between
   the C++17 and C++20 builds, the stock-noble Tier 2 build, and the triage of
   the distro FCL assertion in `test_Distance`.

## Risks and recovery

- `main` moves quickly. Re-check file and line references in
  [packets.md](packets.md) on current `main` before each packet.
- The step-path quiet window is open until M1d merges: `World.cpp` and
  `lcpsolver` hunks wait or go through the PLAN-624 owner.
- The merging PR regenerates `pixi.lock`; never hand-merge it.
- Gazebo lanes: compare against a base variant built from the packet's base
  (`GZ_COMPAT_BASE_VARIANT`); `reset_sensors` fails on every base today.

## Verification commands

- `pixi run lint`, then `pixi run check-lint`.
- `pixi run build`, `pixi run test`, `pixi run test-py`, and
  `pixi run test-gui` for GUI changes.
- `pixi run -e gazebo test-gz`, `pixi run -e gazebo gz-compat-ionic`,
  `pixi run -e gazebo-harmonic gz-compat-harmonic`,
  `pixi run -e gazebo-jetty gz-compat-jetty`.
- `pixi run test-package-core`; `pixi run test-build-requirements` when floors
  change.
