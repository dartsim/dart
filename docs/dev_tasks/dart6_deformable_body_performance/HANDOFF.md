# Handoff - DART 6 deformable body performance

Updated: 2026-07-12

This is a cross-agent/worktree handoff for PR #3382 stabilization. The complete
fresh-session state and commands are in `RESUME.md`; do not use the historical
pre-PR benchmark instructions that this file previously contained.

## Packet contract

**Goal:** land the current deformable-body release slice through #3382 with
review findings fixed, gates honest, and DART 6 compatibility preserved.

**Done when:**

- the mass-matrix review correction is published and the thread is addressed;
- the current PR head has the required clean review evidence;
- branch-local CI is green and exact-base/infrastructure failures are
  dispositioned without weakening product gates;
- remaining competitive-envelope/packet scope decisions are explicit, and
  task docs plus durable owners describe the final landed/open state.

**Risks:** public matrix correctness, DART 6 ABI/API compatibility, default-off
bit identity, noisy performance evidence, and confusing current branch-local
failures with failures already present on `release-6.20`.

## Current checkout

```text
worktree  /home/js/dev/dartsim/dart/task_2
branch    wp-db-native-soft-fallback
impl      2ad156e7b82  (followed locally by this handoff/docs update)
origin    b25462ca5c0
base      fa17fad79b9
PR        https://github.com/dartsim/dart/pull/3382
```

`wp-db-soft-skel-allocation-gates` is already fully incorporated and stale
against the release branch. Do not resume it.

## Completed locally

Commit `2ad156e7b82` fixes the current P2 mass-matrix review finding and adds a
regression proving retained point-mass accelerations cannot affect public mass
or augmented-mass matrices. Verification on that commit:

```text
pixi run lint                                      PASS
DART_DISABLE_COMPILER_CACHE=ON pixi run build     PASS
DART_DISABLE_COMPILER_CACHE=ON pixi run test      PASS (152/152)
test_SoftDynamics                                 PASS (16/16)
INTEGRATION_StepAllocation                        PASS
independent post-fix reviews                      CLEAN x2
```

No GitHub mutation was made for this packet. Before an approved push, fetch and
merge the latest `origin/release-6.20` if it moved, then reverify the merged
state.

## Hosted blockers

- Linux coverage/assert failures reproduce on exact base run `29178779447` in
  `test_MjcfParser` at the same NaN assertion.
- Windows run `29188317164` reached the 300-minute workflow timeout mid-build.
- The automated mass-matrix review thread remains unresolved until the local
  fix is published.
- A fresh Codex review is temporarily quota-blocked; avoid duplicate triggers.
- The original WP-DB.07/WP-DB.08 contracts and several paper-matrix rows are
  not complete; promote or explicitly disposition them before task retirement.
- The formal competitive-implementation envelope still needs maintainer
  sign-off.

See `RESUME.md` for the ordered next actions, dual-PR applicability, and
approval boundaries.
