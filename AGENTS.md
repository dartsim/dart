# Agent Guidelines for DART

This file is the single shared rules file and repository pointer board for
Claude Code, Codex, and Muse; keep shared guidance here (see
`docs/onboarding/ai-tools.md`). Start every task with
`docs/ai/principles.md`, then load only the owner docs needed for the work.
Delegate by default under `docs/ai/orchestration.md`; honor a per-task
`delegation=off` opt-out.

@docs/ai/principles.md

## Project Profile

- **WHAT**: C++17 robotics physics engine with dartpy bindings (nanobind)
- **WHY**: A Python-first DART 7 that stays compatible with Gazebo/gz-physics
- **HOW**: Use repository `pixi run ...` tasks; run `pixi run lint` before commits

## Quick Commands

```bash
pixi run config       # Configure the default CMake/Ninja build
pixi run build        # Build C++ libraries and utilities
pixi run test         # Build and run C++ tests
pixi run build-py-dev # Build dartpy
pixi run test-py      # Run dartpy tests
pixi run test-all     # Build defaults and run C++/Python tests
pixi run test-gui     # Run Filament GUI tests that need a graphics device
pixi run lint         # Format code, docs, TOML, and spelling (auto-fixes)
pixi run check-lint   # Run the non-mutating lint aggregate
```

For downstream-sensitive changes, also run `pixi run -e gazebo test-gz`.
See `docs/onboarding/ci-cd.md` when a gate fails.

## Task-specific Context

| Task                                 | Load these owners                                                                                     |
| ------------------------------------ | ----------------------------------------------------------------------------------------------------- |
| Any task                             | `docs/ai/principles.md`                                                                               |
| Architecture or component boundaries | `docs/onboarding/architecture.md`                                                                     |
| Building or dependencies             | `docs/onboarding/building.md`, `docs/onboarding/build-system.md`                                      |
| Testing or simulation evidence       | `docs/onboarding/testing.md`, `docs/ai/verification.md`; use `dart-verify-sim`                        |
| Performance investigations or claims | `docs/onboarding/profiling.md` § "Performance Methodology"; use `dart-perf` |
| Contribution, branches, or style     | `docs/onboarding/contributing.md`, `docs/onboarding/code-style.md`, `CONTRIBUTING.md`                 |
| Documentation placement              | `docs/README.md`, `docs/information-architecture.md`, `docs/AGENTS.md`                                |
| AI workflows or tooling              | `docs/ai/README.md`, `docs/ai/workflows.md`, `docs/ai/terminology.md`, `docs/onboarding/ai-tools.md`  |
| Planning or autonomous work          | `docs/ai/north-star.md`, `docs/plans/dashboard.md`, `docs/ai/orchestration.md`, `docs/ai/sessions.md` |
| CI failures                          | `docs/onboarding/ci-cd.md`                                                                            |
| Python bindings                      | `docs/onboarding/python-bindings.md`                                                                  |
| Model loading and parsers            | `docs/onboarding/io-parsing.md`                                                                       |
| Release maintenance                  | `docs/onboarding/release-management.md`                                                               |
| Changelog decisions                  | `docs/onboarding/changelog.md`, `docs/onboarding/release-management.md`                               |
| Multi-session dev tasks              | `docs/dev_tasks/README.md`                                                                            |

Subdirectories may provide a closer `AGENTS.md`; instructions accumulate from
the repository root to the working directory.

## Compatibility Rules

- Branch from the resolved target into a non-tracking topic branch
  (`origin/main` by default); never commit directly to `main` or `release-*`.
- `main` is the Python-first DART 7 line ([PLAN-624](docs/plans/dashboard.md));
  `release-6.20` is the DART 6 maintenance line and keeps its own checkout's rules.
- On `main`, preserve C++17, the nanobind binder, `dart::utils` parsers, Filament,
  installed headers, package components (including legacy forwarding components),
  and source and behavior compatibility of the C++ API used by Gazebo/gz-physics
  and gz-sim. PLAN-624 approves the dartpy clean break, the OSG-to-Filament
  renderer replacement merged in #3696, and labeled default-behavior changes
  behind its gates; any other public-API, ABI-layout, or default-behavior change
  needs maintainer approval.
- dartpy uses nanobind as its only binder, under `python/dartpy/`; follow
  `docs/onboarding/python-bindings.md`.
- New fixes land on `main`; backports to a `release-*` branch use
  `dart-backport-pr`. Target the release branch for release-specific packaging,
  CI, and branch guidance. Resolve the next release and milestone from the
  fetched target's `docs/onboarding/release-management.md`.
- Use `.github/PULL_REQUEST_TEMPLATE.md` and set the resolved release milestone
  on PRs.
- Do not prefix commit messages or PR titles with agent tags like `[codex]`;
  use plain descriptive titles.
- Keep private, local, and machine-specific paths out of published files,
  commit messages, and PR text; validate with `scripts/check_local_paths.py`.
- Before every approved push to a published PR branch, first merge the latest
  base branch into it — merge, never rebase a published PR branch.
- GitHub mutations, pushes, PR changes, CI reruns, review-thread mutations, and
  branch deletion require explicit maintainer/user approval.
- Open PRs as drafts, then mark them ready and merge them only as
  `docs/onboarding/ai-tools.md` § "PR Lifecycle" describes.
- Never reply to AI-generated review comments (bot users such as
  `chatgpt-codex-connector[bot]`, `github-code-quality[bot]`,
  `github-actions[bot]`, `copilot[bot]`); make approved local fixes silently.

## AI Capability Sources

Editable workflows and domain skills live in `.claude/commands/` and
`.claude/skills/`. Generated Codex and Muse adapters live in
`.agents/skills/`; do not hand-edit them. Treat
only paths listed by `.agents/skills/.dart-generated.json`, plus the
manifest itself, as DART-generated output: preserve unrelated skills in the
shared `.agents/skills/` directory, and let sync remove only
manifest-owned paths.

```bash
pixi run sync-ai-commands
pixi run check-ai-commands
pixi run python scripts/setup_ai.py
pixi run python scripts/check_ai_infrastructure.py --doctor
```

Under Muse's default shell sandbox, `.git/` and `.agents/` stay
read-only, so run Git mutations (switch, commit, merge) and
`sync-ai-commands` from an unsandboxed shell there (see
`docs/onboarding/ai-tools.md`).

Use `$dart-ultrawork` in Codex or `/dart-ultrawork` in Claude and Muse
Code for large, multi-session, or explicitly autonomous work. Its
project home is `docs/dev_tasks/<task>/`.

## Before Every Commit

- Run `pixi run lint`, even for docs-only changes.
- Run `pixi run build` and focused tests when C++ or Python behavior changes;
  GUI changes also run `pixi run test-gui`.
- Use `dart-changelog` to decide whether `CHANGELOG.md` needs an entry.
- Promote durable facts and remove a completing `docs/dev_tasks/<task>/` folder.
- Run `pixi run install-hooks` once per clone. Its fast staged safety check is
  not a substitute for the full lint requirement. When it blocks a commit, fix
  the reported failure; `DART_SKIP_HOOKS=1` is a maintainer emergency bypass
  to use only when a maintainer explicitly asks.
