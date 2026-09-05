# DART Developer Handbook

This directory is DART's developer handbook: durable contributor, maintainer,
workflow, release, CI, API-policy, and module explanations that let humans and
AI agents understand the project without rediscovering it from raw code. The
directory name predates that role; see [`../README.md`](../README.md) for the
docs map and placement rules.

## Start Here

- Key command: `pixi run test-all` (lint + build + all default-environment
  tests); on Linux CUDA hosts also run `pixi run -e cuda test-all`.
- Building DART? → [building.md](building.md)
- Running tests? → [testing.md](testing.md)
- Contributing code? → [contributing.md](contributing.md) + [code-style.md](code-style.md)
- Updating release notes? → [changelog.md](changelog.md)
- Understanding the engine? → [architecture.md](architecture.md) (classic
  core) and [`../readthedocs/architecture.md`](../readthedocs/architecture.md)
  (DART 7 direction)
- Gazebo integration? → [build-system.md](build-system.md#gazebo-integration-feature)
- Verifying model, simulation, collision, or GUI behavior? →
  [agent-sim-verification.md](agent-sim-verification.md)

## Principles

Handbook pages explain design decisions and the reasons behind them, point to
code as the source of truth (`CMakeLists.txt`, `pixi.toml`, headers), and
describe the current state rather than history. They avoid hardcoded
dependency, file, and version lists that go stale, repeat nothing that is
obvious from the code, and carry no generated footer metadata (enforced by
`pixi run check-lint`). When a fact changes often, link to its authoritative
source instead of copying it.

Design work follows the same rule: start with the simplest solution for the
current problem and add abstraction, hierarchy, or configuration only when a
real pain point demands it.

## Pages

### Build, test, contribute

| Page                                               | Covers                                                                           |
| -------------------------------------------------- | -------------------------------------------------------------------------------- |
| [building.md](building.md)                         | Building from source on every supported platform with pixi; CMake options        |
| [build-system.md](build-system.md)                 | CMake and pixi internals: targets, options, dependencies, Gazebo/gz-physics lane |
| [testing.md](testing.md)                           | Test suite layout, unit vs integration, running and debugging tests              |
| [ci-cd.md](ci-cd.md)                               | GitHub Actions workflows, caching, platform-specific failures                    |
| [contributing.md](contributing.md)                 | Branching, PR workflow, dual-PR bugfix rule, review process                      |
| [code-style.md](code-style.md)                     | C++, Python, and CMake conventions, including snake_case headers                 |
| [changelog.md](changelog.md)                       | When and how to write `CHANGELOG.md` entries                                     |
| [compatibility-policy.md](compatibility-policy.md) | Compiler, C++ standard, and dependency floors tied to Ubuntu LTS                 |
| [error-handling.md](error-handling.md)             | Error-handling philosophy and APIs                                               |

### Architecture and modules

| Page                                     | Covers                                                                                |
| ---------------------------------------- | ------------------------------------------------------------------------------------- |
| [architecture.md](architecture.md)       | Classic DART core: module layers, `World::step` pipeline, key algorithms and patterns |
| [dynamics.md](dynamics.md)               | `dart/dynamics`: Skeleton, BodyNode, Joint, kinematics and dynamics APIs              |
| [constraints.md](constraints.md)         | Constraint solver internals and collision response                                    |
| [aspect-system.md](aspect-system.md)     | Aspect, State, and Properties runtime extension pattern                               |
| [io-parsing.md](io-parsing.md)           | Unified model loading through `dart::io` (URDF, SDF, MJCF, USD)                       |
| [gui-rendering.md](gui-rendering.md)     | Filament renderer, GLFW windowing, ImGui overlays, headless capture                   |
| [python-bindings.md](python-bindings.md) | nanobind `dartpy` architecture, stubs, wheels                                         |
| [api-boundaries.md](api-boundaries.md)   | Public vs internal API policy, Python exposure, symbol visibility, policy gate        |
| [profiling.md](profiling.md)             | Text-first profiling of the DART 7 `World` step                                       |

For the DART 7 multi-physics, multi-solver, multi-backend design see
[`../readthedocs/architecture.md`](../readthedocs/architecture.md) and the
rationale in [`../design/`](../design/README.md); theory lives in
[`../background/`](../background/README.md).

### Release and published docs

| Page                                               | Covers                                                                                  |
| -------------------------------------------------- | --------------------------------------------------------------------------------------- |
| [release-roadmap.md](release-roadmap.md)           | DART 6 LTS vs DART 7 clean-break strategy, gates, compatibility and deprecation policy  |
| [release-management.md](release-management.md)     | Release closeout, backports, release-branch CI fixes, packaging, merge-back to main     |
| [api-documentation.md](api-documentation.md)       | Read the Docs and GitHub Pages pipeline, Doxygen and dartpy API inputs, troubleshooting |
| [dart7-docs-migration.md](dart7-docs-migration.md) | Published pages that still carry DART 6 content and what unblocks their port            |

### AI tooling

| Page                                                   | Covers                                                       |
| ------------------------------------------------------ | ------------------------------------------------------------ |
| [ai-tools.md](ai-tools.md)                             | Claude Code and Codex setup, compatibility, dated evidence   |
| [ai-reviews.md](ai-reviews.md)                         | Handling automated review bots and the review-fix loop       |
| [agent-sim-verification.md](agent-sim-verification.md) | Text-first plus visual verification of 3D scenes and physics |

Agent workflows are `/dart-*` commands in Claude Code and `$dart-*` skills
in Codex; [`../ai/README.md`](../ai/README.md) owns the
operating model and [`../ai/workflows.md`](../ai/workflows.md) the catalog.
