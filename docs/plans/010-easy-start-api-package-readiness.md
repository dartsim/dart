# PLAN-010: Easy-Start API and Package Readiness

- Operating state: `PLAN-010` in `docs/plans/dashboard.md`

## Outcome

A new researcher can install or build DART, run a first simulation, and use the
common-path public API from names, types, defaults, and short examples without
reading a long user guide first.

## Scope

This plan covers first-success workflows:

- Python package install and first simulation.
- C++ package install and first simulation.
- Pixi source checkout, build, and first simulation without manually installing
  dependencies.
- Public API clarity for common-path loading, world creation, stepping,
  state access, and simple visualization.

Out of scope for this plan:

- Full API redesign.
- Release publishing automation.
- Detailed implementation tracking; use `docs/dev_tasks/` if this becomes
  multi-session implementation work.

## Current Evidence

- `README.md` already lists Python, C++, package-manager, and source-oriented
  entry points.
- `docs/onboarding/building.md` documents Pixi source workflows.
- `docs/onboarding/python-bindings.md` documents the Python-first API direction.
- `docs/onboarding/api-boundaries.md` defines public API expectations and
  Python binding boundaries.
- `pixi.toml` owns reproducible source-build and test tasks.

## Workstreams

### 1. First-Success Workflow Matrix

Define the smallest supported path for each user entry point:

| Entry point    | Expected first success                                                    | Evidence to collect                             |
| -------------- | ------------------------------------------------------------------------- | ----------------------------------------------- |
| Python package | Install `dartpy`, create a world, load or create a model, step simulation | Commands, output, supported Python versions     |
| C++ package    | Install C++ package, compile a tiny example, run simulation               | Package commands, CMake snippet, output         |
| Pixi source    | Clone, `pixi install`, build, run a sample                                | Exact Pixi commands and expected success signal |

The matrix should prefer short examples over long prose. Long-form docs should
explain tradeoffs after first success.

Current matrix:

| Entry point    | Current documented install path                                                                                                                              | First-success run path                                                                                                                                                    | Evidence status                                                                                                                                                                                                                                   |
| -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Python package | `uv add dartpy`, `pip install dartpy`, `pixi add dartpy`, or `conda install -c conda-forge dartpy` from `README.md` and `docs/onboarding/python-bindings.md` | Use the README current-package smoke check for published packages; use the DART 7 Python quick-start snippet when running from `main` or future DART 7 package artifacts. | `pixi run test-published-package-quickstarts` installed current `dartpy 6.16.7`, created a file-free skeleton/world, stepped once, and reported `positions_size=6`; the DART 7 snippet passes against the in-tree Release dartpy build.           |
| C++ package    | `pixi add dartsim-cpp` or `conda install -c conda-forge dartsim-cpp` from `README.md`; platform package fallbacks are listed there too.                      | Use the README current-package smoke check for published packages; use the DART 7 C++ quick-start snippet when running from `main` or future DART 7 package artifacts.    | `pixi run test-published-package-quickstarts` built and ran a file-free C++ snippet against current conda-forge `dartsim-cpp`, reporting `positions_size=6`; local DART 7 installed-package verification passed against `/tmp/dart-plan-install`. |
| Pixi source    | `pixi install`, then `pixi run config`, `pixi run build`, or `pixi run test-all` from `docs/onboarding/building.md`.                                         | `pixi run ex headless_simulation --steps 1` runs a deterministic, non-GUI example through the repo's example runner.                                                      | Local source verification passed: the command built `headless_simulation`, loaded two skeletons, stepped once, and reported simulated time `0.001`.                                                                                               |

The immediate friction is release timing, not first success: the README now
points to short DART 7 source snippets and short current-package smoke checks.
Local source, local DART 7 package-project, and currently published package
verification all pass; DART 7 public package artifact publication remains a
release follow-up.

Package-project evidence from 2026-05-17 local verification:

- Regenerated the Release build tree with `DART_BUILD_DARTPY=OFF` to avoid
  Python install RPATH rewriting during temporary package installation.
- Installed the current build to `/tmp/dart-plan-install` with
  `cmake --install build/default/cpp/Release --prefix /tmp/dart-plan-install`.
- Configured and built the packaged `examples/headless_simulation` project with
  `CMAKE_PREFIX_PATH=/tmp/dart-plan-install;<pixi-env>`, then ran it with
  `--steps 1`; it loaded two skeletons and advanced simulated time to `0.001`.
- Configured, built, and ran a minimal CMake project containing the README C++
  snippet against the same temporary install; it produced 100 position lines.
- The installed-package configure path required exporting transitive component
  dependencies for `Tracy` from `dart` and `sdformat` from `dart-utils`.

Published-artifact evidence from 2026-05-17 local verification:

- `pip install dartpy` in an isolated Python 3.12 venv installed `dartpy 6.16.7`;
  importing it showed no top-level `dart.World` or `dart.io`, so the DART 7
  README Python quick start is not yet valid for the current PyPI artifact.
- PyPI metadata reports `dartpy 6.16.7` as the latest normal version and
  `dartpy 7.0.0` as yanked. Installing the exact yanked `dartpy==7.0.0` wheel
  succeeds with a yanked-version warning, but importing it still showed no
  top-level `dart.World` or `dart.io`, so it is not a suitable DART 7 package
  unblocker for the README quick start.
- `pixi search dartpy` reported conda-forge `dartpy 6.16.7` as the latest
  available package metadata.
- `pixi search dartsim-cpp` reported conda-forge `dartsim-cpp 6.16.7` as the
  latest available package metadata, so the DART 7 C++ package quick start is
  blocked on publishing current release artifacts.
- GitHub release evidence checked on 2026-05-17 did not find a remote `v7*` tag
  or a public `v7.0.0` release; the release API returned 404 for `v7.0.0`.
- The `publish_dartpy` workflow run
  `https://github.com/dartsim/dart/actions/runs/25994215823` on `main` passed
  for Linux, macOS, and Windows wheels across Python 3.12, 3.13, and 3.14 and
  uploaded nine workflow artifacts, but the `Wheels | Publish to PyPI` job was
  skipped because the run was a branch push rather than a `v*` tag push. That
  means current wheel build/test evidence is available, while public package
  publication remains blocked on a release tag and upload path.
- `scripts/test_installation.py` now exercises the same DART 7 Python
  first-success API as the README quick start. It passed against the local
  in-tree Release dartpy build with
  `PYTHONPATH=build/default/cpp/Release/python:${PYTHONPATH:-}`.
- `scripts/test_wheel.py` now invokes `scripts/test_installation.py` after
  installing a wheel into its temporary environment, so wheel tests exercise the
  same DART 7 Python first-success API instead of only legacy module paths.
- `pixi run test-cpp-quickstart -- --prefix /tmp/dart-plan-install` now builds
  and runs a temporary CMake project with the README C++ quick-start API against
  the local installed DART prefix and the pixi dependency prefix; the local run
  reported `positions_size=6`.
- `pixi run test-published-package-quickstarts` now verifies the README
  current-package smoke checks against published packages. The Python check
  creates an isolated environment, installs `dartpy 6.16.7`, steps a file-free
  skeleton/world, and reports `positions_size=6`. The C++ check uses
  `pixi exec -s dartsim-cpp -s cmake -s cxx-compiler`, builds a file-free
  skeleton/world snippet against the current conda-forge package, and reports
  `positions_size=6`.

Artifact recheck commands:

```bash
pixi run check-dart7-artifacts
```

The task exits nonzero while either the Python or C++ DART 7 package artifact is
missing, and exits zero once both package surfaces report DART 7 artifacts. It
also reports whether a remote DART 7 tag or public release exists so maintainers
can distinguish "wheels build in CI" from "release publication happened."

Artifact unblock checklist:

- Python package metadata shows a non-yanked DART 7 artifact through the normal
  package path, or conda-forge reports a DART 7 `dartpy` artifact.
- If PyPI is the Python package path, a `v*` release tag exists and the
  `publish_dartpy` tagged workflow run completes the `Wheels | Publish to PyPI`
  job successfully.
- Installing that artifact into an isolated environment and running the README
  Python quick start succeeds with top-level `dart.World`, `dart.io`,
  `parse_skeleton`, `add_skeleton`, and `get_positions`.
- Running `python scripts/test_installation.py` in that isolated environment
  passes all installation sanity checks.
- Running the relevant `pixi run -e py<version>-wheel wheel-test` task passes
  after installing the wheel into its temporary environment.
- `pixi search dartsim-cpp` reports a DART 7 C++ package artifact.
- `pixi run test-cpp-quickstart -- --prefix <published-dart-prefix>`
  succeeds against the published C++ package.

### 2. Common-Path API Audit

Audit whether the common path is self-explanatory:

- world creation;
- skeleton/model loading;
- adding a model to a world;
- stepping simulation;
- reading/writing state;
- basic collision/contact access;
- first visualization path when GUI support is available.

For each friction point, classify the fix as:

- naming/defaults change;
- missing overload or helper;
- missing short example;
- package/build documentation issue;
- deeper design issue that needs a separate plan or dev task.

Current audit:

| Friction point                                                                                                                               | Classification      | Evidence                                                                                                                                                                 | Bounded follow-up                                                                                                   |
| -------------------------------------------------------------------------------------------------------------------------------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------- |
| README Python quick start now uses preferred snake_case names and passes against the in-tree Release dartpy build.                           | Release timing      | `pip install dartpy` currently installs `dartpy 6.16.7`, so README also includes a current-package smoke check that passes against the published wheel.                  | Re-run the DART 7 snippet after DART 7 PyPI/conda artifacts publish, then remove or demote the DART 6.16 fallback.  |
| README C++ quick start now uses a `dart://` sample resource, the unified `dart::io::readSkeleton` API, and installed-package CMake evidence. | Release timing      | A local minimal CMake project passed against `/tmp/dart-plan-install`; README also includes a file-free current-package C++ smoke check that passes against conda-forge. | Re-run against published DART 7 `dartsim-cpp` artifacts on supported platforms, then remove or demote the fallback. |
| Pixi source path now has a verified non-GUI first simulation command surfaced in README.                                                     | Documentation issue | `pixi run ex headless_simulation --steps 1` built and ran locally, loading two skeletons and advancing simulated time to `0.001`.                                        | Keep this as the source first-success command unless a smaller stable example replaces it.                          |
| Package install commands point at currently published DART 6.16 artifacts, while DART 7 snippets target `main`.                              | Release timing      | `pixi run test-published-package-quickstarts` verifies current package first success; `pixi run check-dart7-artifacts` keeps the DART 7 artifact blocker explicit.       | Publish or locate DART 7 package artifacts, then rerun Python and C++ first-success verification.                   |
| First visualization remains split between classic GUI and active Filament work.                                                              | Deeper design issue | Dashboard PLAN-060 points GUI promotion to the Filament dev task.                                                                                                        | Keep visualization out of the first patch unless the Filament dev task promotes a stable first-viewer command.      |

### 3. Package And Source-Build Readiness

Track whether the first-success workflow is available through:

- PyPI or equivalent Python package path;
- conda-forge/Pixi package path;
- C++ package manager path;
- source build through Pixi.

For each package path, record the supported platform and the verification
command. Do not duplicate package version tables here; link to the authoritative
package or build docs.

## Acceptance Criteria

Active implementation is justified when:

- the first-success workflow matrix is filled with concrete commands;
- the API audit names the first high-value friction points;
- each package/source-build path has a verification command or an explicit
  blocker;
- follow-up work is split into bounded tasks or roadmap entries.

This plan is complete when:

- a fresh user can reach first simulation from Python, C++, and Pixi source
  workflows with short instructions;
- common-path examples exercise the recommended public APIs;
- the relevant docs and package surfaces point to those examples;
- verification commands are documented and reproducible.

Completion evidence from 2026-05-17:

- README now points DART 7 source users at the recommended `main` APIs and
  current package users at file-free smoke checks that work with the published
  DART 6.16 artifacts.
- `pixi run test-published-package-quickstarts` verifies fresh Python and C++
  package first success against the currently published packages.
- `scripts/test_installation.py`, `scripts/test_wheel.py`, and
  `scripts/test_cpp_quickstart.py` verify the DART 7 README API against in-tree
  Python, wheel, and local installed-package paths.
- `pixi run check-dart7-artifacts` keeps DART 7 public package publication as a
  release-timing follow-up rather than a first-success blocker.

## Revision Triggers

Revise this plan when:

- package availability changes;
- public API names, defaults, or binding shape changes;
- Pixi tasks change;
- a first-success workflow fails locally or in CI;
- user feedback shows a new setup or API friction point.
