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

## Revision Triggers

Revise this plan when:

- package availability changes;
- public API names, defaults, or binding shape changes;
- Pixi tasks change;
- a first-success workflow fails locally or in CI;
- user feedback shows a new setup or API friction point.
