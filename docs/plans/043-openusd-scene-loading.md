# PLAN-043: OpenUSD Scene Loading

- Operating state: `PLAN-043` in [dashboard.md](dashboard.md).
- Outcome: load USD scenes through DART's unified IO front door into DART 7
  Worlds, with a maintained Filament viewer and stable C++/Python workflows.
- Current evidence: PR #3109 (`fab38a336f7`) landed the textual `.usda`
  scaffold, `ModelFormat::Usd` inference, OFF-path diagnostic, gated integration
  test and `DART_BUILD_IO_USD=OFF` default. The parser currently creates a
  single default-prim link; this is not full USD scene or USDPhysics support.
- Durable usage and implementation boundaries:
  [IO handbook](../onboarding/io-parsing.md#openusd-usda-is-an-opt-in-format),
  [`usd_parser.hpp`](../../dart/io/usd/usd_parser.hpp),
  [GUI handbook](../onboarding/gui-rendering.md) and PLAN-041/042.

## Retirement Decision And Restart Conditions

On 2026-09-06 the maintainer chose to preserve unfinished work here and retire
the USD task handoff. This replaces the 2026-07-04 decision to retain a parked
task folder. It does not authorize dependency enablement or claim completion.
Create a new task home only when the work is selected through the dashboard.

The missing OpenUSD/pxr build environment remains a prerequisite to parser
implementation and ON-path verification. Adding OpenUSD to a Pixi feature or
dedicated CI environment is a separate infrastructure slice; do not infer that
the old concurrent `pixi.toml` lane is still active. Recheck current ownership
and package/toolchain feasibility when that slice is selected. Keep the loader
OFF by default while qualification is incomplete.

The prototype's macOS dartpy pytest abort remains unresolved historical
evidence. Plugin-path and fork-safety explanations were hypotheses, not a
diagnosis. Reproduce and root-cause it in an OpenUSD-enabled environment before
enabling the loader by default on Linux/macOS.

## Remaining Work And Acceptance

1. **Build environment:** provide a reproducible OpenUSD-enabled build and
   exercise the gated `INTEGRATION_io_Usd` test. Preserve a default OFF build
   with no required pxr dependency and a clear unavailable-format diagnostic.
2. **Scene mapping (former Phase 2):** extend the scaffold to child prims,
   links/joints, shapes and inertia, preserving the unified `dart::io` reader
   and `dart::simulation::io::addSkeleton` conversion boundary. Verify a
   multi-prim rigid chain and the Unitree H1 minimal sample when its assets are
   recovered. Keep parser internals out of the public API.
3. **Viewer (former Phase 3):** add a Filament/dartsim example plus headless
   smoke and assessed visual evidence. The earlier OSG viewer is not a valid
   implementation starting point; PLAN-101 owns the maintained application.
4. **Python and platform qualification (former Phase 4):** expose the supported
   scene workflow through dartpy, with Linux/macOS tests and the macOS abort
   resolved. Choose the final Python surface under PLAN-041/042's API policy.

Retain read-only textual `.usda` as the initial envelope. Binary Crate `.usdc`,
packaged `.usdz` and broader USDPhysics mapping remain explicit later work;
USD export and arbitrary authoring are outside the initial loader task.
Qualification must distinguish parsed metadata from semantics actually mapped
into the DART 7 World.

The default path retains lint, build, unit/Python and API-boundary gates, including
`ReadUnit.InfersFormatFromUsdExtension`. The ON path must run the extended
`tests/integration/io/test_usd_parser.cpp`, not report success from its default
compile-out. Full CPU/CUDA gates apply according to
[verification policy](../ai/verification.md); CUDA does not substitute for the
OpenUSD-enabled loader and platform evidence.

## Prototype Recovery Evidence

The former `feature/usd-viewer` prototype is not assumed reachable. Historical
references are feature commit `28bad2773d18ce47bbaef98508d385abe1a8aead` and the
macOS diagnostic tail `82f9a261361..b7a7ac09823`. The old task reported those
objects unavailable in its reflog/fsck state; another clone may be necessary.
Do not treat failed lookup as proof that the prototype never existed.

Useful recovery targets were the old `UsdParser` implementation, pxr finder,
`data/usd/{simple_chain,unitree_h1_minimal}.usda`, Python parser bindings and
`test_usd_parser.py`. The current snake_case parser/finder and Filament policies
supersede the prototype's PascalCase paths and OSG example. Recovery is evidence
for the new implementation, not a request to restore those old interfaces.
