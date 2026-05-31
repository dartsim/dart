# Rigid IPC Solver - Dev Task

## Current Status

- [x] Phase 0: establish the upstream rigid-ipc fixture, test, benchmark, and
      comparison manifest with validation tooling.
- [ ] Phase 1: fixture/import surface for rigid meshes, fixed DOFs, kinematic
      controls, forces, gravity, friction, restitution, and diagnostics.
  - [x] Phase 1a: internal JSON fixture reader for the first mesh-path subset
        with explicit unsupported-field diagnostics.
  - [ ] Phase 1b: turn loaded fixture records into DART-native replay state
        with mesh resolution, fixture-row ownership, and runtime examples.
    - [x] First internal replay path populates an experimental `World` with
          fixture timestep, gravity, rigid body poses, velocities, loads,
          material coefficients, and body-row metadata.
    - [x] Attach supported OBJ, OFF, rigid-ipc MSH, binary STL, and ASCII STL
          mesh assets to DART-native mesh collision shapes and prove replayed
          mesh bodies participate in `World::collide()`.
    - [x] Preserve inline `vertices`/`edges`/`polygons` fixture records and
          replay polygonal inline geometry as native-backed mesh collision
          shapes.
    - [x] Import the upstream IPC comparison `.txt` shape-row subset, including
          mesh poses, scale, material density/Young/Poisson values, initial
          velocities, prescribed linear/angular velocities, Neumann body
          forces, energy model, warm-start flag, self-collision flag, gravity
          disable flag, friction, barrier distance, friction iterations, and
          tolerance metadata, then replay MSH-backed comparison rows into an
          experimental `World`. Path-loaded scripts now use the script
          directory as the default base for upstream relative mesh paths.
    - [x] Add the first runtime replay regression that steps a fixture-populated
          experimental `World` through the existing rigid-body integration
          pipeline, proving fixture replay remains a population bridge and does
          not select an IPC solver.
    - [ ] Add a runtime fixture example and cover remaining comparison
          script commands and mesh formats beyond OBJ/OFF/MSH/STL/inline
          polygons.
- [ ] Phase 2: rigid curved-trajectory CCD and minimum-separation tests.
  - [x] Phase 2a: internal direct CCD test-data reader for upstream edge-edge
        and face-vertex rows.
  - [x] Phase 2b: first internal rigid curved-trajectory ACCD query for 3D
        face-vertex, edge-edge, and point-edge cases, with rotational
        regressions where endpoint-linear primitive CCD misses the mid-step hit,
        plus first minimum-separation regressions.
  - [ ] Phase 2c: codimensional coverage and direct CCD row parity against the
        audited upstream corpus.
    - [x] Add hermetic load regressions for the first audited root
          `tests/data/ccd-test-000..003.json` rows.
    - [x] Add first upstream-style direct evaluator regressions for expected
          edge-vertex, edge-edge, and face-vertex TOI rows.
    - [x] Add internal parameter-space residual helpers for the edge-vertex,
          edge-edge, and face-vertex equations used by interval-root rigid CCD.
    - [x] Add first internal parameter-box subdivision queries over the
          edge-vertex, edge-edge, and face-vertex root domains, with expected-TOI
          regressions.
    - [x] Route the first direct CCD evaluator rows through those subdivision
          queries with the audited reference TOI tolerance.
    - [x] Evaluate the first audited root direct-CCD rows as full-step misses,
          matching the current DART distance query for those rows.
    - [ ] Match those rows with the audited reference's interval-root rigid CCD
          semantics across the corpus.
- [ ] Phase 3: rigid barrier objective, line search, projected Newton, sparse
      assembly, and solver statistics.
  - [x] Phase 3a: first internal pose-sampled rigid primitive barrier scaffold
        for face-vertex and edge-edge terms. The scaffold reuses the current
        DART IPC C2 clamped-log kernel after rigid trajectory interpolation and
        returns world-primitive derivatives only.
  - [x] Phase 3b: first internal two-body reduced-coordinate chain rule for
        face-vertex and edge-edge barrier terms. The result maps the
        world-primitive barrier gradient and Hessian into 6-DOF rigid-pose
        coordinates, includes PSD projection for solver assembly, and is
        covered against finite-difference derivatives.
  - [x] Phase 3c: complete the first local two-body barrier primitive set with
        edge-vertex and vertex-vertex terms. Shared point-edge and point-point
        barrier kernels now reuse the existing distance derivatives, and rigid
        wrappers expose pose-sampled and reduced-coordinate variants with
        finite-difference coverage.
  - [x] Phase 3d: assemble the first generalized-coordinate rigid barrier
        objective rows across scene surfaces. The internal assembly now owns
        active primitive records, dynamic/static body DOF offsets, global
        gradients, and sparse PSD-projected Hessian triplets for
        vertex-vertex, edge-vertex, edge-edge, and face-vertex rows.
  - [x] Phase 3e: add the first conservative rigid surface line-search
        feasibility helper over matching start/end surfaces. The helper checks
        edge-vertex, edge-edge, and face-vertex rows with curved rigid CCD,
        treats initial separation violations and indeterminate CCD exhaustion
        as zero-step unsafe results, and reports the limiting primitive/body
        pair. Vertex-vertex remains barrier-only for now.
  - [x] Phase 3f: add the first internal projected-Newton step scaffold over
        the assembled rigid barrier gradient and sparse PSD-projected Hessian.
        The helper applies diagonal regularization, returns descent-step
        statistics, and consumes the conservative line-search bound to block or
        scale unsafe candidate steps.
  - [x] Phase 3g: add the first internal barrier-only projected-Newton solve
        loop over copied rigid surfaces. The loop recomputes assembly each
        iteration, applies reduced pose deltas to dynamic surfaces, records
        convergence/progress statistics, and uses the conservative line-search
        helper to scale or block candidate steps.
  - [x] Phase 3h: add the first internal generalized dynamics objective terms.
        Per-body terms assemble diagonal quadratic weights and generalized
        force/torque vectors into the same global 6-DOF pose system as barrier
        rows, and the Newton loop can now move copied dynamic surfaces toward
        those dynamics targets even without active contact.
  - [x] Phase 3i: add the first physical dynamics-term construction helper.
        The helper converts pose, generalized velocity, mass, diagonal inertia,
        generalized force/torque, and timestep into the internal quadratic
        dynamics objective term used by the Newton loop.
  - [x] Phase 3j: add the first opt-in runtime rigid IPC world-step stage. The
        stage extracts mesh-like rigid surfaces plus physical mass/inertia,
        velocity, force, torque, and timestep state from free rigid bodies,
        runs the internal projected-Newton IPC solve, and writes solved
        poses/velocities back without replacing the default rigid contact
        stage.
  - [x] Phase 3k: add same-domain rigid solver selection for the default
        experimental `World` step. `World::setRigidBodySolver()` keeps
        sequential impulse as the default and lets callers opt into the rigid
        IPC stage as the free-rigid dynamics solver without double-applying the
        legacy velocity/contact/position stage sequence.
  - [x] Phase 3l: broaden the runtime rigid IPC extractor from mesh/box shapes
        to analytic spheres by deterministically triangulating sphere collision
        geometry into local rigid barrier surfaces before assembly.
  - [x] Phase 3m: expose durable runtime rigid IPC diagnostics by mapping the
        internal solve status, last step norm, last line-search bound, and
        aggregate conservative CCD line-search counters onto the opt-in stage
        stats.
  - [x] Phase 3n: add the first runtime activated-contact regression. Two
        mesh-surface rigid bodies start inside the IPC activation distance; the
        opt-in stage assembles active constraints, reports conservative
        line-search diagnostics, and moves the dynamic body away from a static
        surface.
  - [x] Phase 3o: close the first vertex-vertex line-search gap. The rigid
        curved CCD module now includes point-point ACCD, the conservative
        surface line search checks vertex-vertex rows, and solver/stage
        diagnostics count point-point checks.
  - [x] Phase 3p: harden runtime IPC geometry extraction. The opt-in stage now
        rejects malformed mesh topology, non-finite mesh vertices, and invalid
        box extents before handing runtime shapes to barrier assembly or CCD.
  - [x] Phase 3q: make runtime convergence application policy explicit. The
        opt-in stage now exposes a max-iteration knob for custom pipelines and
        tests, reports whether a result was applied, and skips non-converged
        solve results instead of writing partial poses back silently.
  - [x] Phase 3r: add kinematic (prescribed-motion) rigid bodies. A
        `KinematicBodyTag` body advances by its prescribed linear/angular
        velocity each step and acts as a moving obstacle: the barrier/dynamics
        see it at its end pose, lagged friction uses its start->end motion so it
        drags contacting dynamic bodies, and the conservative CCD line search
        sweeps its start->end motion so a moving obstacle stays anti-tunneling
        safe. `RigidBody::setKinematic()` selects it; the no-kinematic path is
        unchanged (all solver overrides are gated). Covered by prescribed-velocity
        advance, conveyor friction-drag (Fig. 13 mechanism, linear form), and
        moving-wall anti-tunneling regressions. The tag is runtime-only (not yet
        serialized); a Python binding and a turntable demo are follow-ups.
  - [x] Phase 3s: add sufficient-decrease backtracking to the internal
        projected-Newton solve. Feasible Newton steps now run through an
        Armijo-style objective check after conservative CCD scaling. The solve
        records backtracking diagnostics (also surfaced through the opt-in
        runtime stage stats) and, when lagged friction or active-set changes make
        strict Armijo too tight for the finite budget, accepts the best finite
        objective-decreasing candidate instead of treating the step as an unsafe
        line-search block.
  - [ ] Broaden remaining runtime geometry corpus coverage, convergence
        criteria, robust IPC contact behavior across corpus scenes, and
        production-ready default activation criteria.
- [ ] Phase 4: lagged smoothed friction and friction diagnostics.
  - [x] Phase 4a: add lagged smoothed Coulomb friction potentials for the first
        rigid IPC primitive-family set. The internal helpers reuse tangent
        stencils for vertex-vertex, edge-vertex, edge-edge, and face-vertex
        contacts, return value/gradient/Hessian in world coordinates, map
        derivatives to two-body reduced rigid coordinates, and cover static and
        dynamic slip branches plus primitive-family finite-difference
        regressions.
  - [x] Phase 4b: assemble the first lagged friction objective rows into the
        internal projected-Newton system. The assembly uses lagged active
        barrier constraints to estimate normal-force weights, combines per-body
        friction coefficients, scatters reduced friction derivatives into the
        global 6-DOF system, and reports runtime active-friction diagnostics.
  - [x] Phase 4c: add bounded outer lagged-friction passes to the internal
        projected-Newton solve. The solve refreshes active friction rows at the
        solved poses, supports a zero-iteration friction disable, stops early
        against a refreshed momentum-balance tolerance, and reports active
        friction pass counts through runtime diagnostics.
  - [x] Phase 4d: prove lagged friction has an observable runtime effect
        through the opt-in stage. A differential `World` regression brakes a
        tangential slide at an activated mesh contact relative to the
        frictionless solve, and reports active friction constraints/passes.
  - [ ] Extend friction into broader runtime fixture behavior, corpus coverage,
        and production convergence criteria.
- [x] Phase 5a: first same-domain rigid method selection inside the
      experimental `World` without exposing solver registries.
- [ ] Phase 5b: extend solver selection toward persisted scene policy,
      diagnostics, examples, and mixed rigid/deformable coupling.
- [ ] Phase 6: complete the manifest rows as DART-native tests, examples,
      benchmarks, comparison packets, CPU/GPU evidence, and headless Filament
      visuals.

## Goal

Implement the rigid-body implicit-barrier contact method tracked by PLAN-082 in
bounded PRs until DART covers the paper method, upstream fixtures, upstream
tests, benchmark scripts, comparison baselines, and visual evidence with a
DART-owned implementation.

## Non-Goals For The Current Manifest Slice

- No claim that DART has rigid IPC, complete curved-trajectory CCD, projected
  Newton, rigid barrier contact, or friction parity.
- No vendored or runtime dependency on `ipc-sim/rigid-ipc`.
- No public solver registry, upstream-project solver selector, ECS storage, or
  execution-backend resource exposure.

## Key Decisions

- The durable row-level source of truth is
  [`../../plans/082-rigid-implicit-barrier-contact/rigid_ipc_fixture_manifest.json`](../../plans/082-rigid-implicit-barrier-contact/rigid_ipc_fixture_manifest.json).
  This dev task links to it instead of duplicating entry state.
- The audited upstream commit is
  `23b6ba6fbf8434056444ae106356fd2209136988`.
- The initial manifest tracks 798 upstream rows:
  - 300 fixture JSON paths;
  - 405 CCD test data JSON paths;
  - 8 barrier/CCD test source families;
  - 8 benchmark scripts;
  - 77 comparison baseline files.
- Performance rows are not complete until DART records benchmark packets
  against the current DART rigid contact path, the audited reference
  implementation, and the paper scene families. Slower rows need an explicit
  accepted tradeoff instead of a parity claim. The benchmark harness,
  comparison methodology, baseline snapshot, and open performance findings are
  tracked in [`benchmarks.md`](benchmarks.md); `bm_rigid_ipc_solver` is the
  first DART-owned rigid IPC benchmark.
- Keep public DART names method/capability based. Internal code may cite the
  paper and upstream paths for tests and manifest provenance, but user-facing
  APIs should not expose upstream project names as solver identities.
- The initial importer lives under
  `dart/simulation/experimental/io/detail/rigid_ipc_fixture.*` so it can cover
  upstream fixture rows without becoming a public solver-selection API.
- The importer now has an internal replay path that populates an experimental
  `World` from loaded fixture records while preserving mesh paths, scale,
  fixture rotations, fixed DOFs, torque, density, grouping, kinematic metadata,
  and inline vertices/edges/polygons. It also reads the upstream IPC comparison
  `.txt` shape-row subset for mesh poses, scale, material density/Young/Poisson
  values, initial velocities, prescribed velocities, Neumann body forces, and
  scalar solver metadata including energy model, warm-start, self-collision,
  gravity-disable, friction, barrier, friction-iteration, and tolerance fields.
  Path-loaded fixture and script imports remember their source directory so
  upstream relative mesh paths can replay without an out-of-band asset root.
  Supported OBJ, OFF, rigid-ipc MSH, binary STL, ASCII STL, and polygonal inline
  geometry are loaded into experimental mesh collision shapes backed by the
  native collision engine. A focused runtime replay regression now enters
  simulation mode and takes one default `World::step()` on a fixture-populated
  world, verifying ordinary gravity/force integration still owns stepping until
  a solver-selection slice lands. Missing or unsupported mesh assets remain
  explicit replay metadata. It still does not select a rigid IPC solver.
- The same internal module now reads direct CCD test-data rows for `ee`, `ev`,
  and `fv` cases into DART-owned pose and primitive records, then replays those
  records through the internal subdivision-backed curved CCD implementation.
- The direct CCD tests now include first upstream-style edge-vertex, edge-edge,
  and face-vertex expected-TOI evaluator regressions, plus the first audited
  root direct-CCD rows (`tests/data/ccd-test-000..003.json`) as hermetic
  parser/topology and full-step miss regressions. They also cover the
  parameter-space residual equations for edge-vertex, edge-edge, and
  face-vertex expected-TOI rows, and the first parameter-box subdivision queries
  find those expected contacts within the reference TOI tolerance. The direct
  CCD row evaluator now routes those first rows through the subdivision queries,
  while corpus-scale evaluator parity remains open until rigorous interval
  arithmetic and reference corpus semantics land.
- The first curved-trajectory CCD code lives under
  `dart/simulation/experimental/detail/rigid_ipc_ccd.*`. It is an internal
  DART-owned ACCD query for 3D face-vertex, edge-edge, and point-edge cases over
  linearly interpolated rotation vectors, including the first `minSeparation`
  regressions needed by IPC barriers. It now also exposes internal residual
  evaluators, a face-vertex domain helper, and first parameter-box subdivision
  queries needed by the interval-root formulation, but it does not yet implement
  rigorous interval arithmetic, accepted corpus tolerances, or scene-level broad
  phase.
- The first rigid barrier scaffold lives under
  `dart/simulation/experimental/detail/rigid_ipc_barrier.*`. It evaluates
  face-vertex and edge-edge barrier terms by transforming local rigid
  primitives along the linearly interpolated rigid trajectory and then reusing
  the current DART IPC C2 clamped-log world-primitive kernel. Focused tests
  prove parity with the world-space kernel for interpolated and rotated rigid
  poses. The same module now adds a two-body reduced-coordinate chain-rule
  layer that maps those world-primitive derivatives into 6-DOF rigid-pose
  coordinates and can project the local reduced Hessian to PSD. Focused tests
  compare reduced gradients/Hessians against finite differences and assert the
  projected Hessian contract. It now covers the first local face-vertex,
  edge-edge, edge-vertex, and vertex-vertex barrier set by adding shared
  point-edge and point-point kernels on top of the existing distance
  derivatives and rigid pose-sampled/reduced wrappers for those terms. This is
  still internal solver scaffolding: it now owns the first scene-level active
  barrier records and scatters reduced derivatives into global dynamic-body
  gradients and sparse PSD-projected Hessians. It also owns the first
  conservative surface line-search feasibility helper over matching start/end
  surfaces for vertex-vertex, edge-vertex, edge-edge, and face-vertex rows. The
  helper treats initial separation violations and indeterminate CCD exhaustion
  as unsafe zero-step results. The same module now has a first internal
  projected-Newton step helper over the assembled gradient and sparse
  PSD-projected Hessian, including diagonal
  regularization, descent-step statistics, and line-search bound scaling. It
  now also owns a first barrier-only projected-Newton loop over copied rigid
  surfaces that recomputes assembly each iteration, updates dynamic surface
  poses, and records convergence/progress statistics. The same loop now accepts
  first generalized dynamics objective terms: per-body diagonal quadratic
  weights plus generalized force/torque vectors assembled into the global
  6-DOF pose system. A helper now constructs those terms from physical mass,
  diagonal inertia, generalized velocity, force/torque, and timestep. The
  opt-in runtime stage now extracts mesh/box/sphere free rigid-body state,
  exposes solve status plus line-search diagnostics through its last-stage
  stats, and `World` can select the IPC free-rigid dynamics path explicitly. A
  focused runtime regression now verifies activated mesh barriers move a
  dynamic body away from a static surface, and the extractor now skips invalid
  runtime mesh/box geometry before barrier assembly. The runtime stage also now
  skips non-converged solve results instead of applying partial poses silently.
  The first lagged smoothed Coulomb friction potentials now cover
  vertex-vertex, edge-vertex, edge-edge, and face-vertex contacts in world
  coordinates, with reduced rigid-coordinate coverage for vertex-vertex and
  edge-vertex terms. The projected-Newton objective can now assemble lagged
  friction rows from active lagged barrier constraints and reports
  active-friction diagnostics from the opt-in runtime stage.
- Focused rotational tests now prove why endpoint-linear primitive CCD is not
  enough: the primitive endpoints return to their initial positions while the
  curved rigid trajectory contacts mid-step.

## Immediate Next Steps

1. Extend Phase 3 from selectable rigid IPC stepping over mesh/box/sphere
   runtime surfaces, durable stage diagnostics, and the first activated-contact
   runtime regression plus vertex-vertex line-search CCD and invalid runtime
   geometry rejection plus explicit non-converged-result skipping into broader
   convergence criteria, corpus contact behavior, and production-ready default
   activation criteria.
2. Extend Phase 4 from bounded outer lagged-friction passes into runtime
   fixture behavior, corpus coverage, and production convergence criteria.
3. Extend Phase 2 with upstream corpus parity: more direct
   `tests/data/ccd-test-*` evaluator checks, kinematic rows, codimensional
   coverage, rigorous interval arithmetic, and accepted tolerances against the
   audited reference.
4. Extend Phase 1 from mesh and inline replay into fuller fixture coverage:
   fixture-row runtime examples and remaining comparison script commands. The
   first default `World::step()` runtime replay regression is covered, but a
   public-facing example remains open until the importer surface is no longer
   internal-only.
5. Keep selecting P0 rows from `fixtures/3D/unit-tests/tunneling.json`, direct
   `tests/data/ccd-test-*` files, and one simple paper figure fixture.
6. Keep the default `World::step()` behavior unchanged until a tested
   DART-owned method policy can select the implicit-barrier path without
   exposing registry or backend internals.

## Verification

Run the manifest checks before changing or relying on coverage:

```bash
pixi run python scripts/generate_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc
pixi run python scripts/check_rigid_ipc_fixture_manifest.py --upstream-dir /tmp/rigid-ipc
pixi run python scripts/check_rigid_ipc_fixture_manifest.py
pixi run build-simulation-experimental-tests
pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_rigid_ipc_barrier$'
pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_rigid_ipc_fixture$'
pixi run test-simulation-experimental
```

The upstream checkout must be at
`23b6ba6fbf8434056444ae106356fd2209136988`.
