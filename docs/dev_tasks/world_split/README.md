# World Split — Dev Task

## Current Status

- [x] Phase 0: Restore classic `dart::simulation::World` behavior as the
      stable Skeleton pipeline.
- [x] Phase 1: Move the experimental stack under
      `dart::simulation::experimental`.
- [ ] Phase 2: Decide and implement the Python binding split.
- [ ] Phase 3: Document the DART 7 deprecation path for the classic world.
- [ ] Phase 4: Promote the experimental world shape for the DART 8 transition.

## Goal

Keep the classic world stable while developing an experimental ECS-backed world
behind explicit namespaces, build options, tests, and bindings. The long-term
path is to let the experimental world mature into the promoted DART 8 world API
without exposing EnTT or multi-solver scaffolding through the classic world.

## Non-Goals

- No mixing classic and experimental objects in one world.
- No public ECS entity creation through the classic world.
- No compatibility aliases for removed legacy macros or CMake targets.

## Key Decisions

- Classic `dart::simulation::World` stays as the Skeleton/Frame world for the
  DART 7 transition.
- Experimental world code lives under `dart/simulation/experimental/` with
  namespace `dart::simulation::experimental`.
- Experimental build and CMake surfaces use `DART_EXPERIMENTAL_*` and
  `dart_experimental_*` names.
- Python should expose a separate module for the experimental world; the module
  name is still open.

## Immediate Next Steps

1. Decide between `dartpy.simulation_experimental` and an alternative Python
   module name.
2. Update `01_migration.md` with the selected binding shape.
3. Implement the binding split or derive a smaller task if the binding work is
   too large for one session.

## Owner Notes

- Design note: [00_design.md](00_design.md)
- Migration plan: [01_migration.md](01_migration.md)
- Operating state: [docs/plans/dashboard.md](../../plans/dashboard.md)
