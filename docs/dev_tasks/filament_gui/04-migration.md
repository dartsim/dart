# Filament GUI Replacement - Migration and Deprecation

## Guiding principle

Migrate DART-maintained workflows first. Downstream OSG-specific renderer
extension points should not define the replacement API unless maintainers
explicitly accept them as supported long-term DART concepts.

The more detailed north-star plan is
`docs/dev_tasks/filament_gui/08-north-star-migration.md`. That plan is the
source of truth for the intended complete migration to Filament, the
no-renderer-backward-compatibility stance, and the multi-phase promotion path.

## Proposed sequence

1. **MVP example**
   - Build `examples/filament_gui`.
   - Keep implementation local to the example until the dependency and lifetime
     model is proven.

2. **Backend-hidden extraction**
   - Move world traversal, shape descriptors, materials, debug primitives, and
     selection IDs into reusable code.
   - Keep Filament-specific objects private.

3. **Experimental API**
   - Add a constrained `dart::gui::experimental` API.
   - Port one simple example and one interaction-heavy example.
   - Current simple consumer: `examples/gui_scene_diagnostics`.
   - Current interaction-heavy fixture:
     `examples/filament_gui --scene drag-and-drop`.
   - Add Python bindings only for constrained descriptor/debug APIs that are
     stable enough to test without committing to a full viewer surface.

4. **Promotion**
   - Promote the experimental API to primary `dart::gui` in a major DART
     version.
   - Keep old OSG APIs deprecated or isolated during the transition if needed.

5. **Removal**
   - Remove the OSG implementation and public OSG-shaped APIs after the
     deprecation window.
   - Remove the Raylib smoke example and related build options in the same major
     cleanup window unless maintainers choose to keep it as an external example.

## Compatibility notes

- Current code using `dart::gui::Viewer` as an `osgViewer::Viewer` cannot be
  source-compatible with the new API.
- Current code that directly manipulates OSG nodes, event handlers, or scene
  graph state needs a migration story or must become explicitly unsupported.
- The new API should favor DART concepts: worlds, cameras, selections, debug
  draws, tools, and panels.

## Release notes requirements

When promotion starts, release notes should clearly state:

- the first DART version containing the experimental Filament path
- the first version where it becomes the default `dart::gui`
- the planned version where OSG and Raylib are removed
- dependency requirements and package-manager expectations
- the migration path for examples, tutorials, and `dartpy`
