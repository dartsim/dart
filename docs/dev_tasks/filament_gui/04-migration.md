# Filament GUI Replacement - Migration and Deprecation

## Guiding principle

Migrate DART-maintained workflows first. Downstream OSG-specific renderer
extension points should not define the replacement API unless maintainers
explicitly accept them as supported long-term DART concepts.

Current steering tightens this principle: Filament with GLFW3 and Dear ImGui is
the only official built-in renderer, not an experimental alternate backend.
Public names should therefore be `dart::gui` and scope-based example names, not
`dart::gui::experimental` or backend-named examples such as `filament_gui`.

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

4. **Promotion and example restoration**
   - Promote the experimental API to primary `dart::gui` in a major DART
     version.
   - Rename the MVP executable by scope, currently planned as
     `examples/gui_viewer` / `dart_gui_viewer` / `gui_viewer`.
   - Restore all pre-existing user-facing examples as `dart::gui` examples
     before deleting their old OSG implementations. Each restored example
     should keep its historical example entry point and teach the new DART GUI
     surface, while renderer setup remains private.
   - Keep backend-only example names out of the official surface. Do not
     restore Raylib as a renderer.

5. **Removal**
   - Remove the OSG implementation and public OSG-shaped APIs after examples,
     docs, tests, and Python bindings no longer depend on them.
   - Remove Raylib build options, dependency discovery, and backend-specific
     runner aliases in the same major cleanup window.

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
