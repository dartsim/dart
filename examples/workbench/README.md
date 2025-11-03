# DART Workbench Example

Workbench is a Visual Studio–style launcher that consolidates multiple DART
scenes into a single ImGui + OpenSceneGraph application. The left panel lists
the in-app scenes as well as the standalone examples that live under
`examples/`. Selecting an in-app scene loads it into the central viewer, while
standalone entries present their location on disk so they can be explored
separately.

The bottom panel aggregates logs and real-time factor plots so performance
regressions are easy to spot. A properties panel on the right exposes toggles
for the active scene, and additional widgets can be registered by adding new
`ExampleRecord` entries.

## Layout

```
examples/workbench/
├── app/                # Viewer bootstrap + ImGui widget rendering
├── core/               # Shared types and custom WorldNode helpers
├── registry/           # Built-in scene registry and filesystem discovery
└── scenes/             # Individual scene factories (e.g. DoublePendulum)
```

To extend the workbench, add another factory under `scenes/`, register it in
`registry/ExampleRegistry.cpp`, and use the callbacks exposed by
`ExampleSession` to provide custom UI or simulation hooks.
