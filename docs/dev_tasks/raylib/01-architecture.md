# Raylib GUI Backend — Architecture & API Principles

## Problem statement

The current GUI component (`dart-gui`) is an OpenSceneGraph backend, and its public headers are OSG-shaped (inheritance and types). This makes the API both backend-coupled and extremely “open-ended”: users can do many things, including things that are hard to support or keep stable.

For the Raylib replacement, we intentionally take the opposite approach: a new API that is **not** backward compatible with `dart::gui`, and whose public surface is **small, opinionated, and hard to misuse**.

## API design goals (Raylib replacement)

- **No backend leakage**: public headers must not expose OSG/Raylib/OpenGL types.
- **Constrained extension**: customization is possible, but only through DART-owned abstractions that preserve invariants.
- **Single “pit of success”**: default usage should cover the common DART workflows with minimal boilerplate.
- **Stable semantics**: avoid exposing internal render loop timing, resource lifetimes, or scene graph handles.

## Recommended direction: new constrained API + Raylib backend

### 1) Define a minimal public “viewer” surface (new API)

Expose only what is required for DART’s supported workflows:

- window lifecycle + run loop (start/stop, headless/frames mode for smoke tests)
- camera configuration + a small set of supported camera controllers
- attaching a `simulation::World` (or equivalent) + stepping controls (pause/step/realtime-ish)
- input events (keyboard/mouse) in DART-owned types
- picking/selection as a high-level concept (select entities/frames), not raw ray casting API
- overlay/widget hooks (single supported UI approach; no raw UI/backend injection)

### 2) Strict extension points (avoid “raw graphics access”)

Instead of exposing a backend handle, provide explicit extension mechanisms with narrow capabilities, e.g.:

- “layers”/“tools” that can:
  - react to input events
  - draw from a small set of DART-defined debug primitives (lines, points, text, simple shapes)
  - contribute UI via a supported widget mechanism

Avoid APIs that let downstreams:

- own the render loop
- access raw backend state/resources
- keep pointers/handles to internal scene objects

## UI strategy (recommended): Dear ImGui, kept internal

Use Dear ImGui for UI panels/overlays in the Raylib viewer, but do **not** expose ImGui types, context, or headers in the public API.

**Rationale**

- DART already has ImGui-based tooling; reusing that style reduces duplicate UI work.
- ImGui supports the kinds of “simulator UI” surfaces DART needs (inspectors, property panels, debug tools) better than a minimal widget set.
- Keeping ImGui internal preserves the “hard to misuse” goal: users interact with DART-owned concepts, not the UI backend.

**Public API implication**

- Provide only DART-owned extension points (e.g., “panels/tools”) that receive a narrow capability set (simulation controls, selection state, debug draw, etc.).
- Avoid exposing a general-purpose “draw arbitrary UI” hook that effectively becomes “ImGui passthrough”.

**What not to standardize on**

- Avoid making Raylib’s companion widget library (raygui) a core dependency for DART’s primary GUI path; it can be useful for small demos, but it tends to limit the UI as features grow.

### 3) Separate “scene extraction” from rendering (internal)

Factor out logic that is independent of the graphics API:

- traversal of `simulation::World` and discovery of renderables
- stable identifiers for renderables (for selection/picking)
- material/color extraction from DART aspects
- geometry cache keys and invalidation rules (static vs dynamic)

Then keep the Raylib backend focused on:

- translating extracted renderables into draw calls/resources
- window/context creation and input polling
- backend-specific render state (lighting/shaders)

## Legacy OSG posture

- The existing OSG GUI remains as a legacy component during transition.
- The new API does not attempt to wrap or emulate the old `dart::gui` surface.
- Once Raylib meets the replacement bar, OSG becomes opt-in and then removed in a major release.

## Raylib-specific considerations

- Prefer keeping the current “Z-up” convention (match existing GUI docs/examples by setting the Raylib camera up-vector accordingly).
- Start with simple, correct rendering (unlit or basic lit shading) before advanced visuals.
- Plan for meshes and dynamic geometry early (many DART scenes include mesh-based assets).
