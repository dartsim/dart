# Raylib GUI Backend — Testing and CI

## Goals

- Keep non-GUI builds unaffected (Raylib/OSG dependencies should remain optional).
- Provide fast feedback that the Raylib backend stays buildable across supported platforms.

## Recommended testing layers

1. **Build-only coverage**
   - Ensure the Raylib backend compiles and links in CI configurations where GUI is enabled (including the chosen UI integration).

2. **Smoke runtime (best-effort)**
   - Prefer a short “run N frames then exit” mode for demos/tests.
   - When practical, exercise the UI path (to catch ImGui integration regressions) without relying on manual interaction.
   - Gate runtime execution on CI environments that have a usable display/context.

3. **Functional tests (targeted)**
   - Where logic is backend-neutral (scene extraction, picking math, manipulation logic), add unit tests in existing test infrastructure.

## Regression signals to prioritize

- API drift between backends (missing features on Raylib for maintained use cases).
- Build option regressions (Raylib enabled should not silently pull in OSG).
- Performance cliffs for common scenes (mesh-heavy worlds).
