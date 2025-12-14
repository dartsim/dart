# Raylib GUI Backend — Migration and Deprecation

## Guiding principle

Migrate DART-maintained code (examples/tutorials/`dartpy`) first, so users have a supported path before OSG is deprecated.

## Proposed migration sequence

1. **Define the new API (clean break)**
   - Keep the public surface intentionally small and opinionated.
   - Avoid exposing backend types/handles.

2. **Vertical slice conversion**
   - Pick one maintained tutorial/example and port it to the new Raylib viewer path.
   - Use this to validate API shape, camera controls, stepping model, and packaging.

3. **Example/tutorial migration**
   - Migrate remaining maintained examples/tutorials to avoid hard dependencies on OSG.
   - Keep any OSG-only examples explicitly labeled as legacy during the transition.

4. **`dartpy` migration**
   - Bind the backend-neutral surface area rather than binding backend-specific types.
   - Provide a deprecation window for the legacy GUI bindings (no backward-compat requirement for the new API).

## Deprecation plan for OSG

- Make OSG dependency “opt-in” once Raylib meets the replacement bar.
- Provide a time-bounded period where both backends are supported.
- Announce removal in release notes early, with a concise “how to migrate” section and links to the new API.

## Compatibility considerations

- Some downstreams embed custom OSG nodes/event handlers today; decide whether to:
  - offer a backend-specific extension point (OSG-only) during the transition, or
  - replace it with a backend-neutral plugin/hook mechanism, or
  - treat it as an unsupported workflow after removal (must be communicated early).
