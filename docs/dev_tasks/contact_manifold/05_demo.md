# Contact Manifold Cache GUI Demo Plan (05)

## Target Example

- Updated `examples/box_stacking` to add a persistent contact demo
- Two identical worlds are rendered side-by-side:
  - Left: feature OFF (legacy)
  - Right: feature ON (ContactManifoldCache)

## Scenario

- Stack 3 to 5 boxes on a plane
- Optionally add a tall slender box to highlight jitter
- Use fixed time step and deterministic initial conditions

## Runtime Toggle

- ImGui checkboxes toggle the feature per world
- Reset button restores the initial stack for both worlds

## On-screen Diagnostics

- Active manifold count per world
- Persistent contact count per world
- Contact constraint count per world

## Repro Steps

1. Run the example
2. Use the \"Reset stacks\" button to align both worlds
3. Observe jitter and contact stability on the OFF side
4. Compare to the ON side

## Notes

- Keep the demo self-contained and deterministic
- Document the controls in the example README or in the example source
