# Contact Patch Cache GUI Demo Plan (05)

## Target Example

- Update `examples/box_stacking` to add a persistent contact demo
- Create two identical worlds side-by-side:
  - Left: feature OFF (legacy)
  - Right: feature ON (ContactPatchCache)

## Scenario

- Stack 3 to 5 boxes on a plane
- Optionally add a tall slender box to highlight jitter
- Use fixed time step and deterministic initial conditions

## Runtime Toggle

- Add a hotkey or ImGui checkbox to toggle the feature per world
- Add a reset key to restore the initial state

## On-screen Diagnostics

- Contact count used for constraints
- Per-pair contact count (max 4)
- Contact churn count per frame

## Repro Steps

1. Run the example
2. Press reset to align both worlds
3. Observe jitter and contact churn on the OFF side
4. Compare to the ON side

## Notes

- Keep the demo self-contained and deterministic
- Document the controls in the example README or in the example source
