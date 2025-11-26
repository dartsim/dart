# Joint Constraints

This example demonstrates joint constraint handling with SPD (Stable Proportional-Derivative) tracking control and dynamic perturbation testing. The example features a humanoid character that maintains balance while being subjected to external forces.

## Controls

- `1`: Push forward (apply force in +X direction)
- `2`: Push backward (apply force in -X direction)
- `3`: Push right (apply force in +Z direction)
- `4`: Push left (apply force in -Z direction)
- `h`: Toggle harness on/off (adds/removes weld joint constraint to pelvis)
- `Space`: Simulation on/off

## Description

The joint constraints example showcases:

- SPD (Stable Proportional-Derivative) tracking control for balance
- External force perturbations to test robustness
- Dynamic constraint addition/removal (harness feature)
- Ankle strategy for center of pressure control

The controller computes torques to maintain desired joint configurations while the character responds to external forces. The harness feature allows you to weld the pelvis to the world frame, which can help with testing and debugging.
