# Hybrid Dynamics

This example demonstrates hybrid dynamics simulation where some joints are controlled by velocity commands while others are passive or locked. The example features a humanoid character performing programmed arm and leg movements.

## Controls

- `h`: Toggle harness on/off (locks/unlocks the pelvis joint)
- `Space`: Simulation on/off

## Description

The hybrid dynamics example showcases:

- Mixed joint actuation types (passive, velocity-controlled, locked)
- Programmed joint commands using sinusoidal patterns
- Dynamic switching of joint actuator types during simulation

The humanoid character's arms and legs move with sinusoidal patterns while maintaining balance. The harness feature allows you to lock the pelvis joint to help stabilize the character.
