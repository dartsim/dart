# Vehicle

This example demonstrates a simple vehicle simulation with steering control and velocity control for wheels. The vehicle has four wheels with differential drive control.

## Controls

- `w`: Move forward (set back wheels to rotate forward)
- `s`: Stop (set back wheel velocity to zero)
- `x`: Move backward (set back wheels to rotate backward)
- `a`: Rotate steering wheels to left (maximum 30 degrees)
- `d`: Rotate steering wheels to right (maximum -30 degrees)
- `Space`: Simulation on/off

## Description

The vehicle example showcases:
- Basic vehicle kinematics and dynamics
- Steering control with angle limits
- Differential drive control for rear wheels
- PD control for wheel steering and velocity tracking

The vehicle uses proportional-derivative (PD) control to track desired steering angles and wheel velocities. The steering is limited to ±30 degrees for realistic behavior.
