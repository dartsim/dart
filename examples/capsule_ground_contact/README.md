# Capsule Ground Contact Example

This example exercises capsule-plane contact with an ODE collision detector
when ODE is available, and visualizes the scene through the promoted
`dart::gui` runner.

Run it with:

```bash
pixi run ex capsule_ground_contact
```

Headless capture:

```bash
pixi run ex capsule_ground_contact --headless --frames 2 --screenshot /tmp/capsule_ground_contact.ppm
```

Controls:

- Space: play/pause through the shared viewer control and clear capsule
  velocities
- `h`: reset the capsule to the horizontal pose
- `v`: reset the capsule to the vertical pose
- `n`: step one frame while paused
- Escape: exit

The default launch size is 1024x768, matching the historical viewer example.
