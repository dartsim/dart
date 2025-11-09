# DART Logo Chain Example

This example animates a planar 6-DOF chain so that it morphs from a straight configuration into the rainbow hexagon that appears in the DART logo. Each link is rendered in the corresponding logo color and the motion is driven by a lightweight PD controller. Static rigid bodies approximate the arrow and the `DART` lettering so the final pose matches the brand mark.

## Running

Build the project using the repository instructions (for example `pixi run build`), then launch the executable produced in your build tree. With the default CMake layout this is located at:

```bash
./examples/dart_logo_chain/dart_logo_chain
```

Follow the on-screen prompt to start the animation.
