## G1 Puppet

This example demonstrates how to load the Unitree G1 humanoid directly
from its upstream GitHub repository at runtime. It leverages the new
`HttpResourceRetriever` and `PackageResourceRetriever` so that `package://`
URIs inside the URDF can be resolved against an HTTP(S) package root.

### Build

From this directory:

```bash
mkdir build
cd build
cmake ..
make
```

### Run

From the build directory:

```bash
./g1_puppet
```

By default the viewer opens an OSG window after downloading
`https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/g1_description`
and loading `package://g1_description/g1_29dof.urdf`. You can override those
values:

```bash
./g1_puppet \
  --package-uri https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/g1_with_brainco_hand \
  --robot-uri package://g1_with_brainco_hand/g1_29dof_with_hand.urdf
```

Use the mouse to drag individual body nodes. The example runs in kinematic mode
so you can quickly inspect the downloaded model without setting up controllers.
Note that a graphical desktop session is required—when run from a headless
shell (no `DISPLAY` or `WAYLAND_DISPLAY`) the executable will print a warning
and exit because a GUI window cannot be created.
