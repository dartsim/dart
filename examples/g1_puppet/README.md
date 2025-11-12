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

The parser infers the package name from `--robot-uri` when it sees a
`package://` scheme, but you can override it manually with
`--package-name <name>` if needed. `./g1_puppet --help` lists every option.

> **CLI dependency:** The example uses [CLI11](https://github.com/CLIUtils/CLI11)
> for argument parsing. Building DART through `pixi` installs the library
> automatically; standalone builds should make the CLI11 CMake package
> discoverable (e.g., `conda install -c conda-forge cli11`).

Use the mouse to drag individual body nodes. The example runs in kinematic mode
so you can quickly inspect the downloaded model without setting up controllers.
