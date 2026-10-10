# Installed package checks

`pixi run test-package-core` configures and installs the headless core in an
isolated build prefix, then builds/runs two external consumers. One steps a
world and includes SIMD headers; the other uses only the collision factory,
which also checks that static archive linking retains backend registration.

The standalone CMake project accepts `DART_TEST_COMPONENTS` to exercise
installed components, including the legacy `collision-bullet` and
`collision-ode` forwarding targets. Run it against fresh shared, static, and
backend-disabled installations. Use `DART_DIR` to select the installation
explicitly.

Request `io` to link the single `dart-io` target and load installed SKEL, SDF,
MJCF and URDF sample data. Also request `utils` and `utils-urdf` to check their
legacy forwarding targets. The data probe uses staged filesystem paths so
source-tree resource fallback cannot hide missing installed assets.

Request `gui` to exercise installed OSG and ImGui APIs through `dart-gui`.
The same probe accepts legacy `gui-osg` and `external-imgui` components. Test
system and bundled ImGui installations, including a GUI build with IO disabled.
Rendering is checked separately by `offscreen_capture_smoke`.
When staging a build configured for a Pixi prefix, select the staged library
directory and the Pixi dependency directory in the runtime library path, or
configure an install RPATH for that test installation.

`embed/` configures DART as a subdirectory after the parent has found Bullet and
ODE. It checks that DART respects parent-owned imported dependency targets.
Set `DART_TEST_PARENT_ALIASES=ON` to exercise parent-owned aliases as well;
install that build and run the standalone consumer against its installation.

For a pkg-config consumer, put `-std=c++17` after the pkg-config compiler flags;
some FCL distributions include an older standard flag in their metadata.
For CMake consumers, the exported `dart` target supplies `cxx_std_17`.

`DART_BUILD_IO=OFF` and `DART_BUILD_GUI=OFF` select the core-only build.
The legacy `DART_BUILD_GUI_OSG=OFF` also disables the whole GUI subtree. When
re-enabling GUI in an existing build directory, set both GUI switches to `ON`.
