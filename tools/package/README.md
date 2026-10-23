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
MJCF and URDF sample data (SDF only when `HAVE_SDFORMAT` is enabled). Request
`io-sdf` to require SDF support, and check that it is not found on an SDF-off
install. Shared `io` and `io-sdf` consumers should configure with
`CMAKE_DISABLE_FIND_PACKAGE_sdformat=ON` and the versioned
`CMAKE_DISABLE_FIND_PACKAGE_sdformat14`, `CMAKE_DISABLE_FIND_PACKAGE_sdformat15`
and `CMAKE_DISABLE_FIND_PACKAGE_sdformat16` also set to `ON`. Static
SDF-enabled IO consumers must find the selected sdformat major. Also request
`utils` and `utils-urdf` to check their legacy forwarding targets. The data
probe uses staged filesystem paths so source-tree resource fallback cannot
hide missing installed assets.

With `DART_BUILD_UTILS_ASSETS=ON` in the installed build, request
`io;utils-assets` to also construct an offline model retriever and register
the installed Atlas manifest. Request only `utils-assets` to exercise the same
IO probe through its legacy forwarding target. Neither path adds explicit
CURL or OpenSSL links; shared and static consumers check their dependency
propagation. The asset probe makes no downloads or cache writes.

Request `gui` to exercise installed Filament GUI and ImGui APIs through
`dart-gui`. The same probe accepts the legacy `external-imgui` component. Test
system and bundled ImGui installations, including a GUI build with IO disabled.
Rendering is checked separately by `offscreen_capture_smoke` and the
`scripts/test_gui_install.py` consumer.
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
The legacy `DART_BUILD_GUI_OSG=OFF` also disables the whole GUI subtree, and
`DART_BUILD_GUI_OSG=ON` fails because DART 7 removed the OSG GUI. When
re-enabling GUI in an existing build directory, set `DART_BUILD_GUI=ON`.
Requesting the removed `gui-osg` component reports the migration to `gui`.
