# DART Changelog

## DART 7

### [DART 7.0.0 (Unreleased)](https://github.com/dartsim/dart/milestone/105)

* Breaking Changes

  * **Behavior change:** MJCF loading now enforces hinge and slide limits,
    including inherited defaults and MuJoCo 3.x automatic limits; applies
    declared mass, center of mass, and rotated inertia even without geoms;
    places bodies without `pos` at the parent origin in local coordinates;
    orthogonalizes `xyaxes`; and converts hinge `springref` from degrees.
    Hinge and ball ranges are now read in degrees under
    `<compiler angle="degree">` (MuJoCo's default), rather than as raw numbers.
    Coordinates without limits now report infinite position limits instead of
    the declared range (often zero).
    Invalid declarations now fail to load: unknown `limited` or `autolimits`
    values, nonzero ranges with automatic limits under `autolimits="false"`,
    reversed or non-finite limited ranges, a nonzero lower limit on a limited
    ball joint, or `fullinertia` combined with an orientation attribute.
    Models that relied on ignored limits or inertia will simulate differently;
    use `limited="false"` to disable limits explicitly. Ball-joint angular limits
    remain unsupported and warn once per model; support is deferred to a later
    release.
    [#3877](https://github.com/dartsim/dart/pull/3877)

  * `dart/utils/urdf/urdf.hpp`, through `urdf_world_parser.hpp`, no longer
    includes `BackwardCompatibility.hpp`. Code using its
    `URDFDOM_HEADERS_VERSION_*` macros must include
    `dart/utils/urdf/BackwardCompatibility.hpp` directly.
    [#3863](https://github.com/dartsim/dart/pull/3863)

  * DART 7 replaces the OSG renderer with the Filament-backed `dart::gui`
    viewer, including world-node callbacks, widgets, manipulation, DPI
    scaling, and display-free Linux capture. The `gui-osg` component
    (`dart-gui-osg` target), its installed `dart/gui/osg` headers, the
    `dartpy.gui.osg` namespace, and the OpenSceneGraph dependency are removed;
    migrate C++ code to the `gui` component (`dart-gui` target,
    `dart/gui/gui.hpp`) using the
    [visualization migration guide](https://dart.readthedocs.io/en/latest/dart/user_guide/visualization.html).
    Requesting `gui-osg` now reports this migration, and
    `DART_BUILD_GUI_OSG=ON` stops configuration; `DART_BUILD_GUI_OSG=OFF` still
    selects a headless build. These `gui` names previously belonged to the
    GLUT-based GUI shipped through DART 6.19. `DART_BUILD_GUI` (default `ON`)
    builds the Filament GUI and ImGui, which require Filament 1.77.3, GLFW 3.4,
    libpng, libjpeg, and Dear ImGui 1.92.8 or newer. Rendering without a usable
    graphics backend raises `dart::gui::RenderingUnavailableError`
    (`dartpy.gui.RenderingUnavailableError`) naming the missing system
    packages. The `dartpy.gui` bindings are interim; DART 7's Python rendering
    API replaces them before release.
    `DART_ENABLE_GUI_SMOKE_TESTS` (default `OFF`) replaces
    `DART_ENABLE_GUI_OSG_SMOKE_TESTS`; enable it to build the capture smoke test
    on a host with a graphics device.
    [#3696](https://github.com/dartsim/dart/pull/3696)

  * dartpy now uses nanobind 3.1 or newer and requires Python 3.12 or newer.
    Compared with DART 6.20 (pybind11), secondary C++ bases are absent from
    Python `isinstance` checks and MRO; exception types and messages can differ;
    bool parameters accept only `True`/`False`; str parameters reject bytes;
    and const Eigen views are read-only. Convert inputs explicitly and copy
    const views when writable arrays are needed.

  * pybind11 extension modules can no longer pass DART objects to dartpy or
    receive them from it, because nanobind does not share pybind11's type
    registry. Extensions that rely on this can stay on DART 6.20, which keeps
    the pybind11 binder.

  * Remove the `DART_DARTPY_BINDER` and `DART_USE_SYSTEM_PYBIND11` CMake
    options and the pybind11 binder. Remove these options from configurations;
    a legacy `DART_DARTPY_BINDER=pybind11` value fails configure.

  * Remove the `hubo_puppet` demo and installed
    `dart://sample/urdf/drchubo/` sample resources. The `fetch` demo is also
    retired; its model remains available for parser regression coverage.

  * Remove the unlicensed `data/skel/kima/` sample SKEL files and COLLADA
    meshes from the installed sample data.
    [#3880](https://github.com/dartsim/dart/pull/3880)

  * Explicitly delete copy and move construction and assignment for
    `DARTCollisionGroup`, `OdeCollisionGroup`, `EmbedProperties`, and
    `EmbedStateAndProperties`. These operations could not be instantiated
    previously; type traits such as `std::is_copy_constructible` now correctly
    report `false`. Class layouts remain unchanged.
  * Compile optional Bullet and ODE collision backends into the `dart` library.
    Existing CMake targets and components remain available for rebuilt
    applications, but consumers of the separate collision shared libraries
    must rebuild.
  * Combine `dart-utils` and `dart-utils-urdf` into `dart-io`, retaining their
    legacy CMake names and public `dart::utils` headers for rebuilt consumers.
    Enabling IO now requires tinyxml2, urdfdom, and sdformat 14.8 or newer.
  * SDF loading uses canonical sdformat validation, version conversion and
    frame semantics, including authored world gravity and joint effort and
    velocity limits that the old parser ignored. Parse, resource and
    unsupported-model failures now report
    diagnostics and return `nullptr` (`None` in Python), including invalid poses
    that previously threw exceptions. Check the returned value when loading
    SDF. General nested models and non-tree joint structures remain unsupported.
    [#3789](https://github.com/dartsim/dart/pull/3789)
  * Compile ImGui into `dart-gui`, preserving the `external-imgui` component
    and `dart-external-imgui` target as CMake forwarding names. They no longer
    provide the OpenGL2 backend (`imgui_impl_opengl2.h`) or depend on OpenGL,
    and applications linked to the former ImGui shared library must rebuild.

* API

  * URDF world loading returns `nullptr` when an include is missing its
    filename or model name, or an entity is missing its model, instead of
    constructing strings from null attributes.
    [#3863](https://github.com/dartsim/dart/pull/3863)

  * The GUI rejects textures of more than 67,108,864 pixels (8192 x 8192), or
    256 MiB of decoded RGBA data, before allocating them.
    [#3860](https://github.com/dartsim/dart/pull/3860)

  * Add opt-in C++ `World::compile()` and scoped `World::edit()` to freeze
    and recompile model topology, with per-World topology and parameter
    revisions. Worlds that are never compiled retain lazy runtime editing
    and shared model membership; `edit()` on such a World leaves it lazy.
    Scope exit never throws: when the recompile fails, the next `step()` or
    `compile()` throws that error.
    [#3869](https://github.com/dartsim/dart/pull/3869)

  * Worlds that are never compiled keep DART 6 behavior, including version
    propagation and shared model membership, with one labeled change: a
    re-entrant `World::step()` and a topology edit of the stepping World's
    model from its own step callbacks now throw `std::runtime_error` instead
    of corrupting the step. Ownership tracking starts at `compile()`.
    [#3869](https://github.com/dartsim/dart/pull/3869)

  * Add `optimizer::Problem::clearObjective()` and
    `utils::CompositeResourceRetriever::getDefaultRetrievers()`,
    `getSchemaRetrievers()`, and `removeAllRetrievers()` to inspect and release
    owned objective functions and resource retrievers.

  * Prevent concurrent legacy URDF parsing calls from overriding each other's
    resource retriever and root-joint options.
    [#3653](https://github.com/dartsim/dart/pull/3653)

  * SKEL and MJCF loading throws `std::invalid_argument` when a 2-, 3-,
    4- or 6-element vector or pose value has the wrong number of elements,
    instead of writing out of bounds or leaving values unset in release
    builds. An MJCF weld `solref` with one value keeps the default for the
    other.

  * SDF loading resolves named pose frames, joint endpoint frames, expressed
    axes, rotated inertia and relative includes through sdformat. Meshes retain
    their originating document for resource resolution, including Windows native
    paths. DART soft-shape poses,
    empty identity poses and safely normalized quaternion poses remain supported.
    [#3789](https://github.com/dartsim/dart/pull/3789)

  * Loading a SKEL world no longer crashes when the collision detector
    factory cannot create `fcl`, for example after an application
    unregisters the factory's creators. The parser creates the FCL detector
    directly instead and warns once.
    [#3750](https://github.com/dartsim/dart/pull/3750)

* Collision

  * A throwing Shape override or allocation failure during collision-group
    subscription no longer leaves partial collision objects or a stale
    FCL/Bullet detector shape entry; debug builds no longer assert when the
    detector is destroyed.
    [#3869](https://github.com/dartsim/dart/pull/3869)

  * Editing a `CompositeCollisionFilter` during its own query, from one of its
    filters, now throws `std::runtime_error` instead of invalidating the query
    when the edit removes the filter being queried, clears the composite or
    adds a filter that rehashes its set; other edits still apply.
    [#3869](https://github.com/dartsim/dart/pull/3869)

  * Fix a memory leak where a Bullet collision detector used for a collision
    check between two groups was never destroyed.
    [#3875](https://github.com/dartsim/dart/pull/3875)

  * Fix distance queries and nearest points for touching convex pairs in FCL's
    libccd GJK distance in Linux wheels, demonstrated for ellipsoid,
    ellipsoid-sphere and box pairs. FCL 0.7.0 can abort with assertions (as on
    Ubuntu 24.04), or return -1 and zero nearest points. Only Linux wheels carry
    the fix; every other build, including the macOS and Windows wheels, pixi and
    conda-forge builds, and source builds using system FCL, remains affected.
    [#3870](https://github.com/dartsim/dart/pull/3870)

  * Fix undefined behavior when a custom deleter of a replaced collision
    detector or contact surface handler reads the constraint solver's
    current one.
    [#3820](https://github.com/dartsim/dart/pull/3820)

  * An exception thrown by a collision filter during an ODE collision check
    now reaches the caller after ODE finishes the check, instead of unwinding
    through ODE and leaving its collision space locked.
    [#3723](https://github.com/dartsim/dart/pull/3723)

* Constraint

  * Fix simulations blowing up when no joint motion can resolve a contact
    along its normal, such as overlapping links of a self-colliding skeleton
    on revolute joints whose shared axis is the contact normal. Such a contact
    now gets no normal impulse, and with the Dantzig solver alone it no longer
    zeroes the other contacts in its group.
    [#3707](https://github.com/dartsim/dart/pull/3707),
    [#3721](https://github.com/dartsim/dart/pull/3721)

* Dynamics

  * `VoxelGridShape::updateOccupancy()` no longer corrupts memory on very
    long rays or point clouds containing them. Rays that cross more than
    about 100,000 voxels are skipped with a warning (their endpoints are not
    marked occupied), and the other points of a cloud are still inserted.
    [#3859](https://github.com/dartsim/dart/pull/3859)

  * Release scenes assigned to the protected `MeshShape::mMesh` by legacy
    subclasses when the shape is destroyed. These scenes must be heap-allocated
    with `new` or by an assimp import; callers must not free or share them.
    Callers retaining ownership use `setMesh(..., MeshOwnership::None)`.
    [#3865](https://github.com/dartsim/dart/pull/3865)

  * Line segment and point cloud shapes now report bounding boxes that
    include vertices and points edited after a query, and clearing a point
    cloud now refreshes collision geometry.
    [#3826](https://github.com/dartsim/dart/pull/3826)

  * `Skeleton::getSupportIndices()` now names the end effector behind each
    support polygon vertex instead of returning an empty list, so
    `BalanceConstraint` no longer reads past its end.
    [#3823](https://github.com/dartsim/dart/pull/3823)

  * A `LineSegmentShape` built from two points now reports `LINE_SEGMENT`
    from the deprecated `getShapeType()`, like one built from a thickness.
    [#3823](https://github.com/dartsim/dart/pull/3823)

  * Line segments built from two points, planes built from a normal and a
    point, point clouds, and custom shapes built on the default `Shape`
    constructor now always compute their bounding box on the first query,
    instead of possibly returning a zero-size box at the shape's origin.
    Custom shapes also compute their volume instead of possibly returning
    zero.
    [#3819](https://github.com/dartsim/dart/pull/3819)

  * Keep prepared Worlds ready after topology edits in unrelated Worlds,
    while still rebuilding after changes to their own skeletons, solver
    membership, or collision-group dependencies.
    [#3818](https://github.com/dartsim/dart/pull/3818)

  * Point masses added with `SoftBodyNode::addPointMass()` no longer read or
    write their state out of bounds, and they now appear in the soft mesh
    and the skeleton version. `removeAllPointMasses()` no longer leaks the
    removed point masses.
    [#3813](https://github.com/dartsim/dart/pull/3813)

  * Keep a detached node's body and skeleton alive until the node finishes
    destruction, preventing crashes when releasing retained shape nodes,
    including GUI picks collected by Python.
    [#3784](https://github.com/dartsim/dart/pull/3784)

  * Fix crashes when cloning a skeleton, or copying a body node that has an
    end effector, whose inverse kinematics uses an analytical solver. Inverse
    kinematics copied by `BodyNode::copyTo()` or `BodyNode::copyAs()` now
    selects the copies of the original degrees of freedom, keeps the selected
    ones that were not copied when the copy stays in the same skeleton, and
    warns when none remain. Copies with analytical solvers warn and use
    `JacobianDLS` with normal iterative error settings, preserving user bounds
    and task-space regions. Set up the analytical solver again for the copy;
    whole-skeleton clones keep their analytical solvers.
    [#3722](https://github.com/dartsim/dart/pull/3722)

  * Initialize `HeightmapShape` minimum and maximum heights to zero, so a
    heightmap without a height field reports finite heights and bounds.
    [#3744](https://github.com/dartsim/dart/pull/3744)

  * `VoxelGridShape::updateOccupancy()` now writes a single point through
    OctoMap's double-precision update, so a point on a voxel boundary lands
    in the cell that `getOccupancy()` reads.
    [#3737](https://github.com/dartsim/dart/pull/3737)

  * Preserve the generated soft mesh and shape-node properties when cloning
    soft-body skeletons, avoiding duplicate null meshes during collision checks.
    [#3644](https://github.com/dartsim/dart/pull/3644)

  * Keep soft-body skin meshes resizable after `BodyNode::copyTo()` and
    `BodyNode::copyAs()`, preventing invalid mesh updates.

  * Fix mimic joints enforced by the coupler constraint
    (`Joint::setUseCouplerConstraint(true)`). Coupled joints no longer gain
    energy, or diverge when the follower has more inertia than the leader;
    per-DoF couplers on multi-DoF joints no longer crash; and a joint that
    mimics only a later DoF now gets its constraint.
    [#3700](https://github.com/dartsim/dart/pull/3700)

  * Preserve hierarchy-level objectives and `BalanceConstraint` settings when
    cloning IK hierarchies, and allow cloning with a cleared solver. Skip
    applying IK solutions with the wrong number of joint positions,
    including empty results from a cleared solver. Stop `HierarchicalIK`
    constraint gradients from crashing once their `HierarchicalIK` has been
    destroyed; like the constraint value, the gradient now reports the misuse
    and returns zero.
    [#3751](https://github.com/dartsim/dart/pull/3751)

* Python

  * Passing `None` for a frame, body or shape-node argument no longer crashes
    the interpreter. Where C++ defaults the argument to the world frame, `None`
    now means the world frame. Arguments where C++ gives `None` a meaning, such
    as `setParentFrame` to detach, keep that meaning. Everywhere else `None`
    raises `TypeError`, including `World.addSkeleton`, `World.removeSkeleton`,
    `CollisionGroup.addShapeFrame` and `BodyNode.createShapeNode`, which used
    to ignore it. `FreeJoint.setSpatialMotion` now accepts arrays for its
    transform, velocity and acceleration; `None` skips a motion and its frames.
    [#3841](https://github.com/dartsim/dart/pull/3841)

  * Add `dartpy.simulation.WorldState` with `World.capture_state(out=None)` and
    `World.restore_state(state)` for dynamic snapshots; state arrays are
    read-only NumPy copies and a structure mismatch raises `ValueError`.
    [#3842](https://github.com/dartsim/dart/pull/3842)

  * Returned Eigen vectors and matrices are read-only NumPy copies. In-place
    writes now raise `ValueError`; call `.copy()` before writing, then use the
    appropriate setter to update DART state. Eigen arguments that dartpy passes
    into Python overrides, such as `Function.eval(x)`, are read-only too.
    Existing live views and output buffers, such as a gradient's `grad`, keep
    their mutability.
    [#3843](https://github.com/dartsim/dart/pull/3843)

  * Allow clearing `RaycastOption.mFilter` with `None`, restoring unfiltered
    raycasts and releasing the stored Python callback.
    [#3810](https://github.com/dartsim/dart/pull/3810)

  * CMake development builds report the DART version in `dartpy.__version__`
    instead of an empty string. Explicit packaging versions remain supported.
    [#3809](https://github.com/dartsim/dart/pull/3809)

  * Keep gradient arrays passed to Python `optimizer.Function.evalGradient`
    overrides alive after the callback returns, while applying their updates
    to the native solver.
    [#3824](https://github.com/dartsim/dart/pull/3824)

  * Avoid crashes when native code releases Python objects, or calls Python
    collision filters, analytical IK solvers, optimizer overrides or GUI event
    handlers, while Python 3.15 or newer shuts down. Builds against the full Python API were
    affected; the stable-ABI wheels attach through an API that cannot fail
    this way.

  * Add dartpy soft-body construction, typed Euler-joint skeleton editing,
    and world-recording access for Python simulation tutorials.
    [#3644](https://github.com/dartsim/dart/pull/3644)

  * Python overrides stay alive while C++ uses them, graph-object wrappers
    retain their skeleton, and GUI nodes and handlers retain Python overrides
    while native code uses them.

  * Reject a null node when constructing inverse kinematics in dartpy,
    preventing an invalid-input crash.

  * Build `dartpy` wheels with the nanobind binder for CPython's stable ABI
    (new `DART_DARTPY_STABLE_ABI` CMake option): one `cp312-abi3` wheel per
    platform serves Python 3.12 and newer, so each release publishes four
    wheels (Linux x86_64 and aarch64, macOS arm64, Windows x64) instead of
    twelve.

  * `dartpy` is a Python package around a private native extension.
    `import dartpy` and every public `dartpy.<module>` name are unchanged; the
    package adds a `py.typed` marker and an empty `dartpy.experimental`
    subpackage outside semantic versioning. Source builds use scikit-build-core
    through `pyproject.toml`: `setup.py` is gone, sdist and editable installs
    are refused, the wheel no longer ships a `tests` package, and the
    `SKBUILD_BUILD_DIR` environment variable replaces `DARTPY_BUILD_TEMP`.
    Development builds import the package from `build/<env>/cpp/<type>/python`
    instead of its `dartpy` subdirectory.
    [#3847](https://github.com/dartsim/dart/pull/3847)

  * Source builds of `dartpy` on Windows work with the Ninja and NMake
    generators.

* Examples

  * Hide the ssik analytical IK demo's solved marker when no current
    forward-kinematics pose is available, including failed solves and arm
    loads.
    [#3824](https://github.com/dartsim/dart/pull/3824)

  * Replace the four C++ tutorials with Python exercises and solutions in
    `python/tutorials/{multi_pendulum,biped,collisions,dominoes}`. Existing
    `tu-*` Pixi commands now launch Python; the C++ tutorial executables and
    aggregate build target are removed.
    [#3644](https://github.com/dartsim/dart/pull/3644)

* Build

  * Shared installs with `DART_BUILD_UTILS_ASSETS=ON` no longer make consumers
    of `io`, `utils`, `utils-urdf` and `utils-assets` find libcurl and
    OpenSSL; only static installs require consumers to provide them.
    [#3883](https://github.com/dartsim/dart/pull/3883)

  * Release builds against Eigen 5 no longer fail under the default
    warnings-as-errors setting, which stopped at the runtime `JacobiSVD`
    options and `eulerAngles()` call Eigen 5 deprecates, and the C++ tests
    pass against Eigen 5.
    [#3836](https://github.com/dartsim/dart/pull/3836)

  * Raise the minimum assimp version to 5.3 (Ubuntu 24.04's 5.3.1, which
    reports 5.3.0).
    [#3865](https://github.com/dartsim/dart/pull/3865)

  * Remove `ASSIMP_AISCENE_CTOR_DTOR_DEFINED` and
    `ASSIMP_AIMATERIAL_CTOR_DTOR_DEFINED` from the installed `dart/config.hpp`.
    [#3865](https://github.com/dartsim/dart/pull/3865)

  * The installed `BulletCollisionDetector.hpp` no longer includes
    `<assimp/scene.h>`; code using assimp types through it must include that
    header directly.
    [#3865](https://github.com/dartsim/dart/pull/3865)

  * Link only BulletCollision and LinearMath instead of all four Bullet
    libraries, so the exported link interface and wheels no longer carry
    BulletDynamics and BulletSoftBody.
    [#3864](https://github.com/dartsim/dart/pull/3864)

  * `find_package(DART)` no longer requires pkg-config: without it, the
    installed package reads assimp's version from assimp's own CMake package.
    [#3829](https://github.com/dartsim/dart/pull/3829)

  * Restore discovery of the `collision-bullet` component with Ubuntu 22.04's
    Bullet package. The minimum Bullet is 3.05, because upstream Bullet 3.06
    reports itself as 3.05.
    [#3829](https://github.com/dartsim/dart/pull/3829)

  * GCC builds append `-Wl,--no-undefined` to the shared-library linker flags
    instead of replacing them, so flags from `LDFLAGS` reach DART's libraries.
    [#3829](https://github.com/dartsim/dart/pull/3829)

  * Add `DART_BUILD_EXAMPLES` (default `ON`); `OFF` skips the C++ examples,
    which MSVC builds by default.
    [#3829](https://github.com/dartsim/dart/pull/3829)

  * Include the generated `dart/config.hpp` and
    `dart/utils/urdf/BackwardCompatibility.hpp` in headers-only installations.
    [#3811](https://github.com/dartsim/dart/pull/3811)

  * Standard Pixi environments use sdformat 16.1 and urdfdom 6, reducing memory
    used to load large SDF worlds through shared schema descriptions.
    [#3803](https://github.com/dartsim/dart/pull/3803)

  * Allow C++-only projects to find the installed CMake package with the
    `collision-ode` component without enabling C.
    [#3756](https://github.com/dartsim/dart/pull/3756)

  * Restore URDF support and `dart-io` builds with urdfdom 3.x installations
    that ship no CMake version file, such as Ubuntu 22.04 and FreeBSD packages.
    The minimum urdfdom is now 3.0.0, because upstream 3.0.1 reports itself as
    3.0.0.

  * The installed CMake package accepts version requests from 6.0 up to the
    installed version, so `find_package(DART 6.10)` finds DART 7. `EXACT`
    requests and version ranges keep their meaning.
    [#3725](https://github.com/dartsim/dart/pull/3725)

  * MSVC projects that link the installed `dart` CMake target inherit
    `/permissive-` and `/Zc:twoPhase-`, the conformance options DART is
    compiled with, so DART's templates compile in those projects:
    [#1622](https://github.com/dartsim/dart/issues/1622)

  * Add independently configurable headless core builds using
    `DART_BUILD_IO=OFF`, `DART_BUILD_GUI=OFF` and `DART_BUILD_DARTPY=OFF`,
    including the complete removal of graphics dependency discovery and ImGui
    downloads from that build.
  * Allow GUI builds with IO disabled; `dart-gui` depends directly on core.
  * Keep dartpy as one all-in-one nanobind Python module linked to
    `dart`, `dart-io` and `dart-gui`, requiring IO and GUI.

  * Add the optional `utils-assets` component within `dart-io` for C++ and
    dartpy online model retrieval, with pinned sources, verified versioned
    caches, and offline reuse. Native builds opt in with
    `DART_BUILD_UTILS_ASSETS=ON` and require
    libcurl and OpenSSL; Pixi builds enable it by default. The catalog includes
    Atlas v5 and Unitree G1 model-viewing demos.

  * Add `DART_USE_SYSTEM_NANOBIND` (default `OFF`); the bundled build fetches
    nanobind 3.1.0 and its robin_map submodule. With `ON`, CMake uses
    `nanobind_DIR` when set and otherwise asks the build's Python.

  * Add `DART_USE_SYSTEM_FILAMENT` (default `ON`) to build the GUI against an
    installed Filament 1.77.3 SDK. `OFF` uses DART's pinned private SDK, built
    from source on Linux (Clang 20 or newer and Ninja) and downloaded prebuilt
    on macOS arm64 and 64-bit Windows.

  * Isolated Python package builds install scikit-build-core 1.1.1 or newer
    instead of setuptools and wheel.
    [#3791](https://github.com/dartsim/dart/pull/3791)

  * Raise the minimum `ninja` version for isolated Python package builds
    from 1.12.1 to 1.13.2.
    [#3792](https://github.com/dartsim/dart/pull/3792)

## DART 6

### [DART 6.20.1 (2026-10-18)](https://github.com/dartsim/dart/milestone/106?closed=1)

DART 6.20.1 is a patch release on the DART 6 line. It fixes mimic joints that
use the coupler constraint gaining energy or crashing, simulations that blow up
on contacts no joint motion can resolve, and crashes when copying analytical
inverse kinematics, loading SKEL worlds or releasing removed nodes. An
exception from a collision filter no longer leaves ODE's collision space
locked, and SKEL, SDF and MJCF files with the wrong number of vector or pose
values now fail to load instead of writing out of bounds. Builds work again
with urdfdom 3.x packages that ship no CMake version file, and C++-only
projects can find the `collision-ode` component. dartpy wheels now cover
Python 3.12 to 3.14, plus Python 3.15 on Linux; on x86_64 and aarch64 Linux
with glibc 2.28 or newer, pip on Python 3.10 and 3.11 keeps installing 6.20.0.
There is no API or ABI change, so code built against 6.20.0 runs unchanged;
rebuild it to pick up the `NodePtr` and `HeightmapShape` fixes, which are in
header-only templates.

* Collision

  * An exception thrown by a collision filter during an ODE collision check
    now reaches the caller after ODE finishes the check, instead of unwinding
    through ODE and leaving its collision space locked.
    [#3723](https://github.com/dartsim/dart/pull/3723)

* Constraint

  * Fix mimic joints enforced by the coupler constraint
    (`Joint::setUseCouplerConstraint(true)`). Coupled joints no longer gain
    energy, or diverge when the follower has more inertia than the leader,
    and per-DoF couplers on multi-DoF joints no longer crash. With or without
    the coupler, a joint that mimics only a later DoF, such as an SDF
    universal joint with an `<axis2>` mimic, now gets its mimic constraint:
    [#3702](https://github.com/dartsim/dart/pull/3702),
    [#3700](https://github.com/dartsim/dart/pull/3700)

  * Fix simulations blowing up when no joint motion can resolve a contact
    along its normal, such as overlapping links of a self-colliding skeleton
    on revolute joints whose shared axis is the contact normal. Such a contact
    now gets no normal impulse, and with the Dantzig solver alone it no longer
    zeroes the other contacts in its group:
    [#3716](https://github.com/dartsim/dart/pull/3716),
    [#3707](https://github.com/dartsim/dart/pull/3707),
    [#3721](https://github.com/dartsim/dart/pull/3721)

* Dynamics

  * Fix crashes when cloning a skeleton, or copying a body node that has an
    end effector, whose inverse kinematics uses an analytical solver. Inverse
    kinematics copied by `BodyNode::copyTo()` or `BodyNode::copyAs()` now
    selects the copies of the original degrees of freedom, keeps the selected
    ones that were not copied when the copy stays in the same skeleton, and
    warns when none remain. Copies with analytical solvers warn and use
    `JacobianDLS` with normal iterative error settings, preserving user bounds
    and task-space regions. Set up the analytical solver again for the copy;
    whole-skeleton clones keep their analytical solvers. Inverse kinematics
    on a body node of a cloned skeleton now moves the clone; before, it could
    not move it. `IkFast::getDofs()` now returns an empty list when IkFast
    cannot configure, as its warning says.
    [#3722](https://github.com/dartsim/dart/pull/3722)

  * Fix a crash when the last `NodePtr` to a removed node, such as a removed
    shape node, is released: the body and skeleton that the pointer kept alive
    now outlive the node's destruction. `NodePtr` is a header-only template,
    so rebuild code that holds one to get this fix.
    [#3784](https://github.com/dartsim/dart/pull/3784)

  * Initialize `HeightmapShape` minimum and maximum heights to zero, so a
    heightmap without a height field reports finite heights and bounds.
    `HeightmapShape` is a header-only template, so rebuild code that creates
    one to get this fix.
    [#3744](https://github.com/dartsim/dart/pull/3744)

* Parsers

  * SKEL, SDF and MJCF loading, and the `dart::utils` XML conversions such as
    `toVector3d()` and `toIsometry3d()`, throw `std::invalid_argument` when a
    2-, 3-, 4- or 6-element vector or pose value has the wrong number of
    elements, instead of writing out of bounds or leaving values unset in
    release builds. An MJCF weld `solref` with one value keeps the default for
    the other.
    [#3698](https://github.com/dartsim/dart/pull/3698)

  * Loading a SKEL world no longer crashes when the collision detector
    factory cannot create `fcl`, for example after an application
    unregisters the factory's creators. The parser creates the FCL detector
    directly instead and warns once.
    [#3750](https://github.com/dartsim/dart/pull/3750)

* Python

  * dartpy wheels now cover Python 3.12, 3.13 and 3.14 on Linux (x86_64,
    aarch64), macOS (arm64) and Windows. Python 3.10 (end of life) and 3.11
    no longer get wheels, so on x86_64 and aarch64 Linux with glibc 2.28 or
    newer, pip with those versions keeps installing dartpy 6.20.0. Source
    builds still support Python 3.10 and newer:
    [#3704](https://github.com/dartsim/dart/pull/3704)

  * dartpy publishes Python 3.15 wheels for Linux x86_64 and aarch64. macOS
    arm64 and Windows x64 follow once conda-forge's bullet-cpp builds for
    Python 3.15. [#3730](https://github.com/dartsim/dart/pull/3730)

* Build

  * Restore `dart-utils-urdf` builds with urdfdom 3.x installations that ship
    no CMake version file, such as Ubuntu 22.04 and FreeBSD packages. The
    minimum urdfdom is now 3.0.0, because upstream 3.0.1 reports itself as
    3.0.0.
    [#3733](https://github.com/dartsim/dart/pull/3733)

  * Allow C++-only projects to find the installed CMake package with the
    `collision-ode` component without enabling C.
    [#3756](https://github.com/dartsim/dart/pull/3756)

### [DART 6.20.0 (2026-10-11)](https://github.com/dartsim/dart/milestone/99?closed=1)

DART 6.20.0 is a minor release on the DART 6 line focused on contact-heavy
performance. The 3,000-shape scene from
[#3056](https://github.com/dartsim/dart/issues/3056), run in a headless
simulation host with the ODE detector, completes 18.9x faster than with DART
6.19.4, and every body comes to rest and sleeps instead of sinking through the
ground. The default FCL detector now collides exact primitive
shapes, more resting scenes fall asleep and wake when they should, the opt-in
`dart` collision detector becomes a complete backend, and two opt-in friction
solvers are available. The release is not ABI compatible with 6.19: read
Breaking Changes and Behavior Changes before upgrading. The milestone linked
above lists every merged change.

* Breaking Changes

  * Rebuild all code that links against DART. The library SONAME is now
    `libdart.so.6.20`, and `Skeleton`, `World`, `MemoryManager` and
    `SimpleFrame` (plus `ShapeNode` and `SoftBodyNode` in AVX builds) change
    layout. The `SimpleFrame` change fixes a crash when it is constructed with
    GCC 16, or with AVX code generation on older GCC:
    [#3447](https://github.com/dartsim/dart/issues/3447),
    [#3504](https://github.com/dartsim/dart/pull/3504),
    [#3552](https://github.com/dartsim/dart/pull/3552),
    [#3627](https://github.com/dartsim/dart/pull/3627),
    [#3632](https://github.com/dartsim/dart/pull/3632)

  * dartpy is built with pybind11 3.x. Rebuild extension modules that pass
    dartpy objects across the Python/C++ boundary against the same pybind11
    release:
    [#3344](https://github.com/dartsim/dart/pull/3344),
    [#3643](https://github.com/dartsim/dart/pull/3643)

  * Six components are removed. Drop them from
    `find_package(DART COMPONENTS ...)` and migrate code that includes their
    headers:

    * `gui`, the GLUT viewer, including `dart/gui/glut` and `gui.hpp`: use
      `gui-osg`.
    * `optimizer-ipopt`, `optimizer-nlopt` and `optimizer-pagmo`, with the
      multi-objective API, the `dart.optimizer.NloptSolver` binding and the
      `HAVE_IPOPT`, `HAVE_NLOPT`, `HAVE_PAGMO` and `HAVE_SNOPT` macros: use the
      [dart-optimization](https://github.com/dartsim/dart-optimization)
      package. The optimizer core used by inverse kinematics remains.
    * `external-lodepng`, with GLUT screenshots: use the OSG viewer's capture
      support.
    * `external-odelcpsolver`: a native Dantzig solver replaces it, and the
      `dart/lcpsolver/dantzig/*.h` forwarders remain.

    The header-only `dart/external/convhull_3d` and `dart/integration` are
    removed too; use `math::computeConvexHull3D` and `World::step()`:
    [#3076](https://github.com/dartsim/dart/pull/3076),
    [#3088](https://github.com/dartsim/dart/pull/3088),
    [#3105](https://github.com/dartsim/dart/pull/3105),
    [#3116](https://github.com/dartsim/dart/pull/3116),
    [#3122](https://github.com/dartsim/dart/pull/3122)

  * `dart/config.hpp` no longer defines `DART_BUILD_MODE_DEBUG` or
    `DART_BUILD_MODE_RELEASE`; test `NDEBUG` instead. DART's debug-only checks
    now follow `NDEBUG` rather than the build type, and the unused
    `BoxedLcpSolver::canSolve()` is removed:
    [#3627](https://github.com/dartsim/dart/pull/3627)

  * Source builds need GCC 11.2, Clang 13, Apple Clang 14 (Xcode 14.1) or
    Visual Studio 2022, on Ubuntu 22.04, macOS 13 or newer, and dartpy needs
    Python 3.12 and NumPy 1.21.5 or newer. The
    [build requirements](https://dart.readthedocs.io/en/latest/dart/developer_guide/build.html)
    list the dependency versions:
    [#3643](https://github.com/dartsim/dart/pull/3643)

  * The build stops when FCL uses a single-precision libccd (libccd's
    default), which causes momentum drift on shallow contacts. Rebuild libccd
    with `-DENABLE_DOUBLE_PRECISION=ON` and FCL against it, or configure with
    `-DDART_ALLOW_SINGLE_PRECISION_LIBCCD=ON`:
    [#3590](https://github.com/dartsim/dart/issues/3590),
    [#3630](https://github.com/dartsim/dart/pull/3630)

* Behavior Changes

  These change simulation results without any code change.

  * `FCLCollisionDetector`, the default detector, now collides supported
    primitive shapes as exact FCL primitives (`PRIMITIVE`) instead of mesh
    approximations, with corrected contact normals. The default constraint
    solver and the SKEL parser follow. Select `MESH` or the `fcl_mesh` detector
    for the previous behavior:
    [#3131](https://github.com/dartsim/dart/pull/3131)

  * Sleeping (on by default since 6.19) covers more resting scenes: islands
    that use a custom `BodyNodeCollisionFilter` can now sleep, and imported
    worlds whose bodies start at rest on level supports settle within their
    first steps. A stricter quiet check keeps slowly creeping bodies awake, so
    a settling body falls asleep later than in 6.19, about 0.5 s later for a
    box dropped onto a floor. As in 6.19, each island sleeps independently of
    other moving bodies and wakes on contact. Set
    `DeactivationOptions::mEnabled` to `false` for always-active simulation,
    or `DART_CUSTOM_FILTER_SLEEPING=0` to keep custom-filter islands awake:
    [#3086](https://github.com/dartsim/dart/pull/3086),
    [#3112](https://github.com/dartsim/dart/pull/3112),
    [#3565](https://github.com/dartsim/dart/pull/3565),
    [#3632](https://github.com/dartsim/dart/pull/3632),
    [#3683](https://github.com/dartsim/dart/pull/3683)

  * When a step needs more contacts than `CollisionOption::maxNumContacts`
    allows, the cap is now shared across the colliding pairs. Detection used
    to stop at the cap, so the pairs found last got no contacts and their
    bodies fell through their support. Scenes that stay under the cap are
    unchanged; with more colliding pairs than the cap, some pairs still get no
    contacts, so raise the cap if bodies sink:
    [#3551](https://github.com/dartsim/dart/pull/3551)

  * Collision objects for shapes added after `World::addSkeleton()` are
    created in subscription order, so such scenes are reproducible across
    runs; their trajectories change once:
    [#3607](https://github.com/dartsim/dart/pull/3607)

  * The ODE detector adds the support contact it misses for a cylinder resting
    on a plane, so settled cylinders join their island and can sleep:
    [#3203](https://github.com/dartsim/dart/pull/3203)

  * `BulletCollisionDetector` drops contacts with negative penetration depth
    by default. Set `CollisionOption::allowNegativePenetrationDepthContacts`
    to keep these proximity hits:
    [#3136](https://github.com/dartsim/dart/pull/3136)

* Performance

  * Large contact-heavy worlds run much faster: in a headless simulation host
    with the ODE detector and one physics thread, the 3,000-shape scene from
    [#3056](https://github.com/dartsim/dart/issues/3056) completes 3 simulated
    seconds 18.9x faster than with DART 6.19.4, mostly because its bodies now
    settle and sleep early. Awake steps are cheaper too: contact constraints
    are reused across steps and built in parallel, single-body contact groups
    assemble their LCP directly, and steps where every island rests reuse
    cached state:
    [#3140](https://github.com/dartsim/dart/pull/3140),
    [#3142](https://github.com/dartsim/dart/pull/3142),
    [#3146](https://github.com/dartsim/dart/pull/3146),
    [#3147](https://github.com/dartsim/dart/pull/3147),
    [#3149](https://github.com/dartsim/dart/pull/3149),
    [#3151](https://github.com/dartsim/dart/pull/3151)

  * The ODE detector looks up contact-history and contact pairs through sorted
    indexes: an awake 3,000-body step runs about 30% fewer instructions on a
    `PlaneShape` ground and about 20% fewer on a box ground, with bit-exact
    results:
    [#3574](https://github.com/dartsim/dart/pull/3574)

  * `MetaSkeleton::getPositions()` takes about a quarter fewer instructions,
    and the other DOF getters and setters are faster too, with unchanged
    results:
    [#3554](https://github.com/dartsim/dart/pull/3554)

  * Add opt-in multithreading: `World::setNumSimulationThreads()`, also in
    dartpy, solves independent constraint islands and builds contacts in
    parallel. Custom constraints, contact handlers and LCP solvers, and bodies
    shared across islands, keep the serial path:
    [#3071](https://github.com/dartsim/dart/pull/3071),
    [#3111](https://github.com/dartsim/dart/pull/3111),
    [#3149](https://github.com/dartsim/dart/pull/3149)

  * Add opt-in allocation-free stepping: after `World::enterSimulationMode()`
    or the first step, scenes that use the `dart` detector and keep the same
    shapes step without heap allocations. Preparing a world keeps its sleep
    state and does not call custom contact surface handlers:
    [#3297](https://github.com/dartsim/dart/pull/3297),
    [#3549](https://github.com/dartsim/dart/pull/3549),
    [#3563](https://github.com/dartsim/dart/pull/3563),
    [#3587](https://github.com/dartsim/dart/pull/3587),
    [#3615](https://github.com/dartsim/dart/pull/3615)

* Collision

  * The opt-in `dart` detector (`DARTCollisionDetector`) becomes a complete,
    dependency-free backend. It adds capsules, cones, ellipsoids and soft
    meshes, `distance()` and `raycast()` queries, `VoxelGridShape`, and opt-in
    persistent contact manifolds that seed cached impulses, on a dynamic
    AABB-tree broadphase with a much faster contact pipeline. FCL remains the
    default, and the released `DARTCollide` entry points stay compatible:
    [#3056](https://github.com/dartsim/dart/issues/3056),
    [#3126](https://github.com/dartsim/dart/pull/3126),
    [#3307](https://github.com/dartsim/dart/pull/3307),
    [#3321](https://github.com/dartsim/dart/pull/3321),
    [#3343](https://github.com/dartsim/dart/pull/3343),
    [#3352](https://github.com/dartsim/dart/pull/3352),
    [#3355](https://github.com/dartsim/dart/pull/3355),
    [#3358](https://github.com/dartsim/dart/pull/3358),
    [#3360](https://github.com/dartsim/dart/pull/3360),
    [#3368](https://github.com/dartsim/dart/pull/3368),
    [#3381](https://github.com/dartsim/dart/pull/3381)

  * Add `CollisionOption::maxNumContactsPerPair` to cap the contacts of each
    colliding pair; the `dart` detector keeps the deepest contact and
    well-spread support contacts under the cap:
    [#3085](https://github.com/dartsim/dart/pull/3085),
    [#3135](https://github.com/dartsim/dart/pull/3135)

  * Fix `CollisionGroup` bookkeeping: `removeAllShapeFrames()` no longer leaves
    subscriptions that crashed the next update, a two-group `collide()` or
    `distance()` refreshes both groups, and creating or removing a
    `CollisionAspect` takes effect at the next query without reading freed
    records:
    [#3509](https://github.com/dartsim/dart/pull/3509),
    [#3510](https://github.com/dartsim/dart/pull/3510),
    [#3553](https://github.com/dartsim/dart/pull/3553)

  * Fix a crash in the ODE detector when a shape already in a collision group
    is replaced or resized, and keep shape updates when the detector is
    switched after skeletons were added:
    [#3547](https://github.com/dartsim/dart/pull/3547),
    [#3576](https://github.com/dartsim/dart/pull/3576)

  * Fix the FCL detector adding an empty contact with null collision objects
    when contact generation is disabled, which aborted asserts-enabled builds:
    [#3114](https://github.com/dartsim/dart/pull/3114)

* Constraint

  * Add opt-in friction solvers: `NsgsFrictionSolver`, with exact Coulomb,
    associated and box friction laws, and `FbfFrictionSolver`, a
    forward-backward-forward solver for exact Coulomb friction. Both report
    `FrictionSolveStats` and support cloning and dartpy, and NSGS reuses
    converged contact impulses across steps. The default Dantzig/PGS solver
    is unchanged:
    [#3617](https://github.com/dartsim/dart/pull/3617),
    [#3631](https://github.com/dartsim/dart/pull/3631),
    [#3636](https://github.com/dartsim/dart/pull/3636)

  * Add an opt-in matrix-free contact solver to `BoxedLcpConstraintSolver` for
    large contact-only islands:
    [#3361](https://github.com/dartsim/dart/pull/3361)

  * Reject non-finite LCP solutions and skip non-finite contacts, and make box,
    cylinder, capsule, ellipsoid, cone and pyramid shapes reject NaN, infinite
    or non-positive sizes, so invalid geometry no longer crashes the solver or
    spreads NaN through the simulation:
    [#3132](https://github.com/dartsim/dart/pull/3132),
    [#3384](https://github.com/dartsim/dart/pull/3384)

  * Fix split-impulse position correction discarding velocity-phase impulses
    and failing for contacts between several reactive skeletons:
    [#3567](https://github.com/dartsim/dart/pull/3567),
    [#3582](https://github.com/dartsim/dart/pull/3582)

  * Fix constraint parameter setters storing an out-of-range value after
    warning that it was clamped, and make mimic motor constraints robust for
    joints without finite limits:
    [#3137](https://github.com/dartsim/dart/pull/3137),
    [#3511](https://github.com/dartsim/dart/pull/3511)

* Simulation

  * Sleeping bodies wake when their dynamics change: joint spring, damping
    and friction edits, gravity and joint axis changes, `World::reset()`, and
    new manual constraints. Bodies stay awake while a custom
    `ContactSurfaceHandler`, such as a conveyor, is installed; re-applying an
    unchanged gravity no longer wakes them; each `World` sleeps independently;
    and raising the `DeactivationOptions` thresholds lets noisier scenes sleep:
    [#3226](https://github.com/dartsim/dart/pull/3226),
    [#3552](https://github.com/dartsim/dart/pull/3552)

  * Add `CollisionDetectorType` and `World::setCollisionDetector()` /
    `World::getCollisionDetector()`, also in dartpy, plus the C++
    `WorldConfig`, to choose the FCL, Bullet, ODE or `dart` detector without
    reaching into the constraint solver:
    [#2168](https://github.com/dartsim/dart/pull/2168)

  * Cloning a `World` keeps its built-in solver backends and options, split
    impulse and contact settings; a custom boxed LCP backend falls back to the
    clone's default with a warning:
    [#3568](https://github.com/dartsim/dart/pull/3568)

  * Fix `World::getIndex()` after a skeleton is removed or changes its joints.
    `Recording` no longer misreads baked frames after the DOF layout changes:
    adding or removing a skeleton or changing its joints now drops the frames
    it can no longer interpret, with a warning:
    [#3508](https://github.com/dartsim/dart/pull/3508),
    [#3513](https://github.com/dartsim/dart/pull/3513)

* Dynamics

  * Add opt-in adaptive soft contact activation to `SoftBodyNode`, following
    Jain and Liu's soft contact model: point masses near contact stay
    simulated while the rest move rigidly with the parent body. The default
    is unchanged:
    [#3382](https://github.com/dartsim/dart/pull/3382)

  * Fix `Skeleton::cloneSkeleton()` dropping soft bodies' generated meshes and
    shape-node properties, and keep soft-body skin meshes resizable after
    `BodyNode::copyTo()` and `BodyNode::copyAs()`:
    [#3644](https://github.com/dartsim/dart/pull/3644),
    [#3662](https://github.com/dartsim/dart/pull/3662)

  * Add per-DoF actuator types to `Joint`, so individual DoFs of a multi-DoF
    joint can mimic another DoF while the rest keep the joint-wide type:
    [#2222](https://github.com/dartsim/dart/pull/2222),
    [#3169](https://github.com/dartsim/dart/pull/3169)

  * Joint spring stiffness and damping changes take effect on the next step
    instead of after the joint configuration changes:
    [#3552](https://github.com/dartsim/dart/pull/3552)

  * Store meshes as `TriMesh` with material metadata, and add `PolygonMesh` to
    keep quads and n-gons for rendering; the Assimp `aiScene` accessors remain
    as deprecated shims. `ArrowShape` meshes get valid material indices:
    [#3145](https://github.com/dartsim/dart/pull/3145),
    [#3167](https://github.com/dartsim/dart/pull/3167),
    [#3169](https://github.com/dartsim/dart/pull/3169)

  * Fix `TranslationalJoint2D::copy()` and `UniversalJoint::copy()` copying
    from themselves, and an ARM64 crash in `Icosphere`:
    [#3130](https://github.com/dartsim/dart/pull/3130)

  * Compute soft-body helper rest positions, `HeightmapShape<float>` bounds
    and `FCLCollisionDetector`'s `MESH`-mode primitive meshes in double
    precision, and make `ODELCPSolver::Solve` reject problems too large for
    its `int` offsets:
    [#3581](https://github.com/dartsim/dart/pull/3581)

* Parsers

  * Load SDF `<soft_shape>` links as `SoftBodyNode`s instead of rigid bodies,
    and import SDF `<mimic>` joints:
    [#2254](https://github.com/dartsim/dart/pull/2254),
    [#3399](https://github.com/dartsim/dart/pull/3399)

  * Load MJCF stacked hinge and slide joints, `contype`/`conaffinity`
    collision filtering and per-geom friction:
    [#3369](https://github.com/dartsim/dart/pull/3369)

  * Apply URDF planar and floating joint limits, rest positions, damping and
    friction to all their DOFs, and read planar joint axes from `<axis>`:
    [#3134](https://github.com/dartsim/dart/pull/3134)

* Python

  * Linux wheels target glibc 2.28 and newer (`manylinux_2_28`) on x86_64 and,
    new in this release, aarch64:
    [#3661](https://github.com/dartsim/dart/pull/3661)

  * Fix dartpy crashes and errors: discarding copied joints, reading colliding
    shape frames, using contacts after their result is released, returned
    lists that deleted DART-owned objects, and bindings that raised
    `TypeError`, such as `Chain.getBodyNodes()` and
    `CollisionGroup.addShapeFrames()`. `ConstraintSolver.getNumConstraints()`
    returns the count instead of a boolean, and Python collision filters can
    now override collision checks:
    [#3375](https://github.com/dartsim/dart/pull/3375),
    [#3619](https://github.com/dartsim/dart/pull/3619),
    [#3626](https://github.com/dartsim/dart/pull/3626)

  * Add bindings for inverse-kinematics gradient and analytical methods,
    including Python analytical IK callbacks, plus `math.BoundingBox` and a
    default `optimizer.ModularFunction` constructor:
    [#3092](https://github.com/dartsim/dart/pull/3092),
    [#3374](https://github.com/dartsim/dart/pull/3374),
    [#3619](https://github.com/dartsim/dart/pull/3619)

* GUI

  * Add a `DebugOverlay` that draws grids, frames, contacts, velocities,
    centers of mass and collision bounds over the OSG viewer, plus adaptive
    camera viewpoints and headless capture helpers, also in dartpy:
    [#3314](https://github.com/dartsim/dart/pull/3314),
    [#3374](https://github.com/dartsim/dart/pull/3374)

  * Fix a use-after-free after a `MouseEventHandler` is destroyed, leaked
    `InteractiveFrameDnD` tools, `InteractiveFrame` handles not rendering
    under `ImGuiViewer`, and soft-body faces disappearing from some camera
    angles:
    [#3092](https://github.com/dartsim/dart/pull/3092),
    [#3300](https://github.com/dartsim/dart/pull/3300),
    [#3304](https://github.com/dartsim/dart/pull/3304)

  * Add viewer helpers for applications: GUI-scale parsing and
    `ImGuiHandler` scaling, `applyDefaultCameraPose()`,
    `RealTimeWorldNode::getSmoothedRealTimeFactor()`, and the
    `DART_IMGUI_DOCKING` option for bundled ImGui builds:
    [#3092](https://github.com/dartsim/dart/pull/3092),
    [#3209](https://github.com/dartsim/dart/pull/3209),
    [#3426](https://github.com/dartsim/dart/pull/3426)

* Examples and Tutorials

  * Replace the individual GUI examples with one `dart-demos` application: a
    scene navigator, a dockable ImGui workspace with inspectors,
    contact-force display and a live profiler, DPI-aware scaling, and headless
    capture. `hello_world`, `cylindrical_constraint`, `speed_test`,
    and `contact_benchmark` stay standalone; dartpy analytical IK is available
    as the `ssik_ik_gui` scene in the demo catalog:
    [#3301](https://github.com/dartsim/dart/pull/3301),
    [#3426](https://github.com/dartsim/dart/pull/3426),
    [#3640](https://github.com/dartsim/dart/pull/3640)

  * Add deformable-body (`adaptive_soft_contact`, `soft_worm`) and ssik
    inverse-kinematics (`ssik_ik_gui`) scenes to the demo catalog:
    [#3092](https://github.com/dartsim/dart/pull/3092),
    [#3382](https://github.com/dartsim/dart/pull/3382)

* Math

  * Add the header-only `dart/simd` module with portable SIMD vector types,
    math kernels and Eigen interop:
    [#2490](https://github.com/dartsim/dart/pull/2490)

* Build

  * Build as C++17 regardless of the compiler default, and fix a newer
    AppleClang warning, so GCC 16 and AppleClang build with warnings as
    errors:
    [#3138](https://github.com/dartsim/dart/pull/3138),
    [#3505](https://github.com/dartsim/dart/pull/3505)

  * Accept `CMAKE_BUILD_TYPE=None` for distribution packaging, honor
    `DART_SKIP_spdlog=ON` when spdlog is installed, and fix `DART_ARCH_64BITS`
    on 64-bit Windows and aarch64 Linux and FreeBSD detection:
    [#3512](https://github.com/dartsim/dart/pull/3512),
    [#3624](https://github.com/dartsim/dart/pull/3624),
    [#3633](https://github.com/dartsim/dart/pull/3633)

  * Stop a deprecation warning from DART's logging headers with fmt 12.2 and
    newer:
    [#3202](https://github.com/dartsim/dart/pull/3202)

  * Source builds use an installed `sccache` or `ccache` automatically; set
    `DART_DISABLE_COMPILER_CACHE=ON` to build without one:
    [#3326](https://github.com/dartsim/dart/pull/3326)

  * Fetch the bundled ImGui (v1.92.8) at configure time instead of vendoring
    it, with an opt-in system ImGui, and move the IKFast header to
    `dart/dynamics/ikfast.h` while keeping the old include path:
    [#3078](https://github.com/dartsim/dart/pull/3078),
    [#3081](https://github.com/dartsim/dart/pull/3081),
    [#3198](https://github.com/dartsim/dart/pull/3198)

### [DART 6.19.5 (2026-10-04)](https://github.com/dartsim/dart/milestone/104?closed=1)

DART 6.19.5 is a focused patch release on the DART 6 LTS line. It backports
the ODE collision detector's contact-history speedup from the DART 6.20 line,
so contact-heavy scenes like those Gazebo runs step about 2x faster while
producing results bit-identical to DART 6.19.4. There is no API or ABI
change: plugins built against 6.19.4, such as gz-physics, run unchanged.

* Performance

  * Speed up the ODE collision detector in contact-heavy scenes: a 3,000-body
    Gazebo-style scene steps about 2x faster, with bit-exact results:
    [#3555](https://github.com/dartsim/dart/pull/3555),
    [#3329](https://github.com/dartsim/dart/pull/3329)

### [DART 6.19.4 (2026-07-18)](https://github.com/dartsim/dart/milestone/103?closed=1)

DART 6.19.4 is a focused patch release on the DART 6 LTS line. It restores
source builds that disable the OpenSceneGraph GUI component and makes that
optional-dependency contract deterministic in CI. The only additional
maintenance change ignores local `.omx/` state.

* Build

  * Keep source builds configurable without OpenSceneGraph when
    `DART_BUILD_GUI_OSG=OFF` by skipping the OSG-only dynamic-joint-constraints
    example before it requests `osgText`. The ASan configuration now disables
    OpenSceneGraph discovery so CI fails if this dependency becomes mandatory
    again:
    [#3391](https://github.com/dartsim/dart/pull/3391),
    [#3387](https://github.com/dartsim/dart/issues/3387)

* Tooling and Docs

  * Ignore local `.omx/` state so it is not accidentally committed:
    [#3391](https://github.com/dartsim/dart/pull/3391)

### [DART 6.19.3 (2026-06-27)](https://github.com/dartsim/dart/milestone/101?closed=1)

DART 6.19.3 is a patch release on the DART 6 LTS line. It hardens contact
handling against invalid geometry and silences an fmt 12.2 deprecation warning
that leaked through DART's logging headers, both addressing issues reported
through gz-physics. It also recovers from two upstream toolchain regressions,
restoring the Linux Python wheels and compilation on the latest Clang.

* Simulation

  * Reject invalid shape geometry and non-finite contacts instead of crashing.
    `BoxShape`, `CylinderShape`, `CapsuleShape`, `EllipsoidShape`, `ConeShape`,
    and `PyramidShape` now reject non-finite (NaN/Inf) or non-positive
    dimensions, matching `SphereShape`, and `ConstraintSolver` drops any contact
    whose point, normal, or penetration depth is non-finite before building a
    contact constraint. Together these keep a bad shape dimension or a
    non-finite contact (from a mesh or a collision backend) from tripping the
    `ContactConstraint` `mSpatialNormalA` assertion or feeding NaN/Inf into the
    LCP solve:
    [#3117](https://github.com/dartsim/dart/pull/3117),
    [gazebosim/gz-physics#1010](https://github.com/gazebosim/gz-physics/issues/1010)

* Build

  * Stop an fmt 12.2 deprecation warning from leaking out of DART's header-only
    logging. spdlog's variadic `log()` routes the format string through a
    `fmt::format_string` -> `string_view` conversion that fmt 12.2.0 deprecated,
    so the warning surfaced in every downstream that instantiates DART logging.
    DART now pre-formats the message and hands spdlog a ready-made string:
    [#3201](https://github.com/dartsim/dart/pull/3201),
    [gazebosim/gz-physics#1018](https://github.com/gazebosim/gz-physics/issues/1018)

  * Restore the Linux `manylinux` Python wheels. The wheel build installs the
    newest CMake at build time, and CMake 4.3.4 stopped detecting `freeglut3`
    in the build container, which dropped the `dart-gui` target the dartpy wheel
    links and broke configuration. The build now caps CMake below 4.3.4 until a
    fixed release ships:
    [#3197](https://github.com/dartsim/dart/pull/3197)

  * Compile cleanly on the latest Clang. Recent Clang (such as the current
    `macos-latest` image) rejects the deprecated whitespace form of
    user-defined-literal operators under `-Werror`, so the `_pi`, `_rad`, and
    `_deg` suffixes now use the non-deprecated `operator""_x` spelling:
    [#3197](https://github.com/dartsim/dart/pull/3197)

### [DART 6.19.2 (2026-06-19)](https://github.com/dartsim/dart/milestone/100?closed=1)

DART 6.19.2 is a patch release on the DART 6 LTS line. It keeps automatic
sleeping enabled while adding regression coverage for the gz-sim wheel-command
path and extending Gazebo validation to cover the downstream gz-sim failure
reported against DART 6.19.1.

* Tests

  * Cover servo-commanded sleeping bodies against an always-awake baseline, and
    extend the Gazebo validation task to apply the gz-physics
    `ChangedWorldPoses` root-cause patch before running gz-sim's
    `INTEGRATION_entity_system` regression against the source-built DART 6
    plugin:
    [#3068](https://github.com/dartsim/dart/pull/3068),
    [gazebosim/gz-sim#3697](https://github.com/gazebosim/gz-sim/issues/3697)

### [DART 6.19.1 (2026-06-17)](https://github.com/dartsim/dart/milestone/98?closed=1)

* Simulation

  * Preserve the last solved contact body-force cache when an island first
    transitions into automatic deactivation, so joint transmitted-wrench
    queries continue to include contact forces while the island is asleep:
    [#3051](https://github.com/dartsim/dart/pull/3051),
    [gazebosim/gz-physics#1007](https://github.com/gazebosim/gz-physics/issues/1007)

* Performance

  * Speed up the articulated-body forward-dynamics hot path by caching the
    owning `Skeleton` pointer in `BodyNode` and avoiding fixed-size to dynamic
    Eigen temporaries in `math::isNan` and `math::isInf`, improving the
    `BM_Dynamics` benchmark by about 18% while preserving bit-exact results:
    [#3028](https://github.com/dartsim/dart/pull/3028)

  * Remove per-pivot heap allocations in `ContactConstraint` by evaluating
    spatial-normal matrix-vector products directly into the solver buffers,
    reducing contact-heavy `boxes` per-step heap allocations by about 58% and
    improving wall-clock time by about 6% with bit-exact results:
    [#3033](https://github.com/dartsim/dart/pull/3033)

  * Avoid heap temporaries in `BodyNode` force-aggregation routines by writing
    Jacobian-transpose products directly into Skeleton tree-cache segments:
    [#3037](https://github.com/dartsim/dart/pull/3037)

  * Reduce remaining contact-heavy per-step heap allocations by scanning
    external disturbances per DOF and storing contact spatial normals inline,
    cutting `boxes` heap allocations by about 38% on top of the earlier
    allocation reductions while preserving bit-exact finite-state results:
    [#3040](https://github.com/dartsim/dart/pull/3040)

* Tests

  * Build and run the pinned gz-physics test suite in the DART 6 Gazebo CI gate
    instead of only checking that the plugin links, and add macOS x64/arm64
    coverage so downstream Homebrew regressions are caught before release:
    [#3050](https://github.com/dartsim/dart/pull/3050),
    [gazebosim/gz-physics#1007](https://github.com/gazebosim/gz-physics/issues/1007)

  * Cover the sleep-transition wrench-cache regression paths, including the
    final contact impulse before sleep, non-candidate wakeup, and stale
    constraint-island index handling:
    [#3055](https://github.com/dartsim/dart/pull/3055)

  * Loosen the `Issue1184` resting-accuracy regression tolerance from `1e-3` to
    `2e-3` so it is not flipped by micrometer-scale cross-platform
    floating-point differences in the steady-state contact penetration (~1 mm),
    which sat exactly on the old bound; fixes the macOS build of the
    conda-forge feedstock:
    [#3031](https://github.com/dartsim/dart/pull/3031)

### [DART 6.19.0 (2026-06-14)](https://github.com/dartsim/dart/milestone/97?closed=1)

DART 6.19.0 is a minor release on the DART 6 LTS line. It adds automatic body
deactivation ("sleeping"), contact-aware inverse dynamics, and a cylindrical
dynamic joint constraint, together with a constraint-solver hot-path
optimization and build/example fixes for modern toolchains.

* Build

  * Include `<cassert>` before the FCL headers in the FCL compatibility shim so
    DART builds against Eigen 5 and current GCC/Clang toolchains, which no
    longer pull `<cassert>` in transitively (FCL's shape headers call `assert`
    without including it): [#3021](https://github.com/dartsim/dart/pull/3021)

* Constraint

  * Add `CylindricalJointConstraint` for runtime slide-and-rotate attachments
    that leave translation along an axis and rotation about that axis free,
    with headless and GUI examples covering DART 6.x dynamic joint constraints:
    [#2923](https://github.com/dartsim/dart/pull/2923)

  * Optimize the DART 6 constraint-solver hot path (box-stacking benchmarks
    ~1.28x faster) with bit-exact results, adding a lightweight profiling-scope
    helper used to measure it:
    [#3023](https://github.com/dartsim/dart/pull/3023)

* Dynamics

  * Add `ContactInverseDynamics` and `math::solveNonNegativeLeastSquares` to
    compute joint torques together with friction-cone-consistent contact
    forces for tracked motions with contacts, so floating-base inverse
    dynamics no longer requires an external QP solver, with unit tests,
    benchmarks, dartpy bindings for `ContactInverseDynamics` (the
    `math::solveNonNegativeLeastSquares` solver is C++-only), and an ImGui-based
    GUI example: [#2985](https://github.com/dartsim/dart/pull/2985)

* Examples and Tutorials

  * Enlarge the floor and add an FPS / physics real-time-factor HUD to the
    `glut_add_delete_skels` example, so spawned cubes keep room on the ground
    and the effect of adding skeletons on real-time performance is visible:
    [#3019](https://github.com/dartsim/dart/pull/3019)

* GUI

  * Fix every GLUT example failing to open a window on modern Mesa/Xwayland:
    the shared GLUT window no longer requests a legacy, now-unavailable
    accumulation buffer, selecting the framebuffer with a relaxable
    `glutInitDisplayString` instead, while `MotionBlurSimWindow` opts back into
    an accumulation buffer on its own:
    [#3017](https://github.com/dartsim/dart/pull/3017)

* Simulation

  * Add automatic body deactivation ("sleeping") for resting solver islands,
    including wake-on-contact/force handling and island-index diagnostics. It is
    enabled by default, so resting bodies now freeze automatically; the
    detection is a deterministic no-op while bodies are active, and it can be
    turned off via `World::setDeactivationOptions` to retain the previous
    always-active behavior: [#2920](https://github.com/dartsim/dart/pull/2920)

### [DART 6.18.0 (2026-06-05)](https://github.com/dartsim/dart/milestone/94?closed=1)

DART 6.18.0 is a minor release on the DART 6 LTS line. It adds two contact and
constraint robustness improvements surfaced through gz-physics, so resting
capsules/cylinders and non-axis-aligned closed kinematic chains behave correctly
on the classic DART 6 API.

* Collision

  * Stabilize ODE capsule and cylinder resting contacts with a per-pair contact-history manifold (invalidated on sliding and kinematic pose changes) so they no longer sink through the supporting surface: [#2902](https://github.com/dartsim/dart/pull/2902), [gazebosim/gz-physics#692](https://github.com/gazebosim/gz-physics/issues/692)

* Constraint

  * Regularize rank-deficient closed-loop constraints with an additive Constraint Force Mixing floor on the `BallJointConstraint`/`WeldJointConstraint` loop-closure constraints, so non-axis-aligned closed kinematic chains stay enforced instead of drifting open: [#2903](https://github.com/dartsim/dart/pull/2903), [gazebosim/gz-physics#719](https://github.com/gazebosim/gz-physics/issues/719)

### [DART 6.17.1 (2026-06-05)](https://github.com/dartsim/dart/milestone/95?closed=1)

DART 6.17.1 is a patch release on the DART 6 LTS line. It hardens the classic
DART 6 API against non-finite and degenerate inputs surfaced through gz-physics,
so invalid SDF data degrades gracefully (with a warning) instead of aborting.

* Collision

  * Clamp the ODE heightfield extents, vertical scale, and bounds so an extreme `HeightmapShape` scale no longer aborts `dCollideHeightfield`: [#2894](https://github.com/dartsim/dart/pull/2894), [gazebosim/gz-physics#847](https://github.com/gazebosim/gz-physics/issues/847)

* Dynamics

  * Validate `HeightmapShape` scale and height-field values, rejecting non-finite and non-positive inputs: [#2884](https://github.com/dartsim/dart/pull/2884), [gazebosim/gz-physics#847](https://github.com/gazebosim/gz-physics/issues/847)
  * Reject non-finite mass and moment of inertia at the `Inertia` ingest boundary: [#2885](https://github.com/dartsim/dart/pull/2885), [gazebosim/gz-physics#854](https://github.com/gazebosim/gz-physics/issues/854)
  * Initialize `Inertia` to a valid state before applying a rejected moment, and check only the consumed inertia entries: [#2898](https://github.com/dartsim/dart/pull/2898)

* Common

  * Demote the duplicate-name auto-rename message to debug verbosity: [#2895](https://github.com/dartsim/dart/pull/2895), [gazebosim/gz-physics#725](https://github.com/gazebosim/gz-physics/issues/725)

* Build

  * Use pixi for the DART 6 Windows wheel dependencies: [#2882](https://github.com/dartsim/dart/pull/2882)

### [DART 6.17.0 (2026-06-03)](https://github.com/dartsim/dart/milestone/85?closed=1)

DART 6.17.0 is a maintenance release on the DART 6 LTS compatibility line. It
delivers bug fixes and additive improvements to the classic DART 6 API for
DART 6 and gz-physics users.

* GUI

  * Fix `RealTimeWorldNode` stalling when the viewer lags behind real time: [#2088](https://github.com/dartsim/dart/pull/2088)

* Collision

  * Preserve the minimum pair distance in FCL distance queries: [#2243](https://github.com/dartsim/dart/pull/2243), [#1539](https://github.com/dartsim/dart/issues/1539)
  * Fix plane collision handling for the Bullet and FCL backends: [#2246](https://github.com/dartsim/dart/pull/2246)
  * Fix an FCL mesh contact regression: [#2258](https://github.com/dartsim/dart/pull/2258)
  * Fix Bullet ellipsoid rolling by using primitive sphere/ellipsoid shapes: [#2274](https://github.com/dartsim/dart/pull/2274)
  * Handle non-`Skeleton` `MetaSkeleton` subscriptions in `CollisionGroup` without crashing: [#2277](https://github.com/dartsim/dart/pull/2277)
  * Stabilize FCL contact normals and make collision ordering deterministic: [#2282](https://github.com/dartsim/dart/pull/2282)
  * Guard the body-node collision filter against missing body nodes: [#2343](https://github.com/dartsim/dart/pull/2343)
  * Add `Contact` accessors for the shape frame, shape node, and body node: [#2245](https://github.com/dartsim/dart/pull/2245)
  * Fix memory leaks and a use-after-free in the Bullet collision backend: [#2101](https://github.com/dartsim/dart/pull/2101)
  * Fix a Bullet collision-detection correctness issue: [#2091](https://github.com/dartsim/dart/pull/2091)
  * Fix ODE `HeightmapShape` visual/collision mismatch and a dangling-pointer bug: [#2305](https://github.com/dartsim/dart/pull/2305)

* Constraint

  * Recover finite Dantzig LCP solutions instead of zeroing the result when only some entries are NaN: [#2253](https://github.com/dartsim/dart/pull/2253)

* Dynamics

  * Compute `BodyNode` potential energy from the center of mass: [#2224](https://github.com/dartsim/dart/pull/2224)
  * Add `BodyNode` world-transform derivative APIs: [#2131](https://github.com/dartsim/dart/pull/2131)
  * Preserve Collada unit scaling in `MeshShape`: [#2152](https://github.com/dartsim/dart/pull/2152)
  * Add debug assertions for non-finite joint inputs: [#2273](https://github.com/dartsim/dart/pull/2273)
  * Emit collision-shape added/removed signals on shape changes: [#2250](https://github.com/dartsim/dart/pull/2250)
  * Deep-copy shapes when cloning skeletons: [#2239](https://github.com/dartsim/dart/pull/2239), [#896](https://github.com/dartsim/dart/issues/896)
  * Add `PlanarJoint` SE(2) conversion helpers: [#2231](https://github.com/dartsim/dart/pull/2231)
  * Fix joint impulse statefulness by moving impulse state into `GenericJointState`: [#2308](https://github.com/dartsim/dart/pull/2308)
  * Add a state-independent `Joint::integratePositions` overload: [#2309](https://github.com/dartsim/dart/pull/2309)
  * Validate negative physics parameters at runtime instead of asserting: [#2431](https://github.com/dartsim/dart/pull/2431)
  * Clear stale joint commands when the actuator type changes: [#2098](https://github.com/dartsim/dart/pull/2098)
  * Fix the `FreeJoint` world Jacobian translation: [#2298](https://github.com/dartsim/dart/pull/2298)

* Parsers

  * Enforce SDF joint position limits when finite: [#2232](https://github.com/dartsim/dart/pull/2232)
  * Handle tiny/degenerate SDF inertial data and warn on missing `<inertial>`: [#2284](https://github.com/dartsim/dart/pull/2284)

* Common

  * Fix `Signal` thread-safety regressions: [#2181](https://github.com/dartsim/dart/pull/2181)
  * Add `StlAllocator::max_size` and make `deallocate` `noexcept`: [#2287](https://github.com/dartsim/dart/pull/2287)
  * Fix `Aspect` lifecycle when an aspect is replaced: [#2304](https://github.com/dartsim/dart/pull/2304)
  * Make `CloneableMap` copy null-safe: [#2358](https://github.com/dartsim/dart/pull/2358)

* Build

  * Prefer the prefix-local Vulkan loader for the ImGui example: [#2085](https://github.com/dartsim/dart/pull/2085)
  * Allow configuring (or disabling) the examples install destination: [#2100](https://github.com/dartsim/dart/pull/2100)
  * Register `dart::utils` parser sources correctly in CMake: [#2314](https://github.com/dartsim/dart/pull/2314)

### [DART 6.16.8 (2026-05-31)](https://github.com/dartsim/dart/milestone/93?closed=1)

* GUI

  * Fix SEGV in ImFontAtlas::AddFontFromMemoryCompressedTTF when null pointer is passed as compressed font data: [#2516](https://github.com/dartsim/dart/issues/2516)
  * Fix `ImGui::ColorPicker3`/`ColorPicker4` crashes when called without an active ImGui context or window: [#2668](https://github.com/dartsim/dart/issues/2668)
  * Use modern ImGui modifier key names in the OSG ImGui handler for newer ImGui builds. ([#2526](https://github.com/dartsim/dart/pull/2526))

* Constraint

  * Ignore malformed contacts with null collision objects instead of dereferencing them while creating `ContactConstraint`: [#2669](https://github.com/dartsim/dart/issues/2669)

  * Hide bundled ImGui internal formatting helpers from shared library exports: [#2671](https://github.com/dartsim/dart/issues/2671)

* Parsers

  * Return safe defaults from XML attribute helpers when callers pass a null element: [#2678](https://github.com/dartsim/dart/issues/2678)

* Utils

  * Reject mismatched C3D frame or marker counts in `saveC3DFile()` instead of indexing past point data: [#2681](https://github.com/dartsim/dart/issues/2681)

* Common

  * Fix iterator invalidation in Subject/Observer notification loops that caused non-deterministic SEGFAULT on macOS arm64 Debug builds

  * Fix spdlog/fmt 12 builds by treating DART logging format parameters as runtime format strings. ([#2540](https://github.com/dartsim/dart/pull/2540), [#2538](https://github.com/dartsim/dart/issues/2538))

  * Fix PoolAllocator leaking old memory block tables when the table grows. ([#2646](https://github.com/dartsim/dart/pull/2646))

* Simulation

  * Reject NaN, infinite, zero, and negative `World::setTimeStep()` values to prevent invalid timesteps from reaching the ODE LCP solver. ([#2532](https://github.com/dartsim/dart/pull/2532), [#2531](https://github.com/dartsim/dart/issues/2531))

* Tooling and Docs

  * Improve FreeBSD VM CI startup resilience with a release image URL, fallback image downloads, a longer SSH readiness window, fresh VM container startup, KVM group propagation, retry handling, and startup diagnostics. ([#2532](https://github.com/dartsim/dart/pull/2532))
  * Remove duplicate `.gitignore` entries. ([#2735](https://github.com/dartsim/dart/pull/2735))
  * Update Pixi lockfiles for current conda-forge package builds. ([#2766](https://github.com/dartsim/dart/pull/2766), [#2780](https://github.com/dartsim/dart/pull/2780))

### [DART 6.16.7 (2026-02-10)](https://github.com/dartsim/dart/milestone/92?closed=1)

* Build

  * Added support for assimp 6.x while maintaining backward compatibility with assimp 5.x: [#2510](https://github.com/dartsim/dart/pull/2510)

* Dynamics

  * Reject non-finite transforms at Joint public API entry points and guard inertia propagation in kinematic joint variants: [#2497](https://github.com/dartsim/dart/pull/2497)

* Math

  * Enhance verifyTransform to also reject infinity values (previously only checked NaN): [#2497](https://github.com/dartsim/dart/pull/2497)

* Optimizer

  * Validate Problem and MultiObjectiveProblem dimension to prevent excessive allocation from huge inputs: [#2503](https://github.com/dartsim/dart/pull/2503)

* Parsers

  * Replace MJCF parser DART_ASSERT with DART_WARN + identity fallback for non-finite transforms and rotations: [#2497](https://github.com/dartsim/dart/pull/2497)

### [DART 6.16.6 (2026-01-28)](https://github.com/dartsim/dart/milestone/91?closed=1)

* Constraint

  * Fix slip compliance validation to silently handle the -1.0 sentinel value (meaning "use default") instead of logging spurious warnings: [gz-sim#3289](https://github.com/gazebosim/gz-sim/issues/3289)

* Dynamics

  * Guard against non-finite articulated body computations from zero/epsilon mass or extreme spring values: [gz-physics#849](https://github.com/gazebosim/gz-physics/issues/849), [gz-physics#850](https://github.com/gazebosim/gz-physics/issues/850), [gz-physics#851](https://github.com/gazebosim/gz-physics/issues/851), [gz-physics#854](https://github.com/gazebosim/gz-physics/issues/854), [gz-physics#856](https://github.com/gazebosim/gz-physics/issues/856)

### [DART 6.16.5 (2026-01-21)](https://github.com/dartsim/dart/milestone/90?closed=1)

* Constraint

  * Fix crash when joint limits are invalid (lower > upper) by emitting a warning and skipping limit enforcement: [gz-physics#846](https://github.com/gazebosim/gz-physics/issues/846)

  * Validate contact surface parameters to prevent LCP solver crashes: [#2435](https://github.com/dartsim/dart/pull/2435)

  * Warn and continue when the boxed LCP matrix is non-symmetric to avoid assertion failures with invalid contacts: [gz-physics#848](https://github.com/gazebosim/gz-physics/issues/848)

* Dynamics

  * Validate SphereShape radius to prevent assertion failures with NaN/Inf/non-positive values: [#2441](https://github.com/dartsim/dart/pull/2441)

* Parsers
  * Fix null pointer dereference in XmlHelpers getValue* functions: [#2429](https://github.com/dartsim/dart/pull/2429)

### [DART 6.16.4 (2026-01-06)](https://github.com/dartsim/dart/milestone/89?closed=1)

* Physics
  * Fix ODE box-cylinder contact stability with libccd: [#2389](https://github.com/dartsim/dart/pull/2389)
  * Fix ODE cylinder mesh fallback on FreeBSD: [#2388](https://github.com/dartsim/dart/pull/2388)

* Parsers
  * Fix URDF shape parsing on FreeBSD: [#2379](https://github.com/dartsim/dart/pull/2379)

* Tooling and Docs
  * Simplify copyright headers to first publication year (REUSE compliance): [#2439](https://github.com/dartsim/dart/pull/2439)
  * Add Alt Linux repro tasks: [#2381](https://github.com/dartsim/dart/pull/2381)
  * Add FreeBSD VM repro patches: [#2374](https://github.com/dartsim/dart/pull/2374)
  * Update pixi lockfile: [#2377](https://github.com/dartsim/dart/pull/2377)
  * Fix cache Docker CI for release-6.16: [#2362](https://github.com/dartsim/dart/pull/2362)

### [DART 6.16.3 (2025-12-31)](https://github.com/dartsim/dart/milestone/88?closed=1)

* Collision
  * Fix Bullet double-precision includes on release-6.16: [#2334](https://github.com/dartsim/dart/pull/2334)

* Tooling and Docs
  * Fix pixi ex example selection: [f716548](https://github.com/dartsim/dart/commit/f7165480d66)

### [DART 6.16.2 (2025-12-20)](https://github.com/dartsim/dart/milestone/87?closed=1)

* Build
  * Remove DART_BUILD_MODE_DEBUG guards to fix NDEBUG compilation: [#2326](https://github.com/dartsim/dart/pull/2326)
  * Use package.xml version for dartpy packaging: [#2327](https://github.com/dartsim/dart/pull/2327)

* Tests
  * Stabilize World::Cloning in asserts-enabled CI: [#2331](https://github.com/dartsim/dart/pull/2331)

* Tooling and Docs
  * Cover asserts-enabled build and fix API docs PR ref: [#2330](https://github.com/dartsim/dart/pull/2330)
  * Fix publish_dartpy workflow YAML: [#2329](https://github.com/dartsim/dart/pull/2329)
  * Fix manual publish validation on Windows: [#2324](https://github.com/dartsim/dart/pull/2324)
  * Restrict manual dartpy publishing to tags: [#2323](https://github.com/dartsim/dart/pull/2323)
  * Fix dartpy PyPI publishing on release-6.16: [#2322](https://github.com/dartsim/dart/pull/2322)
  * Update pixi lockfile: [#2335](https://github.com/dartsim/dart/pull/2335), [#2307](https://github.com/dartsim/dart/pull/2307)

### [DART 6.16.1 (2025-12-12)](https://github.com/dartsim/dart/milestone/86?closed=1)

* Build
  * Fix FreeListAllocator assertions when building without NDEBUG (e.g., Ubuntu 24.04 GCC 13): [#2295](https://github.com/dartsim/dart/pull/2295)

### [DART 6.16.0 (2025-11-09)](https://github.com/dartsim/dart/milestone/83?closed=1)

* Tested Platforms

  * Linux
    * Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    * Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  * macOS 14 / Clang 15 / arm64
  * Windows / MSVC 19.40 / x86_64

* Simulation
  * Allow servo joints to recover from position limits: [#2086](https://github.com/dartsim/dart/pull/2086)
  * Fix passive joint commands to respect joint actuation limits: [#1997](https://github.com/dartsim/dart/pull/1997)

* Core
  * Replace legacy `assert` macros with `DART_ASSERT` on release-6.16 (backport of #2109): [#2117](https://github.com/dartsim/dart/pull/2117)

* Build
  * Fix pybind11 detection and assert include handling with pixi builds: [#2118](https://github.com/dartsim/dart/pull/2118)
  * Port Eigen compatibility guard to keep 3.4+ builds working on release-6.16: [#2108](https://github.com/dartsim/dart/pull/2108)
  * Add `DART_USE_SYSTEM_TRACY`, `DART_USE_SYSTEM_PYBIND11`, and `DART_USE_SYSTEM_GOOGLEBENCHMARK` toggles: [#1911](https://github.com/dartsim/dart/pull/1911), [#1907](https://github.com/dartsim/dart/pull/1907), [#1904](https://github.com/dartsim/dart/pull/1904)
  * Fix absolute install directory handling in CMake exports: [#2006](https://github.com/dartsim/dart/pull/2006)

* Tooling and Docs
  * Use system `googletest` and `googlebenchmark` in pixi environments and upgrade bundled dependencies: [#1905](https://github.com/dartsim/dart/pull/1905)
  * Switch coverage and API docs workflows to pixi tasks and GitHub Pages deploy actions: [#2036](https://github.com/dartsim/dart/pull/2036), [#2032](https://github.com/dartsim/dart/pull/2032)
  * Install OpenSceneGraph from source on macOS builds to avoid flaky CI: [#2037](https://github.com/dartsim/dart/pull/2037)
  * Restructure documentation to clearly separate C++ and Python guidance: [#2040](https://github.com/dartsim/dart/pull/2040)
  * Add pixi-powered developer tasks (including Python workflows) and refresh GitHub templates: [#2034](https://github.com/dartsim/dart/pull/2034), [#2039](https://github.com/dartsim/dart/pull/2039)
  * Add gz-physics integration tests to CI to guard the public plugin: [#2000](https://github.com/dartsim/dart/pull/2000)

### [DART 6.15.0 (2024-11-15)](https://github.com/dartsim/dart/milestone/77?closed=1)

* Tested Platforms

  * Linux
    * Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    * Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  * macOS 14 / Clang 15 / arm64
  * Windows / MSVC 19.40 / x86_64

* Build
  * Added ImGui 1.91.5 support: [#1872](https://github.com/dartsim/dart/pull/1872)
  * Added nlopt 2.9.0 support: [#1875](https://github.com/dartsim/dart/pull/1875)
  * Fixed imgui is not added as transitive dependency: [#1877](https://github.com/dartsim/dart/pull/1877)

### [DART 6.14.5 (2024-09-08)](https://github.com/dartsim/dart/milestone/82?closed=1)

* Tested Platforms

  * Linux
    * Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    * Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  * macOS 14 / Clang 15 / arm64
  * Windows / MSVC 19.40 / x86_64

* Fixed missing parentheses in config.hpp: [#1838](https://github.com/dartsim/dart/pull/1838)
* Allowed negative scale for MeshShape: [#1841](https://github.com/dartsim/dart/pull/1841)

### [DART 6.14.4 (2024-07-06)](https://github.com/dartsim/dart/milestone/81?closed=1)

* Tested Platforms

  * Linux
    * Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    * Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  * macOS 14 / Clang 15 / arm64
  * Windows / MSVC 19.40 / x86_64

* Fixed GLUT dependency handling on Windows: [#1827](https://github.com/dartsim/dart/pull/1827)

### [DART 6.14.3 (2024-07-05)](https://github.com/dartsim/dart/milestone/80?closed=1)

* Tested Platforms

  * Linux
    * Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    * Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  * macOS 14 / Clang 15 / arm64
  * Windows / MSVC 19.40 / x86_64

* Changed the default CMake option to DART_ENABLE_SIMD=OFF: [#1825](https://github.com/dartsim/dart/pull/1825)

### [DART 6.14.2 (2024-06-28)](https://github.com/dartsim/dart/milestone/79?closed=1)

* Tested Platforms

  * Linux
    * Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    * Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  * macOS 14 / Clang 15 / arm64
  * Windows / MSVC 19.40 / x86_64

* Fixed version definitions: [#1820](https://github.com/dartsim/dart/pull/1820)

### [DART 6.14.2 (2024-06-26)](https://github.com/dartsim/dart/milestone/78?closed=1)

* Tested Platforms

  * Linux
    * Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    * Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  * macOS 14 / Clang 15 / arm64
  * Windows / MSVC 19.40 / x86_64

* Included CTest for BUILD_TESTING option: [#1819](https://github.com/dartsim/dart/pull/1819)

### [DART 6.14.0 (2024-06-24)](https://github.com/dartsim/dart/milestone/73?closed=1)

This release is mostly a maintenance update, including various CI updates and build fixes for recent development environment toolsets and dependencies.

* Tested Platforms

  * Linux
    * Ubuntu 22.04 LTS / GCC 11.4 / x86_64
    * Ubuntu 24.04 LTS / GCC 13.2 / x86_64
  * macOS 14 / Clang 15 / arm64
  * Windows / MSVC 19.40 / x86_64

* Breaking Changes

  * Removed planning component

* Build
  
  * Added Pixi support

* General

  * Added profile interface with Tracy backend support
  * Added benchmark setup, including boxes and kinematics benchmarks

* Dynamics

  * Allowed specifying mimic joint properties per DoF: [#1752](https://github.com/dartsim/dart/pull/1752)
  * [Improved performance in constructing LCP problem](https://github.com/dartsim/dart/commit/76d8fe1a72f6925c06f64eea3d2cd135234b59de)

### [DART 6.13.2 (2024-03-17)](https://github.com/dartsim/dart/milestone/75?closed=1)

* Tested Platforms

  * Linux
    * Ubuntu 22.04 LTS on amd64 / GCC 11.2 / amd64
    * Ubuntu 24.04 LTS on amd64 / GCC 13.2 / amd64
  * macOS 12 (Monterey) / AppleClang 14 / amd64
  * Windows / MSVC 19.38 / amd64

* Build

  * Fixed build with GCC >= 13: [#1793](https://github.com/dartsim/dart/pull/1793)

### [DART 6.13.1 (2024-01-04)](https://github.com/dartsim/dart/milestone/74?closed=1)

* Tested Platforms

  * Ubuntu Focal on amd64 / GCC 9.4 / amd64
  * Ubuntu Jammy on amd64 / GCC 11.3 / amd64
  * macOS 12 (Monterey) / Clang 14 / amd64
  * Windows / MSVC 19.37 / amd64

* Build

  * Fixed build with urdfdom 4.0.0: [#1779](https://github.com/dartsim/dart/pull/1779)
  * Fixed invalid array access in moving skeleton subtree: [#1778](https://github.com/dartsim/dart/pull/1778)

* Dynamics

  * Fixed joint not recovering after reaching position limits in servo mode: [#1774](https://github.com/dartsim/dart/pull/1774)

### [DART 6.13.0 (2022-12-31)](https://github.com/dartsim/dart/milestone/69?closed=1)

* Supported Platforms

  * Ubuntu Focal on amd64 / GCC 9.3 / amd64
  * Ubuntu Jammy on amd64 / GCC 11.2 / amd64
  * macOS 12 (Monterey) / Clang 13 / amd64
  * Windows / Microsoft Visual Studio 2019 / amd64

* Dependency

  * Added required dependencies: fmt
  * Added optional dependencies: spdlog
  * Removed required dependencies: Boost

* Build

  * Dropped supporting FCL < 0.5: [#1647](https://github.com/dartsim/dart/pull/1647)

* Common

  * Added Castable class: [#1634](https://github.com/dartsim/dart/pull/1634)
  * Added spdlog support as underlying logging framework: [#1633](https://github.com/dartsim/dart/pull/1633)
  * Added custom memory allocators: [#1636](https://github.com/dartsim/dart/pull/1636), [#1637](https://github.com/dartsim/dart/pull/1637), [#1639](https://github.com/dartsim/dart/pull/1639), [#1645](https://github.com/dartsim/dart/pull/1645), [#1646](https://github.com/dartsim/dart/pull/1646)
  * Added Stopwatch class to replace Timer: [#1638](https://github.com/dartsim/dart/pull/1638)
  * Removed Boost dependency: [#1648](https://github.com/dartsim/dart/pull/1648), [#1651](https://github.com/dartsim/dart/pull/1651)

* Collision Detection

  * Updated to use convex mesh of Bullet when possible: [#1664](https://github.com/dartsim/dart/pull/1664), [#1667](https://github.com/dartsim/dart/pull/1667)

* Dynamics

  * Fixed setResitutionCoeff() calling setFrictionCoeff(): [#1677](https://github.com/dartsim/dart/pull/1677)
  * Made inertial warnings optional when setting tensor: [#1672](https://github.com/dartsim/dart/pull/1672)
  * Added deep copy for shapes: [#1612](https://github.com/dartsim/dart/pull/1612)
  * Added iterator methods to container-like classes: [#1644](https://github.com/dartsim/dart/pull/1644)
  * Fixed grouping of constraints: [#1624](https://github.com/dartsim/dart/pull/1624), [#1628](https://github.com/dartsim/dart/pull/1628)
  * Fixed issue with removing skeletons without shapes: [#1625](https://github.com/dartsim/dart/pull/1625)
  * Added support for custom contact surface handlers: [#1626](https://github.com/dartsim/dart/pull/1626)

* GUI

  * Fixed depth testing for transparent objects: [#1643](https://github.com/dartsim/dart/pull/1643)
  * Added depth rendering mode: [#1652](https://github.com/dartsim/dart/pull/1652)

### [DART 6.12.2 (2022-07-31)](https://github.com/dartsim/dart/milestone/72?closed=1)

* Build

  * Fixed build with urdfdom 3.1.0 on Windows: [#1675](https://github.com/dartsim/dart/pull/1675)

### [DART 6.12.1 (2021-11-04)](https://github.com/dartsim/dart/milestone/71?closed=1)

* Build

  * Fixed bullet header include: [#1620](https://github.com/dartsim/dart/pull/1620)

* dartpy

  * Added Python bindings for Joint::getWrenchTo{Child|Parent}BodyNode: [#1621](https://github.com/dartsim/dart/pull/1621)

### [DART 6.12.0 (2021-11-01)](https://github.com/dartsim/dart/milestone/66?closed=1)

* API Breaking Changes

  * DART 6.12.0 and later require compilers that support C++17: [#1600](https://github.com/dartsim/dart/pull/1600)
    * Increased minimum CMake version to 3.10.2
    * Increased minimum compiler versions to GCC 7.3.0, Clang 6.0, MSVC 16.0
    * Dropped Ubuntu Xenial (16.04 LTS) support

* Build

  * Remove DART_BUILD_DARTPY option: [#1600](https://github.com/dartsim/dart/pull/1600)

* Dynamics

  * Added joint force/torque getter: [#1616](https://github.com/dartsim/dart/pull/1616)

* Parsers

  * Added default options to DartLoader for missing properties in URDF files: [#1605](https://github.com/dartsim/dart/pull/1605)
  * Allowed SdfParser to set default root joint type: [#1617](https://github.com/dartsim/dart/pull/1617)

* GUI

  * Updated ImGui to 1.84.2: [#1608](https://github.com/dartsim/dart/pull/1608)

* dartpy

  * Added Python bindings for ResourceRetriever and SdfParser: [#1610](https://github.com/dartsim/dart/pull/1610)

### [DART 6.11.2 (2021-10-29)](https://github.com/dartsim/dart/milestone/68?closed=1)

* dartpy

  * Added Python binding for global lighting mode setting: [#1615](https://github.com/dartsim/dart/pull/1615)

### [DART 6.11.1 (2021-08-23)](https://github.com/dartsim/dart/milestone/67?closed=1)

* Dynamics

  * Fixed incorrect LCP construction in JointConstraint for multi-DOFs joints: [#1597](https://github.com/dartsim/dart/pull/1597)

### [DART 6.11.0 (2021-07-15)](https://github.com/dartsim/dart/milestone/64?closed=1)

* Math

  * Added `Mesh`, `TriMesh`, and `Icosphere` classes: [#1579](https://github.com/dartsim/dart/pull/1579)

* Collision

  * Fixed incorrect group-group collision checking for BulletCollisionDetector: [#1585](https://github.com/dartsim/dart/pull/1585), [#717](https://github.com/dartsim/dart/issues/717)

* Dynamics

  * Fixed servo motor doesn't respect joint position limits: [#1587](https://github.com/dartsim/dart/pull/1587)

* GUI

  * Fixed incorrect MultiSphereConvexHull rendering: [#1579](https://github.com/dartsim/dart/pull/1579)
  * Use GLVND over the legacy OpenGL libraries: [#1584](https://github.com/dartsim/dart/pull/1584)

* Build and testing

  * Add DART_ prefix to macros to avoid potential conflicts: [#1586](https://github.com/dartsim/dart/pull/1586)

### [DART 6.10.1 (2021-04-19)](https://github.com/dartsim/dart/milestone/65?closed=1)

* Dynamics

  * Fixed inertia calculation of CapsuleShape: [#1561](https://github.com/dartsim/dart/pull/1561)

* GUI

  * Changed to protect OpenGL attributes shared by ImGui and OSG: [#1558](https://github.com/dartsim/dart/pull/1558)
  * Changed to set backface culling by default: [#1559](https://github.com/dartsim/dart/pull/1559)

* dartpy

  * Added Python binding for BodyNode::getBodyForce(): [#1563](https://github.com/dartsim/dart/pull/1563)

### [DART 6.10.0 (2021-04-09)](https://github.com/dartsim/dart/milestone/58?closed=1)

* Common

  * Removed use of boost::filesystem in public APIs: [#1417](https://github.com/dartsim/dart/pull/1417)
  * Changed Signal to remove connection when they're being disconnected: [#1462](https://github.com/dartsim/dart/pull/1462)

* Collision

  * Added ConeShape support for FCLCollisionDetector: [#1447](https://github.com/dartsim/dart/pull/1447)
  * Fixed segfault from raycast when no ray hit: [#1461](https://github.com/dartsim/dart/pull/1461)
  * Added PyramidShape class: [#1466](https://github.com/dartsim/dart/pull/1466)

* Kinematics

  * Added IkFast parameter accessors to IkFast class: [#1396](https://github.com/dartsim/dart/pull/1396)
  * Changed IkFast to wrap IK solutions into the joint limits for RevoluteJoint: [#1452](https://github.com/dartsim/dart/pull/1452)
  * Added option to specify reference frame of TaskSpaceRegion: [#1548](https://github.com/dartsim/dart/pull/1548)

* Dynamics

  * Fixed friction and restitution of individual shapes in a body: [#1369](https://github.com/dartsim/dart/pull/1369)
  * Fixed soft body simulation when command input is not reset: [#1372](https://github.com/dartsim/dart/pull/1372)
  * Added joint velocity limit constraint support: [#1407](https://github.com/dartsim/dart/pull/1407)
  * Added type property to constrain classes: [#1415](https://github.com/dartsim/dart/pull/1415)
  * Allowed to set joint rest position out of joint limits: [#1418](https://github.com/dartsim/dart/pull/1418)
  * Added secondary friction coefficient parameter: [#1424](https://github.com/dartsim/dart/pull/1424)
  * Allowed to set friction direction per ShapeFrame: [#1427](https://github.com/dartsim/dart/pull/1427)
  * Fixed incorrect vector resizing in BoxedLcpConstraintSolver: [#1459](https://github.com/dartsim/dart/pull/1459)
  * Changed to increment BodyNode version when it's being removed from Skeleton: [#1489](https://github.com/dartsim/dart/pull/1489)
  * Changed to print warning only once from BulletCollisionDetector::collide/distance: [#1546](https://github.com/dartsim/dart/pull/1546)
  * Added force dependent slip: [#1505](https://github.com/dartsim/dart/pull/1505)

* GUI

  * Fixed memory leaks from dart::gui::osg::Viewer: [#1349](https://github.com/dartsim/dart/pull/1349)
  * Added point rendering mode to PointCloudShape: [#1351](https://github.com/dartsim/dart/pull/1351), [#1355](https://github.com/dartsim/dart/pull/1355)
  * Updated ImGui to 1.71: [#1362](https://github.com/dartsim/dart/pull/1362)
  * Updated ImGui to 1.79: [#1498](https://github.com/dartsim/dart/pull/1498)
  * Fixed refresh of LineSegmentShapeNode: [#1381](https://github.com/dartsim/dart/pull/1381)
  * Fixed OSG transparent object sorting: [#1414](https://github.com/dartsim/dart/pull/1414)
  * Added modifier key support to ImGuiHandler: [#1436](https://github.com/dartsim/dart/pull/1436)
  * Fixed mixed intrinsic and extrinsic camera parameters in OpenGL projection matrix: [#1485](https://github.com/dartsim/dart/pull/1485)
  * Enabled mouse middle and right buttons in ImGuiHandler: [#1500](https://github.com/dartsim/dart/pull/1500)

* Parser

  * Allowed parsing SDF up to version 1.6: [#1385](https://github.com/dartsim/dart/pull/1385)
  * Fixed SDF parser not creating dynamics aspect for collision shape: [#1386](https://github.com/dartsim/dart/pull/1386)
  * Added root joint parsing option in URDF parser: [#1399](https://github.com/dartsim/dart/pull/1399), [#1406](https://github.com/dartsim/dart/pull/1406)
  * Enabled URDF parser to read visual and collision names: [#1410](https://github.com/dartsim/dart/pull/1410)
  * Added (experimental) MJCF parser: [#1416](https://github.com/dartsim/dart/pull/1416)

* dartpy

  * Added raycast option and result: [#1343](https://github.com/dartsim/dart/pull/1343)
  * Added GUI event handler: [#1346](https://github.com/dartsim/dart/pull/1346)
  * Added shadow technique: [#1348](https://github.com/dartsim/dart/pull/1348)
  * Added findSolution and solveAndApply to InverseKinematics: [#1358](https://github.com/dartsim/dart/pull/1358)
  * Added InteractiveFrame and ImGui APIs: [#1359](https://github.com/dartsim/dart/pull/1359)
  * Added bindings for Joint::getTransformFrom{Parent|Child}BodyNode(): [#1377](https://github.com/dartsim/dart/pull/1377)
  * Added bindings for BodyNode::getChild{BodyNode|Joint}(): [#1387](https://github.com/dartsim/dart/pull/1387)
  * Added bindings for Inertia: [#1388](https://github.com/dartsim/dart/pull/1388)
  * Added bindings for getting all BodyNodes from a Skeleton: [#1397](https://github.com/dartsim/dart/pull/1397)
  * Added bindings for background color support in osg viewer: [#1398](https://github.com/dartsim/dart/pull/1398)
  * Added bindings for BallJoint::convertToPositions(): [#1408](https://github.com/dartsim/dart/pull/1408)
  * Fixed typos in Skeleton: [#1392](https://github.com/dartsim/dart/pull/1392)
  * Fixed enabling drag and drop for InteractiveFrame: [#1432](https://github.com/dartsim/dart/pull/1432)
  * Added bindings for pointcloud and contact retrieval: [#1455](https://github.com/dartsim/dart/pull/1455)
  * Fixed TypeError from dartpy.dynamics.Node.getBodyNodePtr(): [#1463](https://github.com/dartsim/dart/pull/1463)
  * Added pybind/eigen.h to DistanceResult.cpp for read/write of eigen types: [#1480](https://github.com/dartsim/dart/pull/1480)
  * Added bindings for adding ShapeFrames to CollisionGroup: [#1490](https://github.com/dartsim/dart/pull/1490)
  * Changed dartpy install command to make install-dartpy: [#1503](https://github.com/dartsim/dart/pull/1503)
  * Added bindings for CollisionFilter, CollisionGroup, and Node: [#1545](https://github.com/dartsim/dart/pull/1545)
  * Added bindings for TaskSpaceRegion: [#1550](https://github.com/dartsim/dart/pull/1550)

* Build and testing

  * Fixed compiler warnings from GCC 9.1: [#1366](https://github.com/dartsim/dart/pull/1366)
  * Replaced M_PI with dart::math::constantsd::pi(): [#1367](https://github.com/dartsim/dart/pull/1367)
  * Enabled octomap support on macOS: [#1078](https://github.com/dartsim/dart/pull/1078)
  * Removed dependency on Boost::regex: [#1412](https://github.com/dartsim/dart/pull/1412)
  * Added support new if() IN_LIST operator in DARTConfig.cmake: [#1434](https://github.com/dartsim/dart/pull/1434)
  * Updated Findfcl.cmake to support FCL 0.6: [#1441](https://github.com/dartsim/dart/pull/1441)
  * Added gtest macros for Eigen object comparisons: [#1443](https://github.com/dartsim/dart/pull/1443)
  * Removed gccfilter: [#1464](https://github.com/dartsim/dart/pull/1464)
  * Allowed to set CMAKE_INSTALL_PREFIX on Windows: [#1478](https://github.com/dartsim/dart/pull/1478)
  * Enforced to use OpenSceneGraph 3.7.0 or greater on macOS Catalina: [#1479](https://github.com/dartsim/dart/pull/1479)
  * Fixed compatibility with clang-cl: [#1509](https://github.com/dartsim/dart/pull/1509)
  * Fixed MSVC linking for assimp and fcl: [#1510](https://github.com/dartsim/dart/pull/1510)
  * Fixed AspectWithState-relate compile error on Windows: [#1528](https://github.com/dartsim/dart/pull/1528)
  * Made `dart.pc` relocatable: [#1529](https://github.com/dartsim/dart/pull/1529)
  * Added CI for multiple Linux platforms: arm64 and ppc64le: [#1531](https://github.com/dartsim/dart/pull/1531)
  * Fixed Aspect/Composite-relate tests on Windows/MSVC: [#1541](https://github.com/dartsim/dart/pull/1541), [#1542](https://github.com/dartsim/dart/pull/1542)
  * Added `DART_SKIP_<dep>`_advanced option: [#1529](https://github.com/dartsim/dart/pull/1529)
  * Replaced OpenGL dependency variables with targets: [#1552](https://github.com/dartsim/dart/pull/1552)

* Documentation

  * Updated tutorial documentation and code to reflect new APIs: [#1481](https://github.com/dartsim/dart/pull/1481)

### [DART 6.9.5 (2020-10-17)](https://github.com/dartsim/dart/milestone/63?closed=1)

* Optimization

  * Added Ipopt >= 3.13 support: [#1506](https://github.com/dartsim/dart/pull/1506)

### [DART 6.9.4 (2020-08-30)](https://github.com/dartsim/dart/milestone/62?closed=1)

* Build

  * Added support new if() IN_LIST operator in DARTConfig.cmake (6.9 backport): [#1494](https://github.com/dartsim/dart/pull/1494)

### [DART 6.9.3 (2020-08-26)](https://github.com/dartsim/dart/milestone/61?closed=1)

* Dynamics

  * Changed to update the Properties version of a BodyNode when moved to a new Skeleton: [#1445](https://github.com/dartsim/dart/pull/1445)
  * Fixed incorrect implicit joint damping/spring force computation in inverse dynamics: [#1451](https://github.com/dartsim/dart/pull/1451)

### [DART 6.9.2 (2019-08-16)](https://github.com/dartsim/dart/milestone/60?closed=1)

* Dynamics

  * Allowed constraint force mixing > 1: [#1371](https://github.com/dartsim/dart/pull/1371)

### [DART 6.9.1 (2019-06-06)](https://github.com/dartsim/dart/milestone/59?closed=1)

* Collision

  * Added default constructor to RayHit: [#1345](https://github.com/dartsim/dart/pull/1345)

* dartpy

  * Updated build scripts for uploading dartpy to PyPI: [#1341](https://github.com/dartsim/dart/pull/1341)

### [DART 6.9.0 (2019-05-26)](https://github.com/dartsim/dart/milestone/52?closed=1)

* API Breaking Changes

  * DART 6.9.0 and later require compilers that support C++14.

* Common

  * Deprecated custom make_unique in favor of std::make_unique: [#1317](https://github.com/dartsim/dart/pull/1317)

* Collision Detection

  * Added raycast query to BulletCollisionDetector: [#1309](https://github.com/dartsim/dart/pull/1309)

* Dynamics

  * Added safeguard for accessing Assimp color: [#1313](https://github.com/dartsim/dart/pull/1313)

* Parser

  * Changed URDF parser to use URDF material color when specified: [#1295](https://github.com/dartsim/dart/pull/1295)

* GUI

  * Added heightmap support to OSG renderer: [#1293](https://github.com/dartsim/dart/pull/1293)
  * Improved voxel grid and point cloud rendering performance: [#1294](https://github.com/dartsim/dart/pull/1294)
  * Fixed incorrect alpha value update of InteractiveFrame: [#1297](https://github.com/dartsim/dart/pull/1297)
  * Fixed dereferencing a dangling pointer in WorldNode: [#1311](https://github.com/dartsim/dart/pull/1311)
  * Removed warning of ImGuiViewer + OSG shadow: [#1312](https://github.com/dartsim/dart/pull/1312)
  * Added shape type and color options to PointCloudShape: [#1314](https://github.com/dartsim/dart/pull/1314), [#1316](https://github.com/dartsim/dart/pull/1316)
  * Fixed incorrect transparency of MeshShape for MATERIAL_COLOR mode: [#1315](https://github.com/dartsim/dart/pull/1315)
  * Fixed incorrect PointCloudShape transparency: [#1330](https://github.com/dartsim/dart/pull/1330)
  * Improved voxel rendering by using multiple nodes instead of CompositeShape: [#1334](https://github.com/dartsim/dart/pull/1334)

* Examples and Tutorials

  * Updated examples directory to make dart::gui::osg more accessible: [#1305](https://github.com/dartsim/dart/pull/1305)

* dartpy

  * Switched to pybind11: [#1307](https://github.com/dartsim/dart/pull/1307)
  * Added grid visual: [#1318](https://github.com/dartsim/dart/pull/1318)
  * Added ReferentialSkeleton, Linkage, and Chain: [#1321](https://github.com/dartsim/dart/pull/1321)
  * Enabled WorldNode classes to overload virtual functions in Python: [#1322](https://github.com/dartsim/dart/pull/1322)
  * Added JacobianNode and operational space controller example: [#1323](https://github.com/dartsim/dart/pull/1323)
  * Removed static create() functions in favor of custom constructors: [#1324](https://github.com/dartsim/dart/pull/1324)
  * Added optimizer APIs with GradientDescentSolver and NloptSolver: [#1325](https://github.com/dartsim/dart/pull/1325)
  * Added SimpleFrame: [#1326](https://github.com/dartsim/dart/pull/1326)
  * Added basic inverse kinematics APIs: [#1327](https://github.com/dartsim/dart/pull/1327)
  * Added shapes and ShapeFrame aspects: [#1328](https://github.com/dartsim/dart/pull/1328)
  * Added collision APIs: [#1329](https://github.com/dartsim/dart/pull/1329)
  * Added DegreeOfFreedom and ShapeNode: [#1332](https://github.com/dartsim/dart/pull/1332)
  * Added constraint APIs: [#1333](https://github.com/dartsim/dart/pull/1333)
  * Added BallJoint, RevoluteJoint, joint properties, and chain tutorial (incomplete): [#1335](https://github.com/dartsim/dart/pull/1335)
  * Added all the joints: [#1337](https://github.com/dartsim/dart/pull/1337)
  * Added DART, Bullet, Ode collision detectors: [#1339](https://github.com/dartsim/dart/pull/1339)

### [DART 6.8.5 (2019-05-26)](https://github.com/dartsim/dart/milestone/57?closed=1)

* Collision

  * Fixed handling of submeshes in ODE collision detector: [#1336](https://github.com/dartsim/dart/pull/1336)

#### Compilers Tested

* Linux

  * GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  * GCC 32-bit: 5.4.0

* macOS

  * AppleClang: 9.1.0, 10.0.0

### [DART 6.8.4 (2019-05-03)](https://github.com/dartsim/dart/milestone/56?closed=1)

#### Changes

* GUI

  * Fixed crashing on exiting OSG + ImGui applications: [#1303](https://github.com/dartsim/dart/pull/1303)

#### Compilers Tested

* Linux

  * GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  * GCC 32-bit: 5.4.0

* macOS

  * AppleClang: 9.1.0, 10.0.0

### [DART 6.8.3 (2019-05-01)](https://github.com/dartsim/dart/milestone/55?closed=1)

#### Changes

* Parser

  * Fixed VskParker returning incorrect resource retriever: [#1300](https://github.com/dartsim/dart/pull/1300)

* Build

  * Fixed building with pagmo's optional dependencies: [#1301](https://github.com/dartsim/dart/pull/1301)

#### Compilers Tested

* Linux

  * GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  * GCC 32-bit: 5.4.0

* macOS

  * AppleClang: 9.1.0, 10.0.0

### [DART 6.8.2 (2019-04-23)](https://github.com/dartsim/dart/milestone/54?closed=1)

#### Changes

* Dynamics

  * Fixed BoxedLcpConstraintSolver is not API compatible with 6.7: [#1291](https://github.com/dartsim/dart/pull/1291)

* Build

  * Fixed building with FCL built without Octomap: [#1292](https://github.com/dartsim/dart/pull/1292)

#### Compilers Tested

* Linux

  * GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  * GCC 32-bit: 5.4.0

* macOS

  * AppleClang: 9.1.0, 10.0.0

### [DART 6.8.1 (2019-04-23)](https://github.com/dartsim/dart/milestone/53?closed=1)

#### Changes

* Build System

  * Fixed invalid double quotation marks in DARTFindBoost.cmake: [#1283](https://github.com/dartsim/dart/pull/1283)
  * Disabled octomap support on macOS: [#1284](https://github.com/dartsim/dart/pull/1284)

#### Compilers Tested

* Linux

  * GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  * GCC 32-bit: 5.4.0

* macOS

  * AppleClang: 9.1.0, 10.0.0

### [DART 6.8.0 (2019-04-22)](https://github.com/dartsim/dart/milestone/48?closed=1)

#### Changes

* Kinematics

  * Added findSolution() and solveAndApply() to InverseKinematics and HierarchicalIk classes and deprecated solve(~) member functions: [#1266](https://github.com/dartsim/dart/pull/1266)
  * Added an utility constructor to Linkage::Criteria to create sequence Linkage: [#1273](https://github.com/dartsim/dart/pull/1273)

* Dynamics

  * Fixed incorrect transpose check in Inertia::verifySpatialTensor(): [#1258](https://github.com/dartsim/dart/pull/1258)
  * Allowed BoxedLcpConstraintSolver to have a secondary LCP solver: [#1265](https://github.com/dartsim/dart/pull/1265)

* Simulation

  * The LCP solver will be less aggressive about printing out unnecessary warnings: [#1238](https://github.com/dartsim/dart/pull/1238)
  * Fixed not copying constraints in World::setConstraintSolver(): [#1260](https://github.com/dartsim/dart/pull/1260)

* Collision Detection

  * The BodyNodeCollisionFilter will ignore contacts between immobile bodies: [#1232](https://github.com/dartsim/dart/pull/1232)

* Planning

  * Fixed linking error of FLANN by explicitly linking to lz4: [#1221](https://github.com/dartsim/dart/pull/1221)

* Python

  * Added (experimental) Python binding: [#1237](https://github.com/dartsim/dart/pull/1237)

* Parsers

  * Changed urdf parser to warn if robot model has multi-tree: [#1270](https://github.com/dartsim/dart/pull/1270)

* GUI

  * Updated ImGui to 1.69: [#1274](https://github.com/dartsim/dart/pull/1274)
  * Added VoxelGridShape support to OSG renderer: [#1276](https://github.com/dartsim/dart/pull/1276)
  * Added PointCloudShape and its OSG rendering: [#1277](https://github.com/dartsim/dart/pull/1277)
  * Added grid visual to OSG renderer: [#1278](https://github.com/dartsim/dart/pull/1278), [#1280](https://github.com/dartsim/dart/pull/1280)

* Build System

  * Changed to use GNUInstallDirs for install paths: [#1241](https://github.com/dartsim/dart/pull/1241)
  * Fixed not failing for missing required dependencies: [#1250](https://github.com/dartsim/dart/pull/1250)
  * Fixed attempting to link octomap when not imported: [#1253](https://github.com/dartsim/dart/pull/1253)
  * Fixed not defining boost targets: [#1254](https://github.com/dartsim/dart/pull/1254)

#### Compilers Tested

* Linux

  * GCC 64-bit: 5.4.0, 7.3.0, 8.2.0
  * GCC 32-bit: 5.4.0

* macOS

  * AppleClang: 9.1.0, 10.0.0

### [DART 6.7.3 (2019-02-19)](https://github.com/dartsim/dart/milestone/51?closed=1)

#### Changes

* Dynamics

  * Fixed Skeleton::setState(): [#1245](https://github.com/dartsim/dart/pull/1245)

#### Compilers Tested

* Linux

  * GCC (C++11): 5.4.0, 7.3.0, 8.2.0

* Linux (32-bit)

  * GCC (C++11): 5.4.0

* macOS

  * AppleClang (C++11): 9.1.0

### [DART 6.7.2 (2019-01-17)](https://github.com/dartsim/dart/milestone/50?closed=1)

#### Changes

* Build system

  * Fixed #1223 for the recursive case: [#1227](https://github.com/dartsim/dart/pull/1227)
  * Specified mode for find_package(): [#1228](https://github.com/dartsim/dart/pull/1228)

#### Compilers Tested

* Linux

  * GCC (C++11): 5.4.0, 7.3.0, 8.2.0

* Linux (32-bit)

  * GCC (C++11): 5.4.0

* macOS

  * AppleClang (C++11): 9.1.0

### [DART 6.7.1 (2019-01-15)](https://github.com/dartsim/dart/milestone/49?closed=1)

#### Changes

* Build system

  * Ensure that imported targets of dependencies are always created when finding the dart package: [#1222](https://github.com/dartsim/dart/pull/1222)
  * Set components to not-found when their external dependencies are missing: [#1223](https://github.com/dartsim/dart/pull/1223)

#### Compilers Tested

* Linux

  * GCC (C++11): 5.4.0, 7.3.0, 8.2.0

* Linux (32-bit)

  * GCC (C++11): 5.4.0

* macOS

  * AppleClang (C++11): 9.1.0

### [DART 6.7.0 (2019-01-10)](https://github.com/dartsim/dart/milestone/45?closed=1)

#### Changes

* Build system

  * Fixed compilation warnings for newer versions of compilers: [#1177](https://github.com/dartsim/dart/pull/1177)
  * Changed to generate namespace headers without requiring *.hpp.in files: [#1192](https://github.com/dartsim/dart/pull/1192)
  * Dropped supporting Ubuntu Trusty and started using imported targets of dependencies: [#1212](https://github.com/dartsim/dart/pull/1212)

* Collision Detection

  * CollisionGroups will automatically update their objects when any changes occur to Skeletons or BodyNodes that they are subscribed to: [#1112](https://github.com/dartsim/dart/pull/1112)
  * Contact points with negative penetration depth will be ignored: [#1185](https://github.com/dartsim/dart/pull/1185)

* Math

  * Consolidated random functions into Random class: [#1109](https://github.com/dartsim/dart/pull/1109)

* Dynamics

  * Refactor constraint solver: [#1099](https://github.com/dartsim/dart/pull/1099), [#1101](https://github.com/dartsim/dart/pull/1101)
  * Added mimic joint functionality as a new actuator type: [#1178](https://github.com/dartsim/dart/pull/1178)
  * Added clone function to MetaSkeleton: [#1201](https://github.com/dartsim/dart/pull/1201)

* Optimization

  * Added multi-objective optimization with pagmo2 support: [#1106](https://github.com/dartsim/dart/pull/1106)

* GUI

  * Reorganized OpenGL and GLUT files: [#1088](https://github.com/dartsim/dart/pull/1088)
  * Added the RealTimeWorldNode to display simulations at real-time rates: [#1216](https://github.com/dartsim/dart/pull/1216)

* Misc

  * Updated Googletest to version 1.8.1: [#1214](https://github.com/dartsim/dart/pull/1214)

#### Compilers Tested

* Linux

  * GCC (C++11): 5.4.0, 7.3.0, 8.2.0

* Linux (32-bit)

  * GCC (C++11): 5.4.0

* macOS

  * AppleClang (C++11): 9.1.0

### [DART 6.6.2 (2018-09-03)](https://github.com/dartsim/dart/milestone/47?closed=1)

* Utils

  * Fixed checking file existence in DartResourceRetriever: [#1107](https://github.com/dartsim/dart/pull/1107)

### [DART 6.6.1 (2018-08-04)](https://github.com/dartsim/dart/milestone/46?closed=1)

* Utils

  * Added option to DartResourceRetriever to search from environment variable DART_DATA_PATH: [#1095](https://github.com/dartsim/dart/pull/1095)

* Examples

  * Fixed CMakeLists.txt of humanJointLimits: [#1094](https://github.com/dartsim/dart/pull/1094)

### [DART 6.6.0 (2018-08-02)](https://github.com/dartsim/dart/milestone/44?closed=1)

* Collision detection

  * Added voxel grid map: [#1076](https://github.com/dartsim/dart/pull/1076), [#1083](https://github.com/dartsim/dart/pull/1083)
  * Added heightmap support: [#1069](https://github.com/dartsim/dart/pull/1069)

### [DART 6.5.0 (2018-05-12)](https://github.com/dartsim/dart/milestone/41?closed=1)

* Common

  * Added LockableReference classes: [#1011](https://github.com/dartsim/dart/pull/1011)
  * Added missing \<vector\> to Memory.hpp: [#1057](https://github.com/dartsim/dart/pull/1057)

* GUI

  * Added FOV API to OSG viewer: [#1048](https://github.com/dartsim/dart/pull/1048)

* Parsers

  * Fixed incorrect parsing of continuous joints specified in URDF [#1064](https://github.com/dartsim/dart/pull/1064)

* Simulation

  * Added World::hasSkeleton(): [#1050](https://github.com/dartsim/dart/pull/1050)

* Misc

  * Fixed memory leaks in mesh loading: [#1066](https://github.com/dartsim/dart/pull/1066)

### [DART 6.4.0 (2018-03-26)](https://github.com/dartsim/dart/milestone/39?closed=1)

* Common

  * Added DART_COMMON_DECLARE_SMART_POINTERS macro: [#1022](https://github.com/dartsim/dart/pull/1022)
  * Added ResourceRetriever::getFilePath(): [#972](https://github.com/dartsim/dart/pull/972)

* Kinematics/Dynamics

  * Added relative Jacobian functions to MetaSkeleton: [#997](https://github.com/dartsim/dart/pull/997)
  * Added vectorized joint limit functions: [#996](https://github.com/dartsim/dart/pull/996)
  * Added lazy evaluation for shape's volume and bounding-box computation: [#959](https://github.com/dartsim/dart/pull/959)
  * Added IkFast support as analytic IK solver: [#887](https://github.com/dartsim/dart/pull/887)
  * Added TranslationalJoint2D: [#1003](https://github.com/dartsim/dart/pull/1003)
  * Fixed NaN values caused by zero-length normals in ContactConstraint: [#881](https://github.com/dartsim/dart/pull/881)
  * Extended BodyNode::createShapeNode() to accept more types of arguments: [#986](https://github.com/dartsim/dart/pull/986)

* Collision detection

  * Added FCL 0.6 support (backport of #873): [#936](https://github.com/dartsim/dart/pull/936)

* GUI

  * Added support of rendering texture images: [#973](https://github.com/dartsim/dart/pull/973)
  * Added OSG shadows: [#978](https://github.com/dartsim/dart/pull/978)

* Examples

  * Added humanJointLimits: [#1016](https://github.com/dartsim/dart/pull/1016)

* License

  * Added Personal Robotics Lab and Open Source Robotics Foundation as contributors: [#929](https://github.com/dartsim/dart/pull/929)

* Misc

  * Added World::create(): [#962](https://github.com/dartsim/dart/pull/962)
  * Added MetaSkeleton::hasBodyNode() and MetaSkeleton::hasJoint(): [#1000](https://github.com/dartsim/dart/pull/1000)
  * Suppressed -Winjected-class-name warnings from Clang 5.0.0: [#964](https://github.com/dartsim/dart/pull/964)
  * Suppressed -Wdangling-else warnings from GCC 7.2.0: [#937](https://github.com/dartsim/dart/pull/937)
  * Changed console macros to use global namespace resolutions: [#1010](https://github.com/dartsim/dart/pull/1010)
  * Fixed build with Eigen 3.2.1-3.2.8: [#1042](https://github.com/dartsim/dart/pull/1042)
  * Fixed various build issues with Visual Studio: [#956](https://github.com/dartsim/dart/pull/956)
  * Removed TinyXML dependency: [#993](https://github.com/dartsim/dart/pull/993)

### [DART 6.3.1 (2018-03-21)](https://github.com/dartsim/dart/milestone/42?closed=1)

* Build system

  * Removed an undefined cmake macro/function: [#1036](https://github.com/dartsim/dart/pull/1036)

* ROS support

  * Tweaked package.xml for catkin support: [#1027](https://github.com/dartsim/dart/pull/1027), [#1029](https://github.com/dartsim/dart/pull/1029), [#1031](https://github.com/dartsim/dart/pull/1031), [#1032](https://github.com/dartsim/dart/pull/1031), [#1033](https://github.com/dartsim/dart/pull/1033)

### [DART 6.3.0 (2017-10-04)](https://github.com/dartsim/dart/milestone/36?closed=1)

* Collision detection

  * Added a feature of disabling body node pairs to BodyNodeCollisionFilter: [#911](https://github.com/dartsim/dart/pull/911)

* Kinematics/Dynamics

  * Added setter and getter for WeldJointConstraint::mRelativeTransform: [#910](https://github.com/dartsim/dart/pull/910)

* Parsers

  * Improved SkelParser to read alpha value: [#914](https://github.com/dartsim/dart/pull/914)

* Misc

  * Changed not to use lambda function as an workaround for DART python binding: [#916](https://github.com/dartsim/dart/pull/916)

### [DART 6.2.1 (2017-08-08)](https://github.com/dartsim/dart/milestone/37?closed=1)

* Collision detection

  * Fixed collision checking between objects from the same body node: [#894](https://github.com/dartsim/dart/pull/894)

* Kinematics/Dynamics

  * Fixed transform of ScrewJoint with thread pitch: [#855](https://github.com/dartsim/dart/pull/855)

* Parsers

  * Fixed incorrect reading of <use_parent_model_frame> from SDF: [#893](https://github.com/dartsim/dart/pull/893)
  * Fixed missing reading of joint friction from URDF: [#891](https://github.com/dartsim/dart/pull/891)

* Testing

  * Fixed testing ODE collision detector on macOS: [#884](https://github.com/dartsim/dart/pull/884)
  * Removed redundant main body for each test source file: [#856](https://github.com/dartsim/dart/pull/856)

* Misc

  * Fixed build of dart-gui-osg that depends on the presence of OSG: [#898](https://github.com/dartsim/dart/pull/898)
  * Fixed build of examples and tutorials on macOS: [#889](https://github.com/dartsim/dart/pull/889)
  * Fixed missing overriding method OdePlane::isPlaceable(): [#886](https://github.com/dartsim/dart/pull/886)
  * Replaced use of enum by static constexpr: [#852](https://github.com/dartsim/dart/pull/852), [#904](https://github.com/dartsim/dart/pull/904)

### [DART 6.2.0 (2017-05-15)](https://github.com/dartsim/dart/milestone/30?closed=1)

* Common

  * Added Factory class and applied it to collision detection creation: [#864](https://github.com/dartsim/dart/pull/864)
  * Added readAll() to Resource and ResourceRetriever: [#875](https://github.com/dartsim/dart/pull/875)

* Math

  * Added accessors for diameters and radii of EllipsoidShape, and deprecated EllipsoidShape::get/setSize(): [#829](https://github.com/dartsim/dart/pull/829)
  * Fixed Lemke LCP solver (#808 for DART 6): [#812](https://github.com/dartsim/dart/pull/812)

* Collision Detection

  * Added support of ODE collision detector: [#861](https://github.com/dartsim/dart/pull/861)
  * Fixed incorrect collision filtering of BulletCollisionDetector: [#859](https://github.com/dartsim/dart/pull/859)

* Simulation

  * Fixed World didn't clear collision results on reset: [#863](https://github.com/dartsim/dart/pull/863)

* Parsers

  * Fixed incorrect creation of resource retriever in SkelParser and SdfParser: [#847](https://github.com/dartsim/dart/pull/847), [#849](https://github.com/dartsim/dart/pull/849)

* GUI

  * Added MotionBlurSimWindow: [#840](https://github.com/dartsim/dart/pull/840)
  * Improved MultiSphereShape rendering in GLUT renderer: [#862](https://github.com/dartsim/dart/pull/862)
  * Fixed incorrect parsing of materials and normal scaling from URDF: [#851](https://github.com/dartsim/dart/pull/851)
  * Fixed the OSG renderer not rendering collision geometries: [#851](https://github.com/dartsim/dart/pull/851)
  * Fixed that GUI was rendering white lines with nvidia drivers: [#805](https://github.com/dartsim/dart/pull/805)

* Misc

  * Added createShared() and createUnique() pattern: [#844](https://github.com/dartsim/dart/pull/844)
  * Added Skeleton::getRootJoint(): [#832](https://github.com/dartsim/dart/pull/832)
  * Added CMake targets for code formatting using clang-format: [#811](https://github.com/dartsim/dart/pull/811), [#817](https://github.com/dartsim/dart/pull/817)
  * Renamed MultiSphereShape to MultiSphereConvexHullShape: [#865](https://github.com/dartsim/dart/pull/865)
  * Modified the member function names pertain to lazy evaluation to be more relevant to their functionalities: [#833](https://github.com/dartsim/dart/pull/833)

* Tutorials & Examples

  * Allowed tutorials and examples to be built out of DART source tree: [#842](https://github.com/dartsim/dart/pull/842)
  * Fixed tutorialDominoes-Finished that didn't work with the latest DART: [#807](https://github.com/dartsim/dart/pull/807)

### DART 6.1.2 (2017-01-13)

* Dynamics

  * Fixed bug of ContactConstraint with kinematic joints: [#809](https://github.com/dartsim/dart/pull/809)

* Misc

  * Fixed that ZeroDofJoint::getIndexInTree was called: [#818](https://github.com/dartsim/dart/pull/818)

### DART 6.1.1 (2016-10-14)

* Build

  * Modified to build DART without SIMD options by default: [#790](https://github.com/dartsim/dart/pull/790)
  * Modified to build external libraries as separately build targets: [#787](https://github.com/dartsim/dart/pull/787)
  * Modified to export CMake target files separately per target: [#786](https://github.com/dartsim/dart/pull/786)

* Misc

  * Updated lodepng up to version 20160501: [#791](https://github.com/dartsim/dart/pull/791)

### DART 6.1.0 (2016-10-07)

* Collision detection

  * Added distance API: [#744](https://github.com/dartsim/dart/pull/744)
  * Fixed direction of contact normal of BulletCollisionDetector: [#763](https://github.com/dartsim/dart/pull/763)

* Dynamics

  * Added `computeLagrangian()` to `MetaSkeleton` and `BodyNode`: [#746](https://github.com/dartsim/dart/pull/746)
  * Added new shapes: sphere, capsule, cone, and multi-sphere: [#770](https://github.com/dartsim/dart/pull/770), [#769](https://github.com/dartsim/dart/pull/769), [#745](https://github.com/dartsim/dart/pull/745)
  * Changed base class of joint from SingleDofJoint/MultiDofJoint to GenericJoint: [#747](https://github.com/dartsim/dart/pull/747)

* Planning

  * Fixed incorrect linking to flann library: [#761](https://github.com/dartsim/dart/pull/761)

* Parsers

  * Added `sdf` parsing for `fixed` joint and `material` tag of visual shape: [#775](https://github.com/dartsim/dart/pull/775)
  * Added support of urdfdom_headers 1.0: [#766](https://github.com/dartsim/dart/pull/766)

* GUI

  * Added ImGui for 2D graphical interface: [#781](https://github.com/dartsim/dart/pull/781)

* Examples

  * Added osgAtlasSimbicon and osgTinkertoy: [#781](https://github.com/dartsim/dart/pull/781)

* Misc improvements and bug fixes

  * Added `virtual Shape::getType()` and deprecated `ShapeType Shape::getShapeType()`: [#725](https://github.com/dartsim/dart/pull/725)
  * Changed building with SIMD optional: [#765](https://github.com/dartsim/dart/pull/765), [#760](https://github.com/dartsim/dart/pull/760)
  * Fixed minor build and install issues: [#773](https://github.com/dartsim/dart/pull/773), [#772](https://github.com/dartsim/dart/pull/772)
  * Fixed Doxyfile to show missing member functions in API documentation: [#768](https://github.com/dartsim/dart/pull/768)
  * Fixed typo: [#756](https://github.com/dartsim/dart/pull/756), [#755](https://github.com/dartsim/dart/pull/755)

### DART 6.0.1 (2016-06-29)

* Collision detection

  * Added support of FCL 0.5 and tinyxml2 4.0: [#749](https://github.com/dartsim/dart/pull/749)
  * Added warnings for unsupported shape pairs of DARTCollisionDetector: [#722](https://github.com/dartsim/dart/pull/722)

* Dynamics

  * Fixed total mass is not being updated when bodies removed from Skeleton: [#731](https://github.com/dartsim/dart/pull/731)

* Misc improvements and bug fixes

  * Renamed `DEPRECATED` and `FORCEINLINE` to `DART_DEPRECATED` and `DART_FORCEINLINE` to avoid name conflicts: [#742](https://github.com/dartsim/dart/pull/742)
  * Updated copyright: added CMU to copyright holder, moved individual contributors to CONTRIBUTING.md: [#723](https://github.com/dartsim/dart/pull/723)

### DART 6.0.0 (2016-05-10)

* Common data structures

  * Added `Node`, `Aspect`, `State`, and `Properties`: [#713](https://github.com/dartsim/dart/pull/713), [#712](https://github.com/dartsim/dart/issues/712), [#708](https://github.com/dartsim/dart/pull/708), [#707](https://github.com/dartsim/dart/pull/707), [#659](https://github.com/dartsim/dart/pull/659), [#649](https://github.com/dartsim/dart/pull/649), [#645](https://github.com/dartsim/dart/issues/645), [#607](https://github.com/dartsim/dart/pull/607), [#598](https://github.com/dartsim/dart/pull/598), [#591](https://github.com/dartsim/dart/pull/591), [#531](https://github.com/dartsim/dart/pull/531)
  * Added mathematical constants and user-defined literals for radian, degree, and pi: [#669](https://github.com/dartsim/dart/pull/669), [#314](https://github.com/dartsim/dart/issues/314)
  * Added `ShapeFrame` and `ShapeNode`: [#608](https://github.com/dartsim/dart/pull/608)
  * Added `BoundingBox`: [#547](https://github.com/dartsim/dart/pull/547), [#546](https://github.com/dartsim/dart/issues/546)

* Kinematics

  * Added convenient functions for setting joint limits: [#703](https://github.com/dartsim/dart/pull/703)
  * Added more description on `InverseKinematics::solve()`: [#624](https://github.com/dartsim/dart/pull/624)
  * Added API for utilizing analytical inverse kinematics: [#530](https://github.com/dartsim/dart/pull/530), [#463](https://github.com/dartsim/dart/issues/463)
  * Added color property to `Marker`: [#187](https://github.com/dartsim/dart/issues/187)
  * Improved `Skeleton` to clone `State` as well: [#691](https://github.com/dartsim/dart/pull/691)
  * Improved `ReferentialSkeleton` to be able to add and remove `BodyNode`s and `DegreeOfFreedom`s to/from `Group`s freely: [#557](https://github.com/dartsim/dart/pull/557), [#556](https://github.com/dartsim/dart/issues/556), [#548](https://github.com/dartsim/dart/issues/548)
  * Changed `Marker` into `Node`: [#692](https://github.com/dartsim/dart/pull/692), [#609](https://github.com/dartsim/dart/issues/609)
  * Renamed `Joint::get/setLocal[~]` to `Joint::get/setRelative[~]`: [#715](https://github.com/dartsim/dart/pull/715), [#714](https://github.com/dartsim/dart/issues/714)
  * Renamed `PositionLimited` to `PositionLimitEnforced`: [#447](https://github.com/dartsim/dart/issues/447)
  * Fixed initialization of joint position and velocity: [#691](https://github.com/dartsim/dart/pull/691), [#621](https://github.com/dartsim/dart/pull/621)
  * Fixed `InverseKinematics` when it's used with `FreeJoint` and `BallJoint`: [#683](https://github.com/dartsim/dart/pull/683)
  * Fixed ambiguous overload on `MetaSkeleton::getLinearJacobianDeriv`: [#628](https://github.com/dartsim/dart/pull/628), [#626](https://github.com/dartsim/dart/issues/626)

* Dynamics

  * Added `get/setLCPSolver` functions to `ConstraintSolver`: [#633](https://github.com/dartsim/dart/pull/633)
  * Added `ServoMotorConstraint` as a preliminary implementation for `SERVO` actuator type: [#566](https://github.com/dartsim/dart/pull/566)
  * Improved `ConstraintSolver` to obey C++11 ownership conventions: [#616](https://github.com/dartsim/dart/pull/616)
  * Fixed segfualting of `DantzigLCPSolver` when the constraint dimension is zero: [#634](https://github.com/dartsim/dart/pull/634)
  * Fixed missing implementations in ConstrainedGroup: [#586](https://github.com/dartsim/dart/pull/586)
  * Fixed incorrect applying of joint constraint impulses: [#317](https://github.com/dartsim/dart/issues/317)
  * Deprecated `draw()` functions of dynamics classes: [#654](https://github.com/dartsim/dart/pull/654)

* Collision detection

  * Added `CollisionGroup` and refactored `CollisionDetector` to be more versatile: [#711](https://github.com/dartsim/dart/pull/711), [#704](https://github.com/dartsim/dart/pull/704), [#689](https://github.com/dartsim/dart/pull/689), [#631](https://github.com/dartsim/dart/pull/631), [#642](https://github.com/dartsim/dart/issues/642), [#20](https://github.com/dartsim/dart/issues/20)
  * Improved API for self collision checking options: [#718](https://github.com/dartsim/dart/pull/718), [#702](https://github.com/dartsim/dart/issues/702)
  * Deprecated `BodyNode::isColliding`; collision sets are moved to `CollisionResult`: [#694](https://github.com/dartsim/dart/pull/694), [#670](https://github.com/dartsim/dart/pull/670), [#668](https://github.com/dartsim/dart/pull/668), [#666](https://github.com/dartsim/dart/issues/666)

* Parsers

  * Added back VSK parser: [#602](https://github.com/dartsim/dart/pull/602), [#561](https://github.com/dartsim/dart/pull/561), [#254](https://github.com/dartsim/dart/issues/254)
  * Fixed segfault of `SdfParser` when `nullptr` `ResourceRetriever` is passed: [#663](https://github.com/dartsim/dart/pull/663)

* GUI features

  * Merged `renderer` namespace into `gui` namespace: [#652](https://github.com/dartsim/dart/pull/652), [#589](https://github.com/dartsim/dart/issues/589)
  * Moved `osgDart` under `dart::gui` namespace as `dart::gui::osg`: [#651](https://github.com/dartsim/dart/pull/651)
  * Fixed GlutWindow::screenshot(): [#623](https://github.com/dartsim/dart/pull/623), [#395](https://github.com/dartsim/dart/issues/395)

* Simulation

  * Fixed `World::clone()` didn't clone the collision detector: [#658](https://github.com/dartsim/dart/pull/658)
  * Fixed bug of `World` concurrency: [#577](https://github.com/dartsim/dart/pull/577), [#576](https://github.com/dartsim/dart/issues/576)

* Misc improvements and bug fixes

  * Added `make_unique<T>` that was omitted from C++11: [#639](https://github.com/dartsim/dart/pull/639)
  * Added missing `override` keywords: [#617](https://github.com/dartsim/dart/pull/617), [#535](https://github.com/dartsim/dart/pull/535)
  * Added gcc warning flag `-Wextra`: [#600](https://github.com/dartsim/dart/pull/600)
  * Improved memory management of `constraint` namespace: [#584](https://github.com/dartsim/dart/pull/584), [#583](https://github.com/dartsim/dart/issues/583)
  * Changed the extension of headers from `.h` to `.hpp`: [#709](https://github.com/dartsim/dart/pull/709), [#693](https://github.com/dartsim/dart/pull/693), [#568](https://github.com/dartsim/dart/issues/568)
  * Changed Doxyfile to generate tag file: [#690](https://github.com/dartsim/dart/pull/690)
  * Changed the convention to use `std::size_t` over `size_t`: [#681](https://github.com/dartsim/dart/pull/681), [#656](https://github.com/dartsim/dart/issues/656)
  * Changed CMake to configure preprocessors using `#cmakedefine`: [#648](https://github.com/dartsim/dart/pull/648), [#641](https://github.com/dartsim/dart/pull/641)
  * Updated copyright years: [#679](https://github.com/dartsim/dart/pull/679), [#160](https://github.com/dartsim/dart/issues/160)
  * Renamed directory name `apps` to `examples`: [#685](https://github.com/dartsim/dart/pull/685)
  * Fixed warnings of unused variables in release mode: [#646](https://github.com/dartsim/dart/pull/646)
  * Fixed typo of `getNumPluralAddoName` in utility macro: [#615](https://github.com/dartsim/dart/issues/615)
  * Fixed linker error by adding namespace-scope definitions for `constexpr static` members: [#603](https://github.com/dartsim/dart/pull/603)
  * Fixed segfault from nullptr meshes: [#585](https://github.com/dartsim/dart/pull/585)
  * Fixed typo of tutorial with minor improvements: [#573](https://github.com/dartsim/dart/pull/573)
  * Fixed `NameManager<T>::removeEntries(~)` called a function that does not exist: [#564](https://github.com/dartsim/dart/pull/564), [#554](https://github.com/dartsim/dart/issues/554)
  * Fixed missing definitions for various functions: [#558](https://github.com/dartsim/dart/pull/558), [#555](https://github.com/dartsim/dart/issues/555)
  * Fixed const correctness of `BodyNode::getMomentsOfInertia()`: [#541](https://github.com/dartsim/dart/pull/541), [#540](https://github.com/dartsim/dart/issues/540)
  * Fixed `ftel` bug in Linux with an workaround: [#533](https://github.com/dartsim/dart/pull/533)
  * Removed unnecessary `virtual` keyword for overriding functions: [#680](https://github.com/dartsim/dart/pull/680)
  * Removed deprecated APIs in DART 5: [#678](https://github.com/dartsim/dart/pull/678)

* Build and test issues

  * Added CMake target for code coverage testing, and automatic reporting: [#688](https://github.com/dartsim/dart/pull/688), [#687](https://github.com/dartsim/dart/issues/687), [#638](https://github.com/dartsim/dart/pull/638), [#632](https://github.com/dartsim/dart/pull/632)
  * Added missing `liburdfdom-dev` dependency in Ubuntu package: [#574](https://github.com/dartsim/dart/pull/574)
  * Modulized DART libraries: [#706](https://github.com/dartsim/dart/pull/706), [#675](https://github.com/dartsim/dart/pull/675), [#652](https://github.com/dartsim/dart/pull/652), [#477](https://github.com/dartsim/dart/issues/477)
  * Improved Travis-CI script: [#655](https://github.com/dartsim/dart/pull/655)
  * Improved CMake script by splitting tutorials, examples, and tests into separate targets: [#644](https://github.com/dartsim/dart/pull/644)
  * Improved wording of the cmake warning messages for ASSIMP: [#553](https://github.com/dartsim/dart/pull/553)
  * Changed Travis-CI to treat warning as errors using `-Werror` flags: [#682](https://github.com/dartsim/dart/pull/682), [#677](https://github.com/dartsim/dart/issues/677)
  * Changed Travis-CI to test DART with bullet collision detector: [#650](https://github.com/dartsim/dart/pull/650), [#376](https://github.com/dartsim/dart/issues/376)
  * Changed the minimum requirement of Visual Studio version to 2015: [#592](https://github.com/dartsim/dart/issues/592)
  * Changed CMake to build gui::osg examples when `DART_BUILD_EXAMPLES` is on: [#536](https://github.com/dartsim/dart/pull/536)
  * Simplified Travis-CI tests for general pushes: [#700](https://github.com/dartsim/dart/pull/700)
  * Fixed Eigen memory alignment issue in testCollision.cpp: [#719](https://github.com/dartsim/dart/pull/719)
  * Fixed `BULLET_INCLUDE_DIRS` in `DARTConfig.cmake`: [#697](https://github.com/dartsim/dart/pull/697)
  * Fixed linking with Bullet on OS X El Capitan by supporting for Bullet built with double precision: [#660](https://github.com/dartsim/dart/pull/660), [#657](https://github.com/dartsim/dart/issues/657)
  * Fixed FCL version check logic in the main `CMakeLists.txt`: [#640](https://github.com/dartsim/dart/pull/640)
  * Fixed `find_package(DART)` on optimizer components: [#637](https://github.com/dartsim/dart/pull/637)
  * Fixed linking against `${DART_LIBRARIES}` not working in Ubuntu 14.04: [#630](https://github.com/dartsim/dart/pull/630), [#629](https://github.com/dartsim/dart/issues/629)
  * Fixed Visual Studio 2015 build errors: [#580](https://github.com/dartsim/dart/pull/580)
  * Removed OpenGL dependency from `dart` library: [#667](https://github.com/dartsim/dart/pull/667)
  * Removed version check for Bullet: [#636](https://github.com/dartsim/dart/pull/636), [#625](https://github.com/dartsim/dart/issues/625)

## DART 5

### Version 5.1.6 (2017-08-08)

1. Improved camera movement of OpenGL GUI: smooth zooming and translation
    * [Pull request #843](https://github.com/dartsim/dart/pull/843)

2. Removed debian meta files from the main DART repository
    * [Pull request #853](https://github.com/dartsim/dart/pull/853)

### Version 5.1.5 (2017-01-20)

1. Fixed Lemke LCP solver for several failing cases
    * [Pull request #808](https://github.com/dartsim/dart/pull/808)

1. Increase minimum required Ipopt version to 3.11.9
    * [Pull request #800](https://github.com/dartsim/dart/pull/800)

1. Added support of urdfdom_headers 1.0 for DART 5.1 (backport of [#766](https://github.com/dartsim/dart/pull/766))
    * [Pull request #799](https://github.com/dartsim/dart/pull/799)

### Version 5.1.4 (2016-10-14)

1. Fixed inconsistent frame rate of GlutWindow
    * [Pull request #794](https://github.com/dartsim/dart/pull/794)

### Version 5.1.3 (2016-10-07)

1. Updated to support Bullet built with double precision (backport of [#660](https://github.com/dartsim/dart/pull/660))
    * [Pull request #777](https://github.com/dartsim/dart/pull/777)

1. Modified to use btGImpactMeshShape instead of btConvexTriangleMeshShape for mesh
    * [Pull request #764](https://github.com/dartsim/dart/pull/764)

1. Updated to support FCL 0.5 and tinyxml 4.0 (backport of [#749](https://github.com/dartsim/dart/pull/749))
    * [Pull request #759](https://github.com/dartsim/dart/pull/759)

### Version 5.1.2 (2016-04-25)

1. Fixed inverse kinematics (backporting)
    * [Pull request #684](https://github.com/dartsim/dart/pull/684)

1. Fixed aligned memory allocation with Eigen objects in loading meshes
    * [Pull request #606](https://github.com/dartsim/dart/pull/606)

1. Fixed incorrect applying joint constraint impulses (backporting)
    * [Pull request #579](https://github.com/dartsim/dart/pull/579)

1. Fixed some build and packaging issues
    * [Pull request #559](https://github.com/dartsim/dart/pull/559)
    * [Pull request #595](https://github.com/dartsim/dart/pull/595)
    * [Pull request #696](https://github.com/dartsim/dart/pull/696)

### Version 5.1.1 (2015-11-06)

1. Add bullet dependency to package.xml
    * [Pull request #523](https://github.com/dartsim/dart/pull/523)

1. Improved handling of missing symbols of Assimp package
    * [Pull request #542](https://github.com/dartsim/dart/pull/542)

1. Improved travis-ci build log for Mac
    * [Pull request #529](https://github.com/dartsim/dart/pull/529)

1. Fixed warnings in Function.cpp
    * [Pull request #550](https://github.com/dartsim/dart/pull/550)

1. Fixed build failures on AppVeyor
    * [Pull request #543](https://github.com/dartsim/dart/pull/543)

1. Fixed const qualification of ResourceRetriever
    * [Pull request #534](https://github.com/dartsim/dart/pull/534)
    * [Issue #532](https://github.com/dartsim/dart/issues/532)

1. Fixed aligned memory allocation with Eigen objects
    * [Pull request #527](https://github.com/dartsim/dart/pull/527)

1. Fixed copy safety for various classes
    * [Pull request #526](https://github.com/dartsim/dart/pull/526)
    * [Pull request #539](https://github.com/dartsim/dart/pull/539)
    * [Issue #524](https://github.com/dartsim/dart/issues/524)

### Version 5.1.0 (2015-10-15)

1. Fixed incorrect rotational motion of BallJoint and FreeJoint
    * [Pull request #518](https://github.com/dartsim/dart/pull/518)

1. Removed old documents: dart-tutorial, programmingGuide
    * [Pull request #515](https://github.com/dartsim/dart/pull/515)

1. Fixed aligned memory allocation with Eigen objects
    * [Pull request #513](https://github.com/dartsim/dart/pull/513)

1. Fixed segfault in Linkage::Criteria
    * [Pull request #491](https://github.com/dartsim/dart/pull/491)
    * [Issue #489](https://github.com/dartsim/dart/issues/489)

1. Improved sdf/urdf parser
    * [Pull request #497](https://github.com/dartsim/dart/pull/497)
    * [Pull request #485](https://github.com/dartsim/dart/pull/485)

1. Fixed CMake warnings
    * [Pull request #483](https://github.com/dartsim/dart/pull/483)

1. Fixed build issues on Windows
    * [Pull request #516](https://github.com/dartsim/dart/pull/516)
    * [Pull request #509](https://github.com/dartsim/dart/pull/509)
    * [Pull request #486](https://github.com/dartsim/dart/pull/486)
    * [Pull request #482](https://github.com/dartsim/dart/pull/482)
    * [Issue #487](https://github.com/dartsim/dart/issues/487)

1. Fixed IpoptSolver bugs
    * [Pull request #481](https://github.com/dartsim/dart/pull/481)

1. Added Frame::getTransform(withRespecTo, inCoordinatesOf)
    * [Pull request #475](https://github.com/dartsim/dart/pull/475)
    * [Issue #471](https://github.com/dartsim/dart/issues/471)

1. Improved API documentation -- set the SHOW_USED_FILES tag to NO
    * [Pull request #474](https://github.com/dartsim/dart/pull/474)

1. Added convenience setters for generalized coordinates of FreeJoint
    * [Pull request #470](https://github.com/dartsim/dart/pull/470)
    * [Pull request #507](https://github.com/dartsim/dart/pull/507)

1. Fixed compilation warnings
    * [Pull request #480](https://github.com/dartsim/dart/pull/480)
    * [Pull request #469](https://github.com/dartsim/dart/pull/469)
    * [Issue #418](https://github.com/dartsim/dart/issues/418)

1. Added a mutex to Skeleton
    * [Pull request #466](https://github.com/dartsim/dart/pull/466)

1. Added generic URIs support
    * [Pull request #464](https://github.com/dartsim/dart/pull/464)
    * [Pull request #517](https://github.com/dartsim/dart/pull/517)

1. Added End Effector, Inverse Kinematics, and osgDart
    * [Pull request #461](https://github.com/dartsim/dart/pull/461)
    * [Pull request #495](https://github.com/dartsim/dart/pull/495)
    * [Pull request #502](https://github.com/dartsim/dart/pull/502)
    * [Pull request #506](https://github.com/dartsim/dart/pull/506)
    * [Pull request #514](https://github.com/dartsim/dart/pull/514)
    * [Issue #381](https://github.com/dartsim/dart/issues/381)
    * [Issue #454](https://github.com/dartsim/dart/issues/454)
    * [Issue #478](https://github.com/dartsim/dart/issues/478)

1. Removed outdated packaging scripts
    * [Pull request #456](https://github.com/dartsim/dart/pull/456)

1. Added initial position and initial velocity properties
    * [Pull request #449](https://github.com/dartsim/dart/pull/449)

1. Added a package.xml file for REP-136 support
    * [Pull request #446](https://github.com/dartsim/dart/pull/446)

1. Improved Linkage and Chain Criteria
    * [Pull request #443](https://github.com/dartsim/dart/pull/443)
    * [Issue #437](https://github.com/dartsim/dart/issues/437)

1. Added Joint::isCyclic to mark SO(2) topology
    * [Pull request #441](https://github.com/dartsim/dart/pull/441)

1. Fixed SEGFAULTs in DartLoader
    * [Pull request #439](https://github.com/dartsim/dart/pull/439)

1. Added the SYSTEM flag to include_directories
    * [Pull request #435](https://github.com/dartsim/dart/pull/435)

1. Improved Joint warning
    * [Pull request #430](https://github.com/dartsim/dart/pull/430)

1. Added tutorials (http://dart.readthedocs.org/)
    * [Pull request #504](https://github.com/dartsim/dart/pull/504)
    * [Pull request #484](https://github.com/dartsim/dart/pull/484)
    * [Pull request #423](https://github.com/dartsim/dart/pull/423)
    * [Pull request #511](https://github.com/dartsim/dart/pull/511)

### Version 5.0.2 (2015-09-28)

1. Fixed bug in Jacobian update notifications
    * [Pull request #500](https://github.com/dartsim/dart/pull/500)
    * [Issue #499](https://github.com/dartsim/dart/issues/499)

### Version 5.0.1 (2015-07-28)

1. Improved app indexing for bipedStand and atlasSimbicon
    * [Pull request #417](https://github.com/dartsim/dart/pull/417)

1. Added clipping command when it exceeds the limits
    * [Pull request #419](https://github.com/dartsim/dart/pull/419)

1. Improved CollisionNode's index validity check
    * [Pull request #421](https://github.com/dartsim/dart/pull/421)

1. Standardized warning messages for Joints
    * [Pull request #425](https://github.com/dartsim/dart/pull/425)
    * [Pull request #429](https://github.com/dartsim/dart/pull/429)

1. Fixed bug in SDF parser -- correct child for a joint
    * [Pull request #431](https://github.com/dartsim/dart/pull/431)

1. Fixed SDF parsing for single link model without joint
    * [Pull request #444](https://github.com/dartsim/dart/pull/444)

1. Added missing virtual destructors to Properties in Entity and [Soft]BodyNode
    * [Pull request #458](https://github.com/dartsim/dart/pull/458)

1. Limited maximum required version of Assimp less than 3.0~dfsg-4
    * [Pull request #459](https://github.com/dartsim/dart/pull/459)

1. Fixed SEGFAULTs in DartLoader
    * [Pull request #472](https://github.com/dartsim/dart/pull/472)

### Version 5.0.0 (2015-06-15)

1. Fixed aligned memory allocation with Eigen objects
    * [Pull request #414](https://github.com/dartsim/dart/pull/414)

1. Added some missing API for DegreeOfFreedom
    * [Pull request #408](https://github.com/dartsim/dart/pull/408)

1. Replaced logMaps with Eigen::AngleAxisd
    * [Pull request #407](https://github.com/dartsim/dart/pull/407)

1. Improved FCL collision detector
    * [Pull request #405](https://github.com/dartsim/dart/pull/405)

1. Removed deprecated API and suppressed warnings
    * [Pull request #404](https://github.com/dartsim/dart/pull/404)

1. Added use of OpenGL's multisample anti-aliasing
    * [Pull request #402](https://github.com/dartsim/dart/pull/402)

1. Added computation of differences of generalized coordinates
    * [Pull request #389](https://github.com/dartsim/dart/pull/389)
    * [Issue #290](https://github.com/dartsim/dart/issues/290)

1. Added deprecated and force-linline definitions for clang
    * [Pull request #384](https://github.com/dartsim/dart/pull/384)
    * [Issue #379](https://github.com/dartsim/dart/issues/379)

1. Eradicated memory leaks and made classes copy-safe and clonable
    * [Pull request #369](https://github.com/dartsim/dart/pull/369)
    * [Pull request #390](https://github.com/dartsim/dart/pull/390)
    * [Pull request #391](https://github.com/dartsim/dart/pull/391)
    * [Pull request #392](https://github.com/dartsim/dart/pull/392)
    * [Pull request #397](https://github.com/dartsim/dart/pull/397)
    * [Pull request #415](https://github.com/dartsim/dart/pull/415)
    * [Issue #280](https://github.com/dartsim/dart/issues/280)
    * [Issue #339](https://github.com/dartsim/dart/issues/339)
    * [Issue #370](https://github.com/dartsim/dart/issues/370)
    * [Issue #383](https://github.com/dartsim/dart/issues/383)

1. Improved PlaneShape constructors
    * [Pull request #366](https://github.com/dartsim/dart/pull/366)
    * [Pull request #377](https://github.com/dartsim/dart/pull/377)
    * [Issue #373](https://github.com/dartsim/dart/issues/373)

1. Added appveyor options for parallel build and detailed log
    * [Pull request #365](https://github.com/dartsim/dart/pull/365)

1. Improved robustness and package handling for URDF parsing
    * [Pull request #364](https://github.com/dartsim/dart/pull/364)

1. Fixed bug in BodyNode::_updateBodyJacobianSpatialDeriv()
    * [Pull request #363](https://github.com/dartsim/dart/pull/363)

1. Added alpha channel and Color functions
    * [Pull request #359](https://github.com/dartsim/dart/pull/359)
    * [Issue #358](https://github.com/dartsim/dart/issues/358)

1. Added Jacobian getters to Skeleton
    * [Pull request #357](https://github.com/dartsim/dart/pull/357)

1. Added ArrowShape for visualizing arrows
    * [Pull request #356](https://github.com/dartsim/dart/pull/356)

1. Fixed matrix dimension bug in operationalSpaceControl app
    * [Pull request #354](https://github.com/dartsim/dart/pull/354)

1. Added build type definitions
    * [Pull request #353](https://github.com/dartsim/dart/pull/353)

1. Added Signal class
    * [Pull request #350](https://github.com/dartsim/dart/pull/350)

1. Added LineSegmentShape for visualizing line segments
    * [Pull request #349](https://github.com/dartsim/dart/pull/349)
    * [Issue #346](https://github.com/dartsim/dart/issues/346)

1. Fixed segfault in SoftSdfParser
    * [Pull request #345](https://github.com/dartsim/dart/pull/345)

1. Added subscriptions for destructions and notifications
    * [Pull request #343](https://github.com/dartsim/dart/pull/343)

1. Added NloptSolver::[get/set]NumMaxEvaluations()
    * [Pull request #342](https://github.com/dartsim/dart/pull/342)

1. Added support of Eigen::VectorXd in parser
    * [Pull request #341](https://github.com/dartsim/dart/pull/341)

1. Added Skeleton::getNumJoints()
    * [Pull request #335](https://github.com/dartsim/dart/pull/335)

1. Fixed bug in DARTCollide for sphere-sphere collision
    * [Pull request #332](https://github.com/dartsim/dart/pull/332)

1. Fixed naming issues for Skeletons in World
    * [Pull request #331](https://github.com/dartsim/dart/pull/331)
    * [Issue #330](https://github.com/dartsim/dart/issues/330)

1. Added PlanarJoint support for URDF loader
    * [Pull request #326](https://github.com/dartsim/dart/pull/326)

1. Fixed rotation of the inertia reference frame for URDF loader
    * [Pull request #326](https://github.com/dartsim/dart/pull/326)
    * [Issue #47](https://github.com/dartsim/dart/issues/47)

1. Fixed bug in loading WorldFile
    * [Pull request #325](https://github.com/dartsim/dart/pull/325)

1. Added plotting of 2D trajectories
    * [Pull request #324](https://github.com/dartsim/dart/pull/324)

1. Removed unsupported axis orders of EulerJoint
    * [Pull request #323](https://github.com/dartsim/dart/pull/323)
    * [Issue #321](https://github.com/dartsim/dart/issues/321)

1. Added convenience functions to help with setting joint positions
    * [Pull request #322](https://github.com/dartsim/dart/pull/322)
    * [Pull request #338](https://github.com/dartsim/dart/pull/338)

1. Added Frame class and auto-updating for forward kinematics
    * [Pull request #319](https://github.com/dartsim/dart/pull/319)
    * [Pull request #344](https://github.com/dartsim/dart/pull/344)
    * [Pull request #367](https://github.com/dartsim/dart/pull/367)
    * [Pull request #380](https://github.com/dartsim/dart/pull/380)
    * [Issue #289](https://github.com/dartsim/dart/issues/289)
    * [Issue #294](https://github.com/dartsim/dart/issues/294)
    * [Issue #305](https://github.com/dartsim/dart/issues/305)

1. Added Travis-CI build test for OSX
    * [Pull request #313](https://github.com/dartsim/dart/pull/313)
    * [Issue #258](https://github.com/dartsim/dart/issues/258)

1. Added specification of minimum dependency version
    * [Pull request #306](https://github.com/dartsim/dart/pull/306)

## DART 4

### Version 4.3.7 (2018-01-05)

1. Updated DART 4.3 to be compatible with urdf 1.0/tinyxml2 6/flann 1.9.1
    * [Pull request #955](https://github.com/dartsim/dart/pull/955)

### Version 4.3.6 (2016-04-16)

1. Fixed duplicate entries in Skeleton::mBodyNodes causing segfault in destructor
    * [Issue #671](https://github.com/dartsim/dart/issues/671)
    * [Pull request #672](https://github.com/dartsim/dart/pull/672)

### Version 4.3.5 (2016-01-09)

1. Fixed incorrect applying of joint constraint impulses (backported from 6.0.0)
    * [Pull request #578](https://github.com/dartsim/dart/pull/578)

### Version 4.3.4 (2015-01-24)

1. Fixed build issue with gtest on Mac
    * [Pull request #315](https://github.com/dartsim/dart/pull/315)

### Version 4.3.3 (2015-01-23)

1. Fixed joint Coulomb friction
    * [Pull request #311](https://github.com/dartsim/dart/pull/311)

### Version 4.3.2 (2015-01-22)

1. Fixed installation -- missing headers (utils/urdf, utils/sdf)

### Version 4.3.1 (2015-01-21)

1. Fixed API incompatibility introduced by dart-4.3.0
    * [Issue #303](https://github.com/dartsim/dart/issues/303)
    * [Pull request #309](https://github.com/dartsim/dart/pull/309)

### Version 4.3.0 (2015-01-19)

1. Added name manager for efficient name look-up and unique naming
    * [Pull request #277](https://github.com/dartsim/dart/pull/277)
1. Added all-inclusive header and namespace headers
    * [Pull request #278](https://github.com/dartsim/dart/pull/278)
1. Added DegreeOfFreedom class for getting/setting data of individual generalized coordinates
    * [Pull request #288](https://github.com/dartsim/dart/pull/288)
1. Added hybrid dynamics
    * [Pull request #298](https://github.com/dartsim/dart/pull/298)
1. Added joint actuator types
    * [Pull request #298](https://github.com/dartsim/dart/pull/298)
1. Added Coulomb joint friction
    * [Pull request #301](https://github.com/dartsim/dart/pull/301)
1. Migrated to C++11
    * [Pull request #268](https://github.com/dartsim/dart/pull/268)
    * [Pull request #299](https://github.com/dartsim/dart/pull/299)
1. Improved readability of CMake output messages
    * [Pull request #272](https://github.com/dartsim/dart/pull/272)
1. Fixed const-correctneess of member functions
    * [Pull request #277](https://github.com/dartsim/dart/pull/277)
1. Added handling use of 'package:/' in URDF
    * [Pull request #273](https://github.com/dartsim/dart/pull/273)
    * [Issue #271](https://github.com/dartsim/dart/issues/271)

### Version 4.2.1 (2015-01-07)

1. Fixed version numbering of shared libraries in debian packages
    * [Pull request #286](https://github.com/dartsim/dart/pull/286)
1. Fixed Jacobian and its derivatives of FreeJoint/BallJoint
    * [Pull request #284](https://github.com/dartsim/dart/pull/284)

### Version 4.2.0 (2014-11-22)

1. Added reset functions for Simulation and Recording class
    * [Pull request #231](https://github.com/dartsim/dart/pull/231)
1. Added operational space control example
    * [Pull request #257](https://github.com/dartsim/dart/pull/257)
1. Fixed misuse of Bullet collision shapes
    * [Pull request #228](https://github.com/dartsim/dart/pull/228)
1. Fixed adjacent body pair check for Bullet collision detector
    * [Pull request #246](https://github.com/dartsim/dart/pull/246)
1. Fixed incorrect computation of constraint impulse for BallJointConstraint and WeldJointContraint
    * [Pull request #247](https://github.com/dartsim/dart/pull/247)
1. Improved generation of soft box shape for soft body
    * [Commit ec31f44](https://github.com/dartsim/dart/commit/ec31f44)

### Version 4.1.1 (2014-07-17)

1. Added ABI check script
    * [Pull request #226](https://github.com/dartsim/dart/pull/226)
    * [Pull request #227](https://github.com/dartsim/dart/pull/227)
1. Fixed build issues on Linux
    * [Pull request #214](https://github.com/dartsim/dart/pull/214)
    * [Pull request #219](https://github.com/dartsim/dart/pull/219)
1. Fixed build issues on Windows
    * [Pull request #215](https://github.com/dartsim/dart/pull/215)
    * [Pull request #217](https://github.com/dartsim/dart/pull/217)
1. Fixed unintended warning messages
    * [Pull request #220](https://github.com/dartsim/dart/pull/220)

### Version 4.1.0 (2014-07-02)

1. Fixed bug in switching collision detectors
    * [Issue #127](https://github.com/dartsim/dart/issues/127)
    * [Pull request #195](https://github.com/dartsim/dart/pull/195)
1. Fixed kinematics and dynamics when a skeleton has multiple parent-less bodies
    * [Pull request #196](https://github.com/dartsim/dart/pull/196)
1. Fixed issue on installing DART 4 alongside DART 3 on Linux
    * [Issue #122](https://github.com/dartsim/dart/issues/122)
    * [Pull request #203](https://github.com/dartsim/dart/pull/203)
1. Fixed warnings on gcc
    * [Pull request #206](https://github.com/dartsim/dart/pull/206)
1. Renamed getDof() to getNumDofs()
    * [Pull request #209](https://github.com/dartsim/dart/pull/209)
1. Added cylinder shape for soft body
    * [Pull request #210](https://github.com/dartsim/dart/pull/210)

### Version 4.0.0 (2014-06-02)

1. Added implicit joint spring force and damping force
1. Added planar joint
1. Added soft body dynamics
1. Added computation of velocity and acceleration of COM
1. Added bullet collision detector
  * [Pull request #156](https://github.com/dartsim/dart/pull/156)
1. Improved performance of forward dynamics algorithm
  * [Pull request #188](https://github.com/dartsim/dart/pull/188)
1. Improved dynamics API for Skeleton and Joint
  * [Pull request #161](https://github.com/dartsim/dart/pull/161)
  * [Pull request #192](https://github.com/dartsim/dart/pull/192)
  * [Pull request #193](https://github.com/dartsim/dart/pull/193)
1. Improved constraint dynamics solver
  * [Pull request #184](https://github.com/dartsim/dart/pull/184)
1. Improved calculation of equations of motion using Featherstone algorithm
  * [Issue #85](https://github.com/dartsim/dart/issues/87)
1. Improved optimizer interface and added nlopt solver
  * [Pull request #152](https://github.com/dartsim/dart/pull/152)
1. Fixed self collision bug
  * [Issue #125](https://github.com/dartsim/dart/issues/125)
1. Fixed incorrect integration of BallJoint and FreeJoint
  * [Issue #122](https://github.com/dartsim/dart/issues/122)
  * [Pull request #168](https://github.com/dartsim/dart/pull/168)

## DART 3

### Version 3.0 (2013-11-04)

1. Removed Transformation classes. Their functionality is now included in joint classes.
1. Added Featherstone algorithm. Can currently only be used without collision handling. The old algorithm is still present and used for that case.
1. Removed kinematics namespace. Functionality is moved to dynamics classes.
1. Added dart root namespace
1. A lot of function and variable renames
1. Added constraint namespace
1. Added "common" namespace

## DART 2

### Version 2.6 (2013-09-07)

1. Clean-up of build system:
  * Renamed DART_INCLUDEDIR to the standard-compliant DART_INCLUDE_DIRS in CMake files. Users need to adapt their CMake files for this change.
  * Users no longer need to call find_package(DARTExt) in the CMake files. A call to find_package(DART) also finds its dependencies now.
  * Allow user to overwrite installation prefix
  * Add possibility to include DART header files as '#include \<dart/dynamics/Skeleton.h\>' in addition to '#include \<dynamics/Skeleton.h\>'
  * Allow out-of-source builds
1. URDF loader:
  * Major clean-up
  * Consider mesh scaling factor

### Version 2.5 (2013-07-16)

1. Replaced robotics::World with simulation::World
1. Removed robotics::Robot
1. Added simulation::SimWindow
1. Some speed-up of Eigen calculations
1. Added abstract trajectory interface
1. ConstraintDynamics handles contact, joint limit and other constraint forces simultaneously
1. Improved Lemke algorithm for solving LCP
1. Renamed skeletonDynamics::getQDotVector() to getPoseVelocity()
1. Added abstract CollisionDetector interface allowing for multiple different collision detector implementations.
1. Created math namespace
1. Added System class as base class to Skeleton and Joint
1. URDF loader: Removed ability to load nonstandard URDF files with an object tag
1. URDF loader: Removed ability to load nonstandard URDF files with an object tag
1. Added support for multiple shapes per BodyNode
1. Made urdfdom a dependency instead of including it in the DART source
1. Added function to CollisionDetector to let user check a specific pair of BodyNodes for collision

### Version 2.4 (2013-03-05)

1. Mass and inertia are no longer stored in Shape but in BodyNode.
1. Different shapes for collision and visualization (not just different meshes)
1. Shapes are no longer centered at the COM but can be transformed independently relative to the link frame.
1. Improved URDF support
  * Support for non-mesh shapes
  * Does not create dummy root nodes anymore
  * Support for continuous joints
  * Support for arbitrary joint axes for revolute joints (but not for prismatic joints) instead of only axis-aligned joint axes
  * Support for relative mesh paths even if the robot and world URDF files are in different directories
  * All supported joint types can be root joints
1. Clean-up of the Robot class
1. Removed Object class
1. More robust build and installation process on Linux
