# Gazebo tooling

DART 6.20 checks Gazebo in two ways:

| Lane | Command | gz-physics / gz-sim | Patches | What it checks |
| --- | --- | --- | --- | --- |
| Forward (patched) | `pixi run -e gazebo test-gz` | 8.0.0 / 9.0.0 | [`patches/`](patches/) | gz-physics suite and gz-sim `INTEGRATION_entity_system` pass/fail |
| Unpatched compatibility | `pixi run gz-compat-<lane>` | released tags below | none | full gz-physics suite and full gz-sim `INTEGRATION_` suite against DART 6.19.4's failures (and, optionally, against the change's base), plus a `StepWorld` time limit |

`test-gz` is the forward lane that CI runs. Its patches change gz-physics
source (the `ChangedWorldPoses` tolerance) and relax test expectations, so a
green `test-gz` does not show that released Gazebo still works. It also
configures gz-physics once, so the common tests compile without the dartsim
plugin's `DART_HAS_CONTACT_SURFACE` and skip their contact-callback
expectations (gz-physics `ContactPropertiesCallback`). The unpatched lanes
configure twice and run them.

## Unpatched compatibility lanes

| Lane | Pixi task (environment) | gz-physics | gz-sim | sdformat |
| --- | --- | --- | --- | --- |
| Harmonic | `gz-compat-harmonic` (`gazebo-harmonic`) | `gz-physics7_7.8.0` | `gz-sim8_8.10.0` | 14 |
| Ionic | `gz-compat-ionic` (`gazebo`) | `gz-physics8_8.4.0` | `gz-sim9_9.5.0` | 15 |
| Jetty | `gz-compat-jetty` (`gazebo-jetty`) | `gz-physics9_9.5.2` | `gz-sim10_10.5.0` | 16 |

The other Gazebo libraries come from conda-forge. Ionic uses gz-sim 9.5.0
because 9.6.0 needs a newer gz-common6 than conda-forge ships. Harmonic and
Jetty pin newer urdfdom, fmt and spdlog than DART's default environment, so
their environments take DART's build dependencies from the `gz-compat-base`
feature instead of the default feature. The lane tasks require Linux.

`pixi run gz-compat-<lane>` runs [`compat/lane.sh`](compat/lane.sh)
`<lane> test`, which:

1. builds DART from this checkout and installs it into an emptied
   `.deps/gz-compat/<lane>/candidate/dart`, so no file an earlier build
   installed is left behind (the build directory is kept);
2. builds the released gz-physics with its tests against that DART and runs
   its CTest suite (`PERFORMANCE_` tests excluded);
3. builds gz-sim with its tests against that gz-physics and runs every
   `INTEGRATION_` test serially, with `GZ_SIM_SERVER_CONFIG_PATH` set (without
   it, worlds that declare no systems get no Physics system), a private
   `TMPDIR`, `GZ_IP=127.0.0.1`, a unique `GZ_PARTITION` and the lane's Fuel
   model cache (below), followed by its gz-cmake `check_` test, which records
   a failure for a test that left no results (for example after a crash); a
   test that fails, then passes on its retry, counts as passing (`FLAKY`)
   unless an attempt crashed or timed out, and `compare` reads every attempt
   from the CTest log;
4. runs [`compat/compare_failures.py`](compat/compare_failures.py) (see
   below).

The lanes serialize cloning with `flock` and share gz-physics and gz-sim in
`.deps/gz-compat/<lane>/src/`
and stop when a clone is not at its pinned tag or has local changes, which
they would otherwise build as the released sources. Run
`git reset --hard && git clean -fd` in the clone, or delete it, to restore it.

Pass a step to run one stage, for example
`pixi run gz-compat-jetty test-gz-sim` or `pixi run gz-compat-jetty compare`.
The steps are `dart`, `gz-physics`, `test-gz-physics`, `gz-sim`,
`fuel-cache`, `test-gz-sim`, `compare`, `baseline`, `bench`, `worlds`,
`bench-gz-physics`, `bench-gz-sim`, `sleep-oracle` and `raycast-probe`.
Environment variables:

| Variable | Default | Meaning |
| --- | --- | --- |
| `GZ_COMPAT_DART_SOURCE` | this checkout | DART source tree to build |
| `GZ_COMPAT_VARIANT` | `candidate` | name of that DART build; each variant gets its own gz-physics and gz-sim builds |
| `GZ_COMPAT_BASE_VARIANT` | unset | variant holding the change's base, for the comparison below; the lane stops when it names the candidate's own variant |
| `GZ_COMPAT_DIR` | `.deps/gz-compat/<lane>` | work directory |
| `DART_PARALLEL_JOBS` | `nproc` | build jobs |
| `GZ_COMPAT_TEST_JOBS` | build jobs | parallel gz-physics tests |
| `GZ_COMPAT_MAX_SECONDS_SCALE` | 1 | multiplier for `max-seconds` limits on slower hosts |

gz-cmake gives every gz-physics and gz-sim GoogleTest test a fixed 240 s
`TIMEOUT`, which `ctest --timeout` cannot raise. Its `check_<test>` writes a
failing result for `<test>` when the test left no GoogleTest XML; the lanes
include [`compat/order_check_tests.cmake`](compat/order_check_tests.cmake)
when they configure gz-physics and gz-sim, so that `check_<test>` runs after
`<test>` and cannot write over XML still being written in the parallel
gz-physics run. Rerun the `gz-physics` step to reconfigure a build made
before that.

gz-sim's tests give each case a fake home and delete it afterwards, so every
case downloaded its Fuel models again and unpacked them in the test process.
Test binaries that load assimp before libzip (on Ionic, the two
`model_photo_shoot` tests) call assimp's bundled `zip_open` instead of
libzip's, so the unpacking failed, or crashed in libzip. The `fuel-cache` step,
which `test-gz-sim` runs, downloads every Fuel model named in gz-sim's tests
and in the example worlds they load into `.deps/gz-compat/<lane>/fuel-cache/`
with `gz fuel download`, checks that each one unpacked, and publishes the cache
only then; it rebuilds the cache when that list of models changes. Each
`test-gz-sim` run points `GZ_FUEL_CACHE_PATH` at its own hard-linked copy of
the cache, so every variant reads the same models and a test that still
downloads (one that sets its own resource cache) cannot change the shared
cache. Delete the directory to download the models again.

Results land in `.deps/gz-compat/<lane>/<variant>/results/`: the CTest logs
and JUnit reports, and the per-test GoogleTest XML files. Every variant has
its own builds, temporary directory and transport partition, so lanes and
variants can run concurrently, although timing-sensitive gz-sim tests fail
more often on a loaded host.

### Comparing failures

The rule is that every test DART 6.19.4 passes must still pass: the
candidate's failing tests and cases must be a subset of 6.19.4's, listed in
`compat/<lane>-expected-failures.txt`. A failing CTest test is described by
its failing GoogleTest cases when it exited normally with at least one
failing case. A test that crashed, timed out, did not run or failed without a
failing case is a test-level failure, because its case results are missing,
partial or left over from an earlier attempt; only a test-level entry in the
expected file accepts it, and such an entry accepts any failure of that test.
The JUnit report and the GoogleTest XML keep only a retried test's last
attempt, so `compare` reads the attempts from the CTest log: a crash or
timeout on the first attempt is a test-level failure too, even when the retry
passes or fails only 6.19.4's cases. A crash outside DART (visible in the
stack trace in `gz-sim.log`) shows the same way; rerun the test to tell.

`compare` prints one line per difference. Only `NEW` and `SLOW` fail the
lane:

| Label | Meaning |
| --- | --- |
| `NEW` | fails on the candidate, passes on 6.19.4 (and on the base, if given) |
| `SLOW` | a `max-seconds` case is over its limit (with a base: and was not already, or got more than twice as slow as there) |
| `BASE` | fails on the candidate and on the base (the identical failure: the same case, or for a test-level failure, a test-level failure of the same test), passes on 6.19.4: a regression the change did not introduce |
| `BASE SLOW` | over its limit on the candidate and on the base |
| `REGRESSED` | an expected failure (one 6.19.4 also has, or an accepted difference) that ran and passed on the base: allowed by the rule, but explain it |
| `FLAKY` | failed, then passed on its retry (a first attempt that crashed or timed out is `NEW` instead); names the cases the failed attempt reported |
| `RETRIED` | failed both attempts, and the first attempt failed cases the second did not (named), or reported none |
| `ACCEPTED` | an accepted difference (below) that still fails |
| `STALE` | an accepted difference that now passes; drop the entry |
| `FIXED` | an expected failure that ran and passed |

`compare` stops with an error (exit status 2, as for missing or unreadable
results) when the run lacks a test that the expected-failure file names, or a
`max-seconds` case whose test did not fail at the test level: a run that a
test filter or a renamed test emptied would otherwise pass and skip the time
limit. `lane.sh` itself clears GoogleTest's `GTEST_*` variables, so an
inherited `GTEST_FILTER` or sharding cannot run only part of the suites.

On release-6.20 the lanes fail until the issue #3056 fixes land,
so a change on release-6.20 is judged against its base. Build and test the
base once per lane as its own variant, then compare the change with it:

```bash
git worktree add --detach /tmp/dart-base origin/release-6.20
GZ_COMPAT_VARIANT=base GZ_COMPAT_DART_SOURCE=/tmp/dart-base \
  pixi run gz-compat-ionic test    # its own compare fails; that is expected
GZ_COMPAT_BASE_VARIANT=base pixi run gz-compat-ionic
```

### Expected failures

`compat/<lane>-expected-failures.txt` lists 6.19.4's failures for that lane,
and its header records the host and date. Regenerate a file with the same
lane, building 6.19.4 as a second variant:

```bash
mkdir -p /tmp/dart-6.19.4 && git archive v6.19.4 | tar -x -C /tmp/dart-6.19.4
export GZ_COMPAT_VARIANT=dart-6.19.4 GZ_COMPAT_DART_SOURCE=/tmp/dart-6.19.4
for step in dart gz-physics test-gz-physics test-gz-sim baseline; do
  pixi run gz-compat-jetty "$step"
done
```

`baseline` also writes a `max-seconds` limit of twice the measured time for
the dartsim `StepWorld` case; it is the only gate here that catches a large
slowdown that still passes (it took 16 times longer with DART 6.20's trimesh
ODE cylinders). Entries carrying an inline
`# accepted: <reason>` comment are reviewed differences from 6.19.4 that a
maintainer decided are not DART regressions; `baseline` keeps them, and
`compare` prints them as `ACCEPTED` or `STALE`. Run
`pixi run test-gz-compat-tools` after changing `compare_failures.py` or the
world generator.

## Drivers

[`bench/`](bench/) holds drivers that link one lane's gz-physics, gz-sim and
DART. `pixi run gz-compat-<lane> bench` builds them after the lane has built
gz-sim.

- `gz_physics_step_bench` steps an SDF world through the dartsim plugin the
  way gz-sim's Physics system does (`ConstructSdfWorld`, the world's
  `<max_contacts>` through `SetCollisionPairMaxContacts`, a fresh
  `ForwardStep::Output` per step). It reports time per step, the real-time
  factor, changed poses per step, contacts, and with `--sunk-z` the links of
  non-static models below that height. `pixi run gz-compat-<lane>
  bench-gz-physics` runs gz-sim's `3k_shapes.sdf` for 3000 steps; extra
  arguments replace the defaults (`<world.sdf> <steps> [options]`), and
  `--max-contacts 4` gives the per-pair-4 row.
  `--detector` accepts `ode`, `bullet`, `fcl` and `dart`; the driver fails
  on SDF load errors or if the plugin does not select the requested detector.
  `--contacts-every K` samples every K steps independently of `--window`.
  Samples between timing rows print `step=N contacts=C`; coincident samples
  keep the `contacts=C` field on the timing row.
- `gz_sim_server_bench` runs a world in a `gz::sim::Server` and reports time
  per iteration; it fails when the server did not run every iteration (a
  world that did not load, or a server stopped by a signal).
  `pixi run gz-compat-<lane> bench-gz-sim` runs `3k_shapes.sdf` with its
  real-time factor set to 0 for 1000 iterations.
- `pixi run gz-compat-<lane> worlds` writes the benchmark worlds derived
  from `3k_shapes.sdf` into `.deps/gz-compat/<lane>/worlds/`
  ([`bench/make_worlds.py`](bench/make_worlds.py)), all with real-time factor
  0: `3k_shapes.sdf`, `3k_shapes_drop5cm.sdf` (released 5 cm above the
  ground), `6k_shapes.sdf` (a second copy 4.5 m along +y) and
  `3k_shapes_pendulum.sdf` (plus a contact-free pendulum 500 m away, which
  keeps one body awake).
- `gz_sleep_oracle` is the differential sleep oracle. Each scenario applies
  one state mutation the dartsim plugin exposes and runs the same script with
  DART's deactivation enabled and disabled, reaching DART's `World` through
  the plugin's `RetrieveWorld` feature: settle, mutate, step 500 more, then
  compare the published link poses step by step and the reported contacts
  (their counts on the first step after the mutation and at the end, and the
  final contact points). The scenarios cover joint spring stiffness and
  reference, damping and friction (gz-physics 8 and later), position, velocity and
  effort limits, velocity commands, force, position and velocity, and the
  joint-to-child transform; link and model gravity flags; world gravity;
  free-group pose and linear velocity; link wrenches; model static and
  collision flags; collision and category masks; shape pose and attaching a
  shape; the per-pair contact limit; the collision detector;
  contact-properties callbacks (a conveyor); attaching fixed and revolute
  joints and detaching joints; removing a support under a resting body and
  spawning a model onto one. They do not cover `SetSolver`, free-group
  angular velocity, prismatic joints, joint axes, friction-pyramid slip
  compliance or constructing empty entities. The dartsim plugin cannot change
  a shape's size.

  `pixi run gz-compat-<lane> sleep-oracle` runs all scenarios (`--list`,
  `--scenario NAME`, `--settle N`, `--steps N`, `--tolerance X`). Each row
  shows how many bodies were resting when the mutation was applied, how far
  the mutation moved the reference run, the largest pose difference before
  and after the mutation, and the contact counts (with/without sleeping, at
  the first post-mutation step and at the end). A `MISMATCH` names what differed
  (`poses`, `contacts`): DART's deactivation changed what Gazebo sees, sometimes even
  with nothing asleep. A pose or contact point that is not finite (a run that
  blew up) always differs. A row with `nothing asleep`, or whose mutation was
  `inert` (moved nothing), is `UNEXERCISED`: it cannot reveal a missed wake.
  Mutations that cannot move a body at rest add a kick, so a stale parameter
  still shows; a third run applies the kick alone, and `moved` is how far the
  mutation moved the kicked run away from it. Four scenarios hold their body
  with a joint constraint (friction, a position limit, a servo) or drive it
  (a conveyor), and DART keeps such bodies awake, so they are marked
  `awake by design` and check only that deactivation leaves awake bodies
  alone; `joint_friction_added` and `joint_limits_tightened` apply friction
  and a limit to a sleeping joint instead. The run fails on any mismatch and
  on any unexercised row; `--allow-unexercised` accepts the latter, for
  builds that rarely sleep under gz-physics' filter (DART 6.19.4, and
  release-6.20 until Gazebo worlds can sleep). Changes that let bodies sleep in Gazebo worlds must run it
  without that flag and explain every mismatch. (`detach_joint` gives the
  welded model a second link: `AttachFixedJoint` on a one-link model, as
  gz-sim's `DetachableJoint` does, leaves an empty skeleton that DART counts
  as an awake body, which keeps every island awake.)

  gz-physics installs its own `BodyNodeCollisionFilter` subclass, under which
  DART 6.20 wakes resting bodies within a few steps. `--default-filter` swaps
  in DART's `BodyNodeCollisionFilter`, which shows how sleeping behaves for
  DART users, and skips the scenarios that need gz-physics' filter (masks,
  spawning and removal update it).
- `gz_raycast_probe` (Jetty only: `pixi run gz-compat-jetty raycast-probe`)
  casts single and batched rays through the dartsim plugin at upright and
  lying cylinders, a box, a sphere and the ground, and compares each hit
  point, normal and fraction with the exact intersection. gz-physics 9 casts
  rays itself over the ODE space of its `OdeCollisionGroup` subclass and the
  Bullet world of its `BulletCollisionGroup` subclass, so a DART change to
  those groups or to how ODE cylinders are built (native or mesh) changes
  what Gazebo's rays hit; the gz-physics suite only casts rays at spheres.
  With `--detector bullet` only batched against single rays is gated:
  Bullet's convex raycasts are approximate (also with DART 6.19.4), so compare
  those rows with another DART build. `--detector` takes only `ode` (the
  default) and `bullet`, the detectors gz-physics 9 casts rays with, and the
  probe stops when the plugin does not switch to the requested one.

`contact_benchmark --gz-preset` is the DART-only counterpart for benchmark
rows: it loads an SDF world with the collision setup gz-physics builds (see
`examples/contact_benchmark/GazeboPreset.hpp`) and reports contact demand
against the cap, starved pairs, sunk bodies and changed poses, keeping that
census out of its step times. Its SDF allowlist accepts one world (SDF
1.4–1.6), physics step size and `max_contacts`, scheduling-only real-time
settings, and flat models with names, static flags, plain six-number poses,
links, and fixed/revolute/prismatic/universal/ball joints. Links support
gravity flags, mass and a complete inertia tensor, inertial translation,
and box/sphere/cylinder/plane collisions; the preset uses Gazebo's default
world gravity `0 0 -9.8` and accepts only that explicit world gravity.
Joint axes support XYZ, damping, friction, springs, and position limits
containing zero, with explicit model-frame axes supported in SDF 1.5–1.6;
SDF 1.4 axis joints, closed joint chains, finite effort/velocity limits,
screw/revolute2 joints, and non-default frame or pose attributes are
rejected. Surface values must
retain the default material (`mu`/`mu2` 1, slip and restitution 0, zero
`fdir1` without a frame); collision masks must contain all `0xff` bits and
not exceed `0x7fffffff`. Light, scene, GUI, visual and passive sensor
contents may be ignored, but their embedded plugins are rejected; visual
geometry and poses must still be safely readable by DART. Only empty
standard world-level Physics, UserCommands and SceneBroadcaster
systems are accepted. Everything else, including includes, nested models,
world joints, frames, self-collision, rotated inertia, meshes, capsules,
ellipsoids, heightmaps and engine-specific physics overrides, is rejected
with the first unsupported XML path. Capsules and ellipsoids are absent
from DART's rigid SDF shape reader; screw pitch and revolute2 construction
differ from gz-physics. gz-sim's `3k_shapes.sdf` and all generated benchmark
worlds fit this subset.

The preset's per-pair contact limit comes from the active SDF physics
profile's `<max_contacts>` (20 when omitted). Released gz-sim selects the
first profile, even if a later one is marked `default`.
`--gz-pair-max-contacts N` overrides that value; the global contact cap
remains 10000.

## Not covered yet

- The lanes are not in CI (see `docs/onboarding/ci-cd.md`).
- `test-gz` should configure gz-physics twice once DART 6.20 passes gz-physics
  `ContactPropertiesCallback` again; until then it would turn red.
- Benchmark worlds beyond the generated ones: a box pile, a mesh ground, a
  heightmap (gz-sim's example downloads one from Fuel), a conveyor with a
  payload under a commanded belt, and 3k_shapes with a fixed-base arm under a
  joint controller. The sleep oracle covers the conveyor callback at the
  gz-physics level.
