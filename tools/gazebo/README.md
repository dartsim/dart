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

Release verification of DART 6.20 at `48eb618bb81` passed the unpatched
Harmonic, Ionic and Jetty gz-physics and gz-sim compatibility gates, with
results equal to or better than DART 6.19.5. The only gz-physics failure is
the accepted `UnsupportedRayIntersections` expectation: the `dart` detector
supports raycasts, while the upstream test still expects them to be
unsupported. The stored gate baselines remain DART 6.19.4; the release
comparison with DART 6.19.5 does not replace them.

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
| `GZ_COMPAT_MAX_SECONDS_SCALE` | 1 | finite, positive multiplier for `max-seconds` limits on slower hosts |

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

gz-sim `INTEGRATION_reset_sensors` is a known test race, not a DART
regression: right after `server.Run()`, `ResetFixture.HandleReset` reads the
last asynchronous air-pressure message (`reset_sensors.cc` lines 259 and 352)
and sometimes gets the previous 100 Hz sample, for example 100360.66 versus
100362.99 Pa, while its altitude check passes. In 20 interleaved Harmonic-lane
runs each, the 6.20.0 candidate and DART 6.19.5 both failed 4 times with
identical values. The lane reports it `FLAKY` when the retry passes and `NEW`
when both attempts fail (about 4% of lane runs). If `NEW` names only this
failure, rerun the lane before investigating; any other case or value pattern
in that test is a real failure. The fix belongs in gz-sim's test.

`compare` requires an existing, readable expected-failure file; only
`--write-baseline` may create a missing one. Timing multipliers
(`--max-seconds-scale`, including its environment default, and
`--max-seconds-factor`) must be finite and positive. Expected `max-seconds`
limits and recorded case times must be finite and nonnegative. Missing
GoogleTest result directories are errors, even when no failures are recorded.

`compare` stops with an error (exit status 2, as for missing or unreadable
inputs) when the run lacks a test that the expected-failure file names, a
`max-seconds` case whose test did not fail at the test level, or a CTest test
that the base ran (with a base), or any test listed by a
`test <suite> <ctest test>` line from the 6.19.4 run: a run that a
test filter, a renamed test or a test that stopped registering emptied would
otherwise pass and skip that coverage. Extra tests in the results are allowed.
`lane.sh` itself clears GoogleTest's `GTEST_*` variables, so an
inherited `GTEST_FILTER` or sharding cannot run only part of the suites.

To distinguish failures inherited from a change's base from new failures,
build and test the base once per lane as its own variant, then compare the
change with it. Use the target branch as the base:

```bash
git worktree add --detach .deps/gz-compat-base origin/main
GZ_COMPAT_VARIANT=base GZ_COMPAT_DART_SOURCE=.deps/gz-compat-base \
  pixi run gz-compat-ionic test
GZ_COMPAT_BASE_VARIANT=base pixi run gz-compat-ionic
```

For a backport, use `origin/release-6.20` instead of `origin/main`. Explain
any `BASE` or `REGRESSED` results; a base comparison does not make them
expected release behavior.

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

`baseline` writes every CTest test the run registered as a
`test <suite> <ctest test>` line, sorted by suite then name, in a final section
headed `# CTest tests of the baseline run (compare requires each one)`.
It also writes a `max-seconds` limit of twice the measured time for
the dartsim `StepWorld` case; it is the only gate here that catches a large
slowdown that still passes (it took 16 times longer with DART 6.20's trimesh
ODE cylinders). Entries carrying an inline
`# accepted: <reason>` comment are reviewed differences from 6.19.4 that a
maintainer decided are not DART regressions; `baseline` keeps them, and
`compare` prints them as `ACCEPTED` or `STALE`. Run
`pixi run test-gz-compat-tools` after changing `compare_failures.py` or the
world generator.

## DART C++ API inventory

The inventory is lexical and best-effort: it records uses it can resolve and
retains conservative candidates for unresolved receivers. Known limitations:

- Namespace using-directives and aliases are treated file-wide.
- Source fields are keyed by name across owners.
- Function-local type aliases are not collected.
- Function-local classes and their virtual overrides are not collected.
- DART headers are indexed globally; transitive include availability and per-source
  declaration visibility are not tracked.
- Destructor and managed-lifetime requirements are not recorded.
- Implicit and contextual conversion operators are not recorded.
- Structured-binding element types are not inferred.
- Preprocessor conditions are not generally evaluated.
- C++ template instantiation and overload resolution are not performed.
- Ignored in-tree build artifacts are scanned as Gazebo sources.
- Virtual destructor overrides are not recorded.
- Referenced macro parameters and replacement tokens are not recorded.
- Source-only inheritance chains beyond three hops can be missed.
- Used type-alias target definitions outside field layout dependencies are not recorded.
- Qualified member-function references without calls are not recorded.
- Dependent `receiver->template method<T>()` calls are not recognized.
- Base relationships of referenced DART types are not recorded.
- Trailing return types are not used to infer chained-call receivers.

Operator syntax is associated with declarations for `->`, `[]` and `()` when
the receiver type is known. The Gazebo build and test lanes remain the
authoritative compatibility check.

[`compat/dart_api_inventory.json`](compat/dart_api_inventory.json) records the
DART types, constructions, accessor and mutation calls, public and protected
data-member accesses, free functions, macros, and virtual overrides used by
the gz-physics dartsim plugin, its direct DART test code, and directly by
gz-sim. It includes source revisions, use counts, source locations, and
matching DART header declarations with signatures, constness, and virtual
status. Used members also retain their declared access; free-function entries
retain their declaration candidates. Referenced types retain `final_classes`
from their class dependencies,
so making a DART base final changes the inventory. Enum types and enumerator
declarations retain their normalized enum headers, including underlying types;
unscoped and anonymous enumerators are also indexed in their enclosing namespace
or class. Used enumerators retain initializer and preceding-value dependencies.
A construction, including an array declaration, lists the constructors and
their access, including private and deleted declarations, of the type, its
bases and owned non-static members, including class template specializations.
These are conservative candidates without overload resolution.
Construction helpers include `std::allocate_shared`, `std::make_optional` and
`std::make_from_tuple`, engaged optional `std::in_place` initializers, known
optional `emplace` calls, value-producing `static_cast` targets and placement `new`.
A Gazebo class derived from a DART class counts as a construction of that base
at its base clause and retains its dependencies' pure-virtual declarations.
For brace constructions of classes without declared constructors,
`brace_member_declarations` retains public non-static fields in declaration
order. This is lexical field evidence, without proving C++ aggregate eligibility.
Constructed `std::array` values also retain their value elements' constructor
dependencies, including nested arrays; literal zero extents and pointer elements
do not construct those elements. Unknown extent expressions remain conservative.
Constructed `std::tuple` and `std::pair` values retain owned value elements;
pointer and reference elements do not construct their pointees. Default-initialized
`std::variant` values retain construction dependencies of their first alternative,
with each value declarator checked independently in grouped declarations.
Count initializers
on `std::vector`, `std::deque`, `std::list` and `std::forward_list` retain their
value elements' constructor candidates, including count-plus-allocator arguments
and count arguments forwarded by standard factories. This dependency applies at
zero count too. Braced returns use their enclosing function's declared return
type for construction evidence.
Constructed classes' default field initializer declarations and their named
constant dependencies are retained in `default_member_initializer_declarations`.
Quoted literals in indexed field and constant initializers are retained in
`initializer_literals` as evidence, without parsing their contents as C++.
Used function declarations retain quoted literals from their return and parameter
types, defaults and exception conditions in `declaration_literals`. Function bodies
and comments remain excluded. Export and attribute spellings on used functions
are retained in `declaration_annotations` before macro expansion.
Language-linkage blocks are scanned in their enclosing namespace; used free
functions and namespace variables retain the quoted linkage name with their
declaration literals.
Union types retain `union_declarations`, and union fields retain their enclosing
`union_declaration`, including anonymous union blocks.
By-value declarations and `sizeof`/`alignof` operands retain non-static field
declarations in `layout_declarations`, in base and field order, including
value-owned subobjects and unnamed bit-field padding. Owned union fields also
retain their types' `union_declarations`. Inline field types retain alias targets and enum headers
in `type_alias_declarations` and `enum_declarations`; quoted alias targets retain
`declaration_literals`. Indexed constants in inline array extents appear in
`type_constant_declarations`. Known callees in field `decltype` expressions retain
their declarations in `declared_type_declarations`. Pointer and reference declarations preserve their
indirection; dereferenced layout operands retain the referenced object fields.
Referenced types retain normalized alignment specifiers and their type/constant
dependencies in `alignment_declarations`. Used declarations retain constants
named in parameter defaults in `default_argument_declarations`, including their
initializer dependencies and the constructors required by class-valued defaults. Class annotations, including export macro spellings,
are retained in `annotation_declarations` even when an export macro expands to
nothing on the scanned platform. Constants in `noexcept(...)` conditions and
named bit-field widths are retained in `exception_specification_declarations`
and `bit_width_declarations`. Named array extents in fields and parameters are
retained in `array_bound_declarations`, including initializer dependencies.
This records lexical evidence without computing C++
layout or evaluating constant expressions.
The lexical inventory retains declaration candidates for unresolved receivers
(`unresolved_count`) and overloads, so a member name on a non-DART object can
match a DART declaration. Bullet, ODE, and Assimp types (`bt`, `d`, and `ai`
prefixes) and unknown members of non-DART objects are not DART receivers.
Type lookup uses tracked scopes, from the innermost namespace or class,
including the class of an out-of-line member definition and the bases of
enclosing classes. A type alias is visible only in its file and in the files
that include it; aliases declared inside function bodies are not collected.
Chained source helper calls use return declarations from the receiver's class
and its bases. Source helpers with `auto` or `decltype(auto)` returns infer
receiver candidates from their return expressions, excluding nested functions
and lambdas. Sequence returns retain their DART element types through
begin/end iterators and their const and reverse variants. Iterator arrows and
dereferences preserve pointer elements as pointers.
Set-family returns also preserve their value or pointer elements through
iterators, aliases and range-for declarations. Map-family iterators preserve their
key and mapped values as pair elements, including lookup iterators and both
iterators returned by `equal_range`. Returned `std::pair` and `std::tuple`
values retain each element's receiver shape for `first`, `second` and `std::get`
projection by index or a known unique element type.
Standard forwarding calls, `std::as_const` and `std::optional` accessors retain
contained receiver shapes through value, pointer and array expressions. Returned
standard smart pointers preserve their wrapper types through `get()` and `lock()`
chains.
Source class aliases retain DART bases, and member lookup respects source declarations and
class using-declarations. Lambda parameters and initialized captures stay within
their lambda body; capture initializers use the enclosing scope.
Automatic declarations, including `decltype(auto)`, retain inferred return types.
Concrete `decltype` declarations infer receiver shapes from their unevaluated
expressions. Inferred free-function pointers preserve return shapes through
aliases and invocation, including namespace variables visible through source
includes and using-declarations; ordinary `auto` copies decay reference results.
Unary address expressions preserve built-in pointer shapes or known member
`operator&()` return declarations.
Range-for variables retain the range's element types within the loop body.
Control-statement declarations stay within the complete statement, including
`else` branches, and grouping parentheses preserve receiver types. Conditional
expressions retain both operand candidates without resolving their common type. C-style
casts use the target type, including pointer aliases and source-derived types.
Chained DART data-member accesses use the member's declared type.
Function and data declarations retain grouped names and each declarator's type;
data declarations retain array dimensions. Callable variables and data members
retain their declarations when invoked through qualified names, object receivers
or inherited fields, respecting member-name hiding. Grouped function pointers
retain each name, array shape and any trailing return type. Callback fields do
not construct their return types.
Qualified free-function references count even when they are not calls.
Macro entries also retain DART feature probes and mark whether the current
DART headers declare them (`declared_in_dart`).
Namespace-scope constants and variables retain their declarations in
`variables`; unqualified free-function calls also search the DART namespaces
associated with their argument types and bases. Friend function declarations
belong to the enclosing namespace, including those in private class sections.
Referenced class templates retain their parameter declarations, including
forward-declaration defaults. Direct DART includes are listed in `headers`
with their source-header availability, and attributed declarations remain
eligible for matching. DART free-function returns can also supply the type
of a chained member-call receiver.
`source_present` recognizes source headers and `.hpp.in` templates; availability
of CMake-generated umbrella headers is checked by the Gazebo build lanes.

The extractor does not evaluate preprocessor conditions. It drops only the
branches that a literal `#if 0` or `#if 1` rules out and scans every other
branch, so the inventory can list uses that only a branch inactive in a given
build makes. A use takes the types of every declaration that some build lets
it see, so a variable declared in a branch does not hide other declarations of
it from uses outside that branch. A type alias or Gazebo class defined
differently in alternative branches keeps one of its definitions.

With clean source checkouts, regenerate and check it from the DART root:

```bash
python scripts/gz_dart_api_inventory.py "$GZ_PHYSICS_SOURCE" --gz-sim "$GZ_SIM_SOURCE"
python scripts/gz_dart_api_inventory.py --check "$GZ_PHYSICS_SOURCE" --gz-sim "$GZ_SIM_SOURCE"
```

To refresh the recorded SHAs, fetch each source repository, check out the
chosen revision, and regenerate with both checkouts. Keep the gz-physics
revision aligned with the DART compatibility shim; refresh gz-sim from its
main branch. A check with only gz-physics retains the recorded gz-sim scan and
reports that it was not rescanned. Without a checkout, the command explicitly
reports a skipped check. Run `pixi run test-gz-compat-tools` after extractor
changes.

The `CI gz-physics` inventory job checks out both recorded Gazebo revisions
and runs the freshness command on every pull request and main-branch push.

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
  Steps must be positive integers; explicit contact limits and sampling
  intervals must be nonnegative integers (0 disables contacts or sampling).
  `--sunk-z` must be finite; negative heights are supported. These inputs
  reject trailing characters.
- `gz_sim_server_bench` runs a world in a `gz::sim::Server` and reports time
  per iteration; it fails when the server did not run every iteration (a
  world that did not load, or a server stopped by a signal).
  `pixi run gz-compat-<lane> bench-gz-sim` runs `3k_shapes.sdf` with its
  real-time factor set to 0 for 1000 iterations.
  Iterations must be a positive integer without trailing characters.
  The world filename must not be empty, and an explicit `--engine` must
  name an existing plugin file; otherwise gz-sim can run iterations using
  a default world or without physics.
- `pixi run gz-compat-<lane> worlds` writes the benchmark worlds derived
  from `3k_shapes.sdf` into `.deps/gz-compat/<lane>/worlds/`
  ([`bench/make_worlds.py`](bench/make_worlds.py)), all with real-time factor
  0: `3k_shapes.sdf`, `3k_shapes_drop5cm.sdf` (released 5 cm above the
  ground), `6k_shapes.sdf` (a second copy 4.5 m along +y) and
  `3k_shapes_pendulum.sdf` (plus a contact-free pendulum 500 m away, which
  keeps one body awake).
  Model poses transformed by the generator must contain six finite numbers.
- `gz_sleep_oracle` is the differential sleep oracle. Each scenario applies
  one state mutation the dartsim plugin exposes and runs the same script with
  DART's deactivation enabled and disabled, reaching DART's `World` through
  the plugin's `RetrieveWorld` feature: settle, mutate, step 500 more, then
  compare the published link poses step by step and the reported contacts
  (their counts on the first step after the mutation and at the end, and the
  final contact points, paired one-to-one so a repeated point must occur as
  often in both runs). The scenarios cover joint spring stiffness and
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
  `--scenario NAME`, `--settle N`, `--steps N`, `--tolerance X`). Settle and
  post-mutation step counts must be positive integers. Both this oracle and
  the raycast probe require finite, nonnegative tolerances; these numeric
  inputs reject trailing characters. Each row
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
  older builds that rarely sleep under gz-physics' filter. DART 6.20 sleeping
  changes must run it without that flag and explain every mismatch.
  Release verification at `48eb618bb81` matched 33 of 35 Ionic scenarios.
  The two differences are a detector-private contact-cap edit without
  notification and attachment sensitivity to the prior resting pose;
  DART 6.19.5 had 6 mismatches and 27 unexercised scenarios. These explanations
  do not suppress oracle failures. (`detach_joint` gives the
  welded model a second link: `AttachFixedJoint` on a one-link model, as
  gz-sim's `DetachableJoint` does, leaves an empty skeleton that DART counts
  as an awake body, which keeps every island awake.)

  gz-physics installs its own `BodyNodeCollisionFilter` subclass, under which
  DART 6.20 lets resting islands sleep since
  [#3632](https://github.com/dartsim/dart/pull/3632): the 3,000-shape world
  settles and all bodies sleep. Verify sleeping changes with that custom
  filter in place. `--default-filter` swaps
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
  Release verification at `48eb618bb81` matched all 26 Jetty ODE rays.
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
