# DART 7 Python Naming Rulebook

This rulebook derives the 7.0 Python name of every public `dartpy` member. The
maintainer has approved it. Given a current member, a reader finds its 7.0
spelling, finds that it is dropped, or finds which choice its area pull request
("area PR") records. Area PRs apply these rules, and the old-to-new mapping
generated from those PRs drives the codemod, the stubs, the generated
docstrings and the migration guide.

The [DART 7 Python-first API design](dart7_python_api.md) ("the design") owns
the surrounding rules: the package layout, the model vocabulary, reserved
names, the World surface, modes, arrays and transforms. Names that it fixes are
inputs here (R1).

The rules change bindings only. C++ names, signatures, defaults and
documentation stay as they are, and any Python-side difference, such as
validation that raises, lives in the binding. Where the best Python behavior
differs from C++, Python takes it: C++ parity is best effort and never blocks
the Python API.
[DART 7 Python and C++ Differences](dart7_python_cpp_differences.md) records
each difference in behavior that results.

## Reference code

The rules start from what users write to build a model, step it, read its
state, control it and inspect contacts:

```python
import dartpy as dart

world = dart.World(time_step=1e-3, gravity=(0, 0, -9.81))
arm = dart.io.load_model("arm.urdf")
hand = arm.add_link("hand", parent=arm.links["wrist"],
                    joint=dart.dynamics.WeldJointSpec())          # R32
hand.mass = 0.4                                                   # R6, R7
hand.add_collider(dart.dynamics.BoxShape(size=(0.1, 0.1, 0.05)))  # R32, R53
arm.self_collision_enabled = True                                 # R13
world.add(arm)                                                    # R33
world.compile()

q = arm.dofs.positions                                            # R14
X_WH = hand.world_transform                                       # R18
V_WH = hand.spatial_velocity                                      # R18, R20
v = hand.get_linear_velocity(offset=(0, 0, 0.05), expressed_in=hand)  # R18-R20
J = arm.jacobian(hand)                                            # R20, R22
M = arm.mass_matrix()                                             # R23

arm.dofs.forces = arm.gravity_forces()                            # R14, R23
arm.dofs["shoulder", "elbow"].positions = (0.3, -0.2)             # R15
world.step(10)

for contact in world.collision.contacts:                          # R27, R44
    if hand in (contact.collider_a.link, contact.collider_b.link):
        depth = contact.penetration_depth                         # R45
```

## Applying the rules

Derive a member's 7.0 name in this order. An earlier step wins over a later
one.

1. Scope (R1–R3): fixed names are kept, and deferred members wait for their
   milestone or are dropped at the API freeze.
2. Blanket drops (R4, R5): a dropped member gets no name.
3. Owner (R12, R38, R41, R54): choose the one public class that carries the
   member.
4. Kind, in this order: DOF state (R14–R17); collections, parts and
   building (R27–R36); property and `get_` pairs, frames and Jacobians
   (R18–R22); loaders, structures and options (R42–R44); properties
   (R6–R11); methods (R23–R26); callbacks (R46, R47).
5. Spelling: types and enums (R37, R39, R40), flags (R13), then the word
   rules (R45, R48–R52) on every token. A name derived from a live accessor
   wins over one derived from a field, and the field and any `*Spec` keyword
   take it.
6. Signature (R53–R56).
7. Budget, checks and collisions (R57–R59): for a class over its cap, go
   back to step 3 to move names into a collection or part, or to step 2.
   Then resolve same-name pairs and nouns used with two kinds.

R60 then maps the call sites.

These overlaps are settled:

- a Topology-class setter may still be a property (R6);
- per-DOF state lives only on DOF collections and `Dof` (R14, R15);
- per-DOF arrays (R14) and results the World keeps between steps (the last
  contacts) are properties; any other cached result whose size grows with the
  model is a method (R6);
- energies, the Lagrangian, momenta and wrenches that C++ computes on each call
  are methods named by bare nouns, with or without arguments
  (`kinetic_energy()`, `linear_momentum()`, `angular_momentum(pivot=…)`; R23);
  a value that a dynamics pass stores on the object (`body_force`) is a
  property (R6);
- a joint's relative Jacobian, which has a fixed size, is a property with a
  `get_` form, while link and multibody Jacobians, which grow with the model,
  are methods (R6, R22);
- structural edits of a multibody are methods (R34), not collection edits
  (R33).

If two rules still apply at one step, the more specific one wins. If neither
is more specific, the area PR asks the maintainer.

## Scope and mapping

**R1.** Names that the design or another maintainer decision fixes are kept,
except the error classes that R25 replaces with builtins. This rulebook also
ratifies these provisional names:

- `WorldState` and its fields, except `body_external_forces`, which R48
  renames `link_external_forces`;
- `Transform.identity()`, `inverse()`, `Rotation.from_axis_angle(axis, angle)`,
  `as_rpy()`, and the `matrix`, `translation`, `rotation`, `wxyz` and `xyzw`
  properties;
- `world.solver.primary_solver` and `secondary_solver`, the roles C++ documents
  for `setBoxedLcpSolver` and `setSecondaryBoxedLcpSolver`.

Every other existing name is provisional and is re-derived. That includes the
provisional `dartpy.errors` classes, interim snake_case names
(`is_sleep_candidate`), Python-only names (`setActuatorTypeForDof`,
`convertToTransformOf`, `static_convert`), the shipped `dartpy.viewer` names
and the spellings in the acceptance tests; each area PR updates the test lines
its rules change. When a decision cites a camelCase member
(`setPythonAnalytical`), it names a capability, and these rules give that
capability its spelling.

**R2.** M5 does not rename surfaces that later milestones replace:

- aspect and composite state (M3);
- the interim `dartpy.gui` module, together with the `dartpy.viewer` members
  that take or return its types (M6b).

Their mapping rows are deferred, and the codemod marks their call sites with a
`TODO(dart7)` note. A deferred member that its milestone has not replaced when
the API freeze is declared is dropped, because a milestone that lands in 7.x
may only add names. Before the freeze, the area PR moves shipped examples,
tutorials and demos off it (the `world_history.py` demo moves from composite
state to `capture_state` and `restore_state`), so R5 holds. The C++ recording
is not deferred; R5 drops it.

**R3.** The mapping is keyed by C++ member (qualified name and overload
signature), so a member bound on several classes maps once. Python-only and
interim names get rows keyed by their Python path. Each row records one
outcome and any argument renames or reorders. The outcome is one of:

- renamed;
- merged into another member;
- changed: same spelling, new signature or meaning (`World.step`,
  `World.clone`);
- dropped, with a hint when a replacement exists;
- deferred (R2).

## Drops

**R4.** These are dropped:

- plumbing: cache invalidation (`dirty*`, `needs*Update`), notifications,
  version counters, display lists, data variance, capacity hints (`reserve`)
  and C++ signals (`onNameChanged`);
- solver internals: the `ConstraintBase` step protocol, raw-pointer LCP
  `solve`, impulse dynamics, union-find, islands, sleep candidates, and the
  setters of flags the solver writes (`Skeleton.setResting`,
  `PointMass.setColliding`), whose getters stay read-only predicates (R13);
- aspect machinery: `has/get/set/create/remove/release<X>Aspect`,
  `*AspectProperties`, `duplicate*`, `match*`, and every `Composite` member
  except the state that R2 defers. Aspect values move to the `Collider` and
  `Visual` handles;
- self, pointer and lock accessors (`getPtr`, `getLockableReference`,
  `Skeleton.getSkeleton`, `BodyNode.getBodyNode`);
- type tags, type predicates and downcasts (`getType`, `getStaticType`,
  `isShapeNode`, `asShapeNode`). `isinstance` replaces them (R38);
- duplicates: `*Static` fast paths, per-instantiation copies, const and
  non-const twins, alias paths, `export_values` aliases, and deprecated C++
  members and overloads;
- what the Python standard library already covers: the string and timer
  helpers and the log emitters of `dart.common` (`toUpper`, `tic`, `Stopwatch`,
  `warn`) and `dart.math.Random`. dartpy's own tests reach the emitters through
  `dartpy._testing`, and `dart.common.flush_logs()` stays, because only it
  drains DART's queued diagnostics;
- members Python cannot use (`void*`, `std::ostream*`, unregistered types).
  Redesigning one is a defect PR, unless a rule here redesigns it (R42);
- `getConfiguration`, replaced by `capture_state` and the DOF arrays;
- result builders (`CollisionResult.addContact`).

Later rules drop further members.

**R5.** Members that the design's
[World section](dart7_python_api.md#world-modes-and-mutation-classes) leaves
unbound are dropped, with the hint that section names. The recording accessor,
and `bake()`, which feeds it, hint at `world.capture_state()` snapshots, and
at `dart.Recorder` once it ships. An area PR may drop any other member it
finds no use for, and records why. A member that shipped examples, tutorials,
demos or acceptance tests use is not dropped until its replacement exists, and
the area PR moves those uses to it. Dropping a member never changes C++.

## Accessors and flags

**R6.** A getter, meaning an argument-free C++ accessor named `get*`, `is*`,
`are*` or `has*`, is a property when all three of these hold:

- it returns one of:
  - stored state;
  - a handle: a related object, part, sub-object or collection;
  - per-DOF state (R14);
  - a kinematic quantity in its world form (R18);
  - a fixed-size value (a scalar, string, enum, small vector or matrix,
    `Transform` or `Rotation`) derived in at most linear time;
- it has no effect beyond filling DART's lazy caches;
- it cannot fail on a valid object.

Anything else is a method (R22–R26). That includes any cached result whose
size grows with the model (the mass-matrix family, gravity and Coriolis
forces, link and multibody Jacobians) and anything that does I/O. On a value
type, an argument-free fixed-size result is a property whatever its C++ prefix
(`BoundingBox.computeCenter()` becomes `center`).

The mutation class does not change the outcome. A Topology property such as
`collider.shape` or `joint.actuator` raises `RuntimeError` with the
`ModelFrozenError:` token (R25) when its World is compiled and no `edit()`
block is open, like every Topology mutator. Every setter's docstring names its
mutation class.

**R7.** A property is writable when C++ has a setter that takes just that
value, unless the class is a value or result type (R10). Forms pair by
meaning, not by spelling: every get, set, `is*`, `are*`, `enable*`/`disable*`,
`show`/`hide` and `setActive()`/`setInactive()` form of one value is one
property (`setLimitEnforcement` and `areLimitsEnforced` become
`limits_enabled`). A family of setters, one per enum value, pairs with its
getter into one writable enum property (`setXYPlane`, `setYZPlane`,
`setZXPlane` and `getPlaneType` become `plane_type`). Assigning a value that
needs more data (`PlaneType.ARBITRARY`) raises, and that value has its own
`set_<x>(*, …)` (`set_arbitrary_plane`).

A property round-trips. Where the C++ getter and setter disagree, the two
values get two names: `SoftBodyNode.setMass` sets the rigid part, which becomes
`mass`, while `getMass` includes the point masses, which becomes the read-only
`total_mass`. `multibody.mass` sums the links' `mass`, as C++ does, so `mass`
never includes point masses. A subclass may make a property writable
(`SimpleFrame.relative_transform`), but never read-only.

**R8.** Setter return values are dropped. So are setter forms without
arguments that restore a default; the docstring states the default, and R17
covers explicit resets.

A setter flag takes its C++ default. A flag that is itself state becomes its
own property (`DegreeOfFreedom.setName(name, preserveName)` becomes `dof.name`
and `dof.name_preserved`, and assigning `dof.name` sets `name_preserved`, as
the C++ default does and the docstring says). Any other flag survives only as
`set_<x>(value, *, flag=…)`, and only with a use case for a non-default value.

Assigning a `name` that the owner already uses raises `ValueError` with the
`NameConflictError:` token (R25), as `world.add` does. The binding checks
before calling C++, so C++ still makes names unique for its own callers.

**R9.** A write-only setter becomes a property when C++ has a matching getter,
bound or not, unless R12 or R18 rules that property out
(`FreeJoint.setRelativeTransform(T)` stays `set_relative_transform(T)`).
Without a C++ getter, it becomes `set_<x>(value)`, or a constructor keyword
when only construction needs it.

Setters that fan out to children and store nothing themselves
(`BodyNode.setColor`, `MetaSkeleton.setAlpha`) are dropped; appearance is set
on each visual. A visual's legacy color stays as `visual.rgba`, the lowest
step of the design's material precedence. Migrated code then keeps its
appearance on meshes that carry their own materials.

**R10.** A property returns either a live handle or an immutable value: a
read-only array copy, a tuple, a `Transform`, a `Rotation`, or an immutable
value type such as `Inertia`. Value and result types have no setters or
writable fields (`Inertia.setMass`, `BoundingBox.setMin`, `Contact` fields).
Editing a returned value therefore raises instead of being silently lost, and a
change builds a new value and goes through the owner
(`link.inertia = Inertia(mass=…, local_com=…, moment=…)`).

A property never takes a selector and returns something else:
`world.solver.primary_solver` takes and returns a solver. An enum that selects
a type is a constructor or `WorldConfig` keyword named `<role>_type`
(`WorldConfig(primary_solver_type=SolverType.PGS)`), so the role's own name
always holds an object (R52).

**R11.** Each stored value is held by one property, in its stored form. A
quantity that the design's mutation-class table names keeps its own property
even when it is part of another value (`link.mass` and `link.local_com` beside
`link.inertia`). Where C++ also sets a stored 6-D value by its linear and
angular halves, the halves are the properties, as the design prefers named
3-vectors: `ErrorMethod.setLinearBounds` and `setAngularBounds` become
`linear_bounds` and `angular_bounds`, and the 6-D `setBounds` is dropped.

Other views of a stored value are dropped:

- an encoding or a part of it: RGB beside RGBA, radii beside an ellipsoid's
  stored diameters, `ShapeNode.getRelativeRotation` beside
  `relative_transform`;
- older versions: `getMeshUri` and `getMeshPath` beside `getMeshUri2`, whose
  name passes to `uri`.

Structured values are set whole:

- paired bounds as one `(lower, upper)` tuple (`position_limits`);
- vectors, tensors and poses as one array or value. Setters that take the
  components as separate scalars (`setGravity(x, y, z)`,
  `setMomentOfInertia(Ixx, …)`) are dropped.

A setter that takes several values, with no getter for their combination,
becomes a keyword-only `set_<stem>(…)` named from its C++ stem
(`setNormalAndPoint(normal, point)` becomes
`set_normal_and_point(*, normal, point)`). A setter that also resets other
state becomes a verb (`Problem.setDimension` becomes `resize(n)`).

**R12.** Each piece of state has one public path, on its owner or on one of the
owner's sub-objects (R44):

- `world.time_step` and `world.gravity`, with the solver copies dropped. A
  multibody keeps its own `gravity` and `time_step`, which its dynamics
  computations read; `world.add` and the World's setters overwrite them, as
  their docstrings say;
- `world.collision.detector`, not the solver's reference to the same object;
- the child link's `relative_transform`, not `Joint.getRelativeTransform`;
- World membership, not `ConstraintSolver.addSkeleton`.

**R13.** Boolean names follow the kind of flag:

- A C++ name with check, enforcement, mode, use, enable or disable is a
  settable feature switch, `<feature>_enabled`, and those words are dropped.
  The getter supplies the feature's noun and its number (`limits_enabled`,
  `self_collision_enabled`, `gravity_enabled`, `coupler_constraint_enabled`;
  the field `mEnableAllHits` gives `all_hits_enabled`). A collision check whose
  name has no other collision word replaces "check" with `collision`
  (`setAdjacentBodyCheck` becomes `adjacent_link_collision_enabled`).
- Any other settable adjective or participle keeps its word and its C++
  polarity (`active`, `hidden`, `collidable`, `mobile`, `name_preserved`). A
  boolean named by an imperative or a noun phrase keeps its words
  (`sort_by_closest`, `randomize_constraint_order`, `allow_simulation`).
- `is_<adjective>` and `has_<noun>` are reserved for read-only predicates
  (`is_compiled`, `is_cyclic`, and `is_sleeping` for `Skeleton.isResting`).
- A predicate that takes arguments is a method phrased as a question
  (`descends_from(frame)`, `has_actuator_type(…)`). Membership tests use `in`
  (R27).
- A check that returned a bool and printed (`Inertia.verify`,
  `Joint.checkSanity`) becomes the method `is_valid(…)`, which never prints.

Result types define no `__bool__`. Today `bool(result)` is always true, so
adding `__bool__` would silently change every `if result:`, and the codemod
cannot find those lines.

## DOF state

**R14.** Per-DOF quantities live in two places. On DOF collections
(`multibody.dofs`, `joint.dofs` and selections) they are plural `(n,)` arrays,
or tuples for enum values. On `Dof` they are scalars. The list is exhaustive:

- `positions`, `velocities`, `accelerations`, `forces` and `commands`;
- `position_targets` and `velocity_targets`;
- `initial_positions`, `initial_velocities`, `rest_positions`,
  `spring_stiffnesses`, `damping_coefficients` and `coulomb_frictions`;
- the `(lower, upper)` limits `position_limits`, `velocity_limits`,
  `acceleration_limits` and `force_limits`;
- `actuator_types`, each DOF's type beside the joint-wide
  `joint.actuator_type`;
- the read-only `constraint_forces`, entries of the multibody's generalized
  constraint force vector.

Read-only per-DOF predicates are scalars on `Dof` only (`dof.is_cyclic`,
`dof.has_position_limit`).

Reading returns read-only copies. Assigning writes every DOF in the
collection. It accepts a sequence of matching length, or a scalar that
broadcasts, as in `WorldBatch`. A quantity whose unit depends on a mode keeps
one name, and its docstring gives the unit for each mode (`commands` under
each `ActuatorType`).

**R15.** Per-index accessors (`Joint.getPosition(i)`,
`MetaSkeleton.setForce(i, f)`, `Joint.setDofName(i, …)`) become
`dofs[i].<quantity>`.

Index-subset overloads become selections. Indexing a DOF collection with a
slice, or with a sequence of indices or names, returns a selection that has
the same properties (`arm.dofs[[0, 2]].positions = q`). A name that the
collection lacks raises `KeyError` listing the valid names, as R27 says.

Loops that must not allocate use `dofs.get(field, *, indices=None, out=None)`
and `dofs.set(field, values, *, indices=None)`. They mirror `WorldBatch.set`,
and `field` names an array property. Named collections define no `get(name)`,
so `dofs.get` raises `ValueError` listing the valid fields for any other
string.

A `Joint` exposes per-DOF data only through `joint.dofs`. By default, the `Dof`
of a single-DOF joint carries the joint's name (`arm.dofs["elbow"]`).

**R16.** Index names:

- An object's position in its owner's primary collection is `index`
  (`link.index`, `joint.index`, `dof.index`).
- Other index spaces are `index_in_<space>` (`dof.index_in_joint`,
  `link.index_in_tree`), and the containing tree is `tree_index`.
- A list of indices is `<item>_indices` (`InverseKinematics.getDofs()` becomes
  `dof_indices`).
- `getIndexOf(x, warning)` becomes `collection.index(x)`, which raises
  `ValueError`.

**R17.** `reset_<quantity>()` restores the initial value, or zero where there
is no initial value: `dofs.reset_positions()`, and `dofs.reset_forces()` for
both `MetaSkeleton.resetGeneralizedForces` and `Joint.resetForces`.
`clear_<noun>()` discards accumulated entries
(`link.clear_external_forces()`). Singular or plural follows the host
(`dof.reset_position()`), and a subset is reset through a selection.

## Frames and Jacobians

**R18.** A kinematic quantity of a frame, link or multibody gets two members,
as `world_transform` and `get_transform(relative_to=…)` do:

- a property for its world form, relative to and expressed in the world;
- a `get_<quantity>(…)` method for the parameterized form (R23).

This covers `linear_velocity`, `angular_velocity`, `spatial_velocity`,
`linear_acceleration`, `angular_acceleration`, `spatial_acceleration`, `com`,
`com_linear_velocity`, `com_linear_acceleration`, `com_spatial_velocity` and
`com_spatial_acceleration`. The 6-D spatial quantities get the same pair as
the 3-D ones.

Setters that take a frame are `set_<quantity>(value, *, relative_to=None, …)`.
A property is writable only in its default frame. `FreeJoint.setSpatialMotion`,
whose five frame arguments R19 cannot name, is dropped; its hint calls
`set_transform`, `set_spatial_velocity` and `set_spatial_acceleration` in that
order, as its C++ documentation does.

**R19.** Frame arguments of kinematic queries and setters are keyword-only.
Each has one name, chosen by meaning, not by C++ spelling:

- `relative_to=`: the frame a pose or motion is measured relative to
  (`withRespectTo`, `relativeTo`). It takes a `Link` or `EndEffector` where
  C++ takes a node.
- `expressed_in=`: the frame whose axes a result's components use
  (`inCoordinatesOf`), and nothing more. DART re-expresses a result by
  rotating it, so `expressed_in=` never moves the point that a velocity or a
  moment refers to.
- `offset=`: a point fixed in the queried frame and given in that frame's
  coordinates (`offset`, `localOffset`), unless an R56 flag says otherwise.
  An angular momentum's `pivot=`, the point that the moment is taken about,
  is given the same way.

A joint wrench's frame is not `expressed_in=`:
`Joint.getWrenchToChildBodyNode(withRespectTo)` also moves the moment's
reference point to that frame's origin. The joint area PR gives that argument
its own keyword, documented as the moment about the frame's origin, in the
frame's axes.

A frame that an object stores takes its property's name: the `refFrame` of
`SimpleFrame(…)`, `clone` and `copy` becomes `parent_frame=`, and
`TaskSpaceRegion.setReferenceFrame` becomes `reference_frame`.

**R20.** An omitted frame keyword means the world, with one exception: poses
and positions follow `X_AB` notation, so `get_transform(relative_to=a)` and
`get_com(relative_to=a)` express their result in `a` unless `expressed_in=`
names another frame. No default uses the queried object's own coordinates,
even where the C++ form without a frame does:

- `getSpatialVelocity`, `getSpatialAcceleration`, `getJacobian` and
  `getJacobianSpatialDeriv`, each with or without an offset;
- `BodyNode.getCOMSpatialVelocity()` and `getCOMSpatialAcceleration()`;
- `BodyNode.getLinearMomentum()` and `getAngularMomentum(pivot)`, so the
  momenta take an optional `expressed_in=`
  (`angular_momentum(pivot=…, expressed_in=…)`).

For today's calls that give no frame, the codemod passes the frame they use,
so migrated code keeps its numbers. For the forms above it passes
`expressed_in=`: the receiver, or the link or end effector that a multibody's
Jacobian targets (`frame.get_spatial_velocity(expressed_in=frame)`,
`link.get_com_spatial_velocity(expressed_in=link)`). The docstrings of these
queries show `expressed_in=link` for the own-coordinates form. In world axes,
`jacobian_spatial_derivative()` is the own-coordinates derivative rotated into
the world, not the time derivative of `jacobian()`;
`jacobian_classic_derivative()` is that time derivative, and both docstrings
say so.

`None` for one of R19's frame arguments means what omitting the keyword
means: the world, or, for the `expressed_in=` of a pose or position, its
`relative_to=` frame. A stored frame such as `frame.parent_frame` takes the
world for `None`. Python callers pass `None` to mean the default, so omitting
a frame argument and passing `None` always mean the same.

**R21.** A few frame arguments default to a documented null meaning instead of
the world. Omitting one and passing `None` both keep that meaning, and the
docstrings state it:

- joint wrenches: the joint frame;
- task-space region references: the target's parent;
- `descends_from(None)`: true;
- builder `parent=None`: makes a new root;
- `Chain(start=None)`.

None of these is one of R19's query frames. A joint wrench's frame also moves
the moment's reference point (R19), and the others choose a structure, not the
axes of a result.

C++ documents no null parent for `Detachable.setParentFrame`, so
`frame.parent_frame = None` takes the R20 meaning and attaches the frame to the
world.

**R22.** Link and multibody Jacobians are methods, with the same names on two
kinds of class:

- On `Link` and `EndEffector`, the columns are the dependent DOFs.
- On `Multibody` and `MetaMultibody`, the link or end effector is the first
  argument and the columns are the multibody's `dofs`.

The names are nouns (R23): `jacobian`, `linear_jacobian` and
`angular_jacobian`, plus `com_jacobian` and `com_linear_jacobian` on
multibodies. Time derivatives append `_derivative` and keep DART's spatial and
classic forms (`jacobian_spatial_derivative`, `jacobian_classic_derivative`,
`linear_jacobian_derivative`).

A joint's relative Jacobian has a fixed size, so it follows R23's pair rule:
the property `joint.relative_jacobian` holds it at the current positions,
`joint.get_relative_jacobian(*, positions=None)` evaluates it at other
positions, and the property `joint.relative_jacobian_derivative` holds its
time derivative. Every Jacobian method accepts `out=`.

## Methods

**R23.** Values are nouns and actions are verbs. Four rules apply to every
public class:

1. A method that returns a value is named for that value, as a noun phrase:
   `mass_matrix()`, `gravity_forces()`, `kinetic_energy()`,
   `jacobian(offset=…, expressed_in=…)`, `com_jacobian()`,
   `position_differences(q2, q1, /)`, `distance(other)`. No method that
   returns a value starts with `compute_`, `calc_` or `eval_`.
2. `get_<x>(…)` exists only as the parameterized form of a property `x`,
   `world_x` or `relative_x` on the same class that holds the same quantity in
   its default form: `world_transform` with `get_transform(relative_to=…)`,
   `linear_velocity` with `get_linear_velocity(…)`, `com` with
   `get_com(relative_to=…)`, `relative_jacobian` with
   `get_relative_jacobian(positions=…)`. R18 and R22 list the pairs. The
   process-wide static `get_<x>()` and `set_<x>(value)` pairs of R41, and the
   members that R2 defers, are the only other `get_` names. A lookup that is
   not a collection is a noun (`occupancy(point)`).
3. `compute_<x>()` is only an action that runs an algorithm, stores its
   results in the model and returns `None` (R26). The stubs annotate it as
   returning `None`, so a type checker such as mypy flags
   `x = multibody.compute_forward_kinematics()`. No property or keyword starts
   with `compute_`: a flag that C++ names with compute becomes an R13 feature
   switch (`<feature>_enabled`), and a driver's flags use `update_<x>`
   (`update_transforms=`).
4. A noun has one kind across all classes and all kinds of member: it is an
   attribute everywhere (a property or a field) or a callable everywhere (a
   method or a static). `link.inertia` is a property that holds the full
   `Inertia`, so the shape helper that returns the 3x3 moment for a mass is
   `shape.moment_of_inertia(mass=…)`, not `shape.inertia(mass)`. Energies are
   methods on both `Link` and `Multibody`. A method that a Python protocol
   defines keeps its protocol name and gives its noun no kind, so the
   property `link.index` and the sequence method `multibody.links.index(link)`
   coexist (R16, R53).

The rules have one limit: an operation applied to its arguments keeps its
verb, even when it returns a value. That covers `step`, `solve`,
`find_solution`, `raycast`, `collide`, `resolve`, the `from_`, `as_` and `to_`
forms (R41) and overridable callbacks (R46). Value types name an operation
that returns a new value by its result, as the design's `inverse()` does
(`Inertia.transformed(…)`). Rule 1 also leaves two kinds of method to their
own rules: predicates, which R13 phrases as questions (`is_valid(…)`,
`descends_from(frame)`), and building and lifecycle methods, which keep their
verbs (R32–R36: `add_ik()`, `create_collision_group()`, `clone()`).

Docstrings carry the verbs. A method's docstring is a command, as PEP 257 asks
("Return the mass matrix."), and a property's docstring is a noun phrase ("The
world transform."). Docstrings also state what a name cannot: a property that
costs more than constant time says so (`multibody.com` is linear in the number
of links), and a property or method that returns a copy says so.

No `get_` or `compute_` aliases ship for renamed value methods; the codemod
migrates their call sites (R60). The acceptance tests' provisional
`compute_inverse_mass_matrix()` becomes `inverse_mass_matrix()` when its area
lands.

Python's own guidance backs these rules. PEP 8 asks to avoid properties for
computationally expensive operations, so a call's parentheses, not a prefix,
tell the reader that work is done. PEP 257 puts the verb in the docstring. The
Python Programming FAQ says that a method that mutates an object returns
`None`, which rule 3 keeps visible in the name. A property cannot take
arguments, so the parameterized form of a property needs a method, and `get_`
names it.

**R24.** Methods that return arrays accept `out=`, except overridable methods
(R46). Methods that fill a result object accept `out=` of that type, as
`capture_state(out=…)` does, and `out=` overwrites that result in place.

C++ output and in-out parameters become return values. Several outputs form
one read-only result with named fields, including any status that reports an
outcome (`ik.find_solution()` returns `success` and `positions`). Outputs that
C++ leaves on the object to be read later stay read-only properties
(`GradientDescentSolver.getLastNumIterations` becomes `last_iteration_count`).

**R25.** Errors raise the builtin exception that names the failure. Where a
DART rule rejects the call, in C++ or in the binding, the message starts with
a stable token that names the rule, and tests match that token
(`RuntimeError("ModelFrozenError: …")`):

- `ValueError` for malformed input, an invalid registration, a taken name or
  a checkpoint that does not fit the World it is restored into;
- `KeyError` for a failed name lookup, with a message that lists the valid
  names;
- `FileNotFoundError` for a missing file, built from errno as
  `OSError(errno, strerror, path)` builds it, so the path is its `filename`;
- `RuntimeError` for an operation that the current state rejects, such as a
  Topology edit outside `edit()`;
- `TypeError` for an argument of the wrong type, and `IndexError` for an index
  out of range.

A `bool`, `None` or sentinel return that signals bad input or a missing
resource becomes one of these exceptions. Loaders raise `FileNotFoundError`
for a missing file and `ValueError` for invalid content; the binding tells the
two apart itself, because a null C++ result does not say why loading failed. A
return that reports an expected outcome (convergence, existence, collision)
stays a return value. A setter that C++ logs and ignores on bad input
(`World.setTimeStep(-1)`) gets a binding check that raises `ValueError`. Print
and warning flags (`printWarnings`, `warning`, `print`) are dropped.

A dartpy exception class exists only for a condition that callers handle at
runtime instead of fixing their code. It subclasses exactly one builtin and has
a message-only constructor: `WorldBusyError(RuntimeError)`, and the rendering
errors `RenderingUnavailableError(RuntimeError)` and
`AssetsNotPrefetchedError(RuntimeError)`. No DART base class exists.
`DartError`, `OperationError`, `ValidationError`, `NotFoundError`,
`ModelFrozenError`, `CheckpointMismatchError` and `NameConflictError` are
dropped; their conditions raise builtins, and the last three names stay as
message tokens (`ModelFrozenError:`, `CheckpointMismatchError:`,
`NameConflictError:`). PEP 8 asks for exception hierarchies designed around
the distinctions that catching code needs, and callers branch on a type only
for conditions like these.

**R26.** Drivers that write their results into the model keep their C++ verbs
and return `None`. `compute_forward_kinematics`, `compute_forward_dynamics`
and `compute_inverse_dynamics` are R23's `compute_` actions, and
`integrate_positions` keeps its own verb. Each docstring says where the
results are stored (`compute_inverse_dynamics()` stores the joint forces in
`dofs.forces`). Their flags become keyword-only:
`compute_forward_kinematics(*, update_transforms=True, update_velocities=True,
update_accelerations=True)`. These drivers serve the full dynamics surface,
not World stepping.

## Collections

**R27.** A family made of `getNumX`, `getX(index)`, `getX(name)`, `getXs()`,
`hasX(x)` and `getIndexOf(x)` becomes one collection attribute, named by the
plural (`child_links`, `end_effectors`, `colliders`, `contacts`,
`equality_constraints`).

- Named items form a named collection, as the design defines it. A missing
  name raises `KeyError` with a message that lists the valid names, and `in`
  accepts an item or a name.
- Unnamed items form a read-only sequence that raises `IndexError`, unless R33
  gives it `add`, `remove` and `clear`.

Both kinds support `len`, iteration, indexing, `in` and `.index(x)`; a set of
pairs supports only `len`, iteration and `in`. Collections of model objects
are live views of membership. Sequences in results, and results the World
keeps between steps (`world.collision.contacts`), are tuples of read-only
values (R44).

Collection types, and sub-object types with no C++ class of their own, are
public on purpose; they are not R38's generated helpers, and users cannot
construct them. They are named `<Item>Collection` or `<Owner><Role>`
(`LinkCollection`, `WorldCollision`) and live in the item's or owner's module,
so the stubs can name them.

**R28.** A count is `len(owner.<plural>)`, with these exceptions:

- `num_dofs` exists on every class that has `dofs`, because it sizes arrays;
- `Multibody` and `MetaMultibody` keep `num_links` and `num_joints`, as in the
  design's example;
- an event counter, statistics fields included, is `<event>_count`
  (`step_count`, `iteration_count`);
- a count that is a setting or a field is `<x>_count` (`Contact.numBasis`
  becomes `basis_count`), except caps, which are `max_<x>`
  (`getNumMaxIterations` and `maxNumContacts` become `max_iterations` and
  `max_contacts`), and `World.getNumSimulationThreads`, which becomes the
  design's `threads=` keyword and `world.solver.threads` (R12).

**R29.** A view whose items may share a name, such as a `MetaMultibody` that
spans several multibodies, raises `KeyError` for a shared name, with a message
that lists the items sharing it, instead of picking one. Name lookups that
return lists (`getBodyNodes(name)`) are dropped.

**R30.** Unnamed geometric sets (vertices, points, faces, sphere centers) are
read-only `(N, k)` array properties, replaced by assigning a whole array.
Per-element methods survive only where they carry structure
(`add_vertex(position, *, parent=…)` returns the new index). Sets of objects,
such as point masses, are sequences (`soft_link.point_masses[i]`).

**R31.** Per-tree lookups become collections indexed by tree (`root_links[i]`,
with `root_link` for tree 0 and `len(root_links)` for `getNumTrees`). Per-tree
computations take a keyword-only `tree=`, which `Multibody` adds by declaring
the method again (R54; `mass_matrix(tree=i)`). Other per-tree overloads
(`getNumEndEffectors(treeIndex)`, `getTreeBodyNodes(treeIndex)`) are dropped;
callers filter by `tree_index` instead.

## Building and lifecycle

**R32.** `add_<part>(…)` builds an owned part and returns it. The `*Spec`s
passed in pick the joint and link types, never the method name:
`add_link(name, *, parent=…, joint=<Joint>Spec(…), link=LinkSpec(…) or
SoftLinkSpec(…))` replaces `createRevoluteJointAndBodyNodePair` and its
siblings. Element adders return the new element's index (R30), and
accumulators such as `add_external_force` return `None`.

`create*` and `spawn*` methods are mapped by what happens to the result:

- ones that build owned children become `add_<part>`;
- ones that build an object the receiver does not keep become `create_<x>(…)`
  (`detector.create_collision_group()`);
- link-side copies of multibody builders
  (`BodyNode.createFreeJointAndBodyNodePair`) are dropped.

`Link`, `Joint`, `Dof`, `Collider` and `Visual` have no public constructor.
Builders and loaders create them. When one node carries both a `Collider` and
a `Visual`, removing one keeps the node while the other remains, so
`link.colliders.clear()` and `link.visuals.clear()` together replace
`removeAllShapeNodes()`.

**R33.** Membership changes depend on the owner:

- A container, whose members are what it is (`CollisionGroup`,
  `CompositeCollisionFilter`), adds, removes and clears them itself
  (`group.add(collider_or_link)`, `composite_filter.add(filter)`).
- Any other owner uses one collection per kind of member
  (`world.constraints.add(…)`, `problem.equality_constraints.add(f)`,
  `link_filter.ignored_pairs.add((link_a, link_b))`).

`add(x)` returns `x`, `remove(x)` returns `None`, and `removeAll*` becomes the
receiver's `clear()`. List forms (`addShapeFrames(frames)`,
`ConstraintSolver.addSkeletons`) are dropped; callers loop. The design fixes
World membership (`world.add`, `world.remove`), and it has no `clear()`.

**R34.** Structural edits are methods of the part they change:

- `link.move_to(parent, *, joint=None)`;
- `link.copy_to(parent, *, recursive=True)`;
- `link.split(*, name)`, which returns a new `Multibody` holding the subtree.
  It also replaces `remove(name)`, which returned the removed subtree;
- `multibody.remove(link)`, which discards a subtree.

All of these are Topology edits.

**R35.** `clone(*, …)` returns an independent object. Each class has one
`clone`, and retargeting arguments are keyword-only (`cloneMetaSkeleton`,
`cloneChain` and `cloneWholeBodyIK` all become `clone`).

A C++ `copy(other)` that copies into the receiver becomes `copy_from(other)`,
where the area PR keeps it, so no method named `copy` copies into its
receiver. Value types return new values: `Inertia.transformed(…)` stays, and
the in-place `transform(…)` is dropped.

**R36.** An optional part is a read-only property that is `None` when the part
is absent (`link.ik`, `end_effector.support`). It comes with `add_<part>(…)`,
which raises if the part exists, and `remove_<part>()`. `createIfNull`
arguments, `getOrCreate*` methods and `has<Part>()` predicates are dropped;
users write `link.ik or link.add_ik()` and `end_effector.support is not None`.

A `SimpleFrame` carries one shape, so its `visual` and `collider` are optional
parts that share that shape (`frame.add_visual(shape)`,
`frame.add_collider(shape)`).

## Types, modules and static methods

**R37.** Classes keep their C++ names, including acronym casing, after the
whole-token vocabulary substitutions (R48) (`SoftLink`, `ReferentialMultibody`,
`LinkCollisionFilter`, `DARTCollisionDetector`, `WholeBodyIK`). C++ type
suffixes are removed (`ErrorMethod_t` becomes `ErrorMethod`), and a template
instance takes its template's name (`HeightmapShapef` becomes
`HeightmapShape`). New types are named `<Concept><Kind>` (`RevoluteJointSpec`,
`RaycastResult`).

The top level re-exports exactly the design's allowlist. Every other public
type lives in the lowercase submodule of its C++ namespace (`dart.dynamics`,
`dart.collision`, `dart.constraint`, `dart.simulation`, `dart.math`,
`dart.common`, `dart.optimizer`), with three exceptions:

- `dart::utils` becomes `dart.io` (R42);
- a new type lives in the module of the area that owns it
  (`dart.dynamics.RevoluteJointSpec`, `dart.simulation.WorldConfig`,
  `dart.sensors`);
- the interim `dartpy.gui` is deleted (R2).

**R38.** These types are private: the generated helpers, meaning template
instantiations and binding-internal bases (`GenericJoint_*`,
`TemplatedJacobianBodyNode`), and `Entity`, `Detachable`, `Node`,
`JacobianNode`, `ShapeFrame`, `ShapeNode`, `CollisionObject`, `ZeroDofJoint`,
`Composite`, `Subject` and `Observer`. Their members are defined once, on the
most basic public class that inherits them (`Frame`, `Link`, `Joint`). Where
several public classes inherit one private base with no public class between
them and `Frame` (`Link` and `EndEffector` from `JacobianNode`), each defines
those members, and R54 and R59 treat the definitions as one member. A member
that returns a private type is dropped unless R48 names its public
replacement (`Frame.getChildEntities` is dropped).

Each public class takes its public C++ base as its Python base, so its method
resolution order holds its public C++ bases and
`isinstance(revolute_joint, dart.Joint)` holds. The area PR's bindings re-base
any class whose primary C++ base is a private type, and bind the private
base's members on that public base. Only where a binding spike shows that a
class cannot take its public base does its public base get a custom instance
and subclass check (`__instancecheck__` and `__subclasscheck__` on the base's
type) that accepts that class.

**R39.** A type that appears in the public signatures or fields of only one
public class and its subclasses is nested in that class, wherever C++ declares
it (`EulerJoint.AxisOrder`, `InverseKinematics.ErrorMethod`,
`Linkage.Criteria`). Types the design names (`WorldConfig`), the element types
of results (`RayHit`), and types that several public classes use are
module-level: `dart.dynamics.ActuatorType`, and `dart.dynamics.PlaneType`,
which `PlanarJoint` and `TranslationalJoint2D` share. Flattened Python-only
names (`InverseKinematicsErrorMethod`, `LinkageCriteria`) and alias paths
(`DartLoaderFlags` beside `DartLoader.Flags`) are dropped.

**R40.** Enums follow the design's enum rules, plus these:

- members carry no repeated prefix and no count sentinel (`NUM_*`);
- bit sets are flag enums combined with `|` (IK validity, loader flags);
- every parameter, field and return value that carries enum values uses the
  enum, including those typed `int` today;
- enums that are unbound today (`AxisOrder`, `PlaneType`) get bound;
- integer class constants that act as members become members;
- plain enums do not compare equal to integers.

Constants are `UPPER_CASE`. Constants that repeat an instance value or a C++
default (`NumDofs`, `DefaultActuatorType`) are dropped.

**R41.** Static methods and conversions:

- Alternative constructors are classmethods, or static methods on final value
  types, named `from_<source>(…)` (`Uri.from_string`,
  `Rotation.from_axis_angle`; `SoftBodyNodeHelper.makeBoxProperties` becomes
  `SoftLinkSpec.from_box(*, size=…, …)`). Fixed instances are lowercase
  static methods (`Frame.world()`, `Transform.identity()`).
- Exact forms are properties (`matrix`, `wxyz`, `xyzw`). Lossy or
  multi-valued forms are `as_<form>(…)` (`as_rpy()`). Conversions into another
  public type are `to_<type>()`.
- A static that an instance member, a constructor, `Transform` or `Rotation`
  already covers is dropped with a hint (`FreeJoint.setTransformOf(link, T)`
  hints at `link.parent_joint.set_transform(T)`, and
  `SphereShape.computeVolumeOf(r)` at `SphereShape(radius=r).volume`).
- Any other static whose first argument is a public model object becomes an
  instance method of that object's class. A static whose operands are values
  stays static on its C++ class and is named under R13 if it is a predicate
  (`Inertia.is_valid_moment(moment, *, tolerance=…)`) and under R23
  otherwise. The joint area PR names the static methods of `FreeJoint` and
  `BallJoint` that turn a transform or a rotation into joint positions
  (`convertToPositions`), with a noun other than `positions`, which DOF
  collections hold as a property.
- A family that encodes a convention in its names becomes one `from_`
  constructor and one `as_` form with a keyword-only enum: `eulerXYZToMatrix`,
  `matrixToEulerXYZ` and `EulerJoint.convertToTransformOf` map to
  `Rotation.from_euler(angles, *, order=…)` and
  `rotation.as_euler(*, order=…)`. `expMapRot` and `logMap` become
  `Rotation.from_rotation_vector(v)` and `rotation.as_rotation_vector()`; the
  math area PR names the 6-D exponential-map helpers. Quaternion helpers
  (`expToQuat`, `quatToExp`) and the `verifyRotation` and `verifyTransform`
  checks are dropped, because `Rotation` names quaternion orders and both
  value types validate their inputs. `math.Isometry3`, `Quaternion`,
  `AngleAxis` and the `Transform.from_isometry`/`to_isometry` bridges go with
  their last consumer.
- Process-wide C++ settings stay static `get_<x>()` and `set_<x>(value)`
  methods on their C++ class until a per-World setting replaces them. Apart
  from the members that R2 defers, they are the only argument-free get/set
  pairs and the only `get_` names without a matching property, so R23's check
  allow-lists them. They never become class attributes or World properties.
- No static shares its name with an instance member (R59).

**R42.** `dart.io.load_model` and `load_world` are the entry points, as the
design sets out.

Per-format readers are functions in `dart.io.urdf`, `dart.io.sdf`,
`dart.io.mjcf` and `dart.io.skel`: `read_multibody(path, *, …)`,
`read_multibody_from_string(text, *, base_uri=None)`, `read_world(path, *, …)`
and `read_world_from_string(text, *, base_uri=None)`, where the format has
them. They keep their C++ defaults, so the direct MJCF reader keeps its C++
default collision detector, as the design requires, and they are the codemod's
targets for the C++ parser members. The migration guide recommends
`load_model` and `load_world`. Option fields, loader classes and package
registries become keywords (`package_dirs=`), and format enums move to their
format module (`dart.io.urdf.RootJointType`). Path arguments accept `str` and
`os.PathLike`.

`Uri` remains only where C++ takes or returns one. It is an immutable value
with:

- raising `from_string()` and `from_path()` classmethods;
- one `resolve(relative, *, strict=False)`;
- `str()`;
- read-only components that are `None` when absent.

Mutating parse methods, and static duplicates that return strings, are
dropped. The IO area PR redesigns resources to follow Python's file protocol:
`read(size=-1)` returning `bytes`, `seek(offset, whence=os.SEEK_SET)`,
`tell()`, `close()` and `with`. As an exception to R40, `whence` takes the
`os.SEEK_*` integers, which the binding translates to `Resource::SeekType` by
name, never by value, because the two orders differ; `SeekType` is not public.

## Structures

**R43.** Model objects are `Multibody`, `MetaMultibody` and its views, `Link`,
`Joint`, `Dof`, `EndEffector`, `Support`, `SimpleFrame`, `Collider` and
`Visual`; shapes, collision groups, detectors, solvers and IK modules are not.
The `*Properties`, `*UniqueProperties` and `*AspectProperties` structures of
model objects are not public. Their fields become properties of the live
object and keywords of the matching `*Spec`, under the same names (step 5).
A per-DOF field is a `*Spec` keyword under its R14 name that sets
`joint.dofs.<name>`, as the `*Spec` docstring says. `get<X>Properties()` and
`setProperties(…)` are dropped. The properties structure of any other object
becomes its constructor keywords plus a property per field.

**R44.** Options that one call consumes become that call's keyword-only
arguments. A settings structure that a live object owns becomes constructor
keywords and properties (R43) when it is the owner's only group of settings.
Otherwise it forms a live sub-object with typed setters, named by the
structure's stem without `Options` or `Option`, after R48:
`World.DeactivationOptions` becomes `world.solver.sleep`, placed under a
sub-object the design fixes (`world.solver.sleep.enabled = True`). Results and
statistics are read-only values (`Contact`, `RayHit`), returned fresh or
filled in place through `out=` (R24).

**R45.** Field names drop the `m` prefix, with any `Is` after it, and follow
the word rules; a name that a live accessor gives wins (step 5). Fields for
per-DOF arrays are plural, and symbols are spelled out (`mKv` becomes
`vertex_spring_stiffness`, the name `getVertexSpringStiffness` gives). A
transform `X_AB` held by object `B`, through a field or an accessor, is named
`<a>_transform`, where `<a>` is `world`, `relative` (for `B`'s parent frame),
`parent` or `child`: `mT_ParentBodyToJoint` and
`getTransformFromParentBodyNode` become `parent_transform`, and
`mT_ChildBodyToJoint` becomes `child_transform`.

## Callbacks

**R46.** Overridable methods follow the method rules, within R23's limit: they
keep their C++ verb in snake_case, their return type and the state through
which they report. A Python `Solver.solve()` returns `bool` and reports
through `problem.optimal_solution` and `optimum_value`, which stay writable.
`CollisionFilter.ignoresCollision` becomes
`ignores_collision(collider_a, collider_b)`. `Function.eval` and
`evalGradient` become `evaluate(x)` and `evaluate_gradient(x)`, and the latter
returns the gradient. Trampolines implement the pure virtuals that R4 drops
(`Solver.getType`) and look up only the 7.0 names; the codemod renames method
definitions in user subclasses.

Callbacks receive 7.0 types (`Collider`, not `CollisionObject`). A predicate
callback is named for what `True` means (`RaycastOption.mFilter` becomes
`accepts_hit=`).

**R47.** Event callbacks are `on_<event>`, `pre_<phase>` and `post_<phase>`
keywords and attributes, as in `dartpy.viewer`
(`KeyAction(key, label, *, on_activate=…)`). Other callbacks are named for
what they are (`cost_function`, `factory`, R46's predicates). C++ signals are
not exposed.

## Words

**R48.** The design's vocabulary substitutions apply to whole tokens of every
identifier and keyword:

- `BodyNode`, `bodyNode` and `bn` become `link`;
- `Skeleton`, and `skel` where it abbreviates `Skeleton`, become `multibody`;
  the format name `skel` stays (R49);
- `DegreeOfFreedom` and its synonym `GenCoord` become `dof`, and the synonyms
  merge (`getNumDependentGenCoords` and `getNumDependentDofs` both become
  `len(link.dependent_dofs)`);
- `ShapeFrame` and `ShapeNode` become `collider` in collision contexts and
  `visual` in render contexts, and `CollisionObject` becomes `collider`
  (`DistanceResult.shapeFrame1` becomes `collider_a`; the colliding shape
  frames and body nodes of a `CollisionResult` become `colliders` and
  `links`).

`body` stays when it means the body frame or the rigid-body concept
(`body_force`, `add_rigid_body`). It becomes `link` when it means a link or
per-link data (`body_external_forces` becomes `link_external_forces`).

Other words are fixed too:

- deactivation and resting become `sleep` (`is_sleeping`);
- black list becomes `ignored`;
- schema becomes `scheme`;
- `Global` and `Local` become the world-form property or a frame keyword
  (R18–R20). The exceptions are values stored in the object's own frame
  (`local_com`) and the flags in R56.

**R49.** Abbreviations are spelled out: `Coeff`, `Aug`, `Inv`, `Ext`, `Deriv`,
`Jac`, `Eq`, `Ineq`, `idx`, `tf`, `vel`, `acc`, `erp`, `cfm`, `eval`, `trans`,
`rot`, `tri`, `exp`, `quat` and any other abbreviation, except these, which
stay short:

- `dof`, `com`, `ik`, `id`, `kp`, `kd`, `rgb`, `rgba`, `uri` and `config`.
  Docstrings spell out `com` as "center of mass";
- the format and solver acronyms `urdf`, `sdf`, `mjcf`, `skel`, `lcp`, `pgs`
  and `sor`;
- the design's `rpy`, `wxyz`, `xyzw` and `imu`;
- proper names (`CollisionDetectorType.DART`).

Names never describe the binding. They contain no `Python`, `Static` or
`ForDof` token, no binding-invented `Of` suffix, no version suffix (R11) and no
template-type suffix (R37).

**R50.** Names for pairs:

- symmetric pairs end in `_a` and `_b` (`collider_a`, `nearest_point_b`);
- ordered values of one kind form one tuple-valued plural (`getAxis1` and
  `getAxis2` become `axes`);
- no member or keyword name ends in a digit.

**R51.** Names are plain words:

- no Python keywords, no trailing underscores, and no unit suffixes (units
  follow the design's [Reserved names](dart7_python_api.md#reserved-names));
- no keyword named `type`, `filter`, `input`, `print` or `str`
  (`raycast(from, to)` becomes `raycast(start, end)`; `type` becomes the name
  of the property it sets, or else `kind`). Other built-in names that are the
  plain word (`id`, `min`, `max`) are allowed;
- a math symbol (`Ixx`, `T`, `q2`) never becomes a keyword or field name,
  except R49's short names (`kp`, `kd`). An obvious operand that C++ names
  with a symbol is positional-only (`position_differences(q2, q1, /)`).
  Anywhere else, the symbol becomes a word.

`X_AB` notation stays in documentation and user code.

**R52.** Each concept has one spelling across modules. These are fixed:
`retriever` for any resource retriever, `max_iterations` for any iteration or
sweep cap, and `tolerance` for any convergence threshold. An area PR that
finds another disagreement asks the maintainer and adds the winner here. A
word keeps one meaning across the API. Where C++ uses one name for different
quantities (`getOffset` on planes, IK and shape nodes; `setPositions` on
arrows), the area PR gives each quantity its own name.

## Arguments and overloads

**R53.** At most two arguments are positional, and only these kinds:

- the value being set, including a callback that is that value
  (`ik.set_analytical(solve, *, …)`);
- the item being added, removed or looked up, including a callback being added
  (`add_teardown(callback)`);
- the name of a part being built (`add_link(name, …)`);
- a count (`world.step(n)`);
- the source a loader reads (`load_model(path)`,
  `read_world_from_string(text)`);
- the target of a move or copy (`link.move_to(parent)`);
- a pure function's operands;
- the defining values of a shape or value type (`BoxShape(size)`);
- the link or end effector of a Jacobian method on a multibody.

`World`, `SimpleFrame` and every `*Spec` take keyword arguments only. Frame
keywords, point offsets (`offset=`), indices, options, other callbacks and
every boolean are keyword-only. Methods of a Python protocol keep its
signature (`seek(offset, whence)`). Positional arguments can still be passed
by keyword, except math symbols (R51). Keywords match the name of the property
or field they set, so a `*Spec`'s keywords match the live object's property
names, except per-DOF keywords (R43), type selectors (R10), the builder
`*Spec` keywords (`joint=`, `link=`) and the keywords the design fixes
(`parent=`, `collision_detector=`, `rpy=`).

**R54.** Each method is defined once, on the most basic public class that has
it, and const and non-const twins collapse into one. A subclass declares it
again only to refine it with the same meaning: to make a property writable
(R7), or to add keyword-only options (`Multibody.mass_matrix(*, tree=None)`,
R31). A ladder of default-argument overloads, constructors included, becomes
one signature with keyword-only defaults. A member that equals a sibling
called with a fixed argument folds into that sibling:
`getWorldJacobian(offset)` becomes `jacobian(offset=…)`, whose omitted
`expressed_in=` is the world (R20).

**R55.** Overloads with different meanings become separate names, or keyword
discriminators of which exactly one is given (`moment_of_inertia(*, mass=…)`
or `moment_of_inertia(*, density=…)`; `Inertia(*, mass, local_com, moment)` or
`Inertia.from_spatial_tensor(…)`).

One name dispatches on argument type only when every form does the same
operation on separate public types and no form accepts `None`. Examples are
`world.add`, collection indexing, `group.add(collider_or_link)` and
`result.involves(link_or_collider)`.

An argument that maps to a C++ pointer or object handle, other than one of
R19's frame arguments (R20, R21), accepts `None` only where C++ documents what
a null means, and the docstring states it. Anywhere else, `None` raises
`TypeError`. A keyword-only option may default to `None` meaning "not given"
when its docstring states what omitting it does, and `out=` and `indices=`
always accept `None`.

**R56.** A boolean that chooses the local or the world frame for one argument
becomes a keyword-only `<argument>_local=`, with the C++ default.
`addExtForce(force, offset, isForceLocal, isOffsetLocal)` becomes
`add_external_force(force, *, offset=(0, 0, 0), force_local=False,
offset_local=True)`.

## Budgets, checks, collisions and migration

**R57.** A class's budget counts its visible public names: its own, plus those
it inherits from public bases. Dunder protocols do not count, and each
collection or sub-object counts once. `scripts/nanobind/api_surface.py`
enforces a cap per class from the budget file, once it counts inherited names.

A 7.0 class over its cap fails CI. Raising a cap needs maintainer approval in
the area PR, with a use case for each added name. The caps are:

| Class | Cap |
| --- | --- |
| `World` | 24, the design's list: 8 properties, 11 methods and 5 sub-objects |
| `Multibody` | 60 |
| `MetaMultibody` and its views | 50 |
| `Link` | 65 |
| `EndEffector` | 40 |
| `Joint` | 35, and 45 for each concrete joint type |
| `Dof` | 40 |
| DOF collections and selections | 40 |
| Other collections | 12 |
| `Frame` | 20 |
| `SimpleFrame` | 30 |
| `Collider` and `Visual` | 15 each |
| Value and result types | 15 |
| Any other class | 30 |

R23's naming checks run in `api_surface.py` beside the budgets, on every
public class:

- every `get_<x>` has a property `x`, `world_x` or `relative_x` on its class,
  apart from R41's allow-listed process-wide static methods and the members
  that R2 defers;
- no method that returns a value starts with `compute_`, `calc_` or `eval_`;
- every `compute_` method returns `None`, and no property or keyword starts
  with `compute_`;
- no noun is an attribute on one public class and a callable on another,
  apart from protocol methods (R23's rule 4, R59).

**R58.** A class over its cap cuts members in this order:

1. members that another member answers for every call: a property equal to a
   collection access (`root_link` beside `root_links[0]`), or a method whose
   every call equals a property;
2. members also reachable through a collection or a `Dof`;
3. members that neither the reference code's tasks nor the full dynamics
   surface use.

R18's pairs and R28's counts are never cut.

**R59.** The generator reports every pair of members with different meanings
that would share a name on one class or along its hierarchy, including a
static and an instance member. A refinement under R54 is not a pair. It also
reports every noun that is an attribute on one public class and a callable on
another, fields and static factories included, apart from protocol methods
(R23). The area PR drops or renames one member of each pair, so no name
carries two meanings and no noun has two kinds; `api_surface.py` then fails CI
on a noun with two kinds (R57).

**R60.** The codemod renames call sites and the method definitions in user
subclasses, and moves arguments that become keyword-only (setters that took
indices first). It passes `expressed_in=` where R20 changes a default, so
migrated code keeps its numbers. It rewrites a truth test on a
call whose `bool` return became a result (`if ik.findSolution(q):`) to test
`.success`, and flags every other use of that call. It flags what it cannot
rewrite:

- non-literal `step(x)` calls (design);
- dropped members, with their hints;
- deferred members (R2);
- dropped setter flags passed with non-default values;
- partial updates of tuple-valued properties, with the whole-tuple form as the
  suggested fix;
- writes into arrays that used to be live views and are now copies.

## Worked examples

Each row maps current `dartpy` calls to their 7.0 form and cites the rules
that produce it. Within a cell, semicolons separate the items in the same order
in both columns.

| Current dartpy | 7.0 | Rules |
| --- | --- | --- |
| `BodyNode.getMass()` / `setMass(m)` | `link.mass`, read-write | R6, R7 |
| `Skeleton.enableSelfCollisionCheck()`, `disableSelfCollisionCheck()`, `setSelfCollisionCheck(b)`, `getSelfCollisionCheck()`, `isEnabledSelfCollisionCheck()`; the five adjacent-body-check names | `multibody.self_collision_enabled`; `multibody.adjacent_link_collision_enabled` | R7, R13, R48 |
| `Joint.setLimitEnforcement(b)` / `areLimitsEnforced()`; `BodyNode.setGravityMode(b)`; `Skeleton.setMobile(b)` / `isMobile()` | `joint.limits_enabled`; `link.gravity_enabled`; `multibody.mobile` | R7, R13 |
| `MetaSkeleton.getPositions()` / `setPositions(q)`; `resetGeneralizedForces()` and `Joint.resetForces()` | `multibody.dofs.positions` (a read-only copy) / `multibody.dofs.positions = q`; `multibody.dofs.reset_forces()` and `joint.dofs.reset_forces()` | R14, R17 |
| `MetaSkeleton.getPositions(indices)` / `setPositions(indices, q)` | `multibody.dofs[indices].positions`, or, without allocating, `multibody.dofs.get("positions", indices=indices, out=buffer)` / `multibody.dofs[indices].positions = q` | R15 |
| `Joint.getPosition(i)`, `Joint.setCommand(i, c)`, `MetaSkeleton.setForce(i, f)`, `Joint.setActuatorTypeForDof(i, t)` | `joint.dofs[i].position`, `joint.dofs[i].command = c`, `multibody.dofs[i].force = f`, `joint.dofs[i].actuator_type = t` | R1, R14, R15 |
| `DegreeOfFreedom.setPositionLimits(lower, upper)`, `getPositionLowerLimit()`; `MetaSkeleton.setVelocityUpperLimits(v)` | `dof.position_limits = (lower, upper)`, `dof.position_limits[0]`; flagged by the codemod, with the fix `multibody.dofs.velocity_limits = (multibody.dofs.velocity_limits[0], v)` | R11, R14, R60 |
| `MetaSkeleton.getNumBodyNodes()`, `getBodyNode(i)`, `getBodyNode(name)`; `Skeleton.getIndexOf(bodyNode, warning)`, `hasBodyNode(bodyNode)` | `multibody.num_links`, `multibody.links[i]`, `multibody.links[name]` (a missing name raises `KeyError` listing the valid names); `multibody.links.index(link)`, `link in multibody.links` | R16, R25, R27, R28 |
| `BodyNode.getNumChildBodyNodes()` / `getChildBodyNode(i)`; `Joint.getChildBodyNode()`; `Skeleton.getRootBodyNode(treeIndex)` | `len(link.child_links)` / `link.child_links[i]`; `joint.child_link`; `multibody.root_links[i]`, or `root_link` for tree 0 | R27, R28, R31 |
| `Frame.getWorldTransform()`, `getTransform(withRespectTo)`, `getTransform(withRespectTo, inCoordinatesOf)` | `frame.world_transform`, `frame.get_transform(relative_to=a)`, `frame.get_transform(relative_to=a, expressed_in=b)`, each returning a `Transform` | R18–R20 |
| `Frame.getLinearVelocity(offset, relativeTo, inCoordinatesOf)`; `Frame.getSpatialVelocity(Frame.World(), Frame.World())`; `Frame.getSpatialVelocity()`, which uses the frame's own coordinates | `frame.linear_velocity` for the world form, otherwise `frame.get_linear_velocity(offset=p, relative_to=a, expressed_in=b)`; `frame.spatial_velocity` (`link.spatial_velocity` on a link); `frame.get_spatial_velocity(expressed_in=frame)`, which the codemod writes so migrated code keeps its numbers | R18–R20, R60 |
| `JacobianNode.getLinearJacobian(offset, inCoordinatesOf)`, `getWorldJacobian(offset)`, `getJacobian()`; `MetaSkeleton.getJacobian(node, localOffset, relativeTo, inCoordinatesOf)` | `link.linear_jacobian(offset=p, expressed_in=f)`, `link.jacobian(offset=p)` (the omitted `expressed_in=` is the world), `link.jacobian(expressed_in=link)` (which the codemod writes); `multibody.jacobian(link, offset=p, relative_to=other_link, expressed_in=f, out=None)` | R19, R20, R22, R54, R60 |
| `BodyNode.getCOM()`, `getLocalCOM()`, `getCOMLinearVelocity(relativeTo, inCoordinatesOf)` | `link.com`, `link.local_com`, `link.com_linear_velocity` or `link.get_com_linear_velocity(relative_to=a, expressed_in=b)` | R18, R48, R49 |
| `MetaSkeleton.getMassMatrix()`, `Skeleton.getMassMatrix(treeIndex)`, `getInvMassMatrix()`, `getCoriolisAndGravityForces()`, `computeKineticEnergy()` | `multibody.mass_matrix(out=None)`, `mass_matrix(tree=i)`, `inverse_mass_matrix()`, `coriolis_and_gravity_forces(out=None)`, `kinetic_energy()` | R6, R23, R24, R31, R54 |
| `BodyNode.getLinearMomentum()`, `getAngularMomentum(pivot)`; `Joint.getRelativeJacobian()`, `getRelativeJacobian(q)`, `getRelativeJacobianTimeDeriv()`; `VoxelGridShape.getOccupancy(point)` | `link.linear_momentum(expressed_in=link)`, `link.angular_momentum(pivot=p, expressed_in=link)`, which the codemod writes so migrated code keeps its numbers (the omitted `expressed_in=` is the world); `joint.relative_jacobian`, `joint.get_relative_jacobian(positions=q)`, `joint.relative_jacobian_derivative`; `shape.occupancy(point)` | R6, R19, R20, R22, R23, R53, R60 |
| `Skeleton.computeForwardKinematics(True, False, False)`; `Skeleton.getPositionDifferences(q2, q1)` | `multibody.compute_forward_kinematics(update_transforms=True, update_velocities=False, update_accelerations=False)`, which returns `None`; `multibody.position_differences(q2, q1)` | R23, R26, R51 |
| `Skeleton.createRevoluteJointAndBodyNodePair(parent, jointProperties, bodyProperties)`; `BodyNode.createFreeJointAndBodyNodePair()` | `multibody.add_link(name, parent=parent, joint=dart.dynamics.RevoluteJointSpec(axis=…), link=dart.dynamics.LinkSpec(…))`, which returns the `Link`, whose joint is `link.parent_joint`; dropped | R32, R43 |
| `BodyNode.remove(name)`, `split(skeletonName)`, `moveTo(newParent)`, `copy(otherBodyNode)`; `MetaSkeleton.cloneMetaSkeleton(cloneName)` | `link.split(name=…)` for both, `link.move_to(new_parent)`, `link.copy_from(other)` where the area PR keeps it; `view.clone(name=…)` | R34, R35 |
| `JacobianNode.getIK()`; `getIK(True)` and `getOrCreateIK()`; `clearIK()` | `link.ik` and `end_effector.ik`, which are `None` when absent; `link.ik or link.add_ik()`; `link.remove_ik()` | R36, R38 |
| `InverseKinematics.findSolution(positions)`; `setActive()`, `setInactive()`, `isActive()`; `setPythonAnalytical(solve, dofs, methodName)` | `ik.find_solution()`, a read-only result with `success` and `positions`; `ik.active`; `ik.set_analytical(solve, *, dofs=None, name=…)`, after which `ik.analytical` holds the handle that the C++ setter returned | R1, R7, R8, R23, R24, R36, R49 |
| `Joint.getTransformFromParentBodyNode()`, `setTransformFromChildBodyNode(T)`, `getRelativeTransform()` | `joint.parent_transform`, `joint.child_transform = transform`, and dropped (it equals `joint.child_link.relative_transform`) | R12, R45 |
| `Joint.setActuatorType(ActuatorType.SERVO)`; the alias `dartpy.dynamics.SERVO`; `Joint.DefaultActuatorType` | `joint.actuator_type = dart.dynamics.ActuatorType.VELOCITY_MOTOR`; dropped; dropped | R4, R40 |
| `EulerJoint.getAxisOrder()`, which raises `TypeError` today, and `setAxisOrder(order, renameDofs)`; `PlanarJoint.setXYPlane(renameDofs)`; `UniversalJoint.getAxis1()`, `setAxis2(axis)` | `joint.axis_order`, an `EulerJoint.AxisOrder`; `joint.plane_type = dart.dynamics.PlaneType.XY`, which renames the DOFs as the C++ default does; `joint.axes[0]`, `joint.axes = (joint.axes[0], axis)` | R7, R8, R39, R40, R50 |
| `BodyNode.addExtForce(force, offset, isForceLocal, isOffsetLocal)`, `clearExternalForces()`, `getBodyForce()` | `link.add_external_force(force, offset=p, force_local=False, offset_local=True)`, `link.clear_external_forces()`, `link.body_force` | R6, R17, R48, R56 |
| `Skeleton.setGravity(g)`, `Skeleton.getTimeStep()`, `ConstraintSolver.setTimeStep(dt)`; `Skeleton.isResting()`, `setResting(b)`; `Entity.dirtyTransform()`, `RevoluteJoint.getStaticType()`, `Shape.getType()`, `Skeleton.is_sleep_candidate()` | `multibody.gravity = g` and `multibody.time_step`, which `world.add` and the World's setters overwrite, and dropped (`world.time_step` owns the solver's copy); `multibody.is_sleeping`, and dropped as a solver internal; dropped as plumbing, type tags and solver internals (type tests use `isinstance` or `type(x)`) | R1, R4, R12, R13, R48 |
| `dynamics.SimpleFrame(Frame.World(), name, tf)`; `Detachable.setParentFrame(None)` | `dart.SimpleFrame(name=name, relative_transform=tf)`, whose omitted `parent_frame=` means the world; `frame.parent_frame = None`, which attaches the frame to the world | R19–R21, R53 |
| `BoxShape.getSize()` / `setSize(s)`; `Shape.getVolume()`; `SphereShape.computeVolumeOf(r)`; `MeshShape.getMeshUri2()`, `getMeshUri()` | `box.size` (a copy, where today's getter returns a live view) / `box.size = s`; `shape.volume`; `SphereShape(radius=r).volume`; `mesh.uri` (a `Uri`), and dropped | R6, R10, R11, R41 |
| `Shape.computeInertia(m)`, `computeInertiaFromMass(m)`, `computeInertiaFromDensity(d)` | `shape.moment_of_inertia(mass=m)` for the first two, and `shape.moment_of_inertia(density=d)`, each returning the 3x3 moment, because `link.inertia` holds the full `Inertia` | R23, R55 |
| `BodyNode.createShapeNode(shape)` with `createVisualAspect()`, `createCollisionAspect()` and `createDynamicsAspect()`; `removeAllShapeNodes()`; `VisualAspect.setRGBA(c)` / `getRGBA()`, `setColor(rgb)`, `getAlpha()`, `hide()`, `isHidden()`; `BodyNode.setColor(c)` | `link.add_visual(shape)` and `link.add_collider(shape)`; `link.colliders.clear()` and `link.visuals.clear()`; `visual.rgba`, with `setColor(rgb)` and `getAlpha()` dropped as parts of it, and `visual.hidden`; dropped (color is set on each visual) | R4, R9, R11, R13, R32, R33 |
| `SoftBodyNode.getMass()` / `setMass(m)`; `getNumPointMasses()` / `get_point_mass(i)`; `LineSegmentShape.getVertex(i)` | `soft_link.total_mass` (read-only) / `soft_link.mass = m`; `len(soft_link.point_masses)` / `soft_link.point_masses[i]`; `shape.vertices[i]` from a read-only `(N, 3)` array | R1, R7, R27, R30 |
| `World.addSkeleton(s)`; `getSkeleton(name)`; `getNumSkeletons()`; `getSimFrames()`; `setGravity(x, y, z)`; `getRecording()` and `bake()` | `world.add(multibody)`, which returns it and raises `ValueError` instead of renaming; `world.multibodies[name]`; `len(world.multibodies)`; `world.step_count`; `world.gravity = (x, y, z)`; dropped (hint: keep `world.capture_state()` snapshots, or use `dart.Recorder` once it ships) | R5, R8, R11, R27, R28 |
| `World.getDeactivationOptions().mTimeUntilSleep = t`, which edits a copy today; `World.setTimeStep(-1)`, which C++ ignores today | `world.solver.sleep.time_until_sleep = t` on the live sub-object; `world.time_step = -1` raises `ValueError` | R25, R44, R45, R48 |
| `World.getLastCollisionResult().getContacts()`; `Contact.collisionObject1`, `penetrationDepth`, `triID1`; `CollisionResult.isCollision()`, `inCollision(bodyNode)` | `world.collision.contacts`, a tuple of read-only `Contact` values; `contact.collider_a`, `contact.penetration_depth`, and dropped (only one mesh backend fills it); `len(result.contacts) > 0` (`isCollision` is dropped as a duplicate of that test), `result.involves(link)` | R5, R27, R44, R45, R48, R50, R55 |
| `CollisionGroup.raycast(from, to, option, result)` with `RaycastOption.mFilter`; `addShapeFramesOf(bodyNode)` | `group.raycast(start, end, *, accepts_hit=None, …, out=None)`, which returns a `RaycastResult` whose `ray_hits` hold `RayHit` values; `group.add(link)` | R23, R24, R33, R44, R45, R46, R51 |
| `BodyNodeCollisionFilter.addBodyNodePairToBlackList(b1, b2)`, `removeAllBodyNodePairsFromBlackList()`; a user override `def ignoresCollision(self, o1, o2)` | `link_filter.ignored_pairs.add((link_a, link_b))`, `link_filter.ignored_pairs.clear()`; `def ignores_collision(self, collider_a, collider_b)` | R33, R46, R48, R50 |
| `JointConstraint.setErrorReductionParameter(erp)`; `NsgsFrictionSolver.Law.Coulomb`; the alias `dartpy.simulation.DART`; the provisional `world.solver.primary_solver = SolverType.PGS` | `JointConstraint.set_error_reduction_parameter(value)`, a process-wide static that stays until a per-World setting replaces it; `NsgsFrictionSolver.Law.COULOMB`; only `CollisionDetectorType.DART`; `world.solver.primary_solver = dart.constraint.PgsBoxedLcpSolver()` | R1, R4, R10, R40, R41 |
| `math.Isometry3.Identity()`; `math.eulerXYZToMatrix(angles)`; `math.Random.setSeed(s)` | `dart.Transform.identity()`; `dart.Rotation.from_euler(angles, order=…)`, if the math PR keeps Euler helpers; dropped in favor of Python's `random` | R4, R41 |
| `common.Uri.createFromString(s)`, `fromString(s)`, `getPath()`; `common.toUpper(s)`, `common.warn(msg)` | `Uri.from_string(s)` (raises `ValueError`), dropped, and `uri.path`; dropped in favor of `str.upper` and `logging.getLogger("dartpy")` (dartpy's own tests use `dartpy._testing`) | R4, R5, R25, R41, R42 |
| `utils.DartLoader().parseSkeleton(uri)` after `addPackageDirectory(name, path)`; `utils.MjcfParser.readWorld(uri)` | `dart.io.urdf.read_multibody(path, package_dirs={name: path})`, which keeps the C++ defaults and raises `FileNotFoundError` for a missing file and `ValueError` for invalid content (the migration guide recommends `dart.io.load_model`); `dart.io.mjcf.read_world(path)`, which keeps its C++ default detector | R25, R42 |
| `optimizer.Problem.addEqConstraint(f)`, `getNumEqConstraints()`, `setLowerBounds(lb)` and `setUpperBounds(ub)`; `Solver.getNumMaxIterations()`, `GradientDescentSolver.getLastNumIterations()`; overrides of `eval(self, x)`, `evalGradient(self, x, grad)` and `Solver.solve(self)` | `problem.equality_constraints.add(f)`, `len(problem.equality_constraints)`, `problem.bounds = (lower, upper)`; `solver.max_iterations`, `solver.last_iteration_count`; `evaluate(self, x)`, `evaluate_gradient(self, x)`, which returns the gradient, and `solve(self)`, which still returns `bool` | R11, R24, R27, R28, R33, R46, R49 |
| `BodyNode.get_composite_state()`; `dartpy.gui.Viewer` members; `WorldState.body_external_forces`; `dartpy.viewer.Panel()`, `KeyAction(key, label, callback)` | deferred to M3, and dropped at the API freeze if M3 has not replaced it; deferred to M6b and marked `TODO(dart7)`; `link_external_forces`; `dart.viewer.panel()`, a function under the design's PEP 8 rule, and `KeyAction(key, label, on_activate=…)` | R1, R2, R47, R48, R53 |
