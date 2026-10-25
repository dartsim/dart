# DART 7 Python and C++ Differences

DART 7 designs its Python API first. When the best Python behavior differs from
the C++ API that `dartpy` binds, Python takes the better behavior: C++ parity
is best effort and never blocks Python-API work. C++ changes only where the
Python API needs one, and the C++ API that Gazebo uses stays compatible.

This register lists every such difference in one place, so each can be
resolved later: C++ may adopt the Python behavior, or the two may stay
different on purpose. Spelling alone is not a difference here. The
[DART 7 Python Naming Rulebook](dart7_python_naming.md) and the generated
old-to-new mapping record renames, and
[Model vocabulary](dart7_python_api.md#model-vocabulary) describes how C++
adopts the new vocabulary.

A pull request that creates a difference adds its row here in the same pull
request. A pull request that resolves one deletes the row, or changes its
direction to "C++ keeps" with the reason.

| Area | Python 7.0 | C++ | Why Python differs | Direction |
| --- | --- | --- | --- | --- |
| Omitted frames | An omitted frame keyword means the world, except that a pose or position (`get_transform`, `get_com`) is expressed in its `relative_to=` frame (`X_AB`). A world-form property such as `spatial_velocity` is relative to and expressed in the world. The default spatial Jacobian derivative is therefore the own-coordinates derivative rotated into world axes; the time derivative of the default Jacobian is the classic derivative. | The no-frame forms of `getSpatialVelocity`, `getSpatialAcceleration`, `getJacobian` and `getJacobianSpatialDeriv`, with or without an offset, and `BodyNode::getCOMSpatialVelocity()`, `getCOMSpatialAcceleration()`, `getLinearMomentum()` and `getAngularMomentum(pivot)` use the object's own coordinates. `Joint::getWrenchToChildBodyNode` and `getWrenchToParentBodyNode` default to the joint frame, and a `TaskSpaceRegion` reference defaults to the target's parent frame. The other getters default to the world. | One rule for every query, so sibling calls never default to different frames. Several C++ defaults return a cached own-coordinates value by reference, a saving Python loses because it copies every result. | Open |
| `None` frames | `None` always means the same as omitting the keyword. A few arguments default to a documented null meaning instead of the world: joint wrenches (the joint frame), task-space region references (the target's parent), `descends_from(None)`, a builder's `parent=None` and `Chain(start=None)`. | A null frame pointer is accepted only where an overload documents it. | Python callers pass `None` to mean the default. | Open |
| Default detector | `dart.World` and `dart.io.load_world` for MJCF default to the DART collision detector. | `World` defaults to FCL for Gazebo compatibility; `dart::utils::MjcfParser::readWorld` also keeps FCL. | The DART detector supports exact checkpoints and allocation-free compiled stepping, which FCL does not. | Open |
| Returned arrays | Read-only copies; `out=` fills a caller's buffer. | `const` references into live state. | Python cannot express a `const` view; copies prevent aliasing bugs. | C++ keeps: a reference avoids a copy on every call. |
| Immutable values | `Inertia`, `BoundingBox` and result types such as `Contact` are immutable; a change builds a new value and assigns it through its owner. | `Inertia::setMass`, `BoundingBox::setMin` and the public fields of `Contact` edit a value in place. | An edit to a returned copy raises instead of being silently lost. | C++ keeps: Gazebo builds `Inertia` values with its setters. |
| Bad input | Raises the builtin that names the failure, for example `ValueError` for `world.time_step = -1` and for a malformed URI string. | Some setters log and ignore bad input, such as `World::setTimeStep`, and some functions return `false` or a sentinel, such as `Uri::fromString`. | Python reports errors as exceptions. | Open |
| Lookups | A missing name raises `KeyError` listing the valid names, and an index out of range raises `IndexError`. | `World::getSkeleton(name)` and `Skeleton::getBodyNode(name)` return `nullptr` for a missing name; index getters return `nullptr` out of range, and some assert first in debug builds. | Python reports errors as exceptions, and the listed names show a misspelling. | C++ keeps: callers test the returned pointer, and Gazebo uses these lookups. |
| Shared names | A name that several items of a `MetaMultibody` view share raises `KeyError` listing those items. | `ReferentialSkeleton::getBodyNode(name)` warns and returns the first match. | A lookup that picks one of several items can return the wrong one. | Open |
| Index lookups | `collection.index(x)` raises `ValueError` for an item that the collection does not hold. | `getIndexOf` logs an error and returns `INVALID_INDEX`; debug builds assert first. | Python reports errors as exceptions, as `list.index` does. | Open |
| Optional parts | `add_<part>()` raises when the part exists, as `link.add_ik()` does. | `JacobianNode::createIK()` destroys an existing IK module and creates a new one. | A silent replacement discards the existing part's settings. | Open |
| Loaders | Raise `FileNotFoundError` for a missing file and `ValueError` for invalid content. | Return `nullptr` and log. | Python reports errors as exceptions. | Open |
| Name conflicts | `world.add`, and assigning a `name` already in use, raise `ValueError`. | `World::addSkeleton` and `BodyNode::setName` rename to a unique name. | A silent rename breaks name lookups. | C++ keeps: both return the unique name for callers to use, and Gazebo calls them. |
| Busy World | `WorldBusyError`, a `RuntimeError` subclass, is raised for re-entrant World operations and for any access from another thread while a World steps, reads included. | Re-entrant or competing World operations, and model edits during one, throw `std::runtime_error` with a `WorldBusyError:` message prefix; reads are not checked. | Python callers choose a recovery by exception type, such as a reader thread that retries. Getters fill lazy caches, so the binding checks reads too. | Open |
| Soft-body mass | `soft_link.mass` is the rigid part; `soft_link.total_mass` includes the point masses. | `SoftBodyNode::setMass` sets the rigid part, but `getMass` includes the point masses. | A property must return what was assigned. | Open |
| Stepping | `world.step(n)` takes a count; persistent target channels carry commands across steps. | `World::step(bool resetCommand)` and `World::stepMultiple(count, resetCommand)`. | A count is the common Python call; the flag is easy to misread. | C++ keeps: Gazebo calls `World::step`, whose reset semantics stay. |
| Poses | Immutable `Transform` and `Rotation` values. | `Eigen::Isometry3d` and Eigen rotation types. | Value semantics and input validation. | C++ keeps: the Eigen type matches the pose type that gz-physics uses. |
| Quaternion order | Always named: `wxyz=`, `xyzw=`, `.wxyz`, `.xyzw`. | Eigen's constructor takes `(w, x, y, z)` while its storage is `(x, y, z, w)`. | An unnamed order is a common, silent bug. | C++ keeps: the order is Eigen's, not DART's. |
