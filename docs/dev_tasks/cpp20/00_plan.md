# C++20 Modernization Plan (Phase 7+)

## Status

- Phases 1-6 are complete or in review. This plan covers the next phases.

## Goals

- Modernize the C++ API surface to idiomatic C++20 with no behavior changes.
- Remove unnecessary allocations and conversions introduced by pre-C++20 APIs.
- Keep changes mechanical and easy to review.

## Constraints

- Use existing pixi entry points only (`pixi run ...`).
- Run `pixi run lint` before each commit.
- Keep documentation short and remove this folder when the task is complete.
- API changes are acceptable for DART 7 as long as `pixi run -e gazebo test-gz`
  passes without changes to Gazebo code.

## Phases

### Phase 7: Span-first read-only inputs

- Replace read-only `const std::vector<T>&` parameters with
  `std::span<const T>` where the callee only iterates.
- Remove redundant vector overloads when the span variant fully covers usage.
- Apply to core collections in dynamics, collision, and simulation modules.

### Phase 8: String-view parsing inputs

- Switch parsing and lookup functions that only read input strings to
  `std::string_view` (no storage of the view).
- Focus on XML/SDF/URDF/MJCF utilities and URI/path parsing helpers.
- Keep getters that return stored names as `const std::string&`.

### Phase 9: C++20 container membership

- Replace membership checks like `find(...) != end()` with `.contains(...)`
  where the iterator is not otherwise needed.
- Apply to maps/sets across dynamics, simulation, GUI, and utils.

### Phase 10: Signed sizes with `std::ssize`

- Replace repeated `static_cast<int>(vec.size())` and similar patterns with
  `std::ssize(vec)` when the size is used as a signed value.
- Keep explicit casts when APIs require `int`.

### Phase 11: Type-name string views

- Convert `getType()`/`getStaticType()` families to return
  `std::string_view` backed by `static constexpr` data.
- Update the `Castable` macros and type-name implementations consistently.
- Isolate this phase because it is a broad API signature change.
- Keep `CollisionDetector`/`BoxedLcpSolver` `getType()` returning
  `const std::string&` for gz-physics compatibility; add `getTypeView()` for
  `std::string_view` access.

### Phase 12: Optimization solver type strings

- Switch `math::optimization::Solver::getType()` to return
  `std::string_view` for non-owning type identifiers.
- Update `GradientDescentSolver` and the dartpy trampoline/bindings.

### Phase 13: Span helper inputs

- Replace read-only `const std::vector<T>&` parameters in utils parsing helpers
  and math mesh/LCP utilities with `std::span<const T>`.
- Update call sites to pass spans explicitly where needed.

### Phase 14: Geometry helper spans

- Switch geometry helper inputs (support polygons/hulls) to
  `std::span<const Eigen::Vector2d>`/`std::span<const Eigen::Vector3d>` in
  `dart/math/Geometry`.
- Add span-friendly overloads for convex hull helpers in
  `dart/math/detail/Convhull` and `computeConvexHull3D`, keeping vector returns.
- Update call sites (Skeleton support polygon, TriMesh convex hull,
  PolyhedronVisual) and tests.

### Phase 15: Simulation experimental spans

- Replace read-only `const std::vector<double>&` inputs in simulation
  experimental mappers with `std::span<const double>`.
- Switch `StateSpace::addVariables` to `std::span<const std::string>`.
- Update binary I/O helpers and tests to pass spans explicitly.

### Phase 16: Name lookup string views

- Switch dynamics name lookup APIs to `std::string_view` for read-only queries
  (`MetaSkeleton`/`Skeleton`/`ReferentialSkeleton` name-based accessors).
- Enable heterogeneous lookup in `common::NameManager` and accept
  `std::string_view` for lookup helpers.
- Update call sites and docs to match new signatures.

### Phase 17: Uri string-view inputs

- Switch `common::Uri` parsing/merging helpers to accept `std::string_view`
  for read-only inputs.
- Keep `const char*` overloads for compatibility where appropriate.

### Phase 18: Shared library string views

- Use `std::string_view` for shared-library path and symbol inputs in
  `SharedLibrary` and `SharedLibraryManager`.
- Avoid extra copies beyond those needed for OS APIs (e.g., null-terminated
  strings).

### Phase 19: Resource retriever string views

- Switch resource-retriever configuration inputs (schema names, package paths,
  data directories) to `std::string_view`.
- Keep storage in `std::string` for registries and directory lists.
