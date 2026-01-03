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
