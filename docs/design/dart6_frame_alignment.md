# DART 6 Frame-Derived Class Alignment

This document owns the compatibility decision behind the `alignas`
declarations on the classes that inherit `dart::dynamics::Frame` virtually,
made for [#3447](https://github.com/dartsim/dart/issues/3447) on DART 6.20.

## Rule

- A class that inherits `Frame`, or any base whose alignment exceeds 8 bytes,
  *virtually* declares `alignas(<that base>)`: `ShapeFrame`, `JacobianNode`,
  and `FixedFrame` declare `alignas(Frame)`.
- A generic virtual-inheritance helper declares `alignas` of its parameter:
  `common::Virtual<T>` declares `alignas(max(alignof(T), alignof(void*)))` as
  one specifier. The floor keeps a `T` aligned below the vtable pointer
  well-formed (clang rejects a requested alignment below the natural one),
  and it must be a single specifier because GCC does not combine two `alignas`
  specifiers for the non-virtual part.
- `UNIT_dynamics_FrameBaseAlignment` (`tests/unit/dynamics/`) checks a fixed
  list of Frame-family classes with a non-constructing probe; a class added
  to the family is added to that list. `FixedFrame` already satisfies the
  rule through its embedded `FixedFrameProperties` transform, so its
  `alignas(Frame)` is declared for uniformity and the test cannot detect its
  removal; removing it from `ShapeFrame`, `JacobianNode`, or
  `common::Virtual<T>` fails the test.

## Why

`Frame` holds Eigen members, so `alignof(Frame)` is 16 bytes in SSE builds,
32 with AVX, and 64 with AVX-512. A class that reaches `Frame` only through a
virtual base inherits that complete-object alignment, but under the Itanium
C++ ABI the alignment of its non-virtual part stays 8, and a derived class may
place the base subobject at an 8-byte offset (`SimpleFrame` placed
`ShapeFrame` at offset 8). GCC's base-object constructors assume the full
class alignment of `this` and emit aligned vector stores; since DART 6.17.0
added `std::mutex` members to `common::Signal`, the `ShapeFrame` constructor
carries such a store and faults when invoked from `SimpleFrame`. GCC 16 shows
this at plain `-O2 -march=x86-64`; GCC 15 with `-march=native` (which
`DART_ENABLE_SIMD=ON` adds); clang is unaffected. The `alignas` declarations
make the assumed alignment true, so every base subobject is placed at a fully
aligned offset and no derived class, in DART or downstream, can reintroduce
the exposure. Complete-object alignment does not change, so aligned
allocation paths (`make_aligned_shared`, `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`)
are unaffected, and GCC and clang keep one layout.

## Layout impact (DART 6.20 versus 6.19.4)

Measured with `-fdump-lang-class` (GCC 15.2; GCC 16.2 and clang 22 agree):

| Build | Classes whose size changes | `SimpleFrame` | `ShapeNode` | `SoftBodyNode` |
| --- | --- | --- | --- | --- |
| SSE (`-march=x86-64`, default) | `SimpleFrame`, `dart-gui-osg` `InteractiveFrame`/`InteractiveTool` | 1792 → 1808 | unchanged | unchanged |
| AVX2 (`x86-64-v3`) | + `ShapeNode` | 1856 → 1888 | 1984 → 2016 | unchanged |
| AVX-512 (`x86-64-v4`) | + `SoftBodyNode` | 1984 → 2048 | 2112 → 2176 | 5504 → 5568 |
| Eigen static alignment disabled | none | unchanged | unchanged | unchanged |

`ShapeFrame`, `JacobianNode`, `FixedJacobianNode`, `FixedFrame`, `BodyNode`,
`EndEffector`, `Marker`, `PointMass`, and `Skeleton` keep their sizes at every
level. After the change no DART class has a non-virtual alignment below its
full alignment (scan of the dynamics, simulation, constraint, utils, and gui
headers), and the only virtual bases aligned above 8 bytes are `Frame` and
`FixedFrame`.

## Compatibility boundary

- This is a layout change and belongs to the DART 6.20 line only. The branch
  reports 6.19.4 and builds `libdart.so.6.19` until the release-packaging
  change bumps `package.xml`; that bump precedes any 6.20.0 tag, and the change
  is never cherry-picked to a branch that cuts a 6.19.x.
- Downstream code is rebuilt against DART 6.20 headers. gz-physics and gz-sim
  do not subclass DART frames; they observe `sizeof(SimpleFrame)` through the
  header-inline `SimpleFrame::createShared`, which the Gazebo gate
  (`pixi run -e gazebo test-gz`) re-verifies.
- dartpy exposes no Frame-family trampolines, so the bindings see the change
  only through header layout.
- On MSVC, `common::Virtual<T>` is now declared inside
  `DART_DECLARE_CLASS_WITH_VIRTUAL_BASE_BEGIN`/`_END` like the other
  virtual-base classes, so it also gains that macro's `vtordisp(push, 2)`
  layout and C4324 suppression; this is a second, MSVC-only layout change to
  an installed header.

## Rejected alternative

Keeping `common::Signal`'s constructor out of line for GCC also stops this
crash at zero layout cost (measured), but it is a symptom fix: the
under-aligned base placements remain, so any other constructor that grows a
vectorisable store, in DART or in downstream classes deriving from these
bases, reintroduces the fault. It remains the only option for an ABI-frozen
6.19.x patch release, should one be cut.
