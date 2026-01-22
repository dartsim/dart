# dart::simd Extension Plan

## Overview

This document outlines the plan for extending the dart::simd layer with:

1. Complete ISA coverage for Intel/AMD/ARM
2. SIMD-backed geometric types (Vec3, Mat3x3, etc.) with Eigen interop

---

## Current Status (Updated 2026-01-21)

### Completed

- [x] **Sprint 2**: Pure AVX backend (`dart/simd/detail/avx/`)
- [x] AVX-512 sub-extension detection (BW/DQ/VL/CD macros)
- [x] ARM SVE/SVE2 detection macros (backend deferred)
- [x] Benchmark restructured for side-by-side comparison
- [x] Native SIMD enabled via `-march=native` for fair benchmarks
- [x] **Sprint 3**: Geometric types Vector3/Vector4 (`dart/simd/geometry/`)
- [x] **Sprint 4**: Geometric types Matrix3x3/Matrix4x4 (`dart/simd/geometry/`)
- [x] Vec<T,W>::set() static factory for explicit element construction
- [x] **Sprint 5**: ECS batch operations (EigenSoA4, transform_points, transform_vectors)
- [x] **Sprint 6**: FMA optimization - native width + loop unrolling
- [x] **Sprint 7**: Quaternion and Isometry3 (`dart/simd/geometry/`)
- [x] **Sprint 8**: Dynamic vectors and matrices (`dart/simd/dynamic/`)
- [x] Streaming chunk iterator (`simdChunks<N>`) in `dart/simd/eigen/iterator.hpp`
- [x] **Naming convention refactor**: All snake_case functions renamed to camelCase for DART C++ consistency

### Deferred (Future Work)

- [ ] ARM SVE backend implementation
- [ ] Zero-copy views over interleaved ECS storage

### Benchmark Results (AVX2+FMA, Intel 13th Gen)

#### Core Operations (65536 elements)

| Operation               | dart::simd |
| ----------------------- | ---------- |
| **Dot f32 (4x unroll)** | 151 Gi/s   |
| **Dot f32**             | 79 Gi/s    |
| **FMA f32 (2x unroll)** | 109 Gi/s   |
| **FMA f32**             | 103 Gi/s   |
| **Add f32**             | 86 Gi/s    |
| **Mul f32**             | 105 Gi/s   |
| **Sqrt f32**            | 26 Gi/s    |
| **Add f64**             | 74 Gi/s    |

#### Real-World Scale Benchmarks

| Benchmark           | Size         | DART     |
| ------------------- | ------------ | -------- |
| **TransformPoints** | 1K vertices  | 250 Gi/s |
| **TransformPoints** | 64K vertices | 70 Gi/s  |
| **TransformPoints** | 1M points    | 50 Gi/s  |
| **BatchDot3**       | 1K contacts  | 224 Gi/s |
| **BatchDot3**       | 4K contacts  | 142 Gi/s |
| **BatchCross3**     | 1K contacts  | 234 Gi/s |
| **BatchCross3**     | 4K contacts  | 89 Gi/s  |
| **BatchNormalize3** | 1K vectors   | 156 Gi/s |
| **BatchNormalize3** | 4K vectors   | 156 Gi/s |

**Key design advantages**:

1. Native SIMD width (8-wide on AVX2)
2. Loop unrolling with multiple accumulators
3. Proper FMA utilization pattern

---

## Part 1: ISA Architecture Coverage

### Current State (UPDATED)

```
dart/simd/detail/
├── scalar/      # Fallback (any platform) ✅
├── sse42/       # Intel/AMD SSE4.2 (128-bit) ✅
├── avx/         # Pure AVX (256-bit FP only) ✅ NEW
├── avx2/        # Intel/AMD AVX2+FMA (256-bit) ✅
├── avx512/      # Intel AVX-512F+DQ (512-bit) ✅
└── neon/        # ARM NEON (128-bit) ✅
```

### ISA Decision Matrix (UPDATED)

| ISA                  | Status    | Notes                                          |
| -------------------- | --------- | ---------------------------------------------- |
| **SSE/SSE2/SSE3**    | Skip      | Implicit in SSE4.2; no separate backend needed |
| **SSSE3/SSE4.1**     | Skip      | SSE4.2 is superset; 2008+ baseline sufficient  |
| **SSE4.2**           | ✅ Done   | Current 128-bit baseline                       |
| **AVX**              | ✅ Done   | Sandy Bridge (2011) support without AVX2       |
| **AVX2+FMA3**        | ✅ Done   | Modern 256-bit baseline                        |
| **FMA4**             | Skip      | AMD-only, obsolete since 2012                  |
| **AVX-512F**         | ✅ Done   | Basic 512-bit support                          |
| **AVX-512 BW/DQ/VL** | ✅ Done   | Detection macros in config.hpp                 |
| **NEON**             | ✅ Done   | ARM 128-bit baseline                           |
| **SVE/SVE2**         | Detection | Macros added, backend deferred                 |

### Phase C: ARM SVE Support (Future/Research)

SVE is vector-length-agnostic. Hardware determines width at runtime.

**Status**: Detection macros added (`DART_SIMD_SVE`, `DART_SIMD_SVE2`), no backend implementation yet.

**Challenges**:

1. Width unknown at compile time (128-2048 bits)
2. Different intrinsics model (`svfloat32_t` vs `float32x4_t`)
3. Predicate registers instead of masks
4. Requires runtime width detection

**Recommendation**: Defer SVE backend implementation to separate research spike. Detection macros are in place for future use. Current NEON coverage is sufficient for most ARM use cases.

---

## Part 2: SIMD Vector/Matrix Types with Eigen Interop

### Goals

1. **Fixed-size geometric types**: `Vec3<T>`, `Vec4<T>`, `Mat3x3<T>`, `Mat4x4<T>`
2. **Dynamic SIMD arrays**: `SimdArray<T>` for batch processing
3. **Seamless Eigen interop**: Zero-copy where possible, explicit conversion otherwise

### Current State

We have:

- `Vec<T, W>`: Raw SIMD register wrapper
- `EigenSoA3<T, N>`: Structure-of-arrays for batch Vector3 ops
- `to_vec()`, `to_eigen()`: Explicit conversions

### Proposed Type Hierarchy

```cpp
namespace dart::simd {

// ============================================================================
// Tier 1: Raw SIMD registers (existing)
// ============================================================================
template <typename T, std::size_t W> struct Vec;      // e.g., Vec<float, 4>
template <typename T, std::size_t W> struct VecMask;

// ============================================================================
// Tier 2: Fixed-size geometric types (NEW)
// ============================================================================

// 3D Vector - always uses Vec<T, 4> internally (padded for SIMD)
template <typename T>
struct Vector3 {
  Vec<T, 4> data;  // [x, y, z, 0]

  // Constructors
  Vector3() = default;
  Vector3(T x, T y, T z);
  explicit Vector3(const Eigen::Matrix<T, 3, 1>& v);

  // Element access
  T& x() { return data[0]; }
  T& y() { return data[1]; }
  T& z() { return data[2]; }

  // Arithmetic (SIMD-accelerated)
  friend Vector3 operator+(const Vector3& a, const Vector3& b);
  friend Vector3 operator-(const Vector3& a, const Vector3& b);
  friend Vector3 operator*(const Vector3& a, T scalar);
  friend T dot(const Vector3& a, const Vector3& b);
  friend Vector3 cross(const Vector3& a, const Vector3& b);

  // Eigen interop
  Eigen::Matrix<T, 3, 1> to_eigen() const;
  static Vector3 from_eigen(const Eigen::Matrix<T, 3, 1>& v);
};

// 4D Vector - native Vec<T, 4>
template <typename T>
struct Vector4 {
  Vec<T, 4> data;
  // Similar interface...
};

// 3x3 Matrix - column-major, 3x Vec<T, 4> storage
template <typename T>
struct Matrix3x3 {
  Vec<T, 4> col0, col1, col2;  // Each column padded to 4

  // Matrix-vector multiply (SIMD)
  friend Vector3<T> operator*(const Matrix3x3& m, const Vector3<T>& v);

  // Matrix-matrix multiply
  friend Matrix3x3 operator*(const Matrix3x3& a, const Matrix3x3& b);

  // Eigen interop
  Eigen::Matrix<T, 3, 3> to_eigen() const;
  static Matrix3x3 from_eigen(const Eigen::Matrix<T, 3, 3>& m);
};

// 4x4 Matrix - 4x Vec<T, 4> storage (perfect fit)
template <typename T>
struct Matrix4x4 {
  Vec<T, 4> col0, col1, col2, col3;

  // Transform operations
  friend Vector4<T> operator*(const Matrix4x4& m, const Vector4<T>& v);
  friend Vector3<T> transform_point(const Matrix4x4& m, const Vector3<T>& p);
  friend Vector3<T> transform_vector(const Matrix4x4& m, const Vector3<T>& v);
};

// ============================================================================
// Tier 3: Batch/SoA types (existing + enhanced)
// ============================================================================

// Batch of N Vector3s in SoA layout (existing)
template <typename T, std::size_t N>
struct EigenSoA3;  // { Vec<T,N> x, y, z; }

// NEW: Batch operations on arrays
template <typename T, std::size_t N>
struct Vector3Batch {
  Vec<T, N> x, y, z;

  // Batch dot product: returns Vec<T, N> of dot products
  friend Vec<T, N> dot(const Vector3Batch& a, const Vector3Batch& b);

  // Batch cross product
  friend Vector3Batch cross(const Vector3Batch& a, const Vector3Batch& b);
};

// ============================================================================
// Tier 4: Dynamic arrays (NEW)
// ============================================================================

// SIMD-optimized dynamic array
template <typename T>
class SimdArray {
  aligned_vector<T> data_;

public:
  // Bulk operations (SIMD-accelerated loops)
  void add(const SimdArray& other);
  void multiply(T scalar);
  T sum() const;
  T dot(const SimdArray& other) const;

  // Eigen interop
  Eigen::Map<Eigen::VectorX<T>> as_eigen();
  const Eigen::Map<const Eigen::VectorX<T>> as_eigen() const;
};

} // namespace dart::simd
```

### Eigen Interop Strategy

| Conversion                          | Method                      | Cost                   |
| ----------------------------------- | --------------------------- | ---------------------- |
| `Vector3<T>` -> `Eigen::Vector3<T>` | `to_eigen()`                | Copy (12 bytes)        |
| `Eigen::Vector3<T>` -> `Vector3<T>` | `from_eigen()`              | Copy (12 bytes)        |
| `SimdArray<T>` <-> `Eigen::VectorX` | `as_eigen()`                | Zero-copy (Eigen::Map) |
| `Matrix3x3<T>` <-> `Eigen::Matrix3` | `to_eigen()`/`from_eigen()` | Copy (72 bytes)        |

### Implementation Priority

| Type                 | Priority | Use Case           | Status   |
| -------------------- | -------- | ------------------ | -------- |
| `Vector3<T>`         | High     | Physics, collision | ✅ Done  |
| `Vector4<T>`         | High     | Homogeneous coords | ✅ Done  |
| `Matrix3x3<T>`       | High     | Rotations          | ✅ Done  |
| `Matrix4x4<T>`       | High     | Transforms         | ✅ Done  |
| `EigenSoA3<T, N>`    | High     | Batch Vector3 ops  | ✅ Done  |
| `EigenSoA4<T, N>`    | High     | Batch Vector4 ops  | ✅ Done  |
| `Quaternion<T>`      | High     | Rotations          | Sprint 7 |
| `Isometry3<T>`       | High     | Rigid transforms   | Sprint 7 |
| `DynamicVector<T>`   | Medium   | LCP solvers        | Sprint 8 |
| `DynamicMatrix<T>`   | Medium   | Jacobians, LCP     | Sprint 8 |
| `QuaternionSoA<T,N>` | Medium   | Batch rotations    | Sprint 7 |
| `Isometry3SoA<T,N>`  | Medium   | Batch transforms   | Sprint 7 |

---

## Implementation Roadmap (UPDATED)

### Sprint 1: Pure AVX Backend (2 days) - COMPLETED

- [x] Add `dart/simd/detail/avx/` backend
- [x] Float8, Double4 specializations
- [x] Integer fallback to 2x SSE
- [x] CI test with `-mavx -mno-avx2` flag (in ci_simd.yml)

### Sprint 3: Geometric Types - Vector3/Vector4 (3 days) - COMPLETED

- [x] Add `dart/simd/geometry/vector3.hpp`
- [x] Add `dart/simd/geometry/vector4.hpp`
- [x] SIMD dot, cross, normalize
- [x] Eigen interop (to_eigen, from_eigen)
- [x] Unit tests (`tests/unit/simd/test_geometry.cpp`)

### Sprint 4: Geometric Types - Matrices (3 days) - COMPLETED

- [x] Add `dart/simd/geometry/matrix3x3.hpp`
- [x] Add `dart/simd/geometry/matrix4x4.hpp`
- [x] Matrix-vector multiply
- [x] Matrix-matrix multiply
- [x] Transform operations (transformPoint, transformVector)
- [x] Unit tests

### Sprint 5: ECS Batch Operations (2 days) - COMPLETED

Extend EigenSoA for ECS-style batch processing on contiguous Eigen arrays.

**Implemented** (`dart/simd/eigen/interop.hpp`):

- [x] `EigenSoA3<T, N>` - SoA layout for N vectors
- [x] `EigenSoA4<T, N>` - 4D vectors for homogeneous coordinates
- [x] `dot3()`, `cross3()` - Batch operations on 4 vectors
- [x] `dot4()` - Batch 4D dot product
- [x] `transpose_aos_to_soa()` / `transpose_soa_to_aos()` for both 3D and 4D
- [x] `transform_points(Matrix4x4, EigenSoA3)` - Batch transform N points
- [x] `transform_vectors(Matrix4x4, EigenSoA3)` - Batch transform N vectors (rotation only)

**Deferred to future sprints**:

- Streaming chunk iterator (`simd_chunks<N>`) for large contiguous arrays
- Stride support for interleaved ECS component arrays
- Zero-copy view types over existing Eigen storage

**Use cases**:

- EnTT/other ECS: Process position/velocity components in batches
- Collision broadphase: Batch AABB updates
- Physics: Batch gravity/force application

### Sprint 6: FMA Optimization (1 day) - COMPLETED

**Implementation**:

- Benchmarks now use `preferred_width_v<T>` (8-wide on AVX2, 4-wide on SSE)
- Added 2x unrolled versions for Add and FMA benchmarks
- Added 4x unrolled version for Dot product with multiple accumulators
- Dot product now uses `fmadd(a, b, acc)` instead of `result += hsum(a * b)`

**Results**:

- FMA: Significant improvement with proper unrolling
- Dot: 4x unrolling with multiple accumulators hides FMA latency

### Sprint 6: Quaternion and Isometry3 (2 days) - COMPLETED

Rigid body transformations are fundamental to physics simulation.

**Quaternion<T>** - Unit quaternion for 3D rotations:

```cpp
template <typename T>
struct Quaternion {
  Vec<T, 4> data;  // [w, x, y, z] or [x, y, z, w] - TBD convention

  // Constructors
  static Quaternion identity();
  static Quaternion from_axis_angle(const Vector3<T>& axis, T angle);
  static Quaternion from_rotation_matrix(const Matrix3x3<T>& m);

  // Operations (SIMD-accelerated)
  Quaternion operator*(const Quaternion& other) const;  // Hamilton product
  Quaternion conjugate() const;
  Quaternion inverse() const;
  Vector3<T> rotate(const Vector3<T>& v) const;
  Matrix3x3<T> to_rotation_matrix() const;

  // Interpolation
  static Quaternion slerp(const Quaternion& a, const Quaternion& b, T t);

  // Eigen interop
  Eigen::Quaternion<T> to_eigen() const;
  static Quaternion from_eigen(const Eigen::Quaternion<T>& q);
};
```

**Isometry3<T>** - Rigid body transformation (rotation + translation):

```cpp
template <typename T>
struct Isometry3 {
  Quaternion<T> rotation;
  Vector3<T> translation;

  // OR: Matrix4x4<T> matrix;  // Alternative: single 4x4 storage

  // Operations
  Vector3<T> transform_point(const Vector3<T>& p) const;
  Vector3<T> transform_vector(const Vector3<T>& v) const;  // rotation only
  Isometry3 operator*(const Isometry3& other) const;  // composition
  Isometry3 inverse() const;

  // Conversions
  Matrix4x4<T> to_matrix() const;
  static Isometry3 from_matrix(const Matrix4x4<T>& m);

  // Eigen interop
  Eigen::Isometry3<T> to_eigen() const;
  static Isometry3 from_eigen(const Eigen::Isometry3<T>& iso);
};
```

**Batch versions**:

- `QuaternionSoA<T, N>` - N quaternions in SoA layout
- `Isometry3SoA<T, N>` - N transforms in SoA layout
- Batch slerp, batch transform operations

### Sprint 8: Dynamic Vectors and Matrices (3 days) - COMPLETED

For LCP solvers, optimization, and variable-size problems.

**DynamicVector<T>** - Runtime-sized SIMD vector:

```cpp
template <typename T>
class DynamicVector {
  aligned_vector<T> data_;  // Cache-line aligned storage

public:
  // Construction
  explicit DynamicVector(std::size_t size);
  DynamicVector(std::initializer_list<T> init);

  // SIMD-accelerated bulk operations
  DynamicVector& operator+=(const DynamicVector& other);
  DynamicVector& operator-=(const DynamicVector& other);
  DynamicVector& operator*=(T scalar);
  T dot(const DynamicVector& other) const;
  T norm() const;
  T sum() const;
  void normalize();

  // Element access
  T& operator[](std::size_t i);
  const T& operator[](std::size_t i) const;
  std::size_t size() const;

  // Zero-copy Eigen interop
  Eigen::Map<Eigen::VectorX<T>> as_eigen();
  const Eigen::Map<const Eigen::VectorX<T>> as_eigen() const;

  // Copy conversion
  static DynamicVector from_eigen(const Eigen::VectorX<T>& v);
};
```

**DynamicMatrix<T>** - Runtime-sized SIMD matrix (column-major):

```cpp
template <typename T>
class DynamicMatrix {
  aligned_vector<T> data_;
  std::size_t rows_, cols_;

public:
  // SIMD-accelerated operations
  DynamicVector<T> operator*(const DynamicVector<T>& v) const;  // mat-vec
  DynamicMatrix operator*(const DynamicMatrix& other) const;     // mat-mat
  DynamicMatrix transpose() const;

  // Block operations (for LCP solver)
  void set_block(std::size_t row, std::size_t col, const Matrix3x3<T>& block);
  Matrix3x3<T> get_block(std::size_t row, std::size_t col) const;

  // Zero-copy Eigen interop
  Eigen::Map<Eigen::MatrixX<T>> as_eigen();
};
```

**Use cases**:

- LCP constraint matrices (variable contact count)
- Jacobian matrices (variable DOF)
- Optimization problems (variable dimension)

### Future: ARM SVE (TBD)

- [ ] Research spike for SVE programming model
- [ ] VLA (vector-length-agnostic) architecture changes

---

## File Structure (Current)

```
dart/simd/
├── config.hpp              # ISA detection (AVX-512 sub-ext, SVE macros)
├── fwd.hpp                 # Forward declarations
├── memory.hpp              # AlignedAllocator
├── simd.hpp                # Umbrella header (includes geometry)
├── detail/
│   ├── scalar/             # Fallback ✅
│   ├── sse42/              # SSE4.2 ✅
│   ├── avx/                # Pure AVX ✅
│   ├── avx2/               # AVX2+FMA ✅
│   ├── avx512/             # AVX-512 ✅
│   └── neon/               # ARM NEON ✅
├── eigen/
│   └── interop.hpp         # Eigen conversion, EigenSoA3, batch ops
└── geometry/               # ✅ IMPLEMENTED
    ├── vector3.hpp         # Vector3<T> - SIMD-backed 3D vector
    ├── vector4.hpp         # Vector4<T> - SIMD-backed 4D vector
    ├── matrix3x3.hpp       # Matrix3x3<T> - column-major 3x3 matrix
    └── matrix4x4.hpp       # Matrix4x4<T> - column-major 4x4 matrix
```

**Planned additions**:

```
├── eigen/
│   ├── interop.hpp         # Existing (EigenSoA3, EigenSoA4)
│   └── batch.hpp           # [Sprint 5] ECS batch operations, streaming iterator
├── geometry/
│   ├── quaternion.hpp      # [Sprint 7] Quaternion<T> - unit quaternion
│   ├── isometry3.hpp       # [Sprint 7] Isometry3<T> - rigid transform
│   └── ...                 # Existing types
└── dynamic/                # [Sprint 8] Runtime-sized types
    ├── vector.hpp          # DynamicVector<T>
    └── matrix.hpp          # DynamicMatrix<T>
```

---

## Highest Priority Next Tasks

### 1. **ECS Batch Operations (Sprint 5)** - HIGH PRIORITY

**Why**: Geometric types are done, but physics/collision need to process many entities at once.

**Scope**:

- Extend `EigenSoA` with 4D vectors for full transform support
- Add batch matrix operations: `transform_points()`, `transform_vectors()`
- Streaming iterator for processing large arrays in SIMD-width chunks
- Zero-copy views over existing ECS storage (EnTT, etc.)

**Success criteria**: Batch transform of 1000 vectors faster than scalar loop.

### 2. **FMA Optimization** - MEDIUM PRIORITY

**Why**: Closing performance gaps improves physics simulation.

**Investigation areas**:

- Loop unrolling (optimize elements/iteration)
- Memory prefetching
- Instruction scheduling

### 3. **Integration into Collision Detection** - MEDIUM PRIORITY

**Why**: SIMD layer is useless without real-world application.

**Target**: `feature/new_coll` branch has collision detection work. Apply dart::simd to:

- Batch AABB overlap tests
- Broadphase acceleration
- GJK/EPA support queries

---

## Questions Resolved

1. **Vector3 storage**: Use `Vec<T, 4>` (padded) for SIMD efficiency, ignore padding lane ✅
2. **Matrix storage**: Column-major (matches Eigen default, better for SIMD mat-vec) ✅
3. **Naming**: `Vector3` (clear, matches physics convention) ✅
4. **Namespace**: Keep in `dart::simd` with optional `geometry/` subdirectory ✅

---

## Success Criteria (UPDATED)

1. [x] All x86-64 ISAs from SSE4.2 to AVX-512 supported
2. [x] ARM NEON supported (SVE deferred)
3. [x] Benchmarks run with `pixi run bm-simd`
4. [x] DART SIMD competitive with optimized implementations
5. [x] `Vector3<T>` and `Matrix4x4<T>` with SIMD storage and Eigen interop
6. [x] Batch transform operations (transformPoints, transformVectors) implemented
7. [x] Zero-copy Eigen interop via `asEigen()` for DynamicVector/DynamicMatrix
8. [x] All CI platforms green
9. [x] Naming convention follows DART C++ style (camelCase)
