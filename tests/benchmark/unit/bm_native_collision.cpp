#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/shapes/shape.hpp>
#include <dart/collision/native/types.hpp>

#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <vector>

#include <cstddef>

namespace {

using dart::collision::native::BoxShape;
using dart::collision::native::CapsuleShape;
using dart::collision::native::CollisionOption;
using dart::collision::native::CollisionResult;
using dart::collision::native::ConvexShape;
using dart::collision::native::NarrowPhase;
using dart::collision::native::Shape;
using dart::collision::native::SphereShape;

Eigen::Isometry3d translated(double x, double y, double z)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(x, y, z);
  return transform;
}

std::vector<Eigen::Vector3d> makeOctahedronVertices(double scale = 1.0)
{
  return {
      {scale, 0.0, 0.0},
      {-scale, 0.0, 0.0},
      {0.0, scale, 0.0},
      {0.0, -scale, 0.0},
      {0.0, 0.0, scale},
      {0.0, 0.0, -scale}};
}

void runNarrowPhase(
    benchmark::State& state,
    const Shape& shapeA,
    const Eigen::Isometry3d& tfA,
    const Shape& shapeB,
    const Eigen::Isometry3d& tfB)
{
  CollisionOption option = CollisionOption::fullContacts(8);
  CollisionResult result;
  std::size_t contactCount = 0;
  bool hit = false;

  for (auto _ : state) {
    result.clear();
    hit = NarrowPhase::collide(&shapeA, tfA, &shapeB, tfB, option, result);
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(result.numContacts());
    contactCount += result.numContacts();
  }

  if (!hit) {
    state.SkipWithError("native narrow-phase benchmark case did not collide");
  }

  state.counters["contacts/iter"] = benchmark::Counter(
      static_cast<double>(contactCount)
      / static_cast<double>(state.iterations()));
}

void BM_NativeSphereSphere(benchmark::State& state)
{
  SphereShape sphereA(1.0);
  SphereShape sphereB(1.0);

  runNarrowPhase(
      state,
      sphereA,
      Eigen::Isometry3d::Identity(),
      sphereB,
      translated(1.25, 0.0, 0.0));
}

void BM_NativeSphereBox(benchmark::State& state)
{
  SphereShape sphere(1.0);
  BoxShape box(Eigen::Vector3d::Constant(1.0));

  runNarrowPhase(
      state,
      sphere,
      translated(0.0, 0.0, 1.5),
      box,
      Eigen::Isometry3d::Identity());
}

void BM_NativeBoxBox(benchmark::State& state)
{
  BoxShape boxA(Eigen::Vector3d::Constant(1.0));
  BoxShape boxB(Eigen::Vector3d::Constant(1.0));

  runNarrowPhase(
      state,
      boxA,
      Eigen::Isometry3d::Identity(),
      boxB,
      translated(0.75, 0.1, 0.05));
}

void BM_NativeCapsuleSphere(benchmark::State& state)
{
  CapsuleShape capsule(0.5, 1.0);
  SphereShape sphere(0.5);

  runNarrowPhase(
      state,
      capsule,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0));
}

void BM_NativeCapsuleBox(benchmark::State& state)
{
  CapsuleShape capsule(0.5, 1.0);
  BoxShape box(Eigen::Vector3d::Constant(1.0));

  runNarrowPhase(
      state,
      capsule,
      translated(1.25, 0.0, 0.0),
      box,
      Eigen::Isometry3d::Identity());
}

void BM_NativeCapsuleCapsule(benchmark::State& state)
{
  CapsuleShape capsuleA(0.5, 1.0);
  CapsuleShape capsuleB(0.5, 1.0);

  runNarrowPhase(
      state,
      capsuleA,
      Eigen::Isometry3d::Identity(),
      capsuleB,
      translated(0.75, 0.0, 0.0));
}

void BM_NativeConvexConvex(benchmark::State& state)
{
  ConvexShape convexA(makeOctahedronVertices());
  ConvexShape convexB(makeOctahedronVertices());

  runNarrowPhase(
      state,
      convexA,
      Eigen::Isometry3d::Identity(),
      convexB,
      translated(1.5, 0.0, 0.0));
}

} // namespace

BENCHMARK(BM_NativeSphereSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_NativeSphereBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_NativeBoxBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_NativeCapsuleSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_NativeCapsuleBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_NativeCapsuleCapsule)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_NativeConvexConvex)->Unit(benchmark::kNanosecond);
