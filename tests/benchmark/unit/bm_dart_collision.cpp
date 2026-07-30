#include <dart/config.hpp>

#include <dart/collision/CollisionDetector.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionResult.hpp>
#include <dart/collision/DistanceOption.hpp>
#include <dart/collision/DistanceResult.hpp>
#include <dart/collision/RaycastOption.hpp>
#include <dart/collision/RaycastResult.hpp>
#if HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <dart/collision/dart/Types.hpp>
#include <dart/collision/dart/narrow_phase/NarrowPhase.hpp>
#include <dart/collision/dart/shapes/Shape.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>
#if HAVE_OCTOMAP
  #include <dart/dynamics/VoxelGridShape.hpp>
#endif

#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <memory>
#include <vector>

#include <cstddef>

namespace {

using dart::collision::native::BoxShape;
using dart::collision::native::CapsuleShape;
using dart::collision::native::CollisionOption;
using dart::collision::native::CollisionResult;
using dart::collision::native::ConvexShape;
using dart::collision::native::CylinderShape;
using dart::collision::native::MeshShape;
using dart::collision::native::NarrowPhase;
using dart::collision::native::PlaneShape;
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

MeshShape makeUnitCubeMesh()
{
  std::vector<Eigen::Vector3d> vertices
      = {{-0.5, -0.5, -0.5},
         {0.5, -0.5, -0.5},
         {0.5, 0.5, -0.5},
         {-0.5, 0.5, -0.5},
         {-0.5, -0.5, 0.5},
         {0.5, -0.5, 0.5},
         {0.5, 0.5, 0.5},
         {-0.5, 0.5, 0.5}};
  std::vector<MeshShape::Triangle> triangles
      = {{0, 1, 2},
         {0, 2, 3},
         {4, 6, 5},
         {4, 7, 6},
         {0, 5, 1},
         {0, 4, 5},
         {2, 6, 7},
         {2, 7, 3},
         {0, 7, 4},
         {0, 3, 7},
         {1, 5, 6},
         {1, 6, 2}};

  return MeshShape(std::move(vertices), std::move(triangles));
}

MeshShape makePlaneMesh()
{
  std::vector<Eigen::Vector3d> vertices = {
      {-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0}, {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0}};
  std::vector<MeshShape::Triangle> triangles = {{0, 1, 2}, {0, 2, 3}};
  return MeshShape(std::move(vertices), std::move(triangles));
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
    state.SkipWithError("DART narrow-phase benchmark case did not collide");
  }

  state.counters["contacts/iter"] = benchmark::Counter(
      static_cast<double>(contactCount)
      / static_cast<double>(state.iterations()));
}

std::shared_ptr<dart::dynamics::SimpleFrame> makeDistanceFrame(
    const dart::dynamics::ShapePtr& shape,
    const Eigen::Vector3d& translation = Eigen::Vector3d::Zero())
{
  auto frame = dart::dynamics::SimpleFrame::createShared(
      dart::dynamics::Frame::World());
  frame->setShape(shape);
  frame->setTranslation(translation);
  return frame;
}

#if HAVE_OCTOMAP
std::shared_ptr<dart::dynamics::VoxelGridShape> makeOccupiedVoxelGrid()
{
  auto voxelGrid = std::make_shared<dart::dynamics::VoxelGridShape>(0.1);
  voxelGrid->updateOccupancy(Eigen::Vector3d::Zero(), true);
  return voxelGrid;
}
#endif

void runDetectorCollision(
    benchmark::State& state,
    const dart::collision::CollisionDetectorPtr& detector,
    const dart::dynamics::ShapePtr& shapeA,
    const Eigen::Vector3d& translationA,
    const dart::dynamics::ShapePtr& shapeB,
    const Eigen::Vector3d& translationB)
{
  auto frameA = makeDistanceFrame(shapeA, translationA);
  auto frameB = makeDistanceFrame(shapeB, translationB);
  auto group = detector->createCollisionGroup(frameA.get(), frameB.get());

  dart::collision::CollisionOption option(true, 8u);
  dart::collision::CollisionResult result;
  std::size_t contactCount = 0u;
  bool hit = false;

  for (auto _ : state) {
    result.clear();
    hit = group->collide(option, &result);
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(result.getNumContacts());
    contactCount += result.getNumContacts();
  }

  if (!hit) {
    state.SkipWithError("collision benchmark case did not collide");
  }

  state.counters["contacts/iter"] = benchmark::Counter(
      static_cast<double>(contactCount)
      / static_cast<double>(state.iterations()));
}

#if HAVE_OCTOMAP
void runDARTDetectorCollision(
    benchmark::State& state,
    const dart::dynamics::ShapePtr& shapeA,
    const Eigen::Vector3d& translationA,
    const dart::dynamics::ShapePtr& shapeB,
    const Eigen::Vector3d& translationB)
{
  runDetectorCollision(
      state,
      dart::collision::DARTCollisionDetector::create(),
      shapeA,
      translationA,
      shapeB,
      translationB);
}
#endif

#if HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
void runFclDetectorCollision(
    benchmark::State& state,
    const dart::dynamics::ShapePtr& shapeA,
    const Eigen::Vector3d& translationA,
    const dart::dynamics::ShapePtr& shapeB,
    const Eigen::Vector3d& translationB)
{
  runDetectorCollision(
      state,
      dart::collision::FCLCollisionDetector::create(),
      shapeA,
      translationA,
      shapeB,
      translationB);
}
#endif

void runDetectorDistance(
    benchmark::State& state,
    const dart::collision::CollisionDetectorPtr& detector,
    const dart::dynamics::ShapePtr& shapeA,
    const Eigen::Vector3d& translationA,
    const dart::dynamics::ShapePtr& shapeB,
    const Eigen::Vector3d& translationB)
{
  auto frameA = makeDistanceFrame(shapeA, translationA);
  auto frameB = makeDistanceFrame(shapeB, translationB);
  auto group = detector->createCollisionGroup(frameA.get(), frameB.get());

  dart::collision::DistanceOption option(true, 0.0, nullptr);
  dart::collision::DistanceResult result;
  double distance = 0.0;

  for (auto _ : state) {
    result.clear();
    distance = group->distance(option, &result);
    benchmark::DoNotOptimize(distance);
    benchmark::DoNotOptimize(result.unclampedMinDistance);
  }

  if (!result.found()) {
    state.SkipWithError("distance benchmark case did not produce a result");
  }
}

void runDARTDetectorDistance(
    benchmark::State& state,
    const dart::dynamics::ShapePtr& shapeA,
    const Eigen::Vector3d& translationA,
    const dart::dynamics::ShapePtr& shapeB,
    const Eigen::Vector3d& translationB)
{
  runDetectorDistance(
      state,
      dart::collision::DARTCollisionDetector::create(),
      shapeA,
      translationA,
      shapeB,
      translationB);
}

void runFclDetectorDistance(
    benchmark::State& state,
    const dart::dynamics::ShapePtr& shapeA,
    const Eigen::Vector3d& translationA,
    const dart::dynamics::ShapePtr& shapeB,
    const Eigen::Vector3d& translationB)
{
  runDetectorDistance(
      state,
      dart::collision::FCLCollisionDetector::create(),
      shapeA,
      translationA,
      shapeB,
      translationB);
}

void runDetectorRaycast(
    benchmark::State& state,
    const dart::collision::CollisionDetectorPtr& detector,
    const dart::dynamics::ShapePtr& shape,
    const Eigen::Vector3d& translation)
{
  auto frame = makeDistanceFrame(shape, translation);
  auto group = detector->createCollisionGroup(frame.get());

  const Eigen::Vector3d from(-5.0, 0.0, 0.0);
  const Eigen::Vector3d to(5.0, 0.0, 0.0);
  dart::collision::RaycastOption option(false, false, nullptr);
  dart::collision::RaycastResult result;
  bool hit = false;

  for (auto _ : state) {
    result.clear();
    hit = group->raycast(from, to, option, &result);
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(result.mRayHits.size());
  }

  if (!hit) {
    state.SkipWithError("raycast benchmark case did not produce a hit");
  }
}

void runDARTDetectorRaycast(
    benchmark::State& state,
    const dart::dynamics::ShapePtr& shape,
    const Eigen::Vector3d& translation = Eigen::Vector3d::Zero())
{
  runDetectorRaycast(
      state,
      dart::collision::DARTCollisionDetector::create(),
      shape,
      translation);
}

#if HAVE_BULLET
void runBulletDetectorRaycast(
    benchmark::State& state,
    const dart::dynamics::ShapePtr& shape,
    const Eigen::Vector3d& translation = Eigen::Vector3d::Zero())
{
  runDetectorRaycast(
      state,
      dart::collision::BulletCollisionDetector::create(),
      shape,
      translation);
}
#endif

void BM_DARTNarrowPhaseSphereSphere(benchmark::State& state)
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

void BM_DARTNarrowPhaseSphereBox(benchmark::State& state)
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

void BM_DARTNarrowPhaseBoxBox(benchmark::State& state)
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

void BM_DARTNarrowPhaseCapsuleSphere(benchmark::State& state)
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

void BM_DARTNarrowPhaseCapsuleBox(benchmark::State& state)
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

void BM_DARTNarrowPhaseCapsuleCapsule(benchmark::State& state)
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

void BM_DARTNarrowPhaseConvexConvex(benchmark::State& state)
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

void BM_DARTNarrowPhaseCylinderCylinder(benchmark::State& state)
{
  CylinderShape cylinderA(0.5, 1.0);
  CylinderShape cylinderB(0.5, 1.0);

  runNarrowPhase(
      state,
      cylinderA,
      Eigen::Isometry3d::Identity(),
      cylinderB,
      translated(0.75, 0.0, 0.0));
}

void BM_DARTNarrowPhaseCylinderSphere(benchmark::State& state)
{
  CylinderShape cylinder(0.5, 1.0);
  SphereShape sphere(0.5);

  runNarrowPhase(
      state,
      cylinder,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.75, 0.0, 0.0));
}

void BM_DARTNarrowPhaseCylinderBox(benchmark::State& state)
{
  CylinderShape cylinder(0.5, 1.0);
  BoxShape box(Eigen::Vector3d::Constant(0.5));

  runNarrowPhase(
      state,
      cylinder,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.75, 0.0, 0.0));
}

void BM_DARTNarrowPhaseCylinderCapsule(benchmark::State& state)
{
  CylinderShape cylinder(0.5, 1.0);
  CapsuleShape capsule(0.5, 1.0);

  runNarrowPhase(
      state,
      cylinder,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.75, 0.0, 0.0));
}

void BM_DARTNarrowPhaseCylinderPlane(benchmark::State& state)
{
  CylinderShape cylinder(0.5, 1.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  runNarrowPhase(
      state,
      cylinder,
      translated(0.0, 0.0, 0.25),
      plane,
      Eigen::Isometry3d::Identity());
}

void BM_DARTNarrowPhasePlaneSphere(benchmark::State& state)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(1.0);

  runNarrowPhase(
      state,
      plane,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.0, 0.0, 0.5));
}

void BM_DARTNarrowPhasePlaneBox(benchmark::State& state)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  BoxShape box(Eigen::Vector3d::Constant(1.0));

  runNarrowPhase(
      state,
      plane,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.0, 0.0, 0.5));
}

void BM_DARTNarrowPhasePlaneCapsule(benchmark::State& state)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CapsuleShape capsule(0.5, 2.0);

  runNarrowPhase(
      state,
      plane,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.0, 0.0, 1.3));
}

void BM_DARTNarrowPhasePlaneConvex(benchmark::State& state)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  ConvexShape convex(
      {Eigen::Vector3d(-0.5, -0.5, -0.25),
       Eigen::Vector3d(0.5, -0.5, -0.25),
       Eigen::Vector3d(0.0, 0.5, -0.25),
       Eigen::Vector3d(0.0, 0.0, 0.75)});

  runNarrowPhase(
      state,
      plane,
      Eigen::Isometry3d::Identity(),
      convex,
      Eigen::Isometry3d::Identity());
}

void BM_DARTNarrowPhaseConvexCylinder(benchmark::State& state)
{
  ConvexShape convex(makeOctahedronVertices());
  CylinderShape cylinder(0.5, 1.0);

  runNarrowPhase(
      state,
      convex,
      Eigen::Isometry3d::Identity(),
      cylinder,
      translated(1.25, 0.0, 0.0));
}

void BM_DARTNarrowPhaseMeshMesh(benchmark::State& state)
{
  MeshShape meshA = makeUnitCubeMesh();
  MeshShape meshB = makeUnitCubeMesh();

  runNarrowPhase(
      state,
      meshA,
      Eigen::Isometry3d::Identity(),
      meshB,
      translated(0.25, 0.0, 0.0));
}

void BM_DARTNarrowPhaseMeshSphere(benchmark::State& state)
{
  MeshShape mesh = makePlaneMesh();
  SphereShape sphere(0.5);

  runNarrowPhase(
      state,
      mesh,
      Eigen::Isometry3d::Identity(),
      sphere,
      translated(0.25, -0.25, 0.25));
}

void BM_DARTNarrowPhaseMeshBox(benchmark::State& state)
{
  MeshShape mesh = makePlaneMesh();
  BoxShape box(Eigen::Vector3d::Constant(0.25));

  runNarrowPhase(
      state,
      mesh,
      Eigen::Isometry3d::Identity(),
      box,
      translated(0.0, 0.0, 0.2));
}

void BM_DARTNarrowPhaseMeshCapsule(benchmark::State& state)
{
  MeshShape mesh = makePlaneMesh();
  CapsuleShape capsule(0.25, 0.5);

  runNarrowPhase(
      state,
      mesh,
      Eigen::Isometry3d::Identity(),
      capsule,
      translated(0.25, -0.25, 0.25));
}

void BM_DARTNarrowPhaseMeshConvex(benchmark::State& state)
{
  MeshShape mesh = makePlaneMesh();
  ConvexShape convex(makeOctahedronVertices(0.25));

  runNarrowPhase(
      state,
      mesh,
      Eigen::Isometry3d::Identity(),
      convex,
      translated(0.0, 0.0, 0.2));
}

#if HAVE_OCTOMAP
void BM_CollisionDARTVoxelGridSphere(benchmark::State& state)
{
  runDARTDetectorCollision(
      state,
      makeOccupiedVoxelGrid(),
      Eigen::Vector3d::Zero(),
      std::make_shared<dart::dynamics::SphereShape>(0.01),
      Eigen::Vector3d::Zero());
}
#endif

#if HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
void BM_CollisionFclVoxelGridSphere(benchmark::State& state)
{
  runFclDetectorCollision(
      state,
      makeOccupiedVoxelGrid(),
      Eigen::Vector3d::Zero(),
      std::make_shared<dart::dynamics::SphereShape>(0.01),
      Eigen::Vector3d::Zero());
}
#endif

void BM_DistanceDARTSphereSphere(benchmark::State& state)
{
  runDARTDetectorDistance(
      state,
      std::make_shared<dart::dynamics::SphereShape>(0.25),
      Eigen::Vector3d::Zero(),
      std::make_shared<dart::dynamics::SphereShape>(0.5),
      Eigen::Vector3d(2.0, 0.0, 0.0));
}

void BM_DistanceFclSphereSphere(benchmark::State& state)
{
  runFclDetectorDistance(
      state,
      std::make_shared<dart::dynamics::SphereShape>(0.25),
      Eigen::Vector3d::Zero(),
      std::make_shared<dart::dynamics::SphereShape>(0.5),
      Eigen::Vector3d(2.0, 0.0, 0.0));
}

void BM_DistanceDARTSphereBox(benchmark::State& state)
{
  runDARTDetectorDistance(
      state,
      std::make_shared<dart::dynamics::SphereShape>(0.25),
      Eigen::Vector3d::Zero(),
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d(1.25, 0.0, 0.0));
}

void BM_DistanceFclSphereBox(benchmark::State& state)
{
  runFclDetectorDistance(
      state,
      std::make_shared<dart::dynamics::SphereShape>(0.25),
      Eigen::Vector3d::Zero(),
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d(1.25, 0.0, 0.0));
}

void BM_DistanceDARTBoxBox(benchmark::State& state)
{
  runDARTDetectorDistance(
      state,
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d::Zero(),
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d(1.25, 0.0, 0.0));
}

void BM_DistanceFclBoxBox(benchmark::State& state)
{
  runFclDetectorDistance(
      state,
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d::Zero(),
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d::Constant(0.5)),
      Eigen::Vector3d(1.25, 0.0, 0.0));
}

void BM_DistanceDARTPlaneSphere(benchmark::State& state)
{
  runDARTDetectorDistance(
      state,
      std::make_shared<dart::dynamics::PlaneShape>(
          Eigen::Vector3d::UnitZ(), 0.0),
      Eigen::Vector3d::Zero(),
      std::make_shared<dart::dynamics::SphereShape>(0.25),
      Eigen::Vector3d(0.0, 0.0, 0.75));
}

void BM_DistanceFclPlaneSphere(benchmark::State& state)
{
  runFclDetectorDistance(
      state,
      std::make_shared<dart::dynamics::PlaneShape>(
          Eigen::Vector3d::UnitZ(), 0.0),
      Eigen::Vector3d::Zero(),
      std::make_shared<dart::dynamics::SphereShape>(0.25),
      Eigen::Vector3d(0.0, 0.0, 0.75));
}

void BM_RaycastDARTSphere(benchmark::State& state)
{
  runDARTDetectorRaycast(
      state, std::make_shared<dart::dynamics::SphereShape>(1.0));
}

void BM_RaycastDARTBox(benchmark::State& state)
{
  runDARTDetectorRaycast(
      state,
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(2.0, 2.0, 2.0)));
}

#if HAVE_BULLET
void BM_RaycastBulletSphere(benchmark::State& state)
{
  runBulletDetectorRaycast(
      state, std::make_shared<dart::dynamics::SphereShape>(1.0));
}

void BM_RaycastBulletBox(benchmark::State& state)
{
  runBulletDetectorRaycast(
      state,
      std::make_shared<dart::dynamics::BoxShape>(
          Eigen::Vector3d(2.0, 2.0, 2.0)));
}
#endif

} // namespace

BENCHMARK(BM_DARTNarrowPhaseSphereSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseSphereBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseBoxBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseCapsuleSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseCapsuleBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseCapsuleCapsule)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseConvexConvex)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseCylinderCylinder)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseCylinderSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseCylinderBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseCylinderCapsule)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseCylinderPlane)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhasePlaneSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhasePlaneBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhasePlaneCapsule)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhasePlaneConvex)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseConvexCylinder)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseMeshMesh)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseMeshSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseMeshBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseMeshCapsule)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DARTNarrowPhaseMeshConvex)->Unit(benchmark::kNanosecond);
#if HAVE_OCTOMAP
BENCHMARK(BM_CollisionDARTVoxelGridSphere)->Unit(benchmark::kNanosecond);
#endif
#if HAVE_OCTOMAP && FCL_HAVE_OCTOMAP
BENCHMARK(BM_CollisionFclVoxelGridSphere)->Unit(benchmark::kNanosecond);
#endif
BENCHMARK(BM_DistanceDARTSphereSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DistanceFclSphereSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DistanceDARTSphereBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DistanceFclSphereBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DistanceDARTBoxBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DistanceFclBoxBox)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DistanceDARTPlaneSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_DistanceFclPlaneSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_RaycastDARTSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_RaycastDARTBox)->Unit(benchmark::kNanosecond);
#if HAVE_BULLET
BENCHMARK(BM_RaycastBulletSphere)->Unit(benchmark::kNanosecond);
BENCHMARK(BM_RaycastBulletBox)->Unit(benchmark::kNanosecond);
#endif
