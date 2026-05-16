/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/config.hpp>

#include <dart/collision/dart/dart_collision_detector.hpp>
#include <dart/test/reference_collision/fcl/fcl_collision_detector.hpp>
#include <dart/collision/native/narrow_phase/box_box.hpp>
#include <dart/collision/native/narrow_phase/capsule_box.hpp>
#include <dart/collision/native/narrow_phase/capsule_capsule.hpp>
#include <dart/collision/native/narrow_phase/capsule_sphere.hpp>
#include <dart/collision/native/narrow_phase/convex_convex.hpp>
#include <dart/collision/native/narrow_phase/cylinder_collision.hpp>
#include <dart/collision/native/narrow_phase/mesh_mesh.hpp>
#include <dart/collision/native/narrow_phase/narrow_phase.hpp>
#include <dart/collision/native/narrow_phase/plane_sphere.hpp>
#include <dart/collision/native/narrow_phase/sphere_box.hpp>
#include <dart/collision/native/narrow_phase/sphere_sphere.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/capsule_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/plane_shape.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/sphere_shape.hpp>

#if DART_HAVE_BULLET
  #include <dart/test/reference_collision/bullet/bullet_include.hpp>
  #include <dart/test/reference_collision/bullet/bullet_collision_detector.hpp>
#endif

#if DART_HAVE_ODE
  #include <dart/test/reference_collision/ode/ode_collision_detector.hpp>
  #include <ode/ode.h>
#endif

#include "tests/benchmark/collision/fixtures/edge_cases.hpp"
#include "tests/benchmark/collision/fixtures/shape_factories.hpp"

#include <benchmark/benchmark.h>

#include <fcl/geometry/shape/capsule.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/narrowphase/collision.h>

#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <cmath>

using namespace dart::collision::native;

namespace {

struct PairContext
{
  dart::dynamics::SkeletonPtr skel1;
  dart::dynamics::SkeletonPtr skel2;
  dart::collision::CollisionGroupPtr group;
};

template <typename DetectorPtr>
PairContext MakePairContext(
    const DetectorPtr& detector,
    const std::shared_ptr<dart::dynamics::Shape>& shape1,
    const Eigen::Isometry3d& tf1,
    const std::shared_ptr<dart::dynamics::Shape>& shape2,
    const Eigen::Isometry3d& tf2)
{
  PairContext ctx;
  ctx.skel1
      = dart::benchmark::collision::CreateSingleShapeSkeleton(shape1, tf1);
  ctx.skel2
      = dart::benchmark::collision::CreateSingleShapeSkeleton(shape2, tf2);
  ctx.group = detector->createCollisionGroup();
  dart::benchmark::collision::AddSkeletonToGroup(ctx.group.get(), ctx.skel1);
  dart::benchmark::collision::AddSkeletonToGroup(ctx.group.get(), ctx.skel2);
  return ctx;
}

template <typename DetectorPtr>
void RunDetectorBenchmark(
    benchmark::State& state,
    const DetectorPtr& detector,
    const std::shared_ptr<dart::dynamics::Shape>& shape1,
    const Eigen::Isometry3d& tf1,
    const std::shared_ptr<dart::dynamics::Shape>& shape2,
    const Eigen::Isometry3d& tf2)
{
  auto ctx = MakePairContext(detector, shape1, tf1, shape2, tf2);
  auto option = dart::benchmark::collision::MakeCollisionOption();
  dart::collision::CollisionResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        detector->collide(ctx.group.get(), option, &result));
  }
}

Eigen::Isometry3d OffsetTransform(double x, double y, double z)
{
  return dart::benchmark::collision::MakeTransform(Eigen::Vector3d(x, y, z));
}

std::vector<Eigen::Vector3d> MakeBenchmarkOctahedronVertices(double scale)
{
  return {
      {scale, 0.0, 0.0},
      {-scale, 0.0, 0.0},
      {0.0, scale, 0.0},
      {0.0, -scale, 0.0},
      {0.0, 0.0, scale},
      {0.0, 0.0, -scale}};
}

std::vector<Eigen::Vector3d> MakeBenchmarkCubeMeshVertices(double halfExtent)
{
  const double h = halfExtent;
  return {
      {-h, -h, -h},
      {h, -h, -h},
      {h, h, -h},
      {-h, h, -h},
      {-h, -h, h},
      {h, -h, h},
      {h, h, h},
      {-h, h, h}};
}

std::vector<MeshShape::Triangle> MakeBenchmarkCubeMeshTriangles()
{
  return {
      {0, 2, 1},
      {0, 3, 2},
      {4, 5, 6},
      {4, 6, 7},
      {0, 1, 5},
      {0, 5, 4},
      {2, 3, 7},
      {2, 7, 6},
      {0, 4, 7},
      {0, 7, 3},
      {1, 2, 6},
      {1, 6, 5}};
}

enum class RawCollisionSanity
{
  kRequireCollision,
  kAllowNoCollision
};

fcl::Transform3<double> MakeFclTransform(const Eigen::Isometry3d& tf)
{
  fcl::Transform3<double> fclTf = fcl::Transform3<double>::Identity();
  fclTf.linear() = tf.linear();
  fclTf.translation() = tf.translation();
  return fclTf;
}

void RunFclRawCollisionBenchmark(
    benchmark::State& state,
    const std::shared_ptr<fcl::CollisionGeometry<double>>& geometry1,
    const Eigen::Isometry3d& tf1,
    const std::shared_ptr<fcl::CollisionGeometry<double>>& geometry2,
    const Eigen::Isometry3d& tf2,
    RawCollisionSanity sanity = RawCollisionSanity::kRequireCollision)
{
  fcl::CollisionObject<double> object1(geometry1, MakeFclTransform(tf1));
  fcl::CollisionObject<double> object2(geometry2, MakeFclTransform(tf2));
  object1.computeAABB();
  object2.computeAABB();

  fcl::CollisionRequest<double> request;
  request.enable_contact = true;
  request.num_max_contacts = 8u;
  fcl::CollisionResult<double> result;

  benchmark::DoNotOptimize(fcl::collide(&object1, &object2, request, result));
  if (sanity == RawCollisionSanity::kRequireCollision
      && !result.isCollision()) {
    state.SkipWithError("FCL raw benchmark setup did not collide.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        fcl::collide(&object1, &object2, request, result));
  }
}

#if DART_HAVE_BULLET
btTransform MakeBulletTransform(const Eigen::Isometry3d& tf)
{
  btMatrix3x3 basis(
      static_cast<btScalar>(tf.linear()(0, 0)),
      static_cast<btScalar>(tf.linear()(0, 1)),
      static_cast<btScalar>(tf.linear()(0, 2)),
      static_cast<btScalar>(tf.linear()(1, 0)),
      static_cast<btScalar>(tf.linear()(1, 1)),
      static_cast<btScalar>(tf.linear()(1, 2)),
      static_cast<btScalar>(tf.linear()(2, 0)),
      static_cast<btScalar>(tf.linear()(2, 1)),
      static_cast<btScalar>(tf.linear()(2, 2)));
  btTransform bulletTf;
  bulletTf.setBasis(basis);
  bulletTf.setOrigin(btVector3(
      static_cast<btScalar>(tf.translation().x()),
      static_cast<btScalar>(tf.translation().y()),
      static_cast<btScalar>(tf.translation().z())));
  return bulletTf;
}

struct CountingBulletContactCallback final
  : public btCollisionWorld::ContactResultCallback
{
  btScalar addSingleResult(
      btManifoldPoint&,
      const btCollisionObjectWrapper*,
      int,
      int,
      const btCollisionObjectWrapper*,
      int,
      int) override
  {
    ++numContacts;
    return 0.0;
  }

  int numContacts{0};
};

void RunBulletRawCollisionBenchmark(
    benchmark::State& state,
    btCollisionShape& shape1,
    const Eigen::Isometry3d& tf1,
    btCollisionShape& shape2,
    const Eigen::Isometry3d& tf2,
    RawCollisionSanity sanity = RawCollisionSanity::kRequireCollision)
{
  btDefaultCollisionConfiguration config;
  btCollisionDispatcher dispatcher(&config);
  btDbvtBroadphase broadphase;
  btCollisionWorld world(&dispatcher, &broadphase, &config);

  btCollisionObject object1;
  object1.setCollisionShape(&shape1);
  object1.setWorldTransform(MakeBulletTransform(tf1));

  btCollisionObject object2;
  object2.setCollisionShape(&shape2);
  object2.setWorldTransform(MakeBulletTransform(tf2));

  CountingBulletContactCallback sanityCheck;
  world.contactPairTest(&object1, &object2, sanityCheck);
  if (sanity == RawCollisionSanity::kRequireCollision
      && sanityCheck.numContacts == 0) {
    state.SkipWithError("Bullet raw benchmark setup did not collide.");
    return;
  }

  for (auto _ : state) {
    CountingBulletContactCallback callback;
    world.contactPairTest(&object1, &object2, callback);
    benchmark::DoNotOptimize(callback.numContacts);
  }
}
#endif

#if DART_HAVE_ODE
struct OdeRuntime
{
  OdeRuntime()
  {
    const auto initialized = dInitODE2(0);
    DART_ASSERT(initialized);
    DART_UNUSED(initialized);
    dAllocateODEDataForThread(dAllocateMaskAll);
  }

  ~OdeRuntime()
  {
    dCloseODE();
  }
};

void SetOdeTransform(dGeomID geom, const Eigen::Isometry3d& tf)
{
  dGeomSetPosition(
      geom, tf.translation().x(), tf.translation().y(), tf.translation().z());

  dMatrix3 rotation;
  rotation[0] = tf.linear()(0, 0);
  rotation[1] = tf.linear()(0, 1);
  rotation[2] = tf.linear()(0, 2);
  rotation[3] = 0.0;
  rotation[4] = tf.linear()(1, 0);
  rotation[5] = tf.linear()(1, 1);
  rotation[6] = tf.linear()(1, 2);
  rotation[7] = 0.0;
  rotation[8] = tf.linear()(2, 0);
  rotation[9] = tf.linear()(2, 1);
  rotation[10] = tf.linear()(2, 2);
  rotation[11] = 0.0;
  dGeomSetRotation(geom, rotation);
}

void RunOdeRawCollisionBenchmark(
    benchmark::State& state,
    dGeomID geom1,
    const Eigen::Isometry3d& tf1,
    dGeomID geom2,
    const Eigen::Isometry3d& tf2,
    RawCollisionSanity sanity = RawCollisionSanity::kRequireCollision)
{
  SetOdeTransform(geom1, tf1);
  SetOdeTransform(geom2, tf2);

  std::array<dContactGeom, 8> contacts;
  auto sanityContacts = dCollide(
      geom1,
      geom2,
      static_cast<int>(contacts.size()),
      contacts.data(),
      sizeof(dContactGeom));
  if (sanity == RawCollisionSanity::kRequireCollision
      && sanityContacts <= 0) {
    state.SkipWithError("ODE raw benchmark setup did not collide.");
    return;
  }

  for (auto _ : state) {
    auto numContacts = dCollide(
        geom1,
        geom2,
        static_cast<int>(contacts.size()),
        contacts.data(),
        sizeof(dContactGeom));
    benchmark::DoNotOptimize(numContacts);
  }
}
#endif

} // namespace

static void BM_NarrowPhase_SphereSphere_Native(benchmark::State& state)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(1.5, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideSpheres(s1, tf1, s2, tf2, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_SphereSphere_Native);

static std::vector<SpherePair> MakeSphereSphereBatchPairs(
    std::size_t batchSize, const SphereShape& sphere1, const SphereShape& sphere2)
{
  std::mt19937 rng(314159u);
  std::uniform_real_distribution<double> offset(-0.10, 0.10);

  std::vector<SpherePair> pairs;
  pairs.reserve(batchSize);
  for (std::size_t i = 0; i < batchSize; ++i) {
    Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    tf1.translation() = Eigen::Vector3d(
        offset(rng), offset(rng), offset(rng));

    Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
    tf2.translation()
        = Eigen::Vector3d(0.85 + offset(rng), offset(rng), offset(rng));

    pairs.push_back(SpherePair{&sphere1, &sphere2, tf1, tf2});
  }
  return pairs;
}

static void BM_NarrowPhase_SphereSphere_Native_Batch(
    benchmark::State& state, std::size_t batchSize)
{
  SphereShape s1(0.75);
  SphereShape s2(0.60);

  const auto pairs = MakeSphereSphereBatchPairs(batchSize, s1, s2);
  std::vector<CollisionResult> results(batchSize);
  CollisionOption option;

  for (auto _ : state) {
    for (auto& result : results)
      result.clear();
    collideSpheresBatch(pairs, results, option);
    benchmark::DoNotOptimize(results.data());
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations())
      * static_cast<int64_t>(batchSize));
  state.counters["pairs_per_iteration"] = static_cast<double>(batchSize);
}

struct SphereSphereBatchBenchmarkRegistrar
{
  SphereSphereBatchBenchmarkRegistrar()
  {
    for (const std::size_t batchSize : {1u, 10u, 100u, 1000u}) {
      const std::string name
          = "BM_NarrowPhase_SphereSphere_Native_Batch_N"
            + std::to_string(batchSize);
      benchmark::RegisterBenchmark(
          name.c_str(), BM_NarrowPhase_SphereSphere_Native_Batch, batchSize);
    }
  }
};

static SphereSphereBatchBenchmarkRegistrar
    g_sphere_sphere_batch_benchmark_registrar;

static void BM_NarrowPhase_SphereSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(1.5, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_SphereSphere_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_SphereSphere_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(1.5, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_SphereSphere_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_SphereSphere_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(1.5, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_SphereSphere_ODE);
#endif

static void BM_NarrowPhase_BoxBox_Native(benchmark::State& state)
{
  const Eigen::Vector3d half_extents(0.5, 0.5, 0.5);
  BoxShape b1(half_extents);
  BoxShape b2(half_extents);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideBoxes(b1, tf1, b2, tf2, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_BoxBox_Native);

static std::vector<BoxPair> MakeBoxBoxBatchPairs(
    std::size_t batchSize, const BoxShape& box1, const BoxShape& box2)
{
  std::mt19937 rng(1701u);
  std::uniform_real_distribution<double> offset(-0.15, 0.15);
  std::uniform_real_distribution<double> angle(-0.35, 0.35);

  std::vector<BoxPair> pairs;
  pairs.reserve(batchSize);
  for (std::size_t i = 0; i < batchSize; ++i) {
    Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    tf1.linear()
        = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(0.5 * angle(rng), Eigen::Vector3d::UnitY()))
              .toRotationMatrix();
    tf1.translation() = Eigen::Vector3d(
        0.05 * offset(rng), offset(rng), offset(rng));

    Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
    tf2.linear()
        = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitY())
           * Eigen::AngleAxisd(-0.25 * angle(rng), Eigen::Vector3d::UnitZ()))
              .toRotationMatrix();
    tf2.translation() = Eigen::Vector3d(
        0.9 + offset(rng), offset(rng), offset(rng));

    pairs.push_back(BoxPair{&box1, &box2, tf1, tf2});
  }
  return pairs;
}

static void BM_NarrowPhase_BoxBox_Native_Batch(
    benchmark::State& state, std::size_t batchSize)
{
  const Eigen::Vector3d half_extents(0.5, 0.5, 0.5);
  BoxShape b1(half_extents);
  BoxShape b2(half_extents);

  const auto pairs = MakeBoxBoxBatchPairs(batchSize, b1, b2);
  std::vector<CollisionResult> results(batchSize);
  CollisionOption option;

  for (auto _ : state) {
    for (auto& result : results)
      result.clear();
    collideBoxesBatch(pairs, results, option);
    benchmark::DoNotOptimize(results.data());
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations())
      * static_cast<int64_t>(batchSize));
  state.counters["pairs_per_iteration"]
      = static_cast<double>(batchSize);
}

struct BoxBoxBatchBenchmarkRegistrar
{
  BoxBoxBatchBenchmarkRegistrar()
  {
    for (const std::size_t batchSize : {1u, 10u, 100u, 1000u}) {
      const std::string name
          = "BM_NarrowPhase_BoxBox_Native_Batch_N"
            + std::to_string(batchSize);
      benchmark::RegisterBenchmark(
          name.c_str(), BM_NarrowPhase_BoxBox_Native_Batch, batchSize);
    }
  }
};

static BoxBoxBatchBenchmarkRegistrar g_box_box_batch_benchmark_registrar;

static void BM_NarrowPhase_BoxBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  const Eigen::Vector3d size(1.0, 1.0, 1.0);
  auto shape1 = std::make_shared<dart::dynamics::BoxShape>(size);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(size);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_BoxBox_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_BoxBox_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  const Eigen::Vector3d size(1.0, 1.0, 1.0);
  auto shape1 = std::make_shared<dart::dynamics::BoxShape>(size);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(size);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_BoxBox_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_BoxBox_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  const Eigen::Vector3d size(1.0, 1.0, 1.0);
  auto shape1 = std::make_shared<dart::dynamics::BoxShape>(size);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(size);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_BoxBox_ODE);
#endif

static void BM_NarrowPhase_CapsuleCapsule_Native(benchmark::State& state)
{
  CapsuleShape c1(0.5, 2.0);
  CapsuleShape c2(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideCapsules(c1, tf1, c2, tf2, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_CapsuleCapsule_Native);

static std::vector<CapsulePair> MakeCapsuleCapsuleBatchPairs(
    std::size_t batchSize,
    const CapsuleShape& capsule1,
    const CapsuleShape& capsule2)
{
  std::mt19937 rng(271828u);
  std::uniform_real_distribution<double> offset(-0.05, 0.05);
  std::uniform_real_distribution<double> angle(-0.20, 0.20);

  std::vector<CapsulePair> pairs;
  pairs.reserve(batchSize);
  for (std::size_t i = 0; i < batchSize; ++i) {
    Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    tf1.linear()
        = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(0.5 * angle(rng), Eigen::Vector3d::UnitY()))
              .toRotationMatrix();
    tf1.translation() = Eigen::Vector3d(
        offset(rng), offset(rng), offset(rng));

    Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
    tf2.linear()
        = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(0.5 * angle(rng), Eigen::Vector3d::UnitY()))
              .toRotationMatrix();
    tf2.translation() = Eigen::Vector3d(
        0.45 + offset(rng), offset(rng), offset(rng));

    pairs.push_back(CapsulePair{&capsule1, &capsule2, tf1, tf2});
  }
  return pairs;
}

static void BM_NarrowPhase_CapsuleCapsule_Native_Batch(
    benchmark::State& state, std::size_t batchSize)
{
  CapsuleShape c1(0.35, 1.2);
  CapsuleShape c2(0.30, 1.0);

  const auto pairs = MakeCapsuleCapsuleBatchPairs(batchSize, c1, c2);
  std::vector<CollisionResult> results(batchSize);
  CollisionOption option;

  for (auto _ : state) {
    for (auto& result : results)
      result.clear();
    collideCapsulesBatch(pairs, results, option);
    benchmark::DoNotOptimize(results.data());
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations())
      * static_cast<int64_t>(batchSize));
  state.counters["pairs_per_iteration"] = static_cast<double>(batchSize);
}

struct CapsuleCapsuleBatchBenchmarkRegistrar
{
  CapsuleCapsuleBatchBenchmarkRegistrar()
  {
    for (const std::size_t batchSize : {1u, 10u, 100u, 1000u}) {
      const std::string name
          = "BM_NarrowPhase_CapsuleCapsule_Native_Batch_N"
            + std::to_string(batchSize);
      benchmark::RegisterBenchmark(
          name.c_str(), BM_NarrowPhase_CapsuleCapsule_Native_Batch, batchSize);
    }
  }
};

static CapsuleCapsuleBatchBenchmarkRegistrar
    g_capsule_capsule_batch_benchmark_registrar;

static void BM_NarrowPhase_CapsuleCapsule_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleCapsule_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_CapsuleCapsule_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleCapsule_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_CapsuleCapsule_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleCapsule_ODE);
#endif

static void BM_NarrowPhase_SphereBox_Native(benchmark::State& state)
{
  SphereShape sphere(0.5);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  const Eigen::Isometry3d tf_sphere = OffsetTransform(0.8, 0.0, 0.0);
  const Eigen::Isometry3d tf_box = Eigen::Isometry3d::Identity();

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collideSphereBox(sphere, tf_sphere, box, tf_box, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_SphereBox_Native);

static void BM_NarrowPhase_SphereBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(0.5);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = OffsetTransform(0.8, 0.0, 0.0);
  const Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_SphereBox_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_SphereBox_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(0.5);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = OffsetTransform(0.8, 0.0, 0.0);
  const Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_SphereBox_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_SphereBox_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(0.5);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = OffsetTransform(0.8, 0.0, 0.0);
  const Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_SphereBox_ODE);
#endif

static void BM_NarrowPhaseRawReference_SphereSphere_Native(
    benchmark::State& state)
{
  BM_NarrowPhase_SphereSphere_Native(state);
}
BENCHMARK(BM_NarrowPhaseRawReference_SphereSphere_Native);

static void BM_NarrowPhaseRawReference_BoxBox_Native(benchmark::State& state)
{
  BM_NarrowPhase_BoxBox_Native(state);
}
BENCHMARK(BM_NarrowPhaseRawReference_BoxBox_Native);

static void BM_NarrowPhaseRawReference_SphereBox_Native(benchmark::State& state)
{
  BM_NarrowPhase_SphereBox_Native(state);
}
BENCHMARK(BM_NarrowPhaseRawReference_SphereBox_Native);

static void BM_NarrowPhaseRawReference_SphereSphere_FCL(benchmark::State& state)
{
  auto sphere1 = std::make_shared<fcl::Sphere<double>>(1.0);
  auto sphere2 = std::make_shared<fcl::Sphere<double>>(1.0);

  RunFclRawCollisionBenchmark(
      state,
      sphere1,
      Eigen::Isometry3d::Identity(),
      sphere2,
      OffsetTransform(1.5, 0.0, 0.0));
}
BENCHMARK(BM_NarrowPhaseRawReference_SphereSphere_FCL);

static void BM_NarrowPhaseRawReference_BoxBox_FCL(benchmark::State& state)
{
  auto box1 = std::make_shared<fcl::Box<double>>(1.0, 1.0, 1.0);
  auto box2 = std::make_shared<fcl::Box<double>>(1.0, 1.0, 1.0);

  RunFclRawCollisionBenchmark(
      state,
      box1,
      Eigen::Isometry3d::Identity(),
      box2,
      OffsetTransform(0.8, 0.0, 0.0));
}
BENCHMARK(BM_NarrowPhaseRawReference_BoxBox_FCL);

static void BM_NarrowPhaseRawReference_SphereBox_FCL(benchmark::State& state)
{
  auto sphere = std::make_shared<fcl::Sphere<double>>(0.5);
  auto box = std::make_shared<fcl::Box<double>>(1.0, 1.0, 1.0);

  RunFclRawCollisionBenchmark(
      state,
      sphere,
      OffsetTransform(0.8, 0.0, 0.0),
      box,
      Eigen::Isometry3d::Identity());
}
BENCHMARK(BM_NarrowPhaseRawReference_SphereBox_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhaseRawReference_SphereSphere_Bullet(
    benchmark::State& state)
{
  btSphereShape sphere1(1.0);
  btSphereShape sphere2(1.0);

  RunBulletRawCollisionBenchmark(
      state,
      sphere1,
      Eigen::Isometry3d::Identity(),
      sphere2,
      OffsetTransform(1.5, 0.0, 0.0));
}
BENCHMARK(BM_NarrowPhaseRawReference_SphereSphere_Bullet);

static void BM_NarrowPhaseRawReference_BoxBox_Bullet(benchmark::State& state)
{
  btBoxShape box1(btVector3(0.5, 0.5, 0.5));
  btBoxShape box2(btVector3(0.5, 0.5, 0.5));

  RunBulletRawCollisionBenchmark(
      state,
      box1,
      Eigen::Isometry3d::Identity(),
      box2,
      OffsetTransform(0.8, 0.0, 0.0));
}
BENCHMARK(BM_NarrowPhaseRawReference_BoxBox_Bullet);

static void BM_NarrowPhaseRawReference_SphereBox_Bullet(
    benchmark::State& state)
{
  btSphereShape sphere(0.5);
  btBoxShape box(btVector3(0.5, 0.5, 0.5));

  RunBulletRawCollisionBenchmark(
      state,
      sphere,
      OffsetTransform(0.8, 0.0, 0.0),
      box,
      Eigen::Isometry3d::Identity());
}
BENCHMARK(BM_NarrowPhaseRawReference_SphereBox_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhaseRawReference_SphereSphere_ODE(benchmark::State& state)
{
  OdeRuntime ode;
  dGeomID sphere1 = dCreateSphere(0, 1.0);
  dGeomID sphere2 = dCreateSphere(0, 1.0);

  RunOdeRawCollisionBenchmark(
      state,
      sphere1,
      Eigen::Isometry3d::Identity(),
      sphere2,
      OffsetTransform(1.5, 0.0, 0.0));

  dGeomDestroy(sphere1);
  dGeomDestroy(sphere2);
}
BENCHMARK(BM_NarrowPhaseRawReference_SphereSphere_ODE);

static void BM_NarrowPhaseRawReference_BoxBox_ODE(benchmark::State& state)
{
  OdeRuntime ode;
  dGeomID box1 = dCreateBox(0, 1.0, 1.0, 1.0);
  dGeomID box2 = dCreateBox(0, 1.0, 1.0, 1.0);

  RunOdeRawCollisionBenchmark(
      state,
      box1,
      Eigen::Isometry3d::Identity(),
      box2,
      OffsetTransform(0.8, 0.0, 0.0));

  dGeomDestroy(box1);
  dGeomDestroy(box2);
}
BENCHMARK(BM_NarrowPhaseRawReference_BoxBox_ODE);

static void BM_NarrowPhaseRawReference_SphereBox_ODE(benchmark::State& state)
{
  OdeRuntime ode;
  dGeomID sphere = dCreateSphere(0, 0.5);
  dGeomID box = dCreateBox(0, 1.0, 1.0, 1.0);

  RunOdeRawCollisionBenchmark(
      state,
      sphere,
      OffsetTransform(0.8, 0.0, 0.0),
      box,
      Eigen::Isometry3d::Identity());

  dGeomDestroy(sphere);
  dGeomDestroy(box);
}
BENCHMARK(BM_NarrowPhaseRawReference_SphereBox_ODE);
#endif

static void BM_NarrowPhase_CapsuleSphere_Native(benchmark::State& state)
{
  CapsuleShape capsule(0.5, 2.0);
  SphereShape sphere(0.5);

  const Eigen::Isometry3d tf_capsule = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf_sphere = OffsetTransform(0.8, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideCapsuleSphere(
        capsule, tf_capsule, sphere, tf_sphere, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_CapsuleSphere_Native);

static void BM_NarrowPhase_CapsuleSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleSphere_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_CapsuleSphere_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleSphere_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_CapsuleSphere_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleSphere_ODE);
#endif

static void BM_NarrowPhase_CapsuleBox_Native(benchmark::State& state)
{
  CapsuleShape capsule(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  const Eigen::Isometry3d tf_capsule = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf_box = OffsetTransform(0.8, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collideCapsuleBox(capsule, tf_capsule, box, tf_box, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_CapsuleBox_Native);

static void BM_NarrowPhase_CapsuleBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleBox_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_CapsuleBox_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleBox_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_CapsuleBox_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleBox_ODE);
#endif

static void BM_NarrowPhase_CylinderCylinder_Native(benchmark::State& state)
{
  CylinderShape c1(0.5, 2.0);
  CylinderShape c2(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collideCylinders(c1, tf1, c2, tf2, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_CylinderCylinder_Native);

static std::vector<CylinderPair> MakeCylinderCylinderBatchPairs(
    std::size_t batchSize,
    const CylinderShape& cylinder1,
    const CylinderShape& cylinder2)
{
  std::mt19937 rng(161803u);
  std::uniform_real_distribution<double> offset(-0.05, 0.05);
  std::uniform_real_distribution<double> angle(-0.15, 0.15);

  std::vector<CylinderPair> pairs;
  pairs.reserve(batchSize);
  for (std::size_t i = 0; i < batchSize; ++i) {
    Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    tf1.linear()
        = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(0.5 * angle(rng), Eigen::Vector3d::UnitY()))
              .toRotationMatrix();
    tf1.translation() = Eigen::Vector3d(
        offset(rng), offset(rng), offset(rng));

    Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
    tf2.linear()
        = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(0.5 * angle(rng), Eigen::Vector3d::UnitY()))
              .toRotationMatrix();
    tf2.translation() = Eigen::Vector3d(
        0.45 + offset(rng), offset(rng), offset(rng));

    pairs.push_back(CylinderPair{&cylinder1, &cylinder2, tf1, tf2});
  }
  return pairs;
}

static void BM_NarrowPhase_CylinderCylinder_Native_Batch(
    benchmark::State& state, std::size_t batchSize)
{
  CylinderShape c1(0.45, 1.2);
  CylinderShape c2(0.40, 1.0);

  const auto pairs = MakeCylinderCylinderBatchPairs(batchSize, c1, c2);
  std::vector<CollisionResult> results(batchSize);
  CollisionOption option;

  for (auto _ : state) {
    for (auto& result : results)
      result.clear();
    collideCylindersBatch(pairs, results, option);
    benchmark::DoNotOptimize(results.data());
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations())
      * static_cast<int64_t>(batchSize));
  state.counters["pairs_per_iteration"] = static_cast<double>(batchSize);
}

struct CylinderCylinderBatchBenchmarkRegistrar
{
  CylinderCylinderBatchBenchmarkRegistrar()
  {
    for (const std::size_t batchSize : {1u, 10u, 100u, 1000u}) {
      const std::string name
          = "BM_NarrowPhase_CylinderCylinder_Native_Batch_N"
            + std::to_string(batchSize);
      benchmark::RegisterBenchmark(
          name.c_str(), BM_NarrowPhase_CylinderCylinder_Native_Batch, batchSize);
    }
  }
};

static CylinderCylinderBatchBenchmarkRegistrar
    g_cylinder_cylinder_batch_benchmark_registrar;

static void BM_NarrowPhase_ConvexConvex_Native(benchmark::State& state)
{
  ConvexShape c1(MakeBenchmarkOctahedronVertices(0.60));
  ConvexShape c2(MakeBenchmarkOctahedronVertices(0.55));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.65, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 1;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collideConvexConvex(c1, tf1, c2, tf2, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_ConvexConvex_Native);

static std::vector<ConvexPair> MakeConvexConvexBatchPairs(
    std::size_t batchSize, const ConvexShape& convex1, const ConvexShape& convex2)
{
  std::mt19937 rng(577215u);
  std::uniform_real_distribution<double> offset(-0.04, 0.04);
  std::uniform_real_distribution<double> angle(-0.12, 0.12);

  std::vector<ConvexPair> pairs;
  pairs.reserve(batchSize);
  for (std::size_t i = 0; i < batchSize; ++i) {
    Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    tf1.linear()
        = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(0.5 * angle(rng), Eigen::Vector3d::UnitY()))
              .toRotationMatrix();
    tf1.translation() = Eigen::Vector3d(
        offset(rng), offset(rng), offset(rng));

    Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
    tf2.linear()
        = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(0.5 * angle(rng), Eigen::Vector3d::UnitY()))
              .toRotationMatrix();
    tf2.translation() = Eigen::Vector3d(
        0.65 + offset(rng), offset(rng), offset(rng));

    pairs.push_back(ConvexPair{&convex1, &convex2, tf1, tf2});
  }
  return pairs;
}

static void BM_NarrowPhase_ConvexConvex_Native_Batch(
    benchmark::State& state, std::size_t batchSize)
{
  ConvexShape c1(MakeBenchmarkOctahedronVertices(0.60));
  ConvexShape c2(MakeBenchmarkOctahedronVertices(0.55));

  const auto pairs = MakeConvexConvexBatchPairs(batchSize, c1, c2);
  std::vector<CollisionResult> results(batchSize);
  CollisionOption option;
  option.maxNumContacts = 1;

  for (auto _ : state) {
    for (auto& result : results)
      result.clear();
    collideConvexConvexBatch(pairs, results, option);
    benchmark::DoNotOptimize(results.data());
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations())
      * static_cast<int64_t>(batchSize));
  state.counters["pairs_per_iteration"] = static_cast<double>(batchSize);
}

struct ConvexConvexBatchBenchmarkRegistrar
{
  ConvexConvexBatchBenchmarkRegistrar()
  {
    for (const std::size_t batchSize : {1u, 10u, 100u, 1000u}) {
      const std::string name
          = "BM_NarrowPhase_ConvexConvex_Native_Batch_N"
            + std::to_string(batchSize);
      benchmark::RegisterBenchmark(
          name.c_str(), BM_NarrowPhase_ConvexConvex_Native_Batch, batchSize);
    }
  }
};

static ConvexConvexBatchBenchmarkRegistrar
    g_convex_convex_batch_benchmark_registrar;

static void BM_NarrowPhase_MeshMesh_Native(benchmark::State& state)
{
  MeshShape m1(
      MakeBenchmarkCubeMeshVertices(1.0), MakeBenchmarkCubeMeshTriangles());
  MeshShape m2(
      MakeBenchmarkCubeMeshVertices(1.0), MakeBenchmarkCubeMeshTriangles());

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(1.20, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;
  option.maxNumContacts = 8;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collideMeshMesh(m1, tf1, m2, tf2, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_MeshMesh_Native);

static std::vector<MeshPair> MakeMeshMeshBatchPairs(
    std::size_t batchSize, const MeshShape& mesh1, const MeshShape& mesh2)
{
  std::mt19937 rng(141421u);
  std::uniform_real_distribution<double> offset(-0.03, 0.03);
  std::uniform_real_distribution<double> angle(-0.05, 0.05);

  std::vector<MeshPair> pairs;
  pairs.reserve(batchSize);
  for (std::size_t i = 0; i < batchSize; ++i) {
    Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    tf1.linear()
        = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(0.5 * angle(rng), Eigen::Vector3d::UnitY()))
              .toRotationMatrix();
    tf1.translation() = Eigen::Vector3d(
        offset(rng), offset(rng), offset(rng));

    Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
    tf2.linear()
        = (Eigen::AngleAxisd(angle(rng), Eigen::Vector3d::UnitX())
           * Eigen::AngleAxisd(0.5 * angle(rng), Eigen::Vector3d::UnitY()))
              .toRotationMatrix();
    tf2.translation() = Eigen::Vector3d(
        1.20 + offset(rng), offset(rng), offset(rng));

    pairs.push_back(MeshPair{&mesh1, &mesh2, tf1, tf2});
  }
  return pairs;
}

static void BM_NarrowPhase_MeshMesh_Native_Batch(
    benchmark::State& state, std::size_t batchSize)
{
  MeshShape m1(
      MakeBenchmarkCubeMeshVertices(1.0), MakeBenchmarkCubeMeshTriangles());
  MeshShape m2(
      MakeBenchmarkCubeMeshVertices(1.0), MakeBenchmarkCubeMeshTriangles());

  const auto pairs = MakeMeshMeshBatchPairs(batchSize, m1, m2);
  std::vector<CollisionResult> results(batchSize);
  CollisionOption option;
  option.maxNumContacts = 8;

  for (auto _ : state) {
    for (auto& result : results)
      result.clear();
    collideMeshMeshBatch(pairs, results, option);
    benchmark::DoNotOptimize(results.data());
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations())
      * static_cast<int64_t>(batchSize));
  state.counters["pairs_per_iteration"] = static_cast<double>(batchSize);
}

struct MeshMeshBatchBenchmarkRegistrar
{
  MeshMeshBatchBenchmarkRegistrar()
  {
    for (const std::size_t batchSize : {1u, 10u, 100u, 1000u}) {
      const std::string name
          = "BM_NarrowPhase_MeshMesh_Native_Batch_N"
            + std::to_string(batchSize);
      benchmark::RegisterBenchmark(
          name.c_str(), BM_NarrowPhase_MeshMesh_Native_Batch, batchSize);
    }
  }
};

static MeshMeshBatchBenchmarkRegistrar g_mesh_mesh_batch_benchmark_registrar;

struct NarrowPhaseDispatcherBatchShapes
{
  NarrowPhaseDispatcherBatchShapes()
    : convex1(MakeBenchmarkOctahedronVertices(0.60)),
      convex2(MakeBenchmarkOctahedronVertices(0.55)),
      mesh1(MakeBenchmarkCubeMeshVertices(1.0), MakeBenchmarkCubeMeshTriangles()),
      mesh2(MakeBenchmarkCubeMeshVertices(1.0), MakeBenchmarkCubeMeshTriangles())
  {
  }

  SphereShape sphere1{0.75};
  SphereShape sphere2{0.60};
  BoxShape box1{Eigen::Vector3d(0.5, 0.4, 0.3)};
  BoxShape box2{Eigen::Vector3d(0.4, 0.5, 0.3)};
  ConvexShape convex1;
  ConvexShape convex2;
  MeshShape mesh1;
  MeshShape mesh2;
};

static std::vector<NarrowPhasePair> MakeNarrowPhaseDispatcherBatchPairs(
    std::size_t batchSize, const NarrowPhaseDispatcherBatchShapes& shapes)
{
  std::vector<NarrowPhasePair> pairs;
  pairs.reserve(batchSize);
  for (std::size_t i = 0; i < batchSize; ++i) {
    switch (i % 4u) {
      case 0u:
        pairs.push_back(
            NarrowPhasePair{
                &shapes.sphere1,
                &shapes.sphere2,
                Eigen::Isometry3d::Identity(),
                OffsetTransform(0.85, 0.0, 0.0)});
        break;
      case 1u:
        pairs.push_back(
            NarrowPhasePair{
                &shapes.box1,
                &shapes.box2,
                OffsetTransform(0.0, 0.1, 0.0),
                OffsetTransform(0.65, -0.1, 0.0)});
        break;
      case 2u:
        pairs.push_back(
            NarrowPhasePair{
                &shapes.convex1,
                &shapes.convex2,
                Eigen::Isometry3d::Identity(),
                OffsetTransform(0.65, 0.0, 0.0)});
        break;
      default:
        pairs.push_back(
            NarrowPhasePair{
                &shapes.mesh1,
                &shapes.mesh2,
                Eigen::Isometry3d::Identity(),
                OffsetTransform(1.20, 0.0, 0.0)});
        break;
    }
  }
  return pairs;
}

static void BM_NarrowPhase_CollideBatchDispatcher_Native_Batch(
    benchmark::State& state, std::size_t batchSize)
{
  const NarrowPhaseDispatcherBatchShapes shapes;
  const auto pairs = MakeNarrowPhaseDispatcherBatchPairs(batchSize, shapes);
  std::vector<CollisionResult> results(batchSize);
  CollisionOption option;
  option.maxNumContacts = 8;

  for (auto _ : state) {
    for (auto& result : results)
      result.clear();
    NarrowPhase::collideBatch(pairs, results, option);
    benchmark::DoNotOptimize(results.data());
    benchmark::ClobberMemory();
  }

  state.SetItemsProcessed(
      static_cast<int64_t>(state.iterations())
      * static_cast<int64_t>(batchSize));
  state.counters["pairs_per_iteration"] = static_cast<double>(batchSize);
}

struct NarrowPhaseCollideBatchDispatcherBenchmarkRegistrar
{
  NarrowPhaseCollideBatchDispatcherBenchmarkRegistrar()
  {
    for (const std::size_t batchSize : {1u, 10u, 100u, 1000u}) {
      const std::string name
          = "BM_NarrowPhase_CollideBatchDispatcher_Native_Batch_N"
            + std::to_string(batchSize);
      benchmark::RegisterBenchmark(
          name.c_str(),
          BM_NarrowPhase_CollideBatchDispatcher_Native_Batch,
          batchSize);
    }
  }
};

static NarrowPhaseCollideBatchDispatcherBenchmarkRegistrar
    g_narrow_phase_collide_batch_dispatcher_benchmark_registrar;

static void BM_NarrowPhase_CylinderCylinder_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderCylinder_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_CylinderCylinder_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderCylinder_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_CylinderCylinder_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderCylinder_ODE);
#endif

static void BM_NarrowPhase_CylinderSphere_Native(benchmark::State& state)
{
  CylinderShape cylinder(0.5, 2.0);
  SphereShape sphere(0.5);

  const Eigen::Isometry3d tf_cylinder = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf_sphere = OffsetTransform(0.8, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideCylinderSphere(
        cylinder, tf_cylinder, sphere, tf_sphere, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_CylinderSphere_Native);

static void BM_NarrowPhase_CylinderSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderSphere_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_CylinderSphere_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderSphere_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_CylinderSphere_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderSphere_ODE);
#endif

static void BM_NarrowPhase_CylinderBox_Native(benchmark::State& state)
{
  CylinderShape cylinder(0.5, 2.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  const Eigen::Isometry3d tf_cylinder = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf_box = OffsetTransform(0.8, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collideCylinderBox(cylinder, tf_cylinder, box, tf_box, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_CylinderBox_Native);

static void BM_NarrowPhase_CylinderBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderBox_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_CylinderBox_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderBox_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_CylinderBox_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderBox_ODE);
#endif

static void BM_NarrowPhase_CylinderCapsule_Native(benchmark::State& state)
{
  CylinderShape cylinder(0.5, 2.0);
  CapsuleShape capsule(0.5, 2.0);

  const Eigen::Isometry3d tf_cylinder = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf_capsule = OffsetTransform(0.8, 0.0, 0.0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideCylinderCapsule(
        cylinder, tf_cylinder, capsule, tf_capsule, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_CylinderCapsule_Native);

static void BM_NarrowPhase_CylinderCapsule_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderCapsule_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_CylinderCapsule_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderCapsule_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_CylinderCapsule_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderCapsule_ODE);
#endif

static void BM_NarrowPhase_CylinderPlane_Native(benchmark::State& state)
{
  CylinderShape cylinder(0.5, 2.0);
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);

  const Eigen::Isometry3d tf_cylinder = OffsetTransform(0.0, 0.0, 0.3);
  const Eigen::Isometry3d tf_plane = Eigen::Isometry3d::Identity();

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideCylinderPlane(
        cylinder, tf_cylinder, plane, tf_plane, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_CylinderPlane_Native);

static void BM_NarrowPhase_CylinderPlane_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);

  const Eigen::Isometry3d tf1 = OffsetTransform(0.0, 0.0, 0.3);
  const Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderPlane_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_CylinderPlane_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);

  const Eigen::Isometry3d tf1 = OffsetTransform(0.0, 0.0, 0.3);
  const Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderPlane_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_CylinderPlane_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);

  const Eigen::Isometry3d tf1 = OffsetTransform(0.0, 0.0, 0.3);
  const Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderPlane_ODE);
#endif

static void BM_NarrowPhase_PlaneSphere_Native(benchmark::State& state)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  SphereShape sphere(0.5);

  const Eigen::Isometry3d tf_plane = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf_sphere = OffsetTransform(0.0, 0.0, 0.3);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collidePlaneSphere(plane, tf_plane, sphere, tf_sphere, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_PlaneSphere_Native);

static void BM_NarrowPhase_PlaneSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.0, 0.0, 0.3);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_PlaneSphere_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_PlaneSphere_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.0, 0.0, 0.3);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_PlaneSphere_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_PlaneSphere_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.0, 0.0, 0.3);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_PlaneSphere_ODE);
#endif

static void BM_NarrowPhase_PlaneBox_Native(benchmark::State& state)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  BoxShape box(Eigen::Vector3d(0.5, 0.5, 0.5));

  const Eigen::Isometry3d tf_plane = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf_box = OffsetTransform(0.0, 0.0, 0.3);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        collidePlaneBox(plane, tf_plane, box, tf_box, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_PlaneBox_Native);

static void BM_NarrowPhase_PlaneBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.0, 0.0, 0.3);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_PlaneBox_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_PlaneBox_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.0, 0.0, 0.3);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_PlaneBox_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_PlaneBox_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.0, 0.0, 0.3);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_PlaneBox_ODE);
#endif

static void BM_NarrowPhase_PlaneCapsule_Native(benchmark::State& state)
{
  PlaneShape plane(Eigen::Vector3d::UnitZ(), 0.0);
  CapsuleShape capsule(0.5, 2.0);

  const Eigen::Isometry3d tf_plane = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf_capsule = OffsetTransform(0.0, 0.0, 0.3);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collidePlaneCapsule(
        plane, tf_plane, capsule, tf_capsule, result, option));
  }
}
BENCHMARK(BM_NarrowPhase_PlaneCapsule_Native);

static void BM_NarrowPhase_PlaneCapsule_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.0, 0.0, 0.3);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_PlaneCapsule_FCL);

#if DART_HAVE_BULLET
static void BM_NarrowPhase_PlaneCapsule_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.0, 0.0, 0.3);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_PlaneCapsule_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_PlaneCapsule_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();

  auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.0, 0.0, 0.3);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_PlaneCapsule_ODE);
#endif

namespace edge_case_bench {

using dart::benchmark::collision::EdgeCase;
using dart::benchmark::collision::MakeBoxBoxTransforms;
using dart::benchmark::collision::MakeBoxSpec;
using dart::benchmark::collision::MakeCapsuleBoxTransforms;
using dart::benchmark::collision::MakeCapsuleCapsuleTransforms;
using dart::benchmark::collision::MakeCapsuleSpec;
using dart::benchmark::collision::MakeCapsuleSphereTransforms;
using dart::benchmark::collision::MakeSphereBoxTransforms;
using dart::benchmark::collision::MakeSphereSpec;
using dart::benchmark::collision::MakeSphereSphereTransforms;
using dart::benchmark::collision::PairKind;
using dart::benchmark::collision::ScaleFromIndex;

void AddScaleArgs(benchmark::Benchmark* bench)
{
  bench->Arg(0)->Arg(1)->Arg(2);
}

void RunNarrowPhaseCaseNative(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  const double scale = ScaleFromIndex(static_cast<int>(state.range(0)));
  const auto sphereSpec = MakeSphereSpec(scale);
  const auto capsuleSpec = MakeCapsuleSpec(scale);
  const auto boxSpec = MakeBoxSpec(scale, edge == EdgeCase::kThinFeature);

  CollisionResult result;
  CollisionOption option = CollisionOption::fullContacts();

  switch (pair) {
    case PairKind::kSphereSphere: {
      SphereShape s1(sphereSpec.radius);
      SphereShape s2(sphereSpec.radius);
      const auto tfs = MakeSphereSphereTransforms(
          sphereSpec.radius, sphereSpec.radius, edge);

      for (auto _ : state) {
        result.clear();
        benchmark::DoNotOptimize(
            collideSpheres(s1, tfs.tf1, s2, tfs.tf2, result, option));
      }
      return;
    }
    case PairKind::kBoxBox: {
      BoxShape b1(boxSpec.halfExtents);
      BoxShape b2(boxSpec.halfExtents);
      const auto tfs = MakeBoxBoxTransforms(
          boxSpec.halfExtents, boxSpec.halfExtents, edge);

      for (auto _ : state) {
        result.clear();
        benchmark::DoNotOptimize(
            collideBoxes(b1, tfs.tf1, b2, tfs.tf2, result, option));
      }
      return;
    }
    case PairKind::kCapsuleCapsule: {
      CapsuleShape c1(capsuleSpec.radius, capsuleSpec.height);
      CapsuleShape c2(capsuleSpec.radius, capsuleSpec.height);
      const auto tfs = MakeCapsuleCapsuleTransforms(
          capsuleSpec.radius, capsuleSpec.radius, edge);

      for (auto _ : state) {
        result.clear();
        benchmark::DoNotOptimize(
            collideCapsules(c1, tfs.tf1, c2, tfs.tf2, result, option));
      }
      return;
    }
    case PairKind::kSphereBox: {
      SphereShape sphere(sphereSpec.radius);
      BoxShape box(boxSpec.halfExtents);
      const auto tfs = MakeSphereBoxTransforms(
          sphereSpec.radius, boxSpec.halfExtents, edge);

      for (auto _ : state) {
        result.clear();
        benchmark::DoNotOptimize(
            collideSphereBox(sphere, tfs.tf1, box, tfs.tf2, result, option));
      }
      return;
    }
    case PairKind::kCapsuleSphere: {
      CapsuleShape capsule(capsuleSpec.radius, capsuleSpec.height);
      SphereShape sphere(sphereSpec.radius);
      const auto tfs = MakeCapsuleSphereTransforms(
          capsuleSpec.radius, sphereSpec.radius, edge);

      for (auto _ : state) {
        result.clear();
        benchmark::DoNotOptimize(collideCapsuleSphere(
            capsule, tfs.tf1, sphere, tfs.tf2, result, option));
      }
      return;
    }
    case PairKind::kCapsuleBox: {
      CapsuleShape capsule(capsuleSpec.radius, capsuleSpec.height);
      BoxShape box(boxSpec.halfExtents);
      const auto tfs = MakeCapsuleBoxTransforms(
          capsuleSpec.radius, boxSpec.halfExtents, edge);

      for (auto _ : state) {
        result.clear();
        benchmark::DoNotOptimize(
            collideCapsuleBox(capsule, tfs.tf1, box, tfs.tf2, result, option));
      }
      return;
    }
  }
}

template <typename DetectorPtr>
void RunNarrowPhaseCaseDetector(
    benchmark::State& state,
    const DetectorPtr& detector,
    PairKind pair,
    EdgeCase edge)
{
  const double scale = ScaleFromIndex(static_cast<int>(state.range(0)));
  const auto sphereSpec = MakeSphereSpec(scale);
  const auto capsuleSpec = MakeCapsuleSpec(scale);
  const auto boxSpec = MakeBoxSpec(scale, edge == EdgeCase::kThinFeature);

  switch (pair) {
    case PairKind::kSphereSphere: {
      auto shape1
          = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      auto shape2
          = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      const auto tfs = MakeSphereSphereTransforms(
          sphereSpec.radius, sphereSpec.radius, edge);
      RunDetectorBenchmark(state, detector, shape1, tfs.tf1, shape2, tfs.tf2);
      return;
    }
    case PairKind::kBoxBox: {
      auto shape1 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      auto shape2 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      const auto tfs = MakeBoxBoxTransforms(
          boxSpec.halfExtents, boxSpec.halfExtents, edge);
      RunDetectorBenchmark(state, detector, shape1, tfs.tf1, shape2, tfs.tf2);
      return;
    }
    case PairKind::kCapsuleCapsule: {
      auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      const auto tfs = MakeCapsuleCapsuleTransforms(
          capsuleSpec.radius, capsuleSpec.radius, edge);
      RunDetectorBenchmark(state, detector, shape1, tfs.tf1, shape2, tfs.tf2);
      return;
    }
    case PairKind::kSphereBox: {
      auto shape1
          = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      auto shape2 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      const auto tfs = MakeSphereBoxTransforms(
          sphereSpec.radius, boxSpec.halfExtents, edge);
      RunDetectorBenchmark(state, detector, shape1, tfs.tf1, shape2, tfs.tf2);
      return;
    }
    case PairKind::kCapsuleSphere: {
      auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      auto shape2
          = std::make_shared<dart::dynamics::SphereShape>(sphereSpec.radius);
      const auto tfs = MakeCapsuleSphereTransforms(
          capsuleSpec.radius, sphereSpec.radius, edge);
      RunDetectorBenchmark(state, detector, shape1, tfs.tf1, shape2, tfs.tf2);
      return;
    }
    case PairKind::kCapsuleBox: {
      auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(
          capsuleSpec.radius, capsuleSpec.height);
      auto shape2 = std::make_shared<dart::dynamics::BoxShape>(boxSpec.size);
      const auto tfs = MakeCapsuleBoxTransforms(
          capsuleSpec.radius, boxSpec.halfExtents, edge);
      RunDetectorBenchmark(state, detector, shape1, tfs.tf1, shape2, tfs.tf2);
      return;
    }
  }
}

static void BM_NarrowPhase_EdgeCases_Native(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  RunNarrowPhaseCaseNative(state, pair, edge);
}

static void BM_NarrowPhase_EdgeCases_FCL(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();
  RunNarrowPhaseCaseDetector(state, detector, pair, edge);
}

#if DART_HAVE_BULLET
static void BM_NarrowPhase_EdgeCases_Bullet(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();
  RunNarrowPhaseCaseDetector(state, detector, pair, edge);
}
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_EdgeCases_ODE(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();
  RunNarrowPhaseCaseDetector(state, detector, pair, edge);
}
#endif

static void BM_NarrowPhaseAdapter_EdgeCases_Native(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::DartCollisionDetector::create();
  RunNarrowPhaseCaseDetector(state, detector, pair, edge);
}

static void BM_NarrowPhaseAdapter_EdgeCases_FCL(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::FCLCollisionDetector::createReference();
  RunNarrowPhaseCaseDetector(state, detector, pair, edge);
}

#if DART_HAVE_BULLET
static void BM_NarrowPhaseAdapter_EdgeCases_Bullet(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::BulletCollisionDetector::createReference();
  RunNarrowPhaseCaseDetector(state, detector, pair, edge);
}
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhaseAdapter_EdgeCases_ODE(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::OdeCollisionDetector::createReference();
  RunNarrowPhaseCaseDetector(state, detector, pair, edge);
}
#endif

void RunNarrowPhaseCaseFclRaw(
    benchmark::State& state,
    PairKind pair,
    EdgeCase edge)
{
  const double scale = ScaleFromIndex(static_cast<int>(state.range(0)));
  const auto sphereSpec = MakeSphereSpec(scale);
  const auto capsuleSpec = MakeCapsuleSpec(scale);
  const auto boxSpec = MakeBoxSpec(scale, edge == EdgeCase::kThinFeature);
  const auto sanity = RawCollisionSanity::kAllowNoCollision;

  switch (pair) {
    case PairKind::kSphereSphere: {
      auto shape1 = std::make_shared<fcl::Sphere<double>>(sphereSpec.radius);
      auto shape2 = std::make_shared<fcl::Sphere<double>>(sphereSpec.radius);
      const auto tfs = MakeSphereSphereTransforms(
          sphereSpec.radius, sphereSpec.radius, edge);
      RunFclRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
    case PairKind::kBoxBox: {
      auto shape1 = std::make_shared<fcl::Box<double>>(
          boxSpec.size.x(), boxSpec.size.y(), boxSpec.size.z());
      auto shape2 = std::make_shared<fcl::Box<double>>(
          boxSpec.size.x(), boxSpec.size.y(), boxSpec.size.z());
      const auto tfs = MakeBoxBoxTransforms(
          boxSpec.halfExtents, boxSpec.halfExtents, edge);
      RunFclRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
    case PairKind::kCapsuleCapsule: {
      auto shape1 = std::make_shared<fcl::Capsule<double>>(
          capsuleSpec.radius, capsuleSpec.height);
      auto shape2 = std::make_shared<fcl::Capsule<double>>(
          capsuleSpec.radius, capsuleSpec.height);
      const auto tfs = MakeCapsuleCapsuleTransforms(
          capsuleSpec.radius, capsuleSpec.radius, edge);
      RunFclRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
    case PairKind::kSphereBox: {
      auto shape1 = std::make_shared<fcl::Sphere<double>>(sphereSpec.radius);
      auto shape2 = std::make_shared<fcl::Box<double>>(
          boxSpec.size.x(), boxSpec.size.y(), boxSpec.size.z());
      const auto tfs = MakeSphereBoxTransforms(
          sphereSpec.radius, boxSpec.halfExtents, edge);
      RunFclRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
    case PairKind::kCapsuleSphere: {
      auto shape1 = std::make_shared<fcl::Capsule<double>>(
          capsuleSpec.radius, capsuleSpec.height);
      auto shape2 = std::make_shared<fcl::Sphere<double>>(sphereSpec.radius);
      const auto tfs = MakeCapsuleSphereTransforms(
          capsuleSpec.radius, sphereSpec.radius, edge);
      RunFclRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
    case PairKind::kCapsuleBox: {
      auto shape1 = std::make_shared<fcl::Capsule<double>>(
          capsuleSpec.radius, capsuleSpec.height);
      auto shape2 = std::make_shared<fcl::Box<double>>(
          boxSpec.size.x(), boxSpec.size.y(), boxSpec.size.z());
      const auto tfs = MakeCapsuleBoxTransforms(
          capsuleSpec.radius, boxSpec.halfExtents, edge);
      RunFclRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
  }
}

#if DART_HAVE_BULLET
btScalar ToBtScalar(double value)
{
  return static_cast<btScalar>(value);
}

btVector3 ToBtVector3(const Eigen::Vector3d& value)
{
  return btVector3(ToBtScalar(value.x()), ToBtScalar(value.y()), ToBtScalar(value.z()));
}

void RunNarrowPhaseCaseBulletRaw(
    benchmark::State& state,
    PairKind pair,
    EdgeCase edge)
{
  const double scale = ScaleFromIndex(static_cast<int>(state.range(0)));
  const auto sphereSpec = MakeSphereSpec(scale);
  const auto capsuleSpec = MakeCapsuleSpec(scale);
  const auto boxSpec = MakeBoxSpec(scale, edge == EdgeCase::kThinFeature);
  const auto sanity = RawCollisionSanity::kAllowNoCollision;

  switch (pair) {
    case PairKind::kSphereSphere: {
      btSphereShape shape1(ToBtScalar(sphereSpec.radius));
      btSphereShape shape2(ToBtScalar(sphereSpec.radius));
      const auto tfs = MakeSphereSphereTransforms(
          sphereSpec.radius, sphereSpec.radius, edge);
      RunBulletRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
    case PairKind::kBoxBox: {
      btBoxShape shape1(ToBtVector3(boxSpec.halfExtents));
      btBoxShape shape2(ToBtVector3(boxSpec.halfExtents));
      const auto tfs = MakeBoxBoxTransforms(
          boxSpec.halfExtents, boxSpec.halfExtents, edge);
      RunBulletRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
    case PairKind::kCapsuleCapsule: {
      btCapsuleShapeZ shape1(
          ToBtScalar(capsuleSpec.radius), ToBtScalar(capsuleSpec.height));
      btCapsuleShapeZ shape2(
          ToBtScalar(capsuleSpec.radius), ToBtScalar(capsuleSpec.height));
      const auto tfs = MakeCapsuleCapsuleTransforms(
          capsuleSpec.radius, capsuleSpec.radius, edge);
      RunBulletRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
    case PairKind::kSphereBox: {
      btSphereShape shape1(ToBtScalar(sphereSpec.radius));
      btBoxShape shape2(ToBtVector3(boxSpec.halfExtents));
      const auto tfs = MakeSphereBoxTransforms(
          sphereSpec.radius, boxSpec.halfExtents, edge);
      RunBulletRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
    case PairKind::kCapsuleSphere: {
      btCapsuleShapeZ shape1(
          ToBtScalar(capsuleSpec.radius), ToBtScalar(capsuleSpec.height));
      btSphereShape shape2(ToBtScalar(sphereSpec.radius));
      const auto tfs = MakeCapsuleSphereTransforms(
          capsuleSpec.radius, sphereSpec.radius, edge);
      RunBulletRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
    case PairKind::kCapsuleBox: {
      btCapsuleShapeZ shape1(
          ToBtScalar(capsuleSpec.radius), ToBtScalar(capsuleSpec.height));
      btBoxShape shape2(ToBtVector3(boxSpec.halfExtents));
      const auto tfs = MakeCapsuleBoxTransforms(
          capsuleSpec.radius, boxSpec.halfExtents, edge);
      RunBulletRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      return;
    }
  }
}
#endif

#if DART_HAVE_ODE
void RunNarrowPhaseCaseOdeRaw(
    benchmark::State& state,
    PairKind pair,
    EdgeCase edge)
{
  const double scale = ScaleFromIndex(static_cast<int>(state.range(0)));
  const auto sphereSpec = MakeSphereSpec(scale);
  const auto capsuleSpec = MakeCapsuleSpec(scale);
  const auto boxSpec = MakeBoxSpec(scale, edge == EdgeCase::kThinFeature);
  const auto sanity = RawCollisionSanity::kAllowNoCollision;

  OdeRuntime ode;

  switch (pair) {
    case PairKind::kSphereSphere: {
      dGeomID shape1 = dCreateSphere(0, sphereSpec.radius);
      dGeomID shape2 = dCreateSphere(0, sphereSpec.radius);
      const auto tfs = MakeSphereSphereTransforms(
          sphereSpec.radius, sphereSpec.radius, edge);
      RunOdeRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      dGeomDestroy(shape1);
      dGeomDestroy(shape2);
      return;
    }
    case PairKind::kBoxBox: {
      dGeomID shape1 = dCreateBox(
          0, boxSpec.size.x(), boxSpec.size.y(), boxSpec.size.z());
      dGeomID shape2 = dCreateBox(
          0, boxSpec.size.x(), boxSpec.size.y(), boxSpec.size.z());
      const auto tfs = MakeBoxBoxTransforms(
          boxSpec.halfExtents, boxSpec.halfExtents, edge);
      RunOdeRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      dGeomDestroy(shape1);
      dGeomDestroy(shape2);
      return;
    }
    case PairKind::kCapsuleCapsule: {
      dGeomID shape1
          = dCreateCapsule(0, capsuleSpec.radius, capsuleSpec.height);
      dGeomID shape2
          = dCreateCapsule(0, capsuleSpec.radius, capsuleSpec.height);
      const auto tfs = MakeCapsuleCapsuleTransforms(
          capsuleSpec.radius, capsuleSpec.radius, edge);
      RunOdeRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      dGeomDestroy(shape1);
      dGeomDestroy(shape2);
      return;
    }
    case PairKind::kSphereBox: {
      dGeomID shape1 = dCreateSphere(0, sphereSpec.radius);
      dGeomID shape2 = dCreateBox(
          0, boxSpec.size.x(), boxSpec.size.y(), boxSpec.size.z());
      const auto tfs = MakeSphereBoxTransforms(
          sphereSpec.radius, boxSpec.halfExtents, edge);
      RunOdeRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      dGeomDestroy(shape1);
      dGeomDestroy(shape2);
      return;
    }
    case PairKind::kCapsuleSphere: {
      dGeomID shape1
          = dCreateCapsule(0, capsuleSpec.radius, capsuleSpec.height);
      dGeomID shape2 = dCreateSphere(0, sphereSpec.radius);
      const auto tfs = MakeCapsuleSphereTransforms(
          capsuleSpec.radius, sphereSpec.radius, edge);
      RunOdeRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      dGeomDestroy(shape1);
      dGeomDestroy(shape2);
      return;
    }
    case PairKind::kCapsuleBox: {
      dGeomID shape1
          = dCreateCapsule(0, capsuleSpec.radius, capsuleSpec.height);
      dGeomID shape2 = dCreateBox(
          0, boxSpec.size.x(), boxSpec.size.y(), boxSpec.size.z());
      const auto tfs = MakeCapsuleBoxTransforms(
          capsuleSpec.radius, boxSpec.halfExtents, edge);
      RunOdeRawCollisionBenchmark(
          state, shape1, tfs.tf1, shape2, tfs.tf2, sanity);
      dGeomDestroy(shape1);
      dGeomDestroy(shape2);
      return;
    }
  }
}
#endif

static void BM_NarrowPhaseRawReference_EdgeCases_Native(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  RunNarrowPhaseCaseNative(state, pair, edge);
}

static void BM_NarrowPhaseRawReference_EdgeCases_FCL(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  RunNarrowPhaseCaseFclRaw(state, pair, edge);
}

#if DART_HAVE_BULLET
static void BM_NarrowPhaseRawReference_EdgeCases_Bullet(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  RunNarrowPhaseCaseBulletRaw(state, pair, edge);
}
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhaseRawReference_EdgeCases_ODE(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  RunNarrowPhaseCaseOdeRaw(state, pair, edge);
}
#endif

} // namespace edge_case_bench

using dart::benchmark::collision::EdgeCase;
using dart::benchmark::collision::PairKind;

using EdgeCaseBenchmark = void (*)(benchmark::State&, PairKind, EdgeCase);

static std::string EdgeCaseLabel(PairKind pair, EdgeCase edge)
{
  std::string label = dart::benchmark::collision::PairKindName(pair);
  label += "_";
  label += dart::benchmark::collision::EdgeCaseName(edge);
  return label;
}

static void RegisterEdgeCaseBenchmark(
    const char* prefix, EdgeCaseBenchmark fn, PairKind pair, EdgeCase edge)
{
  std::string name = std::string(prefix) + "/" + EdgeCaseLabel(pair, edge);
  auto* bench = benchmark::RegisterBenchmark(name.c_str(), fn, pair, edge);
  edge_case_bench::AddScaleArgs(bench);
}

template <
    std::size_t BasePairCount,
    std::size_t BoxPairCount,
    std::size_t BaseEdgeCount,
    std::size_t BoxEdgeCount>
static void RegisterEdgeCaseBenchmarkGroup(
    const char* prefix,
    EdgeCaseBenchmark fn,
    const std::array<PairKind, BasePairCount>& base_pairs,
    const std::array<PairKind, BoxPairCount>& box_pairs,
    const std::array<EdgeCase, BaseEdgeCount>& base_edges,
    const std::array<EdgeCase, BoxEdgeCount>& box_edges)
{
  for (EdgeCase edge : base_edges) {
    for (PairKind pair : base_pairs)
      RegisterEdgeCaseBenchmark(prefix, fn, pair, edge);
  }
  for (EdgeCase edge : box_edges) {
    for (PairKind pair : box_pairs)
      RegisterEdgeCaseBenchmark(prefix, fn, pair, edge);
  }
}

static void RegisterNarrowPhaseEdgeCases()
{
  const std::array<EdgeCase, 3> base_edges
      = {EdgeCase::kTouching, EdgeCase::kDeepPenetration, EdgeCase::kGrazing};
  const std::array<EdgeCase, 4> box_edges
      = {EdgeCase::kTouching,
         EdgeCase::kDeepPenetration,
         EdgeCase::kGrazing,
         EdgeCase::kThinFeature};
  const std::array<PairKind, 3> base_pairs = {PairKind::kSphereSphere,
                                             PairKind::kCapsuleCapsule,
                                             PairKind::kCapsuleSphere};
  const std::array<PairKind, 3> box_pairs
      = {PairKind::kBoxBox, PairKind::kSphereBox, PairKind::kCapsuleBox};

  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhase_EdgeCases_Native",
      edge_case_bench::BM_NarrowPhase_EdgeCases_Native,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);
  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhase_EdgeCases_FCL",
      edge_case_bench::BM_NarrowPhase_EdgeCases_FCL,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);

#if DART_HAVE_BULLET
  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhase_EdgeCases_Bullet",
      edge_case_bench::BM_NarrowPhase_EdgeCases_Bullet,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);
#endif

#if DART_HAVE_ODE
  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhase_EdgeCases_ODE",
      edge_case_bench::BM_NarrowPhase_EdgeCases_ODE,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);
#endif

  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhaseAdapter_EdgeCases_Native",
      edge_case_bench::BM_NarrowPhaseAdapter_EdgeCases_Native,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);
  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhaseAdapter_EdgeCases_FCL",
      edge_case_bench::BM_NarrowPhaseAdapter_EdgeCases_FCL,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);

#if DART_HAVE_BULLET
  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhaseAdapter_EdgeCases_Bullet",
      edge_case_bench::BM_NarrowPhaseAdapter_EdgeCases_Bullet,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);
#endif

#if DART_HAVE_ODE
  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhaseAdapter_EdgeCases_ODE",
      edge_case_bench::BM_NarrowPhaseAdapter_EdgeCases_ODE,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);
#endif

  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhaseRawReference_EdgeCases_Native",
      edge_case_bench::BM_NarrowPhaseRawReference_EdgeCases_Native,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);
  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhaseRawReference_EdgeCases_FCL",
      edge_case_bench::BM_NarrowPhaseRawReference_EdgeCases_FCL,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);

#if DART_HAVE_BULLET
  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhaseRawReference_EdgeCases_Bullet",
      edge_case_bench::BM_NarrowPhaseRawReference_EdgeCases_Bullet,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);
#endif

#if DART_HAVE_ODE
  RegisterEdgeCaseBenchmarkGroup(
      "BM_NarrowPhaseRawReference_EdgeCases_ODE",
      edge_case_bench::BM_NarrowPhaseRawReference_EdgeCases_ODE,
      base_pairs,
      box_pairs,
      base_edges,
      box_edges);
#endif
}

struct NarrowPhaseEdgeCaseRegistrar
{
  NarrowPhaseEdgeCaseRegistrar()
  {
    RegisterNarrowPhaseEdgeCases();
  }
};

static NarrowPhaseEdgeCaseRegistrar g_narrow_phase_edge_case_registrar;

namespace accuracy {

constexpr double kDepthWarn = 0.05;

using NativeRunner = std::function<void(CollisionResult&)>;

bool verifyPairAccuracy(
    const char* label,
    const NativeRunner& runner,
    const std::shared_ptr<dart::dynamics::Shape>& shape1,
    const Eigen::Isometry3d& tf1,
    const std::shared_ptr<dart::dynamics::Shape>& shape2,
    const Eigen::Isometry3d& tf2,
    double depth_warn = kDepthWarn)
{
  CollisionResult expResult;
  runner(expResult);

  auto detector = dart::collision::FCLCollisionDetector::createReference();
  auto ctx = MakePairContext(detector, shape1, tf1, shape2, tf2);
  auto option = dart::benchmark::collision::MakeCollisionOption();
  dart::collision::CollisionResult fclResult;
  detector->collide(ctx.group.get(), option, &fclResult);

  bool expCollided = expResult.numContacts() > 0;
  bool fclCollided = fclResult.getNumContacts() > 0;

  if (expCollided != fclCollided) {
    std::cerr << "ACCURACY FAIL: " << label
              << " collision detection mismatch\n";
    std::cerr << "  Native: " << (expCollided ? "collided" : "no collision")
              << "\n";
    std::cerr << "  FCL: " << (fclCollided ? "collided" : "no collision")
              << "\n";
    return false;
  }

  if (!expCollided) {
    return true;
  }

  const auto& nativeContact = expResult.getContact(0);
  const auto& fclContact = fclResult.getContact(0);

  double depthDiff
      = std::abs(nativeContact.depth - fclContact.penetrationDepth);
  if (depthDiff > depth_warn) {
    std::cerr << "ACCURACY WARN: " << label << " depth diff " << depthDiff
              << "\n";
  }

  return true;
}

void runAccuracyChecks()
{
  std::cout << "\n=== ACCURACY VERIFICATION ===\n";

  bool allPassed = true;

  auto logResult = [&](const char* label, bool passed) {
    std::cout << label << ": " << (passed ? "PASS" : "FAIL") << "\n";
    if (!passed) {
      allPassed = false;
    }
  };

  {
    SphereShape exp1(1.0);
    SphereShape exp2(1.0);
    const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf2 = OffsetTransform(1.5, 0.0, 0.0);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideSpheres(exp1, tf1, exp2, tf2, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
    auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

    logResult(
        "Sphere-Sphere",
        verifyPairAccuracy("Sphere-Sphere", runner, shape1, tf1, shape2, tf2));
  }

  {
    BoxShape exp1(Eigen::Vector3d(0.5, 0.5, 0.5));
    BoxShape exp2(Eigen::Vector3d(0.5, 0.5, 0.5));
    const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideBoxes(exp1, tf1, exp2, tf2, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(1.0, 1.0, 1.0));
    auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(1.0, 1.0, 1.0));

    logResult(
        "Box-Box",
        verifyPairAccuracy("Box-Box", runner, shape1, tf1, shape2, tf2));
  }

  {
    CapsuleShape exp1(0.5, 2.0);
    CapsuleShape exp2(0.5, 2.0);
    const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideCapsules(exp1, tf1, exp2, tf2, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
    auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

    logResult(
        "Capsule-Capsule",
        verifyPairAccuracy(
            "Capsule-Capsule", runner, shape1, tf1, shape2, tf2));
  }

  {
    SphereShape exp_sphere(0.5);
    BoxShape exp_box(Eigen::Vector3d(0.5, 0.5, 0.5));
    const Eigen::Isometry3d tf_sphere = OffsetTransform(0.8, 0.0, 0.0);
    const Eigen::Isometry3d tf_box = Eigen::Isometry3d::Identity();

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideSphereBox(
          exp_sphere, tf_sphere, exp_box, tf_box, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::SphereShape>(0.5);
    auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(1.0, 1.0, 1.0));

    logResult(
        "Sphere-Box",
        verifyPairAccuracy(
            "Sphere-Box", runner, shape1, tf_sphere, shape2, tf_box));
  }

  {
    CapsuleShape exp_capsule(0.5, 2.0);
    SphereShape exp_sphere(0.5);
    const Eigen::Isometry3d tf_capsule = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf_sphere = OffsetTransform(0.8, 0.0, 0.0);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideCapsuleSphere(
          exp_capsule, tf_capsule, exp_sphere, tf_sphere, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
    auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

    logResult(
        "Capsule-Sphere",
        verifyPairAccuracy(
            "Capsule-Sphere", runner, shape1, tf_capsule, shape2, tf_sphere));
  }

  {
    CapsuleShape exp_capsule(0.5, 2.0);
    BoxShape exp_box(Eigen::Vector3d(0.5, 0.5, 0.5));
    const Eigen::Isometry3d tf_capsule = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf_box = OffsetTransform(0.8, 0.0, 0.0);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideCapsuleBox(
          exp_capsule, tf_capsule, exp_box, tf_box, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
    auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(1.0, 1.0, 1.0));

    logResult(
        "Capsule-Box",
        verifyPairAccuracy(
            "Capsule-Box", runner, shape1, tf_capsule, shape2, tf_box));
  }

  {
    CylinderShape exp1(0.5, 2.0);
    CylinderShape exp2(0.5, 2.0);
    const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideCylinders(exp1, tf1, exp2, tf2, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
    auto shape2 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);

    logResult(
        "Cylinder-Cylinder",
        verifyPairAccuracy(
            "Cylinder-Cylinder", runner, shape1, tf1, shape2, tf2));
  }

  {
    CylinderShape exp_cyl(0.5, 2.0);
    SphereShape exp_sphere(0.5);
    const Eigen::Isometry3d tf_cyl = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf_sphere = OffsetTransform(0.8, 0.0, 0.0);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideCylinderSphere(
          exp_cyl, tf_cyl, exp_sphere, tf_sphere, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
    auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

    logResult(
        "Cylinder-Sphere",
        verifyPairAccuracy(
            "Cylinder-Sphere", runner, shape1, tf_cyl, shape2, tf_sphere));
  }

  {
    CylinderShape exp_cyl(0.5, 2.0);
    BoxShape exp_box(Eigen::Vector3d(0.5, 0.5, 0.5));
    const Eigen::Isometry3d tf_cyl = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf_box = OffsetTransform(0.8, 0.0, 0.0);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideCylinderBox(
          exp_cyl, tf_cyl, exp_box, tf_box, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
    auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(1.0, 1.0, 1.0));

    logResult(
        "Cylinder-Box",
        verifyPairAccuracy(
            "Cylinder-Box", runner, shape1, tf_cyl, shape2, tf_box));
  }

  {
    CylinderShape exp_cyl(0.5, 2.0);
    CapsuleShape exp_capsule(0.5, 2.0);
    const Eigen::Isometry3d tf_cyl = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf_capsule = OffsetTransform(0.8, 0.0, 0.0);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideCylinderCapsule(
          exp_cyl, tf_cyl, exp_capsule, tf_capsule, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
    auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

    logResult(
        "Cylinder-Capsule",
        verifyPairAccuracy(
            "Cylinder-Capsule", runner, shape1, tf_cyl, shape2, tf_capsule));
  }

  {
    CylinderShape exp_cyl(0.5, 2.0);
    PlaneShape exp_plane(Eigen::Vector3d::UnitZ(), 0.0);
    const Eigen::Isometry3d tf_cyl = OffsetTransform(0.0, 0.0, 0.3);
    const Eigen::Isometry3d tf_plane = Eigen::Isometry3d::Identity();

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collideCylinderPlane(
          exp_cyl, tf_cyl, exp_plane, tf_plane, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
    auto shape2 = std::make_shared<dart::dynamics::PlaneShape>(
        Eigen::Vector3d::UnitZ(), 0.0);

    logResult(
        "Cylinder-Plane",
        verifyPairAccuracy(
            "Cylinder-Plane", runner, shape1, tf_cyl, shape2, tf_plane));
  }

  {
    PlaneShape exp_plane(Eigen::Vector3d::UnitZ(), 0.0);
    SphereShape exp_sphere(0.5);
    const Eigen::Isometry3d tf_plane = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf_sphere = OffsetTransform(0.0, 0.0, 0.3);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collidePlaneSphere(
          exp_plane, tf_plane, exp_sphere, tf_sphere, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
        Eigen::Vector3d::UnitZ(), 0.0);
    auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

    logResult(
        "Plane-Sphere",
        verifyPairAccuracy(
            "Plane-Sphere", runner, shape1, tf_plane, shape2, tf_sphere));
  }

  {
    PlaneShape exp_plane(Eigen::Vector3d::UnitZ(), 0.0);
    BoxShape exp_box(Eigen::Vector3d(0.5, 0.5, 0.5));
    const Eigen::Isometry3d tf_plane = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf_box = OffsetTransform(0.0, 0.0, 0.3);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collidePlaneBox(
          exp_plane, tf_plane, exp_box, tf_box, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
        Eigen::Vector3d::UnitZ(), 0.0);
    auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d(1.0, 1.0, 1.0));

    logResult(
        "Plane-Box",
        verifyPairAccuracy(
            "Plane-Box", runner, shape1, tf_plane, shape2, tf_box));
  }

  {
    PlaneShape exp_plane(Eigen::Vector3d::UnitZ(), 0.0);
    CapsuleShape exp_capsule(0.5, 2.0);
    const Eigen::Isometry3d tf_plane = Eigen::Isometry3d::Identity();
    const Eigen::Isometry3d tf_capsule = OffsetTransform(0.0, 0.0, 0.3);

    auto runner = [&](CollisionResult& result) {
      CollisionOption option;
      (void)collidePlaneCapsule(
          exp_plane, tf_plane, exp_capsule, tf_capsule, result, option);
    };

    auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
        Eigen::Vector3d::UnitZ(), 0.0);
    auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

    logResult(
        "Plane-Capsule",
        verifyPairAccuracy(
            "Plane-Capsule", runner, shape1, tf_plane, shape2, tf_capsule));
  }

  std::cout << "=== ACCURACY VERIFICATION " << (allPassed ? "PASSED" : "FAILED")
            << " ===\n\n";
}

} // namespace accuracy

int main(int argc, char** argv)
{
  accuracy::runAccuracyChecks();

  ::benchmark::Initialize(&argc, argv);
  ::benchmark::RunSpecifiedBenchmarks();
  ::benchmark::Shutdown();
  return 0;
}
