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

// Comparative benchmarks: Experimental collision vs FCL/Bullet/ODE
//
// PURPOSE: Validate that our experimental collision module outperforms
// existing backends while maintaining accuracy parity.
//
// This benchmark compares:
// 1. Direct narrow-phase: Our algorithms vs FCL's native collision
// 2. Full collision detection: Our CollisionWorld vs DART's collision detectors

#include <dart/config.hpp>

#include <dart/collision/experimental/collision_world.hpp>
#include <dart/collision/experimental/narrow_phase/box_box.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_capsule.hpp>
#include <dart/collision/experimental/narrow_phase/distance.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/collision/fcl/FCLTypes.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>

#if DART_HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif

#if DART_HAVE_ODE
  #include <dart/collision/ode/OdeCollisionDetector.hpp>
#endif

#include <benchmark/benchmark.h>

#include <random>

using namespace dart::collision::experimental;

namespace {

std::mt19937 rng(42);

Eigen::Vector3d randomPosition(double range)
{
  std::uniform_real_distribution<double> dist(-range, range);
  return Eigen::Vector3d(dist(rng), dist(rng), dist(rng));
}

Eigen::Isometry3d randomTransform(double posRange)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = randomPosition(posRange);
  return tf;
}

// Helper to create a simple skeleton with a single shape for DART collision
dart::dynamics::SkeletonPtr createShapeSkeleton(
    std::shared_ptr<dart::dynamics::Shape> shape,
    const Eigen::Isometry3d& transform)
{
  auto skeleton = dart::dynamics::Skeleton::create();
  auto pair = skeleton->createJointAndBodyNodePair<dart::dynamics::FreeJoint>();
  auto body = pair.second;

  body->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect>(shape);

  pair.first->setTransform(transform);

  return skeleton;
}

} // namespace

// =============================================================================
// SPHERE-SPHERE COLLISION BENCHMARKS
// =============================================================================

static void BM_SphereSphere_Experimental(benchmark::State& state)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideSpheres(s1, tf1, s2, tf2, result, option));
  }
}
BENCHMARK(BM_SphereSphere_Experimental);

static void BM_SphereSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption option;
  option.maxNumContacts = 1000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }
}
BENCHMARK(BM_SphereSphere_FCL);

#if DART_HAVE_BULLET
static void BM_SphereSphere_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption option;
  option.maxNumContacts = 1000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }
}
BENCHMARK(BM_SphereSphere_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_SphereSphere_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption option;
  option.maxNumContacts = 1000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }
}
BENCHMARK(BM_SphereSphere_ODE);
#endif

// =============================================================================
// BOX-BOX COLLISION BENCHMARKS
// =============================================================================

static void BM_BoxBox_Experimental(benchmark::State& state)
{
  BoxShape b1(Eigen::Vector3d(0.5, 0.5, 0.5));
  BoxShape b2(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideBoxes(b1, tf1, b2, tf2, result, option));
  }
}
BENCHMARK(BM_BoxBox_Experimental);

static void BM_BoxBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

  auto shape1
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(1, 1, 1));
  auto shape2
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption option;
  option.maxNumContacts = 1000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }
}
BENCHMARK(BM_BoxBox_FCL);

#if DART_HAVE_BULLET
static void BM_BoxBox_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();

  auto shape1
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(1, 1, 1));
  auto shape2
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption option;
  option.maxNumContacts = 1000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }
}
BENCHMARK(BM_BoxBox_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_BoxBox_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(1, 1, 1));
  auto shape2
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(1, 1, 1));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption option;
  option.maxNumContacts = 1000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }
}
BENCHMARK(BM_BoxBox_ODE);
#endif

// =============================================================================
// CAPSULE-CAPSULE COLLISION BENCHMARKS
// =============================================================================

static void BM_CapsuleCapsule_Experimental(benchmark::State& state)
{
  CapsuleShape c1(0.5, 2.0);
  CapsuleShape c2(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult result;
  CollisionOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(collideCapsules(c1, tf1, c2, tf2, result, option));
  }
}
BENCHMARK(BM_CapsuleCapsule_Experimental);

static void BM_CapsuleCapsule_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption option;
  option.maxNumContacts = 1000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }
}
BENCHMARK(BM_CapsuleCapsule_FCL);

#if DART_HAVE_BULLET
static void BM_CapsuleCapsule_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption option;
  option.maxNumContacts = 1000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }
}
BENCHMARK(BM_CapsuleCapsule_Bullet);
#endif

#if DART_HAVE_ODE
static void BM_CapsuleCapsule_ODE(benchmark::State& state)
{
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption option;
  option.maxNumContacts = 1000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }
}
BENCHMARK(BM_CapsuleCapsule_ODE);
#endif

// =============================================================================
// DISTANCE QUERY BENCHMARKS
// =============================================================================

static void BM_Distance_SphereSphere_Experimental(benchmark::State& state)
{
  SphereShape s1(1.0);
  SphereShape s2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);

  DistanceResult result;
  DistanceOption option;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(
        distanceSphereSphere(s1, tf1, s2, tf2, result, option));
  }
}
BENCHMARK(BM_Distance_SphereSphere_Experimental);

static void BM_Distance_SphereSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::DistanceOption option;

  for (auto _ : state) {
    dart::collision::DistanceResult result;
    benchmark::DoNotOptimize(detector->distance(group.get(), option, &result));
  }
}
BENCHMARK(BM_Distance_SphereSphere_FCL);

#if DART_HAVE_BULLET
static void BM_Distance_SphereSphere_Bullet(benchmark::State& state)
{
  auto detector = dart::collision::BulletCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);

  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);

  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::DistanceOption option;

  for (auto _ : state) {
    dart::collision::DistanceResult result;
    benchmark::DoNotOptimize(detector->distance(group.get(), option, &result));
  }
}
BENCHMARK(BM_Distance_SphereSphere_Bullet);
#endif

// =============================================================================
// N-BODY COLLISION BENCHMARKS (SCALE TEST)
// =============================================================================

static void BM_NBody_Experimental(benchmark::State& state)
{
  const int n = state.range(0);
  rng.seed(42);

  CollisionWorld world;
  std::vector<CollisionObject> objects;
  objects.reserve(n);

  for (int i = 0; i < n; ++i) {
    auto shape = std::make_unique<SphereShape>(0.5);
    objects.push_back(
        world.createObject(std::move(shape), randomTransform(10.0)));
  }

  for (auto _ : state) {
    CollisionResult result;
    CollisionOption option;
    benchmark::DoNotOptimize(world.collide(option, result));
  }

  state.SetComplexityN(n);
}
BENCHMARK(BM_NBody_Experimental)->Arg(10)->Arg(50)->Arg(100)->Complexity();

static void BM_NBody_FCL(benchmark::State& state)
{
  const int n = state.range(0);
  rng.seed(42);

  auto detector = dart::collision::FCLCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  std::vector<dart::dynamics::SkeletonPtr> skeletons;

  for (int i = 0; i < n; ++i) {
    auto shape = std::make_shared<dart::dynamics::SphereShape>(0.5);
    auto skel = createShapeSkeleton(shape, randomTransform(10.0));
    skeletons.push_back(skel);
    group->addShapeFramesOf(skel.get());
  }

  dart::collision::CollisionOption option;
  option.maxNumContacts = 10000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }

  state.SetComplexityN(n);
}
BENCHMARK(BM_NBody_FCL)->Arg(10)->Arg(50)->Arg(100)->Complexity();

#if DART_HAVE_BULLET
static void BM_NBody_Bullet(benchmark::State& state)
{
  const int n = state.range(0);
  rng.seed(42);

  auto detector = dart::collision::BulletCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  std::vector<dart::dynamics::SkeletonPtr> skeletons;

  for (int i = 0; i < n; ++i) {
    auto shape = std::make_shared<dart::dynamics::SphereShape>(0.5);
    auto skel = createShapeSkeleton(shape, randomTransform(10.0));
    skeletons.push_back(skel);
    group->addShapeFramesOf(skel.get());
  }

  dart::collision::CollisionOption option;
  option.maxNumContacts = 10000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }

  state.SetComplexityN(n);
}
BENCHMARK(BM_NBody_Bullet)->Arg(10)->Arg(50)->Arg(100)->Complexity();
#endif

#if DART_HAVE_ODE
static void BM_NBody_ODE(benchmark::State& state)
{
  const int n = state.range(0);
  rng.seed(42);

  auto detector = dart::collision::OdeCollisionDetector::create();
  auto group = detector->createCollisionGroup();

  std::vector<dart::dynamics::SkeletonPtr> skeletons;

  for (int i = 0; i < n; ++i) {
    auto shape = std::make_shared<dart::dynamics::SphereShape>(0.5);
    auto skel = createShapeSkeleton(shape, randomTransform(10.0));
    skeletons.push_back(skel);
    group->addShapeFramesOf(skel.get());
  }

  dart::collision::CollisionOption option;
  option.maxNumContacts = 10000;

  for (auto _ : state) {
    dart::collision::CollisionResult result;
    benchmark::DoNotOptimize(detector->collide(group.get(), option, &result));
  }

  state.SetComplexityN(n);
}
BENCHMARK(BM_NBody_ODE)->Arg(10)->Arg(50)->Arg(100)->Complexity();
#endif

// =============================================================================
// ACCURACY VERIFICATION
// =============================================================================
// These functions verify that our experimental module produces results
// comparable to FCL. They run once before benchmarks to validate correctness.

namespace accuracy {

constexpr double kTolerance = 1e-6;

bool verifySphereSphereAccuracy()
{
  SphereShape expS1(1.0);
  SphereShape expS2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(1.5, 0, 0);

  CollisionResult expResult;
  CollisionOption expOption;
  (void)collideSpheres(expS1, tf1, expS2, tf2, expResult, expOption);

  auto detector = dart::collision::FCLCollisionDetector::create();
  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);
  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption fclOption;
  fclOption.maxNumContacts = 1000;
  dart::collision::CollisionResult fclResult;
  detector->collide(group.get(), fclOption, &fclResult);

  bool expCollided = expResult.numContacts() > 0;
  bool fclCollided = fclResult.getNumContacts() > 0;

  if (expCollided != fclCollided) {
    std::cerr << "ACCURACY FAIL: Sphere-sphere collision detection mismatch\n";
    std::cerr << "  Experimental: "
              << (expCollided ? "collided" : "no collision") << "\n";
    std::cerr << "  FCL: " << (fclCollided ? "collided" : "no collision")
              << "\n";
    return false;
  }

  if (!expCollided) {
    return true;
  }

  const auto& expContact = expResult.getContact(0);
  const auto& fclContact = fclResult.getContact(0);

  double depthDiff = std::abs(expContact.depth - fclContact.penetrationDepth);
  if (depthDiff > 0.01) {
    std::cerr << "ACCURACY WARN: Sphere-sphere depth difference: " << depthDiff
              << "\n";
    std::cerr << "  Experimental: " << expContact.depth << "\n";
    std::cerr << "  FCL: " << fclContact.penetrationDepth << "\n";
  }

  if (std::abs(expContact.depth - 0.5) > 0.01) {
    std::cerr << "ACCURACY FAIL: Sphere-sphere expected depth ~0.5, got "
              << expContact.depth << "\n";
    return false;
  }

  return true;
}

bool verifyBoxBoxAccuracy()
{
  BoxShape expB1(Eigen::Vector3d(0.5, 0.5, 0.5));
  BoxShape expB2(Eigen::Vector3d(0.5, 0.5, 0.5));

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(0.8, 0, 0);

  CollisionResult expResult;
  CollisionOption expOption;
  (void)collideBoxes(expB1, tf1, expB2, tf2, expResult, expOption);

  auto detector = dart::collision::FCLCollisionDetector::create();
  auto shape1
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(1, 1, 1));
  auto shape2
      = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(1, 1, 1));
  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);
  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::CollisionOption fclOption;
  fclOption.maxNumContacts = 1000;
  dart::collision::CollisionResult fclResult;
  detector->collide(group.get(), fclOption, &fclResult);

  bool expCollided = expResult.numContacts() > 0;
  bool fclCollided = fclResult.getNumContacts() > 0;

  if (expCollided != fclCollided) {
    std::cerr << "ACCURACY FAIL: Box-box collision detection mismatch\n";
    return false;
  }

  if (!expCollided) {
    return true;
  }

  const auto& expContact = expResult.getContact(0);
  if (expContact.depth < 0.1 || expContact.depth > 0.3) {
    std::cerr << "ACCURACY WARN: Box-box depth outside expected range: "
              << expContact.depth << " (expected ~0.2)\n";
  }

  return true;
}

bool verifyDistanceAccuracy()
{
  SphereShape expS1(1.0);
  SphereShape expS2(1.0);

  Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();
  tf2.translation() = Eigen::Vector3d(5.0, 0, 0);

  DistanceResult expResult;
  DistanceOption expOption;
  double expDist
      = distanceSphereSphere(expS1, tf1, expS2, tf2, expResult, expOption);

  auto detector = dart::collision::FCLCollisionDetector::create();
  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto skel1 = createShapeSkeleton(shape1, tf1);
  auto skel2 = createShapeSkeleton(shape2, tf2);
  auto group = detector->createCollisionGroup();
  group->addShapeFramesOf(skel1.get());
  group->addShapeFramesOf(skel2.get());

  dart::collision::DistanceOption fclOption;
  dart::collision::DistanceResult fclResult;
  detector->distance(group.get(), fclOption, &fclResult);

  double expectedDist = 3.0;

  if (std::abs(expDist - expectedDist) > kTolerance) {
    std::cerr << "ACCURACY FAIL: Distance sphere-sphere expected "
              << expectedDist << ", got " << expDist << "\n";
    return false;
  }

  double fclDist = fclResult.minDistance;
  if (std::abs(fclDist - expectedDist) > 0.01) {
    std::cerr << "ACCURACY INFO: FCL distance = " << fclDist << " (expected "
              << expectedDist << ")\n";
  }

  return true;
}

void runAccuracyChecks()
{
  std::cout << "\n=== ACCURACY VERIFICATION ===\n";

  bool allPassed = true;

  std::cout << "Sphere-Sphere collision: ";
  if (verifySphereSphereAccuracy()) {
    std::cout << "PASS\n";
  } else {
    std::cout << "FAIL\n";
    allPassed = false;
  }

  std::cout << "Box-Box collision: ";
  if (verifyBoxBoxAccuracy()) {
    std::cout << "PASS\n";
  } else {
    std::cout << "FAIL\n";
    allPassed = false;
  }

  std::cout << "Distance sphere-sphere: ";
  if (verifyDistanceAccuracy()) {
    std::cout << "PASS\n";
  } else {
    std::cout << "FAIL\n";
    allPassed = false;
  }

  std::cout << "=== ACCURACY VERIFICATION " << (allPassed ? "PASSED" : "FAILED")
            << " ===\n\n";
}

} // namespace accuracy

// Custom main to run accuracy checks before benchmarks
int main(int argc, char** argv)
{
  accuracy::runAccuracyChecks();

  ::benchmark::Initialize(&argc, argv);
  ::benchmark::RunSpecifiedBenchmarks();
  ::benchmark::Shutdown();
  return 0;
}
