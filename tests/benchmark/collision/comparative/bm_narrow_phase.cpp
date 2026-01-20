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

#include <dart/collision/experimental/narrow_phase/box_box.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_box.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_capsule.hpp>
#include <dart/collision/experimental/narrow_phase/capsule_sphere.hpp>
#include <dart/collision/experimental/narrow_phase/cylinder_collision.hpp>
#include <dart/collision/experimental/narrow_phase/plane_sphere.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_box.hpp>
#include <dart/collision/experimental/narrow_phase/sphere_sphere.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>

#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SphereShape.hpp>

#if DART_HAVE_BULLET
  #include <dart/collision/bullet/BulletCollisionDetector.hpp>
#endif

#if DART_HAVE_ODE
  #include <dart/collision/ode/OdeCollisionDetector.hpp>
#endif

#include "tests/benchmark/collision/fixtures/edge_cases.hpp"
#include "tests/benchmark/collision/fixtures/shape_factories.hpp"

#include <benchmark/benchmark.h>

#include <functional>
#include <iostream>
#include <memory>

#include <cmath>

using namespace dart::collision::experimental;

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

} // namespace

static void BM_NarrowPhase_SphereSphere_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_SphereSphere_Experimental);

static void BM_NarrowPhase_SphereSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(1.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(1.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(1.5, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_SphereSphere_ODE);
#endif

static void BM_NarrowPhase_BoxBox_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_BoxBox_Experimental);

static void BM_NarrowPhase_BoxBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  const Eigen::Vector3d size(1.0, 1.0, 1.0);
  auto shape1 = std::make_shared<dart::dynamics::BoxShape>(size);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(size);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_BoxBox_ODE);
#endif

static void BM_NarrowPhase_CapsuleCapsule_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_CapsuleCapsule_Experimental);

static void BM_NarrowPhase_CapsuleCapsule_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleCapsule_ODE);
#endif

static void BM_NarrowPhase_SphereBox_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_SphereBox_Experimental);

static void BM_NarrowPhase_SphereBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::SphereShape>(0.5);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = OffsetTransform(0.8, 0.0, 0.0);
  const Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_SphereBox_ODE);
#endif

static void BM_NarrowPhase_CapsuleSphere_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_CapsuleSphere_Experimental);

static void BM_NarrowPhase_CapsuleSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleSphere_ODE);
#endif

static void BM_NarrowPhase_CapsuleBox_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_CapsuleBox_Experimental);

static void BM_NarrowPhase_CapsuleBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CapsuleBox_ODE);
#endif

static void BM_NarrowPhase_CylinderCylinder_Experimental(
    benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_CylinderCylinder_Experimental);

static void BM_NarrowPhase_CylinderCylinder_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderCylinder_ODE);
#endif

static void BM_NarrowPhase_CylinderSphere_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_CylinderSphere_Experimental);

static void BM_NarrowPhase_CylinderSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderSphere_ODE);
#endif

static void BM_NarrowPhase_CylinderBox_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_CylinderBox_Experimental);

static void BM_NarrowPhase_CylinderBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(1.0, 1.0, 1.0));

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderBox_ODE);
#endif

static void BM_NarrowPhase_CylinderCapsule_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_CylinderCapsule_Experimental);

static void BM_NarrowPhase_CylinderCapsule_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::CapsuleShape>(0.5, 2.0);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.8, 0.0, 0.0);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderCapsule_ODE);
#endif

static void BM_NarrowPhase_CylinderPlane_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_CylinderPlane_Experimental);

static void BM_NarrowPhase_CylinderPlane_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::CylinderShape>(0.5, 2.0);
  auto shape2 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);

  const Eigen::Isometry3d tf1 = OffsetTransform(0.0, 0.0, 0.3);
  const Eigen::Isometry3d tf2 = Eigen::Isometry3d::Identity();

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_CylinderPlane_ODE);
#endif

static void BM_NarrowPhase_PlaneSphere_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_PlaneSphere_Experimental);

static void BM_NarrowPhase_PlaneSphere_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

  auto shape1 = std::make_shared<dart::dynamics::PlaneShape>(
      Eigen::Vector3d::UnitZ(), 0.0);
  auto shape2 = std::make_shared<dart::dynamics::SphereShape>(0.5);

  const Eigen::Isometry3d tf1 = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d tf2 = OffsetTransform(0.0, 0.0, 0.3);

  RunDetectorBenchmark(state, detector, shape1, tf1, shape2, tf2);
}
BENCHMARK(BM_NarrowPhase_PlaneSphere_ODE);
#endif

static void BM_NarrowPhase_PlaneBox_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_PlaneBox_Experimental);

static void BM_NarrowPhase_PlaneBox_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

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

static void BM_NarrowPhase_PlaneCapsule_Experimental(benchmark::State& state)
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
BENCHMARK(BM_NarrowPhase_PlaneCapsule_Experimental);

static void BM_NarrowPhase_PlaneCapsule_FCL(benchmark::State& state)
{
  auto detector = dart::collision::FCLCollisionDetector::create();

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
  auto detector = dart::collision::BulletCollisionDetector::create();

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
  auto detector = dart::collision::OdeCollisionDetector::create();

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

void AddScaleArgs(benchmark::internal::Benchmark* bench)
{
  bench->Arg(0)->Arg(1)->Arg(2);
}

void RunNarrowPhaseCaseExperimental(
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

static void BM_NarrowPhase_EdgeCases_Experimental(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  RunNarrowPhaseCaseExperimental(state, pair, edge);
}

static void BM_NarrowPhase_EdgeCases_FCL(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::FCLCollisionDetector::create();
  RunNarrowPhaseCaseDetector(state, detector, pair, edge);
}

#if DART_HAVE_BULLET
static void BM_NarrowPhase_EdgeCases_Bullet(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::BulletCollisionDetector::create();
  RunNarrowPhaseCaseDetector(state, detector, pair, edge);
}
#endif

#if DART_HAVE_ODE
static void BM_NarrowPhase_EdgeCases_ODE(
    benchmark::State& state, PairKind pair, EdgeCase edge)
{
  auto detector = dart::collision::OdeCollisionDetector::create();
  RunNarrowPhaseCaseDetector(state, detector, pair, edge);
}
#endif

} // namespace edge_case_bench

using dart::benchmark::collision::EdgeCase;
using dart::benchmark::collision::PairKind;

static void RegisterNarrowPhaseEdgeCases()
{
  using edge_case_bench::AddScaleArgs;

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      SphereSphere_Touching,
      PairKind::kSphereSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      SphereSphere_DeepPenetration,
      PairKind::kSphereSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      SphereSphere_Grazing,
      PairKind::kSphereSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      BoxBox_Touching,
      PairKind::kBoxBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      BoxBox_DeepPenetration,
      PairKind::kBoxBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      BoxBox_Grazing,
      PairKind::kBoxBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      BoxBox_ThinFeature,
      PairKind::kBoxBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      CapsuleCapsule_Touching,
      PairKind::kCapsuleCapsule,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      CapsuleCapsule_DeepPenetration,
      PairKind::kCapsuleCapsule,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      CapsuleCapsule_Grazing,
      PairKind::kCapsuleCapsule,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      SphereBox_Touching,
      PairKind::kSphereBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      SphereBox_DeepPenetration,
      PairKind::kSphereBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      SphereBox_Grazing,
      PairKind::kSphereBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      SphereBox_ThinFeature,
      PairKind::kSphereBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      CapsuleSphere_Touching,
      PairKind::kCapsuleSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      CapsuleSphere_DeepPenetration,
      PairKind::kCapsuleSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      CapsuleSphere_Grazing,
      PairKind::kCapsuleSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      CapsuleBox_Touching,
      PairKind::kCapsuleBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      CapsuleBox_DeepPenetration,
      PairKind::kCapsuleBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      CapsuleBox_Grazing,
      PairKind::kCapsuleBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Experimental,
      CapsuleBox_ThinFeature,
      PairKind::kCapsuleBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      SphereSphere_Touching,
      PairKind::kSphereSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      SphereSphere_DeepPenetration,
      PairKind::kSphereSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      SphereSphere_Grazing,
      PairKind::kSphereSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      BoxBox_Touching,
      PairKind::kBoxBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      BoxBox_DeepPenetration,
      PairKind::kBoxBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      BoxBox_Grazing,
      PairKind::kBoxBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      BoxBox_ThinFeature,
      PairKind::kBoxBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      CapsuleCapsule_Touching,
      PairKind::kCapsuleCapsule,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      CapsuleCapsule_DeepPenetration,
      PairKind::kCapsuleCapsule,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      CapsuleCapsule_Grazing,
      PairKind::kCapsuleCapsule,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      SphereBox_Touching,
      PairKind::kSphereBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      SphereBox_DeepPenetration,
      PairKind::kSphereBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      SphereBox_Grazing,
      PairKind::kSphereBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      SphereBox_ThinFeature,
      PairKind::kSphereBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      CapsuleSphere_Touching,
      PairKind::kCapsuleSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      CapsuleSphere_DeepPenetration,
      PairKind::kCapsuleSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      CapsuleSphere_Grazing,
      PairKind::kCapsuleSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      CapsuleBox_Touching,
      PairKind::kCapsuleBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      CapsuleBox_DeepPenetration,
      PairKind::kCapsuleBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      CapsuleBox_Grazing,
      PairKind::kCapsuleBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_FCL,
      CapsuleBox_ThinFeature,
      PairKind::kCapsuleBox,
      EdgeCase::kThinFeature));

#if DART_HAVE_BULLET
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      SphereSphere_Touching,
      PairKind::kSphereSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      SphereSphere_DeepPenetration,
      PairKind::kSphereSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      SphereSphere_Grazing,
      PairKind::kSphereSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      BoxBox_Touching,
      PairKind::kBoxBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      BoxBox_DeepPenetration,
      PairKind::kBoxBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      BoxBox_Grazing,
      PairKind::kBoxBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      BoxBox_ThinFeature,
      PairKind::kBoxBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      CapsuleCapsule_Touching,
      PairKind::kCapsuleCapsule,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      CapsuleCapsule_DeepPenetration,
      PairKind::kCapsuleCapsule,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      CapsuleCapsule_Grazing,
      PairKind::kCapsuleCapsule,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      SphereBox_Touching,
      PairKind::kSphereBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      SphereBox_DeepPenetration,
      PairKind::kSphereBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      SphereBox_Grazing,
      PairKind::kSphereBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      SphereBox_ThinFeature,
      PairKind::kSphereBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      CapsuleSphere_Touching,
      PairKind::kCapsuleSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      CapsuleSphere_DeepPenetration,
      PairKind::kCapsuleSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      CapsuleSphere_Grazing,
      PairKind::kCapsuleSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      CapsuleBox_Touching,
      PairKind::kCapsuleBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      CapsuleBox_DeepPenetration,
      PairKind::kCapsuleBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      CapsuleBox_Grazing,
      PairKind::kCapsuleBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_Bullet,
      CapsuleBox_ThinFeature,
      PairKind::kCapsuleBox,
      EdgeCase::kThinFeature));
#endif

#if DART_HAVE_ODE
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      SphereSphere_Touching,
      PairKind::kSphereSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      SphereSphere_DeepPenetration,
      PairKind::kSphereSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      SphereSphere_Grazing,
      PairKind::kSphereSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      BoxBox_Touching,
      PairKind::kBoxBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      BoxBox_DeepPenetration,
      PairKind::kBoxBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      BoxBox_Grazing,
      PairKind::kBoxBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      BoxBox_ThinFeature,
      PairKind::kBoxBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      CapsuleCapsule_Touching,
      PairKind::kCapsuleCapsule,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      CapsuleCapsule_DeepPenetration,
      PairKind::kCapsuleCapsule,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      CapsuleCapsule_Grazing,
      PairKind::kCapsuleCapsule,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      SphereBox_Touching,
      PairKind::kSphereBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      SphereBox_DeepPenetration,
      PairKind::kSphereBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      SphereBox_Grazing,
      PairKind::kSphereBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      SphereBox_ThinFeature,
      PairKind::kSphereBox,
      EdgeCase::kThinFeature));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      CapsuleSphere_Touching,
      PairKind::kCapsuleSphere,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      CapsuleSphere_DeepPenetration,
      PairKind::kCapsuleSphere,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      CapsuleSphere_Grazing,
      PairKind::kCapsuleSphere,
      EdgeCase::kGrazing));

  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      CapsuleBox_Touching,
      PairKind::kCapsuleBox,
      EdgeCase::kTouching));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      CapsuleBox_DeepPenetration,
      PairKind::kCapsuleBox,
      EdgeCase::kDeepPenetration));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      CapsuleBox_Grazing,
      PairKind::kCapsuleBox,
      EdgeCase::kGrazing));
  AddScaleArgs(BENCHMARK_CAPTURE(
      BM_NarrowPhase_EdgeCases_ODE,
      CapsuleBox_ThinFeature,
      PairKind::kCapsuleBox,
      EdgeCase::kThinFeature));
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

using ExperimentalRunner = std::function<void(CollisionResult&)>;

bool verifyPairAccuracy(
    const char* label,
    const ExperimentalRunner& runner,
    const std::shared_ptr<dart::dynamics::Shape>& shape1,
    const Eigen::Isometry3d& tf1,
    const std::shared_ptr<dart::dynamics::Shape>& shape2,
    const Eigen::Isometry3d& tf2,
    double depth_warn = kDepthWarn)
{
  CollisionResult expResult;
  runner(expResult);

  auto detector = dart::collision::FCLCollisionDetector::create();
  auto ctx = MakePairContext(detector, shape1, tf1, shape2, tf2);
  auto option = dart::benchmark::collision::MakeCollisionOption();
  dart::collision::CollisionResult fclResult;
  detector->collide(ctx.group.get(), option, &fclResult);

  bool expCollided = expResult.numContacts() > 0;
  bool fclCollided = fclResult.getNumContacts() > 0;

  if (expCollided != fclCollided) {
    std::cerr << "ACCURACY FAIL: " << label
              << " collision detection mismatch\n";
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
