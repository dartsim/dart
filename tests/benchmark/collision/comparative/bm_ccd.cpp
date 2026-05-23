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

// Comparative continuous-collision benchmark: native conservative-advancement
// convex cast vs. each reference engine's best available continuous query on
// the same swept box-box scenario.
//
//   - Reference A: dedicated conservative-advancement continuous collision
//     (translational motion, shape-vs-shape).
//   - Reference B: dedicated GJK-based convex cast (ray cast against the
//     configuration-space obstacle).
//   - Reference C: substepped discrete narrow phase (this engine has no native
//     continuous query, so trajectory sampling is the only option).

#include <dart/config.hpp>

#include <dart/collision/native/narrow_phase/ccd.hpp>
#include <dart/collision/native/shapes/shape.hpp>
#include <dart/collision/native/types.hpp>

#if DART_HAVE_FCL
  #include <fcl/geometry/shape/box.h>
  #include <fcl/geometry/shape/capsule.h>
  #include <fcl/geometry/shape/convex.h>
  #include <fcl/geometry/shape/cylinder.h>
  #include <fcl/geometry/shape/sphere.h>
  #include <fcl/math/motion/spline_motion.h>
  #include <fcl/math/motion/translation_motion.h>
  #include <fcl/narrowphase/continuous_collision.h>
#endif

#if DART_HAVE_BULLET
  #include <BulletCollision/CollisionShapes/btBoxShape.h>
  #include <BulletCollision/CollisionShapes/btCapsuleShape.h>
  #include <BulletCollision/CollisionShapes/btConvexHullShape.h>
  #include <BulletCollision/CollisionShapes/btCylinderShape.h>
  #include <BulletCollision/CollisionShapes/btSphereShape.h>
  #include <BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h>
  #include <BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h>
  #include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
  #include <BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h>
#endif

#if DART_HAVE_ODE
  #include "tests/benchmark/collision/fixtures/shape_factories.hpp"

  #include <dart/test/reference_collision/ode/ode_collision_detector.hpp>

  #include <dart/collision/dart/dart_collision_detector.hpp>

  #include <dart/dynamics/box_shape.hpp>
  #include <dart/dynamics/capsule_shape.hpp>
  #include <dart/dynamics/cylinder_shape.hpp>
  #include <dart/dynamics/free_joint.hpp>
  #include <dart/dynamics/skeleton.hpp>
  #include <dart/dynamics/sphere_shape.hpp>
#endif

#include <benchmark/benchmark.h>

#include <array>
#include <memory>
#include <numbers>

using namespace dart::collision::native;

namespace {

const Eigen::Vector3d kBoxHalf(0.5, 0.5, 0.5);
const Eigen::Vector3d kSweepStart(-5.0, 0.0, 0.0);
const Eigen::Vector3d kSweepEnd(5.0, 0.0, 0.0);

std::vector<Eigen::Vector3d> BoxVertices(const Eigen::Vector3d& half)
{
  return {
      {-half.x(), -half.y(), -half.z()},
      {half.x(), -half.y(), -half.z()},
      {half.x(), half.y(), -half.z()},
      {-half.x(), half.y(), -half.z()},
      {-half.x(), -half.y(), half.z()},
      {half.x(), -half.y(), half.z()},
      {half.x(), half.y(), half.z()},
      {-half.x(), half.y(), half.z()}};
}

Eigen::Isometry3d MakeTranslation(const Eigen::Vector3d& t)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = t;
  return tf;
}

} // namespace

//==============================================================================
// Native: single conservative-advancement convex cast
//==============================================================================

static void BM_CCD_ConvexCast_BoxBox_Native(benchmark::State& state)
{
  ConvexShape shapeA(BoxVertices(kBoxHalf));
  ConvexShape shapeB(BoxVertices(kBoxHalf));

  const Eigen::Isometry3d startTf = MakeTranslation(kSweepStart);
  const Eigen::Isometry3d endTf = MakeTranslation(kSweepEnd);
  const Eigen::Isometry3d staticTf = Eigen::Isometry3d::Identity();

  CcdOption option = CcdOption::standard();
  CcdResult result;

  if (!convexCast(
          shapeA, startTf, endTf, shapeB, staticTf, staticTf, option, result)) {
    state.SkipWithError("Native convex cast setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    bool hit = convexCast(
        shapeA, startTf, endTf, shapeB, staticTf, staticTf, option, result);
    double toi = result.timeOfImpact;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_BoxBox_Native);

//==============================================================================
// Reference A: dedicated conservative-advancement continuous collision
//==============================================================================

#if DART_HAVE_FCL
static void BM_CCD_ConvexCast_BoxBox_ReferenceA(benchmark::State& state)
{
  auto boxA = std::make_shared<fcl::Box<double>>(
      2.0 * kBoxHalf.x(), 2.0 * kBoxHalf.y(), 2.0 * kBoxHalf.z());
  auto boxB = std::make_shared<fcl::Box<double>>(
      2.0 * kBoxHalf.x(), 2.0 * kBoxHalf.y(), 2.0 * kBoxHalf.z());

  const Eigen::Isometry3d tfAbeg = MakeTranslation(kSweepStart);
  const Eigen::Isometry3d tfAend = MakeTranslation(kSweepEnd);
  const Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();

  fcl::ContinuousCollisionRequest<double> request(
      30,
      1e-4,
      fcl::CCDM_LINEAR,
      fcl::GST_INDEP,
      fcl::CCDC_CONSERVATIVE_ADVANCEMENT);

  fcl::ContinuousCollisionResult<double> sanity;
  fcl::continuousCollide(
      boxA.get(), tfAbeg, tfAend, boxB.get(), tfB, tfB, request, sanity);
  if (!sanity.is_collide) {
    state.SkipWithError("Reference A continuous collision setup did not hit.");
    return;
  }

  for (auto _ : state) {
    fcl::ContinuousCollisionResult<double> result;
    fcl::continuousCollide(
        boxA.get(), tfAbeg, tfAend, boxB.get(), tfB, tfB, request, result);
    double toc = result.time_of_contact;
    benchmark::DoNotOptimize(toc);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_BoxBox_ReferenceA);
#endif

//==============================================================================
// Reference B: dedicated GJK-based convex cast
//==============================================================================

#if DART_HAVE_BULLET
static void BM_CCD_ConvexCast_BoxBox_ReferenceB(benchmark::State& state)
{
  btBoxShape boxA(btVector3(kBoxHalf.x(), kBoxHalf.y(), kBoxHalf.z()));
  btBoxShape boxB(btVector3(kBoxHalf.x(), kBoxHalf.y(), kBoxHalf.z()));
  btVoronoiSimplexSolver simplexSolver;
  btGjkConvexCast caster(&boxA, &boxB, &simplexSolver);

  btTransform fromA;
  fromA.setIdentity();
  fromA.setOrigin(btVector3(kSweepStart.x(), kSweepStart.y(), kSweepStart.z()));
  btTransform toA;
  toA.setIdentity();
  toA.setOrigin(btVector3(kSweepEnd.x(), kSweepEnd.y(), kSweepEnd.z()));
  btTransform fromB;
  fromB.setIdentity();
  const btTransform toB = fromB;

  btConvexCast::CastResult sanity;
  sanity.m_fraction = btScalar(1.0);
  if (!caster.calcTimeOfImpact(fromA, toA, fromB, toB, sanity)) {
    state.SkipWithError("Reference B convex cast setup did not hit.");
    return;
  }

  for (auto _ : state) {
    btConvexCast::CastResult result;
    result.m_fraction = btScalar(1.0);
    bool hit = caster.calcTimeOfImpact(fromA, toA, fromB, toB, result);
    btScalar toi = result.m_fraction;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_BoxBox_ReferenceB);
#endif

//==============================================================================
// Reference C: substepped discrete narrow phase (no native continuous query)
//==============================================================================

#if DART_HAVE_ODE
namespace {

void RunOdeSubstepCcd(
    benchmark::State& state,
    const std::shared_ptr<dart::dynamics::Shape>& movingShape,
    const std::shared_ptr<dart::dynamics::Shape>& staticShape,
    const Eigen::Isometry3d& startTf = MakeTranslation(kSweepStart),
    const Eigen::Isometry3d& endTf = MakeTranslation(kSweepEnd))
{
  constexpr int kSubsteps = 256;

  auto detector = dart::collision::OdeCollisionDetector::createReference();
  auto movingSkel = dart::benchmark::collision::CreateSingleShapeSkeleton(
      movingShape, startTf);
  auto staticSkel = dart::benchmark::collision::CreateSingleShapeSkeleton(
      staticShape, Eigen::Isometry3d::Identity());
  auto group = detector->createCollisionGroup();
  dart::benchmark::collision::AddSkeletonToGroup(group.get(), movingSkel);
  dart::benchmark::collision::AddSkeletonToGroup(group.get(), staticSkel);
  auto* joint
      = dynamic_cast<dart::dynamics::FreeJoint*>(movingSkel->getJoint(0));

  auto option = dart::benchmark::collision::MakeCollisionOption();

  const Eigen::Quaterniond qStart(startTf.rotation());
  const Eigen::Quaterniond qEnd(endTf.rotation());

  auto substepCast = [&]() -> double {
    dart::collision::CollisionResult result;
    for (int i = 0; i <= kSubsteps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(kSubsteps);
      Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
      tf.translation()
          = (1.0 - t) * startTf.translation() + t * endTf.translation();
      tf.linear() = qStart.slerp(t, qEnd).toRotationMatrix();
      joint->setTransform(tf);
      result.clear();
      if (detector->collide(group.get(), option, &result)
          && result.getNumContacts() > 0u) {
        return t;
      }
    }
    return -1.0;
  };

  if (substepCast() < 0.0) {
    state.SkipWithError("Reference C substepping setup did not hit.");
    return;
  }

  for (auto _ : state) {
    double toi = substepCast();
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}

} // namespace

static void BM_CCD_ConvexCast_BoxBox_ReferenceC(benchmark::State& state)
{
  RunOdeSubstepCcd(
      state,
      std::make_shared<dart::dynamics::BoxShape>(2.0 * kBoxHalf),
      std::make_shared<dart::dynamics::BoxShape>(2.0 * kBoxHalf));
}
BENCHMARK(BM_CCD_ConvexCast_BoxBox_ReferenceC);
#endif

//==============================================================================
// Sphere-sphere sweep: native closed-form cast vs. each engine's best CCD
//==============================================================================

static void BM_CCD_SphereCast_SphereSphere_Native(benchmark::State& state)
{
  SphereShape target(kBoxHalf.x());
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  CcdOption option = CcdOption::standard();
  CcdResult result;

  if (!sphereCastSphere(
          kSweepStart,
          kSweepEnd,
          kBoxHalf.x(),
          target,
          targetTf,
          option,
          result)) {
    state.SkipWithError("Native sphere cast setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    bool hit = sphereCastSphere(
        kSweepStart, kSweepEnd, kBoxHalf.x(), target, targetTf, option, result);
    double toi = result.timeOfImpact;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SphereCast_SphereSphere_Native);

#if DART_HAVE_FCL
static void BM_CCD_SphereCast_SphereSphere_ReferenceA(benchmark::State& state)
{
  auto sphereA = std::make_shared<fcl::Sphere<double>>(kBoxHalf.x());
  auto sphereB = std::make_shared<fcl::Sphere<double>>(kBoxHalf.x());

  const Eigen::Isometry3d tfAbeg = MakeTranslation(kSweepStart);
  const Eigen::Isometry3d tfAend = MakeTranslation(kSweepEnd);
  const Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();

  fcl::ContinuousCollisionRequest<double> request(
      30,
      1e-4,
      fcl::CCDM_LINEAR,
      fcl::GST_INDEP,
      fcl::CCDC_CONSERVATIVE_ADVANCEMENT);

  fcl::ContinuousCollisionResult<double> sanity;
  fcl::continuousCollide(
      sphereA.get(), tfAbeg, tfAend, sphereB.get(), tfB, tfB, request, sanity);
  if (!sanity.is_collide) {
    state.SkipWithError("Reference A sphere continuous collision did not hit.");
    return;
  }

  for (auto _ : state) {
    fcl::ContinuousCollisionResult<double> result;
    fcl::continuousCollide(
        sphereA.get(),
        tfAbeg,
        tfAend,
        sphereB.get(),
        tfB,
        tfB,
        request,
        result);
    double toc = result.time_of_contact;
    benchmark::DoNotOptimize(toc);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SphereCast_SphereSphere_ReferenceA);
#endif

#if DART_HAVE_BULLET
static void BM_CCD_SphereCast_SphereSphere_ReferenceB(benchmark::State& state)
{
  btSphereShape sphereA(kBoxHalf.x());
  btSphereShape sphereB(kBoxHalf.x());
  btVoronoiSimplexSolver simplexSolver;
  btGjkConvexCast caster(&sphereA, &sphereB, &simplexSolver);

  btTransform fromA;
  fromA.setIdentity();
  fromA.setOrigin(btVector3(kSweepStart.x(), kSweepStart.y(), kSweepStart.z()));
  btTransform toA;
  toA.setIdentity();
  toA.setOrigin(btVector3(kSweepEnd.x(), kSweepEnd.y(), kSweepEnd.z()));
  btTransform fromB;
  fromB.setIdentity();
  const btTransform toB = fromB;

  btConvexCast::CastResult sanity;
  sanity.m_fraction = btScalar(1.0);
  if (!caster.calcTimeOfImpact(fromA, toA, fromB, toB, sanity)) {
    state.SkipWithError("Reference B sphere convex cast did not hit.");
    return;
  }

  for (auto _ : state) {
    btConvexCast::CastResult result;
    result.m_fraction = btScalar(1.0);
    bool hit = caster.calcTimeOfImpact(fromA, toA, fromB, toB, result);
    btScalar toi = result.m_fraction;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SphereCast_SphereSphere_ReferenceB);
#endif

#if DART_HAVE_ODE
static void BM_CCD_SphereCast_SphereSphere_ReferenceC(benchmark::State& state)
{
  RunOdeSubstepCcd(
      state,
      std::make_shared<dart::dynamics::SphereShape>(kBoxHalf.x()),
      std::make_shared<dart::dynamics::SphereShape>(kBoxHalf.x()));
}
BENCHMARK(BM_CCD_SphereCast_SphereSphere_ReferenceC);
#endif

//==============================================================================
// Capsule-capsule sweep: native specialized cast vs. each engine's best CCD
//==============================================================================

namespace {
constexpr double kCapsuleRadius = 0.4;
constexpr double kCapsuleHeight = 1.0;
} // namespace

static void BM_CCD_CapsuleCast_CapsuleCapsule_Native(benchmark::State& state)
{
  CapsuleShape capsule(kCapsuleRadius, kCapsuleHeight);
  CapsuleShape target(kCapsuleRadius, kCapsuleHeight);

  const Eigen::Isometry3d startTf = MakeTranslation(kSweepStart);
  const Eigen::Isometry3d endTf = MakeTranslation(kSweepEnd);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  CcdOption option = CcdOption::standard();
  CcdResult result;

  if (!capsuleCastCapsule(
          startTf, endTf, capsule, target, targetTf, option, result)) {
    state.SkipWithError("Native capsule cast setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    bool hit = capsuleCastCapsule(
        startTf, endTf, capsule, target, targetTf, option, result);
    double toi = result.timeOfImpact;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_CapsuleCast_CapsuleCapsule_Native);

#if DART_HAVE_FCL
static void BM_CCD_CapsuleCast_CapsuleCapsule_ReferenceA(
    benchmark::State& state)
{
  auto capA
      = std::make_shared<fcl::Capsule<double>>(kCapsuleRadius, kCapsuleHeight);
  auto capB
      = std::make_shared<fcl::Capsule<double>>(kCapsuleRadius, kCapsuleHeight);

  const Eigen::Isometry3d tfAbeg = MakeTranslation(kSweepStart);
  const Eigen::Isometry3d tfAend = MakeTranslation(kSweepEnd);
  const Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();

  fcl::ContinuousCollisionRequest<double> request(
      30,
      1e-4,
      fcl::CCDM_LINEAR,
      fcl::GST_INDEP,
      fcl::CCDC_CONSERVATIVE_ADVANCEMENT);

  fcl::ContinuousCollisionResult<double> sanity;
  fcl::continuousCollide(
      capA.get(), tfAbeg, tfAend, capB.get(), tfB, tfB, request, sanity);
  if (!sanity.is_collide) {
    state.SkipWithError(
        "Reference A capsule continuous collision did not hit.");
    return;
  }

  for (auto _ : state) {
    fcl::ContinuousCollisionResult<double> result;
    fcl::continuousCollide(
        capA.get(), tfAbeg, tfAend, capB.get(), tfB, tfB, request, result);
    double toc = result.time_of_contact;
    benchmark::DoNotOptimize(toc);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_CapsuleCast_CapsuleCapsule_ReferenceA);
#endif

#if DART_HAVE_BULLET
static void BM_CCD_CapsuleCast_CapsuleCapsule_ReferenceB(
    benchmark::State& state)
{
  btCapsuleShapeZ capA(kCapsuleRadius, kCapsuleHeight);
  btCapsuleShapeZ capB(kCapsuleRadius, kCapsuleHeight);
  btVoronoiSimplexSolver simplexSolver;
  btGjkConvexCast caster(&capA, &capB, &simplexSolver);

  btTransform fromA;
  fromA.setIdentity();
  fromA.setOrigin(btVector3(kSweepStart.x(), kSweepStart.y(), kSweepStart.z()));
  btTransform toA;
  toA.setIdentity();
  toA.setOrigin(btVector3(kSweepEnd.x(), kSweepEnd.y(), kSweepEnd.z()));
  btTransform fromB;
  fromB.setIdentity();
  const btTransform toB = fromB;

  btConvexCast::CastResult sanity;
  sanity.m_fraction = btScalar(1.0);
  if (!caster.calcTimeOfImpact(fromA, toA, fromB, toB, sanity)) {
    state.SkipWithError("Reference B capsule convex cast did not hit.");
    return;
  }

  for (auto _ : state) {
    btConvexCast::CastResult result;
    result.m_fraction = btScalar(1.0);
    bool hit = caster.calcTimeOfImpact(fromA, toA, fromB, toB, result);
    btScalar toi = result.m_fraction;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_CapsuleCast_CapsuleCapsule_ReferenceB);
#endif

#if DART_HAVE_ODE
static void BM_CCD_CapsuleCast_CapsuleCapsule_ReferenceC(
    benchmark::State& state)
{
  RunOdeSubstepCcd(
      state,
      std::make_shared<dart::dynamics::CapsuleShape>(
          kCapsuleRadius, kCapsuleHeight),
      std::make_shared<dart::dynamics::CapsuleShape>(
          kCapsuleRadius, kCapsuleHeight));
}
BENCHMARK(BM_CCD_CapsuleCast_CapsuleCapsule_ReferenceC);
#endif

//==============================================================================
// Mixed pair (sphere vs. box): native closed-form slab cast vs. each engine
//==============================================================================

static void BM_CCD_SphereCast_SphereBox_Native(benchmark::State& state)
{
  BoxShape target(kBoxHalf);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  CcdOption option = CcdOption::standard();
  CcdResult result;

  if (!sphereCastBox(
          kSweepStart,
          kSweepEnd,
          kBoxHalf.x(),
          target,
          targetTf,
          option,
          result)) {
    state.SkipWithError("Native sphere-box cast setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    bool hit = sphereCastBox(
        kSweepStart, kSweepEnd, kBoxHalf.x(), target, targetTf, option, result);
    double toi = result.timeOfImpact;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SphereCast_SphereBox_Native);

#if DART_HAVE_FCL
static void BM_CCD_SphereCast_SphereBox_ReferenceA(benchmark::State& state)
{
  auto sphere = std::make_shared<fcl::Sphere<double>>(kBoxHalf.x());
  auto box = std::make_shared<fcl::Box<double>>(
      2.0 * kBoxHalf.x(), 2.0 * kBoxHalf.y(), 2.0 * kBoxHalf.z());

  const Eigen::Isometry3d tfAbeg = MakeTranslation(kSweepStart);
  const Eigen::Isometry3d tfAend = MakeTranslation(kSweepEnd);
  const Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();

  fcl::ContinuousCollisionRequest<double> request(
      30,
      1e-4,
      fcl::CCDM_LINEAR,
      fcl::GST_INDEP,
      fcl::CCDC_CONSERVATIVE_ADVANCEMENT);

  fcl::ContinuousCollisionResult<double> sanity;
  fcl::continuousCollide(
      sphere.get(), tfAbeg, tfAend, box.get(), tfB, tfB, request, sanity);
  if (!sanity.is_collide) {
    state.SkipWithError(
        "Reference A sphere-box continuous collision did not hit.");
    return;
  }

  for (auto _ : state) {
    fcl::ContinuousCollisionResult<double> result;
    fcl::continuousCollide(
        sphere.get(), tfAbeg, tfAend, box.get(), tfB, tfB, request, result);
    double toc = result.time_of_contact;
    benchmark::DoNotOptimize(toc);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SphereCast_SphereBox_ReferenceA);
#endif

#if DART_HAVE_BULLET
static void BM_CCD_SphereCast_SphereBox_ReferenceB(benchmark::State& state)
{
  btSphereShape sphere(kBoxHalf.x());
  btBoxShape box(btVector3(kBoxHalf.x(), kBoxHalf.y(), kBoxHalf.z()));
  btVoronoiSimplexSolver simplexSolver;
  btGjkConvexCast caster(&sphere, &box, &simplexSolver);

  btTransform fromA;
  fromA.setIdentity();
  fromA.setOrigin(btVector3(kSweepStart.x(), kSweepStart.y(), kSweepStart.z()));
  btTransform toA;
  toA.setIdentity();
  toA.setOrigin(btVector3(kSweepEnd.x(), kSweepEnd.y(), kSweepEnd.z()));
  btTransform fromB;
  fromB.setIdentity();
  const btTransform toB = fromB;

  btConvexCast::CastResult sanity;
  sanity.m_fraction = btScalar(1.0);
  if (!caster.calcTimeOfImpact(fromA, toA, fromB, toB, sanity)) {
    state.SkipWithError("Reference B sphere-box convex cast did not hit.");
    return;
  }

  for (auto _ : state) {
    btConvexCast::CastResult result;
    result.m_fraction = btScalar(1.0);
    bool hit = caster.calcTimeOfImpact(fromA, toA, fromB, toB, result);
    btScalar toi = result.m_fraction;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SphereCast_SphereBox_ReferenceB);
#endif

#if DART_HAVE_ODE
static void BM_CCD_SphereCast_SphereBox_ReferenceC(benchmark::State& state)
{
  RunOdeSubstepCcd(
      state,
      std::make_shared<dart::dynamics::SphereShape>(kBoxHalf.x()),
      std::make_shared<dart::dynamics::BoxShape>(2.0 * kBoxHalf));
}
BENCHMARK(BM_CCD_SphereCast_SphereBox_ReferenceC);
#endif

//==============================================================================
// Convex polytope (octahedron): native GJK conservative advancement vs. the
// reference convex-CCD engines on a non-box hull. ODE has no convex CCD; its
// substepping-loss pattern is established by the cases above.
//==============================================================================

namespace {
constexpr double kOctScale = 0.6;

std::vector<Eigen::Vector3d> OctahedronVertices()
{
  const double s = kOctScale;
  return {{s, 0, 0}, {-s, 0, 0}, {0, s, 0}, {0, -s, 0}, {0, 0, s}, {0, 0, -s}};
}
} // namespace

static void BM_CCD_ConvexCast_Octahedron_Native(benchmark::State& state)
{
  ConvexShape shapeA(OctahedronVertices());
  ConvexShape shapeB(OctahedronVertices());

  const Eigen::Isometry3d startTf = MakeTranslation(kSweepStart);
  const Eigen::Isometry3d endTf = MakeTranslation(kSweepEnd);
  const Eigen::Isometry3d staticTf = Eigen::Isometry3d::Identity();

  CcdOption option = CcdOption::standard();
  CcdResult result;

  if (!convexCast(
          shapeA, startTf, endTf, shapeB, staticTf, staticTf, option, result)) {
    state.SkipWithError("Native octahedron cast setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    bool hit = convexCast(
        shapeA, startTf, endTf, shapeB, staticTf, staticTf, option, result);
    double toi = result.timeOfImpact;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_Octahedron_Native);

#if DART_HAVE_FCL
namespace {
std::shared_ptr<fcl::Convex<double>> MakeFclOctahedron()
{
  auto vertices = std::make_shared<std::vector<fcl::Vector3<double>>>();
  for (const auto& v : OctahedronVertices()) {
    vertices->push_back(v);
  }
  // Flat face list: [vertexCount, i0, i1, i2] per triangular face (8 faces).
  auto faces = std::make_shared<std::vector<int>>(
      std::vector<int>{3, 0, 2, 4, 3, 2, 1, 4, 3, 1, 3, 4, 3, 3, 0, 4,
                       3, 2, 0, 5, 3, 1, 2, 5, 3, 3, 1, 5, 3, 0, 3, 5});
  return std::make_shared<fcl::Convex<double>>(vertices, 8, faces, false);
}
} // namespace

static void BM_CCD_ConvexCast_Octahedron_ReferenceA(benchmark::State& state)
{
  auto octA = MakeFclOctahedron();
  auto octB = MakeFclOctahedron();

  const Eigen::Isometry3d tfAbeg = MakeTranslation(kSweepStart);
  const Eigen::Isometry3d tfAend = MakeTranslation(kSweepEnd);
  const Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();

  fcl::ContinuousCollisionRequest<double> request(
      30,
      1e-4,
      fcl::CCDM_LINEAR,
      fcl::GST_INDEP,
      fcl::CCDC_CONSERVATIVE_ADVANCEMENT);

  fcl::ContinuousCollisionResult<double> sanity;
  fcl::continuousCollide(
      octA.get(), tfAbeg, tfAend, octB.get(), tfB, tfB, request, sanity);
  if (!sanity.is_collide) {
    state.SkipWithError(
        "Reference A octahedron continuous collision did not hit.");
    return;
  }

  for (auto _ : state) {
    fcl::ContinuousCollisionResult<double> result;
    fcl::continuousCollide(
        octA.get(), tfAbeg, tfAend, octB.get(), tfB, tfB, request, result);
    double toc = result.time_of_contact;
    benchmark::DoNotOptimize(toc);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_Octahedron_ReferenceA);
#endif

#if DART_HAVE_BULLET
static void BM_CCD_ConvexCast_Octahedron_ReferenceB(benchmark::State& state)
{
  btConvexHullShape hullA;
  btConvexHullShape hullB;
  for (const auto& v : OctahedronVertices()) {
    hullA.addPoint(btVector3(v.x(), v.y(), v.z()), false);
    hullB.addPoint(btVector3(v.x(), v.y(), v.z()), false);
  }
  hullA.recalcLocalAabb();
  hullB.recalcLocalAabb();
  btVoronoiSimplexSolver simplexSolver;
  btGjkConvexCast caster(&hullA, &hullB, &simplexSolver);

  btTransform fromA;
  fromA.setIdentity();
  fromA.setOrigin(btVector3(kSweepStart.x(), kSweepStart.y(), kSweepStart.z()));
  btTransform toA;
  toA.setIdentity();
  toA.setOrigin(btVector3(kSweepEnd.x(), kSweepEnd.y(), kSweepEnd.z()));
  btTransform fromB;
  fromB.setIdentity();
  const btTransform toB = fromB;

  btConvexCast::CastResult sanity;
  sanity.m_fraction = btScalar(1.0);
  if (!caster.calcTimeOfImpact(fromA, toA, fromB, toB, sanity)) {
    state.SkipWithError("Reference B octahedron convex cast did not hit.");
    return;
  }

  for (auto _ : state) {
    btConvexCast::CastResult result;
    result.m_fraction = btScalar(1.0);
    bool hit = caster.calcTimeOfImpact(fromA, toA, fromB, toB, result);
    btScalar toi = result.m_fraction;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_Octahedron_ReferenceB);
#endif

//==============================================================================
// Rotational motion: box translating AND rotating 90° about Z. Exercises the
// angular-motion path (native uses the slerp branch + angular motion bound; the
// reference engines use their interpolated/screw motion or angular convex
// cast).
//==============================================================================

namespace {

Eigen::Isometry3d RotatingStartTf()
{
  return MakeTranslation(Eigen::Vector3d(-3.0, 0.0, 0.0));
}

Eigen::Isometry3d RotatingEndTf()
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.rotate(
      Eigen::AngleAxisd(
          std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitZ()));
  return tf;
}

} // namespace

static void BM_CCD_ConvexCast_BoxBoxRotating_Native(benchmark::State& state)
{
  ConvexShape shapeA(BoxVertices(kBoxHalf));
  ConvexShape shapeB(BoxVertices(kBoxHalf));

  const Eigen::Isometry3d startTf = RotatingStartTf();
  const Eigen::Isometry3d endTf = RotatingEndTf();
  const Eigen::Isometry3d staticTf = Eigen::Isometry3d::Identity();

  CcdOption option = CcdOption::standard();
  CcdResult result;

  if (!convexCast(
          shapeA, startTf, endTf, shapeB, staticTf, staticTf, option, result)) {
    state.SkipWithError("Native rotating box cast setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    bool hit = convexCast(
        shapeA, startTf, endTf, shapeB, staticTf, staticTf, option, result);
    double toi = result.timeOfImpact;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_BoxBoxRotating_Native);

#if DART_HAVE_FCL
static void BM_CCD_ConvexCast_BoxBoxRotating_ReferenceA(benchmark::State& state)
{
  auto boxA = std::make_shared<fcl::Box<double>>(
      2.0 * kBoxHalf.x(), 2.0 * kBoxHalf.y(), 2.0 * kBoxHalf.z());
  auto boxB = std::make_shared<fcl::Box<double>>(
      2.0 * kBoxHalf.x(), 2.0 * kBoxHalf.y(), 2.0 * kBoxHalf.z());

  const Eigen::Isometry3d tfAbeg = RotatingStartTf();
  const Eigen::Isometry3d tfAend = RotatingEndTf();
  const Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();

  fcl::ContinuousCollisionRequest<double> request(
      30,
      1e-4,
      fcl::CCDM_LINEAR,
      fcl::GST_INDEP,
      fcl::CCDC_CONSERVATIVE_ADVANCEMENT);

  fcl::ContinuousCollisionResult<double> sanity;
  fcl::continuousCollide(
      boxA.get(), tfAbeg, tfAend, boxB.get(), tfB, tfB, request, sanity);
  if (!sanity.is_collide) {
    state.SkipWithError(
        "Reference A rotating continuous collision did not hit.");
    return;
  }

  for (auto _ : state) {
    fcl::ContinuousCollisionResult<double> result;
    fcl::continuousCollide(
        boxA.get(), tfAbeg, tfAend, boxB.get(), tfB, tfB, request, result);
    double toc = result.time_of_contact;
    benchmark::DoNotOptimize(toc);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_BoxBoxRotating_ReferenceA);
#endif

#if DART_HAVE_BULLET
namespace {
btTransform EigenToBt(const Eigen::Isometry3d& tf)
{
  btTransform out;
  out.setIdentity();
  const Eigen::Vector3d t = tf.translation();
  out.setOrigin(btVector3(t.x(), t.y(), t.z()));
  const Eigen::Matrix3d r = tf.rotation();
  out.setBasis(btMatrix3x3(
      r(0, 0),
      r(0, 1),
      r(0, 2),
      r(1, 0),
      r(1, 1),
      r(1, 2),
      r(2, 0),
      r(2, 1),
      r(2, 2)));
  return out;
}
} // namespace

static void BM_CCD_ConvexCast_BoxBoxRotating_ReferenceB(benchmark::State& state)
{
  btBoxShape boxA(btVector3(kBoxHalf.x(), kBoxHalf.y(), kBoxHalf.z()));
  btBoxShape boxB(btVector3(kBoxHalf.x(), kBoxHalf.y(), kBoxHalf.z()));
  btVoronoiSimplexSolver simplexSolver;
  btGjkEpaPenetrationDepthSolver penetrationSolver;
  // Angular-motion-aware continuous convex collision (the linear GJK convex
  // cast ignores rotation; this entry point accounts for it).
  btContinuousConvexCollision caster(
      &boxA, &boxB, &simplexSolver, &penetrationSolver);

  const btTransform fromA = EigenToBt(RotatingStartTf());
  const btTransform toA = EigenToBt(RotatingEndTf());
  btTransform fromB;
  fromB.setIdentity();
  const btTransform toB = fromB;

  btConvexCast::CastResult sanity;
  sanity.m_fraction = btScalar(1.0);
  if (!caster.calcTimeOfImpact(fromA, toA, fromB, toB, sanity)) {
    state.SkipWithError("Reference B rotating convex cast did not hit.");
    return;
  }

  for (auto _ : state) {
    btConvexCast::CastResult result;
    result.m_fraction = btScalar(1.0);
    bool hit = caster.calcTimeOfImpact(fromA, toA, fromB, toB, result);
    btScalar toi = result.m_fraction;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_BoxBoxRotating_ReferenceB);
#endif

#if DART_HAVE_ODE
static void BM_CCD_ConvexCast_BoxBoxRotating_ReferenceC(benchmark::State& state)
{
  RunOdeSubstepCcd(
      state,
      std::make_shared<dart::dynamics::BoxShape>(2.0 * kBoxHalf),
      std::make_shared<dart::dynamics::BoxShape>(2.0 * kBoxHalf),
      RotatingStartTf(),
      RotatingEndTf());
}
BENCHMARK(BM_CCD_ConvexCast_BoxBoxRotating_ReferenceC);
#endif

//==============================================================================
// Cylinder target: native exact swept-sphere-vs-cylinder cast vs. each engine
//==============================================================================

namespace {
constexpr double kCylinderRadius = 0.5;
constexpr double kCylinderHeight = 1.0;
} // namespace

static void BM_CCD_SphereCast_SphereCylinder_Native(benchmark::State& state)
{
  CylinderShape target(kCylinderRadius, kCylinderHeight);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  CcdOption option = CcdOption::standard();
  CcdResult result;

  if (!sphereCastCylinder(
          kSweepStart,
          kSweepEnd,
          kBoxHalf.x(),
          target,
          targetTf,
          option,
          result)) {
    state.SkipWithError("Native sphere-cylinder cast setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    bool hit = sphereCastCylinder(
        kSweepStart, kSweepEnd, kBoxHalf.x(), target, targetTf, option, result);
    double toi = result.timeOfImpact;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SphereCast_SphereCylinder_Native);

#if DART_HAVE_FCL
static void BM_CCD_SphereCast_SphereCylinder_ReferenceA(benchmark::State& state)
{
  auto sphere = std::make_shared<fcl::Sphere<double>>(kBoxHalf.x());
  auto cylinder = std::make_shared<fcl::Cylinder<double>>(
      kCylinderRadius, kCylinderHeight);

  const Eigen::Isometry3d tfAbeg = MakeTranslation(kSweepStart);
  const Eigen::Isometry3d tfAend = MakeTranslation(kSweepEnd);
  const Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();

  fcl::ContinuousCollisionRequest<double> request(
      30,
      1e-4,
      fcl::CCDM_LINEAR,
      fcl::GST_INDEP,
      fcl::CCDC_CONSERVATIVE_ADVANCEMENT);

  fcl::ContinuousCollisionResult<double> sanity;
  fcl::continuousCollide(
      sphere.get(), tfAbeg, tfAend, cylinder.get(), tfB, tfB, request, sanity);
  if (!sanity.is_collide) {
    state.SkipWithError(
        "Reference A sphere-cylinder continuous collision did not hit.");
    return;
  }

  for (auto _ : state) {
    fcl::ContinuousCollisionResult<double> result;
    fcl::continuousCollide(
        sphere.get(),
        tfAbeg,
        tfAend,
        cylinder.get(),
        tfB,
        tfB,
        request,
        result);
    double toc = result.time_of_contact;
    benchmark::DoNotOptimize(toc);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SphereCast_SphereCylinder_ReferenceA);
#endif

#if DART_HAVE_BULLET
static void BM_CCD_SphereCast_SphereCylinder_ReferenceB(benchmark::State& state)
{
  btSphereShape sphere(kBoxHalf.x());
  btCylinderShapeZ cylinder(
      btVector3(kCylinderRadius, kCylinderRadius, 0.5 * kCylinderHeight));
  btVoronoiSimplexSolver simplexSolver;
  btGjkConvexCast caster(&sphere, &cylinder, &simplexSolver);

  btTransform fromA;
  fromA.setIdentity();
  fromA.setOrigin(btVector3(kSweepStart.x(), kSweepStart.y(), kSweepStart.z()));
  btTransform toA;
  toA.setIdentity();
  toA.setOrigin(btVector3(kSweepEnd.x(), kSweepEnd.y(), kSweepEnd.z()));
  btTransform fromB;
  fromB.setIdentity();
  const btTransform toB = fromB;

  btConvexCast::CastResult sanity;
  sanity.m_fraction = btScalar(1.0);
  if (!caster.calcTimeOfImpact(fromA, toA, fromB, toB, sanity)) {
    state.SkipWithError("Reference B sphere-cylinder convex cast did not hit.");
    return;
  }

  for (auto _ : state) {
    btConvexCast::CastResult result;
    result.m_fraction = btScalar(1.0);
    bool hit = caster.calcTimeOfImpact(fromA, toA, fromB, toB, result);
    btScalar toi = result.m_fraction;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SphereCast_SphereCylinder_ReferenceB);
#endif

#if DART_HAVE_ODE
static void BM_CCD_SphereCast_SphereCylinder_ReferenceC(benchmark::State& state)
{
  RunOdeSubstepCcd(
      state,
      std::make_shared<dart::dynamics::SphereShape>(kBoxHalf.x()),
      std::make_shared<dart::dynamics::CylinderShape>(
          kCylinderRadius, kCylinderHeight));
}
BENCHMARK(BM_CCD_SphereCast_SphereCylinder_ReferenceC);
#endif

//==============================================================================
// Screw motion: box translating along Z while rotating about Z (helical sweep)
//==============================================================================

namespace {

Eigen::Isometry3d ScrewStartTf()
{
  return MakeTranslation(Eigen::Vector3d(0.0, 0.0, -2.0));
}

Eigen::Isometry3d ScrewEndTf()
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.rotate(
      Eigen::AngleAxisd(
          std::numbers::pi_v<double> / 2.0, Eigen::Vector3d::UnitZ()));
  return tf;
}

} // namespace

static void BM_CCD_ConvexCast_BoxBoxScrew_Native(benchmark::State& state)
{
  ConvexShape shapeA(BoxVertices(kBoxHalf));
  ConvexShape shapeB(BoxVertices(kBoxHalf));

  const Eigen::Isometry3d startTf = ScrewStartTf();
  const Eigen::Isometry3d endTf = ScrewEndTf();
  const Eigen::Isometry3d staticTf = Eigen::Isometry3d::Identity();

  CcdOption option = CcdOption::standard();
  CcdResult result;

  if (!convexCast(
          shapeA, startTf, endTf, shapeB, staticTf, staticTf, option, result)) {
    state.SkipWithError("Native screw cast setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    bool hit = convexCast(
        shapeA, startTf, endTf, shapeB, staticTf, staticTf, option, result);
    double toi = result.timeOfImpact;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_BoxBoxScrew_Native);

#if DART_HAVE_FCL
static void BM_CCD_ConvexCast_BoxBoxScrew_ReferenceA(benchmark::State& state)
{
  auto boxA = std::make_shared<fcl::Box<double>>(
      2.0 * kBoxHalf.x(), 2.0 * kBoxHalf.y(), 2.0 * kBoxHalf.z());
  auto boxB = std::make_shared<fcl::Box<double>>(
      2.0 * kBoxHalf.x(), 2.0 * kBoxHalf.y(), 2.0 * kBoxHalf.z());

  const Eigen::Isometry3d tfAbeg = ScrewStartTf();
  const Eigen::Isometry3d tfAend = ScrewEndTf();
  const Eigen::Isometry3d tfB = Eigen::Isometry3d::Identity();

  // CCDM_SCREW selects FCL's dedicated screw-motion interpolation.
  fcl::ContinuousCollisionRequest<double> request(
      30,
      1e-4,
      fcl::CCDM_SCREW,
      fcl::GST_INDEP,
      fcl::CCDC_CONSERVATIVE_ADVANCEMENT);

  fcl::ContinuousCollisionResult<double> sanity;
  fcl::continuousCollide(
      boxA.get(), tfAbeg, tfAend, boxB.get(), tfB, tfB, request, sanity);
  if (!sanity.is_collide) {
    state.SkipWithError("Reference A screw continuous collision did not hit.");
    return;
  }

  for (auto _ : state) {
    fcl::ContinuousCollisionResult<double> result;
    fcl::continuousCollide(
        boxA.get(), tfAbeg, tfAend, boxB.get(), tfB, tfB, request, result);
    double toc = result.time_of_contact;
    benchmark::DoNotOptimize(toc);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_BoxBoxScrew_ReferenceA);
#endif

#if DART_HAVE_BULLET
static void BM_CCD_ConvexCast_BoxBoxScrew_ReferenceB(benchmark::State& state)
{
  btBoxShape boxA(btVector3(kBoxHalf.x(), kBoxHalf.y(), kBoxHalf.z()));
  btBoxShape boxB(btVector3(kBoxHalf.x(), kBoxHalf.y(), kBoxHalf.z()));
  btVoronoiSimplexSolver simplexSolver;
  btGjkEpaPenetrationDepthSolver penetrationSolver;
  btContinuousConvexCollision caster(
      &boxA, &boxB, &simplexSolver, &penetrationSolver);

  const btTransform fromA = EigenToBt(ScrewStartTf());
  const btTransform toA = EigenToBt(ScrewEndTf());
  btTransform fromB;
  fromB.setIdentity();
  const btTransform toB = fromB;

  btConvexCast::CastResult sanity;
  sanity.m_fraction = btScalar(1.0);
  if (!caster.calcTimeOfImpact(fromA, toA, fromB, toB, sanity)) {
    state.SkipWithError("Reference B screw convex cast did not hit.");
    return;
  }

  for (auto _ : state) {
    btConvexCast::CastResult result;
    result.m_fraction = btScalar(1.0);
    bool hit = caster.calcTimeOfImpact(fromA, toA, fromB, toB, result);
    btScalar toi = result.m_fraction;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_BoxBoxScrew_ReferenceB);
#endif

#if DART_HAVE_ODE
static void BM_CCD_ConvexCast_BoxBoxScrew_ReferenceC(benchmark::State& state)
{
  RunOdeSubstepCcd(
      state,
      std::make_shared<dart::dynamics::BoxShape>(2.0 * kBoxHalf),
      std::make_shared<dart::dynamics::BoxShape>(2.0 * kBoxHalf),
      ScrewStartTf(),
      ScrewEndTf());
}
BENCHMARK(BM_CCD_ConvexCast_BoxBoxScrew_ReferenceC);
#endif

//==============================================================================
// Spline motion: box A follows a cubic-Bezier curve that bulges away from the
// chord between its endpoints, colliding with a static box the chord never
// reaches. Only a polynomial (spline) motion model can represent this path --
// the linear and screw casts of References B/C cannot, so a straight cast over
// the same endpoints misses the collision entirely. Reference A is the one
// engine with a real spline-motion model; it is driven here with matched cubic
// control points (its public continuous-collide entry point builds a degenerate
// spline from endpoints, so the explicit-motion entry point is used instead).
//==============================================================================

namespace {

// Cubic-Bezier translation control points; the curve dips to (0, -1, 0) at its
// midpoint while the chord stays at y = 2.
const std::array<Eigen::Vector3d, 4> kSplineTranslation
    = {Eigen::Vector3d(-2.0, 2.0, 0.0),
       Eigen::Vector3d(-2.0, -2.0, 0.0),
       Eigen::Vector3d(2.0, -2.0, 0.0),
       Eigen::Vector3d(2.0, 2.0, 0.0)};
const std::array<Eigen::Vector3d, 4> kSplineRotation
    = {Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero(),
       Eigen::Vector3d::Zero()};
const Eigen::Vector3d kSplineTargetTranslation(0.0, -1.0, 0.0);

Eigen::Vector3d evalSplineTranslation(double t)
{
  const double u = 1.0 - t;
  const double b0 = u * u * u;
  const double b1 = 3.0 * t * u * u;
  const double b2 = 3.0 * t * t * u;
  const double b3 = t * t * t;
  return b0 * kSplineTranslation[0] + b1 * kSplineTranslation[1]
         + b2 * kSplineTranslation[2] + b3 * kSplineTranslation[3];
}

} // namespace

static void BM_CCD_SplineCast_BoxBox_Native(benchmark::State& state)
{
  ConvexShape shapeA(BoxVertices(kBoxHalf));
  ConvexShape shapeB(BoxVertices(kBoxHalf));
  const Eigen::Isometry3d targetTf = MakeTranslation(kSplineTargetTranslation);

  CcdOption option = CcdOption::standard();
  CcdResult result;

  if (!splineCast(
          shapeA,
          kSplineTranslation,
          kSplineRotation,
          shapeB,
          targetTf,
          option,
          result)) {
    state.SkipWithError("Native spline cast setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    bool hit = splineCast(
        shapeA,
        kSplineTranslation,
        kSplineRotation,
        shapeB,
        targetTf,
        option,
        result);
    double toi = result.timeOfImpact;
    benchmark::DoNotOptimize(hit);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SplineCast_BoxBox_Native);

#if DART_HAVE_FCL
namespace {

// The reference engine's spline motion evaluates its four control points in the
// uniform cubic B-spline basis, whereas the native cast uses the Bezier
// (Bernstein) basis. To make both engines trace the *identical* curve, the
// Bezier control points are converted to the equivalent B-spline control points
// (a basis change between two parameterizations of the same cubic polynomial).
std::array<Eigen::Vector3d, 4> BezierToBSpline(
    const std::array<Eigen::Vector3d, 4>& p)
{
  const Eigen::Vector3d c0 = p[0];
  const Eigen::Vector3d c1 = 3.0 * (p[1] - p[0]);
  const Eigen::Vector3d c2 = 3.0 * (p[0] - 2.0 * p[1] + p[2]);
  const Eigen::Vector3d c3 = p[3] - 3.0 * p[2] + 3.0 * p[1] - p[0];
  return {
      c0 - c1 + (2.0 / 3.0) * c2,
      c0 - (1.0 / 3.0) * c2,
      c0 + c1 + (2.0 / 3.0) * c2,
      c0 + 2.0 * c1 + (11.0 / 3.0) * c2 + 6.0 * c3};
}

} // namespace

static void BM_CCD_SplineCast_BoxBox_ReferenceA(benchmark::State& state)
{
  auto boxA = std::make_shared<fcl::Box<double>>(
      2.0 * kBoxHalf.x(), 2.0 * kBoxHalf.y(), 2.0 * kBoxHalf.z());
  auto boxB = std::make_shared<fcl::Box<double>>(
      2.0 * kBoxHalf.x(), 2.0 * kBoxHalf.y(), 2.0 * kBoxHalf.z());

  // Convert the native cast's Bezier control points to the B-spline control
  // points that make the reference engine trace the exact same curve.
  const std::array<Eigen::Vector3d, 4> td = BezierToBSpline(kSplineTranslation);
  const std::array<Eigen::Vector3d, 4> rd = BezierToBSpline(kSplineRotation);
  auto motionA = std::make_shared<fcl::SplineMotion<double>>(
      td[0], td[1], td[2], td[3], rd[0], rd[1], rd[2], rd[3]);
  const Eigen::Isometry3d targetTf = MakeTranslation(kSplineTargetTranslation);
  auto motionB
      = std::make_shared<fcl::TranslationMotion<double>>(targetTf, targetTf);

  fcl::ContinuousCollisionRequest<double> request(
      30,
      1e-4,
      fcl::CCDM_SPLINE,
      fcl::GST_INDEP,
      fcl::CCDC_CONSERVATIVE_ADVANCEMENT);

  fcl::ContinuousCollisionResult<double> sanity;
  fcl::continuousCollide(
      boxA.get(), motionA.get(), boxB.get(), motionB.get(), request, sanity);
  if (!sanity.is_collide) {
    state.SkipWithError("Reference A spline continuous collision did not hit.");
    return;
  }

  for (auto _ : state) {
    fcl::ContinuousCollisionResult<double> result;
    fcl::continuousCollide(
        boxA.get(), motionA.get(), boxB.get(), motionB.get(), request, result);
    double toc = result.time_of_contact;
    benchmark::DoNotOptimize(toc);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SplineCast_BoxBox_ReferenceA);
#endif

#if DART_HAVE_ODE
static void BM_CCD_SplineCast_BoxBox_ReferenceC(benchmark::State& state)
{
  constexpr int kSubsteps = 256;

  auto detector = dart::collision::OdeCollisionDetector::createReference();
  auto movingSkel = dart::benchmark::collision::CreateSingleShapeSkeleton(
      std::make_shared<dart::dynamics::BoxShape>(2.0 * kBoxHalf),
      MakeTranslation(evalSplineTranslation(0.0)));
  auto staticSkel = dart::benchmark::collision::CreateSingleShapeSkeleton(
      std::make_shared<dart::dynamics::BoxShape>(2.0 * kBoxHalf),
      MakeTranslation(kSplineTargetTranslation));
  auto group = detector->createCollisionGroup();
  dart::benchmark::collision::AddSkeletonToGroup(group.get(), movingSkel);
  dart::benchmark::collision::AddSkeletonToGroup(group.get(), staticSkel);
  auto* joint
      = dynamic_cast<dart::dynamics::FreeJoint*>(movingSkel->getJoint(0));

  auto option = dart::benchmark::collision::MakeCollisionOption();

  auto substepCast = [&]() -> double {
    dart::collision::CollisionResult result;
    for (int i = 0; i <= kSubsteps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(kSubsteps);
      joint->setTransform(MakeTranslation(evalSplineTranslation(t)));
      result.clear();
      if (detector->collide(group.get(), option, &result)
          && result.getNumContacts() > 0u) {
        return t;
      }
    }
    return -1.0;
  };

  if (substepCast() < 0.0) {
    state.SkipWithError("Reference C spline substepping setup did not hit.");
    return;
  }

  for (auto _ : state) {
    double toi = substepCast();
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_SplineCast_BoxBox_ReferenceC);
#endif

BENCHMARK_MAIN();
