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

#include <dart/collision/native/narrow_phase/ccd.hpp>
#include <dart/collision/native/narrow_phase/gjk.hpp>
#include <dart/collision/native/narrow_phase/primitive_ccd.hpp>
#include <dart/collision/native/shapes/shape.hpp>

#include <benchmark/benchmark.h>

#include <vector>

using namespace dart::collision::native;

namespace {

constexpr double kSphereRadius = 0.5;
constexpr double kCapsuleRadius = 0.4;
constexpr double kCapsuleHeight = 2.0;
const Eigen::Vector3d kHalfExtents(0.5, 0.5, 0.5);

Eigen::Isometry3d MakeTransform(const Eigen::Vector3d& translation)
{
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  return tf;
}

std::vector<Eigen::Vector3d> MakeBoxVertices(const Eigen::Vector3d& half)
{
  return {
      {-half.x(), -half.y(), -half.z()},
      {half.x(), -half.y(), -half.z()},
      {half.x(), half.y(), -half.z()},
      {-half.x(), half.y(), -half.z()},
      {-half.x(), -half.y(), half.z()},
      {half.x(), -half.y(), half.z()},
      {half.x(), half.y(), half.z()},
      {-half.x(), half.y(), half.z()},
  };
}

std::vector<Eigen::Vector3i> MakeBoxTriangles()
{
  return {
      {0, 1, 2},
      {0, 2, 3}, // -Z
      {4, 6, 5},
      {4, 7, 6}, // +Z
      {3, 2, 6},
      {3, 6, 7}, // +Y
      {0, 5, 1},
      {0, 4, 5}, // -Y
      {0, 3, 7},
      {0, 7, 4}, // -X
      {1, 5, 6},
      {1, 6, 2}, // +X
  };
}

bool requireCcdHit(
    benchmark::State& state,
    bool hit,
    const CcdResult& result,
    const char* message)
{
  if (!hit || !result.isHit()) {
    state.SkipWithError(message);
    return false;
  }

  return true;
}

void consumeCcdResult(bool hit, const CcdResult& result)
{
  bool resultHit = result.hit;
  double timeOfImpact = result.timeOfImpact;
  double pointX = result.point.x();
  double normalX = result.normal.x();
  auto* object = result.object;
  benchmark::DoNotOptimize(hit);
  benchmark::DoNotOptimize(resultHit);
  benchmark::DoNotOptimize(timeOfImpact);
  benchmark::DoNotOptimize(pointX);
  benchmark::DoNotOptimize(normalX);
  benchmark::DoNotOptimize(object);
  benchmark::ClobberMemory();
}

} // namespace

static void BM_CCD_SphereCast_Sphere(benchmark::State& state)
{
  SphereShape target(1.0);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d start(-5.0, 0.0, 0.0);
  const Eigen::Vector3d end(5.0, 0.0, 0.0);

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          sphereCastSphere(
              start, end, kSphereRadius, target, targetTf, option, sanity),
          sanity,
          "Sphere-sphere CCD benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = sphereCastSphere(
        start, end, kSphereRadius, target, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_SphereCast_Sphere);

static void BM_CCD_SphereCast_Box(benchmark::State& state)
{
  BoxShape target(kHalfExtents);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d start(-5.0, 0.0, 0.0);
  const Eigen::Vector3d end(5.0, 0.0, 0.0);

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          sphereCastBox(
              start, end, kSphereRadius, target, targetTf, option, sanity),
          sanity,
          "Sphere-box CCD benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = sphereCastBox(
        start, end, kSphereRadius, target, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_SphereCast_Box);

static void BM_CCD_SphereCast_Plane(benchmark::State& state)
{
  PlaneShape target(Eigen::Vector3d::UnitZ(), 0.0);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d start(0.0, 0.0, 5.0);
  const Eigen::Vector3d end(0.0, 0.0, -5.0);

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          sphereCastPlane(
              start, end, kSphereRadius, target, targetTf, option, sanity),
          sanity,
          "Sphere-plane CCD benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = sphereCastPlane(
        start, end, kSphereRadius, target, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_SphereCast_Plane);

static void BM_CCD_SphereCast_Cylinder(benchmark::State& state)
{
  CylinderShape target(1.0, 2.0);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d start(-5.0, 0.0, 0.0);
  const Eigen::Vector3d end(5.0, 0.0, 0.0);

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          sphereCastCylinder(
              start, end, kSphereRadius, target, targetTf, option, sanity),
          sanity,
          "Sphere-cylinder CCD benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = sphereCastCylinder(
        start, end, kSphereRadius, target, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_SphereCast_Cylinder);

static void BM_CCD_SphereCast_Convex(benchmark::State& state)
{
  ConvexShape target(MakeBoxVertices(kHalfExtents));
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d start(-5.0, 0.0, 0.0);
  const Eigen::Vector3d end(5.0, 0.0, 0.0);

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          sphereCastConvex(
              start, end, kSphereRadius, target, targetTf, option, sanity),
          sanity,
          "Sphere-convex CCD benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = sphereCastConvex(
        start, end, kSphereRadius, target, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_SphereCast_Convex);

static void BM_CCD_SphereCast_Mesh(benchmark::State& state)
{
  MeshShape target(MakeBoxVertices(kHalfExtents), MakeBoxTriangles());
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d start(-5.0, 0.0, 0.0);
  const Eigen::Vector3d end(5.0, 0.0, 0.0);

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          sphereCastMesh(
              start, end, kSphereRadius, target, targetTf, option, sanity),
          sanity,
          "Sphere-mesh CCD benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = sphereCastMesh(
        start, end, kSphereRadius, target, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_SphereCast_Mesh);

static void BM_CCD_CapsuleCast_Capsule(benchmark::State& state)
{
  CapsuleShape capsule(kCapsuleRadius, kCapsuleHeight);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Isometry3d startTf
      = MakeTransform(Eigen::Vector3d(-5.0, 0.0, 0.0));
  const Eigen::Isometry3d endTf = MakeTransform(Eigen::Vector3d(5.0, 0.0, 0.0));

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          capsuleCastCapsule(
              startTf, endTf, capsule, capsule, targetTf, option, sanity),
          sanity,
          "Capsule-capsule CCD benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = capsuleCastCapsule(
        startTf, endTf, capsule, capsule, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_CapsuleCast_Capsule);

static void BM_CCD_CapsuleCast_Box(benchmark::State& state)
{
  CapsuleShape capsule(kCapsuleRadius, kCapsuleHeight);
  BoxShape target(kHalfExtents);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Isometry3d startTf
      = MakeTransform(Eigen::Vector3d(0.0, 0.0, -5.0));
  const Eigen::Isometry3d endTf = MakeTransform(Eigen::Vector3d(0.0, 0.0, 5.0));

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          capsuleCastBox(
              startTf, endTf, capsule, target, targetTf, option, sanity),
          sanity,
          "Capsule-box CCD benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = capsuleCastBox(
        startTf, endTf, capsule, target, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_CapsuleCast_Box);

static void BM_CCD_CapsuleCast_Convex(benchmark::State& state)
{
  CapsuleShape capsule(kCapsuleRadius, kCapsuleHeight);
  ConvexShape target(MakeBoxVertices(kHalfExtents));
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Isometry3d startTf
      = MakeTransform(Eigen::Vector3d(-5.0, 0.0, 0.0));
  const Eigen::Isometry3d endTf = MakeTransform(Eigen::Vector3d(5.0, 0.0, 0.0));

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          capsuleCastConvex(
              startTf, endTf, capsule, target, targetTf, option, sanity),
          sanity,
          "Capsule-convex CCD benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = capsuleCastConvex(
        startTf, endTf, capsule, target, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_CapsuleCast_Convex);

static void BM_CCD_CapsuleCast_Mesh(benchmark::State& state)
{
  CapsuleShape capsule(kCapsuleRadius, kCapsuleHeight);
  MeshShape target(MakeBoxVertices(kHalfExtents), MakeBoxTriangles());
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Isometry3d startTf
      = MakeTransform(Eigen::Vector3d(-5.0, 0.0, 0.0));
  const Eigen::Isometry3d endTf = MakeTransform(Eigen::Vector3d(5.0, 0.0, 0.0));

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          capsuleCastMesh(
              startTf, endTf, capsule, target, targetTf, option, sanity),
          sanity,
          "Capsule-mesh CCD benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = capsuleCastMesh(
        startTf, endTf, capsule, target, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_CapsuleCast_Mesh);

static void BM_CCD_ConservativeAdvancement_Convex(benchmark::State& state)
{
  ConvexShape shapeA(MakeBoxVertices(kHalfExtents));
  ConvexShape shapeB(MakeBoxVertices(kHalfExtents));

  const Eigen::Isometry3d startTf
      = MakeTransform(Eigen::Vector3d(-5.0, 0.0, 0.0));
  const Eigen::Isometry3d endTf = MakeTransform(Eigen::Vector3d(5.0, 0.0, 0.0));
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  CcdOption option = CcdOption::standard();
  CcdResult result;

  CcdResult sanity;
  if (!requireCcdHit(
          state,
          conservativeAdvancement(
              shapeA, startTf, endTf, shapeB, targetTf, option, sanity),
          sanity,
          "Conservative advancement benchmark setup did not hit.")) {
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit = conservativeAdvancement(
        shapeA, startTf, endTf, shapeB, targetTf, option, result);
    consumeCcdResult(hit, result);
  }
}
BENCHMARK(BM_CCD_ConservativeAdvancement_Convex);

//==============================================================================
// Primitive CCD (IPC-class: point-triangle, edge-edge)
//==============================================================================

namespace {

void consumePrimitiveResult(bool hit, const CcdPrimitiveResult& result)
{
  bool resultHit = result.hit;
  double toi = result.timeOfImpact;
  benchmark::DoNotOptimize(hit);
  benchmark::DoNotOptimize(resultHit);
  benchmark::DoNotOptimize(toi);
  benchmark::ClobberMemory();
}

} // namespace

static void BM_CCD_PointTriangle_ACCD(benchmark::State& state)
{
  const Eigen::Vector3d a(-1, -1, 0);
  const Eigen::Vector3d b(1, -1, 0);
  const Eigen::Vector3d c(0, 1, 0);
  const Eigen::Vector3d pStart(0, 0, 1);
  const Eigen::Vector3d pEnd(0, 0, -1);

  CcdOption option = CcdOption::standard();
  CcdPrimitiveResult result;

  CcdPrimitiveResult sanity;
  if (!pointTriangleCcd(pStart, pEnd, a, a, b, b, c, c, option, sanity)
      || !sanity.isHit()) {
    state.SkipWithError("Point-triangle CCD benchmark setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit
        = pointTriangleCcd(pStart, pEnd, a, a, b, b, c, c, option, result);
    consumePrimitiveResult(hit, result);
  }
}
BENCHMARK(BM_CCD_PointTriangle_ACCD);

static void BM_CCD_EdgeEdge_ACCD(benchmark::State& state)
{
  const Eigen::Vector3d a0(-1, 0, 0.5);
  const Eigen::Vector3d a1(-1, 0, -0.5);
  const Eigen::Vector3d b0(1, 0, 0.5);
  const Eigen::Vector3d b1(1, 0, -0.5);
  const Eigen::Vector3d c0(0, -1, 0);
  const Eigen::Vector3d d0(0, 1, 0);

  CcdOption option = CcdOption::standard();
  CcdPrimitiveResult result;

  CcdPrimitiveResult sanity;
  if (!edgeEdgeCcd(a0, a1, b0, b1, c0, c0, d0, d0, option, sanity)
      || !sanity.isHit()) {
    state.SkipWithError("Edge-edge CCD benchmark setup did not hit.");
    return;
  }

  for (auto _ : state) {
    result.clear();
    const bool hit
        = edgeEdgeCcd(a0, a1, b0, b1, c0, c0, d0, d0, option, result);
    consumePrimitiveResult(hit, result);
  }
}
BENCHMARK(BM_CCD_EdgeEdge_ACCD);

//==============================================================================
// Algorithmic comparison: conservative advancement vs. uniform substepping
//
// A continuous-collision engine that lacks a dedicated convex cast must sample
// the trajectory at K substeps and run a discrete intersection test at each
// (the naive baseline). Both paths below use the SAME GJK kernel, so the
// comparison isolates the algorithmic win of conservative advancement, which
// converges in a handful of distance evaluations regardless of trajectory
// length while remaining more accurate than any fixed substep resolution.
//==============================================================================

namespace {

Eigen::Isometry3d InterpolateTransform(
    const Eigen::Isometry3d& start, const Eigen::Isometry3d& end, double t)
{
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = (1.0 - t) * start.translation() + t * end.translation();
  const Eigen::Quaterniond qStart(start.rotation());
  const Eigen::Quaterniond qEnd(end.rotation());
  out.linear() = qStart.slerp(t, qEnd).toRotationMatrix();
  return out;
}

double SubsteppingConvexCast(
    const ConvexShape& shapeA,
    const Eigen::Isometry3d& tfAStart,
    const Eigen::Isometry3d& tfAEnd,
    const ConvexShape& shapeB,
    const Eigen::Isometry3d& tfB,
    int substeps)
{
  auto supportB = [&](const Eigen::Vector3d& dir) {
    return tfB * shapeB.support(tfB.rotation().transpose() * dir);
  };
  for (int i = 0; i <= substeps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(substeps);
    const Eigen::Isometry3d tfA = InterpolateTransform(tfAStart, tfAEnd, t);
    auto supportA = [&](const Eigen::Vector3d& dir) {
      return tfA * shapeA.support(tfA.rotation().transpose() * dir);
    };
    if (Gjk::intersect(supportA, supportB)) {
      return t;
    }
  }
  return -1.0;
}

} // namespace

static void BM_CCD_ConvexCast_Substepping(benchmark::State& state)
{
  const int substeps = static_cast<int>(state.range(0));

  ConvexShape shapeA(MakeBoxVertices(kHalfExtents));
  ConvexShape shapeB(MakeBoxVertices(kHalfExtents));

  const Eigen::Isometry3d startTf
      = MakeTransform(Eigen::Vector3d(-5.0, 0.0, 0.0));
  const Eigen::Isometry3d endTf = MakeTransform(Eigen::Vector3d(5.0, 0.0, 0.0));
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  if (SubsteppingConvexCast(shapeA, startTf, endTf, shapeB, targetTf, substeps)
      < 0.0) {
    state.SkipWithError("Substepping CCD baseline did not hit.");
    return;
  }

  for (auto _ : state) {
    double toi = SubsteppingConvexCast(
        shapeA, startTf, endTf, shapeB, targetTf, substeps);
    benchmark::DoNotOptimize(toi);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_CCD_ConvexCast_Substepping)->Arg(256)->Arg(1024);

BENCHMARK_MAIN();
