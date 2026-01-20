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

#include <dart/collision/experimental/narrow_phase/ccd.hpp>
#include <dart/collision/experimental/shapes/shape.hpp>

#include <benchmark/benchmark.h>

#include <vector>

using namespace dart::collision::experimental;

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

} // namespace

static void BM_CCD_SphereCast_Sphere(benchmark::State& state)
{
  SphereShape target(1.0);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Vector3d start(-5.0, 0.0, 0.0);
  const Eigen::Vector3d end(5.0, 0.0, 0.0);

  CcdOption option = CcdOption::standard();
  CcdResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(sphereCastSphere(
        start, end, kSphereRadius, target, targetTf, option, result));
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

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(sphereCastBox(
        start, end, kSphereRadius, target, targetTf, option, result));
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

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(sphereCastPlane(
        start, end, kSphereRadius, target, targetTf, option, result));
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

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(sphereCastCylinder(
        start, end, kSphereRadius, target, targetTf, option, result));
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

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(sphereCastConvex(
        start, end, kSphereRadius, target, targetTf, option, result));
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

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(sphereCastMesh(
        start, end, kSphereRadius, target, targetTf, option, result));
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

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(capsuleCastCapsule(
        startTf, endTf, capsule, capsule, targetTf, option, result));
  }
}
BENCHMARK(BM_CCD_CapsuleCast_Capsule);

static void BM_CCD_CapsuleCast_Box(benchmark::State& state)
{
  CapsuleShape capsule(kCapsuleRadius, kCapsuleHeight);
  BoxShape target(kHalfExtents);
  const Eigen::Isometry3d targetTf = Eigen::Isometry3d::Identity();

  const Eigen::Isometry3d startTf
      = MakeTransform(Eigen::Vector3d(-5.0, 0.0, 0.0));
  const Eigen::Isometry3d endTf = MakeTransform(Eigen::Vector3d(5.0, 0.0, 0.0));

  CcdOption option = CcdOption::standard();
  CcdResult result;

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(capsuleCastBox(
        startTf, endTf, capsule, target, targetTf, option, result));
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

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(capsuleCastConvex(
        startTf, endTf, capsule, target, targetTf, option, result));
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

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(capsuleCastMesh(
        startTf, endTf, capsule, target, targetTf, option, result));
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

  for (auto _ : state) {
    result.clear();
    benchmark::DoNotOptimize(conservativeAdvancement(
        shapeA, startTf, endTf, shapeB, targetTf, option, result));
  }
}
BENCHMARK(BM_CCD_ConservativeAdvancement_Convex);

BENCHMARK_MAIN();
