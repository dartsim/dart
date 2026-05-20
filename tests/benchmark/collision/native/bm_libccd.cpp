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

#include <dart/collision/native/narrow_phase/gjk.hpp>
#include <dart/collision/native/narrow_phase/mpr.hpp>

#include <benchmark/benchmark.h>
#include <ccd/ccd.h>

using namespace dart::collision::native;

namespace {

struct LibccdSphere
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 1.0;
};

struct LibccdBox
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d halfExtents = Eigen::Vector3d::Ones();
};

void toCcdVec3(ccd_vec3_t* out, const Eigen::Vector3d& v)
{
  ccdVec3Set(out, v.x(), v.y(), v.z());
}

void supportSphere(const void* obj, const ccd_vec3_t* dir, ccd_vec3_t* vec)
{
  const auto* sphere = static_cast<const LibccdSphere*>(obj);
  Eigen::Vector3d d(dir->v[0], dir->v[1], dir->v[2]);
  if (d.squaredNorm() < 1e-12) {
    d = Eigen::Vector3d::UnitX();
  } else {
    d.normalize();
  }
  const Eigen::Vector3d point = sphere->center + sphere->radius * d;
  toCcdVec3(vec, point);
}

void supportBox(const void* obj, const ccd_vec3_t* dir, ccd_vec3_t* vec)
{
  const auto* box = static_cast<const LibccdBox*>(obj);
  Eigen::Vector3d point = box->center;
  point.x()
      += (dir->v[0] >= 0.0) ? box->halfExtents.x() : -box->halfExtents.x();
  point.y()
      += (dir->v[1] >= 0.0) ? box->halfExtents.y() : -box->halfExtents.y();
  point.z()
      += (dir->v[2] >= 0.0) ? box->halfExtents.z() : -box->halfExtents.z();
  toCcdVec3(vec, point);
}

void centerSphere(const void* obj, ccd_vec3_t* center)
{
  const auto* sphere = static_cast<const LibccdSphere*>(obj);
  toCcdVec3(center, sphere->center);
}

void centerBox(const void* obj, ccd_vec3_t* center)
{
  const auto* box = static_cast<const LibccdBox*>(obj);
  toCcdVec3(center, box->center);
}

SupportFunction makeSphereSupport(const Eigen::Vector3d& center, double radius)
{
  return [center, radius](const Eigen::Vector3d& dir) -> Eigen::Vector3d {
    if (dir.squaredNorm() < 1e-12) {
      return center + Eigen::Vector3d(radius, 0, 0);
    }
    return center + radius * dir.normalized();
  };
}

SupportFunction makeBoxSupport(
    const Eigen::Vector3d& center, const Eigen::Vector3d& halfExtents)
{
  return [center, halfExtents](const Eigen::Vector3d& dir) {
    Eigen::Vector3d result = center;
    result.x() += (dir.x() >= 0) ? halfExtents.x() : -halfExtents.x();
    result.y() += (dir.y() >= 0) ? halfExtents.y() : -halfExtents.y();
    result.z() += (dir.z() >= 0) ? halfExtents.z() : -halfExtents.z();
    return result;
  };
}

void configureCcd(
    ccd_t& ccd,
    ccd_support_fn support1,
    ccd_support_fn support2,
    ccd_center_fn center1,
    ccd_center_fn center2)
{
  CCD_INIT(&ccd);
  ccd.support1 = support1;
  ccd.support2 = support2;
  ccd.center1 = center1;
  ccd.center2 = center2;
  ccd.max_iterations = 64;
  ccd.epa_tolerance = 1e-6;
  ccd.mpr_tolerance = 1e-6;
}

void consumeIntersectResult(bool result)
{
  benchmark::DoNotOptimize(result);
  benchmark::ClobberMemory();
}

void consumeGjkEpaResult(const GjkResult& gjk, const EpaResult& epa)
{
  bool intersecting = gjk.intersecting;
  int simplexSize = gjk.simplex.size;
  bool epaSuccess = epa.success;
  double depth = epa.depth;
  benchmark::DoNotOptimize(intersecting);
  benchmark::DoNotOptimize(simplexSize);
  benchmark::DoNotOptimize(epaSuccess);
  benchmark::DoNotOptimize(depth);
  benchmark::ClobberMemory();
}

void consumeMprResult(const MprResult& mpr)
{
  bool success = mpr.success;
  double depth = mpr.depth;
  double normalX = mpr.normal.x();
  benchmark::DoNotOptimize(success);
  benchmark::DoNotOptimize(depth);
  benchmark::DoNotOptimize(normalX);
  benchmark::ClobberMemory();
}

void consumeCcdPenetrationResult(
    int status, double depth, const ccd_vec3_t& dir, const ccd_vec3_t& pos)
{
  double dirX = dir.v[0];
  double posX = pos.v[0];
  benchmark::DoNotOptimize(status);
  benchmark::DoNotOptimize(depth);
  benchmark::DoNotOptimize(dirX);
  benchmark::DoNotOptimize(posX);
  benchmark::ClobberMemory();
}

} // namespace

static void BM_Dart_GjkIntersect_SphereSphere(benchmark::State& state)
{
  const Eigen::Vector3d centerA(0.0, 0.0, 0.0);
  const Eigen::Vector3d centerB(1.5, 0.0, 0.0);
  const double radius = 1.0;

  auto supportA = makeSphereSupport(centerA, radius);
  auto supportB = makeSphereSupport(centerB, radius);

  if (!Gjk::intersect(supportA, supportB, centerB - centerA)) {
    state.SkipWithError("DART GJK sphere-sphere setup did not intersect.");
    return;
  }

  for (auto _ : state) {
    const bool result = Gjk::intersect(supportA, supportB, centerB - centerA);
    consumeIntersectResult(result);
  }
}
BENCHMARK(BM_Dart_GjkIntersect_SphereSphere);

static void BM_Libccd_GjkIntersect_SphereSphere(benchmark::State& state)
{
  LibccdSphere sphereA{Eigen::Vector3d(0.0, 0.0, 0.0), 1.0};
  LibccdSphere sphereB{Eigen::Vector3d(1.5, 0.0, 0.0), 1.0};

  ccd_t ccd;
  configureCcd(ccd, supportSphere, supportSphere, centerSphere, centerSphere);

  if (ccdGJKIntersect(&sphereA, &sphereB, &ccd) != 1) {
    state.SkipWithError("libccd GJK sphere-sphere setup did not intersect.");
    return;
  }

  for (auto _ : state) {
    int result = ccdGJKIntersect(&sphereA, &sphereB, &ccd);
    benchmark::DoNotOptimize(result);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_Libccd_GjkIntersect_SphereSphere);

static void BM_Dart_GjkEpa_SphereSphere(benchmark::State& state)
{
  const Eigen::Vector3d centerA(0.0, 0.0, 0.0);
  const Eigen::Vector3d centerB(1.5, 0.0, 0.0);
  const double radius = 1.0;

  auto supportA = makeSphereSupport(centerA, radius);
  auto supportB = makeSphereSupport(centerB, radius);

  const GjkResult sanityGjk = Gjk::query(supportA, supportB, centerB - centerA);
  if (!sanityGjk.intersecting) {
    state.SkipWithError("DART GJK/EPA sphere-sphere setup did not intersect.");
    return;
  }

  for (auto _ : state) {
    GjkResult gjk = Gjk::query(supportA, supportB, centerB - centerA);
    EpaResult epa = Epa::penetration(supportA, supportB, gjk.simplex);
    consumeGjkEpaResult(gjk, epa);
  }
}
BENCHMARK(BM_Dart_GjkEpa_SphereSphere);

static void BM_Libccd_GjkEpa_SphereSphere(benchmark::State& state)
{
  LibccdSphere sphereA{Eigen::Vector3d(0.0, 0.0, 0.0), 1.0};
  LibccdSphere sphereB{Eigen::Vector3d(1.5, 0.0, 0.0), 1.0};

  ccd_t ccd;
  configureCcd(ccd, supportSphere, supportSphere, centerSphere, centerSphere);

  double sanityDepth = 0.0;
  ccd_vec3_t sanityDir;
  ccd_vec3_t sanityPos;
  if (ccdGJKPenetration(
          &sphereA, &sphereB, &ccd, &sanityDepth, &sanityDir, &sanityPos)
      != 0) {
    state.SkipWithError("libccd GJK/EPA sphere-sphere setup did not collide.");
    return;
  }

  for (auto _ : state) {
    double depth = 0.0;
    ccd_vec3_t dir;
    ccd_vec3_t pos;
    const int status
        = ccdGJKPenetration(&sphereA, &sphereB, &ccd, &depth, &dir, &pos);
    consumeCcdPenetrationResult(status, depth, dir, pos);
  }
}
BENCHMARK(BM_Libccd_GjkEpa_SphereSphere);

static void BM_Dart_Mpr_SphereSphere(benchmark::State& state)
{
  const Eigen::Vector3d centerA(0.0, 0.0, 0.0);
  const Eigen::Vector3d centerB(1.5, 0.0, 0.0);
  const double radius = 1.0;

  auto supportA = makeSphereSupport(centerA, radius);
  auto supportB = makeSphereSupport(centerB, radius);

  if (!Mpr::penetration(supportA, supportB, centerA, centerB).success) {
    state.SkipWithError("DART MPR sphere-sphere setup did not collide.");
    return;
  }

  for (auto _ : state) {
    MprResult mpr = Mpr::penetration(supportA, supportB, centerA, centerB);
    consumeMprResult(mpr);
  }
}
BENCHMARK(BM_Dart_Mpr_SphereSphere);

static void BM_Libccd_Mpr_SphereSphere(benchmark::State& state)
{
  LibccdSphere sphereA{Eigen::Vector3d(0.0, 0.0, 0.0), 1.0};
  LibccdSphere sphereB{Eigen::Vector3d(1.5, 0.0, 0.0), 1.0};

  ccd_t ccd;
  configureCcd(ccd, supportSphere, supportSphere, centerSphere, centerSphere);

  double sanityDepth = 0.0;
  ccd_vec3_t sanityDir;
  ccd_vec3_t sanityPos;
  if (ccdMPRPenetration(
          &sphereA, &sphereB, &ccd, &sanityDepth, &sanityDir, &sanityPos)
      != 0) {
    state.SkipWithError("libccd MPR sphere-sphere setup did not collide.");
    return;
  }

  for (auto _ : state) {
    double depth = 0.0;
    ccd_vec3_t dir;
    ccd_vec3_t pos;
    const int status
        = ccdMPRPenetration(&sphereA, &sphereB, &ccd, &depth, &dir, &pos);
    consumeCcdPenetrationResult(status, depth, dir, pos);
  }
}
BENCHMARK(BM_Libccd_Mpr_SphereSphere);

static void BM_Dart_GjkIntersect_BoxBox(benchmark::State& state)
{
  const Eigen::Vector3d centerA(0.0, 0.0, 0.0);
  const Eigen::Vector3d centerB(1.2, 0.5, 0.0);
  const Eigen::Vector3d halfExtents(1.0, 1.0, 1.0);

  auto supportA = makeBoxSupport(centerA, halfExtents);
  auto supportB = makeBoxSupport(centerB, halfExtents);

  if (!Gjk::intersect(supportA, supportB, centerB - centerA)) {
    state.SkipWithError("DART GJK box-box setup did not intersect.");
    return;
  }

  for (auto _ : state) {
    const bool result = Gjk::intersect(supportA, supportB, centerB - centerA);
    consumeIntersectResult(result);
  }
}
BENCHMARK(BM_Dart_GjkIntersect_BoxBox);

static void BM_Libccd_GjkIntersect_BoxBox(benchmark::State& state)
{
  LibccdBox boxA{
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0)};
  LibccdBox boxB{
      Eigen::Vector3d(1.2, 0.5, 0.0), Eigen::Vector3d(1.0, 1.0, 1.0)};

  ccd_t ccd;
  configureCcd(ccd, supportBox, supportBox, centerBox, centerBox);

  if (ccdGJKIntersect(&boxA, &boxB, &ccd) != 1) {
    state.SkipWithError("libccd GJK box-box setup did not intersect.");
    return;
  }

  for (auto _ : state) {
    int result = ccdGJKIntersect(&boxA, &boxB, &ccd);
    benchmark::DoNotOptimize(result);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(BM_Libccd_GjkIntersect_BoxBox);

BENCHMARK_MAIN();
