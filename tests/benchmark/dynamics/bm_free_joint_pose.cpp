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

#include <dart/math/geometry.hpp>

#include <Eigen/Geometry>
#include <benchmark/benchmark.h>

#include <algorithm>
#include <vector>

#include <cmath>

namespace {

struct PoseSample
{
  Eigen::Isometry3d pose;
  Eigen::Vector6d spatialVelocityInChild;
};

Eigen::Isometry3d makePose(int index)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

  const double t = static_cast<double>(index);
  pose.linear() = dart::math::expMapRot(
      Eigen::Vector3d(
          0.17 * std::sin(0.31 * t) + 0.04 * t,
          -0.23 * std::cos(0.19 * t),
          0.11 * std::sin(0.13 * t)));
  pose.translation() = Eigen::Vector3d(
      0.35 * std::sin(0.07 * t),
      -0.27 * std::cos(0.11 * t),
      0.19 * std::sin(0.17 * t));

  return pose;
}

Eigen::Vector6d makeSpatialVelocityInChild(int index)
{
  const double t = static_cast<double>(index);
  Eigen::Vector6d velocity;
  velocity << 1.0 + 0.15 * std::sin(0.37 * t), -0.7 + 0.08 * std::cos(0.23 * t),
      0.5 + 0.11 * std::sin(0.41 * t), 1.4 + 0.2 * std::cos(0.29 * t),
      -0.9 + 0.17 * std::sin(0.31 * t), 0.6 + 0.13 * std::cos(0.17 * t);
  return velocity;
}

const std::vector<PoseSample>& samples()
{
  static const std::vector<PoseSample> data = [] {
    std::vector<PoseSample> result;
    result.reserve(256);
    for (int i = 0; i < 256; ++i) {
      result.push_back({makePose(i), makeSpatialVelocityInChild(i)});
    }
    return result;
  }();

  return data;
}

double dtFromState(const benchmark::State& state)
{
  return static_cast<double>(state.range(0)) * 1e-6;
}

Eigen::Isometry3d stepWorldTangentIsometry(
    const Eigen::Isometry3d& pose,
    const Eigen::Vector6d& spatialVelocityInChild,
    double dt)
{
  // Matches FreeJoint's current contract: generalized velocities are
  // world-frame components, and the relative spatial velocity is expressed in
  // the child frame by the joint Jacobian before the pose update.
  Eigen::Isometry3d next = pose;
  const Eigen::Matrix3d rotation = pose.linear();
  next.linear()
      = rotation * dart::math::expMapRot(spatialVelocityInChild.head<3>() * dt);
  next.translation() += rotation * spatialVelocityInChild.tail<3>() * dt;
  return next;
}

Eigen::Isometry3d stepBodyTwistIsometry(
    const Eigen::Isometry3d& pose,
    const Eigen::Vector6d& spatialVelocityInChild,
    double dt)
{
  return pose * dart::math::expMap(spatialVelocityInChild * dt);
}

Eigen::Matrix4d stepBodyTwistMatrix4(
    const Eigen::Isometry3d& pose,
    const Eigen::Vector6d& spatialVelocityInChild,
    double dt)
{
  return pose.matrix()
         * dart::math::expMap(spatialVelocityInChild * dt).matrix();
}

Eigen::Isometry3d stepBodyTwistQuaternion(
    const Eigen::Isometry3d& pose,
    const Eigen::Vector6d& spatialVelocityInChild,
    double dt)
{
  const Eigen::Isometry3d delta
      = dart::math::expMap(spatialVelocityInChild * dt);
  Eigen::Quaterniond rotation(pose.linear());
  rotation = rotation * Eigen::Quaterniond(delta.linear());
  rotation.normalize();

  Eigen::Isometry3d next = Eigen::Isometry3d::Identity();
  next.linear() = rotation.toRotationMatrix();
  next.translation() = pose.translation() + pose.linear() * delta.translation();
  return next;
}

Eigen::Isometry3d stepWorldTangentQuaternion(
    const Eigen::Isometry3d& pose,
    const Eigen::Vector6d& spatialVelocityInChild,
    double dt)
{
  Eigen::Quaterniond rotation(pose.linear());
  rotation
      = rotation * dart::math::expToQuat(spatialVelocityInChild.head<3>() * dt);
  rotation.normalize();

  Eigen::Isometry3d next = Eigen::Isometry3d::Identity();
  next.linear() = rotation.toRotationMatrix();
  next.translation() = pose.translation()
                       + pose.linear() * spatialVelocityInChild.tail<3>() * dt;
  return next;
}

struct ErrorSummary
{
  double maxWorldTranslationError = 0.0;
  double meanWorldTranslationError = 0.0;
  double maxBodyTranslationError = 0.0;
  double meanBodyTranslationError = 0.0;
  double maxRotationError = 0.0;
};

template <typename StepFn>
ErrorSummary summarizeError(double dt, StepFn step)
{
  ErrorSummary summary;

  for (const PoseSample& sample : samples()) {
    const Eigen::Isometry3d expectedWorld = stepWorldTangentIsometry(
        sample.pose, sample.spatialVelocityInChild, dt);
    const Eigen::Isometry3d expectedBody
        = stepBodyTwistIsometry(sample.pose, sample.spatialVelocityInChild, dt);
    const Eigen::Isometry3d actual
        = step(sample.pose, sample.spatialVelocityInChild, dt);

    const double worldTranslationError
        = (actual.translation() - expectedWorld.translation()).norm();
    const double bodyTranslationError
        = (actual.translation() - expectedBody.translation()).norm();
    const double rotationError
        = dart::math::logMap(
              expectedBody.linear().transpose() * actual.linear())
              .norm();

    summary.maxWorldTranslationError
        = std::max(summary.maxWorldTranslationError, worldTranslationError);
    summary.meanWorldTranslationError += worldTranslationError;
    summary.maxBodyTranslationError
        = std::max(summary.maxBodyTranslationError, bodyTranslationError);
    summary.meanBodyTranslationError += bodyTranslationError;
    summary.maxRotationError
        = std::max(summary.maxRotationError, rotationError);
  }

  summary.meanWorldTranslationError
      /= static_cast<double>(std::max<std::size_t>(1, samples().size()));
  summary.meanBodyTranslationError
      /= static_cast<double>(std::max<std::size_t>(1, samples().size()));
  return summary;
}

void addCounters(benchmark::State& state, const ErrorSummary& summary)
{
  state.counters["max_world_trans_err_m"] = summary.maxWorldTranslationError;
  state.counters["mean_world_trans_err_m"] = summary.meanWorldTranslationError;
  state.counters["max_body_trans_err_m"] = summary.maxBodyTranslationError;
  state.counters["mean_body_trans_err_m"] = summary.meanBodyTranslationError;
  state.counters["max_rot_err_rad"] = summary.maxRotationError;
}

template <typename StepFn>
void runIsometryBenchmark(benchmark::State& state, StepFn step)
{
  const double dt = dtFromState(state);

  for (auto _ : state) {
    for (const PoseSample& sample : samples()) {
      const Eigen::Isometry3d next
          = step(sample.pose, sample.spatialVelocityInChild, dt);
      benchmark::DoNotOptimize(next.matrix().data());
    }
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(samples().size()));
}

} // namespace

static void BM_FreeJointPose_WorldTangentIsometry(benchmark::State& state)
{
  runIsometryBenchmark(state, stepWorldTangentIsometry);
  state.PauseTiming();
  addCounters(
      state,
      summarizeError(
          dtFromState(state),
          [](const Eigen::Isometry3d& pose,
             const Eigen::Vector6d& spatialVelocityInChild,
             double dt) {
            return stepWorldTangentIsometry(pose, spatialVelocityInChild, dt);
          }));
  state.ResumeTiming();
}

static void BM_FreeJointPose_BodyTwistIsometry(benchmark::State& state)
{
  runIsometryBenchmark(state, stepBodyTwistIsometry);
  state.PauseTiming();
  addCounters(
      state,
      summarizeError(
          dtFromState(state),
          [](const Eigen::Isometry3d& pose,
             const Eigen::Vector6d& spatialVelocityInChild,
             double dt) {
            return stepBodyTwistIsometry(pose, spatialVelocityInChild, dt);
          }));
  state.ResumeTiming();
}

static void BM_FreeJointPose_BodyTwistMatrix4(benchmark::State& state)
{
  const double dt = dtFromState(state);

  for (auto _ : state) {
    for (const PoseSample& sample : samples()) {
      const Eigen::Matrix4d next = stepBodyTwistMatrix4(
          sample.pose, sample.spatialVelocityInChild, dt);
      benchmark::DoNotOptimize(next.data());
    }
  }

  state.SetItemsProcessed(
      state.iterations() * static_cast<int64_t>(samples().size()));
  addCounters(
      state,
      summarizeError(
          dt,
          [](const Eigen::Isometry3d& pose,
             const Eigen::Vector6d& spatialVelocityInChild,
             double dt) {
            Eigen::Isometry3d next = Eigen::Isometry3d::Identity();
            next.matrix()
                = stepBodyTwistMatrix4(pose, spatialVelocityInChild, dt);
            return next;
          }));
}

static void BM_FreeJointPose_BodyTwistQuaternion(benchmark::State& state)
{
  runIsometryBenchmark(state, stepBodyTwistQuaternion);
  addCounters(
      state,
      summarizeError(
          dtFromState(state),
          [](const Eigen::Isometry3d& pose,
             const Eigen::Vector6d& spatialVelocityInChild,
             double dt) {
            return stepBodyTwistQuaternion(pose, spatialVelocityInChild, dt);
          }));
}

static void BM_FreeJointPose_WorldTangentQuaternion(benchmark::State& state)
{
  runIsometryBenchmark(state, stepWorldTangentQuaternion);
  addCounters(
      state,
      summarizeError(
          dtFromState(state),
          [](const Eigen::Isometry3d& pose,
             const Eigen::Vector6d& spatialVelocityInChild,
             double dt) {
            return stepWorldTangentQuaternion(pose, spatialVelocityInChild, dt);
          }));
}

BENCHMARK(BM_FreeJointPose_WorldTangentIsometry)
    ->Arg(1000)
    ->Arg(10000)
    ->Arg(50000);
BENCHMARK(BM_FreeJointPose_BodyTwistIsometry)
    ->Arg(1000)
    ->Arg(10000)
    ->Arg(50000);
BENCHMARK(BM_FreeJointPose_BodyTwistMatrix4)->Arg(1000)->Arg(10000)->Arg(50000);
BENCHMARK(BM_FreeJointPose_BodyTwistQuaternion)
    ->Arg(1000)
    ->Arg(10000)
    ->Arg(50000);
BENCHMARK(BM_FreeJointPose_WorldTangentQuaternion)
    ->Arg(1000)
    ->Arg(10000)
    ->Arg(50000);

BENCHMARK_MAIN();
