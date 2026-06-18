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

#include "tests/common/lcpsolver/lcp_test_harness.hpp"

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/contact.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/comps/multibody.hpp>
#include <dart/simulation/compute/multibody_dynamics.hpp>
#include <dart/simulation/compute/unified_constraint.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/rigid_contact/boxed_lcp_contact.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/link.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>
#include <dart/simulation/world_options.hpp>

#include <Eigen/Dense>
#include <dart/simulation/compute/cuda/lcp_jacobi_batch_cuda.cuh>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <cmath>
#include <cstddef>

namespace cuda = dart::simulation::compute::cuda;
namespace compute = dart::simulation::compute;
namespace sx = dart::simulation;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

double makeDenseBoxGroundHalfExtent(int boxCount)
{
  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(boxCount))));
  const int rows = (boxCount + columns - 1) / columns;
  constexpr double kSpacing = 2.0;
  return std::max(
      20.0, kSpacing * static_cast<double>(std::max(columns, rows) - 1) + 1.0);
}

struct WorldContactProblem
{
  dart::math::LcpProblem problem;
  std::size_t contactCount{0};
};

struct WorldContactBatch
{
  cuda::LcpBatchCudaProblem packet;
  std::vector<dart::math::LcpProblem> problems;
};

struct WorldContactGroupedBatch
{
  std::vector<cuda::LcpBatchCudaProblem> packets;
  std::vector<std::vector<dart::math::LcpProblem>> problemsByGroup;
};

enum class SyntheticCudaFamily
{
  Standard,
  Boxed,
  FrictionIndex
};

const char* getSyntheticCudaFamilyName(const SyntheticCudaFamily family)
{
  switch (family) {
    case SyntheticCudaFamily::Standard:
      return "Standard";
    case SyntheticCudaFamily::Boxed:
      return "Boxed";
    case SyntheticCudaFamily::FrictionIndex:
      return "FrictionIndex";
  }

  return "Unknown";
}

enum class ArticulatedCudaContactCase
{
  Ground,
  RigidImpact,
  CrossLinkImpact
};

struct ThreeAxisPrismaticCudaRobot
{
  entt::entity multibody = entt::null;
  entt::entity link = entt::null;
};

void appendProblem(
    cuda::LcpBatchCudaProblem& packet,
    const Eigen::MatrixXd& A,
    const Eigen::VectorXd& b,
    const Eigen::VectorXd& lo,
    const Eigen::VectorXd& hi,
    const Eigen::VectorXi& findex,
    const Eigen::VectorXd& expected,
    std::vector<double>& expectedBatch)
{
  ASSERT_EQ(A.rows(), packet.problemSize);
  ASSERT_EQ(A.cols(), packet.problemSize);
  ASSERT_EQ(b.size(), packet.problemSize);
  ASSERT_EQ(lo.size(), packet.problemSize);
  ASSERT_EQ(hi.size(), packet.problemSize);
  ASSERT_EQ(findex.size(), packet.problemSize);
  ASSERT_EQ(expected.size(), packet.problemSize);

  for (Eigen::Index r = 0; r < A.rows(); ++r) {
    for (Eigen::Index c = 0; c < A.cols(); ++c) {
      packet.A.push_back(A(r, c));
    }
  }

  for (Eigen::Index i = 0; i < b.size(); ++i) {
    packet.b.push_back(b[i]);
    packet.lo.push_back(lo[i]);
    packet.hi.push_back(hi[i]);
    packet.findex.push_back(findex[i]);
    packet.x.push_back(0.0);
    expectedBatch.push_back(expected[i]);
  }
  ++packet.problemCount;
}

void appendLcpProblem(
    cuda::LcpBatchCudaProblem& packet, const dart::math::LcpProblem& problem)
{
  ASSERT_EQ(problem.A.rows(), packet.problemSize);
  ASSERT_EQ(problem.A.cols(), packet.problemSize);
  ASSERT_EQ(problem.b.size(), packet.problemSize);
  ASSERT_EQ(problem.lo.size(), packet.problemSize);
  ASSERT_EQ(problem.hi.size(), packet.problemSize);
  ASSERT_EQ(problem.findex.size(), packet.problemSize);

  for (Eigen::Index r = 0; r < problem.A.rows(); ++r) {
    for (Eigen::Index c = 0; c < problem.A.cols(); ++c) {
      packet.A.push_back(problem.A(r, c));
    }
  }

  for (Eigen::Index i = 0; i < problem.b.size(); ++i) {
    packet.b.push_back(problem.b[i]);
    packet.lo.push_back(problem.lo[i]);
    packet.hi.push_back(problem.hi[i]);
    packet.findex.push_back(problem.findex[i]);
    packet.x.push_back(0.0);
  }
  ++packet.problemCount;
}

cuda::LcpBatchCudaProblem makeStandardBatch(std::vector<double>& expected)
{
  cuda::LcpBatchCudaProblem packet;
  packet.problemSize = 4;
  packet.iterations = 256;

  Eigen::Matrix4d A0;
  A0 << 5.0, 0.5, 0.3, 0.2, 0.5, 5.0, 0.4, 0.3, 0.3, 0.4, 5.0, 0.5, 0.2, 0.3,
      0.5, 5.0;
  const Eigen::Vector4d x0(0.3, 0.2, 0.15, 0.1);
  appendProblem(
      packet,
      A0,
      A0 * x0,
      Eigen::Vector4d::Zero(),
      Eigen::Vector4d::Constant(kInf),
      Eigen::Vector4i::Constant(-1),
      x0,
      expected);

  Eigen::Matrix4d A1;
  A1 << 6.0, -0.25, 0.2, 0.1, -0.25, 5.5, 0.3, 0.2, 0.2, 0.3, 6.5, -0.15, 0.1,
      0.2, -0.15, 5.25;
  const Eigen::Vector4d x1(0.1, 0.4, 0.25, 0.35);
  appendProblem(
      packet,
      A1,
      A1 * x1,
      Eigen::Vector4d::Zero(),
      Eigen::Vector4d::Constant(kInf),
      Eigen::Vector4i::Constant(-1),
      x1,
      expected);

  return packet;
}

cuda::LcpBatchCudaProblem makeBoxedBatch(std::vector<double>& expected)
{
  cuda::LcpBatchCudaProblem packet;
  packet.problemSize = 4;
  packet.iterations = 256;

  Eigen::Matrix4d A0;
  A0 << 3.0, 0.2, 0.1, 0.0, 0.2, 2.8, 0.0, 0.1, 0.1, 0.0, 3.2, 0.2, 0.0, 0.1,
      0.2, 2.6;
  const Eigen::Vector4d lo0(-0.2, 0.0, -0.5, 0.0);
  const Eigen::Vector4d hi0(0.5, 0.4, 0.5, kInf);
  const Eigen::Vector4d x0(0.5, 0.2, -0.5, 0.3);
  const Eigen::Vector4d w0(-0.2, 0.0, 0.3, 0.0);
  appendProblem(
      packet,
      A0,
      A0 * x0 - w0,
      lo0,
      hi0,
      Eigen::Vector4i::Constant(-1),
      x0,
      expected);

  Eigen::Matrix4d A1;
  A1 << 4.0, -0.1, 0.2, 0.0, -0.1, 3.5, 0.1, 0.2, 0.2, 0.1, 3.75, -0.1, 0.0,
      0.2, -0.1, 3.25;
  const Eigen::Vector4d lo1(0.0, -0.3, -0.2, -kInf);
  const Eigen::Vector4d hi1(0.6, 0.6, 0.25, 0.4);
  const Eigen::Vector4d x1(0.15, -0.3, 0.25, 0.4);
  const Eigen::Vector4d w1(0.0, 0.4, -0.25, -0.2);
  appendProblem(
      packet,
      A1,
      A1 * x1 - w1,
      lo1,
      hi1,
      Eigen::Vector4i::Constant(-1),
      x1,
      expected);

  return packet;
}

cuda::LcpBatchCudaProblem makeFrictionIndexBatch(std::vector<double>& expected)
{
  cuda::LcpBatchCudaProblem packet;
  packet.problemSize = 6;
  packet.iterations = 256;

  Eigen::Matrix<double, 6, 6> A0;
  A0.setZero();
  A0.diagonal() << 4.0, 3.0, 2.5, 5.0, 3.5, 3.0;
  const Eigen::Matrix<double, 6, 1> x0(1.0, 0.2, -0.1, 0.75, -0.15, 0.1);
  Eigen::Matrix<double, 6, 1> lo0;
  lo0 << 0.0, -0.5, -0.5, 0.0, -0.4, -0.4;
  Eigen::Matrix<double, 6, 1> hi0;
  hi0 << kInf, 0.5, 0.5, kInf, 0.4, 0.4;
  Eigen::Matrix<int, 6, 1> findex0;
  findex0 << -1, 0, 0, -1, 3, 3;
  appendProblem(packet, A0, A0 * x0, lo0, hi0, findex0, x0, expected);

  Eigen::Matrix<double, 6, 6> A1;
  A1.setZero();
  A1.diagonal() << 6.0, 4.0, 3.5, 4.5, 3.25, 2.75;
  const Eigen::Matrix<double, 6, 1> x1(0.6, -0.2, 0.15, 1.2, 0.25, -0.35);
  Eigen::Matrix<double, 6, 1> lo1;
  lo1 << 0.0, -0.6, -0.6, 0.0, -0.3, -0.3;
  Eigen::Matrix<double, 6, 1> hi1;
  hi1 << kInf, 0.6, 0.6, kInf, 0.3, 0.3;
  Eigen::Matrix<int, 6, 1> findex1;
  findex1 << -1, 0, 0, -1, 3, 3;
  appendProblem(packet, A1, A1 * x1, lo1, hi1, findex1, x1, expected);

  return packet;
}

double signedPatternValue(const int row, const int col, const int variant)
{
  const int value = 37 * (row + 1) + 17 * (col + 3) + 11 * (variant + 5);
  const double scaled = static_cast<double>(value % 23) / 11.0 - 1.0;
  return scaled == 0.0 ? 0.25 : scaled;
}

Eigen::MatrixXd makeDiagonalDominantMatrix(const int n, const int variant)
{
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
  for (int row = 0; row < n; ++row) {
    for (int col = row + 1; col < n; ++col) {
      const double value = 0.02 * signedPatternValue(row, col, variant)
                           / static_cast<double>(std::max(1, n - 1));
      A(row, col) = value;
      A(col, row) = value;
    }
  }

  for (int row = 0; row < n; ++row) {
    A(row, row) = 3.0 + 0.25 * static_cast<double>((row + variant) % 7)
                  + A.row(row).cwiseAbs().sum();
  }

  return A;
}

dart::math::LcpProblem makeSyntheticStandardProblem(
    const int n, const int variant)
{
  Eigen::MatrixXd A = makeDiagonalDominantMatrix(n, variant);
  Eigen::VectorXd x(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    if ((i + variant) % 4 == 0) {
      x[i] = 0.0;
      w[i] = 0.2 + 0.01 * static_cast<double>(i + 1);
    } else {
      x[i] = 0.05 * static_cast<double>((i % 5) + 1)
             + 0.005 * static_cast<double>(variant);
    }
  }

  Eigen::VectorXd b = A * x - w;
  return dart::math::LcpProblem(
      std::move(A),
      std::move(b),
      Eigen::VectorXd::Zero(n),
      Eigen::VectorXd::Constant(n, kInf),
      Eigen::VectorXi::Constant(n, -1));
}

dart::math::LcpProblem makeSyntheticBoxedProblem(const int n, const int variant)
{
  Eigen::MatrixXd A = makeDiagonalDominantMatrix(n, 100 + variant);
  Eigen::VectorXd lo(n);
  Eigen::VectorXd hi(n);
  Eigen::VectorXd x(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  for (int i = 0; i < n; ++i) {
    lo[i] = -0.5 - 0.02 * static_cast<double>(i % 3);
    hi[i] = 0.45 + 0.03 * static_cast<double>((i + variant) % 4);
    switch ((i + variant) % 4) {
      case 0:
        x[i] = lo[i];
        w[i] = 0.15 + 0.01 * static_cast<double>(i);
        break;
      case 1:
        x[i] = hi[i];
        w[i] = -0.12 - 0.01 * static_cast<double>(i);
        break;
      case 2:
        x[i] = 0.5 * (lo[i] + hi[i]);
        break;
      default:
        hi[i] = kInf;
        x[i] = 0.1 + 0.02 * static_cast<double>(i % 5);
        break;
    }
  }

  Eigen::VectorXd b = A * x - w;
  return dart::math::LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      Eigen::VectorXi::Constant(n, -1));
}

dart::math::LcpProblem makeSyntheticFrictionIndexProblem(
    const int contactCount, const int variant)
{
  const int n = 3 * contactCount;
  Eigen::MatrixXd A = makeDiagonalDominantMatrix(n, 200 + variant);
  Eigen::VectorXd lo = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd hi = Eigen::VectorXd::Zero(n);
  Eigen::VectorXi findex = Eigen::VectorXi::Constant(n, -1);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd w = Eigen::VectorXd::Zero(n);

  for (int contact = 0; contact < contactCount; ++contact) {
    const int base = 3 * contact;
    const double normal = 0.4 + 0.08 * static_cast<double>(contact + 1)
                          + 0.01 * static_cast<double>(variant);
    const double mu = 0.25 + 0.05 * static_cast<double>((contact % 3) + 1);

    lo[base] = 0.0;
    hi[base] = kInf;
    x[base] = normal;

    lo[base + 1] = -mu;
    hi[base + 1] = mu;
    findex[base + 1] = base;

    lo[base + 2] = -mu;
    hi[base + 2] = mu;
    findex[base + 2] = base;

    if ((contact + variant) % 3 == 0) {
      x[base + 1] = 0.2 * mu * normal;
      x[base + 2] = -0.15 * mu * normal;
    } else if ((contact + variant) % 3 == 1) {
      x[base + 1] = mu * normal;
      x[base + 2] = -0.1 * mu * normal;
      w[base + 1] = -0.2;
    } else {
      x[base + 1] = 0.1 * mu * normal;
      x[base + 2] = -mu * normal;
      w[base + 2] = 0.2;
    }
  }

  Eigen::VectorXd b = A * x - w;
  return dart::math::LcpProblem(
      std::move(A),
      std::move(b),
      std::move(lo),
      std::move(hi),
      std::move(findex));
}

std::vector<int> syntheticGroupArgs(const SyntheticCudaFamily family)
{
  switch (family) {
    case SyntheticCudaFamily::Standard:
    case SyntheticCudaFamily::Boxed:
      return {8, 16, 32, 64, 96, 128, 192, 256};
    case SyntheticCudaFamily::FrictionIndex:
      return {2, 4, 8, 16, 32, 48, 64, 96};
  }

  return {};
}

dart::math::LcpProblem makeSyntheticProblem(
    const SyntheticCudaFamily family, const int arg, const int variant)
{
  switch (family) {
    case SyntheticCudaFamily::Standard:
      return makeSyntheticStandardProblem(arg, variant);
    case SyntheticCudaFamily::Boxed:
      return makeSyntheticBoxedProblem(arg, variant);
    case SyntheticCudaFamily::FrictionIndex:
      return makeSyntheticFrictionIndexProblem(arg, variant);
  }

  return makeSyntheticStandardProblem(arg, variant);
}

WorldContactBatch makeSyntheticBatch(
    const SyntheticCudaFamily family,
    const int arg,
    const int batchSize,
    const std::size_t iterations)
{
  cuda::LcpBatchCudaProblem packet;
  packet.problemSize = family == SyntheticCudaFamily::FrictionIndex
                           ? static_cast<std::size_t>(3 * arg)
                           : static_cast<std::size_t>(arg);
  packet.iterations = iterations;

  std::vector<dart::math::LcpProblem> problems;
  problems.reserve(static_cast<std::size_t>(batchSize));
  for (int variant = 0; variant < batchSize; ++variant) {
    auto problem = makeSyntheticProblem(family, arg, variant);
    appendLcpProblem(packet, problem);
    problems.push_back(std::move(problem));
  }

  return {std::move(packet), std::move(problems)};
}

WorldContactGroupedBatch makeSyntheticGroupedBatch(
    const SyntheticCudaFamily family,
    const int variantsPerGroup,
    const std::size_t iterations)
{
  WorldContactGroupedBatch grouped;
  const std::vector<int> groupArgs = syntheticGroupArgs(family);
  grouped.packets.reserve(groupArgs.size());
  grouped.problemsByGroup.reserve(groupArgs.size());

  int variantBase = 0;
  for (const int arg : groupArgs) {
    cuda::LcpBatchCudaProblem packet;
    packet.problemSize = family == SyntheticCudaFamily::FrictionIndex
                             ? static_cast<std::size_t>(3 * arg)
                             : static_cast<std::size_t>(arg);
    packet.iterations = iterations;

    std::vector<dart::math::LcpProblem> problems;
    problems.reserve(static_cast<std::size_t>(variantsPerGroup));
    for (int variant = 0; variant < variantsPerGroup; ++variant) {
      auto problem = makeSyntheticProblem(family, arg, variantBase + variant);
      appendLcpProblem(packet, problem);
      problems.push_back(std::move(problem));
    }

    grouped.packets.push_back(std::move(packet));
    grouped.problemsByGroup.push_back(std::move(problems));
    variantBase += variantsPerGroup;
  }

  return grouped;
}

std::optional<WorldContactProblem> makeWorldContactProblem(
    int contactCount, int variant, std::string& errorMessage)
{
  if (contactCount <= 0) {
    errorMessage = "contact count must be positive";
    return std::nullopt;
  }

  constexpr double kFriction = 0.7;
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(32.0, 32.0, 0.5)));
  ground.setFriction(kFriction);

  const int columns = static_cast<int>(
      std::ceil(std::sqrt(static_cast<double>(contactCount))));
  constexpr double kSpacing = 2.0;
  for (int i = 0; i < contactCount; ++i) {
    const int row = i / columns;
    const int col = i - row * columns;
    const double variantScale = static_cast<double>(variant + 1);

    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.5);
    sphereOptions.linearVelocity = Eigen::Vector3d(
        0.25 + 0.03 * variantScale - 0.02 * static_cast<double>(i % 3),
        -0.18 - 0.02 * variantScale + 0.015 * static_cast<double>(i % 5),
        -0.16 - 0.01 * variantScale);
    auto sphere
        = world.addRigidBody("sphere_" + std::to_string(i), sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(kFriction);
  }

  const std::vector<sx::Contact> contacts = world.collide();
  if (contacts.size() != static_cast<std::size_t>(contactCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " contacts, expected " + std::to_string(contactCount);
    return std::nullopt;
  }

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(world), contacts, world.getTimeStep());
  if (snapshot.contactCount != static_cast<std::size_t>(contactCount)
      || snapshot.size() != static_cast<Eigen::Index>(3 * contactCount)) {
    errorMessage = "boxed-LCP contact snapshot shape did not match contacts";
    return std::nullopt;
  }

  return WorldContactProblem{
      dart::math::LcpProblem(
          snapshot.A, snapshot.b, snapshot.lo, snapshot.hi, snapshot.findex),
      snapshot.contactCount};
}

std::optional<WorldContactProblem> makeWorldStackContactProblem(
    int sphereCount, int variant, std::string& errorMessage)
{
  if (sphereCount < 2) {
    errorMessage = "sphere count must be at least two";
    return std::nullopt;
  }

  constexpr double kFriction = 0.6;
  sx::WorldOptions options;
  options.timeStep = 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(Eigen::Vector3d(8.0, 8.0, 0.5)));
  ground.setFriction(kFriction);

  const double variantScale = static_cast<double>(variant);
  for (int i = 0; i < sphereCount; ++i) {
    sx::RigidBodyOptions sphereOptions;
    sphereOptions.position
        = Eigen::Vector3d(0.0, 0.0, 0.5 + static_cast<double>(i));
    sphereOptions.linearVelocity = Eigen::Vector3d(
        0.18 - 0.06 * static_cast<double>(i) + 0.01 * variantScale,
        -0.12 + 0.05 * static_cast<double>(i) - 0.008 * variantScale,
        -0.16 - 0.08 * static_cast<double>(i) - 0.005 * variantScale);
    auto sphere = world.addRigidBody(
        "stack_sphere_" + std::to_string(i), sphereOptions);
    sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
    sphere.setFriction(kFriction);
  }

  const std::vector<sx::Contact> contacts = world.collide();
  if (contacts.size() != static_cast<std::size_t>(sphereCount)) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " stack contacts, expected " + std::to_string(sphereCount);
    return std::nullopt;
  }

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(world), contacts, world.getTimeStep());
  if (snapshot.contactCount != static_cast<std::size_t>(sphereCount)
      || snapshot.size() != static_cast<Eigen::Index>(3 * sphereCount)) {
    errorMessage
        = "boxed-LCP stack contact snapshot shape did not match contacts";
    return std::nullopt;
  }

  return WorldContactProblem{
      dart::math::LcpProblem(
          snapshot.A, snapshot.b, snapshot.lo, snapshot.hi, snapshot.findex),
      snapshot.contactCount};
}

std::optional<WorldContactProblem> makeWorldBoxContactProblem(
    int boxCount, int variant, std::string& errorMessage)
{
  if (boxCount <= 0) {
    errorMessage = "dense box contact box count must be positive";
    return std::nullopt;
  }

  constexpr double kFriction = 0.5;
  sx::WorldOptions options;
  options.timeStep = boxCount >= 24 ? 0.001 : 0.005;
  options.gravity = Eigen::Vector3d(0.0, 0.0, -9.81);
  options.contactSolverMethod = sx::ContactSolverMethod::BoxedLcp;
  sx::World world(options);

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Eigen::Vector3d(0.0, 0.0, -0.5);
  auto ground = world.addRigidBody("ground", groundOptions);
  const double groundHalfExtent = makeDenseBoxGroundHalfExtent(boxCount);
  ground.setCollisionShape(
      sx::CollisionShape::makeBox(
          Eigen::Vector3d(groundHalfExtent, groundHalfExtent, 0.5)));
  ground.setFriction(kFriction);

  const int columns
      = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(boxCount))));
  constexpr double kSpacing = 2.0;
  const double variantScale = static_cast<double>(variant);
  for (int i = 0; i < boxCount; ++i) {
    const int row = i / columns;
    const int col = i - row * columns;
    sx::RigidBodyOptions boxOptions;
    boxOptions.position = Eigen::Vector3d(
        kSpacing * static_cast<double>(col),
        kSpacing * static_cast<double>(row),
        0.5);
    boxOptions.linearVelocity = Eigen::Vector3d(
        0.35 + 0.02 * variantScale - 0.015 * static_cast<double>(i % 3),
        -0.2 + 0.015 * variantScale + 0.01 * static_cast<double>(i % 2),
        -0.02 - 0.002 * variantScale);
    auto box = world.addRigidBody("box_" + std::to_string(i), boxOptions);
    box.setCollisionShape(
        sx::CollisionShape::makeBox(Eigen::Vector3d(0.5, 0.5, 0.5)));
    box.setFriction(kFriction);
  }

  const std::vector<sx::Contact> contacts = world.collide();
  const auto expectedContacts = static_cast<std::size_t>(4 * boxCount);
  if (contacts.size() != expectedContacts) {
    errorMessage = "World::collide returned " + std::to_string(contacts.size())
                   + " box contacts, expected "
                   + std::to_string(expectedContacts);
    return std::nullopt;
  }

  const sx::detail::BoxedLcpContactSnapshot snapshot
      = sx::detail::solveBoxedLcpContacts(
          sx::detail::registryOf(world), contacts, world.getTimeStep());
  if (snapshot.contactCount != contacts.size()
      || snapshot.size()
             != static_cast<Eigen::Index>(3 * snapshot.contactCount)) {
    errorMessage
        = "boxed-LCP dense box contact snapshot shape did not match contacts";
    return std::nullopt;
  }
  if (snapshot.bodyCount != static_cast<std::size_t>(boxCount)) {
    errorMessage = "dense box contact snapshot body count changed";
    return std::nullopt;
  }

  return WorldContactProblem{
      dart::math::LcpProblem(
          snapshot.A, snapshot.b, snapshot.lo, snapshot.hi, snapshot.findex),
      snapshot.contactCount};
}

ThreeAxisPrismaticCudaRobot addThreeAxisPrismaticCudaRobot(
    sx::World& world, const std::string& name)
{
  auto robot = world.addMultibody(name);
  auto base = robot.addLink("base");

  sx::JointSpec xSpec;
  xSpec.name = "x";
  xSpec.type = sx::JointType::Prismatic;
  xSpec.axis = Eigen::Vector3d::UnitX();
  auto xLink = robot.addLink("x_link", base, xSpec);
  xLink.setMass(1.0);
  xLink.setInertia(Eigen::Matrix3d::Identity());

  sx::JointSpec ySpec;
  ySpec.name = "y";
  ySpec.type = sx::JointType::Prismatic;
  ySpec.axis = Eigen::Vector3d::UnitY();
  auto yLink = robot.addLink("y_link", xLink, ySpec);
  yLink.setMass(1.0);
  yLink.setInertia(Eigen::Matrix3d::Identity());

  sx::JointSpec zSpec;
  zSpec.name = "z";
  zSpec.type = sx::JointType::Prismatic;
  zSpec.axis = Eigen::Vector3d::UnitZ();
  auto tip = robot.addLink("tip", yLink, zSpec);
  tip.setMass(1.0);
  tip.setInertia(Eigen::Matrix3d::Identity());

  return {
      sx::detail::toRegistryEntity(robot.getEntity()),
      sx::detail::toRegistryEntity(tip.getEntity())};
}

bool completeCudaCrossMultibodyRows(
    sx::detail::WorldRegistry& registry,
    std::vector<compute::UnifiedMultibodyContact>& multibodyContacts,
    const std::vector<Eigen::VectorXd>& multibodyVelocities,
    std::string& errorMessage)
{
  if (multibodyVelocities.size() != multibodyContacts.size()) {
    errorMessage = "cross-multibody articulated velocity count changed";
    return false;
  }

  const auto findMultibodyIndex
      = [&](entt::entity multibody) -> std::optional<std::size_t> {
    for (std::size_t i = 0; i < multibodyContacts.size(); ++i) {
      if (multibodyContacts[i].multibody == multibody) {
        return i;
      }
    }
    return std::nullopt;
  };

  constexpr double restitutionThreshold = 1e-2;
  for (std::size_t primaryIndex = 0; primaryIndex < multibodyContacts.size();
       ++primaryIndex) {
    auto& primaryContact = multibodyContacts[primaryIndex];
    const Eigen::VectorXd& primaryVelocity = multibodyVelocities[primaryIndex];

    for (auto& row : primaryContact.problem.rows) {
      if (row.otherMultibody == entt::null) {
        continue;
      }

      const auto otherIndex = findMultibodyIndex(row.otherMultibody);
      if (!otherIndex.has_value()) {
        errorMessage = "cross-multibody articulated contact target is missing";
        return false;
      }

      const auto& otherStructure
          = registry.get<sx::comps::MultibodyStructure>(row.otherMultibody);
      const Eigen::MatrixXd otherWorldJacobian
          = compute::computeMultibodyLinkWorldJacobian(
              registry, otherStructure, row.otherLink);
      if (otherWorldJacobian.rows() != 6) {
        errorMessage = "cross-multibody articulated Jacobian shape changed";
        return false;
      }
      const Eigen::MatrixXd otherPointJacobian
          = otherWorldJacobian.bottomRows<3>();

      row.otherNormalJacobian = otherPointJacobian.transpose() * row.normal;
      row.otherTangentJacobian1 = otherPointJacobian.transpose() * row.tangent1;
      row.otherTangentJacobian2 = otherPointJacobian.transpose() * row.tangent2;

      const Eigen::MatrixXd& otherInverseMass
          = multibodyContacts[*otherIndex].problem.inverseMass;
      row.normalDenominator += row.otherNormalJacobian.dot(
          otherInverseMass * row.otherNormalJacobian);
      row.tangentDenominator1 += row.otherTangentJacobian1.dot(
          otherInverseMass * row.otherTangentJacobian1);
      row.tangentDenominator2 += row.otherTangentJacobian2.dot(
          otherInverseMass * row.otherTangentJacobian2);

      const Eigen::VectorXd& otherVelocity = multibodyVelocities[*otherIndex];
      const auto relativeVelocity = [&](const Eigen::VectorXd& primaryJacobian,
                                        const Eigen::VectorXd& otherJacobian) {
        return primaryJacobian.dot(primaryVelocity)
               - otherJacobian.dot(otherVelocity);
      };

      const double approachingVelocity
          = relativeVelocity(row.normalJacobian, row.otherNormalJacobian);
      row.restitutionTarget = (approachingVelocity < -restitutionThreshold)
                                  ? -row.restitution * approachingVelocity
                                  : 0.0;
      row.normalRhs
          = -approachingVelocity + std::max(row.bias, row.restitutionTarget);
      row.tangentRhs1
          = -relativeVelocity(row.tangentJacobian1, row.otherTangentJacobian1);
      row.tangentRhs2
          = -relativeVelocity(row.tangentJacobian2, row.otherTangentJacobian2);
      row.active = row.normalDenominator > 0.0;
    }
  }

  return true;
}

std::optional<WorldContactProblem> makeArticulatedUnifiedContactProblem(
    ArticulatedCudaContactCase contactCase,
    int contactCount,
    int variant,
    std::string& errorMessage)
{
  if (contactCount <= 0) {
    errorMessage = "articulated contact count must be positive";
    return std::nullopt;
  }

  constexpr double kTimeStep = 0.005;
  constexpr double kFriction = 0.45;
  sx::World world;
  world.setGravity(Eigen::Vector3d::Zero());

  auto& registry = sx::detail::registryOf(world);
  std::vector<compute::UnifiedMultibodyContact> multibodyContacts;
  std::vector<Eigen::VectorXd> multibodyVelocities;
  multibodyContacts.reserve(static_cast<std::size_t>(2 * contactCount));
  multibodyVelocities.reserve(static_cast<std::size_t>(2 * contactCount));

  const double variantScale = static_cast<double>(variant);
  for (int i = 0; i < contactCount; ++i) {
    const auto robot = addThreeAxisPrismaticCudaRobot(
        world,
        "three_axis_" + std::to_string(variant) + "_" + std::to_string(i));

    const Eigen::Vector3d point(
        1.5 * static_cast<double>(i),
        0.2 * static_cast<double>(i % 2),
        0.05 * static_cast<double>(i % 3));

    compute::LinkContact contact;
    contact.link = robot.link;
    contact.point = point;
    contact.depth = 0.004 + 0.0005 * static_cast<double>(i % 3);
    contact.friction = kFriction;
    contact.restitution = 0.0;

    Eigen::VectorXd nextVelocity(3);
    switch (contactCase) {
      case ArticulatedCudaContactCase::Ground:
        contact.normal = Eigen::Vector3d::UnitZ();
        nextVelocity << 0.35 + 0.02 * static_cast<double>(i % 3)
                            + 0.006 * variantScale,
            -0.25 + 0.015 * static_cast<double>(i % 2) - 0.004 * variantScale,
            -0.55 - 0.01 * static_cast<double>(i % 4) - 0.003 * variantScale;
        break;
      case ArticulatedCudaContactCase::RigidImpact: {
        sx::RigidBodyOptions obstacleOptions;
        obstacleOptions.mass = 1.5;
        obstacleOptions.inertia = 0.8 * Eigen::Matrix3d::Identity();
        obstacleOptions.position = point;
        obstacleOptions.linearVelocity = Eigen::Vector3d(
            0.08 + 0.01 * static_cast<double>(i % 2), -0.03, 0.02);
        auto obstacle = world.addRigidBody(
            "dynamic_obstacle_" + std::to_string(variant) + "_"
                + std::to_string(i),
            obstacleOptions);

        contact.normal = Eigen::Vector3d::UnitX();
        contact.otherBody = sx::detail::toRegistryEntity(obstacle.getEntity());
        nextVelocity << -0.85 - 0.02 * static_cast<double>(i % 3)
                            - 0.004 * variantScale,
            0.22 - 0.015 * static_cast<double>(i % 2) + 0.003 * variantScale,
            -0.18 + 0.01 * static_cast<double>(i % 4) - 0.002 * variantScale;
        break;
      }
      case ArticulatedCudaContactCase::CrossLinkImpact: {
        const auto target = addThreeAxisPrismaticCudaRobot(
            world,
            "three_axis_target_" + std::to_string(variant) + "_"
                + std::to_string(i));
        contact.normal = Eigen::Vector3d::UnitX();
        contact.otherLink = target.link;
        contact.otherMultibody = target.multibody;
        nextVelocity << -0.85 - 0.02 * static_cast<double>(i % 3)
                            - 0.004 * variantScale,
            0.22 - 0.015 * static_cast<double>(i % 2) + 0.003 * variantScale,
            -0.18 + 0.01 * static_cast<double>(i % 4) - 0.002 * variantScale;

        Eigen::VectorXd targetVelocity(3);
        targetVelocity << 0.08 + 0.01 * static_cast<double>(i % 2)
                              + 0.002 * variantScale,
            -0.03 - 0.004 * static_cast<double>(i % 3), 0.02;

        const auto& primaryStructure
            = registry.get<sx::comps::MultibodyStructure>(robot.multibody);
        const std::vector<compute::LinkContact> linkContacts{contact};
        auto primaryProblem = compute::assembleMultibodyLinkContactProblem(
            registry, primaryStructure, nextVelocity, kTimeStep, linkContacts);
        if (primaryProblem.rows.size() != 1u) {
          errorMessage
              = "cross-multibody articulated contact did not assemble one row";
          return std::nullopt;
        }

        const auto& targetStructure
            = registry.get<sx::comps::MultibodyStructure>(target.multibody);
        const std::vector<compute::LinkContact> targetContacts;
        auto targetProblem = compute::assembleMultibodyLinkContactProblem(
            registry,
            targetStructure,
            targetVelocity,
            kTimeStep,
            targetContacts);
        if (!targetProblem.rows.empty()) {
          errorMessage
              = "cross-multibody target contact unexpectedly assembled rows";
          return std::nullopt;
        }

        multibodyContacts.push_back(
            {robot.multibody, std::move(primaryProblem)});
        multibodyVelocities.push_back(std::move(nextVelocity));
        multibodyContacts.push_back(
            {target.multibody, std::move(targetProblem)});
        multibodyVelocities.push_back(std::move(targetVelocity));
        continue;
      }
    }

    const auto& structure
        = registry.get<sx::comps::MultibodyStructure>(robot.multibody);
    const std::vector<compute::LinkContact> linkContacts{contact};
    auto linkProblem = compute::assembleMultibodyLinkContactProblem(
        registry, structure, nextVelocity, kTimeStep, linkContacts);
    if (linkProblem.rows.size() != 1u || !linkProblem.rows[0].active) {
      errorMessage = "articulated link contact did not assemble one active row";
      return std::nullopt;
    }

    const auto& row = linkProblem.rows[0];
    if (row.normalDenominator <= 0.0 || row.tangentDenominator1 <= 0.0
        || row.tangentDenominator2 <= 0.0) {
      errorMessage = "articulated link contact has a nonpositive denominator";
      return std::nullopt;
    }

    multibodyContacts.push_back({robot.multibody, std::move(linkProblem)});
    multibodyVelocities.push_back(std::move(nextVelocity));
  }

  if (!completeCudaCrossMultibodyRows(
          registry, multibodyContacts, multibodyVelocities, errorMessage)) {
    return std::nullopt;
  }

  std::size_t activeRowCount = 0;
  for (const auto& multibodyContact : multibodyContacts) {
    for (const auto& row : multibodyContact.problem.rows) {
      if (!row.active) {
        errorMessage = "articulated link contact row is inactive";
        return std::nullopt;
      }
      if (row.normalDenominator <= 0.0 || row.tangentDenominator1 <= 0.0
          || row.tangentDenominator2 <= 0.0) {
        errorMessage = "articulated link contact has a nonpositive denominator";
        return std::nullopt;
      }
      ++activeRowCount;
    }
  }
  if (activeRowCount != static_cast<std::size_t>(contactCount)) {
    errorMessage = "articulated unified contact active row count changed";
    return std::nullopt;
  }

  compute::RigidBodyContactProblem emptyRigid;
  const auto unified = compute::assembleUnifiedConstraintProblem(
      emptyRigid, multibodyContacts);
  const Eigen::Index expectedRows
      = compute::UnifiedConstraintProblem::kRowsPerContact * contactCount;
  if (unified.delassus.rows() != expectedRows
      || unified.delassus.cols() != expectedRows
      || unified.rhs.size() != expectedRows
      || unified.findex.size() != expectedRows) {
    errorMessage = "articulated unified contact shape changed";
    return std::nullopt;
  }

  const auto solution = compute::solveUnifiedConstraintProblem(unified);
  if (!solution.succeeded) {
    errorMessage = "articulated unified contact reference solve failed";
    return std::nullopt;
  }

  dart::math::LcpProblem problem(
      unified.delassus, unified.rhs, unified.lo, unified.hi, unified.findex);
  dart::math::LcpOptions options;
  options.maxIterations = 1024;
  options.absoluteTolerance = 1e-6;
  options.relativeTolerance = 1e-4;
  options.complementarityTolerance = 1e-6;
  const auto check
      = dart::test::CheckLcpSolution(problem, solution.lambda, options);
  if (!check.ok) {
    errorMessage
        = "articulated unified contact reference violates LCP contract";
    return std::nullopt;
  }

  return WorldContactProblem{
      std::move(problem), static_cast<std::size_t>(contactCount)};
}

std::optional<WorldContactBatch> makeWorldContactBatch(
    int contactCount,
    int batchSize,
    std::size_t iterations,
    std::string& errorMessage)
{
  WorldContactBatch batch;
  batch.packet.problemSize = static_cast<std::size_t>(3 * contactCount);
  batch.packet.iterations = iterations;
  batch.problems.reserve(static_cast<std::size_t>(batchSize));

  for (int i = 0; i < batchSize; ++i) {
    auto fixture = makeWorldContactProblem(contactCount, i, errorMessage);
    if (!fixture.has_value()) {
      return std::nullopt;
    }

    appendLcpProblem(batch.packet, fixture->problem);
    batch.problems.push_back(std::move(fixture->problem));
  }

  return batch;
}

std::optional<WorldContactBatch> makeWorldStackContactBatch(
    int sphereCount,
    int batchSize,
    std::size_t iterations,
    std::string& errorMessage)
{
  WorldContactBatch batch;
  batch.packet.problemSize = static_cast<std::size_t>(3 * sphereCount);
  batch.packet.iterations = iterations;
  batch.problems.reserve(static_cast<std::size_t>(batchSize));

  for (int i = 0; i < batchSize; ++i) {
    auto fixture = makeWorldStackContactProblem(sphereCount, i, errorMessage);
    if (!fixture.has_value()) {
      return std::nullopt;
    }

    appendLcpProblem(batch.packet, fixture->problem);
    batch.problems.push_back(std::move(fixture->problem));
  }

  return batch;
}

std::optional<WorldContactBatch> makeWorldBoxContactBatch(
    int boxCount,
    int batchSize,
    std::size_t iterations,
    std::string& errorMessage)
{
  if (boxCount <= 0) {
    errorMessage = "dense box contact box count must be positive";
    return std::nullopt;
  }
  if (batchSize <= 0) {
    errorMessage = "dense box contact batch size must be positive";
    return std::nullopt;
  }

  WorldContactBatch batch;
  batch.packet.iterations = iterations;
  batch.problems.reserve(static_cast<std::size_t>(batchSize));

  for (int i = 0; i < batchSize; ++i) {
    auto fixture = makeWorldBoxContactProblem(boxCount, i, errorMessage);
    if (!fixture.has_value()) {
      return std::nullopt;
    }

    const std::size_t problemSize
        = static_cast<std::size_t>(fixture->problem.b.size());
    if (batch.packet.problemSize == 0u) {
      batch.packet.problemSize = problemSize;
    } else if (batch.packet.problemSize != problemSize) {
      errorMessage = "dense box contact batch problem shape changed";
      return std::nullopt;
    }
    if (fixture->contactCount != static_cast<std::size_t>(4 * boxCount)
        || problemSize != static_cast<std::size_t>(12 * boxCount)) {
      errorMessage = "dense box contact batch contact shape changed";
      return std::nullopt;
    }

    appendLcpProblem(batch.packet, fixture->problem);
    batch.problems.push_back(std::move(fixture->problem));
  }

  return batch;
}

std::optional<WorldContactGroupedBatch> makeWorldBoxContactGroupedBatch(
    int variantsPerBoxCount,
    std::size_t iterations,
    std::string& errorMessage,
    int maxBoxCount = 96)
{
  if (variantsPerBoxCount <= 0) {
    errorMessage = "variants per dense box count must be positive";
    return std::nullopt;
  }
  if (maxBoxCount <= 0) {
    errorMessage = "maximum dense box count must be positive";
    return std::nullopt;
  }

  constexpr std::array<int, 10> kBoxCounts{1, 2, 4, 8, 16, 24, 32, 48, 64, 96};
  WorldContactGroupedBatch grouped;
  grouped.packets.reserve(kBoxCounts.size());
  grouped.problemsByGroup.reserve(kBoxCounts.size());

  int variantBase = 0;
  for (const int boxCount : kBoxCounts) {
    if (boxCount > maxBoxCount) {
      continue;
    }

    cuda::LcpBatchCudaProblem packet;
    packet.problemSize = static_cast<std::size_t>(12 * boxCount);
    packet.iterations = iterations;

    std::vector<dart::math::LcpProblem> problems;
    problems.reserve(static_cast<std::size_t>(variantsPerBoxCount));

    for (int i = 0; i < variantsPerBoxCount; ++i) {
      auto fixture
          = makeWorldBoxContactProblem(boxCount, variantBase + i, errorMessage);
      if (!fixture.has_value()) {
        return std::nullopt;
      }
      if (fixture->contactCount != static_cast<std::size_t>(4 * boxCount)) {
        errorMessage = "dense box grouped batch contact shape changed";
        return std::nullopt;
      }

      appendLcpProblem(packet, fixture->problem);
      problems.push_back(std::move(fixture->problem));
    }

    grouped.packets.push_back(std::move(packet));
    grouped.problemsByGroup.push_back(std::move(problems));
    variantBase += variantsPerBoxCount;
  }

  return grouped;
}

std::optional<WorldContactGroupedBatch> makeWorldContactGroupedBatch(
    int variantsPerContactCount,
    std::size_t iterations,
    std::string& errorMessage)
{
  if (variantsPerContactCount <= 0) {
    errorMessage = "variants per contact count must be positive";
    return std::nullopt;
  }

  constexpr std::array<int, 7> kContactCounts{1, 2, 4, 8, 16, 24, 32};
  WorldContactGroupedBatch grouped;
  grouped.packets.reserve(kContactCounts.size());
  grouped.problemsByGroup.reserve(kContactCounts.size());

  int variantBase = 0;
  for (const int contactCount : kContactCounts) {
    cuda::LcpBatchCudaProblem packet;
    packet.problemSize = static_cast<std::size_t>(3 * contactCount);
    packet.iterations = iterations;

    std::vector<dart::math::LcpProblem> problems;
    problems.reserve(static_cast<std::size_t>(variantsPerContactCount));

    for (int i = 0; i < variantsPerContactCount; ++i) {
      auto fixture = makeWorldContactProblem(
          contactCount, variantBase + i, errorMessage);
      if (!fixture.has_value()) {
        return std::nullopt;
      }

      appendLcpProblem(packet, fixture->problem);
      problems.push_back(std::move(fixture->problem));
    }

    grouped.packets.push_back(std::move(packet));
    grouped.problemsByGroup.push_back(std::move(problems));
    variantBase += variantsPerContactCount;
  }

  return grouped;
}

std::optional<WorldContactGroupedBatch> makeWorldStackContactGroupedBatch(
    int variantsPerSphereCount,
    std::size_t iterations,
    std::string& errorMessage)
{
  if (variantsPerSphereCount <= 0) {
    errorMessage = "variants per sphere count must be positive";
    return std::nullopt;
  }

  constexpr std::array<int, 17> kSphereCounts{
      2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 24, 32};
  WorldContactGroupedBatch grouped;
  grouped.packets.reserve(kSphereCounts.size());
  grouped.problemsByGroup.reserve(kSphereCounts.size());

  int variantBase = 0;
  for (const int sphereCount : kSphereCounts) {
    cuda::LcpBatchCudaProblem packet;
    packet.problemSize = static_cast<std::size_t>(3 * sphereCount);
    packet.iterations = iterations;

    std::vector<dart::math::LcpProblem> problems;
    problems.reserve(static_cast<std::size_t>(variantsPerSphereCount));

    for (int i = 0; i < variantsPerSphereCount; ++i) {
      auto fixture = makeWorldStackContactProblem(
          sphereCount, variantBase + i, errorMessage);
      if (!fixture.has_value()) {
        return std::nullopt;
      }

      appendLcpProblem(packet, fixture->problem);
      problems.push_back(std::move(fixture->problem));
    }

    grouped.packets.push_back(std::move(packet));
    grouped.problemsByGroup.push_back(std::move(problems));
    variantBase += variantsPerSphereCount;
  }

  return grouped;
}

std::optional<WorldContactGroupedBatch>
makeArticulatedUnifiedContactGroupedBatch(
    int variantsPerContactCount,
    std::size_t iterations,
    std::string& errorMessage)
{
  if (variantsPerContactCount <= 0) {
    errorMessage = "variants per articulated contact count must be positive";
    return std::nullopt;
  }

  constexpr std::array<int, 6> kContactCounts{1, 4, 8, 16, 24, 32};
  WorldContactGroupedBatch grouped;
  grouped.packets.reserve(kContactCounts.size());
  grouped.problemsByGroup.reserve(kContactCounts.size());

  int variantBase = 0;
  for (const int contactCount : kContactCounts) {
    cuda::LcpBatchCudaProblem packet;
    packet.problemSize = static_cast<std::size_t>(3 * contactCount);
    packet.iterations = iterations;

    std::vector<dart::math::LcpProblem> problems;
    problems.reserve(static_cast<std::size_t>(3 * variantsPerContactCount));

    for (int variant = 0; variant < variantsPerContactCount; ++variant) {
      for (const ArticulatedCudaContactCase contactCase :
           {ArticulatedCudaContactCase::Ground,
            ArticulatedCudaContactCase::RigidImpact,
            ArticulatedCudaContactCase::CrossLinkImpact}) {
        auto fixture = makeArticulatedUnifiedContactProblem(
            contactCase, contactCount, variantBase + variant, errorMessage);
        if (!fixture.has_value()) {
          return std::nullopt;
        }

        appendLcpProblem(packet, fixture->problem);
        problems.push_back(std::move(fixture->problem));
      }
    }

    grouped.packets.push_back(std::move(packet));
    grouped.problemsByGroup.push_back(std::move(problems));
    variantBase += variantsPerContactCount;
  }

  return grouped;
}

std::optional<WorldContactGroupedBatch> makeMixedContactGroupedBatch(
    int variantsPerScenario, std::size_t iterations, std::string& errorMessage)
{
  if (variantsPerScenario <= 0) {
    errorMessage = "variants per mixed contact scenario must be positive";
    return std::nullopt;
  }

  WorldContactGroupedBatch grouped;
  std::vector<std::size_t> groupSizes;

  auto appendFixture = [&](std::optional<WorldContactProblem> fixture) {
    if (!fixture.has_value()) {
      return false;
    }

    const auto problemSize
        = static_cast<std::size_t>(fixture->problem.b.size());
    auto groupIt = std::ranges::find(groupSizes, problemSize);
    if (groupIt == groupSizes.end()) {
      groupSizes.push_back(problemSize);
      grouped.problemsByGroup.emplace_back();
      groupIt = std::prev(groupSizes.end());
    }

    const auto groupIndex
        = static_cast<std::size_t>(std::distance(groupSizes.begin(), groupIt));
    grouped.problemsByGroup[groupIndex].push_back(std::move(fixture->problem));
    return true;
  };

  for (int variant = 0; variant < variantsPerScenario; ++variant) {
    if (!appendFixture(makeWorldContactProblem(1, variant, errorMessage))
        || !appendFixture(makeWorldContactProblem(4, variant, errorMessage))
        || !appendFixture(
            makeWorldStackContactProblem(2, variant, errorMessage))
        || !appendFixture(
            makeWorldStackContactProblem(5, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::Ground, 1, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::RigidImpact, 1, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::CrossLinkImpact,
            1,
            variant,
            errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::Ground, 4, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::RigidImpact, 4, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::CrossLinkImpact,
            4,
            variant,
            errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::Ground, 8, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::RigidImpact, 8, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::CrossLinkImpact,
            8,
            variant,
            errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::Ground, 16, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::RigidImpact, 16, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::CrossLinkImpact,
            16,
            variant,
            errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::Ground, 24, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::RigidImpact, 24, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::CrossLinkImpact,
            24,
            variant,
            errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::Ground, 32, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::RigidImpact, 32, variant, errorMessage))
        || !appendFixture(makeArticulatedUnifiedContactProblem(
            ArticulatedCudaContactCase::CrossLinkImpact,
            32,
            variant,
            errorMessage))) {
      return std::nullopt;
    }
  }

  grouped.packets.reserve(grouped.problemsByGroup.size());
  for (std::size_t group = 0; group < grouped.problemsByGroup.size(); ++group) {
    cuda::LcpBatchCudaProblem packet;
    packet.problemSize = groupSizes[group];
    packet.iterations = iterations;
    for (const auto& problem : grouped.problemsByGroup[group]) {
      appendLcpProblem(packet, problem);
    }
    grouped.packets.push_back(std::move(packet));
  }

  return grouped;
}

void expectNearExpectedSolution(
    const cuda::LcpBatchCudaProblem& packet,
    const std::vector<double>& expected,
    double tolerance)
{
  ASSERT_EQ(packet.x.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(packet.x[i], expected[i], tolerance) << "index " << i;
  }
}

void expectBatchSatisfiesLcpContract(
    const cuda::LcpBatchCudaProblem& packet,
    const std::vector<dart::math::LcpProblem>& problems)
{
  ASSERT_EQ(packet.problemCount, problems.size());
  ASSERT_EQ(packet.x.size(), packet.problemCount * packet.problemSize);

  dart::math::LcpOptions options;
  options.maxIterations = static_cast<int>(packet.iterations);
  options.absoluteTolerance = 1e-6;
  options.relativeTolerance = 1e-4;
  options.complementarityTolerance = 1e-6;

  for (std::size_t problemIndex = 0; problemIndex < problems.size();
       ++problemIndex) {
    Eigen::VectorXd x(static_cast<Eigen::Index>(packet.problemSize));
    for (std::size_t row = 0; row < packet.problemSize; ++row) {
      x[static_cast<Eigen::Index>(row)]
          = packet.x[problemIndex * packet.problemSize + row];
    }

    const auto check
        = dart::test::CheckLcpSolution(problems[problemIndex], x, options);
    EXPECT_TRUE(check.ok) << "problem " << problemIndex
                          << " residual=" << check.residual
                          << " complementarity=" << check.complementarity
                          << " bound_violation=" << check.boundViolation
                          << " message=" << check.message;
  }
}

void expectGroupedBatchSatisfiesLcpContract(
    const WorldContactGroupedBatch& grouped)
{
  ASSERT_EQ(grouped.packets.size(), grouped.problemsByGroup.size());
  for (std::size_t group = 0; group < grouped.packets.size(); ++group) {
    expectBatchSatisfiesLcpContract(
        grouped.packets[group], grouped.problemsByGroup[group]);
  }
}

} // namespace

//==============================================================================
TEST(CudaLcpJacobiBatch, RejectsMismatchedBuffersBeforeRuntime)
{
  cuda::LcpJacobiBatchCudaProblem packet;
  packet.problemSize = 2;
  packet.problemCount = 1;
  packet.iterations = 8;
  packet.A.resize(3);
  packet.b.resize(2);
  packet.lo.resize(2);
  packet.hi.resize(2);
  packet.findex.resize(2);
  packet.x.resize(2);

  EXPECT_THROW(
      cuda::solveBoxedLcpJacobiBatchCuda(packet), sx::InvalidArgumentException);
}

//==============================================================================
TEST(CudaLcpJacobiBatch, RejectsInvalidFindexBeforeRuntime)
{
  cuda::LcpJacobiBatchCudaProblem packet;
  packet.problemSize = 2;
  packet.problemCount = 1;
  packet.iterations = 8;
  packet.A = {2.0, 0.0, 0.0, 2.0};
  packet.b = {1.0, 0.0};
  packet.lo = {0.0, -0.5};
  packet.hi = {kInf, 0.5};
  packet.findex = {-1, 1};
  packet.x = {0.0, 0.0};

  EXPECT_THROW(
      cuda::solveBoxedLcpJacobiBatchCuda(packet), sx::InvalidArgumentException);
}

//==============================================================================
TEST(CudaLcpJacobiBatch, EmptyBatchReturnsBeforeCudaRuntime)
{
  cuda::LcpJacobiBatchCudaProblem packet;
  packet.iterations = 1;
  EXPECT_NO_THROW(cuda::solveBoxedLcpJacobiBatchCuda(packet));
}

//==============================================================================
TEST(CudaLcpJacobiBatch, StandardBatchMatchesKnownSolutions)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector<double> expected;
  auto packet = makeStandardBatch(expected);
  cuda::solveBoxedLcpJacobiBatchCuda(packet);

  expectNearExpectedSolution(packet, expected, 1e-8);
}

//==============================================================================
TEST(CudaLcpJacobiBatch, FrictionIndexBatchMatchesKnownSolutions)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector<double> expected;
  auto packet = makeFrictionIndexBatch(expected);
  cuda::solveBoxedLcpJacobiBatchCuda(packet);

  expectNearExpectedSolution(packet, expected, 1e-8);
}

//==============================================================================
TEST(CudaLcpJacobiBatch, BoxedBatchMatchesKnownSolutions)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector<double> expected;
  auto packet = makeBoxedBatch(expected);
  cuda::solveBoxedLcpJacobiBatchCuda(packet);

  expectNearExpectedSolution(packet, expected, 1e-8);
}

//==============================================================================
TEST(CudaLcpJacobiBatch, VariableSizeSyntheticStandardBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerProblemArg : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerProblemArg=" + std::to_string(variantsPerProblemArg));
    auto fixture = makeSyntheticGroupedBatch(
        SyntheticCudaFamily::Standard, variantsPerProblemArg, 512);

    cuda::solveBoxedLcpJacobiGroupedBatchCuda(fixture.packets);

    expectGroupedBatchSatisfiesLcpContract(fixture);
  }
}

//==============================================================================
TEST(CudaLcpJacobiBatch, VariableSizeSyntheticBoxedBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerProblemArg : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerProblemArg=" + std::to_string(variantsPerProblemArg));
    auto fixture = makeSyntheticGroupedBatch(
        SyntheticCudaFamily::Boxed, variantsPerProblemArg, 512);

    cuda::solveBoxedLcpJacobiGroupedBatchCuda(fixture.packets);

    expectGroupedBatchSatisfiesLcpContract(fixture);
  }
}

//==============================================================================
TEST(
    CudaLcpJacobiBatch,
    VariableSizeSyntheticFrictionIndexBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerProblemArg : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerProblemArg=" + std::to_string(variantsPerProblemArg));
    auto fixture = makeSyntheticGroupedBatch(
        SyntheticCudaFamily::FrictionIndex, variantsPerProblemArg, 512);

    cuda::solveBoxedLcpJacobiGroupedBatchCuda(fixture.packets);

    expectGroupedBatchSatisfiesLcpContract(fixture);
  }
}

//==============================================================================
TEST(CudaLcpJacobiBatch, LargerSyntheticBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::array<std::pair<SyntheticCudaFamily, int>, 9> cases{{
      {SyntheticCudaFamily::Standard, 128},
      {SyntheticCudaFamily::Boxed, 128},
      {SyntheticCudaFamily::FrictionIndex, 48},
      {SyntheticCudaFamily::Standard, 192},
      {SyntheticCudaFamily::Boxed, 192},
      {SyntheticCudaFamily::FrictionIndex, 64},
      {SyntheticCudaFamily::Standard, 256},
      {SyntheticCudaFamily::Boxed, 256},
      {SyntheticCudaFamily::FrictionIndex, 96},
  }};
  for (const auto& [family, arg] : cases) {
    SCOPED_TRACE(
        std::string("family=") + getSyntheticCudaFamilyName(family)
        + " arg=" + std::to_string(arg));
    auto fixture = makeSyntheticBatch(family, arg, 4, 512);

    cuda::solveBoxedLcpJacobiBatchCuda(fixture.packet);

    expectBatchSatisfiesLcpContract(fixture.packet, fixture.problems);
  }
}

//==============================================================================
TEST(CudaLcpJacobiBatch, WorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::string errorMessage;
  auto fixture = makeWorldContactBatch(4, 4, 512, errorMessage);
  ASSERT_TRUE(fixture.has_value()) << errorMessage;

  cuda::solveBoxedLcpJacobiBatchCuda(fixture->packet);

  expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
}

//==============================================================================
TEST(CudaLcpJacobiBatch, LargerWorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::string errorMessage;
  auto fixture = makeWorldContactBatch(8, 4, 512, errorMessage);
  ASSERT_TRUE(fixture.has_value()) << errorMessage;

  cuda::solveBoxedLcpJacobiBatchCuda(fixture->packet);

  expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
}

//==============================================================================
TEST(CudaLcpJacobiBatch, DenserWorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int contactCount : {16, 24, 32}) {
    SCOPED_TRACE("contactCount=" + std::to_string(contactCount));
    std::string errorMessage;
    auto fixture = makeWorldContactBatch(contactCount, 4, 512, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpJacobiBatchCuda(fixture->packet);

    expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
  }
}

//==============================================================================
TEST(CudaLcpJacobiBatch, DenseBoxWorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int boxCount : {1, 4, 8, 16, 24, 32, 48, 64, 96, 128}) {
    SCOPED_TRACE("boxCount=" + std::to_string(boxCount));
    std::string errorMessage;
    auto fixture = makeWorldBoxContactBatch(boxCount, 4, 8192, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;
    fixture->packet.relaxation = 0.25;

    cuda::solveBoxedLcpJacobiBatchCuda(fixture->packet);

    expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
  }
}

//==============================================================================
TEST(CudaLcpJacobiBatch, DenseBoxWorldContactGroupedBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerBoxCount : {2, 3}) {
    std::string errorMessage;
    auto fixture = makeWorldBoxContactGroupedBatch(
        variantsPerBoxCount, 8192, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;
    for (auto& packet : fixture->packets) {
      packet.relaxation = 0.25;
    }

    cuda::solveBoxedLcpJacobiGroupedBatchCuda(fixture->packets);

    expectGroupedBatchSatisfiesLcpContract(*fixture);
  }
}

//==============================================================================
// Execute the largest dense box-face fixture as a single-problem CUDA batch for
// the dedicated largest-fixture gate.
TEST(CudaLcpJacobiBatch, DenseBoxWorldContactLargestFixtureSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  constexpr int boxCount = 128;
  std::string errorMessage;
  auto fixture = makeWorldBoxContactBatch(boxCount, 1, 8192, errorMessage);
  ASSERT_TRUE(fixture.has_value()) << errorMessage;
  fixture->packet.relaxation = 0.25;

  cuda::solveBoxedLcpJacobiBatchCuda(fixture->packet);

  expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
}

//==============================================================================
TEST(
    CudaLcpJacobiBatch, HomogeneousStackedWorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int sphereCount :
       {5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 24, 32}) {
    SCOPED_TRACE("sphereCount=" + std::to_string(sphereCount));
    std::string errorMessage;
    auto fixture
        = makeWorldStackContactBatch(sphereCount, 4, 8192, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;
    fixture->packet.relaxation = 0.25;

    cuda::solveBoxedLcpJacobiBatchCuda(fixture->packet);

    expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
  }
}

//==============================================================================
TEST(CudaLcpJacobiBatch, VariableSizeWorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerContactCount : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerContactCount=" + std::to_string(variantsPerContactCount));
    std::string errorMessage;
    auto fixture = makeWorldContactGroupedBatch(
        variantsPerContactCount, 512, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpJacobiGroupedBatchCuda(fixture->packets);

    expectGroupedBatchSatisfiesLcpContract(*fixture);
  }
}

//==============================================================================
TEST(CudaLcpJacobiBatch, StackedWorldContactGroupedBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerSphereCount : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerSphereCount=" + std::to_string(variantsPerSphereCount));
    std::string errorMessage;
    auto fixture = makeWorldStackContactGroupedBatch(
        variantsPerSphereCount, 8192, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;
    for (auto& packet : fixture->packets) {
      packet.relaxation = 0.25;
    }

    cuda::solveBoxedLcpJacobiGroupedBatchCuda(fixture->packets);

    expectGroupedBatchSatisfiesLcpContract(*fixture);
  }
}

//==============================================================================
TEST(
    CudaLcpJacobiBatch,
    ArticulatedUnifiedContactGroupedBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerContactCount : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerContactCount=" + std::to_string(variantsPerContactCount));
    std::string errorMessage;
    auto fixture = makeArticulatedUnifiedContactGroupedBatch(
        variantsPerContactCount, 1024, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpJacobiGroupedBatchCuda(fixture->packets);

    expectGroupedBatchSatisfiesLcpContract(*fixture);
  }
}

//==============================================================================
TEST(CudaLcpJacobiBatch, MixedContactGroupedBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerScenario : {2, 3}) {
    SCOPED_TRACE("variantsPerScenario=" + std::to_string(variantsPerScenario));
    std::string errorMessage;
    auto fixture
        = makeMixedContactGroupedBatch(variantsPerScenario, 1024, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpJacobiGroupedBatchCuda(fixture->packets);

    expectGroupedBatchSatisfiesLcpContract(*fixture);
  }
}

//==============================================================================
TEST(CudaLcpRedBlackGaussSeidelBatch, StandardBatchMatchesKnownSolutions)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector<double> expected;
  auto packet = makeStandardBatch(expected);
  cuda::solveBoxedLcpRedBlackGaussSeidelBatchCuda(packet);

  expectNearExpectedSolution(packet, expected, 1e-8);
}

//==============================================================================
TEST(CudaLcpRedBlackGaussSeidelBatch, FrictionIndexBatchMatchesKnownSolutions)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector<double> expected;
  auto packet = makeFrictionIndexBatch(expected);
  cuda::solveBoxedLcpRedBlackGaussSeidelBatchCuda(packet);

  expectNearExpectedSolution(packet, expected, 1e-8);
}

//==============================================================================
TEST(CudaLcpRedBlackGaussSeidelBatch, BoxedBatchMatchesKnownSolutions)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector<double> expected;
  auto packet = makeBoxedBatch(expected);
  cuda::solveBoxedLcpRedBlackGaussSeidelBatchCuda(packet);

  expectNearExpectedSolution(packet, expected, 1e-8);
}

//==============================================================================
TEST(
    CudaLcpRedBlackGaussSeidelBatch,
    VariableSizeSyntheticBatchesSatisfyLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const SyntheticCudaFamily family :
       {SyntheticCudaFamily::Standard,
        SyntheticCudaFamily::Boxed,
        SyntheticCudaFamily::FrictionIndex}) {
    SCOPED_TRACE(std::string("family=") + getSyntheticCudaFamilyName(family));
    auto fixture = makeSyntheticGroupedBatch(family, 2, 256);

    cuda::solveBoxedLcpRedBlackGaussSeidelGroupedBatchCuda(fixture.packets);

    expectGroupedBatchSatisfiesLcpContract(fixture);
  }
}

//==============================================================================
TEST(CudaLcpRedBlackGaussSeidelBatch, WorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::string errorMessage;
  auto fixture = makeWorldContactBatch(4, 4, 256, errorMessage);
  ASSERT_TRUE(fixture.has_value()) << errorMessage;

  cuda::solveBoxedLcpRedBlackGaussSeidelBatchCuda(fixture->packet);

  expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
}

//==============================================================================
TEST(CudaLcpPgsBatch, StandardBatchMatchesKnownSolutions)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector<double> expected;
  auto packet = makeStandardBatch(expected);
  cuda::solveBoxedLcpPgsBatchCuda(packet);

  expectNearExpectedSolution(packet, expected, 1e-8);
}

//==============================================================================
TEST(CudaLcpPgsBatch, FrictionIndexBatchMatchesKnownSolutions)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector<double> expected;
  auto packet = makeFrictionIndexBatch(expected);
  cuda::solveBoxedLcpPgsBatchCuda(packet);

  expectNearExpectedSolution(packet, expected, 1e-8);
}

//==============================================================================
TEST(CudaLcpPgsBatch, VariableSizeSyntheticStandardBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerProblemArg : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerProblemArg=" + std::to_string(variantsPerProblemArg));
    auto fixture = makeSyntheticGroupedBatch(
        SyntheticCudaFamily::Standard, variantsPerProblemArg, 256);

    cuda::solveBoxedLcpPgsGroupedBatchCuda(fixture.packets);

    expectGroupedBatchSatisfiesLcpContract(fixture);
  }
}

//==============================================================================
TEST(CudaLcpPgsBatch, VariableSizeSyntheticBoxedBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerProblemArg : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerProblemArg=" + std::to_string(variantsPerProblemArg));
    auto fixture = makeSyntheticGroupedBatch(
        SyntheticCudaFamily::Boxed, variantsPerProblemArg, 256);

    cuda::solveBoxedLcpPgsGroupedBatchCuda(fixture.packets);

    expectGroupedBatchSatisfiesLcpContract(fixture);
  }
}

//==============================================================================
TEST(
    CudaLcpPgsBatch,
    VariableSizeSyntheticFrictionIndexBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerProblemArg : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerProblemArg=" + std::to_string(variantsPerProblemArg));
    auto fixture = makeSyntheticGroupedBatch(
        SyntheticCudaFamily::FrictionIndex, variantsPerProblemArg, 256);

    cuda::solveBoxedLcpPgsGroupedBatchCuda(fixture.packets);

    expectGroupedBatchSatisfiesLcpContract(fixture);
  }
}

//==============================================================================
TEST(CudaLcpPgsBatch, LargerSyntheticBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  const std::array<std::pair<SyntheticCudaFamily, int>, 9> cases{{
      {SyntheticCudaFamily::Standard, 128},
      {SyntheticCudaFamily::Boxed, 128},
      {SyntheticCudaFamily::FrictionIndex, 48},
      {SyntheticCudaFamily::Standard, 192},
      {SyntheticCudaFamily::Boxed, 192},
      {SyntheticCudaFamily::FrictionIndex, 64},
      {SyntheticCudaFamily::Standard, 256},
      {SyntheticCudaFamily::Boxed, 256},
      {SyntheticCudaFamily::FrictionIndex, 96},
  }};
  for (const auto& [family, arg] : cases) {
    SCOPED_TRACE(
        std::string("family=") + getSyntheticCudaFamilyName(family)
        + " arg=" + std::to_string(arg));
    auto fixture = makeSyntheticBatch(family, arg, 4, 256);

    cuda::solveBoxedLcpPgsBatchCuda(fixture.packet);

    expectBatchSatisfiesLcpContract(fixture.packet, fixture.problems);
  }
}

//==============================================================================
TEST(CudaLcpPgsBatch, WorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::string errorMessage;
  auto fixture = makeWorldContactBatch(4, 4, 256, errorMessage);
  ASSERT_TRUE(fixture.has_value()) << errorMessage;

  cuda::solveBoxedLcpPgsBatchCuda(fixture->packet);

  expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
}

//==============================================================================
TEST(CudaLcpPgsBatch, LargerWorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::string errorMessage;
  auto fixture = makeWorldContactBatch(8, 4, 256, errorMessage);
  ASSERT_TRUE(fixture.has_value()) << errorMessage;

  cuda::solveBoxedLcpPgsBatchCuda(fixture->packet);

  expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
}

//==============================================================================
TEST(CudaLcpPgsBatch, DenserWorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int contactCount : {16, 24, 32}) {
    SCOPED_TRACE("contactCount=" + std::to_string(contactCount));
    std::string errorMessage;
    auto fixture = makeWorldContactBatch(contactCount, 4, 256, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpPgsBatchCuda(fixture->packet);

    expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
  }
}

//==============================================================================
TEST(CudaLcpPgsBatch, DenseBoxWorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  constexpr std::array<int, 6> kBoxCounts{1, 16, 24, 32, 48, 64};
  for (const int boxCount : kBoxCounts) {
    SCOPED_TRACE("boxCount=" + std::to_string(boxCount));
    std::string errorMessage;
    auto fixture = makeWorldBoxContactBatch(boxCount, 4, 1024, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpPgsBatchCuda(fixture->packet);

    expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
  }
}

//==============================================================================
TEST(CudaLcpDenseBoxFixture, LargerGridKeepsFaceContactShape)
{
  constexpr int boxCount = 128;
  std::string errorMessage;
  const auto fixture = makeWorldBoxContactProblem(boxCount, 0, errorMessage);
  ASSERT_TRUE(fixture.has_value()) << errorMessage;

  constexpr Eigen::Index expectedRows = 12 * boxCount;
  EXPECT_EQ(fixture->contactCount, static_cast<std::size_t>(4 * boxCount));
  EXPECT_EQ(fixture->problem.b.size(), expectedRows);
  EXPECT_EQ(fixture->problem.A.rows(), expectedRows);
  EXPECT_EQ(fixture->problem.A.cols(), expectedRows);
  EXPECT_EQ((fixture->problem.findex.array() < 0).count(), 4 * boxCount);
}

//==============================================================================
// Execute a large dense box-face fixture as a single-problem CUDA batch. The
// 128-box shape boundary is covered by CudaLcpDenseBoxFixture above.
TEST(
    CudaLcpPgsBatch,
    DenseBoxWorldContactLargeRuntimeFixtureSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  constexpr int boxCount = 64;
  std::string errorMessage;
  auto fixture = makeWorldBoxContactBatch(boxCount, 1, 1024, errorMessage);
  ASSERT_TRUE(fixture.has_value()) << errorMessage;

  cuda::solveBoxedLcpPgsBatchCuda(fixture->packet);

  expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
}

//==============================================================================
TEST(CudaLcpPgsBatch, DenseBoxWorldContactGroupedBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::string errorMessage;
  auto fixture = makeWorldBoxContactGroupedBatch(2, 1024, errorMessage, 48);
  ASSERT_TRUE(fixture.has_value()) << errorMessage;

  cuda::solveBoxedLcpPgsGroupedBatchCuda(fixture->packets);

  expectGroupedBatchSatisfiesLcpContract(*fixture);
}

//==============================================================================
TEST(CudaLcpPgsBatch, HomogeneousStackedWorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int sphereCount :
       {5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 24, 32}) {
    SCOPED_TRACE("sphereCount=" + std::to_string(sphereCount));
    std::string errorMessage;
    auto fixture
        = makeWorldStackContactBatch(sphereCount, 4, 8192, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpPgsBatchCuda(fixture->packet);

    expectBatchSatisfiesLcpContract(fixture->packet, fixture->problems);
  }
}

//==============================================================================
TEST(CudaLcpPgsBatch, VariableSizeWorldContactBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerContactCount : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerContactCount=" + std::to_string(variantsPerContactCount));
    std::string errorMessage;
    auto fixture = makeWorldContactGroupedBatch(
        variantsPerContactCount, 256, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpPgsGroupedBatchCuda(fixture->packets);

    expectGroupedBatchSatisfiesLcpContract(*fixture);
  }
}

//==============================================================================
TEST(CudaLcpPgsBatch, StackedWorldContactGroupedBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerSphereCount : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerSphereCount=" + std::to_string(variantsPerSphereCount));
    std::string errorMessage;
    auto fixture = makeWorldStackContactGroupedBatch(
        variantsPerSphereCount, 8192, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpPgsGroupedBatchCuda(fixture->packets);

    expectGroupedBatchSatisfiesLcpContract(*fixture);
  }
}

//==============================================================================
TEST(CudaLcpPgsBatch, ArticulatedUnifiedContactGroupedBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerContactCount : {2, 3}) {
    SCOPED_TRACE(
        "variantsPerContactCount=" + std::to_string(variantsPerContactCount));
    std::string errorMessage;
    auto fixture = makeArticulatedUnifiedContactGroupedBatch(
        variantsPerContactCount, 512, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpPgsGroupedBatchCuda(fixture->packets);

    expectGroupedBatchSatisfiesLcpContract(*fixture);
  }
}

//==============================================================================
TEST(CudaLcpPgsBatch, MixedContactGroupedBatchSatisfiesLcpContract)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  for (const int variantsPerScenario : {2, 3}) {
    SCOPED_TRACE("variantsPerScenario=" + std::to_string(variantsPerScenario));
    std::string errorMessage;
    auto fixture
        = makeMixedContactGroupedBatch(variantsPerScenario, 512, errorMessage);
    ASSERT_TRUE(fixture.has_value()) << errorMessage;

    cuda::solveBoxedLcpPgsGroupedBatchCuda(fixture->packets);

    expectGroupedBatchSatisfiesLcpContract(*fixture);
  }
}

//==============================================================================
TEST(CudaLcpPgsBatch, BoxedBatchMatchesKnownSolutions)
{
  if (!cuda::isCudaRuntimeAvailable()) {
    GTEST_SKIP() << "CUDA runtime has no available device";
  }

  std::vector<double> expected;
  auto packet = makeBoxedBatch(expected);
  cuda::solveBoxedLcpPgsBatchCuda(packet);

  expectNearExpectedSolution(packet, expected, 1e-8);
}
