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
 *   INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 *   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 *   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/body/collision_shape.hpp>
#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/rigid_avbd/rigid_block_kernel.hpp>
#include <dart/simulation/detail/rigid_avbd/rigid_world_contact.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/multibody/joint.hpp>
#include <dart/simulation/multibody/link.hpp>
#include <dart/simulation/multibody/multibody.hpp>
#include <dart/simulation/world.hpp>

#include <dart/common/memory_manager.hpp>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <algorithm>
#include <iterator>
#include <limits>
#include <optional>
#include <string_view>
#include <vector>

#include <cmath>

namespace vbd = dart::simulation::detail::deformable_vbd;

namespace {

using Vec3 = Eigen::Vector3d;
namespace common = dart::common;
namespace sx = dart::simulation;

class CountingMemoryAllocator final : public common::MemoryAllocator
{
public:
  std::string_view getType() const override
  {
    return "CountingMemoryAllocator";
  }

  void* allocate(std::size_t bytes) noexcept override
  {
    ++allocations;
    return common::MemoryAllocator::GetDefault().allocate(bytes);
  }

  void* allocate(std::size_t bytes, std::size_t alignment) noexcept override
  {
    ++allocations;
    return common::MemoryAllocator::GetDefault().allocate(bytes, alignment);
  }

  void deallocate(void* pointer, std::size_t bytes) override
  {
    common::MemoryAllocator::GetDefault().deallocate(pointer, bytes);
  }

  void deallocate(
      void* pointer, std::size_t bytes, std::size_t alignment) override
  {
    common::MemoryAllocator::GetDefault().deallocate(pointer, bytes, alignment);
  }

  std::size_t allocations = 0u;
};

//==============================================================================
Eigen::Quaterniond rotationX(double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vec3::UnitX()));
}

//==============================================================================
Eigen::Quaterniond rotationY(double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vec3::UnitY()));
}

//==============================================================================
Eigen::Quaterniond rotationZ(double angle)
{
  return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Vec3::UnitZ()));
}

//==============================================================================
template <typename EntityVector>
std::size_t findEntityIndex(const EntityVector& entities, entt::entity entity)
{
  const auto it = std::find(entities.begin(), entities.end(), entity);
  EXPECT_NE(it, entities.end());
  return static_cast<std::size_t>(std::distance(entities.begin(), it));
}

} // namespace

//==============================================================================
TEST(AvbdRigidBlock, NormalizeRigidOrientationKeepsUnitAndRejectsInvalid)
{
  const Eigen::Quaterniond unit = Eigen::Quaterniond::Identity();
  const Eigen::Quaterniond normalizedUnit
      = vbd::normalizeAvbdRigidOrientation(unit);

  EXPECT_DOUBLE_EQ(normalizedUnit.w(), 1.0);
  EXPECT_DOUBLE_EQ(normalizedUnit.x(), 0.0);
  EXPECT_DOUBLE_EQ(normalizedUnit.y(), 0.0);
  EXPECT_DOUBLE_EQ(normalizedUnit.z(), 0.0);

  const Eigen::Quaterniond scaled(2.0, 0.0, 0.0, 0.0);
  const Eigen::Quaterniond normalizedScaled
      = vbd::normalizeAvbdRigidOrientation(scaled);
  EXPECT_NEAR(normalizedScaled.norm(), 1.0, 1e-15);
  EXPECT_DOUBLE_EQ(normalizedScaled.w(), 1.0);

  const Eigen::Quaterniond zero(0.0, 0.0, 0.0, 0.0);
  const Eigen::Quaterniond normalizedZero
      = vbd::normalizeAvbdRigidOrientation(zero);
  EXPECT_DOUBLE_EQ(normalizedZero.w(), 1.0);
  EXPECT_DOUBLE_EQ(normalizedZero.x(), 0.0);

  const Eigen::Quaterniond invalid(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
  const Eigen::Quaterniond normalizedInvalid
      = vbd::normalizeAvbdRigidOrientation(invalid);
  EXPECT_DOUBLE_EQ(normalizedInvalid.w(), 1.0);
  EXPECT_DOUBLE_EQ(normalizedInvalid.x(), 0.0);
}

//==============================================================================
TEST(AvbdRigidBlock, BodyWorldPointKeepsOriginAnchorAtPosition)
{
  vbd::AvbdRigidBodyState state;
  state.position = Vec3(1.0, -2.0, 3.0);
  state.orientation = rotationZ(0.25 * vbd::kAvbdRigidPi);

  const Vec3 originWorld = vbd::avbdRigidBodyWorldPoint(state, Vec3::Zero());

  EXPECT_EQ(originWorld, state.position);

  const Vec3 offsetWorld = vbd::avbdRigidBodyWorldPoint(state, Vec3::UnitX());

  EXPECT_GT((offsetWorld - state.position).norm(), 0.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidStepUpdatesTranslationAndOrientation)
{
  vbd::AvbdRigidBodyState state;

  vbd::Vector6d step = vbd::Vector6d::Zero();
  step.head<3>() = Vec3(1.0, -2.0, 3.0);
  step.tail<3>() = 0.5 * vbd::kAvbdRigidPi * Vec3::UnitZ();

  vbd::applyAvbdRigidBodyStep(state, step);

  EXPECT_DOUBLE_EQ(state.position.x(), 1.0);
  EXPECT_DOUBLE_EQ(state.position.y(), -2.0);
  EXPECT_DOUBLE_EQ(state.position.z(), 3.0);
  EXPECT_NEAR(
      (state.orientation.toRotationMatrix()
       - rotationZ(0.5 * vbd::kAvbdRigidPi).toRotationMatrix())
          .norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(state.orientation.norm(), 1.0, 1e-15);
}

//==============================================================================
TEST(AvbdRigidBlock, InertiaTermSolvesBackToInertialTarget)
{
  vbd::AvbdRigidBodyState state;
  state.position = Vec3(1.0, -2.0, 0.5);
  state.orientation = rotationZ(0.2);

  vbd::AvbdRigidBodyState target;
  target.position = Vec3::Zero();
  target.orientation = Eigen::Quaterniond::Identity();

  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia.diagonal() = Vec3(4.0, 5.0, 6.0);

  vbd::AvbdRigidBodyBlock block;
  vbd::addAvbdRigidBodyInertiaTerm(
      block, /*mass=*/2.0, inertia, /*timeStep=*/0.5, state, target);
  const vbd::Vector6d step = vbd::solveAvbdRigidBodyBlock(block);

  EXPECT_NEAR((step.head<3>() + state.position).norm(), 0.0, 1e-12);
  EXPECT_NEAR(step.tail<3>().x(), 0.0, 1e-12);
  EXPECT_NEAR(step.tail<3>().y(), 0.0, 1e-12);
  EXPECT_NEAR(step.tail<3>().z(), -0.2, 1e-12);

  vbd::applyAvbdRigidBodyStep(state, step);
  EXPECT_NEAR(state.position.norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (state.orientation.toRotationMatrix() - Eigen::Matrix3d::Identity())
          .norm(),
      0.0,
      1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, LowerTriangleInertiaTermMatchesFullForScaledOrientations)
{
  vbd::AvbdRigidBodyState state;
  state.position = Vec3(1.0, -2.0, 0.5);
  state.orientation = rotationZ(0.4);
  state.orientation.coeffs() *= 3.0;

  vbd::AvbdRigidBodyState target;
  target.position = Vec3(-0.5, 0.25, 1.5);
  target.orientation = rotationZ(-0.35);
  target.orientation.coeffs() *= 2.0;

  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  inertia.diagonal() = Vec3(4.0, 5.0, 6.0);

  vbd::AvbdRigidBodyBlock fullBlock;
  vbd::addAvbdRigidBodyInertiaTerm(
      fullBlock, /*mass=*/2.0, inertia, /*timeStep=*/0.5, state, target);

  vbd::AvbdRigidBodyBlock lowerBlock;
  vbd::addAvbdRigidBodyInertiaTermLowerTriangle(
      lowerBlock, /*mass=*/2.0, inertia, /*timeStep=*/0.5, state, target);

  EXPECT_NEAR((fullBlock.force - lowerBlock.force).norm(), 0.0, 1e-12);
  for (int row = 0; row < fullBlock.hessian.rows(); ++row) {
    for (int col = 0; col <= row; ++col) {
      EXPECT_NEAR(
          lowerBlock.hessian(row, col), fullBlock.hessian(row, col), 1e-12);
    }
  }
}

//==============================================================================
TEST(AvbdRigidBlock, PointAttachmentStampsForceTorqueAndPsdHessian)
{
  vbd::AvbdRigidBodyState state;

  vbd::AvbdRigidPointAttachmentRow row;
  row.localPoint = Vec3::UnitY();
  row.target = Vec3(1.0, 1.0, 0.0);
  row.axis = Vec3::UnitX();
  row.state.stiffness = 40.0;

  vbd::AvbdRigidBodyBlock block;
  const double forceMagnitude
      = vbd::addAvbdRigidPointAttachment(block, state, row, /*alpha=*/0.0);

  vbd::Vector6d expectedDirection = vbd::Vector6d::Zero();
  expectedDirection.head<3>() = Vec3::UnitX();
  expectedDirection.tail<3>() = Vec3(0.0, 0.0, -1.0);

  EXPECT_DOUBLE_EQ(forceMagnitude, 40.0);
  EXPECT_NEAR((block.force - 40.0 * expectedDirection).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (block.hessian - 40.0 * expectedDirection * expectedDirection.transpose())
          .norm(),
      0.0,
      1e-12);
  EXPECT_GE(
      block.hessian.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),
      -1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, PointAttachmentOriginAnchorDirectionStaysTranslational)
{
  vbd::AvbdRigidBodyState state;
  state.position = Vec3(-1.0, 0.25, 2.0);
  state.orientation = rotationX(0.25 * vbd::kAvbdRigidPi);

  vbd::AvbdRigidPointAttachmentRow row;
  row.axis = Vec3(0.5, -1.0, 2.0).normalized();

  const vbd::Vector6d direction
      = vbd::avbdRigidPointAttachmentDirection(state, row);

  vbd::Vector6d expected = vbd::Vector6d::Zero();
  expected.head<3>() = row.axis;

  EXPECT_EQ(direction, expected);
}

//==============================================================================
TEST(AvbdRigidBlock, PointAttachmentDualUpdateGrowsInsideBounds)
{
  vbd::AvbdRigidBodyState state;

  vbd::AvbdRigidPointAttachmentRow row;
  row.target = Vec3::UnitX();
  row.axis = Vec3::UnitX();
  row.state.stiffness = 10.0;
  row.state.lambda = 1.0;

  vbd::AvbdRigidPointAttachmentOptions options;
  options.alpha = 0.0;
  options.beta = 3.0;

  const vbd::AvbdScalarRowState updated
      = vbd::updateAvbdRigidPointAttachmentRow(row.state, state, row, options);

  EXPECT_DOUBLE_EQ(updated.lambda, 11.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 13.0);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairStampsEqualAndOppositeRigidDirections)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3::UnitX();

  vbd::AvbdRigidPointPairRow row;
  row.axis = Vec3::UnitX();
  row.state.stiffness = 25.0;

  vbd::AvbdRigidBodyBlock blockA;
  vbd::AvbdRigidBodyBlock blockB;
  const double forceMagnitude = vbd::addAvbdRigidPointPair(
      blockA, blockB, stateA, stateB, row, /*alpha=*/0.0);

  vbd::Vector6d expectedA = vbd::Vector6d::Zero();
  expectedA.head<3>() = Vec3::UnitX();

  vbd::Vector6d expectedB = vbd::Vector6d::Zero();
  expectedB.head<3>() = -Vec3::UnitX();

  EXPECT_DOUBLE_EQ(forceMagnitude, 25.0);
  EXPECT_NEAR((blockA.force - 25.0 * expectedA).norm(), 0.0, 1e-12);
  EXPECT_NEAR((blockB.force - 25.0 * expectedB).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (blockA.hessian - 25.0 * expectedA * expectedA.transpose()).norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(
      (blockB.hessian - 25.0 * expectedB * expectedB.transpose()).norm(),
      0.0,
      1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairIncludesTorqueDirections)
{
  vbd::AvbdRigidBodyState stateA;
  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(0.0, 1.0, 0.0);

  vbd::AvbdRigidPointPairRow row;
  row.localPointA = Vec3::UnitY();
  row.localPointB = -Vec3::UnitY();
  row.axis = Vec3::UnitX();

  const vbd::Vector6d firstDirection
      = vbd::avbdRigidPointPairDirectionA(stateA, row);
  const vbd::Vector6d secondDirection
      = vbd::avbdRigidPointPairDirectionB(stateB, row);

  EXPECT_NEAR((firstDirection.head<3>() - Vec3::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (firstDirection.tail<3>() - Vec3(0.0, 0.0, -1.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR((secondDirection.head<3>() + Vec3::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (secondDirection.tail<3>() - Vec3(0.0, 0.0, -1.0)).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairOriginAnchorDirectionStaysTranslational)
{
  vbd::AvbdRigidBodyState stateA;
  stateA.position = Vec3(0.5, -1.0, 2.0);
  stateA.orientation = rotationZ(0.25 * vbd::kAvbdRigidPi);

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(-2.0, 1.0, 0.25);
  stateB.orientation = rotationY(0.5 * vbd::kAvbdRigidPi);

  vbd::AvbdRigidPointPairRow row;
  row.axis = Vec3(1.0, -2.0, 0.5).normalized();

  const vbd::Vector6d firstDirection
      = vbd::avbdRigidPointPairDirectionA(stateA, row);
  const vbd::Vector6d secondDirection
      = vbd::avbdRigidPointPairDirectionB(stateB, row);

  vbd::Vector6d expectedFirst = vbd::Vector6d::Zero();
  expectedFirst.head<3>() = row.axis;
  vbd::Vector6d expectedSecond = vbd::Vector6d::Zero();
  expectedSecond.head<3>() = -row.axis;

  EXPECT_EQ(firstDirection, expectedFirst);
  EXPECT_EQ(secondDirection, expectedSecond);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairDualUpdateUsesBounds)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3::UnitX();

  vbd::AvbdRigidPointPairRow row;
  row.axis = Vec3::UnitX();
  row.state.stiffness = 10.0;
  row.bounds.lower = 0.0;
  row.bounds.upper = 2.0;

  vbd::AvbdRigidPointAttachmentOptions options;
  options.beta = 4.0;

  const vbd::AvbdScalarRowState updated = vbd::updateAvbdRigidPointPairRow(
      row.state, stateA, stateB, row, options);

  EXPECT_DOUBLE_EQ(updated.lambda, 2.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 10.0);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairFiniteUpdateKeepsCappedStiffness)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3::UnitX();

  vbd::AvbdRigidPointPairRow row;
  row.axis = Vec3::UnitX();
  row.materialStiffness = 20.0;
  row.state.stiffness = 20.0;
  row.state.lambda = 99.0;

  vbd::AvbdRigidPointAttachmentOptions options;
  options.beta = 4.0;
  options.maxStiffness = 12.0;

  const vbd::AvbdScalarRowState updated = vbd::updateAvbdRigidPointPairRow(
      row.state, stateA, stateB, row, options);

  EXPECT_DOUBLE_EQ(updated.lambda, 0.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 12.0);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairDistanceSpringStampsRadialFiniteStiffness)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = 2.0 * Vec3::UnitX();

  vbd::AvbdRigidPointPairDistanceSpringRow row;
  row.restLength = 1.0;
  row.state.stiffness = 8.0;

  vbd::AvbdRigidBodyBlock blockA;
  vbd::AvbdRigidBodyBlock blockB;
  const double forceMagnitude = vbd::addAvbdRigidPointPairDistanceSpring(
      blockA, blockB, stateA, stateB, row);

  vbd::Vector6d expectedA = vbd::Vector6d::Zero();
  expectedA.head<3>() = Vec3::UnitX();

  vbd::Vector6d expectedB = vbd::Vector6d::Zero();
  expectedB.head<3>() = -Vec3::UnitX();

  vbd::Matrix6d expectedHessian = vbd::Matrix6d::Zero();
  expectedHessian.topLeftCorner<3, 3>().diagonal() = Vec3(8.0, 4.0, 4.0);

  EXPECT_DOUBLE_EQ(
      vbd::avbdRigidPointPairDistanceSpringConstraintValue(stateA, stateB, row),
      1.0);
  EXPECT_DOUBLE_EQ(forceMagnitude, 8.0);
  EXPECT_NEAR((blockA.force - 8.0 * expectedA).norm(), 0.0, 1e-12);
  EXPECT_NEAR((blockB.force - 8.0 * expectedB).norm(), 0.0, 1e-12);
  EXPECT_NEAR((blockA.hessian - expectedHessian).norm(), 0.0, 1e-12);
  EXPECT_NEAR((blockB.hessian - expectedHessian).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, DistanceSpringOriginAnchorHessianStaysTranslational)
{
  vbd::AvbdRigidBodyState state;
  state.position = Vec3(1.0, -2.0, 0.5);
  state.orientation = rotationY(0.25 * vbd::kAvbdRigidPi);

  const Vec3 axis = Vec3(1.0, 2.0, 0.5).normalized();
  const double length = 2.0;
  const double restLength = 1.0;
  const double stiffness = 6.0;

  vbd::AvbdRigidBodyBlock block;
  vbd::addAvbdRigidDistanceSpringHessianAtWorldPoint(
      block,
      state,
      state.position,
      axis,
      length,
      restLength,
      stiffness,
      /*clampToPsd=*/true);

  const Eigen::Matrix3d nnT = axis * axis.transpose();
  const Eigen::Matrix3d pointHessian
      = stiffness
        * (nnT
           + (1.0 - restLength / length) * (Eigen::Matrix3d::Identity() - nnT));
  vbd::Matrix6d expectedHessian = vbd::Matrix6d::Zero();
  expectedHessian.topLeftCorner<3, 3>() = pointHessian;

  EXPECT_NEAR((block.hessian - expectedHessian).norm(), 0.0, 1e-12);
  const double translationAngularNorm
      = block.hessian.topRightCorner<3, 3>().norm();
  const double angularNorm = block.hessian.bottomRightCorner<3, 3>().norm();
  EXPECT_NEAR(translationAngularNorm, 0.0, 1e-12);
  EXPECT_NEAR(angularNorm, 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, DistanceSpringOriginAnchorDirectionStaysTranslational)
{
  vbd::AvbdRigidBodyState state;
  state.position = Vec3(-0.5, 1.0, 2.0);
  state.orientation = rotationY(0.25 * vbd::kAvbdRigidPi);

  const Vec3 axis = Vec3(0.25, -1.0, 2.0).normalized();
  const vbd::Vector6d direction
      = vbd::avbdRigidDistanceSpringDirectionAtWorldPoint(
          state, state.position, axis);

  vbd::Vector6d expected = vbd::Vector6d::Zero();
  expected.head<3>() = axis;
  EXPECT_EQ(direction, expected);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairDistanceSpringUpdateRampsWithoutDual)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(3.0, 4.0, 0.0);

  vbd::AvbdRigidPointPairDistanceSpringRow row;
  row.restLength = 2.0;
  row.materialStiffness = 20.0;
  row.state.stiffness = 4.0;
  row.state.lambda = 99.0;

  vbd::AvbdRigidPointPairDistanceSpringOptions options;
  options.beta = 2.0;
  options.maxStiffness = 9.0;

  const vbd::AvbdScalarRowState updated
      = vbd::updateAvbdRigidPointPairDistanceSpringRow(
          row.state, stateA, stateB, row, options);

  EXPECT_DOUBLE_EQ(updated.lambda, 0.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 9.0);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairDistanceSpringUpdateKeepsCappedStiffness)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(3.0, 4.0, 0.0);

  vbd::AvbdRigidPointPairDistanceSpringRow row;
  row.restLength = 2.0;
  row.materialStiffness = 20.0;
  row.state.stiffness = 20.0;
  row.state.lambda = 99.0;

  vbd::AvbdRigidPointPairDistanceSpringOptions options;
  options.beta = 2.0;
  options.maxStiffness = 12.0;

  const vbd::AvbdScalarRowState updated
      = vbd::updateAvbdRigidPointPairDistanceSpringRow(
          row.state, stateA, stateB, row, options);

  EXPECT_DOUBLE_EQ(updated.lambda, 0.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 12.0);
}

//==============================================================================
TEST(AvbdRigidBlock, PointPairDistanceSpringStepReducesStretch)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = 2.0 * Vec3::UnitX();
  const vbd::AvbdRigidBodyState inertialTargetB = stateB;

  vbd::AvbdRigidPointPairDistanceSpringRow row;
  row.restLength = 1.0;
  row.state.stiffness = 8.0;

  vbd::AvbdRigidBodyBlock blockA;
  vbd::AvbdRigidBodyBlock blockB;
  vbd::addAvbdRigidBodyInertiaTerm(
      blockB,
      /*mass=*/1.0,
      Eigen::Matrix3d::Identity(),
      /*timeStep=*/1.0,
      stateB,
      inertialTargetB);
  vbd::addAvbdRigidPointPairDistanceSpring(blockA, blockB, stateA, stateB, row);

  const double initialDistance
      = vbd::avbdRigidPointPairDistanceSpringRelativePosition(
            stateA, stateB, row)
            .norm();
  vbd::applyAvbdRigidBodyStep(stateB, vbd::solveAvbdRigidBodyBlock(blockB));
  const double finalDistance
      = vbd::avbdRigidPointPairDistanceSpringRelativePosition(
            stateA, stateB, row)
            .norm();

  EXPECT_LT(finalDistance, initialDistance);
  EXPECT_GT(finalDistance, row.restLength);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidDistanceSpringBuilderUsesScratchForLargeInputs)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[0].position = Vec3(0.25, -0.1, 0.05);
  states[1].position = Vec3(1.5, 0.3, -0.2);

  constexpr std::size_t kActiveSprings
      = vbd::detail::kAvbdRigidSmallRowStackCapacity + 1u;
  std::vector<vbd::AvbdRigidBodyPointPairDistanceSpringRow> springs(
      kActiveSprings + 1u);
  for (std::size_t i = 0; i < kActiveSprings; ++i) {
    auto& spring = springs[i];
    spring.bodyA = 0;
    spring.bodyB = 1;
    spring.endpointA
        = {11u + static_cast<std::uint32_t>(i),
           vbd::packAvbdContactFeatureId(
               vbd::AvbdContactFeatureKind::Vertex,
               static_cast<std::uint32_t>(i))};
    spring.endpointB
        = {23u,
           vbd::packAvbdContactFeatureId(
               vbd::AvbdContactFeatureKind::Vertex,
               static_cast<std::uint32_t>(i + 1u))};
    spring.rowIndex = static_cast<std::uint32_t>(i);
    spring.row.localPointA = Vec3(0.01 * static_cast<double>(i), 0.0, 0.0);
    spring.row.localPointB = Vec3(0.0, 0.02 * static_cast<double>(i), 0.0);
    spring.row.restLength = 0.75 + 0.01 * static_cast<double>(i);
    spring.row.materialStiffness = 20.0 + static_cast<double>(i);
    spring.startStiffness = 2.0;
    spring.maxStiffness = 80.0;
  }
  springs.back() = springs.front();
  springs.back().bodyB = 99u;

  vbd::AvbdScalarRowInventory springInventory;
  std::vector<vbd::AvbdRigidBodyPointPairDistanceSpringRow> springRows;
  vbd::AvbdRigidDistanceSpringRowScratch scratch;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  vbd::buildAvbdRigidDistanceSpringRows(
      states, springs, springInventory, springRows, scratch, warmStart);

  ASSERT_EQ(scratch.activeRows.size(), kActiveSprings);
  ASSERT_EQ(springInventory.size(), kActiveSprings);
  ASSERT_EQ(springRows.size(), kActiveSprings);
  EXPECT_EQ(springRows.back().rowIndex, kActiveSprings - 1u);
  EXPECT_EQ(
      springInventory[0].descriptor.key.role,
      vbd::AvbdScalarRowRole::RigidDistanceSpring);
  EXPECT_DOUBLE_EQ(springRows[0].row.materialStiffness, 20.0);
  EXPECT_DOUBLE_EQ(
      springRows.back().row.materialStiffness,
      20.0 + static_cast<double>(kActiveSprings - 1u));

  springInventory[0].state.stiffness = 30.0;
  springInventory[0].state.lambda = 3.0;
  springs[0].row.materialStiffness = 25.0;
  vbd::buildAvbdRigidDistanceSpringRows(
      states, springs, springInventory, springRows, scratch, warmStart);

  ASSERT_EQ(springInventory.size(), kActiveSprings);
  ASSERT_EQ(springRows.size(), kActiveSprings);
  EXPECT_DOUBLE_EQ(springInventory[0].state.stiffness, 25.0);
  EXPECT_DOUBLE_EQ(springInventory[0].state.lambda, 0.0);
  EXPECT_DOUBLE_EQ(springRows[0].row.materialStiffness, 25.0);
}

//==============================================================================
TEST(AvbdRigidBlock, ContactNormalPointPairUsesGapOffsetAndBounds)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(0.1, 0.0, 0.0);

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 100.0;

  const vbd::AvbdRigidPointPairRow row = vbd::makeAvbdRigidContactNormalRow(
      Vec3::Zero(),
      Vec3::Zero(),
      -Vec3::UnitX(),
      /*targetDistance=*/0.2,
      rowState);

  EXPECT_DOUBLE_EQ(row.bounds.lower, 0.0);
  EXPECT_TRUE(std::isinf(row.bounds.upper));
  EXPECT_NEAR(
      vbd::avbdRigidPointPairConstraintValue(stateA, stateB, row), 0.1, 1e-12);

  vbd::AvbdRigidBodyBlock blockA;
  vbd::AvbdRigidBodyBlock blockB;
  const double forceMagnitude = vbd::addAvbdRigidPointPair(
      blockA, blockB, stateA, stateB, row, /*alpha=*/0.0);

  EXPECT_DOUBLE_EQ(forceMagnitude, 10.0);
  EXPECT_NEAR(
      (blockA.force.head<3>() - Vec3(-10.0, 0.0, 0.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (blockB.force.head<3>() - Vec3(10.0, 0.0, 0.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(blockA.force.tail<3>().norm(), 0.0, 1e-12);
  EXPECT_NEAR(blockB.force.tail<3>().norm(), 0.0, 1e-12);
  EXPECT_GE(
      blockA.hessian.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),
      -1e-12);
  EXPECT_GE(
      blockB.hessian.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),
      -1e-12);

  stateB.position = Vec3(0.3, 0.0, 0.0);
  const vbd::AvbdScalarRowState separated = vbd::updateAvbdRigidPointPairRow(
      row.state, stateA, stateB, row, vbd::AvbdRigidPointAttachmentOptions{});
  EXPECT_DOUBLE_EQ(separated.lambda, 0.0);
  EXPECT_DOUBLE_EQ(separated.stiffness, row.state.stiffness);
}

//==============================================================================
TEST(AvbdRigidBlock, ContactFrictionPointPairUsesLaggedRelativeOffset)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(0.0, 0.4, 0.0);

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 100.0;

  const vbd::AvbdRigidPointPairRow row
      = vbd::makeAvbdRigidContactFrictionTangentRow(
          Vec3::Zero(),
          Vec3::Zero(),
          2.0 * Vec3::UnitY(),
          Vec3(0.0, 0.1, 0.0),
          /*forceLimit=*/5.0,
          rowState);

  EXPECT_NEAR(row.axis.norm(), 1.0, 1e-12);
  EXPECT_NEAR(row.offset, -0.1, 1e-12);
  EXPECT_DOUBLE_EQ(row.bounds.lower, -5.0);
  EXPECT_DOUBLE_EQ(row.bounds.upper, 5.0);
  EXPECT_NEAR(
      vbd::avbdRigidPointPairConstraintValue(stateA, stateB, row), 0.3, 1e-12);

  vbd::AvbdRigidBodyBlock blockA;
  vbd::AvbdRigidBodyBlock blockB;
  const double forceMagnitude = vbd::addAvbdRigidPointPair(
      blockA, blockB, stateA, stateB, row, /*alpha=*/0.0);

  EXPECT_DOUBLE_EQ(forceMagnitude, 5.0);
  EXPECT_NEAR(
      (blockA.force.head<3>() - Vec3(0.0, 5.0, 0.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (blockB.force.head<3>() - Vec3(0.0, -5.0, 0.0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(blockA.force.tail<3>().norm(), 0.0, 1e-12);
  EXPECT_NEAR(blockB.force.tail<3>().norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, ContactFrictionPointPairProjectsStaticForceToCone)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(1.0, 1.0, 0.0);

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 10.0;

  vbd::AvbdRigidPointPairRow rowX = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitX(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowState);
  vbd::AvbdRigidPointPairRow rowY = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitY(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowState);

  vbd::AvbdRigidPointPairFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  ASSERT_TRUE(
      vbd::avbdRigidPointPairFrictionPreviousDualInsideCone(rowX, rowY));
  bool clamped = false;
  const Eigen::Vector2d force = vbd::avbdRigidPointPairFrictionTangentPairForce(
      stateA, stateB, rowX, rowY, options, &clamped);

  EXPECT_TRUE(clamped);
  EXPECT_NEAR(force.norm(), 5.0, 1e-12);
  EXPECT_NEAR(force.x(), force.y(), 1e-12);
  EXPECT_GT(force.x(), 0.0);

  vbd::AvbdRigidBodyBlock blockA;
  vbd::AvbdRigidBodyBlock blockB;
  const Eigen::Vector2d stampedForce
      = vbd::addAvbdRigidPointPairFrictionTangentPair(
          blockA, blockB, stateA, stateB, rowX, rowY, options);

  EXPECT_NEAR((stampedForce - force).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (blockA.force.head<3>() - Vec3(force.x(), force.y(), 0.0)).norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(
      (blockB.force.head<3>() + Vec3(force.x(), force.y(), 0.0)).norm(),
      0.0,
      1e-12);
  EXPECT_GE(
      blockA.hessian.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),
      -1e-12);
  EXPECT_GE(
      blockB.hessian.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff(),
      -1e-12);

  vbd::updateAvbdRigidPointPairFrictionTangentPair(
      rowX, rowY, stateA, stateB, options);
  EXPECT_NEAR(std::hypot(rowX.state.lambda, rowY.state.lambda), 5.0, 1e-12);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 10.0);
}

//==============================================================================
TEST(AvbdRigidBlock, ContactFrictionPointPairSwitchesToDynamicSlipDirection)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.position = Vec3(0.0, 2.0, 0.0);

  vbd::AvbdScalarRowState rowStateX;
  rowStateX.stiffness = 10.0;
  rowStateX.lambda = 5.0;
  vbd::AvbdRigidPointPairRow rowX = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitX(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowStateX);

  vbd::AvbdScalarRowState rowStateY;
  rowStateY.stiffness = 20.0;
  vbd::AvbdRigidPointPairRow rowY = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitY(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowStateY);

  vbd::AvbdRigidPointPairFrictionOptions options;
  options.alpha = 0.0;
  options.beta = 100.0;

  ASSERT_FALSE(
      vbd::avbdRigidPointPairFrictionPreviousDualInsideCone(rowX, rowY));
  const Eigen::Vector2d force = vbd::avbdRigidPointPairFrictionTangentPairForce(
      stateA, stateB, rowX, rowY, options);
  EXPECT_NEAR(force.x(), 0.0, 1e-12);
  EXPECT_NEAR(force.y(), 5.0, 1e-12);

  vbd::updateAvbdRigidPointPairFrictionTangentPair(
      rowX, rowY, stateA, stateB, options);
  EXPECT_NEAR(rowX.state.lambda, 0.0, 1e-12);
  EXPECT_NEAR(rowY.state.lambda, 5.0, 1e-12);
  EXPECT_DOUBLE_EQ(rowX.state.stiffness, 10.0);
  EXPECT_DOUBLE_EQ(rowY.state.stiffness, 20.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidRowDriverSeparatesContactPair)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = Vec3(0.5, 0.0, 0.0);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias{
      Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 100.0;

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairRow> pointPairs(1);
  pointPairs[0].bodyA = 0;
  pointPairs[0].bodyB = 1;
  pointPairs[0].row = vbd::makeAvbdRigidContactNormalRow(
      Vec3::Zero(),
      Vec3::Zero(),
      -Vec3::UnitX(),
      /*targetDistance=*/1.0,
      rowState);
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularPairs;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionPairs;

  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 1;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;
  vbd::AvbdRigidBodyRowIndexScratch rowIndexScratch;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          pointPairs,
          angularPairs,
          frictionPairs,
          options,
          rowOptions,
          frictionOptions,
          &rowIndexScratch);

  EXPECT_EQ(stats.iterations, 1u);
  EXPECT_EQ(stats.bodyUpdates, 1u);
  EXPECT_TRUE(rowIndexScratch.attachmentRowOffsets.empty());
  EXPECT_TRUE(rowIndexScratch.distanceSpringRowOffsets.empty());
  EXPECT_TRUE(rowIndexScratch.angularPairRowOffsets.empty());
  EXPECT_TRUE(rowIndexScratch.frictionPairRowOffsets.empty());
  ASSERT_EQ(rowIndexScratch.pointPairRowOffsets.size(), 3u);
  EXPECT_EQ(rowIndexScratch.pointPairRowOffsets[0], 0u);
  EXPECT_EQ(rowIndexScratch.pointPairRowOffsets[1], 1u);
  EXPECT_EQ(rowIndexScratch.pointPairRowOffsets[2], 2u);
  ASSERT_EQ(rowIndexScratch.pointPairRowIndices.size(), 2u);
  EXPECT_EQ(rowIndexScratch.pointPairRowIndices[0], 0u);
  EXPECT_EQ(rowIndexScratch.pointPairRowIndices[1], 0u);
  EXPECT_NEAR(states[0].position.norm(), 0.0, 1e-12);
  EXPECT_GT(states[1].position.x(), 0.9);
  EXPECT_GT(pointPairs[0].row.state.lambda, 0.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidRowDriverReducesDistanceSpringStretch)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = 2.0 * Vec3::UnitX();
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias{
      Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairRow> pointPairs;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularPairs;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionPairs;

  std::vector<vbd::AvbdRigidBodyPointPairDistanceSpringRow> distanceSprings(1);
  distanceSprings[0].bodyA = 0;
  distanceSprings[0].bodyB = 1;
  distanceSprings[0].row.restLength = 1.0;
  distanceSprings[0].row.state.stiffness = 8.0;
  distanceSprings[0].row.state.lambda = 42.0;
  distanceSprings[0].row.materialStiffness = 20.0;

  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 1;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;
  vbd::AvbdRigidPointPairDistanceSpringOptions distanceSpringOptions;
  distanceSpringOptions.beta = 2.0;
  distanceSpringOptions.maxStiffness = 20.0;
  vbd::AvbdRigidBodyRowIndexScratch rowIndexScratch;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          pointPairs,
          angularPairs,
          frictionPairs,
          options,
          rowOptions,
          frictionOptions,
          &rowIndexScratch,
          distanceSprings,
          distanceSpringOptions);

  EXPECT_EQ(stats.iterations, 1u);
  EXPECT_EQ(stats.bodyUpdates, 1u);
  EXPECT_TRUE(rowIndexScratch.attachmentRowOffsets.empty());
  EXPECT_TRUE(rowIndexScratch.pointPairRowOffsets.empty());
  EXPECT_TRUE(rowIndexScratch.angularPairRowOffsets.empty());
  EXPECT_TRUE(rowIndexScratch.frictionPairRowOffsets.empty());
  ASSERT_EQ(rowIndexScratch.distanceSpringRowOffsets.size(), 3u);
  EXPECT_EQ(rowIndexScratch.distanceSpringRowOffsets[0], 0u);
  EXPECT_EQ(rowIndexScratch.distanceSpringRowOffsets[1], 1u);
  EXPECT_EQ(rowIndexScratch.distanceSpringRowOffsets[2], 2u);
  ASSERT_EQ(rowIndexScratch.distanceSpringRowIndices.size(), 2u);
  EXPECT_EQ(rowIndexScratch.distanceSpringRowIndices[0], 0u);
  EXPECT_EQ(rowIndexScratch.distanceSpringRowIndices[1], 0u);
  EXPECT_NEAR(states[0].position.norm(), 0.0, 1e-12);
  EXPECT_LT(states[1].position.x(), 2.0);
  EXPECT_GT(states[1].position.x(), 1.0);
  EXPECT_DOUBLE_EQ(distanceSprings[0].row.state.lambda, 0.0);
  EXPECT_GT(distanceSprings[0].row.state.stiffness, 8.0);
  EXPECT_LT(distanceSprings[0].row.state.stiffness, 20.0);
}

//==============================================================================
TEST(AvbdRigidBlock, LargeDistanceSpringRowsUseProvidedScratchAllocator)
{
  constexpr std::size_t kRowCount
      = vbd::detail::kAvbdRigidSmallRowStackCapacity + 1u;

  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = 2.0 * Vec3::UnitX();
  const vbd::AvbdContactEndpointId endpointA{
      5, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const vbd::AvbdContactEndpointId endpointB{
      7, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};

  std::vector<vbd::AvbdRigidBodyPointPairDistanceSpringRow> springs;
  springs.reserve(kRowCount);
  for (std::size_t row = 0u; row < kRowCount; ++row) {
    vbd::AvbdRigidBodyPointPairDistanceSpringRow spring;
    spring.bodyA = 0;
    spring.bodyB = 1;
    spring.endpointA = endpointA;
    spring.endpointB = endpointB;
    spring.rowIndex = static_cast<std::uint32_t>(row);
    spring.row.restLength = 1.0;
    spring.row.materialStiffness = 20.0;
    spring.startStiffness = 8.0;
    spring.maxStiffness = 20.0;
    springs.push_back(spring);
  }

  common::MemoryManager memoryManager;
  auto& allocator = memoryManager.getFreeAllocator();
  auto& freeList = memoryManager.getFreeListAllocator();
  vbd::AvbdScalarRowInventory springInventory(allocator);
  springInventory.reserve(kRowCount);
  using DistanceSpringAllocator
      = common::StlAllocator<vbd::AvbdRigidBodyPointPairDistanceSpringRow>;
  std::vector<
      vbd::AvbdRigidBodyPointPairDistanceSpringRow,
      DistanceSpringAllocator>
      springRows(DistanceSpringAllocator{allocator});
  springRows.reserve(kRowCount);
  vbd::AvbdRigidDistanceSpringRowScratch scratch(allocator);

  const auto allocationsBefore = freeList.getAllocationCount();
  vbd::buildAvbdRigidDistanceSpringRows(
      states, springs, springInventory, springRows, scratch);

  EXPECT_GT(freeList.getAllocationCount(), allocationsBefore)
      << "large rigid AVBD distance-spring staging should borrow the "
         "provided scratch allocator";
  EXPECT_EQ(springRows.size(), kRowCount);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidRowDriverAppliesFrictionPair)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = Vec3(0.0, 1.0, 0.0);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias{
      Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 10.0;

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairRow> pointPairs;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularPairs;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionPairs(1);
  frictionPairs[0].bodyA = 0;
  frictionPairs[0].bodyB = 1;
  frictionPairs[0].first = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitX(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowState);
  frictionPairs[0].second = vbd::makeAvbdRigidContactFrictionTangentRow(
      Vec3::Zero(),
      Vec3::Zero(),
      Vec3::UnitY(),
      Vec3::Zero(),
      /*forceLimit=*/5.0,
      rowState);

  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 1;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          pointPairs,
          angularPairs,
          frictionPairs,
          options,
          rowOptions,
          frictionOptions);

  EXPECT_EQ(stats.iterations, 1u);
  EXPECT_EQ(stats.bodyUpdates, 1u);
  EXPECT_NEAR(states[1].position.x(), 0.0, 1e-12);
  EXPECT_LT(states[1].position.y(), 1.0);
  EXPECT_NEAR(frictionPairs[0].first.state.lambda, 0.0, 1e-12);
  EXPECT_NEAR(frictionPairs[0].second.state.lambda, 5.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidRowDriverHonorsConvergenceDisplacement)
{
  std::vector<vbd::AvbdRigidBodyState> states(1);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0};
  const std::vector<Eigen::Matrix3d> inertias{Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {0u};

  vbd::AvbdScalarRowState rowState;
  rowState.stiffness = 10.0;

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments(1);
  attachments[0].body = 0;
  attachments[0].row.target = Vec3::Zero();
  attachments[0].row.axis = Vec3::UnitX();
  attachments[0].row.state = rowState;
  std::vector<vbd::AvbdRigidBodyPointPairRow> pointPairs;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularPairs;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionPairs;

  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 8;
  options.convergenceDisplacement = 1e-12;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          pointPairs,
          angularPairs,
          frictionPairs,
          options,
          rowOptions,
          frictionOptions);

  EXPECT_EQ(stats.iterations, 1u);
  EXPECT_EQ(stats.bodyUpdates, 1u);
  EXPECT_NEAR(states[0].position.norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidContactManifoldBuilderCreatesWarmStartedRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[0].position = Vec3(0.25, -0.1, 0.05);
  states[1].position = Vec3(-0.5, 0.3, -0.2);

  std::vector<vbd::AvbdRigidContactManifoldPoint> contacts(1);
  contacts[0].bodyA = 0;
  contacts[0].bodyB = 1;
  contacts[0].endpointA
      = {42,
         vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Vertex, 4)};
  contacts[0].endpointB = {
      7, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Face, 2)};
  contacts[0].point = Vec3(0.1, 0.2, -0.15);
  contacts[0].normalFromAtoB = 2.0 * Vec3::UnitX();
  contacts[0].depth = 0.2;
  contacts[0].frictionCoefficient = 0.5;
  contacts[0].startStiffness = 80.0;
  contacts[0].maxStiffness = 400.0;
  contacts[0].row = 3;

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), 1u);
  EXPECT_EQ(
      normalInventory[0].descriptor.key.objectA, contacts[0].endpointB.object);
  EXPECT_EQ(
      normalInventory[0].descriptor.key.featureA,
      contacts[0].endpointB.feature);
  EXPECT_EQ(normalInventory[0].descriptor.key.row, 3u);
  EXPECT_DOUBLE_EQ(normalInventory[0].descriptor.startStiffness, 80.0);
  EXPECT_DOUBLE_EQ(normalInventory[0].descriptor.maxStiffness, 400.0);
  ASSERT_EQ(normalRows.size(), 1u);
  EXPECT_EQ(normalRows[0].bodyA, 0u);
  EXPECT_EQ(normalRows[0].bodyB, 1u);
  EXPECT_NEAR((normalRows[0].row.axis + Vec3::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      vbd::avbdRigidPointPairConstraintValue(
          states[0], states[1], normalRows[0].row),
      0.2,
      1e-12);

  normalInventory[0].state.lambda = 8.0;
  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(frictionRows.size(), 1u);
  EXPECT_DOUBLE_EQ(frictionRows[0].first.bounds.lower, -4.0);
  EXPECT_DOUBLE_EQ(frictionRows[0].first.bounds.upper, 4.0);
  EXPECT_DOUBLE_EQ(frictionRows[0].second.bounds.lower, -4.0);
  EXPECT_DOUBLE_EQ(frictionRows[0].second.bounds.upper, 4.0);
  EXPECT_NEAR(
      (frictionRows[0].first.localPointA - normalRows[0].row.localPointA)
          .norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(
      (frictionRows[0].first.localPointB - normalRows[0].row.localPointB)
          .norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(
      (frictionRows[0].second.localPointA - normalRows[0].row.localPointA)
          .norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(
      (frictionRows[0].second.localPointB - normalRows[0].row.localPointB)
          .norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(frictionRows[0].first.axis.dot(Vec3::UnitX()), 0.0, 1e-12);
  EXPECT_NEAR(frictionRows[0].second.axis.dot(Vec3::UnitX()), 0.0, 1e-12);
  EXPECT_NEAR(
      frictionRows[0].first.axis.dot(frictionRows[0].second.axis), 0.0, 1e-12);
  EXPECT_GT(frictionRows[0].first.localPointA.norm(), 0.0);
  EXPECT_GT(frictionRows[0].first.localPointB.norm(), 0.0);
  EXPECT_DOUBLE_EQ(frictionRows[0].first.offset, 0.0);
  EXPECT_DOUBLE_EQ(frictionRows[0].second.offset, 0.0);
  EXPECT_NEAR(
      vbd::avbdRigidPointPairConstraintValue(
          states[0], states[1], frictionRows[0].first),
      0.0,
      1e-12);
  EXPECT_NEAR(
      vbd::avbdRigidPointPairConstraintValue(
          states[0], states[1], frictionRows[0].second),
      0.0,
      1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidContactManifoldBuilderSkipsZeroLimitFrictionRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);

  std::vector<vbd::AvbdRigidContactManifoldPoint> contacts(1);
  contacts[0].bodyA = 0;
  contacts[0].bodyB = 1;
  contacts[0].endpointA
      = {42,
         vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Vertex, 4)};
  contacts[0].endpointB = {
      7, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Face, 2)};
  contacts[0].point = Vec3(0.1, 0.2, -0.15);
  contacts[0].normalFromAtoB = Vec3::UnitX();
  contacts[0].depth = 0.2;
  contacts[0].frictionCoefficient = 0.5;
  contacts[0].startStiffness = 80.0;
  contacts[0].maxStiffness = 400.0;

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);
  ASSERT_EQ(normalInventory.size(), 1u);
  normalInventory[0].state.lambda = 8.0;

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);
  ASSERT_EQ(frictionInventory.size(), 2u);
  ASSERT_EQ(frictionRows.size(), 1u);

  normalInventory[0].state.lambda = 0.0;
  contacts[0].startStiffness = 0.0;
  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), 1u);
  ASSERT_EQ(normalRows.size(), 1u);
  EXPECT_TRUE(frictionInventory.records().empty());
  EXPECT_TRUE(frictionRows.empty());

  normalInventory[0].state.lambda = 8.0;
  contacts[0].startStiffness = 80.0;
  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);
  ASSERT_EQ(frictionInventory.size(), 2u);
  ASSERT_EQ(frictionRows.size(), 1u);

  contacts[0].frictionCoefficient = 0.0;
  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(normalInventory.size(), 1u);
  ASSERT_EQ(normalRows.size(), 1u);
  EXPECT_TRUE(frictionInventory.records().empty());
  EXPECT_TRUE(frictionRows.empty());
}

//==============================================================================
TEST(
    AvbdRigidBlock,
    RigidContactManifoldFrictionProjectsWarmStartedDualAcrossTangentBasis)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);

  std::vector<vbd::AvbdRigidContactManifoldPoint> contacts(1);
  contacts[0].bodyA = 0;
  contacts[0].bodyB = 1;
  contacts[0].endpointA = {
      42, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Face, 4)};
  contacts[0].endpointB = {
      7, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Face, 2)};
  contacts[0].point = Vec3::Zero();
  contacts[0].normalFromAtoB = Vec3::UnitZ();
  contacts[0].depth = 0.2;
  contacts[0].frictionCoefficient = 1.0;
  contacts[0].startStiffness = 80.0;
  contacts[0].maxStiffness = 400.0;

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);
  ASSERT_EQ(normalInventory.size(), 1u);
  normalInventory[0].state.lambda = 10.0;

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);
  ASSERT_EQ(frictionInventory.size(), 2u);
  frictionInventory[0].state.lambda = 3.0;
  frictionInventory[1].state.lambda = 4.0;
  const Vec3 previousFirstAxis = frictionInventory[0].direction;
  const Vec3 previousSecondAxis = frictionInventory[1].direction;

  contacts[0].normalFromAtoB = Vec3(1.0, 0.0, 1.0).normalized();
  const Eigen::Matrix<double, 3, 2> currentBasis
      = vbd::avbdRigidContactTangentBasis(contacts[0].normalFromAtoB);
  const Eigen::Vector2d expected = vbd::projectAvbdFrictionDualToTangentPair(
      3.0,
      4.0,
      previousFirstAxis,
      previousSecondAxis,
      currentBasis.col(0),
      currentBasis.col(1));
  ASSERT_GT((expected - Eigen::Vector2d(3.0, 4.0)).norm(), 1e-6);

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);

  ASSERT_EQ(frictionInventory.size(), 2u);
  EXPECT_NEAR(frictionInventory[0].state.lambda, expected.x(), 1e-12);
  EXPECT_NEAR(frictionInventory[1].state.lambda, expected.y(), 1e-12);
  EXPECT_NEAR(
      (frictionInventory[0].direction - currentBasis.col(0)).norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(
      (frictionInventory[1].direction - currentBasis.col(1)).norm(),
      0.0,
      1e-12);
  ASSERT_EQ(frictionRows.size(), 1u);
  EXPECT_NEAR(frictionRows[0].first.state.lambda, expected.x(), 1e-12);
  EXPECT_NEAR(frictionRows[0].second.state.lambda, expected.y(), 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidContactManifoldBuilderUsesScratchForLargeManifolds)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[0].position = Vec3(0.25, -0.1, 0.05);
  states[1].position = Vec3(-0.5, 0.3, -0.2);

  constexpr std::size_t kActiveContacts
      = vbd::detail::kAvbdRigidSmallRowStackCapacity + 1u;
  std::vector<vbd::AvbdRigidContactManifoldPoint> contacts(kActiveContacts);
  for (std::size_t i = 0; i < contacts.size(); ++i) {
    auto& contact = contacts[i];
    contact.bodyA = 0;
    contact.bodyB = 1;
    contact.endpointA
        = {42u + static_cast<std::uint32_t>(i),
           vbd::packAvbdContactFeatureId(
               vbd::AvbdContactFeatureKind::Vertex,
               static_cast<std::uint32_t>(i))};
    contact.endpointB = {
        7u,
        vbd::packAvbdContactFeatureId(
            vbd::AvbdContactFeatureKind::Face, static_cast<std::uint32_t>(i))};
    contact.point = Vec3(0.01 * static_cast<double>(i), 0.2, -0.15);
    contact.normalFromAtoB = Vec3::UnitZ();
    contact.depth = 0.2 + 0.001 * static_cast<double>(i);
    contact.frictionCoefficient = 0.5;
    contact.startStiffness = 80.0;
    contact.maxStiffness = 400.0;
    contact.row = static_cast<std::uint32_t>(i);
  }

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRigidContactManifoldRowScratch scratch;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      scratch,
      warmStart);

  ASSERT_EQ(normalInventory.size(), kActiveContacts);
  ASSERT_EQ(normalRows.size(), kActiveContacts);
  EXPECT_EQ(scratch.activeContacts.size(), kActiveContacts);
  EXPECT_EQ(scratch.contactLocalPoints.size(), kActiveContacts);
  EXPECT_NEAR((normalRows[0].row.axis + Vec3::UnitZ()).norm(), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(
      normalRows.back().row.previousConstraintValue, contacts.back().depth);

  for (std::size_t i = 0; i < normalInventory.size(); ++i) {
    normalInventory[i].state.lambda = 8.0;
  }
  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      scratch,
      warmStart);

  ASSERT_EQ(frictionInventory.size(), 2u * kActiveContacts);
  ASSERT_EQ(frictionRows.size(), kActiveContacts);
  frictionInventory[0].state.lambda = 3.0;
  frictionInventory[1].state.lambda = 4.0;
  const Vec3 previousFirstAxis = frictionInventory[0].direction;
  const Vec3 previousSecondAxis = frictionInventory[1].direction;

  contacts[0].normalFromAtoB = Vec3(1.0, 0.0, 1.0).normalized();
  const Eigen::Matrix<double, 3, 2> currentBasis
      = vbd::avbdRigidContactTangentBasis(contacts[0].normalFromAtoB);
  const Eigen::Vector2d expected = vbd::projectAvbdFrictionDualToTangentPair(
      3.0,
      4.0,
      previousFirstAxis,
      previousSecondAxis,
      currentBasis.col(0),
      currentBasis.col(1));

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      scratch,
      warmStart);

  ASSERT_EQ(frictionInventory.size(), 2u * kActiveContacts);
  ASSERT_EQ(frictionRows.size(), kActiveContacts);
  EXPECT_NEAR(frictionInventory[0].state.lambda, expected.x(), 1e-12);
  EXPECT_NEAR(frictionInventory[1].state.lambda, expected.y(), 1e-12);
  EXPECT_NEAR(
      (frictionRows[0].first.axis - currentBasis.col(0)).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (frictionRows[0].second.axis - currentBasis.col(1)).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPointJointBuilderCreatesWarmStartedLinearRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);

  std::vector<vbd::AvbdRigidPointJoint> joints(1);
  joints[0].bodyA = 0;
  joints[0].bodyB = 1;
  joints[0].endpointA = {
      12, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].endpointB = {
      3, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].localPointA = Vec3(0.1, 0.2, 0.3);
  joints[0].localPointB = Vec3(-0.2, 0.4, 0.1);
  joints[0].startStiffness = 70.0;
  joints[0].maxStiffness = 500.0;
  joints[0].row = 4;

  vbd::AvbdScalarRowInventory linearInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearRows;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 0.5;
  warmStart.gamma = 0.25;

  vbd::buildAvbdRigidPointJointRows(
      states, joints, linearInventory, linearRows, warmStart);

  ASSERT_EQ(linearInventory.size(), 3u);
  ASSERT_EQ(linearRows.size(), 3u);
  const Eigen::Vector3d expectedPreviousLinear(-0.3, 0.2, -0.2);
  for (std::uint8_t axis = 0; axis < 3u; ++axis) {
    const auto& descriptor = linearInventory[axis].descriptor;
    EXPECT_EQ(descriptor.key.role, vbd::AvbdScalarRowRole::JointLinear);
    EXPECT_EQ(descriptor.key.row, 4u);
    EXPECT_EQ(descriptor.key.axis, axis);
    EXPECT_DOUBLE_EQ(descriptor.startStiffness, 70.0);
    EXPECT_DOUBLE_EQ(descriptor.maxStiffness, 500.0);
    EXPECT_TRUE(std::isinf(descriptor.bounds.lower));
    EXPECT_LT(descriptor.bounds.lower, 0.0);
    EXPECT_TRUE(std::isinf(descriptor.bounds.upper));
    EXPECT_GT(descriptor.bounds.upper, 0.0);
    EXPECT_EQ(linearRows[axis].bodyA, 0u);
    EXPECT_EQ(linearRows[axis].bodyB, 1u);
    EXPECT_NEAR(
        (linearRows[axis].row.axis - Vec3::Unit(axis)).norm(), 0.0, 1e-12);
    EXPECT_NEAR(
        linearRows[axis].row.previousConstraintValue,
        expectedPreviousLinear[axis],
        1e-12);
  }

  linearInventory[0].state.lambda = 8.0;
  linearInventory[0].state.stiffness = 120.0;
  vbd::buildAvbdRigidPointJointRows(
      states, joints, linearInventory, linearRows, warmStart);

  ASSERT_EQ(linearInventory.size(), 3u);
  EXPECT_DOUBLE_EQ(linearInventory[0].state.lambda, 1.0);
  EXPECT_DOUBLE_EQ(linearInventory[0].state.stiffness, 70.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPointJointBuilderCreatesFiniteLinearRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = Vec3(0.5, 0.0, 0.0);

  std::vector<vbd::AvbdRigidPointJoint> joints(1);
  joints[0].bodyA = 0;
  joints[0].bodyB = 1;
  joints[0].endpointA = {
      1, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].endpointB = {
      2, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].linearAxisMask = 1u;
  joints[0].startStiffness = 4.0;
  joints[0].linearMaterialStiffness = 20.0;
  joints[0].maxStiffness = 100.0;

  vbd::AvbdScalarRowInventory linearInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearRows;
  vbd::buildAvbdRigidPointJointRows(
      states, joints, linearInventory, linearRows);

  ASSERT_EQ(linearInventory.size(), 1u);
  ASSERT_EQ(linearRows.size(), 1u);
  EXPECT_EQ(
      linearInventory[0].descriptor.kind,
      vbd::AvbdScalarRowKind::FiniteStiffness);
  EXPECT_DOUBLE_EQ(linearInventory[0].descriptor.materialStiffness, 20.0);
  EXPECT_DOUBLE_EQ(linearRows[0].row.materialStiffness, 20.0);
  EXPECT_DOUBLE_EQ(linearRows[0].row.previousConstraintValue, 0.0);

  linearRows[0].row.state.lambda = 9.0;
  linearRows[0].row.state.stiffness = 4.0;
  vbd::AvbdRigidPointAttachmentOptions options;
  options.beta = 10.0;
  options.maxStiffness = 100.0;
  const vbd::AvbdScalarRowState updated = vbd::updateAvbdRigidPointPairRow(
      linearRows[0].row.state,
      states[0],
      states[1],
      linearRows[0].row,
      options);

  EXPECT_DOUBLE_EQ(updated.lambda, 0.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 9.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPointJointBuilderCreatesFiniteAngularRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].orientation = rotationZ(0.5);

  std::vector<vbd::AvbdRigidPointJoint> joints(1);
  joints[0].bodyA = 0;
  joints[0].bodyB = 1;
  joints[0].endpointA = {
      1, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].endpointB = {
      2, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].angularAxisMask = 1u << 2u;
  joints[0].startStiffness = 4.0;
  joints[0].angularMaterialStiffness = 20.0;
  joints[0].maxStiffness = 100.0;

  vbd::AvbdScalarRowInventory angularInventory;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  vbd::buildAvbdRigidPointJointAngularRows(
      states, joints, angularInventory, angularRows);

  ASSERT_EQ(angularInventory.size(), 1u);
  ASSERT_EQ(angularRows.size(), 1u);
  EXPECT_EQ(
      angularInventory[0].descriptor.kind,
      vbd::AvbdScalarRowKind::FiniteStiffness);
  EXPECT_DOUBLE_EQ(angularInventory[0].descriptor.materialStiffness, 20.0);
  EXPECT_DOUBLE_EQ(angularRows[0].row.materialStiffness, 20.0);
  EXPECT_DOUBLE_EQ(angularRows[0].row.previousConstraintValue, 0.0);

  angularRows[0].row.state.lambda = 9.0;
  angularRows[0].row.state.stiffness = 4.0;
  vbd::AvbdRigidPointAttachmentOptions options;
  options.beta = 10.0;
  options.maxStiffness = 100.0;
  const vbd::AvbdScalarRowState updated = vbd::updateAvbdRigidAngularPairRow(
      angularRows[0].row.state,
      states[0],
      states[1],
      angularRows[0].row,
      options);

  EXPECT_DOUBLE_EQ(updated.lambda, 0.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 9.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPointJointRowsDriveAnchorsTogether)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[0].position = Vec3::Zero();
  states[1].position = Vec3::UnitX();
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias
      = {Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  std::vector<vbd::AvbdRigidPointJoint> joints(1);
  joints[0].bodyA = 0;
  joints[0].bodyB = 1;
  joints[0].endpointA = {
      1, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].endpointB = {
      2, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].startStiffness = 100.0;
  joints[0].maxStiffness = 1000.0;

  vbd::AvbdScalarRowInventory linearInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  vbd::buildAvbdRigidPointJointRows(
      states, joints, linearInventory, linearRows);

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 8;
  options.regularization = 1e-12;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  rowOptions.beta = 1000.0;
  rowOptions.maxStiffness = 1000.0;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          linearRows,
          angularRows,
          frictionRows,
          options,
          rowOptions,
          frictionOptions);

  EXPECT_GT(stats.bodyUpdates, 0u);
  EXPECT_LT(states[1].position.x(), 0.25);
  EXPECT_NEAR(states[1].position.y(), 0.0, 1e-12);
  EXPECT_NEAR(states[1].position.z(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPointJointBuilderCreatesWarmStartedAngularRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);

  std::vector<vbd::AvbdRigidPointJoint> joints(1);
  joints[0].bodyA = 0;
  joints[0].bodyB = 1;
  joints[0].endpointA = {
      12, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].endpointB = {
      3, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].targetRelativeOrientation = rotationZ(0.25);
  joints[0].startStiffness = 70.0;
  joints[0].maxStiffness = 500.0;
  joints[0].row = 4;

  vbd::AvbdScalarRowInventory angularInventory;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 0.5;
  warmStart.gamma = 0.25;

  vbd::buildAvbdRigidPointJointAngularRows(
      states, joints, angularInventory, angularRows, warmStart);

  ASSERT_EQ(angularInventory.size(), 3u);
  ASSERT_EQ(angularRows.size(), 3u);
  const Eigen::Vector3d expectedPreviousAngular(0.0, 0.0, -0.25);
  for (std::uint8_t axis = 0; axis < 3u; ++axis) {
    const auto& descriptor = angularInventory[axis].descriptor;
    EXPECT_EQ(descriptor.key.role, vbd::AvbdScalarRowRole::JointAngular);
    EXPECT_EQ(descriptor.key.row, 4u);
    EXPECT_EQ(descriptor.key.axis, axis);
    EXPECT_DOUBLE_EQ(descriptor.startStiffness, 70.0);
    EXPECT_DOUBLE_EQ(descriptor.maxStiffness, 500.0);
    EXPECT_TRUE(std::isinf(descriptor.bounds.lower));
    EXPECT_LT(descriptor.bounds.lower, 0.0);
    EXPECT_TRUE(std::isinf(descriptor.bounds.upper));
    EXPECT_GT(descriptor.bounds.upper, 0.0);
    EXPECT_EQ(angularRows[axis].bodyA, 0u);
    EXPECT_EQ(angularRows[axis].bodyB, 1u);
    EXPECT_NEAR(
        (angularRows[axis].row.axis - Vec3::Unit(axis)).norm(), 0.0, 1e-12);
    EXPECT_NEAR(
        (angularRows[axis].row.targetRelativeOrientation.toRotationMatrix()
         - rotationZ(0.25).toRotationMatrix())
            .norm(),
        0.0,
        1e-12);
    EXPECT_NEAR(
        angularRows[axis].row.previousConstraintValue,
        expectedPreviousAngular[axis],
        1e-12);
  }

  angularInventory[0].state.lambda = 8.0;
  angularInventory[0].state.stiffness = 120.0;
  vbd::buildAvbdRigidPointJointAngularRows(
      states, joints, angularInventory, angularRows, warmStart);

  ASSERT_EQ(angularInventory.size(), 3u);
  EXPECT_DOUBLE_EQ(angularInventory[0].state.lambda, 1.0);
  EXPECT_DOUBLE_EQ(angularInventory[0].state.stiffness, 70.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidAngularMotorBuilderCreatesBoundedRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  const vbd::AvbdContactEndpointId endpointA{
      12, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const vbd::AvbdContactEndpointId endpointB{
      3, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const std::vector<vbd::AvbdRigidAngularMotor> motors{
      vbd::makeAvbdRigidAngularMotor(
          /*bodyA=*/0,
          /*bodyB=*/1,
          endpointA,
          endpointB,
          rotationZ(0.1),
          2.0 * Vec3::UnitZ(),
          /*targetSpeed=*/2.0,
          /*maxTorque=*/4.0,
          /*startStiffness=*/70.0,
          /*maxStiffness=*/500.0,
          /*row=*/6)};

  vbd::AvbdScalarRowInventory motorInventory;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> motorRows;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 0.5;
  warmStart.gamma = 0.25;

  vbd::buildAvbdRigidAngularMotorRows(
      states,
      motors,
      motorInventory,
      motorRows,
      /*timeStep=*/0.25,
      warmStart);

  ASSERT_EQ(motorInventory.size(), 1u);
  ASSERT_EQ(motorRows.size(), 1u);

  const auto& descriptor = motorInventory[0].descriptor;
  EXPECT_EQ(descriptor.key.role, vbd::AvbdScalarRowRole::Motor);
  EXPECT_EQ(descriptor.key.row, 6u);
  EXPECT_EQ(descriptor.key.axis, 0u);
  EXPECT_DOUBLE_EQ(descriptor.bounds.lower, -4.0);
  EXPECT_DOUBLE_EQ(descriptor.bounds.upper, 4.0);
  EXPECT_DOUBLE_EQ(descriptor.startStiffness, 70.0);
  EXPECT_DOUBLE_EQ(descriptor.maxStiffness, 500.0);

  EXPECT_EQ(motorRows[0].bodyA, 0u);
  EXPECT_EQ(motorRows[0].bodyB, 1u);
  EXPECT_NEAR((motorRows[0].row.axis - Vec3::UnitZ()).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (motorRows[0].row.targetRelativeOrientation.toRotationMatrix()
       - rotationZ(0.1).toRotationMatrix())
          .norm(),
      0.0,
      1e-12);
  EXPECT_DOUBLE_EQ(motorRows[0].row.offset, -0.5);
  EXPECT_DOUBLE_EQ(motorRows[0].row.previousConstraintValue, 0.0);
  EXPECT_DOUBLE_EQ(motorRows[0].row.bounds.lower, -4.0);
  EXPECT_DOUBLE_EQ(motorRows[0].row.bounds.upper, 4.0);

  motorInventory[0].state.lambda = 8.0;
  motorInventory[0].state.stiffness = 120.0;
  vbd::buildAvbdRigidAngularMotorRows(
      states,
      motors,
      motorInventory,
      motorRows,
      /*timeStep=*/0.25,
      warmStart);

  ASSERT_EQ(motorInventory.size(), 1u);
  EXPECT_DOUBLE_EQ(motorInventory[0].state.lambda, 1.0);
  EXPECT_DOUBLE_EQ(motorInventory[0].state.stiffness, 70.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidLinearMotorBuilderCreatesBoundedRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = 2.0 * Vec3::UnitX();
  const vbd::AvbdContactEndpointId endpointA{
      5, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const vbd::AvbdContactEndpointId endpointB{
      7, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const std::vector<vbd::AvbdRigidLinearMotor> linearMotors{
      vbd::makeAvbdRigidLinearMotor(
          /*bodyA=*/0,
          /*bodyB=*/1,
          endpointA,
          endpointB,
          Vec3::Zero(),
          Vec3::Zero(),
          2.0 * Vec3::UnitX(),
          /*targetSpeed=*/0.75,
          /*maxForce=*/6.0,
          /*startStiffness=*/50.0,
          /*maxStiffness=*/400.0,
          /*row=*/3)};

  vbd::AvbdScalarRowInventory motorInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearMotorRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularMotorRows;
  vbd::buildAvbdRigidMotorRows(
      states,
      linearMotors,
      std::span<const vbd::AvbdRigidAngularMotor>(),
      motorInventory,
      linearMotorRows,
      angularMotorRows,
      /*timeStep=*/0.2);

  ASSERT_EQ(motorInventory.size(), 1u);
  ASSERT_EQ(linearMotorRows.size(), 1u);
  EXPECT_TRUE(angularMotorRows.empty());

  const auto& descriptor = motorInventory[0].descriptor;
  EXPECT_EQ(descriptor.key.role, vbd::AvbdScalarRowRole::Motor);
  EXPECT_EQ(descriptor.key.row, 3u);
  EXPECT_DOUBLE_EQ(descriptor.bounds.lower, -6.0);
  EXPECT_DOUBLE_EQ(descriptor.bounds.upper, 6.0);
  EXPECT_DOUBLE_EQ(descriptor.startStiffness, 50.0);
  EXPECT_DOUBLE_EQ(descriptor.maxStiffness, 400.0);

  EXPECT_EQ(linearMotorRows[0].bodyA, 0u);
  EXPECT_EQ(linearMotorRows[0].bodyB, 1u);
  EXPECT_NEAR((linearMotorRows[0].row.axis - Vec3::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_DOUBLE_EQ(linearMotorRows[0].row.offset, -2.15);
  EXPECT_NEAR(
      vbd::avbdRigidPointPairConstraintValue(
          states[0], states[1], linearMotorRows[0].row),
      -0.15,
      1e-12);
  EXPECT_DOUBLE_EQ(linearMotorRows[0].row.bounds.lower, -6.0);
  EXPECT_DOUBLE_EQ(linearMotorRows[0].row.bounds.upper, 6.0);
}

//==============================================================================
TEST(AvbdRigidBlock, LargeRigidMotorRowsUseProvidedScratchAllocator)
{
  constexpr std::size_t kRowCount
      = vbd::detail::kAvbdRigidSmallRowStackCapacity + 1u;

  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = 2.0 * Vec3::UnitX();
  const vbd::AvbdContactEndpointId endpointA{
      5, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const vbd::AvbdContactEndpointId endpointB{
      7, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};

  std::vector<vbd::AvbdRigidLinearMotor> linearMotors;
  linearMotors.reserve(kRowCount);
  for (std::size_t row = 0u; row < kRowCount; ++row) {
    linearMotors.push_back(
        vbd::makeAvbdRigidLinearMotor(
            /*bodyA=*/0,
            /*bodyB=*/1,
            endpointA,
            endpointB,
            Vec3::Zero(),
            Vec3::Zero(),
            Vec3::UnitX(),
            /*targetSpeed=*/0.25,
            /*maxForce=*/6.0,
            /*startStiffness=*/50.0,
            /*maxStiffness=*/400.0,
            static_cast<std::uint32_t>(row)));
  }

  common::MemoryManager memoryManager;
  auto& allocator = memoryManager.getFreeAllocator();
  auto& freeList = memoryManager.getFreeListAllocator();
  vbd::AvbdScalarRowInventory motorInventory(allocator);
  motorInventory.reserve(kRowCount);
  using PointPairAllocator
      = common::StlAllocator<vbd::AvbdRigidBodyPointPairRow>;
  using AngularPairAllocator
      = common::StlAllocator<vbd::AvbdRigidBodyAngularPairRow>;
  std::vector<vbd::AvbdRigidBodyPointPairRow, PointPairAllocator>
      linearMotorRows(PointPairAllocator{allocator});
  std::vector<vbd::AvbdRigidBodyAngularPairRow, AngularPairAllocator>
      angularMotorRows(AngularPairAllocator{allocator});
  linearMotorRows.reserve(kRowCount);
  angularMotorRows.reserve(kRowCount);
  vbd::AvbdRigidMotorRowScratch scratch(allocator);

  const auto allocationsBefore = freeList.getAllocationCount();
  vbd::buildAvbdRigidMotorRows(
      states,
      linearMotors,
      std::span<const vbd::AvbdRigidAngularMotor>(),
      motorInventory,
      linearMotorRows,
      angularMotorRows,
      /*timeStep=*/0.2,
      scratch);

  EXPECT_GT(freeList.getAllocationCount(), allocationsBefore)
      << "large rigid AVBD motor-row staging should borrow the provided "
         "scratch allocator";
  EXPECT_EQ(linearMotorRows.size(), kRowCount);
  EXPECT_TRUE(angularMotorRows.empty());
}

//==============================================================================
TEST(AvbdRigidBlock, RigidMotorBuilderReusesLargeInputScratch)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = 2.0 * Vec3::UnitX();
  const vbd::AvbdContactEndpointId endpointA{
      5, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const vbd::AvbdContactEndpointId endpointB{
      7, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};

  std::vector<vbd::AvbdRigidLinearMotor> linearMotors;
  linearMotors.reserve(9);
  for (std::uint32_t row = 0; row < 9u; ++row) {
    linearMotors.push_back(
        vbd::makeAvbdRigidLinearMotor(
            /*bodyA=*/0,
            /*bodyB=*/1,
            endpointA,
            endpointB,
            Vec3::Zero(),
            Vec3::Zero(),
            Vec3::UnitX(),
            /*targetSpeed=*/0.75,
            /*maxForce=*/6.0,
            /*startStiffness=*/50.0,
            /*maxStiffness=*/400.0,
            row));
  }

  std::vector<vbd::AvbdRigidAngularMotor> angularMotors;
  angularMotors.reserve(8);
  for (std::uint32_t row = 0; row < 8u; ++row) {
    angularMotors.push_back(
        vbd::makeAvbdRigidAngularMotor(
            /*bodyA=*/0,
            /*bodyB=*/1,
            endpointA,
            endpointB,
            Eigen::Quaterniond::Identity(),
            Vec3::UnitZ(),
            /*targetSpeed=*/1.0,
            /*maxTorque=*/4.0,
            /*startStiffness=*/70.0,
            /*maxStiffness=*/500.0,
            100u + row));
  }

  vbd::AvbdScalarRowInventory motorInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearMotorRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularMotorRows;
  vbd::AvbdRigidMotorRowScratch scratch;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;
  vbd::buildAvbdRigidMotorRows(
      states,
      linearMotors,
      angularMotors,
      motorInventory,
      linearMotorRows,
      angularMotorRows,
      /*timeStep=*/0.2,
      scratch,
      warmStart);

  ASSERT_EQ(motorInventory.size(), 17u);
  EXPECT_EQ(linearMotorRows.size(), linearMotors.size());
  EXPECT_EQ(angularMotorRows.size(), angularMotors.size());
  EXPECT_EQ(scratch.activeLinearRows.size(), linearMotors.size());
  EXPECT_EQ(scratch.activeAngularRows.size(), angularMotors.size());
  EXPECT_EQ(motorInventory[0].descriptor.key.row, 0u);
  EXPECT_EQ(motorInventory[8].descriptor.key.row, 8u);
  EXPECT_EQ(motorInventory[9].descriptor.key.row, 100u);
  EXPECT_EQ(motorInventory[16].descriptor.key.row, 107u);

  const std::size_t linearCapacity = scratch.activeLinearRows.capacity();
  const std::size_t angularCapacity = scratch.activeAngularRows.capacity();
  motorInventory[0].state.lambda = 3.0;
  vbd::buildAvbdRigidMotorRows(
      states,
      linearMotors,
      angularMotors,
      motorInventory,
      linearMotorRows,
      angularMotorRows,
      /*timeStep=*/0.2,
      scratch,
      warmStart);

  EXPECT_EQ(scratch.activeLinearRows.capacity(), linearCapacity);
  EXPECT_EQ(scratch.activeAngularRows.capacity(), angularCapacity);
  EXPECT_EQ(linearMotorRows.size(), linearMotors.size());
  EXPECT_EQ(angularMotorRows.size(), angularMotors.size());
  EXPECT_DOUBLE_EQ(motorInventory[0].state.lambda, 3.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidAngularMotorRowsDriveTargetAngularStep)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias
      = {Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  const std::vector<vbd::AvbdRigidAngularMotor> motors{
      vbd::makeAvbdRigidAngularMotor(
          /*bodyA=*/0,
          /*bodyB=*/1,
          {1,
           vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)},
          {2,
           vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)},
          Eigen::Quaterniond::Identity(),
          Vec3::UnitZ(),
          /*targetSpeed=*/0.5,
          /*maxTorque=*/1000.0,
          /*startStiffness=*/100.0,
          /*maxStiffness=*/1000.0)};

  vbd::AvbdScalarRowInventory motorInventory;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> motorRows;
  vbd::buildAvbdRigidAngularMotorRows(
      states, motors, motorInventory, motorRows, /*timeStep=*/0.25);

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairRow> pointPairs;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 8;
  options.regularization = 1e-12;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  rowOptions.beta = 1000.0;
  rowOptions.maxStiffness = 1000.0;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/0.25,
          attachments,
          pointPairs,
          motorRows,
          frictionRows,
          options,
          rowOptions,
          frictionOptions);

  const Vec3 error = vbd::avbdRigidBodyOrientationError(
      states[1].orientation, Eigen::Quaterniond::Identity());
  EXPECT_GT(stats.bodyUpdates, 0u);
  EXPECT_NEAR(error.x(), 0.0, 1e-12);
  EXPECT_NEAR(error.y(), 0.0, 1e-12);
  EXPECT_GT(error.z(), 0.02);
  EXPECT_NEAR(error.z(), 0.125, 0.05);
  ASSERT_EQ(motorRows.size(), 1u);
  EXPECT_LE(std::abs(motorRows[0].row.state.lambda), 1000.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidLinearMotorRowsDriveTargetLinearStep)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = 2.0 * Vec3::UnitX();
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias
      = {Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  const std::vector<vbd::AvbdRigidLinearMotor> linearMotors{
      vbd::makeAvbdRigidLinearMotor(
          /*bodyA=*/0,
          /*bodyB=*/1,
          {1,
           vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)},
          {2,
           vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)},
          Vec3::Zero(),
          Vec3::Zero(),
          Vec3::UnitX(),
          /*targetSpeed=*/0.5,
          /*maxForce=*/1000.0,
          /*startStiffness=*/100.0,
          /*maxStiffness=*/1000.0)};

  vbd::AvbdScalarRowInventory motorInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearMotorRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularMotorRows;
  vbd::buildAvbdRigidMotorRows(
      states,
      linearMotors,
      std::span<const vbd::AvbdRigidAngularMotor>(),
      motorInventory,
      linearMotorRows,
      angularMotorRows,
      /*timeStep=*/0.25);

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularPairs;
  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 8;
  options.regularization = 1e-12;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  rowOptions.beta = 1000.0;
  rowOptions.maxStiffness = 1000.0;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/0.25,
          attachments,
          linearMotorRows,
          angularPairs,
          frictionRows,
          options,
          rowOptions,
          frictionOptions);

  EXPECT_GT(stats.bodyUpdates, 0u);
  EXPECT_GT(states[1].position.x(), 2.02);
  EXPECT_NEAR(states[1].position.x(), 2.125, 0.05);
  ASSERT_EQ(linearMotorRows.size(), 1u);
  EXPECT_LE(std::abs(linearMotorRows[0].row.state.lambda), 1000.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidAngularPairRowsReportFractureThreshold)
{
  vbd::AvbdScalarRowState scalarState;
  scalarState.lambda = -2.5;
  EXPECT_TRUE(
      vbd::avbdRigidScalarRowExceedsFractureThreshold(scalarState, 2.5));
  EXPECT_FALSE(
      vbd::avbdRigidScalarRowExceedsFractureThreshold(scalarState, 2.6));
  EXPECT_FALSE(
      vbd::avbdRigidScalarRowExceedsFractureThreshold(scalarState, 0.0));

  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows(3);
  angularRows[0].row.state.lambda = 3.0;
  angularRows[0].row.state.stiffness = 40.0;
  angularRows[0].row.previousConstraintValue = 0.25;
  angularRows[1].row.state.lambda = 4.0;
  angularRows[1].row.state.stiffness = 50.0;
  angularRows[1].row.previousConstraintValue = -0.5;

  EXPECT_DOUBLE_EQ(vbd::avbdRigidAngularPairLambdaNorm(angularRows), 5.0);
  EXPECT_TRUE(
      vbd::avbdRigidAngularPairRowsExceedFractureThreshold(angularRows, 5.0));
  EXPECT_FALSE(
      vbd::avbdRigidAngularPairRowsExceedFractureThreshold(angularRows, 5.1));

  vbd::resetAvbdRigidAngularPairRowsAfterFracture(angularRows);

  for (const vbd::AvbdRigidBodyAngularPairRow& row : angularRows) {
    EXPECT_DOUBLE_EQ(row.row.state.lambda, 0.0);
    EXPECT_DOUBLE_EQ(row.row.state.stiffness, 0.0);
    EXPECT_DOUBLE_EQ(row.row.previousConstraintValue, 0.0);
  }
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPointJointBuilderHonorsAxisMasks)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[1].position = Vec3(1.0, 2.0, 3.0);
  states[1].orientation = rotationX(0.2) * rotationY(-0.3);

  std::vector<vbd::AvbdRigidPointJoint> joints(1);
  joints[0].bodyA = 0;
  joints[0].bodyB = 1;
  joints[0].endpointA = {
      12, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].endpointB = {
      3, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].linearAxisMask = static_cast<std::uint8_t>((1u << 0) | (1u << 2));
  joints[0].angularAxisMask = static_cast<std::uint8_t>(1u << 1);
  joints[0].startStiffness = 70.0;
  joints[0].maxStiffness = 500.0;
  joints[0].row = 4;

  vbd::AvbdScalarRowInventory linearInventory;
  vbd::AvbdScalarRowInventory angularInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  vbd::buildAvbdRigidPointJointConstraintRows(
      states,
      joints,
      linearInventory,
      angularInventory,
      linearRows,
      angularRows);

  ASSERT_EQ(linearInventory.size(), 2u);
  ASSERT_EQ(linearRows.size(), 2u);
  EXPECT_EQ(linearInventory[0].descriptor.key.axis, 0u);
  EXPECT_EQ(linearInventory[1].descriptor.key.axis, 2u);
  EXPECT_NEAR((linearRows[0].row.axis - Vec3::UnitX()).norm(), 0.0, 1e-12);
  EXPECT_NEAR((linearRows[1].row.axis - Vec3::UnitZ()).norm(), 0.0, 1e-12);

  ASSERT_EQ(angularInventory.size(), 1u);
  ASSERT_EQ(angularRows.size(), 1u);
  EXPECT_EQ(angularInventory[0].descriptor.key.axis, 1u);
  EXPECT_NEAR((angularRows[0].row.axis - Vec3::UnitY()).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPointJointBuilderClearsLargeLinearOnlyAngularRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(7);
  for (std::size_t i = 0; i < states.size(); ++i) {
    states[i].position = Vec3(static_cast<double>(i), 0.25 * i, 0.0);
  }

  const auto endpoint = [](std::uint64_t object) {
    return vbd::AvbdContactEndpointId{
        object,
        vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  };

  std::vector<vbd::AvbdRigidPointJoint> angularSeed(1);
  angularSeed[0].bodyA = 0;
  angularSeed[0].bodyB = 1;
  angularSeed[0].endpointA = endpoint(1);
  angularSeed[0].endpointB = endpoint(2);
  angularSeed[0].linearAxisMask = 0u;
  angularSeed[0].angularAxisMask = vbd::kAvbdRigidJointAllAxesMask;
  angularSeed[0].startStiffness = 50.0;
  angularSeed[0].maxStiffness = 1000.0;

  vbd::AvbdScalarRowInventory linearInventory;
  vbd::AvbdScalarRowInventory angularInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  vbd::buildAvbdRigidPointJointConstraintRows(
      states,
      angularSeed,
      linearInventory,
      angularInventory,
      linearRows,
      angularRows);

  ASSERT_EQ(linearInventory.size(), 0u);
  ASSERT_EQ(linearRows.size(), 0u);
  ASSERT_EQ(angularInventory.size(), 3u);
  ASSERT_EQ(angularRows.size(), 3u);

  std::vector<vbd::AvbdRigidPointJoint> linearOnly(6);
  for (std::size_t i = 0; i < linearOnly.size(); ++i) {
    vbd::AvbdRigidPointJoint& joint = linearOnly[i];
    joint.bodyA = 0;
    joint.bodyB = static_cast<std::uint32_t>(i + 1u);
    joint.endpointA = endpoint(10u + i);
    joint.endpointB = endpoint(20u + i);
    joint.linearAxisMask = vbd::kAvbdRigidJointAllAxesMask;
    joint.angularAxisMask = 0u;
    joint.startStiffness = 50.0;
    joint.maxStiffness = 1000.0;
    joint.row = static_cast<std::uint32_t>(i);
  }

  vbd::buildAvbdRigidPointJointConstraintRows(
      states,
      linearOnly,
      linearInventory,
      angularInventory,
      linearRows,
      angularRows);

  EXPECT_EQ(linearInventory.size(), 18u);
  EXPECT_EQ(linearRows.size(), 18u);
  EXPECT_TRUE(angularInventory.empty());
  EXPECT_TRUE(angularRows.empty());
}

//==============================================================================
TEST(AvbdRigidBlock, RigidRevolutePointJointConfigLeavesHingeAxisFree)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  const Vec3 hingeAxis = Vec3(1.0, 2.0, 3.0).normalized();
  const vbd::AvbdContactEndpointId endpointA{
      12, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const vbd::AvbdContactEndpointId endpointB{
      3, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const vbd::AvbdRigidPointJoint joint = vbd::makeAvbdRigidRevolutePointJoint(
      /*bodyA=*/0,
      /*bodyB=*/1,
      endpointA,
      endpointB,
      Vec3::Zero(),
      Vec3::Zero(),
      Eigen::Quaterniond::Identity(),
      hingeAxis,
      /*startStiffness=*/70.0,
      /*maxStiffness=*/500.0,
      /*row=*/4);

  EXPECT_EQ(joint.linearAxisMask, vbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(joint.angularAxisMask, vbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_NEAR(joint.angularAxes.col(0).dot(hingeAxis), 0.0, 1e-12);
  EXPECT_NEAR(joint.angularAxes.col(1).dot(hingeAxis), 0.0, 1e-12);
  EXPECT_NEAR(std::abs(joint.angularAxes.col(2).dot(hingeAxis)), 1.0, 1e-12);
  EXPECT_NEAR(
      joint.angularAxes.col(0).dot(joint.angularAxes.col(1)), 0.0, 1e-12);

  vbd::AvbdScalarRowInventory linearInventory;
  vbd::AvbdScalarRowInventory angularInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  const std::vector<vbd::AvbdRigidPointJoint> joints{joint};
  vbd::buildAvbdRigidPointJointConstraintRows(
      states,
      joints,
      linearInventory,
      angularInventory,
      linearRows,
      angularRows);

  ASSERT_EQ(linearRows.size(), 3u);
  ASSERT_EQ(angularRows.size(), 2u);
  EXPECT_EQ(angularInventory[0].descriptor.key.axis, 0u);
  EXPECT_EQ(angularInventory[1].descriptor.key.axis, 1u);
  EXPECT_NEAR(angularRows[0].row.axis.dot(hingeAxis), 0.0, 1e-12);
  EXPECT_NEAR(angularRows[1].row.axis.dot(hingeAxis), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPrismaticPointJointConfigLeavesTranslationAxisFree)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  const Vec3 translationAxis = Vec3(-0.25, 0.5, 1.0).normalized();
  const vbd::AvbdContactEndpointId endpointA{
      12, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const vbd::AvbdContactEndpointId endpointB{
      3, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const vbd::AvbdRigidPointJoint joint = vbd::makeAvbdRigidPrismaticPointJoint(
      /*bodyA=*/0,
      /*bodyB=*/1,
      endpointA,
      endpointB,
      Vec3::Zero(),
      Vec3::Zero(),
      Eigen::Quaterniond::Identity(),
      translationAxis,
      /*startStiffness=*/70.0,
      /*maxStiffness=*/500.0,
      /*row=*/4);

  EXPECT_EQ(joint.linearAxisMask, vbd::avbdRigidJointAllButAxisMask(2u));
  EXPECT_EQ(joint.angularAxisMask, vbd::kAvbdRigidJointAllAxesMask);
  EXPECT_NEAR(joint.linearAxes.col(0).dot(translationAxis), 0.0, 1e-12);
  EXPECT_NEAR(joint.linearAxes.col(1).dot(translationAxis), 0.0, 1e-12);
  EXPECT_NEAR(
      std::abs(joint.linearAxes.col(2).dot(translationAxis)), 1.0, 1e-12);

  vbd::AvbdScalarRowInventory linearInventory;
  vbd::AvbdScalarRowInventory angularInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  const std::vector<vbd::AvbdRigidPointJoint> joints{joint};
  vbd::buildAvbdRigidPointJointConstraintRows(
      states,
      joints,
      linearInventory,
      angularInventory,
      linearRows,
      angularRows);

  ASSERT_EQ(linearRows.size(), 2u);
  ASSERT_EQ(angularRows.size(), 3u);
  EXPECT_EQ(linearInventory[0].descriptor.key.axis, 0u);
  EXPECT_EQ(linearInventory[1].descriptor.key.axis, 1u);
  EXPECT_NEAR(linearRows[0].row.axis.dot(translationAxis), 0.0, 1e-12);
  EXPECT_NEAR(linearRows[1].row.axis.dot(translationAxis), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidAngularPairTargetUsesParentLocalOrientation)
{
  vbd::AvbdRigidBodyState parent;
  parent.orientation = rotationY(0.8);

  const Eigen::Quaterniond parentInitial = rotationZ(-0.4);
  const Eigen::Quaterniond childInitial = rotationX(0.7);
  vbd::AvbdRigidAngularPairRow row;
  row.targetRelativeOrientation = parentInitial.conjugate() * childInitial;

  const Eigen::Quaterniond expected
      = parent.orientation * parentInitial.conjugate() * childInitial;
  EXPECT_NEAR(
      (vbd::avbdRigidAngularPairTargetOrientationB(parent, row)
           .toRotationMatrix()
       - expected.toRotationMatrix())
          .norm(),
      0.0,
      1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidAngularFiniteUpdateKeepsCappedStiffness)
{
  vbd::AvbdRigidBodyState stateA;

  vbd::AvbdRigidBodyState stateB;
  stateB.orientation = rotationZ(0.5);

  vbd::AvbdRigidAngularPairRow row;
  row.axis = Vec3::UnitZ();
  row.materialStiffness = 20.0;
  row.state.stiffness = 20.0;
  row.state.lambda = 99.0;

  vbd::AvbdRigidPointAttachmentOptions options;
  options.beta = 4.0;
  options.maxStiffness = 12.0;

  const vbd::AvbdScalarRowState updated = vbd::updateAvbdRigidAngularPairRow(
      row.state, stateA, stateB, row, options);

  EXPECT_DOUBLE_EQ(updated.lambda, 0.0);
  EXPECT_DOUBLE_EQ(updated.stiffness, 12.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPointJointAngularRowsDriveOrientationsTogether)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[0].orientation = Eigen::Quaterniond::Identity();
  states[1].orientation = rotationZ(0.6);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias
      = {Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  std::vector<vbd::AvbdRigidPointJoint> joints(1);
  joints[0].bodyA = 0;
  joints[0].bodyB = 1;
  joints[0].endpointA = {
      1, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].endpointB = {
      2, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].startStiffness = 100.0;
  joints[0].maxStiffness = 1000.0;

  vbd::AvbdScalarRowInventory angularInventory;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  vbd::buildAvbdRigidPointJointAngularRows(
      states, joints, angularInventory, angularRows);

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairRow> pointPairs;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 8;
  options.regularization = 1e-12;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  rowOptions.beta = 1000.0;
  rowOptions.maxStiffness = 1000.0;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          pointPairs,
          angularRows,
          frictionRows,
          options,
          rowOptions,
          frictionOptions);

  const Vec3 error = vbd::avbdRigidBodyOrientationError(
      states[1].orientation, states[0].orientation);
  EXPECT_GT(stats.bodyUpdates, 0u);
  EXPECT_NEAR(error.x(), 0.0, 1e-12);
  EXPECT_NEAR(error.y(), 0.0, 1e-12);
  EXPECT_LT(std::abs(error.z()), 0.05);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPointJointAxisMasksLeaveHingeAxisFree)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[0].position = Vec3::Zero();
  states[0].orientation = Eigen::Quaterniond::Identity();
  states[1].position = Vec3::UnitX();
  states[1].orientation = rotationZ(0.6);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias
      = {Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  std::vector<vbd::AvbdRigidPointJoint> joints(1);
  joints[0].bodyA = 0;
  joints[0].bodyB = 1;
  joints[0].endpointA = {
      1, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].endpointB = {
      2, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].linearAxisMask = vbd::kAvbdRigidJointAllAxesMask;
  joints[0].angularAxisMask = static_cast<std::uint8_t>((1u << 0) | (1u << 1));
  joints[0].startStiffness = 100.0;
  joints[0].maxStiffness = 1000.0;

  vbd::AvbdScalarRowInventory linearInventory;
  vbd::AvbdScalarRowInventory angularInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  vbd::buildAvbdRigidPointJointConstraintRows(
      states,
      joints,
      linearInventory,
      angularInventory,
      linearRows,
      angularRows);
  ASSERT_EQ(linearRows.size(), 3u);
  ASSERT_EQ(angularRows.size(), 2u);

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 8;
  options.regularization = 1e-12;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  rowOptions.beta = 1000.0;
  rowOptions.maxStiffness = 1000.0;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          linearRows,
          angularRows,
          frictionRows,
          options,
          rowOptions,
          frictionOptions);

  const Vec3 error = vbd::avbdRigidBodyOrientationError(
      states[1].orientation, states[0].orientation);
  EXPECT_GT(stats.bodyUpdates, 0u);
  EXPECT_LT(states[1].position.x(), 0.25);
  EXPECT_NEAR(error.x(), 0.0, 1e-12);
  EXPECT_NEAR(error.y(), 0.0, 1e-12);
  EXPECT_GT(std::abs(error.z()), 0.4);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidPointJointConstraintRowsDrivePoseTogether)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  states[0].position = Vec3::Zero();
  states[0].orientation = Eigen::Quaterniond::Identity();
  states[1].position = Vec3::UnitX();
  states[1].orientation = rotationZ(0.6);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias
      = {Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  std::vector<vbd::AvbdRigidPointJoint> joints(1);
  joints[0].bodyA = 0;
  joints[0].bodyB = 1;
  joints[0].endpointA = {
      1, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].endpointB = {
      2, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  joints[0].startStiffness = 100.0;
  joints[0].maxStiffness = 1000.0;

  vbd::AvbdScalarRowInventory linearInventory;
  vbd::AvbdScalarRowInventory angularInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> linearRows;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  vbd::buildAvbdRigidPointJointConstraintRows(
      states,
      joints,
      linearInventory,
      angularInventory,
      linearRows,
      angularRows);

  ASSERT_EQ(linearRows.size(), 3u);
  ASSERT_EQ(angularRows.size(), 3u);

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 8;
  options.regularization = 1e-12;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  rowOptions.beta = 1000.0;
  rowOptions.maxStiffness = 1000.0;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;

  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          linearRows,
          angularRows,
          frictionRows,
          options,
          rowOptions,
          frictionOptions);

  const Vec3 error = vbd::avbdRigidBodyOrientationError(
      states[1].orientation, states[0].orientation);
  EXPECT_GT(stats.bodyUpdates, 0u);
  EXPECT_LT(states[1].position.x(), 0.25);
  EXPECT_NEAR(states[1].position.y(), 0.0, 1e-12);
  EXPECT_NEAR(states[1].position.z(), 0.0, 1e-12);
  EXPECT_LT(std::abs(error.z()), 0.05);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidContactManifoldBuilderSkipsInactiveRows)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);

  std::vector<vbd::AvbdRigidContactManifoldPoint> contacts(2);
  contacts[0].bodyA = 0;
  contacts[0].bodyB = 1;
  contacts[0].normalFromAtoB = Vec3::UnitX();
  contacts[0].depth = 0.0;
  contacts[1].bodyA = 0;
  contacts[1].bodyB = 1;
  contacts[1].normalFromAtoB = Vec3::Zero();
  contacts[1].depth = 0.1;

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;

  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows);

  EXPECT_TRUE(normalInventory.empty());
  EXPECT_TRUE(frictionInventory.empty());
  EXPECT_TRUE(normalRows.empty());
  EXPECT_TRUE(frictionRows.empty());
}

//==============================================================================
TEST(AvbdRigidBlock, RigidContactManifoldRowsDriveSeparation)
{
  std::vector<vbd::AvbdRigidBodyState> states(2);
  const std::vector<vbd::AvbdRigidBodyState> inertialTargets = states;
  const std::vector<double> masses = {1.0, 1.0};
  const std::vector<Eigen::Matrix3d> inertias{
      Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Identity()};
  const std::vector<std::uint8_t> fixed = {1u, 0u};

  std::vector<vbd::AvbdRigidContactManifoldPoint> contacts(1);
  contacts[0].bodyA = 0;
  contacts[0].bodyB = 1;
  contacts[0].endpointA = {
      1, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  contacts[0].endpointB = {
      2, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  contacts[0].point = Vec3::Zero();
  contacts[0].normalFromAtoB = Vec3::UnitX();
  contacts[0].depth = 0.5;
  contacts[0].startStiffness = 100.0;

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::buildAvbdRigidContactManifoldRows(
      states,
      contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows);

  std::vector<vbd::AvbdRigidBodyPointAttachmentRow> attachments;
  std::vector<vbd::AvbdRigidBodyAngularPairRow> angularRows;
  vbd::AvbdRigidBlockDescentOptions options;
  options.iterations = 1;
  vbd::AvbdRigidPointAttachmentOptions rowOptions;
  vbd::AvbdRigidPointPairFrictionOptions frictionOptions;
  const vbd::AvbdRigidBlockDescentStats stats
      = vbd::blockDescentRigidBodiesAvbdRows(
          states,
          masses,
          inertias,
          fixed,
          inertialTargets,
          /*timeStep=*/1.0,
          attachments,
          normalRows,
          angularRows,
          frictionRows,
          options,
          rowOptions,
          frictionOptions);

  EXPECT_EQ(stats.bodyUpdates, 1u);
  EXPECT_GT(states[1].position.x(), 0.4);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotBuildsManifoldRows)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Vec3(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox(Vec3(2.0, 2.0, 0.25)));
  ground.setFriction(0.25);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 2.0;
  sphereOptions.position = Vec3(0.0, 0.0, 0.45);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sphere.setFriction(0.64);

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_FALSE(contacts.empty());

  vbd::AvbdRigidWorldContactOptions options;
  options.startStiffness = 90.0;
  options.maxStiffness = 700.0;
  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts, options);

  ASSERT_EQ(snapshot.entities.size(), 2u);
  ASSERT_EQ(snapshot.states.size(), snapshot.entities.size());
  ASSERT_EQ(snapshot.masses.size(), snapshot.entities.size());
  ASSERT_EQ(snapshot.bodyInertias.size(), snapshot.entities.size());
  ASSERT_EQ(snapshot.fixed.size(), snapshot.entities.size());
  ASSERT_EQ(snapshot.contacts.size(), contacts.size());

  const std::size_t groundIndex = findEntityIndex(
      snapshot.entities,
      dart::simulation::detail::toRegistryEntity(ground.getEntity()));
  const std::size_t sphereIndex = findEntityIndex(
      snapshot.entities,
      dart::simulation::detail::toRegistryEntity(sphere.getEntity()));
  EXPECT_EQ(snapshot.fixed[groundIndex], 1u);
  EXPECT_EQ(snapshot.fixed[sphereIndex], 0u);
  EXPECT_DOUBLE_EQ(snapshot.masses[sphereIndex], 2.0);
  EXPECT_NEAR(
      (snapshot.states[sphereIndex].position - sphere.getTranslation()).norm(),
      0.0,
      1e-12);

  for (std::size_t i = 0; i < snapshot.contacts.size(); ++i) {
    const vbd::AvbdRigidContactManifoldPoint& manifoldPoint
        = snapshot.contacts[i];
    const sx::Contact& sourceContact = contacts[i];
    const std::size_t bodyA = findEntityIndex(
        snapshot.entities,
        dart::simulation::detail::toRegistryEntity(
            sourceContact.bodyA.getEntity()));
    const std::size_t bodyB = findEntityIndex(
        snapshot.entities,
        dart::simulation::detail::toRegistryEntity(
            sourceContact.bodyB.getEntity()));

    EXPECT_EQ(manifoldPoint.bodyA, bodyA);
    EXPECT_EQ(manifoldPoint.bodyB, bodyB);
    EXPECT_EQ(
        manifoldPoint.endpointA.object,
        vbd::avbdRigidWorldContactObjectId(
            dart::simulation::detail::toRegistryEntity(
                sourceContact.bodyA.getEntity())));
    EXPECT_EQ(
        manifoldPoint.endpointB.object,
        vbd::avbdRigidWorldContactObjectId(
            dart::simulation::detail::toRegistryEntity(
                sourceContact.bodyB.getEntity())));
    const auto expectEndpointFeature
        = [&](const vbd::AvbdContactEndpointId& endpoint, entt::entity entity) {
            if (entity
                == dart::simulation::detail::toRegistryEntity(
                    ground.getEntity())) {
              const std::uint64_t groundFeatureCode
                  = vbd::avbdBoxContactFeatureCode(
                      sourceContact.point - ground.getTranslation(),
                      Vec3(2.0, 2.0, 0.25));
              EXPECT_EQ(
                  vbd::avbdContactFeatureKind(endpoint.feature),
                  vbd::AvbdContactFeatureKind::Face);
              EXPECT_EQ(
                  vbd::avbdContactFeatureLocalIndex(endpoint.feature),
                  vbd::packAvbdBoxContactFeatureId(
                      /*boxIndex=*/0, groundFeatureCode));
            } else {
              EXPECT_EQ(
                  vbd::avbdContactFeatureKind(endpoint.feature),
                  vbd::AvbdContactFeatureKind::Body);
            }
          };
    expectEndpointFeature(
        manifoldPoint.endpointA,
        dart::simulation::detail::toRegistryEntity(
            sourceContact.bodyA.getEntity()));
    expectEndpointFeature(
        manifoldPoint.endpointB,
        dart::simulation::detail::toRegistryEntity(
            sourceContact.bodyB.getEntity()));
    EXPECT_NEAR((manifoldPoint.point - sourceContact.point).norm(), 0.0, 1e-12);
    EXPECT_NEAR(
        (manifoldPoint.normalFromAtoB - sourceContact.normal).norm(),
        0.0,
        1e-12);
    EXPECT_DOUBLE_EQ(manifoldPoint.depth, sourceContact.depth);
    EXPECT_NEAR(manifoldPoint.frictionCoefficient, 0.4, 1e-12);
    EXPECT_DOUBLE_EQ(manifoldPoint.startStiffness, 90.0);
    EXPECT_DOUBLE_EQ(manifoldPoint.maxStiffness, 700.0);
    EXPECT_EQ(manifoldPoint.row, i);
  }

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::buildAvbdRigidContactManifoldRows(
      snapshot.states,
      snapshot.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows);

  ASSERT_EQ(normalRows.size(), snapshot.contacts.size());
  for (std::size_t i = 0; i < normalRows.size(); ++i) {
    const auto& row = normalRows[i];
    const Vec3 pointA = vbd::avbdRigidBodyWorldPoint(
        snapshot.states[row.bodyA], row.row.localPointA);
    const Vec3 pointB = vbd::avbdRigidBodyWorldPoint(
        snapshot.states[row.bodyB], row.row.localPointB);
    EXPECT_NEAR((pointA - snapshot.contacts[i].point).norm(), 0.0, 1e-12);
    EXPECT_NEAR((pointB - snapshot.contacts[i].point).norm(), 0.0, 1e-12);
  }
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotBuildIntoClearsReusedState)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Vec3(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox(Vec3(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Vec3(0.0, 0.0, 0.45);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_FALSE(contacts.empty());

  vbd::AvbdRigidWorldContactSnapshot snapshot;
  snapshot.contacts.reserve(64u);
  snapshot.entities.reserve(64u);
  snapshot.states.reserve(64u);
  snapshot.masses.reserve(64u);
  snapshot.bodyInertias.reserve(64u);
  snapshot.fixed.reserve(64u);
  const std::size_t contactCapacity = snapshot.contacts.capacity();
  const std::size_t entityCapacity = snapshot.entities.capacity();
  const std::size_t stateCapacity = snapshot.states.capacity();

  snapshot.inertialTargets.push_back(vbd::AvbdRigidBodyState{});
  snapshot.joints.push_back(vbd::AvbdRigidPointJoint{});
  snapshot.jointEntities.push_back(entt::null);
  snapshot.linearMotors.push_back(vbd::AvbdRigidLinearMotor{});
  snapshot.motors.push_back(vbd::AvbdRigidAngularMotor{});
  snapshot.distanceSprings.push_back(
      vbd::AvbdRigidBodyPointPairDistanceSpringRow{});
  snapshot.distanceSpringEntities.push_back(entt::null);

  vbd::buildAvbdRigidWorldContactSnapshotInto(
      dart::simulation::detail::registryOf(world), contacts, snapshot);

  ASSERT_EQ(snapshot.contacts.size(), contacts.size());
  EXPECT_TRUE(snapshot.inertialTargets.empty());
  EXPECT_TRUE(snapshot.joints.empty());
  EXPECT_TRUE(snapshot.jointEntities.empty());
  EXPECT_TRUE(snapshot.linearMotors.empty());
  EXPECT_TRUE(snapshot.motors.empty());
  EXPECT_TRUE(snapshot.distanceSprings.empty());
  EXPECT_TRUE(snapshot.distanceSpringEntities.empty());
  EXPECT_GE(snapshot.contacts.capacity(), contactCapacity);
  EXPECT_GE(snapshot.entities.capacity(), entityCapacity);
  EXPECT_GE(snapshot.states.capacity(), stateCapacity);

  const vbd::AvbdRigidWorldContactSnapshot expected
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);
  ASSERT_EQ(snapshot.entities.size(), expected.entities.size());
  ASSERT_EQ(snapshot.contacts.size(), expected.contacts.size());
  EXPECT_EQ(snapshot.fixed, expected.fixed);
  for (std::size_t i = 0; i < snapshot.contacts.size(); ++i) {
    EXPECT_EQ(snapshot.contacts[i].bodyA, expected.contacts[i].bodyA);
    EXPECT_EQ(snapshot.contacts[i].bodyB, expected.contacts[i].bodyB);
    EXPECT_EQ(snapshot.contacts[i].row, expected.contacts[i].row);
  }
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotPersistsFeatureScopedRows)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions boxOptions;
  auto box = world.addRigidBody("box", boxOptions);
  box.setCollisionShape(sx::CollisionShape::makeBox(Vec3(1.0, 1.0, 1.0)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Vec3(2.0, 0.0, 0.0);
  auto sphereA = world.addRigidBody("sphere_a", sphereOptions);
  sphereA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sphereOptions.position = Vec3(0.0, 2.0, 0.0);
  auto sphereB = world.addRigidBody("sphere_b", sphereOptions);
  sphereB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody boxBody(box.getEntity(), &world);
  const sx::CollisionBody sphereBodyA(sphereA.getEntity(), &world);
  const sx::CollisionBody sphereBodyB(sphereB.getEntity(), &world);
  const std::vector<sx::Contact> contacts{
      {boxBody, sphereBodyA, Vec3(1.0, 0.1, 0.0), Vec3::UnitX(), 0.1},
      {boxBody, sphereBodyB, Vec3(0.0, 1.0, 0.1), Vec3::UnitY(), 0.2},
      {boxBody, sphereBodyA, Vec3(1.0, -0.1, 0.0), Vec3::UnitX(), 0.3}};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), contacts.size());
  // Same-feature rows are sorted by canonical-local contact point, not by the
  // input query order, so the lower-y face contact owns the first warm-start
  // row.
  EXPECT_EQ(snapshot.contacts[0].row, 1u);
  EXPECT_EQ(snapshot.contacts[1].row, 0u);
  EXPECT_EQ(snapshot.contacts[2].row, 0u);
  EXPECT_EQ(
      snapshot.contacts[0].endpointA.feature,
      snapshot.contacts[2].endpointA.feature);
  EXPECT_NE(
      snapshot.contacts[0].endpointA.feature,
      snapshot.contacts[1].endpointA.feature);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[1].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
// A compound body carries several collision shapes; a contact on a secondary
// shape must be encoded in that shape's own feature-id sub-range (using the
// shape's local frame and index), not aliased onto shape 0. Without per-shape
// scoping the two contacts below would collapse to the same warm-start row.
// Regression for the multi-shape CollisionGeometry endpoint fix.
TEST(AvbdRigidBlock, RigidWorldContactSnapshotScopesFeatureToCollidingShape)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeBox(Vec3(1.0, 1.0, 1.0)));
  sx::CollisionShape offsetBox
      = sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5));
  offsetBox.localTransform.translation() = Vec3(4.0, 0.0, 0.0);
  compound.addCollisionShape(offsetBox);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Vec3(2.0, 0.0, 0.0);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody compoundBody(compound.getEntity(), &world);
  const sx::CollisionBody sphereBody(sphere.getEntity(), &world);

  // Same +x face hit, but on different shapes of the compound body: the first
  // on the primary box (shape 0), the second on the offset box (shape 1). The
  // offset point expressed in shape 1's local frame is (0.5, 0.1, 0.0), i.e.
  // its own +x face.
  sx::Contact onShape0{
      compoundBody, sphereBody, Vec3(1.0, 0.1, 0.0), Vec3::UnitX(), 0.1};
  onShape0.shapeIndexA = 0;
  onShape0.localPointA = Vec3(1.0, 0.1, 0.0);
  sx::Contact onShape1{
      compoundBody, sphereBody, Vec3(4.5, 0.1, 0.0), Vec3::UnitX(), 0.1};
  onShape1.shapeIndexA = 1;
  onShape1.localPointA = Vec3(0.5, 0.1, 0.0);
  const std::vector<sx::Contact> contacts{onShape0, onShape1};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 2u);
  // Distinct shapes must not alias onto the same feature id.
  EXPECT_NE(
      snapshot.contacts[0].endpointA.feature,
      snapshot.contacts[1].endpointA.feature);
  // Both contacts resolve to a box face once mapped into their own shape frame.
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[1].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
TEST(
    AvbdRigidBlock,
    RigidWorldContactSnapshotMapsSingleShapeFallbackToShapeFrame)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto boxBody = world.addRigidBody("offset_box");
  sx::CollisionShape offsetBox
      = sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5));
  offsetBox.localTransform.translation() = Vec3(4.0, 0.0, 0.0);
  boxBody.setCollisionShape(offsetBox);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Vec3(4.1, 1.0, 0.0);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody boxCollisionBody(boxBody.getEntity(), &world);
  const sx::CollisionBody sphereBody(sphere.getEntity(), &world);
  const Vec3 worldPoint(4.1, 0.5, 0.0);
  const std::vector<sx::Contact> contacts{
      {boxCollisionBody, sphereBody, worldPoint, Vec3::UnitY(), 0.1}};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 1u);
  const Vec3 shapeLocalPoint = offsetBox.localTransform.inverse() * worldPoint;
  const std::uint64_t featureCode
      = vbd::avbdBoxContactFeatureCode(shapeLocalPoint, offsetBox.halfExtents);
  const std::uint64_t bodyFrameFeatureCode
      = vbd::avbdBoxContactFeatureCode(worldPoint, offsetBox.halfExtents);
  EXPECT_NE(featureCode, bodyFrameFeatureCode);
  const std::uint64_t expectedFeature = vbd::packAvbdContactFeatureId(
      vbd::avbdBoxContactFeatureKind(featureCode),
      vbd::packAvbdBoxContactFeatureId(0, featureCode));
  EXPECT_EQ(snapshot.contacts[0].endpointA.feature, expectedFeature);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
TEST(
    AvbdRigidBlock,
    RigidWorldContactSnapshotMapsExplicitEndpointBShapeToShapeFrame)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto sphere = world.addRigidBody("sphere");
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5)));
  sx::CollisionShape offsetBox
      = sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5));
  offsetBox.localTransform.translation() = Vec3(4.0, 0.0, 0.0);
  compound.addCollisionShape(offsetBox);

  const sx::CollisionBody sphereBody(sphere.getEntity(), &world);
  const sx::CollisionBody compoundBody(compound.getEntity(), &world);
  const Vec3 worldPoint(4.1, 0.5, 0.0);
  const Vec3 shapeLocalPoint = offsetBox.localTransform.inverse() * worldPoint;
  sx::Contact contact{
      sphereBody, compoundBody, worldPoint, -Vec3::UnitY(), 0.1};
  contact.shapeIndexB = 1;
  contact.localPointB = shapeLocalPoint;
  const std::vector<sx::Contact> contacts{contact};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 1u);
  const std::uint64_t featureCode
      = vbd::avbdBoxContactFeatureCode(shapeLocalPoint, offsetBox.halfExtents);
  const std::uint64_t defaultLocalFeatureCode
      = vbd::avbdBoxContactFeatureCode(Vec3::Zero(), offsetBox.halfExtents);
  EXPECT_NE(featureCode, defaultLocalFeatureCode);
  const std::uint64_t expectedFeature = vbd::packAvbdContactFeatureId(
      vbd::avbdBoxContactFeatureKind(featureCode),
      vbd::kAvbdRigidWorldShapeFeatureStride
          + vbd::packAvbdBoxContactFeatureId(0, featureCode));
  EXPECT_EQ(snapshot.contacts[0].endpointB.feature, expectedFeature);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointB.feature),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
TEST(
    AvbdRigidBlock,
    RigidWorldContactSnapshotMapsExplicitEndpointAShapeToShapeFrame)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5)));
  sx::CollisionShape offsetBox
      = sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5));
  offsetBox.localTransform.translation() = Vec3(4.0, 0.0, 0.0);
  compound.addCollisionShape(offsetBox);

  auto sphere = world.addRigidBody("sphere");
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody compoundBody(compound.getEntity(), &world);
  const sx::CollisionBody sphereBody(sphere.getEntity(), &world);
  const Vec3 worldPoint(4.1, 0.5, 0.0);
  const Vec3 shapeLocalPoint = offsetBox.localTransform.inverse() * worldPoint;
  sx::Contact contact{compoundBody, sphereBody, worldPoint, Vec3::UnitY(), 0.1};
  contact.shapeIndexA = 1;
  contact.localPointA = shapeLocalPoint;
  const std::vector<sx::Contact> contacts{contact};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 1u);
  const std::uint64_t featureCode
      = vbd::avbdBoxContactFeatureCode(shapeLocalPoint, offsetBox.halfExtents);
  const std::uint64_t defaultLocalFeatureCode
      = vbd::avbdBoxContactFeatureCode(Vec3::Zero(), offsetBox.halfExtents);
  EXPECT_NE(featureCode, defaultLocalFeatureCode);
  const std::uint64_t expectedFeature = vbd::packAvbdContactFeatureId(
      vbd::avbdBoxContactFeatureKind(featureCode),
      vbd::kAvbdRigidWorldShapeFeatureStride
          + vbd::packAvbdBoxContactFeatureId(0, featureCode));
  EXPECT_EQ(snapshot.contacts[0].endpointA.feature, expectedFeature);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
TEST(
    AvbdRigidBlock,
    RigidWorldContactSnapshotUsesNarrowPhaseLocalPointForExplicitShapeFeature)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5)));
  sx::CollisionShape offsetBox
      = sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5));
  offsetBox.localTransform.translation() = Vec3(4.0, 0.0, 0.0);
  compound.addCollisionShape(offsetBox);

  auto sphere = world.addRigidBody("sphere");
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody compoundBody(compound.getEntity(), &world);
  const sx::CollisionBody sphereBody(sphere.getEntity(), &world);
  const Vec3 staleWorldPoint = Vec3::Zero();
  const Vec3 shapeLocalPoint(0.5, 0.1, 0.0);

  sx::Contact endpointA{
      compoundBody, sphereBody, staleWorldPoint, Vec3::UnitX(), 0.1};
  endpointA.shapeIndexA = 1;
  endpointA.localPointA = shapeLocalPoint;

  sx::Contact endpointB{
      sphereBody, compoundBody, staleWorldPoint, -Vec3::UnitX(), 0.1};
  endpointB.shapeIndexB = 1;
  endpointB.localPointB = shapeLocalPoint;

  const std::vector<sx::Contact> contacts{endpointA, endpointB};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 2u);
  const std::uint64_t featureCode
      = vbd::avbdBoxContactFeatureCode(shapeLocalPoint, offsetBox.halfExtents);
  const Vec3 staleShapeLocalPoint
      = offsetBox.localTransform.inverse() * staleWorldPoint;
  const std::uint64_t staleFeatureCode = vbd::avbdBoxContactFeatureCode(
      staleShapeLocalPoint, offsetBox.halfExtents);
  EXPECT_NE(featureCode, staleFeatureCode);

  const std::uint64_t expectedFeature = vbd::packAvbdContactFeatureId(
      vbd::avbdBoxContactFeatureKind(featureCode),
      vbd::kAvbdRigidWorldShapeFeatureStride
          + vbd::packAvbdBoxContactFeatureId(0, featureCode));
  EXPECT_EQ(snapshot.contacts[0].endpointA.feature, expectedFeature);
  EXPECT_EQ(snapshot.contacts[1].endpointB.feature, expectedFeature);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[1].endpointB.feature),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotInfersUniqueCompoundShapeFallback)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5)));
  sx::CollisionShape offsetBox
      = sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5));
  offsetBox.localTransform.translation() = Vec3(4.0, 0.0, 0.0);
  compound.addCollisionShape(offsetBox);

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Vec3(4.1, 1.0, 0.0);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody compoundBody(compound.getEntity(), &world);
  const sx::CollisionBody sphereBody(sphere.getEntity(), &world);
  const Vec3 worldPoint(4.1, 0.5, 0.0);
  const std::vector<sx::Contact> contacts{
      {compoundBody, sphereBody, worldPoint, Vec3::UnitY(), 0.1}};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 1u);
  const Vec3 shapeLocalPoint = offsetBox.localTransform.inverse() * worldPoint;
  const std::uint64_t featureCode
      = vbd::avbdBoxContactFeatureCode(shapeLocalPoint, offsetBox.halfExtents);
  const std::uint64_t expectedFeature = vbd::packAvbdContactFeatureId(
      vbd::avbdBoxContactFeatureKind(featureCode),
      vbd::kAvbdRigidWorldShapeFeatureStride
          + vbd::packAvbdBoxContactFeatureId(0, featureCode));
  EXPECT_EQ(snapshot.contacts[0].endpointA.feature, expectedFeature);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
TEST(
    AvbdRigidBlock,
    RigidWorldContactSnapshotLeavesAmbiguousCompoundShapeFallbackAtBody)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5)));
  compound.addCollisionShape(sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Vec3(1.0, 0.0, 0.0);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody compoundBody(compound.getEntity(), &world);
  const sx::CollisionBody sphereBody(sphere.getEntity(), &world);
  const Vec3 worldPoint(0.5, 0.0, 0.0);
  const std::vector<sx::Contact> contacts{
      {compoundBody, sphereBody, worldPoint, Vec3::UnitX(), 0.1}};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 1u);
  const entt::entity compoundEntity
      = dart::simulation::detail::toRegistryEntity(compound.getEntity());
  EXPECT_EQ(
      snapshot.contacts[0].endpointA.feature,
      vbd::avbdRigidWorldBodyEndpointId(compoundEntity).feature);
}

//==============================================================================
// A compound body that mixes shape types must keep feature ids disjoint across
// types too. The per-type packer offsets only reserve room for one shape of
// each type, so different shape slots need separate blocks before applying
// primitive-local feature ids.
TEST(AvbdRigidBlock, RigidWorldContactSnapshotKeepsCompoundShapeTypesDisjoint)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeBox(Vec3(1.0, 1.0, 1.0)));
  sx::CollisionShape offsetBox
      = sx::CollisionShape::makeBox(Vec3(0.5, 0.5, 0.5));
  offsetBox.localTransform.translation() = Vec3(4.0, 0.0, 0.0);
  compound.addCollisionShape(offsetBox); // shape index 1
  sx::CollisionShape offsetCylinder
      = sx::CollisionShape::makeCylinder(/*radius=*/1.0, /*halfHeight=*/2.0);
  offsetCylinder.localTransform.translation() = Vec3(0.0, 4.0, 0.0);
  compound.addCollisionShape(offsetCylinder); // shape index 2

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Vec3(2.0, 0.0, 0.0);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody compoundBody(compound.getEntity(), &world);
  const sx::CollisionBody sphereBody(sphere.getEntity(), &world);

  // -x face of the box at shape index 1 (shape-local (-0.5, 0.1, 0.0)).
  sx::Contact onBox{
      compoundBody, sphereBody, Vec3(3.5, 0.1, 0.0), Vec3::UnitX(), 0.1};
  onBox.shapeIndexA = 1;
  onBox.localPointA = Vec3(-0.5, 0.1, 0.0);
  // -z cap of the cylinder at shape index 2 (shape-local (0.0, 0.0, -1.9)).
  sx::Contact onCylinder{
      compoundBody, sphereBody, Vec3(0.0, 4.0, -1.9), Vec3::UnitZ(), 0.1};
  onCylinder.shapeIndexA = 2;
  onCylinder.localPointA = Vec3(0.0, 0.0, -1.9);
  const std::vector<sx::Contact> contacts{onBox, onCylinder};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 2u);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[1].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
  // A box on one shape and a cylinder on another must not share a feature id.
  EXPECT_NE(
      snapshot.contacts[0].endpointA.feature,
      snapshot.contacts[1].endpointA.feature);
}

//==============================================================================
// Shape types without a specialized feature code (sphere) still must be scoped
// by shape index, so two such shapes on one compound body do not alias onto the
// same body-level warm-start row.
TEST(AvbdRigidBlock, RigidWorldContactSnapshotScopesUnsupportedShapesByIndex)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeSphere(0.5)); // shape 0
  sx::CollisionShape offsetSphere = sx::CollisionShape::makeSphere(0.5);
  offsetSphere.localTransform.translation() = Vec3(4.0, 0.0, 0.0);
  compound.addCollisionShape(offsetSphere); // shape index 1

  sx::RigidBodyOptions otherOptions;
  otherOptions.position = Vec3(2.0, 0.0, 0.0);
  auto other = world.addRigidBody("other", otherOptions);
  other.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody compoundBody(compound.getEntity(), &world);
  const sx::CollisionBody otherBody(other.getEntity(), &world);

  sx::Contact onShape0{
      compoundBody, otherBody, Vec3(0.5, 0.0, 0.0), Vec3::UnitX(), 0.1};
  onShape0.shapeIndexA = 0;
  onShape0.localPointA = Vec3(0.5, 0.0, 0.0);
  sx::Contact onShape1{
      compoundBody, otherBody, Vec3(3.5, 0.0, 0.0), Vec3::UnitX(), 0.1};
  onShape1.shapeIndexA = 1;
  onShape1.localPointA = Vec3(-0.5, 0.0, 0.0);
  const std::vector<sx::Contact> contacts{onShape0, onShape1};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 2u);
  // Spheres have no face/edge code, so both fall back to a body feature — but
  // scoped per shape, so the two shapes must not collapse to one row.
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Body);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[1].endpointA.feature),
      vbd::AvbdContactFeatureKind::Body);
  EXPECT_NE(
      snapshot.contacts[0].endpointA.feature,
      snapshot.contacts[1].endpointA.feature);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotInfersUniquePlaneShapeFeature)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sx::CollisionShape offsetPlane
      = sx::CollisionShape::makePlane(Vec3::UnitZ(), 0.0);
  offsetPlane.localTransform.translation() = Vec3(4.0, 0.0, 0.0);
  compound.addCollisionShape(offsetPlane);

  sx::RigidBodyOptions otherOptions;
  otherOptions.position = Vec3(4.25, 0.25, 1.0);
  auto other = world.addRigidBody("other", otherOptions);
  other.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody compoundBody(compound.getEntity(), &world);
  const sx::CollisionBody otherBody(other.getEntity(), &world);
  const Vec3 worldPoint(4.25, 0.25, 0.0);
  const std::vector<sx::Contact> contacts{
      {compoundBody, otherBody, worldPoint, Vec3::UnitZ(), 0.1}};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 1u);
  const std::uint64_t expectedFeature = vbd::packAvbdContactFeatureId(
      vbd::AvbdContactFeatureKind::Face,
      vbd::kAvbdRigidWorldShapeFeatureStride
          + vbd::kAvbdRigidWorldPlaneContactFeatureIdOffset);
  EXPECT_EQ(snapshot.contacts[0].endpointA.feature, expectedFeature);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotInfersUniqueMeshShapeFallback)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto compound = world.addRigidBody("compound");
  compound.addCollisionShape(sx::CollisionShape::makeSphere(0.5));
  sx::CollisionShape offsetMesh = sx::CollisionShape::makeMesh(
      {Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)},
      {Eigen::Vector3i(0, 1, 2)});
  offsetMesh.localTransform.translation() = Vec3(4.0, 0.0, 0.0);
  compound.addCollisionShape(offsetMesh);

  sx::RigidBodyOptions otherOptions;
  otherOptions.position = Vec3(4.25, 0.25, 1.0);
  auto other = world.addRigidBody("other", otherOptions);
  other.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody compoundBody(compound.getEntity(), &world);
  const sx::CollisionBody otherBody(other.getEntity(), &world);
  const Vec3 worldPoint(4.25, 0.25, 0.0);
  const std::vector<sx::Contact> contacts{
      {compoundBody, otherBody, worldPoint, Vec3::UnitZ(), 0.1}};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 1u);
  const std::uint64_t expectedFeature = vbd::packAvbdContactFeatureId(
      vbd::AvbdContactFeatureKind::Face,
      vbd::kAvbdRigidWorldShapeFeatureStride
          + vbd::kAvbdRigidWorldMeshContactFeatureIdOffset);
  EXPECT_EQ(snapshot.contacts[0].endpointA.feature, expectedFeature);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotUsesMeshFaceFeatureIds)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto meshBody = world.addRigidBody("mesh");
  meshBody.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Vec3(0.0, 0.0, 0.0),
           Vec3(1.0, 0.0, 0.0),
           Vec3(1.0, 1.0, 0.0),
           Vec3(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2), Eigen::Vector3i(0, 2, 3)}));

  sx::RigidBodyOptions otherOptions;
  otherOptions.position = Vec3(0.5, 0.5, 1.0);
  auto other = world.addRigidBody("other", otherOptions);
  other.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody meshCollisionBody(meshBody.getEntity(), &world);
  const sx::CollisionBody otherBody(other.getEntity(), &world);
  sx::Contact onTriangle0{
      meshCollisionBody, otherBody, Vec3(0.75, 0.25, 0.0), Vec3::UnitZ(), 0.1};
  onTriangle0.shapeIndexA = 0;
  onTriangle0.localPointA = Vec3(0.75, 0.25, 0.0);
  sx::Contact onTriangle1{
      meshCollisionBody, otherBody, Vec3(0.25, 0.75, 0.0), Vec3::UnitZ(), 0.1};
  onTriangle1.shapeIndexA = 0;
  onTriangle1.localPointA = Vec3(0.25, 0.75, 0.0);
  const std::vector<sx::Contact> contacts{onTriangle0, onTriangle1};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 2u);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureLocalIndex(snapshot.contacts[0].endpointA.feature),
      vbd::kAvbdRigidWorldMeshContactFeatureIdOffset);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[1].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureLocalIndex(snapshot.contacts[1].endpointA.feature),
      vbd::kAvbdRigidWorldMeshContactFeatureIdOffset + 1u);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotRowsIgnoreContactOrder)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto meshBody = world.addRigidBody("mesh");
  meshBody.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));

  sx::RigidBodyOptions otherOptions;
  otherOptions.position = Vec3(0.5, 0.5, 1.0);
  auto other = world.addRigidBody("other", otherOptions);
  other.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody meshCollisionBody(meshBody.getEntity(), &world);
  const sx::CollisionBody otherBody(other.getEntity(), &world);
  sx::Contact left{
      meshCollisionBody, otherBody, Vec3(0.25, 0.25, 0.0), Vec3::UnitZ(), 0.1};
  left.shapeIndexA = 0;
  left.localPointA = Vec3(0.25, 0.25, 0.0);
  sx::Contact right{
      meshCollisionBody, otherBody, Vec3(0.75, 0.1, 0.0), Vec3::UnitZ(), 0.1};
  right.shapeIndexA = 0;
  right.localPointA = Vec3(0.75, 0.1, 0.0);

  const auto buildSnapshot = [&](std::vector<sx::Contact> contacts) {
    return vbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world), contacts);
  };
  const auto rowAtX = [](const vbd::AvbdRigidWorldContactSnapshot& snapshot,
                         double x) -> std::optional<std::uint32_t> {
    for (const vbd::AvbdRigidContactManifoldPoint& contact :
         snapshot.contacts) {
      if (std::abs(contact.point.x() - x) <= 1e-12) {
        return contact.row;
      }
    }
    return std::nullopt;
  };

  const vbd::AvbdRigidWorldContactSnapshot forward
      = buildSnapshot({left, right});
  const vbd::AvbdRigidWorldContactSnapshot reversed
      = buildSnapshot({right, left});

  ASSERT_EQ(forward.contacts.size(), 2u);
  ASSERT_EQ(reversed.contacts.size(), 2u);
  ASSERT_TRUE(rowAtX(forward, 0.25).has_value());
  ASSERT_TRUE(rowAtX(reversed, 0.25).has_value());
  ASSERT_TRUE(rowAtX(forward, 0.75).has_value());
  ASSERT_TRUE(rowAtX(reversed, 0.75).has_value());
  EXPECT_EQ(*rowAtX(forward, 0.25), *rowAtX(reversed, 0.25));
  EXPECT_EQ(*rowAtX(forward, 0.75), *rowAtX(reversed, 0.75));
  EXPECT_NE(*rowAtX(reversed, 0.25), *rowAtX(reversed, 0.75));

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;
  vbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);
  ASSERT_EQ(normalInventory.size(), 2u);
  for (std::size_t i = 0; i < normalInventory.size(); ++i) {
    normalInventory[i].state.lambda
        = 10.0 + static_cast<double>(normalInventory[i].descriptor.key.row);
  }

  vbd::buildAvbdRigidContactManifoldRows(
      reversed.states,
      reversed.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);
  ASSERT_EQ(normalInventory.size(), 2u);
  ASSERT_EQ(normalRows.size(), 2u);
  for (std::size_t i = 0; i < normalInventory.size(); ++i) {
    const double expectedLambda
        = 10.0 + static_cast<double>(normalInventory[i].descriptor.key.row);
    EXPECT_DOUBLE_EQ(normalInventory[i].state.lambda, expectedLambda);
    EXPECT_DOUBLE_EQ(normalRows[i].row.state.lambda, expectedLambda);
  }
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactRowsFallbackUsesSnapshotAllocator)
{
  CountingMemoryAllocator allocator;
  vbd::AvbdRigidWorldContactSnapshot snapshot(allocator);
  snapshot.contacts.reserve(2u);

  const vbd::AvbdContactEndpointId endpointA{
      1u, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};
  const vbd::AvbdContactEndpointId endpointB{
      2u, vbd::packAvbdContactFeatureId(vbd::AvbdContactFeatureKind::Body, 0)};

  vbd::AvbdRigidContactManifoldPoint right;
  right.endpointA = endpointA;
  right.endpointB = endpointB;
  right.point = Vec3(0.75, 0.0, 0.0);
  right.row = 99u;
  vbd::AvbdRigidContactManifoldPoint left = right;
  left.point = Vec3(0.25, 0.0, 0.0);

  snapshot.contacts.push_back(right);
  snapshot.contacts.push_back(left);

  const std::size_t allocationsBeforeRows = allocator.allocations;
  vbd::detail::assignAvbdRigidWorldContactRows(snapshot);

  EXPECT_GT(allocator.allocations, allocationsBeforeRows)
      << "row-assignment fallback scratch should borrow the snapshot "
         "allocator";
  ASSERT_EQ(snapshot.contacts.size(), 2u);
  EXPECT_EQ(snapshot.contacts[0].row, 1u);
  EXPECT_EQ(snapshot.contacts[1].row, 0u);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotRowsIgnoreEndpointOrder)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto meshBody = world.addRigidBody("mesh");
  meshBody.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));

  sx::RigidBodyOptions otherOptions;
  otherOptions.position = Vec3(0.5, 0.5, 1.0);
  auto other = world.addRigidBody("other", otherOptions);
  other.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody meshCollisionBody(meshBody.getEntity(), &world);
  const sx::CollisionBody otherBody(other.getEntity(), &world);
  sx::Contact left{
      meshCollisionBody, otherBody, Vec3(0.25, 0.25, 0.0), Vec3::UnitZ(), 0.1};
  left.shapeIndexA = 0;
  left.localPointA = Vec3(0.25, 0.25, 0.0);
  sx::Contact right{
      meshCollisionBody, otherBody, Vec3(0.75, 0.1, 0.0), Vec3::UnitZ(), 0.1};
  right.shapeIndexA = 0;
  right.localPointA = Vec3(0.75, 0.1, 0.0);

  sx::Contact leftSwapped{
      otherBody, meshCollisionBody, left.point, -Vec3::UnitZ(), left.depth};
  leftSwapped.shapeIndexB = 0;
  leftSwapped.localPointB = left.localPointA;
  sx::Contact rightSwapped{
      otherBody, meshCollisionBody, right.point, -Vec3::UnitZ(), right.depth};
  rightSwapped.shapeIndexB = 0;
  rightSwapped.localPointB = right.localPointA;

  const auto buildSnapshot = [&](std::vector<sx::Contact> contacts) {
    return vbd::buildAvbdRigidWorldContactSnapshot(
        dart::simulation::detail::registryOf(world), contacts);
  };
  const auto rowAtX = [](const vbd::AvbdRigidWorldContactSnapshot& snapshot,
                         double x) -> std::optional<std::uint32_t> {
    for (const vbd::AvbdRigidContactManifoldPoint& contact :
         snapshot.contacts) {
      if (std::abs(contact.point.x() - x) <= 1e-12) {
        return contact.row;
      }
    }
    return std::nullopt;
  };

  const vbd::AvbdRigidWorldContactSnapshot forward
      = buildSnapshot({left, right});
  const vbd::AvbdRigidWorldContactSnapshot swapped
      = buildSnapshot({leftSwapped, rightSwapped});

  ASSERT_EQ(forward.contacts.size(), 2u);
  ASSERT_EQ(swapped.contacts.size(), 2u);
  ASSERT_TRUE(rowAtX(forward, 0.25).has_value());
  ASSERT_TRUE(rowAtX(swapped, 0.25).has_value());
  ASSERT_TRUE(rowAtX(forward, 0.75).has_value());
  ASSERT_TRUE(rowAtX(swapped, 0.75).has_value());
  EXPECT_EQ(*rowAtX(forward, 0.25), *rowAtX(swapped, 0.25));
  EXPECT_EQ(*rowAtX(forward, 0.75), *rowAtX(swapped, 0.75));
  EXPECT_NE(*rowAtX(swapped, 0.25), *rowAtX(swapped, 0.75));

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  std::vector<vbd::AvbdRigidBodyPointPairRow> normalRows;
  std::vector<vbd::AvbdRigidBodyPointPairFrictionRows> frictionRows;
  vbd::AvbdRowWarmStartOptions warmStart;
  warmStart.alpha = 1.0;
  warmStart.gamma = 1.0;
  vbd::buildAvbdRigidContactManifoldRows(
      forward.states,
      forward.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);
  ASSERT_EQ(normalInventory.size(), 2u);
  for (std::size_t i = 0; i < normalInventory.size(); ++i) {
    normalInventory[i].state.lambda
        = 20.0 + static_cast<double>(normalInventory[i].descriptor.key.row);
  }

  vbd::buildAvbdRigidContactManifoldRows(
      swapped.states,
      swapped.contacts,
      normalInventory,
      frictionInventory,
      normalRows,
      frictionRows,
      warmStart);
  ASSERT_EQ(normalInventory.size(), 2u);
  ASSERT_EQ(normalRows.size(), 2u);
  for (std::size_t i = 0; i < normalInventory.size(); ++i) {
    const double expectedLambda
        = 20.0 + static_cast<double>(normalInventory[i].descriptor.key.row);
    EXPECT_DOUBLE_EQ(normalInventory[i].state.lambda, expectedLambda);
    EXPECT_DOUBLE_EQ(normalRows[i].row.state.lambda, expectedLambda);
  }
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotUsesMeshEdgeAndVertexFeatureIds)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto meshBody = world.addRigidBody("mesh");
  meshBody.setCollisionShape(
      sx::CollisionShape::makeMesh(
          {Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0), Vec3(0.0, 1.0, 0.0)},
          {Eigen::Vector3i(0, 1, 2)}));

  sx::RigidBodyOptions otherOptions;
  otherOptions.position = Vec3(0.5, 0.5, 1.0);
  auto other = world.addRigidBody("other", otherOptions);
  other.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody meshCollisionBody(meshBody.getEntity(), &world);
  const sx::CollisionBody otherBody(other.getEntity(), &world);
  sx::Contact onEdge{
      meshCollisionBody, otherBody, Vec3(0.25, 0.0, 0.0), Vec3::UnitZ(), 0.1};
  onEdge.shapeIndexA = 0;
  onEdge.localPointA = Vec3(0.25, 0.0, 0.0);
  sx::Contact onVertex{
      meshCollisionBody, otherBody, Vec3::Zero(), Vec3::UnitZ(), 0.1};
  onVertex.shapeIndexA = 0;
  onVertex.localPointA = Vec3::Zero();
  const std::vector<sx::Contact> contacts{onEdge, onVertex};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 2u);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Edge);
  EXPECT_EQ(
      vbd::avbdContactFeatureLocalIndex(snapshot.contacts[0].endpointA.feature),
      vbd::kAvbdRigidWorldMeshContactFeatureIdOffset + 1u);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[1].endpointA.feature),
      vbd::AvbdContactFeatureKind::Vertex);
  EXPECT_EQ(
      vbd::avbdContactFeatureLocalIndex(snapshot.contacts[1].endpointA.feature),
      vbd::kAvbdRigidWorldMeshContactFeatureIdOffset);
}

//==============================================================================
// Box contacts resolve to Face/Edge/Vertex feature kinds through the production
// snapshot path, at parity with the existing cylinder/mesh/capsule feature-ID
// guards. The mesh/cylinder tests cover Edge/Vertex for their primitives, but
// every box snapshot test only ever resolves to Face; this guards the box
// Edge/Vertex branches of `avbdRigidWorldContactEndpointId` so a contact
// migrating between a face, edge, and corner gets a distinct feature id (and
// therefore resets its warm-start row instead of aliasing onto a stale one).
TEST(AvbdRigidBlock, RigidWorldContactSnapshotUsesBoxEdgeAndVertexFeatureIds)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  auto boxBody = world.addRigidBody("box");
  boxBody.setCollisionShape(sx::CollisionShape::makeBox(Vec3(1.0, 1.0, 1.0)));

  sx::RigidBodyOptions probeOptions;
  probeOptions.position = Vec3(3.0, 0.0, 0.0);
  auto probe = world.addRigidBody("probe", probeOptions);
  probe.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody boxCollisionBody(boxBody.getEntity(), &world);
  const sx::CollisionBody probeBody(probe.getEntity(), &world);

  // Body-local points strictly outside the unit cube (half extents (1, 1, 1))
  // on one, two, and three axes resolve to Face, Edge, and Vertex respectively.
  const Vec3 onFacePoint(1.1, 0.1, 0.0);
  const Vec3 onEdgePoint(1.1, 1.1, 0.0);
  const Vec3 onVertexPoint(1.1, 1.1, 1.1);

  sx::Contact onFace{
      boxCollisionBody, probeBody, onFacePoint, Vec3::UnitX(), 0.1};
  onFace.shapeIndexA = 0;
  onFace.localPointA = onFacePoint;
  sx::Contact onEdge{
      boxCollisionBody, probeBody, onEdgePoint, Vec3::UnitX(), 0.1};
  onEdge.shapeIndexA = 0;
  onEdge.localPointA = onEdgePoint;
  sx::Contact onVertex{
      boxCollisionBody, probeBody, onVertexPoint, Vec3::UnitX(), 0.1};
  onVertex.shapeIndexA = 0;
  onVertex.localPointA = onVertexPoint;
  const std::vector<sx::Contact> contacts{onFace, onEdge, onVertex};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), 3u);

  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointA.feature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[1].endpointA.feature),
      vbd::AvbdContactFeatureKind::Edge);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[2].endpointA.feature),
      vbd::AvbdContactFeatureKind::Vertex);

  // The three box features must be pairwise distinct.
  EXPECT_NE(
      snapshot.contacts[0].endpointA.feature,
      snapshot.contacts[1].endpointA.feature);
  EXPECT_NE(
      snapshot.contacts[0].endpointA.feature,
      snapshot.contacts[2].endpointA.feature);
  EXPECT_NE(
      snapshot.contacts[1].endpointA.feature,
      snapshot.contacts[2].endpointA.feature);

  // Local index matches the production box feature packing for the single-shape
  // body (shape block 0).
  const Vec3 halfExtents(1.0, 1.0, 1.0);
  EXPECT_EQ(
      vbd::avbdContactFeatureLocalIndex(snapshot.contacts[0].endpointA.feature),
      vbd::packAvbdBoxContactFeatureId(
          0, vbd::avbdBoxContactFeatureCode(onFacePoint, halfExtents)));
  EXPECT_EQ(
      vbd::avbdContactFeatureLocalIndex(snapshot.contacts[1].endpointA.feature),
      vbd::packAvbdBoxContactFeatureId(
          0, vbd::avbdBoxContactFeatureCode(onEdgePoint, halfExtents)));
  EXPECT_EQ(
      vbd::avbdContactFeatureLocalIndex(snapshot.contacts[2].endpointA.feature),
      vbd::packAvbdBoxContactFeatureId(
          0, vbd::avbdBoxContactFeatureCode(onVertexPoint, halfExtents)));
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotUsesCylinderFeatureIds)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions cylinderOptions;
  auto cylinder = world.addRigidBody("cylinder", cylinderOptions);
  cylinder.setCollisionShape(
      sx::CollisionShape::makeCylinder(/*radius=*/1.0, /*halfHeight=*/2.0));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Vec3(3.0, 0.0, 0.0);
  auto sphereA = world.addRigidBody("sphere_a", sphereOptions);
  sphereA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sphereOptions.position = Vec3(0.0, 0.0, 3.0);
  auto sphereB = world.addRigidBody("sphere_b", sphereOptions);
  sphereB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody cylinderBody(cylinder.getEntity(), &world);
  const sx::CollisionBody sphereBodyA(sphereA.getEntity(), &world);
  const sx::CollisionBody sphereBodyB(sphereB.getEntity(), &world);
  const std::vector<sx::Contact> contacts{
      {cylinderBody, sphereBodyA, Vec3(1.1, 0.0, 0.0), Vec3::UnitX(), 0.1},
      {cylinderBody, sphereBodyB, Vec3(0.0, 0.0, 2.1), Vec3::UnitZ(), 0.2},
      {cylinderBody,
       sphereBodyA,
       Vec3(1.1, 0.0, 2.1),
       Vec3(1.0, 0.0, 1.0).normalized(),
       0.3}};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), contacts.size());
  EXPECT_EQ(snapshot.contacts[0].row, 0u);
  EXPECT_EQ(snapshot.contacts[1].row, 0u);
  EXPECT_EQ(snapshot.contacts[2].row, 0u);

  const std::uint64_t sideFeature = snapshot.contacts[0].endpointA.feature;
  const std::uint64_t capFeature = snapshot.contacts[1].endpointA.feature;
  const std::uint64_t rimFeature = snapshot.contacts[2].endpointA.feature;
  EXPECT_NE(sideFeature, capFeature);
  EXPECT_NE(sideFeature, rimFeature);
  EXPECT_NE(capFeature, rimFeature);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(sideFeature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(capFeature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(rimFeature),
      vbd::AvbdContactFeatureKind::Edge);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(snapshot.contacts[0].endpointB.feature),
      vbd::AvbdContactFeatureKind::Body);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotUsesCapsuleFeatureIds)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions capsuleOptions;
  auto capsule = world.addRigidBody("capsule", capsuleOptions);
  capsule.setCollisionShape(
      sx::CollisionShape::makeCapsule(/*radius=*/1.0, /*halfHeight=*/2.0));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.position = Vec3(3.0, 0.0, 0.0);
  auto sphereA = world.addRigidBody("sphere_a", sphereOptions);
  sphereA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sphereOptions.position = Vec3(0.0, 0.0, 3.0);
  auto sphereB = world.addRigidBody("sphere_b", sphereOptions);
  sphereB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sphereOptions.position = Vec3(0.0, 0.0, -3.0);
  auto sphereC = world.addRigidBody("sphere_c", sphereOptions);
  sphereC.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const sx::CollisionBody capsuleBody(capsule.getEntity(), &world);
  const sx::CollisionBody sphereBodyA(sphereA.getEntity(), &world);
  const sx::CollisionBody sphereBodyB(sphereB.getEntity(), &world);
  const sx::CollisionBody sphereBodyC(sphereC.getEntity(), &world);
  const std::vector<sx::Contact> contacts{
      {capsuleBody, sphereBodyA, Vec3(1.1, 0.0, 0.0), Vec3::UnitX(), 0.1},
      {capsuleBody, sphereBodyB, Vec3(0.0, 0.0, 2.1), Vec3::UnitZ(), 0.2},
      {capsuleBody, sphereBodyC, Vec3(0.0, 0.0, -2.1), -Vec3::UnitZ(), 0.3}};

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  ASSERT_EQ(snapshot.contacts.size(), contacts.size());
  EXPECT_EQ(snapshot.contacts[0].row, 0u);
  EXPECT_EQ(snapshot.contacts[1].row, 0u);
  EXPECT_EQ(snapshot.contacts[2].row, 0u);

  const std::uint64_t sideFeature = snapshot.contacts[0].endpointA.feature;
  const std::uint64_t topFeature = snapshot.contacts[1].endpointA.feature;
  const std::uint64_t bottomFeature = snapshot.contacts[2].endpointA.feature;
  EXPECT_NE(sideFeature, topFeature);
  EXPECT_NE(sideFeature, bottomFeature);
  EXPECT_NE(topFeature, bottomFeature);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(sideFeature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(topFeature),
      vbd::AvbdContactFeatureKind::Face);
  EXPECT_EQ(
      vbd::avbdContactFeatureKind(bottomFeature),
      vbd::AvbdContactFeatureKind::Face);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldSnapshotSolvesPointJointRows)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  linkOptions.orientation = rotationZ(0.6);
  auto link = world.addRigidBody("link", linkOptions);

  vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          std::span<const sx::Contact>());

  std::vector<vbd::AvbdRigidWorldPointJointInput> joints(1);
  joints[0].bodyA
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  joints[0].bodyB
      = dart::simulation::detail::toRegistryEntity(link.getEntity());
  joints[0].anchorA = Vec3::Zero();
  joints[0].anchorB = Vec3::UnitX();
  joints[0].targetRelativeOrientation = Eigen::Quaterniond::Identity();
  joints[0].startStiffness = 100.0;
  joints[0].maxStiffness = 1000.0;
  EXPECT_EQ(
      vbd::appendAvbdRigidWorldPointJoints(
          dart::simulation::detail::registryOf(world), joints, snapshot),
      1u);
  ASSERT_EQ(snapshot.joints.size(), 1u);

  const std::size_t linkIndex = findEntityIndex(
      snapshot.entities,
      dart::simulation::detail::toRegistryEntity(link.getEntity()));
  const double initialLinkX = snapshot.states[linkIndex].position.x();

  vbd::AvbdRigidWorldContactSolveOptions solveOptions;
  solveOptions.descent.iterations = 8;
  solveOptions.descent.regularization = 1e-12;
  solveOptions.row.beta = 1000.0;
  solveOptions.row.maxStiffness = 1000.0;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  vbd::AvbdScalarRowInventory jointLinearInventory;
  vbd::AvbdScalarRowInventory jointAngularInventory;
  const vbd::AvbdRigidWorldContactSolveResult solveResult
      = vbd::solveAvbdRigidWorldContactSnapshot(
          snapshot,
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          /*timeStep=*/1.0,
          solveOptions);

  EXPECT_EQ(solveResult.normalRows, 0u);
  EXPECT_EQ(solveResult.frictionRows, 0u);
  EXPECT_EQ(solveResult.jointLinearRows, 3u);
  EXPECT_EQ(solveResult.jointAngularRows, 3u);
  EXPECT_GT(solveResult.stats.bodyUpdates, 0u);
  EXPECT_EQ(jointLinearInventory.size(), 3u);
  EXPECT_EQ(jointAngularInventory.size(), 3u);
  EXPECT_LT(snapshot.states[linkIndex].position.x(), 0.25 * initialLinkX);
  const Vec3 error = vbd::avbdRigidBodyOrientationError(
      snapshot.states[linkIndex].orientation, Eigen::Quaterniond::Identity());
  EXPECT_LT(std::abs(error.z()), 0.05);

  const vbd::AvbdRigidWorldContactApplyResult applyResult
      = vbd::applyAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          snapshot,
          /*timeStep=*/1.0);
  EXPECT_EQ(applyResult.bodies, 1u);
  EXPECT_NEAR(link.getTransform().translation().x(), 0.0, 0.25);
  EXPECT_LT(std::abs(link.getAngularVelocity().z()), 0.65);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactApplyReusesSnapshotDirtyStackAllocator)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.mass = 1.0;
  auto body = world.addRigidBody("body", bodyOptions);

  common::MemoryManager memoryManager;
  auto& allocator = memoryManager.getFreeAllocator();
  auto& freeList = memoryManager.getFreeListAllocator();

  vbd::AvbdRigidWorldContactSnapshot snapshot(allocator);
  snapshot.entities.push_back(
      dart::simulation::detail::toRegistryEntity(body.getEntity()));
  snapshot.states.push_back(
      vbd::AvbdRigidBodyState{Vec3(0.25, 0.0, 0.0), rotationZ(0.1)});
  snapshot.fixed.push_back(0u);

  EXPECT_EQ(
      snapshot.frameDirtyStack.get_allocator(),
      common::StlAllocator<entt::entity>{allocator});

  const auto allocationsBeforeApply = freeList.getAllocationCount();
  const vbd::AvbdRigidWorldContactApplyResult firstApply
      = vbd::applyAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          snapshot,
          /*timeStep=*/1.0);

  EXPECT_EQ(firstApply.bodies, 1u);
  EXPECT_GT(freeList.getAllocationCount(), allocationsBeforeApply)
      << "AVBD rigid-world contact writeback should allocate frame dirty "
         "traversal scratch through the snapshot allocator";

  const auto allocationsAfterFirstApply = freeList.getAllocationCount();
  snapshot.states[0].position = Vec3(0.5, 0.0, 0.0);
  const vbd::AvbdRigidWorldContactApplyResult secondApply
      = vbd::applyAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          snapshot,
          /*timeStep=*/1.0);

  EXPECT_EQ(secondApply.bodies, 1u);
  EXPECT_EQ(freeList.getAllocationCount(), allocationsAfterFirstApply)
      << "AVBD rigid-world contact writeback should reuse the snapshot-owned "
         "dirty traversal stack for same-shape writes";
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactStepSolvesPointJointRows)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  linkOptions.orientation = rotationZ(0.6);
  auto link = world.addRigidBody("link", linkOptions);

  std::vector<vbd::AvbdRigidWorldPointJointInput> joints(1);
  joints[0].bodyA
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  joints[0].bodyB
      = dart::simulation::detail::toRegistryEntity(link.getEntity());
  joints[0].anchorA = Vec3::Zero();
  joints[0].anchorB = Vec3::UnitX();
  joints[0].targetRelativeOrientation = Eigen::Quaterniond::Identity();
  joints[0].startStiffness = 100.0;
  joints[0].maxStiffness = 1000.0;

  vbd::AvbdRigidWorldContactStepOptions stepOptions;
  stepOptions.solve.descent.iterations = 8;
  stepOptions.solve.descent.regularization = 1e-12;
  stepOptions.solve.row.beta = 1000.0;
  stepOptions.solve.row.maxStiffness = 1000.0;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  vbd::AvbdScalarRowInventory jointLinearInventory;
  vbd::AvbdScalarRowInventory jointAngularInventory;

  const vbd::AvbdRigidWorldContactStepResult result
      = vbd::runAvbdRigidWorldContactStep(
          dart::simulation::detail::registryOf(world),
          std::span<const sx::Contact>(),
          joints,
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          /*timeStep=*/1.0,
          stepOptions);

  EXPECT_EQ(result.contacts, 0u);
  EXPECT_EQ(result.joints, 1u);
  EXPECT_EQ(result.solve.jointLinearRows, 3u);
  EXPECT_EQ(result.solve.jointAngularRows, 3u);
  EXPECT_EQ(result.apply.bodies, 1u);
  EXPECT_NEAR(link.getTransform().translation().x(), 0.0, 0.25);
  const Vec3 error = vbd::avbdRigidBodyOrientationError(
      Eigen::Quaterniond(link.getTransform().linear()),
      Eigen::Quaterniond::Identity());
  EXPECT_LT(std::abs(error.z()), 0.05);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldSolveClearsAbsentRowFamilyInventories)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity entityA
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  const entt::entity entityB
      = dart::simulation::detail::toRegistryEntity(link.getEntity());

  vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          registry, std::span<const sx::Contact>());
  std::vector<vbd::AvbdRigidWorldPointJointInput> joints(1);
  joints[0].bodyA = entityA;
  joints[0].bodyB = entityB;
  joints[0].anchorA = Vec3::Zero();
  joints[0].anchorB = Vec3::UnitX();
  joints[0].targetRelativeOrientation = Eigen::Quaterniond::Identity();
  joints[0].useAngularMotor = true;
  joints[0].motorTargetSpeed = 0.5;
  joints[0].motorMaxTorque = 5.0;
  joints[0].startStiffness = 10.0;
  joints[0].maxStiffness = 100.0;
  EXPECT_EQ(
      vbd::appendAvbdRigidWorldPointJoints(registry, joints, snapshot), 1u);

  std::vector<vbd::AvbdRigidWorldDistanceSpringInput> springs(1);
  springs[0].bodyA = entityA;
  springs[0].bodyB = entityB;
  springs[0].anchorA = Vec3::Zero();
  springs[0].anchorB = Vec3::UnitX();
  springs[0].restLength = 0.5;
  springs[0].startStiffness = 10.0;
  springs[0].materialStiffness = 50.0;
  springs[0].maxStiffness = 100.0;
  EXPECT_EQ(
      vbd::appendAvbdRigidWorldDistanceSprings(registry, springs, snapshot),
      1u);

  const std::size_t bodyA = findEntityIndex(snapshot.entities, entityA);
  const std::size_t bodyB = findEntityIndex(snapshot.entities, entityB);
  ASSERT_LT(bodyA, snapshot.entities.size());
  ASSERT_LT(bodyB, snapshot.entities.size());
  vbd::AvbdRigidContactManifoldPoint contact;
  contact.bodyA = static_cast<std::uint32_t>(bodyA);
  contact.bodyB = static_cast<std::uint32_t>(bodyB);
  contact.endpointA = vbd::avbdRigidWorldBodyEndpointId(entityA);
  contact.endpointB = vbd::avbdRigidWorldBodyEndpointId(entityB);
  contact.point = 0.5 * Vec3::UnitX();
  contact.normalFromAtoB = Vec3::UnitX();
  contact.depth = 0.1;
  contact.frictionCoefficient = 0.5;
  contact.startStiffness = 10.0;
  contact.maxStiffness = 100.0;
  snapshot.contacts.push_back(contact);

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  vbd::AvbdScalarRowInventory jointLinearInventory;
  vbd::AvbdScalarRowInventory jointAngularInventory;
  vbd::AvbdScalarRowInventory motorInventory;
  vbd::AvbdScalarRowInventory distanceSpringInventory;
  vbd::AvbdRigidWorldContactSolveScratch scratch;
  vbd::AvbdRigidWorldContactSolveOptions solveOptions;
  solveOptions.descent.iterations = 1;
  solveOptions.descent.regularization = 1e-12;

  const vbd::AvbdRigidWorldContactSolveResult populated
      = vbd::solveAvbdRigidWorldContactSnapshot(
          snapshot,
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          motorInventory,
          distanceSpringInventory,
          scratch,
          /*timeStep=*/1.0,
          solveOptions);
  EXPECT_GT(populated.normalRows, 0u);
  EXPECT_GT(populated.frictionRows, 0u);
  EXPECT_GT(populated.jointLinearRows, 0u);
  EXPECT_GT(populated.jointAngularRows, 0u);
  EXPECT_GT(populated.motorRows, 0u);
  EXPECT_GT(populated.distanceSpringRows, 0u);
  EXPECT_FALSE(normalInventory.empty());
  EXPECT_FALSE(frictionInventory.empty());
  EXPECT_FALSE(jointLinearInventory.empty());
  EXPECT_FALSE(jointAngularInventory.empty());
  EXPECT_FALSE(motorInventory.empty());
  EXPECT_FALSE(distanceSpringInventory.empty());

  snapshot.contacts.clear();
  snapshot.joints.clear();
  snapshot.jointEntities.clear();
  snapshot.linearMotors.clear();
  snapshot.motors.clear();
  snapshot.distanceSprings.clear();
  snapshot.distanceSpringEntities.clear();

  const vbd::AvbdRigidWorldContactSolveResult cleared
      = vbd::solveAvbdRigidWorldContactSnapshot(
          snapshot,
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          motorInventory,
          distanceSpringInventory,
          scratch,
          /*timeStep=*/1.0,
          solveOptions);
  EXPECT_EQ(cleared.normalRows, 0u);
  EXPECT_EQ(cleared.frictionRows, 0u);
  EXPECT_EQ(cleared.jointLinearRows, 0u);
  EXPECT_EQ(cleared.jointAngularRows, 0u);
  EXPECT_EQ(cleared.motorRows, 0u);
  EXPECT_EQ(cleared.distanceSpringRows, 0u);
  EXPECT_TRUE(normalInventory.empty());
  EXPECT_TRUE(frictionInventory.empty());
  EXPECT_TRUE(jointLinearInventory.empty());
  EXPECT_TRUE(jointAngularInventory.empty());
  EXPECT_TRUE(motorInventory.empty());
  EXPECT_TRUE(distanceSpringInventory.empty());
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldAppendRowsKeepEndpointPairOrdinals)
{
  sx::World world;

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.mass = 1.0;
  auto bodyA = world.addRigidBody("body_a", bodyOptions);
  bodyOptions.position = Vec3::UnitX();
  auto bodyB = world.addRigidBody("body_b", bodyOptions);

  const entt::entity entityA
      = dart::simulation::detail::toRegistryEntity(bodyA.getEntity());
  const entt::entity entityB
      = dart::simulation::detail::toRegistryEntity(bodyB.getEntity());
  auto& registry = dart::simulation::detail::registryOf(world);

  vbd::AvbdRigidWorldContactSnapshot jointSnapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          registry, std::span<const sx::Contact>());
  std::vector<vbd::AvbdRigidWorldPointJointInput> joints(2);
  joints[0].bodyA = entityA;
  joints[0].bodyB = entityB;
  joints[0].anchorA = Vec3::Zero();
  joints[0].anchorB = Vec3::UnitX();
  joints[1] = joints[0];
  joints[1].anchorA = Vec3::UnitY();
  joints[1].anchorB = Vec3::UnitX() + Vec3::UnitY();

  EXPECT_EQ(
      vbd::appendAvbdRigidWorldPointJoints(registry, joints, jointSnapshot),
      2u);
  ASSERT_EQ(jointSnapshot.joints.size(), 2u);
  EXPECT_EQ(jointSnapshot.joints[0].row, 0u);
  EXPECT_EQ(jointSnapshot.joints[1].row, 1u);

  std::vector<vbd::AvbdRigidWorldPointJointInput> extraJoint = {joints[0]};
  extraJoint[0].anchorA = Vec3::UnitZ();
  extraJoint[0].anchorB = Vec3::UnitX() + Vec3::UnitZ();
  EXPECT_EQ(
      vbd::appendAvbdRigidWorldPointJoints(registry, extraJoint, jointSnapshot),
      1u);
  ASSERT_EQ(jointSnapshot.joints.size(), 3u);
  EXPECT_EQ(jointSnapshot.joints[2].row, 2u);

  vbd::AvbdRigidWorldContactSnapshot springSnapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          registry, std::span<const sx::Contact>());
  std::vector<vbd::AvbdRigidWorldDistanceSpringInput> springs(2);
  springs[0].bodyA = entityA;
  springs[0].bodyB = entityB;
  springs[0].anchorA = Vec3::Zero();
  springs[0].anchorB = Vec3::UnitX();
  springs[0].restLength = 1.0;
  springs[1] = springs[0];
  springs[1].anchorA = Vec3::UnitY();
  springs[1].anchorB = Vec3::UnitX() + Vec3::UnitY();

  EXPECT_EQ(
      vbd::appendAvbdRigidWorldDistanceSprings(
          registry, springs, springSnapshot),
      2u);
  ASSERT_EQ(springSnapshot.distanceSprings.size(), 2u);
  EXPECT_EQ(springSnapshot.distanceSprings[0].rowIndex, 0u);
  EXPECT_EQ(springSnapshot.distanceSprings[1].rowIndex, 1u);

  std::vector<vbd::AvbdRigidWorldDistanceSpringInput> extraSpring
      = {springs[0]};
  extraSpring[0].anchorA = Vec3::UnitZ();
  extraSpring[0].anchorB = Vec3::UnitX() + Vec3::UnitZ();
  EXPECT_EQ(
      vbd::appendAvbdRigidWorldDistanceSprings(
          registry, extraSpring, springSnapshot),
      1u);
  ASSERT_EQ(springSnapshot.distanceSprings.size(), 3u);
  EXPECT_EQ(springSnapshot.distanceSprings[2].rowIndex, 2u);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldPairConstraintRowsSkipStaticStaticPairs)
{
  sx::World world;

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.isStatic = true;
  auto bodyA = world.addRigidBody("static_a", bodyOptions);
  bodyOptions.position = Vec3::UnitX();
  auto bodyB = world.addRigidBody("static_b", bodyOptions);

  const entt::entity entityA
      = dart::simulation::detail::toRegistryEntity(bodyA.getEntity());
  const entt::entity entityB
      = dart::simulation::detail::toRegistryEntity(bodyB.getEntity());
  auto& registry = dart::simulation::detail::registryOf(world);

  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = entityA;
  joint.childLink = entityB;
  registry.emplace<vbd::AvbdRigidWorldPointJointConfig>(jointEntity);

  const entt::entity springEntity = registry.create();
  auto& springConfig
      = registry.emplace<vbd::AvbdRigidWorldDistanceSpringConfig>(springEntity);
  springConfig.bodyA = entityA;
  springConfig.bodyB = entityB;
  springConfig.restLength = 1.0;

  EXPECT_TRUE(vbd::extractAvbdRigidWorldPointJointInputs(registry).empty());
  EXPECT_TRUE(vbd::extractAvbdRigidWorldDistanceSpringInputs(registry).empty());

  vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          registry, std::span<const sx::Contact>());

  std::vector<vbd::AvbdRigidWorldPointJointInput> joints(1);
  joints[0].bodyA = entityA;
  joints[0].bodyB = entityB;
  joints[0].anchorA = Vec3::Zero();
  joints[0].anchorB = Vec3::UnitX();
  EXPECT_EQ(
      vbd::appendAvbdRigidWorldPointJoints(registry, joints, snapshot), 0u);
  EXPECT_TRUE(snapshot.entities.empty());
  EXPECT_TRUE(snapshot.joints.empty());
  EXPECT_TRUE(snapshot.jointEntities.empty());

  std::vector<vbd::AvbdRigidWorldDistanceSpringInput> springs(1);
  springs[0].bodyA = entityA;
  springs[0].bodyB = entityB;
  springs[0].anchorA = Vec3::Zero();
  springs[0].anchorB = Vec3::UnitX();
  springs[0].restLength = 1.0;
  EXPECT_EQ(
      vbd::appendAvbdRigidWorldDistanceSprings(registry, springs, snapshot),
      0u);
  EXPECT_TRUE(snapshot.entities.empty());
  EXPECT_TRUE(snapshot.distanceSprings.empty());
  EXPECT_TRUE(snapshot.distanceSpringEntities.empty());
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldAppendRowsUseLinearBodyIndexForSmallSnapshots)
{
  sx::World world;

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.mass = 1.0;
  auto bodyA = world.addRigidBody("body_a", bodyOptions);
  bodyOptions.position = Vec3::UnitX();
  auto bodyB = world.addRigidBody("body_b", bodyOptions);

  const entt::entity entityA
      = dart::simulation::detail::toRegistryEntity(bodyA.getEntity());
  const entt::entity entityB
      = dart::simulation::detail::toRegistryEntity(bodyB.getEntity());
  auto& registry = dart::simulation::detail::registryOf(world);

  vbd::AvbdRigidWorldContactSnapshot snapshot;
  const Eigen::Isometry3d bodyATransform = bodyA.getTransform();
  snapshot.entities.push_back(entityA);
  snapshot.states.push_back(
      vbd::AvbdRigidBodyState{
          bodyATransform.translation(),
          Eigen::Quaterniond(bodyATransform.linear())});
  snapshot.masses.push_back(bodyA.getMass());
  snapshot.bodyInertias.push_back(bodyA.getInertia());
  snapshot.fixed.push_back(0u);

  std::vector<vbd::AvbdRigidWorldPointJointInput> joints(1);
  joints[0].bodyA = entityA;
  joints[0].bodyB = entityB;
  joints[0].anchorA = Vec3::Zero();
  joints[0].anchorB = Vec3::UnitX();

  EXPECT_TRUE(snapshot.entityBodyIndices.empty());
  EXPECT_EQ(
      vbd::appendAvbdRigidWorldPointJoints(registry, joints, snapshot), 1u);

  ASSERT_EQ(snapshot.entities.size(), 2u);
  EXPECT_EQ(snapshot.entities[0], entityA);
  EXPECT_EQ(snapshot.entities[1], entityB);
  EXPECT_TRUE(snapshot.entityBodyIndices.empty());
  ASSERT_EQ(snapshot.joints.size(), 1u);
  EXPECT_EQ(snapshot.joints[0].bodyA, 0u);
  EXPECT_EQ(snapshot.joints[0].bodyB, 1u);

  EXPECT_EQ(
      vbd::appendAvbdRigidWorldPointJoints(registry, joints, snapshot), 1u);
  ASSERT_EQ(snapshot.entities.size(), 2u);
  EXPECT_TRUE(snapshot.entityBodyIndices.empty());
  ASSERT_EQ(snapshot.joints.size(), 2u);
  EXPECT_EQ(snapshot.joints[1].bodyA, 0u);
  EXPECT_EQ(snapshot.joints[1].bodyB, 1u);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotReserveKeepsSmallBodyIndexLinear)
{
  vbd::AvbdRigidWorldContactSnapshot snapshot;
  const std::size_t initialBucketCount
      = snapshot.entityBodyIndices.bucket_count();

  vbd::reserveAvbdRigidWorldContactSnapshot(
      snapshot,
      vbd::detail::kAvbdRigidSmallRowStackCapacity,
      /*contactCapacity=*/0,
      /*jointCapacity=*/1,
      /*motorCapacity=*/1);

  EXPECT_EQ(snapshot.entityBodyIndices.bucket_count(), initialBucketCount);
  EXPECT_TRUE(snapshot.entityBodyIndices.empty());

  vbd::reserveAvbdRigidWorldContactSnapshot(
      snapshot,
      vbd::detail::kAvbdRigidSmallRowStackCapacity + 1u,
      /*contactCapacity=*/0,
      /*jointCapacity=*/1,
      /*motorCapacity=*/1);

  EXPECT_GT(snapshot.entityBodyIndices.bucket_count(), initialBucketCount);
  EXPECT_TRUE(snapshot.entityBodyIndices.empty());
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldAppendRowsUseReservedBodyIndexMapImmediately)
{
  sx::World world;

  sx::RigidBodyOptions bodyOptions;
  bodyOptions.mass = 1.0;
  auto bodyA = world.addRigidBody("body_a", bodyOptions);
  bodyOptions.position = Vec3::UnitX();
  auto bodyB = world.addRigidBody("body_b", bodyOptions);

  const entt::entity entityA
      = dart::simulation::detail::toRegistryEntity(bodyA.getEntity());
  const entt::entity entityB
      = dart::simulation::detail::toRegistryEntity(bodyB.getEntity());
  auto& registry = dart::simulation::detail::registryOf(world);

  vbd::AvbdRigidWorldContactSnapshot snapshot;
  vbd::detail::reserveAvbdRigidWorldBodyIndexMap(
      snapshot, vbd::detail::kAvbdRigidSmallRowStackCapacity + 1u);

  std::vector<vbd::AvbdRigidWorldPointJointInput> joints(1);
  joints[0].bodyA = entityA;
  joints[0].bodyB = entityB;
  joints[0].anchorA = Vec3::Zero();
  joints[0].anchorB = Vec3::UnitX();

  EXPECT_TRUE(snapshot.entityBodyIndices.empty());
  EXPECT_EQ(
      vbd::appendAvbdRigidWorldPointJoints(registry, joints, snapshot), 1u);

  ASSERT_EQ(snapshot.entities.size(), 2u);
  ASSERT_EQ(snapshot.entityBodyIndices.size(), 2u);
  EXPECT_EQ(snapshot.entityBodyIndices.at(entityA), 0u);
  EXPECT_EQ(snapshot.entityBodyIndices.at(entityB), 1u);
  ASSERT_EQ(snapshot.joints.size(), 1u);
  EXPECT_EQ(snapshot.joints[0].bodyA, 0u);
  EXPECT_EQ(snapshot.joints[0].bodyB, 1u);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldDistanceSpringApiFeedsRadialRows)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = 2.0 * Vec3::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  world.addRigidBodyDistanceSpring(
      "radial_spring",
      base,
      link,
      /*restLength=*/1.0,
      /*stiffness=*/200.0);

  std::vector<vbd::AvbdRigidWorldDistanceSpringInput> springs
      = vbd::extractAvbdRigidWorldDistanceSpringInputs(
          dart::simulation::detail::registryOf(world));
  ASSERT_EQ(springs.size(), 1u);
  EXPECT_FALSE(springs[0].anchorsAreLocal);
  EXPECT_TRUE(static_cast<bool>(springs[0].bodyAView));
  EXPECT_TRUE(static_cast<bool>(springs[0].bodyBView));
  EXPECT_TRUE(springs[0].bodyAView.isStatic);
  EXPECT_FALSE(springs[0].bodyBView.isStatic);

  vbd::AvbdRigidWorldContactStepOptions stepOptions;
  stepOptions.solve.descent.iterations = 8;
  stepOptions.solve.descent.regularization = 1e-12;
  stepOptions.solve.distanceSpring.beta = 1.0;
  stepOptions.solve.distanceSpring.maxStiffness = 200.0;

  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  vbd::AvbdScalarRowInventory jointLinearInventory;
  vbd::AvbdScalarRowInventory jointAngularInventory;
  vbd::AvbdScalarRowInventory motorInventory;
  vbd::AvbdScalarRowInventory distanceSpringInventory;

  const double initialDistance
      = (link.getTranslation() - base.getTranslation()).norm();
  const vbd::AvbdRigidWorldContactStepResult result
      = vbd::runAvbdRigidWorldContactStep(
          dart::simulation::detail::registryOf(world),
          std::span<const sx::Contact>(),
          std::span<const vbd::AvbdRigidWorldPointJointInput>(),
          springs,
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          motorInventory,
          distanceSpringInventory,
          /*timeStep=*/1.0,
          stepOptions);

  EXPECT_EQ(result.contacts, 0u);
  EXPECT_EQ(result.joints, 0u);
  EXPECT_EQ(result.distanceSprings, 1u);
  EXPECT_EQ(result.solve.distanceSpringRows, 1u);
  EXPECT_EQ(distanceSpringInventory.size(), 1u);
  EXPECT_EQ(result.apply.bodies, 1u);
  EXPECT_LT(
      (link.getTranslation() - base.getTranslation()).norm(), initialDistance);
  EXPECT_LT(link.getLinearVelocity().x(), 0.0);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldPointJointFractureMarksJointBroken)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  linkOptions.orientation = rotationZ(0.6);
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;

  std::vector<vbd::AvbdRigidWorldPointJointInput> joints(1);
  joints[0].joint = jointEntity;
  joints[0].bodyA = sx::detail::toRegistryEntity(base.getEntity());
  joints[0].bodyB = sx::detail::toRegistryEntity(link.getEntity());
  joints[0].anchorA = Vec3::Zero();
  joints[0].anchorB = Vec3::UnitX();
  joints[0].targetRelativeOrientation = Eigen::Quaterniond::Identity();
  joints[0].startStiffness = 100.0;
  joints[0].maxStiffness = 1000.0;
  joints[0].fractureThreshold = 1e-12;

  vbd::AvbdRigidWorldContactStepOptions stepOptions;
  stepOptions.solve.descent.iterations = 8;
  stepOptions.solve.descent.regularization = 1e-12;
  stepOptions.solve.row.beta = 1000.0;
  stepOptions.solve.row.maxStiffness = 1000.0;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  vbd::AvbdScalarRowInventory jointLinearInventory;
  vbd::AvbdScalarRowInventory jointAngularInventory;

  const vbd::AvbdRigidWorldContactStepResult result
      = vbd::runAvbdRigidWorldContactStep(
          registry,
          std::span<const sx::Contact>(),
          joints,
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          /*timeStep=*/1.0,
          stepOptions);

  EXPECT_EQ(result.joints, 1u);
  EXPECT_EQ(result.solve.jointLinearRows, 3u);
  EXPECT_EQ(result.solve.jointAngularRows, 3u);
  EXPECT_EQ(result.solve.fracturedJoints, 1u);
  ASSERT_EQ(result.solve.fracturedJointIndices.size(), 1u);
  EXPECT_EQ(result.solve.fracturedJointIndices[0], 0u);
  EXPECT_EQ(result.fracturedJoints, 1u);
  EXPECT_TRUE(registry.get<sx::comps::Joint>(jointEntity).broken);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldPointJointFractureUsesSolveScratchAllocator)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  linkOptions.orientation = rotationZ(0.6);
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;

  vbd::AvbdRigidWorldPointJointInput input;
  input.joint = jointEntity;
  input.bodyA = sx::detail::toRegistryEntity(base.getEntity());
  input.bodyB = sx::detail::toRegistryEntity(link.getEntity());
  input.anchorA = Vec3::Zero();
  input.anchorB = Vec3::UnitX();
  input.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  input.startStiffness = 100.0;
  input.maxStiffness = 1000.0;
  input.fractureThreshold = 1e-12;

  common::MemoryManager memoryManager;
  auto& allocator = memoryManager.getFreeAllocator();
  auto& freeList = memoryManager.getFreeListAllocator();

  vbd::AvbdRigidWorldContactSnapshot snapshot(allocator);
  vbd::AvbdRigidWorldContactBuildScratch buildScratch(allocator);
  vbd::buildAvbdRigidWorldContactSnapshot(
      registry, std::span<const sx::Contact>(), snapshot, buildScratch);
  vbd::reserveAvbdRigidWorldContactSnapshot(
      snapshot,
      /*bodyCapacity=*/2u,
      /*contactCapacity=*/0u,
      /*jointCapacity=*/1u,
      /*motorCapacity=*/0u);
  ASSERT_EQ(
      vbd::appendAvbdRigidWorldPointJoints(
          registry,
          std::span<const vbd::AvbdRigidWorldPointJointInput>{&input, 1u},
          snapshot,
          buildScratch),
      1u);
  vbd::predictAvbdRigidWorldContactInertialTargets(
      registry, snapshot, /*timeStep=*/1.0);

  vbd::AvbdScalarRowInventory normalInventory(allocator);
  vbd::AvbdScalarRowInventory frictionInventory(allocator);
  vbd::AvbdScalarRowInventory jointLinearInventory(allocator);
  vbd::AvbdScalarRowInventory jointAngularInventory(allocator);
  vbd::AvbdScalarRowInventory motorInventory(allocator);
  vbd::AvbdScalarRowInventory distanceSpringInventory(allocator);
  jointLinearInventory.reserve(3u);
  jointAngularInventory.reserve(3u);

  vbd::AvbdRigidWorldContactSolveScratch solveScratch(allocator);
  vbd::reserveAvbdRigidWorldContactSolveScratch(
      solveScratch,
      /*contactCapacity=*/0u,
      /*jointCapacity=*/1u,
      /*motorCapacity=*/0u,
      /*bodyCapacity=*/2u);

  vbd::AvbdRigidWorldContactSolveOptions solveOptions;
  solveOptions.descent.iterations = 8;
  solveOptions.descent.regularization = 1e-12;
  solveOptions.row.beta = 1000.0;
  solveOptions.row.maxStiffness = 1000.0;

  const auto allocationsBeforeSolve = freeList.getAllocationCount();
  const vbd::AvbdRigidWorldContactSolveResult result
      = vbd::solveAvbdRigidWorldContactSnapshot(
          snapshot,
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          motorInventory,
          distanceSpringInventory,
          /*timeStep=*/1.0,
          solveScratch,
          solveOptions);

  EXPECT_GT(freeList.getAllocationCount(), allocationsBeforeSolve)
      << "fracture-index result storage should borrow the provided solve "
         "scratch allocator";
  EXPECT_EQ(result.fracturedJoints, 1u);
  ASSERT_EQ(result.fracturedJointIndices.size(), 1u);
  EXPECT_EQ(result.fracturedJointIndices[0], 0u);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldPointJointInputPreservesAxisConfig)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          std::span<const sx::Contact>());

  const Vec3 freeAxis = Vec3(0.25, 0.5, 1.0).normalized();
  const Eigen::Matrix3d jointAxes
      = vbd::avbdRigidJointAxesFromFreeAxis(freeAxis);
  std::vector<vbd::AvbdRigidWorldPointJointInput> joints(1);
  joints[0].bodyA
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  joints[0].bodyB
      = dart::simulation::detail::toRegistryEntity(link.getEntity());
  joints[0].anchorA = Vec3::Zero();
  joints[0].anchorB = Vec3::UnitX();
  joints[0].linearAxes = jointAxes;
  joints[0].angularAxes = jointAxes;
  joints[0].linearAxisMask = vbd::avbdRigidJointAllButAxisMask(2u);
  joints[0].angularAxisMask = vbd::avbdRigidJointAllButAxisMask(2u);
  joints[0].startStiffness = 100.0;
  joints[0].maxStiffness = 1000.0;

  EXPECT_EQ(
      vbd::appendAvbdRigidWorldPointJoints(
          dart::simulation::detail::registryOf(world), joints, snapshot),
      1u);

  ASSERT_EQ(snapshot.joints.size(), 1u);
  EXPECT_EQ(snapshot.joints[0].linearAxisMask, joints[0].linearAxisMask);
  EXPECT_EQ(snapshot.joints[0].angularAxisMask, joints[0].angularAxisMask);
  EXPECT_NEAR((snapshot.joints[0].linearAxes - jointAxes).norm(), 0.0, 1e-12);
  EXPECT_NEAR((snapshot.joints[0].angularAxes - jointAxes).norm(), 0.0, 1e-12);

  vbd::AvbdRigidWorldContactSolveOptions solveOptions;
  solveOptions.descent.iterations = 1;
  solveOptions.descent.regularization = 1e-12;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  vbd::AvbdScalarRowInventory jointLinearInventory;
  vbd::AvbdScalarRowInventory jointAngularInventory;
  const vbd::AvbdRigidWorldContactSolveResult solveResult
      = vbd::solveAvbdRigidWorldContactSnapshot(
          snapshot,
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          /*timeStep=*/1.0,
          solveOptions);

  EXPECT_EQ(solveResult.jointLinearRows, 2u);
  EXPECT_EQ(solveResult.jointAngularRows, 2u);
  ASSERT_EQ(jointLinearInventory.size(), 2u);
  ASSERT_EQ(jointAngularInventory.size(), 2u);
  EXPECT_EQ(jointLinearInventory[0].descriptor.key.axis, 0u);
  EXPECT_EQ(jointLinearInventory[1].descriptor.key.axis, 1u);
  EXPECT_EQ(jointAngularInventory[0].descriptor.key.axis, 0u);
  EXPECT_EQ(jointAngularInventory[1].descriptor.key.axis, 1u);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldRevoluteVelocityActuatorBuildsMotorRows)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyRevoluteJoint(
      "motorized_hinge", base, link, Vec3::UnitZ());
  joint.setActuatorType(sx::ActuatorType::Velocity);
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, 0.75));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -500.0),
      Eigen::VectorXd::Constant(1, 500.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const std::vector<vbd::AvbdRigidWorldPointJointInput> joints
      = vbd::extractAvbdRigidWorldPointJointInputs(registry);
  ASSERT_EQ(joints.size(), 1u);
  EXPECT_TRUE(joints[0].useAngularMotor);
  EXPECT_DOUBLE_EQ(joints[0].motorTargetSpeed, 0.75);
  EXPECT_DOUBLE_EQ(joints[0].motorMaxTorque, 500.0);

  vbd::AvbdRigidWorldContactStepOptions stepOptions;
  stepOptions.solve.descent.iterations = 8;
  stepOptions.solve.descent.regularization = 1e-12;
  stepOptions.solve.row.beta = 1000.0;
  stepOptions.solve.row.maxStiffness = 1000.0;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  vbd::AvbdScalarRowInventory jointLinearInventory;
  vbd::AvbdScalarRowInventory jointAngularInventory;
  vbd::AvbdScalarRowInventory motorInventory;

  const vbd::AvbdRigidWorldContactStepResult result
      = vbd::runAvbdRigidWorldContactStep(
          registry,
          std::span<const sx::Contact>(),
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          motorInventory,
          /*timeStep=*/0.25,
          stepOptions);

  EXPECT_EQ(result.contacts, 0u);
  EXPECT_EQ(result.joints, 1u);
  EXPECT_EQ(result.motors, 1u);
  EXPECT_EQ(result.solve.jointLinearRows, 3u);
  EXPECT_EQ(result.solve.jointAngularRows, 2u);
  EXPECT_EQ(result.solve.motorRows, 1u);
  EXPECT_EQ(result.apply.bodies, 1u);
  ASSERT_EQ(motorInventory.size(), 1u);
  EXPECT_EQ(
      motorInventory[0].descriptor.key.role, vbd::AvbdScalarRowRole::Motor);
  EXPECT_DOUBLE_EQ(motorInventory[0].descriptor.bounds.lower, -500.0);
  EXPECT_DOUBLE_EQ(motorInventory[0].descriptor.bounds.upper, 500.0);

  EXPECT_GT(link.getAngularVelocity().z(), 0.05);
  EXPECT_LT(link.getAngularVelocity().z(), 1.25);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactStepFallbackUsesInventoryAllocator)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyRevoluteJoint(
      "motorized_hinge", base, link, Vec3::UnitZ());
  joint.setActuatorType(sx::ActuatorType::Velocity);
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, 0.75));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -500.0),
      Eigen::VectorXd::Constant(1, 500.0));

  vbd::AvbdRigidWorldContactStepOptions stepOptions;
  stepOptions.solve.descent.iterations = 8;
  stepOptions.solve.descent.regularization = 1e-12;
  stepOptions.solve.row.beta = 1000.0;
  stepOptions.solve.row.maxStiffness = 1000.0;

  CountingMemoryAllocator allocator;
  vbd::AvbdScalarRowInventory normalInventory(allocator);
  vbd::AvbdScalarRowInventory frictionInventory(allocator);
  vbd::AvbdScalarRowInventory jointLinearInventory(allocator);
  vbd::AvbdScalarRowInventory jointAngularInventory(allocator);
  vbd::AvbdScalarRowInventory motorInventory(allocator);
  normalInventory.reserve(0u);
  frictionInventory.reserve(0u);
  jointLinearInventory.reserve(3u);
  jointAngularInventory.reserve(2u);
  motorInventory.reserve(1u);

  const std::size_t allocationsBeforeStep = allocator.allocations;
  const vbd::AvbdRigidWorldContactStepResult result
      = vbd::runAvbdRigidWorldContactStep(
          dart::simulation::detail::registryOf(world),
          std::span<const sx::Contact>(),
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          motorInventory,
          /*timeStep=*/0.25,
          stepOptions);

  EXPECT_GT(allocator.allocations, allocationsBeforeStep)
      << "registry no-scratch AVBD step fallback should borrow the row "
         "inventory allocator for local joint inputs, snapshot, and solve "
         "scratch";
  EXPECT_EQ(result.contacts, 0u);
  EXPECT_EQ(result.joints, 1u);
  EXPECT_EQ(result.motors, 1u);
  EXPECT_EQ(result.solve.jointLinearRows, 3u);
  EXPECT_EQ(result.solve.jointAngularRows, 2u);
  EXPECT_EQ(result.solve.motorRows, 1u);
  EXPECT_EQ(result.apply.bodies, 1u);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldPrismaticVelocityActuatorBuildsLinearMotorRows)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitZ();
  auto link = world.addRigidBody("slider", linkOptions);

  auto joint = world.addRigidBodyPrismaticJoint(
      "motorized_slider", base, link, Vec3::UnitZ());
  joint.setActuatorType(sx::ActuatorType::Velocity);
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, 0.6));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -400.0),
      Eigen::VectorXd::Constant(1, 400.0));

  auto& registry = dart::simulation::detail::registryOf(world);
  const std::vector<vbd::AvbdRigidWorldPointJointInput> joints
      = vbd::extractAvbdRigidWorldPointJointInputs(registry);
  ASSERT_EQ(joints.size(), 1u);
  EXPECT_TRUE(joints[0].useLinearMotor);
  EXPECT_FALSE(joints[0].useAngularMotor);
  EXPECT_DOUBLE_EQ(joints[0].motorTargetSpeed, 0.6);
  EXPECT_DOUBLE_EQ(joints[0].motorMaxForce, 400.0);

  vbd::AvbdRigidWorldContactStepOptions stepOptions;
  stepOptions.solve.descent.iterations = 8;
  stepOptions.solve.descent.regularization = 1e-12;
  stepOptions.solve.row.beta = 1000.0;
  stepOptions.solve.row.maxStiffness = 1000.0;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  vbd::AvbdScalarRowInventory jointLinearInventory;
  vbd::AvbdScalarRowInventory jointAngularInventory;
  vbd::AvbdScalarRowInventory motorInventory;

  const vbd::AvbdRigidWorldContactStepResult result
      = vbd::runAvbdRigidWorldContactStep(
          registry,
          std::span<const sx::Contact>(),
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          motorInventory,
          /*timeStep=*/0.25,
          stepOptions);

  EXPECT_EQ(result.contacts, 0u);
  EXPECT_EQ(result.joints, 1u);
  EXPECT_EQ(result.motors, 1u);
  EXPECT_EQ(result.solve.jointLinearRows, 2u);
  EXPECT_EQ(result.solve.jointAngularRows, 3u);
  EXPECT_EQ(result.solve.motorRows, 1u);
  EXPECT_EQ(result.apply.bodies, 1u);
  ASSERT_EQ(motorInventory.size(), 1u);
  EXPECT_EQ(
      motorInventory[0].descriptor.key.role, vbd::AvbdScalarRowRole::Motor);
  EXPECT_DOUBLE_EQ(motorInventory[0].descriptor.bounds.lower, -400.0);
  EXPECT_DOUBLE_EQ(motorInventory[0].descriptor.bounds.upper, 400.0);

  EXPECT_GT(link.getLinearVelocity().z(), 0.05);
  EXPECT_LT(link.getLinearVelocity().z(), 1.0);
}

//==============================================================================
TEST(
    AvbdRigidBlock,
    RigidWorldRevoluteVelocityActuatorStepsWithoutCollisionGeometry)
{
  sx::WorldOptions options;
  options.gravity = Vec3::Zero();
  options.timeStep = 0.005;
  sx::World world(options);

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto joint = world.addRigidBodyRevoluteJoint(
      "motorized_hinge", base, link, Vec3::UnitZ());
  joint.setActuatorType(sx::ActuatorType::Velocity);
  joint.setCommandVelocity(Eigen::VectorXd::Constant(1, 0.75));
  joint.setEffortLimits(
      Eigen::VectorXd::Constant(1, -500.0),
      Eigen::VectorXd::Constant(1, 500.0));

  ASSERT_FALSE(base.hasCollisionShape());
  ASSERT_FALSE(link.hasCollisionShape());

  world.enterSimulationMode();
  world.step();

  EXPECT_GT(link.getAngularVelocity().z(), 0.05);
  EXPECT_LT(link.getAngularVelocity().z(), 1.25);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldExtractsFixedJointInputs)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  linkOptions.orientation = rotationZ(0.6);
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const vbd::AvbdRigidWorldEndpoint baseEndpoint
      = vbd::classifyAvbdRigidWorldEndpoint(
          registry, sx::detail::toRegistryEntity(base.getEntity()));
  EXPECT_EQ(baseEndpoint.kind, vbd::AvbdRigidWorldEndpointKind::FreeRigidBody);
  EXPECT_TRUE(baseEndpoint.canProjectAsRigidBody);

  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.breakForce = 1.0e12;
  joint.parentLink
      = dart::simulation::detail::toRegistryEntity(base.getEntity());
  joint.childLink
      = dart::simulation::detail::toRegistryEntity(link.getEntity());
  auto& config
      = registry.emplace<vbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Vec3::Zero();
  config.localAnchorB = Vec3::Zero();
  config.targetRelativeOrientation = Eigen::Quaterniond::Identity();
  config.startStiffness = 100.0;
  config.maxStiffness = 1000.0;

  const std::vector<vbd::AvbdRigidWorldPointJointInput> joints
      = vbd::extractAvbdRigidWorldPointJointInputs(registry);
  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(
      joints[0].bodyA,
      dart::simulation::detail::toRegistryEntity(base.getEntity()));
  EXPECT_EQ(
      joints[0].bodyB,
      dart::simulation::detail::toRegistryEntity(link.getEntity()));
  EXPECT_NEAR(joints[0].anchorA.x(), 0.0, 1e-12);
  EXPECT_NEAR(joints[0].anchorB.x(), 1.0, 1e-12);
  EXPECT_FALSE(joints[0].anchorsAreLocal);
  EXPECT_DOUBLE_EQ(joints[0].startStiffness, 100.0);
  EXPECT_DOUBLE_EQ(joints[0].maxStiffness, 1000.0);
  EXPECT_EQ(joints[0].joint, jointEntity);
  EXPECT_DOUBLE_EQ(joints[0].fractureThreshold, 1.0e12);
  EXPECT_TRUE(static_cast<bool>(joints[0].bodyAView));
  EXPECT_TRUE(static_cast<bool>(joints[0].bodyBView));
  EXPECT_TRUE(joints[0].bodyAView.isStatic);
  EXPECT_FALSE(joints[0].bodyBView.isStatic);

  std::vector<vbd::AvbdRigidWorldPointJointInput> localOnlyJoints;
  vbd::extractAvbdRigidWorldPointJointInputsInto(
      registry, localOnlyJoints, /*includeWorldAnchors=*/false);
  ASSERT_EQ(localOnlyJoints.size(), 1u);
  EXPECT_TRUE(localOnlyJoints[0].anchorsAreLocal);
  EXPECT_TRUE(static_cast<bool>(localOnlyJoints[0].bodyAView));
  EXPECT_TRUE(static_cast<bool>(localOnlyJoints[0].bodyBView));
  EXPECT_NEAR(localOnlyJoints[0].anchorA.norm(), 0.0, 1e-12);
  EXPECT_NEAR(localOnlyJoints[0].anchorB.norm(), 0.0, 1e-12);

  vbd::AvbdRigidWorldContactStepOptions stepOptions;
  stepOptions.solve.descent.iterations = 8;
  stepOptions.solve.descent.regularization = 1e-12;
  stepOptions.solve.row.beta = 1000.0;
  stepOptions.solve.row.maxStiffness = 1000.0;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  vbd::AvbdScalarRowInventory jointLinearInventory;
  vbd::AvbdScalarRowInventory jointAngularInventory;

  const vbd::AvbdRigidWorldContactStepResult result
      = vbd::runAvbdRigidWorldContactStep(
          registry,
          std::span<const sx::Contact>(),
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          /*timeStep=*/1.0,
          stepOptions);

  EXPECT_EQ(result.joints, 1u);
  EXPECT_EQ(result.solve.jointLinearRows, 3u);
  EXPECT_EQ(result.solve.jointAngularRows, 3u);
  EXPECT_EQ(result.apply.bodies, 1u);
  EXPECT_NEAR(link.getTransform().translation().x(), 0.0, 0.25);
  const Vec3 error = vbd::avbdRigidBodyOrientationError(
      Eigen::Quaterniond(link.getTransform().linear()),
      Eigen::Quaterniond::Identity());
  EXPECT_LT(std::abs(error.z()), 0.05);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldPairConstraintConfigPresenceGuardsExtraction)
{
  sx::World world;
  auto& registry = dart::simulation::detail::registryOf(world);

  EXPECT_FALSE(vbd::hasAvbdRigidWorldPointJointConfigs(registry));
  EXPECT_FALSE(vbd::hasAvbdRigidWorldDistanceSpringConfigs(registry));
  EXPECT_FALSE(vbd::hasAvbdRigidWorldPairConstraintConfigs(registry));

  const entt::entity jointEntity = registry.create();
  registry.emplace<sx::comps::Joint>(jointEntity);
  registry.emplace<vbd::AvbdRigidWorldPointJointConfig>(jointEntity);

  EXPECT_TRUE(vbd::hasAvbdRigidWorldPointJointConfigs(registry));
  EXPECT_FALSE(vbd::hasAvbdRigidWorldDistanceSpringConfigs(registry));
  EXPECT_TRUE(vbd::hasAvbdRigidWorldPairConstraintConfigs(registry));

  std::vector<vbd::AvbdRigidWorldPointJointInput> jointInputs(1);
  vbd::extractAvbdRigidWorldPointJointInputsInto(registry, jointInputs);
  EXPECT_TRUE(jointInputs.empty());

  registry.remove<vbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  const entt::entity springEntity = registry.create();
  registry.emplace<vbd::AvbdRigidWorldDistanceSpringConfig>(springEntity);

  EXPECT_FALSE(vbd::hasAvbdRigidWorldPointJointConfigs(registry));
  EXPECT_TRUE(vbd::hasAvbdRigidWorldDistanceSpringConfigs(registry));
  EXPECT_TRUE(vbd::hasAvbdRigidWorldPairConstraintConfigs(registry));

  std::vector<vbd::AvbdRigidWorldDistanceSpringInput> springInputs(1);
  vbd::extractAvbdRigidWorldDistanceSpringInputsInto(registry, springInputs);
  EXPECT_TRUE(springInputs.empty());

  registry.remove<vbd::AvbdRigidWorldDistanceSpringConfig>(springEntity);

  EXPECT_FALSE(vbd::hasAvbdRigidWorldPointJointConfigs(registry));
  EXPECT_FALSE(vbd::hasAvbdRigidWorldDistanceSpringConfigs(registry));
  EXPECT_FALSE(vbd::hasAvbdRigidWorldPairConstraintConfigs(registry));
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldSphericalJointExtractsLinearOnlyRows)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  sx::Joint joint = world.addRigidBodySphericalJoint("socket", base, link);
  EXPECT_EQ(joint.getType(), sx::JointType::Spherical);
  EXPECT_EQ(joint.getDOFCount(), 3u);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity
      = sx::detail::toRegistryEntity(joint.getEntity());
  ASSERT_TRUE(
      registry.all_of<vbd::AvbdRigidWorldPointJointConfig>(jointEntity));
  const auto& config
      = registry.get<vbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  EXPECT_EQ(config.linearAxisMask, vbd::kAvbdRigidJointAllAxesMask);
  EXPECT_EQ(config.angularAxisMask, 0u);

  Eigen::Isometry3d drifted = Eigen::Isometry3d::Identity();
  drifted.linear() = rotationZ(0.6).toRotationMatrix();
  drifted.translation() = Vec3(1.25, 0.25, 0.0);
  link.setTransform(drifted);

  vbd::AvbdRigidWorldContactStepOptions stepOptions;
  stepOptions.solve.descent.iterations = 8;
  stepOptions.solve.descent.regularization = 1e-12;
  stepOptions.solve.row.beta = 1000.0;
  stepOptions.solve.row.maxStiffness = 1000.0;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  vbd::AvbdScalarRowInventory jointLinearInventory;
  vbd::AvbdScalarRowInventory jointAngularInventory;

  const vbd::AvbdRigidWorldContactStepResult result
      = vbd::runAvbdRigidWorldContactStep(
          registry,
          std::span<const sx::Contact>(),
          normalInventory,
          frictionInventory,
          jointLinearInventory,
          jointAngularInventory,
          /*timeStep=*/1.0,
          stepOptions);

  EXPECT_EQ(result.joints, 1u);
  EXPECT_EQ(result.solve.jointLinearRows, 3u);
  EXPECT_EQ(result.solve.jointAngularRows, 0u);
  EXPECT_EQ(result.apply.bodies, 1u);
  EXPECT_NEAR(link.getTransform().translation().x(), 1.0, 0.25);
  EXPECT_NEAR(link.getTransform().translation().y(), 0.0, 0.25);
  const Vec3 orientationError = vbd::avbdRigidBodyOrientationError(
      Eigen::Quaterniond(link.getTransform().linear()), rotationZ(0.6));
  EXPECT_LT(orientationError.norm(), 0.02);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldSkipsBrokenPointJointInputs)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3::UnitX();
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.breakForce = 42.0;
  joint.parentLink = sx::detail::toRegistryEntity(base.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(link.getEntity());

  auto& config
      = registry.emplace<vbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  config.localAnchorA = Vec3::Zero();
  config.localAnchorB = Vec3::Zero();
  config.startStiffness = 100.0;
  config.maxStiffness = 1000.0;

  std::vector<vbd::AvbdRigidWorldPointJointInput> joints
      = vbd::extractAvbdRigidWorldPointJointInputs(registry);
  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0].joint, jointEntity);
  EXPECT_DOUBLE_EQ(joints[0].fractureThreshold, 42.0);

  joint.broken = true;
  joints = vbd::extractAvbdRigidWorldPointJointInputs(registry);
  EXPECT_TRUE(joints.empty());
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldPointJointInputsRotateAxesWithParentFrame)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions sliderOptions;
  sliderOptions.mass = 1.0;
  sliderOptions.position = Vec3::UnitZ();
  auto slider = world.addRigidBody("slider", sliderOptions);
  sx::Joint sliderJoint = world.addRigidBodyPrismaticJoint(
      "slider_joint", base, slider, Vec3::UnitZ());

  sx::RigidBodyOptions hingeOptions;
  hingeOptions.mass = 1.0;
  hingeOptions.position = 2.0 * Vec3::UnitZ();
  auto hinge = world.addRigidBody("hinge", hingeOptions);
  sx::Joint hingeJoint = world.addRigidBodyRevoluteJoint(
      "hinge_joint", base, hinge, Vec3::UnitZ());

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity sliderJointEntity
      = sx::detail::toRegistryEntity(sliderJoint.getEntity());
  const entt::entity hingeJointEntity
      = sx::detail::toRegistryEntity(hingeJoint.getEntity());
  const auto& sliderConfig
      = registry.get<vbd::AvbdRigidWorldPointJointConfig>(sliderJointEntity);
  const auto& hingeConfig
      = registry.get<vbd::AvbdRigidWorldPointJointConfig>(hingeJointEntity);

  Eigen::Isometry3d baseTransform = Eigen::Isometry3d::Identity();
  baseTransform.linear()
      = rotationY(0.5 * vbd::kAvbdRigidPi).toRotationMatrix();
  base.setTransform(baseTransform);

  const std::vector<vbd::AvbdRigidWorldPointJointInput> joints
      = vbd::extractAvbdRigidWorldPointJointInputs(registry);
  ASSERT_EQ(joints.size(), 2u);

  const auto findJointInput = [&](const sx::RigidBody& child) {
    const entt::entity childEntity
        = sx::detail::toRegistryEntity(child.getEntity());
    const auto it = std::find_if(
        joints.begin(),
        joints.end(),
        [&](const vbd::AvbdRigidWorldPointJointInput& input) {
          return input.bodyB == childEntity;
        });
    EXPECT_NE(it, joints.end());
    return it;
  };

  const Eigen::Matrix3d parentRotation = baseTransform.linear();
  const auto sliderInput = findJointInput(slider);
  ASSERT_NE(sliderInput, joints.end());
  EXPECT_NEAR(
      (sliderInput->linearAxes - parentRotation * sliderConfig.linearAxes)
          .norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(
      (sliderInput->angularAxes - parentRotation * sliderConfig.angularAxes)
          .norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(sliderInput->linearAxes.col(2).dot(Vec3::UnitX()), 1.0, 1e-12);

  const auto hingeInput = findJointInput(hinge);
  ASSERT_NE(hingeInput, joints.end());
  EXPECT_NEAR(
      (hingeInput->angularAxes - parentRotation * hingeConfig.angularAxes)
          .norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(hingeInput->angularAxes.col(2).dot(Vec3::UnitX()), 1.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldFixedJointHelperDerivesCurrentPoseConfig)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions baseOptions;
  baseOptions.isStatic = true;
  baseOptions.position = Vec3(0.25, -0.5, 0.125);
  baseOptions.orientation = rotationZ(-0.4);
  auto base = world.addRigidBody("base", baseOptions);

  sx::RigidBodyOptions linkOptions;
  linkOptions.mass = 1.0;
  linkOptions.position = Vec3(1.5, -0.25, 0.4);
  linkOptions.orientation = rotationZ(0.7);
  auto link = world.addRigidBody("link", linkOptions);

  auto& registry = dart::simulation::detail::registryOf(world);
  const entt::entity jointEntity = registry.create();
  auto& joint = registry.emplace<sx::comps::Joint>(jointEntity);
  joint.type = sx::comps::JointType::Fixed;
  joint.parentLink = sx::detail::toRegistryEntity(base.getEntity());
  joint.childLink = sx::detail::toRegistryEntity(link.getEntity());

  ASSERT_TRUE(
      vbd::configureAvbdRigidWorldFixedJointFromCurrentPose(
          registry,
          jointEntity,
          /*startStiffness=*/125.0,
          /*maxStiffness=*/1000.0));

  const auto& config
      = registry.get<vbd::AvbdRigidWorldPointJointConfig>(jointEntity);
  const Vec3 expectedWorldAnchor = linkOptions.position;
  EXPECT_TRUE(config.enabled);
  EXPECT_NEAR(
      (config.localAnchorA
       - base.getTransform().inverse() * expectedWorldAnchor)
          .norm(),
      0.0,
      1e-12);
  EXPECT_NEAR(config.localAnchorB.norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (config.targetRelativeOrientation.toRotationMatrix()
       - (baseOptions.orientation.conjugate() * linkOptions.orientation)
             .toRotationMatrix())
          .norm(),
      0.0,
      1e-12);
  EXPECT_DOUBLE_EQ(config.startStiffness, 125.0);
  EXPECT_DOUBLE_EQ(config.maxStiffness, 1000.0);

  const std::vector<vbd::AvbdRigidWorldPointJointInput> joints
      = vbd::extractAvbdRigidWorldPointJointInputs(registry);
  ASSERT_EQ(joints.size(), 1u);
  EXPECT_EQ(joints[0].bodyA, sx::detail::toRegistryEntity(base.getEntity()));
  EXPECT_EQ(joints[0].bodyB, sx::detail::toRegistryEntity(link.getEntity()));
  EXPECT_FALSE(joints[0].anchorsAreLocal);
  EXPECT_NEAR((joints[0].anchorA - expectedWorldAnchor).norm(), 0.0, 1e-12);
  EXPECT_NEAR((joints[0].anchorB - expectedWorldAnchor).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (joints[0].targetRelativeOrientation.toRotationMatrix()
       - config.targetRelativeOrientation.toRotationMatrix())
          .norm(),
      0.0,
      1e-12);

  std::vector<vbd::AvbdRigidWorldPointJointInput> localOnlyJoints;
  vbd::extractAvbdRigidWorldPointJointInputsInto(
      registry, localOnlyJoints, /*includeWorldAnchors=*/false);
  ASSERT_EQ(localOnlyJoints.size(), 1u);
  EXPECT_TRUE(localOnlyJoints[0].anchorsAreLocal);
  EXPECT_NEAR(
      (localOnlyJoints[0].anchorA - config.localAnchorA).norm(), 0.0, 1e-12);
  EXPECT_NEAR(
      (localOnlyJoints[0].anchorB - config.localAnchorB).norm(), 0.0, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldFixedJointHelperRejectsMultibodyLinks)
{
  sx::World world;
  sx::Multibody robot = world.addMultibody("robot");
  sx::Link root = robot.addLink("root");
  const sx::Link child = robot.addLink(
      "child",
      root,
      sx::JointSpec{
          .name = "fixed",
          .type = sx::JointType::Fixed,
          .transformFromParent
          = Eigen::Isometry3d(Eigen::Translation3d(Vec3::UnitX()))});
  const std::optional<sx::Joint> fixed = robot.getJoint("fixed");
  ASSERT_TRUE(fixed.has_value());

  auto& registry = dart::simulation::detail::registryOf(world);
  const vbd::AvbdRigidWorldEndpoint rootEndpoint
      = vbd::classifyAvbdRigidWorldEndpoint(
          registry, sx::detail::toRegistryEntity(root.getEntity()));
  EXPECT_EQ(rootEndpoint.kind, vbd::AvbdRigidWorldEndpointKind::MultibodyLink);
  EXPECT_FALSE(rootEndpoint.canProjectAsRigidBody);

  const vbd::AvbdRigidWorldEndpoint childEndpoint
      = vbd::classifyAvbdRigidWorldEndpoint(
          registry, sx::detail::toRegistryEntity(child.getEntity()));
  EXPECT_EQ(childEndpoint.kind, vbd::AvbdRigidWorldEndpointKind::MultibodyLink);
  EXPECT_FALSE(childEndpoint.canProjectAsRigidBody);

  EXPECT_FALSE(
      vbd::configureAvbdRigidWorldFixedJointFromCurrentPose(
          registry,
          sx::detail::toRegistryEntity(fixed->getEntity()),
          100.0,
          1000.0));
  EXPECT_FALSE(registry.all_of<vbd::AvbdRigidWorldPointJointConfig>(
      sx::detail::toRegistryEntity(fixed->getEntity())));
  EXPECT_EQ(
      vbd::configureAvbdRigidWorldPointJointsFromCurrentPoses(registry), 0u);

  registry.emplace<vbd::AvbdRigidWorldPointJointConfig>(
      sx::detail::toRegistryEntity(fixed->getEntity()));
  EXPECT_TRUE(vbd::extractAvbdRigidWorldPointJointInputs(registry).empty());
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotSolveMovesDynamicBody)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Vec3(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox(Vec3(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Vec3(0.0, 0.0, 0.4);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  vbd::AvbdRigidWorldContactOptions contactOptions;
  contactOptions.startStiffness = 200.0;
  vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          world.collide(),
          contactOptions);
  ASSERT_FALSE(snapshot.contacts.empty());

  const std::size_t sphereIndex = findEntityIndex(
      snapshot.entities,
      dart::simulation::detail::toRegistryEntity(sphere.getEntity()));
  const double initialSphereZ = snapshot.states[sphereIndex].position.z();

  vbd::AvbdRigidWorldContactSolveOptions solveOptions;
  solveOptions.descent.iterations = 4;
  solveOptions.descent.convergenceDisplacement = 1e-12;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  const vbd::AvbdRigidWorldContactSolveResult result
      = vbd::solveAvbdRigidWorldContactSnapshot(
          snapshot,
          normalInventory,
          frictionInventory,
          /*timeStep=*/1.0,
          solveOptions);

  EXPECT_EQ(result.normalRows, snapshot.contacts.size());
  EXPECT_GT(result.stats.bodyUpdates, 0u);
  EXPECT_GT(snapshot.states[sphereIndex].position.z(), initialSphereZ);

  ASSERT_FALSE(normalInventory.empty());
  bool foundPositiveNormalDual = false;
  for (const vbd::AvbdScalarRowRecord& record : normalInventory.records()) {
    foundPositiveNormalDual
        = foundPositiveNormalDual || record.state.lambda > 0.0;
  }
  EXPECT_TRUE(foundPositiveNormalDual);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotApplyWritesDynamicBodyState)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Vec3(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox(Vec3(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Vec3(0.0, 0.0, 0.4);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  vbd::AvbdRigidWorldContactOptions contactOptions;
  contactOptions.startStiffness = 200.0;
  vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          world.collide(),
          contactOptions);
  ASSERT_FALSE(snapshot.contacts.empty());

  const std::size_t sphereIndex = findEntityIndex(
      snapshot.entities,
      dart::simulation::detail::toRegistryEntity(sphere.getEntity()));
  const double initialSphereZ = sphere.getTransform().translation().z();

  vbd::AvbdRigidWorldContactSolveOptions solveOptions;
  solveOptions.descent.iterations = 4;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  const vbd::AvbdRigidWorldContactSolveResult solveResult
      = vbd::solveAvbdRigidWorldContactSnapshot(
          snapshot,
          normalInventory,
          frictionInventory,
          /*timeStep=*/1.0,
          solveOptions);
  ASSERT_GT(solveResult.stats.bodyUpdates, 0u);
  ASSERT_GT(snapshot.states[sphereIndex].position.z(), initialSphereZ);

  const double writebackTimeStep = 0.5;
  const vbd::AvbdRigidWorldContactApplyResult applyResult
      = vbd::applyAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world),
          snapshot,
          writebackTimeStep);

  EXPECT_EQ(applyResult.bodies, 1u);
  EXPECT_NEAR(
      sphere.getTransform().translation().z(),
      snapshot.states[sphereIndex].position.z(),
      1e-12);
  EXPECT_NEAR(
      sphere.getLinearVelocity().z(),
      (snapshot.states[sphereIndex].position.z() - initialSphereZ)
          / writebackTimeStep,
      1e-12);
  EXPECT_NEAR(ground.getTransform().translation().z(), -0.25, 1e-12);
  EXPECT_TRUE(ground.getLinearVelocity().isApprox(Vec3::Zero(), 1e-12));
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactStepSolvesAndWritesDynamicBody)
{
  sx::World world;
  world.setGravity(Vec3::Zero());

  sx::RigidBodyOptions groundOptions;
  groundOptions.isStatic = true;
  groundOptions.position = Vec3(0.0, 0.0, -0.25);
  auto ground = world.addRigidBody("ground", groundOptions);
  ground.setCollisionShape(sx::CollisionShape::makeBox(Vec3(2.0, 2.0, 0.25)));

  sx::RigidBodyOptions sphereOptions;
  sphereOptions.mass = 1.0;
  sphereOptions.position = Vec3(0.0, 0.0, 0.4);
  auto sphere = world.addRigidBody("sphere", sphereOptions);
  sphere.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const double initialSphereZ = sphere.getTransform().translation().z();
  const auto contacts = world.collide();
  ASSERT_FALSE(contacts.empty());

  vbd::AvbdRigidWorldContactStepOptions options;
  options.contact.startStiffness = 200.0;
  options.solve.descent.iterations = 4;
  vbd::AvbdScalarRowInventory normalInventory;
  vbd::AvbdScalarRowInventory frictionInventory;
  const double timeStep = 0.5;
  const vbd::AvbdRigidWorldContactStepResult result
      = vbd::runAvbdRigidWorldContactStep(
          dart::simulation::detail::registryOf(world),
          contacts,
          normalInventory,
          frictionInventory,
          timeStep,
          options);

  EXPECT_EQ(result.bodies, 2u);
  EXPECT_EQ(result.contacts, contacts.size());
  EXPECT_EQ(result.apply.bodies, 1u);
  EXPECT_GT(result.solve.stats.bodyUpdates, 0u);
  EXPECT_GT(sphere.getTransform().translation().z(), initialSphereZ);
  EXPECT_NEAR(
      sphere.getLinearVelocity().z(),
      (sphere.getTransform().translation().z() - initialSphereZ) / timeStep,
      1e-12);
  EXPECT_NEAR(ground.getTransform().translation().z(), -0.25, 1e-12);
}

//==============================================================================
TEST(AvbdRigidBlock, RigidWorldContactSnapshotSkipsStaticPairs)
{
  sx::World world;

  sx::RigidBodyOptions optionsA;
  optionsA.isStatic = true;
  auto bodyA = world.addRigidBody("static_a", optionsA);
  bodyA.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  sx::RigidBodyOptions optionsB;
  optionsB.isStatic = true;
  optionsB.position = Vec3(0.25, 0.0, 0.0);
  auto bodyB = world.addRigidBody("static_b", optionsB);
  bodyB.setCollisionShape(sx::CollisionShape::makeSphere(0.5));

  const std::vector<sx::Contact> contacts = world.collide();
  ASSERT_FALSE(contacts.empty());

  const vbd::AvbdRigidWorldContactSnapshot snapshot
      = vbd::buildAvbdRigidWorldContactSnapshot(
          dart::simulation::detail::registryOf(world), contacts);

  EXPECT_TRUE(snapshot.entities.empty());
  EXPECT_TRUE(snapshot.contacts.empty());
}

//==============================================================================
TEST(AvbdRigidBlock, SolveRejectsIndefiniteHessian)
{
  vbd::AvbdRigidBodyBlock block;
  block.force.setOnes();
  block.hessian.diagonal().array() = -1.0;

  EXPECT_NEAR(vbd::solveAvbdRigidBodyBlock(block).norm(), 0.0, 1e-12);
}
