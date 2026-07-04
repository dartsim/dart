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

#ifndef DART_UNITTESTS_IO_ROUND_TRIP_HELPERS_HPP_
#define DART_UNITTESTS_IO_ROUND_TRIP_HELPERS_HPP_

#include "helpers/gtest_utils.hpp"

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

#include <string>
#include <string_view>

#include <cstddef>

namespace dart::test {

//==============================================================================
inline const dynamics::BodyNode* requireBodyNode(
    const dynamics::Skeleton& skeleton, std::string_view name)
{
  const auto* bodyNode = skeleton.getBodyNode(std::string(name));
  EXPECT_NE(bodyNode, nullptr) << "Missing BodyNode [" << name << "]";
  return bodyNode;
}

//==============================================================================
template <class JointT>
const JointT* requireJoint(
    const dynamics::Skeleton& skeleton, std::string_view name)
{
  const auto* joint
      = dynamic_cast<const JointT*>(skeleton.getJoint(std::string(name)));
  EXPECT_NE(joint, nullptr) << "Missing or wrong-type Joint [" << name << "]";
  return joint;
}

//==============================================================================
inline void expectJointTopology(
    const dynamics::Joint& joint,
    const dynamics::BodyNode* expectedParent,
    const dynamics::BodyNode* expectedChild)
{
  EXPECT_EQ(joint.getParentBodyNode(), expectedParent);
  EXPECT_EQ(joint.getChildBodyNode(), expectedChild);
}

//==============================================================================
inline void expectBodyInertia(
    const dynamics::BodyNode& bodyNode,
    double expectedMass,
    const Eigen::Vector3d& expectedLocalCom,
    const Eigen::Matrix3d& expectedMoment,
    double tolerance)
{
  EXPECT_NEAR(bodyNode.getMass(), expectedMass, tolerance);
  EXPECT_VECTOR_NEAR(bodyNode.getLocalCOM(), expectedLocalCom, tolerance);
  EXPECT_MATRIX_NEAR(
      bodyNode.getInertia().getMoment(), expectedMoment, tolerance);
}

//==============================================================================
inline void expectDofPositionLimits(
    const dynamics::Joint& joint,
    std::size_t dofIndex,
    double expectedLower,
    double expectedUpper,
    double tolerance)
{
  EXPECT_NEAR(joint.getPositionLowerLimit(dofIndex), expectedLower, tolerance);
  EXPECT_NEAR(joint.getPositionUpperLimit(dofIndex), expectedUpper, tolerance);
}

//==============================================================================
inline void expectDofDynamics(
    const dynamics::Joint& joint,
    std::size_t dofIndex,
    double expectedDamping,
    double expectedFriction,
    double expectedRest,
    double expectedSpringStiffness,
    double tolerance)
{
  EXPECT_NEAR(
      joint.getDampingCoefficient(dofIndex), expectedDamping, tolerance);
  EXPECT_NEAR(joint.getCoulombFriction(dofIndex), expectedFriction, tolerance);
  EXPECT_NEAR(joint.getRestPosition(dofIndex), expectedRest, tolerance);
  EXPECT_NEAR(
      joint.getSpringStiffness(dofIndex), expectedSpringStiffness, tolerance);
}

//==============================================================================
template <class AspectT, class ShapeT>
const ShapeT* requireShape(
    const dynamics::BodyNode& bodyNode,
    std::size_t index,
    std::size_t expectedCount)
{
  const std::size_t count = bodyNode.getNumShapeNodesWith<AspectT>();
  EXPECT_EQ(count, expectedCount);
  if (index >= count) {
    return nullptr;
  }

  const auto* shapeNode = bodyNode.getShapeNodeWith<AspectT>(index);
  EXPECT_NE(shapeNode, nullptr);
  if (!shapeNode) {
    return nullptr;
  }

  const auto* shape = dynamic_cast<const ShapeT*>(shapeNode->getShape().get());
  EXPECT_NE(shape, nullptr)
      << "Unexpected shape type in ShapeNode [" << shapeNode->getName() << "]";
  return shape;
}

} // namespace dart::test

#endif // DART_UNITTESTS_IO_ROUND_TRIP_HELPERS_HPP_
