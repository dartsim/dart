/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "helpers/GTestUtils.hpp"

#include "dart/utils/DartResourceRetriever.hpp"
#include "dart/utils/sdf/SdfParser.hpp"
#include "dart/utils/sdf/detail/SdfHelpers.hpp"

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/common/Uri.hpp>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <sdf/Root.hh>
#include <sdf/sdf.hh>

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>

using dart::common::Uri;
using dart::dynamics::Joint;
using dart::dynamics::MimicConstraintType;
using dart::dynamics::SkeletonPtr;
using dart::simulation::WorldPtr;
using dart::utils::SdfParser::detail::ElementEnumerator;
using dart::utils::SdfParser::detail::getAttributeString;
using dart::utils::SdfParser::detail::getElement;
using dart::utils::SdfParser::detail::getValueDouble;
using dart::utils::SdfParser::detail::hasAttribute;
using dart::utils::SdfParser::detail::hasElement;

namespace {

struct MimicSpec
{
  std::string model;
  std::string followerJoint;
  std::string referenceJoint;
  std::size_t referenceDof = 0;
  double multiplier = 1.0;
  double offset = 0.0;
};

std::vector<MimicSpec> parseMimicSpecs(const std::string& sdfText)
{
  sdf::Root root;
  const auto errors = root.LoadSdfString(sdfText);
  if (!errors.empty()) {
    GTEST_FAIL()
        << "Failed to load SDF from text: " << errors.front().Message();
  }

  const auto rootElement = root.Element();
  if (!rootElement)
    return {};

  const auto worldElement = getElement(rootElement, "world");
  if (!worldElement)
    return {};

  std::vector<MimicSpec> specs;
  ElementEnumerator modelEnum(worldElement, "model");
  while (modelEnum.next()) {
    const auto modelElement = modelEnum.get();
    if (!modelElement)
      continue;

    const auto modelName = getAttributeString(modelElement, "name");
    ElementEnumerator jointEnum(modelElement, "joint");
    while (jointEnum.next()) {
      const auto jointElement = jointEnum.get();
      if (!jointElement || !hasElement(jointElement, "axis"))
        continue;

      const auto axisElement = getElement(jointElement, "axis");
      if (!hasElement(axisElement, "mimic"))
        continue;

      const auto mimicElement = getElement(axisElement, "mimic");
      if (!mimicElement)
        continue;

      MimicSpec spec;
      spec.model = modelName;
      spec.followerJoint = getAttributeString(jointElement, "name");
      spec.referenceJoint = getAttributeString(mimicElement, "joint");
      const auto axisAttribute = hasAttribute(mimicElement, "axis")
                                     ? getAttributeString(mimicElement, "axis")
                                     : std::string();
      spec.referenceDof = axisAttribute == "axis2" ? 1u : 0u;
      spec.multiplier = hasElement(mimicElement, "multiplier")
                            ? getValueDouble(mimicElement, "multiplier")
                            : 1.0;
      spec.offset = hasElement(mimicElement, "offset")
                        ? getValueDouble(mimicElement, "offset")
                        : 0.0;

      specs.push_back(spec);
    }
  }

  return specs;
}

void configureMimicCouplers(
    const std::vector<MimicSpec>& specs, const WorldPtr& world)
{
  for (const auto& spec : specs) {
    const auto skeleton = world->getSkeleton(spec.model);
    ASSERT_NE(nullptr, skeleton);

    auto* follower = skeleton->getJoint(spec.followerJoint);
    auto* reference = skeleton->getJoint(spec.referenceJoint);
    ASSERT_NE(nullptr, follower);
    ASSERT_NE(nullptr, reference);
    ASSERT_GT(follower->getNumDofs(), 0u);
    ASSERT_GT(reference->getNumDofs(), 0u);

    std::vector<dart::dynamics::MimicDofProperties> mimicProps
        = follower->getMimicDofProperties();
    mimicProps.resize(follower->getNumDofs());

    const std::size_t followerIndex
        = std::min(spec.referenceDof, follower->getNumDofs() - 1);
    const std::size_t referenceIndex
        = std::min(spec.referenceDof, reference->getNumDofs() - 1);
    auto& prop = mimicProps[followerIndex];
    prop.mReferenceJoint = reference;
    prop.mReferenceDofIndex = referenceIndex;
    prop.mMultiplier = spec.multiplier;
    prop.mOffset = spec.offset;
    prop.mConstraintType = MimicConstraintType::Coupler;

    follower->setMimicJointDofs(mimicProps);
    follower->setActuatorType(Joint::MIMIC);
    follower->setUseCouplerConstraint(true);
  }
}

bool hasFiniteState(const SkeletonPtr& skeleton)
{
  bool finite = true;
  skeleton->eachBodyNode([&](dart::dynamics::BodyNode* bn) {
    const auto tf = bn->getWorldTransform();
    if (!tf.matrix().array().isFinite().all())
      finite = false;
  });
  return finite;
}

Eigen::Vector3d getTranslation(const dart::dynamics::BodyNode* bn)
{
  if (bn == nullptr)
    return Eigen::Vector3d::Zero();
  return bn->getWorldTransform().translation();
}

} // namespace

//==============================================================================
TEST(MimicConstraint, PendulumMimicWorldFromSdf)
{
  const std::string worldUri
      = "dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf";

  auto retriever = std::make_shared<dart::utils::DartResourceRetriever>();
  dart::utils::SdfParser::Options options(retriever);

  WorldPtr world = dart::utils::SdfParser::readWorld(Uri(worldUri), options);
  ASSERT_TRUE(world);

  auto resource = retriever->retrieve(Uri(worldUri));
  ASSERT_TRUE(resource);
  std::string sdfText(resource->getSize(), '\0');
  const auto read = resource->read(sdfText.data(), 1, sdfText.size());
  ASSERT_EQ(read, resource->getSize());

  const auto specs = parseMimicSpecs(sdfText);
  ASSERT_FALSE(specs.empty());
  configureMimicCouplers(specs, world);

  const auto slowFollower
      = world->getSkeleton("pendulum_with_base_mimic_slow_follows_fast");
  ASSERT_TRUE(slowFollower);
  const auto fastFollower
      = world->getSkeleton("pendulum_with_base_mimic_fast_follows_slow");
  ASSERT_TRUE(fastFollower);

  const auto slowBase = slowFollower->getBodyNode("base");
  const auto fastBase = fastFollower->getBodyNode("base");
  ASSERT_NE(nullptr, slowBase);
  ASSERT_NE(nullptr, fastBase);

  const Eigen::Vector3d slowBaseStart = getTranslation(slowBase);
  const Eigen::Vector3d fastBaseStart = getTranslation(fastBase);

  constexpr int steps = 32;
  for (int i = 0; i < steps; ++i) {
    world->step();
    ASSERT_TRUE(hasFiniteState(slowFollower));
    ASSERT_TRUE(hasFiniteState(fastFollower));
  }

  const Eigen::Vector3d slowBaseNow = getTranslation(slowBase);
  const Eigen::Vector3d fastBaseNow = getTranslation(fastBase);

  EXPECT_LT((slowBaseNow.head<2>() - slowBaseStart.head<2>()).norm(), 0.5);
  EXPECT_LT((fastBaseNow.head<2>() - fastBaseStart.head<2>()).norm(), 0.5);
  EXPECT_LT(std::abs(slowBaseNow.z() - slowBaseStart.z()), 0.5);
  EXPECT_LT(std::abs(fastBaseNow.z() - fastBaseStart.z()), 0.5);

  auto* slowJoint = slowFollower->getJoint("slow_joint");
  auto* fastJoint = slowFollower->getJoint("fast_joint");
  ASSERT_NE(nullptr, slowJoint);
  ASSERT_NE(nullptr, fastJoint);

  const double slowAngle = slowJoint->getPosition(0);
  const double fastAngle = fastJoint->getPosition(0);

  ASSERT_TRUE(std::isfinite(slowAngle));
  ASSERT_TRUE(std::isfinite(fastAngle));
  EXPECT_LT(std::abs(slowAngle - fastAngle), 0.2);
}
