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

#include "TestHelpers.hpp"
#include "dart/collision/CollisionDetector.hpp"
#include "dart/collision/CollisionOption.hpp"
#include "dart/collision/CollisionResult.hpp"
#include "dart/collision/dart/DARTCollisionDetector.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/ResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ShapeFrame.hpp"
#include "dart/dynamics/ShapeNode.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/math/Constants.hpp"
#include "dart/utils/CompositeResourceRetriever.hpp"
#include "dart/utils/DartResourceRetriever.hpp"
#include "dart/utils/mjcf/MjcfParser.hpp"
#include "dart/utils/mjcf/detail/GeomCollisionFilter.hpp"
#include "dart/utils/mjcf/detail/Inertial.hpp"
#include "dart/utils/mjcf/detail/MujocoModel.hpp"
#include "dart/utils/mjcf/detail/Types.hpp"
#include "dart/utils/mjcf/detail/Utils.hpp"
#include "dart/utils/mjcf/detail/WeldAttributes.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cmath>
#include <cstddef>

using namespace dart;
using namespace utils::MjcfParser::detail;

//==============================================================================
common::ResourceRetrieverPtr createRetriever()
{
  auto newRetriever = std::make_shared<utils::CompositeResourceRetriever>();
  newRetriever->addSchemaRetriever(
      "file", std::make_shared<common::LocalResourceRetriever>());
  newRetriever->addSchemaRetriever(
      "dart", utils::DartResourceRetriever::create());
  return newRetriever;
}

struct XmlRetriever : common::LocalResourceRetriever
{
  std::string xml;
  std::string readAll(const common::Uri&) override
  {
    return xml;
  }
};

//==============================================================================
TEST(MjcfParserTest, WorldbodySites)
{
  for (const auto* geom : {"", "<geom type='sphere' size='.2'/>"}) {
    auto retriever = std::make_shared<XmlRetriever>();
    retriever->xml = std::string("<mujoco><worldbody>") + geom + R"(
      <site name="target" pos="1 2 3" size=".1"/>
      <site name="other" pos="4 5 6" size=".2"/>
    </worldbody></mujoco>)";
    MujocoModel mujoco;
    ASSERT_TRUE(mujoco.read("memory.xml", retriever).empty());
    const auto& worldbody = mujoco.getWorldbody();
    ASSERT_EQ(worldbody.getNumSites(), 2u);
    EXPECT_EQ(worldbody.getSite(0).getName(), "target");
    EXPECT_EQ(worldbody.getSite(1).getName(), "other");
    EXPECT_TRUE(equals(
        worldbody.getSite(0).getRelativeTransform().translation().eval(),
        Eigen::Vector3d(1, 2, 3)));

    const utils::MjcfParser::Options options(retriever);
    auto world = utils::MjcfParser::readWorld("memory.xml", options);
    ASSERT_NE(world, nullptr);
    EXPECT_EQ(world->getNumSkeletons(), *geom == '\0' ? 2u : 3u);
    auto target
        = world->getSkeleton(options.mSiteSkeletonNamePrefix + "target");
    ASSERT_NE(target, nullptr);
    EXPECT_FALSE(target->isMobile());
    EXPECT_TRUE(equals(
        target->getBodyNode(0)->getWorldTransform().translation().eval(),
        Eigen::Vector3d(1, 2, 3)));
    auto* shape = target->getBodyNode(0)->getShapeNode(0);
    EXPECT_TRUE(shape->hasVisualAspect());
    EXPECT_FALSE(shape->hasCollisionAspect());
    EXPECT_NE(
        world->getSkeleton(options.mSiteSkeletonNamePrefix + "other"), nullptr);
  }
}

//==============================================================================
TEST(MjcfParserTest, FreeJointShorthand)
{
  auto retriever = std::make_shared<XmlRetriever>();
  retriever->xml = R"(
    <mujoco><worldbody><body name="floating" pos="1 2 3">
      <freejoint/>
      <geom type="sphere" size=".1"/>
    </body></worldbody></mujoco>)";
  auto world = utils::MjcfParser::readWorld("memory.xml", {retriever});
  ASSERT_NE(world, nullptr);
  auto skel = world->getSkeleton("floating");
  ASSERT_NE(skel, nullptr);
  EXPECT_EQ(skel->getNumDofs(), 6u);
  EXPECT_EQ(
      skel->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());
  EXPECT_TRUE(equals(
      skel->getRootBodyNode()->getWorldTransform().translation().eval(),
      Eigen::Vector3d(1, 2, 3)));
}

//==============================================================================
TEST(MjcfParserTest, FreeJointShorthandIgnoresDefaults)
{
  auto retriever = std::make_shared<XmlRetriever>();
  retriever->xml = R"(
    <mujoco>
      <default>
        <joint type="slide" damping="7" limited="true" range="-1 2"
               springref=".5" pos=".1 .2 .3"/>
        <default class="child"><joint damping="9" springref="1"/></default>
      </default>
      <worldbody>
        <body name="main"><freejoint name="main_free"/>
          <geom type="sphere" size=".1"/></body>
        <body name="child" childclass="child"><freejoint name="child_free" group="2"/>
          <geom type="sphere" size=".1"/></body>
        <body name="ordinary" childclass="child"><joint name="inherited"/>
          <geom type="sphere" size=".1"/></body>
      </worldbody>
    </mujoco>)";
  MujocoModel mujoco;
  ASSERT_TRUE(mujoco.read("memory.xml", retriever).empty());
  const auto& worldbody = mujoco.getWorldbody();
  ASSERT_EQ(worldbody.getNumRootBodies(), 3u);
  for (auto i = 0u; i < 2u; ++i) {
    const auto& body = worldbody.getRootBody(i);
    ASSERT_EQ(body.getNumJoints(), 1u);
    const auto& joint = body.getJoint(0);
    EXPECT_EQ(joint.getName(), body.getName() + "_free");
    EXPECT_EQ(joint.getType(), JointType::FREE);
    EXPECT_FALSE(joint.isLimited());
    EXPECT_DOUBLE_EQ(joint.getDamping(), 0);
    EXPECT_DOUBLE_EQ(joint.getSpringRef(), 0);
    EXPECT_TRUE(joint.getPos().isZero());
    EXPECT_TRUE(joint.getRange().isZero());
  }
  const auto& inherited = worldbody.getRootBody(2).getJoint(0);
  EXPECT_EQ(inherited.getType(), JointType::SLIDE);
  EXPECT_DOUBLE_EQ(inherited.getDamping(), 9);
  EXPECT_DOUBLE_EQ(inherited.getSpringRef(), 1);
  EXPECT_TRUE(inherited.isLimited());

  auto world = utils::MjcfParser::readWorld("memory.xml", {retriever});
  ASSERT_NE(world, nullptr);
  for (const auto* name : {"main", "child"}) {
    auto skel = world->getSkeleton(name);
    ASSERT_NE(skel, nullptr);
    EXPECT_EQ(skel->getNumDofs(), 6u);
    auto* joint = skel->getRootJoint();
    EXPECT_EQ(joint->getType(), dynamics::FreeJoint::getStaticType());
    for (auto i = 0u; i < joint->getNumDofs(); ++i) {
      EXPECT_DOUBLE_EQ(joint->getDampingCoefficient(i), 0);
      EXPECT_DOUBLE_EQ(joint->getSpringStiffness(i), 0);
      EXPECT_EQ(joint->getPositionLowerLimit(i), -math::constantsd::inf());
      EXPECT_EQ(joint->getPositionUpperLimit(i), math::constantsd::inf());
    }
  }
}

//==============================================================================
TEST(MjcfParserTest, ParseDetailQuadruped)
{
  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  const auto errors = mujoco.read(
      "dart://sample/mjcf/deepmind/quadruped.xml", createRetriever());
  ASSERT_TRUE(errors.empty());
  EXPECT_EQ(mujoco.getModel(), "quadruped");
  EXPECT_EQ(mujoco.getCompiler().getAngle(), Angle::DEGREE);
  EXPECT_EQ(mujoco.getCompiler().getCoordinate(), Coordinate::LOCAL);
  EXPECT_EQ(mujoco.getOption().getIntegrator(), Integrator::EULER);
  EXPECT_DOUBLE_EQ(mujoco.getOption().getTimestep(), 0.005);
  const auto& worldbody = mujoco.getWorldbody();
  EXPECT_EQ(worldbody.getNumGeoms(), 5u);
  ASSERT_EQ(worldbody.getNumSites(), 1u);
  EXPECT_EQ(worldbody.getSite(0).getName(), "target");
  ASSERT_EQ(worldbody.getNumRootBodies(), 2u);
  const auto& torso = worldbody.getRootBody(0);
  EXPECT_EQ(torso.getName(), "torso");
  EXPECT_TRUE(equals(
      torso.getRelativeTransform().translation().eval(),
      Eigen::Vector3d(0, 0, 0.57)));
  ASSERT_EQ(torso.getNumJoints(), 1u);
  EXPECT_EQ(torso.getJoint(0).getType(), JointType::FREE);
  EXPECT_EQ(torso.getJoint(0).getName(), "root");
}

//==============================================================================
TEST(MjcfParserTest, DefaultSettings)
{
  // For the details, see http://mujoco.org/book/modeling.html#CDefault

  const auto uri = "dart://sample/mjcf/test/default.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto boxSkel = world->getSkeleton(options.mGeomSkeletonNamePrefix);
  ASSERT_NE(boxSkel, nullptr);
  auto boxShapeNode = boxSkel->getShapeNode(0);
  ASSERT_NE(boxShapeNode, nullptr);
  auto boxVisualAspect = boxShapeNode->getVisualAspect();
  ASSERT_NE(boxVisualAspect, nullptr);
  EXPECT_TRUE(boxVisualAspect->getRGBA().isApprox(Eigen::Vector4d(1, 0, 0, 1)));

  auto bodySkel = world->getSkeleton("body0");
  ASSERT_NE(bodySkel, nullptr);

  auto ellipsoidShapeNode = bodySkel->getShapeNode(0);
  ASSERT_NE(ellipsoidShapeNode, nullptr);
  auto ellipsoidVisualAspect = ellipsoidShapeNode->getVisualAspect();
  ASSERT_NE(ellipsoidVisualAspect, nullptr);
  ellipsoidVisualAspect->getRGBA().isApprox(Eigen::Vector4d(0, 1, 0, 1));

  auto sphereShapeNode = bodySkel->getShapeNode(1);
  ASSERT_NE(sphereShapeNode, nullptr);
  auto sphereVisualAspect = sphereShapeNode->getVisualAspect();
  ASSERT_NE(sphereVisualAspect, nullptr);
  sphereVisualAspect->getRGBA().isApprox(Eigen::Vector4d(0, 0, 1, 1));

  auto cylinderShapeNode = bodySkel->getShapeNode(2);
  ASSERT_NE(cylinderShapeNode, nullptr);
  auto cylinderVisualAspect = cylinderShapeNode->getVisualAspect();
  ASSERT_NE(cylinderVisualAspect, nullptr);
  cylinderVisualAspect->getRGBA().isApprox(Eigen::Vector4d(1, 0, 0, 1));
}

//==============================================================================
TEST(MjcfParserTest, IncludeDefaultSettings)
{
  // For the details, see http://mujoco.org/book/modeling.html#CDefault

  const auto uri = "dart://sample/mjcf/test/include_main.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto boxSkel = world->getSkeleton(options.mGeomSkeletonNamePrefix);
  ASSERT_NE(boxSkel, nullptr);
  auto boxShapeNode = boxSkel->getShapeNode(0);
  ASSERT_NE(boxShapeNode, nullptr);
  auto boxVisualAspect = boxShapeNode->getVisualAspect();
  ASSERT_NE(boxVisualAspect, nullptr);
  EXPECT_TRUE(boxVisualAspect->getRGBA().isApprox(Eigen::Vector4d(1, 0, 0, 1)));

  auto bodySkel = world->getSkeleton("body0");
  ASSERT_NE(bodySkel, nullptr);

  auto ellipsoidShapeNode = bodySkel->getShapeNode(0);
  ASSERT_NE(ellipsoidShapeNode, nullptr);
  auto ellipsoidVisualAspect = ellipsoidShapeNode->getVisualAspect();
  ASSERT_NE(ellipsoidVisualAspect, nullptr);
  ellipsoidVisualAspect->getRGBA().isApprox(Eigen::Vector4d(0, 1, 0, 1));

  auto sphereShapeNode = bodySkel->getShapeNode(1);
  ASSERT_NE(sphereShapeNode, nullptr);
  auto sphereVisualAspect = sphereShapeNode->getVisualAspect();
  ASSERT_NE(sphereVisualAspect, nullptr);
  sphereVisualAspect->getRGBA().isApprox(Eigen::Vector4d(0, 0, 1, 1));

  auto cylinderShapeNode = bodySkel->getShapeNode(2);
  ASSERT_NE(cylinderShapeNode, nullptr);
  auto cylinderVisualAspect = cylinderShapeNode->getVisualAspect();
  ASSERT_NE(cylinderVisualAspect, nullptr);
  cylinderVisualAspect->getRGBA().isApprox(Eigen::Vector4d(1, 0, 0, 1));
}

//==============================================================================
TEST(MjcfParserTest, Quadruped)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/deepmind/quadruped.xml");
  ASSERT_NE(world, nullptr);
  // Five root geoms, a target site and two root bodies.
  ASSERT_EQ(world->getNumSkeletons(), 8u);
  auto skel = world->getSkeleton("torso");
  ASSERT_NE(skel, nullptr);
  // Torso + four four-body legs + one carrier for each hip's two hinges.
  EXPECT_EQ(skel->getNumBodyNodes(), 21u);
  EXPECT_EQ(skel->getNumJoints(), 21u);
  EXPECT_EQ(skel->getNumDofs(), 22u);
  EXPECT_EQ(
      skel->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());
  ASSERT_NE(world->getSkeleton("ball"), nullptr);
  EXPECT_EQ(world->getSkeleton("ball")->getNumDofs(), 6u);
  auto yaw = skel->getJoint("yaw_front_left");
  ASSERT_NE(yaw, nullptr);
  EXPECT_TRUE(yaw->areLimitsEnforced());
  EXPECT_DOUBLE_EQ(
      yaw->getPositionLowerLimit(0), -50 * math::constantsd::pi() / 180);
  EXPECT_DOUBLE_EQ(
      yaw->getPositionUpperLimit(0), 50 * math::constantsd::pi() / 180);
}

//==============================================================================
TEST(MjcfParserTest, Cartpole)
{
  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  ASSERT_TRUE(
      mujoco.read("dart://sample/mjcf/deepmind/cartpole.xml", createRetriever())
          .empty());
  EXPECT_EQ(mujoco.getOption().getIntegrator(), Integrator::RK4);
  EXPECT_DOUBLE_EQ(mujoco.getOption().getTimestep(), 0.01);
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/deepmind/cartpole.xml");
  ASSERT_NE(world, nullptr);
  // Floor, two rails and the cart body.
  ASSERT_EQ(world->getNumSkeletons(), 4u);
  auto skel = world->getSkeleton("cart");
  ASSERT_NE(skel, nullptr);
  ASSERT_EQ(skel->getNumBodyNodes(), 2u);
  EXPECT_EQ(skel->getNumJoints(), 2u);
  EXPECT_EQ(skel->getNumDofs(), 2u);
  EXPECT_EQ(
      skel->getRootJoint()->getType(),
      dynamics::PrismaticJoint::getStaticType());
  EXPECT_DOUBLE_EQ(skel->getRootJoint()->getPositionLowerLimit(0), -1.8);
  EXPECT_DOUBLE_EQ(skel->getRootJoint()->getPositionUpperLimit(0), 1.8);
  EXPECT_DOUBLE_EQ(skel->getBodyNode("cart")->getMass(), 1.0);
  EXPECT_DOUBLE_EQ(skel->getBodyNode("pole_1")->getMass(), 0.1);
  const auto options = utils::MjcfParser::Options();
  for (const auto* name : {"floor", "rail1", "rail2"})
    EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + name));
}

//==============================================================================
TEST(MjcfParserTest, Acrobot)
{
  auto world
      = utils::MjcfParser::readWorld("dart://sample/mjcf/deepmind/acrobot.xml");
  ASSERT_NE(world, nullptr);
  // Floor, target site and the two-link arm.
  ASSERT_EQ(world->getNumSkeletons(), 3u);
  auto skel = world->getSkeleton("upper_arm");
  ASSERT_NE(skel, nullptr);
  ASSERT_EQ(skel->getNumBodyNodes(), 2u);
  EXPECT_EQ(skel->getNumJoints(), 2u);
  EXPECT_EQ(skel->getNumDofs(), 2u);
  EXPECT_EQ(
      skel->getRootJoint()->getType(),
      dynamics::RevoluteJoint::getStaticType());
  EXPECT_DOUBLE_EQ(skel->getBodyNode("upper_arm")->getMass(), 1.0);
  EXPECT_DOUBLE_EQ(skel->getBodyNode("lower_arm")->getMass(), 1.0);
}

//==============================================================================
TEST(MjcfParserTest, Pendulum)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/deepmind/pendulum.xml");
  ASSERT_NE(world, nullptr);
  ASSERT_EQ(world->getNumSkeletons(), 2u);
  auto skel = world->getSkeleton("pole");
  ASSERT_NE(skel, nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 1u);
  EXPECT_EQ(skel->getNumJoints(), 1u);
  EXPECT_EQ(skel->getNumDofs(), 1u);
  EXPECT_DOUBLE_EQ(skel->getMass(), 1.0);
}

//==============================================================================
TEST(MjcfParserTest, Reacher)
{
  auto world
      = utils::MjcfParser::readWorld("dart://sample/mjcf/deepmind/reacher.xml");
  ASSERT_NE(world, nullptr);
  // Seven root geoms and one three-body arm.
  ASSERT_EQ(world->getNumSkeletons(), 8u);
  auto skel = world->getSkeleton("arm");
  ASSERT_NE(skel, nullptr);
  EXPECT_EQ(skel->getNumBodyNodes(), 3u);
  EXPECT_EQ(skel->getNumJoints(), 3u);
  EXPECT_EQ(skel->getNumDofs(), 2u);
  EXPECT_EQ(
      skel->getRootJoint()->getType(),
      dynamics::RevoluteJoint::getStaticType());
  auto wrist = skel->getJoint("wrist");
  ASSERT_NE(wrist, nullptr);
  EXPECT_TRUE(wrist->areLimitsEnforced());
  EXPECT_DOUBLE_EQ(
      wrist->getPositionLowerLimit(0), -160 * math::constantsd::pi() / 180);
  EXPECT_DOUBLE_EQ(
      wrist->getPositionUpperLimit(0), 160 * math::constantsd::pi() / 180);
  const auto options = utils::MjcfParser::Options();
  for (const auto* name :
       {"ground",
        "wall_x",
        "wall_y",
        "wall_neg_x",
        "wall_neg_y",
        "root",
        "target"})
    EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + name));
}

//==============================================================================
TEST(MjcfParserTest, VisualOnlyGeoms)
{
  const auto uri = "dart://sample/mjcf/test/manipulation/scene.xml";
  const auto options = utils::MjcfParser::Options();
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  // The number of skeletons are the sum of the number of root <geom>s and root
  // <body>s in <worldbody>.
  ASSERT_EQ(world->getNumSkeletons(), 6);

  auto armSkel = world->getSkeleton("arm");
  ASSERT_NE(armSkel, nullptr);
  ASSERT_EQ(armSkel->getNumBodyNodes(), 3);
  EXPECT_EQ(
      armSkel->getRootJoint()->getType(),
      dynamics::RevoluteJoint::getStaticType());

  auto objectSkel = world->getSkeleton("object");
  ASSERT_NE(objectSkel, nullptr);

  auto goalSkel = world->getSkeleton("goal");
  ASSERT_NE(goalSkel, nullptr);

  EXPECT_TRUE(world->hasSkeleton(options.mGeomSkeletonNamePrefix + "table"));

  // The inherited zero masks keep the arm and goal markers visual-only.
  auto armBody = armSkel->getBodyNode("arm");
  ASSERT_NE(armBody, nullptr);
  ASSERT_GT(armBody->getNumShapeNodes(), 0u);
  EXPECT_FALSE(armBody->getShapeNode(0)->has<dynamics::CollisionAspect>());

  auto goalRootBody = goalSkel->getRootBodyNode();
  ASSERT_NE(goalRootBody, nullptr);
  ASSERT_GT(goalRootBody->getNumShapeNodes(), 0u);
  EXPECT_FALSE(goalRootBody->getShapeNode(0)->has<dynamics::CollisionAspect>());

  auto markerBody = goalSkel->getBodyNode("marker");
  ASSERT_NE(markerBody, nullptr);
  ASSERT_GT(markerBody->getNumShapeNodes(), 0u);
  EXPECT_FALSE(markerBody->getShapeNode(0)->has<dynamics::CollisionAspect>());

  // Geoms that explicitly re-enable collision (contype="1" conaffinity="1")
  // must still get a CollisionAspect: the table and the arm-tip geoms.
  auto tableSkel
      = world->getSkeleton(options.mGeomSkeletonNamePrefix + "table");
  ASSERT_NE(tableSkel, nullptr);
  ASSERT_GT(tableSkel->getNumBodyNodes(), 0u);
  ASSERT_GT(tableSkel->getBodyNode(0)->getNumShapeNodes(), 0u);
  auto tableShapeNode = tableSkel->getBodyNode(0)->getShapeNode(0);
  EXPECT_TRUE(tableShapeNode->has<dynamics::CollisionAspect>());
  // The table geom doesn't override friction, so it inherits the <default>
  // class's friction=".0 .0 .0".
  ASSERT_TRUE(tableShapeNode->has<dynamics::DynamicsAspect>());
  EXPECT_DOUBLE_EQ(
      tableShapeNode->getDynamicsAspect()->getFrictionCoeff(), 0.0);

  auto tipBody = armSkel->getBodyNode("tip");
  ASSERT_NE(tipBody, nullptr);
  ASSERT_GT(tipBody->getNumShapeNodes(), 0u);
  EXPECT_TRUE(tipBody->getShapeNode(0)->has<dynamics::CollisionAspect>());

  auto wristBody = armSkel->getBodyNode("wrist");
  ASSERT_NE(wristBody, nullptr);
  ASSERT_GT(wristBody->getNumShapeNodes(), 0u);
  EXPECT_TRUE(wristBody->getShapeNode(0)->has<dynamics::CollisionAspect>());

  // The heavy marker overlaps the table but must never generate a contact.
  world->getConstraintSolver()->setCollisionDetector(
      collision::DARTCollisionDetector::create());

  for (auto i = 0; i < 50; ++i) {
    world->step();

    const auto& result = world->getConstraintSolver()->getLastCollisionResult();
    for (auto j = 0u; j < result.getNumContacts(); ++j) {
      const auto& contact = result.getContact(j);
      const auto bn1 = contact.getBodyNodePtr1();
      const auto bn2 = contact.getBodyNodePtr2();
      const bool involvesMarker = (bn1 && bn1.get() == markerBody)
                                  || (bn2 && bn2.get() == markerBody);
      EXPECT_FALSE(involvesMarker)
          << "Contact involving the visual-only marker at step " << i
          << ", contact index " << j;
    }
  }

  EXPECT_TRUE(armSkel->getPositions().allFinite());
  EXPECT_TRUE(armSkel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, GeomFriction)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/manipulation/scene.xml");
  ASSERT_NE(world, nullptr);
  const auto options = utils::MjcfParser::Options();
  // Top-level default, nested default class, then an explicit override.
  for (const auto& entry :
       {std::make_pair("table", 0.0), std::make_pair("floor", 0.8)}) {
    auto skel
        = world->getSkeleton(options.mGeomSkeletonNamePrefix + entry.first);
    ASSERT_NE(skel, nullptr);
    auto shape = skel->getBodyNode(0)->getShapeNode(0);
    ASSERT_TRUE(shape->has<dynamics::DynamicsAspect>());
    EXPECT_DOUBLE_EQ(
        shape->getDynamicsAspect()->getFrictionCoeff(), entry.second);
  }
  auto arm = world->getSkeleton("arm");
  ASSERT_NE(arm, nullptr);
  auto wrist = arm->getBodyNode("wrist");
  ASSERT_NE(wrist, nullptr);
  auto shape = wrist->getShapeNode(0);
  ASSERT_TRUE(shape->has<dynamics::DynamicsAspect>());
  EXPECT_DOUBLE_EQ(shape->getDynamicsAspect()->getFrictionCoeff(), 0.9);
}

//==============================================================================
TEST(MjcfParserTest, ContypeConaffinityPairRule)
{
  // A minimal scene with three mutually-overlapping static spheres exercising
  // MuJoCo's contype/conaffinity pair rule in isolation:
  //   collides(A, B) := (contype_A & conaffinity_B) || (contype_B &
  //   conaffinity_A)
  // bodyA: contype=1 conaffinity=0
  // bodyB: contype=0 conaffinity=1
  // bodyC: contype=1 conaffinity=0 (same masks as bodyA)
  // A vs B -> (1 & 1) || (0 & 0) = 1 -> collide
  // A vs C -> (1 & 0) || (1 & 0) = 0 -> do not collide
  const auto uri = "dart://sample/mjcf/test/contype_conaffinity.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  auto skelA = world->getSkeleton("bodyA");
  auto skelB = world->getSkeleton("bodyB");
  auto skelC = world->getSkeleton("bodyC");
  ASSERT_NE(skelA, nullptr);
  ASSERT_NE(skelB, nullptr);
  ASSERT_NE(skelC, nullptr);

  world->getConstraintSolver()->setCollisionDetector(
      collision::DARTCollisionDetector::create());

  world->step();

  const auto& result = world->getConstraintSolver()->getLastCollisionResult();

  auto hasContactBetween
      = [&](const dynamics::SkeletonPtr& s1, const dynamics::SkeletonPtr& s2) {
          for (auto i = 0u; i < result.getNumContacts(); ++i) {
            const auto& contact = result.getContact(i);
            const auto bn1 = contact.getBodyNodePtr1();
            const auto bn2 = contact.getBodyNodePtr2();
            if (!bn1 || !bn2)
              continue;

            const bool matches = (bn1->getSkeleton().get() == s1.get()
                                  && bn2->getSkeleton().get() == s2.get())
                                 || (bn1->getSkeleton().get() == s2.get()
                                     && bn2->getSkeleton().get() == s1.get());
            if (matches)
              return true;
          }
          return false;
        };

  EXPECT_TRUE(hasContactBetween(skelA, skelB));
  EXPECT_FALSE(hasContactBetween(skelA, skelC));
}

//==============================================================================
TEST(MjcfParserTest, IncludedMeshScene)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/manipulation/scene.xml");
  ASSERT_NE(world, nullptr);
  ASSERT_EQ(world->getNumSkeletons(), 6u);
  auto arm = world->getSkeleton("arm");
  ASSERT_NE(arm, nullptr);
  ASSERT_EQ(arm->getNumBodyNodes(), 3u);
  EXPECT_EQ(arm->getNumDofs(), 2u);
  auto tip = arm->getBodyNode("tip");
  ASSERT_NE(tip, nullptr);
  ASSERT_NE(tip->getParentBodyNode(), nullptr);
  EXPECT_EQ(tip->getParentBodyNode()->getName(), "wrist");
  EXPECT_EQ(
      tip->getParentJoint()->getType(), dynamics::WeldJoint::getStaticType());
  auto object = world->getSkeleton("object");
  ASSERT_NE(object, nullptr);
  EXPECT_EQ(object->getNumBodyNodes(), 1u);
  EXPECT_EQ(object->getNumDofs(), 2u);
  auto shape = object->getBodyNode(0)->getShapeNode(0)->getShape();
  ASSERT_NE(shape, nullptr);
  EXPECT_EQ(shape->getType(), dynamics::MeshShape::getStaticType());
  EXPECT_TRUE(shape->getBoundingBox().computeFullExtents().isApprox(
      Eigen::Vector3d::Constant(0.04), 1e-6));
  ASSERT_NE(world->getSkeleton("ball"), nullptr);
  EXPECT_EQ(
      world->getSkeleton("ball")->getRootJoint()->getType(),
      dynamics::FreeJoint::getStaticType());
}

//==============================================================================
TEST(MjcfParserTest, FailedWorldWithWeld)
{
  const auto retriever = std::make_shared<XmlRetriever>();
  retriever->xml = R"(<mujoco><worldbody><body name="invalid">
    <joint type="ball"/><joint type="hinge"/>
    <geom type="sphere" size=".1"/>
    </body></worldbody><equality><weld body1="invalid"/></equality></mujoco>)";
  // Parsing succeeds; construction rejects the ball/hinge stack before welds.
  auto mujoco = utils::MjcfParser::detail::MujocoModel();
  ASSERT_TRUE(mujoco.read("invalid.xml", retriever).empty());
  EXPECT_EQ(
      utils::MjcfParser::readWorld(
          "invalid.xml", utils::MjcfParser::Options(retriever)),
      nullptr);
}

//==============================================================================
// Bodies with more <joint>s than DART has a predefined multi-DOF joint for
// (i.e. anything other than 2 or 3 slide joints) are represented as a chain
// of single-DOF joints connected by intermediate massless BodyNodes. Those
// BodyNodes are named "<ownerBodyName>__mjcf_dof<i>" and carry no shapes.
std::size_t countStackedJointDummyBodies(const dynamics::SkeletonPtr& skel)
{
  std::size_t count = 0u;
  for (auto i = 0u; i < skel->getNumBodyNodes(); ++i) {
    if (skel->getBodyNode(i)->getName().find("__mjcf_dof")
        != std::string::npos) {
      ++count;
    }
  }
  return count;
}

//==============================================================================
// A stacked joint needs inertialess carrier links to preserve the MJCF body's
// mass and spatial inertia exactly. Individual zero-inertia BodyNodes trigger
// DART's conservative local-link warning, but the relevant dynamics invariant
// is that descendant body inertia makes the generalized mass matrix positive
// definite and forward dynamics finite.
void expectStackedJointDynamicsWellPosed(const dynamics::SkeletonPtr& skeleton)
{
  ASSERT_NE(skeleton, nullptr);
  const Eigen::MatrixXd massMatrix = skeleton->getMassMatrix();
  ASSERT_TRUE(massMatrix.allFinite());

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(massMatrix);
  ASSERT_EQ(eigensolver.info(), Eigen::Success);
  EXPECT_GT(eigensolver.eigenvalues().minCoeff(), 1e-10);

  skeleton->setForces(Eigen::VectorXd::LinSpaced(
      skeleton->getNumDofs(), 0.01, 0.01 * skeleton->getNumDofs()));
  skeleton->computeForwardDynamics();
  EXPECT_TRUE(skeleton->getAccelerations().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, HumanoidStackedHingeJoints)
{
  // The humanoid model has several bodies with 2 or 3 stacked hinge joints
  // (lower_waist: 2, right_thigh/left_thigh: 3, right_upper_arm/left_upper_arm:
  // 2) that are not any of the predefined multi-DOF joint compositions.
  const auto uri = "dart://sample/mjcf/deepmind/humanoid.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto humanoidSkel = world->getSkeleton("torso");
  ASSERT_NE(humanoidSkel, nullptr);

  // 6 DOFs from the free root joint + 21 DOFs from the hinge joints.
  EXPECT_EQ(humanoidSkel->getNumDofs(), 27u);

  // 16 real bodies + 9 carriers: waist (1), thighs (4), feet (2), arms (2).
  EXPECT_EQ(humanoidSkel->getNumBodyNodes(), 25u);
  EXPECT_EQ(countStackedJointDummyBodies(humanoidSkel), 9u);

  const std::vector<std::string> realBodyNames
      = {"torso",
         "head",
         "lower_waist",
         "pelvis",
         "right_thigh",
         "right_shin",
         "right_foot",
         "left_thigh",
         "left_shin",
         "left_foot",
         "right_upper_arm",
         "right_lower_arm",
         "right_hand",
         "left_upper_arm",
         "left_lower_arm",
         "left_hand"};
  for (const auto& name : realBodyNames) {
    EXPECT_NE(humanoidSkel->getBodyNode(name), nullptr) << name;
  }

  EXPECT_EQ(
      humanoidSkel->getRootJoint()->getType(),
      dynamics::FreeJoint::getStaticType());

  expectStackedJointDynamicsWellPosed(humanoidSkel);

  world->getConstraintSolver()->setCollisionDetector(
      collision::DARTCollisionDetector::create());

  for (auto i = 0; i < 100; ++i) {
    world->step();
  }

  EXPECT_TRUE(humanoidSkel->getPositions().allFinite());
  EXPECT_TRUE(humanoidSkel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, HumanoidDefaultDetector)
{
  // Exercise the same humanoid through the parser's default detector.
  const auto uri = "dart://sample/mjcf/deepmind/humanoid.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto humanoidSkel = world->getSkeleton("torso");
  ASSERT_NE(humanoidSkel, nullptr);

  EXPECT_EQ(humanoidSkel->getNumDofs(), 27u);
  EXPECT_EQ(humanoidSkel->getNumBodyNodes(), 25u);
  EXPECT_EQ(countStackedJointDummyBodies(humanoidSkel), 9u);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(humanoidSkel->getPositions().allFinite());
  EXPECT_TRUE(humanoidSkel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, WalkerMixedJointStack)
{
  // torso has a 2-slide + 1-hinge stack (rootz, rootx, rooty), which is not
  // the predefined 3-slide TranslationalJoint composition.
  const auto uri = "dart://sample/mjcf/deepmind/walker.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto skel = world->getSkeleton("torso");
  ASSERT_NE(skel, nullptr);

  // 7 real bodies (torso and three in each leg)
  // + 2 dummies for the torso's 3-joint stack.
  EXPECT_EQ(skel->getNumBodyNodes(), 9u);
  EXPECT_EQ(countStackedJointDummyBodies(skel), 2u);

  // 3 DOFs from torso's stack + 1 DOF from each of the 6 single-hinge bodies.
  EXPECT_EQ(skel->getNumDofs(), 9u);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(skel->getPositions().allFinite());
  EXPECT_TRUE(skel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, HopperMixedJointStack)
{
  const auto uri = "dart://sample/mjcf/deepmind/hopper.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto skel = world->getSkeleton("torso");
  ASSERT_NE(skel, nullptr);

  // 5 real bodies (torso, pelvis, thigh, calf, foot) + 2 root-stack carriers.
  EXPECT_EQ(skel->getNumBodyNodes(), 7u);
  EXPECT_EQ(countStackedJointDummyBodies(skel), 2u);
  EXPECT_EQ(skel->getNumDofs(), 7u);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(skel->getPositions().allFinite());
  EXPECT_TRUE(skel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, CheetahMixedJointStack)
{
  const auto uri = "dart://sample/mjcf/deepmind/cheetah.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 2);

  auto skel = world->getSkeleton("torso");
  ASSERT_NE(skel, nullptr);

  // 7 real bodies (torso, bthigh, bshin, bfoot, fthigh, fshin, ffoot) + 2
  // dummies for torso's stack.
  EXPECT_EQ(skel->getNumBodyNodes(), 9u);
  EXPECT_EQ(countStackedJointDummyBodies(skel), 2u);
  EXPECT_EQ(skel->getNumDofs(), 9u);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(skel->getPositions().allFinite());
  EXPECT_TRUE(skel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, SwimmerMixedJointStack)
{
  const auto uri = "dart://sample/mjcf/deepmind/swimmer.xml";
  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  ASSERT_EQ(world->getNumSkeletons(), 3);

  auto skel = world->getSkeleton("head");
  ASSERT_NE(skel, nullptr);

  // Head and two generated segments + 2 carriers for the head's root stack.
  EXPECT_EQ(skel->getNumBodyNodes(), 5u);
  EXPECT_EQ(countStackedJointDummyBodies(skel), 2u);
  EXPECT_EQ(skel->getNumDofs(), 5u);
  // Only the three inertial geoms contribute mass (0.01 each).
  EXPECT_DOUBLE_EQ(skel->getMass(), 0.03);
  // The generated segment hinges inherit limited="true" with +/-120 degrees.
  for (const auto* name : {"joint_0", "joint_1"}) {
    const auto* joint = skel->getJoint(name);
    ASSERT_NE(joint, nullptr) << name;
    EXPECT_TRUE(joint->areLimitsEnforced()) << name;
    EXPECT_DOUBLE_EQ(
        joint->getPositionLowerLimit(0), -120 * math::constantsd::pi() / 180);
    EXPECT_DOUBLE_EQ(
        joint->getPositionUpperLimit(0), 120 * math::constantsd::pi() / 180);
  }

  expectStackedJointDynamicsWellPosed(skel);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(skel->getPositions().allFinite());
  EXPECT_TRUE(skel->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, UnnamedRootBodyWithUnnamedJointStack)
{
  // Regression: a root <body> without a name whose joints are also unnamed
  // used to crash in the stacked-joint chain path (nullptr parent BodyNode in
  // the unnamed-joint name fallback), and unnamed root bodies must keep
  // inheriting a generated BodyNode name instead of a synthetic
  // "__mjcf_dof" link name or the World's generic default.
  const auto uri = "dart://sample/mjcf/test/unnamed_root_joint_stack.xml";

  auto world = utils::MjcfParser::readWorld(uri);
  ASSERT_NE(world, nullptr);

  dynamics::SkeletonPtr stacked = nullptr;
  for (std::size_t i = 0u; i < world->getNumSkeletons(); ++i) {
    auto skel = world->getSkeleton(i);
    if (skel->getNumDofs() == 2u) {
      stacked = skel;
      break;
    }
  }
  ASSERT_NE(stacked, nullptr);

  // Two chained revolute joints: one synthetic intermediate link plus the
  // real body.
  EXPECT_EQ(stacked->getNumBodyNodes(), 2u);
  EXPECT_NE(stacked->getRootJoint(), nullptr);

  // The skeleton name must come from the real (non-synthetic) BodyNode.
  EXPECT_EQ(stacked->getName().find("__mjcf_dof"), std::string::npos);
  EXPECT_FALSE(stacked->getName().empty());

  expectStackedJointDynamicsWellPosed(stacked);

  for (auto i = 0; i < 10; ++i) {
    world->step();
  }

  EXPECT_TRUE(stacked->getPositions().allFinite());
  EXPECT_TRUE(stacked->getVelocities().allFinite());
}

//==============================================================================
TEST(MjcfParserTest, GeomFilterPreservesDeactivationSnapshots)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/deactivation_filter.xml");
  ASSERT_NE(world, nullptr);

  auto box = world->getSkeleton("box");
  ASSERT_NE(box, nullptr);
  ASSERT_TRUE(box->isMobile());
  EXPECT_EQ(box->getNumDofs(), 6u);
  EXPECT_EQ(
      box->getRootJoint()->getType(), dynamics::FreeJoint::getStaticType());

  // A free body must settle into a valid contact island before it can sleep.
  for (std::size_t i = 0u; i < 10000u && !box->isResting(); ++i)
    world->step();
  ASSERT_TRUE(box->isResting());

  const Eigen::VectorXd frozenPositions = box->getPositions();
  for (std::size_t i = 0u; i < 100u; ++i) {
    world->step();
    ASSERT_TRUE(box->isResting()) << "MJCF box spuriously woke at step " << i;
    EXPECT_TRUE(box->getPositions().isApprox(frozenPositions, 1e-12));
  }

  auto* geomFilter = dynamic_cast<GeomCollisionFilter*>(
      world->getConstraintSolver()->getCollisionOption().collisionFilter.get());
  ASSERT_NE(geomFilter, nullptr);
  auto* boxShape = box->getBodyNode(0u)->getShapeNode(0u);
  ASSERT_NE(boxShape, nullptr);
  geomFilter->setGeomBitmasks(boxShape, 0, 0);

  world->step();
  EXPECT_FALSE(box->isResting())
      << "collision-filter revision change did not invalidate the snapshot";
}

//==============================================================================
TEST(MjcfParserTest, GeomFilterDoesNotRetainRemovedSkeletons)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/contype_conaffinity.xml");
  ASSERT_NE(world, nullptr);
  ASSERT_GT(world->getNumSkeletons(), 0u);

  auto collisionFilter
      = world->getConstraintSolver()->getCollisionOption().collisionFilter;
  ASSERT_NE(collisionFilter, nullptr);

  std::vector<dynamics::WeakSkeletonPtr> removedSkeletons;
  while (world->getNumSkeletons() > 0u) {
    auto skeleton = world->getSkeleton(0u);
    removedSkeletons.emplace_back(skeleton);
    world->removeSkeleton(skeleton);
  }

  for (const auto& skeleton : removedSkeletons)
    EXPECT_TRUE(skeleton.expired());

  const auto addOverlappingBox = [&world](const std::string& name) {
    auto skeleton = dynamics::Skeleton::create(name);
    auto pair = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>();
    pair.second->createShapeNodeWith<dynamics::CollisionAspect>(
        std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()));
    world->addSkeleton(skeleton);
    return skeleton;
  };

  const auto box1 = addOverlappingBox("replacement_1");
  const auto box2 = addOverlappingBox("replacement_2");
  auto collisionGroup = world->getConstraintSolver()
                            ->getCollisionDetector()
                            ->createCollisionGroup(box1.get(), box2.get());
  collision::CollisionOption option;
  option.collisionFilter = collisionFilter;
  EXPECT_TRUE(collisionGroup->collide(option));
}

//==============================================================================
TEST(MjcfParserTest, StackedJointPreservesLogicalBodyAdjacency)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/stacked_joint_adjacency.xml");
  ASSERT_NE(world, nullptr);

  auto skeleton = world->getSkeleton("parent");
  ASSERT_NE(skeleton, nullptr);
  // A free parent and two child hinges, including the massless carrier.
  EXPECT_EQ(skeleton->getNumDofs(), 8u);
  EXPECT_EQ(
      skeleton->getRootJoint()->getType(),
      dynamics::FreeJoint::getStaticType());
  skeleton->enableSelfCollisionCheck();

  auto collisionGroup = world->getConstraintSolver()
                            ->getCollisionDetector()
                            ->createCollisionGroup(skeleton.get());
  ASSERT_EQ(collisionGroup->getNumShapeFrames(), 2u);
  collision::CollisionOption option;
  option.collisionFilter
      = world->getConstraintSolver()->getCollisionOption().collisionFilter;
  collision::CollisionResult result;

  EXPECT_FALSE(collisionGroup->collide(option, &result));

  skeleton->enableAdjacentBodyCheck();
  result.clear();
  EXPECT_TRUE(collisionGroup->collide(option, &result));
  EXPECT_GT(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(MjcfParserTest, WeldSolrefAcceptsFewerValues)
{
  // MJCF allows fewer solref values than the maximum; the rest keep defaults.
  tinyxml2::XMLDocument doc;
  ASSERT_EQ(
      doc.Parse(R"(<weld body1="a" solref="0.05"/>)"), tinyxml2::XML_SUCCESS);
  WeldAttributes attributes;
  EXPECT_TRUE(appendWeldAttributes(attributes, doc.RootElement()).empty());
  EXPECT_DOUBLE_EQ(attributes.mSolRef[0], 0.05);
  EXPECT_DOUBLE_EQ(attributes.mSolRef[1], 1.0);

  ASSERT_EQ(
      doc.Parse(R"(<weld body1="a" solref="1 2 3"/>)"), tinyxml2::XML_SUCCESS);
  WeldAttributes invalid;
  EXPECT_FALSE(appendWeldAttributes(invalid, doc.RootElement()).empty());
}

//==============================================================================
TEST(MjcfParserTest, JointLimitSemantics)
{
  testing::internal::CaptureStderr();
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/joint_limits.xml");
  const auto warnings = testing::internal::GetCapturedStderr();
  const std::string warning
      = "Ball joint limits are not supported yet, ignored.";
  const auto firstWarning = warnings.find(warning);
  ASSERT_NE(firstWarning, std::string::npos);
  EXPECT_NE(
      warnings.find("First limited ball joint: 'ball_true'"),
      std::string::npos);
  EXPECT_EQ(
      warnings.find(warning, firstWarning + warning.size()), std::string::npos);
  ASSERT_NE(world, nullptr);
  auto skeleton = world->getSkeleton("limits");
  ASSERT_NE(skeleton, nullptr);

  for (const auto& name :
       {"hinge_true",
        "auto_range",
        "implicit_auto",
        "inherited_true",
        "inherited_child",
        "inherited_auto",
        "stacked_hinge"}) {
    SCOPED_TRACE(name);
    const auto* joint = skeleton->getJoint(name);
    ASSERT_NE(joint, nullptr);
    EXPECT_TRUE(joint->areLimitsEnforced());
    EXPECT_NEAR(
        joint->getPositionLowerLimit(0), -math::constantsd::pi() / 6, 1e-12);
    EXPECT_NEAR(
        joint->getPositionUpperLimit(0), math::constantsd::pi() / 3, 1e-12);
  }
  for (const auto& name :
       {"false_range",
        "false_no_range",
        "auto_no_range",
        "auto_zero",
        "auto_reversed",
        "inherited_zero",
        "inherited_false",
        "overridden_false",
        "stacked_false_hinge",
        "stacked_false_slide",
        "ball_true",
        "ball_no_range"}) {
    SCOPED_TRACE(name);
    const auto* joint = skeleton->getJoint(name);
    ASSERT_NE(joint, nullptr);
    EXPECT_FALSE(joint->areLimitsEnforced());
    for (std::size_t i = 0; i < joint->getNumDofs(); ++i) {
      EXPECT_EQ(
          joint->getPositionLowerLimit(i),
          -std::numeric_limits<double>::infinity());
      EXPECT_EQ(
          joint->getPositionUpperLimit(i),
          std::numeric_limits<double>::infinity());
    }
  }
  EXPECT_FALSE(
      world->getSkeleton("ball_auto")->getRootJoint()->areLimitsEnforced());
  EXPECT_FALSE(world->getSkeleton("free")->getRootJoint()->areLimitsEnforced());
  for (const auto& name :
       {"slide_true", "stacked_slide", "stacked_limited_slide"}) {
    const auto* joint = skeleton->getJoint(name);
    ASSERT_NE(joint, nullptr);
    EXPECT_TRUE(joint->areLimitsEnforced());
    EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), -0.1);
    EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 0.2);
  }
  for (const auto& name : {"slides2", "slides3"}) {
    SCOPED_TRACE(name);
    const auto* joint = skeleton->getBodyNode(name)->getParentJoint();
    ASSERT_NE(joint, nullptr);
    EXPECT_TRUE(joint->areLimitsEnforced());
    for (std::size_t i = 0; i + 1 < joint->getNumDofs(); ++i) {
      EXPECT_EQ(
          joint->getPositionLowerLimit(i),
          -std::numeric_limits<double>::infinity());
      EXPECT_EQ(
          joint->getPositionUpperLimit(i),
          std::numeric_limits<double>::infinity());
    }
    EXPECT_DOUBLE_EQ(
        joint->getPositionLowerLimit(joint->getNumDofs() - 1), -0.1);
    EXPECT_DOUBLE_EQ(
        joint->getPositionUpperLimit(joint->getNumDofs() - 1), 0.2);
  }
}

//==============================================================================
TEST(MjcfParserTest, JointLimitsConstrainMotion)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/joint_limits.xml");
  ASSERT_NE(world, nullptr);
  auto skeleton = world->getSkeleton("limits");
  ASSERT_NE(skeleton, nullptr);
  for (const auto& name : {"hinge_true", "false_range", "slide_true"}) {
    auto* joint = skeleton->getJoint(name);
    const double upper
        = std::string(name) == "slide_true" ? 0.2 : math::constantsd::pi() / 3;
    joint->setPosition(0, upper - 0.01);
    joint->setVelocity(0, 1);
  }
  for (const auto& name : {"slides2", "slides3"}) {
    auto* joint = skeleton->getBodyNode(name)->getParentJoint();
    joint->setVelocities(Eigen::VectorXd::Ones(joint->getNumDofs()));
    joint->setPosition(joint->getNumDofs() - 1, 0.19);
  }
  for (int i = 0; i < 100; ++i)
    world->step();
  for (const auto& name :
       {"hinge_true", "slide_true", "slides2_x", "slides3_x"}) {
    const auto* joint = skeleton->getJoint(name);
    ASSERT_NE(joint, nullptr);
    const auto last = joint->getNumDofs() - 1;
    EXPECT_LE(
        joint->getPosition(last), joint->getPositionUpperLimit(last) + 1e-5);
  }
  EXPECT_GT(
      skeleton->getJoint("false_range")->getPosition(0),
      math::constantsd::pi() / 3 + 0.05);
  for (const auto& name : {"slides2", "slides3"}) {
    const auto* joint = skeleton->getBodyNode(name)->getParentJoint();
    EXPECT_GT(joint->getPosition(0), 0.05);
  }
}

//==============================================================================
TEST(MjcfParserTest, ExplicitLimitsWithAutolimitsDisabled)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/autolimits_false.xml");
  ASSERT_NE(world, nullptr);
  auto skeleton = world->getSkeleton("explicit_true");
  ASSERT_NE(skeleton, nullptr);
  const auto* joint = skeleton->getRootJoint();
  EXPECT_TRUE(joint->areLimitsEnforced());
  EXPECT_DOUBLE_EQ(joint->getPositionLowerLimit(0), -0.3);
  EXPECT_DOUBLE_EQ(joint->getPositionUpperLimit(0), 0.6);
  EXPECT_DOUBLE_EQ(joint->getRestPosition(0), 0.3);
  EXPECT_FALSE(skeleton->getJoint("explicit_false")->areLimitsEnforced());
  for (const auto& name : {"auto_zero", "inherited_zero"}) {
    const auto* zero = skeleton->getJoint(name);
    ASSERT_NE(zero, nullptr);
    EXPECT_FALSE(zero->areLimitsEnforced());
    EXPECT_EQ(
        zero->getPositionLowerLimit(0),
        -std::numeric_limits<double>::infinity());
    EXPECT_EQ(
        zero->getPositionUpperLimit(0),
        std::numeric_limits<double>::infinity());
  }
}

//==============================================================================
TEST(MjcfParserTest, InvalidJointLimitDeclarations)
{
  const auto retriever = std::make_shared<XmlRetriever>();
  struct InvalidLimit
  {
    const char* compiler;
    const char* attributes;
    const char* error;
  };
  for (const auto& invalid :
       {InvalidLimit{
            "",
            "limited='invalid'",
            "Invalid attribute for 'limited': invalid"},
        InvalidLimit{
            "", "limited='True'", "Invalid attribute for 'limited': True"},
        InvalidLimit{
            "",
            "limited='true'",
            "range[0] should be smaller than range[1] in joint '(noname)'"},
        InvalidLimit{
            "",
            "name='hinge' limited='true' range='1 -1'",
            "range[0] should be smaller than range[1] in joint 'hinge'"},
        InvalidLimit{
            "",
            "limited='true' range='nan 1'",
            "Non-finite range in joint '(noname)'"},
        InvalidLimit{
            "",
            "limited='true' range='-1 inf'",
            "Non-finite range in joint '(noname)'"},
        InvalidLimit{
            "",
            "type='ball' limited='true' range='1 30'",
            "range[0] should be 0 in ball joint '(noname)'"},
        InvalidLimit{
            "",
            "type='ball' limited='auto' range='1 30'",
            "range[0] should be 0 in ball joint '(noname)'"},
        InvalidLimit{
            "",
            "limited='auto' range='-1 inf'",
            "Non-finite range in joint '(noname)'"},
        InvalidLimit{
            "autolimits='false'",
            "range='-1 1'",
            "Joint '(noname)' has a range with limited='auto' but autolimits "
            "is false"},
        InvalidLimit{
            "autolimits='false'",
            "limited='auto' range='-1 1'",
            "Joint '(noname)' has a range with limited='auto' but autolimits "
            "is false"},
        InvalidLimit{
            "autolimits='false'",
            "limited='auto' range='1 -1'",
            "Joint '(noname)' has a range with limited='auto' but autolimits "
            "is false"}}) {
    SCOPED_TRACE(invalid.attributes);
    retriever->xml = std::string("<mujoco><compiler ") + invalid.compiler
        + "/><worldbody><body><joint " + invalid.attributes
        + "/><inertial pos='0 0 0' mass='1' diaginertia='2 3 4'/></body></worldbody></mujoco>";
    MujocoModel model;
    const auto errors = model.read("memory.xml", retriever);
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_EQ(errors[0].getCode(), ErrorCode::ATTRIBUTE_INVALID);
    EXPECT_NE(errors[0].getMessage().find(invalid.error), std::string::npos)
        << errors[0].getMessage();
    EXPECT_EQ(utils::MjcfParser::readWorld("memory.xml", {retriever}), nullptr);
  }
  for (const auto& compiler : {"", "autolimits='true'", "autolimits='false'"}) {
    SCOPED_TRACE(compiler);
    retriever->xml = std::string("<mujoco><compiler ") + compiler
        + "/><worldbody><body><joint limited='auto'/><inertial pos='0 0 0' mass='1' diaginertia='2 3 4'/></body></worldbody></mujoco>";
    MujocoModel model;
    ASSERT_TRUE(model.read("memory.xml", retriever).empty());
    EXPECT_FALSE(model.getWorldbody().getRootBody(0).getJoint(0).isLimited());
  }
  for (const auto& value : {"invalid", "True", "False", "1"}) {
    SCOPED_TRACE(value);
    retriever->xml = std::string("<mujoco><compiler autolimits='") + value
                     + "'/></mujoco>";
    MujocoModel model;
    EXPECT_FALSE(model.read("memory.xml", retriever).empty());
    EXPECT_EQ(utils::MjcfParser::readWorld("memory.xml", {retriever}), nullptr);
  }
}

//==============================================================================
TEST(MjcfParserTest, FailedWorldDoesNotWarnAboutLimits)
{
  const auto retriever = std::make_shared<XmlRetriever>();
  for (const auto& bodies : {
           R"(<body><joint type="ball" limited="true" range="0 60"/><joint/>
              <inertial pos="0 0 0" mass="1" diaginertia="2 3 4"/></body>)",
           R"(<body><joint type="ball" limited="true" range="0 60"/>
              <inertial pos="0 0 0" mass="1" diaginertia="2 3 4"/>
              <body><joint type="ball"/><joint/>
                <inertial pos="0 0 0" mass="1" diaginertia="2 3 4"/>
              </body></body>)",
           R"(<body><joint type="ball" limited="true" range="0 60"/>
              <inertial pos="0 0 0" mass="1" diaginertia="2 3 4"/></body>
              <body><joint type="ball"/><joint/>
              <inertial pos="0 0 0" mass="1" diaginertia="2 3 4"/></body>)"}) {
    SCOPED_TRACE(bodies);
    retriever->xml
        = std::string("<mujoco><worldbody>") + bodies + "</worldbody></mujoco>";
    testing::internal::CaptureStderr();
    const auto world = utils::MjcfParser::readWorld("memory.xml", {retriever});
    const auto warnings = testing::internal::GetCapturedStderr();
    EXPECT_EQ(world, nullptr);
    EXPECT_EQ(
        warnings.find("Ball joint limits are not supported yet, ignored."),
        std::string::npos);
  }
}

//==============================================================================
TEST(MjcfParserTest, SpringReferenceAngleUnits)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/joint_limits.xml");
  ASSERT_NE(world, nullptr);
  const auto skeleton = world->getSkeleton("limits");
  ASSERT_NE(skeleton, nullptr);
  EXPECT_NEAR(
      skeleton->getJoint("hinge_true")->getRestPosition(0),
      math::constantsd::pi() / 6,
      1e-12);
  EXPECT_DOUBLE_EQ(skeleton->getJoint("slide_true")->getRestPosition(0), 0.1);

  const auto retriever = std::make_shared<XmlRetriever>();
  retriever->xml = R"(<mujoco><worldbody><body>
    <joint type="ball" springref="30"/>
    <inertial pos="0 0 0" mass="1" diaginertia="2 3 4"/>
    </body></worldbody></mujoco>)";
  MujocoModel model;
  ASSERT_TRUE(model.read("memory.xml", retriever).empty());
  EXPECT_DOUBLE_EQ(
      model.getWorldbody().getRootBody(0).getJoint(0).getSpringRef(), 30);
}

//==============================================================================
TEST(MjcfParserTest, DeclaredInertiaInBodyFrame)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/declared_inertia.xml");
  ASSERT_NE(world, nullptr);
  auto skeleton = world->getSkeleton("quat");
  ASSERT_NE(skeleton, nullptr);
  Eigen::Matrix3d rotated;
  rotated << 2.5, -0.5, 0, -0.5, 2.5, 0, 0, 0, 4;
  for (const auto& name :
       {"quat", "axisangle", "euler", "xyaxes", "no_body_pos"}) {
    SCOPED_TRACE(name);
    const auto* body = skeleton->getBodyNode(name);
    ASSERT_NE(body, nullptr);
    EXPECT_DOUBLE_EQ(body->getMass(), 2);
    EXPECT_TRUE(
        body->getLocalCOM().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3), 1e-12));
    EXPECT_TRUE(body->getInertia().getMoment().isApprox(rotated, 1e-12));
  }
  EXPECT_TRUE(skeleton->getBodyNode("no_body_pos")
                  ->getRelativeTransform()
                  .isApprox(Eigen::Isometry3d::Identity(), 1e-12));
  EXPECT_TRUE(skeleton->getBodyNode("zaxis")->getInertia().getMoment().isApprox(
      Eigen::Vector3d(4, 3, 2).asDiagonal().toDenseMatrix(), 1e-12));
  Eigen::Matrix3d full;
  full << 3, 0.2, -0.3, 0.2, 4, 0.4, -0.3, 0.4, 5;
  const auto* body = skeleton->getBodyNode("full");
  EXPECT_DOUBLE_EQ(body->getMass(), 3);
  EXPECT_TRUE(
      body->getLocalCOM().isApprox(Eigen::Vector3d(-0.1, 0.2, -0.3), 1e-12));
  EXPECT_TRUE(body->getInertia().getMoment().isApprox(full, 1e-12));
  EXPECT_DOUBLE_EQ(skeleton->getBodyNode("no_inertial")->getMass(), 7);
  EXPECT_TRUE(skeleton->getBodyNode("no_inertial")
                  ->getLocalCOM()
                  .isApprox(Eigen::Vector3d(0.1, 0.2, 0.3), 1e-12));
  EXPECT_DOUBLE_EQ(skeleton->getBodyNode("empty")->getMass(), 1);
}

//==============================================================================
TEST(MjcfParserTest, OmittedBodyPositionAndOrthogonalizedXYAxes)
{
  auto world = utils::MjcfParser::readWorld(
      "dart://sample/mjcf/test/declared_inertia.xml");
  ASSERT_NE(world, nullptr);
  const auto skeleton = world->getSkeleton("quat");
  ASSERT_NE(skeleton, nullptr);
  const auto* body = skeleton->getBodyNode("no_inertial");
  ASSERT_NE(body, nullptr);
  EXPECT_TRUE(body->getRelativeTransform().translation().isZero(1e-12));
  EXPECT_TRUE(body->getWorldTransform().translation().isApprox(
      body->getParentBodyNode()->getWorldTransform().translation(), 1e-12));
  EXPECT_TRUE(
      body->getLocalCOM().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3), 1e-12));
  Eigen::Matrix3d expected;
  const double c = std::sqrt(0.5);
  expected << c, -c, 0, c, c, 0, 0, 0, 1;
  EXPECT_TRUE(body->getRelativeTransform().linear().isApprox(expected, 1e-12));
  ASSERT_EQ(body->getNumShapeNodes(), 2u);
  for (std::size_t i = 0; i < body->getNumShapeNodes(); ++i) {
    const Eigen::Matrix3d rotation
        = body->getShapeNode(i)->getRelativeTransform().linear();
    EXPECT_TRUE(rotation.isApprox(expected, 1e-12));
    EXPECT_TRUE((rotation.transpose() * rotation)
                    .isApprox(Eigen::Matrix3d::Identity(), 1e-12));
  }
}

//==============================================================================
TEST(MjcfParserTest, FullInertiaConflictsWithOrientation)
{
  const auto retriever = std::make_shared<XmlRetriever>();
  for (const auto& orientation :
       {"quat='1 0 0 0'",
        "axisangle='0 0 1 0'",
        "euler='0 0 0'",
        "xyaxes='1 0 0 0 1 0'",
        "zaxis='0 0 1'"}) {
    SCOPED_TRACE(orientation);
    const std::string inertial = std::string("<inertial pos='0 0 0' mass='1' ")
                                 + "fullinertia='3 4 5 0.2 -0.3 0.4' "
                                 + orientation + "/>";
    tinyxml2::XMLDocument doc;
    ASSERT_EQ(doc.Parse(inertial.c_str()), tinyxml2::XML_SUCCESS);
    Inertial parsed;
    const auto errors = parsed.read(doc.RootElement());
    ASSERT_EQ(errors.size(), 1u);
    EXPECT_EQ(errors[0].getCode(), ErrorCode::ATTRIBUTE_CONFLICT);
    retriever->xml = "<mujoco><worldbody><body>" + inertial
                     + "</body></worldbody></mujoco>";
    EXPECT_EQ(utils::MjcfParser::readWorld("memory.xml", {retriever}), nullptr);
  }
}
