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

// Tests for the contact_benchmark `--gz-preset` helpers, which mirror how the
// gz-physics dartsim plugin configures a world (plane boxes, per-pair contact
// limit, contact cap) and report what Gazebo would see (contact demand,
// starved pairs, sunk bodies, changed poses).

#include "GazeboPreset.hpp"

#include <dart/collision/ode/OdeCollisionDetector.hpp>

#include <dart/dart.hpp>

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <cmath>

using namespace dart;
using namespace dart::examples::contact_benchmark;

namespace {

using GazeboOdeDetector
    = GazeboPairLimitedDetector<collision::OdeCollisionDetector>;

//==============================================================================
dynamics::SkeletonPtr addGroundPlane(
    simulation::World& world,
    const Eigen::Vector3d& normal = Eigen::Vector3d::UnitZ(),
    double height = 0.0)
{
  auto ground = dynamics::Skeleton::create("ground");
  auto* body = ground->createJointAndBodyNodePair<dynamics::WeldJoint>().second;
  body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(
      std::make_shared<dynamics::PlaneShape>(normal, 0.0));
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = height;
  body->getParentJoint()->setTransformFromParentBodyNode(transform);
  ground->setMobile(false);
  world.addSkeleton(ground);
  return ground;
}

//==============================================================================
dynamics::SkeletonPtr addFreeBody(
    simulation::World& world,
    const std::string& name,
    const dynamics::ShapePtr& shape,
    const Eigen::Isometry3d& transform)
{
  auto skeleton = dynamics::Skeleton::create(name);
  auto* body
      = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>().second;
  body->createShapeNodeWith<
      dynamics::CollisionAspect,
      dynamics::DynamicsAspect>(shape);
  dynamics::Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(shape->computeInertia(1.0));
  body->setInertia(inertia);
  dynamics::FreeJoint::setTransformOf(body, transform);
  world.addSkeleton(skeleton);
  return skeleton;
}

//==============================================================================
Eigen::Isometry3d translation(double x, double y, double z)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(x, y, z);
  return transform;
}

//==============================================================================
// A world whose contact demand (four box-on-ground contacts per box) exceeds
// the global cap, configured the way --gz-preset configures it.
simulation::WorldPtr createStarvedBoxWorld(
    std::size_t boxes,
    std::size_t maxNumContacts,
    std::size_t pairLimit = kGazeboDefaultCollisionPairMaxContacts)
{
  auto world = simulation::World::create();
  addGroundPlane(*world);
  for (std::size_t i = 0; i < boxes; ++i) {
    addFreeBody(
        *world,
        "box" + std::to_string(i),
        std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
        translation(2.0 * static_cast<double>(i), 0.0, 0.4999));
  }
  rebuildPlanesLikeGazebo(*world);

  auto* solver = world->getConstraintSolver();
  solver->setCollisionDetector(GazeboOdeDetector::create(pairLimit));
  solver->getCollisionOption().maxNumContacts = maxNumContacts;
  solver->getCollisionOption().collisionFilter
      = std::make_shared<GazeboContactFilter>();
  return world;
}

} // namespace

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, PairLimitKeepsLeadingContactsThenDeepest)
{
  // Two real collision objects so CollisionResult can cache them.
  auto world = createStarvedBoxWorld(1, kGazeboMaxNumContacts);
  world->step();
  const auto& solved = world->getLastCollisionResult();
  ASSERT_GT(solved.getNumContacts(), 0u);
  collision::Contact base = solved.getContact(0);
  collision::Contact swapped = base;
  std::swap(swapped.collisionObject1, swapped.collisionObject2);

  collision::CollisionResult result;
  const std::vector<double> depths{0.1, 0.2, 0.05, 0.3, 0.01};
  for (std::size_t i = 0; i < depths.size(); ++i) {
    // Alternate the object order: gz-physics counts both orders as one pair.
    collision::Contact contact = (i % 2 == 1) ? swapped : base;
    contact.penetrationDepth = depths[i];
    result.addContact(contact);
  }

  collision::CollisionResult unlimited = result;
  limitCollisionPairMaxContacts(
      unlimited, std::numeric_limits<std::size_t>::max());
  EXPECT_EQ(unlimited.getNumContacts(), 5u);

  limitCollisionPairMaxContacts(result, 2u);
  ASSERT_EQ(result.getNumContacts(), 2u);
  EXPECT_DOUBLE_EQ(result.getContact(0).penetrationDepth, 0.1);
  // The second kept slot is replaced by the deepest later contact.
  EXPECT_DOUBLE_EQ(result.getContact(1).penetrationDepth, 0.3);

  limitCollisionPairMaxContacts(result, 0u);
  EXPECT_EQ(result.getNumContacts(), 0u);
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, RebuildsPlanesAsGazeboBoxes)
{
  auto world = simulation::World::create();
  auto flat = addGroundPlane(*world, Eigen::Vector3d::UnitZ(), 0.25);
  const Eigen::Vector3d tiltedNormal
      = Eigen::Vector3d(0.0, 1.0, 1.0).normalized();
  auto tilted = addGroundPlane(*world, tiltedNormal, 5.0);

  const auto groundTop = rebuildPlanesLikeGazebo(*world);
  ASSERT_TRUE(groundTop.has_value());
  EXPECT_DOUBLE_EQ(*groundTop, 0.25);

  for (const auto& skeleton : {flat, tilted}) {
    auto* shapeNode = skeleton->getBodyNode(0)->getShapeNode(0);
    const auto box = std::dynamic_pointer_cast<const dynamics::BoxShape>(
        shapeNode->getShape());
    ASSERT_NE(box, nullptr);
    EXPECT_TRUE(box->getSize().isApprox(
        Eigen::Vector3d::Constant(kGazeboPlaneBoxSize)));
  }

  // The top face of each box lies on its plane.
  const Eigen::Isometry3d& flatBox
      = flat->getBodyNode(0)->getShapeNode(0)->getWorldTransform();
  EXPECT_TRUE(flatBox.translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.25 - 0.5 * kGazeboPlaneBoxSize)));
  const Eigen::Isometry3d& tiltedBox
      = tilted->getBodyNode(0)->getShapeNode(0)->getWorldTransform();
  EXPECT_TRUE(
      (tiltedBox.linear() * Eigen::Vector3d::UnitZ()).isApprox(tiltedNormal));
  EXPECT_TRUE(tiltedBox.translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 5.0)
      - 0.5 * kGazeboPlaneBoxSize * tiltedNormal));
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, RebuildsPlanesInAWorldThatAlreadyCollides)
{
  // The SDF parser hands contact_benchmark a world whose ODE detector already
  // has collision objects for the planes; replacing their shapes must not
  // leave the detector with stale objects.
  auto world = simulation::World::create();
  world->getConstraintSolver()->setCollisionDetector(
      collision::OdeCollisionDetector::create());
  addGroundPlane(*world);
  auto box = addFreeBody(
      *world,
      "box",
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      translation(0.0, 0.0, 0.4999));
  world->step();
  ASSERT_GT(world->getLastCollisionResult().getNumContacts(), 0u);

  ASSERT_TRUE(rebuildPlanesLikeGazebo(*world).has_value());
  for (int i = 0; i < 10; ++i)
    world->step();
  EXPECT_GT(world->getLastCollisionResult().getNumContacts(), 0u);
  EXPECT_NEAR(
      box->getBodyNode(0)->getWorldTransform().translation().z(), 0.5, 1e-2);
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, ComputesLowestPointOfRotatedPrimitives)
{
  auto world = simulation::World::create();
  // A cylinder lying on its side and rolled about its own axis still touches
  // the ground with its rim: the lowest point is one radius below the center.
  Eigen::Isometry3d lying = translation(0.0, 0.0, 1.0);
  lying.rotate(Eigen::AngleAxisd(
      0.5 * math::constantsd::pi(), Eigen::Vector3d::UnitX()));
  lying.rotate(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()));
  auto cylinder = addFreeBody(
      *world,
      "cylinder",
      std::make_shared<dynamics::CylinderShape>(0.5, 2.0),
      lying);
  EXPECT_NEAR(
      computeLowestPointZ(*cylinder->getBodyNode(0)->getShapeNode(0)),
      0.5,
      1e-12);

  Eigen::Isometry3d tilted = translation(0.0, 0.0, 1.0);
  tilted.rotate(Eigen::AngleAxisd(
      0.25 * math::constantsd::pi(), Eigen::Vector3d::UnitX()));
  auto box = addFreeBody(
      *world,
      "box",
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(1.0, 1.0, 2.0)),
      tilted);
  EXPECT_NEAR(
      computeLowestPointZ(*box->getBodyNode(0)->getShapeNode(0)),
      1.0 - (0.5 + 1.0) * std::sqrt(0.5),
      1e-12);

  Eigen::Isometry3d spun = translation(0.0, 0.0, 1.0);
  spun.rotate(
      Eigen::AngleAxisd(1.0, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()));
  auto sphere = addFreeBody(
      *world, "sphere", std::make_shared<dynamics::SphereShape>(0.5), spun);
  EXPECT_NEAR(
      computeLowestPointZ(*sphere->getBodyNode(0)->getShapeNode(0)),
      0.5,
      1e-12);
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, ReportsStarvedPairsAndSunkBodies)
{
  // Five resting boxes want 20 contacts. A cap of 3 leaves at least two pairs
  // without a contact, however the solver shares it out.
  auto world = createStarvedBoxWorld(5, 3);
  const auto demand = measureGazeboContactDemand(*world);
  world->step();
  EXPECT_EQ(world->getLastCollisionResult().getNumContacts(), 3u);

  EXPECT_EQ(demand.rawContacts, 20u);
  EXPECT_EQ(demand.contacts, 20u);
  EXPECT_EQ(demand.pairs, 5u);
  const auto starved
      = countStarvedPairs(demand, world->getLastCollisionResult());
  EXPECT_GE(starved, 2u);
  EXPECT_LT(starved, 5u);

  // Bodies without contacts fall through the ground box.
  EXPECT_EQ(countSunkSkeletons(*world, 0.0), 0u);
  for (int i = 0; i < 300; ++i)
    world->step();
  EXPECT_GE(countSunkSkeletons(*world, 0.0), starved);

  // With room for every contact nothing starves or sinks.
  auto covered = createStarvedBoxWorld(5, kGazeboMaxNumContacts);
  for (int i = 0; i < 300; ++i)
    covered->step();
  const auto coveredDemand = measureGazeboContactDemand(*covered);
  covered->step();
  EXPECT_EQ(
      countStarvedPairs(coveredDemand, covered->getLastCollisionResult()), 0u);
  EXPECT_EQ(countSunkSkeletons(*covered, 0.0), 0u);
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, MeasuresDemandBeforeAndAfterThePairLimit)
{
  // The global cap truncates the detector's output before gz-physics' per-pair
  // limit runs, so the demand that matters for starvation is the raw one.
  auto world = createStarvedBoxWorld(5, kGazeboMaxNumContacts, 2u);
  const auto demand = measureGazeboContactDemand(*world);
  EXPECT_EQ(demand.rawContacts, 20u);
  EXPECT_EQ(demand.contacts, 10u);
  EXPECT_EQ(demand.pairs, 5u);
  EXPECT_EQ(demand.awakePairs.size(), 5u);

  // The census works on clones: the world's detector keeps its limit.
  world->step();
  EXPECT_EQ(world->getLastCollisionResult().getNumContacts(), 10u);
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, JudgesStarvationAtTheStateTheStepDetected)
{
  // A box 0.1 mm above the ground, falling fast enough to touch it during the
  // step: the step's detection had no contact to give it, so it is not
  // starved even though it touches the ground afterwards.
  auto world = createStarvedBoxWorld(1, kGazeboMaxNumContacts);
  auto* body = world->getSkeleton("box0")->getBodyNode(0);
  dynamics::FreeJoint::setTransformOf(body, translation(0.0, 0.0, 0.5001));
  static_cast<dynamics::FreeJoint*>(body->getParentJoint())
      ->setLinearVelocity(Eigen::Vector3d(0.0, 0.0, -1.0));

  const auto demand = measureGazeboContactDemand(*world);
  world->step();
  EXPECT_EQ(demand.pairs, 0u);
  EXPECT_EQ(countStarvedPairs(demand, world->getLastCollisionResult()), 0u);
  EXPECT_EQ(measureGazeboContactDemand(*world).pairs, 1u);
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, FindsBitmasksThatCanFilterAPair)
{
  const auto find = [](const std::string& contact) {
    const std::string sdf
        = "<sdf><world><model><link><collision/><collision><surface><contact>"
          + contact + "</contact></surface></collision></link></model></world>"
          + "</sdf>";
    tinyxml2::XMLDocument document;
    EXPECT_EQ(document.Parse(sdf.c_str()), tinyxml2::XML_SUCCESS);
    return findGazeboFilteringBitmask(document);
  };

  // gz-physics' defaults, and masks that keep all their bits, filter nothing.
  EXPECT_FALSE(find(""));
  EXPECT_FALSE(find("<collide_bitmask>0xff</collide_bitmask>"));
  EXPECT_FALSE(
      find("<collide_bitmask>65535</collide_bitmask>"
           "<category_bitmask> 0x1FF </category_bitmask>"));

  EXPECT_EQ(
      find("<collide_bitmask>0x01</collide_bitmask>"), "collide_bitmask 0x01");
  EXPECT_EQ(
      find("<collide_bitmask>0xff</collide_bitmask>"
           "<category_bitmask>254</category_bitmask>"),
      "category_bitmask 254");
  EXPECT_EQ(
      find("<collide_bitmask>all</collide_bitmask>"), "collide_bitmask all");

  // gz-physics reads masks above INT_MAX as 0; sdformat stores -1 as
  // 0xffffffff.
  EXPECT_FALSE(find("<collide_bitmask>2147483647</collide_bitmask>"));
  EXPECT_EQ(
      find("<collide_bitmask>0xffffffff</collide_bitmask>"),
      "collide_bitmask 0xffffffff");
  EXPECT_EQ(
      find("<category_bitmask>-1</category_bitmask>"), "category_bitmask -1");
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, FindsModelsTheSdfParserSkips)
{
  const auto find = [](const std::string& world) {
    const std::string sdf = "<sdf><world>" + world + "</world></sdf>";
    tinyxml2::XMLDocument document;
    EXPECT_EQ(document.Parse(sdf.c_str()), tinyxml2::XML_SUCCESS);
    return findSdfSkippedModel(document);
  };

  EXPECT_FALSE(
      find("<model><link><collision/></link><joint/></model><model/>"));
  // sdformat does not build models from <population> either.
  EXPECT_FALSE(find("<population><model name=\"box\"/></population>"));
  EXPECT_EQ(
      find("<model/><include><uri>model://ground_plane</uri></include>"),
      "<include> model://ground_plane");
  // A model included into a model.
  EXPECT_EQ(
      find("<model><include><uri>model://arm</uri></include></model>"),
      "<include> model://arm");
  EXPECT_EQ(find("<include/>"), "<include> ");
  EXPECT_EQ(
      find("<model name=\"outer\"><link/><model name=\"inner\"><link/></model>"
           "</model>"),
      "nested <model> inner");
  EXPECT_EQ(
      find("<model name=\"a\"><link name=\"link\"/></model>"
           "<model name=\"b\"><link name=\"link\"/></model>"
           "<joint name=\"weld\" type=\"fixed\"><parent>a::link</parent>"
           "<child>b::link</child></joint>"),
      "world <joint> weld");
  EXPECT_EQ(find("<joint/>"), "world <joint> ");
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, FindsUnsupportedSdfFrameSemantics)
{
  const auto find = [](const std::string& world) {
    const std::string sdf
        = "<sdf version=\"1.7\"><world>" + world + "</world></sdf>";
    tinyxml2::XMLDocument document;
    EXPECT_EQ(document.Parse(sdf.c_str()), tinyxml2::XML_SUCCESS);
    return findSdfSkippedModel(document);
  };

  EXPECT_EQ(find("<frame name=\"offset\"/>"), "<frame> offset");
  EXPECT_EQ(find("<model><frame name=\"offset\"/></model>"), "<frame> offset");
  EXPECT_EQ(find("<frame/>"), "<frame> ");
  EXPECT_EQ(
      find("<model><pose relative_to=\"other\">1 0 0 0 0 0</pose></model>"),
      "<pose relative_to=\"other\">");
  EXPECT_EQ(
      find("<model><link><pose relative_to=\"other\">1 0 0 0 0 0</pose>"
           "</link></model>"),
      "<pose relative_to=\"other\">");
  EXPECT_EQ(
      find("<model><link><collision><pose relative_to=\"other\">1 0 0 0 0 0"
           "</pose></collision></link></model>"),
      "<pose relative_to=\"other\">");
  EXPECT_FALSE(
      find("<model><pose>1 0 0 0 0 0</pose><link><collision>"
           "<pose relative_to=\"\">0 0 0 0 0 0</pose>"
           "</collision></link></model>"));
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, ReadsActivePhysicsContactLimit)
{
  const auto limit = [](const std::string& physics) {
    const std::string sdf = "<sdf><world>" + physics + "</world></sdf>";
    tinyxml2::XMLDocument document;
    EXPECT_EQ(document.Parse(sdf.c_str()), tinyxml2::XML_SUCCESS);
    return sdfCollisionPairMaxContacts(document);
  };

  EXPECT_EQ(limit(""), 20u);
  EXPECT_EQ(limit("<physics/>"), 20u);
  EXPECT_EQ(limit("<physics><max_contacts>4</max_contacts></physics>"), 4u);
  EXPECT_EQ(limit("<physics><max_contacts>0</max_contacts></physics>"), 0u);
  EXPECT_EQ(
      limit("<physics><max_contacts>4</max_contacts></physics>"
            "<physics><max_contacts>8</max_contacts></physics>"),
      4u);
  EXPECT_EQ(
      limit("<physics><max_contacts>4</max_contacts></physics>"
            "<physics default=\"true\"><max_contacts>8</max_contacts></physics>"
            "<physics default=\"1\"><max_contacts>12</max_contacts></physics>"),
      4u);
  EXPECT_EQ(
      limit("<physics/>"
            "<physics default=\"1\"><max_contacts>8</max_contacts></physics>"),
      20u);
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, PublishesChangedPosesLikeGazebo)
{
  auto world = simulation::World::create();
  auto skeleton = addFreeBody(
      *world,
      "box",
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      translation(0.0, 0.0, 1.0));
  auto* body = skeleton->getBodyNode(0);

  ChangedPoseTracker tracker;
  // gz-physics publishes every pose the first time.
  EXPECT_EQ(tracker.update(*world), 1u);
  EXPECT_EQ(tracker.update(*world), 0u);

  dynamics::FreeJoint::setTransformOf(body, translation(0.0, 0.0, 1.0 + 6e-7));
  EXPECT_EQ(tracker.update(*world), 0u);

  // Drift accumulates against the last published pose, so a second small move
  // publishes the body.
  dynamics::FreeJoint::setTransformOf(
      body, translation(0.0, 0.0, 1.0 + 1.2e-6));
  EXPECT_EQ(tracker.update(*world), 1u);
  EXPECT_EQ(tracker.update(*world), 0u);
}
