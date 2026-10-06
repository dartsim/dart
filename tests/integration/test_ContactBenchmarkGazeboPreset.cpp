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
#include <optional>
#include <string>
#include <string_view>
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

//==============================================================================
std::optional<std::string> findUnsupportedInWorld(
    std::string_view contents, std::string_view version = "1.6")
{
  const std::string sdf = "<sdf version=\"" + std::string(version)
                          + "\"><world name=\"test\">" + std::string(contents)
                          + "</world></sdf>";
  tinyxml2::XMLDocument document;
  EXPECT_EQ(document.Parse(sdf.c_str()), tinyxml2::XML_SUCCESS);
  return findUnsupportedGazeboPresetSdf(document);
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
TEST(ContactBenchmarkGazeboPreset, AcceptsSupportedGazeboPresetSdf)
{
  const std::string_view worlds[]
      = {"",
         "<model name=\"empty\"/>",
         R"(<gravity>0 0 -9.8</gravity>
         <physics name="1ms" type="ignored" default="true">
           <max_step_size>0.001</max_step_size>
           <real_time_factor>0</real_time_factor><max_contacts>4</max_contacts>
         </physics>
         <model name="primitives">
           <static>false</static><pose>1 2 3 0.1 0.2 0.3</pose>
           <link name="body">
             <pose>0 0 1 0.3 0.2 0.1</pose>
             <inertial><pose>0.1 0.2 0.3 0 0 0</pose><mass>2</mass>
               <inertia><ixx>1</ixx><iyy>2</iyy><izz>3</izz>
                 <ixy>0.1</ixy><ixz>0.2</ixz><iyz>0.3</iyz></inertia>
             </inertial>
             <collision name="box"><pose>1 0 0 0 0 0</pose>
               <geometry><box><size>1 2 3</size></box></geometry>
             </collision>
             <collision name="sphere">
               <geometry><sphere><radius>0.5</radius></sphere></geometry>
             </collision>
             <collision name="cylinder">
               <geometry><cylinder><radius>0.5</radius><length>2</length>
               </cylinder></geometry>
             </collision>
             <collision name="plane">
               <geometry><plane><normal>0 0 1</normal><size>10 20</size>
               </plane></geometry>
             </collision>
           </link>
         </model>)",
         R"(<model name="pendulum"><pose>0 500 2 0 0 0</pose>
           <link name="bob"><inertial><mass>1</mass><inertia>
             <ixx>0.004</ixx><iyy>0.004</iyy><izz>0.004</izz>
             <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
           </inertia></inertial><collision name="collision">
             <geometry><sphere><radius>0.1</radius></sphere></geometry>
           </collision></link>
           <joint name="pivot" type="revolute"><parent>world</parent>
             <child>bob</child><pose>-0.5 0 0 0 0 0</pose>
             <axis><xyz>0 1 0</xyz></axis>
           </joint>
         </model>)",
         R"(<scene><ambient>1 1 1 1</ambient></scene>
         <gui><camera name="view"><pose>1 2 3 0 0 0</pose></camera></gui>
         <light name="sun" type="directional"><direction>0 0 -1</direction>
         </light>
         <model name="decorated"><link name="body">
           <visual name="mesh"><geometry><mesh><uri>model://decorative</uri>
             <scale>1 2 3</scale>
           </mesh></geometry></visual>
           <sensor name="camera" type="camera"><camera>
             <horizontal_fov>1</horizontal_fov></camera></sensor>
         </link></model>)",
         R"(<plugin filename="gz-sim-physics-system"
                 name="gz::sim::systems::Physics"/>
         <plugin filename="gz-sim-user-commands-system"
                 name="gz::sim::systems::UserCommands"/>
         <plugin filename="gz-sim-scene-broadcaster-system"
                 name="gz::sim::systems::SceneBroadcaster"/>)",
         R"(<plugin filename="ignition-gazebo-physics-system"
                 name="ignition::gazebo::systems::Physics"/>)"};

  for (const auto world : worlds) {
    const auto unsupported = findUnsupportedInWorld(world);
    EXPECT_FALSE(unsupported.has_value()) << unsupported.value_or("") << world;
  }
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, AcceptsEquivalentGazeboPresetJoints)
{
  for (const std::string_view type :
       {"fixed", "revolute", "prismatic", "universal", "ball"}) {
    std::string joint = "<joint name=\"joint\" type=\"" + std::string(type)
                        + "\"><parent>base</parent><child>child</child>"
                          "<pose>0 0 1 0.1 0.2 0.3</pose>";
    if (type != "fixed" && type != "ball") {
      joint += "<axis><xyz expressed_in=\"\">0 0 1</xyz>"
               "<use_parent_model_frame>false</use_parent_model_frame>"
               "<dynamics><damping>0.1</damping><friction>0.2</friction>"
               "<spring_reference>0.3</spring_reference>"
               "<spring_stiffness>0.4</spring_stiffness></dynamics>"
               "<limit><lower>-1</lower><upper>1</upper>"
               "<effort>-1</effort><velocity>-1</velocity></limit></axis>";
    }
    if (type == "universal")
      joint += "<axis2><xyz>0 1 0</xyz></axis2>";
    joint += "</joint>";
    const auto unsupported = findUnsupportedInWorld(
        "<model name=\"arm\"><link name=\"base\"/><link name=\"child\"/>"
        + joint + "</model>");
    EXPECT_FALSE(unsupported.has_value()) << unsupported.value_or("") << type;
  }

  for (const std::string_view type : {"revolute", "prismatic"}) {
    const auto unsupported = findUnsupportedInWorld(
        "<model name=\"body\"><link name=\"link\"/>"
        "<joint name=\"joint\" type=\"" + std::string(type)
        + "\"><parent>world</parent><child>link</child>"
          "<axis><xyz>0 0 2</xyz></axis></joint></model>");
    EXPECT_FALSE(unsupported.has_value()) << unsupported.value_or("") << type;
  }
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, RejectsUnsupportedGazeboPresetElements)
{
  struct RejectedWorld
  {
    std::string_view contents;
    std::string_view path;
  };
  const RejectedWorld worlds[] = {
      {"<include><uri>model://ground_plane</uri></include>",
       "/sdf/world/include"},
      {"<model name=\"outer\"><include><uri>model://arm</uri></include>"
       "</model>",
       "/sdf/world/model/include"},
      {"<model name=\"outer\"><model name=\"inner\"/></model>",
       "/sdf/world/model/model"},
      {"<joint name=\"world_joint\" type=\"fixed\"/>", "/sdf/world/joint"},
      {"<frame name=\"offset\"/>", "/sdf/world/frame"},
      {"<model name=\"body\"><frame name=\"offset\"/></model>",
       "/sdf/world/model/frame"},
      {"<model name=\"body\"><link name=\"link\"><collision name=\"mesh\">"
       "<geometry><mesh><uri>file://collision.obj</uri></mesh></geometry>"
       "</collision></link></model>",
       "/sdf/world/model/link/collision/geometry/mesh"},
      {"<model name=\"body\"><link name=\"link\"><collision name=\"terrain\">"
       "<geometry><heightmap><uri>file://heightmap.png</uri></heightmap>"
       "</geometry></collision></link></model>",
       "/sdf/world/model/link/collision/geometry/heightmap"},
      {"<model name=\"body\"><link name=\"link\"><collision name=\"capsule\">"
       "<geometry><capsule><radius>1</radius><length>2</length></capsule>"
       "</geometry></collision></link></model>",
       "/sdf/world/model/link/collision/geometry/capsule"},
      {"<model name=\"body\"><link name=\"link\"><collision name=\"ellipsoid\">"
       "<geometry><ellipsoid><radii>1 2 3</radii></ellipsoid></geometry>"
       "</collision></link></model>",
       "/sdf/world/model/link/collision/geometry/ellipsoid"},
      {"<model name=\"body\"><link name=\"link\"><soft_shape/></link></model>",
       "/sdf/world/model/link/soft_shape"},
      {"<physics><unexpected/></physics>", "/sdf/world/physics/unexpected"},
      {"<model name=\"body\"><unexpected/></model>",
       "/sdf/world/model/unexpected"},
      {"<unexpected/>", "/sdf/world/unexpected"},
      {"<plugin filename=\"libapply_force.so\" name=\"ApplyForce\"/>",
       "/sdf/world/plugin"},
      {"<plugin filename=\"gz-sim-physics-system\" "
       "name=\"gz::sim::systems::Physics\"><engine>bullet</engine></plugin>",
       "/sdf/world/plugin/engine"},
      {"<model name=\"body\"><plugin filename=\"gz-sim-physics-system\" "
       "name=\"gz::sim::systems::Physics\"/></model>",
       "/sdf/world/model/plugin"},
      {"<model name=\"body\"><link name=\"link\"/>"
       "<joint name=\"joint\" type=\"revolute\"><parent>world</parent>"
       "<child>link</child></joint></model>",
       "/sdf/world/model/joint/axis"},
      {"<model name=\"body\"><link name=\"link\"/>"
       "<joint name=\"joint\" type=\"universal\"><parent>world</parent>"
       "<child>link</child><axis><xyz>0 0 1</xyz></axis></joint></model>",
       "/sdf/world/model/joint/axis2"},
      {"<scene><plugin name=\"Force\" filename=\"force\"/></scene>",
       "/sdf/world/scene/plugin"},
      {"<light name=\"sun\" type=\"directional\"><plugin name=\"Force\" "
       "filename=\"force\"/></light>",
       "/sdf/world/light/plugin"},
      {"<gui><plugin name=\"Force\" filename=\"force\"/></gui>",
       "/sdf/world/gui/plugin"},
      {"<model name=\"body\"><link name=\"link\"><visual name=\"visual\">"
       "<plugin name=\"Force\" filename=\"force\"/></visual></link></model>",
       "/sdf/world/model/link/visual/plugin"},
      {"<model name=\"body\"><link name=\"link\"><sensor name=\"sensor\" "
       "type=\"camera\"><camera><plugin name=\"Force\" filename=\"force\"/>"
       "</camera></sensor></link></model>",
       "/sdf/world/model/link/sensor/camera/plugin"},
      {"<model name=\"body\"><link name=\"link\"><visual name=\"visual\"/>"
       "</link></model>",
       "/sdf/world/model/link/visual/geometry"},
      {"<model name=\"body\"><link name=\"link\"><visual name=\"visual\">"
       "<pose>0 0 0 0 0 0 1</pose><geometry><sphere><radius>1</radius>"
       "</sphere></geometry></visual></link></model>",
       "/sdf/world/model/link/visual/pose"}};

  for (const auto& world : worlds) {
    const auto unsupported = findUnsupportedInWorld(world.contents);
    ASSERT_TRUE(unsupported.has_value()) << world.contents;
    EXPECT_NE(unsupported->find(world.path), std::string::npos) << *unsupported;
  }
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, RejectsUnsupportedGazeboPresetAttributes)
{
  struct RejectedWorld
  {
    std::string_view contents;
    std::string_view path;
  };
  const RejectedWorld worlds[]
      = {{"<model name=\"body\"><pose relative_to=\"offset\">0 0 0 0 0 0"
          "</pose></model>",
          "/sdf/world/model/pose/@relative_to"},
         {"<model name=\"body\"><link name=\"link\">"
          "<pose degrees=\"true\">0 0 0 0 0 90</pose></link></model>",
          "/sdf/world/model/link/pose/@degrees"},
         {"<model name=\"body\"><link name=\"link\"><collision name=\"shape\">"
          "<pose rotation_format=\"quat_xyzw\">0 0 0 0 0 0 1</pose>"
          "<geometry><sphere><radius>1</radius></sphere></geometry>"
          "</collision></link></model>",
          "/sdf/world/model/link/collision/pose/@rotation_format"},
         {"<model name=\"body\" unexpected=\"true\"/>",
          "/sdf/world/model/@unexpected"},
         {"<model name=\"body\"><link name=\"link\"><inertial auto=\"true\"/>"
          "</link></model>",
          "/sdf/world/model/link/inertial/@auto"},
         {"<model name=\"body\"><link name=\"link\"/>"
          "<joint name=\"joint\" type=\"screw\"><parent>world</parent>"
          "<child>link</child><axis><xyz>0 0 1</xyz></axis></joint></model>",
          "/sdf/world/model/joint/@type"},
         {"<model name=\"body\"><link name=\"link\"/>"
          "<joint name=\"joint\" type=\"revolute2\"><parent>world</parent>"
          "<child>link</child><axis><xyz>0 0 1</xyz></axis>"
          "<axis2><xyz>0 1 0</xyz></axis2></joint></model>",
          "/sdf/world/model/joint/@type"},
         {"<model name=\"body\"><link name=\"link\"/>"
          "<joint name=\"joint\" type=\"revolute\"><parent>world</parent>"
          "<child>link</child><axis><xyz expressed_in=\"model\">0 0 "
          "1</xyz></axis></joint></model>",
          "/sdf/world/model/joint/axis/xyz/@expressed_in"}};

  for (const auto& world : worlds) {
    const auto unsupported = findUnsupportedInWorld(world.contents);
    ASSERT_TRUE(unsupported.has_value()) << world.contents;
    EXPECT_NE(unsupported->find(world.path), std::string::npos) << *unsupported;
  }
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, RejectsNonEquivalentGazeboPresetValues)
{
  const auto rejectLink = [](std::string_view contents, std::string_view path) {
    const auto unsupported = findUnsupportedInWorld(
        "<model name=\"body\"><link name=\"link\">" + std::string(contents)
        + "</link></model>");
    ASSERT_TRUE(unsupported.has_value()) << contents;
    EXPECT_NE(unsupported->find(path), std::string::npos) << *unsupported;
  };

  rejectLink("<self_collide>true</self_collide>", "/link/self_collide");
  rejectLink("<inertial><mass>2</mass></inertial>", "/link/inertial");
  rejectLink(
      "<inertial><inertia><ixx>1</ixx></inertia></inertial>",
      "/link/inertial/inertia");
  rejectLink(
      "<inertial><pose>0 0 0 0 0 0.1</pose></inertial>", "/link/inertial/pose");

  struct RejectedSurface
  {
    std::string_view contents;
    std::string_view path;
  };
  const RejectedSurface surfaces[]
      = {{"<friction><ode><mu>0.1</mu></ode></friction>",
          "/surface/friction/ode/mu"},
         {"<friction><ode><mu2>0.2</mu2></ode></friction>",
          "/surface/friction/ode/mu2"},
         {"<friction><ode><slip1>0.3</slip1></ode></friction>",
          "/surface/friction/ode/slip1"},
         {"<friction><ode><slip2>0.4</slip2></ode></friction>",
          "/surface/friction/ode/slip2"},
         {"<friction><ode><fdir1>1 0 0</fdir1></ode></friction>",
          "/surface/friction/ode/fdir1"},
         {"<friction><ode><fdir1 frame=\"link\">0 0 0</fdir1></ode></friction>",
          "/surface/friction/ode/fdir1/@frame"},
         {"<bounce><restitution_coefficient>0.5</restitution_coefficient></"
          "bounce>",
          "/surface/bounce/restitution_coefficient"},
         {"<contact><collide_bitmask>0x01</collide_bitmask></contact>",
          "/surface/contact/collide_bitmask"},
         {"<contact><category_bitmask>254</category_bitmask></contact>",
          "/surface/contact/category_bitmask"},
         {"<contact><collide_bitmask>all</collide_bitmask></contact>",
          "/surface/contact/collide_bitmask"},
         {"<contact><collide_bitmask>0xffffffff</collide_bitmask></contact>",
          "/surface/contact/collide_bitmask"},
         {"<contact><category_bitmask>-1</category_bitmask></contact>",
          "/surface/contact/category_bitmask"},
         {"<contact><collide_bitmask>255junk</collide_bitmask></contact>",
          "/surface/contact/collide_bitmask"}};
  for (const auto& surface : surfaces) {
    rejectLink(
        "<collision name=\"shape\"><geometry><sphere><radius>1</radius>"
        "</sphere></geometry><surface>"
            + std::string(surface.contents) + "</surface></collision>",
        surface.path);
  }

  EXPECT_TRUE(findUnsupportedInWorld(
      "<model name=\"body\"><self_collide>true</self_collide></model>"));
  EXPECT_TRUE(findUnsupportedInWorld("<gravity>0 0 -9.81</gravity>"));
  EXPECT_TRUE(
      findUnsupportedInWorld("<physics><gravity>0 0 -9.8</gravity></physics>"));

  const std::string_view axes[]
      = {"<xyz>0 0 0</xyz>",
         "<xyz>0 0 1e-8</xyz>",
         "<xyz>0 0 1</xyz><limit><lower>1</lower><upper>2</upper></limit>",
         "<xyz>0 0 1</xyz><limit><effort>1</effort></limit>",
         "<xyz>0 0 1</xyz><limit><velocity>2</velocity></limit>"};
  for (const auto axis : axes) {
    const auto unsupported = findUnsupportedInWorld(
        "<model name=\"body\"><link name=\"link\"/>"
        "<joint name=\"joint\" type=\"revolute\"><parent>world</parent>"
        "<child>link</child><axis>"
        + std::string(axis) + "</axis></joint></model>");
    ASSERT_TRUE(unsupported.has_value()) << axis;
    EXPECT_NE(unsupported->find("/model/joint/axis/"), std::string::npos)
        << *unsupported;
  }
  const auto unsupported = findUnsupportedInWorld(
      "<model name=\"body\"><link name=\"link\"/>"
      "<joint name=\"joint\" type=\"universal\"><parent>world</parent>"
      "<child>link</child><axis><xyz>0 0 2</xyz></axis>"
      "<axis2><xyz>0 1 0</xyz></axis2></joint></model>");
  ASSERT_TRUE(unsupported.has_value());
  EXPECT_NE(unsupported->find("/model/joint/axis/xyz"), std::string::npos)
      << *unsupported;
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, RejectsMalformedGazeboPresetContactLimits)
{
  for (const std::string_view limit : {"4.5", "4junk"}) {
    const auto unsupported = findUnsupportedInWorld(
        "<physics><max_contacts>" + std::string(limit)
        + "</max_contacts></physics>");
    ASSERT_TRUE(unsupported.has_value()) << limit;
    EXPECT_NE(
        unsupported->find("/sdf/world/physics/max_contacts"), std::string::npos)
        << *unsupported;
  }
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, RejectsInvalidGazeboPresetJointTrees)
{
  struct RejectedJoints
  {
    std::string_view contents;
    std::string_view diagnostic;
  };
  const RejectedJoints joints[] = {
      {R"(<joint name="joint" type="fixed"><parent>missing</parent>
            <child>a</child></joint>)",
       "/model/joint/parent"},
      {R"(<joint name="joint" type="fixed"><parent>world</parent>
            <child>missing</child></joint>)",
       "/model/joint/child"},
      {R"(<joint name="joint" type="fixed"><parent>a</parent>
            <child>a</child></joint>)",
       "closes a joint chain"},
      {R"(<joint name="first" type="fixed"><parent>a</parent><child>b</child>
          </joint><joint name="second" type="fixed"><parent>b</parent>
            <child>a</child></joint>)",
       "closes a joint chain"},
      {R"(<joint name="first" type="fixed"><parent>world</parent><child>a</child>
          </joint><joint name="second" type="fixed"><parent>b</parent>
            <child>a</child></joint>)",
       "has multiple parent joints"}};

  for (const auto& joint : joints) {
    const auto unsupported = findUnsupportedInWorld(
        "<model name=\"body\"><link name=\"a\"/><link name=\"b\"/>"
        + std::string(joint.contents) + "</model>");
    ASSERT_TRUE(unsupported.has_value()) << joint.contents;
    EXPECT_NE(unsupported->find(joint.diagnostic), std::string::npos)
        << *unsupported;
    EXPECT_NE(unsupported->find("/sdf/world/model/joint/"), std::string::npos)
        << *unsupported;
  }
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, AcceptsDefaultPoseAndUnfilteredBitmasks)
{
  const std::string_view contacts[]
      = {"",
         "<collide_bitmask>0xff</collide_bitmask>",
         "<collide_bitmask>65535</collide_bitmask>"
         "<category_bitmask> 0x1FF </category_bitmask>",
         "<collide_bitmask>2147483647</collide_bitmask>"};
  for (const auto contact : contacts) {
    const auto unsupported = findUnsupportedInWorld(
        "<model name=\"body\"><self_collide>false</self_collide>"
        "<pose relative_to=\"\" degrees=\"false\" "
        "rotation_format=\"euler_rpy\">"
        "0 0 0 0 0 0</pose><link name=\"link\"><self_collide>0</self_collide>"
        "<collision name=\"shape\"><geometry><sphere><radius>1</radius>"
        "</sphere></geometry><surface><contact>"
        + std::string(contact)
        + "</contact></surface></collision></link></model>");
    EXPECT_FALSE(unsupported.has_value())
        << unsupported.value_or("") << contact;
  }
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, AcceptsDefaultSurfaceMaterial)
{
  const auto unsupported = findUnsupportedInWorld(
      R"(<model name="body"><link name="link"><collision name="shape">
           <geometry><sphere><radius>1</radius></sphere></geometry>
           <surface>
             <friction><ode><mu>1</mu><mu2>1</mu2><slip1>0</slip1>
               <slip2>0</slip2><fdir1>0 0 0</fdir1></ode></friction>
             <bounce><restitution_coefficient>0</restitution_coefficient>
             </bounce>
           </surface>
         </collision></link></model>)");
  EXPECT_FALSE(unsupported.has_value()) << unsupported.value_or("");
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, HandlesVersionedJointAxisSemantics)
{
  const std::string_view simple
      = R"(<model name="body"><link name="link"><collision name="shape">
             <geometry><sphere><radius>1</radius></sphere></geometry>
           </collision></link></model>)";
  for (const auto contents : {std::string_view{}, simple}) {
    const auto unsupported = findUnsupportedInWorld(contents, "1.4");
    EXPECT_FALSE(unsupported.has_value()) << unsupported.value_or("");
  }

  const std::string_view joint = R"(<model name="body"><link name="link"/>
             <joint name="joint" type="revolute"><parent>world</parent>
               <child>link</child><axis><xyz>0 0 1</xyz>
                 <use_parent_model_frame>true</use_parent_model_frame>
               </axis>
             </joint>
           </model>)";
  const auto unsupported = findUnsupportedInWorld(joint, "1.4");
  ASSERT_TRUE(unsupported.has_value());
  EXPECT_NE(
      unsupported->find("/sdf/world/model/joint/axis"), std::string::npos);
  for (const std::string_view version : {"1.5", "1.6"}) {
    const auto supported = findUnsupportedInWorld(joint, version);
    EXPECT_FALSE(supported.has_value()) << supported.value_or("");
  }
}

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, ReportsFirstUnsupportedGazeboPresetInput)
{
  const auto unsupported = findUnsupportedInWorld(
      "<include><uri>model://ground_plane</uri></include><frame "
      "name=\"later\"/>");
  ASSERT_TRUE(unsupported.has_value());
  EXPECT_NE(unsupported->find("/sdf/world/include"), std::string::npos);
  EXPECT_EQ(unsupported->find("/frame"), std::string::npos);
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
  world->setGravity(Eigen::Vector3d::Zero());
  addGroundPlane(*world);
  auto skeleton = addFreeBody(
      *world,
      "box",
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      translation(0.0, 0.0, 1.0));
  auto* body = skeleton->getBodyNode(0);

  ChangedPoseTracker tracker;
  // With no warmup, the first Write after stepping includes every body,
  // including the static ground and the body whose pose did not move.
  world->step();
  EXPECT_EQ(tracker.update(*world), 2u);
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

//==============================================================================
TEST(ContactBenchmarkGazeboPreset, KeepsPublishedPoseBaselineThroughWarmup)
{
  auto world = simulation::World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(0.001);
  auto skeleton = addFreeBody(
      *world,
      "box",
      std::make_shared<dynamics::BoxShape>(Eigen::Vector3d::Ones()),
      translation(0.0, 0.0, 1.0));
  static_cast<dynamics::FreeJoint*>(skeleton->getBodyNode(0)->getParentJoint())
      ->setLinearVelocity(Eigen::Vector3d(0.0, 0.0, 6e-4));

  ChangedPoseTracker tracker;
  // The first warmup step publishes; the second drifts by 6e-7 and keeps
  // the last published baseline. Priming only after warmup loses that drift.
  for (const std::size_t expected : {1u, 0u}) {
    world->step();
    EXPECT_EQ(tracker.update(*world), expected);
  }

  world->step();
  EXPECT_EQ(tracker.update(*world), 1u);
  world->step();
  EXPECT_EQ(tracker.update(*world), 0u);
}
