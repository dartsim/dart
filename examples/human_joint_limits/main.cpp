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

#include "HumanArmJointLimitConstraint.hpp"
#include "HumanLegJointLimitConstraint.hpp"

#include <dart/gui/osg/all.hpp>
#include <dart/utils/all.hpp>
#include <dart/collision/bullet/all.hpp>
#include <dart/collision/ode/ode.hpp>
#include <dart/all.hpp>

#include <cmath>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart;
using namespace utils;

class HumanJointLimitsWorldNode : public dart::gui::osg::WorldNode
{
public:
  explicit HumanJointLimitsWorldNode(const WorldPtr& world)
    : dart::gui::osg::WorldNode(world), ts(0)
  {
  }

  void customPreRefresh() override
  {
    if (ts == 0) {
      auto skel = mWorld->getSkeleton("human");

      auto shldJointl = skel->getJoint("j_bicep_left");
      auto elbowJointl = skel->getJoint("j_forearm_left");
      constraint_larm = std::make_shared<HumanArmJointLimitConstraint>(
          shldJointl, elbowJointl, false);
      mWorld->getConstraintSolver()->addConstraint(constraint_larm);

      auto shldJointr = skel->getJoint("j_bicep_right");
      auto elbowJointr = skel->getJoint("j_forearm_right");
      constraint_rarm = std::make_shared<HumanArmJointLimitConstraint>(
          shldJointr, elbowJointr, true);
      mWorld->getConstraintSolver()->addConstraint(constraint_rarm);

      auto thighJointl = skel->getJoint("j_thigh_left");
      auto shinJointl = skel->getJoint("j_shin_left");
      auto ankleJointl = skel->getJoint("j_heel_left");
      constraint_lleg = std::make_shared<HumanLegJointLimitConstraint>(
          thighJointl, shinJointl, ankleJointl, false);
      mWorld->getConstraintSolver()->addConstraint(constraint_lleg);

      auto thighJointr = skel->getJoint("j_thigh_right");
      auto shinJointr = skel->getJoint("j_shin_right");
      auto ankleJointr = skel->getJoint("j_heel_right");
      constraint_rleg = std::make_shared<HumanLegJointLimitConstraint>(
          thighJointr, shinJointr, ankleJointr, true);
      mWorld->getConstraintSolver()->addConstraint(constraint_rleg);
    }

    // Uncomment to apply external force
    // Eigen::Vector3d force = Eigen::Vector3d(0,0,-4);
    // Eigen::Vector3d location(0.0, -0.2, 0.0);
    // mWorld->getSkeleton("human")->getBodyNode("l-lowerarm")->addExtForce(force, location, true, true);

    mWorld->step();
    ts++;
  }

private:
  int ts;
  HumanArmJointLimitConstraintPtr constraint_larm;
  HumanArmJointLimitConstraintPtr constraint_rarm;
  HumanLegJointLimitConstraintPtr constraint_lleg;
  HumanLegJointLimitConstraintPtr constraint_rleg;
};

class HumanJointLimitsEventHandler : public ::osgGA::GUIEventHandler
{
public:
  HumanJointLimitsEventHandler()
  {
  }

  virtual bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (::osgGA::GUIEventAdapter::KEYDOWN == ea.getEventType()) {
      // Add any specific key handling here if needed
      // For now, we'll let the default behavior handle space bar for simulation toggle
    }
    return false;
  }
};

int main()
{
  WorldPtr world = SkelParser::readWorld(
      DART_DATA_PATH "/skel/kima/kima_human_edited.skel");
  assert(world != nullptr);

  auto skel = world->getSkeleton("human");
  for (auto joint : skel->getJoints()) {
    joint->setLimitEnforcement(true);
  }

  // Create the world node
  ::osg::ref_ptr<HumanJointLimitsWorldNode> node
      = new HumanJointLimitsWorldNode(world);

  // Create the viewer
  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);

  // Add event handler
  viewer.addEventHandler(new HumanJointLimitsEventHandler());

  // Print instructions
  std::cout << viewer.getInstructions() << std::endl;
  std::cout << "space bar: simulation on/off" << std::endl;

  // Set up window
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Run the viewer
  viewer.run();

  return 0;
}
