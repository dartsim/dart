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

#include <dart/simulation/experimental/body/rigid_body.hpp>
#include <dart/simulation/experimental/comps/joint.hpp>
#include <dart/simulation/experimental/multi_body/multi_body.hpp>
#include <dart/simulation/experimental/world.hpp>

#include <iostream>

namespace sim_exp = dart::simulation::experimental;

int main()
{
  std::cout << "=== Simulation Experimental Hello World ===\n\n";

  sim_exp::World world;

  // Create a simple robot arm (3-link serial chain)
  auto robot = world.addMultiBody("robot_arm");

  auto base = robot.addLink("base");
  std::cout << "Created base link: " << base.getName() << "\n";

  auto link1 = robot.addLink(
      "link1",
      {.parentLink = base,
       .jointName = "shoulder",
       .jointType = sim_exp::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitZ()});

  auto link2 = robot.addLink(
      "link2",
      {.parentLink = link1,
       .jointName = "elbow",
       .jointType = sim_exp::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitY()});

  auto link3 = robot.addLink(
      "end_effector",
      {.parentLink = link2,
       .jointName = "wrist",
       .jointType = sim_exp::comps::JointType::Revolute,
       .axis = Eigen::Vector3d::UnitX()});

  std::cout << "\nRobot structure:\n";
  std::cout << "  Links: " << robot.getLinkCount() << "\n";
  std::cout << "  Joints: " << robot.getJointCount() << "\n";
  std::cout << "  DOFs: " << robot.getDOFCount() << "\n";

  // Set joint positions
  auto shoulder = robot.getJoint("shoulder");
  auto elbow = robot.getJoint("elbow");
  auto wrist = robot.getJoint("wrist");

  if (shoulder && elbow && wrist) {
    shoulder->setPosition(Eigen::VectorXd::Constant(1, 0.5));
    elbow->setPosition(Eigen::VectorXd::Constant(1, -0.3));
    wrist->setPosition(Eigen::VectorXd::Constant(1, 0.1));

    std::cout << "\nJoint positions:\n";
    std::cout << "  shoulder: " << shoulder->getPosition().transpose() << "\n";
    std::cout << "  elbow: " << elbow->getPosition().transpose() << "\n";
    std::cout << "  wrist: " << wrist->getPosition().transpose() << "\n";
  }

  // Create a rigid body
  auto box = world.addRigidBody(
      "falling_box",
      {.mass = 1.0,
       .inertia = Eigen::Matrix3d::Identity() * 0.1,
       .position = Eigen::Vector3d(0, 0, 2),
       .orientation = Eigen::Quaterniond::Identity()});

  std::cout << "\nRigid body:\n";
  std::cout << "  Name: " << box.getName() << "\n";
  std::cout << "  Mass: " << box.getMass() << " kg\n";
  std::cout << "  Position: " << box.getPosition().transpose() << "\n";

  // Apply a force
  box.addForce(Eigen::Vector3d(0, 0, -9.81));
  std::cout << "  Applied force: " << box.getForce().transpose() << " N\n";

  std::cout << "\nWorld summary:\n";
  std::cout << "  MultiBodies: " << world.getMultiBodyCount() << "\n";
  std::cout << "  RigidBodies: " << world.getRigidBodyCount() << "\n";

  std::cout << "\n=== Done ===\n";
  return 0;
}
