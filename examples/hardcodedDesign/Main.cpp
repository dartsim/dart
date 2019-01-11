/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

/**
 * @file Main.cpp
 * @author Can Erdogan (rewritten by Michael X. Grey)
 * @date Feb 02, 2013  (rewritten April 09, 2015)
 * @brief This application shows the creation of a kinematic skeleton from scratch without
 * the use of a model file. Run the program without arguments and you can use the buttons
 * {1,2} to move the corresponding joints. The key '-' will make the joints move in the negative
 * direction.
 */

#include <dart/dart.hpp>

#include "MyWindow.hpp"

int main(int argc, char* argv[]) {
  // Create Left Leg skeleton
  dart::dynamics::SkeletonPtr LeftLegSkel = dart::dynamics::Skeleton::create();

  double mass = 1.0;

  // BodyNode 1: Left Hip Yaw (LHY)
  dart::dynamics::BodyNode::Properties body;
  body.mName = "LHY";
  dart::dynamics::ShapePtr shape(
        new dart::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));
  body.mInertia.setMass(mass);

  dart::dynamics::RevoluteJoint::Properties joint;
  joint.mName = "LHY";
  joint.mAxis = Eigen::Vector3d(0.0, 0.0, 1.0);
  joint.mPositionLowerLimits[0] = -dart::math::constantsd::pi();
  joint.mPositionUpperLimits[0] =  dart::math::constantsd::pi();

  // You can get the newly created Joint and BodyNode pointers like this
  std::pair<dart::dynamics::Joint*, dart::dynamics::BodyNode*> pair =
      LeftLegSkel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
        nullptr, joint, body);
  pair.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  dart::dynamics::BodyNode* parent = pair.second;

  // BodyNode 2: Left Hip Roll (LHR) whose parent is: LHY
  body = dart::dynamics::BodyNode::Properties(); // create a fresh properties container
  body.mName = "LHR";
  shape = dart::dynamics::ShapePtr(
        new dart::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));

  joint.mName = "LHR";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 0.5);

  // You can get the specific type of Joint Pointer instead of just a basic Joint pointer
  std::pair<dart::dynamics::RevoluteJoint*, dart::dynamics::BodyNode*> pair1 =
  LeftLegSkel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
        parent, joint, body);
  pair1.first->setAxis(Eigen::Vector3d(1.0, 0.0, 0.0));
  auto shapeNode1 = pair1.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode1->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));
  pair1.second->setLocalCOM(shapeNode1->getRelativeTranslation());
  pair1.second->setMass(mass);

  // BodyNode 3: Left Hip Pitch (LHP) whose parent is: LHR
  body = dart::dynamics::BodyNode::Properties(); // create a fresh properties container
  body.mName = "LHP";
  shape = dart::dynamics::ShapePtr(
        new dart::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));

  joint.mName = "LHP";
  joint.mAxis = Eigen::Vector3d(0.0, 1.0, 0.0);
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 1.0);

  // Or you can completely ignore the return value of this function
  std::pair<dart::dynamics::RevoluteJoint*, dart::dynamics::BodyNode*> pair2 =
  LeftLegSkel->createJointAndBodyNodePair<dart::dynamics::RevoluteJoint>(
        LeftLegSkel->getBodyNode(1), joint, body);
  auto shapeNode2 = pair2.second->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(shape);
  shapeNode2->setRelativeTranslation(Eigen::Vector3d(0.0, 0.0, 0.5));
  pair2.second->setLocalCOM(shapeNode2->getRelativeTranslation());
  pair2.second->setMass(mass);

  // Window stuff
  MyWindow window(LeftLegSkel);
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Skeleton example");
  glutMainLoop();

  return 0;
}
