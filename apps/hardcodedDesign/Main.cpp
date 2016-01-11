/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Can Erdogan
 *            Michael X. Grey
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#include "dart/dart.h"

#include "apps/hardcodedDesign/MyWindow.h"

int main(int argc, char* argv[]) {
  // Create Left Leg skeleton
  kido::dynamics::SkeletonPtr LeftLegSkel = kido::dynamics::Skeleton::create();

  double mass = 1.0;

  // BodyNode 1: Left Hip Yaw (LHY)
  kido::dynamics::BodyNode::Properties body;
  body.mName = "LHY";
  kido::dynamics::ShapePtr shape(
        new kido::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));
  body.mVizShapes.push_back(shape); // Use 'shape' for visualizing
  body.mColShapes.push_back(shape); // Use 'shape' for collision detection
  body.mInertia.setMass(mass);

  kido::dynamics::RevoluteJoint::Properties joint;
  joint.mName = "LHY";
  joint.mAxis = Eigen::Vector3d(0.0, 0.0, 1.0);
  joint.mPositionLowerLimit = -DART_PI;
  joint.mPositionUpperLimit =  DART_PI;

  // You can get the newly created Joint and BodyNode pointers like this
  std::pair<kido::dynamics::Joint*, kido::dynamics::BodyNode*> pair =
      LeftLegSkel->createJointAndBodyNodePair<kido::dynamics::RevoluteJoint>(
        nullptr, joint, body);
  kido::dynamics::BodyNode* parent = pair.second;


  // BodyNode 2: Left Hip Roll (LHR) whose parent is: LHY
  body = kido::dynamics::BodyNode::Properties(); // create a fresh properties container
  body.mName = "LHR";
  shape = kido::dynamics::ShapePtr(
        new kido::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));
  shape->setOffset(Eigen::Vector3d(0.0, 0.0, 0.5));
  body.mVizShapes.push_back(shape);
  body.mColShapes.push_back(shape);
  body.mInertia.setLocalCOM(shape->getOffset());
  body.mInertia.setMass(mass);

  joint.mName = "LHR";
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 0.5);

  // You can get the specific type of Joint Pointer instead of just a basic Joint pointer
  std::pair<kido::dynamics::RevoluteJoint*, kido::dynamics::BodyNode*> nextPair =
  LeftLegSkel->createJointAndBodyNodePair<kido::dynamics::RevoluteJoint>(
        parent, joint, body);
  nextPair.first->setAxis(Eigen::Vector3d(1.0, 0.0, 0.0));


  // BodyNode 3: Left Hip Pitch (LHP) whose parent is: LHR
  body = kido::dynamics::BodyNode::Properties(); // create a fresh properties container
  body.mName = "LHP";
  shape = kido::dynamics::ShapePtr(
        new kido::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0)));
  shape->setOffset(Eigen::Vector3d(0.0, 0.0, 0.5));
  body.mVizShapes.push_back(shape);
  body.mColShapes.push_back(shape);
  body.mInertia.setLocalCOM(shape->getOffset());
  body.mInertia.setMass(mass);

  joint.mName = "LHP";
  joint.mAxis = Eigen::Vector3d(0.0, 1.0, 0.0);
  joint.mT_ParentBodyToJoint = Eigen::Translation3d(0.0, 0.0, 1.0);

  // Or you can completely ignore the return value of this function
  LeftLegSkel->createJointAndBodyNodePair<kido::dynamics::RevoluteJoint>(
        LeftLegSkel->getBodyNode(1), joint, body);


  // Window stuff
  MyWindow window(LeftLegSkel);
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Skeleton example");
  glutMainLoop();

  return 0;
}
