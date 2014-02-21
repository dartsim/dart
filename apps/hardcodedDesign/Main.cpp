/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Can Erdogan
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
 * @author Can Erdogan
 * @date Feb 02, 2013
 * @brief This application shows the creation of a kinematic skeleton from scratch without
 * the use of a model file. Run the program without arguments and you can use the buttons
 * {1,2} to move the corresponding joints. The key '-' will make the joints move in the negative
 * direction.
 */

#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "apps/hardcodedDesign/MyWindow.h"

namespace dart {
namespace dynamics {
class Joint;
class PrismaticJoint;
class RevoluteJoint;
}
}

/// \brief Function headers
enum TypeOfDOF {
  DOF_X, DOF_Y, DOF_Z, DOF_ROLL, DOF_PITCH, DOF_YAW
};

/// \brief Add a DOF to a given joint
dart::dynamics::Joint* create1DOFJoint(double val, double min, double max,
                                       int type) {
  // Create the transformation based on the type
  dart::dynamics::Joint* newJoint = NULL;
  if (type == DOF_X)
    newJoint = new dart::dynamics::PrismaticJoint(
                 Eigen::Vector3d(1.0, 0.0, 0.0));
  else if (type == DOF_Y)
    newJoint = new dart::dynamics::PrismaticJoint(
                 Eigen::Vector3d(0.0, 1.0, 0.0));
  else if (type == DOF_Z)
    newJoint = new dart::dynamics::PrismaticJoint(
                 Eigen::Vector3d(0.0, 0.0, 1.0));
  else if (type == DOF_YAW)
    newJoint = new dart::dynamics::RevoluteJoint(
                 Eigen::Vector3d(0.0, 0.0, 1.0));
  else if (type == DOF_PITCH)
    newJoint = new dart::dynamics::RevoluteJoint(
                 Eigen::Vector3d(0.0, 1.0, 0.0));
  else if (type == DOF_ROLL)
    newJoint = new dart::dynamics::RevoluteJoint(
                 Eigen::Vector3d(1.0, 0.0, 0.0));
  // Add the transformation to the joint, set the min/max values and set it to
  // the skeleton
  newJoint->getGenCoord(0)->set_q(val);
  newJoint->getGenCoord(0)->set_qMin(min);
  newJoint->getGenCoord(0)->set_qMax(max);

  return newJoint;
}

int main(int argc, char* argv[]) {
  // Create Left Leg skeleton
  dart::dynamics::Skeleton LeftLegSkel;

  // Pointers to be used during the Skeleton building
  Eigen::Matrix3d inertiaMatrix;
  inertiaMatrix << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  double mass = 1.0;

  // ***** BodyNode 1: Left Hip Yaw (LHY) ***** *
  dart::dynamics::BodyNode* node = new dart::dynamics::BodyNode("LHY");
  dart::dynamics::Joint* joint = create1DOFJoint(0.0, 0.0, DART_PI, DOF_YAW);
  joint->setName("LHY");
  dart::dynamics::Shape* shape =
      new dart::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0));
  node->addVisualizationShape(shape);
  node->addCollisionShape(shape);
  node->setMass(mass);
  node->setParentJoint(joint);
  LeftLegSkel.addBodyNode(node);

  // ***** BodyNode 2: Left Hip Roll (LHR) whose parent is: LHY *****\

  dart::dynamics::BodyNode* parent_node = LeftLegSkel.getBodyNode("LHY");
  node = new dart::dynamics::BodyNode("LHR");
  joint = create1DOFJoint(0.0, 0.0, DART_PI, DOF_ROLL);
  joint->setName("LHR");
  Eigen::Isometry3d T(Eigen::Translation3d(0.0, 0.0, 0.5));
  joint->setTransformFromParentBodyNode(T);
  shape = new dart::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0));
  shape->setOffset(Eigen::Vector3d(0.0, 0.0, 0.5));
  node->setLocalCOM(shape->getOffset());
  node->setMass(mass);
  node->addVisualizationShape(shape);
  node->addCollisionShape(shape);
  node->setParentJoint(joint);
  parent_node->addChildBodyNode(node);
  LeftLegSkel.addBodyNode(node);

  // ***** BodyNode 3: Left Hip Pitch (LHP) whose parent is: LHR *****
  parent_node = LeftLegSkel.getBodyNode("LHR");
  node = new dart::dynamics::BodyNode("LHP");
  joint = create1DOFJoint(0.0, 0.0, DART_PI, DOF_ROLL);
  joint->setName("LHP");
  T = Eigen::Translation3d(0.0, 0.0, 1.0);
  joint->setTransformFromParentBodyNode(T);
  shape = new dart::dynamics::BoxShape(Eigen::Vector3d(0.3, 0.3, 1.0));
  shape->setOffset(Eigen::Vector3d(0.0, 0.0, 0.5));
  node->setLocalCOM(shape->getOffset());
  node->setMass(mass);
  dart::dynamics::Shape* shape1 =
      new dart::dynamics::EllipsoidShape(Eigen::Vector3d(0.3, 0.3, 1.0));
  shape1->setOffset(Eigen::Vector3d(0.0, 0.0, 0.5));
  node->addVisualizationShape(shape1);
  node->addCollisionShape(shape);
  node->setParentJoint(joint);
  parent_node->addChildBodyNode(node);
  LeftLegSkel.addBodyNode(node);

  // Initialize the skeleton
  LeftLegSkel.init();

  // Window stuff
  MyWindow window(&LeftLegSkel);
  glutInit(&argc, argv);
  window.initWindow(640, 480, "Skeleton example");
  glutMainLoop();

  return 0;
}
