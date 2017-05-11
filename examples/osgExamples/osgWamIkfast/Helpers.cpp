/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "Helpers.hpp"

#include "RelaxedPosture.hpp"

//==============================================================================
SkeletonPtr createGround()
{
  // Create a Skeleton to represent the ground
  SkeletonPtr ground = Skeleton::create("ground");
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  double thickness = 0.01;
  tf.translation() = Eigen::Vector3d(0,0,-thickness/2.0);
  WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<WeldJoint>(nullptr, joint);
  ShapePtr groundShape =
      std::make_shared<BoxShape>(Eigen::Vector3d(10,10,thickness));

  auto shapeNode = ground->getBodyNode(0)->createShapeNodeWith<
      VisualAspect, CollisionAspect, DynamicsAspect>(groundShape);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue(0.2));

  return ground;
}

//==============================================================================
SkeletonPtr createWam()
{
  dart::utils::DartLoader urdfParser;
  urdfParser.addPackageDirectory("herb_description", DART_DATA_PATH"/urdf/wam");
  SkeletonPtr wam
      = urdfParser.parseSkeleton(DART_DATA_PATH"/urdf/wam/wam.urdf");

  return wam;
}

//==============================================================================
void setStartupConfiguration(const SkeletonPtr& wam)
{
  wam->getDof("/j1")->setPosition(0.0);
  wam->getDof("/j2")->setPosition(0.0);
  wam->getDof("/j3")->setPosition(0.0);
  wam->getDof("/j4")->setPosition(0.0);
  wam->getDof("/j5")->setPosition(0.0);
  wam->getDof("/j6")->setPosition(0.0);
  wam->getDof("/j7")->setPosition(0.0);
}

//==============================================================================
void setupEndEffectors(const SkeletonPtr& wam)
{
  Eigen::VectorXd rootjoint_weights = Eigen::VectorXd::Ones(7);
  rootjoint_weights = 0.01*rootjoint_weights;

  double extra_error_clamp = 0.1;

  Eigen::Vector3d linearBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Vector3d angularBounds =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());

  Eigen::Isometry3d tf_hand(Eigen::Isometry3d::Identity());
  tf_hand.translate(Eigen::Vector3d(0.0, 0.0, -0.09));

  EndEffector* ee = wam->getBodyNode("/wam7")->createEndEffector("ee");
  ee->setDefaultRelativeTransform(tf_hand, true);

  auto wam7_target = std::make_shared<dart::gui::osg::InteractiveFrame>(
        Frame::World(), "lh_target");

  ee->getIK(true)->setTarget(wam7_target);
  //  ee->getIK()->useWholeBody();

  ee->getIK()->setGradientMethod<dart::dynamics::ImportedIkfast>("libexample_wamIkd");
  ee->getIK()->getAnalytical()->setExtraDofUtilization(IK::Analytical::POST_ANALYTICAL);
  ee->getIK()->getAnalytical()->setExtraErrorLengthClamp(extra_error_clamp);
  //  ee->getIK()->getGradientMethod().setComponentWeights(rootjoint_weights);

  ee->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  ee->getIK()->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);

  linearBounds[2] = 1e-8;

  angularBounds[0] = 1e-8;
  angularBounds[1] = 1e-8;
}

//==============================================================================
void enableDragAndDrops(dart::gui::osg::Viewer& viewer, const SkeletonPtr& wam)
{
  // Turn on drag-and-drop for the whole Skeleton
  for (std::size_t i=0; i < wam->getNumBodyNodes(); ++i)
    viewer.enableDragAndDrop(wam->getBodyNode(i), false, false);

  for (std::size_t i=0; i < wam->getNumEndEffectors(); ++i)
  {
    EndEffector* ee = wam->getEndEffector(i);
    if (!ee->getIK())
      continue;

    // Check whether the target is an interactive frame, and add it if it is
    if (const auto& frame = std::dynamic_pointer_cast<dart::gui::osg::InteractiveFrame>(
          ee->getIK()->getTarget()))
      viewer.enableDragAndDrop(frame.get());
  }
}

//==============================================================================
void setupWholeBodySolver(const SkeletonPtr& wam)
{
  std::shared_ptr<dart::optimizer::GradientDescentSolver> solver =
      std::dynamic_pointer_cast<dart::optimizer::GradientDescentSolver>(
        wam->getIK(true)->getSolver());

  std::size_t nDofs = wam->getNumDofs();

  double default_weight = 0.01;
  Eigen::VectorXd weights = default_weight * Eigen::VectorXd::Ones(nDofs);
  weights[2] = 0.0;
  weights[3] = 0.0;
  weights[4] = 0.0;

  Eigen::VectorXd lower_posture = Eigen::VectorXd::Constant(nDofs,
                                                            -std::numeric_limits<double>::infinity());
  lower_posture[0] = -0.35;
  lower_posture[1] = -0.35;
  lower_posture[5] =  0.55;

  Eigen::VectorXd upper_posture = Eigen::VectorXd::Constant(nDofs,
                                                            std::numeric_limits<double>::infinity());
  upper_posture[0] =  0.35;
  upper_posture[1] =  0.50;
  upper_posture[5] =  0.95;

  std::shared_ptr<RelaxedPosture> objective = std::make_shared<RelaxedPosture>(
        wam->getPositions(), lower_posture, upper_posture, weights);

  wam->getIK()->setObjective(objective);

  std::shared_ptr<dart::constraint::BalanceConstraint> balance =
      std::make_shared<dart::constraint::BalanceConstraint>(wam->getIK());
  wam->getIK()->getProblem()->addEqConstraint(balance);

  balance->setErrorMethod(dart::constraint::BalanceConstraint::FROM_CENTROID);
  balance->setBalanceMethod(dart::constraint::BalanceConstraint::SHIFT_SUPPORT);

  solver->setNumMaxIterations(5);
}
