/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include "Helpers.hpp"

#include <dart/dart.hpp>

#include <sstream>

using namespace dart;
using namespace dart::math;
using namespace dart::dynamics;

//==============================================================================
SkeletonPtr createGround()
{
  // Create a Skeleton to represent the ground
  SkeletonPtr ground = Skeleton::create("ground");
  Isometry3d tf(Isometry3d::Identity());
  double thickness = 0.01;
  tf.translation() = Vector3d(0, 0, -thickness / 2.0);
  WeldJoint::Properties joint;
  joint.mT_ParentBodyToJoint = tf;
  ground->createJointAndBodyNodePair<WeldJoint>(nullptr, joint);
  ShapePtr groundShape
      = std::make_shared<BoxShape>(Vector3d(10, 10, thickness));

  auto shapeNode = ground->getBodyNode(0)
                       ->createShapeNodeWith<
                           VisualAspect,
                           CollisionAspect,
                           DynamicsAspect>(groundShape);
  shapeNode->getVisualAspect()->setColor(Colord::Blue(0.2));

  return ground;
}

//==============================================================================
SkeletonPtr createWam()
{
  dart::io::DartLoader urdfParser;
  urdfParser.addPackageDirectory(
      "herb_description", DART_DATA_LOCAL_PATH "/urdf/wam");
  SkeletonPtr wam
      = urdfParser.parseSkeleton(DART_DATA_LOCAL_PATH "/urdf/wam/wam.urdf");

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
  Vector3d linearBounds = Vector3d::Constant(inf<double>());

  Vector3d angularBounds = Vector3d::Constant(inf<double>());

  Isometry3d tf_hand(Isometry3d::Identity());
  tf_hand.translate(Vector3d(0.0, 0.0, -0.09));

  EndEffector* ee = wam->getBodyNode("/wam7")->createEndEffector("ee");
  ee->setDefaultRelativeTransform(tf_hand, true);

  auto wam7_target = std::make_shared<dart::gui::osg::InteractiveFrame>(
      Frame::World(), "lh_target");

  ee->getIK(true)->setTarget(wam7_target);

  std::stringstream ss;
  ss << DART_SHARED_LIB_PREFIX << "wamIk";
#if (DART_OS_LINUX || DART_OS_MACOS) && !NDEBUG
  ss << "d";
#endif
  ss << "." << DART_SHARED_LIB_EXTENSION;
  std::string libName = ss.str();

  std::vector<std::size_t> ikFastDofs{0, 1, 3, 4, 5, 6};
  std::vector<std::size_t> ikFastFreeDofs{2};
  ee->getIK()->setGradientMethod<SharedLibraryIkFast>(
      libName, ikFastDofs, ikFastFreeDofs);

  ee->getIK()->getErrorMethod().setLinearBounds(-linearBounds, linearBounds);
  ee->getIK()->getErrorMethod().setAngularBounds(-angularBounds, angularBounds);
}

//==============================================================================
void enableDragAndDrops(dart::gui::osg::Viewer& viewer, const SkeletonPtr& wam)
{
  // Turn on drag-and-drop for the whole Skeleton
  for (std::size_t i = 0; i < wam->getNumBodyNodes(); ++i)
    viewer.enableDragAndDrop(wam->getBodyNode(i), false, false);

  for (std::size_t i = 0; i < wam->getNumEndEffectors(); ++i) {
    EndEffector* ee = wam->getEndEffector(i);
    if (!ee->getIK())
      continue;

    // Check whether the target is an interactive frame, and add it if it is
    const auto& frame
        = std::dynamic_pointer_cast<dart::gui::osg::InteractiveFrame>(
            ee->getIK()->getTarget());

    if (frame)
      viewer.enableDragAndDrop(frame.get());
  }
}
