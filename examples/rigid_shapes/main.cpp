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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
 *
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

#include "dart/common/Macros.hpp"

#include <dart/gui/osg/All.hpp>

#include <dart/utils/All.hpp>

#include <dart/All.hpp>

#include <fcl/config.h>

#include <iostream>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::gui;
using namespace dart::utils;
using namespace dart::math;

class RigidShapesEventHandler : public ::osgGA::GUIEventHandler
{
public:
  RigidShapesEventHandler(const WorldPtr& world) : mWorld(world) {}

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == ::osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case 'q':
        case 'Q':
          spawnBox(
              getRandomTransform(),
              Random::uniform<Eigen::Vector3d>(0.05, 0.25));
          return true;
        case 'w':
        case 'W':
          spawnEllipsoid(
              getRandomTransform(),
              Random::uniform<Eigen::Vector3d>(0.025, 0.125));
          return true;
        case 'e':
        case 'E': {
          const double radius = Random::uniform(0.05, 0.25);
          const double height = Random::uniform(0.1, 0.5);
          spawnCylinder(getRandomTransform(), radius, height);
          return true;
        }
        case 'a':
        case 'A':
          if (mWorld->getNumSkeletons() > 1) {
            mWorld->removeSkeleton(
                mWorld->getSkeleton(mWorld->getNumSkeletons() - 1));
          }
          return true;
        default:
          return false;
      }
    }
    return false;
  }

private:
  Eigen::Isometry3d getRandomTransform()
  {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

    const Eigen::Vector3d rotation = Random::uniform<Eigen::Vector3d>(-pi, pi);
    const Eigen::Vector3d position = Eigen::Vector3d(
        Random::uniform(-1.0, 1.0),
        Random::uniform(0.5, 1.0),
        Random::uniform(-1.0, 1.0));

    T.translation() = position;
    T.linear() = expMapRot(rotation);

    return T;
  }

  void spawnBox(
      const Eigen::Isometry3d& _T,
      const Eigen::Vector3d& _size,
      double _mass = 10)
  {
    SkeletonPtr newSkeleton = Skeleton::create();

    ShapePtr newShape(new BoxShape(_size));

    BodyNode::Properties bodyProp;
    bodyProp.mName = "box_link";
    bodyProp.mInertia.setMass(_mass);

    FreeJoint::Properties jointProp;
    jointProp.mName = "box_joint";
    jointProp.mT_ParentBodyToJoint = _T;

    auto pair = newSkeleton->createJointAndBodyNodePair<FreeJoint>(
        nullptr, jointProp, bodyProp);
    auto shapeNode = pair.second->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(newShape);
    shapeNode->getVisualAspect()->setColor(
        Random::uniform<Eigen::Vector3d>(0.0, 1.0));

    mWorld->addSkeleton(newSkeleton);
  }

  void spawnEllipsoid(
      const Eigen::Isometry3d& _T,
      const Eigen::Vector3d& _radii,
      double _mass = 10)
  {
    SkeletonPtr newSkeleton = Skeleton::create();

    ShapePtr newShape(new EllipsoidShape(_radii * 2.0));

    BodyNode::Properties bodyProp;
    bodyProp.mName = "ellipsoid_link";
    bodyProp.mInertia.setMass(_mass);

    FreeJoint::Properties jointProp;
    jointProp.mName = "ellipsoid_joint";
    jointProp.mT_ParentBodyToJoint = _T;

    auto pair = newSkeleton->createJointAndBodyNodePair<FreeJoint>(
        nullptr, jointProp, bodyProp);
    auto shapeNode = pair.second->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(newShape);
    shapeNode->getVisualAspect()->setColor(
        Random::uniform<Eigen::Vector3d>(0.0, 1.0));

    mWorld->addSkeleton(newSkeleton);
  }

  void spawnCylinder(
      const Eigen::Isometry3d& _T,
      double _radius,
      double _height,
      double _mass = 10)
  {
    SkeletonPtr newSkeleton = Skeleton::create();

    ShapePtr newShape(new CylinderShape(_radius, _height));

    BodyNode::Properties bodyProp;
    bodyProp.mName = "cylinder_link";
    bodyProp.mInertia.setMass(_mass);

    FreeJoint::Properties jointProp;
    jointProp.mName = "cylinder_joint";
    jointProp.mT_ParentBodyToJoint = _T;

    auto pair = newSkeleton->createJointAndBodyNodePair<FreeJoint>(
        nullptr, jointProp, bodyProp);
    auto shapeNode = pair.second->createShapeNodeWith<
        VisualAspect,
        CollisionAspect,
        DynamicsAspect>(newShape);
    shapeNode->getVisualAspect()->setColor(
        Random::uniform<Eigen::Vector3d>(0.0, 1.0));

    mWorld->addSkeleton(newSkeleton);
  }

protected:
  WorldPtr mWorld;
};

class CustomWorldNode : public RealTimeWorldNode
{
public:
  CustomWorldNode(const WorldPtr& world) : RealTimeWorldNode(world) {}

  void customPreStep() override {}
};

int main()
{
  WorldPtr myWorld = SkelParser::readWorld("dart://sample/skel/shapes.skel");
  DART_ASSERT(myWorld != nullptr);

  auto handler = new RigidShapesEventHandler(myWorld);

  ::osg::ref_ptr<CustomWorldNode> node = new CustomWorldNode(myWorld);

  auto viewer = Viewer();
  viewer.addWorldNode(node);
  viewer.addEventHandler(handler);

  viewer.addInstructionText("space bar: simulation on/off\n");
  viewer.addInstructionText("'q': spawn a random cube\n");
  viewer.addInstructionText("'w': spawn a random ellipsoid\n");
  viewer.addInstructionText("'e': spawn a random cylinder\n");
  viewer.addInstructionText("'a': delete a spawned object at last\n");
  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.0f, 2.0f, 2.0f),
      ::osg::Vec3(0.0f, 0.0f, 0.0f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));

  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();

  return 0;
}
