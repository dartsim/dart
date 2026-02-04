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

#include <dart/gui/all.hpp>

#include <dart/collision/ode/ode_collision_detector.hpp>

#include <dart/all.hpp>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;

namespace {

constexpr double kCapsuleRadius = 0.2;
constexpr double kCapsuleHeight = 0.6;

Eigen::Isometry3d makeHorizontalPose()
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(Eigen::Vector3d(0, 0, kCapsuleRadius + 0.1));
  pose.rotate(Eigen::AngleAxisd(dart::math::half_pi, Eigen::Vector3d::UnitY()));
  return pose;
}

Eigen::Isometry3d makeVerticalPose()
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(Eigen::Vector3d(0, 0, 0.5 * kCapsuleHeight + kCapsuleRadius));
  return pose;
}

SkeletonPtr makeGround()
{
  auto ground = Skeleton::create("ground");
  auto pair = ground->createJointAndBodyNodePair<WeldJoint>();

  // Collide with an infinite plane so the demo specifically exercises capsule /
  // plane contacts.
  const Eigen::Vector3d planeNormal = Eigen::Vector3d::UnitZ();
  auto planeShape = std::make_shared<PlaneShape>(planeNormal, 0.0);
  pair.second->createShapeNodeWith<CollisionAspect, DynamicsAspect>(planeShape);

  // Use a thin box purely for visualization so users can see the ground plane.
  constexpr double kVisualThickness = 0.1;
  auto visualShape
      = std::make_shared<BoxShape>(Eigen::Vector3d(10, 10, kVisualThickness));
  auto visualNode = pair.second->createShapeNodeWith<VisualAspect>(visualShape);
  Eigen::Isometry3d visualOffset = Eigen::Isometry3d::Identity();
  visualOffset.translate(planeNormal.normalized() * (-0.5 * kVisualThickness));
  visualNode->setRelativeTransform(visualOffset);
  const Eigen::Vector3d groundColor = Eigen::Vector3d::Constant(0.7);
  visualNode->getVisualAspect()->setColor(groundColor);
  visualNode->getVisualAspect()->setShadowed(false);

  ground->setMobile(false);

  return ground;
}

struct CapsuleSkeleton
{
  SkeletonPtr skeleton;
  FreeJoint* joint;
  BodyNode* body;
};

CapsuleSkeleton makeCapsuleSkeleton()
{
  CapsuleSkeleton capsule;
  capsule.skeleton = Skeleton::create("capsule");

  FreeJoint::Properties joint;
  joint.mName = "capsule_joint";
  joint.mT_ParentBodyToJoint = makeHorizontalPose();

  BodyNode::Properties body;
  body.mName = "capsule_body";

  auto result = capsule.skeleton->createJointAndBodyNodePair<FreeJoint>(
      nullptr, joint, body);

  auto shape = std::make_shared<CapsuleShape>(kCapsuleRadius, kCapsuleHeight);
  auto shapeNode = result.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  const Eigen::Vector3d capsuleColor(0.2, 0.4, 0.8);
  shapeNode->getVisualAspect()->setColor(capsuleColor);

  Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(shape->computeInertia(inertia.getMass()));
  result.second->setInertia(inertia);

  capsule.joint = result.first;
  capsule.body = result.second;

  return capsule;
}

class CapsuleEventHandler : public ::osgGA::GUIEventHandler
{
public:
  CapsuleEventHandler(const WorldPtr& world, FreeJoint* capsuleJoint)
    : mWorld(world),
      mJoint(capsuleJoint),
      mHorizontal(makeHorizontalPose()),
      mVertical(makeVerticalPose())
  {
    std::cout << "Controls:\n"
              << "  [H] Reset capsule to horizontal pose\n"
              << "  [V] Reset capsule to vertical pose\n"
              << "  [Space] Clear velocities\n"
              << "This example uses ODE with persistent manifolds to keep the\n"
              << "capsule from sinking when lying on its side.\n";
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() != ::osgGA::GUIEventAdapter::KEYDOWN) {
      return false;
    }

    switch (ea.getKey()) {
      case 'h':
      case 'H':
        resetPose(mHorizontal);
        return true;
      case 'v':
      case 'V':
        resetPose(mVertical);
        return true;
      case ' ':
        mJoint->setVelocities(Eigen::Vector6d::Zero());
        return true;
      default:
        return false;
    }
  }

private:
  void resetPose(const Eigen::Isometry3d& pose)
  {
    mWorld->reset();
    mJoint->setRelativeTransform(pose);
    mJoint->setVelocities(Eigen::Vector6d::Zero());
  }

  WorldPtr mWorld;
  FreeJoint* mJoint;
  Eigen::Isometry3d mHorizontal;
  Eigen::Isometry3d mVertical;
};

} // namespace

int main()
{
#if !DART_HAVE_ODE
  DART_ERROR(
      "capsule_ground_contact requires DART to be built with ODE support.\n");
  return 1;
#endif

  auto world = World::create();
  world->setTimeStep(0.001);
  auto odeDetector = collision::OdeCollisionDetector::create();
  if (!odeDetector) {
    DART_ERROR("Failed to create the ODE collision detector.\n");
    return 1;
  }
  world->setCollisionDetector(odeDetector);

  world->addSkeleton(makeGround());
  auto capsule = makeCapsuleSkeleton();
  world->addSkeleton(capsule.skeleton);

  osg::ref_ptr<RealTimeWorldNode> worldNode = new RealTimeWorldNode(world);

  Viewer viewer;
  viewer.addWorldNode(worldNode);
  viewer.addEventHandler(new CapsuleEventHandler(world, capsule.joint));
  viewer.setUpViewInWindow(100, 100, 1024, 768);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.5, 2.5, 1.5),
      ::osg::Vec3(0.0, 0.0, 0.2),
      ::osg::Vec3(-0.2, -0.2, 0.95));
  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.run();
}
