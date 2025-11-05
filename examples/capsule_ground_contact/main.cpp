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

#include <dart/gui/osg/osg.hpp>

#include <dart/all.hpp>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui::osg;

namespace {

constexpr double kCapsuleRadius = 0.2;
constexpr double kCapsuleHeight = 0.6;

Eigen::Isometry3d makeHorizontalPose()
{
  return Eigen::Translation3d(0, 0, kCapsuleRadius + 0.1)
         * Eigen::AngleAxisd(
             dart::math::constantsd::half_pi(), Eigen::Vector3d::UnitY());
}

Eigen::Isometry3d makeVerticalPose()
{
  return Eigen::Translation3d(0, 0, 0.5 * kCapsuleHeight + kCapsuleRadius);
}

SkeletonPtr makeGround()
{
  auto ground = Skeleton::create("ground");
  auto pair = ground->createJointAndBodyNodePair<WeldJoint>();

  auto shape = std::make_shared<BoxShape>(Eigen::Vector3d(10, 10, 0.1));
  auto node = pair.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  node->getVisualAspect()->setColor(Eigen::Vector3d::Constant(0.7));
  node->getVisualAspect()->setCastShadows(false);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translate(Eigen::Vector3d(0, 0, -0.05));
  pair.first->setTransformFromParentBodyNode(pose);
  ground->setMobile(false);

  return ground;
}

std::pair<FreeJoint*, BodyNode*> makeCapsuleSkeleton()
{
  auto skeleton = Skeleton::create("capsule");
  FreeJoint::Properties joint;
  joint.mName = "capsule_joint";
  joint.mT_ParentBodyToJoint = makeHorizontalPose();

  BodyNode::Properties body;
  body.mName = "capsule_body";

  auto result
      = skeleton->createJointAndBodyNodePair<FreeJoint>(nullptr, joint, body);

  auto shape = std::make_shared<CapsuleShape>(kCapsuleRadius, kCapsuleHeight);
  auto shapeNode = result.second->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.2, 0.4, 0.8));

  Inertia inertia;
  inertia.setMass(1.0);
  inertia.setMoment(shape->computeInertia(inertia.getMass()));
  result.second->setInertia(inertia);

  return result;
}

class CapsuleEventHandler : public ::osgGA::GUIEventHandler
{
public:
  CapsuleEventHandler(
      const WorldPtr& world, FreeJoint* capsuleJoint, BodyNode* capsuleBody)
    : mWorld(world),
      mJoint(capsuleJoint),
      mBody(capsuleBody),
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
        mBody->setVelocities(Eigen::Vector6d::Zero());
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
    mBody->setVelocities(Eigen::Vector6d::Zero());
  }

  WorldPtr mWorld;
  FreeJoint* mJoint;
  BodyNode* mBody;
  Eigen::Isometry3d mHorizontal;
  Eigen::Isometry3d mVertical;
};

} // namespace

int main()
{
  auto world = World::create();
  world->setTimeStep(0.001);
  world->getConstraintSolver()->setCollisionDetector(
      collision::OdeCollisionDetector::create());

  world->addSkeleton(makeGround());
  auto capsule = makeCapsuleSkeleton();
  world->addSkeleton(capsule.first->getSkeleton());

  osg::ref_ptr<RealTimeWorldNode> worldNode = new RealTimeWorldNode(world);

  Viewer viewer;
  viewer.addWorldNode(worldNode);
  viewer.addEventHandler(
      new CapsuleEventHandler(world, capsule.first, capsule.second));
  viewer.setUpViewInWindow(100, 100, 1024, 768);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.5, 2.5, 1.5),
      ::osg::Vec3(0.0, 0.0, 0.2),
      ::osg::Vec3(-0.2, -0.2, 0.95));
  viewer.setCameraManipulator(viewer.getCameraManipulator());
  viewer.run();
}
