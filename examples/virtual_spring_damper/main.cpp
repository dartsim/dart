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

#include <dart/dart.hpp>

using namespace dart;
using namespace dart::dynamics;
using namespace dart::simulation;

class SpringDamperNode : public gui::osg::RealTimeWorldNode
{
public:
  SpringDamperNode(const WorldPtr& world, BodyNode* bodyA, BodyNode* bodyB)
    : gui::osg::RealTimeWorldNode(world), mBodyA(bodyA), mBodyB(bodyB)
  {
    mRestTransform = mBodyA->getTransform(mBodyB);
  }

  void customPreStep() override
  {
    Eigen::Isometry3d T = mBodyA->getTransform(mBodyB);
    Eigen::Vector3d dx = T.translation() - mRestTransform.translation();
    Eigen::Matrix3d Rerror = mRestTransform.linear().transpose() * T.linear();
    Eigen::Vector3d rotVec = math::logMap(Rerror);

    Eigen::Vector6d vel = mBodyA->getSpatialVelocity(mBodyB, mBodyB);
    Eigen::Vector3d relAngVel = vel.head<3>();
    Eigen::Vector3d relLinVel = vel.tail<3>();

    Eigen::Vector3d forceB = -mLinearK * dx - mLinearD * relLinVel;
    Eigen::Vector3d torqueB = -mAngularK * rotVec - mAngularD * relAngVel;

    Eigen::Matrix3d RB = mBodyB->getWorldTransform().linear();
    Eigen::Vector3d forceW = RB * forceB;
    Eigen::Vector3d torqueW = RB * torqueB;

    mBodyA->addExtForce(forceW);
    mBodyB->addExtForce(-forceW);
    mBodyA->addExtTorque(torqueW);
    mBodyB->addExtTorque(-torqueW);
  }

private:
  BodyNode* mBodyA;
  BodyNode* mBodyB;
  Eigen::Isometry3d mRestTransform;
  double mLinearK = 1000.0;
  double mLinearD = 50.0;
  double mAngularK = 200.0;
  double mAngularD = 10.0;
};

static SkeletonPtr createBox(
    const std::string& name,
    const Eigen::Vector3d& size,
    const Eigen::Vector3d& color,
    const Eigen::Vector3d& position)
{
  SkeletonPtr skel = Skeleton::create(name);
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>();
  auto body = pair.second;

  auto shape = std::make_shared<BoxShape>(size);
  auto sn = body->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(shape);
  sn->getVisualAspect()->setColor(color);
  body->setInertia(
      Inertia(1.0, Eigen::Vector3d::Zero(), shape->computeInertia(1.0)));

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;
  pair.first->setTransformFromParentBodyNode(tf);

  return skel;
}

int main()
{
  auto world = World::create();
  world->setGravity(Eigen::Vector3d(0, 0, -9.81));

  auto boxA = createBox(
      "boxA",
      Eigen::Vector3d(0.2, 0.2, 0.2),
      Color::Red(),
      Eigen::Vector3d(-0.3, 0, 0.5));
  auto boxB = createBox(
      "boxB",
      Eigen::Vector3d(0.2, 0.2, 0.2),
      Color::Blue(),
      Eigen::Vector3d(0.3, 0, 0.5));

  auto ground = Skeleton::create("ground");
  auto groundBody = ground->createJointAndBodyNodePair<WeldJoint>().second;
  auto groundShape = std::make_shared<BoxShape>(Eigen::Vector3d(5.0, 5.0, 0.1));
  auto groundSN = groundBody->createShapeNodeWith<
      VisualAspect,
      CollisionAspect,
      DynamicsAspect>(groundShape);
  groundSN->getVisualAspect()->setColor(Color::LightGray());

  world->addSkeleton(boxA);
  world->addSkeleton(boxB);
  world->addSkeleton(ground);

  auto node
      = new SpringDamperNode(world, boxA->getBodyNode(0), boxB->getBodyNode(0));

  gui::osg::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.addInstructionText("Press space to start simulation.\n");
  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 640, 480);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.0f, 2.0f, 1.5f),
      ::osg::Vec3(0.0f, 0.0f, 0.5f),
      ::osg::Vec3(-0.2f, -0.3f, 0.94f));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  viewer.run();

  return 0;
}
