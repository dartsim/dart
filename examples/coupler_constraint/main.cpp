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

#ifdef DART_COUPLER_HEADLESS_DEFAULT

  #include <iostream>

  #include <cstdlib>

int main(int /*argc*/, char* /*argv*/[])
{
  std::cout << "Coupler constraint GUI example built in headless mode; "
               "skipping viewer startup."
            << std::endl;
  return EXIT_SUCCESS;
}

#else

  #include <dart/gui/osg/RealTimeWorldNode.hpp>
  #include <dart/gui/osg/Viewer.hpp>

  #include <dart/simulation/World.hpp>

  #include <dart/dynamics/BodyNode.hpp>
  #include <dart/dynamics/BoxShape.hpp>
  #include <dart/dynamics/Inertia.hpp>
  #include <dart/dynamics/RevoluteJoint.hpp>
  #include <dart/dynamics/ShapeNode.hpp>
  #include <dart/dynamics/Skeleton.hpp>

  #include <dart/math/Constants.hpp>

  #include <Eigen/Dense>
  #include <Eigen/Geometry>
  #include <osgGA/GUIEventAdapter>
  #include <osgGA/GUIEventHandler>

  #include <iostream>
  #include <memory>
  #include <utility>

  #include <cassert>
  #include <cstdlib>

using dart::dynamics::BodyNode;
using dart::dynamics::BodyNodePtr;
using dart::dynamics::Joint;
using dart::dynamics::RevoluteJoint;
using dart::dynamics::Skeleton;
using dart::dynamics::SkeletonPtr;
using dart::simulation::World;
using dart::simulation::WorldPtr;

namespace {

SkeletonPtr createCoupledPendulum()
{
  SkeletonPtr skeleton = Skeleton::create("coupled_pendulum");

  // Reference link (root)
  RevoluteJoint::Properties rootJointProps;
  rootJointProps.mName = "reference_joint";
  rootJointProps.mAxis = Eigen::Vector3d::UnitZ();
  rootJointProps.mT_ParentBodyToJoint = Eigen::Isometry3d::Identity();
  rootJointProps.mT_ChildBodyToJoint = Eigen::Translation3d(-0.15, 0.0, 0.0);

  BodyNode::Properties rootBodyProps;
  rootBodyProps.mName = "reference_body";
  dart::dynamics::Inertia rootInertia;
  rootInertia.setMass(1.0);
  rootInertia.setMoment(0.01, 0.01, 0.02, 0.0, 0.0, 0.0);
  rootBodyProps.mInertia = rootInertia;

  auto rootPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, rootJointProps, rootBodyProps);
  auto* referenceJoint = rootPair.first;
  auto* referenceBody = rootPair.second;

  referenceJoint->setActuatorType(Joint::FORCE);
  referenceJoint->setForceLowerLimit(0, -50.0);
  referenceJoint->setForceUpperLimit(0, 50.0);
  referenceJoint->setDampingCoefficient(0, 0.01);

  auto referenceShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.3, 0.04, 0.04));
  auto referenceShapeNode = referenceBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(referenceShape);
  referenceShapeNode->setRelativeTranslation(Eigen::Vector3d(0.15, 0.0, 0.0));
  referenceShapeNode->getVisualAspect()->setColor(
      Eigen::Vector3d(0.3, 0.6, 0.9));

  // Coupled link (follower)
  RevoluteJoint::Properties followerJointProps;
  followerJointProps.mName = "coupled_joint";
  followerJointProps.mAxis = Eigen::Vector3d::UnitZ();
  followerJointProps.mT_ParentBodyToJoint = Eigen::Translation3d(0.3, 0.0, 0.0);
  followerJointProps.mT_ChildBodyToJoint
      = Eigen::Translation3d(-0.15, 0.0, 0.0);

  BodyNode::Properties followerBodyProps;
  followerBodyProps.mName = "coupled_body";
  dart::dynamics::Inertia followerInertia;
  followerInertia.setMass(1.0);
  followerInertia.setMoment(0.01, 0.01, 0.02, 0.0, 0.0, 0.0);
  followerBodyProps.mInertia = followerInertia;

  auto followerPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
      referenceBody, followerJointProps, followerBodyProps);
  auto* coupledJoint = followerPair.first;
  auto* coupledBody = followerPair.second;

  coupledJoint->setActuatorType(Joint::MIMIC);
  coupledJoint->setMimicJoint(referenceJoint, -1.0, 0.0);
  coupledJoint->setUseCouplerConstraint(true);
  coupledJoint->setForceLowerLimit(0, -50.0);
  coupledJoint->setForceUpperLimit(0, 50.0);
  coupledJoint->setDampingCoefficient(0, 0.01);

  auto coupledShape = std::make_shared<dart::dynamics::BoxShape>(
      Eigen::Vector3d(0.3, 0.04, 0.04));
  auto coupledShapeNode = coupledBody->createShapeNodeWith<
      dart::dynamics::VisualAspect,
      dart::dynamics::CollisionAspect,
      dart::dynamics::DynamicsAspect>(coupledShape);
  coupledShapeNode->setRelativeTranslation(Eigen::Vector3d(0.15, 0.0, 0.0));
  coupledShapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.9, 0.4, 0.3));

  // Provide a small initial offset so the constraint is active immediately
  referenceJoint->setPosition(0, 15.0 * dart::math::constantsd::pi() / 180.0);
  coupledJoint->setPosition(0, -15.0 * dart::math::constantsd::pi() / 180.0);

  return skeleton;
}

class CouplerController
{
public:
  explicit CouplerController(SkeletonPtr skeleton)
    : mSkeleton(std::move(skeleton)),
      mInitialPositions(mSkeleton->getPositions()),
      mTorqueSteps(0),
      mCouplerEnabled(true)
  {
    assert(mSkeleton);
  }

  void update()
  {
    // Clear any commands on the reference joint so it responds purely to
    // incoming constraint impulses.
    auto* referenceJoint = mSkeleton->getJoint("reference_joint");
    referenceJoint->setForce(0, 0.0);

    if (mTorqueSteps > 0) {
      auto* coupledBody = mSkeleton->getBodyNode("coupled_body");
      coupledBody->addExtTorque(Eigen::Vector3d(0.0, 0.0, mTorqueMagnitude));
      --mTorqueSteps;
    }
  }

  void triggerFollowerImpulse()
  {
    mTorqueSteps = 240;
  }

  void toggleCoupler()
  {
    mCouplerEnabled = !mCouplerEnabled;
    auto* coupledJoint = mSkeleton->getJoint("coupled_joint");
    coupledJoint->setUseCouplerConstraint(mCouplerEnabled);
  }

  bool isCouplerEnabled() const
  {
    return mCouplerEnabled;
  }

  void reset()
  {
    mSkeleton->setPositions(mInitialPositions);
    mSkeleton->setVelocities(Eigen::VectorXd::Zero(mSkeleton->getNumDofs()));
    mSkeleton->clearExternalForces();
    mTorqueSteps = 0;
  }

private:
  SkeletonPtr mSkeleton;
  Eigen::VectorXd mInitialPositions;
  int mTorqueSteps;
  double mTorqueMagnitude{4.0};
  bool mCouplerEnabled;
};

class CouplerWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  CouplerWorldNode(const WorldPtr& world, CouplerController* controller)
    : RealTimeWorldNode(world), mController(controller)
  {
  }

  void customPreStep() override
  {
    mController->update();
  }

private:
  CouplerController* mController;
};

class CouplerEventHandler : public ::osgGA::GUIEventHandler
{
public:
  CouplerEventHandler(CouplerController* controller) : mController(controller)
  {
  }

  bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() != ::osgGA::GUIEventAdapter::KEYDOWN)
      return false;

    switch (ea.getKey()) {
      case 'c':
        mController->toggleCoupler();
        std::cout
            << (mController->isCouplerEnabled() ? "Coupler enabled"
                                                : "Coupler disabled")
            << std::endl;
        return true;
      case 'p':
        mController->triggerFollowerImpulse();
        std::cout << "Applied torque pulse to the follower link" << std::endl;
        return true;
      case 'r':
        mController->reset();
        std::cout << "Reset configuration" << std::endl;
        return true;
      default:
        return false;
    }
  }

private:
  CouplerController* mController;
};

} // namespace

int main(int /*argc*/, char* /*argv*/[])
{
  WorldPtr world = World::create();
  world->setGravity(Eigen::Vector3d::Zero());
  world->setTimeStep(1e-3);

  SkeletonPtr skeleton = createCoupledPendulum();
  world->addSkeleton(skeleton);

  auto controller = std::make_unique<CouplerController>(skeleton);

  auto* worldNode = new CouplerWorldNode(world, controller.get());
  auto* handler = new CouplerEventHandler(controller.get());

  dart::gui::osg::Viewer viewer;
  if (osg::GraphicsContext::getWindowingSystemInterface() == nullptr) {
    std::cerr << "No OSG windowing system detected. Running the GUI example "
                 "requires an active display server.\n";
    return EXIT_FAILURE;
  }
  viewer.addWorldNode(worldNode);
  viewer.addEventHandler(handler);

  viewer.addInstructionText("space: toggle simulation");
  viewer.addInstructionText("'p': apply torque pulse to the follower link");
  viewer.addInstructionText("'c': toggle the coupler constraint");
  viewer.addInstructionText("'r': reset configuration");
  std::cout << viewer.getInstructions() << std::endl;

  viewer.setUpViewInWindow(0, 0, 960, 720);
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(1.5f, 1.5f, 1.2f),
      ::osg::Vec3(0.4f, 0.0f, 0.2f),
      ::osg::Vec3(0.0f, 0.0f, 1.0f));
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  if (!viewer.isRealized())
    viewer.realize();

  osg::ref_ptr<osg::GraphicsContext> gc
      = viewer.getCamera() ? viewer.getCamera()->getGraphicsContext() : nullptr;
  if (!viewer.isRealized() || !gc || !gc->valid()) {
    std::cerr << "Failed to create an OSG window. "
              << "Ensure DISPLAY is set or use a virtual framebuffer.\n";
    return EXIT_FAILURE;
  }

  const int runResult = viewer.run();
  if (runResult != 0) {
    std::cerr << "OSG viewer exited early (status " << runResult
              << "). Ensure a valid OpenGL context is available.\n";
    return runResult;
  }

  return 0;
}

#endif // DART_COUPLER_HEADLESS_DEFAULT
