#include "helpers/dynamics_helpers.hpp"

#include "dart/dynamics/inertia.hpp"
#include "dart/io/read.hpp"
#include "dart/math/random.hpp"
#include "dart/simulation/world.hpp"
#include "dynamics_test_fixture.hpp"

using namespace dart;
using namespace simulation;
using namespace test;
using namespace Eigen;

TEST_F(DynamicsTest, HybridDynamics)
{
  const double tol = 1e-8;
  const double timeStep = 1e-3;
#if !defined(NDEBUG)
  const std::size_t numFrames = 50; // 0.05 secs
#else
  const std::size_t numFrames = 5e+3; // 5 secs
#endif // ------- Debug mode

  // Load world and skeleton
  WorldPtr world = dart::io::readWorld(
      "dart://sample/skel/test/hybrid_dynamics_test.skel");
  world->setTimeStep(timeStep);
  EXPECT_TRUE(world != nullptr);
  EXPECT_NEAR(world->getTimeStep(), timeStep, tol);

  SkeletonPtr skel = world->getSkeleton("skeleton 1");
  EXPECT_TRUE(skel != nullptr);
  EXPECT_NEAR(skel->getTimeStep(), timeStep, tol);

  const std::size_t numDofs = skel->getNumDofs();

  // Zero initial states
  Eigen::VectorXd q0 = Eigen::VectorXd::Zero(numDofs);
  Eigen::VectorXd dq0 = Eigen::VectorXd::Zero(numDofs);

  // Initialize the skeleton with the zero initial states
  skel->setPositions(q0);
  skel->setVelocities(dq0);
  EXPECT_TRUE(equals(skel->getPositions(), q0));
  EXPECT_TRUE(equals(skel->getVelocities(), dq0));

  // Make sure all the joint actuator types
  EXPECT_EQ(skel->getJoint(0)->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(skel->getJoint(1)->getActuatorType(), Joint::ACCELERATION);
  EXPECT_EQ(skel->getJoint(2)->getActuatorType(), Joint::VELOCITY);
  EXPECT_EQ(skel->getJoint(3)->getActuatorType(), Joint::ACCELERATION);
  EXPECT_EQ(skel->getJoint(4)->getActuatorType(), Joint::VELOCITY);

  // Prepare command for each joint types per simulation steps
  Eigen::MatrixXd command = Eigen::MatrixXd::Zero(numFrames, numDofs);
  Eigen::VectorXd amp = Eigen::VectorXd::Zero(numDofs);
  for (std::size_t i = 0; i < numDofs; ++i) {
    amp[i] = math::Random::uniform(-1.5, 1.5);
  }
  for (std::size_t i = 0; i < numFrames; ++i) {
    for (std::size_t j = 0; j < numDofs; ++j) {
      command(i, j) = amp[j] * std::sin(i * timeStep);
    }
  }

  // Record joint forces for joint[1~4]
  Eigen::MatrixXd forces = Eigen::MatrixXd::Zero(numFrames, numDofs);
  for (std::size_t i = 0; i < numFrames; ++i) {
    skel->setCommands(command.row(i));

    world->step(false);

    forces.row(i) = skel->getForces();

    EXPECT_NEAR(command(i, 0), skel->getForce(0), tol);
    EXPECT_NEAR(command(i, 1), skel->getAcceleration(1), tol);
    EXPECT_NEAR(command(i, 2), skel->getVelocity(2), tol);
    EXPECT_NEAR(command(i, 3), skel->getAcceleration(3), tol);
    EXPECT_NEAR(command(i, 4), skel->getVelocity(4), tol);
  }

  // Restore the skeleton to the initial state
  skel->setPositions(q0);
  skel->setVelocities(dq0);
  EXPECT_TRUE(equals(skel->getPositions(), q0));
  EXPECT_TRUE(equals(skel->getVelocities(), dq0));

  // Change all the actuator types to force
  skel->getJoint(0)->setActuatorType(Joint::FORCE);
  skel->getJoint(1)->setActuatorType(Joint::FORCE);
  skel->getJoint(2)->setActuatorType(Joint::FORCE);
  skel->getJoint(3)->setActuatorType(Joint::FORCE);
  skel->getJoint(4)->setActuatorType(Joint::FORCE);
  EXPECT_EQ(skel->getJoint(0)->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(skel->getJoint(1)->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(skel->getJoint(2)->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(skel->getJoint(3)->getActuatorType(), Joint::FORCE);
  EXPECT_EQ(skel->getJoint(4)->getActuatorType(), Joint::FORCE);

  // Test if the skeleton moves as the command with the joint forces
  Eigen::MatrixXd output = Eigen::MatrixXd::Zero(numFrames, numDofs);
  for (std::size_t i = 0; i < numFrames; ++i) {
    skel->setCommands(forces.row(i));

    world->step(false);

    output(i, 0) = skel->getJoint(0)->getForce(0);
    output(i, 1) = skel->getJoint(1)->getAcceleration(0);
    output(i, 2) = skel->getJoint(2)->getVelocity(0);
    output(i, 3) = skel->getJoint(3)->getAcceleration(0);
    output(i, 4) = skel->getJoint(4)->getVelocity(0);

    EXPECT_NEAR(command(i, 0), output(i, 0), tol);
    EXPECT_NEAR(command(i, 1), output(i, 1), tol);
    EXPECT_NEAR(command(i, 2), output(i, 2), tol);
    EXPECT_NEAR(command(i, 3), output(i, 3), tol);
    EXPECT_NEAR(command(i, 4), output(i, 4), tol);
  }
}

TEST_F(DynamicsTest, OffsetCom)
{
  WorldPtr world = World::create();
  ASSERT_TRUE(world != nullptr);
  world->setGravity(Eigen::Vector3d(0.0, 0.0, 0.0));
  const double dt = 0.001;
  world->setTimeStep(dt);

  SkeletonPtr boxSkel = createBox(Vector3d(1.0, 1.0, 1.0));
  world->addSkeleton(boxSkel);
  BodyNode* box = boxSkel->getBodyNode(0);
  dynamics::Inertia inertia;
  inertia.setMass(60);
  inertia.setMoment(
      box->getShapeNode(0)->getShape()->computeInertia(inertia.getMass()));
  inertia.setLocalCOM({0, 500, 0});
  box->setInertia(inertia);
  Eigen::Isometry3d boxInitialPose = box->getWorldTransform();
  Eigen::Vector3d torque(0, 0, 100);
  box->addExtTorque(torque, false);

  {
    Vector3d comVel = box->getCOMLinearVelocity();
    EXPECT_TRUE(equals(Vector3d(0, 0, 0), comVel))
        << "comVel: " << comVel.transpose();
  }
  {
    Vector3d linVel = box->getLinearVelocity();
    EXPECT_TRUE(equals(Vector3d(0, 0, 0), linVel))
        << "linVel: " << linVel.transpose();
  }

  world->step();
  // Velocity at COM should be zero since the object is just rotating.
  {
    Vector3d comVel = box->getCOMLinearVelocity();
    EXPECT_TRUE(equals(Vector3d(0, 0, 0), comVel, 1e-4))
        << "comVel: " << comVel.transpose();
  }
  {
    Vector3d angVel = box->getAngularVelocity();
    // We can compute expectedAngAccel like so because the initial angular
    // velocity is zero.
    Vector3d expectedAngAccel = inertia.getMoment().inverse() * torque;
    Vector3d expectedAngVel = expectedAngAccel * dt;
    EXPECT_TRUE(equals(expectedAngVel, angVel))
        << "angVel: " << angVel.transpose();
    Vector3d linVel = box->getLinearVelocity();
    Vector3d expLinVel
        = angVel.cross(boxInitialPose.linear() * -box->getLocalCOM());
    EXPECT_TRUE(equals(expLinVel, linVel))
        << "Expected: " << expLinVel.transpose()
        << "\nActual: " << linVel.transpose();

    Vector3d angAccel = box->getAngularAcceleration();
    EXPECT_TRUE(equals(expectedAngAccel, angAccel))
        << "angAccel: " << angAccel.transpose();
    Vector3d linAccel = box->getLinearAcceleration();
    Vector3d expLinAccel
        = expectedAngAccel.cross(boxInitialPose.linear() * -box->getLocalCOM());
    EXPECT_TRUE(equals(expLinAccel, linAccel))
        << "Expected: " << expLinAccel.transpose()
        << "\nActual: " << linAccel.transpose();
  }
}
