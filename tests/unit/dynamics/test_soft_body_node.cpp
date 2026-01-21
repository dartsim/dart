// Copyright (c) 2011-2025, The DART development contributors

#include <dart/simulation/World.hpp>

#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace dart;
using namespace dart::dynamics;

TEST(SoftBodyNode, AccessorsAndDynamics)
{
  auto skeleton = Skeleton::create("soft-body");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(1.0),
      Eigen::Isometry3d::Identity(),
      1.0,
      10.0,
      10.0,
      0.1);

  EXPECT_GT(softBody->getNumPointMasses(), 0u);
  EXPECT_GT(softBody->getNumFaces(), 0u);
  EXPECT_GT(softBody->getMass(), 0.0);

  const auto face = softBody->getFace(0);
  EXPECT_GE(face[0], 0);
  EXPECT_GE(face[1], 0);
  EXPECT_GE(face[2], 0);

  skeleton->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  skeleton->computeForwardDynamics();
  skeleton->computeInverseDynamics(true, true, true);

  const auto massMatrix = skeleton->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), skeleton->getNumDofs());
  EXPECT_EQ(massMatrix.cols(), skeleton->getNumDofs());

  auto world = simulation::World::create();
  world->addSkeleton(skeleton);
  world->step();

  auto* pointMass = softBody->getPointMass(0);
  ASSERT_NE(pointMass, nullptr);
  EXPECT_TRUE(pointMass->getPositions().array().isFinite().all());
}

TEST(SoftBodyNode, MassMatricesAndForces)
{
  auto skeleton = Skeleton::create("soft-body-mass");
  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint, SoftBodyNode>();
  auto* softBody = pair.second;

  SoftBodyNodeHelper::setBox(
      softBody,
      Eigen::Vector3d::Constant(0.5),
      Eigen::Isometry3d::Identity(),
      2.0,
      5.0,
      6.0,
      0.2);

  softBody->setVertexSpringStiffness(20.0);
  softBody->setEdgeSpringStiffness(30.0);
  softBody->setDampingCoefficient(0.05);
  EXPECT_DOUBLE_EQ(softBody->getVertexSpringStiffness(), 20.0);
  EXPECT_DOUBLE_EQ(softBody->getEdgeSpringStiffness(), 30.0);

  const auto dofs = skeleton->getNumDofs();
  skeleton->setPositions(Eigen::VectorXd::Constant(dofs, 0.1));
  skeleton->setVelocities(Eigen::VectorXd::Constant(dofs, 0.2));

  const auto massMatrix = skeleton->getMassMatrix();
  const auto augMassMatrix = skeleton->getAugMassMatrix();
  const auto invMassMatrix = skeleton->getInvMassMatrix();
  const auto invAugMassMatrix = skeleton->getInvAugMassMatrix();

  EXPECT_EQ(massMatrix.rows(), dofs);
  EXPECT_EQ(augMassMatrix.rows(), dofs);
  EXPECT_EQ(invMassMatrix.rows(), dofs);
  EXPECT_EQ(invAugMassMatrix.rows(), dofs);

  const auto coriolis = skeleton->getCoriolisAndGravityForces();
  EXPECT_EQ(coriolis.size(), dofs);

  const auto com = skeleton->getCOM();
  EXPECT_TRUE(com.array().isFinite().all());
  const auto comJac = skeleton->getCOMJacobian();
  EXPECT_EQ(comJac.cols(), dofs);

  skeleton->clearExternalForces();
  skeleton->clearInternalForces();

  EXPECT_TRUE(std::isfinite(skeleton->computeKineticEnergy()));
  EXPECT_TRUE(std::isfinite(skeleton->computePotentialEnergy()));
}
