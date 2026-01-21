// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/Group.hpp>
#include <dart/dynamics/InvalidIndex.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>

using namespace dart;
using namespace dart::dynamics;

TEST(ReferentialSkeleton, BodyNodeLookupAndIndexing)
{
  auto skeletonA = Skeleton::create("refA");
  auto rootPairA = skeletonA->createJointAndBodyNodePair<FreeJoint>();
  auto* rootA = rootPairA.second;
  rootA->setName("dup");

  auto skeletonB = Skeleton::create("refB");
  auto rootPairB = skeletonB->createJointAndBodyNodePair<FreeJoint>();
  auto* rootB = rootPairB.second;
  rootB->setName("dup");

  std::array<BodyNode*, 2> bodyNodes = {rootA, rootB};
  auto group = Group::create("ref-group", bodyNodes);

  EXPECT_EQ(group->getNumBodyNodes(), 2u);
  EXPECT_EQ(group->getBodyNode(0), rootA);
  EXPECT_EQ(group->getBodyNode(1), rootB);

  EXPECT_EQ(group->getBodyNode("dup"), rootA);
  const auto named = group->getBodyNodes("dup");
  EXPECT_EQ(named.size(), 2u);
  EXPECT_EQ(named[0], rootA);
  EXPECT_EQ(named[1], rootB);

  EXPECT_TRUE(group->hasBodyNode(rootB));
  EXPECT_EQ(group->getIndexOf(rootB, false), 1u);

  EXPECT_EQ(
      group->getIndexOf(static_cast<const BodyNode*>(nullptr), false),
      INVALID_INDEX);
}

TEST(ReferentialSkeleton, DofsJacobianAndComputation)
{
  auto skelA = Skeleton::create("ref-skel-a");
  auto pairA = skelA->createJointAndBodyNodePair<FreeJoint>();
  auto* bodyA = pairA.second;
  bodyA->setMass(2.0);

  auto skelB = Skeleton::create("ref-skel-b");
  auto pairB = skelB->createJointAndBodyNodePair<RevoluteJoint>();
  auto* bodyB = pairB.second;
  bodyB->setMass(3.0);
  pairB.first->setAxis(Eigen::Vector3d::UnitZ());

  std::array<BodyNode*, 2> bodyNodes = {bodyA, bodyB};
  auto group = Group::create("ref-group-ik", bodyNodes);

  EXPECT_EQ(group->getNumSkeletons(), 2u);
  EXPECT_TRUE(group->hasSkeleton(skelA.get()));
  EXPECT_TRUE(group->hasSkeleton(skelB.get()));

  EXPECT_EQ(group->getNumJoints(), 2u);
  EXPECT_EQ(group->getNumDofs(), skelA->getNumDofs() + skelB->getNumDofs());

  EXPECT_NE(group->getJoint(0), nullptr);
  EXPECT_NE(group->getJoint(1), nullptr);
  EXPECT_NE(group->getDof(0), nullptr);

  const auto& massMatrix = group->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), static_cast<int>(group->getNumDofs()));
  EXPECT_EQ(massMatrix.cols(), static_cast<int>(group->getNumDofs()));

  const auto jac = group->getJacobian(bodyA);
  EXPECT_EQ(jac.cols(), static_cast<int>(group->getNumDofs()));

  const auto worldJac = group->getWorldJacobian(bodyB);
  EXPECT_EQ(worldJac.cols(), static_cast<int>(group->getNumDofs()));

  const auto linearJac = group->getLinearJacobian(bodyA);
  EXPECT_EQ(linearJac.cols(), static_cast<int>(group->getNumDofs()));

  const auto com = group->getCOM();
  EXPECT_TRUE(com.array().isFinite().all());
  EXPECT_GE(group->computeKineticEnergy(), 0.0);
  EXPECT_TRUE(std::isfinite(group->computePotentialEnergy()));

  const auto comJac = group->getCOMJacobian();
  EXPECT_EQ(comJac.cols(), static_cast<int>(group->getNumDofs()));

  group->clearExternalForces();
  group->clearInternalForces();
}

TEST(ReferentialSkeleton, JacobiansAndDynamics)
{
  auto skelA = Skeleton::create("ref-ops-a");
  auto pairA = skelA->createJointAndBodyNodePair<FreeJoint>();
  pairA.first->setName("joint_a");
  pairA.second->setName("body_a");
  pairA.second->setMass(2.0);

  auto skelB = Skeleton::create("ref-ops-b");
  auto pairB = skelB->createJointAndBodyNodePair<RevoluteJoint>();
  pairB.first->setAxis(Eigen::Vector3d::UnitZ());
  pairB.first->setName("joint_b");
  pairB.second->setName("body_b");
  pairB.second->setMass(3.0);

  std::array<BodyNode*, 2> bodyNodes = {pairA.second, pairB.second};
  auto group = Group::create("ref-group-ops", bodyNodes);

  EXPECT_NEAR(group->getMass(), 5.0, 1e-12);

  EXPECT_NE(group->getJoint("joint_a"), nullptr);
  EXPECT_EQ(group->getJoints().size(), 2u);
  EXPECT_EQ(group->getJoints("joint_b").size(), 1u);
  EXPECT_TRUE(group->hasJoint(pairB.first));
  EXPECT_NE(group->getIndexOf(pairB.first, false), INVALID_INDEX);

  auto dof = group->getDof(0);
  ASSERT_NE(dof, nullptr);
  EXPECT_NE(group->getIndexOf(dof, false), INVALID_INDEX);

  const auto dofSpan = group->getDofs();
  EXPECT_EQ(dofSpan.size(), group->getNumDofs());

  const auto constDofs = static_cast<const Group&>(*group).getDofs();
  EXPECT_EQ(constDofs.size(), group->getNumDofs());

  skelA->setPositions(Eigen::VectorXd::Constant(skelA->getNumDofs(), 0.1));
  skelB->setPositions(Eigen::VectorXd::Constant(skelB->getNumDofs(), -0.2));
  skelA->setVelocities(Eigen::VectorXd::Constant(skelA->getNumDofs(), 0.3));
  skelB->setVelocities(Eigen::VectorXd::Constant(skelB->getNumDofs(), -0.4));

  const auto jac = group->getJacobian(pairA.second);
  EXPECT_EQ(jac.cols(), static_cast<int>(group->getNumDofs()));
  const auto jacOffset
      = group->getJacobian(pairA.second, Eigen::Vector3d::UnitX());
  EXPECT_EQ(jacOffset.cols(), static_cast<int>(group->getNumDofs()));

  const auto worldJac
      = group->getWorldJacobian(pairB.second, Eigen::Vector3d::UnitY());
  EXPECT_EQ(worldJac.cols(), static_cast<int>(group->getNumDofs()));

  const auto linearJac = group->getLinearJacobian(
      pairA.second, Eigen::Vector3d::UnitZ(), Frame::World());
  EXPECT_EQ(linearJac.cols(), static_cast<int>(group->getNumDofs()));
  const auto angularJac
      = group->getAngularJacobian(pairA.second, Frame::World());
  EXPECT_EQ(angularJac.cols(), static_cast<int>(group->getNumDofs()));

  const auto jacDeriv = group->getJacobianSpatialDeriv(pairA.second);
  EXPECT_EQ(jacDeriv.cols(), static_cast<int>(group->getNumDofs()));
  const auto jacClassicDeriv = group->getJacobianClassicDeriv(
      pairA.second, Eigen::Vector3d::UnitX(), Frame::World());
  EXPECT_EQ(jacClassicDeriv.cols(), static_cast<int>(group->getNumDofs()));

  const auto linearJacDeriv
      = group->getLinearJacobianDeriv(pairA.second, Frame::World());
  EXPECT_EQ(linearJacDeriv.cols(), static_cast<int>(group->getNumDofs()));
  const auto angularJacDeriv
      = group->getAngularJacobianDeriv(pairA.second, Frame::World());
  EXPECT_EQ(angularJacDeriv.cols(), static_cast<int>(group->getNumDofs()));

  const auto com = group->getCOM();
  EXPECT_TRUE(com.array().isFinite().all());
  const auto comSpatialVel = group->getCOMSpatialVelocity();
  EXPECT_TRUE(comSpatialVel.array().isFinite().all());
  const auto comLinearVel = group->getCOMLinearVelocity();
  EXPECT_TRUE(comLinearVel.array().isFinite().all());
  const auto comSpatialAcc = group->getCOMSpatialAcceleration();
  EXPECT_TRUE(comSpatialAcc.array().isFinite().all());
  const auto comLinearAcc = group->getCOMLinearAcceleration();
  EXPECT_TRUE(comLinearAcc.array().isFinite().all());

  const auto comJac = group->getCOMJacobian();
  EXPECT_EQ(comJac.cols(), static_cast<int>(group->getNumDofs()));
  const auto comLinearJac = group->getCOMLinearJacobian();
  EXPECT_EQ(comLinearJac.cols(), static_cast<int>(group->getNumDofs()));
  const auto comJacDeriv = group->getCOMJacobianSpatialDeriv();
  EXPECT_EQ(comJacDeriv.cols(), static_cast<int>(group->getNumDofs()));
  const auto comLinearJacDeriv = group->getCOMLinearJacobianDeriv();
  EXPECT_EQ(comLinearJacDeriv.cols(), static_cast<int>(group->getNumDofs()));

  const auto massMatrix = group->getMassMatrix();
  EXPECT_EQ(massMatrix.rows(), static_cast<int>(group->getNumDofs()));
  const auto augMassMatrix = group->getAugMassMatrix();
  EXPECT_EQ(augMassMatrix.rows(), static_cast<int>(group->getNumDofs()));
  const auto invMassMatrix = group->getInvMassMatrix();
  EXPECT_EQ(invMassMatrix.rows(), static_cast<int>(group->getNumDofs()));
  const auto invAugMassMatrix = group->getInvAugMassMatrix();
  EXPECT_EQ(invAugMassMatrix.rows(), static_cast<int>(group->getNumDofs()));

  const auto coriolis = group->getCoriolisForces();
  EXPECT_EQ(coriolis.size(), group->getNumDofs());
  const auto gravity = group->getGravityForces();
  EXPECT_EQ(gravity.size(), group->getNumDofs());
  const auto coriolisGravity = group->getCoriolisAndGravityForces();
  EXPECT_EQ(coriolisGravity.size(), group->getNumDofs());
  const auto external = group->getExternalForces();
  EXPECT_EQ(external.size(), group->getNumDofs());
}
