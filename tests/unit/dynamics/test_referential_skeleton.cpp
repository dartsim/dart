// Copyright (c) 2011-2025, The DART development contributors

#include <dart/dynamics/chain.hpp>
#include <dart/dynamics/frame.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/group.hpp>
#include <dart/dynamics/invalid_index.hpp>
#include <dart/dynamics/linkage.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

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

TEST(ReferentialSkeleton, GroupChainLinkageHasAndIndex)
{
  auto skeleton = Skeleton::create("ref_chain");
  auto rootPair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  auto grandPair
      = childPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  std::array<BodyNode*, 2> groupNodes = {rootPair.second, childPair.second};
  auto group = Group::create("group_has", groupNodes);

  EXPECT_TRUE(group->hasBodyNode(rootPair.second));
  EXPECT_TRUE(group->hasBodyNode(childPair.second));
  EXPECT_EQ(group->getIndexOf(childPair.second, false), 1u);

  auto otherSkeleton = Skeleton::create("ref_other");
  auto otherPair = otherSkeleton->createJointAndBodyNodePair<RevoluteJoint>();
  EXPECT_FALSE(group->hasBodyNode(otherPair.second));
  EXPECT_EQ(group->getIndexOf(otherPair.second, false), INVALID_INDEX);

  auto chain = Chain::create(rootPair.second, grandPair.second, "chain_has");
  ASSERT_NE(chain, nullptr);
  EXPECT_TRUE(chain->hasBodyNode(childPair.second));
  EXPECT_NE(chain->getIndexOf(childPair.second, false), INVALID_INDEX);
  EXPECT_EQ(chain->getIndexOf(otherPair.second, false), INVALID_INDEX);

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(rootPair.second);
  criteria.mTargets.emplace_back(grandPair.second);
  auto linkage = Linkage::create(criteria, "linkage_has");
  ASSERT_NE(linkage, nullptr);
  EXPECT_TRUE(linkage->hasBodyNode(grandPair.second));
  EXPECT_NE(linkage->getIndexOf(grandPair.second, false), INVALID_INDEX);
  EXPECT_EQ(linkage->getIndexOf(otherPair.second, false), INVALID_INDEX);
}

TEST(ReferentialSkeleton, GroupAddRemoveNoWarnings)
{
  auto skeleton = Skeleton::create("group_add_remove");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  auto* joint = childPair.first;
  auto* dof = joint->getDof(0);

  auto group = Group::create("group_ops");
  EXPECT_TRUE(group->addBodyNode(childPair.second, false));
  EXPECT_TRUE(group->addJoint(joint, false, false));
  EXPECT_TRUE(group->addDof(dof, false, false));

  EXPECT_TRUE(group->removeDof(dof, false, false));
  EXPECT_TRUE(group->removeJoint(joint, false, false));
  EXPECT_TRUE(group->removeBodyNode(childPair.second, false));
}

TEST(ReferentialSkeleton, JacobianDerivativesOnGroup)
{
  auto skeleton = Skeleton::create("ref_jac_group");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  std::array<BodyNode*, 2> bodyNodes = {rootPair.second, childPair.second};
  auto group = Group::create("jac_group", bodyNodes);

  const auto jacClassic = group->getJacobianClassicDeriv(childPair.second);
  EXPECT_EQ(jacClassic.cols(), static_cast<int>(group->getNumDofs()));
  const auto jacClassicFrame
      = group->getJacobianClassicDeriv(childPair.second, Frame::World());
  EXPECT_EQ(jacClassicFrame.cols(), static_cast<int>(group->getNumDofs()));
  const auto jacClassicOffset = group->getJacobianClassicDeriv(
      childPair.second, Eigen::Vector3d::UnitX(), Frame::World());
  EXPECT_EQ(jacClassicOffset.cols(), static_cast<int>(group->getNumDofs()));

  const auto linearDeriv
      = group->getLinearJacobianDeriv(childPair.second, Frame::World());
  EXPECT_EQ(linearDeriv.cols(), static_cast<int>(group->getNumDofs()));
  const auto linearDerivOffset = group->getLinearJacobianDeriv(
      childPair.second, Eigen::Vector3d::UnitZ(), Frame::World());
  EXPECT_EQ(linearDerivOffset.cols(), static_cast<int>(group->getNumDofs()));

  const auto angularDeriv
      = group->getAngularJacobianDeriv(childPair.second, Frame::World());
  EXPECT_EQ(angularDeriv.cols(), static_cast<int>(group->getNumDofs()));
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

TEST(ReferentialSkeleton, ConstAccessors)
{
  auto skeleton = Skeleton::create("ref_const");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setName("dup");

  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setName("joint_dup");
  const std::string childName = childPair.second->getName();

  std::array<BodyNode*, 2> bodyNodes = {rootPair.second, childPair.second};
  auto group = Group::create("ref_const_group", bodyNodes);

  const Group* constGroup = group.get();
  EXPECT_EQ(constGroup->getBodyNode("dup"), rootPair.second);

  const auto constBodyNodes = constGroup->getBodyNodes("dup");
  EXPECT_EQ(constBodyNodes.size(), 1u);

  const auto* jointByName = constGroup->getJoint("joint_dup");
  ASSERT_NE(jointByName, nullptr);

  const auto jointsByName = constGroup->getJoints("joint_dup");
  EXPECT_EQ(jointsByName.size(), 1u);

  const auto allJoints = constGroup->getJoints();
  EXPECT_EQ(allJoints.size(), constGroup->getNumJoints());
}

TEST(ReferentialSkeleton, JointVectorsAndLookup)
{
  auto skeleton = Skeleton::create("ref_joint_vec");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.first->setName("root_joint");
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setName("child_joint");

  std::array<BodyNode*, 2> bodyNodes = {rootPair.second, childPair.second};
  auto group = Group::create("ref_joint_group", bodyNodes);

  auto joints = group->getJoints();
  EXPECT_EQ(joints.size(), 2u);
  EXPECT_EQ(group->getJoints("child_joint").size(), 1u);
  EXPECT_TRUE(group->getJoints("missing").empty());

  const Group* constGroup = group.get();
  auto constJoints = constGroup->getJoints();
  EXPECT_EQ(constJoints.size(), 2u);
}

TEST(ReferentialSkeleton, DofIndexingAccessors)
{
  auto skeleton = Skeleton::create("ref_dof");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  std::array<BodyNode*, 1> bodyNodes = {rootPair.second};
  auto group = Group::create("ref_dof_group", bodyNodes);

  EXPECT_EQ(group->getNumDofs(), rootPair.first->getNumDofs());
  EXPECT_NE(group->getDof(0), nullptr);

  auto dofs = group->getDofs();
  EXPECT_EQ(dofs.size(), group->getNumDofs());

  const auto* dof = group->getDof(0);
  EXPECT_EQ(group->getIndexOf(dof, false), 0u);

  const Group* constGroup = group.get();
  EXPECT_EQ(constGroup->getDofs().size(), group->getNumDofs());
}

TEST(ReferentialSkeleton, BodyNodeNameLookupEmpty)
{
  auto skeleton = Skeleton::create("ref_names");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  rootPair.second->setName("root");

  std::array<BodyNode*, 1> bodyNodes = {rootPair.second};
  auto group = Group::create("ref_name_group", bodyNodes);

  EXPECT_TRUE(group->getBodyNodes("missing").empty());

  const Group* constGroup = group.get();
  EXPECT_TRUE(constGroup->getBodyNodes("missing").empty());
}

TEST(ReferentialSkeletonGroup, NullComponentOperations)
{
  auto group = Group::create("group_null");

  EXPECT_FALSE(group->addBodyNode(nullptr, false));
  EXPECT_FALSE(group->removeBodyNode(nullptr, false));
  EXPECT_FALSE(group->addComponent(nullptr, false));
  EXPECT_FALSE(group->removeComponent(nullptr, false));
  EXPECT_FALSE(group->addJoint(nullptr, false, false));
  EXPECT_FALSE(group->removeJoint(nullptr, false, false));
  EXPECT_FALSE(group->addDof(nullptr, false, false));
  EXPECT_FALSE(group->removeDof(nullptr, false, false));
}

TEST(ReferentialSkeletonGroup, DuplicateAndMissingBodyNodes)
{
  auto skeleton = Skeleton::create("group_body_nodes");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto* body = rootPair.second;

  auto group = Group::create("group_body_nodes");
  EXPECT_TRUE(group->addBodyNode(body, false));
  EXPECT_FALSE(group->addBodyNode(body, false));
  EXPECT_TRUE(group->removeBodyNode(body, false));
  EXPECT_FALSE(group->removeBodyNode(body, false));
}

TEST(ReferentialSkeletonGroup, JointAndDofManagement)
{
  auto skeleton = Skeleton::create("group_joints");
  auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>();
  pair.first->setAxis(Eigen::Vector3d::UnitZ());

  auto* joint = pair.first;
  auto* dof = joint->getDof(0);

  auto group = Group::create("group_joints");
  EXPECT_TRUE(group->addJoint(joint, false, false));
  EXPECT_TRUE(group->addDof(dof, false, false));
  EXPECT_EQ(group->getNumJoints(), 1u);
  EXPECT_EQ(group->getNumDofs(), 1u);
}

TEST(ReferentialSkeleton, LockableReferenceAndRename)
{
  auto skeleton = Skeleton::create("lockable_skel");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();

  std::array<BodyNode*, 1> bodyNodes = {rootPair.second};
  auto group = Group::create("lockable_group", bodyNodes);

  auto lockable = group->getLockableReference();
  ASSERT_NE(lockable, nullptr);
  lockable->lock();
  lockable->unlock();

  const auto& newName = group->setName("lockable_group_renamed");
  EXPECT_EQ(newName, "lockable_group_renamed");
}

TEST(ReferentialSkeletonGroup, AddRemoveMultipleBodies)
{
  auto skeleton = Skeleton::create("group_multi_body");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  std::array<BodyNode*, 2> bodyNodes = {rootPair.second, childPair.second};
  auto group = Group::create("group_multi_body");

  EXPECT_TRUE(group->addBodyNodes(bodyNodes, false));
  EXPECT_EQ(group->getNumBodyNodes(), 2u);

  EXPECT_TRUE(group->removeBodyNodes(bodyNodes, false));
  EXPECT_EQ(group->getNumBodyNodes(), 0u);
}

TEST(ReferentialSkeletonGroup, AddRemoveMultipleJoints)
{
  auto skeleton = Skeleton::create("group_multi_joint");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitZ());

  std::array<Joint*, 2> joints = {rootPair.first, childPair.first};
  auto group = Group::create("group_multi_joint");

  EXPECT_TRUE(group->addJoints(joints, true, false));
  EXPECT_EQ(group->getNumJoints(), 2u);
  EXPECT_EQ(
      group->getNumDofs(),
      rootPair.first->getNumDofs() + childPair.first->getNumDofs());

  EXPECT_TRUE(group->removeJoints(joints, true, false));
  EXPECT_EQ(group->getNumJoints(), 0u);
  EXPECT_EQ(group->getNumDofs(), 0u);
}

TEST(ReferentialSkeleton, BodyNodeSpanAccessors)
{
  auto skeleton = Skeleton::create("ref_span");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  std::array<BodyNode*, 2> bodyNodes = {rootPair.second, childPair.second};
  auto group = Group::create("ref_span_group", bodyNodes);

  auto& mutableNodes = group->getBodyNodes();
  EXPECT_EQ(mutableNodes.size(), 2u);

  const Group* constGroup = group.get();
  const auto constNodes = constGroup->getBodyNodes();
  EXPECT_EQ(constNodes.size(), 2u);
  EXPECT_EQ(constNodes[1], childPair.second);
}

TEST(ReferentialSkeleton, RemoveBodyNodeUpdatesIndices)
{
  auto skeleton = Skeleton::create("ref_remove");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();

  std::array<BodyNode*, 2> bodyNodes = {rootPair.second, childPair.second};
  auto group = Group::create("ref_remove_group", bodyNodes);
  EXPECT_EQ(group->getNumBodyNodes(), 2u);

  EXPECT_TRUE(group->removeBodyNode(childPair.second, false));
  EXPECT_EQ(group->getNumBodyNodes(), 1u);
  EXPECT_EQ(group->getBodyNode(0), rootPair.second);
}

TEST(ReferentialSkeleton, ConstJointAndDofLookups)
{
  auto skeleton = Skeleton::create("ref_const_access");
  auto rootPair = skeleton->createJointAndBodyNodePair<FreeJoint>();
  auto childPair
      = rootPair.second->createChildJointAndBodyNodePair<RevoluteJoint>();
  childPair.first->setAxis(Eigen::Vector3d::UnitZ());

  std::array<BodyNode*, 2> bodyNodes = {rootPair.second, childPair.second};
  auto group = Group::create("ref_const_access_group", bodyNodes);

  const Group* constGroup = group.get();
  const auto* joint = constGroup->getJoint(childPair.first->getName());
  ASSERT_NE(joint, nullptr);
  const auto* dof = constGroup->getDof(0);
  ASSERT_NE(dof, nullptr);

  EXPECT_NE(constGroup->getIndexOf(joint, false), INVALID_INDEX);
  EXPECT_NE(constGroup->getIndexOf(dof, false), INVALID_INDEX);
}
