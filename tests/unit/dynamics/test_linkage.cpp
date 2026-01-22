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

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/branch.hpp>
#include <dart/dynamics/chain.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/linkage.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

//==============================================================================
SkeletonPtr createBranchingSkeleton()
{
  auto skel = Skeleton::create("branching");

  auto pair0 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, RevoluteJoint::Properties(), BodyNode::AspectProperties("root"));
  pair0.first->setName("root_joint");
  auto* root = pair0.second;

  auto pair1 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      root, RevoluteJoint::Properties(), BodyNode::AspectProperties("child1"));
  pair1.first->setName("joint1");

  auto pair2 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      root, RevoluteJoint::Properties(), BodyNode::AspectProperties("child2"));
  pair2.first->setName("joint2");

  auto pair3 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      pair1.second,
      RevoluteJoint::Properties(),
      BodyNode::AspectProperties("grandchild1"));
  pair3.first->setName("joint3");

  auto pair4 = skel->createJointAndBodyNodePair<RevoluteJoint>(
      pair2.second,
      RevoluteJoint::Properties(),
      BodyNode::AspectProperties("grandchild2"));
  pair4.first->setName("joint4");

  return skel;
}

SkeletonPtr createChainSkeleton(std::size_t numBodies)
{
  auto skel = Skeleton::create("chain");

  BodyNode* parent = nullptr;
  for (std::size_t i = 0; i < numBodies; ++i) {
    auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>(
        parent,
        RevoluteJoint::Properties(),
        BodyNode::AspectProperties("body_" + std::to_string(i)));
    pair.first->setName("joint_" + std::to_string(i));
    parent = pair.second;
  }

  return skel;
}

//==============================================================================
TEST(LinkageTests, CriteriaTarget)
{
  auto skel = createChainSkeleton(5);

  Linkage::Criteria::Target target;
  EXPECT_EQ(target.mNode.lock(), nullptr);
  EXPECT_EQ(target.mPolicy, Linkage::Criteria::INCLUDE);
  EXPECT_FALSE(target.mChain);

  auto* body2 = skel->getBodyNode(2);
  Linkage::Criteria::Target target2(body2, Linkage::Criteria::DOWNSTREAM, true);
  EXPECT_EQ(target2.mNode.lock(), body2);
  EXPECT_EQ(target2.mPolicy, Linkage::Criteria::DOWNSTREAM);
  EXPECT_TRUE(target2.mChain);
}

//==============================================================================
TEST(LinkageTests, CriteriaTerminal)
{
  auto skel = createChainSkeleton(3);

  Linkage::Criteria::Terminal terminal;
  EXPECT_EQ(terminal.mTerminal.lock(), nullptr);
  EXPECT_TRUE(terminal.mInclusive);

  auto* body1 = skel->getBodyNode(1);
  Linkage::Criteria::Terminal terminal2(body1, false);
  EXPECT_EQ(terminal2.mTerminal.lock(), body1);
  EXPECT_FALSE(terminal2.mInclusive);
}

//==============================================================================
TEST(LinkageTests, CreateEmptyLinkage)
{
  Linkage::Criteria criteria;
  auto linkage = Linkage::create(criteria, "empty_linkage");

  ASSERT_NE(linkage, nullptr);
  EXPECT_EQ(linkage->getName(), "empty_linkage");
  EXPECT_EQ(linkage->getNumBodyNodes(), 0u);
  EXPECT_EQ(linkage->getNumJoints(), 0u);
  EXPECT_EQ(linkage->getNumDofs(), 0u);
}

//==============================================================================
TEST(LinkageTests, CreateLinkageFromChain)
{
  auto skel = createChainSkeleton(5);
  auto* start = skel->getBodyNode(1);
  auto* end = skel->getBodyNode(3);

  Linkage::Criteria criteria(start, end);
  auto linkage = Linkage::create(criteria, "chain_linkage");

  ASSERT_NE(linkage, nullptr);
  EXPECT_EQ(linkage->getName(), "chain_linkage");
  EXPECT_GE(linkage->getNumBodyNodes(), 1u);
  EXPECT_TRUE(linkage->hasBodyNode(end));
}

//==============================================================================
TEST(LinkageTests, CreateLinkageWithDownstreamExpansion)
{
  auto skel = createBranchingSkeleton();
  auto* root = skel->getBodyNode("root");

  Linkage::Criteria criteria;
  criteria.mStart
      = Linkage::Criteria::Target(root, Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "downstream_linkage");

  ASSERT_NE(linkage, nullptr);
  EXPECT_GE(linkage->getNumBodyNodes(), 1u);
  EXPECT_TRUE(linkage->hasBodyNode(root));
}

//==============================================================================
TEST(LinkageTests, CreateLinkageWithUpstreamExpansion)
{
  auto skel = createChainSkeleton(5);
  auto* leaf = skel->getBodyNode(4);

  Linkage::Criteria criteria;
  criteria.mStart
      = Linkage::Criteria::Target(leaf, Linkage::Criteria::UPSTREAM, false);

  auto linkage = Linkage::create(criteria, "upstream_linkage");

  ASSERT_NE(linkage, nullptr);
  EXPECT_GE(linkage->getNumBodyNodes(), 1u);
  EXPECT_TRUE(linkage->hasBodyNode(leaf));
}

//==============================================================================
TEST(LinkageTests, CloneLinkage)
{
  auto skel = createChainSkeleton(4);
  auto* start = skel->getBodyNode(0);
  auto* end = skel->getBodyNode(2);

  Linkage::Criteria criteria(start, end);
  auto linkage = Linkage::create(criteria, "original");

  auto clone = linkage->cloneLinkage("cloned");
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getName(), "cloned");
  EXPECT_EQ(clone->getNumBodyNodes(), linkage->getNumBodyNodes());
  EXPECT_EQ(clone->getNumDofs(), linkage->getNumDofs());

  auto clone2 = linkage->cloneLinkage();
  EXPECT_EQ(clone2->getName(), "original");
}

//==============================================================================
TEST(LinkageTests, CloneMetaSkeleton)
{
  auto skel = createChainSkeleton(3);
  auto* start = skel->getBodyNode(0);
  auto* end = skel->getBodyNode(2);

  Linkage::Criteria criteria(start, end);
  auto linkage = Linkage::create(criteria, "linkage");

  MetaSkeletonPtr clone = linkage->cloneMetaSkeleton("meta_clone");
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->getName(), "meta_clone");
}

//==============================================================================
TEST(LinkageTests, IsAssembled)
{
  auto skel = createChainSkeleton(4);
  auto* start = skel->getBodyNode(0);
  auto* end = skel->getBodyNode(3);

  Linkage::Criteria criteria(start, end);
  auto linkage = Linkage::create(criteria, "linkage");

  EXPECT_TRUE(linkage->isAssembled());
}

//==============================================================================
TEST(LinkageTests, SatisfyCriteria)
{
  auto skel = createChainSkeleton(5);
  auto* start = skel->getBodyNode(1);
  auto* end = skel->getBodyNode(3);

  Linkage::Criteria criteria(start, end);
  auto linkage = Linkage::create(criteria, "linkage");

  std::size_t initialCount = linkage->getNumBodyNodes();

  linkage->satisfyCriteria();
  EXPECT_EQ(linkage->getNumBodyNodes(), initialCount);
}

//==============================================================================
TEST(LinkageTests, Reassemble)
{
  auto skel = createChainSkeleton(4);
  auto* start = skel->getBodyNode(0);
  auto* end = skel->getBodyNode(2);

  Linkage::Criteria criteria(start, end);
  auto linkage = Linkage::create(criteria, "linkage");

  linkage->reassemble();
  EXPECT_TRUE(linkage->isAssembled());
}

//==============================================================================
TEST(LinkageTests, LinkageWithTerminals)
{
  auto skel = createChainSkeleton(5);
  auto* start = skel->getBodyNode(0);
  auto* terminal = skel->getBodyNode(2);

  Linkage::Criteria criteria;
  criteria.mStart
      = Linkage::Criteria::Target(start, Linkage::Criteria::DOWNSTREAM, false);
  criteria.mTerminals.push_back(Linkage::Criteria::Terminal(terminal, true));

  auto linkage = Linkage::create(criteria, "linkage_with_terminal");

  ASSERT_NE(linkage, nullptr);
  EXPECT_TRUE(linkage->hasBodyNode(start));
  EXPECT_TRUE(linkage->hasBodyNode(terminal));
  EXPECT_FALSE(linkage->hasBodyNode(skel->getBodyNode(4)));
}

//==============================================================================
TEST(LinkageTests, LinkageExcludePolicy)
{
  auto skel = createChainSkeleton(4);
  auto* start = skel->getBodyNode(0);
  auto* target = skel->getBodyNode(2);

  Linkage::Criteria criteria;
  criteria.mStart
      = Linkage::Criteria::Target(start, Linkage::Criteria::INCLUDE, false);
  criteria.mTargets.push_back(
      Linkage::Criteria::Target(target, Linkage::Criteria::EXCLUDE, false));

  auto linkage = Linkage::create(criteria, "exclude_linkage");

  ASSERT_NE(linkage, nullptr);
  EXPECT_TRUE(linkage->hasBodyNode(start));
  EXPECT_FALSE(linkage->hasBodyNode(target));
}

//==============================================================================
TEST(LinkageTests, ReferentialSkeletonProperties)
{
  auto skel = createChainSkeleton(5);

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "ref_skel_test");

  EXPECT_EQ(linkage->getNumSkeletons(), 1u);
  EXPECT_TRUE(linkage->hasSkeleton(skel.get()));

  auto lockable = linkage->getLockableReference();
  ASSERT_NE(lockable, nullptr);
}

//==============================================================================
TEST(LinkageTests, GetBodyNodeByIndex)
{
  auto skel = createChainSkeleton(4);

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  for (std::size_t i = 0; i < linkage->getNumBodyNodes(); ++i) {
    auto* body = linkage->getBodyNode(i);
    ASSERT_NE(body, nullptr);
    EXPECT_EQ(linkage->getIndexOf(body), i);
  }
}

//==============================================================================
TEST(LinkageTests, GetJointByIndex)
{
  auto skel = createChainSkeleton(4);

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  for (std::size_t i = 0; i < linkage->getNumJoints(); ++i) {
    auto* joint = linkage->getJoint(i);
    ASSERT_NE(joint, nullptr);
    EXPECT_EQ(linkage->getIndexOf(joint), i);
  }
}

//==============================================================================
TEST(LinkageTests, GetDofByIndex)
{
  auto skel = createChainSkeleton(4);

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  for (std::size_t i = 0; i < linkage->getNumDofs(); ++i) {
    auto* dof = linkage->getDof(i);
    ASSERT_NE(dof, nullptr);
    EXPECT_EQ(linkage->getIndexOf(dof), i);
  }
}

//==============================================================================
TEST(LinkageTests, MassComputation)
{
  auto skel = createChainSkeleton(3);

  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    Inertia inertia;
    inertia.setMass(1.0);
    skel->getBodyNode(i)->setInertia(inertia);
  }

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  double mass = linkage->getMass();
  EXPECT_DOUBLE_EQ(mass, 3.0);
}

//==============================================================================
TEST(LinkageTests, PositionOperations)
{
  auto skel = createChainSkeleton(3);

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  Eigen::VectorXd positions(linkage->getNumDofs());
  positions.setOnes();
  linkage->setPositions(positions);

  Eigen::VectorXd result = linkage->getPositions();
  EXPECT_EQ(result.size(), static_cast<int>(linkage->getNumDofs()));

  for (int i = 0; i < result.size(); ++i) {
    EXPECT_DOUBLE_EQ(result[i], 1.0);
  }

  linkage->resetPositions();
  result = linkage->getPositions();
  for (int i = 0; i < result.size(); ++i) {
    EXPECT_DOUBLE_EQ(result[i], 0.0);
  }
}

//==============================================================================
TEST(LinkageTests, VelocityOperations)
{
  auto skel = createChainSkeleton(3);

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  Eigen::VectorXd velocities(linkage->getNumDofs());
  velocities.setConstant(2.0);
  linkage->setVelocities(velocities);

  Eigen::VectorXd result = linkage->getVelocities();
  for (int i = 0; i < result.size(); ++i) {
    EXPECT_DOUBLE_EQ(result[i], 2.0);
  }

  linkage->resetVelocities();
  result = linkage->getVelocities();
  for (int i = 0; i < result.size(); ++i) {
    EXPECT_DOUBLE_EQ(result[i], 0.0);
  }
}

//==============================================================================
TEST(LinkageTests, JacobianComputation)
{
  auto skel = createChainSkeleton(3);

  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    Inertia inertia;
    inertia.setMass(1.0);
    skel->getBodyNode(i)->setInertia(inertia);
  }

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  auto* endBody = linkage->getBodyNode(linkage->getNumBodyNodes() - 1);
  ASSERT_NE(endBody, nullptr);

  math::Jacobian J = linkage->getJacobian(endBody);
  EXPECT_EQ(J.rows(), 6);
  EXPECT_EQ(J.cols(), static_cast<int>(linkage->getNumDofs()));
}

//==============================================================================
TEST(LinkageTests, COMComputation)
{
  auto skel = createChainSkeleton(3);

  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    Inertia inertia;
    inertia.setMass(1.0);
    skel->getBodyNode(i)->setInertia(inertia);
  }

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  Eigen::Vector3d com = linkage->getCOM();
  EXPECT_FALSE(std::isnan(com[0]));
  EXPECT_FALSE(std::isnan(com[1]));
  EXPECT_FALSE(std::isnan(com[2]));
}

//==============================================================================
TEST(LinkageTests, EnergyComputation)
{
  auto skel = createChainSkeleton(3);

  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    Inertia inertia;
    inertia.setMass(1.0);
    skel->getBodyNode(i)->setInertia(inertia);
  }

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  Eigen::VectorXd velocities(linkage->getNumDofs());
  velocities.setConstant(1.0);
  linkage->setVelocities(velocities);

  double KE = linkage->computeKineticEnergy();
  EXPECT_GT(KE, 0.0);

  double PE = linkage->computePotentialEnergy();
  (void)PE;
}

//==============================================================================
TEST(LinkageTests, ClearForces)
{
  auto skel = createChainSkeleton(3);

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  linkage->clearExternalForces();
  linkage->clearInternalForces();
}

//==============================================================================
TEST(LinkageTests, MassMatrix)
{
  auto skel = createChainSkeleton(3);

  for (std::size_t i = 0; i < skel->getNumBodyNodes(); ++i) {
    Inertia inertia;
    inertia.setMass(1.0);
    inertia.setMoment(0.1, 0.1, 0.1, 0.0, 0.0, 0.0);
    skel->getBodyNode(i)->setInertia(inertia);
  }

  Linkage::Criteria criteria;
  criteria.mStart = Linkage::Criteria::Target(
      skel->getBodyNode(0), Linkage::Criteria::DOWNSTREAM, false);

  auto linkage = Linkage::create(criteria, "linkage");

  const Eigen::MatrixXd& M = linkage->getMassMatrix();
  EXPECT_EQ(M.rows(), static_cast<int>(linkage->getNumDofs()));
  EXPECT_EQ(M.cols(), static_cast<int>(linkage->getNumDofs()));

  const Eigen::MatrixXd& Minv = linkage->getInvMassMatrix();
  EXPECT_EQ(Minv.rows(), static_cast<int>(linkage->getNumDofs()));
  EXPECT_EQ(Minv.cols(), static_cast<int>(linkage->getNumDofs()));
}
