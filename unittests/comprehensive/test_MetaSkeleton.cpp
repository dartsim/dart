/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <iostream>
#include <gtest/gtest.h>
#include "TestHelpers.hpp"

#include "dart/common/sub_ptr.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/utils/SkelParser.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

//==============================================================================
std::vector<common::Uri> getFileList()
{
  std::vector<common::Uri> fileList;
  fileList.push_back("dart://sample/skel/test/chainwhipa.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/single_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/double_pendulum_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_revolute_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_eulerxyz_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_20.skel");
  fileList.push_back("dart://sample/skel/test/serial_chain_ball_joint_40.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/simple_tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_euler_joint.skel");
  fileList.push_back("dart://sample/skel/test/tree_structure_ball_joint.skel");
  fileList.push_back("dart://sample/skel/fullbody1.skel");

  return fileList;
}

//==============================================================================
std::vector<SkeletonPtr> getSkeletons()
{
  const auto fileList = getFileList();

  std::vector<WorldPtr> worlds;
  for(std::size_t i=0; i<fileList.size(); ++i)
    worlds.push_back(utils::SkelParser::readWorld(fileList[i]));

  std::vector<SkeletonPtr> skeletons;
  for(std::size_t i=0; i<worlds.size(); ++i)
  {
    WorldPtr world = worlds[i];
    for(std::size_t j=0; j<world->getNumSkeletons(); ++j)
      skeletons.push_back(world->getSkeleton(j));
  }

  return skeletons;
}

//==============================================================================
TEST(MetaSkeleton, Referential)
{
  std::vector<SkeletonPtr> skeletons = getSkeletons();

#ifndef NDEBUG // Debug mode
  std::size_t numIterations = 1;
#else // Release mode
  std::size_t numIterations = 20;
#endif

  for(std::size_t i=0; i<skeletons.size(); ++i)
  {
    SkeletonPtr skeleton = skeletons[i];

    const auto& skelJoints = skeleton->getJoints();
    EXPECT_TRUE(skeleton->getNumJoints() == skelJoints.size());
    for(auto* joint : skelJoints)
      EXPECT_TRUE(skeleton->hasJoint(joint));

    for(std::size_t j=0; j<skeleton->getNumTrees(); ++j)
    {
      BranchPtr tree = Branch::create(skeleton->getRootBodyNode(j));

      const std::vector<BodyNode*>& skelBns = skeleton->getTreeBodyNodes(j);
      EXPECT_TRUE(tree->getNumBodyNodes() == skelBns.size());
      for(BodyNode* bn : skelBns)
      {
        EXPECT_FALSE(tree->getIndexOf(bn) == INVALID_INDEX);
        EXPECT_TRUE(tree->getBodyNode(tree->getIndexOf(bn)) == bn);
        EXPECT_TRUE(skeleton->hasBodyNode(bn));
      }

      const std::vector<DegreeOfFreedom*>& skelDofs = skeleton->getTreeDofs(j);
      EXPECT_TRUE(tree->getNumDofs() == skelDofs.size());
      for(DegreeOfFreedom* dof : skelDofs)
      {
        EXPECT_FALSE(tree->getIndexOf(dof) == INVALID_INDEX);
        EXPECT_TRUE(tree->getDof(tree->getIndexOf(dof)) == dof);
      }

      Eigen::VectorXd q = tree->getPositions();
      Eigen::VectorXd dq = tree->getVelocities();
      Eigen::VectorXd ddq = tree->getAccelerations();

      for(std::size_t k=0; k<numIterations; ++k)
      {
        for(int r=0; r<q.size(); ++r)
        {
          q[r] = math::Random::uniform<double>(-10, 10);
          dq[r] = math::Random::uniform<double>(-10, 10);
          ddq[r] = math::Random::uniform<double>(-10, 10);
        }

        tree->setPositions(q);
        tree->setVelocities(dq);
        tree->setAccelerations(ddq);

        EXPECT_TRUE( equals(q, tree->getPositions(), 0.0) );
        EXPECT_TRUE( equals(dq, tree->getVelocities(), 0.0) );
        EXPECT_TRUE( equals(ddq, tree->getAccelerations(), 0.0) );

        const Eigen::MatrixXd& skelMassMatrix = skeleton->getMassMatrix();
        const Eigen::MatrixXd& treeMassMatrix = tree->getMassMatrix();

        const Eigen::MatrixXd& skelAugM = skeleton->getAugMassMatrix();
        const Eigen::MatrixXd& treeAugM = tree->getAugMassMatrix();

        const Eigen::MatrixXd& skelInvM = skeleton->getInvMassMatrix();
        const Eigen::MatrixXd& treeInvM = tree->getInvMassMatrix();

        const Eigen::MatrixXd& skelInvAugM = skeleton->getInvAugMassMatrix();
        const Eigen::MatrixXd& treeInvAugM = tree->getInvAugMassMatrix();

        const Eigen::VectorXd& skelCvec = skeleton->getCoriolisForces();
        const Eigen::VectorXd& treeCvec = tree->getCoriolisForces();

        const Eigen::VectorXd& skelFg = skeleton->getGravityForces();
        const Eigen::VectorXd& treeFg = tree->getGravityForces();

        const Eigen::VectorXd& skelCg = skeleton->getCoriolisAndGravityForces();
        const Eigen::VectorXd& treeCg = tree->getCoriolisAndGravityForces();

        const Eigen::VectorXd& skelFext = skeleton->getExternalForces();
        const Eigen::VectorXd& treeFext = tree->getExternalForces();

        const Eigen::VectorXd& skelFc = skeleton->getConstraintForces();
        const Eigen::VectorXd& treeFc = tree->getConstraintForces();

        const std::size_t nDofs = tree->getNumDofs();
        for(std::size_t r1=0; r1<nDofs; ++r1)
        {
          const std::size_t sr1 = tree->getDof(r1)->getIndexInSkeleton();
          for(std::size_t r2=0; r2<nDofs; ++r2)
          {
            const std::size_t sr2 = tree->getDof(r2)->getIndexInSkeleton();

            EXPECT_TRUE( skelMassMatrix(sr1,sr2) == treeMassMatrix(r1,r2) );
            EXPECT_TRUE( skelAugM(sr1,sr2) == treeAugM(r1,r2) );
            EXPECT_TRUE( skelInvM(sr1,sr2) == treeInvM(r1,r2) );
            EXPECT_TRUE( skelInvAugM(sr1,sr2) == treeInvAugM(r1,r2) );
          }

          EXPECT_TRUE( skelCvec[sr1] == treeCvec[r1] );
          EXPECT_TRUE( skelFg[sr1]   == treeFg[r1] );
          EXPECT_TRUE( skelCg[sr1]   == treeCg[r1] );
          EXPECT_TRUE( skelFext[sr1] == treeFext[r1] );
          EXPECT_TRUE( skelFext[sr1] == treeFext[r1] );
          EXPECT_TRUE( skelFc[sr1]   == treeFc[r1] );
        }

        const std::size_t numBodyNodes = tree->getNumBodyNodes();
        for(std::size_t m=0; m<numBodyNodes; ++m)
        {
          const BodyNode* bn = tree->getBodyNode(m);
          const Eigen::MatrixXd Jtree = tree->getJacobian(bn);
          const Eigen::MatrixXd Jskel = skeleton->getJacobian(bn);

          for(std::size_t r2=0; r2<nDofs; ++r2)
          {
            const std::size_t sr2 = tree->getDof(r2)->getIndexInSkeleton();
            for(std::size_t r1=0; r1<6; ++r1)
            {
              EXPECT_TRUE( Jtree(r1,r2) == Jskel(r1, sr2) );
            }
          }
        }
      }
    }
  }
}

//==============================================================================
template <class JointType = RevoluteJoint>
BodyNode* addBodyNode(BodyNode* bn, const std::string& name)
{
  BodyNode* result = bn->createChildJointAndBodyNodePair<JointType>().second;
  result->setName(name);
  return result;
}

//==============================================================================
SkeletonPtr constructLinkageTestSkeleton()
{
  SkeletonPtr skel = Skeleton::create();
  BodyNode* bn = skel->createJointAndBodyNodePair<RevoluteJoint>().second;
  bn->setName("c1b1");
  bn = addBodyNode<FreeJoint>(bn, "c1b2");

  BodyNode* c1b3 = addBodyNode(bn, "c1b3");
  bn = addBodyNode(c1b3, "c2b1");
  bn = addBodyNode(bn, "c2b2");
  addBodyNode(bn, "c2b3");

  bn = addBodyNode(c1b3, "c3b1");
  bn = addBodyNode(bn, "c3b2");
  BodyNode* c3b3 = addBodyNode(bn, "c3b3");
  bn = addBodyNode(c3b3, "c4b1");
  bn = addBodyNode(bn, "c4b2");
  addBodyNode(bn, "c4b3");
  addBodyNode(c3b3, "c3b4");

  bn = addBodyNode(c1b3, "c5b1");
  addBodyNode(bn, "c5b2");

  return skel;
}

//==============================================================================
void checkForBodyNodes(
    std::size_t& /*count*/,
    const ReferentialSkeletonPtr& /*refSkel*/,
    const SkeletonPtr& /*skel*/)
{
  // Do nothing
}

//==============================================================================
// Variadic function for testing a ReferentialSkeleton for a series of BodyNode
// names
template <typename ... Args>
void checkForBodyNodes(
    std::size_t& count,
    const ReferentialSkeletonPtr& refSkel,
    const SkeletonPtr& skel,
    const std::string& name,
    Args ... args)
{
  bool contains = refSkel->getIndexOf(skel->getBodyNode(name)) != INVALID_INDEX;
  EXPECT_TRUE(contains);
  if(!contains)
  {
    dtmsg << "The ReferentialSkeleton [" << refSkel->getName() << "] does NOT "
          << "contain the BodyNode [" << name << "] of the Skeleton ["
          << skel->getName() << "]\n";
  }

  ++count;
  checkForBodyNodes(count, refSkel, skel, args...);
}

//==============================================================================
template <typename ... Args>
std::size_t checkForBodyNodes(
    const ReferentialSkeletonPtr& refSkel,
    const SkeletonPtr& skel,
    bool checkCount,
    Args ... args)
{
  std::size_t count = 0;
  checkForBodyNodes(count, refSkel, skel, args...);

  if(checkCount)
  {
    bool countValid = (count == refSkel->getNumBodyNodes());
    EXPECT_TRUE(countValid);
    if(!countValid)
    {
      dtmsg << "The number of BodyNodes checked for [" << count << "] "
            << "does not equal the number [" << refSkel->getNumBodyNodes()
            << "] in the ReferentialSkeleton [" << refSkel->getName() << "]\n";
    }
  }

  return count;
}

//==============================================================================
void checkLinkageJointConsistency(const ReferentialSkeletonPtr& refSkel)
{
  EXPECT_TRUE(refSkel->getNumBodyNodes() == refSkel->getNumJoints());

  // Linkages should have the property:
  // getJoint(i) == getBodyNode(i)->getParentJoint()
  for(std::size_t i=0; i < refSkel->getNumJoints(); ++i)
  {
    EXPECT_EQ(refSkel->getJoint(i), refSkel->getBodyNode(i)->getParentJoint());
    EXPECT_EQ(refSkel->getIndexOf(refSkel->getJoint(i)), i);
  }
}

//==============================================================================
TEST(MetaSkeleton, Linkage)
{
  // Test a variety of uses of Linkage::Criteria
  SkeletonPtr skel = constructLinkageTestSkeleton();

  BranchPtr subtree = Branch::create(skel->getBodyNode("c3b3"), "subtree");
  checkForBodyNodes(subtree, skel, true,
                    "c3b3", "c3b4", "c4b1", "c4b2", "c4b3");

  ChainPtr midchain = Chain::create(skel->getBodyNode("c1b3"),
                 skel->getBodyNode("c3b4"), "midchain");
  checkForBodyNodes(midchain, skel, true, "c3b1", "c3b2", "c3b3");
  checkLinkageJointConsistency(midchain);

  Linkage::Criteria criteria;
  criteria.mStart = skel->getBodyNode("c5b2");
  criteria.mTargets.push_back(
        Linkage::Criteria::Target(skel->getBodyNode("c4b3")));
  LinkagePtr path = Linkage::create(criteria, "path");
  checkForBodyNodes(path, skel, true, "c5b2", "c5b1", "c1b3", "c3b1", "c3b2",
                                      "c3b3", "c4b1", "c4b2", "c4b3");
  checkLinkageJointConsistency(path);

  skel->getBodyNode(0)->copyTo(nullptr);
  criteria.mTargets.clear();
  criteria.mStart = skel->getBodyNode("c3b1");
  criteria.mStart.mPolicy = Linkage::Criteria::UPSTREAM;
  criteria.mTargets.push_back(
        Linkage::Criteria::Target(skel->getBodyNode("c3b1(1)"),
                                  Linkage::Criteria::UPSTREAM));

  LinkagePtr combinedTreeBases = Linkage::create(criteria, "combinedTreeBases");
  checkForBodyNodes(combinedTreeBases, skel, true,
                    "c3b1",    "c1b3",    "c2b1",    "c2b2",    "c2b3",
                    "c3b1(1)", "c1b3(1)", "c2b1(1)", "c2b2(1)", "c2b3(1)",
                    "c5b1",    "c5b2",    "c1b2",    "c1b1",
                    "c5b1(1)", "c5b2(1)", "c1b2(1)", "c1b1(1)");
  checkLinkageJointConsistency(combinedTreeBases);

  SkeletonPtr skel2 = skel->getBodyNode(0)->copyAs("skel2");
  criteria.mTargets.clear();
  criteria.mTargets.push_back(
        Linkage::Criteria::Target(skel2->getBodyNode("c3b1"),
                                  Linkage::Criteria::UPSTREAM));
  LinkagePtr combinedSkelBases = Linkage::create(criteria, "combinedSkelBases");
  std::size_t count = 0;
  count += checkForBodyNodes(combinedSkelBases, skel, false,
                             "c3b1", "c1b3", "c2b1", "c2b2", "c2b3",
                             "c5b1", "c5b2", "c1b2", "c1b1");
  count += checkForBodyNodes(combinedSkelBases, skel2, false,
                             "c3b1", "c1b3", "c2b1", "c2b2", "c2b3",
                             "c5b1", "c5b2", "c1b2", "c1b1");
  EXPECT_TRUE( count == combinedSkelBases->getNumBodyNodes() );

  ChainPtr downstreamFreeJoint = Chain::create(skel->getBodyNode("c1b1"),
      skel->getBodyNode("c1b3"), Chain::IncludeBoth, "downstreamFreeJoint");
  checkForBodyNodes(downstreamFreeJoint, skel, true, "c1b1");
  checkLinkageJointConsistency(downstreamFreeJoint);

  ChainPtr emptyChain = Chain::create(skel->getBodyNode("c1b1"),
      skel->getBodyNode("c1b3"), "emptyChain");
  checkForBodyNodes(emptyChain, skel, true);
  checkLinkageJointConsistency(emptyChain);

  ChainPtr chainFromNull = Chain::create(nullptr, skel->getBodyNode("c1b2"),
                                         "chainFromNull");
  checkForBodyNodes(chainFromNull, skel, true, "c1b1");
  checkLinkageJointConsistency(chainFromNull);

  ChainPtr upstreamFreeJoint = Chain::create(skel->getBodyNode("c1b3"),
                          skel->getBodyNode("c1b1"), "upstreamFreeJoint");
  checkForBodyNodes(upstreamFreeJoint, skel, true, "c1b3", "c1b2");
  checkLinkageJointConsistency(upstreamFreeJoint);

  // Using nullptr as the target should bring us towards the root of the tree
  ChainPtr upTowardsRoot =
      Chain::create(skel->getBodyNode("c1b3"), nullptr, "upTowardsRoot");
  checkForBodyNodes(upTowardsRoot, skel, true, "c1b3", "c1b2");
  checkLinkageJointConsistency(upTowardsRoot);

  criteria.mTargets.clear();
  criteria.mTargets.push_back(skel->getBodyNode("c4b3"));
  criteria.mStart = skel->getBodyNode("c1b3");
  criteria.mTerminals.push_back(skel->getBodyNode("c3b2"));
  LinkagePtr terminatedLinkage = Linkage::create(criteria, "terminatedLinkage");
  checkForBodyNodes(terminatedLinkage, skel, true,
                    "c1b3", "c3b1", "c3b2");
  checkLinkageJointConsistency(terminatedLinkage);

  criteria.mStart = skel->getBodyNode("c1b1");
  criteria.mStart.mPolicy = Linkage::Criteria::DOWNSTREAM;
  criteria.mTargets.clear();
  criteria.mTerminals.clear();
  criteria.mTerminals.push_back(
        Linkage::Criteria::Terminal(skel->getBodyNode("c2b1"), false));
  criteria.mTerminals.push_back(skel->getBodyNode("c3b3"));
  LinkagePtr terminatedSubtree = Linkage::create(criteria, "terminatedSubtree");
  checkForBodyNodes(terminatedSubtree, skel, true,
                    "c1b1", "c1b2", "c1b3", "c5b1",
                    "c5b2", "c3b1", "c3b2", "c3b3");
  checkLinkageJointConsistency(terminatedSubtree);

  criteria.mStart.mPolicy = Linkage::Criteria::UPSTREAM;
  criteria.mStart.mNode = skel->getBodyNode("c3b1");
  LinkagePtr terminatedUpstream = Linkage::create(criteria, "terminatedUpstream");
  checkForBodyNodes(terminatedUpstream, skel, true,
                    "c3b1", "c1b3", "c5b1", "c5b2", "c1b2", "c1b1");
  checkLinkageJointConsistency(terminatedUpstream);
}

//==============================================================================
TEST(MetaSkeleton, Group)
{
  SkeletonPtr skel = constructLinkageTestSkeleton();

  // Make twice as many BodyNodes in the Skeleton
  SkeletonPtr skel2 = constructLinkageTestSkeleton();
  skel2->getRootBodyNode()->moveTo(skel, nullptr);

  // Test nullptr construction
  GroupPtr nullGroup = Group::create("null_group", nullptr);
  EXPECT_EQ(nullGroup->getNumBodyNodes(), 0u);
  EXPECT_EQ(nullGroup->getNumJoints(), 0u);
  EXPECT_EQ(nullGroup->getNumDofs(), 0u);

  // Test conversion from Skeleton
  GroupPtr skel1Group = Group::create("skel1_group", skel);
  EXPECT_EQ(skel1Group->getNumBodyNodes(), skel->getNumBodyNodes());
  EXPECT_EQ(skel1Group->getNumJoints(), skel->getNumJoints());
  EXPECT_EQ(skel1Group->getNumDofs(), skel->getNumDofs());

  for(std::size_t i=0; i < skel1Group->getNumBodyNodes(); ++i)
    EXPECT_EQ(skel1Group->getBodyNode(i), skel->getBodyNode(i));

  for(std::size_t i=0; i < skel1Group->getNumJoints(); ++i)
    EXPECT_EQ(skel1Group->getJoint(i), skel->getJoint(i));

  for(std::size_t i=0; i < skel1Group->getNumDofs(); ++i)
    EXPECT_EQ(skel1Group->getDof(i), skel->getDof(i));

  // Test arbitrary Groups by plucking random BodyNodes, Joints, and
  // DegreesOfFreedom from a Skeleton.
  GroupPtr group = Group::create();
  std::vector<BodyNode*> bodyNodes;
  std::vector<Joint*> joints;
  std::vector<DegreeOfFreedom*> dofs;
  for(std::size_t i=0; i < 2*skel->getNumBodyNodes(); ++i)
  {
    std::size_t randomIndex = Random::uniform<std::size_t>(0, skel->getNumBodyNodes()-1);
    BodyNode* bn = skel->getBodyNode(randomIndex);
    if(group->addBodyNode(bn, false))
      bodyNodes.push_back(bn);

    randomIndex = Random::uniform<std::size_t>(0, skel->getNumJoints()-1);
    Joint* joint = skel->getJoint(randomIndex);
    if(group->addJoint(joint, false, false))
      joints.push_back(joint);

    randomIndex = Random::uniform<std::size_t>(0, skel->getNumDofs()-1);
    DegreeOfFreedom* dof = skel->getDof(randomIndex);
    if(group->addDof(dof, false, false))
      dofs.push_back(dof);
  }

  EXPECT_EQ(group->getNumBodyNodes(), bodyNodes.size());
  EXPECT_EQ(group->getNumJoints(), joints.size());
  EXPECT_EQ(group->getNumDofs(), dofs.size());

  for(std::size_t i=0; i < group->getNumBodyNodes(); ++i)
    EXPECT_EQ(group->getBodyNode(i), bodyNodes[i]);

  for(std::size_t i=0; i < group->getNumJoints(); ++i)
    EXPECT_EQ(group->getJoint(i), joints[i]);

  for(std::size_t i=0; i < group->getNumDofs(); ++i)
    EXPECT_EQ(group->getDof(i), dofs[i]);
}

//==============================================================================
TEST(MetaSkeleton, LockSkeletonMutexesWithLockGuard)
{
  // Test a variety of uses of Linkage::Criteria
  SkeletonPtr skel = constructLinkageTestSkeleton();

  BranchPtr subtree = Branch::create(skel->getBodyNode("c3b3"), "subtree");
  checkForBodyNodes(subtree, skel, true,
                    "c3b3", "c3b4", "c4b1", "c4b2", "c4b3");

  ChainPtr midchain = Chain::create(skel->getBodyNode("c1b3"),
                 skel->getBodyNode("c3b4"), "midchain");
  checkForBodyNodes(midchain, skel, true, "c3b1", "c3b2", "c3b3");
  checkLinkageJointConsistency(midchain);

  {
    auto mutex = subtree->getLockableReference();
    std::lock_guard<common::LockableReference> lock(*mutex);
  }

  {
    auto mutex = midchain->getLockableReference();
    std::lock_guard<common::LockableReference> lock(*mutex);
  }
}

//==============================================================================
TEST(MetaSkeleton, GetJointsAndBodyNodes)
{
  auto skelA = Skeleton::create();
  auto skelB = Skeleton::create();

  BodyNode* bodyNodeA0;
  BodyNode* bodyNodeA1;
  BodyNode* bodyNodeA2;

  BodyNode* bodyNodeB0;
  BodyNode* bodyNodeB1;
  BodyNode* bodyNodeB2;

  Joint* jointA0;
  Joint* jointA1;
  Joint* jointA2;

  Joint* jointB0;
  Joint* jointB1;
  Joint* jointB2;

  std::tie(jointA0, bodyNodeA0)
      = skelA->createJointAndBodyNodePair<FreeJoint>();
  std::tie(jointA1, bodyNodeA1)
      = skelA->createJointAndBodyNodePair<FreeJoint>(bodyNodeA0);
  std::tie(jointA2, bodyNodeA2)
      = skelA->createJointAndBodyNodePair<FreeJoint>(bodyNodeA1);

  std::tie(jointB0, bodyNodeB0)
      = skelB->createJointAndBodyNodePair<FreeJoint>();
  std::tie(jointB1, bodyNodeB1)
      = skelB->createJointAndBodyNodePair<FreeJoint>(bodyNodeB0);
  std::tie(jointB2, bodyNodeB2)
      = skelB->createJointAndBodyNodePair<FreeJoint>(bodyNodeB1);

  EXPECT_TRUE(skelA->getNumBodyNodes() == 3u);
  EXPECT_TRUE(skelB->getNumBodyNodes() == 3u);

  EXPECT_TRUE(skelA->getJoints().size() == 3u);
  EXPECT_TRUE(skelB->getJoints().size() == 3u);

  bodyNodeA0->setName("bodyNode0");
  bodyNodeA1->setName("bodyNode1");
  bodyNodeA2->setName("bodyNode2");

  bodyNodeB0->setName("bodyNode0");
  bodyNodeB1->setName("bodyNode1");
  bodyNodeB2->setName("bodyNode2");

  jointA0->setName("joint0");
  jointA1->setName("joint1");
  jointA2->setName("joint2");

  jointB0->setName("joint0");
  jointB1->setName("joint1");
  jointB2->setName("joint2");

  EXPECT_TRUE(bodyNodeA0 == skelA->getBodyNode(bodyNodeA0->getName()));
  EXPECT_TRUE(bodyNodeA1 == skelA->getBodyNode(bodyNodeA1->getName()));
  EXPECT_TRUE(bodyNodeA2 == skelA->getBodyNode(bodyNodeA2->getName()));

  EXPECT_TRUE(bodyNodeB0 == skelB->getBodyNode(bodyNodeB0->getName()));
  EXPECT_TRUE(bodyNodeB1 == skelB->getBodyNode(bodyNodeB1->getName()));
  EXPECT_TRUE(bodyNodeB2 == skelB->getBodyNode(bodyNodeB2->getName()));

  EXPECT_TRUE(skelA->getBodyNodes("wrong name").empty());
  EXPECT_TRUE(skelB->getBodyNodes("wrong name").empty());

  EXPECT_TRUE(skelA->getBodyNodes(bodyNodeA0->getName()).size() == 1u);
  EXPECT_TRUE(skelA->getBodyNodes(bodyNodeA1->getName()).size() == 1u);
  EXPECT_TRUE(skelA->getBodyNodes(bodyNodeA2->getName()).size() == 1u);

  EXPECT_TRUE(skelB->getBodyNodes(bodyNodeB0->getName()).size() == 1u);
  EXPECT_TRUE(skelB->getBodyNodes(bodyNodeB1->getName()).size() == 1u);
  EXPECT_TRUE(skelB->getBodyNodes(bodyNodeB2->getName()).size() == 1u);

  EXPECT_TRUE(jointA0 == skelA->getJoint(jointA0->getName()));
  EXPECT_TRUE(jointA1 == skelA->getJoint(jointA1->getName()));
  EXPECT_TRUE(jointA2 == skelA->getJoint(jointA2->getName()));

  EXPECT_TRUE(jointB0 == skelB->getJoint(jointB0->getName()));
  EXPECT_TRUE(jointB1 == skelB->getJoint(jointB1->getName()));
  EXPECT_TRUE(jointB2 == skelB->getJoint(jointB2->getName()));

  EXPECT_TRUE(skelA->getJoints("wrong name").empty());
  EXPECT_TRUE(skelB->getJoints("wrong name").empty());

  EXPECT_TRUE(skelA->getJoints(jointA0->getName()).size() == 1u);
  EXPECT_TRUE(skelA->getJoints(jointA1->getName()).size() == 1u);
  EXPECT_TRUE(skelA->getJoints(jointA2->getName()).size() == 1u);

  EXPECT_TRUE(skelB->getJoints(jointB0->getName()).size() == 1u);
  EXPECT_TRUE(skelB->getJoints(jointB1->getName()).size() == 1u);
  EXPECT_TRUE(skelB->getJoints(jointB2->getName()).size() == 1u);

  auto group = Group::create();
  group->addBodyNode(bodyNodeA0);
  group->addBodyNode(bodyNodeB0);
  group->addJoint(jointA0);
  group->addJoint(jointB0);
  EXPECT_TRUE(group->getJoints("wrong name").empty());
  EXPECT_TRUE(group->getBodyNodes("wrong name").empty());
  EXPECT_TRUE(group->getBodyNode("bodyNode0") == bodyNodeA0
              || group->getBodyNode("bodyNode0") == bodyNodeB0);
  EXPECT_TRUE(group->getJoint("joint0") == jointA0
              || group->getJoint("joint0") == jointB0);
  EXPECT_EQ(group->getBodyNodes("bodyNode0").size(), 2u);
  EXPECT_EQ(group->getBodyNodes("bodyNode1").size(), 0u);
  EXPECT_EQ(group->getBodyNodes("bodyNode2").size(), 0u);
  EXPECT_EQ(group->getJoints("joint0").size(), 2u);
  EXPECT_EQ(group->getJoints("joint1").size(), 0u);
  EXPECT_EQ(group->getJoints("joint2").size(), 0u);
}

TEST(MetaSkeleton, ReferentialSkeletonHoldsStrongReferencesOfBodyNodes)
{
  // Create skeletons
  SkeletonPtr skel1 = constructLinkageTestSkeleton();
  SkeletonPtr skel2 = constructLinkageTestSkeleton();
  EXPECT_TRUE(skel1->getNumBodyNodes() > 1);
  EXPECT_TRUE(skel2->getNumBodyNodes() > 2);

  // Create group from the skeletons
  GroupPtr group = Group::create();
  group->addBodyNode(skel1->getBodyNode(0));
  group->addBodyNode(skel2->getBodyNode(1));
  EXPECT_EQ(group->getNumBodyNodes(), 2);

  const std::string name1 = group->getBodyNode(0)->getName();
  const std::string name2 = group->getBodyNode(1)->getName();
  EXPECT_EQ(group->getBodyNode(0)->getName(), name1);
  EXPECT_EQ(group->getBodyNode(1)->getName(), name2);

  // Release the ownership of skel1 and skel2
  skel1.reset();
  skel2.reset();

  // However, bodyNodes (and the Skeletons) are still alive because the group is
  // alive
  EXPECT_EQ(group->getBodyNode(0)->getName(), name1);
  EXPECT_EQ(group->getBodyNode(1)->getName(), name2);
}

//==============================================================================
template <typename RefSkelType>
void testReferentialSkeletonClone(
    const RefSkelType& skel, const RefSkelType& skelClone)
{
  EXPECT_NE(skel, nullptr);
  EXPECT_NE(skelClone, nullptr);
  EXPECT_EQ(skel->getName(), skelClone->getName());
  EXPECT_EQ(skel->getNumBodyNodes(), skelClone->getNumBodyNodes());
  EXPECT_EQ(skel->getNumJoints(), skelClone->getNumJoints());
  EXPECT_EQ(skel->getNumDofs(), skelClone->getNumDofs());

  // The skeleton instance of group clone should be different from the original,
  // but the properties should be the same.
  for (std::size_t i = 0u; i < skel->getNumBodyNodes(); ++i)
  {
    const auto& bodyNode = skel->getBodyNode(i);
    const auto& skel = bodyNode->getSkeleton();

    const auto& bodyNodeClone = skelClone->getBodyNode(i);
    const auto& skelClone = bodyNodeClone->getSkeleton();

    EXPECT_NE(skel, skelClone);
    EXPECT_EQ(skel->getNumBodyNodes(), skelClone->getNumBodyNodes());
    EXPECT_EQ(skel->getNumJoints(), skelClone->getNumJoints());
    EXPECT_EQ(skel->getNumDofs(), skelClone->getNumDofs());
  }
  for (std::size_t i = 0u; i < skel->getNumJoints(); ++i)
  {
    const auto& joint = skel->getJoint(i);
    const auto& skel = joint->getSkeleton();

    const auto& jointClone = skelClone->getJoint(i);
    const auto& skelClone = jointClone->getSkeleton();

    EXPECT_NE(skel, skelClone);
    EXPECT_EQ(skel->getNumBodyNodes(), skelClone->getNumBodyNodes());
    EXPECT_EQ(skel->getNumJoints(), skelClone->getNumJoints());
    EXPECT_EQ(skel->getNumDofs(), skelClone->getNumDofs());
  }
}

//==============================================================================
TEST(MetaSkeleton, CloneReferentialSkeletons)
{
  SkeletonPtr skel1 = constructLinkageTestSkeleton();
  SkeletonPtr skel2 = constructLinkageTestSkeleton();

  // Group
  GroupPtr group = Group::create();
  group->addBodyNode(skel1->getBodyNode(0));
  group->addBodyNode(skel2->getBodyNode(0));
  group->addJoint(skel1->getJoint(1));
  group->addJoint(skel2->getJoint(1));
  GroupPtr groupClone = std::dynamic_pointer_cast<Group>(
      group->cloneMetaSkeleton());
  testReferentialSkeletonClone(group, groupClone);

  // Linkage
  Linkage::Criteria criteria;
  criteria.mStart = skel1->getBodyNode("c5b2");
  criteria.mTargets.push_back(
      Linkage::Criteria::Target(skel1->getBodyNode("c4b3")));
  LinkagePtr linkage = Linkage::create(criteria);
  LinkagePtr linkageClone = std::dynamic_pointer_cast<Linkage>(
      linkage->cloneMetaSkeleton());
  testReferentialSkeletonClone(linkage, linkageClone);

  // Chain
  ChainPtr chain = Chain::create(
      skel1->getBodyNode("c1b3"), skel1->getBodyNode("c3b4"), "midchain");
  ChainPtr chainClone = std::dynamic_pointer_cast<Chain>(
      chain->cloneMetaSkeleton());
  testReferentialSkeletonClone(chain, chainClone);

  // Branch
  BranchPtr branch = Branch::create(skel1->getBodyNode("c3b3"), "subtree");
  checkForBodyNodes(
      branch, skel1, true, "c3b3", "c3b4", "c4b1", "c4b2", "c4b3");
  BranchPtr branchClone = std::dynamic_pointer_cast<Branch>(
      branch->cloneMetaSkeleton());
  testReferentialSkeletonClone(branch, branchClone);
}
