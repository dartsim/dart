/*
 * Copyright (c) 2011-2017, The DART development contributors
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

void constructSubtree(std::vector<BodyNode*>& _tree, BodyNode* bn)
{
  _tree.push_back(bn);
  for(std::size_t i=0; i<bn->getNumChildBodyNodes(); ++i)
    constructSubtree(_tree, bn->getChildBodyNode(i));
}

TEST(Skeleton, Restructuring)
{
  std::vector<SkeletonPtr> skeletons = getSkeletons();

#ifndef NDEBUG
  std::size_t numIterations = 10;
#else
  std::size_t numIterations = 2*skeletons.size();
#endif

  for(const auto& skeleton : skeletons)
    EXPECT_TRUE(skeleton->checkIndexingConsistency());

  // Test moves within the current Skeleton
  for(std::size_t i=0; i<numIterations; ++i)
  {
    std::size_t index = floor(math::random(0, skeletons.size()));
    index = std::min(index, skeletons.size()-1);
    SkeletonPtr skeleton = skeletons[index];
    EXPECT_TRUE(skeleton->checkIndexingConsistency());
    SkeletonPtr original = skeleton->clone();
    EXPECT_TRUE(original->checkIndexingConsistency());

    std::size_t maxNode = skeleton->getNumBodyNodes()-1;
    BodyNode* bn1 = skeleton->getBodyNode(floor(math::random(0, maxNode)));
    BodyNode* bn2 = skeleton->getBodyNode(ceil(math::random(0, maxNode)));

    if(bn1 == bn2)
    {
      --i;
      continue;
    }

    BodyNode* child = bn1->descendsFrom(bn2)? bn1 : bn2;
    BodyNode* parent = child == bn1? bn2 : bn1;

    child->moveTo(parent);

    EXPECT_TRUE(skeleton->getNumBodyNodes() == original->getNumBodyNodes());
    if(skeleton->getNumBodyNodes() == original->getNumBodyNodes())
    {
      for(std::size_t j=0; j<skeleton->getNumBodyNodes(); ++j)
      {
        // Make sure no BodyNodes have been lost or gained in translation
        std::string name = original->getBodyNode(j)->getName();
        BodyNode* bn = skeleton->getBodyNode(name);
        EXPECT_FALSE(bn == nullptr);
        if(bn)
        {
          EXPECT_TRUE(bn->getName() == name);
        }

        name = skeleton->getBodyNode(j)->getName();
        bn = original->getBodyNode(name);
        EXPECT_FALSE(bn == nullptr);
        if(bn)
        {
          EXPECT_TRUE(bn->getName() == name);
        }

        // Make sure no Joints have been lost or gained in translation
        name = original->getJoint(j)->getName();
        Joint* joint = skeleton->getJoint(name);
        EXPECT_FALSE(joint == nullptr);
        if(joint)
        {
          EXPECT_TRUE(joint->getName() == name);
        }

        name = skeleton->getJoint(j)->getName();
        joint = original->getJoint(name);
        EXPECT_FALSE(joint == nullptr);
        if(joint)
        {
          EXPECT_TRUE(joint->getName() == name);
        }
      }
    }

    EXPECT_TRUE(skeleton->getNumDofs() == original->getNumDofs());
    for(std::size_t j=0; j<skeleton->getNumDofs(); ++j)
    {
      std::string name = original->getDof(j)->getName();
      DegreeOfFreedom* dof = skeleton->getDof(name);
      EXPECT_FALSE(dof == nullptr);
      if(dof)
      {
        EXPECT_TRUE(dof->getName() == name);
      }

      name = skeleton->getDof(j)->getName();
      dof = original->getDof(name);
      EXPECT_FALSE(dof == nullptr);
      if(dof)
      {
        EXPECT_TRUE(dof->getName() == name);
      }
    }
  }

  // Test moves between Skeletons
  for(std::size_t i=0; i<numIterations; ++i)
  {
    std::size_t fromIndex = floor(math::random(0, skeletons.size()));
    fromIndex = std::min(fromIndex, skeletons.size()-1);
    SkeletonPtr fromSkel = skeletons[fromIndex];

    if(fromSkel->getNumBodyNodes() == 0)
    {
      --i;
      continue;
    }

    std::size_t toIndex = floor(math::random(0, skeletons.size()));
    toIndex = std::min(toIndex, skeletons.size()-1);
    SkeletonPtr toSkel = skeletons[toIndex];

    if(toSkel->getNumBodyNodes() == 0)
    {
      --i;
      continue;
    }

    BodyNode* childBn = fromSkel->getBodyNode(
          floor(math::random(0, fromSkel->getNumBodyNodes()-1)));
    BodyNode* parentBn = toSkel->getBodyNode(
          floor(math::random(0, toSkel->getNumBodyNodes()-1)));

    if(fromSkel == toSkel)
    {
      if(childBn == parentBn)
      {
        --i;
        continue;
      }

      if(parentBn->descendsFrom(childBn))
      {
        BodyNode* tempBn = childBn;
        childBn = parentBn;
        parentBn = tempBn;

        SkeletonPtr tempSkel = fromSkel;
        fromSkel = toSkel;
        toSkel = tempSkel;
      }
    }

    BodyNode* originalParent = childBn->getParentBodyNode();
    std::vector<BodyNode*> subtree;
    constructSubtree(subtree, childBn);

    // Move to a new Skeleton
    childBn->moveTo(parentBn);
    EXPECT_TRUE(childBn->getSkeleton()->checkIndexingConsistency());
    EXPECT_TRUE(parentBn->getSkeleton()->checkIndexingConsistency());

    // Make sure all the objects have moved
    for(std::size_t j=0; j<subtree.size(); ++j)
    {
      BodyNode* bn = subtree[j];
      EXPECT_TRUE(bn->getSkeleton() == toSkel);
    }

    // Move to the Skeleton's root while producing a new Joint type
    sub_ptr<Joint> originalJoint = childBn->getParentJoint();
    childBn->moveTo<FreeJoint>(nullptr);

    // The original parent joint should be deleted now
    EXPECT_TRUE(originalJoint == nullptr);

    // The BodyNode should now be a root node
    EXPECT_TRUE(childBn->getParentBodyNode() == nullptr);

    // The subtree should still be in the same Skeleton
    for(std::size_t j=0; j<subtree.size(); ++j)
    {
      BodyNode* bn = subtree[j];
      EXPECT_TRUE(bn->getSkeleton() == toSkel);
    }

    // Create some new Skeletons and mangle them all up

    childBn->copyTo<RevoluteJoint>(fromSkel, originalParent);

    SkeletonPtr temporary = childBn->split("temporary");
    SkeletonPtr other_temporary =
        childBn->split<PrismaticJoint>("other temporary");
    SkeletonPtr another_temporary = childBn->copyAs("another temporary");
    SkeletonPtr last_temporary = childBn->copyAs<ScrewJoint>("last temporary");

    childBn->copyTo(another_temporary->getBodyNode(
                      another_temporary->getNumBodyNodes()-1));
    childBn->copyTo<PlanarJoint>(another_temporary->getBodyNode(0));
    childBn->copyTo<TranslationalJoint>(temporary, nullptr);
    childBn->moveTo(last_temporary,
        last_temporary->getBodyNode(last_temporary->getNumBodyNodes()-1));
    childBn->moveTo<BallJoint>(last_temporary, nullptr);
    childBn->moveTo<EulerJoint>(last_temporary,
                                last_temporary->getBodyNode(0));
    childBn->changeParentJointType<FreeJoint>();

    // Test the non-recursive copying
    if(toSkel->getNumBodyNodes() > 1)
    {
      SkeletonPtr singleBodyNode =
          toSkel->getBodyNode(0)->copyAs("single", false);
      EXPECT_TRUE(singleBodyNode->getNumBodyNodes() == 1);

      std::pair<Joint*, BodyNode*> singlePair =
          toSkel->getBodyNode(0)->copyTo(nullptr, false);
      EXPECT_TRUE(singlePair.second->getNumChildBodyNodes() == 0);
    }

    // Check that the mangled Skeletons are all self-consistent
    EXPECT_TRUE(fromSkel->checkIndexingConsistency());
    EXPECT_TRUE(toSkel->checkIndexingConsistency());
    EXPECT_TRUE(temporary->checkIndexingConsistency());
    EXPECT_TRUE(other_temporary->checkIndexingConsistency());
    EXPECT_TRUE(another_temporary->checkIndexingConsistency());
    EXPECT_TRUE(last_temporary->checkIndexingConsistency());
  }
}

TEST(Skeleton, Persistence)
{
  WeakBodyNodePtr weakBnPtr;
  SoftBodyNodePtr softBnPtr;
  WeakSoftBodyNodePtr weakSoftBnPtr;
  WeakSkeletonPtr weakSkelPtr;
  {
    BodyNodePtr strongPtr;
    {
      {
        SkeletonPtr skeleton = createThreeLinkRobot(
              Eigen::Vector3d(1.0, 1.0, 1.0), DOF_X,
              Eigen::Vector3d(1.0, 1.0, 1.0), DOF_Y,
              Eigen::Vector3d(1.0, 1.0, 1.0), DOF_Z);
        weakSkelPtr = skeleton;

        // Test usability of the BodyNodePtr
        strongPtr = skeleton->getBodyNode(0);
        weakBnPtr = strongPtr;
        ConstBodyNodePtr constPtr = strongPtr;

        EXPECT_FALSE( strongPtr == nullptr );
        EXPECT_FALSE( nullptr == strongPtr );

        EXPECT_TRUE( strongPtr == skeleton->getBodyNode(0) );
        EXPECT_TRUE( skeleton->getBodyNode(0) == strongPtr );
        EXPECT_TRUE( constPtr == strongPtr );
        EXPECT_TRUE( weakBnPtr.lock() == strongPtr );

        EXPECT_FALSE( strongPtr < constPtr );
        EXPECT_FALSE( strongPtr < skeleton->getBodyNode(0) );
        EXPECT_FALSE( strongPtr < weakBnPtr.lock() );
        EXPECT_FALSE( skeleton->getBodyNode(0) < strongPtr );
        EXPECT_FALSE( weakBnPtr.lock() < strongPtr);

        EXPECT_FALSE( strongPtr > constPtr );
        EXPECT_FALSE( strongPtr > skeleton->getBodyNode(0) );
        EXPECT_FALSE( strongPtr > weakBnPtr.lock() );
        EXPECT_FALSE( skeleton->getBodyNode(0) > strongPtr );
        EXPECT_FALSE( weakBnPtr.lock() > strongPtr );

        BodyNodePtr tail = skeleton->getBodyNode(skeleton->getNumBodyNodes()-1);
        std::pair<Joint*, SoftBodyNode*> pair =
            skeleton->createJointAndBodyNodePair<RevoluteJoint, SoftBodyNode>(
              tail);

        softBnPtr = pair.second;
        weakSoftBnPtr = softBnPtr;
        WeakBodyNodePtr otherWeakPtr = weakSoftBnPtr; // Test convertability

        // Test usability of the DegreeOfFreedomPtr
        DegreeOfFreedomPtr dof = skeleton->getDof(1);
        WeakDegreeOfFreedomPtr weakdof = dof;
        ConstDegreeOfFreedomPtr const_dof = dof;
        WeakConstDegreeOfFreedomPtr const_weakdof = weakdof;
        const_weakdof = const_dof;

        EXPECT_TRUE( dof == skeleton->getDof(1) );
        EXPECT_TRUE( dof == const_dof );
        EXPECT_TRUE( weakdof.lock() == const_weakdof.lock() );
        EXPECT_TRUE( const_weakdof.lock() == skeleton->getDof(1) );
        EXPECT_TRUE( skeleton->getDof(1) == const_weakdof.lock() );

        EXPECT_FALSE( dof < const_dof );
        EXPECT_FALSE( dof < skeleton->getDof(1) );
        EXPECT_FALSE( dof < weakdof.lock() );
        EXPECT_FALSE( skeleton->getDof(1) < dof );
        EXPECT_FALSE( weakdof.lock() < dof );

        EXPECT_FALSE( dof > const_dof );
        EXPECT_FALSE( dof > skeleton->getDof(1) );
        EXPECT_FALSE( dof > weakdof.lock() );
        EXPECT_FALSE( skeleton->getDof(1) > dof );
        EXPECT_FALSE( weakdof.lock() > dof );

        dof = nullptr;
        weakdof = nullptr;
        const_dof = nullptr;
        const_weakdof = nullptr;

        EXPECT_TRUE( dof == nullptr );
        EXPECT_TRUE( nullptr == dof );
        EXPECT_TRUE( weakdof.lock() == nullptr );
        EXPECT_TRUE( nullptr == weakdof.lock() );
        EXPECT_TRUE( const_dof == nullptr );
        EXPECT_TRUE( const_weakdof.lock() == nullptr );

        EXPECT_FALSE( dof < const_dof );

        // Test usability of the JointPtr
        JointPtr joint = skeleton->getJoint(1);
        WeakJointPtr weakjoint = joint;
        ConstJointPtr const_joint = joint;
        WeakConstJointPtr const_weakjoint = const_joint;

        EXPECT_TRUE( joint == skeleton->getJoint(1) );
        EXPECT_TRUE( joint == const_joint );
        EXPECT_TRUE( weakjoint.lock() == const_weakjoint.lock() );
        EXPECT_TRUE( const_weakjoint.lock() == skeleton->getJoint(1) );

        EXPECT_FALSE( joint < const_joint );
        EXPECT_FALSE( joint < skeleton->getJoint(1) );
        EXPECT_FALSE( joint < weakjoint.lock() );
        EXPECT_FALSE( skeleton->getJoint(1) < joint );
        EXPECT_FALSE( weakjoint.lock() < joint );

        EXPECT_FALSE( joint > const_joint );
        EXPECT_FALSE( joint > skeleton->getJoint(1) );
        EXPECT_FALSE( joint > weakjoint.lock() );
        EXPECT_FALSE( skeleton->getJoint(1) > joint );
        EXPECT_FALSE( weakjoint.lock() > joint );

        joint = nullptr;
        weakjoint = nullptr;
        const_joint = nullptr;
        const_weakjoint = nullptr;

        EXPECT_TRUE( joint == nullptr );
        EXPECT_TRUE( weakjoint.lock() == nullptr );
        EXPECT_TRUE( const_joint == nullptr );
        EXPECT_TRUE( const_weakjoint.lock() == nullptr );
      }

      // The BodyNode should still be alive, because a BodyNodePtr still
      // references it
      EXPECT_FALSE(weakBnPtr.expired());

      // The Skeleton should still be alive, because a BodyNodePtr still
      // references one of its BodyNodes
      EXPECT_FALSE(weakSkelPtr.lock() == nullptr);

      // Take the BodyNode out of its Skeleton and put it into a temporary one
      strongPtr->remove();

      // The BodyNode should still be alive, because a BodyNodePtr still
      // references it
      EXPECT_FALSE(weakBnPtr.expired());

      // The Skeleton should be destroyed, because it lost the only BodyNode
      // that still had a reference
      EXPECT_TRUE(weakSkelPtr.lock() == nullptr);

      // Update the weakSkelPtr so it references the Skeleton that still exists
      weakSkelPtr = strongPtr->getSkeleton();
      EXPECT_FALSE(weakSkelPtr.lock() == nullptr);

      // Change the BodyNode that this BodyNodePtr is referencing
      strongPtr = strongPtr->getChildBodyNode(0);

      // Make sure the Skeleton is still alive. If the SkeletonPtr being used
      // by the BodyNodePtr is not swapped atomically, then this will fail,
      // which means we cannot rely on BodyNodePtr to keep BodyNodes alive.
      EXPECT_FALSE(weakSkelPtr.lock() == nullptr);
    }

    SkeletonPtr other_skeleton = createThreeLinkRobot(
          Eigen::Vector3d(1.0, 1.0, 1.0), DOF_X,
          Eigen::Vector3d(1.0, 1.0, 1.0), DOF_Y,
          Eigen::Vector3d(1.0, 1.0, 1.0), DOF_Z);
    BodyNode* tail = other_skeleton->getBodyNode(
          other_skeleton->getNumBodyNodes()-1);

    WeakConstBodyNodePtr weakParentPtr;
    {
      ConstBodyNodePtr parent = strongPtr;
      parent = parent->getParentBodyNode();
      weakParentPtr = parent;
      strongPtr->moveTo(tail);

      // The Skeleton should still be alive because 'parent' exists
      EXPECT_FALSE(weakSkelPtr.lock() == nullptr);
    }

    // Now that 'parent' is out of scope, the Skeleton should be gone
    EXPECT_TRUE(weakSkelPtr.lock() == nullptr);
    EXPECT_TRUE(weakParentPtr.lock() == nullptr);

    weakBnPtr = strongPtr;
    weakSkelPtr = strongPtr->getSkeleton();
    EXPECT_FALSE(weakBnPtr.expired());
    EXPECT_FALSE(weakSkelPtr.expired());
  }

  // softBnPtr still exists, so it should be keeping the Skeleton active
  EXPECT_FALSE(weakBnPtr.expired());

  std::weak_ptr<Skeleton> weakSkel = softBnPtr->remove();

  // Now that the SoftBodyNode which is holding the reference has been moved to
  // another Skeleton, the weakBnPtr and weakSkelPtr should disappear
  EXPECT_TRUE(weakBnPtr.expired());
  EXPECT_TRUE(weakSkelPtr.expired());

  // The WeakSoftBodyNodePtr should not have expired yet, because a strong
  // reference to its SoftBodyNode still exists
  EXPECT_FALSE(weakSoftBnPtr.expired());

  // Test the user-defined copy constructor
  SoftBodyNodePtr otherSoftBnPtr = softBnPtr;

  softBnPtr = nullptr;

  EXPECT_FALSE(weakSkel.lock() == nullptr);
  EXPECT_FALSE(weakSoftBnPtr.lock() == nullptr);

  BodyNodePtr strongPtr = otherSoftBnPtr;

  otherSoftBnPtr = nullptr;

  BodyNodePtr otherStrongPtr = strongPtr;

  strongPtr = nullptr;

  EXPECT_FALSE(weakSkel.lock() == nullptr);
  EXPECT_FALSE(weakSoftBnPtr.lock() == nullptr);

  otherStrongPtr = nullptr;

  // Now that all the strong BodyNodePtrs have been cleared, the
  // WeakSoftBodyNodePtr should also be cleared
  EXPECT_TRUE(weakSoftBnPtr.lock() == nullptr);
  EXPECT_TRUE(weakSkel.lock() == nullptr);
}

class GenericNode final : public dart::dynamics::Node,
                          public AccessoryNode<GenericNode>
{
public:

  GenericNode(BodyNode* bn, const std::string& name)
    : Node(bn), mName(name) { }

  const std::string& setName(const std::string& newName) override
  {
    mName = registerNameChange(newName);
    return mName;
  }

  const std::string& getName() const override
  {
    return mName;
  }

protected:

  Node* cloneNode(BodyNode* bn) const override
  {
    return new GenericNode(bn, mName);
  }

  std::string mName;
};

TEST(Skeleton, NodePersistence)
{
  SkeletonPtr skel = Skeleton::create();
  skel->createJointAndBodyNodePair<FreeJoint>(nullptr);

  //--------------------------------------------------------------------------
  // Testing EndEffector, which is a specialized Node type
  //--------------------------------------------------------------------------
  {
    EndEffector* manip = skel->getBodyNode(0)->createEndEffector("manip");

    // Test both methods of adding a Support to an EndEffector
    manip->createAspect<Support>();
    manip->createSupport();

    EXPECT_EQ(skel->getEndEffector("manip"), manip);
    EXPECT_EQ(skel->getEndEffector(0), manip);
//    EXPECT_EQ(skel->getBodyNode(0)->getEndEffector(0), manip);

    WeakEndEffectorPtr weakManip = manip;

    EXPECT_NE(weakManip.lock(), nullptr);

    manip->remove();

    // The Node has been removed, and no strong reference to it exists, so it
    // should be gone from the Skeleton
    EXPECT_EQ(skel->getEndEffector("manip"), nullptr);
    EXPECT_EQ(skel->getNumEndEffectors(), 0u);
//    EXPECT_EQ(skel->getBodyNode(0)->getNumEndEffectors(), 0u);

    EXPECT_EQ(weakManip.lock(), nullptr);
  }

  {
    EndEffector* manip = skel->getBodyNode(0)->createEndEffector("manip");

    EXPECT_EQ(skel->getEndEffector("manip"), manip);
    EXPECT_EQ(skel->getEndEffector(0), manip);
//    EXPECT_EQ(skel->getBodyNode(0)->getEndEffector(0), manip);

    EndEffectorPtr strongManip = manip;
    WeakEndEffectorPtr weakManip = strongManip;

    EXPECT_FALSE(weakManip.lock() == nullptr);

    manip->remove();

    // The Node has been removed, so no reference to it will exist in the
    // Skeleton
#ifdef NDEBUG // Release Mode
    EXPECT_NE(skel->getEndEffector("manip"), manip);
    EXPECT_EQ(skel->getEndEffector("manip"), nullptr);

    EXPECT_NE(skel->getEndEffector(0), manip);
    EXPECT_EQ(skel->getEndEffector(0), nullptr);
#endif        // Release Mode

#ifdef NDEBUG // Release Mode
    // But it will not remain in the BodyNode's indexing.
    // Note: We should only run this test in release mode, because otherwise it
    // will trigger an assertion.
//    EXPECT_NE(skel->getBodyNode(0)->getEndEffector(0), manip);
//    EXPECT_EQ(skel->getBodyNode(0)->getEndEffector(0), nullptr);
#endif        // Release Mode

    EXPECT_NE(weakManip.lock(), nullptr);

    strongManip = nullptr;

    // The Node has been removed, and no strong reference to it exists any
    // longer, so it should be gone from the Skeleton
    EXPECT_EQ(skel->getEndEffector("manip"), nullptr);
    EXPECT_EQ(skel->getNumEndEffectors(), 0u);
//    EXPECT_EQ(skel->getBodyNode(0)->getNumEndEffectors(), 0u);

    EXPECT_EQ(weakManip.lock(), nullptr);
  }

  using GenericNodePtr = TemplateNodePtr<GenericNode, BodyNode>;
  using WeakGenericNodePtr = TemplateWeakNodePtr<GenericNode, BodyNode>;
  //--------------------------------------------------------------------------
  // Testing GenericNode, which is NOT a specialized Node type
  //--------------------------------------------------------------------------
  {
    GenericNode* node =
        skel->getBodyNode(0)->createNode<GenericNode>("node");

    EXPECT_EQ(skel->getNode<GenericNode>("node"), node);
    EXPECT_EQ(skel->getNode<GenericNode>(0), node);
    EXPECT_EQ(skel->getBodyNode(0)->getNode<GenericNode>(0), node);

    WeakGenericNodePtr weakNode = node;

    EXPECT_NE(weakNode.lock(), nullptr);

    node->remove();

    // The Node has been removed, and no strong reference to it exists, so it
    // should be gone from the Skeleton
    EXPECT_EQ(skel->getNode<GenericNode>("node"), nullptr);
    EXPECT_EQ(skel->getNumNodes<GenericNode>(), 0u);
    EXPECT_EQ(skel->getBodyNode(0)->getNumNodes<GenericNode>(), 0u);

    EXPECT_EQ(weakNode.lock(), nullptr);
  }

  {
    GenericNode* node =
        skel->getBodyNode(0)->createNode<GenericNode>("node");

    EXPECT_EQ(skel->getNode<GenericNode>("node"), node);
    EXPECT_EQ(skel->getNode<GenericNode>(0), node);
    EXPECT_EQ(skel->getBodyNode(0)->getNode<GenericNode>(0), node);

    GenericNodePtr strongNode = node;
    WeakGenericNodePtr weakNode = strongNode;

    EXPECT_FALSE(weakNode.lock() == nullptr);

    node->remove();

    // The Node has been removed, so no reference to it will exist in the
    // Skeleton
#ifdef NDEBUG // Release Mode
    EXPECT_NE(skel->getNode<GenericNode>("node"), node);
    EXPECT_EQ(skel->getNode<GenericNode>("node"), nullptr);

    EXPECT_NE(skel->getNode<GenericNode>(0), node);
    EXPECT_EQ(skel->getNode<GenericNode>(0), nullptr);
#endif        // Release Mode

#ifdef NDEBUG // Release Mode
    // But it will not remain in the BodyNode's indexing.
    // Note: We should only run this test in release mode, because otherwise it
    // will trigger an assertion.
    EXPECT_NE(skel->getBodyNode(0)->getNode<GenericNode>(0), node);
    EXPECT_EQ(skel->getBodyNode(0)->getNode<GenericNode>(0), nullptr);
#endif        // Release Mode

    EXPECT_NE(weakNode.lock(), nullptr);

    strongNode = nullptr;

    // The Node has been removed, and no strong reference to it exists any
    // longer, so it should be gone from the Skeleton
    EXPECT_EQ(skel->getNode<GenericNode>("node"), nullptr);
    EXPECT_EQ(skel->getNumNodes<GenericNode>(), 0u);
    EXPECT_EQ(skel->getBodyNode(0)->getNumNodes<GenericNode>(), 0u);

    EXPECT_EQ(weakNode.lock(), nullptr);
  }
}

TEST(Skeleton, CloneNodeOrdering)
{
  // This test checks that the ordering of Nodes in a cloned Skeleton will match
  // the ordering of Nodes in the original that was copied.

  SkeletonPtr skel = Skeleton::create();
  skel->createJointAndBodyNodePair<FreeJoint>(nullptr);
  skel->createJointAndBodyNodePair<FreeJoint>(nullptr);
  skel->createJointAndBodyNodePair<FreeJoint>(nullptr);

  // Add Nodes in the reverse order, so that their indexing is different from
  // the BodyNodes they are attached to
  for(int i=skel->getNumBodyNodes()-1; i > 0; --i)
  {
    skel->getBodyNode(i)->createEndEffector("manip_"+std::to_string(i));
  }

  skel->getBodyNode(1)->createEndEffector("other_manip");
  skel->getBodyNode(0)->createEndEffector("another_manip");
  skel->getBodyNode(2)->createEndEffector("yet_another_manip");

  SkeletonPtr clone = skel->clone();

  for(std::size_t i=0; i < skel->getNumEndEffectors(); ++i)
  {
    EXPECT_EQ(skel->getEndEffector(i)->getName(),
              clone->getEndEffector(i)->getName());
  }
}

TEST(Skeleton, ZeroDofJointInReferential)
{
  // This is a regression test which makes sure that the BodyNodes of
  // ZeroDofJoints will be correctly included in linkages.
  SkeletonPtr skel = Skeleton::create();

  BodyNode* bn = skel->createJointAndBodyNodePair<RevoluteJoint>().second;
  BodyNode* zeroDof1 = skel->createJointAndBodyNodePair<WeldJoint>(bn).second;
  bn = skel->createJointAndBodyNodePair<PrismaticJoint>(zeroDof1).second;
  BodyNode* zeroDof2 = skel->createJointAndBodyNodePair<WeldJoint>(bn).second;

  BranchPtr branch = Branch::create(skel->getBodyNode(0));
  EXPECT_EQ(branch->getNumBodyNodes(), skel->getNumBodyNodes());
  EXPECT_FALSE(branch->getIndexOf(zeroDof1) == INVALID_INDEX);
  EXPECT_FALSE(branch->getIndexOf(zeroDof2) == INVALID_INDEX);
}

TEST(Skeleton, ZeroDofJointConstraintForces)
{
  // This is a regression test which makes sure that the BodyNodes of
  // ZeroDofJoints will be correctly aggregate constraint forces.
  SkeletonPtr skel = Skeleton::create();

  BodyNode* bn = skel->createJointAndBodyNodePair<RevoluteJoint>().second;
  BodyNode* zeroDof1 = skel->createJointAndBodyNodePair<WeldJoint>(bn).second;
  bn = skel->createJointAndBodyNodePair<PrismaticJoint>(zeroDof1).second;
  skel->createJointAndBodyNodePair<WeldJoint>(bn);

  const auto numSkelDofs = skel->getNumDofs();
  for (auto& bodyNode : skel->getBodyNodes())
    bodyNode->setConstraintImpulse(Eigen::Vector6d::Random());

  // Make sure this does not cause seg-fault
  Eigen::VectorXd constraintForces = skel->getConstraintForces();
  EXPECT_EQ(constraintForces.size(), static_cast<int>(numSkelDofs));
}

TEST(Skeleton, Referential)
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
    for(std::size_t j=0; j<skeleton->getNumTrees(); ++j)
    {
      BranchPtr tree = Branch::create(skeleton->getRootBodyNode(j));

      const std::vector<BodyNode*>& skelBns = skeleton->getTreeBodyNodes(j);
      EXPECT_TRUE(tree->getNumBodyNodes() == skelBns.size());
      for(BodyNode* bn : skelBns)
      {
        EXPECT_FALSE(tree->getIndexOf(bn) == INVALID_INDEX);
        EXPECT_TRUE(tree->getBodyNode(tree->getIndexOf(bn)) == bn);
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
          q[r] = math::random(-10, 10);
          dq[r] = math::random(-10, 10);
          ddq[r] = math::random(-10, 10);
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

template <class JointType = RevoluteJoint>
BodyNode* addBodyNode(BodyNode* bn, const std::string& name)
{
  BodyNode* result = bn->createChildJointAndBodyNodePair<JointType>().second;
  result->setName(name);
  return result;
}

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

void checkForBodyNodes(
    std::size_t& /*count*/,
    const ReferentialSkeletonPtr& /*refSkel*/,
    const SkeletonPtr& /*skel*/)
{
  // Do nothing
}

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

TEST(Skeleton, Linkage)
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

TEST(Skeleton, Group)
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
    std::size_t randomIndex = floor(random(0, skel->getNumBodyNodes()));
    BodyNode* bn = skel->getBodyNode(randomIndex);
    if(group->addBodyNode(bn, false))
      bodyNodes.push_back(bn);

    randomIndex = floor(random(0, skel->getNumJoints()));
    Joint* joint = skel->getJoint(randomIndex);
    if(group->addJoint(joint, false, false))
      joints.push_back(joint);

    randomIndex = floor(random(0, skel->getNumDofs()));
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

TEST(Skeleton, Configurations)
{
  SkeletonPtr twoLink = createTwoLinkRobot(Vector3d::Ones(), DOF_YAW,
                                           Vector3d::Ones(), DOF_ROLL);

  SkeletonPtr threeLink = createThreeLinkRobot(Vector3d::Ones(), DOF_PITCH,
                                               Vector3d::Ones(), DOF_ROLL,
                                               Vector3d::Ones(), DOF_YAW);

  Skeleton::Configuration c2 = twoLink->getConfiguration();
  Skeleton::Configuration c3 = threeLink->getConfiguration();

  EXPECT_FALSE(c2 == c3);
  EXPECT_TRUE(c2 == c2);
  EXPECT_TRUE(c3 == c3);
  EXPECT_TRUE(c2 != c3);

  twoLink->setPosition(0, 1.0);
  EXPECT_FALSE(c2 == twoLink->getConfiguration());

  threeLink->setPosition(1, 2.0);
  EXPECT_FALSE(c3 == twoLink->getConfiguration());

  c2 = twoLink->getConfiguration(Skeleton::CONFIG_VELOCITIES);
  EXPECT_TRUE(c2.mPositions.size() == 0);
  EXPECT_TRUE(c2.mVelocities.size() == static_cast<int>(twoLink->getNumDofs()));
  EXPECT_TRUE(c2.mAccelerations.size() == 0);
}

TEST(Skeleton, LinearJacobianDerivOverload)
{
  // Regression test for #626: Make sure that getLinearJacobianDeriv's overload
  // is working appropriately.
  SkeletonPtr skeleton = createThreeLinkRobot(Vector3d::Ones(), DOF_PITCH,
                                              Vector3d::Ones(), DOF_ROLL,
                                              Vector3d::Ones(), DOF_YAW);

  skeleton->getLinearJacobianDeriv(skeleton->getBodyNode(0));

  LinkagePtr linkage = Branch::create(skeleton->getBodyNode(0));
  linkage->getLinearJacobianDeriv(linkage->getBodyNode(0));
}

TEST(Skeleton, Updating)
{
  // Make sure that structural properties get automatically updated correctly

  // RevoluteJoint
  SkeletonPtr skeleton = createTwoLinkRobot(Vector3d::Ones(), DOF_PITCH,
                                            Vector3d::Ones(), DOF_ROLL);

  Joint* joint0 = skeleton->getJoint(0);
  Joint* joint1 = skeleton->getJoint(1);

  math::Jacobian J0i = joint0->getRelativeJacobian();
  joint0->get<RevoluteJoint::Aspect>()->setProperties(
        joint1->get<RevoluteJoint::Aspect>()->getProperties());

  math::Jacobian J0f = joint0->getRelativeJacobian();
  EXPECT_FALSE(equals(J0i, J0f));

  // PrismaticJoint
  skeleton = createTwoLinkRobot(Vector3d::Ones(), DOF_X,
                                Vector3d::Ones(), DOF_Y);
  joint0 = skeleton->getJoint(0);
  joint1 = skeleton->getJoint(1);

  J0i = joint0->getRelativeJacobian();
  joint0->get<PrismaticJoint::Aspect>()->setProperties(
        joint1->get<PrismaticJoint::Aspect>()->getProperties());
  J0f = joint0->getRelativeJacobian();
  EXPECT_FALSE(equals(J0i, J0f));

  skeleton = Skeleton::create();
  ScrewJoint* screw = skeleton->createJointAndBodyNodePair<ScrewJoint>().first;

  screw->setAxis(Eigen::Vector3d::UnitX());
  screw->setPitch(2);

  J0i = screw->getRelativeJacobian();
  screw->setAxis(Eigen::Vector3d::UnitY());
  J0f = screw->getRelativeJacobian();
  EXPECT_FALSE(equals(J0i, J0f));

  J0i = J0f;
  screw->setPitch(3);
  J0f = screw->getRelativeJacobian();
  EXPECT_FALSE(equals(J0i, J0f));

  // Regression test for Pull Request #731
  const double originalMass = skeleton->getMass();
  BodyNode* lastBn = skeleton->getBodyNode(skeleton->getNumBodyNodes()-1);
  const double removedMass = lastBn->getMass();
  EXPECT_FALSE(removedMass == 0.0);
  lastBn->remove();
  const double newMass = skeleton->getMass();
  EXPECT_FALSE(originalMass == newMass);
  EXPECT_TRUE(newMass == originalMass - removedMass);
}

//==============================================================================
TEST(Skeleton, GetJointsAndBodyNodes)
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
