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
#include "dart/dynamics/SoftBodyNode.hpp"

using namespace dart;
using namespace math;
using namespace dynamics;

//==============================================================================
TEST(NameManagement, Skeleton)
{
  SkeletonPtr skel = Skeleton::create();

  std::pair<Joint*, BodyNode*> pair;
  pair = skel->createJointAndBodyNodePair<RevoluteJoint>(
        nullptr, GenericJoint<R1Space>::Properties(std::string("joint")));
  Joint* joint1 = pair.first;
  BodyNode* body1 = pair.second;

  pair = skel->createJointAndBodyNodePair<TranslationalJoint>(
        body1, GenericJoint<R3Space>::Properties(std::string("joint")));
  Joint* joint2 = pair.first;
  BodyNode* body2 = pair.second;

  pair = skel->createJointAndBodyNodePair<FreeJoint>(
        body2, GenericJoint<SE3Space>::Properties(std::string("joint")));
  Joint* joint3 = pair.first;
  BodyNode* body3 = pair.second;

  // Testing whether the repeated names of BodyNodes and Joints get resolved
  // correctly as BodyNodes get added to the Skeleton
  EXPECT_FALSE(body1->getName() == body2->getName());
  EXPECT_FALSE(body2->getName() == body3->getName());
  EXPECT_FALSE(body3->getName() == body1->getName());

  EXPECT_TRUE(body1->getName() == "BodyNode");
  EXPECT_TRUE(body2->getName() == "BodyNode(1)");
  EXPECT_TRUE(body3->getName() == "BodyNode(2)");

  EXPECT_FALSE(joint1->getName() == joint2->getName());
  EXPECT_FALSE(joint2->getName() == joint3->getName());
  EXPECT_FALSE(joint3->getName() == joint1->getName());

  EXPECT_TRUE(joint1->getDof(0)->getName() == "joint");

  EXPECT_TRUE(joint2->getDof(0)->getName() == "joint(1)_x");
  EXPECT_TRUE(joint2->getDof(1)->getName() == "joint(1)_y");
  EXPECT_TRUE(joint2->getDof(2)->getName() == "joint(1)_z");

  EXPECT_TRUE(joint3->getDof(0)->getName() == "joint(2)_rot_x");
  EXPECT_TRUE(joint3->getDof(1)->getName() == "joint(2)_rot_y");
  EXPECT_TRUE(joint3->getDof(2)->getName() == "joint(2)_rot_z");
  EXPECT_TRUE(joint3->getDof(3)->getName() == "joint(2)_pos_x");
  EXPECT_TRUE(joint3->getDof(4)->getName() == "joint(2)_pos_y");
  EXPECT_TRUE(joint3->getDof(5)->getName() == "joint(2)_pos_z");

  // Testing whether the repeated names of BodyNodes get resolved correctly
  // as they are changed with BodyNode::setName(~)
  std::string newname1 = body1->setName("same_name");
  std::string newname2 = body2->setName("same_name");
  std::string newname3 = body3->setName("same_name");

  EXPECT_FALSE(body1->getName() == body2->getName());
  EXPECT_FALSE(body2->getName() == body3->getName());
  EXPECT_FALSE(body3->getName() == body1->getName());

  EXPECT_TRUE(body1->getName() == newname1);
  EXPECT_TRUE(body2->getName() == newname2);
  EXPECT_TRUE(body3->getName() == newname3);

  EXPECT_TRUE(skel->getBodyNode(newname1) == body1);
  EXPECT_TRUE(skel->getBodyNode(newname2) == body2);
  EXPECT_TRUE(skel->getBodyNode(newname3) == body3);

  // Testing whether the repeated names of Joints get resolved correctly
  // as they are changed with Joint::setName(~)
  newname1 = joint1->setName("another_name");
  newname2 = joint2->setName("another_name");
  newname3 = joint3->setName("another_name");

  EXPECT_FALSE(joint1->getName() == joint2->getName());
  EXPECT_FALSE(joint2->getName() == joint3->getName());
  EXPECT_FALSE(joint3->getName() == joint1->getName());

  EXPECT_TRUE(joint1->getName() == newname1);
  EXPECT_TRUE(joint2->getName() == newname2);
  EXPECT_TRUE(joint3->getName() == newname3);

  EXPECT_TRUE(skel->getJoint(newname1) == joint1);
  EXPECT_TRUE(skel->getJoint(newname2) == joint2);
  EXPECT_TRUE(skel->getJoint(newname3) == joint3);

  // Testing whether unique names get accidentally changed by the NameManager
  std::string unique_name = body2->setName("a_unique_name");
  EXPECT_TRUE(body2->getName() == unique_name);
  EXPECT_TRUE(body2->getName() == "a_unique_name");
  EXPECT_TRUE(skel->getBodyNode("a_unique_name") == body2);

  EXPECT_FALSE(body1->getName() == body2->getName());
  EXPECT_FALSE(body2->getName() == body3->getName());
  EXPECT_FALSE(body3->getName() == body1->getName());

  unique_name = joint3->setName("a_unique_name");
  EXPECT_TRUE(joint3->getName() == unique_name);
  EXPECT_TRUE(joint3->getName() == "a_unique_name");
  EXPECT_TRUE(skel->getJoint("a_unique_name") == joint3);

  // Testing whether the DegreeOfFreedom names get updated correctly upon their
  // joint's name change
  EXPECT_TRUE(joint3->getDof(0)->getName() == "a_unique_name_rot_x");
  EXPECT_TRUE(joint3->getDof(1)->getName() == "a_unique_name_rot_y");
  EXPECT_TRUE(joint3->getDof(2)->getName() == "a_unique_name_rot_z");
  EXPECT_TRUE(joint3->getDof(3)->getName() == "a_unique_name_pos_x");
  EXPECT_TRUE(joint3->getDof(4)->getName() == "a_unique_name_pos_y");
  EXPECT_TRUE(joint3->getDof(5)->getName() == "a_unique_name_pos_z");

  EXPECT_TRUE(joint3->getDof(0) == skel->getDof("a_unique_name_rot_x"));
  EXPECT_TRUE(joint3->getDof(3) == skel->getDof("a_unique_name_pos_x"));

  // Note: The following assumes the joint order in the Skeleton is:
  // RevoluteJoint -> TranslationalJoint -> FreeJoint
  EXPECT_TRUE(joint1->getDof(0) == skel->getDof(0));

  EXPECT_TRUE(joint2->getDof(0) == skel->getDof(1));
  EXPECT_TRUE(joint2->getDof(1) == skel->getDof(2));
  EXPECT_TRUE(joint2->getDof(2) == skel->getDof(3));

  EXPECT_TRUE(joint3->getDof(0) == skel->getDof(4));
  EXPECT_TRUE(joint3->getDof(1) == skel->getDof(5));
  EXPECT_TRUE(joint3->getDof(2) == skel->getDof(6));
  EXPECT_TRUE(joint3->getDof(3) == skel->getDof(7));
  EXPECT_TRUE(joint3->getDof(4) == skel->getDof(8));
  EXPECT_TRUE(joint3->getDof(5) == skel->getDof(9));

  // Test whether the return of getIndexInSkeleton() and the index of the
  // corresponding DegreeOfFreedom in the Skeleton are same
  for (std::size_t i = 0; i < skel->getNumDofs(); ++i)
    EXPECT_TRUE(skel->getDof(i)->getIndexInSkeleton() == i);

  // Test whether all the joint names are still unique
  EXPECT_FALSE(joint1->getName() == joint2->getName());
  EXPECT_FALSE(joint2->getName() == joint3->getName());
  EXPECT_FALSE(joint3->getName() == joint1->getName());

  // Make sure that the Skeleton gives back nullptr for non existent names
  EXPECT_TRUE(skel->getBodyNode("nonexistent_name") == nullptr);
  EXPECT_TRUE(skel->getJoint("nonexistent_name") == nullptr);
  EXPECT_TRUE(skel->getSoftBodyNode("nonexistent_name") == nullptr);

  // Test Node Names
  EndEffector* ee1 = body1->createEndEffector("ee");
  EndEffector* ee2 = body1->createEndEffector("ee");

  EXPECT_TRUE(skel->getEndEffector("ee") == ee1);
  EXPECT_TRUE(skel->getEndEffector("ee(1)") == ee2);
}

//==============================================================================
TEST(NameManagement, SetPattern)
{
  dart::common::NameManager< std::shared_ptr<Entity> > test_mgr("test", "name");

  std::shared_ptr<Entity> entity0(new SimpleFrame(Frame::World(), "name"));
  std::shared_ptr<Entity> entity1(new SimpleFrame(Frame::World(), "name"));
  std::shared_ptr<Entity> entity2(new SimpleFrame(Frame::World(), "name"));

  test_mgr.setPattern("%s(%d)");

  test_mgr.issueNewNameAndAdd(entity0->getName(), entity0);
  test_mgr.issueNewNameAndAdd(entity1->getName(), entity1);
  test_mgr.issueNewNameAndAdd(entity2->getName(), entity2);

  EXPECT_TRUE( test_mgr.getObject("name") == entity0);
  EXPECT_TRUE( test_mgr.getObject("name(1)") == entity1);
  EXPECT_TRUE( test_mgr.getObject("name(2)") == entity2);

  test_mgr.clear();

  entity0->setName("Entity");
  entity1->setName("Entity");
  entity2->setName("Entity");

  test_mgr.setPattern("(%d)-%s");
  test_mgr.issueNewNameAndAdd(entity0->getName(), entity0);
  test_mgr.issueNewNameAndAdd(entity1->getName(), entity1);
  test_mgr.issueNewNameAndAdd(entity2->getName(), entity2);

  EXPECT_TRUE( test_mgr.getObject("Entity") == entity0);
  EXPECT_TRUE( test_mgr.getObject("(1)-Entity") == entity1 );
  EXPECT_TRUE( test_mgr.getObject("(2)-Entity") == entity2 );
}

//==============================================================================
TEST(NameManagement, Regression554)
{
  dart::common::NameManager< std::shared_ptr<int> > test_mgr("test", "name");

  std::shared_ptr<int> int0(new int(0));
  std::shared_ptr<int> int1(new int(1));
  std::shared_ptr<int> int2(new int(2));
  std::shared_ptr<int> int_another0(new int(0));

  test_mgr.issueNewNameAndAdd(std::to_string(*int0), int0);
  test_mgr.issueNewNameAndAdd(std::to_string(*int1), int1);
  test_mgr.issueNewNameAndAdd(std::to_string(*int2), int2);

  EXPECT_TRUE( test_mgr.getObject("0") == int0 );
  EXPECT_TRUE( test_mgr.getObject("1") == int1 );
  EXPECT_TRUE( test_mgr.getObject("2") == int2 );

  bool res1 = test_mgr.addName(std::to_string(*int_another0), int_another0);
  EXPECT_FALSE( res1 );

  test_mgr.removeEntries(std::to_string(*int_another0), int_another0);
  bool res2 = test_mgr.addName(std::to_string(*int_another0), int_another0);
  EXPECT_TRUE( res2 );

  EXPECT_TRUE( test_mgr.getObject("0") == int_another0 );
  EXPECT_TRUE( test_mgr.getObject("1") == int1 );
  EXPECT_TRUE( test_mgr.getObject("2") == int2 );
}

//==============================================================================
TEST(NameManagement, WorldSkeletons)
{
  dart::simulation::WorldPtr world1(new dart::simulation::World);
  world1->setName("world1");

  dart::dynamics::SkeletonPtr skel0 = dart::dynamics::Skeleton::create();
  dart::dynamics::SkeletonPtr skel1 = dart::dynamics::Skeleton::create();
  dart::dynamics::SkeletonPtr skel2 = dart::dynamics::Skeleton::create();

  world1->addSkeleton(skel0);
  world1->addSkeleton(skel1);
  world1->addSkeleton(skel2);

  EXPECT_TRUE(skel0->getName() == "Skeleton");
  EXPECT_TRUE(skel1->getName() == "Skeleton(1)");
  EXPECT_TRUE(skel2->getName() == "Skeleton(2)");

  skel1->setName("Skeleton");
  EXPECT_TRUE(skel1->getName() == "Skeleton(1)");
  skel1->setName("OtherName");
  EXPECT_TRUE(skel1->getName() == "OtherName");

  skel2->setName("Skeleton");
  EXPECT_TRUE(skel2->getName() == "Skeleton(1)");
  skel2->setName("OtherName");
  EXPECT_TRUE(skel2->getName() == "OtherName(1)");

  EXPECT_TRUE( skel0 == world1->getSkeleton(skel0->getName()) );
  EXPECT_TRUE( skel1 == world1->getSkeleton(skel1->getName()) );
  EXPECT_TRUE( skel2 == world1->getSkeleton(skel2->getName()) );

  dart::simulation::WorldPtr world2(new dart::simulation::World);
  world2->setName("world2");

  dart::dynamics::SkeletonPtr skel3 =
      dart::dynamics::Skeleton::create("OtherName");
  world2->addSkeleton(skel3);
  world2->addSkeleton(skel2);
  world2->addSkeleton(skel1);

  EXPECT_TRUE(skel3->getName() == "OtherName");
  EXPECT_TRUE(skel1->getName() == "OtherName(2)");
  EXPECT_TRUE(skel2->getName() == "OtherName(1)");

  skel3->setName("Skeleton(1)");
  skel1->setName("Skeleton");

  EXPECT_TRUE(skel3->getName() == "Skeleton(1)");
  EXPECT_TRUE(skel1->getName() == "Skeleton(1)(1)");

  EXPECT_TRUE( skel0 == world1->getSkeleton(skel0->getName()) );
  EXPECT_TRUE( skel1 == world1->getSkeleton(skel1->getName()) );
  EXPECT_TRUE( skel2 == world1->getSkeleton(skel2->getName()) );

  EXPECT_TRUE( skel3 == world2->getSkeleton(skel3->getName()) );
  EXPECT_TRUE( skel1 == world2->getSkeleton(skel1->getName()) );
  EXPECT_TRUE( skel2 == world2->getSkeleton(skel2->getName()) );

  world2->removeSkeleton(skel1);

  skel1->setName("Skeleton");
  EXPECT_TRUE(skel1->getName() == "Skeleton(1)");
}

//==============================================================================
TEST(NameManagement, WorldSimpleFrames)
{
  dart::simulation::WorldPtr world1(new dart::simulation::World);

  dart::dynamics::SimpleFramePtr frame0(
        new dart::dynamics::SimpleFrame(Frame::World(), "Frame"));
  dart::dynamics::SimpleFramePtr frame1(
        new dart::dynamics::SimpleFrame(Frame::World(), "Frame"));
  dart::dynamics::SimpleFramePtr frame2(
        new dart::dynamics::SimpleFrame(Frame::World(), "Frame"));

  world1->addSimpleFrame(frame0);
  world1->addSimpleFrame(frame1);
  world1->addSimpleFrame(frame2);

  EXPECT_TRUE(frame0->getName() == "Frame");
  EXPECT_TRUE(frame1->getName() == "Frame(1)");
  EXPECT_TRUE(frame2->getName() == "Frame(2)");

  frame1->setName("Frame");
  EXPECT_TRUE(frame1->getName() == "Frame(1)");
  frame1->setName("OtherName");
  EXPECT_TRUE(frame1->getName() == "OtherName");

  frame2->setName("Frame");
  EXPECT_TRUE(frame2->getName() == "Frame(1)");
  frame2->setName("OtherName");
  EXPECT_TRUE(frame2->getName() == "OtherName(1)");

  EXPECT_TRUE( frame0 == world1->getSimpleFrame(frame0->getName()) );
  EXPECT_TRUE( frame1 == world1->getSimpleFrame(frame1->getName()) );
  EXPECT_TRUE( frame2 == world1->getSimpleFrame(frame2->getName()) );

  dart::simulation::WorldPtr world2(new dart::simulation::World);
  world2->setName("world2");

  dart::dynamics::SimpleFramePtr frame3(
        new dart::dynamics::SimpleFrame(Frame::World(), "OtherName"));
  world2->addSimpleFrame(frame3);
  world2->addSimpleFrame(frame2);
  world2->addSimpleFrame(frame1);

  EXPECT_TRUE(frame3->getName() == "OtherName");
  EXPECT_TRUE(frame1->getName() == "OtherName(2)");
  EXPECT_TRUE(frame2->getName() == "OtherName(1)");

  frame3->setName("Frame(1)");
  frame1->setName("Frame");

  EXPECT_TRUE(frame3->getName() == "Frame(1)");
  EXPECT_TRUE(frame1->getName() == "Frame(1)(1)");

  EXPECT_TRUE( frame0 == world1->getSimpleFrame(frame0->getName()) );
  EXPECT_TRUE( frame1 == world1->getSimpleFrame(frame1->getName()) );
  EXPECT_TRUE( frame2 == world1->getSimpleFrame(frame2->getName()) );

  EXPECT_TRUE( frame3 == world2->getSimpleFrame(frame3->getName()) );
  EXPECT_TRUE( frame1 == world2->getSimpleFrame(frame1->getName()) );
  EXPECT_TRUE( frame2 == world2->getSimpleFrame(frame2->getName()) );

  world2->removeSimpleFrame(frame1);

  frame1->setName("Frame");
  EXPECT_TRUE(frame1->getName() == "Frame(1)");
}

//==============================================================================
TEST(NameManagement, JointDegreeOfFreedom)
{
  SkeletonPtr subtree = Skeleton::create("subtree");
  EulerJoint::Properties jointProperties;
  jointProperties.mName = "j_bicep_right";
  JointPtr joint = subtree->createJointAndBodyNodePair<EulerJoint>(
        nullptr, jointProperties).first;
  std::string newName = "j_bicep_right_inverse";

  EXPECT_TRUE( joint->getDof(0)->getName() == "j_bicep_right_x" );
  EXPECT_TRUE( joint->getDof(1)->getName() == "j_bicep_right_y" );
  EXPECT_TRUE( joint->getDof(2)->getName() == "j_bicep_right_z" );

  joint->setName(newName, false);

  EXPECT_TRUE( joint->getDof(0)->getName() == "j_bicep_right_x" );
  EXPECT_TRUE( joint->getDof(1)->getName() == "j_bicep_right_y" );
  EXPECT_TRUE( joint->getDof(2)->getName() == "j_bicep_right_z" );

  joint->setName(newName, true);

  EXPECT_TRUE( joint->getDof(0)->getName() == "j_bicep_right_inverse_x" );
  EXPECT_TRUE( joint->getDof(1)->getName() == "j_bicep_right_inverse_y" );
  EXPECT_TRUE( joint->getDof(2)->getName() == "j_bicep_right_inverse_z" );

  newName = "j_bicep_left";
  joint->setName(newName, true);

  EXPECT_TRUE( joint->getDof(0)->getName() == "j_bicep_left_x" );
  EXPECT_TRUE( joint->getDof(1)->getName() == "j_bicep_left_y" );
  EXPECT_TRUE( joint->getDof(2)->getName() == "j_bicep_left_z" );
}
