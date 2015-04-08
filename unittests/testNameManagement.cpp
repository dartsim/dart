/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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
#include "TestHelpers.h"
#include "dart/dynamics/SoftBodyNode.h"

using namespace dart;
using namespace math;
using namespace dynamics;

//==============================================================================
TEST(NameManagement, Basic)
{
  //--------------------------------------------------------------------------
  //
  //--------------------------------------------------------------------------
  // Bodies
  BodyNode* body1 = new BodyNode;
  BodyNode* body2 = new BodyNode;
  BodyNode* body3 = new BodyNode;

  // TODO: Add some SoftBodyNodes into this test.
  // Grey is not familiar with construction of SoftBodyNodes,
  // so he is leaving that for someone else for the time being.

  // Joints
  Joint* joint1 = new RevoluteJoint(Eigen::Vector3d::UnitX(), "joint");
  Joint* joint2 = new TranslationalJoint("joint");
  Joint* joint3 = new FreeJoint("joint");

  // Testing whether DegreesOfFreedom get named correctly
  EXPECT_TRUE(joint1->getDof(0)->getName() == "joint");

  EXPECT_TRUE(joint2->getDof(0)->getName() == "joint_x");
  EXPECT_TRUE(joint2->getDof(1)->getName() == "joint_y");
  EXPECT_TRUE(joint2->getDof(2)->getName() == "joint_z");

  EXPECT_TRUE(joint3->getDof(0)->getName() == "joint_rot_x");
  EXPECT_TRUE(joint3->getDof(1)->getName() == "joint_rot_y");
  EXPECT_TRUE(joint3->getDof(2)->getName() == "joint_rot_z");
  EXPECT_TRUE(joint3->getDof(3)->getName() == "joint_pos_x");
  EXPECT_TRUE(joint3->getDof(4)->getName() == "joint_pos_y");
  EXPECT_TRUE(joint3->getDof(5)->getName() == "joint_pos_z");

  // Skeleton
  Skeleton* skel = new Skeleton;

  body1->setParentJoint(joint1);
  body2->setParentJoint(joint2);
  body3->setParentJoint(joint3);

  body1->addChildBodyNode(body2);
  body2->addChildBodyNode(body3);

  skel->addBodyNode(body1);
  skel->addBodyNode(body2);
  skel->addBodyNode(body3);

  skel->init();

  // Testing whether the repeated names of BodyNodes and Joints get resolved
  // correctly as BodyNodes get added to the Skeleton
  EXPECT_FALSE(body1->getName() == body2->getName());
  EXPECT_FALSE(body2->getName() == body3->getName());
  EXPECT_FALSE(body3->getName() == body1->getName());

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
  EXPECT_TRUE(body2->getName() == "a_unique_name");
  EXPECT_TRUE(skel->getBodyNode("a_unique_name") == body2);

  EXPECT_FALSE(body1->getName() == body2->getName());
  EXPECT_FALSE(body2->getName() == body3->getName());
  EXPECT_FALSE(body3->getName() == body1->getName());

  unique_name = joint3->setName("a_unique_name");
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
  for (size_t i = 0; i < skel->getNumDofs(); ++i)
    EXPECT_TRUE(skel->getDof(i)->getIndexInSkeleton() == i);

  // Test whether all the joint names are still unique
  EXPECT_FALSE(joint1->getName() == joint2->getName());
  EXPECT_FALSE(joint2->getName() == joint3->getName());
  EXPECT_FALSE(joint3->getName() == joint1->getName());

  // Make sure that the Skeleton gives back NULL for non existent names
  EXPECT_TRUE(skel->getBodyNode("nonexistent_name") == NULL);
  EXPECT_TRUE(skel->getJoint("nonexistent_name") == NULL);
  EXPECT_TRUE(skel->getSoftBodyNode("nonexistent_name") == NULL);

  Joint* oldJoint = body3->getParentJoint();
  std::string oldJointName = oldJoint->getName();
  Joint* newJoint = new RevoluteJoint(Eigen::Vector3d(1,0,0), "a_new_joint");
  body3->setParentJoint(newJoint);
  EXPECT_TRUE(skel->getJoint("a_new_joint") == newJoint);

  // Make sure that the Skeleton returns NULL on any Joint names that have been
  // taken away from it
  EXPECT_FALSE(skel->getJoint(oldJointName) == oldJoint);
  EXPECT_TRUE(skel->getJoint(oldJointName) == NULL);
}

//==============================================================================
TEST(NameManagement, SetPattern)
{
  dart::common::NameManager<BodyNode*> test_mgr;

  BodyNode* bn0 = new BodyNode("name");
  BodyNode* bn1 = new BodyNode("name");
  BodyNode* bn2 = new BodyNode("name");

  test_mgr.setPattern("%s(%d)");

  test_mgr.issueNewNameAndAdd(bn0->getName(), bn0);
  test_mgr.issueNewNameAndAdd(bn1->getName(), bn1);
  test_mgr.issueNewNameAndAdd(bn2->getName(), bn2);

  EXPECT_TRUE( test_mgr.getObject("name") == bn0);
  EXPECT_TRUE( test_mgr.getObject("name(1)") == bn1);
  EXPECT_TRUE( test_mgr.getObject("name(2)") == bn2);

  test_mgr.clear();

  bn0->setName("bodynode");
  bn1->setName("bodynode");
  bn2->setName("bodynode");

  test_mgr.setPattern("(%d)-%s");
  test_mgr.issueNewNameAndAdd(bn0->getName(), bn0);
  test_mgr.issueNewNameAndAdd(bn1->getName(), bn1);
  test_mgr.issueNewNameAndAdd(bn2->getName(), bn2);

  EXPECT_TRUE( test_mgr.getObject("bodynode") == bn0);
  EXPECT_TRUE( test_mgr.getObject("(1)-bodynode") == bn1 );
  EXPECT_TRUE( test_mgr.getObject("(2)-bodynode") == bn2 );

  delete bn0;
  delete bn1;
  delete bn2;
}

//==============================================================================
TEST(NameManagement, WorldSkeletons)
{
  dart::simulation::WorldPtr world1(new dart::simulation::World);

  dart::dynamics::SkeletonPtr skel0(new dart::dynamics::Skeleton);
  dart::dynamics::SkeletonPtr skel1(new dart::dynamics::Skeleton);
  dart::dynamics::SkeletonPtr skel2(new dart::dynamics::Skeleton);

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

  dart::dynamics::SkeletonPtr skel3(new dart::dynamics::Skeleton("OtherName"));
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

  world1->addFrame(frame0);
  world1->addFrame(frame1);
  world1->addFrame(frame2);

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

  EXPECT_TRUE( frame0 == world1->getFrame(frame0->getName()) );
  EXPECT_TRUE( frame1 == world1->getFrame(frame1->getName()) );
  EXPECT_TRUE( frame2 == world1->getFrame(frame2->getName()) );

  dart::simulation::WorldPtr world2(new dart::simulation::World);

  dart::dynamics::SimpleFramePtr frame3(
        new dart::dynamics::SimpleFrame(Frame::World(), "OtherName"));
  world2->addFrame(frame3);
  world2->addFrame(frame2);
  world2->addFrame(frame1);

  EXPECT_TRUE(frame3->getName() == "OtherName");
  EXPECT_TRUE(frame1->getName() == "OtherName(2)");
  EXPECT_TRUE(frame2->getName() == "OtherName(1)");

  frame3->setName("Frame(1)");
  frame1->setName("Frame");

  EXPECT_TRUE(frame3->getName() == "Frame(1)");
  EXPECT_TRUE(frame1->getName() == "Frame(1)(1)");

  EXPECT_TRUE( frame0 == world1->getFrame(frame0->getName()) );
  EXPECT_TRUE( frame1 == world1->getFrame(frame1->getName()) );
  EXPECT_TRUE( frame2 == world1->getFrame(frame2->getName()) );

  EXPECT_TRUE( frame3 == world2->getFrame(frame3->getName()) );
  EXPECT_TRUE( frame1 == world2->getFrame(frame1->getName()) );
  EXPECT_TRUE( frame2 == world2->getFrame(frame2->getName()) );

  world2->removeFrame(frame1);

  frame1->setName("Frame");
  EXPECT_TRUE(frame1->getName() == "Frame(1)");
}

//==============================================================================
int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
