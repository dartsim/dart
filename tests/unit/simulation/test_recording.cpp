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

#include <dart/simulation/recording.hpp>

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;

//==============================================================================
// Helper: Create a skeleton with FreeJoint (6 DOFs)
SkeletonPtr createFreeJointSkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);
  auto pair = skel->createJointAndBodyNodePair<FreeJoint>(
      nullptr, FreeJoint::Properties(), BodyNode::AspectProperties("body"));

  Inertia inertia;
  inertia.setMass(1.0);
  pair.second->setInertia(inertia);

  return skel;
}

//==============================================================================
// Helper: Create a skeleton with RevoluteJoint (1 DOF)
SkeletonPtr createRevoluteJointSkeleton(const std::string& name)
{
  auto skel = Skeleton::create(name);
  auto pair = skel->createJointAndBodyNodePair<RevoluteJoint>(
      nullptr, RevoluteJoint::Properties(), BodyNode::AspectProperties("body"));

  Inertia inertia;
  inertia.setMass(1.0);
  pair.second->setInertia(inertia);

  return skel;
}

//==============================================================================
TEST(RecordingTest, ConstructorFromSkeletons)
{
  auto skel1 = createFreeJointSkeleton("skel1");
  auto skel2 = createRevoluteJointSkeleton("skel2");

  std::vector<SkeletonPtr> skeletons = {skel1, skel2};
  Recording recording(skeletons);

  EXPECT_EQ(recording.getNumSkeletons(), 2);
  EXPECT_EQ(recording.getNumDofs(0), 6); // FreeJoint has 6 DOFs
  EXPECT_EQ(recording.getNumDofs(1), 1); // RevoluteJoint has 1 DOF
  EXPECT_EQ(recording.getNumFrames(), 0);
}

//==============================================================================
TEST(RecordingTest, ConstructorFromDofList)
{
  std::vector<int> dofs = {6, 3, 1};
  Recording recording(dofs);

  EXPECT_EQ(recording.getNumSkeletons(), 3);
  EXPECT_EQ(recording.getNumDofs(0), 6);
  EXPECT_EQ(recording.getNumDofs(1), 3);
  EXPECT_EQ(recording.getNumDofs(2), 1);
  EXPECT_EQ(recording.getNumFrames(), 0);
}

//==============================================================================
TEST(RecordingTest, AddStateAndGetNumFrames)
{
  std::vector<int> dofs = {3};
  Recording recording(dofs);

  EXPECT_EQ(recording.getNumFrames(), 0);

  Eigen::VectorXd state1(3);
  state1 << 1.0, 2.0, 3.0;
  recording.addState(state1);
  EXPECT_EQ(recording.getNumFrames(), 1);

  Eigen::VectorXd state2(3);
  state2 << 4.0, 5.0, 6.0;
  recording.addState(state2);
  EXPECT_EQ(recording.getNumFrames(), 2);

  Eigen::VectorXd state3(3);
  state3 << 7.0, 8.0, 9.0;
  recording.addState(state3);
  EXPECT_EQ(recording.getNumFrames(), 3);
}

//==============================================================================
TEST(RecordingTest, GetNumSkeletonsAndDofs)
{
  auto skel1 = createFreeJointSkeleton("skel1");
  auto skel2 = createFreeJointSkeleton("skel2");
  auto skel3 = createRevoluteJointSkeleton("skel3");

  std::vector<SkeletonPtr> skeletons = {skel1, skel2, skel3};
  Recording recording(skeletons);

  EXPECT_EQ(recording.getNumSkeletons(), 3);
  EXPECT_EQ(recording.getNumDofs(0), 6);
  EXPECT_EQ(recording.getNumDofs(1), 6);
  EXPECT_EQ(recording.getNumDofs(2), 1);
}

//==============================================================================
TEST(RecordingTest, GetConfigAndGenCoord)
{
  // Two skeletons: 3 DOFs and 2 DOFs
  std::vector<int> dofs = {3, 2};
  Recording recording(dofs);

  // State: [skel0: 1,2,3] [skel1: 4,5]
  Eigen::VectorXd state(5);
  state << 1.0, 2.0, 3.0, 4.0, 5.0;
  recording.addState(state);

  // Test getConfig for skeleton 0
  Eigen::VectorXd config0 = recording.getConfig(0, 0);
  EXPECT_EQ(config0.size(), 3);
  EXPECT_DOUBLE_EQ(config0[0], 1.0);
  EXPECT_DOUBLE_EQ(config0[1], 2.0);
  EXPECT_DOUBLE_EQ(config0[2], 3.0);

  // Test getConfig for skeleton 1
  Eigen::VectorXd config1 = recording.getConfig(0, 1);
  EXPECT_EQ(config1.size(), 2);
  EXPECT_DOUBLE_EQ(config1[0], 4.0);
  EXPECT_DOUBLE_EQ(config1[1], 5.0);

  // Test getGenCoord for individual DOFs
  EXPECT_DOUBLE_EQ(recording.getGenCoord(0, 0, 0), 1.0);
  EXPECT_DOUBLE_EQ(recording.getGenCoord(0, 0, 1), 2.0);
  EXPECT_DOUBLE_EQ(recording.getGenCoord(0, 0, 2), 3.0);
  EXPECT_DOUBLE_EQ(recording.getGenCoord(0, 1, 0), 4.0);
  EXPECT_DOUBLE_EQ(recording.getGenCoord(0, 1, 1), 5.0);
}

//==============================================================================
TEST(RecordingTest, ContactPoints)
{
  // Single skeleton with 3 DOFs
  std::vector<int> dofs = {3};
  Recording recording(dofs);

  // State format: [3 DOFs] + [contact1: point(3) + force(3)] + [contact2:
  // point(3) + force(3)] Total: 3 + 6 + 6 = 15 elements
  Eigen::VectorXd state(15);
  // DOFs
  state.segment(0, 3) << 0.1, 0.2, 0.3;
  // Contact 1: point at (1,2,3), force (10,20,30)
  state.segment(3, 3) << 1.0, 2.0, 3.0;
  state.segment(6, 3) << 10.0, 20.0, 30.0;
  // Contact 2: point at (4,5,6), force (40,50,60)
  state.segment(9, 3) << 4.0, 5.0, 6.0;
  state.segment(12, 3) << 40.0, 50.0, 60.0;

  recording.addState(state);

  // Test getNumContacts
  EXPECT_EQ(recording.getNumContacts(0), 2);

  // Test getContactPoint
  Eigen::Vector3d point0 = recording.getContactPoint(0, 0);
  EXPECT_DOUBLE_EQ(point0[0], 1.0);
  EXPECT_DOUBLE_EQ(point0[1], 2.0);
  EXPECT_DOUBLE_EQ(point0[2], 3.0);

  Eigen::Vector3d point1 = recording.getContactPoint(0, 1);
  EXPECT_DOUBLE_EQ(point1[0], 4.0);
  EXPECT_DOUBLE_EQ(point1[1], 5.0);
  EXPECT_DOUBLE_EQ(point1[2], 6.0);

  // Test getContactForce
  Eigen::Vector3d force0 = recording.getContactForce(0, 0);
  EXPECT_DOUBLE_EQ(force0[0], 10.0);
  EXPECT_DOUBLE_EQ(force0[1], 20.0);
  EXPECT_DOUBLE_EQ(force0[2], 30.0);

  Eigen::Vector3d force1 = recording.getContactForce(0, 1);
  EXPECT_DOUBLE_EQ(force1[0], 40.0);
  EXPECT_DOUBLE_EQ(force1[1], 50.0);
  EXPECT_DOUBLE_EQ(force1[2], 60.0);
}

//==============================================================================
TEST(RecordingTest, Clear)
{
  std::vector<int> dofs = {2};
  Recording recording(dofs);

  // Add some states
  Eigen::VectorXd state(2);
  state << 1.0, 2.0;
  recording.addState(state);
  recording.addState(state);
  recording.addState(state);

  EXPECT_EQ(recording.getNumFrames(), 3);

  // Clear and verify
  recording.clear();
  EXPECT_EQ(recording.getNumFrames(), 0);

  // Skeleton info should be preserved
  EXPECT_EQ(recording.getNumSkeletons(), 1);
  EXPECT_EQ(recording.getNumDofs(0), 2);
}

//==============================================================================
TEST(RecordingTest, UpdateNumGenCoords)
{
  // Start with one skeleton
  auto skel1 = createRevoluteJointSkeleton("skel1");
  std::vector<SkeletonPtr> skeletons1 = {skel1};
  Recording recording(skeletons1);

  EXPECT_EQ(recording.getNumSkeletons(), 1);
  EXPECT_EQ(recording.getNumDofs(0), 1);

  auto skel2 = createFreeJointSkeleton("skel2");
  auto skel3 = createFreeJointSkeleton("skel3");
  std::vector<SkeletonPtr> skeletons2 = {skel2, skel3};
  recording.updateNumGenCoords(skeletons2);

  EXPECT_EQ(recording.getNumSkeletons(), 2);
  EXPECT_EQ(recording.getNumDofs(0), 6);
  EXPECT_EQ(recording.getNumDofs(1), 6);
}

//==============================================================================
TEST(RecordingTest, MultipleFramesRetrieval)
{
  std::vector<int> dofs = {2};
  Recording recording(dofs);

  // Add multiple frames with different values
  for (int i = 0; i < 5; ++i) {
    Eigen::VectorXd state(2);
    state << static_cast<double>(i), static_cast<double>(i * 10);
    recording.addState(state);
  }

  EXPECT_EQ(recording.getNumFrames(), 5);

  // Verify each frame
  for (int i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(recording.getGenCoord(i, 0, 0), static_cast<double>(i));
    EXPECT_DOUBLE_EQ(
        recording.getGenCoord(i, 0, 1), static_cast<double>(i * 10));
  }
}

//==============================================================================
TEST(RecordingTest, EmptyRecording)
{
  std::vector<int> dofs = {3, 2};
  Recording recording(dofs);

  EXPECT_EQ(recording.getNumFrames(), 0);
  EXPECT_EQ(recording.getNumSkeletons(), 2);
  EXPECT_EQ(recording.getNumDofs(0), 3);
  EXPECT_EQ(recording.getNumDofs(1), 2);
}

//==============================================================================
TEST(RecordingTest, SingleSkeletonSingleDof)
{
  std::vector<int> dofs = {1};
  Recording recording(dofs);

  Eigen::VectorXd state(1);
  state << 0.5;
  recording.addState(state);

  EXPECT_EQ(recording.getNumFrames(), 1);
  EXPECT_EQ(recording.getNumSkeletons(), 1);
  EXPECT_EQ(recording.getNumDofs(0), 1);
  EXPECT_DOUBLE_EQ(recording.getGenCoord(0, 0, 0), 0.5);

  Eigen::VectorXd config = recording.getConfig(0, 0);
  EXPECT_EQ(config.size(), 1);
  EXPECT_DOUBLE_EQ(config[0], 0.5);
}

//==============================================================================
TEST(RecordingTest, NoContactsState)
{
  std::vector<int> dofs = {4};
  Recording recording(dofs);

  // State with only DOFs, no contacts
  Eigen::VectorXd state(4);
  state << 1.0, 2.0, 3.0, 4.0;
  recording.addState(state);

  EXPECT_EQ(recording.getNumContacts(0), 0);
}
