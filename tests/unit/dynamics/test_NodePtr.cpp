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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

namespace {

// Helper to create a skeleton with a body and shape node
SkeletonPtr createSkeletonWithShapeNode()
{
  auto skeleton = Skeleton::create("test_skeleton");

  BodyNode::Properties bodyProps;
  bodyProps.mName = "body";

  FreeJoint::Properties jointProps;
  jointProps.mName = "joint";

  auto pair = skeleton->createJointAndBodyNodePair<FreeJoint>(
      nullptr, jointProps, bodyProps);
  BodyNode* body = pair.second;

  auto shape = std::make_shared<EllipsoidShape>(Eigen::Vector3d::Constant(1.0));
  body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(
      shape, "shape_node");

  return skeleton;
}

} // namespace

//==============================================================================
// TemplateNodePtr tests (using ShapeNodePtr)
//==============================================================================

TEST(NodePtrTest, DefaultConstructor)
{
  ShapeNodePtr ptr;
  EXPECT_EQ(ptr.get(), nullptr);
  EXPECT_FALSE(ptr);
}

TEST(NodePtrTest, ConstructFromRawPointer)
{
  auto skeleton = createSkeletonWithShapeNode();
  ShapeNode* rawPtr = skeleton->getBodyNode(0)->getShapeNode(0);

  ShapeNodePtr ptr(rawPtr);
  EXPECT_EQ(ptr.get(), rawPtr);
  EXPECT_TRUE(ptr);
}

TEST(NodePtrTest, ConstructFromNullptr)
{
  ShapeNodePtr ptr(nullptr);
  EXPECT_EQ(ptr.get(), nullptr);
  EXPECT_FALSE(ptr);
}

TEST(NodePtrTest, CopyConstructor)
{
  auto skeleton = createSkeletonWithShapeNode();
  ShapeNode* rawPtr = skeleton->getBodyNode(0)->getShapeNode(0);

  ShapeNodePtr ptr1(rawPtr);
  ShapeNodePtr ptr2(ptr1);

  EXPECT_EQ(ptr1.get(), ptr2.get());
  EXPECT_EQ(ptr2.get(), rawPtr);
}

TEST(NodePtrTest, AssignmentFromRawPointer)
{
  auto skeleton = createSkeletonWithShapeNode();
  ShapeNode* rawPtr = skeleton->getBodyNode(0)->getShapeNode(0);

  ShapeNodePtr ptr;
  ptr = rawPtr;

  EXPECT_EQ(ptr.get(), rawPtr);
}

TEST(NodePtrTest, DereferenceOperator)
{
  auto skeleton = createSkeletonWithShapeNode();
  ShapeNode* rawPtr = skeleton->getBodyNode(0)->getShapeNode(0);

  ShapeNodePtr ptr(rawPtr);
  EXPECT_EQ(&(*ptr), rawPtr);
  EXPECT_EQ((*ptr).getName(), "shape_node");
}

TEST(NodePtrTest, ArrowOperator)
{
  auto skeleton = createSkeletonWithShapeNode();
  ShapeNode* rawPtr = skeleton->getBodyNode(0)->getShapeNode(0);

  ShapeNodePtr ptr(rawPtr);
  EXPECT_EQ(ptr->getName(), "shape_node");
}

TEST(NodePtrTest, GetMethod)
{
  auto skeleton = createSkeletonWithShapeNode();
  ShapeNode* rawPtr = skeleton->getBodyNode(0)->getShapeNode(0);

  ShapeNodePtr ptr(rawPtr);
  EXPECT_EQ(ptr.get(), rawPtr);

  ShapeNodePtr nullPtr;
  EXPECT_EQ(nullPtr.get(), nullptr);
}

TEST(NodePtrTest, SetMethod)
{
  auto skeleton = createSkeletonWithShapeNode();
  ShapeNode* rawPtr = skeleton->getBodyNode(0)->getShapeNode(0);

  ShapeNodePtr ptr;
  ptr.set(rawPtr);
  EXPECT_EQ(ptr.get(), rawPtr);

  ptr.set(nullptr);
  EXPECT_EQ(ptr.get(), nullptr);
}

TEST(NodePtrTest, KeepsSkeletonAlive)
{
  ShapeNodePtr ptr;
  std::weak_ptr<Skeleton> weakSkeleton;

  {
    auto skeleton = createSkeletonWithShapeNode();
    weakSkeleton = skeleton;
    ptr = skeleton->getBodyNode(0)->getShapeNode(0);

    // skeleton shared_ptr goes out of scope, but ptr should keep it alive
  }

  EXPECT_FALSE(weakSkeleton.expired());
  EXPECT_NE(ptr.get(), nullptr);
  EXPECT_EQ(ptr->getName(), "shape_node");

  ptr = nullptr;
  EXPECT_TRUE(weakSkeleton.expired());
}

//==============================================================================
// TemplateWeakNodePtr tests (using WeakShapeNodePtr)
//==============================================================================

TEST(WeakNodePtrTest, WeakDefaultConstructor)
{
  WeakShapeNodePtr weak;
  EXPECT_EQ(weak.lock().get(), nullptr);
}

TEST(WeakNodePtrTest, WeakConstructFromRawPointer)
{
  auto skeleton = createSkeletonWithShapeNode();
  ShapeNode* rawPtr = skeleton->getBodyNode(0)->getShapeNode(0);

  WeakShapeNodePtr weak(rawPtr);
  EXPECT_NE(weak.lock().get(), nullptr);
  EXPECT_EQ(weak.lock().get(), rawPtr);
}

TEST(WeakNodePtrTest, WeakLockReturnsValidPtr)
{
  auto skeleton = createSkeletonWithShapeNode();
  ShapeNode* rawPtr = skeleton->getBodyNode(0)->getShapeNode(0);

  WeakShapeNodePtr weak(rawPtr);
  ShapeNodePtr strong = weak.lock();

  EXPECT_EQ(strong.get(), rawPtr);
  EXPECT_EQ(strong->getName(), "shape_node");
}

TEST(WeakNodePtrTest, WeakExpiredAfterSkeletonDeleted)
{
  WeakShapeNodePtr weak;

  {
    auto skeleton = createSkeletonWithShapeNode();
    weak = skeleton->getBodyNode(0)->getShapeNode(0);
    EXPECT_NE(weak.lock().get(), nullptr);
  }

  // Skeleton is deleted, weak should now return nullptr on lock
  EXPECT_EQ(weak.lock().get(), nullptr);
}

TEST(WeakNodePtrTest, WeakSetNullptr)
{
  auto skeleton = createSkeletonWithShapeNode();
  ShapeNode* rawPtr = skeleton->getBodyNode(0)->getShapeNode(0);

  WeakShapeNodePtr weak(rawPtr);
  EXPECT_NE(weak.lock().get(), nullptr);

  weak.set(nullptr);
  EXPECT_EQ(weak.lock().get(), nullptr);
}
