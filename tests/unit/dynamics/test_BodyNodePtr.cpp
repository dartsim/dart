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
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <gtest/gtest.h>

using namespace dart;
using namespace dart::dynamics;

namespace {

// Helper to create a simple skeleton with one body
SkeletonPtr createSingleBodySkeleton()
{
  auto skeleton = Skeleton::create("single_body");

  BodyNode::Properties bodyProps;
  bodyProps.mName = "body";

  FreeJoint::Properties jointProps;
  jointProps.mName = "joint";

  skeleton->createJointAndBodyNodePair<FreeJoint>(
      nullptr, jointProps, bodyProps);

  return skeleton;
}

// Helper to create a chain skeleton with multiple bodies
SkeletonPtr createChainSkeleton(std::size_t numBodies)
{
  auto skeleton = Skeleton::create("chain");

  BodyNode* parent = nullptr;
  for (std::size_t i = 0; i < numBodies; ++i) {
    BodyNode::Properties bodyProps;
    bodyProps.mName = "body_" + std::to_string(i);

    RevoluteJoint::Properties jointProps;
    jointProps.mName = "joint_" + std::to_string(i);

    auto pair = skeleton->createJointAndBodyNodePair<RevoluteJoint>(
        parent, jointProps, bodyProps);
    parent = pair.second;
  }

  return skeleton;
}

} // namespace

//==============================================================================
// TemplateBodyNodePtr tests
//==============================================================================

TEST(BodyNodePtrTest, DefaultConstructor)
{
  BodyNodePtr ptr;
  EXPECT_EQ(ptr.get(), nullptr);
  EXPECT_FALSE(ptr);
}

TEST(BodyNodePtrTest, ConstructFromRawPointer)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr ptr(rawPtr);
  EXPECT_EQ(ptr.get(), rawPtr);
  EXPECT_TRUE(ptr);
}

TEST(BodyNodePtrTest, ConstructFromNullptr)
{
  BodyNodePtr ptr(nullptr);
  EXPECT_EQ(ptr.get(), nullptr);
  EXPECT_FALSE(ptr);
}

TEST(BodyNodePtrTest, CopyConstructor)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr ptr1(rawPtr);
  BodyNodePtr ptr2(ptr1);

  EXPECT_EQ(ptr1.get(), ptr2.get());
  EXPECT_EQ(ptr2.get(), rawPtr);
}

TEST(BodyNodePtrTest, CopyConstructorFromConstPtr)
{
  auto skeleton = createSingleBodySkeleton();
  const BodyNode* rawPtr = skeleton->getBodyNode(0);

  ConstBodyNodePtr ptr1(rawPtr);
  ConstBodyNodePtr ptr2(ptr1);

  EXPECT_EQ(ptr1.get(), ptr2.get());
  EXPECT_EQ(ptr2.get(), rawPtr);
}

TEST(BodyNodePtrTest, AssignmentOperator)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr ptr1(rawPtr);
  BodyNodePtr ptr2;
  ptr2 = ptr1;

  EXPECT_EQ(ptr1.get(), ptr2.get());
}

TEST(BodyNodePtrTest, AssignmentFromRawPointer)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr ptr;
  ptr = rawPtr;

  EXPECT_EQ(ptr.get(), rawPtr);
}

TEST(BodyNodePtrTest, AssignmentFromNullptr)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr ptr(rawPtr);
  EXPECT_NE(ptr.get(), nullptr);

  ptr = nullptr;
  EXPECT_EQ(ptr.get(), nullptr);
}

TEST(BodyNodePtrTest, DereferenceOperator)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr ptr(rawPtr);
  EXPECT_EQ(&(*ptr), rawPtr);
  EXPECT_EQ((*ptr).getName(), "body");
}

TEST(BodyNodePtrTest, ArrowOperator)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr ptr(rawPtr);
  EXPECT_EQ(ptr->getName(), "body");
}

TEST(BodyNodePtrTest, ImplicitConversion)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr ptr(rawPtr);

  // Implicit conversion to raw pointer
  BodyNode* converted = ptr;
  EXPECT_EQ(converted, rawPtr);
}

TEST(BodyNodePtrTest, SetMethod)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr ptr;
  ptr.set(rawPtr);
  EXPECT_EQ(ptr.get(), rawPtr);

  ptr.set(nullptr);
  EXPECT_EQ(ptr.get(), nullptr);
}

TEST(BodyNodePtrTest, SetSamePointer)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr ptr(rawPtr);
  ptr.set(rawPtr); // Should be a no-op
  EXPECT_EQ(ptr.get(), rawPtr);
}

TEST(BodyNodePtrTest, KeepsSkeletonAlive)
{
  BodyNodePtr ptr;
  std::weak_ptr<Skeleton> weakSkeleton;

  {
    auto skeleton = createSingleBodySkeleton();
    weakSkeleton = skeleton;
    ptr = skeleton->getBodyNode(0);

    // skeleton goes out of scope, but ptr should keep it alive
  }

  EXPECT_FALSE(weakSkeleton.expired());
  EXPECT_NE(ptr.get(), nullptr);
  EXPECT_EQ(ptr->getName(), "body");

  ptr = nullptr;
  EXPECT_TRUE(weakSkeleton.expired());
}

TEST(BodyNodePtrTest, MultiplePointersKeepSkeletonAlive)
{
  std::weak_ptr<Skeleton> weakSkeleton;
  BodyNodePtr ptr1;
  BodyNodePtr ptr2;

  {
    auto skeleton = createSingleBodySkeleton();
    weakSkeleton = skeleton;
    ptr1 = skeleton->getBodyNode(0);
    ptr2 = skeleton->getBodyNode(0);
  }

  EXPECT_FALSE(weakSkeleton.expired());

  ptr1 = nullptr;
  EXPECT_FALSE(weakSkeleton.expired()); // ptr2 still holds reference

  ptr2 = nullptr;
  EXPECT_TRUE(weakSkeleton.expired());
}

TEST(BodyNodePtrTest, ChainSkeletonMultipleBodies)
{
  std::weak_ptr<Skeleton> weakSkeleton;
  BodyNodePtr ptr0;
  BodyNodePtr ptr1;
  BodyNodePtr ptr2;

  {
    auto skeleton = createChainSkeleton(3);
    weakSkeleton = skeleton;
    ptr0 = skeleton->getBodyNode(0);
    ptr1 = skeleton->getBodyNode(1);
    ptr2 = skeleton->getBodyNode(2);
  }

  EXPECT_FALSE(weakSkeleton.expired());
  EXPECT_EQ(ptr0->getName(), "body_0");
  EXPECT_EQ(ptr1->getName(), "body_1");
  EXPECT_EQ(ptr2->getName(), "body_2");

  ptr0 = nullptr;
  ptr1 = nullptr;
  EXPECT_FALSE(weakSkeleton.expired());

  ptr2 = nullptr;
  EXPECT_TRUE(weakSkeleton.expired());
}

//==============================================================================
// TemplateWeakBodyNodePtr tests
//==============================================================================

TEST(WeakBodyNodePtrTest, DefaultConstructor)
{
  WeakBodyNodePtr ptr;
  EXPECT_TRUE(ptr.expired());
  EXPECT_EQ(ptr.lock().get(), nullptr);
}

TEST(WeakBodyNodePtrTest, ConstructFromRawPointer)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  WeakBodyNodePtr weak(rawPtr);
  EXPECT_FALSE(weak.expired());

  auto strong = weak.lock();
  EXPECT_EQ(strong.get(), rawPtr);
}

TEST(WeakBodyNodePtrTest, ConstructFromNullptr)
{
  WeakBodyNodePtr weak(nullptr);
  EXPECT_TRUE(weak.expired());
}

TEST(WeakBodyNodePtrTest, CopyConstructor)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  WeakBodyNodePtr weak1(rawPtr);
  WeakBodyNodePtr weak2(weak1);

  EXPECT_FALSE(weak1.expired());
  EXPECT_FALSE(weak2.expired());
  EXPECT_EQ(weak1.lock().get(), weak2.lock().get());
}

TEST(WeakBodyNodePtrTest, AssignmentFromRawPointer)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  WeakBodyNodePtr weak;
  weak = rawPtr;

  EXPECT_FALSE(weak.expired());
  EXPECT_EQ(weak.lock().get(), rawPtr);
}

TEST(WeakBodyNodePtrTest, AssignmentFromWeakPtr)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  WeakBodyNodePtr weak1(rawPtr);
  WeakBodyNodePtr weak2;
  weak2 = weak1;

  EXPECT_FALSE(weak2.expired());
  EXPECT_EQ(weak1.lock().get(), weak2.lock().get());
}

TEST(WeakBodyNodePtrTest, ExpiredAfterSkeletonDeleted)
{
  WeakBodyNodePtr weak;

  {
    auto skeleton = createSingleBodySkeleton();
    weak = skeleton->getBodyNode(0);
    EXPECT_FALSE(weak.expired());
  }

  EXPECT_TRUE(weak.expired());
  EXPECT_EQ(weak.lock().get(), nullptr);
}

TEST(WeakBodyNodePtrTest, LockReturnsValidPtrWhileSkeletonAlive)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  WeakBodyNodePtr weak(rawPtr);
  BodyNodePtr strong = weak.lock();

  EXPECT_EQ(strong.get(), rawPtr);
  EXPECT_FALSE(weak.expired());
}

TEST(WeakBodyNodePtrTest, LockKeepsSkeletonAlive)
{
  WeakBodyNodePtr weak;
  std::weak_ptr<Skeleton> weakSkeleton;

  {
    auto skeleton = createSingleBodySkeleton();
    weakSkeleton = skeleton;
    weak = skeleton->getBodyNode(0);
  }

  // Skeleton should be expired since only weak reference exists
  EXPECT_TRUE(weakSkeleton.expired());
  EXPECT_TRUE(weak.expired());

  // Lock should return nullptr since skeleton was deleted
  EXPECT_EQ(weak.lock().get(), nullptr);
}

TEST(WeakBodyNodePtrTest, LockWhileStrongRefExists)
{
  BodyNodePtr strong;
  WeakBodyNodePtr weak;
  std::weak_ptr<Skeleton> weakSkeleton;

  {
    auto skeleton = createSingleBodySkeleton();
    weakSkeleton = skeleton;
    strong = skeleton->getBodyNode(0);
    weak = skeleton->getBodyNode(0);
  }

  // Skeleton should still be alive due to strong ref
  EXPECT_FALSE(weakSkeleton.expired());
  EXPECT_FALSE(weak.expired());

  // Lock should succeed
  auto locked = weak.lock();
  EXPECT_EQ(locked.get(), strong.get());
}

TEST(WeakBodyNodePtrTest, SetNullptr)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  WeakBodyNodePtr weak(rawPtr);
  EXPECT_FALSE(weak.expired());

  weak.set(nullptr);
  EXPECT_TRUE(weak.expired());
}

TEST(WeakBodyNodePtrTest, SetFromExpiredWeakPtr)
{
  WeakBodyNodePtr weak1;
  WeakBodyNodePtr weak2;

  {
    auto skeleton = createSingleBodySkeleton();
    weak1 = skeleton->getBodyNode(0);
  }

  // weak1 is now expired
  EXPECT_TRUE(weak1.expired());

  // Assigning an expired weak ptr should result in null
  weak2 = weak1;
  EXPECT_TRUE(weak2.expired());
}

//==============================================================================
// ConstBodyNodePtr tests
//==============================================================================

TEST(ConstBodyNodePtrTest, ConstructFromConstPointer)
{
  auto skeleton = createSingleBodySkeleton();
  const BodyNode* rawPtr = skeleton->getBodyNode(0);

  ConstBodyNodePtr ptr(rawPtr);
  EXPECT_EQ(ptr.get(), rawPtr);
}

TEST(ConstBodyNodePtrTest, ConstructFromNonConstPtr)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  // Should be able to construct const ptr from non-const
  ConstBodyNodePtr constPtr(rawPtr);
  EXPECT_EQ(constPtr.get(), rawPtr);
}

TEST(ConstBodyNodePtrTest, AssignFromBodyNodePtr)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  BodyNodePtr nonConstPtr(rawPtr);
  ConstBodyNodePtr constPtr;
  constPtr = nonConstPtr;

  EXPECT_EQ(constPtr.get(), rawPtr);
}

//==============================================================================
// WeakConstBodyNodePtr tests
//==============================================================================

TEST(WeakConstBodyNodePtrTest, BasicUsage)
{
  auto skeleton = createSingleBodySkeleton();
  const BodyNode* rawPtr = skeleton->getBodyNode(0);

  WeakConstBodyNodePtr weak(rawPtr);
  EXPECT_FALSE(weak.expired());

  auto strong = weak.lock();
  EXPECT_EQ(strong.get(), rawPtr);
}

TEST(WeakConstBodyNodePtrTest, CopyFromNonConstWeakPtr)
{
  auto skeleton = createSingleBodySkeleton();
  BodyNode* rawPtr = skeleton->getBodyNode(0);

  WeakBodyNodePtr nonConstWeak(rawPtr);
  WeakConstBodyNodePtr constWeak(nonConstWeak);

  EXPECT_FALSE(constWeak.expired());
  EXPECT_EQ(constWeak.lock().get(), rawPtr);
}
