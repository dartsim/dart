/*
 * Copyright (c) 2011-2025, The DART development contributors
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

// Regression test for https://github.com/dartsim/dart/issues/3447.
//
// Classes that inherit Frame virtually get their complete-object alignment
// (16/32/64 bytes, from Frame's Eigen members) from that virtual base only.
// Under the Itanium C++ ABI a derived class may then place such a base at an
// offset that is merely 8-byte aligned (SimpleFrame placed ShapeFrame at
// offset 8), while GCC's base-object constructors assume the full alignment
// of `this` and emit aligned vector stores that fault. The fix declares those
// classes alignas(Frame); this test guards that rule for every class in the
// Frame family.

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/EndEffector.hpp>
#include <dart/dynamics/EntityNode.hpp>
#include <dart/dynamics/FixedFrame.hpp>
#include <dart/dynamics/FixedJacobianNode.hpp>
#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/JacobianNode.hpp>
#include <dart/dynamics/Marker.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/dynamics/SphereShape.hpp>
#include <dart/dynamics/TemplatedJacobianNode.hpp>

#include <dart/common/Virtual.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include <cstddef>
#include <cstdint>

using namespace dart;
using namespace dart::dynamics;

namespace {

struct Vptr
{
  virtual ~Vptr() = default;
};

// Offset at which a derived class places C as a non-primary, non-virtual base.
// The Itanium ABI only guarantees nvalign(C) for that placement; the rule
// under test requires it to be alignof(C). No object is ever created: Probe is
// never instantiated (C may be abstract), and the derived-to-base conversion
// of a non-virtual base is a compile-time constant offset, so the deliberate
// fake pointer is never dereferenced.
template <class C>
std::ptrdiff_t nonVirtualBaseOffset()
{
  struct Probe : Vptr, C
  {
  };
  const auto probeAddress = static_cast<std::uintptr_t>(alignof(Probe)) * 64u;
  auto* probe = reinterpret_cast<Probe*>(probeAddress);
  const auto baseAddress
      = reinterpret_cast<std::uintptr_t>(static_cast<C*>(probe));
  return static_cast<std::ptrdiff_t>(baseAddress - probeAddress);
}

template <class C>
bool nonVirtualPartIsFullyAligned()
{
  return nonVirtualBaseOffset<C>() % static_cast<std::ptrdiff_t>(alignof(C))
         == 0;
}

template <class Base, class Derived>
bool baseSubobjectIsAligned(const Derived* object)
{
  const auto address
      = reinterpret_cast<std::uintptr_t>(static_cast<const Base*>(object));
  return address % alignof(Base) == 0;
}

} // namespace

//==============================================================================
TEST(FrameBaseAlignment, NonVirtualPartIsAsAlignedAsTheClass)
{
  if (alignof(Frame) <= 8) {
    GTEST_SKIP() << "Eigen static alignment is disabled, so alignas(Frame) is "
                    "a no-op and the base placement is trivially aligned.";
  }

  // Classes that name `virtual Frame` directly.
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<ShapeFrame>()));
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<JacobianNode>()));
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<FixedFrame>()));

  // Classes that reach Frame through the generic virtual-inheritance helper
  // and through non-virtual derivation.
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<common::Virtual<FixedFrame>>()));
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<FixedJacobianNode>()));
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<ShapeNode>()));
  EXPECT_TRUE(
      (nonVirtualPartIsFullyAligned<TemplatedJacobianNode<BodyNode>>()));
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<
               TemplatedJacobianNode<FixedJacobianNode>>()));
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<
               EntityNode<TemplatedJacobianNode<FixedJacobianNode>>>()));
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<
               common::CompositeJoiner<FixedJacobianNode, ShapeFrame>>()));
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<BodyNode>()));
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<SoftBodyNode>()));
  EXPECT_TRUE((nonVirtualPartIsFullyAligned<SimpleFrame>()));
}

//==============================================================================
TEST(FrameBaseAlignment, ConstructedObjectsPlaceBasesAtAlignedAddresses)
{
  // The gz-physics path that crashed in #3447: allocation through the
  // header-inline createShared, construction inside libdart.
  const SimpleFramePtr simpleFrame
      = SimpleFrame::createShared(Frame::World(), "frame");
  ASSERT_NE(simpleFrame, nullptr);
  EXPECT_TRUE((baseSubobjectIsAligned<ShapeFrame>(simpleFrame.get())));
  EXPECT_TRUE((baseSubobjectIsAligned<Frame>(simpleFrame.get())));

  const SkeletonPtr skeleton = Skeleton::create("skeleton");
  BodyNode* body = skeleton->createJointAndBodyNodePair<FreeJoint>().second;
  ASSERT_NE(body, nullptr);
  EXPECT_TRUE((baseSubobjectIsAligned<TemplatedJacobianNode<BodyNode>>(body)));
  EXPECT_TRUE((baseSubobjectIsAligned<JacobianNode>(body)));
  EXPECT_TRUE((baseSubobjectIsAligned<Frame>(body)));

  ShapeNode* shapeNode = body->createShapeNodeWith<VisualAspect>(
      std::make_shared<SphereShape>(0.1));
  ASSERT_NE(shapeNode, nullptr);
  EXPECT_TRUE((baseSubobjectIsAligned<ShapeFrame>(shapeNode)));
  EXPECT_TRUE((baseSubobjectIsAligned<FixedJacobianNode>(shapeNode)));
  EXPECT_TRUE((baseSubobjectIsAligned<Frame>(shapeNode)));

  // EndEffector and Marker are final, so they can only be checked as objects.
  EndEffector* endEffector = body->createEndEffector("end_effector");
  ASSERT_NE(endEffector, nullptr);
  EXPECT_TRUE((baseSubobjectIsAligned<FixedJacobianNode>(endEffector)));
  EXPECT_TRUE((baseSubobjectIsAligned<Frame>(endEffector)));

  Marker* marker = body->createMarker(std::string("marker"));
  ASSERT_NE(marker, nullptr);
  EXPECT_TRUE((baseSubobjectIsAligned<FixedJacobianNode>(marker)));
  EXPECT_TRUE((baseSubobjectIsAligned<Frame>(marker)));
}
