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

#include <gtest/gtest.h>

#include "dart/common/sub_ptr.hpp"
#include "dart/dynamics/SimpleFrame.hpp"
#include "dart/dynamics/BoxShape.hpp"

using namespace dart;
using namespace dynamics;

TEST(Subjects, Notifications)
{
  sub_ptr<Detachable> entity_ptr = new SimpleFrame(Frame::World(), "entity");
  sub_ptr<SimpleFrame> frame_ptr = new SimpleFrame(Frame::World(), "frame");

  EXPECT_TRUE(entity_ptr.valid());
  EXPECT_TRUE(frame_ptr.valid());

  delete entity_ptr;
  delete frame_ptr;

  EXPECT_FALSE(entity_ptr.valid());
  EXPECT_FALSE(frame_ptr.valid());

  EXPECT_TRUE(entity_ptr.get() == nullptr);
  EXPECT_TRUE(frame_ptr.get() == nullptr);
}

Entity* getPointer(Entity* _ptr)
{
  return _ptr;
}

TEST(Subjects, ImplicitConversion)
{
  sub_ptr<Entity> entity_ptr = new SimpleFrame(Frame::World(), "entity");

  // This checks whether the sub_ptr class can successfully be implicitly
  // converted to the type of class it's supposed to be pointing to
  EXPECT_TRUE( getPointer(entity_ptr) == entity_ptr.get() );

  delete entity_ptr;
}
