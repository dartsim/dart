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
 *   * Redistributions of this software must retain the above copyright
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

#include "dart/config.hpp"

#include <gtest/gtest.h>

#if DART_HAVE_BULLET
  #include "dart/collision/bullet/detail/bullet_contact.hpp"
  #include "dart/collision/collision_option.hpp"
#endif

using namespace dart;
using namespace dart::collision;

#if DART_HAVE_BULLET
using dart::collision::bullet::detail::shouldReportContact;

TEST(BulletContact, FiltersNegativePenetrationDepth)
{
  CollisionOption option;

  btManifoldPoint cp;
  cp.m_normalWorldOnB = btVector3(0, 0, 1);
  cp.m_distance1 = 0.01; // positive => negative penetration depth

  EXPECT_FALSE(shouldReportContact(cp, option));

  option.allowNegativePenetrationDepthContacts = true;
  EXPECT_TRUE(shouldReportContact(cp, option));
}

TEST(BulletContact, StillReportsPenetratingContacts)
{
  CollisionOption option;

  btManifoldPoint cp;
  cp.m_normalWorldOnB = btVector3(0, 0, 1);
  cp.m_distance1 = -0.02;

  EXPECT_TRUE(shouldReportContact(cp, option));
}

TEST(BulletContact, RejectsZeroLengthNormals)
{
  CollisionOption option;
  option.allowNegativePenetrationDepthContacts = true;

  btManifoldPoint cp;
  cp.m_normalWorldOnB = btVector3(0, 0, 0);
  cp.m_distance1 = -0.01;

  EXPECT_FALSE(shouldReportContact(cp, option));
}
#endif
