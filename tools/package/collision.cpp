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

#if defined(DART_TEST_BULLET_DOUBLE) && !defined(BT_USE_DOUBLE_PRECISION)
  #error The Bullet component must export its double-precision definition
#endif

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/collision/bullet/BulletCollisionGroup.hpp>
#include <dart/collision/ode/OdeCollisionDetector.hpp>
#include <dart/collision/ode/OdeCollisionGroup.hpp>

#include <btBulletCollisionCommon.h>
#include <ode/ode.h>

#include <type_traits>

static_assert(
    std::is_same<dReal, double>::value, "ODE must use double precision");
#ifdef DART_TEST_BULLET_DOUBLE
static_assert(
    std::is_same<btScalar, double>::value, "Bullet must use double precision");
#endif

int main()
{
  const auto ode = dart::collision::OdeCollisionDetector::create();
  const auto bullet = dart::collision::BulletCollisionDetector::create();
  return ode && bullet ? 0 : 1;
}
