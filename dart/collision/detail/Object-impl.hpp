/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#pragma once

#include "dart/collision/Object.hpp"

#include <cassert>

#include "dart/collision/Engine.hpp"
#include "dart/collision/Group.hpp"

namespace dart {
namespace collision2 {

//==============================================================================
template <typename S>
Object<S>::Object()
{
  // Do nothing
}

//==============================================================================
template <typename S>
Object<S>::~Object()
{
  // Do nothing
}

//==============================================================================
template <typename S>
Engine<S>* Object<S>::getEngine()
{
  return mGroup->getEngine();
}

//==============================================================================
template <typename S>
const Engine<S>* Object<S>::getEngine() const
{
  return mGroup->getEngine();
}

//==============================================================================
template <typename S>
const void* Object<S>::getUserData() const
{
  return mUserData;
}

//==============================================================================
template <typename S>
math::ConstGeometryPtr Object<S>::getShape() const
{
  return mShape;
}

//==============================================================================
template <typename S>
Object<S>::Object(Group<S>* collisionGroup, math::GeometryPtr shape)
  : mGroup(collisionGroup), mShape(std::move(shape))
{
  assert(mGroup);
  assert(mShape);
}

} // namespace collision2
} // namespace dart
