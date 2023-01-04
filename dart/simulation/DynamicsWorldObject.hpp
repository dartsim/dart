/*
 * Copyright (c) 2011-2023, The DART development contributors
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
 *
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

#include <dart/simulation/Fwd.hpp>

namespace dart::simulation {

template <typename S>
class DynamicsWorldObject
{
public:
  using Scalar = S;

  explicit DynamicsWorldObject(DynamicsWorld<S>* dynamicsWorld);
  virtual ~DynamicsWorldObject();

  [[nodiscard]] const DynamicsWorld<S>* getDynamicsWorld() const;

  bool load();
  void unload();

protected:
  virtual bool loadImpl() = 0;
  virtual void unloadImpl() = 0;
  DynamicsWorld<S>* mDynamicsWorld{nullptr};
  bool mInitialized{false};
};

DART_TEMPLATE_CLASS_HEADER(SIMULATION, DynamicsWorldObject);

} // namespace dart::simulation

//==============================================================================
// Implementation
//==============================================================================

namespace dart::simulation {

//==============================================================================
template <typename S>
DynamicsWorldObject<S>::DynamicsWorldObject(DynamicsWorld<S>* dynamicsWorld)
  : mDynamicsWorld(dynamicsWorld)
{
  DART_ASSERT(dynamicsWorld);
}

//==============================================================================
template <typename S>
DynamicsWorldObject<S>::~DynamicsWorldObject()
{
  DART_ASSERT(!mInitialized);

  // Engine should always live longer than engine objects.
  DART_ASSERT(mDynamicsWorld);
}

//==============================================================================
template <typename S>
const DynamicsWorld<S>* DynamicsWorldObject<S>::getDynamicsWorld() const
{
  return mDynamicsWorld;
}

//==============================================================================
template <typename S>
bool DynamicsWorldObject<S>::load()
{
  if (mInitialized)
    return false;

  if (!loadImpl())
    return false;

  mInitialized = true;
  return true;
}

//==============================================================================
template <typename S>
void DynamicsWorldObject<S>::unload()
{
  if (!mInitialized)
    return;

  unloadImpl();
  mInitialized = false;
}

} // namespace dart::simulation
