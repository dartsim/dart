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
 *   * This code incorporates portions of Open Dynamics Engine
 *     (Copyright (c) 2001-2004, Russell L. Smith. All rights
 *     reserved.) and portions of FCL (Copyright (c) 2011, Willow
 *     Garage, Inc. All rights reserved.), which were released under
 *     the same BSD license as below
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

#include <vector>

namespace dart::simulation {

template <typename S>
class Engine
{
public:
  using Scalar = S;

  explicit Engine(bool startOnCreation = true);
  ~Engine();

  void start();
  void stop();
  [[nodiscard]] bool isStarted() const;

  DynamicsWorld<S>* createDynamicsWorld();
  bool destroyDynamicsWorld(DynamicsWorld<S>* world);

private:
  bool mStarted{false};

  std::vector<DynamicsWorld<S>*> mWorlds;
};

DART_TEMPLATE_CLASS_HEADER(SIMULATION, Engine);

} // namespace dart::simulation

//==============================================================================
// Implementation
//==============================================================================

#include <dart/simulation/DynamicsWorld.hpp>

#include <algorithm>
#include <iostream>

#include <cassert>

namespace dart::simulation {

//==============================================================================
template <typename S>
Engine<S>::Engine(bool startOnCreation)
{
  if (startOnCreation)
    start();
}

//==============================================================================
template <typename S>
Engine<S>::~Engine()
{
  stop();
}

//==============================================================================
template <typename S>
void Engine<S>::start()
{
  if (mStarted)
    return;

  mStarted = true;
}

//==============================================================================
template <typename S>
void Engine<S>::stop()
{
  if (!mStarted) {
    assert(mWorlds.empty());
    return;
  }

  while (!mWorlds.empty())
    destroyDynamicsWorld(mWorlds.front());

  mStarted = false;
}

//==============================================================================
template <typename S>
bool Engine<S>::isStarted() const
{
  return mStarted;
}

//==============================================================================
template <typename S>
DynamicsWorld<S>* Engine<S>::createDynamicsWorld()
{
  if (!mStarted)
    return nullptr;

  auto world = new DynamicsWorld(this);

  if (!world->load()) {
    delete world;
    return nullptr;
  }

  mWorlds.push_back(world);

  return world;
}

//==============================================================================
template <typename S>
bool Engine<S>::destroyDynamicsWorld(DynamicsWorld<S>* world)
{
  if (!mStarted)
    return false;

  if (!world)
    return false;

  const auto result = std::find(mWorlds.begin(), mWorlds.end(), world);
  if (result == mWorlds.end())
    return false;

  mWorlds.erase(result);

  world->unload();

  delete world;

  return true;
}

} // namespace dart::simulation
