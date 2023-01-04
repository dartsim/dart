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

namespace dart::simulation {

template <typename S>
class EngineObject
{
public:
  using Scalar = S;

  explicit EngineObject(Engine<S>* engine);
  virtual ~EngineObject();

  [[nodiscard]] const Engine<S>* getEngine() const;

  bool load();
  void unload();

protected:
  virtual bool loadImpl() = 0;
  virtual void unloadImpl() = 0;
  Engine<S>* mEngine{nullptr};
  bool mInitialized{false};
};

DART_TEMPLATE_CLASS_HEADER(SIMULATION, EngineObject);

} // namespace dart::simulation

namespace dart::simulation {

//==============================================================================
template <typename S>
EngineObject<S>::EngineObject(Engine<S>* engine) : mEngine(engine)
{
  assert(engine);
}

//==============================================================================
template <typename S>
EngineObject<S>::~EngineObject()
{
  assert(!mInitialized);

  // Engine should always live longer than engine objects.
  assert(mEngine);
}

//==============================================================================
template <typename S>
const Engine<S>* EngineObject<S>::getEngine() const
{
  return mEngine;
}

//==============================================================================
template <typename S>
bool EngineObject<S>::load()
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
void EngineObject<S>::unload()
{
  if (!mInitialized)
    return;

  unloadImpl();
  mInitialized = false;
}

} // namespace dart::simulation
