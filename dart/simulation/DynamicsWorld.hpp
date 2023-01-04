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

#include <dart/simulation/EngineObject.hpp>
#include <dart/simulation/Fwd.hpp>

#include <string>
#include <vector>

namespace dart::simulation {

template <typename S>
class DynamicsWorld : public EngineObject<S>
{
public:
  using Scalar = S;

  explicit DynamicsWorld(Engine<S>* engine);
  ~DynamicsWorld() override;

  void setName(const std::string& name);
  [[nodiscard]] const std::string& getName() const;

  /// Creates a rigid body in edit mode
  ///
  /// Returns nullptr if:
  /// - it failed to allocate or initialize a rigid body, or
  /// - this world is in edit mode.
  RigidBody<S>* createRigidBody();

  void destroyRigidBody(RigidBody<S>* rigidBody);

  /// Sets the edit mode
  void setEditMode(bool editMode = true);

  /// Returns whether this world is in edit mode
  [[nodiscard]] bool isEditMode() const;

  /// Returns !isEditMode()
  [[nodiscard]] bool isSimulationMode() const;

  /// Spawns a rigid body in simulation mode
  RigidBody<S>* spawnRigidBody();

  bool step();

protected:
  bool loadImpl() override;
  void unloadImpl() override;

private:
  void switchToEditMode();
  void switchToSimulationMode();

  std::string mName;
  bool mEditMode{true};
  std::vector<RigidBody<S>*> mRigidBodies;
};

DART_TEMPLATE_CLASS_HEADER(SIMULATION, DynamicsWorld);

} // namespace dart::simulation

//==============================================================================
// Implementation
//==============================================================================

#include <dart/simulation/RigidBody.hpp>

#include <dart/common/Logging.hpp>
#include <dart/common/Macros.hpp>

#include <iostream>

#include <cassert>

namespace dart::simulation {

//==============================================================================
template <typename S>
DynamicsWorld<S>::DynamicsWorld(Engine<S>* engine) : EngineObject<S>(engine)
{
  assert(engine);
  assert(mEngine);
}

//==============================================================================
template <typename S>
DynamicsWorld<S>::~DynamicsWorld()
{
  assert(mEngine);
}

//==============================================================================
template <typename S>
void DynamicsWorld<S>::setName(const std::string& name)
{
  mName = name;
}

//==============================================================================
template <typename S>
const std::string& DynamicsWorld<S>::getName() const
{
  return mName;
}

//==============================================================================
template <typename S>
RigidBody<S>* DynamicsWorld<S>::createRigidBody()
{
  if (!mEditMode) {
    DART_WARN(
        "Cannot create a rigid body in simulation mode. Use spawnRigidBody() "
        "instead.");
    return nullptr;
  }

  auto body = new RigidBody(this);
  if (!body->load()) {
    delete body;
    return nullptr;
  }

  mRigidBodies.push_back(body);

  return body;
}

//==============================================================================
template <typename S>
void DynamicsWorld<S>::destroyRigidBody(RigidBody<S>* rigidBody)
{
  if (!mEditMode) {
    DART_WARN(
        "Cannot destroy a rigid body in simulation mode. Use spawnRigidBody() "
        "instead.");
    return;
  }

  const auto result
      = std::find(mRigidBodies.begin(), mRigidBodies.end(), rigidBody);
  if (result == mRigidBodies.end())
    return;

  mRigidBodies.erase(result);

  rigidBody->unload();

  delete rigidBody;
}

//==============================================================================
template <typename S>
void DynamicsWorld<S>::setEditMode(bool editMode)
{
  if (mEditMode == editMode)
    return;

  if (editMode)
    switchToEditMode();
  else
    switchToSimulationMode();

  mEditMode = editMode;
}

//==============================================================================
template <typename S>
bool DynamicsWorld<S>::isEditMode() const
{
  return mEditMode;
}

//==============================================================================
template <typename S>
bool DynamicsWorld<S>::isSimulationMode() const
{
  return !isEditMode();
}

//==============================================================================
template <typename S>
RigidBody<S>* DynamicsWorld<S>::spawnRigidBody()
{
  if (mEditMode) {
    DART_WARN(
        "Cannot spawn a rigid body in edit mode. Use createRigidBody() "
        "instead.");
    return nullptr;
  }

  return nullptr;
}

//==============================================================================
template <typename S>
bool DynamicsWorld<S>::step()
{
  DART_NOT_IMPLEMENTED;

  if (isEditMode())
    return false;

  return true;
}

//==============================================================================
template <typename S>
bool DynamicsWorld<S>::loadImpl()
{
  return true;
}

//==============================================================================
template <typename S>
void DynamicsWorld<S>::unloadImpl()
{
  // Switch to edit mode
  setEditMode(true);

  // Clear rigid bodies
  while (!mRigidBodies.empty())
    destroyRigidBody(mRigidBodies.front());
}

//==============================================================================
template <typename S>
void DynamicsWorld<S>::switchToEditMode()
{
  assert(!mEditMode);
}

//==============================================================================
template <typename S>
void DynamicsWorld<S>::switchToSimulationMode()
{
  assert(mEditMode);
}

} // namespace dart::simulation
