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

#include <dart/simulation/DynamicsWorldObject.hpp>
#include <dart/simulation/Fwd.hpp>

#include <string>

namespace dart::simulation {

template <typename S>
class RigidBody : public DynamicsWorldObject<S>
{
public:
  using Scalar = S;

  explicit RigidBody(DynamicsWorld<S>* engine);
  ~RigidBody() override;

  void setName(const std::string& name);
  [[nodiscard]] const std::string& getName() const;

protected:
  bool loadImpl() override;
  void unloadImpl() override;

private:
  std::string mName;
};

DART_TEMPLATE_CLASS_HEADER(SIMULATION, RigidBody);

} // namespace dart::simulation

//==============================================================================
// Implementation
//==============================================================================

#include <iostream>

#include <cassert>

namespace dart::simulation {

//==============================================================================
template <typename S>
RigidBody<S>::RigidBody(DynamicsWorld<S>* dynamicsWorld)
  : DynamicsWorldObject<S>(dynamicsWorld)
{
  DART_ASSERT(dynamicsWorld);
}

//==============================================================================
template <typename S>
RigidBody<S>::~RigidBody()
{
  // Do nothing
}

//==============================================================================
template <typename S>
void RigidBody<S>::setName(const std::string& name)
{
  mName = name;
}

//==============================================================================
template <typename S>
const std::string& RigidBody<S>::getName() const
{
  return mName;
}

//==============================================================================
template <typename S>
bool RigidBody<S>::loadImpl()
{
  return true;
}

//==============================================================================
template <typename S>
void RigidBody<S>::unloadImpl()
{
  // Do nothing
}

} // namespace dart::simulation
