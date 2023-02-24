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

#include <dart/common/Containers.hpp>

#include <dart/physics/Fwd.hpp>

namespace dart::physics {

template <typename S = double>
class RigidBody
{
public:
  using Scalar = S;

  /// Constructor
  explicit RigidBody(RigidBodyBatch<S>* batch);

  /// Destructor
  virtual ~RigidBody();

  [[nodiscard]] auto getPosition() const;

private:
  friend class RigidBodyBatch<S>;

  /// Initialize the RigidBody
  virtual bool init(size_t index)
  {
    m_index = index;
    return true;
  }

  /// Finalize the RigidBody
  virtual void finalize()
  {
    // Empty
  }

  [[nodiscard]] size_t getIndex() const
  {
    return m_index;
  }

  size_t m_index;
  RigidBodyBatch<S>* m_batch;
};

DART_TEMPLATE_CLASS_HEADER(PHYSICS, RigidBody);

} // namespace dart::physics

//==============================================================================
// Implementation
//==============================================================================

#include <dart/physics/RigidBodyBatch.hpp>

namespace dart::physics {

//==============================================================================
template <typename S>
RigidBody<S>::RigidBody(RigidBodyBatch<S>* batch) : m_batch(batch)
{
  DART_ASSERT(m_batch);
}

//==============================================================================
template <typename S>
RigidBody<S>::~RigidBody()
{
  // Do nothing
}

//==============================================================================
template <typename S>
auto RigidBody<S>::getPosition() const
{
  return m_batch->getPosition(m_index);
}

} // namespace dart::physics
