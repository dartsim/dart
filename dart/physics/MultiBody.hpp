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

#include <dart/physics/MultiBodyBatchData.hpp>
#include <dart/physics/MultiBodyJoint.hpp>
#include <dart/physics/MultiBodyLink.hpp>

namespace dart::physics {

template <typename S = double>
class MultiBody
{
public:
  using Scalar = S;

  /// Constructor
  explicit MultiBody(MultiBodyBatch<S>* batch);

  /// Destructor
  virtual ~MultiBody();

  /// Returns the degree of freedom
  [[nodiscard]] size_t getNumDofs() const;

  /// Returns the generalized positions of the given MultiBody.
  ///
  /// @return The generalized positions of the given MultiBody.
  [[nodiscard]] auto getPositions() const;

  /// @{ @name Structural Changes

  MultiBodyLink<S>* createLink()
  {
    return nullptr;
  }

  /// @}

private:
  friend class MultiBodyBatch<S>;

  /// Initialize the MultiBody
  virtual bool init(MultiBodyBatchData<S>* data)
  {
    DART_ASSERT(data);
    m_multibody_index = 0;

    m_data = data;
    m_data->reset();

    return true;
  }

  /// Finalize the MultiBody
  virtual void finalize()
  {
    // Empty
  }

  /// Set the MultiBody index
  void setMultiBodyIndex(size_t index)
  {
    m_multibody_index = index;
  }

  /// Returns the MultiBody index
  [[nodiscard]] size_t getMultiBodyIndex() const
  {
    return m_multibody_index;
  }

  /// Returns the MultiBodyBatch
  [[nodiscard]] MultiBodyBatchData<S>* getMultiBodyBatchData()
  {
    return m_data;
  }

  /// Returns the MultiBodyBatch
  [[nodiscard]] const MultiBodyBatchData<S>* getMultiBodyBatchData() const
  {
    return m_data;
  }

  MultiBodyBatch<S>* m_batch;
  MultiBodyBatchData<S>* m_data;
  size_t m_multibody_index{0};

  // Links
  std::vector<MultiBodyLink<S>*> mLinks;
};

DART_TEMPLATE_CLASS_HEADER(PHYSICS, MultiBody);

} // namespace dart::physics

//==============================================================================
// Implementation
//==============================================================================

#include <dart/common/Macros.hpp>

#include <dart/physics/MultiBodyBatch.hpp>
#include <dart/physics/World.hpp>

namespace dart::physics {

//==============================================================================
template <typename S>
MultiBody<S>::MultiBody(MultiBodyBatch<S>* batch) : m_batch(batch), mLinks(1)
{
  DART_ASSERT(m_batch);
}

//==============================================================================
template <typename S>
MultiBody<S>::~MultiBody()
{
  // Do nothing
}

//==============================================================================
template <typename S>
size_t MultiBody<S>::getNumDofs() const
{
  return m_data->dofs_count[m_multibody_index];
}

//==============================================================================
template <typename S>
auto MultiBody<S>::getPositions() const
{
  const auto num_dofs = m_data->dofs_count[m_multibody_index];
  const auto gen_coord_offset = m_data->dofs_offset[m_multibody_index];
  return m_data->positions.segment(gen_coord_offset, num_dofs);
}

} // namespace dart::physics
