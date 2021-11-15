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

#include "dart/collision/dart/broad_phase/broad_phase_algorithm.hpp"

namespace dart::collision::detail {

namespace {

template <typename Scalar>
class Callback : public BroadPhaseCallback<Scalar>
{
public:
  Callback(detail::BroadPhaseOverlappingPairs<Scalar>& pairs) : m_pairs(pairs)
  {
    // Do nothing
  }

  bool add_pair(
      DartObject<Scalar>* object_a, DartObject<Scalar>* object_b) override
  {
    m_pairs.add(object_a, object_b);
    return false;
  }

  bool remove_pair(
      DartObject<Scalar>* object_a, DartObject<Scalar>* object_b) override
  {
    m_pairs.remove(object_a, object_b);
    return false;
  }

private:
  detail::BroadPhaseOverlappingPairs<Scalar>& m_pairs;
};

} // namespace

//==============================================================================
template <typename Scalar>
BroadPhaseAlgorithm<Scalar>::BroadPhaseAlgorithm()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
BroadPhaseAlgorithm<Scalar>::~BroadPhaseAlgorithm()
{
  // Do nothing
}

//==============================================================================
template <typename Scalar>
void BroadPhaseAlgorithm<Scalar>::update_overlapping_pairs(
    Scalar time_step, detail::BroadPhaseOverlappingPairs<Scalar>& pairs)
{
  compute_overlapping_pairs(time_step, Callback(pairs));
}

} // namespace dart::collision::detail
