/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include <unordered_set>

#include "dart/collision/export.hpp"
#include "dart/collision/type.hpp"
#include "dart/common/macro.hpp"

namespace dart::collision {

template <typename Scalar_>
class CollisionFilter
{
public:
  // Type aliases
  using Scalar = Scalar_;

  /// Destructor.
  virtual ~CollisionFilter();

  /// Returns true if the given two Objects should be checked by the
  /// collision detector, false otherwise.
  virtual bool ignores(
      const Object<Scalar>* object1, const Object<Scalar>* object2) const = 0;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, CollisionFilter)

template <typename Scalar>
class CompositeCollisionFilter : public CollisionFilter<Scalar>
{
public:
  /// Destructor
  ~CompositeCollisionFilter() override = default;

  /// Adds a collision filter to this CompositeCollisionFilter.
  void add_collision_filter(ConstCollisionFilterPtr<Scalar> filter);

  /// Removes a collision filter from this CompositeCollisionFilter.
  void remove_collision_filter(const ConstCollisionFilterPtr<Scalar>& filter);

  /// Removes all the collision filters from this CompositeCollisionFilter.
  void remove_all_collision_filters();

  // Documentation inherited
  bool ignores(const Object<Scalar>* object1, const Object<Scalar>* object2)
      const override;

protected:
  /// Collision filters
  std::unordered_set<ConstCollisionFilterPtr<Scalar>> m_filters;
};

DART_TEMPLATE_CLASS_HEADER(COLLISION, CompositeCollisionFilter)

} // namespace dart::collision

#include "dart/collision/detail/collision_filter_impl.hpp"
