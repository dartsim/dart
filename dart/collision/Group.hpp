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

#include "dart/collision/Types.hpp"
#include "dart/math/geometry/Geometry.hpp"

namespace dart {
namespace collision2 {

template <typename S_>
class Group
{
public:
  // Type aliases
  using S = S_;

  /// Destructor
  virtual ~Group() = default;

  /// Return collision detection engine associated with this Group
  Engine<S>* getEngine();

  /// Return (const) collision detection engine associated with this
  /// Group
  const Engine<S>* getEngine() const;

  /// Creates a collision object.
  virtual ObjectPtr<S> createObject(math::GeometryPtr shape) = 0;

  template <typename... Args>
  ObjectPtr<S> createSphereObject(Args&&... args);

protected:
  /// Constructor
  ///
  /// @param[in] collisionDetector: Collision detector that created this group.
  Group(Engine<S>* collisionDetector);

  Engine<S>* mEngine;

private:
  /// Set this to true to have this Group check for updates
  /// automatically. Default is true.
  bool mUpdateAutomatically;
};

using Groupf = Group<float>;
using Groupd = Group<double>;

extern template class Group<double>;

} // namespace collision2
} // namespace dart

#include "dart/collision/detail/Group-impl.hpp"
