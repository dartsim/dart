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

#include <memory>
#include <string>
#include <unordered_map>

#include "dart/collision/CollisionOption.hpp"
#include "dart/collision/Types.hpp"
#include "dart/common/Factory.hpp"
#include "dart/math/SmartPointer.hpp"

namespace dart {
namespace collision2 {

template <typename S_>
class CollisionDetector
{
public:
  // Type aliases
  using S = S_;

  /// Creates a new collision detector.
  ///
  /// @param[in] engineName: Name of the underlying collision detection engine
  /// to create.
  static CollisionDetectorPtr<S> create(const std::string& engineName);

  /// Destructor
  virtual ~CollisionDetector();

  /// Returns collision detection engine type as a std::string.
  virtual const std::string& getType() const = 0;

  /// Creates a collision group.
  virtual CollisionGroupPtr<S> createCollisionGroup() = 0;

  // TODO(JS): Add distance() and raycast()
protected:
  template <typename Derived>
  using Registrar = common::FactoryRegistrar<
      std::string,
      CollisionDetector<S>,
      Derived,
      std::shared_ptr<CollisionDetector<S>>>;

  //  class CollisionObjectManager;
  //  class ManagerForUnsharableCollisionObjects;
  //  class ManagerForSharableCollisionObjects;

  /// Constructor
  CollisionDetector() = default;

private:
  using Factory = common::Factory<
      std::string,
      CollisionDetector,
      std::shared_ptr<CollisionDetector>>;

  using SingletonFactory = common::Singleton<Factory>;

  static std::unordered_map<std::string, CollisionDetectorPtr<S>>
      mCollisionDetectors;
};

extern template class CollisionDetector<double>;

} // namespace collision2
} // namespace dart

#include "dart/collision/detail/CollisionDetector-impl.hpp"
