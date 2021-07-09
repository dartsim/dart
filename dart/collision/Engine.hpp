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
#include "dart/collision/Object.hpp"
#include "dart/collision/Types.hpp"
#include "dart/common/Factory.hpp"
#include "dart/math/SmartPointer.hpp"

namespace dart {
namespace collision2 {

template <typename S_>
class Engine
{
public:
  // Type aliases
  using S = S_;

  /// Creates a new collision detector.
  ///
  /// @param[in] engineName: Name of the underlying collision detection engine
  /// to create.
  static EnginePtr<S> create(const std::string& engineName);

  /// Destructor
  virtual ~Engine();

  /// Returns collision detection engine type as a std::string.
  virtual const std::string& getType() const = 0;

  /// Creates a collision group.
  virtual GroupPtr<S> createGroup() = 0;

  template <typename GeometryType, typename... Args>
  ObjectPtr<S> createObject(Args&&... args);

  template <typename... Args>
  ObjectPtr<S> createSphereObject(Args&&... args);

  // TODO(JS): Add options and results as parameters
  virtual bool collide(ObjectPtr<S> object1, ObjectPtr<S> object2) = 0;

  // TODO(JS): Add distance() and raycast()
protected:
  template <typename Derived>
  using Registrar = common::FactoryRegistrar<
      std::string,
      Engine<S>,
      Derived,
      std::shared_ptr<Engine<S>>>;

  //  class ObjectManager;
  //  class ManagerForUnsharableObjects;
  //  class ManagerForSharableObjects;

  /// Constructor
  Engine() = default;

private:
  using Factory = common::Factory<std::string, Engine, std::shared_ptr<Engine>>;

  using SingletonFactory = common::Singleton<Factory>;

  static std::unordered_map<std::string, EnginePtr<S>> mEngines;

  Group<S>* getDefaultGroup();

  GroupPtr<S> mDefaultGroup;
};

extern template class Engine<double>;

} // namespace collision2
} // namespace dart

#include "dart/collision/detail/Engine-impl.hpp"
