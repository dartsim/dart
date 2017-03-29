/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_COMMON_FACTORY_HPP_
#define DART_COMMON_FACTORY_HPP_

#include <map>
#include <string>
#include <functional>

#include "dart/common/StlHelpers.hpp"

namespace dart {
namespace common {

/// Implementation of the Factory Pattern.
template <typename KeyT, typename BaseT>
class Factory final
{
public:
  using CreatorMap = std::map<KeyT, std::function<BaseT*()>>;

  /// Registers a object creator function with a key.
  static std::pair<typename Factory<KeyT, BaseT>::CreatorMap::iterator, bool>
  registerCreator(const KeyT& key, const std::function<BaseT*()>& func);

  /// Registers the default object creator function with a key.
  template <typename Derived>
  static std::pair<typename Factory<KeyT, BaseT>::CreatorMap::iterator, bool>
  registerCreator(const KeyT& key);

  /// Unregisters the object creator function that is registered with a key. Do
  /// nothing if there is no creator function associated with the key.
  static void unregisterCreator(const KeyT& key);

  /// Unregisters all the object creator functions.
  static void unregisterAllCreators();

  /// Returns true if an object creator function is registered with the key.
  /// Otherwise, returns false.
  static bool canCreate(const KeyT& key);

  /// Creates an object of the class that is registered with a key. Returns
  /// nullptr if there is no object creator function associated with the key.
  static BaseT* create(const KeyT& key);
  // TODO(JS): Add create() for creating smart_pointers
  // (see: https://github.com/dartsim/dart/pull/845)

protected:
  /// Constructor is disabled. This class is a pure static class.
  Factory() = delete;

  /// Destructor is disabled. This class is a pure static class.
  ~Factory() = delete;

private:
  /// Returns the object creator function map. A static map is created when this
  /// function is firstly called.
  static CreatorMap& getMap();
};

/// Helper class to register a object creator function.
template <typename KeyT, typename Base, typename Derived>
class FactoryRegister final
{
public:
  /// Returns the static instance of FactoryRegister.
  static FactoryRegister<KeyT, Base, Derived>& getInstance(const KeyT& key);

private:
  /// Constructor. Interanlly, this constructor registers Derived class with
  /// the key and the default creator function.
  FactoryRegister(const KeyT& key);

  /// Constructor is disabled. This class is a pure static class.
  FactoryRegister(const FactoryRegister<KeyT, Base, Derived>&) = delete;

  /// Destructor is disabled. This class is a pure static class.
  FactoryRegister& operator=(
      const FactoryRegister<KeyT, Base, Derived>&) = delete;
};

template <typename KeyT, typename Base, typename Derived>
class FactoryScopedRegister final
{
public:
  /// Constructor
  FactoryScopedRegister(const KeyT& key);

  /// Destructor
  ~FactoryScopedRegister();

private:
  /// Constructor is disabled. This class is a pure static class.
  FactoryScopedRegister(
      const FactoryScopedRegister<KeyT, Base, Derived>&) = delete;

  /// Destructor is disabled. This class is a pure static class.
  FactoryScopedRegister& operator=(
      const FactoryScopedRegister<KeyT, Base, Derived>&) = delete;

private:
  /// Index to the map to be used to delete the item when this class is
  /// destructed.
  typename Factory<KeyT, Base>::CreatorMap::iterator mIndex;
};

#define DART_CONCATENATE(x, y) x##y
#define DART_GEN_UNIQUE_NAME_DETAIL(x, unique_key) DART_CONCATENATE(x, unique_key)
#define DART_GEN_UNIQUE_NAME(seed_name)                                        \
  DART_GEN_UNIQUE_NAME_DETAIL(seed_name, __LINE__)

#define DART_REGISTER_OBJECT_TO_FACTORY(key_type, key, base, derived)          \
  namespace {                                                                  \
  const auto& DART_GEN_UNIQUE_NAME(factory_register)                           \
      = ::dart::common::FactoryRegister<key_type, base, derived>               \
        ::getInstance(key);            \
  }

#define DART_REGISTER_OBJECT_TO_FACTORY_IN_FUNCTION_DETAIL(                    \
      unique_name, key_type, key, base, derived)                               \
  const static auto& unique_name                                               \
    = ::dart::common::FactoryRegister<key_type, base, derived>                 \
      ::getInstance(key);                                                      \
  DART_UNUSED(unique_name)

#define DART_REGISTER_OBJECT_TO_FACTORY_IN_FUNCTION(                           \
    key_type, key, base, derived)                                              \
  DART_REGISTER_OBJECT_TO_FACTORY_IN_FUNCTION_DETAIL(                          \
      DART_GEN_UNIQUE_NAME(factory_register), key_type, key, base, derived)

#define DART_UNREGISTER_OBJECT_TO_FACTORY(key_type, key)                       \
  namespace {                                                                  \
  const FactoryUnregister<key_type, base, derived>&                            \
  DART_GEN_UNIQUE_NAME(factory_unregister)                                     \
      = ::dart::common::FactoryUnregister<key_type, base, derived>             \
        ::getInstance(key);          \
  }

} // namespace common
} // namespace dart

#include "dart/common/detail/Factory-impl.hpp"

#endif // DART_COMMON_FACTORY_HPP_
