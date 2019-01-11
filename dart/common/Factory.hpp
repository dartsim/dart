/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_COMMON_FACTORY_HPP_
#define DART_COMMON_FACTORY_HPP_

#include <functional>
#include <unordered_map>
#include <memory>
#include <unordered_set>

#include "dart/common/StlHelpers.hpp"
#include "dart/common/Singleton.hpp"

namespace dart {
namespace common {

/// Implementation of the Abstract Factory Pattern.
///
/// Example:
/// \code
/// using CdFactory = Factory<std::string, CollisionDetector>;
///
/// auto factory = CdFactory();
/// factory.registerCreator<FclCollisionDetector>("fcl");
/// auto fclCd = CdFactory::create("fcl");
/// \endcode
template <typename KeyT,
          typename BaseT,
          typename HeldT = std::shared_ptr<BaseT>,
          typename... Args>
class Factory
{
public:
  struct EnumClassHash;

  using This = Factory<KeyT, BaseT, HeldT>;
  using Creator = std::function<HeldT(Args...)>;
  template <typename Key>
  using HashType = typename std::conditional<
      std::is_enum<Key>::value, EnumClassHash, std::hash<Key>>::type;
  using CreatorMap = std::unordered_map<KeyT, Creator, HashType<KeyT>>;

  /// Default constructor.
  Factory() = default;

  /// Destructor
  virtual ~Factory() = default;

  /// Registers a object creator function with a key.
  void registerCreator(const KeyT& key, Creator creator);

  /// Registers the default object creator function with a key.
  template <typename Derived>
  void registerCreator(const KeyT& key);

  /// Unregisters the object creator function that is registered with a key. Do
  /// nothing if there is no creator function associated with the key.
  void unregisterCreator(const KeyT& key);

  /// Unregisters all the object creator functions.
  void unregisterAllCreators();

  /// Returns true if an object creator function is registered with the key.
  /// Otherwise, returns false.
  bool canCreate(const KeyT& key);

  /// Creates an object of the class that is registered with a key. Returns
  /// nullptr if there is no object creator function associated with the key.
  HeldT create(const KeyT& key, Args&&... args);
  // TODO(JS): Add create() for creating smart_pointers
  // (see: https://github.com/dartsim/dart/pull/845)

  /// Get a set of the keys that are available for this Creator
  std::unordered_set<KeyT> getKeys() const;

private:
  template <typename Derived>
  static HeldT defaultCreator(Args&&... args);

  /// Object creator function map.
  CreatorMap mCreatorMap;
};

/// Helper class to register a object creator function to the Singleton.
template <typename KeyT,
          typename BaseT,
          typename DerivedT,
          typename HeldT = std::shared_ptr<BaseT>,
          typename... Args>
class FactoryRegistrar final
{
public:
  using This = FactoryRegistrar<KeyT, BaseT, DerivedT, HeldT>;
  using FactoryType = Factory<KeyT, BaseT, HeldT, Args...>;
  using SingletonFactory = Singleton<FactoryType>;
  using Creator = typename FactoryType::Creator;

  /// Constructor. Interanlly, this constructor registers Derived class with
  /// the key and the default creator function.
  FactoryRegistrar(const KeyT& key, Creator creator);

  /// Constructor. Interanlly, this constructor registers Derived class with
  /// the key and the default creator function.
  explicit FactoryRegistrar(const KeyT& key);
};

} // namespace common
} // namespace dart

#include "dart/common/detail/Factory-impl.hpp"

#endif // DART_COMMON_FACTORY_HPP_
