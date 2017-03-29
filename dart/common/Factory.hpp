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

#include <functional>
#include <map>
#include <memory>

#include "dart/common/StlHelpers.hpp"

namespace dart {
namespace common {

/// Implementation of the Abstract Factory Pattern.
///
/// Factory class is a pure static class (i.e., no need to create an instance to
/// use this class).
///
/// Example:
/// \code
/// using CdFactory = Factory<std::string, CollisionDetector>;
///
/// CdFactory::registerCreator<FclCollisionDetector>("fcl");
/// auto fclCd = CdFactory::create("fcl");
/// \endcode
template <typename KeyT,
          typename BaseT,
          template<typename...> class SmartPointerT = std::shared_ptr>
class Factory final
{
public:
  using ReturnType = SmartPointerT<BaseT>;
  using Creator = std::function<ReturnType()>;
  using CreatorMap = std::map<KeyT, Creator>;
  using This = Factory<KeyT, BaseT, SmartPointerT>;
  using RegisterResult = std::pair<typename CreatorMap::iterator, bool>;

  /// Registers a object creator function with a key.
  static RegisterResult registerCreator(const KeyT& key, Creator creator);

  /// Registers the default object creator function with a key.
  template <typename Derived>
  static RegisterResult registerCreator(const KeyT& key);

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
  static ReturnType create(const KeyT& key);
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

/// The default creator. Specialize this template class for smart pointer types
/// other than std::unique_ptr and std::shared_ptr.
template <typename T, template<typename...> class SmartPointerT>
struct DefaultCreator
{
  static SmartPointerT<T> run()
  {
    return SmartPointerT<T>(new T());
  }
};

/// Helper class to register a object creator function.
template <typename KeyT,
          typename BaseT,
          typename DerivedT,
          template<typename...> class SmartPointerT = std::shared_ptr>
class FactoryRegistrar final
{
public:
  using FactoryType = Factory<KeyT, BaseT, SmartPointerT>;
  using Creator = typename FactoryType::Creator;
  using This = FactoryRegistrar<KeyT, BaseT, DerivedT, SmartPointerT>;

  /// Returns the static instance of FactoryRegistrar.
  static This& getInstance(
      const KeyT& key,
      Creator creator = []()
          -> decltype(DefaultCreator<DerivedT, SmartPointerT>::run())
      {
        return DefaultCreator<DerivedT, SmartPointerT>::run();
      });

private:
  /// Constructor. Interanlly, this constructor registers Derived class with
  /// the key and the default creator function.
  FactoryRegistrar(const KeyT& key, Creator creator);

  /// Constructor. Interanlly, this constructor registers Derived class with
  /// the key and the default creator function.
  FactoryRegistrar(const KeyT& key);

  /// Constructor is disabled. This class is a pure static class.
  FactoryRegistrar(const This&) = delete;

  /// Destructor is disabled. This class is a pure static class.
  FactoryRegistrar& operator=(const This&) = delete;
};

#define DART_CONCATENATE(x, y) x##y

#define DART_GEN_UNIQUE_NAME_DETAIL(x, unique_key)                             \
  DART_CONCATENATE(x, unique_key)

#define DART_GEN_UNIQUE_NAME(seed_name)                                        \
  DART_GEN_UNIQUE_NAME_DETAIL(seed_name, __LINE__)

#define DART_REGISTER_CREATOR_TO_FACTORY(key_type, key, base, derived, creator)\
  namespace {                                                                  \
  const auto& DART_GEN_UNIQUE_NAME(unique_name_seed)                           \
      = dart::common::FactoryRegistrar<key_type, base, derived>                \
        ::getInstance(key, creator);                                           \
  }

#define DART_REGISTER_DEFAULT_CREATOR_TO_FACTORY(                              \
    key_type, key, base, derived, smart_ptr_type)                              \
  namespace {                                                                  \
  const auto& DART_GEN_UNIQUE_NAME(unique_name_seed)                           \
      = dart::common::FactoryRegistrar<key_type, base, derived>                \
        ::getInstance(key);                                                    \
  }

#define DART_REGISTER_DEFAULT_UNIQUE_PTR_CREATOR_TO_FACTORY(                   \
    key_type, key, base, derived)                                              \
  DART_REGISTER_DEFAULT_CREATOR_TO_FACTORY(                                    \
      key_type, key, base, derived, std::unique_ptr)

#define DART_REGISTER_DEFAULT_SHARED_PTR_CREATOR_TO_FACTORY(                   \
    key_type, key, base, derived)                                              \
  DART_REGISTER_DEFAULT_CREATOR_TO_FACTORY(                                    \
      key_type, key, base, derived, std::shared_ptr)

} // namespace common
} // namespace dart

#include "dart/common/detail/Factory-impl.hpp"

#endif // DART_COMMON_FACTORY_HPP_
