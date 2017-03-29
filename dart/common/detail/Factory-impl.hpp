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

#ifndef DART_COMMON_DETAIL_FACTORY_IMPL_HPP_
#define DART_COMMON_DETAIL_FACTORY_IMPL_HPP_

#include "dart/common/Factory.hpp"

#include "dart/common/Memory.hpp"
#include "dart/common/Console.hpp"

namespace dart {
namespace common {

//==============================================================================
template <typename KeyT, typename BaseT, template<typename...> class SmartPointerT>
typename Factory<KeyT, BaseT, SmartPointerT>::RegisterResult
Factory<KeyT, BaseT, SmartPointerT>::registerCreator(
    const KeyT& key, Creator creator)
{
  const auto result = getMap().insert(std::make_pair(key, creator));

  const auto inserted = result.second;
  if (!inserted)
  {
    dtwarn << "[Factory] An object creator fuction for the object of '"
           << typeid(BaseT).name() << "' class with the key (type: '"
           << typeid(KeyT).name() << "') is already registered. "
           << "Register is ignored.\n";
    // TODO(JS): Print the key if the << operator is defined.
  }

  return result;
}

//==============================================================================
template <typename T>
struct DefaultCreator<T, std::unique_ptr>
{
  static std::shared_ptr<T> run()
  {
    return dart::common::make_unique<T>();
  }
};

//==============================================================================
template <typename T>
struct DefaultCreator<T, std::shared_ptr>
{
  static std::shared_ptr<T> run()
  {
    return std::make_shared<T>();
  }
};

//==============================================================================
template <typename KeyT,
          typename BaseT,
          template<typename...> class SmartPointerT>
template <typename Derived>
typename Factory<KeyT, BaseT, SmartPointerT>::RegisterResult
Factory<KeyT, BaseT, SmartPointerT>::registerCreator(const KeyT& key)
{
  return registerCreator(
      key, [](void) -> ReturnType
  {
    return DefaultCreator<Derived, SmartPointerT>::run();
  });
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          template<typename...> class SmartPointerT>
void Factory<KeyT, BaseT, SmartPointerT>::unregisterCreator(const KeyT& key)
{
  getMap().erase(key);
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          template<typename...> class SmartPointerT>
void Factory<KeyT, BaseT, SmartPointerT>::unregisterAllCreators()
{
  getMap().clear();
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          template<typename...> class SmartPointerT>
typename Factory<KeyT, BaseT, SmartPointerT>::ReturnType
Factory<KeyT, BaseT, SmartPointerT>::create(const KeyT& key)
{
  const auto it = getMap().find(key);

  const auto found = (it != getMap().end());
  if (!found)
  {
    dtwarn << "[Factory] Failed to create an object of '"
           << typeid(BaseT).name() << "' class with the key (type: '"
           << typeid(KeyT).name() << "'). Returning nullptr instead.\n";
    // TODO(JS): Print the key if the << operator is defined.

    return nullptr;
  }

  return it->second();
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          template<typename...> class SmartPointerT>
bool Factory<KeyT, BaseT, SmartPointerT>::canCreate(const KeyT& key)
{
  const auto it = getMap().find(key);

  return (it != getMap().end());
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          template<typename...> class SmartPointerT>
typename Factory<KeyT, BaseT, SmartPointerT>::CreatorMap&
Factory<KeyT, BaseT, SmartPointerT>::getMap()
{
  static CreatorMap map;

  return map;
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename DerivedT,
          template<typename...> class SmartPointerT>
FactoryRegistrar<KeyT, BaseT, DerivedT, SmartPointerT>&
FactoryRegistrar<KeyT, BaseT, DerivedT, SmartPointerT>::
getInstance(const KeyT& key, Creator creator)
{
  static FactoryRegistrar<KeyT, BaseT, DerivedT, SmartPointerT> instance(key, creator);

  return instance;
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename DerivedT,
          template<typename...> class SmartPointerT>
FactoryRegistrar<KeyT, BaseT, DerivedT, SmartPointerT>::
FactoryRegistrar(const KeyT& key, Creator creator)
{
  Factory<KeyT, BaseT, SmartPointerT>::registerCreator(key, creator);
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename DerivedT,
          template<typename...> class SmartPointerT>
FactoryRegistrar<KeyT, BaseT, DerivedT, SmartPointerT>::
FactoryRegistrar(const KeyT& key)
{
  Factory<KeyT, BaseT, SmartPointerT>::template registerCreator<DerivedT>(key);
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_FACTORY_IMPL_HPP_
