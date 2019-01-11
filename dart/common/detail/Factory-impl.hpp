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

#ifndef DART_COMMON_DETAIL_FACTORY_IMPL_HPP_
#define DART_COMMON_DETAIL_FACTORY_IMPL_HPP_

#include "dart/common/Factory.hpp"

#include "dart/common/Memory.hpp"
#include "dart/common/Console.hpp"

namespace dart {
namespace common {

namespace detail {

//==============================================================================
/// The default creator. Specialize this template class for smart pointer types
/// other than std::unique_ptr and std::shared_ptr.
template <typename T, typename HeldT, typename... Args>
struct DefaultCreator
{
  static HeldT run(Args&&... args)
  {
    return HeldT(new T(std::forward<Args>(args)...));
  }
};

//==============================================================================
template <typename T, typename... Args>
struct DefaultCreator<T, std::unique_ptr<T>, Args...>
{
  static std::unique_ptr<T> run(Args&&... args)
  {
    return dart::common::make_unique<T>(std::forward<Args>(args)...);
  }
};

//==============================================================================
template <typename T, typename... Args>
struct DefaultCreator<T, std::shared_ptr<T>, Args...>
{
  static std::shared_ptr<T> run(Args&&... args)
  {
    return std::make_shared<T>(std::forward<Args>(args)...);
  }
};

} // namespace detail

//==============================================================================
// std::hash doesn't support enum type. This is an workaround for it.
// Reference: http://stackoverflow.com/a/24847480
template <typename KeyT,
          typename BaseT,
          typename HeldT,
          typename... Args>
struct Factory<KeyT, BaseT, HeldT, Args...>::EnumClassHash
{
  template <typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename HeldT,
          typename... Args>
void Factory<KeyT, BaseT, HeldT, Args...>::registerCreator(
    const KeyT& key, Creator creator)
{
  mCreatorMap[key] = std::move(creator);
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename HeldT,
          typename... Args>
template <typename Derived>
void Factory<KeyT, BaseT, HeldT, Args...>::registerCreator(
    const KeyT& key)
{
  return registerCreator(
      key,
      &Factory::defaultCreator<Derived>
  );
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename HeldT,
          typename... Args>
void Factory<KeyT, BaseT, HeldT, Args...>::unregisterCreator(
    const KeyT& key)
{
  mCreatorMap.erase(key);
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename HeldT,
          typename... Args>
void Factory<KeyT, BaseT, HeldT, Args...>::unregisterAllCreators()
{
  mCreatorMap.clear();
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename HeldT,
          typename... Args>
bool Factory<KeyT, BaseT, HeldT, Args...>::canCreate(const KeyT& key)
{
  const auto it = mCreatorMap.find(key);

  return (it != mCreatorMap.end());
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename HeldT,
          typename... Args>
HeldT Factory<KeyT, BaseT, HeldT, Args...>::create(
    const KeyT& key, Args&&... args)
{
  const auto it = mCreatorMap.find(key);

  const auto found = (it != mCreatorMap.end());
  if (!found)
  {
    dtwarn << "[Factory] Failed to create an object of '"
           << typeid(BaseT).name() << "' class with the key (type: '"
           << typeid(KeyT).name() << "'). Returning nullptr instead.\n";
    // TODO(JS): Print the key if the << operator is defined.

    return nullptr;
  }

  return it->second(std::forward<Args>(args)...);
}

//==============================================================================
template <typename KeyT,
         typename BaseT,
         typename HeldT,
         typename... Args>
std::unordered_set<KeyT> Factory<KeyT, BaseT, HeldT, Args...>::getKeys() const
{
  std::unordered_set<KeyT> keys;
  for(const auto& entry : mCreatorMap)
    keys.insert(entry.first);

  return keys;
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename HeldT,
          typename... Args>
template <typename Derived>
HeldT Factory<KeyT, BaseT, HeldT, Args...>::defaultCreator(Args&&... args)
{
  return detail::DefaultCreator<Derived, HeldT, Args...>::run(
        std::forward<Args>(args)...);
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename DerivedT,
          typename HeldT,
          typename... Args>
FactoryRegistrar<KeyT, BaseT, DerivedT, HeldT, Args...>::
FactoryRegistrar(const KeyT& key, Creator creator)
{
  SingletonFactory::getSingleton().registerCreator(key, creator);
}

//==============================================================================
template <typename KeyT,
          typename BaseT,
          typename DerivedT,
          typename HeldT,
          typename... Args>
FactoryRegistrar<KeyT, BaseT, DerivedT, HeldT, Args...>::
FactoryRegistrar(const KeyT& key)
{
  SingletonFactory::getSingleton().template registerCreator<DerivedT>(key);
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_FACTORY_IMPL_HPP_
