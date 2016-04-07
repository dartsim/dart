/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_COMMON_DETAIL_COMPOSITESTATEANDPROPERTIES_H_
#define DART_COMMON_DETAIL_COMPOSITESTATEANDPROPERTIES_H_

#include <map>
#include <unordered_set>
#include <typeinfo>
#include <typeindex>

#include "dart/common/Aspect.h"

namespace dart {
namespace common {
namespace detail {

//==============================================================================
template <class AspectT>
struct GetAspect
{
  using Type = typename static_if_else<
      std::is_base_of<Aspect, AspectT>::value,
      AspectT, typename AspectT::Aspect>::type;
};

//==============================================================================
template <class AspectT>
struct GetState
{
  using Type = typename GetAspect<AspectT>::Type::State;
};

//==============================================================================
template <class AspectT>
struct GetProperties
{
  using Type = typename GetAspect<AspectT>::Type::Properties;
};

//==============================================================================
using CompositeStateMap = std::map< std::type_index, std::unique_ptr<Aspect::State> >;
using CompositePropertiesMap = std::map< std::type_index, std::unique_ptr<Aspect::Properties> >;

//==============================================================================
template <typename MapType, template <class> class GetData>
class CompositeData : public CloneableMap<MapType>
{
public:

  /// Forwarding constructor
  template <typename... Args>
  CompositeData(Args&&... args)
    : CloneableMap<MapType>(std::forward<Args>(args)...)
  {
    // Do nothing
  }

  virtual ~CompositeData() = default;

  /// Create (or replace) a piece of data in this
  template <class AspectT, typename... Args>
  typename GetData<AspectT>::Type& create(Args&&... args)
  {
    using Data = typename GetData<AspectT>::Type;
    using AspectType = typename GetAspect<AspectT>::Type;
    using DataType = typename GetData<Aspect>::Type;

    std::unique_ptr<DataType>& data = this->mMap[typeid(AspectType)]
        = make_unique<Data>(std::forward<Args>(args)...);
    return static_cast<Data&>(*data);
  }

  template <class AspectT>
  typename GetData<AspectT>::Type* get()
  {
    using Data = typename GetData<AspectT>::Type;
    using AspectType = typename GetAspect<AspectT>::Type;

    typename MapType::iterator it = this->mMap.find(typeid(AspectType));
    if(this->mMap.end() == it)
      return nullptr;

    return static_cast<Data*>(it->second.get());
  }

  template <class AspectT>
  const typename GetData<AspectT>::Type* get() const
  {
    return const_cast<CompositeData<MapType, GetData>*>(this)->get<AspectT>();
  }

  template <class AspectT, typename... Args>
  typename GetData<AspectT>::Type& getOrCreate(Args&&... args)
  {
    using Data = typename GetData<AspectT>::Type;
    using AspectType = typename GetAspect<AspectT>::Type;

    auto& it = this->mMap.insert(
          std::make_pair<std::type_index, std::unique_ptr<Data>>(
            typeid(AspectType), nullptr));

    const bool exists = !it.second;
    if(!exists)
      it.first = make_unique<Data>(std::forward<Args>(args)...);

    return static_cast<Data&>(*it.first);
  }

};

//==============================================================================
using CompositeState = CompositeData<CompositeStateMap, GetState>;
using CompositeProperties = CompositeData<CompositePropertiesMap, GetProperties>;

//==============================================================================
template <class CompositeType, template<class> class GetData, typename... Args>
class ComposeData
{
public:
  virtual ~ComposeData() = default;

  void setFrom(const CompositeType&)
  {
    // Do nothing
  }

protected:

  void _addData(CompositeType&) const
  {
    // Do nothing
  }
};

//==============================================================================
template <class CompositeType, template<class> class GetData, class AspectT,
          typename... Remainder>
struct ComposeData<CompositeType, GetData, AspectT, Remainder...> :
    public GetData<AspectT>::Type,
    public ComposeData<CompositeType, GetData, Remainder...>
{
public:

  enum Delegate_t { Delegate };

  using Base = typename GetData<AspectT>::Type;
  using AspectType = typename GetAspect<AspectT>::Type;

  template <typename Arg>
  struct ConvertIfData
  {
    using Type = typename static_if_else<
        std::is_base_of<typename Base::Data, Arg>::value,
        typename Base::Data, Arg>::type;
  };

  ComposeData() = default;

  virtual ~ComposeData() = default;

  template <typename... Args>
  ComposeData(Args&&... args)
    : ComposeData<CompositeType, GetData, Remainder...>(std::forward<Args>(args)...)
  {
    // Do nothing
  }

  operator CompositeType() const
  {
    CompositeType composite;
    _addData(composite);

    return composite;
  }

  void setFrom(const CompositeType& composite)
  {
    const Base* data = composite.template get<AspectType>();
    if(data)
      static_cast<Base&>(*this) = *data;

    ComposeData<CompositeType, GetData, Remainder...>::setFrom(composite);
  }

  ComposeData& operator =(const CompositeType& composite)
  {
    setFrom(composite);
    return *this;
  }

protected:

  void _addData(CompositeType& composite) const
  {
    composite.template create<AspectType>(static_cast<const Base&>(*this));
    ComposeData<CompositeType, GetData, Remainder...>::_addData(composite);
  }

  void _findData()
  {
    // Do nothing
  }

  template <typename Arg1, typename... Args>
  void _findData(Arg1 arg1, Args&&... args)
  {
    _useIfData(static_cast<const typename ConvertIfData<Arg1>::Type&>(arg1));
    _findData(std::forward<Args>(args)...);
  }

  template <typename Arg>
  void _useIfData(Arg)
  {
    // Do nothing
  }

  void _useIfData(const typename Base::Data& data)
  {
    static_cast<Base&>(*this) = data;
  }
};

//==============================================================================
template <typename... Data>
using MakeCompositeState = ComposeData<CompositeState, GetState, Data...>;

template <typename... Data>
using MakeCompositeProperties =
    ComposeData<CompositeProperties, GetProperties, Data...>;

} // namespace detail
} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_COMPOSITESTATEANDPROPERTIES_H_
