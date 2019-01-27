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

#ifndef DART_COMMON_DETAIL_COMPOSITEDATA_HPP_
#define DART_COMMON_DETAIL_COMPOSITEDATA_HPP_

#include <Eigen/Core>

#include <map>
#include <unordered_set>
#include <typeinfo>
#include <typeindex>

#include "dart/common/Aspect.hpp"

namespace dart {
namespace common {
namespace detail {

//==============================================================================
// This default template definition will be called when AspectOrComposite is
// an Aspect.
template <class AspectOrComposite, bool isAspect>
struct GetAspectImpl
{
  using Type = AspectOrComposite;
};

//==============================================================================
// This template specialization will be called when AspectOrComposite is not
// an Aspect (and is presumably a composite that defines a nested Aspect type).
template <class AspectOrComposite>
struct GetAspectImpl<AspectOrComposite, false>
{
  // If you get a compiler error that leads you here, then you are trying to
  // ask for the Aspect of an object that is not associated with any Aspect.
  // That means it does not define a nested Aspect type (such as how
  // RevoluteJoint defines the RevoluteJoint::Aspect).
  //
  // Whatever function is leading to the error must be given a template type
  // that either defines a nested type with the name Aspect, or else inherits
  // from the type dart::common::Aspect.
  using Type = typename AspectOrComposite::Aspect;
};

//==============================================================================
template <class AspectT>
struct GetAspect
{
  using Type = typename GetAspectImpl<
    AspectT, std::is_base_of<Aspect, AspectT>::value>::Type;
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

  template <class AspectT>
  bool has() const
  {
    return (get<AspectT>() != nullptr);
  }
};

//==============================================================================
using CompositeState = CompositeData<CompositeStateMap, GetState>;
using CompositeProperties = CompositeData<CompositePropertiesMap, GetProperties>;

//==============================================================================
template <class CompositeType, template<class> class GetData, typename... Aspects>
class ComposeData
{
public:

  ComposeData() = default;

  ComposeData(const CompositeType&)
  {
    // Do nothing
  }

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

  enum DelegateTag { Delegate };

  using ThisClass = ComposeData<CompositeType, GetData, AspectT, Remainder...>;
  using Base = typename GetData<AspectT>::Type;
  using Data = typename Base::Data;
  using AspectType = typename GetAspect<AspectT>::Type;

  template <typename Arg>
  struct ConvertIfData
  {
    using Type = typename std::conditional<
        std::is_base_of<typename Base::Data, Arg>::value,
        typename Base::Data, Arg>::type;
  };

  template <typename Arg>
  struct ConvertIfComposite
  {
    using Type = typename std::conditional<
        std::is_base_of<CompositeType, Arg>::value,
        CompositeType, Arg>::type;
  };

  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(ThisClass)

  ComposeData() = default;

  virtual ~ComposeData() = default;

  template <typename Arg1, typename... Args>
  ComposeData(const Arg1& arg1, const Args&... args)
    : ComposeData(
        Delegate,
        static_cast<const typename ConvertIfData<Arg1>::Type&>(arg1),
        args...)
  {
    // This constructor delegates
  }

  /// Grab relevant data out of a composite object
  ComposeData(const CompositeType& composite)
    : ComposeData<CompositeType, GetData, Remainder...>(composite)
  {
    _setBaseFrom(composite);
  }

  template <typename... Aspects>
  ComposeData(const ComposeData<CompositeType, GetData, Aspects...>& composite)
    : ComposeData(static_cast<const CompositeType&>(composite))
  {
    // This is a delegating constructor. If we get passed another ComposeData
    // object, then we convert it into a composite to ensure that we grab all
    // of its aspects.
  }

  // Dev Note: We must not use the argument 'composite' as a temporary, or else
  // it will get deleted when it reaches the last base constructor, preventing
  // any higher level constructors from calling _setBaseFrom(~) on it.
  ComposeData(CompositeType&& composite)
    : ComposeData(static_cast<const CompositeType&>(composite))
  {
    // This is a delegating constructor
  }

  operator CompositeType() const
  {
    CompositeType composite;
    _addData(composite);

    return composite;
  }

  void setFrom(const CompositeType& composite)
  {
    _setBaseFrom(composite);
    ComposeData<CompositeType, GetData, Remainder...>::setFrom(composite);
  }

  ComposeData& operator =(const CompositeType& composite)
  {
    setFrom(composite);
    return *this;
  }

  /// Grab any relevant data and copy it into this composite. Note that there
  /// will be NO compilation error, even if there is no relevant data in any of
  /// the arguments that get passed in. It will simply ignore all the arguments
  /// silently.
  template <typename... Args>
  void copy(const Args&... args)
  {
    _findData(args...);
  }

protected:

  template <typename... Args>
  ComposeData(DelegateTag, const Args&... args)
    : ComposeData<CompositeType, GetData, Remainder...>(args...)
  {
    // Pass all the arguments along to the next base class
  }

  template <typename... Args>
  ComposeData(DelegateTag, const Data& arg1, const Args&... args)
    : Base(arg1),
      ComposeData<CompositeType, GetData, Remainder...>(args...)
  {
    // Peel off the first argument and then pass along the rest
  }

  void _setBaseFrom(const CompositeType& composite)
  {
    const Base* data = composite.template get<AspectType>();
    if(data)
      static_cast<Base&>(*this) = *data;
  }

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
  void _findData(const Arg1& arg1, const Args&... args)
  {
    _attemptToUse(static_cast<const typename ConvertIfData<Arg1>::Type&>(arg1));
    _findData(args...);
  }

  template <typename Arg>
  void _attemptToUse(const Arg&)
  {
    // Do nothing
  }

  void _attemptToUse(const typename Base::Data& data)
  {
    static_cast<Base&>(*this) = data;
  }

  void _attemptToUse(const CompositeType& composite)
  {
    _setBaseFrom(composite);
  }
};

//==============================================================================
template <typename... Aspects>
using MakeCompositeState = ComposeData<CompositeState, GetState, Aspects...>;

template <typename... Data>
using MakeCompositeProperties =
    ComposeData<CompositeProperties, GetProperties, Data...>;

} // namespace detail
} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_COMPOSITEDATA_HPP_
