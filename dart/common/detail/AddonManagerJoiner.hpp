/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#ifndef DART_COMMON_DETAIL_ADDONMANAGERJOINER_HPP_
#define DART_COMMON_DETAIL_ADDONMANAGERJOINER_HPP_

#include "dart/common/AddonManagerJoiner.hpp"
#include "dart/common/detail/TemplateJoinerDispatchMacro.hpp"

namespace dart {
namespace common {

//==============================================================================
template <class Base1, class Base2>
template <typename Base1Arg, typename... Base2Args>
AddonManagerJoiner<Base1, Base2>::AddonManagerJoiner(
    Base1Arg&& arg1, Base2Args&&... args2)
  : Base1(std::forward<Base1Arg>(arg1)),
    Base2(std::forward<Base2Args>(args2)...)
{
  // Do nothing
}

//==============================================================================
template <class Base1, class Base2>
template <typename Base1Arg>
AddonManagerJoiner<Base1, Base2>::AddonManagerJoiner(
    Base1Arg&& arg1, NoArg_t)
  : Base1(std::forward<Base1Arg>(arg1)),
    Base2()
{
  // Do nothing
}

//==============================================================================
template <class Base1, class Base2>
template <typename... Base2Args>
AddonManagerJoiner<Base1, Base2>::AddonManagerJoiner(
    NoArg_t, Base2Args&&... args2)
  : Base1(),
    Base2(std::forward<Base2Args>(args2)...)
{
  // Do nothing
}

//==============================================================================
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(bool, AddonManagerJoiner, has, () const, ())
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(T*, AddonManagerJoiner, get, (), ())
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(const T*, AddonManagerJoiner, get, () const, ())
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(void, AddonManagerJoiner, set, (const T* addon), (addon))
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(void, AddonManagerJoiner, set, (std::unique_ptr<T>&& addon), (std::move(addon)))
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(void, AddonManagerJoiner, erase, (), ())
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(std::unique_ptr<T>, AddonManagerJoiner, release, (), ())

//==============================================================================
// Because this function requires a comma inside of its template argument list,
// it is not easily fit into a macro like the other functions, so we just
// implement it explicitly.
template <class Base1, class Base2>
template <class T, typename ...Args>
T* AddonManagerJoiner<Base1, Base2>::create(Args&&... args)
{
  if(Base1::template isSpecializedFor<T>())
    return Base1::template create<T, Args...>(std::forward<Args>(args)...);

  return Base2::template create<T, Args...>(std::forward<Args>(args)...);
}

//==============================================================================
template <class Base1, class Base2>
template <class T>
constexpr bool AddonManagerJoiner<Base1, Base2>::isSpecializedFor()
{
  return (Base1::template isSpecializedFor<T>()
          || Base2::template isSpecializedFor<T>());
}

//==============================================================================
template <class Base1, class Base2, class... OtherBases>
template <typename... Args>
AddonManagerJoiner<Base1, Base2, OtherBases...>::AddonManagerJoiner(
    Args&&... args)
  : AddonManagerJoiner<Base1, AddonManagerJoiner<Base2, OtherBases...>>(
      std::forward<Args>(args)...)
{
  // Do nothing
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_ADDONMANAGERJOINER_HPP_

