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

#ifndef DART_COMMON_DETAIL_COMPOSITEJOINER_H_
#define DART_COMMON_DETAIL_COMPOSITEJOINER_H_

#include "dart/common/CompositeJoiner.h"
#include "dart/common/detail/TemplateJoinerDispatchMacro.h"

namespace dart {
namespace common {

//==============================================================================
template <class Base1, class Base2>
template <typename Base1Arg, typename... Base2Args>
CompositeJoiner<Base1, Base2>::CompositeJoiner(
    Base1Arg&& arg1, Base2Args&&... args2)
  : Base1(std::forward<Base1Arg>(arg1)),
    Base2(std::forward<Base2Args>(args2)...)
{
  // Do nothing
}

//==============================================================================
template <class Base1, class Base2>
template <typename Base1Arg>
CompositeJoiner<Base1, Base2>::CompositeJoiner(
    Base1Arg&& arg1, NoArgTag)
  : Base1(std::forward<Base1Arg>(arg1)),
    Base2()
{
  // Do nothing
}

//==============================================================================
template <class Base1, class Base2>
template <typename... Base2Args>
CompositeJoiner<Base1, Base2>::CompositeJoiner(
    NoArgTag, Base2Args&&... args2)
  : Base1(),
    Base2(std::forward<Base2Args>(args2)...)
{
  // Do nothing
}

//==============================================================================
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(bool, CompositeJoiner, has, () const, ())
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(T*, CompositeJoiner, get, (), ())
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(const T*, CompositeJoiner, get, () const, ())
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(void, CompositeJoiner, set, (const T* aspect), (aspect))
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(void, CompositeJoiner, set, (std::unique_ptr<T>&& aspect), (std::move(aspect)))
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(void, CompositeJoiner, removeAspect, (), ())
DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(std::unique_ptr<T>, CompositeJoiner, releaseAspect, (), ())

//==============================================================================
// Because this function requires a comma inside of its template argument list,
// it is not easily fit into a macro like the other functions, so we just
// implement it explicitly.
template <class Base1, class Base2>
template <class T, typename ...Args>
T* CompositeJoiner<Base1, Base2>::createAspect(Args&&... args)
{
  if(Base1::template isSpecializedFor<T>())
    return Base1::template createAspect<T, Args...>(std::forward<Args>(args)...);

  return Base2::template createAspect<T, Args...>(std::forward<Args>(args)...);
}

//==============================================================================
template <class Base1, class Base2>
template <class T>
constexpr bool CompositeJoiner<Base1, Base2>::isSpecializedFor()
{
  return (Base1::template isSpecializedFor<T>()
          || Base2::template isSpecializedFor<T>());
}

//==============================================================================
template <class Base1, class Base2, class... OtherBases>
template <typename... Args>
CompositeJoiner<Base1, Base2, OtherBases...>::CompositeJoiner(
    Args&&... args)
  : CompositeJoiner<Base1, CompositeJoiner<Base2, OtherBases...>>(
      std::forward<Args>(args)...)
{
  // Do nothing
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_COMPOSITEJOINER_H_

