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

#ifndef DART_COMMON_DETAIL_TEMPLATEJOINERDISPATCHMACRO_HPP_
#define DART_COMMON_DETAIL_TEMPLATEJOINERDISPATCHMACRO_HPP_

//==============================================================================
/// This macro provides the implementation for most of the member functions in
/// common::CompositeJoiner, dynamics::NodeManagerJoinerForBodyNode, and
/// NodeManagerJoinerForSkeleton. The member functions of those classes share
/// essentially the same logic, so it makes sense to have a single macro that
/// provides the implementation for all of them.
#define DETAIL_DART_COMMON_IRREGULAR_TEMPLATEJOINERDISPATCH_IMPL(ReturnType, ClassName, Function, Suffix, SpecializationChecker, Args)\
  template <class Base1, class Base2>\
  template <class T>\
  ReturnType ClassName <Base1, Base2>:: Function Suffix\
  {\
    if(Base1::template SpecializationChecker <T>())\
      return Base1::template Function <T> Args ;\
  \
    return Base2::template Function <T> Args ;\
  }

#define DETAIL_DART_COMMON_TEMPLATEJOINERDISPATCH_IMPL(ReturnType, ClassName, Function, Suffix, Args)\
  DETAIL_DART_COMMON_IRREGULAR_TEMPLATEJOINERDISPATCH_IMPL(ReturnType, ClassName, Function, Suffix, isSpecializedFor, Args)

#endif // DART_COMMON_DETAIL_TEMPLATEJOINERDISPATCHMACRO_HPP_
