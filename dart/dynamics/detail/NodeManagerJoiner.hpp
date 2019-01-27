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

#ifndef DART_DYNAMICS_DETAIL_NODEMANAGERJOINER_HPP_
#define DART_DYNAMICS_DETAIL_NODEMANAGERJOINER_HPP_

#include "dart/dynamics/NodeManagerJoiner.hpp"
#include "dart/common/detail/TemplateJoinerDispatchMacro.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
template <class Base1, class Base2>
template <typename Base1Arg, typename... Base2Args>
NodeManagerJoinerForBodyNode<Base1, Base2>::NodeManagerJoinerForBodyNode(
    Base1Arg&& arg1, Base2Args&&... args2)
  : Base1(std::forward<Base1Arg>(arg1)),
    Base2(std::forward<Base2Args>(args2)...)
{
  // Do nothing
}

//==============================================================================
template <class Base1, class Base2>
template <typename Base1Arg>
NodeManagerJoinerForBodyNode<Base1, Base2>::NodeManagerJoinerForBodyNode(
    Base1Arg&& arg1, common::NoArgTag)
  : Base1(std::forward<Base1Arg>(arg1)),
    Base2()
{
  // Do nothing
}

//==============================================================================
template <class Base1, class Base2>
template <typename... Base2Args>
NodeManagerJoinerForBodyNode<Base1, Base2>::NodeManagerJoinerForBodyNode(
    common::NoArgTag, Base2Args&&... args2)
  : Base1(),
    Base2(std::forward<Base2Args>(args2)...)
{
  // Do nothing
}

//==============================================================================
DETAIL_DART_COMMON_IRREGULAR_TEMPLATEJOINERDISPATCH_IMPL(std::size_t, NodeManagerJoinerForBodyNode, getNumNodes, () const, isSpecializedForNode, ())
DETAIL_DART_COMMON_IRREGULAR_TEMPLATEJOINERDISPATCH_IMPL(T*, NodeManagerJoinerForBodyNode, getNode, (std::size_t index), isSpecializedForNode, (index))
DETAIL_DART_COMMON_IRREGULAR_TEMPLATEJOINERDISPATCH_IMPL(const T*, NodeManagerJoinerForBodyNode, getNode, (std::size_t index) const, isSpecializedForNode, (index))

//==============================================================================
template <class Base1, class Base2>
template <class T>
constexpr bool NodeManagerJoinerForBodyNode<Base1, Base2>::isSpecializedForNode()
{
  return (Base1::template isSpecializedForNode<T>()
          || Base2::template isSpecializedForNode<T>());
}

//==============================================================================
template <class Base1, class Base2, class... OtherBases>
template <typename... Args>
NodeManagerJoinerForBodyNode<Base1, Base2, OtherBases...>::NodeManagerJoinerForBodyNode(
    Args&&... args)
  : NodeManagerJoinerForBodyNode<Base1, NodeManagerJoinerForBodyNode<Base2, OtherBases...>>(
      std::forward<Args>(args)...)
{
  // Do nothing
}

//==============================================================================
template <class Base1, class Base2>
template <typename... Args>
NodeManagerJoinerForSkeleton<Base1, Base2>::NodeManagerJoinerForSkeleton(
    Args&&... args)
  : NodeManagerJoinerForBodyNode<Base1, Base2>(std::forward<Args>(args)...)
{
  // Do nothing
}

//==============================================================================
DETAIL_DART_COMMON_IRREGULAR_TEMPLATEJOINERDISPATCH_IMPL(std::size_t, NodeManagerJoinerForSkeleton, getNumNodes, (std::size_t treeIndex) const, isSpecializedForNode, (treeIndex))
DETAIL_DART_COMMON_IRREGULAR_TEMPLATEJOINERDISPATCH_IMPL(T*, NodeManagerJoinerForSkeleton, getNode, (std::size_t treeIndex, std::size_t nodeIndex), isSpecializedForNode, (treeIndex, nodeIndex))
DETAIL_DART_COMMON_IRREGULAR_TEMPLATEJOINERDISPATCH_IMPL(const T*, NodeManagerJoinerForSkeleton, getNode, (std::size_t treeIndex, std::size_t nodeIndex) const, isSpecializedForNode, (treeIndex, nodeIndex))
DETAIL_DART_COMMON_IRREGULAR_TEMPLATEJOINERDISPATCH_IMPL(T*, NodeManagerJoinerForSkeleton, getNode, (const std::string& name), isSpecializedForNode, (name))
DETAIL_DART_COMMON_IRREGULAR_TEMPLATEJOINERDISPATCH_IMPL(const T*, NodeManagerJoinerForSkeleton, getNode, (const std::string& name) const, isSpecializedForNode, (name))

//==============================================================================
template <class Base1, class Base2, class... OtherBases>
template <typename... Args>
NodeManagerJoinerForSkeleton<Base1, Base2, OtherBases...>::NodeManagerJoinerForSkeleton(
    Args&&... args)
  : NodeManagerJoinerForSkeleton<Base1, NodeManagerJoinerForSkeleton<Base2, OtherBases...>>(
      std::forward<Args>(args)...)
{
  // Do nothing
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_NODEMANAGERJOINER_HPP_
