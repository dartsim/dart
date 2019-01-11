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

#ifndef DART_DYNAMICS_NODEMANAGERJOINER_HPP_
#define DART_DYNAMICS_NODEMANAGERJOINER_HPP_

#include <string>

#include "dart/common/Empty.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
/// Declaration of the variadic template
template <class... OtherBases>
class NodeManagerJoinerForBodyNode { };

//==============================================================================
/// Special case of only having 1 class: we do nothing but inherit it.
template <class Base1>
class NodeManagerJoinerForBodyNode<Base1> : public Base1 { };

//==============================================================================
/// NodeManagerJoiner allows classes that inherit from various
/// SpecializedNodeManager types to be inherited by a single derived class. This
/// class solved the diamond-of-death problem for multiple
/// SpecializedNodeManager inheritance.
template <class Base1, class Base2>
class NodeManagerJoinerForBodyNode<Base1, Base2> : public Base1, public Base2
{
public:

  /// Default constructor
  NodeManagerJoinerForBodyNode() = default;

  /// This constructor allows one argument to be passed to the Base1 constructor
  /// and arbitrarily many arguments to be passed to the Base2 constructor.
  template <typename Base1Arg, typename... Base2Args>
  NodeManagerJoinerForBodyNode(Base1Arg&& arg1, Base2Args&&... args2);

  /// This constructor passes one argument to the Base1 constructor and no
  /// arguments to the Base2 constructor.
  template <typename Base1Arg>
  NodeManagerJoinerForBodyNode(Base1Arg&& arg1, common::NoArgTag);

  /// This constructor passes no arguments to the Base1 constructor and
  /// arbitrarily many arguments to the Base2 constructor.
  template <typename... Base2Args>
  NodeManagerJoinerForBodyNode(common::NoArgTag, Base2Args&&... args2);

  template <class NodeType>
  std::size_t getNumNodes() const;

  template <class NodeType>
  NodeType* getNode(std::size_t index);

  template <class NodeType>
  const NodeType* getNode(std::size_t index) const;

  template <class NodeType>
  static constexpr bool isSpecializedForNode();

};

//==============================================================================
/// This is the variadic version of the NodeManagerJoinerForBodyNode class which
/// allows you to include arbitrarily many base classes in the joining.
template <class Base1, class Base2, class... OtherBases>
class NodeManagerJoinerForBodyNode<Base1, Base2, OtherBases...> :
    public NodeManagerJoinerForBodyNode< Base1, NodeManagerJoinerForBodyNode<Base2, OtherBases...> >
{
public:

  NodeManagerJoinerForBodyNode() = default;

  template <typename... Args>
  NodeManagerJoinerForBodyNode(Args&&... args);

};

//==============================================================================
/// Declaration of variadic template
template <class... OtherBases>
class NodeManagerJoinerForSkeleton { };

//==============================================================================
/// Special case of only having 1 class: we do nothing but inherit it
template <class Base1>
class NodeManagerJoinerForSkeleton<Base1> : public Base1 { };

//==============================================================================
template <class Base1, class Base2>
class NodeManagerJoinerForSkeleton<Base1, Base2> :
    public NodeManagerJoinerForBodyNode<Base1, Base2>
{
public:

  using Base = NodeManagerJoinerForBodyNode<Base1, Base2>;
  using Base::getNumNodes;
  using Base::getNode;
  using Base::isSpecializedForNode;

  /// Default constructor
  NodeManagerJoinerForSkeleton() = default;

  /// Forwards construction to NodeManagerJoinerForBodyNode
  template <typename... Args>
  NodeManagerJoinerForSkeleton(Args&&... args);

  /// Get the number of Nodes of the specified type that are in the treeIndexth
  /// tree of this Skeleton
  template <class NodeType>
  std::size_t getNumNodes(std::size_t treeIndex) const;

  /// Get the nodeIndexth Node of the specified type within the tree of
  /// treeIndex.
  template <class NodeType>
  NodeType* getNode(std::size_t treeIndex, std::size_t nodeIndex);

  /// Get the nodeIndexth Node of the specified type within the tree of
  /// treeIndex.
  template <class NodeType>
  const NodeType* getNode(std::size_t treeIndex, std::size_t nodeIndex) const;

  /// Get the Node of the specified type with the given name.
  template <class NodeType>
  NodeType* getNode(const std::string& name);

  /// Get the Node of the specified type with the given name.
  template <class NodeType>
  const NodeType* getNode(const std::string& name) const;

};

//==============================================================================
/// This is the variadic version of the NodeManagerJoinerForSkeleton class which
/// allows you to include arbitrarily many base classes in the joining.
template <class Base1, class Base2, class... OtherBases>
class NodeManagerJoinerForSkeleton<Base1, Base2, OtherBases...> :
    public NodeManagerJoinerForSkeleton< Base1, NodeManagerJoinerForSkeleton<Base2, OtherBases...> >
{
public:

  NodeManagerJoinerForSkeleton() = default;

  template <typename... Args>
  NodeManagerJoinerForSkeleton(Args&&... args);

};

} // namespace dynamics
} // namespace dart

#include "dart/dynamics/detail/NodeManagerJoiner.hpp"

#endif // DART_DYNAMICS_NODEMANAGERJOINER_HPP_
