/*
 * Copyright (c) 2011-2025, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#ifndef DART_DYNAMICS_DETAIL_COMPOSITENODE_HPP_
#define DART_DYNAMICS_DETAIL_COMPOSITENODE_HPP_

#include <dart/dynamics/CompositeNode.hpp>

namespace dart {
namespace dynamics {

//==============================================================================
template <class Base>
void CompositeStateNode<Base>::setNodeState(const Node::State& otherState)
{
  common::Composite::setCompositeState(static_cast<const State&>(otherState));
}

//==============================================================================
template <class Base>
std::unique_ptr<Node::State> CompositeStateNode<Base>::getNodeState() const
{
  return std::make_unique<State>(common::Composite::getCompositeState());
}

//==============================================================================
template <class Base>
void CompositeStateNode<Base>::copyNodeStateTo(
    std::unique_ptr<Node::State>& outputState) const
{
  common::Composite::copyCompositeStateTo(static_cast<State&>(*outputState));
}

//==============================================================================
template <class Base>
void CompositePropertiesNode<Base>::setNodeProperties(
    const Node::Properties& otherProperties)
{
  common::Composite::setCompositeProperties(
      static_cast<const Properties&>(otherProperties));
}

//==============================================================================
template <class Base>
std::unique_ptr<Node::Properties>
CompositePropertiesNode<Base>::getNodeProperties() const
{
  return std::make_unique<Properties>(
      common::Composite::getCompositeProperties());
}

//==============================================================================
template <class Base>
void CompositePropertiesNode<Base>::copyNodePropertiesTo(
    std::unique_ptr<Node::Properties>& outputProperties) const
{
  common::Composite::copyCompositePropertiesTo(
      static_cast<Properties&>(*outputProperties));
}

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_COMPOSITENODE_HPP_
