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

#ifndef DART_DYNAMICS_DETAIL_BASICNODEMANAGER_H_
#define DART_DYNAMICS_DETAIL_BASICNODEMANAGER_H_

#include <map>
#include <typeindex>
#include <unordered_set>

#include "dart/dynamics/Node.h"

namespace dart {
namespace dynamics {
namespace detail {

class BasicNodeManager
{
public:

  using NodeMap = std::map<std::type_index, std::vector<Node*> >;
  using NodeDestructorSet = std::unordered_set<NodeDestructorPtr>;

  /// Get the number of Nodes corresponding to the specified type
  template <class NodeType>
  size_t getNumNodes() const;

  /// Get the Node of the specified type and the specified index
  template <class NodeType>
  NodeType* getNode(size_t index);

  /// Get the Node of the specified type and the specified index
  template <class NodeType>
  const NodeType* getNode(size_t index) const;

protected:

  template <class T> struct type { };

  /// Map that retrieves the Nodes of a specified type
  NodeMap mNodeMap;

  /// A set for storing the Node destructors
  NodeDestructorSet mNodeDestructors;

};

//==============================================================================
template <class NodeType>
size_t BasicNodeManager::getNumNodes() const
{
  NodeMap::const_iterator it = mNodeMap.find(typeid(NodeType));
  if(mNodeMap.end() == it)
    return 0;

  return it->second.size();
}

//==============================================================================
template <class NodeType>
NodeType* BasicNodeManager::getNode(size_t index)
{
  NodeMap::const_iterator it = mNodeMap.find(typeid(NodeType));
  if(mNodeMap.end() == it)
    return nullptr;

  return static_cast<NodeType*>(
        getVectorObjectIfAvailable(index, it->second));
}

//==============================================================================
template <class NodeType>
const NodeType* BasicNodeManager::getNode(size_t index) const
{
  return const_cast<BasicNodeManager*>(this)->getNode<NodeType>(index);
}

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_BASICNODEMANAGER_H_
