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

#ifndef DART_DYNAMICS_DETAIL_NODE_H_
#define DART_DYNAMICS_DETAIL_NODE_H_

#include <cassert>

#include "dart/dynamics/Node.h"

namespace dart {
namespace dynamics {

//==============================================================================
template <typename T>
static T getVectorObjectIfAvailable(size_t _index, const std::vector<T>& _vec)
{
  assert(_index < _vec.size());
  if(_index < _vec.size())
    return _vec[_index];

  return nullptr;
}

//==============================================================================
template <class NodeType>
size_t AccessoryNode<NodeType>::getIndexInBodyNode() const
{
  return static_cast<const NodeType*>(this)->mIndexInBodyNode;
}

//==============================================================================
template <class NodeType>
size_t AccessoryNode<NodeType>::getIndexInSkeleton() const
{
  return static_cast<const NodeType*>(this)->mIndexInSkeleton;
}

//==============================================================================
template <class NodeType>
size_t AccessoryNode<NodeType>::getIndexInTree() const
{
  return static_cast<const NodeType*>(this)->mIndexInTree;
}

//==============================================================================
template <class NodeType>
size_t AccessoryNode<NodeType>::getTreeIndex() const
{
  return static_cast<const NodeType*>(this)->getBodyNodePtr()->getTreeIndex();
}

//==============================================================================
template <class NodeType>
void AccessoryNode<NodeType>::remove()
{
  return static_cast<NodeType*>(this)->stageForRemoval();
}

//==============================================================================
template <class NodeType>
void AccessoryNode<NodeType>::reattach()
{
  static_cast<NodeType*>(this)->attach();
}

} // namespace dynamics
} // namespace dart

//==============================================================================
#define DART_ENABLE_NODE_SPECIALIZATION()                                                         \
  public:                                                                                         \
  template <class T> size_t getNumNodes() const { return BodyNode::getNumNodes<T>(); }            \
  template <class T> T* getNode(size_t index) { return BodyNode::getNode<T>(index); }             \
  template <class T> const T* getNode(size_t index) const { return BodyNode::getNode<T>(index); }

//==============================================================================
#define DETAIL_DART_INSTANTIATE_SPECIALIZED_NODE( NodeName, it )        \
  mNodeMap[typeid( NodeName )] = std::vector<dart::dynamics::Node*>();  \
  it = mNodeMap.find(typeid( NodeName ));

//==============================================================================
#define DART_INSTANTIATE_SPECIALIZED_NODE_IRREGULAR_PLURAL( NodeName, PluralNodeName )  \
  DETAIL_DART_INSTANTIATE_SPECIALIZED_NODE( NodeName, m ## PluralNodeName ## Iterator )

//==============================================================================
#define DART_INSTANTIATE_SPECALIZED_NODE( NodeName )                            \
  DART_INSTANTIATE_SPECIALIZED_NODE_IRREGULAR_PLURAL( NodeName, NodeName ## s )

//==============================================================================
#define DETAIL_DART_SPECIALIZE_NODE_INTERNAL( NodeName, PluralName, it )                    \
  private: NodeMap::iterator it ; public:                                                   \
                                                                                            \
  inline size_t getNum ## PluralName () const                                               \
  { return it->second.size(); }                                                             \
                                                                                            \
  inline NodeName * get ## NodeName (size_t index)                                          \
  { return static_cast< NodeName *>(getVectorObjectIfAvailable(index, it->second)); }       \
                                                                                            \
  inline const NodeName * get ## NodeName (size_t index) const                              \
  { return static_cast<const NodeName *>(getVectorObjectIfAvailable(index, it->second)); }

//==============================================================================
#define DART_SPECIALIZE_NODE_INTERNAL_IRREGULAR_PLURAL( NodeName, PluralNodeName )                  \
  DETAIL_DART_SPECIALIZE_NODE_INTERNAL( NodeName, PluralNodeName, m ## PluralNodeName ## Iterator )

//==============================================================================
#define DART_SPECIALIZE_NODE_INTERNAL( NodeName )                           \
  DART_SPECIALIZE_NODE_INTERNAL_IRREGULAR_PLURAL( NodeName, NodeName ## s )

//==============================================================================
#define DETAIL_DART_SPECIALIZE_NODE_EXTERNAL( BodyNodeType, NodeName, PluralName )                                                \
  template <> inline size_t BodyNodeType :: getNumNodes< NodeName >() const { return getNum ## PluralName (); }                   \
  template <> inline NodeName * BodyNodeType :: getNode< NodeName >(size_t index) { return get ## NodeName (index); }             \
  template <> inline const NodeName * BodyNodeType :: getNode< NodeName >(size_t index) const { return get ## NodeName (index); }

//==============================================================================
#define DART_SPECIALIZE_NODE_EXTERNAL_IRREGULAR_PLURAL( BodyNodeType, NodeName, PluralNodeName ) \
  DETAIL_DART_SPECIALIZE_NODE_EXTERNAL( BodyNodeType, NodeName, PluralNodeName )

//==============================================================================
#define DART_SPECIALIZE_NODE_EXTERNAL( BodyNodeType, NodeName )                           \
  DART_SPECIALIZE_NODE_EXTERNAL_IRREGULAR_PLURAL( BodyNodeType, NodeName, NodeName ## s )


#endif // DART_DYNAMICS_DETAIL_NODE_H_
