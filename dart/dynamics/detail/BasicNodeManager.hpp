/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#ifndef DART_DYNAMICS_DETAIL_BASICNODEMANAGER_HPP_
#define DART_DYNAMICS_DETAIL_BASICNODEMANAGER_HPP_

#include <map>
#include <typeindex>
#include <unordered_set>

#include "dart/common/NameManager.hpp"
#include "dart/common/Empty.hpp"
#include "dart/dynamics/Node.hpp"

namespace dart {
namespace dynamics {
namespace detail {

class BasicNodeManagerForBodyNode
{
public:

  using NodeMap = std::map< std::type_index, std::vector<Node*> >;
  using NodeDestructorSet = std::unordered_set<NodeDestructorPtr>;
  using NodeNameMgrMap = std::map< std::type_index, common::NameManager<Node*> >;
  using SpecializedTreeNodes = std::map<std::type_index, std::vector<NodeMap::iterator>*>;

  /// Default constructor
  BasicNodeManagerForBodyNode() = default;

  /// Delete copy constructors and assignment operators
  BasicNodeManagerForBodyNode(const BasicNodeManagerForBodyNode&) = delete;
  BasicNodeManagerForBodyNode& operator=(const BasicNodeManagerForBodyNode&) = delete;

  /// Get the number of Nodes corresponding to the specified type
  template <class NodeType>
  std::size_t getNumNodes() const;

  /// Get the Node of the specified type and the specified index
  template <class NodeType>
  NodeType* getNode(std::size_t index);

  /// Get the Node of the specified type and the specified index
  template <class NodeType>
  const NodeType* getNode(std::size_t index) const;

  /// Check if this Manager is specialized for a specific type of Node
  template <class NodeType>
  static constexpr bool isSpecializedForNode();

protected:

  template <class T> struct type { };

  /// Map that retrieves the Nodes of a specified type
  NodeMap mNodeMap;

  /// A set for storing the Node destructors
  NodeDestructorSet mNodeDestructors;

};

class BasicNodeManagerForSkeleton : public virtual BasicNodeManagerForBodyNode
{
public:

  using BasicNodeManagerForBodyNode::getNumNodes;
  using BasicNodeManagerForBodyNode::getNode;

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

protected:

  /// NameManager for tracking Nodes
  NodeNameMgrMap mNodeNameMgrMap;

  /// A NodeMap for each tree to allow tree Nodes to be accessed independently
  std::vector<NodeMap> mTreeNodeMaps;

  /// A map that allows SpecializedNodeManagers to have a direct iterator to
  /// the tree-wise storage of its specialized Node. Each entry in this map
  /// contains a pointer to a vector of iterators. Each vector of iterators is
  /// stored in its corresponding SpecializedNodeManager. This system allows
  /// Node specialization to be extensible, enabling custom derived Skeleton
  /// types that are specialized for more than the default specialized Nodes.
  SpecializedTreeNodes mSpecializedTreeNodes;

};

//==============================================================================
template <class NodeType>
std::size_t BasicNodeManagerForBodyNode::getNumNodes() const
{
  NodeMap::const_iterator it = mNodeMap.find(typeid(NodeType));
  if(mNodeMap.end() == it)
    return 0;

  return it->second.size();
}

//==============================================================================
template <class NodeType>
NodeType* BasicNodeManagerForBodyNode::getNode(std::size_t index)
{
  NodeMap::const_iterator it = mNodeMap.find(typeid(NodeType));
  if(mNodeMap.end() == it)
    return nullptr;

  return static_cast<NodeType*>(
        getVectorObjectIfAvailable(index, it->second));
}

//==============================================================================
template <class NodeType>
const NodeType* BasicNodeManagerForBodyNode::getNode(std::size_t index) const
{
  return const_cast<BasicNodeManagerForBodyNode*>(this)->getNode<NodeType>(index);
}

//==============================================================================
template <class NodeType>
constexpr bool BasicNodeManagerForBodyNode::isSpecializedForNode()
{
  // When invoked through a BasicNodeManager, this should always return false.
  return false;
}

//==============================================================================
template <class NodeType>
std::size_t BasicNodeManagerForSkeleton::getNumNodes(std::size_t treeIndex) const
{
  if(treeIndex >= mTreeNodeMaps.size())
  {
    dterr << "[Skeleton::getNumNodes<" << typeid(NodeType).name() << ">] "
          << "Requested tree index (" << treeIndex << "), but there are only ("
          << mTreeNodeMaps.size() << ") trees available\n";
    assert(false);
    return 0;
  }

  const NodeMap& nodeMap = mTreeNodeMaps[treeIndex];
  NodeMap::const_iterator it = nodeMap.find(typeid(NodeType));
  if(nodeMap.end() == it)
    return 0;

  return it->second.size();
}

//==============================================================================
template <class NodeType>
NodeType* BasicNodeManagerForSkeleton::getNode(
    std::size_t treeIndex, std::size_t nodeIndex)
{
  if(treeIndex >= mTreeNodeMaps.size())
  {
    dterr << "[Skeleton::getNode<" << typeid(NodeType).name() << ">] "
          << "Requested tree index (" << treeIndex << "), but there are only ("
          << mTreeNodeMaps.size() << ") trees available\n";
    assert(false);
    return nullptr;
  }

  const NodeMap& nodeMap = mTreeNodeMaps[treeIndex];
  NodeMap::const_iterator it = nodeMap.find(typeid(NodeType));
  if(nodeMap.end() == it)
  {
    dterr << "[Skeleton::getNode<" << typeid(NodeType).name() << ">] "
          << "Requested index (" << nodeIndex << ") within tree (" << treeIndex
          << "), but there are no Nodes of the requested type in this tree\n";
    assert(false);
    return nullptr;
  }

  if(nodeIndex >= it->second.size())
  {
    dterr << "[Skeleton::getNode<" << typeid(NodeType).name() << ">] "
          << "Requested index (" << nodeIndex << ") within tree (" << treeIndex
          << "), but there are only (" << it->second.size() << ") Nodes of the "
          << "requested type within that tree\n";
    assert(false);
    return nullptr;
  }

  return static_cast<NodeType*>(it->second[nodeIndex]);
}

//==============================================================================
template <class NodeType>
const NodeType* BasicNodeManagerForSkeleton::getNode(
    std::size_t treeIndex, std::size_t nodeIndex) const
{
  return const_cast<BasicNodeManagerForSkeleton*>(this)->getNode<NodeType>(
        treeIndex, nodeIndex);
}

//==============================================================================
template <class NodeType>
NodeType* BasicNodeManagerForSkeleton::getNode(const std::string& name)
{
  NodeNameMgrMap::const_iterator it = mNodeNameMgrMap.find(typeid(NodeType));

  if(mNodeNameMgrMap.end() == it)
    return nullptr;

  return static_cast<NodeType*>(it->second.getObject(name));
}

//==============================================================================
template <class NodeType>
const NodeType* BasicNodeManagerForSkeleton::getNode(
    const std::string& name) const
{
  return const_cast<BasicNodeManagerForSkeleton*>(
        this)->getNode<NodeType>(name);
}

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_IRREGULAR( TypeName, AspectName, PluralAspectName )\
  inline std::size_t getNum ## PluralAspectName () const\
  { return getNumNodes< TypeName >(); }\
  inline TypeName * get ## AspectName (std::size_t index)\
  { return getNode< TypeName >(index); }\
  inline const TypeName * get ## AspectName (std::size_t index) const\
  { return getNode< TypeName >(index); }

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE( AspectName )\
  DART_BAKE_SPECIALIZED_NODE_IRREGULAR( AspectName, AspectName, AspectName ## s )

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_SKEL_IRREGULAR( TypeName, AspectName, PluralAspectName )\
  DART_BAKE_SPECIALIZED_NODE_IRREGULAR( TypeName, AspectName, PluralAspectName )\
  inline std::size_t getNum ## PluralAspectName (std::size_t treeIndex) const\
  { return getNumNodes< TypeName >(treeIndex); }\
  inline TypeName * get ## AspectName (std::size_t treeIndex, std::size_t nodeIndex)\
  { return getNode< TypeName >(treeIndex, nodeIndex); }\
  inline const TypeName * get ## AspectName (std::size_t treeIndex, std::size_t nodeIndex) const\
  { return getNode< TypeName >(treeIndex, nodeIndex); }\
  \
  inline TypeName * get ## AspectName (const std::string& name)\
  { return getNode< TypeName >(name); }\
  inline const TypeName * get ## AspectName (const std::string& name) const\
  { return getNode< TypeName >(name); }

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_SKEL( AspectName )\
  DART_BAKE_SPECIALIZED_NODE_SKEL_IRREGULAR( AspectName, AspectName, AspectName ## s)

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_IRREGULAR_DECLARATIONS( TypeName, AspectName, PluralAspectName )\
  std::size_t getNum ## PluralAspectName () const;\
  TypeName * get ## AspectName (std::size_t index);\
  const TypeName * get ## AspectName (std::size_t index) const;

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_DECLARATIONS( AspectName )\
  DART_BAKE_SPECIALIZED_NODE_IRREGULAR_DECLARATIONS( AspectName, AspectName, AspectName ## s )

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_SKEL_IRREGULAR_DECLARATIONS( TypeName, AspectName, PluralAspectName )\
  DART_BAKE_SPECIALIZED_NODE_IRREGULAR_DECLARATIONS( TypeName, AspectName, PluralAspectName )\
  std::size_t getNum ## PluralAspectName (std::size_t treeIndex) const;\
  TypeName * get ## AspectName (std::size_t treeIndex, std::size_t nodeIndex);\
  const TypeName * get ## AspectName (std::size_t treeIndex, std::size_t nodeIndex) const;\
  \
  TypeName * get ## AspectName (const std::string& name);\
  const TypeName * get ## AspectName (const std::string& name) const;

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_SKEL_DECLARATIONS( AspectName )\
  DART_BAKE_SPECIALIZED_NODE_SKEL_IRREGULAR_DECLARATIONS( AspectName, AspectName, AspectName ## s )

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_IRREGULAR_DEFINITIONS( ClassName, TypeName, AspectName, PluralAspectName )\
  std::size_t ClassName :: getNum ## PluralAspectName () const\
  { return getNumNodes< TypeName >(); }\
  TypeName * ClassName :: get ## AspectName (std::size_t index)\
  { return getNode< TypeName >(index); }\
  const TypeName * ClassName :: get ## AspectName (std::size_t index) const\
  { return getNode< TypeName >(index); }

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_DEFINITIONS( ClassName, AspectName )\
  DART_BAKE_SPECIALIZED_NODE_IRREGULAR_DEFINITIONS( ClassName, AspectName, AspectName, AspectName ## s )

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_SKEL_IRREGULAR_DEFINITIONS( ClassName, TypeName, AspectName, PluralAspectName )\
  DART_BAKE_SPECIALIZED_NODE_IRREGULAR_DEFINITIONS( ClassName, TypeName, AspectName, PluralAspectName )\
  std::size_t ClassName :: getNum ## PluralAspectName (std::size_t treeIndex) const\
  { return getNumNodes< TypeName >(treeIndex); }\
  TypeName * ClassName :: get ## AspectName (std::size_t treeIndex, std::size_t nodeIndex)\
  { return getNode< TypeName >(treeIndex, nodeIndex); }\
  const TypeName * ClassName :: get ## AspectName (std::size_t treeIndex, std::size_t nodeIndex) const\
  { return getNode< TypeName >(treeIndex, nodeIndex); }\
  \
  TypeName * ClassName :: get ## AspectName (const std::string& name)\
  { return getNode< TypeName >(name); }\
  const TypeName * ClassName :: get ## AspectName (const std::string& name) const\
  { return getNode< TypeName >(name); }

//==============================================================================
#define DART_BAKE_SPECIALIZED_NODE_SKEL_DEFINITIONS( ClassName, AspectName )\
  DART_BAKE_SPECIALIZED_NODE_SKEL_IRREGULAR_DEFINITIONS( ClassName, AspectName, AspectName, AspectName ## s )

} // namespace detail
} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_DETAIL_BASICNODEMANAGER_HPP_
