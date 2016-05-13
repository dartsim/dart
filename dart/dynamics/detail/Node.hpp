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

#ifndef DART_DYNAMICS_DETAIL_NODE_HPP_
#define DART_DYNAMICS_DETAIL_NODE_HPP_

#include <cassert>

#include "dart/dynamics/Node.hpp"

#include "dart/common/StlHelpers.hpp"

namespace dart {
namespace dynamics {

//==============================================================================
template <class NodeType>
std::size_t AccessoryNode<NodeType>::getIndexInBodyNode() const
{
  return static_cast<const NodeType*>(this)->mIndexInBodyNode;
}

//==============================================================================
template <class NodeType>
std::size_t AccessoryNode<NodeType>::getIndexInSkeleton() const
{
  return static_cast<const NodeType*>(this)->mIndexInSkeleton;
}

//==============================================================================
template <class NodeType>
std::size_t AccessoryNode<NodeType>::getIndexInTree() const
{
  return static_cast<const NodeType*>(this)->mIndexInTree;
}

//==============================================================================
template <class NodeType>
std::size_t AccessoryNode<NodeType>::getTreeIndex() const
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
// Macros for specializing Nodes within BodyNodes
//==============================================================================
#define DART_ENABLE_NODE_SPECIALIZATION()                                                                         \
  public:                                                                                                         \
  template <class T> std::size_t getNumNodes() const { return dart::dynamics::BodyNode::getNumNodes<T>(); }            \
  template <class T> T* getNode(std::size_t index) { return dart::dynamics::BodyNode::getNode<T>(index); }             \
  template <class T> const T* getNode(std::size_t index) const { return dart::dynamics::BodyNode::getNode<T>(index); }

//==============================================================================
#define DETAIL_DART_INSTANTIATE_SPECIALIZED_NODE( NodeName, it )        \
  mNodeMap[typeid( NodeName )] = std::vector<dart::dynamics::Node*>();  \
  it = mNodeMap.find(typeid( NodeName ));

//==============================================================================
#define DART_INSTANTIATE_SPECIALIZED_NODE_IRREGULAR( NodeName, PluralName )         \
  DETAIL_DART_INSTANTIATE_SPECIALIZED_NODE( NodeName, m ## PluralName ## Iterator )

//==============================================================================
#define DART_INSTANTIATE_SPECALIZED_NODE( NodeName )                            \
  DART_INSTANTIATE_SPECIALIZED_NODE_IRREGULAR( NodeName, NodeName ## s )

//==============================================================================
#define DETAIL_DART_SPECIALIZED_NODE_INLINE( NodeName, PluralName, it )                    \
  private: dart::dynamics::BodyNode::NodeMap::iterator it ; public:                         \
                                                                                            \
  inline std::size_t getNum ## PluralName () const                                               \
  { return it->second.size(); }                                                             \
                                                                                            \
  inline NodeName * get ## NodeName (std::size_t index)                                          \
  { return static_cast< NodeName *>(getVectorObjectIfAvailable(index, it->second)); }       \
                                                                                            \
  inline const NodeName * get ## NodeName (std::size_t index) const                              \
  { return static_cast<const NodeName *>(getVectorObjectIfAvailable(index, it->second)); }

//==============================================================================
#define DART_SPECIALIZED_NODE_INLINE_IRREGULAR( NodeName, PluralName )                     \
  DETAIL_DART_SPECIALIZED_NODE_INLINE( NodeName, PluralName, m ## PluralName ## Iterator )

//==============================================================================
#define DART_SPECIALIZED_NODE_INTERNAL( NodeName )                               \
  DART_SPECIALIZED_NODE_INLINE_IRREGULAR( NodeName, NodeName ## s )

//==============================================================================
#define DETAIL_DART_SPECIALIZED_NODE_DECLARE( NodeName, PluralName, it ) \
  private: dart::dynamics::BodyNode::NodeMap::iterator it ; public:\
  std::size_t getNum ## PluralName() const;\
  NodeName * get ## NodeName (std::size_t index);\
  const NodeName * get ## NodeName(std::size_t index) const;

//==============================================================================
#define DART_SPECIALIZED_NODE_DECLARE_IRREGULAR( NodeName, PluralName )\
  DETAIL_DART_SPECIALIZED_NODE_DECLARE( NodeName, PluralName, m ## PluralName ## Iterator )

//==============================================================================
#define DART_SPECIALIZED_NODE_DECLARE( NodeName )\
  DART_SPECIALIZED_NODE_DECLARE_IRREGULAR( NodeName, NodeName ## s )

//==============================================================================
#define DETAIL_DART_SPECIALIZED_NODE_DEFINE( BodyNodeType, NodeName, PluralName, it )\
  std::size_t BodyNodeType :: getNum ## PluralName () const\
  { return it->second.size(); }\
  \
  NodeName * BodyNodeType :: get ## NodeName (std::size_t index)\
  { return static_cast<NodeName *>(getVectorObjectIfAvailable(index, it->second)); }\
  \
  const NodeName * BodyNodeType :: get ## NodeName (std::size_t index) const\
  { return static_cast<const NodeName *>(getVectorObjectIfAvailable(index, it->second)); }

//==============================================================================
#define DART_SPECIALIZED_NODE_DEFINE_IRREGULAR( BodyNodeType, NodeName, PluralName )\
  DETAIL_DART_SPECIALIZED_NODE_DEFINE( BodyNodeType, NodeName, PluralName, m ## PluralName ## Iterator )

//==============================================================================
#define DART_SPECIALIZED_NODE_DEFINE( BodyNodeType, NodeName )\
  DART_SPECIALIZED_NODE_DEFINE_IRREGULAR( BodyNodeType, NodeName, NodeName ## s )

//==============================================================================
#define DETAIL_DART_SPECIALIZED_NODE_TEMPLATE( BodyNodeType, NodeName, PluralName )                                                \
  template <> inline std::size_t BodyNodeType :: getNumNodes< NodeName >() const { return getNum ## PluralName (); }                   \
  template <> inline NodeName * BodyNodeType :: getNode< NodeName >(std::size_t index) { return get ## NodeName (index); }             \
  template <> inline const NodeName * BodyNodeType :: getNode< NodeName >(std::size_t index) const { return get ## NodeName (index); }

//==============================================================================
#define DART_SPECIALIZED_NODE_TEMPLATE_IRREGULAR( BodyNodeType, NodeName, PluralName ) \
  DETAIL_DART_SPECIALIZED_NODE_TEMPLATE( BodyNodeType, NodeName, PluralName )

//==============================================================================
#define DART_SPECIALIZED_NODE_TEMPLATE( BodyNodeType, NodeName )                     \
  DART_SPECIALIZED_NODE_TEMPLATE_IRREGULAR( BodyNodeType, NodeName, NodeName ## s )

//==============================================================================
// Macros for specializing Nodes within Skeletons
//==============================================================================
#define DART_SKEL_ENABLE_NODE_SPECIALIZATION()                                                                                                      \
  public:                                                                                                                                           \
  template <class T> std::size_t getNumNodes() const { return dart::dynamics::Skeleton::getNumNodes<T>(); }                                              \
  template <class T> std::size_t getNumNodes(std::size_t treeIndex) const { return dart::dynamics::Skeleton::getNumNodes<T>(treeIndex); }                     \
  template <class T> T* getNode(std::size_t index) { return dart::dynamics::Skeleton::getNode<T>(index); }                                               \
  template <class T> T* getNode(std::size_t nodeIdx, std::size_t treeIndex) { return dart::dynamics::Skeleton::getNode<T>(nodeIdx, treeIndex); }              \
  template <class T> const T* getNode(std::size_t index) const { return dart::dynamics::Skeleton::getNode<T>(index); }                                   \
  template <class T> const T* getNode(std::size_t nodeIdx, std::size_t treeIndex) const {  return dart::dynamics::Skeleton::getNode<T>(nodeIdx, treeIndex); } \
  template <class T> T* getNode(const std::string& name) { return dart::dynamics::Skeleton::getNode<T>(name); }                                     \
  template <class T> const T* getNode(const std::string& name) const { return dart::dynamics::Skeleton::getNode<T>(name); }

//==============================================================================
#define DART_SKEL_INSTANTIATE_SPECIALIZED_NODE_IRREGULAR( NodeName, PluralName )  \
  mSpecializedTreeNodes[typeid( NodeName )] = &mTree ## PluralName ## Iterators;  \
  mSkelCache.mNodeMap[typeid( NodeName )] = std::vector<dart::dynamics::Node*>(); \
  m ## PluralName ## Iterator = mSkelCache.mNodeMap.find(typeid( NodeName ));     \
  mNodeNameMgrMap[typeid( NodeName )] = dart::common::NameManager<Node*>(         \
    std::string("Skeleton::") + #NodeName + " | " + mAspectProperties.mName );           \
  mNameMgrFor ## PluralName = &mNodeNameMgrMap.find(typeid( NodeName) )->second;

//==============================================================================
#define DART_SKEL_INSTANTIATE_SPECIALIZED_NODE( NodeName )                      \
  DART_SKEL_INSTANTIATE_SPECIALIZED_NODE_IRREGULAR( NodeName, NodeName ## s );

//==============================================================================
#define DETAIL_DART_WARN_TREE_INDEX(treeIts, treeIndex, func) \
  if(treeIndex >= treeIts.size()) \
  { dterr << "[" << #func << "] Requesting an invalid tree (" << treeIndex << "). " \
          << "The number of trees in this Skeleton is: " << treeIts.size() << "\n"; \
  assert(false); return 0; }

//==============================================================================
#define DETAIL_DART_SKEL_SPECIALIZED_NODE_INLINE( NodeName, PluralName, skelIt, treeIts, NameMgr )           \
  private:                                                                                                    \
  dart::dynamics::Skeleton::NodeMap::iterator skelIt;                                                         \
  std::vector<dart::dynamics::Skeleton::NodeMap::iterator> treeIts;                                           \
  dart::common::NameManager<dart::dynamics::Node*>* NameMgr;                                                  \
  public:                                                                                                     \
  inline std::size_t getNum ## PluralName () const                                                                 \
  { return skelIt->second.size(); }                                                                           \
  inline std::size_t getNum ## PluralName (std::size_t treeIndex) const                                                 \
  { DETAIL_DART_WARN_TREE_INDEX(treeIts, treeIndex, getNum ## PluralName);                                    \
  treeIts [treeIndex]->second.size(); }                                                                       \
                                                                                                              \
  inline NodeName * get ## NodeName (std::size_t index)                                                            \
  { return static_cast< NodeName *>(getVectorObjectIfAvailable(index, skelIt ->second)); }                    \
  inline NodeName * get ## NodeName (std::size_t treeIndex, std::size_t nodeIndex)                                      \
  { DETAIL_DART_WARN_TREE_INDEX(treeIts, treeIndex, get ## NodeName);                                         \
  return static_cast< NodeName *>(getVectorObjectIfAvailable(nodeIndex, treeIts[treeIndex]->second)); }       \
                                                                                                              \
  inline const NodeName * get ## NodeName (std::size_t index) const                                                \
  { return static_cast<const NodeName *>(getVectorObjectIfAvailable(index, skelIt ->second)); }               \
  inline const NodeName * get ## NodeName (std::size_t treeIndex, std::size_t nodeIndex) const                          \
  { DETAIL_DART_WARN_TREE_INDEX(treeIts, treeIndex, get ## NodeName);                                         \
  return static_cast<const NodeName *>(getVectorObjectIfAvailable(nodeIndex, treeIts[treeIndex]->second)); }  \
                                                                                                              \
  inline NodeName * get ## NodeName (const std::string& name)                                                 \
  { return static_cast< NodeName *>(NameMgr->getObject(name)); }                                              \
  inline const NodeName * get ## NodeName (const std::string& name) const                                     \
  { return static_cast<const NodeName *>(NameMgr->getObject(name)); }

//==============================================================================
#define DART_SKEL_SPECIALIZED_NODE_INLINE_IRREGULAR( NodeName, PluralName )                                                                                  \
  DETAIL_DART_SKEL_SPECIALIZED_NODE_INLINE( NodeName, PluralName, m ## PluralName ## Iterator, mTree ## PluralName ## Iterators, mNameMgrFor ## PluralName );

//==============================================================================
#define DART_SKEL_SPECIALIZED_NODE_INLINE( NodeName ) \
  DART_SKEL_SPECIALIZED_NODE_INLINE_IRREGULAR( NodeName, NodeName ## s);

//==============================================================================
#define DETAIL_DART_SKEL_SPECIALIZED_NODE_DECLARE( NodeName, PluralName, skelIt, treeIts, NameMgr )           \
  private:\
  dart::dynamics::Skeleton::NodeMap::iterator skelIt;                                                         \
  std::vector<dart::dynamics::Skeleton::NodeMap::iterator> treeIts;                                           \
  dart::common::NameManager<dart::dynamics::Node*>* NameMgr;                                                  \
  public:                                                                                                     \
  std::size_t getNum ## PluralName () const;                                                                 \
  \
  std::size_t getNum ## PluralName (std::size_t treeIndex) const;                                                 \
  \
  NodeName * get ## NodeName (std::size_t index);                                                            \
  \
  NodeName * get ## NodeName (std::size_t treeIndex, std::size_t nodeIndex);                                      \
  \
  const NodeName * get ## NodeName (std::size_t index) const;                                                \
  \
  const NodeName * get ## NodeName (std::size_t treeIndex, std::size_t nodeIndex) const;                          \
  \
  NodeName * get ## NodeName (const std::string& name);                                                 \
  \
  const NodeName * get ## NodeName (const std::string& name) const;

//==============================================================================
#define DART_SKEL_SPECIALIZED_NODE_DECLARE_IRREGULAR( NodeName, PluralName )\
  DETAIL_DART_SKEL_SPECIALIZED_NODE_DECLARE( NodeName, PluralName, m ## PluralName ## Iterator, mTree ## PluralName ## Iterators, mNameMgrFor ## PluralName );

//==============================================================================
#define DART_SKEL_SPECIALIZED_NODE_DECLARE( NodeName )\
  DART_SKEL_SPECIALIZED_NODE_DECLARE_IRREGULAR( NodeName, NodeName ## s );

//==============================================================================
#define DETAIL_DART_SKEL_SPECIALIZED_NODE_DEFINE( SkeletonType, NodeName, PluralName, skelIt, treeIts, NameMgr )\
  std::size_t SkeletonType :: getNum ## PluralName () const                                                                 \
  { return skelIt->second.size(); }                                                                           \
  std::size_t SkeletonType :: getNum ## PluralName (std::size_t treeIndex) const                                                 \
  { DETAIL_DART_WARN_TREE_INDEX(treeIts, treeIndex, getNum ## PluralName);                                    \
  return treeIts [treeIndex]->second.size(); }                                                                       \
                                                                                                              \
  NodeName * SkeletonType :: get ## NodeName (std::size_t index)                                                            \
  { return static_cast< NodeName *>(getVectorObjectIfAvailable(index, skelIt ->second)); }                    \
  NodeName * SkeletonType :: get ## NodeName (std::size_t treeIndex, std::size_t nodeIndex)                                      \
  { DETAIL_DART_WARN_TREE_INDEX(treeIts, treeIndex, get ## NodeName);                                         \
  return static_cast< NodeName *>(getVectorObjectIfAvailable(nodeIndex, treeIts[treeIndex]->second)); }       \
                                                                                                              \
  const NodeName * SkeletonType :: get ## NodeName (std::size_t index) const                                                \
  { return static_cast<const NodeName *>(getVectorObjectIfAvailable(index, skelIt ->second)); }               \
  const NodeName * SkeletonType :: get ## NodeName (std::size_t treeIndex, std::size_t nodeIndex) const                          \
  { DETAIL_DART_WARN_TREE_INDEX(treeIts, treeIndex, get ## NodeName);                                         \
  return static_cast<const NodeName *>(getVectorObjectIfAvailable(nodeIndex, treeIts[treeIndex]->second)); }  \
                                                                                                              \
  NodeName * SkeletonType :: get ## NodeName (const std::string& name)                                                 \
  { return static_cast< NodeName *>(NameMgr->getObject(name)); }                                              \
  const NodeName * SkeletonType :: get ## NodeName (const std::string& name) const                                     \
  { return static_cast<const NodeName *>(NameMgr->getObject(name)); }

//==============================================================================
#define DART_SKEL_SPECIALIZED_NODE_DEFINED_IRREGULAR( SkeletonType, NodeName, PluralName )\
  DETAIL_DART_SKEL_SPECIALIZED_NODE_DEFINE( SkeletonType, NodeName, PluralName, m ## PluralName ## Iterator, mTree ## PluralName ## Iterators, mNameMgrFor ## PluralName );

//==============================================================================
#define DART_SKEL_SPECIALIZED_NODE_DEFINE( SkeletonType, NodeName )\
  DART_SKEL_SPECIALIZED_NODE_DEFINED_IRREGULAR( SkeletonType, NodeName, NodeName ## s );

//==============================================================================
#define DETAIL_DART_SKEL_SPECIALIZED_NODE_TEMPLATE( SkelType, NodeName, PluralName )                                                                               \
  template <> inline std::size_t SkelType :: getNumNodes< NodeName >() const { return getNum ## PluralName (); }                                                       \
  template <> inline std::size_t SkelType :: getNumNodes< NodeName >(std::size_t index) const { return getNum ## PluralName(index); }                                       \
  template <> inline NodeName* SkelType :: getNode< NodeName >(std::size_t index) { return get ## NodeName (index); }                                                  \
  template <> inline NodeName* SkelType :: getNode< NodeName >(std::size_t treeIndex, std::size_t nodeIndex) { return get ## NodeName(treeIndex, nodeIndex); }              \
  template <> inline const NodeName* SkelType :: getNode< NodeName >(std::size_t index) const { return get ## NodeName (index); }                                      \
  template <> inline const NodeName* SkelType :: getNode< NodeName >(std::size_t treeIndex, std::size_t nodeIndex) const { return get ## NodeName(treeIndex, nodeIndex); }  \
  template <> inline NodeName* SkelType::getNode< NodeName >(const std::string& name) { return get ## NodeName (name); }                                          \
  template <> inline const NodeName* SkelType::getNode< NodeName >(const std::string& name) const { return get ## NodeName (name); }

//==============================================================================
#define DART_SKEL_SPECIALIZED_NODE_TEMPLATE_IRREGULAR( SkelType, NodeName, PluralName ) \
  DETAIL_DART_SKEL_SPECIALIZED_NODE_TEMPLATE( SkelType, NodeName, PluralName )

//==============================================================================
#define DART_SKEL_SPECIALIZED_NODE_TEMPLATE( SkelType, NodeName )                      \
  DART_SKEL_SPECIALIZED_NODE_TEMPLATE_IRREGULAR( SkelType, NodeName, NodeName ## s );


#endif // DART_DYNAMICS_DETAIL_NODE_HPP_
