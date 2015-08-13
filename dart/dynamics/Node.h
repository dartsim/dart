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

#ifndef DART_DYNAMICS_NODE_H_
#define DART_DYNAMICS_NODE_H_

#include <memory>

#include "dart/common/Subject.h"
#include "dart/common/Extensible.h"
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace dynamics {

class BodyNode;
class Node;

class NodeCleaner final
{
public:

  /// Constructor
  NodeCleaner(Node* _node);

  /// Do not copy
  NodeCleaner(const NodeCleaner& _cleaner) = delete;

  /// Non-virtual destructor (this class cannot be inherited)
  ~NodeCleaner();

  Node* getNode() const;

private:

  /// Node that this Cleaner is responsible for
  Node* mNode;

};

/// The Node class is a base class for any object that attaches to a BodyNode.
/// This base class handles ownership and reference counting for the classes
/// that inherit it.
///
/// In most cases, when creating your own custom Node class, you will also want
/// to inherit from AccessoryNode using CRTP.
class Node : public virtual common::Subject
{
public:

  friend class BodyNode;
  friend class Skeleton;
  template<class> friend class AccessoryNode;
  template<class, class> friend class TemplateNodePtr;
  template<class, class> friend class TemplateWeakNodePtr;

  /// If your Node has a State class, then that State class should inherit this
  /// Node::State class. This allows us to safely serialize, store, and clone
  /// the states of arbitrary Node extensions. If your Node is stateless, then
  /// you do not have to worry about extending this class, because
  /// Node::getNodeState() will simply return a nullptr by default, which is
  /// taken to indicate that it is stateless.
  ///
  /// The distinction between the State class and the Properties class is that
  /// State will get stored in BodyNode::ExtendedState whereas Properties will
  /// get stored in BodyNode::ExtendedProperties. Typically Properties are
  /// values that only change rarely if ever, whereas State contains values that
  /// might change as often as every time step.
  ///
  /// If your Node has a State, be sure to call setNodeStatePtr() during the
  /// construction of your Node.
  class State : public common::Extensible<State> { };

  /// If your Node has a Properties class, then that Properties class should
  /// inherit this Node::Properties class. This allows us to safely serialize,
  /// store, and clone the properties of arbitrary Node extensions. If your
  /// Node has no properties, then you do not have to worry about extending
  /// this class, because Node::getNodeProperties() will simply return a nullptr
  /// by default, which is taken to indicate that it has no properties.
  ///
  /// The distinction between the State class and the Properties class is that
  /// State will get stored in BodyNode::ExtendedState whereas Properties will
  /// get stored in BodyNode::ExtendedProperties. Typically Properties are
  /// values that only change rarely if ever, whereas State contains values that
  /// might change as often as every time step.
  ///
  /// If your Node has Properties, then be sure to call setNodePropertiesPtr()
  /// during the construction of your Node.
  class Properties : public common::Extensible<Properties> { };

  /// Virtual destructor
  virtual ~Node() = default;

  /// Set the name of this Node
  virtual const std::string& setName(const std::string& newName) = 0;

  /// Get the name of this Node
  virtual const std::string& getName() const = 0;

  /// Set the State of this Node. By default, this does nothing.
  virtual void setNodeState(const std::unique_ptr<State>& otherState);

  /// Get the State of this Node. By default, this returns a nullptr which
  /// implies that the Node is stateless.
  virtual const State* getNodeState() const;

  /// Set the Properties of this Node. By default, this does nothing.
  virtual void setNodeProperties(const std::unique_ptr<Properties>& properties);

  /// Get the Properties of this Node. By default, this returns a nullptr which
  /// implies that the Node has no properties.
  virtual const Properties* getNodeProperties() const;

  /// Get a pointer to the BodyNode that this Node is associated with
  BodyNodePtr getBodyNodePtr();

  /// Get a pointer to the BodyNode that this Node is associated with
  ConstBodyNodePtr getBodyNodePtr() const;

  /// Returns true if this Node has been staged for removal from its BodyNode.
  /// It will be deleted once all strong references to it expire. If it is an
  /// AccessoryNode, you can call reattach() to prevent that from happening.
  bool isRemoved() const;

private:

  std::shared_ptr<NodeCleaner> generateCleaner();

protected:

  /// Allow your Node implementation to be cloned into a new BodyNode
  virtual Node* cloneNode(BodyNode* bn) const = 0;

  /// Constructor
  Node(BodyNode* _bn);

  /// Inform the Skeleton that the name of this Node has changed. If the name is
  /// already taken by another Node of the same type, then this function will
  /// return a modified version which is unique. If the name is not already
  /// taken, then it will just return the same name that the function was given.
  std::string registerNameChange(const std::string& newName);

  /// Set the State pointer for this Node.
  ///
  /// This should be called during construction of your Node, if your Node has a
  /// State.
  void setNodeStatePtr(State* ptr = nullptr);

  /// Set the Properties pointer for this Node.
  ///
  /// This should be called during construction of your Node, if your Node has a
  /// set of Properties.
  void setNodePropertiesPtr(Properties* ptr = nullptr);

  /// Attach the Node to its BodyNode
  void attach();

  /// When all external references to the Node disappear, it will be deleted
  void stageForRemoval();

  /// weak pointer to the cleaner for this Node. We use a shared_ptr "cleaner"
  /// class instead of managing Nodes directly with shared_ptrs because this
  /// scheme allows BodyNodes to circumvent the shared_ptr management by setting
  /// the mNode member of the Cleaner to a nullptr. That way the BodyNode can
  /// never be deleted by its Cleaner.
  std::weak_ptr<NodeCleaner> mCleaner;

  /// Pointer to the State of this Node
  State* mNodeStatePtr;

  /// Pointer to the Properties of this Node
  Properties* mNodePropertiesPtr;

  /// Pointer to the BodyNode that this Node is attached to
  BodyNode* mBodyNode;

  /// bool that tracks whether this Node is attached to its BodyNode
  bool mAmAttached;

  /// The index of this Node within its vector in its BodyNode's NodeMap
  size_t mIndexInBodyNode;

  /// The index of this Node within its vector in its Skeleton's NodeMap
  size_t mIndexInSkeleton;

  /// Index of this Node within its tree
  size_t mIndexInTree;
};

/// AccessoryNode provides an interface for Nodes to get their index within the
/// list of Nodes, as well as detach and reattach. This uses CRTP to get around
/// the diamond of death problem.
template <class NodeType>
class AccessoryNode
{
public:

  /// Virtual destructor
  virtual ~AccessoryNode() = default;

  /// Get the index of this Node within its BodyNode.
  size_t getIndexInBodyNode() const;

  /// Get the index of this Node within its Skeleton.
  size_t getIndexInSkeleton() const;

  /// Get the index of this Node within its tree.
  size_t getIndexInTree() const;

  /// Get the index of this Node's tree within its Skeleton
  size_t getTreeIndex() const;

  /// Stage the Node for removal. When all strong references to the Node expire,
  /// the Node will be removed from its BodyNode and deleted.
  void remove();

  /// Undo the effectos of calling remove(). The Node will continue to exist,
  /// even when all the strong references to it expire.
  void reattach();

protected:

  /// Prevent a non-inheriting class from constructing one
  AccessoryNode() = default;

};

} // namespace dynamics
} // namespace dart

#include "dart/dynamics/detail/Node.h"

#endif // DART_DYNAMICS_NODE_H_
