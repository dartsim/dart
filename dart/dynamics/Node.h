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

#ifndef KIDO_DYNAMICS_NODE_H_
#define KIDO_DYNAMICS_NODE_H_

#include <memory>

#include "dart/common/Subject.h"
#include "dart/dynamics/SmartPointer.h"

namespace kido {
namespace dynamics {

class BodyNode;
class Node;

class NodeDestructor final
{
public:

  /// Constructor
  NodeDestructor(Node* _node);

  /// Do not copy
  NodeDestructor(const NodeDestructor& _other) = delete;

  /// Non-virtual destructor (this class cannot be inherited)
  ~NodeDestructor();

private:

  /// Node that this Destructor is responsible for
  Node* mNode;

};

/// The Node class is a base class for BodyNode and any object that attaches to
/// a BodyNode. This base class handles ownership and reference counting for the
/// classes that inherit it.
class Node : public virtual common::Subject
{
public:

  friend class BodyNode;
  template<class, class> friend class TemplateNodePtr;
  template<class, class> friend class TemplateWeakNodePtr;

  /// Virtual destructor
  virtual ~Node() = default;

  /// Get a pointer to the BodyNode that this Node is associated with
  BodyNodePtr getBodyNodePtr();

  /// Get a pointer to the BodyNode that this Node is associated with
  ConstBodyNodePtr getBodyNodePtr() const;

  /// Returns true if this Node has been staged for removal from its BodyNode.
  /// It will be deleted once all strong references to it expire. If it is an
  /// AccessoryNode, you can call reattach() to prevent that from happening.
  bool isRemoved() const;

private:

  std::shared_ptr<NodeDestructor> getOrCreateDestructor();

protected:

  /// Construct a typical Node that will be attached to a BodyNode
  enum ConstructNode_t { ConstructNode };

  /// Construct the Node base of a BodyNode
  enum ConstructBodyNode_t { ConstructBodyNode };

  /// Used when constructing a pure abstract class, because calling the Node
  /// constructor is just a formality
  enum ConstructAbstract_t { ConstructAbstract };

  /// Used when constructing a Node type that does NOT inherit from BodyNode
  Node(ConstructNode_t, BodyNode* _bn);

  /// Used when constructing a BodyNode
  Node(ConstructBodyNode_t);

  /// Used when constructing a pure abstract type
  Node(ConstructAbstract_t);

  /// Attach the Node to its BodyNode
  void attach();

  /// When all external references to the Node disappear, it will be deleted
  void stageForRemoval();

  /// weak pointer to the destructor for this Node. We use a shared_ptr
  /// "destructor" class instead of managing Nodes directly with shared_ptrs
  /// because this scheme allows BodyNodes to circumvent the shared_ptr
  /// management by setting the mNode member of the Destructor to a nullptr.
  /// That way the BodyNode can never be deleted by its Destructor.
  std::weak_ptr<NodeDestructor> mDestructor;

  /// Pointer to the BodyNode that this Node is attached to
  BodyNode* mBodyNode;
};

class AccessoryNode : public virtual Node
{
public:

  /// Virtual destructor
  virtual ~AccessoryNode() = default;

  /// Stage the Node for removal. When all strong references to the Node expire,
  /// the Node will be removed from its BodyNode and deleted.
  void remove();

  /// Undo the effectos of calling remove(). The Node will continue to exist,
  /// even when all the strong references to it expire.
  void reattach();

protected:

  /// Prevent a non-inheriting class from constructing one
  AccessoryNode();

};

} // namespace dynamics
} // namespace kido

#endif // KIDO_DYNAMICS_NODE_H_
