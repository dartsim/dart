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

#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace dynamics {

class BodyNode;

/// The Node class is a base class for any object that attaches to a BodyNode.
/// This base class handles ownership and reference counting for the classes
/// that inherit it.
class Node
{
public:

  friend class BodyNode;

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

  class Cleaner
  {
  public:

    Cleaner(Node* _node);

    Cleaner(const Cleaner& _cleaner) = delete;

    ~Cleaner();

  private:

    Node* mNode;

  };

  std::shared_ptr<Cleaner> generateCleaner();

protected:

  /// Construct a typical Node that will be attached to a BodyNode
  enum ConstructNode_t { ConstructNode };

  /// Construct the Node base of a BodyNode
  enum ConstructBodyNode_t { ConstructBodyNode };

  /// Used when constructing a pure abstract class, because calling the Node
  /// constructor is just a formality
  enum ConstructAbstract_t { ConstructAbstract };

  Node(ConstructNode_t, BodyNode* _bn);

  Node(ConstructBodyNode_t);

  Node(ConstructAbstract_t);

  void attach();

  void stageForRemoval();

  std::weak_ptr<Cleaner> mCleaner;

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
} // namespace dart

#endif // DART_DYNAMICS_NODE_H_
