/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>,
 *            Tobias Kunz <tobias@gatech.edu>
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

#ifndef DART_COLLISION_COLLISIONDETECTOR_H_
#define DART_COLLISION_COLLISIONDETECTOR_H_

#include <vector>
#include <map>

#include <Eigen/Dense>

#include "dart/collision/CollisionNode.h"
#include "dart/dynamics/SmartPointer.h"

namespace dart {
namespace collision {

class CollisionObject;

/// Contact information
struct Contact {
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// Contact point w.r.t. the world frame
  Eigen::Vector3d point;

  /// Contact normal vector from bodyNode2 to bodyNode1 w.r.t. the world frame
  Eigen::Vector3d normal;

  /// Contact force acting on bodyNode1 w.r.t. the world frame
  ///
  /// The contact force acting on bodyNode2 is -force, which is the opposite
  /// direction of the force.
  Eigen::Vector3d force;

  /// First colliding body node
  dynamics::WeakBodyNodePtr bodyNode1;

  /// Second colliding body node
  dynamics::WeakBodyNodePtr bodyNode2;

  /// First colliding shape of the first body node
  dynamics::ShapePtr shape1;

  /// Second colliding shape of the first body node
  dynamics::ShapePtr shape2;

  collision::CollisionObject* collisionObject1;
  collision::CollisionObject* collisionObject2;
  // TODO(JS): shared_ptr

  /// Penetration depth
  double penetrationDepth;

  // TODO(JS): triID1 will be deprecated when we don't use fcl_mesh
  /// \brief
  int triID1;

  // TODO(JS): triID2 will be deprecated when we don't use fcl_mesh
  /// \brief
  int triID2;

  // TODO(JS): userData is an experimental variable.
  /// \brief User data.
  void* userData;
};

/// \brief class CollisionDetector
class CollisionDetector
{
public:
  /// \brief Constructor
  CollisionDetector();

  /// \brief Destructor
  virtual ~CollisionDetector();

  /// \brief Add skeleton
  virtual void addSkeleton(const dynamics::SkeletonPtr& _skeleton);

  /// \brief Remove skeleton
  virtual void removeSkeleton(const dynamics::SkeletonPtr& _skeleton);

  /// \brief Remove all skeletons
  virtual void removeAllSkeletons();

  // TODO(JS): Change accessibility to private
  /// \brief
  virtual void addCollisionSkeletonNode(dynamics::BodyNode* _bodyNode,
                                        bool _isRecursive = false);

  // TODO(JS): Change accessibility to private
  /// \brief
  virtual void removeCollisionSkeletonNode(dynamics::BodyNode* _bodyNode,
                                           bool _isRecursive = false);

  /// \brief
  virtual CollisionNode* createCollisionNode(dynamics::BodyNode* _bodyNode) = 0;

  /// \brief
  void enablePair(dynamics::BodyNode* _node1, dynamics::BodyNode* _node2);

  /// \brief
  void disablePair(dynamics::BodyNode* _node1, dynamics::BodyNode* _node2);

  /// Return true if there exists at least one contact
  /// \param[in] _checkAllCollision True to detect every collisions
  /// \param[in] _calculateContactPoints True to get contact points
  virtual bool detectCollision(bool _checkAllCollisions,
                               bool _calculateContactPoints) = 0;

  /// Return true if there exists contacts between two bodies
  /// \param[in] _calculateContactPoints True to get contact points
  bool detectCollision(dynamics::BodyNode* _node1, dynamics::BodyNode* _node2,
                       bool _calculateContactPoints);

  /// \brief
  size_t getNumContacts();

  /// \brief
  Contact& getContact(int _idx);

  /// \brief
  void clearAllContacts();

  /// \brief
  int getNumMaxContacts() const;

  /// \brief
  void setNumMaxContacs(int _num);

  /// \brief
  bool isCollidable(const CollisionNode* _node1, const CollisionNode* _node2);

protected:
  /// \brief
  virtual bool detectCollision(CollisionNode* _node1, CollisionNode* _node2,
                               bool _calculateContactPoints) = 0;

  /// \brief
  std::vector<Contact> mContacts;

  /// \brief
  std::vector<CollisionNode*> mCollisionNodes;

  /// \brief
  int mNumMaxContacts;

  /// \brief Skeleton array
  std::vector<dynamics::SkeletonPtr> mSkeletons;

private:
  /// \brief Return true if _skeleton is contained
  bool containSkeleton(const dynamics::SkeletonPtr& _skeleton);

  /// \brief
  bool getPairCollidable(const CollisionNode* _node1,
                         const CollisionNode* _node2);

  /// \brief
  void setPairCollidable(const CollisionNode* _node1,
                         const CollisionNode* _node2,
                         bool _val);

  /// \brief Return true if _bodyNode1 and _bodyNode2 are adjacent bodies
  bool isAdjacentBodies(const dynamics::BodyNode* _bodyNode1,
                        const dynamics::BodyNode* _bodyNode2) const;

  /// \brief
  CollisionNode* getCollisionNode(const dynamics::BodyNode* _bodyNode);

  /// \brief
  std::map<const dynamics::BodyNode*, CollisionNode*> mBodyCollisionMap;

  /// \brief
  std::vector<std::vector<bool> > mCollidablePairs;
};

}  // namespace collision
}  // namespace dart

#endif  // DART_COLLISION_COLLISIONDETECTOR_H_
