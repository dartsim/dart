/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Chen Tang <ctang40@gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"

#include <algorithm>
#include <cmath>

#include <fcl/collision.h>

#include "dart/renderer/LoadOpengl.h"
#include "dart/math/Helpers.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/PointMass.h"
#include "dart/collision/CollisionNode.h"
#include "dart/collision/fcl_mesh/CollisionShapes.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionNode.h"

namespace dart {
namespace collision {

//==============================================================================
FCLMeshCollisionDetector::FCLMeshCollisionDetector()
{
}

//==============================================================================
FCLMeshCollisionDetector::~FCLMeshCollisionDetector()
{
}

//==============================================================================
CollisionNode*FCLMeshCollisionDetector::createCollisionNode(
    dynamics::BodyNode* _bodyNode)
{
  return new FCLMeshCollisionNode(_bodyNode);
}

//==============================================================================
bool FCLMeshCollisionDetector::detectCollision(bool _checkAllCollisions,
                                               bool _calculateContactPoints)
{
  //----------------------------------------------------------------------------
  // Clear previous contact informations
  //----------------------------------------------------------------------------

  // Release userData for soft body
  for (int i = 0; i < mContacts.size(); ++i)
    delete static_cast<collision::SoftCollisionInfo*>(mContacts[i].userData);

  // Update the positions of vertices on meshs
  for (int i = 0; i < mCollisionNodes.size(); ++i)
    static_cast<FCLMeshCollisionNode*>(mCollisionNodes[i])->updateShape();

  // Clear previous contacts
  mContacts.clear();

  // Set the colliding state of body nodes and point masses to false
  for (int i = 0; i < mCollisionNodes.size(); i++)
  {
    mCollisionNodes[i]->getBodyNode()->setColliding(false);

    dynamics::SoftBodyNode* sb = dynamic_cast<dynamics::SoftBodyNode*>(
                                   mCollisionNodes[i]->getBodyNode());
    if (sb)
    {
      for (int j = 0; j < sb->getNumPointMasses(); ++j)
        sb->getPointMass(j)->setColliding(false);
    }
  }

  //----------------------------------------------------------------------------
  // Detect collisions
  //----------------------------------------------------------------------------

  bool collision = false;

  FCLMeshCollisionNode* FCLMeshCollisionNode1 = NULL;
  FCLMeshCollisionNode* FCLMeshCollisionNode2 = NULL;

  for (int i = 0; i < mCollisionNodes.size(); i++)
  {
    FCLMeshCollisionNode1
        = static_cast<FCLMeshCollisionNode*>(mCollisionNodes[i]);
    for (int j = i + 1; j < mCollisionNodes.size(); j++)
    {
      FCLMeshCollisionNode2
          = static_cast<FCLMeshCollisionNode*>(mCollisionNodes[j]);
      if (!isCollidable(FCLMeshCollisionNode1, FCLMeshCollisionNode2))
        continue;

      std::vector<Contact>* contactPoints
          = _calculateContactPoints ? &mContacts : NULL;
      if (FCLMeshCollisionNode1->detectCollision(FCLMeshCollisionNode2,
                                                 contactPoints,
                                                 mNumMaxContacts))
      {
        collision = true;
        mCollisionNodes[i]->getBodyNode()->setColliding(true);
        mCollisionNodes[j]->getBodyNode()->setColliding(true);

        if (!_checkAllCollisions)
          return true;
      }
    }
  }

  //----------------------------------------------------------------------------
  // Post processing for soft bodies
  //----------------------------------------------------------------------------

  // Convert face contact to vertex contact
  dynamics::BodyNode* body1 = NULL;
  dynamics::BodyNode* body2 = NULL;
  dynamics::SoftBodyNode* soft1 = NULL;
  dynamics::SoftBodyNode* soft2 = NULL;
  int nContacts = mContacts.size();
  for (int i = 0; i < nContacts; ++i)
  {
    Contact& cnt = mContacts[i];
    body1 = cnt.collisionNode1->getBodyNode();
    body2 = cnt.collisionNode2->getBodyNode();
    soft1 = dynamic_cast<dynamics::SoftBodyNode*>(body1);
    soft2 = dynamic_cast<dynamics::SoftBodyNode*>(body2);
//    cnt.userData = NULL;

    // If body1 is soft body node
    if (soft1 != NULL
        && static_cast<collision::SoftCollisionInfo*>(cnt.userData)->isSoft1)
    {
      assert(cnt.triID1 != -1);

      dynamics::PointMass* pm
          = selectCollidingPointMass(soft1, cnt.point, cnt.triID1);

      cnt.point = pm->getWorldPosition();

      static_cast<collision::SoftCollisionInfo*>(cnt.userData)->pm1 = pm;
      static_cast<collision::SoftCollisionInfo*>(cnt.userData)->pm2 = NULL;

      pm->setColliding(true);
    }

    if (soft2 != NULL
        && static_cast<collision::SoftCollisionInfo*>(cnt.userData)->isSoft2)
    {
      assert(cnt.triID2 != -1);

      dynamics::PointMass* pm
          = selectCollidingPointMass(soft2, cnt.point, cnt.triID2);

      cnt.point = pm->getWorldPosition();

      static_cast<collision::SoftCollisionInfo*>(cnt.userData)->pm1 = NULL;
      static_cast<collision::SoftCollisionInfo*>(cnt.userData)->pm2 = pm;

      pm->setColliding(true);
    }

    if (soft1 != NULL && soft2 != NULL
        && static_cast<collision::SoftCollisionInfo*>(cnt.userData)->isSoft1
        && static_cast<collision::SoftCollisionInfo*>(cnt.userData)->isSoft2)
    {
      assert(cnt.triID1 != -1);
      assert(cnt.triID2 != -1);

      dynamics::PointMass* pm1
          = selectCollidingPointMass(soft1, cnt.point, cnt.triID1);
      dynamics::PointMass* pm2
          = selectCollidingPointMass(soft2, cnt.point, cnt.triID2);

      // TODO(JS): Two different point mass has different contact point!!
      cnt.point = pm1->getWorldPosition();
//      cnt.point = pm2->getWorldPosition();

      static_cast<collision::SoftCollisionInfo*>(cnt.userData)->pm1 = pm1;
      static_cast<collision::SoftCollisionInfo*>(cnt.userData)->pm2 = pm2;

      pm1->setColliding(true);
      pm2->setColliding(true);
    }
  }

  return collision;
}

//==============================================================================
bool FCLMeshCollisionDetector::detectCollision(CollisionNode* _node1,
                                               CollisionNode* _node2,
                                               bool _calculateContactPoints)
{
  FCLMeshCollisionNode* collisionNode1 =
      static_cast<FCLMeshCollisionNode*>(_node1);
  FCLMeshCollisionNode* collisionNode2 =
      static_cast<FCLMeshCollisionNode*>(_node2);
  return collisionNode1->detectCollision(
        collisionNode2,
        _calculateContactPoints ? &mContacts : NULL,
        mNumMaxContacts);
}

//==============================================================================
void FCLMeshCollisionDetector::draw()
{
  for (int i = 0; i < mCollisionNodes.size(); i++)
    static_cast<FCLMeshCollisionNode*>(
        mCollisionNodes[i])->drawCollisionSkeletonNode();
}

//==============================================================================
dynamics::PointMass* FCLMeshCollisionDetector::selectCollidingPointMass(
    const dynamics::SoftBodyNode* _softBodyNode,
    const Eigen::Vector3d& _point,
    int _faceId)
{
  dynamics::PointMass* pointMass = NULL;

  const Eigen::Vector3i& face = _softBodyNode->getFace(_faceId);

  dynamics::PointMass* pm0 = _softBodyNode->getPointMass(face[0]);
  dynamics::PointMass* pm1 = _softBodyNode->getPointMass(face[1]);
  dynamics::PointMass* pm2 = _softBodyNode->getPointMass(face[2]);

  const Eigen::Vector3d& pos1 = pm0->getWorldPosition();
  const Eigen::Vector3d& pos2 = pm1->getWorldPosition();
  const Eigen::Vector3d& pos3 = pm2->getWorldPosition();

  double dist0 = (pos1 - _point).dot(pos1 - _point);
  double dist1 = (pos2 - _point).dot(pos2 - _point);
  double dist2 = (pos3 - _point).dot(pos3 - _point);

  if (dist0 > dist1)
  {
    if (dist1 > dist2)
      pointMass = pm2;
    else
      pointMass = pm1;
  }
  else
  {
    if (dist0 > dist2)
      pointMass = pm2;
    else
      pointMass = pm0;
  }

  return pointMass;
}

}  // namespace collision
}  // namespace dart
