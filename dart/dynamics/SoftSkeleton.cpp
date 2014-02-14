/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#include "dart/dynamics/SoftSkeleton.h"

#include <string>
#include <vector>

#include <dart/dynamics/Joint.h>

#include "dart/dynamics/PointMass.h"
#include "dart/dynamics/SoftBodyNode.h"

namespace dart {
namespace dynamics {

SoftSkeleton::SoftSkeleton(const std::string& _name)
  : Skeleton(_name)
{
}

SoftSkeleton::~SoftSkeleton()
{
}

void SoftSkeleton::addSoftBodyNode(SoftBodyNode* _body)
{
  assert(_body && _body->getParentJoint());
  mSoftBodyNodes.push_back(_body);

  Skeleton::addBodyNode(_body);
}

SoftBodyNode* SoftSkeleton::getSoftBodyNode(int _idx) const
{
  assert(0 <= _idx && _idx < mSoftBodyNodes.size());
  return mSoftBodyNodes[_idx];
}

int SoftSkeleton::getNumSoftBodyNodes() const
{
  return mSoftBodyNodes.size();
}

int SoftSkeleton::getNumRigidBodyNodes() const
{
  return mBodyNodes.size() - mSoftBodyNodes.size();
}

SoftBodyNode* SoftSkeleton::getSoftBodyNode(const std::string& _name) const
{
  assert(!_name.empty());
  for (std::vector<SoftBodyNode*>::const_iterator itrSoftBodyNode =
       mSoftBodyNodes.begin();
       itrSoftBodyNode != mSoftBodyNodes.end(); ++itrSoftBodyNode)
  {
    if ((*itrSoftBodyNode)->getName() == _name)
      return *itrSoftBodyNode;
  }
  return NULL;
}

void SoftSkeleton::init(double _timeStep, const Eigen::Vector3d& _gravity)
{
  // Initialize this skeleton.
  Skeleton::init(_timeStep, _gravity);

  // Store generalized coordinates of point masses of all the soft body nodes
  // in this soft skeleton to mPointMassGenCoords additionaly. All the
  // generalized coordinates of the joints and the point masses are stored in
  // mGenCoord.
  mPointMassGenCoords.clear();
  for (int i = 0; i < mSoftBodyNodes.size(); i++)
  {
    SoftBodyNode* softBodyNode = mSoftBodyNodes[i];
    softBodyNode->aggregatePointMassGenCoords(&mPointMassGenCoords);
  }
}

void SoftSkeleton::updateExternalForceVector()
{
  Skeleton::updateExternalForceVector();

  for (std::vector<SoftBodyNode*>::iterator it = mSoftBodyNodes.begin();
       it != mSoftBodyNodes.end(); ++it)
  {
    double kv = (*it)->getVertexSpringStiffness();
    double ke = (*it)->getEdgeSpringStiffness();

    for (int i = 0; i < (*it)->getNumPointMasses(); ++i)
    {
      PointMass* pm = (*it)->getPointMass(i);
      int nN = pm->getNumConnectedPointMasses();

      // Vertex restoring force
      Eigen::Vector3d Fext = -(kv + nN * ke) * pm->get_q()
                             - (mTimeStep * (kv + nN*ke)) * pm->get_dq();

      // Edge restoring force
      for (int j = 0; j < nN; ++j)
      {
        Fext += ke * (pm->getConnectedPointMass(j)->get_q()
                        + mTimeStep * pm->getConnectedPointMass(j)->get_dq());
      }

      // Assign
      int iStart = pm->getGenCoord(0)->getSkeletonIndex();
      mFext.segment<3>(iStart) = Fext;
    }
  }
}

void SoftSkeleton::updateDampingForceVector()
{
  Skeleton::updateDampingForceVector();

  for (std::vector<SoftBodyNode*>::iterator it = mSoftBodyNodes.begin();
       it != mSoftBodyNodes.end(); ++it)
  {
    for (int i = 0; i < (*it)->getNumPointMasses(); ++i)
    {
      PointMass* pm = (*it)->getPointMass(i);
      int iStart = pm->getGenCoord(0)->getSkeletonIndex();
      mFd.segment<3>(iStart) = -(*it)->getDampingCoefficient() * pm->get_dq();
    }
  }
}

}  // namespace dynamics
}  // namespace dart

