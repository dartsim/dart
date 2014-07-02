/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "dart/constraint/ConstraintSolver.h"

#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/collision/dart/DARTCollisionDetector.h"
#ifdef HAVE_BULLET_COLLISION
  #include "dart/collision/bullet/BulletCollisionDetector.h"
#endif
#include "dart/constraint/ConstrainedGroup.h"
#include "dart/constraint/ContactConstraint.h"
#include "dart/constraint/SoftContactConstraint.h"
#include "dart/constraint/JointLimitConstraint.h"
#include "dart/constraint/DantzigLCPSolver.h"
#include "dart/constraint/PGSLCPSolver.h"

namespace dart {
namespace constraint {

using namespace dynamics;

//==============================================================================
ConstraintSolver::ConstraintSolver(double _timeStep)
  : mCollisionDetector(new collision::FCLMeshCollisionDetector()),
    mTimeStep(_timeStep),
    mLCPSolver(new DantzigLCPSolver(mTimeStep))
{
  assert(_timeStep > 0.0);
}

//==============================================================================
ConstraintSolver::~ConstraintSolver()
{
  delete mCollisionDetector;
}

//==============================================================================
void ConstraintSolver::addSkeleton(Skeleton* _skeleton)
{
  assert(_skeleton != NULL
      && "Null pointer skeleton is now allowed to add to ConstraintSover.");

  if (containSkeleton(_skeleton) == false)
  {
    mSkeletons.push_back(_skeleton);
    mCollisionDetector->addSkeleton(_skeleton);
    mConstrainedGroups.reserve(mSkeletons.size());
  }
  else
  {
    dtwarn << "Skeleton [" << _skeleton->getName()
           << "] is already in ConstraintSolver." << std::endl;
  }
}

//==============================================================================
void ConstraintSolver::addSkeletons(const std::vector<Skeleton*>& _skeletons)
{
  size_t numAddedSkeletons = 0;

  for (std::vector<Skeleton*>::const_iterator it = _skeletons.begin();
       it != _skeletons.end(); ++it)
  {
    assert(*it != NULL
        && "Null pointer skeleton is now allowed to add to ConstraintSover.");

    if (containSkeleton(*it) == false)
    {
      mSkeletons.push_back(*it);
      mCollisionDetector->addSkeleton(*it);

      ++numAddedSkeletons;
    }
    else
    {
      dtwarn << "Skeleton [" << (*it)->getName()
             << "] is already in ConstraintSolver." << std::endl;
    }
  }

  mConstrainedGroups.reserve(mSkeletons.size());
}

//==============================================================================
void ConstraintSolver::removeSkeleton(Skeleton* _skeleton)
{
  assert(_skeleton != NULL
      && "Null pointer skeleton is now allowed to add to ConstraintSover.");

  if (containSkeleton(_skeleton))
  {
    mSkeletons.erase(remove(mSkeletons.begin(), mSkeletons.end(), _skeleton),
                     mSkeletons.end());
    mCollisionDetector->removeSkeleton(_skeleton);
    mConstrainedGroups.reserve(mSkeletons.size());
  }
  else
  {
    dtwarn << "Skeleton [" << _skeleton->getName()
           << "] is not in ConstraintSolver." << std::endl;
  }
}

//==============================================================================
void ConstraintSolver::removeSkeletons(
    const std::vector<Skeleton*>& _skeletons)
{
  size_t numRemovedSkeletons = 0;

  for (std::vector<Skeleton*>::const_iterator it = _skeletons.begin();
       it != _skeletons.end(); ++it)
  {
    assert(*it != NULL
        && "Null pointer skeleton is now allowed to add to ConstraintSover.");

    if (containSkeleton(*it))
    {
      mSkeletons.erase(remove(mSkeletons.begin(), mSkeletons.end(), *it),
                       mSkeletons.end());
      mCollisionDetector->removeSkeleton(*it);

      ++numRemovedSkeletons;
    }
    else
    {
      dtwarn << "Skeleton [" << (*it)->getName()
             << "] is not in ConstraintSolver." << std::endl;
    }
  }

  mConstrainedGroups.reserve(mSkeletons.size());
}

//==============================================================================
void ConstraintSolver::removeAllSkeletons()
{
  mCollisionDetector->removeAllSkeletons();
  mSkeletons.clear();
}

//==============================================================================
void ConstraintSolver::addConstraint(Constraint* _constraint)
{
  assert(_constraint);

  if (containConstraint(_constraint))
  {
    dtwarn << "Constraint solver already contains constraint that you are "
           << "trying to add." << std::endl;
    return;
  }

  mManualConstraints.push_back(_constraint);
}

//==============================================================================
void ConstraintSolver::removeConstraint(Constraint* _constraint)
{
  assert(_constraint);

  if (!containConstraint(_constraint))
  {
    dtwarn << "Constraint solver deos not contain constraint that you are "
           << "trying to remove." << std::endl;
    return;
  }

  mManualConstraints.erase(remove(mManualConstraints.begin(),
                                  mManualConstraints.end(), _constraint),
                           mManualConstraints.end());
}

//==============================================================================
void ConstraintSolver::removeAllConstraints()
{
  mManualConstraints.clear();
}

//==============================================================================
void ConstraintSolver::setTimeStep(double _timeStep)
{
  assert(_timeStep > 0.0 && "Time step should be positive value.");
  mTimeStep = _timeStep;

  if (mLCPSolver)
    mLCPSolver->setTimeStep(mTimeStep);
}

//==============================================================================
double ConstraintSolver::getTimeStep() const
{
  return mTimeStep;
}

//==============================================================================
void ConstraintSolver::setCollisionDetector(
    collision::CollisionDetector* _collisionDetector)
{
  assert(_collisionDetector && "Invalid collision detector.");

  // Add skeletons in the constraint solver to new collision detector
  for (size_t i = 0; i < mSkeletons.size(); ++i)
    _collisionDetector->addSkeleton(mSkeletons[i]);

  // Release the old collision detector
  delete mCollisionDetector;

  // Change the collision detector of the constraint solver to new one
  mCollisionDetector = _collisionDetector;
}

//==============================================================================
collision::CollisionDetector* ConstraintSolver::getCollisionDetector() const
{
  return mCollisionDetector;
}

//==============================================================================
void ConstraintSolver::solve()
{
  for (size_t i = 0; i < mSkeletons.size(); ++i)
    mSkeletons[i]->clearConstraintImpulses();

  // Update constraints and collect active constraints
  updateConstraints();

  // Build constrained groups
  buildConstrainedGroups();

  // Solve constrained groups
  solveConstrainedGroups();
}

//==============================================================================
bool ConstraintSolver::containSkeleton(const Skeleton* _skeleton) const
{
  assert(_skeleton != NULL && "Not allowed to insert null pointer skeleton.");

  for (std::vector<Skeleton*>::const_iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    if ((*it) == _skeleton)
      return true;
  }

  return false;
}

//==============================================================================
bool ConstraintSolver::checkAndAddSkeleton(Skeleton* _skeleton)
{
  if (!containSkeleton(_skeleton))
  {
    mSkeletons.push_back(_skeleton);
    return true;
  }
  else
  {
    dtwarn << "Skeleton [" << _skeleton->getName()
           << "] is already in ConstraintSolver." << std::endl;
    return false;
  }
}

//==============================================================================
bool ConstraintSolver::containConstraint(const Constraint* _constraint) const
{
  if (std::find(mManualConstraints.begin(), mManualConstraints.end(),
                _constraint)
      != mManualConstraints.end())
  {
    return true;
  }

  return false;
}

//==============================================================================
bool ConstraintSolver::checkAndAddConstraint(Constraint* _constraint)
{
  if (!containConstraint(_constraint))
  {
    mManualConstraints.push_back(_constraint);
    return true;
  }
  else
  {
    dtwarn << "Constraint is already in ConstraintSolver." << std::endl;
    return false;
  }
}

//==============================================================================
void ConstraintSolver::updateConstraints()
{
  // Clear previous active constraint list
  mActiveConstraints.clear();

  //----------------------------------------------------------------------------
  // Update manual constraints
  //----------------------------------------------------------------------------
  for (std::vector<Constraint*>::iterator it = mManualConstraints.begin();
       it != mManualConstraints.end(); ++it)
  {
    (*it)->update();

    if ((*it)->isActive())
      mActiveConstraints.push_back(*it);
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: contact constraints
  //----------------------------------------------------------------------------
  mCollisionDetector->clearAllContacts();
  mCollisionDetector->detectCollision(true, true);

  // Destroy previous contact constraints
  for (std::vector<ContactConstraint*>::const_iterator it
       = mContactConstraints.begin();
       it != mContactConstraints.end(); ++it)
  {
    delete *it;
  }
  mContactConstraints.clear();

  // Destroy previous soft contact constraints
  for (std::vector<SoftContactConstraint*>::const_iterator it
       = mSoftContactConstraints.begin();
       it != mSoftContactConstraints.end(); ++it)
  {
    delete *it;
  }
  mSoftContactConstraints.clear();

  // Create new contact constraints
  for (size_t i = 0; i < mCollisionDetector->getNumContacts(); ++i)
  {
    const collision::Contact& ct = mCollisionDetector->getContact(i);

    if (isSoftContact(ct))
      mSoftContactConstraints.push_back(new SoftContactConstraint(ct));
    else
      mContactConstraints.push_back(new ContactConstraint(ct));
  }

  // Add the new contact constraints to dynamic constraint list
  for (std::vector<ContactConstraint*>::const_iterator it
       = mContactConstraints.begin();
       it != mContactConstraints.end(); ++it)
  {
    (*it)->update();

    if ((*it)->isActive())
      mActiveConstraints.push_back(*it);
  }

  // Add the new soft contact constraints to dynamic constraint list
  for (std::vector<SoftContactConstraint*>::const_iterator it
       = mSoftContactConstraints.begin();
       it != mSoftContactConstraints.end(); ++it)
  {
    (*it)->update();

    if ((*it)->isActive())
      mActiveConstraints.push_back(*it);
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: joint limit constraints
  //----------------------------------------------------------------------------
  // Destroy previous joint limit constraints
  for (std::vector<JointLimitConstraint*>::const_iterator it
       = mJointLimitConstraints.begin();
       it != mJointLimitConstraints.end(); ++it)
  {
    delete *it;
  }
  mJointLimitConstraints.clear();

  // Create new joint limit constraints
  for (std::vector<Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    for (size_t i = 0; i < (*it)->getNumBodyNodes(); i++)
    {
      dynamics::Joint* joint = (*it)->getBodyNode(i)->getParentJoint();
      if (joint->isPositionLimited())
        mJointLimitConstraints.push_back(new JointLimitConstraint(joint));
    }
  }

  // Add active joint limit
  for (std::vector<JointLimitConstraint*>::const_iterator it
       = mJointLimitConstraints.begin();
       it != mJointLimitConstraints.end(); ++it)
  {
    (*it)->update();

    if ((*it)->isActive())
      mActiveConstraints.push_back(*it);
  }
}

//==============================================================================
void ConstraintSolver::buildConstrainedGroups()
{
  // Clear constrained groups
  mConstrainedGroups.clear();

  // Exit if there is no active constraint
  if (mActiveConstraints.empty())
    return;

  //----------------------------------------------------------------------------
  // Unite skeletons according to constraints's relationships
  //----------------------------------------------------------------------------
  for (std::vector<Constraint*>::iterator it = mActiveConstraints.begin();
       it != mActiveConstraints.end(); ++it)
  {
    (*it)->uniteSkeletons();
  }

  //----------------------------------------------------------------------------
  // Build constraint groups
  //----------------------------------------------------------------------------
  for (std::vector<Constraint*>::const_iterator it = mActiveConstraints.begin();
       it != mActiveConstraints.end(); ++it)
  {
    bool found = false;
    dynamics::Skeleton* skel = (*it)->getRootSkeleton();

    for (std::vector<ConstrainedGroup>::const_iterator itConstGroup
         = mConstrainedGroups.begin();
         itConstGroup != mConstrainedGroups.end(); ++itConstGroup)
    {
      if ((*itConstGroup).mRootSkeleton == skel)
      {
        found = true;
        break;
      }
    }

    if (found)
      continue;

    ConstrainedGroup newConstGroup;
    newConstGroup.mRootSkeleton = skel;
    skel->mUnionIndex = mConstrainedGroups.size();
    mConstrainedGroups.push_back(newConstGroup);
  }

  // Add active constraints to constrained groups
  for (std::vector<Constraint*>::const_iterator it = mActiveConstraints.begin();
       it != mActiveConstraints.end(); ++it)
  {
    dynamics::Skeleton* skel = (*it)->getRootSkeleton();
    mConstrainedGroups[skel->mUnionIndex].addConstraint(*it);
  }

  //----------------------------------------------------------------------------
  // Reset union since we don't need union information anymore.
  //----------------------------------------------------------------------------
  for (std::vector<dynamics::Skeleton*>::iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    (*it)->resetUnion();
  }
}

//==============================================================================
void ConstraintSolver::solveConstrainedGroups()
{
  for (std::vector<ConstrainedGroup>::iterator it = mConstrainedGroups.begin();
       it != mConstrainedGroups.end(); ++it)
  {
    mLCPSolver->solve(&(*it));
  }
}

//==============================================================================
bool ConstraintSolver::isSoftContact(const collision::Contact& _contact) const
{
  if (dynamic_cast<dynamics::SoftBodyNode*>(_contact.bodyNode1)
      || dynamic_cast<dynamics::SoftBodyNode*>(_contact.bodyNode2))
    return true;

  return false;
}

}  // namespace constraint
}  // namespace dart
