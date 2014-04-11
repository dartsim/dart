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
#include "dart/dynamics/Skeleton.h"
#include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#include "dart/collision/dart/DARTCollisionDetector.h"
#ifdef HAVE_BULLET_COLLISION
  #include "dart/collision/bullet/BulletCollisionDetector.h"
#endif
#include "dart/constraint/ConstrainedGroup.h"
//#include "dart/constraint/BallJointConstraint.h"
//#include "dart/constraint/ClosedLoopConstraint.h"
#include "dart/constraint/ContactConstraint.h"
#include "dart/constraint/JointLimitConstraint.h"
//#include "dart/constraint/RevoluteJointConstraint.h"
//#include "dart/constraint/WeldJointConstraint.h"

namespace dart {
namespace constraint {

using namespace dynamics;

//==============================================================================
ConstraintSolver::ConstraintSolver(const std::vector<dynamics::Skeleton*>& _skeletons,
    double _timeStep,
    bool   _useODE)
  : mTimeStep(_timeStep),
    mUseODE(_useODE),
    mCollisionDetector(new collision::FCLMeshCollisionDetector())
{
  init();
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



    // TODO(JS): Dont't initialize this solver but just add _skeleton.
    init();
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
  int numAddedSkeletons = 0;

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

//  if (numAddedSkeletons > 0)
  // TODO(JS): Dont't initialize this solver but just add _skeletons.
  init();
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

    // TODO(JS): Dont't initialize this solver but just remove _skeleton.
    init();
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
  int numRemovedSkeletons = 0;

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

//  if (numRemovedSkeletons > 0)
  // TODO(JS): Dont't initialize this solver but just remove _skeletons.
  init();
}

//==============================================================================
void ConstraintSolver::removeAllSkeletons()
{
  std::cout << "ConstraintSolver::removeAllSkeletons(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolver::addConstraint(Constraint* _constraint)
{
  std::cout << "ConstraintSolver::addConstraint(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolver::addConstraints(const std::vector<Constraint*>& _constraints)
{
  std::cout << "ConstraintSolver::addConstraints(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolver::removeConstraint(Constraint* _constraint)
{
  std::cout << "ConstraintSolver::removeConstraint(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolver::removeConstraints(const std::vector<Constraint*>& _constraints)
{
  std::cout << "ConstraintSolver::removeConstraints(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolver::removeAllConstraints()
{
  std::cout << "ConstraintSolver::removeAllConstraints(): "
            << "Not implemented yet."
            << std::endl;
}

//==============================================================================
void ConstraintSolver::setTimeStep(double _timeStep)
{
  assert(_timeStep > 0.0 && "Time step should be positive value.");
  mTimeStep = _timeStep;
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
  for (int i = 0; i < mSkeletons.size(); ++i)
    mSkeletons[i]->clearConstraintImpulses();

  // Refresh dynamic constraint list based on the new states of skeletons
  updateDynamicConstraints();

  //
  buildConstrainedGroups();

  //
  solveConstrainedGroups();
}

//==============================================================================
void ConstraintSolver::init()
{
  bakeConstraints();

  //----------------------------------------------------------------------------
  // Static constraints
  //----------------------------------------------------------------------------
  mStaticConstraints.clear();

  // Closed loop constraints
//  for (std::vector<ClosedLoopConstraint*>::const_iterator it
//       = mBakedClosedLoopConstraints.begin();
//       it != mBakedClosedLoopConstraints.end(); ++it)
//  {
//    mStaticConstraints.push_back(*it);
//  }

  //----------------------------------------------------------------------------
  // Dynamic constraints
  //----------------------------------------------------------------------------
  mDynamicConstraints.clear();

  int maxNumDynamicConstraints = 0;
  maxNumDynamicConstraints += mBakedContactConstraints.size();
  maxNumDynamicConstraints += mBakedJointLimitContraints.size();
  maxNumDynamicConstraints += mBakedJointConstraints.size();

  mDynamicConstraints.reserve(maxNumDynamicConstraints);

  //----------------------------------------------------------------------------
  // Constraint groups
  //----------------------------------------------------------------------------
  // TODO(JS): Create one ConstrainedGroup for test
  mConstrainedGroups.clear();
  mConstrainedGroups.reserve(mSkeletons.size());
}

//==============================================================================
void ConstraintSolver::bakeConstraints()
{
  // Contact constraints
  bakeContactConstraints();

  // Joint limit constraints
  bakeJointLimitConstraints();

  // closed loop constraints
  bakeClosedLoopConstraints();

  // Joint constraints
  bakeJointConstraints();
}

//==============================================================================
void ConstraintSolver::bakeContactConstraints()
{
//  dtdbg << "ConstraintSolver::__bakeContactConstraints(): "
//        << "Not implemented yet."
//        << std::endl;

  // Reset baked contact constraints
//  for (std::vector<ContactConstraint*>::iterator it
//           = mBakedContactConstraints.begin();
//       it != mBakedContactConstraints.end(); ++it)
//  {
//    delete *it;
//  }
//  mBakedContactConstraints.clear();

  // TODO(JS):
//  mCollisionDetector->removeAllSkeletons();
//  for (std::vector<Skeleton*>::iterator it = mSkeletons.begin();
//       it != mSkeletons.end(); ++it)
//  {
//    mCollisionDetector->addSkeleton(*it);
//  }
}

//==============================================================================
void ConstraintSolver::bakeJointLimitConstraints()
{
//  std::cout << "ConstraintSolver::__bakeJointLimitConstraints(): "
//            << "Not implemented yet."
//            << std::endl;
}

//==============================================================================
void ConstraintSolver::bakeClosedLoopConstraints()
{
//  std::cout << "ConstraintSolver::__bakeClosedLoopConstraints(): "
//            << "Not implemented yet."
//            << std::endl;
}

//==============================================================================
void ConstraintSolver::bakeJointConstraints()
{
//  std::cout << "ConstraintSolver::__bakeJointConstraints(): "
//            << "Not implemented yet."
//            << std::endl;
}

//==============================================================================
bool ConstraintSolver::containSkeleton(const Skeleton* _skeleton) const
{
  assert(_skeleton != NULL && "Now allowed to insert null pointer skeleton.");

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
  std::cout << "ConstraintSolverTEST::_containConstraint(): "
            << "Not implemented."
            << std::endl;

  return false;
}

//==============================================================================
bool ConstraintSolver::checkAndAddConstraint(Constraint* _constraint)
{
  std::cout << "ConstraintSolverTEST::_checkAndAddConstraint(): "
            << "Not implemented."
            << std::endl;

  if (!containConstraint(_constraint))
  {
//    mConstraints.push_back(_constraint);
  }
  else
  {
//    dtwarn << "Constraint [" << _constraint->getName()
//           << "] is already in ConstraintSolver." << std::endl;
  }

  return false;
}

//==============================================================================
void ConstraintSolver::updateDynamicConstraints()
{
  mDynamicConstraints.clear();

  //----------------------------------------------------------------------------
  // Populate contact constraints
  //----------------------------------------------------------------------------
  mCollisionDetector->clearAllContacts();
  mCollisionDetector->detectCollision(true, true);

  // Clear previous contact constraints
  for (std::vector<ContactConstraint*>::const_iterator it
       = mBakedContactConstraints.begin();
       it != mBakedContactConstraints.end(); ++it)
  {
    delete *it;
  }
  mBakedContactConstraints.clear();

  // Create new contact constraints
  for (int i = 0; i < mCollisionDetector->getNumContacts(); ++i)
  {
    const collision::Contact& ct = mCollisionDetector->getContact(i);
    mBakedContactConstraints.push_back(new ContactConstraint(ct));
  }

  // Add the new contact constraints to dynamic constraint list
  for (std::vector<ContactConstraint*>::const_iterator it
       = mBakedContactConstraints.begin();
       it != mBakedContactConstraints.end(); ++it)
  {
    // if ((*it)->isActive())
    mDynamicConstraints.push_back(*it);
  }

  //----------------------------------------------------------------------------
  // Inspect joint limit constraints
  //----------------------------------------------------------------------------
  // Joint limit constraints
  for (std::vector<JointLimitConstraint*>::const_iterator it
       = mBakedJointLimitContraints.begin();
       it != mBakedJointLimitContraints.end(); ++it)
  {
    if ((*it)->isActive())
      mDynamicConstraints.push_back(*it);
  }
}

//==============================================================================
void ConstraintSolver::buildConstrainedGroups()
{
  mConstrainedGroups.clear();

  if (mStaticConstraints.empty() && mDynamicConstraints.empty())
    return;

  //----------------------------------------------------------------------------
  // Build skeleton unions using constraints
  //----------------------------------------------------------------------------
  // TODO(JS): Warm start
  // Static constraints
  for (std::vector<Constraint*>::iterator it = mStaticConstraints.begin();
       it != mStaticConstraints.end(); ++it)
  {
    (*it)->uniteSkeletons();
  }

  // Dynamics constraints
  for (std::vector<Constraint*>::iterator it = mDynamicConstraints.begin();
       it != mDynamicConstraints.end(); ++it)
  {
    (*it)->uniteSkeletons();
  }

  // DEBUG CODE ////////////////////////////////////////////////////////////////
//  for (int i = 0; i < mSkeletons.size(); ++i)
//  {
//    std::cout << "Skeleton[" << i << "]: "
//              << mSkeletons[i] << ", "
//              << mSkeletons[i]->mUnionRootSkeleton << ", "
//              << mSkeletons[i]->mUnionSize << std::endl;
//  }
//  std::cout << std::endl;

  //----------------------------------------------------------------------------
  // Build constraint groups
  //----------------------------------------------------------------------------
  std::vector<dynamics::Skeleton*> possibleSkeletonGroupRoots;
  possibleSkeletonGroupRoots.reserve(mSkeletons.size());

  for (std::vector<Constraint*>::iterator it = mStaticConstraints.begin();
       it != mStaticConstraints.end(); ++it)
  {
    dynamics::Skeleton* skel = (*it)->getRootSkeleton();

    if (std::find(possibleSkeletonGroupRoots.begin(),
                  possibleSkeletonGroupRoots.end(), skel)
        == possibleSkeletonGroupRoots.end())
    {
      possibleSkeletonGroupRoots.push_back(skel);
    }
  }

  for (std::vector<Constraint*>::iterator it = mDynamicConstraints.begin();
       it != mDynamicConstraints.end(); ++it)
  {
    dynamics::Skeleton* skel = (*it)->getRootSkeleton();

    if (std::find(possibleSkeletonGroupRoots.begin(),
                  possibleSkeletonGroupRoots.end(), skel)
        == possibleSkeletonGroupRoots.end())
    {
      possibleSkeletonGroupRoots.push_back(skel);
    }
  }

  for (std::vector<dynamics::Skeleton*>::iterator itSkel
       = possibleSkeletonGroupRoots.begin();
       itSkel != possibleSkeletonGroupRoots.end(); ++itSkel)
  {
    bool found = false;

    for (std::vector<ConstrainedGroup>::iterator itConstGroup
         = mConstrainedGroups.begin();
         itConstGroup != mConstrainedGroups.end(); ++itConstGroup)
    {
      if ((*itConstGroup).mRootSkeleton == *itSkel)
      {
        found = true;
        break;
      }
    }

    if (found)
      continue;

    ConstrainedGroup newConstGroup(this);
    newConstGroup.mRootSkeleton = *itSkel;
    (*itSkel)->mUnionIndex = mConstrainedGroups.size();
    mConstrainedGroups.push_back(newConstGroup);
  }

  //----------------------------------------------------------------------------
  // Add Constraints to constrained groups
  //----------------------------------------------------------------------------
  // Static constraints
  for (std::vector<Constraint*>::iterator it = mStaticConstraints.begin();
       it != mStaticConstraints.end(); ++it)
  {
    dynamics::Skeleton* skel = (*it)->getRootSkeleton();
    mConstrainedGroups[skel->mUnionIndex].addConstraint(*it);
  }

  // Dynamics constraints
  for (std::vector<Constraint*>::iterator it = mDynamicConstraints.begin();
       it != mDynamicConstraints.end(); ++it)
  {
    dynamics::Skeleton* skel = (*it)->getRootSkeleton();
    mConstrainedGroups[skel->mUnionIndex].addConstraint(*it);
  }

  //----------------------------------------------------------------------------
  // Reset union. We don't need union information anymore.
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
  // TODO(JS): Parallel computing is possible here.
  for (std::vector<ConstrainedGroup>::iterator it = mConstrainedGroups.begin();
      it != mConstrainedGroups.end(); ++it)
  {
    (*it).solve();
  }
}

}  // namespace constraint
}  // namespace dart
