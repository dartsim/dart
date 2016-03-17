/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
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

#include "dart/constraint/ConstraintSolver.h"

#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/SoftBodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/Skeleton.h"

#if DART_USE_FCLMESHCOLLISIONDETECTOR
  #include "dart/collision/fcl_mesh/FCLMeshCollisionDetector.h"
#else
  #include "dart/collision/fcl/FCLCollisionDetector.h"
#endif

#include "dart/constraint/ConstrainedGroup.h"
#include "dart/constraint/ContactConstraint.h"
#include "dart/constraint/SoftContactConstraint.h"
#include "dart/constraint/JointLimitConstraint.h"
#include "dart/constraint/ServoMotorConstraint.h"
#include "dart/constraint/JointCoulombFrictionConstraint.h"
#include "dart/constraint/DantzigLCPSolver.h"
#include "dart/constraint/PGSLCPSolver.h"

namespace dart {
namespace constraint {

using namespace dynamics;

//==============================================================================
ConstraintSolver::ConstraintSolver(double _timeStep)
#if DART_USE_FCLMESHCOLLISIONDETECTOR
  : mCollisionDetector(new collision::FCLMeshCollisionDetector),
#else
  : mCollisionDetector(new collision::FCLCollisionDetector),
#endif
    mTimeStep(_timeStep),
    mLCPSolver(new DantzigLCPSolver(mTimeStep))
{
  assert(_timeStep > 0.0);
}

//==============================================================================
ConstraintSolver::~ConstraintSolver()
{
}

//==============================================================================
void ConstraintSolver::addSkeleton(const SkeletonPtr& _skeleton)
{
  assert(_skeleton != nullptr
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
void ConstraintSolver::addSkeletons(const std::vector<SkeletonPtr>& _skeletons)
{
  size_t numAddedSkeletons = 0;

  for (std::vector<SkeletonPtr>::const_iterator it = _skeletons.begin();
       it != _skeletons.end(); ++it)
  {
    assert(*it != nullptr
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
void ConstraintSolver::removeSkeleton(
    const SkeletonPtr& _skeleton)
{
  assert(_skeleton != nullptr
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
    const std::vector<SkeletonPtr>& _skeletons)
{
  size_t numRemovedSkeletons = 0;

  for (std::vector<SkeletonPtr>::const_iterator it = _skeletons.begin();
       it != _skeletons.end(); ++it)
  {
    assert(*it != nullptr
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
void ConstraintSolver::addConstraint(const ConstraintBasePtr& _constraint)
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
void ConstraintSolver::removeConstraint(const ConstraintBasePtr& _constraint)
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
  setCollisionDetector(
    std::unique_ptr<collision::CollisionDetector>(_collisionDetector));
}

//==============================================================================
void ConstraintSolver::setCollisionDetector(
  std::unique_ptr<collision::CollisionDetector>&& _collisionDetector)
{
  assert(_collisionDetector && "Invalid collision detector.");

  // Change the collision detector of the constraint solver to new one
  mCollisionDetector = std::move(_collisionDetector);

  // Add skeletons in the constraint solver to new collision detector
  for (size_t i = 0; i < mSkeletons.size(); ++i)
    mCollisionDetector->addSkeleton(mSkeletons[i]);
}

//==============================================================================
collision::CollisionDetector* ConstraintSolver::getCollisionDetector() const
{
  return mCollisionDetector.get();
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
bool ConstraintSolver::containSkeleton(const ConstSkeletonPtr& _skeleton) const
{
  assert(_skeleton != nullptr && "Not allowed to insert null pointer skeleton.");

  for (std::vector<SkeletonPtr>::const_iterator it = mSkeletons.begin();
       it != mSkeletons.end(); ++it)
  {
    if ((*it) == _skeleton)
      return true;
  }

  return false;
}

//==============================================================================
bool ConstraintSolver::checkAndAddSkeleton(const SkeletonPtr& _skeleton)
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
bool ConstraintSolver::containConstraint(
    const ConstConstraintBasePtr& _constraint) const
{
  return std::find(mManualConstraints.begin(),
                   mManualConstraints.end(),
                   _constraint) != mManualConstraints.end();
}

//==============================================================================
bool ConstraintSolver::checkAndAddConstraint(
    const ConstraintBasePtr& _constraint)
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
  for (auto& manualConstraint : mManualConstraints)
  {
    manualConstraint->update();

    if (manualConstraint->isActive())
      mActiveConstraints.push_back(manualConstraint);
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: contact constraints
  //----------------------------------------------------------------------------
  mCollisionDetector->clearAllContacts();
  mCollisionDetector->detectCollision(true, true);

  // Destroy previous contact constraints
  mContactConstraints.clear();

  // Destroy previous soft contact constraints
  mSoftContactConstraints.clear();

  // Create new contact constraints
  for (size_t i = 0; i < mCollisionDetector->getNumContacts(); ++i)
  {
    collision::Contact& ct = mCollisionDetector->getContact(i);

    if (isSoftContact(ct))
    {
      mSoftContactConstraints.push_back(
            std::make_shared<SoftContactConstraint>(ct, mTimeStep));
    }
    else
    {
      mContactConstraints.push_back(
            std::make_shared<ContactConstraint>(ct, mTimeStep));
    }
  }

  // Add the new contact constraints to dynamic constraint list
  for (const auto& contactConstraint : mContactConstraints)
  {
    contactConstraint->update();

    if (contactConstraint->isActive())
      mActiveConstraints.push_back(contactConstraint);
  }

  // Add the new soft contact constraints to dynamic constraint list
  for (const auto& softContactConstraint : mSoftContactConstraints)
  {
    softContactConstraint->update();

    if (softContactConstraint->isActive())
      mActiveConstraints.push_back(softContactConstraint);
  }

  //----------------------------------------------------------------------------
  // Update automatic constraints: joint constraints
  //----------------------------------------------------------------------------
  // Destroy previous joint constraints
  mJointLimitConstraints.clear();
  mServoMotorConstraints.clear();
  mJointCoulombFrictionConstraints.clear();

  // Create new joint constraints
  for (const auto& skel : mSkeletons)
  {
    const size_t numJoints = skel->getNumJoints();
    for (size_t i = 0; i < numJoints; i++)
    {
      dynamics::Joint* joint = skel->getJoint(i);

      if (joint->isKinematic())
        continue;

      const size_t dof = joint->getNumDofs();
      for (size_t j = 0; j < dof; ++j)
      {
        if (joint->getCoulombFriction(j) != 0.0)
        {
          mJointCoulombFrictionConstraints.push_back(
                std::make_shared<JointCoulombFrictionConstraint>(joint));
          break;
        }
      }

      if (joint->isPositionLimitEnforced())
        mJointLimitConstraints.push_back(
              std::make_shared<JointLimitConstraint>(joint));

      if (joint->getActuatorType() == dynamics::Joint::SERVO)
        mServoMotorConstraints.push_back(
              std::make_shared<ServoMotorConstraint>(joint));
    }
  }

  // Add active joint limit
  for (auto& jointLimitConstraint : mJointLimitConstraints)
  {
    jointLimitConstraint->update();

    if (jointLimitConstraint->isActive())
      mActiveConstraints.push_back(jointLimitConstraint);
  }

  for (auto& servoMotorConstraint : mServoMotorConstraints)
  {
    servoMotorConstraint->update();

    if (servoMotorConstraint->isActive())
      mActiveConstraints.push_back(servoMotorConstraint);
  }

  for (auto& jointFrictionConstraint : mJointCoulombFrictionConstraints)
  {
    jointFrictionConstraint->update();

    if (jointFrictionConstraint->isActive())
      mActiveConstraints.push_back(jointFrictionConstraint);
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
  for (std::vector<ConstraintBasePtr>::iterator it = mActiveConstraints.begin();
       it != mActiveConstraints.end(); ++it)
  {
    (*it)->uniteSkeletons();
  }

  //----------------------------------------------------------------------------
  // Build constraint groups
  //----------------------------------------------------------------------------
  for (std::vector<ConstraintBasePtr>::const_iterator it = mActiveConstraints.begin();
       it != mActiveConstraints.end(); ++it)
  {
    bool found = false;
    dynamics::SkeletonPtr skel = (*it)->getRootSkeleton();

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
  for (std::vector<ConstraintBasePtr>::const_iterator it = mActiveConstraints.begin();
       it != mActiveConstraints.end(); ++it)
  {
    dynamics::SkeletonPtr skel = (*it)->getRootSkeleton();
    mConstrainedGroups[skel->mUnionIndex].addConstraint(*it);
  }

  //----------------------------------------------------------------------------
  // Reset union since we don't need union information anymore.
  //----------------------------------------------------------------------------
  for (std::vector<dynamics::SkeletonPtr>::iterator it = mSkeletons.begin();
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
  if (dynamic_cast<dynamics::SoftBodyNode*>(_contact.bodyNode1.lock().get())
      || dynamic_cast<dynamics::SoftBodyNode*>(_contact.bodyNode2.lock().get()))
    return true;

  return false;
}

}  // namespace constraint
}  // namespace dart
