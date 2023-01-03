/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/dynamics/ConstraintSolver.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/Macros.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/CollisionFilter.hpp"
#include "dart/dynamics/CollisionGroup.hpp"
#include "dart/dynamics/CollisionObject.hpp"
#include "dart/dynamics/ConstrainedGroup.hpp"
#include "dart/dynamics/Contact.hpp"
#include "dart/dynamics/ContactConstraint.hpp"
#include "dart/dynamics/ContactSurface.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/JointConstraint.hpp"
#include "dart/dynamics/JointCoulombFrictionConstraint.hpp"
#include "dart/dynamics/MimicMotorConstraint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SoftContactConstraint.hpp"
#include "dart/dynamics/fcl/FCLCollisionDetector.hpp"

#include <algorithm>

namespace dart {
namespace dynamics {

using namespace dynamics;

//==============================================================================
ConstraintSolver::ConstraintSolver()
  : mCollisionDetector(collision::FCLCollisionDetector::create()),
    mCollisionGroup(mCollisionDetector->createCollisionGroupAsSharedPtr()),
    mCollisionOption(collision::CollisionOption(
        true, 1000u, std::make_shared<collision::BodyNodeCollisionFilter>())),
    mTimeStep(0.001),
    mContactSurfaceHandler(std::make_shared<DefaultContactSurfaceHandler>())
{
  auto cd = std::static_pointer_cast<collision::FCLCollisionDetector>(
      mCollisionDetector);

  cd->setPrimitiveShapeType(collision::FCLCollisionDetector::MESH);
  // TODO(JS): Consider using FCL's primitive shapes once FCL addresses
  // incorrect contact point computation.
  // (see: https://github.com/flexible-collision-library/fcl/issues/106)
}

//==============================================================================
void ConstraintSolver::addSkeleton(const SkeletonPtr& skeleton)
{
  assert(
      skeleton
      && "Null pointer skeleton is now allowed to add to ConstraintSover.");

  if (hasSkeleton(skeleton))
  {
    dtwarn << "[ConstraintSolver::addSkeleton] Attempting to add "
           << "skeleton '" << skeleton->getName()
           << "', which already exists in the ConstraintSolver.\n";

    return;
  }

  mCollisionGroup->subscribeTo(skeleton);
  mSkeletons.push_back(skeleton);
  mConstrainedGroups.reserve(mSkeletons.size());
}

//==============================================================================
void ConstraintSolver::addSkeletons(const std::vector<SkeletonPtr>& skeletons)
{
  for (const auto& skeleton : skeletons)
    addSkeleton(skeleton);
}

//==============================================================================
const std::vector<SkeletonPtr>& ConstraintSolver::getSkeletons() const
{
  return mSkeletons;
}

//==============================================================================
void ConstraintSolver::removeSkeleton(const SkeletonPtr& skeleton)
{
  assert(
      skeleton
      && "Null pointer skeleton is now allowed to add to ConstraintSover.");

  if (!hasSkeleton(skeleton))
  {
    dtwarn << "[ConstraintSolver::removeSkeleton] Attempting to remove "
           << "skeleton '" << skeleton->getName()
           << "', which doesn't exist in the ConstraintSolver.\n";
  }

  mCollisionGroup->unsubscribeFrom(skeleton.get());
  mSkeletons.erase(
      remove(mSkeletons.begin(), mSkeletons.end(), skeleton), mSkeletons.end());
  mConstrainedGroups.reserve(mSkeletons.size());
}

//==============================================================================
void ConstraintSolver::removeSkeletons(
    const std::vector<SkeletonPtr>& skeletons)
{
  for (const auto& skeleton : skeletons)
    removeSkeleton(skeleton);
}

//==============================================================================
void ConstraintSolver::removeAllSkeletons()
{
  mCollisionGroup->removeAllShapeFrames();
  mSkeletons.clear();
}

//==============================================================================
void ConstraintSolver::addConstraint(const ConstraintBasePtr& constraint)
{
  assert(constraint);

  if (containConstraint(constraint))
  {
    dtwarn << "Constraint solver already contains constraint that you are "
           << "trying to add." << std::endl;
    return;
  }

  mManualConstraints.push_back(constraint);
}

//==============================================================================
void ConstraintSolver::removeConstraint(const ConstraintBasePtr& constraint)
{
  assert(constraint);

  if (!containConstraint(constraint))
  {
    dtwarn << "Constraint solver deos not contain constraint that you are "
           << "trying to remove." << std::endl;
    return;
  }

  mManualConstraints.erase(
      remove(mManualConstraints.begin(), mManualConstraints.end(), constraint),
      mManualConstraints.end());
}

//==============================================================================
void ConstraintSolver::removeAllConstraints()
{
  mManualConstraints.clear();
}

//==============================================================================
std::size_t ConstraintSolver::getNumConstraints() const
{
  return mManualConstraints.size();
}

//==============================================================================
ConstraintBasePtr ConstraintSolver::getConstraint(std::size_t index)
{
  return mManualConstraints[index];
}

//==============================================================================
ConstConstraintBasePtr ConstraintSolver::getConstraint(std::size_t index) const
{
  return mManualConstraints[index];
}

//==============================================================================
void ConstraintSolver::clearLastCollisionResult()
{
  mCollisionResult.clear();
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
    const std::shared_ptr<collision::CollisionDetector>& collisionDetector)
{
  if (!collisionDetector)
  {
    dtwarn << "[ConstraintSolver::setCollisionDetector] Attempting to assign "
           << "nullptr as the new collision detector to the constraint solver, "
           << "which is not allowed. Ignoring.\n";
    return;
  }

  if (mCollisionDetector == collisionDetector)
    return;

  mCollisionDetector = collisionDetector;

  mCollisionGroup = mCollisionDetector->createCollisionGroupAsSharedPtr();

  for (const auto& skeleton : mSkeletons)
    mCollisionGroup->addShapeFramesOf(skeleton.get());
}

//==============================================================================
collision::CollisionDetectorPtr ConstraintSolver::getCollisionDetector()
{
  return mCollisionDetector;
}

//==============================================================================
collision::ConstCollisionDetectorPtr ConstraintSolver::getCollisionDetector()
    const
{
  return mCollisionDetector;
}

//==============================================================================
collision::CollisionGroupPtr ConstraintSolver::getCollisionGroup()
{
  return mCollisionGroup;
}

//==============================================================================
collision::ConstCollisionGroupPtr ConstraintSolver::getCollisionGroup() const
{
  return mCollisionGroup;
}

//==============================================================================
collision::CollisionOption& ConstraintSolver::getCollisionOption()
{
  return mCollisionOption;
}

//==============================================================================
const collision::CollisionOption& ConstraintSolver::getCollisionOption() const
{
  return mCollisionOption;
}

//==============================================================================
collision::CollisionResult& ConstraintSolver::getLastCollisionResult()
{
  return mCollisionResult;
}

//==============================================================================
const collision::CollisionResult& ConstraintSolver::getLastCollisionResult()
    const
{
  return mCollisionResult;
}

//==============================================================================
void ConstraintSolver::solve()
{
  for (auto& skeleton : mSkeletons)
  {
    skeleton->clearConstraintImpulses();
  }

  // Update constraints and collect active constraints
  updateConstraints();

  // Build constrained groups
  buildConstrainedGroups();

  // Solve constrained groups
  solveConstrainedGroups();
}

//==============================================================================
void ConstraintSolver::setFromOtherConstraintSolver(
    const ConstraintSolver& other)
{
  removeAllSkeletons();
  mManualConstraints.clear();

  addSkeletons(other.getSkeletons());
  mManualConstraints = other.mManualConstraints;

  mContactSurfaceHandler = other.mContactSurfaceHandler;
}

//==============================================================================
bool ConstraintSolver::hasSkeleton(const ConstSkeletonPtr& skeleton) const
{
#if _WIN32
  DART_ASSERT(
      skeleton != nullptr && "Not allowed to insert null pointer skeleton.");
#else
  DART_ASSERT(
      skeleton != nullptr, "Not allowed to insert null pointer skeleton.");
#endif

  for (const auto& itrSkel : mSkeletons)
  {
    if (itrSkel == skeleton)
      return true;
  }

  return false;
}

//==============================================================================
bool ConstraintSolver::checkAndAddSkeleton(const SkeletonPtr& skeleton)
{
  if (!hasSkeleton(skeleton))
  {
    mSkeletons.push_back(skeleton);
    return true;
  }
  else
  {
    dtwarn << "Skeleton [" << skeleton->getName()
           << "] is already in ConstraintSolver." << std::endl;
    return false;
  }
}

//==============================================================================
bool ConstraintSolver::containConstraint(
    const ConstConstraintBasePtr& constraint) const
{
  return std::find(
             mManualConstraints.begin(), mManualConstraints.end(), constraint)
         != mManualConstraints.end();
}

//==============================================================================
bool ConstraintSolver::checkAndAddConstraint(
    const ConstraintBasePtr& constraint)
{
  if (!containConstraint(constraint))
  {
    mManualConstraints.push_back(constraint);
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
  mCollisionResult.clear();

  mCollisionGroup->collide(mCollisionOption, &mCollisionResult);

  // Destroy previous contact constraints
  mContactConstraints.clear();

  // Destroy previous soft contact constraints
  mSoftContactConstraints.clear();

  // Create a mapping of contact pairs to the number of contacts between them
  using ContactPair
      = std::pair<collision::CollisionObject*, collision::CollisionObject*>;

  // Compare contact pairs while ignoring their order in the pair.
  struct ContactPairCompare
  {
    ContactPair getSortedPair(const ContactPair& a) const
    {
      if (a.first < a.second)
        return std::make_pair(a.second, a.first);
      return a;
    }

    bool operator()(const ContactPair& a, const ContactPair& b) const
    {
      // Sort each pair and then do a lexicographical comparison
      return getSortedPair(a) < getSortedPair(b);
    }
  };

  std::map<ContactPair, size_t, ContactPairCompare> contactPairMap;
  std::vector<collision::Contact*> contacts;

  // Create new contact constraints
  for (auto i = 0u; i < mCollisionResult.getNumContacts(); ++i)
  {
    auto& contact = mCollisionResult.getContact(i);

    if (collision::Contact::isZeroNormal(contact.normal))
    {
      // Skip this contact. This is because we assume that a contact with
      // zero-length normal is invalid.
      continue;
    }

    // If penetration depth is negative, then the collision isn't really
    // happening and the contact point should be ignored.
    // TODO(MXG): Investigate ways to leverage the proximity information of a
    //            negative penetration to improve collision handling.
    if (contact.penetrationDepth < 0.0)
      continue;

    if (isSoftContact(contact))
    {
      mSoftContactConstraints.push_back(
          std::make_shared<SoftContactConstraint>(contact, mTimeStep));
    }
    else
    {
      // Increment the count of contacts between the two collision objects
      ++contactPairMap[std::make_pair(
          contact.collisionObject1, contact.collisionObject2)];

      contacts.push_back(&contact);
    }
  }

  // Add the new contact constraints to dynamic constraint list
  for (auto* contact : contacts)
  {
    std::size_t numContacts = 1;
    auto it = contactPairMap.find(
        std::make_pair(contact->collisionObject1, contact->collisionObject2));
    if (it != contactPairMap.end())
      numContacts = it->second;

    auto contactConstraint = mContactSurfaceHandler->createConstraint(
        *contact, numContacts, mTimeStep);
    mContactConstraints.push_back(contactConstraint);

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
  mJointConstraints.clear();
  mMimicMotorConstraints.clear();
  mJointCoulombFrictionConstraints.clear();

  // Create new joint constraints
  for (const auto& skel : mSkeletons)
  {
    const std::size_t numJoints = skel->getNumJoints();
    for (std::size_t i = 0; i < numJoints; i++)
    {
      dynamics::Joint* joint = skel->getJoint(i);

      if (joint->isKinematic())
        continue;

      const std::size_t dof = joint->getNumDofs();
      for (std::size_t j = 0; j < dof; ++j)
      {
        if (joint->getCoulombFriction(j) != 0.0)
        {
          mJointCoulombFrictionConstraints.push_back(
              std::make_shared<JointCoulombFrictionConstraint>(joint));
          break;
        }
      }

      if (joint->areLimitsEnforced()
          || joint->getActuatorType() == dynamics::Joint::SERVO)
      {
        mJointConstraints.push_back(std::make_shared<JointConstraint>(joint));
      }

      if (joint->getActuatorType() == dynamics::Joint::MIMIC
          && joint->getMimicJoint())
      {
        mMimicMotorConstraints.push_back(std::make_shared<MimicMotorConstraint>(
            joint,
            joint->getMimicJoint(),
            joint->getMimicMultiplier(),
            joint->getMimicOffset()));
      }
    }
  }

  // Add active joint limit
  for (auto& jointLimitConstraint : mJointConstraints)
  {
    jointLimitConstraint->update();

    if (jointLimitConstraint->isActive())
      mActiveConstraints.push_back(jointLimitConstraint);
  }

  for (auto& mimicMotorConstraint : mMimicMotorConstraints)
  {
    mimicMotorConstraint->update();

    if (mimicMotorConstraint->isActive())
      mActiveConstraints.push_back(mimicMotorConstraint);
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
  for (const auto& activeConstraint : mActiveConstraints)
    activeConstraint->uniteSkeletons();

  //----------------------------------------------------------------------------
  // Build constraint groups
  //----------------------------------------------------------------------------
  for (const auto& activeConstraint : mActiveConstraints)
  {
    bool found = false;
    const auto& skel = activeConstraint->getRootSkeleton();

    for (const auto& constrainedGroup : mConstrainedGroups)
    {
      if (constrainedGroup.mRootSkeleton == skel)
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
  for (const auto& activeConstraint : mActiveConstraints)
  {
    const auto& skel = activeConstraint->getRootSkeleton();
    mConstrainedGroups[skel->mUnionIndex].addConstraint(activeConstraint);
  }

  //----------------------------------------------------------------------------
  // Reset union since we don't need union information anymore.
  //----------------------------------------------------------------------------
  for (auto& skeleton : mSkeletons)
    skeleton->resetUnion();
}

//==============================================================================
void ConstraintSolver::solveConstrainedGroups()
{
  for (auto& constraintGroup : mConstrainedGroups)
    solveConstrainedGroup(constraintGroup);
}

//==============================================================================
bool ConstraintSolver::isSoftContact(const collision::Contact& contact) const
{
  auto* shapeNode1 = contact.collisionObject1->getShapeFrame()->asShapeNode();
  auto* shapeNode2 = contact.collisionObject2->getShapeFrame()->asShapeNode();
  assert(shapeNode1);
  assert(shapeNode2);

  auto* bodyNode1 = shapeNode1->getBodyNodePtr().get();
  auto* bodyNode2 = shapeNode2->getBodyNodePtr().get();

  auto bodyNode1IsSoft
      = dynamic_cast<const dynamics::SoftBodyNode*>(bodyNode1) != nullptr;

  auto bodyNode2IsSoft
      = dynamic_cast<const dynamics::SoftBodyNode*>(bodyNode2) != nullptr;

  return bodyNode1IsSoft || bodyNode2IsSoft;
}

//==============================================================================
ContactSurfaceHandlerPtr ConstraintSolver::getLastContactSurfaceHandler() const
{
  return mContactSurfaceHandler;
}

//==============================================================================
void ConstraintSolver::addContactSurfaceHandler(
    ContactSurfaceHandlerPtr handler)
{
  // sanity check, do not add the same handler twice
  if (handler == mContactSurfaceHandler)
  {
    dterr << "Adding the same contact surface handler for the second time, "
          << "ignoring.\n";
    return;
  }
  handler->setParent(mContactSurfaceHandler);
  mContactSurfaceHandler = std::move(handler);
}

//==============================================================================
bool ConstraintSolver::removeContactSurfaceHandler(
    const ContactSurfaceHandlerPtr& handler)
{
  bool found = false;
  ContactSurfaceHandlerPtr current = mContactSurfaceHandler;
  ContactSurfaceHandlerPtr previous = nullptr;
  while (current != nullptr)
  {
    if (current == handler)
    {
      if (previous != nullptr)
        previous->mParent = current->mParent;
      else
        mContactSurfaceHandler = current->mParent;
      found = true;
      break;
    }
    previous = current;
    current = current->mParent;
  }

  if (mContactSurfaceHandler == nullptr)
    dterr << "No contact surface handler remained. This is an error. Add at "
          << "least DefaultContactSurfaceHandler." << std::endl;

  return found;
}

} // namespace dynamics
} // namespace dart
