#include "dart/constraint/CouplerConstraint.hpp"
#include "dart/common/Console.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/external/odelcpsolver/lcp.h"

#define DART_CFM 1e-9

namespace dart {
namespace constraint {

double CouplerConstraint::mConstraintForceMixing = DART_CFM;

//==============================================================================
CouplerConstraint::CouplerConstraint(
    dynamics::Joint* joint,
    const std::vector<dynamics::MimicDofProperties>& mimicDofProperties)
  : ConstraintBase(),
    mJoint(joint),
    mMimicProps(mimicDofProperties),
    mBodyNode(joint->getChildBodyNode()),
    mAppliedImpulseIndex(0)
{
  assert(joint);
  assert(joint->getNumDofs() <= mMimicProps.size());
  assert(mBodyNode);

  std::fill(mLifeTime, mLifeTime + 6, 0);
  std::fill(mActive, mActive + 6, false);
}

//==============================================================================
CouplerConstraint::~CouplerConstraint()
{
  // Do nothing
}

//==============================================================================
const std::string& CouplerConstraint::getType() const
{
  return getStaticType();
}

//==============================================================================
const std::string& CouplerConstraint::getStaticType()
{
  static const std::string name = "CouplerConstraint";
  return name;
}

//==============================================================================
void CouplerConstraint::setConstraintForceMixing(double cfm)
{
  // Clamp constraint force mixing parameter if it is out of the range
  if (cfm < 1e-9) {
    dtwarn << "[CouplerConstraint::setConstraintForceMixing] "
           << "Constraint force mixing parameter[" << cfm
           << "] is lower than 1e-9. "
           << "It is set to 1e-9.\n";
    mConstraintForceMixing = 1e-9;
  }

  mConstraintForceMixing = cfm;
}

//==============================================================================
double CouplerConstraint::getConstraintForceMixing()
{
  return mConstraintForceMixing;
}

//==============================================================================
void CouplerConstraint::update()
{
  // Reset dimension
  mDim = 0;

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    const auto& mimicProp = mMimicProps[i];

    double timeStep = mJoint->getSkeleton()->getTimeStep();
    double qError
        = mimicProp.mReferenceJoint->getPosition(mimicProp.mReferenceDofIndex)
              * mimicProp.mMultiplier
          + mimicProp.mOffset - mJoint->getPosition(i);
    double desiredVelocity = math::clip(
        qError / timeStep,
        mJoint->getVelocityLowerLimit(i),
        mJoint->getVelocityUpperLimit(i));

    mNegativeVelocityError[i] = desiredVelocity - mJoint->getVelocity(i);

    if (mNegativeVelocityError[i] != 0.0) {
      mUpperBound[i] = mJoint->getForceUpperLimit(i) * timeStep;
      mLowerBound[i] = mJoint->getForceLowerLimit(i) * timeStep;

      if (mActive[i]) {
        ++(mLifeTime[i]);
      } else {
        mActive[i] = true;
        mLifeTime[i] = 0;
      }

      ++mDim;
    } else {
      mActive[i] = false;
    }
  }
}

//==============================================================================
void CouplerConstraint::getInformation(ConstraintInfo* lcp)
{
  std::size_t index = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    assert(lcp->w[index] == 0.0);

    lcp->b[index] = mNegativeVelocityError[i];
    lcp->lo[index] = mLowerBound[i];
    lcp->hi[index] = mUpperBound[i];

    assert(lcp->findex[index] == -1);

    if (mLifeTime[i])
      lcp->x[index] = mOldX[i];
    else
      lcp->x[index] = 0.0;

    index++;
  }
}

//==============================================================================
void CouplerConstraint::applyUnitImpulse(std::size_t index)
{
  assert(index < mDim && "Invalid Index.");

  std::size_t localIndex = 0;
  const dynamics::SkeletonPtr& skeleton = mJoint->getSkeleton();

  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    if (localIndex == index) {
      const auto& mimicProp = mMimicProps[i];

      skeleton->clearConstraintImpulses();

      double impulse = 1.0;
      mJoint->setConstraintImpulse(i, impulse);

      // Using const_cast to remove constness for methods that modify state
      const_cast<dynamics::Joint*>(mimicProp.mReferenceJoint)->setConstraintImpulse(
          mimicProp.mReferenceDofIndex, -impulse * mimicProp.mMultiplier);

      skeleton->updateBiasImpulse(mBodyNode);
      const_cast<dynamics::Skeleton*>(mimicProp.mReferenceJoint->getSkeleton().get())
          ->updateBiasImpulse(const_cast<dynamics::BodyNode*>(mimicProp.mReferenceJoint->getChildBodyNode()));

      skeleton->updateVelocityChange();
      const_cast<dynamics::Skeleton*>(mimicProp.mReferenceJoint->getSkeleton().get())
          ->updateVelocityChange();

      mJoint->setConstraintImpulse(i, 0.0);
      const_cast<dynamics::Joint*>(mimicProp.mReferenceJoint)->setConstraintImpulse(mimicProp.mReferenceDofIndex, 0.0);
    }

    ++localIndex;
  }

  mAppliedImpulseIndex = index;
}

//==============================================================================
void CouplerConstraint::getVelocityChange(double* delVel, bool withCfm)
{
  assert(delVel != nullptr && "Null pointer is not allowed.");

  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    if (mJoint->getSkeleton()->isImpulseApplied())
      delVel[localIndex] = mJoint->getVelocityChange(i);
    else
      delVel[localIndex] = 0.0;

    ++localIndex;
  }

  if (withCfm) {
    delVel[mAppliedImpulseIndex]
        += delVel[mAppliedImpulseIndex] * mConstraintForceMixing;
  }

  assert(localIndex == mDim);
}

//==============================================================================
void CouplerConstraint::excite()
{
  mJoint->getSkeleton()->setImpulseApplied(true);
  for (const auto& mimicProp : mMimicProps)
  {
    const_cast<dynamics::Skeleton*>(mimicProp.mReferenceJoint->getSkeleton().get())
        ->setImpulseApplied(true);
  }
}

//==============================================================================
void CouplerConstraint::unexcite()
{
  mJoint->getSkeleton()->setImpulseApplied(false);
  for (const auto& mimicProp : mMimicProps)
  {
    const_cast<dynamics::Skeleton*>(mimicProp.mReferenceJoint->getSkeleton().get())
        ->setImpulseApplied(false);
  }
}

//==============================================================================
void CouplerConstraint::applyImpulse(double* lambda)
{
  std::size_t localIndex = 0;
  std::size_t dof = mJoint->getNumDofs();
  for (std::size_t i = 0; i < dof; ++i) {
    if (mActive[i] == false)
      continue;

    mJoint->setConstraintImpulse(
        i, mJoint->getConstraintImpulse(i) + lambda[localIndex]);

    auto& mimicProp = mMimicProps[i];

    // Using const_cast to remove constness for methods that modify state
    const_cast<dynamics::Joint*>(mimicProp.mReferenceJoint)->setConstraintImpulse(
        mimicProp.mReferenceDofIndex,
        mimicProp.mReferenceJoint->getConstraintImpulse(mimicProp.mReferenceDofIndex)
        - lambda[localIndex] * mimicProp.mMultiplier);

    mOldX[i] = lambda[localIndex];

    ++localIndex;
  }
}

//==============================================================================
dynamics::SkeletonPtr CouplerConstraint::getRootSkeleton() const
{
  return ConstraintBase::getRootSkeleton(mJoint->getSkeleton()->getSkeleton());
}

//==============================================================================
bool CouplerConstraint::isActive() const
{
  if (mJoint->getActuatorType() == dynamics::Joint::MIMIC)
    return true;

  return false;
}

} // namespace constraint
} // namespace dart
