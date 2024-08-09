#include "dart/constraint/CouplerConstraint.hpp"

namespace dart {
namespace constraint {

CouplerConstraint::CouplerConstraint(dynamics::BodyNode* body1, dynamics::BodyNode* body2, double ratio)
  : mBodyNode1(body1), mBodyNode2(body2), mRatio(ratio), mImpulse(Eigen::Vector6d::Zero()) {}

void CouplerConstraint::setRatio(double ratio)
{
  mRatio = ratio;
}

double CouplerConstraint::getRatio() const
{
  return mRatio;
}

void CouplerConstraint::update()
{
  // Implement the logic to update the constraint based on the ratio
  // For simplicity, we'll assume a direct proportional relationship
  Eigen::Vector6d velocity1 = mBodyNode1->getSpatialVelocity();
  Eigen::Vector6d velocity2 = mBodyNode2->getSpatialVelocity();

  mImpulse = mRatio * (velocity2 - velocity1);
}

void CouplerConstraint::applyImpulse()
{
  // Apply equal and opposite impulses to the connected bodies
  mBodyNode1->addConstraintImpulse(mImpulse);
  mBodyNode2->addConstraintImpulse(-mImpulse);
}

} // namespace constraint
} // namespace dart
