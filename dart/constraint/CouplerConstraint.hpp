#ifndef DART_CONSTRAINT_COUPLERCONSTRAINT_HPP_
#define DART_CONSTRAINT_COUPLERCONSTRAINT_HPP_

#include <dart/dynamics/ConstraintBase.hpp>
#include <dart/dynamics/BodyNode.hpp>

namespace dart {
namespace constraint {

class CouplerConstraint : public ConstraintBase
{
public:
  CouplerConstraint(dynamics::BodyNode* body1, dynamics::BodyNode* body2, double ratio);

  void setRatio(double ratio);
  double getRatio() const;

  void update() override;
  void applyImpulse() override;

private:
  dynamics::BodyNode* mBodyNode1;
  dynamics::BodyNode* mBodyNode2;
  double mRatio;
  Eigen::Vector6d mImpulse;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_COUPLERCONSTRAINT_HPP_

