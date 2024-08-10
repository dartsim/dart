#ifndef DART_CONSTRAINT_COUPLERCONSTRAINT_HPP_
#define DART_CONSTRAINT_COUPLERCONSTRAINT_HPP_

#include <dart/constraint/ConstraintBase.hpp>
#include <dart/dynamics/MimicDofProperties.hpp>
#include <vector>

namespace dart {
namespace constraint {

/// Coupler constraint that couples the motions of two joints by applying
/// equal and opposite impulses based on MimicDofProperties.
class CouplerConstraint : public ConstraintBase
{
public:
  /// Constructor that creates a CouplerConstraint using the given
  /// MimicDofProperties for each dependent joint's DoF.
  /// \param[in] joint The dependent joint.
  /// \param[in] mimicDofProperties A vector of MimicDofProperties for each DoF
  /// of the dependent joint.
  explicit CouplerConstraint(
      dynamics::Joint* joint,
      const std::vector<dynamics::MimicDofProperties>& mimicDofProperties);

  /// Destructor
  ~CouplerConstraint() override;

  // Documentation inherited
  const std::string& getType() const override;

  /// Returns constraint type for this class.
  static const std::string& getStaticType();

  /// Set global constraint force mixing parameter
  static void setConstraintForceMixing(double cfm);

  /// Get global constraint force mixing parameter
  static double getConstraintForceMixing();

  // Friendship
  friend class ConstraintSolver;
  friend class ConstrainedGroup;

protected:
  // Documentation inherited
  void update() override;

  // Documentation inherited
  void getInformation(ConstraintInfo* lcp) override;

  // Documentation inherited
  void applyUnitImpulse(std::size_t index) override;

  // Documentation inherited
  void getVelocityChange(double* delVel, bool withCfm) override;

  // Documentation inherited
  void excite() override;

  // Documentation inherited
  void unexcite() override;

  // Documentation inherited
  void applyImpulse(double* lambda) override;

  // Documentation inherited
  dynamics::SkeletonPtr getRootSkeleton() const override;

  // Documentation inherited
  bool isActive() const override;

private:
  /// Dependent joint whose motion is influenced by the reference joint.
  dynamics::Joint* mJoint;

  /// Vector of MimicDofProperties for the dependent joint.
  std::vector<dynamics::MimicDofProperties> mMimicProps;

  /// BodyNode associated with the dependent joint.
  dynamics::BodyNode* mBodyNode;

  /// Index of the applied impulse for the dependent joint.
  std::size_t mAppliedImpulseIndex;

  /// Array storing the lifetime of each constraint (in iterations).
  std::size_t mLifeTime[6];

  /// Array indicating whether each constraint is active or not.
  bool mActive[6];

  /// Array storing the negative velocity errors for each constraint.
  double mNegativeVelocityError[6];

  /// Array storing the previous values of the constraint forces.
  double mOldX[6];

  /// Array storing the upper bounds for the constraint forces.
  double mUpperBound[6];

  /// Array storing the lower bounds for the constraint forces.
  double mLowerBound[6];

  /// Global constraint force mixing parameter in the range of [1e-9, 1]. The
  /// default is 1e-5
  /// \sa http://www.ode.org/ode-latest-userguide.html#sec_3_8_0
  static double mConstraintForceMixing;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_COUPLERCONSTRAINT_HPP_
