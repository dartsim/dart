/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_CONSTRAINT_CONTACTSURFACE_HPP_
#define DART_CONSTRAINT_CONTACTSURFACE_HPP_

#include <Eigen/Core>

#include "dart/collision/Contact.hpp"
#include "dart/constraint/SmartPointer.hpp"
#include "dart/dynamics/ShapeNode.hpp"

#define DART_RESTITUTION_COEFF_THRESHOLD 1e-3
#define DART_FRICTION_COEFF_THRESHOLD 1e-3
#define DART_BOUNCING_VELOCITY_THRESHOLD 1e-1
#define DART_MAX_BOUNCING_VELOCITY 1e+2
#define DART_CONTACT_CONSTRAINT_EPSILON_SQUARED 1e-12

namespace dart {
namespace constraint {

constexpr double DART_DEFAULT_FRICTION_COEFF = 1.0;
constexpr double DART_DEFAULT_RESTITUTION_COEFF = 0.0;
// Slip compliance is combined through addition,
// so set to half the global default value
constexpr double DART_DEFAULT_SLIP_COMPLIANCE = 0.0;
const Eigen::Vector3d DART_DEFAULT_FRICTION_DIR = Eigen::Vector3d::UnitZ();
const Eigen::Vector3d DART_DEFAULT_CONTACT_SURFACE_MOTION_VELOCITY
    = Eigen::Vector3d::Zero();

/// Computed parameters of the contact surface
struct ContactSurfaceParams
{
  /// Primary Coefficient of Friction
  double mPrimaryFrictionCoeff{DART_DEFAULT_FRICTION_COEFF};

  /// Secondary Coefficient of Friction
  double mSecondaryFrictionCoeff{DART_DEFAULT_FRICTION_COEFF};

  /// Coefficient of restitution
  double mRestitutionCoeff{DART_DEFAULT_RESTITUTION_COEFF};

  /// Primary Coefficient of Slip Compliance
  double mPrimarySlipCompliance{DART_DEFAULT_SLIP_COMPLIANCE};

  /// Secondary Coefficient of Slip Compliance
  double mSecondarySlipCompliance{DART_DEFAULT_SLIP_COMPLIANCE};

  /// First frictional direction (in world frame)
  Eigen::Vector3d mFirstFrictionalDirection{DART_DEFAULT_FRICTION_DIR};

  /// Velocity of the contact independent of friction
  /// x = vel. in direction of contact normal
  /// y = vel. in first friction direction
  /// z = vel. in second friction direction
  Eigen::Vector3d mContactSurfaceMotionVelocity{
      DART_DEFAULT_CONTACT_SURFACE_MOTION_VELOCITY};

private:
  /// Used for future-compatibility. Add any newly added fields here so that
  /// ABI doesn't change. The data should be accessed via non-virtual getters
  /// and setters added to this struct. When this field starts to be used,
  /// custom destructor, copy/move constructors and copy/move assignment
  /// operators have to be defined that will take care of copying/destroying
  /// the extra data.
  void* mExtraData{nullptr};
};

/// Class used to determine the properties of a contact constraint based on the
/// two colliding bodies and information about their contact.
class ContactSurfaceHandler
{
public:
  /// Constructor
  /// \param[in] parent Optional parent handler. In ConstraintSolver, the parent
  ///                   handler is automatically set to the previous handler
  ///                   when adding a new one. It is suggested to keep this
  ///                   paradigm if used elsewhere.
  explicit ContactSurfaceHandler(ContactSurfaceHandlerPtr parent = nullptr);

  /// Create parameters of the contact constraint. This method should combine
  /// the collision properties of the two colliding bodies and write the
  /// combined result in the returned object. It is also passed the total number
  /// of contact points reported on the same collision body - that is useful
  /// e.g. for normalization of forces by the number of contact points.
  virtual ContactSurfaceParams createParams(
      const collision::Contact& contact,
      size_t numContactsOnCollisionObject) const;

  /// Create the constraint that represents contact between two collision
  /// objects.
  virtual ContactConstraintPtr createConstraint(
      collision::Contact& contact,
      size_t numContactsOnCollisionObject,
      double timeStep) const;

  /// Set the optional parent handler (ignored if parent.get() == this)
  void setParent(ContactSurfaceHandlerPtr parent);

  friend class ConstraintSolver;

protected:
  /// The optional parent handler
  ContactSurfaceHandlerPtr mParent;
};

/// Default contact surface handler. It chooses friction direction of the body
/// with lower friction coefficient. It also adjusts slip compliance by
/// mutliplying it with the number of contact points.
class DefaultContactSurfaceHandler : public ContactSurfaceHandler
{
public:
  // Documentation inherited
  ContactSurfaceParams createParams(
      const collision::Contact& contact,
      size_t numContactsOnCollisionObject) const override;

  // Documentation inherited
  ContactConstraintPtr createConstraint(
      collision::Contact& contact,
      size_t numContactsOnCollisionObject,
      double timeStep) const override;

protected:
  static double computeFrictionCoefficient(
      const dynamics::ShapeNode* shapeNode);
  static double computePrimaryFrictionCoefficient(
      const dynamics::ShapeNode* shapeNode);
  static double computeSecondaryFrictionCoefficient(
      const dynamics::ShapeNode* shapeNode);
  static double computePrimarySlipCompliance(
      const dynamics::ShapeNode* shapeNode);
  static double computeSecondarySlipCompliance(
      const dynamics::ShapeNode* shapeNode);
  static Eigen::Vector3d computeWorldFirstFrictionDir(
      const dynamics::ShapeNode* shapenode);
  static double computeRestitutionCoefficient(
      const dynamics::ShapeNode* shapeNode);

  friend class ContactConstraint;
};

} // namespace constraint
} // namespace dart

#endif // DART_CONSTRAINT_CONTACTSURFACE_HPP_
