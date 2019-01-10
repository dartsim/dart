/*
 * Copyright (c) 2011-2019, The DART development contributors
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

#ifndef DART_DYNAMICS_TRANSLATIONALJOINT2D_HPP_
#define DART_DYNAMICS_TRANSLATIONALJOINT2D_HPP_

#include "dart/dynamics/detail/PlanarJointAspect.hpp"
#include "dart/dynamics/detail/TranslationalJoint2DAspect.hpp"

namespace dart {
namespace dynamics {

/// TranslationalJoint2D represents a 2-dof joint, which has two orthogonal
/// translational axes.
///
/// First and second coordiantes represent the translations along first and
/// second translational axes, respectively.
class TranslationalJoint2D : public detail::TranslationalJoint2DBase
{
public:
  friend class Skeleton;
  using PlaneType = detail::PlaneType;
  using UniqueProperties = detail::TranslationalJoint2DUniqueProperties;
  using Properties = detail::TranslationalJoint2DProperties;
  using Base = detail::TranslationalJoint2DBase;

  DART_BAKE_SPECIALIZED_ASPECT_IRREGULAR(Aspect, TranslationalJoint2DAspect)

  TranslationalJoint2D(const TranslationalJoint2D&) = delete;

  /// Destructor
  virtual ~TranslationalJoint2D();

  /// Sets the Properties of this TranslationalJoint2D
  void setProperties(const Properties& properties);

  /// Sets the Properties of this TranslationalJoint2D
  void setProperties(const UniqueProperties& properties);

  /// Sets the AspectProperties of this TranslationalJoint2D
  void setAspectProperties(const AspectProperties& properties);

  /// Returns the Properties of this TranslationalJoint2D
  Properties getTranslationalJoint2DProperties() const;

  /// Copies the Properties of another TranslationalJoint2D
  void copy(const TranslationalJoint2D& otherJoint);

  /// Copies the Properties of another TranslationalJoint2D
  void copy(const TranslationalJoint2D* otherJoint);

  /// Copies the Properties of another TranslationalJoint2D
  TranslationalJoint2D& operator=(const TranslationalJoint2D& otherJoint);

  // Documentation inherited
  const std::string& getType() const override;

  /// Returns joint type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  bool isCyclic(std::size_t index) const override;

  /// Sets plane type as XY-plane
  /// \param[in] renameDofs If true, the names of dofs in this joint will be
  /// renmaed according to the plane type.
  void setXYPlane(bool renameDofs = true);

  /// Sets plane type as YZ-plane
  /// \param[in] renameDofs If true, the names of dofs in this joint will be
  /// renmaed according to the plane type.
  void setYZPlane(bool renameDofs = true);

  /// Sets plane type as ZX-plane
  /// \param[in] renameDofs If true, the names of dofs in this joint will be
  /// renmaed according to the plane type.
  void setZXPlane(bool renameDofs = true);

  /// Sets plane type as arbitrary plane with two orthogonal translational axes
  /// \param[in] transAxis1 First translational axis
  /// \param[in] transAxis2 Second translational axis
  /// \param[in] renameDofs If true, the names of dofs in this joint will be
  /// renmaed according to the plane type.
  void setArbitraryPlane(
      const Eigen::Vector3d& transAxis1,
      const Eigen::Vector3d& transAxis2,
      bool renameDofs = true);

  /// Returns plane type
  PlaneType getPlaneType() const;

  /// Returns first translational axis
  Eigen::Vector3d getTranslationalAxis1() const;

  /// Returns second translational axis
  Eigen::Vector3d getTranslationalAxis2() const;

  // Documentation inherited
  Eigen::Matrix<double, 6, 2> getRelativeJacobianStatic(
      const Eigen::Vector2d& positions) const override;

protected:
  /// Constructor called by Skeleton class
  explicit TranslationalJoint2D(const Properties& properties);

  // Documentation inherited
  Joint* clone() const override;

  using GenericJoint<math::R2Space>::getRelativeJacobianStatic;

  // Documentation inherited
  void updateDegreeOfFreedomNames() override;

  // Documentation inherited
  void updateRelativeTransform() const override;

  // Documentation inherited
  void updateRelativeJacobian(bool mandatory = true) const override;

  // Documentation inherited
  void updateRelativeJacobianTimeDeriv() const override;
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_TRANSLATIONALJOINT2D_HPP_
