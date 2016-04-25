/*
 * Copyright (c) 2014-2016, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_PLANARRJOINT_H_
#define DART_DYNAMICS_PLANARRJOINT_H_

#include "dart/dynamics/detail/PlanarJointAspect.h"

namespace dart {
namespace dynamics {

/// PlanarJoint represents a 3-dof joint, which has two orthogonal translational
/// axes and one rotational axis.
///
/// First and second coordiantes represent translation along first and second
/// translational axese, respectively. Third coordinate represents rotation
/// along rotational axis.
class PlanarJoint : public detail::PlanarJointBase
{
public:

  friend class Skeleton;
  using PlaneType = detail::PlaneType;
  using UniqueProperties = detail::PlanarJointUniqueProperties;
  using Properties = detail::PlanarJointProperties;
  using Base = detail::PlanarJointBase;

  DART_BAKE_SPECIALIZED_ASPECT_IRREGULAR(Aspect, PlanarJointAspect)

  PlanarJoint(const PlanarJoint&) = delete;

  /// Destructor
  virtual ~PlanarJoint();

  /// Set the Properties of this PlanarJoint
  void setProperties(const Properties& _properties);

  /// Set the Properties of this PlanarJoint
  void setProperties(const UniqueProperties& _properties);

  /// Set the AspectProperties of this PlanarJoint
  void setAspectProperties(const AspectProperties& properties);

  /// Get the Properties of this PlanarJoint
  Properties getPlanarJointProperties() const;

  /// Copy the Properties of another PlanarJoint
  void copy(const PlanarJoint& _otherJoint);

  /// Copy the Properties of another PlanarJoint
  void copy(const PlanarJoint* _otherJoint);

  /// Same as copy(const PlanarJoint&)
  PlanarJoint& operator=(const PlanarJoint& _otherJoint);

  // Documentation inherited
  const std::string& getType() const override;

  /// Get joint type for this class
  static const std::string& getStaticType();

  // Documentation inherited
  bool isCyclic(std::size_t _index) const override;

  /// \brief Set plane type as XY-plane
  /// \param[in] _renameDofs If true, the names of dofs in this joint will be
  /// renmaed according to the plane type.
  void setXYPlane(bool _renameDofs = true);

  /// \brief Set plane type as YZ-plane
  /// \param[in] _renameDofs If true, the names of dofs in this joint will be
  /// renmaed according to the plane type.
  void setYZPlane(bool _renameDofs = true);

  /// \brief Set plane type as ZX-plane
  /// \param[in] _renameDofs If true, the names of dofs in this joint will be
  /// renmaed according to the plane type.
  void setZXPlane(bool _renameDofs = true);

  /// \brief Set plane type as arbitrary plane with two orthogonal translational
  /// axes
  /// \param[in] _renameDofs If true, the names of dofs in this joint will be
  /// renmaed according to the plane type.
  void setArbitraryPlane(const Eigen::Vector3d& _transAxis1,
                         const Eigen::Vector3d& _transAxis2,
                         bool _renameDofs = true);

  /// Return plane type
  PlaneType getPlaneType() const;

  /// Return rotational axis
  const Eigen::Vector3d& getRotationalAxis() const;

  /// Return first translational axis
  const Eigen::Vector3d& getTranslationalAxis1() const;

  /// Return second translational axis
  const Eigen::Vector3d& getTranslationalAxis2() const;

  // Documentation inherited
  Eigen::Matrix<double, 6, 3> getLocalJacobianStatic(
      const Eigen::Vector3d& _positions) const override;

protected:

  /// Constructor called by Skeleton class
  PlanarJoint(const Properties& properties);

  // Documentation inherited
  Joint* clone() const override;

  using MultiDofJoint::getLocalJacobianStatic;

  /// Set the names of this joint's DegreesOfFreedom. Used during construction
  /// and when the Plane type is changed.
  void updateDegreeOfFreedomNames() override;

  // Documentation inherited
  void updateLocalTransform() const override;

  // Documentation inherited
  void updateLocalJacobian(bool =true) const override;

  // Documentation inherited
  void updateLocalJacobianTimeDeriv() const override;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_PLANARRJOINT_H_

