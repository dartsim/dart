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

#ifndef DART_DYNAMICS_INERTIA_HPP_
#define DART_DYNAMICS_INERTIA_HPP_

#include <array>

#include "dart/math/MathTypes.hpp"

namespace dart {
namespace dynamics {

class Inertia
{
public:

  /// Enumeration for minimal inertia parameters
  enum Param {

    // Overall mass
    MASS = 0,

    // Center of mass components
    COM_X, COM_Y, COM_Z,

    // Moment of inertia components
    I_XX, I_YY, I_ZZ,
    I_XY, I_XZ, I_YZ

  };

  Inertia(double _mass=1, const Eigen::Vector3d& _com = Eigen::Vector3d::Zero(),
          const Eigen::Matrix3d& _momentOfInertia = Eigen::Matrix3d::Identity());

  Inertia(const Eigen::Matrix6d& _spatialInertiaTensor);

  Inertia(double _mass,
          double _comX, double _comY, double _comZ,
          double _Ixx, double _Iyy, double _Izz,
          double _Ixy, double _Ixz, double _Iyz);

  /// Set an inertial parameter
  void setParameter(Param _param, double _value);

  /// Get an inertial parameter
  double getParameter(Param _param) const;

  /// Set the mass
  void setMass(double _mass);

  /// Get the mass
  double getMass() const;

  /// Set the center of mass with respect to the Body-fixed frame
  void setLocalCOM(const Eigen::Vector3d& _com);

  /// Get the center of mass with respect to the Body-fixed frame
  const Eigen::Vector3d& getLocalCOM() const;

  /// Set the moment of inertia (about the center of mass). Note that only the
  /// top-right corner of the matrix will be used, because a well-formed inertia
  /// matrix is always symmetric.
  void setMoment(const Eigen::Matrix3d& _moment);

  /// Set the moment of inertia (about the center of mass)
  void setMoment(double _Ixx, double _Iyy, double _Izz,
                 double _Ixy, double _Ixz, double _Iyz);

  /// Get the moment of inertia
  Eigen::Matrix3d getMoment() const;

  /// Set the spatial tensor
  void setSpatialTensor(const Eigen::Matrix6d& _spatial);

  /// Get the spatial inertia tensor
  const Eigen::Matrix6d& getSpatialTensor() const;

  /// Returns true iff _moment is a physically valid moment of inertia
  static bool verifyMoment(const Eigen::Matrix3d& _moment,
                           bool _printWarnings = true,
                           double _tolerance = 1e-8);

  /// Returns true iff _spatial is a physically valid spatial inertia tensor
  static bool verifySpatialTensor(const Eigen::Matrix6d& _spatial,
                                  bool _printWarnings = true,
                                  double _tolerance = 1e-8);

  /// Returns true iff this Inertia object is physically valid
  bool verify(bool _printWarnings = true,
              double _tolerance = 1e-8) const;

  /// Check for equality
  bool operator==(const Inertia& other) const;

protected:

  /// Compute the spatial tensor based on the inertial parameters
  void computeSpatialTensor();

  /// Compute the inertial parameters from the spatial tensor
  void computeParameters();

  /// Overall mass
  double mMass;

  /// Center of mass in the Body frame
  Eigen::Vector3d mCenterOfMass;

  /// The six parameters of the moment of inertia located at the center of mass
  std::array<double,6> mMoment;

  /// Cache for generalized spatial inertia of the Body
  Eigen::Matrix6d mSpatialTensor;

public:
  // To get byte-aligned Eigen vectors
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace dynamics
} // namespace dart

#endif // DART_DYNAMICS_INERTIA_HPP_
