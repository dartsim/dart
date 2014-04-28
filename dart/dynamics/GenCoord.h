/*
 * Copyright (c) 2011-2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_DYNAMICS_GENCOORD_H_
#define DART_DYNAMICS_GENCOORD_H_

#include <cstddef>
#include <string>

#include "dart/common/Deprecated.h"
#include "dart/math/Geometry.h"

namespace dart {
namespace dynamics {

/// \brief Generalized coordinate
class GenCoord
{
public:
  /// \brief Constructor
  GenCoord();

  /// \brief Destructor
  virtual ~GenCoord();

  /// \brief Set name
  void setName(const std::string& _name);

  /// \brief Get name
  const std::string& getName() const;

  /// \brief Set skeleton index
  void setSkeletonIndex(size_t _idx);

  /// \brief Get skeleton index
  size_t getSkeletonIndex() const;

  //----------------------------------------------------------------------------
  // Position
  // Deprecated functions will be remove by DART 4.0.1 or DART 4.1
  //----------------------------------------------------------------------------
  /// \brief Set position
  void setPos(double _pos);

  /// \brief Set configuration
  /// \warning Don't use me any more
  DEPRECATED(4.0) void setConfig(double _config);

  /// \brief Get position
  double getPos() const;

  /// \brief Get configuration
  /// \warning Don't use me any more
  DEPRECATED(4.0) double getConfig() const;

  /// \brief Set lower bound for configuration
  void setPosMin(double _posMin);

  /// \brief Set lower bound for configuration
  /// \warning Don't use me any more
  DEPRECATED(4.0) void setConfigMin(double _configMin);

  /// \brief Get lower bound for configuration
  double getPosMin() const;

  /// \brief Get lower bound for configuration
  /// \warning Don't use me any more
  DEPRECATED(4.0) double getConfigMin() const;

  /// \brief Set upper bound for configuration
  void setPosMax(double _posMax);

  /// \brief Set upper bound for configuration
  /// \warning Don't use me any more
  DEPRECATED(4.0) void setConfigMax(double _configMax);

  /// \brief Get upper bound for configuration
  double getPosMax() const;

  /// \brief Get upper bound for configuration
  /// \warning Don't use me any more
  DEPRECATED(4.0) double getConfigMax() const;

  /// \brief Set derivative w.r.t. arbitrary scalar value
  void setPosDeriv(double _posDeriv);

  /// \brief Set derivative w.r.t. arbitrary scalar value
  /// \warning Don't use me any more
  DEPRECATED(4.0) void setConfigDeriv(double _configDeriv);

  /// \brief Get derivative w.r.t. arbitrary scalar value
  double getPosDeriv() const;

  /// \brief Get derivative w.r.t. arbitrary scalar value
  /// \warning Don't use me any more
  DEPRECATED(4.0) double getConfigDeriv() const;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------
  /// \brief Set generalized velocity
  void setVel(double _vel);

  /// \brief Get generalized velocity
  double getVel() const;

  /// \brief Set lower bound for generalized velocity
  void setVelMin(double _velMin);

  /// \brief Get lower bound for generalized velocity
  double getVelMin() const;

  /// \brief Set upper bound for generalized velocity
  void setVelMax(double _velMax);

  /// \brief Get upper bound for generalized velocity
  double getVelMax() const;

  /// \brief Set derivative w.r.t. arbitrary scalar value
  void setVelDeriv(double _velDeriv);

  /// \brief Get derivative w.r.t. arbitrary scalar value
  double getVelDeriv() const;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------
  /// \brief Set generalized acceleration
  void setAcc(double _acc);

  /// \brief Get generalized acceleration
  double getAcc() const;

  /// \brief Set lower bound for generalized acceleration
  void setAccMin(double _accMin);

  /// \brief Get lower bound for generalized acceleration
  double getAccMin() const;

  /// \brief Set upper bound for generalized acceleration
  void setAccMax(double _accMax);

  /// \brief Get upper bound for generalized acceleration
  double getAccMax() const;

  /// \brief Set derivative w.r.t. arbitrary scalar value
  void setAccDeriv(double _accDeriv);

  /// \brief Get derivative w.r.t. arbitrary scalar value
  double getAccDeriv() const;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------
  /// \brief Set generalized force
  void setForce(double _force);

  /// \brief Get generalized force
  double getForce() const;

  /// \brief Set lower bound for generalized force
  void setForceMin(double _forceMin);

  /// \brief Get lower bound for generalized force
  double getForceMin() const;

  /// \brief Set upper bound for generalized force
  void setForceMax(double _forceMax);

  /// \brief Get upper bound for generalized force
  double getForceMax() const;

  /// \brief Set derivative w.r.t. arbitrary scalar value
  void setForceDeriv(double _forceDeriv);

  /// \brief Get derivative w.r.t. arbitrary scalar value
  double getForceDeriv() const;

  //----------------------------------------------------------------------------
  // Impulse
  //----------------------------------------------------------------------------
  /// \brief Set velocity change
  void setVelChange(double _velChange);

  /// \brief Get velocity change
  double getVelChange() const;

//  /// \brief Set impulse
//  void setImpulse(double _impulse);

//  /// \brief Get impulse
//  double getImpulse() const;

  /// \brief Set generalized constraint impulse
  void setConstraintImpulse(double _constraintImpulse);

  /// \brief Get generalized constraint impulse
  double getConstraintImpulse() const;

  //----------------------------------------------------------------------------
  // Integration
  //----------------------------------------------------------------------------
  /// \brief Integrate configuration with generalized velocity and _dt
  void integrateConfig(double _dt);

  /// \brief Integrate generalized velocity with generalized acceleration and
  /// _dt
  void integrateVel(double _dt);

protected:
  /// \brief Name
  std::string mName;

  /// \brief Index in Skeleton
  size_t mSkelIndex;

  //----------------------------------------------------------------------------
  // Configuration
  //----------------------------------------------------------------------------
  /// \brief Position
  double mPos;

  /// \brief Lower bound for position
  double mPosMin;

  /// \brief Upper bound for position
  double mPosMax;

  /// \brief Derivatives w.r.t. an arbitrary scalr variable
  double mPosDeriv;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------
  /// \brief Generalized velocity
  double mVel;

  /// \brief Min value allowed.
  double mVelMin;

  /// \brief Max value allowed.
  double mVelMax;

  /// \brief Derivatives w.r.t. an arbitrary scalr variable
  double mVelDeriv;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------
  /// \brief Generalized acceleration
  double mAcc;

  /// \brief Min value allowed.
  double mAccMin;

  /// \brief upper bound for generalized acceleration
  double mAccMax;

  /// \brief Derivatives w.r.t. an arbitrary scalr variable
  double mAccDeriv;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------
  /// \brief Generalized force
  double mForce;

  /// \brief Min value allowed.
  double mForceMin;

  /// \brief Max value allowed.
  double mForceMax;

  /// \brief Derivatives w.r.t. an arbitrary scalr variable
  double mForceDeriv;

  //----------------------------------------------------------------------------
  // Impulse
  //----------------------------------------------------------------------------
  /// \brief Change of generalized velocity
  double mVelChange;

//  /// \brief Generalized impulse
//  double mImpulse;

  /// \brief Generalized constraint impulse
  double mConstraintImpulse;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_GENCOORD_H_
