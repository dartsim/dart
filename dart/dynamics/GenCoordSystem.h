/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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

#ifndef DART_DYNAMICS_GENCOORDSYSTEM_H_
#define DART_DYNAMICS_GENCOORDSYSTEM_H_

#include <cstddef>
#include <vector>
#include <string>

#include <Eigen/Dense>

#include "dart/dynamics/GenCoord.h"

namespace dart {
namespace dynamics {

/// Base class for generalized coordinate systems
class GenCoordSystem
{
public:
  /// Get number of generalized coordinates
  virtual size_t getDof() const = 0;

  //----------------------------------------------------------------------------
  // Position
  //----------------------------------------------------------------------------

  /// Set a single position
  virtual void setPosition(size_t _index, double _position) = 0;

  /// Get a single position
  virtual double getPosition(size_t _index) const = 0;

  /// Set positions
  virtual void setPositions(const Eigen::VectorXd& _positions) = 0;

  /// Get positions
  virtual Eigen::VectorXd getPositions() const = 0;

  /// Set zero all the positions
  virtual void resetPositions() = 0;

  /// Set lower limit of position
  virtual void setPositionLowerLimit(size_t _index, double _position) = 0;

  /// Get lower limit for position
  virtual double getPositionLowerLimit(size_t _index) = 0;

  /// Set upper limit for position
  virtual void setPositionUpperLimit(size_t _index, double _position) = 0;

  /// Get upper limit for position
  virtual double getPositionUpperLimit(size_t _index) = 0;

  //----------------------------------------------------------------------------
  // Velocity
  //----------------------------------------------------------------------------

  /// Set a single velocity
  virtual void setVelocity(size_t _index, double _velocity) = 0;

  /// Get a single velocity
  virtual double getVelocity(size_t _index) const = 0;

  /// Set velocities
  virtual void setVelocities(const Eigen::VectorXd& _velocities) = 0;

  /// Get velocities
  virtual Eigen::VectorXd getVelocities() const = 0;

  /// Set zero all the velocities
  virtual void resetVelocities() = 0;

  /// Set lower limit of velocity
  virtual void setVelocityLowerLimit(size_t _index, double _velocity) = 0;

  /// Get lower limit of velocity
  virtual double getVelocityLowerLimit(size_t _index) = 0;

  /// Set upper limit of velocity
  virtual void setVelocityUpperLimit(size_t _index, double _velocity) = 0;

  /// Get upper limit of velocity
  virtual double getVelocityUpperLimit(size_t _index) = 0;

  //----------------------------------------------------------------------------
  // Acceleration
  //----------------------------------------------------------------------------

  /// Set a single acceleration
  virtual void setAcceleration(size_t _index, double _acceleration) = 0;

  /// Get a single acceleration
  virtual double getAcceleration(size_t _index) const = 0;

  /// Set accelerations
  virtual void setAccelerations(const Eigen::VectorXd& _accelerations) = 0;

  /// Get accelerations
  virtual Eigen::VectorXd getAccelerations() const = 0;

  /// Set zero all the accelerations
  virtual void resetAccelerations() = 0;

  /// Set lower limit of acceleration
  virtual void setAccelerationLowerLimit(size_t _index, double _acceleration) = 0;

  /// Get lower limit of acceleration
  virtual double getAccelerationLowerLimit(size_t _index) = 0;

  /// Set upper limit of acceleration
  virtual void setAccelerationUpperLimit(size_t _index, double _acceleration) = 0;

  /// Get upper limit of acceleration
  virtual double getAccelerationUpperLimit(size_t _index) = 0;

  //----------------------------------------------------------------------------
  // Force
  //----------------------------------------------------------------------------

  /// Set a single force
  virtual void setForce(size_t _index, double _force) = 0;

  /// Get a single force
  virtual double getForce(size_t _index) = 0;

  /// Set forces
  virtual void setForces(const Eigen::VectorXd& _forces) = 0;

  /// Get forces
  virtual Eigen::VectorXd getForces() const = 0;

  /// Set zero all the forces
  virtual void resetForces() = 0;

  /// Set lower limit of force
  virtual void setForceLowerLimit(size_t _index, double _force) = 0;

  /// Get lower limit of force
  virtual double getForceLowerLimit(size_t _index) = 0;

  /// Set upper limit of position
  virtual void setForceUpperLimit(size_t _index, double _force) = 0;

  /// Get upper limit of position
  virtual double getForceUpperLimit(size_t _index) = 0;

  //----------------------------------------------------------------------------
  // Velocity change
  //----------------------------------------------------------------------------

  /// Set a single velocity change
  virtual void setVelocityChange(size_t _index, double _velocityChange) = 0;

  /// Get a single velocity change
  virtual double getVelocityChange(size_t _index) = 0;

  /// Set zero all the velocity change
  virtual void resetVelocityChanges() = 0;

  //----------------------------------------------------------------------------
  // Constraint impulse
  //----------------------------------------------------------------------------

  /// Set a single constraint impulse
  virtual void setConstraintImpulse(size_t _index, double _impulse) = 0;

  /// Get a single constraint impulse
  virtual double getConstraintImpulse(size_t _index) = 0;

  /// Set zero all the constraint impulses
  virtual void resetConstraintImpulses() = 0;

  //----------------------------------------------------------------------------
  // Integration
  //----------------------------------------------------------------------------

  /// Integrate positions using Euler method
  virtual void integratePositions(double _dt) = 0;

  /// Integrate velocities using Euler method
  virtual void integrateVelocities(double _dt) = 0;

protected:
  /// Constructor
  GenCoordSystem();

  /// Destructor
  virtual ~GenCoordSystem();
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_GENCOORDSYSTEM_H_
