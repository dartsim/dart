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

/// \brief Base class for generalized coordinate systems
class GenCoordSystem
{
public:
  /// \brief Get number of generalized coordinates
  size_t getNumGenCoords() const;

  /// \brief Get generalized coordinate whose index is _idx
  GenCoord* getGenCoord(size_t _idx) const;

  /// \brief Get gneralized coordinate whose name is _name
  GenCoord* getGenCoord(const std::string& _name) const;

  //--------------------------- Configurations ---------------------------------
  /// \brief Set configurations defined in terms of generalized coordinates
  virtual void setConfigs(const Eigen::VectorXd& _configs);

  /// \brief Get configurations defined in terms of generalized coordinates
  virtual Eigen::VectorXd getConfigs() const;

  /// \brief Set lower bounds for configurations
  virtual void setConfigsMin(const Eigen::VectorXd& _configsMin);

  /// \brief Get lower bounds for configurations
  virtual Eigen::VectorXd getConfigsMin() const;

  /// \brief Set upper bounds for configurations
  virtual void setConfigsMax(const Eigen::VectorXd& _configsMax);

  /// \brief Get uppoer bounds for configurations
  virtual Eigen::VectorXd getConfigsMax() const;

  //----------------------------- Velocities -----------------------------------
  /// \brief Set generalized velocities
  virtual void setGenVels(const Eigen::VectorXd& _vels);

  /// \brief Get generalized velocities
  virtual Eigen::VectorXd getGenVels() const;

  /// \brief Set lower bounds for generalized velocities
  virtual void setGenVelsMin(const Eigen::VectorXd& _velsMin);

  /// \brief Get lower bounds for generalized velocities
  virtual Eigen::VectorXd getGenVelsMin() const;

  /// \brief Set upper bounds for generalized velocities
  virtual void setGenVelsMax(const Eigen::VectorXd& _velsMax);

  /// \brief Get uppoer bounds for generalized velocities
  virtual Eigen::VectorXd getGenVelsMax() const;

  //---------------------------- Accelerations ---------------------------------
  /// \brief Set generalized accelerations
  virtual void setGenAccs(const Eigen::VectorXd& _accs);

  /// \brief Get generalized accelerations
  virtual Eigen::VectorXd getGenAccs() const;

  /// \brief Set lower bounds for generalized accelerations
  virtual void setGenAccsMin(const Eigen::VectorXd& _accsMin);

  /// \brief Get lower bounds for generalized accelerations
  virtual Eigen::VectorXd getGenAccsMin() const;

  /// \brief Set upper bounds for generalized accelerations
  virtual void setGenAccsMax(const Eigen::VectorXd& _accsMax);

  /// \brief Get uppoer bounds for generalized accelerations
  virtual Eigen::VectorXd getGenAccsMax() const;

  //------------------------------- Forces -------------------------------------
  /// \brief Set generalized forces
  virtual void setGenForces(const Eigen::VectorXd& _forces);

  /// \brief Get generalized forces
  virtual Eigen::VectorXd getGenForces() const;

  /// \brief Set lower bounds for generalized forces
  virtual void setGenForcesMin(const Eigen::VectorXd& _forcesMin);

  /// \brief Get lower bounds for generalized forces
  virtual Eigen::VectorXd getGenForcesMin() const;

  /// \brief Set upper bounds for generalized forces
  virtual void setGenForcesMax(const Eigen::VectorXd& _forcesMax);

  /// \brief Get uppoer bounds for generalized forces
  virtual Eigen::VectorXd getGenForcesMax() const;

  //------------------------------- Impulse ------------------------------------
  /// \brief Set velocity change
  virtual void setVelsChange(const Eigen::VectorXd& _velsChange);

  /// \brief Get velocity change
  virtual Eigen::VectorXd getVelsChange() const;

//  /// \brief Set impulses
//  virtual void setImpulses(const Eigen::VectorXd& _impulses);

//  /// \brief Get impulses
//  virtual Eigen::VectorXd getImpulses() const;

  /// \brief Set generalized constraint impulses
  void setConstraintImpulses(const Eigen::VectorXd& _constImps);

  /// \brief Get generalized constraint impulses
  Eigen::VectorXd getConstraintImpulses() const;

  //----------------------------- Integration ----------------------------------
  /// \brief Integrate configurations with timestep _dt
  virtual void integrateConfigs(double _dt);

  /// \brief Integrate generalized velocities with timespte _dt
  virtual void integrateGenVels(double _dt);

protected:
  /// \brief Array of pointers to generalized coordinates
  std::vector<GenCoord*> mGenCoords;

protected:
  /// \brief Constructor
  GenCoordSystem();

  /// \brief Destructor
  virtual ~GenCoordSystem();
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_GENCOORDSYSTEM_H_
