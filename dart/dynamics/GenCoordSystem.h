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

#include <vector>
#include <string>

#include <Eigen/Dense>

#include "dart/dynamics/GenCoord.h"

namespace dart {
namespace dynamics {

/// \brief System is a base class for every classes that has Dofs.
class GenCoordSystem
{
public:
  /// \brief Constructor
  GenCoordSystem();

  /// \brief Destructor
  virtual ~GenCoordSystem();

  /// \brief Get number of generalized coordinates in this system
  int getNumGenCoords() const;

  /// \brief Get generalized coordinate whose index is _idx
  GenCoord* getGenCoord(int _idx) const;

  /// \brief Get gneralized coordinate whose name is _name
  GenCoord* getGenCoord(const std::string& _name) const;

  /// \brief Set generalized coordinate vector
  void set_q(const Eigen::VectorXd& _q);

  /// \brief Set generalized velocity vector
  void set_dq(const Eigen::VectorXd& _dq);

  /// \brief Set generalized acceleration vector
  void set_ddq(const Eigen::VectorXd& _ddq);

  /// \brief Set generalized force vector (internal forces)
  void set_tau(const Eigen::VectorXd& _tau);

  /// \brief
  void setGenPosMin(const Eigen::VectorXd& _qMin);

  /// \brief
  void set_dqMin(const Eigen::VectorXd& _dqMin);

  /// \brief
  void set_ddqMin(const Eigen::VectorXd& _ddqMin);

  /// \brief
  void set_tauMin(const Eigen::VectorXd& _tauMin);

  /// \brief
  void setGenPosMax(const Eigen::VectorXd& _qMax);

  /// \brief
  void set_dqMax(const Eigen::VectorXd& _dqMax);

  /// \brief
  void set_ddqMax(const Eigen::VectorXd& _ddqMax);

  /// \brief
  void set_tauMax(const Eigen::VectorXd& _tauMax);

  /// \brief
  Eigen::VectorXd get_q() const;

  /// \brief
  Eigen::VectorXd get_dq() const;

  /// \brief
  Eigen::VectorXd get_ddq() const;

  /// \brief
  Eigen::VectorXd get_tau() const;

  /// \brief
  Eigen::VectorXd get_qMin() const;

  /// \brief
  Eigen::VectorXd get_dqMin() const;

  /// \brief
  Eigen::VectorXd get_ddqMin() const;

  /// \brief
  Eigen::VectorXd get_tauMin() const;

  /// \brief
  Eigen::VectorXd get_qMax() const;

  /// \brief
  Eigen::VectorXd get_dqMax() const;

  /// \brief
  Eigen::VectorXd get_ddqMax() const;

  /// \brief
  Eigen::VectorXd get_tauMax() const;

protected:
  /// \brief Pointers to Dofs.
  std::vector<GenCoord*> mGenCoords;
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_GENCOORDSYSTEM_H_
