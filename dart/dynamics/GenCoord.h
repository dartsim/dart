/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
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

#include <cassert>
#include <string>

#include "dart/math/Geometry.h"

namespace dart {
namespace dynamics {

class Joint;

/// \brief Generalized coordinate.
/// A set of generalized coordiante describes the configuration of a system.
class GenCoord {
public:
  /// \brief
  GenCoord();

  /// \brief
  virtual ~GenCoord();

  /// \brief
  void setName(const std::string& _name);

  /// \brief
  const std::string& getName() const;

  /// \brief
  int getSkeletonIndex() const;

  /// \brief
  void setSkeletonIndex(int _idx);

public:
  double get_q() const { return q; }            ///< Configuration
  double get_dq() const { return dq; }          ///< Velocity
  double get_ddq() const { return ddq; }        ///< Acceleration
  double get_tau() const { return tau; }        ///< torque or force

  double get_qMin() const { return qMin; }      ///< Minimum value of q
  double get_dqMin() const { return dqMin; }    ///< Minimum value of dq
  double get_ddqMin() const { return ddqMin; }  ///< Minimum value of ddq
  double get_tauMin() const { return tauMin; }  ///< Minimum value of tau

  double get_qMax() const { return qMax; }      ///< Maximum value of q
  double get_dqMax() const { return dqMax; }    ///< Maximum value of dq
  double get_ddqMax() const { return ddqMax; }  ///< Maximum value of ddq
  double get_tauMax() const { return tauMax; }  ///< Maximum value of tau

  void set_q(double _q) { assert(_q == _q); q = _q; }
  void set_dq(double _dq) { assert(_dq == _dq); dq = _dq; }
  void set_ddq(double _ddq) { assert(_ddq == _ddq); ddq = _ddq; }
  void set_tau(double _tau) { assert(_tau == _tau); tau = _tau; }

  void set_qMin(double _qMin) { qMin = _qMin; }
  void set_dqMin(double _dqMin) { dqMin = _dqMin; }
  void set_ddqMin(double _ddqMin) { ddqMin = _ddqMin; }
  void set_tauMin(double _tauMin) { tauMin = _tauMin; }

  void set_qMax(double _qMax) { qMax = _qMax; }
  void set_dqMax(double _dqMax) { dqMax = _dqMax; }
  void set_ddqMax(double _ddqMax) { ddqMax = _ddqMax; }
  void set_tauMax(double _tauMax) { tauMax = _tauMax; }

protected:
  /// \brief
  std::string mName;

  /// \brief Unique to dof in model.
  int mSkelIndex;

  /// \brief Joint to which it belongs.
  // Joint *mJoint;

  //--------------------------------------------------------------------------
  // Position
  //--------------------------------------------------------------------------
  double q;       ///< Position
  double qMin;    ///< Min value allowed.
  double qMax;    ///< Max value allowed.
  double DqDp;    ///< derivatives w.r.t. an arbitrary scalr variable p

  //--------------------------------------------------------------------------
  // Velocity
  //--------------------------------------------------------------------------
  double dq;       ///< Velocity
  double dqMin;    ///< Min value allowed.
  double dqMax;    ///< Max value allowed.
  double DdqDp;    ///< derivatives w.r.t. an arbitrary scalr variable p

  //--------------------------------------------------------------------------
  // Force (torque)
  //--------------------------------------------------------------------------
  double ddq;       ///< Acceleration
  double ddqMin;    ///< Min value allowed.
  double ddqMax;    ///< Max value allowed.
  double DddqDp;    ///< derivatives w.r.t. an arbitrary scalr variable p

  //--------------------------------------------------------------------------
  // Force (torque)
  //--------------------------------------------------------------------------
  double tau;       ///< Force (torque)
  double tauMin;    ///< Min value allowed.
  double tauMax;    ///< Max value allowed.
  double DtauDp;    ///< derivatives w.r.t. an arbitrary scalr variable p
};

}  // namespace dynamics
}  // namespace dart

#endif  // DART_DYNAMICS_GENCOORD_H_
