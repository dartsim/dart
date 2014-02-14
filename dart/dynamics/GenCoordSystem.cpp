/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#include "dart/dynamics/GenCoordSystem.h"

#include <string>

namespace dart {
namespace dynamics {

GenCoordSystem::GenCoordSystem() {
}

GenCoordSystem::~GenCoordSystem() {
}

GenCoord* GenCoordSystem::getGenCoord(int _idx) const {
  assert(0 <= _idx && _idx < getNumGenCoords());
  return mGenCoords[_idx];
}

GenCoord* GenCoordSystem::getGenCoord(const std::string& _name) const {
  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    if (mGenCoords[i]->getName() == _name)
      return mGenCoords[i];

  return NULL;
}

void GenCoordSystem::set_q(const Eigen::VectorXd& _q) {
  assert(_q.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_q(_q[i]);
}

void GenCoordSystem::set_dq(const Eigen::VectorXd& _dq) {
  assert(_dq.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_dq(_dq[i]);
}

void GenCoordSystem::set_ddq(const Eigen::VectorXd& _ddq) {
  assert(_ddq.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_ddq(_ddq[i]);
}

void GenCoordSystem::set_tau(const Eigen::VectorXd& _tau) {
  assert(_tau.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_tau(_tau[i]);
}

void GenCoordSystem::set_qMin(const Eigen::VectorXd& _qMin) {
  assert(_qMin.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_qMin(_qMin[i]);
}

void GenCoordSystem::set_dqMin(const Eigen::VectorXd& _dqMin) {
  assert(_dqMin.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_dqMin(_dqMin[i]);
}

void GenCoordSystem::set_ddqMin(const Eigen::VectorXd& _ddqMin) {
  assert(_ddqMin.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_ddqMin(_ddqMin[i]);
}

void GenCoordSystem::set_tauMin(const Eigen::VectorXd& _tauMin) {
  assert(_tauMin.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_tauMin(_tauMin[i]);
}


void GenCoordSystem::set_qMax(const Eigen::VectorXd& _qMax) {
  assert(_qMax.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_qMax(_qMax[i]);
}

void GenCoordSystem::set_dqMax(const Eigen::VectorXd& _dqMax) {
  assert(_dqMax.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_dqMax(_dqMax[i]);
}

void GenCoordSystem::set_ddqMax(const Eigen::VectorXd& _ddqMax) {
  assert(_ddqMax.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_ddqMax(_ddqMax[i]);
}

void GenCoordSystem::set_tauMax(const Eigen::VectorXd& _tauMax) {
  assert(_tauMax.size() == getNumGenCoords());

  int size = getNumGenCoords();

  for (int i = 0; i < size; ++i)
    mGenCoords[i]->set_tauMax(_tauMax[i]);
}

Eigen::VectorXd GenCoordSystem::get_q() const {
  int size = getNumGenCoords();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    q(i) = mGenCoords[i]->get_q();
  }

  return q;
}

Eigen::VectorXd GenCoordSystem::get_dq() const {
  int size = getNumGenCoords();
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    dq(i) = mGenCoords[i]->get_dq();
  }

  return dq;
}

Eigen::VectorXd GenCoordSystem::get_ddq() const {
  int size = getNumGenCoords();
  Eigen::VectorXd ddq = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    ddq(i) = mGenCoords[i]->get_ddq();
  }

  return ddq;
}

Eigen::VectorXd GenCoordSystem::get_tau() const {
  int size = getNumGenCoords();
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    tau(i) = mGenCoords[i]->get_tau();
  }

  return tau;
}

Eigen::VectorXd GenCoordSystem::get_qMax() const {
  int size = getNumGenCoords();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    q(i) = mGenCoords[i]->get_qMax();
  }

  return q;
}

Eigen::VectorXd GenCoordSystem::get_dqMax() const {
  int size = getNumGenCoords();
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    dq(i) = mGenCoords[i]->get_dqMax();
  }

  return dq;
}

Eigen::VectorXd GenCoordSystem::get_ddqMax() const {
  int size = getNumGenCoords();
  Eigen::VectorXd ddq = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    ddq(i) = mGenCoords[i]->get_ddqMax();
  }

  return ddq;
}

Eigen::VectorXd GenCoordSystem::get_tauMax() const {
  int size = getNumGenCoords();
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    tau(i) = mGenCoords[i]->get_tauMax();
  }

  return tau;
}

Eigen::VectorXd GenCoordSystem::get_qMin() const {
  int size = getNumGenCoords();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    q(i) = mGenCoords[i]->get_qMin();
  }

  return q;
}

Eigen::VectorXd GenCoordSystem::get_dqMin() const {
  int size = getNumGenCoords();
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    dq(i) = mGenCoords[i]->get_dqMin();
  }

  return dq;
}

Eigen::VectorXd GenCoordSystem::get_ddqMin() const {
  int size = getNumGenCoords();
  Eigen::VectorXd ddq = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    ddq(i) = mGenCoords[i]->get_ddqMin();
  }

  return ddq;
}

Eigen::VectorXd GenCoordSystem::get_tauMin() const {
  int size = getNumGenCoords();
  Eigen::VectorXd tau = Eigen::VectorXd::Zero(size);

  for (int i = 0; i < size; ++i) {
    tau(i) = mGenCoords[i]->get_tauMin();
  }

  return tau;
}

int GenCoordSystem::getNumGenCoords() const {
  return mGenCoords.size();
}

}  // namespace dynamics
}  // namespace dart
