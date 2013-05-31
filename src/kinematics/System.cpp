/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/14/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include "kinematics/System.h"

namespace kinematics
{

System::System()
{
}

System::~System()
{
}

Dof* System::getDof(int _idx) const
{
    assert(0 <= _idx && _idx < getNumDofs());

    return mDofs[_idx];
}

Dof* System::getDof(const char* const _name) const
{
    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        if (strcmp(mDofs[i]->getName(), _name))
            return mDofs[i];

    return NULL;
}

void System::backupInitState()
{
    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
    {
        mDofs[i]->init_q = mDofs[i]->q;
        mDofs[i]->init_dq = mDofs[i]->dq;
        mDofs[i]->init_ddq = mDofs[i]->ddq;
    }
}

void System::restoreInitState()
{
    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
    {
        mDofs[i]->q = mDofs[i]->init_q;
        mDofs[i]->dq = mDofs[i]->init_dq;
        mDofs[i]->ddq = mDofs[i]->init_ddq;
    }
}

void System::set_q(const Eigen::VectorXd& _q)
{
    assert(_q.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->q = _q[i];
}

void System::set_dq(const Eigen::VectorXd& _dq)
{
    assert(_dq.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->dq = _dq[i];
}

void System::set_ddq(const Eigen::VectorXd& _ddq)
{
    assert(_ddq.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->ddq = _ddq[i];
}

void System::set_tau(const Eigen::VectorXd& _tau)
{
    assert(_tau.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->tau = _tau[i];
}

void System::set_qMin(const Eigen::VectorXd& _qMin)
{
    assert(_qMin.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->qMin = _qMin[i];
}

void System::set_dqMin(const Eigen::VectorXd& _dqMin)
{
    assert(_dqMin.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->dqMin = _dqMin[i];
}

void System::set_ddqMin(const Eigen::VectorXd& _ddqMin)
{
    assert(_ddqMin.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->ddqMin = _ddqMin[i];
}

void System::set_tauMin(const Eigen::VectorXd& _tauMin)
{
    assert(_tauMin.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->tauMin = _tauMin[i];
}


void System::set_qMax(const Eigen::VectorXd& _qMax)
{
    assert(_qMax.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->qMax = _qMax[i];
}

void System::set_dqMax(const Eigen::VectorXd& _dqMax)
{
    assert(_dqMax.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->dqMax = _dqMax[i];
}

void System::set_ddqMax(const Eigen::VectorXd& _ddqMax)
{
    assert(_ddqMax.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->ddqMax = _ddqMax[i];
}

void System::set_tauMax(const Eigen::VectorXd& _tauMax)
{
    assert(_tauMax.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->tauMax = _tauMax[i];
}

void System::set_DqDp(const Eigen::VectorXd& _DqDp)
{
    assert(_DqDp.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->DqDp = _DqDp[i];
}

void System::set_DdqDp(const Eigen::VectorXd& _DdqDp)
{
    assert(_DdqDp.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->DdqDp = _DdqDp[i];
}

void System::set_DddqDp(const Eigen::VectorXd& _DddqDp)
{
    assert(_DddqDp.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->DddqDp = _DddqDp[i];
}

void System::set_DtauDp(const Eigen::VectorXd& _DtauDp)
{
    assert(_DtauDp.size() == getNumDofs());

    int size = getNumDofs();

    for (int i = 0; i < size; ++i)
        mDofs[i]->DtauDp = _DtauDp[i];
}

Eigen::VectorXd System::get_q() const
{
    int size = getNumDofs();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(size);

    for (int i = 0; i < size; ++i)
        q(i) = mDofs[i]->q;

    return q;
}

Eigen::VectorXd System::get_dq() const
{
    int size = getNumDofs();
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(size);

    for (int i = 0; i < size; ++i)
        dq(i) = mDofs[i]->dq;

    return dq;
}

Eigen::VectorXd System::get_ddq() const
{
    int size = getNumDofs();
    Eigen::VectorXd ddq = Eigen::VectorXd::Zero(size);

    for (int i = 0; i < size; ++i)
        ddq(i) = mDofs[i]->ddq;

    return ddq;
}

Eigen::VectorXd System::get_tau() const
{
    int size = getNumDofs();
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(size);

    for (int i = 0; i < size; ++i)
        tau(i) = mDofs[i]->tau;

    return tau;
}

} // namespace kinematics
