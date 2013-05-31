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

#ifndef DART_KINEMATICS_SYSTEM_H
#define DART_KINEMATICS_SYSTEM_H

#include <vector>
#include "kinematics/Dof.h"

namespace kinematics
{

/// @brief System is a base class for every classes that has Dofs.
class System
{
public:
    /// @brief
    System();

    /// @brief
    virtual ~System();

    /// @brief
    int getNumDofs() const { return mDofs.size(); }

    /// @brief
    const std::vector<Dof*>& getDofs() const { return mDofs; }

    /// @brief
    Dof* getDof(int _idx) const;

    /// @brief
    Dof* getDof(const char* const _name) const;

    /// @brief Backup current state as initial state.
    void backupInitState();

    /// @brief Restore the stored initial state.
    void restoreInitState();

    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    void set_q(const Eigen::VectorXd& _q);
    void set_dq(const Eigen::VectorXd& _dq);
    void set_ddq(const Eigen::VectorXd& _ddq);
    void set_tau(const Eigen::VectorXd& _tau);

    void set_qMin(const Eigen::VectorXd& _qMin);
    void set_dqMin(const Eigen::VectorXd& _dqMin);
    void set_ddqMin(const Eigen::VectorXd& _ddqMin);
    void set_tauMin(const Eigen::VectorXd& _tauMin);

    void set_qMax(const Eigen::VectorXd& _qMax);
    void set_dqMax(const Eigen::VectorXd& _dqMax);
    void set_ddqMax(const Eigen::VectorXd& _ddqMax);
    void set_tauMax(const Eigen::VectorXd& _tauMax);

    void set_DqDp(const Eigen::VectorXd& _DqDp);
    void set_DdqDp(const Eigen::VectorXd& _DdqDp);
    void set_DddqDp(const Eigen::VectorXd& _DddqDp);
    void set_DtauDp(const Eigen::VectorXd& _DtauDp);


    //--------------------------------------------------------------------------
    //
    //--------------------------------------------------------------------------
    Eigen::VectorXd get_q() const;
    Eigen::VectorXd get_dq() const;
    Eigen::VectorXd get_ddq() const;
    Eigen::VectorXd get_tau() const;

    Eigen::VectorXd get_qMin() const;
    Eigen::VectorXd get_dqMin() const;
    Eigen::VectorXd get_ddqMin() const;
    Eigen::VectorXd get_tauMin() const;

    Eigen::VectorXd get_qMax() const;
    Eigen::VectorXd get_dqMax() const;
    Eigen::VectorXd get_ddqMax() const;
    Eigen::VectorXd get_tauMax() const;

    Eigen::VectorXd get_DqDp() const;
    Eigen::VectorXd get_DdqDp() const;
    Eigen::VectorXd get_DddqDp() const;
    Eigen::VectorXd get_DtauDp() const;

protected:
    /// @brief Pointers to Dofs.
    std::vector<Dof*> mDofs;

private:

};

} // namespace kinematics

#endif // DART_KINEMATICS_SYSTEM_H
