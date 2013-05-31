/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>,
 *            Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_KINEMATICS_DOF_H
#define DART_KINEMATICS_DOF_H

#include <cstring>
#include <Eigen/Dense>
#include <vector>

#include "utils/Deprecated.h"

namespace kinematics
{

#define MAX_DOF_NAME 128

class Joint;
class Transformation;

class Dof
{
public:
    // TODO: All publics? class vs struct?

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

    //--------------------------------------------------------------------------
    // Initial values
    //--------------------------------------------------------------------------
    double init_q;
    double init_dq;
    double init_ddq;

    //std::vector<double> q_history;

    double get_q() const { return q; }
    double get_dq() const { return dq; }
    double get_ddq() const { return ddq; }
    double get_tau() const { return tau; }

    void set_q(double _q) { q = _q; }
    void set_dq(double _dq) { dq = _dq; }
    void set_ddq(double _ddq) { ddq = _ddq; }
    void set_tau(double _tau) { tau = _tau; }

public:
    /// @brief
    Dof();

    /// @brief
    Dof(double _val);

    /// @brief
    Dof(double _val, double _min, double _max);

    /// @brief
    Dof(double _val, const char *_name);

    /// @brief
    Dof(double _val, const char *_name, double _min, double _max);

    virtual ~Dof() {}

public:
    /// @brief
    void init();

    /// @brief
    void setName(char *_n) { strcpy(mName, _n); }

    /// @brief
    char* getName() { return mName; }

    /// @brief
    void setValue(double _v);

    /// @brief
    double getValue() const { return q; }

    /// @brief
    double getMin() const { return qMin; }

    /// @brief
    double getMax() const { return qMax; }

    /// @brief
    void setMin(double _min) { qMin = _min; }

    /// @brief
    void setMax(double _max) { qMax = _max; }

    /// @brief
    int getSkelIndex() const { return mSkelIndex; }

    /// @brief
    void setSkelIndex(int _idx) { mSkelIndex = _idx; }

    /// @brief
    bool isVariable() const { return mVariable; }

    /// @brief
    void setVariable() { mVariable = true; }

    /// @brief
    void setTrans(Transformation *_t){ mTrans = _t; }

    /// @brief
    Transformation* getTrans() const{ return mTrans; }

    /// @brief
    void setJoint(Joint *_j) { mJoint = _j; }

    /// @brief
    Joint *getJoint() const { return mJoint; }

protected:
    /// @brief
    char mName[MAX_DOF_NAME];

    /// @brief Unique to dof in model.
    int mSkelIndex;

    /// @brief Transformation associated with
    Transformation *mTrans;

    /// @brief Joint to which it belongs.
    Joint *mJoint;

    /// @brief True when it is a variable and included int he model.
    bool mVariable;
};

} // namespace kinematics

#endif // #ifndef DART_KINEMATICS_DOF_H

