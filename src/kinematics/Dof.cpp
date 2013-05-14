/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#include "Dof.h"
#include "Transformation.h"

double inf = 1e9;

namespace kinematics {

Dof::Dof()
    : q(0.0),
      qMin(-inf),
      qMax(inf),
      mSkelIndex(-1),
      mVariable(false),
      mTrans(NULL),
      mJoint(NULL) {
    strcpy(mName, "dof");
}

Dof::Dof(double _v)
    : q(_v),
      qMin(-inf),
      qMax(inf),
      mSkelIndex(-1),
      mVariable(false),
      mTrans(NULL),
      mJoint(NULL) {
    strcpy(mName, "dof");
}

Dof::Dof(double _v, const char *_name)
    : q(_v),
      qMin(-inf),
      qMax(inf),
      mSkelIndex(-1),
      mVariable(false),
      mTrans(NULL),
      mJoint(NULL) {
    strcpy(mName, _name);
}

Dof::Dof(double _v, double _min, double _max)
    : q(_v),
      qMin(_min),
      qMax(_max),
      mSkelIndex(-1),
      mVariable(false),
      mTrans(NULL),
      mJoint(NULL) {
    strcpy(mName, "dof");
}

Dof::Dof(double _v, const char *_name, double _min, double _max)
    : q(_v),
      qMin(_min),
      qMax(_max),
      mSkelIndex(-1),
      mVariable(false),
      mTrans(NULL),
      mJoint(NULL) {
    strcpy(mName, _name);
}

void Dof::setValue(double _v) {
    q = _v;
    if (mTrans != NULL)
        mTrans->setDirty();
}

void Dof::init() {
    strcpy(mName, "dof");

    q = dq = ddq = tau = 0.0;
    qMin = dqMin = ddqMin = tauMin = -inf;
    qMax = dqMax = ddqMax = tauMax = inf;
    DqDp = DdqDp = DddqDp = DtauDp = 0.0;

    mSkelIndex = -1;
    mVariable = false;
    mTrans = NULL;
    mJoint = NULL;	// remains null if const dof
}

} // namespace kinematics

