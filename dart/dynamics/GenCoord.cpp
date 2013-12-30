/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
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

#include <limits>
#include <string>

#include "dart/dynamics/GenCoord.h"

namespace dart {
namespace dynamics {

GenCoord::GenCoord()
  : q(0.0),
    dq(0.0),
    ddq(0.0),
    tau(0.0),
    qMin(-std::numeric_limits<double>::infinity()),
    dqMin(-std::numeric_limits<double>::infinity()),
    ddqMin(-std::numeric_limits<double>::infinity()),
    tauMin(-std::numeric_limits<double>::infinity()),
    qMax(std::numeric_limits<double>::infinity()),
    dqMax(std::numeric_limits<double>::infinity()),
    ddqMax(std::numeric_limits<double>::infinity()),
    tauMax(std::numeric_limits<double>::infinity()),
    DqDp(0.0),
    DdqDp(0.0),
    DddqDp(0.0),
    DtauDp(0.0),
    //      mSkelIndex(-1),
    //      mJoint(NULL),
    mName("dof") {
}

GenCoord::~GenCoord() {
}

void GenCoord::setName(const std::string& _name) {
  mName = _name;
}

const std::string& GenCoord::getName() const {
  return mName;
}

int GenCoord::getSkeletonIndex() const {
  return mSkelIndex;
}

void GenCoord::setSkeletonIndex(int _idx) {
  mSkelIndex = _idx;
}

}  // namespace dynamics
}  // namespace dart
