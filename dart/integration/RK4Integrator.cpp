/*
 * Copyright (c) 2011-2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Kristin Siu <kasiu@gatech.edu>
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

#include "dart/integration/RK4Integrator.h"

#include "dart/math/MathTypes.h"

namespace dart {
namespace integration {

RK4Integrator::RK4Integrator()
{
}

RK4Integrator::~RK4Integrator()
{
}

void RK4Integrator::integrate(IntegrableSystem* _system, double _dt) const
{
//  // TODO(kasiu): Slow. Needs to be optimized.
//  Eigen::VectorXd  q = _system->getConfigs();
//  Eigen::VectorXd dq = _system->getGenVels();

//  _system->evalAccs();
////   dq1 = _system->getGenVels();
////  ddq1 = _system->getAccs();

//  _system->integrateConfigs(0.5 * _dt);
//  _system->integrateGenVels(0.5 * _dt);

//   dq2 = _system->getGenVels();
//  ddq2 = _system->evalAccs();

//  _system->setConfigs( q);
//  _system->setGenVels(dq);
//  _system->integrateConfigs( dq2, 0.5 * _dt);
//  _system->integrateGenVels(ddq2, 0.5 * _dt);

//   dq3 = _system->getGenVels();
//  ddq3 = _system->evalAccs();

//  _system->setConfigs( q);
//  _system->setGenVels(dq);
//  _system->integrateConfigs( dq3, _dt);
//  _system->integrateGenVels(ddq3, _dt);

//   dq4 = _system->getGenVels();
//  ddq4 = _system->evalAccs();

//  _system->setConfigs( q);
//  _system->setGenVels(dq);
//  _system->integrateConfigs(
//        DART_1_6 * ( dq1 + (2.0 *  dq2) + (2.0 *  dq3) +  dq4), _dt);
//  _system->integrateGenVels(
//        DART_1_6 * (ddq1 + (2.0 * ddq2) + (2.0 * ddq3) + ddq4), _dt);
}

}  // namespace integration
}  // namespace dart
