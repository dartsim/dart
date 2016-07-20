/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/integration/RK4Integrator.hpp"

namespace dart {
namespace integration {

//==============================================================================
RK4Integrator::RK4Integrator()
{
}

//==============================================================================
RK4Integrator::~RK4Integrator()
{
}

//==============================================================================
void RK4Integrator::integrate(IntegrableSystem* _system, double _dt)
{
  //----------------------------------------------------------------------------
  // compute ddq1
  q1   = _system->getConfigs();
  dq1  = _system->getGenVels();
  ddq1 = _system->evalGenAccs();

  //----------------------------------------------------------------------------
  // q2 = q1 + dq1 * 0.5 * _dt
  _system->integrateConfigs(dq1, 0.5 * _dt);

  // q2 = dq1 + ddq1 * 0.5 * _dt
  _system->integrateGenVels(ddq1, 0.5 * _dt);

  // compute ddq2
  dq2  = _system->getGenVels();
  ddq2 = _system->evalGenAccs();

  //----------------------------------------------------------------------------
  // q3 = q1 + dq2 * 0.5 * _dt
  _system->setConfigs(q1);
  _system->integrateConfigs(dq2, 0.5 * _dt);

  // dq3 = dq1 + ddq2 * 0.5 * _dt
  _system->setGenVels(dq1);
  _system->integrateGenVels(ddq2, 0.5 * _dt);

  // compute ddq3
  dq3  = _system->getGenVels();
  ddq3 = _system->evalGenAccs();

  //----------------------------------------------------------------------------
  // q4 = q1 + dq3 * _dt
  _system->integrateConfigs(dq3, _dt);

  // dq4 = dq1 + ddq3 * _dt
  _system->setGenVels(dq1);
  _system->integrateGenVels(ddq3, _dt);

  // compute ddq4
  dq4  = _system->getGenVels();
  ddq4 = _system->evalGenAccs();

  //----------------------------------------------------------------------------
  // q = q1 + dq5 * _dt
  //   where dq5 = (1/6) * (dq1 + (2.0 * dq2) + (2.0 * dq3) + dq4)
  _system->setConfigs(q1);
  _system->integrateConfigs(
        1.0 / 6.0 * (dq1 + (2.0 * dq2) + (2.0 * dq3) + dq4), _dt);

  // dq = dq1 + ddq5 * _dt
  //   where dq5 = (1/6) * (ddq1 + (2.0 * ddq2) + (2.0 * ddq3) + ddq4)
  _system->setGenVels(dq1);
  _system->integrateGenVels(
        1.0 / 6.0 * (ddq1 + (2.0 * ddq2) + (2.0 * ddq3) + ddq4), _dt);
}

}  // namespace integration
}  // namespace dart
