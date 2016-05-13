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

#ifndef DART_INTEGRATION_RK4INTEGRATOR_HPP_
#define DART_INTEGRATION_RK4INTEGRATOR_HPP_

#include "dart/integration/Integrator.hpp"

namespace dart {
namespace integration {

/// \brief class RK4Integrator
class RK4Integrator : public Integrator
{
public:
  /// \brief Constructor
  RK4Integrator();

  /// \brief Destructor
  virtual ~RK4Integrator();

  // Documentation inherited
  void integrate(IntegrableSystem* _system, double _dt) override;

private:
  /// \brief Initial configurations
  Eigen::VectorXd q1;

  /// \brief Chache data for generalized velocities
  Eigen::VectorXd dq1, dq2, dq3, dq4;

  /// \brief Chache data for generalized accelerations
  Eigen::VectorXd ddq1, ddq2, ddq3, ddq4;
};

}  // namespace integration
}  // namespace dart

#endif  // DART_INTEGRATION_RK4INTEGRATOR_HPP_
