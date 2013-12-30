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

#ifndef DART_INTEGRATION_INTEGRATOR_H_
#define DART_INTEGRATION_INTEGRATOR_H_

#include <vector>

#include <Eigen/Dense>

namespace dart {
namespace integration {

/// \brief Any class that uses an integrator should implement this interface.
class IntegrableSystem {
public:
  /// \brief Default constructor.
  IntegrableSystem();

  /// \brief Default destructor.
  virtual ~IntegrableSystem();

public:
  /// \brief Get state of the system.
  virtual Eigen::VectorXd getState() const = 0;

  /// \brief Set state of the system.
  virtual void setState(const Eigen::VectorXd& _state) = 0;

  /// \brief Evaluate the derivatives of the system.
  virtual Eigen::VectorXd evalDeriv() = 0;
};

// TODO(kasiu): Consider templating the class (which currently only works on
// arbitrarily-sized vectors of doubles)
/// \brief
class Integrator {
public:
  /// \brief Default constructor.
  Integrator();

  /// \brief Default destructor.
  virtual ~Integrator();

public:
  /// \brief Integrate the system with time step dt.
  virtual void integrate(IntegrableSystem* system, double dt) const = 0;
};

}  // namespace integration
}  // namespace dart

#endif  // DART_INTEGRATION_INTEGRATOR_H_
