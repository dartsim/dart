/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#ifndef DART_UTILS_MJCF_DETAIL_OPTION_HPP_
#define DART_UTILS_MJCF_DETAIL_OPTION_HPP_

#include <Eigen/Core>
#include <tinyxml2.h>

#include "dart/utils/mjcf/detail/Error.hpp"
#include "dart/utils/mjcf/detail/Types.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

class Option final
{
public:
  Option() = default;

  double getTimestep() const;
  double getApiRate() const;
  double getImpRatio() const;
  const Eigen::Vector3d& getGravity() const;
  const Eigen::Vector3d& getWind() const;
  const Eigen::Vector3d& getMagnetic() const;
  double getDensity() const;
  double getViscosity() const;
  Integrator getIntegrator() const;
  CollisionType getCollision() const;
  ConeType getCone() const;
  JacobianType getJacobian() const;
  SolverType getSolver() const;
  int getIterations() const;
  double getTolerance() const;
  int getNoSlipIterations() const;
  double getNoSlipTolerance() const;
  int getMprIterations() const;
  double getMprTolerance() const;

private:
  // Private memebers used by MujocoModel class
  friend class MujocoModel;
  Errors read(tinyxml2::XMLElement* element);

private:
  double mTimestep{0.002};
  double mApiRate{100};
  double mImpRatio{1};
  Eigen::Vector3d mGravity{Eigen::Vector3d(0, 0, -9.81)};
  Eigen::Vector3d mWind{Eigen::Vector3d::Zero()};
  Eigen::Vector3d mMagnetic{Eigen::Vector3d(0, -0.5, 0)};
  double mDensity{0};
  double mViscosity{0};
  Integrator mIntegrator{Integrator::EULER};
  CollisionType mCollision{CollisionType::ALL};
  ConeType mCone{ConeType::PYRAMIDAL};
  JacobianType mJacobian{JacobianType::AUTO};
  SolverType mSolver{SolverType::NEWTON};
  int mIterations{100};
  double mTolerance{1e-8};
  int mNoSlipIterations{0};
  double mNoSlipTolerance{1e-6};
  int mMprIterations{50};
  double mMprTolerance{1e-6};
};

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_OPTION_HPP_
