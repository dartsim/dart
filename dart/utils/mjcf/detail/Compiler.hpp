/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#ifndef DART_UTILS_MJCF_DETAIL_COMPILER_HPP_
#define DART_UTILS_MJCF_DETAIL_COMPILER_HPP_

#include <Eigen/Core>
#include <tinyxml2.h>

#include "dart/common/Platform.hpp"
#include "dart/common/ResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/utils/mjcf/detail/Error.hpp"
#include "dart/utils/mjcf/detail/Types.hpp"

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

class Compiler final
{
public:
  Compiler() = default;

  void setBaseUri(const common::Uri& baseUri);
  const common::Uri& getBaseUri() const;
  void setResourceRetriever(const common::ResourceRetrieverPtr& retriever);
  common::ResourceRetrieverPtr getResourceRetriever() const;

  double getBoundMass() const;
  double getBoundInertia() const;
  double getSetTotalMass() const;
  bool getBalanceInertia() const;
  bool getStripPath() const;
  Coordinate getCoordinate() const;
  Angle getAngle() const;
  bool getFitAabb() const;
  const std::string& getEulerSeq() const;
  const std::string& getMeshDir() const;
  const std::string& getTextureDir() const;
  bool getDiscardVisual() const;
  bool getConvexHull() const;
  bool getUserThread() const;
  bool getFuseStatic() const;
  InertiaFromGeom getInertiaFromGeom() const;
  const Eigen::Vector2i& getInertiaGroupRange() const;

private:
  // Private memebers used by MujocoModel class
  friend class MujocoModel;
  Errors read(tinyxml2::XMLElement* element);

private:
  common::Uri mBaseUri;
  common::ResourceRetrieverPtr mRetriever;

  double mBoundMass{0};
  double mBoundInertia{0};
  double mSetTotalMass{-1};
  bool mBalanceInertia{false};
  bool mStripPath{false};
  Coordinate mCoordinate{Coordinate::LOCAL};
  Angle mAngle{Angle::DEGREE};
  bool mFitAabb{false};
  std::string mEulerSeq{"xyz"};
  std::string mMeshDir{""};
  std::string mTextureDir{""};
  bool mDiscardVisual{false};
  bool mConvexHull{true};
  bool mUserThread{true};
  bool mFuseStatic{false};
#if DART_OS_WINDOWS
  InertiaFromGeom mInertiaFromGeom{InertiaFromGeom::IFG_AUTO};
#else
  InertiaFromGeom mInertiaFromGeom{InertiaFromGeom::AUTO};
#endif
  Eigen::Vector2i mInertiaGroupRange{Eigen::Vector2i(0, 5)};
};

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_COMPILER_HPP_
