/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#ifndef DART_UTILS_MJCF_DETAIL_WELD_HPP_
#define DART_UTILS_MJCF_DETAIL_WELD_HPP_

#include <dart/io/mjcf/detail/Default.hpp>
#include <dart/io/mjcf/detail/Error.hpp>
#include <dart/io/mjcf/detail/WeldAttributes.hpp>

#include <tinyxml2.h>

namespace dart {
namespace io {
namespace MjcfParser {
namespace detail {

class Weld final
{
public:
  Weld() = default;

  const std::string& getName() const;
  bool getActive() const;
  const Eigen::Vector2d& getSolRef() const;
  const Eigen::Matrix<double, 5, 1>& getSolImp() const;
  const std::string& getBody1() const;
  const std::string& getBody2() const;
  const std::optional<Eigen::Isometry3d>& getRelativeTransform() const;

private:
  // Private memebers used by MujocoModel class
  friend class Equality;
  Errors read(tinyxml2::XMLElement* element, const Defaults& defaults);

private:
  WeldAttributes mAttributes;

  std::string mName;
  bool mActive{true};
  Eigen::Vector2d mSolRef;
  Eigen::Matrix<double, 5, 1> mSolImp;
  std::string mBody1;
  std::string mBody2;
  bool mUsePredefinedRelativeTransform{true};
  // Relative transform from body1 to body2
  std::optional<Eigen::Isometry3d> mRelativeTransfrom;
};

} // namespace detail
} // namespace MjcfParser
} // namespace io
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_WELD_HPP_
