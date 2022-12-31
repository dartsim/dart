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

#ifndef DART_UTILS_MJCF_DETAIL_BODY_HPP_
#define DART_UTILS_MJCF_DETAIL_BODY_HPP_

#include "dart/utils/mjcf/detail/BodyAttributes.hpp"
#include "dart/utils/mjcf/detail/Compiler.hpp"
#include "dart/utils/mjcf/detail/Error.hpp"
#include "dart/utils/mjcf/detail/Geom.hpp"
#include "dart/utils/mjcf/detail/Inertial.hpp"
#include "dart/utils/mjcf/detail/Joint.hpp"
#include "dart/utils/mjcf/detail/Site.hpp"

#include <tinyxml2.h>

namespace dart {
namespace utils {
namespace MjcfParser {
namespace detail {

class Size;

class Body final
{
public:
  /// Default constructor.
  Body() = default;

  /// \{ \name Attributes

  /// Returns 'name' attribute.
  const std::string& getName() const;

  /// Returns 'mocap' attribute.
  bool getMocap() const;

  /// Sets the transform relative to the parent <body> or <worldbody>.
  void setRelativeTransform(const Eigen::Isometry3d& tf);

  /// Returns the transform relative to the parent <body> or <worldbody>.
  const Eigen::Isometry3d& getRelativeTransform() const;

  /// Sets the world transform of this <body>.
  void setWorldTransform(const Eigen::Isometry3d& tf);

  /// Returns the world transform of this <body>.
  const Eigen::Isometry3d& getWorldTransform() const;

  /// \}

  /// \{ \name Child elements

  /// Returns <inertial> element.
  ///
  /// When <inertial> element is not specified in the XML file, then it's
  /// inferred from <geom> elements. If both of <inertial> and <inertial> are
  /// missing, then the parser must be failed to parse the file.
  const Inertial& getInertial() const;

  /// Returns the number of <joint> elements.
  std::size_t getNumJoints() const;

  /// Returns a <joint> element at \c index.
  const Joint& getJoint(std::size_t index) const;

  /// Returns the number of child <body> elements.
  std::size_t getNumChildBodies() const;

  /// Returns a child <body> element at \c index.
  const Body& getChildBody(std::size_t index) const;

  /// Returns the number of <geom> elements.
  std::size_t getNumGeoms() const;

  /// Returns a <geom> element at \c index.
  const Geom& getGeom(std::size_t index) const;

  /// Returns the number of <site> elements.
  std::size_t getNumSites() const;

  /// Returns a <site> element at \c index.
  const Site& getSite(std::size_t index) const;

  /// \}

private:
  // Private members used by Worldbody class
  friend class Worldbody;
  Errors read(
      tinyxml2::XMLElement* element,
      const std::optional<Size>& size,
      const Defaults& defaults,
      const Default* currentDefault);

  /// Updates attributes and elements that doesn't require any other elements.
  Errors preprocess(const Compiler& compiler);

  /// Updates attributes and elements that require the preprocessed child
  /// elements of this <body>.
  Errors compile(const Compiler& compiler);

  /// Updates attributes and elements that require the compiled parent element.
  Errors postprocess(const Body* body, const Compiler& compiler);

private:
  Inertial computeInertialFromGeoms(
      const std::vector<Geom>& geoms, const Compiler& compiler);

  BodyAttributes mAttributes;

  /// Name of the body.
  std::string mName{""};

  /// If this attribute is "true", the body is labeled as a mocap body.
  bool mMocap{false};

  Eigen::Isometry3d mRelativeTransform{Eigen::Isometry3d::Identity()};

  Eigen::Isometry3d mWorldTransform{Eigen::Isometry3d::Identity()};

  Eigen::VectorXd mUser;

  Inertial mInertial;

  std::vector<Joint> mJoints;
  std::vector<Body> mChildBodies;
  std::vector<Geom> mGeoms;
  std::vector<Site> mSites;
};

} // namespace detail
} // namespace MjcfParser
} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_MJCF_DETAIL_BODY_HPP_
