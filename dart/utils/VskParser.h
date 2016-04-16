/*
 * Copyright (c) 2011-2015, Georgia Tech Research Corporation
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

#ifndef DART_UTILS_VSKPARSER_H_
#define DART_UTILS_VSKPARSER_H_

#include "dart/common/ResourceRetriever.h"
#include "dart/common/Uri.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {
namespace utils {

namespace VskParser
{
  /// Options struct is additional information that helps building a skeleton
  /// that can be used in kinematics or dynamics simulation. VSK file format
  /// itself doesn't provide essential properties for it such as body's shape,
  /// mass, and inertia.
  struct Options
  {
    /// Resource retriever. LocalResourceRetriever is used if it's nullptr.
    common::ResourceRetrieverPtr retrieverOrNullptr;

    /// The default shape for body node is ellipsoid. The size of ellipsoid of
    /// each body node are determined by the relative transformation from a body
    /// node and its child body node. defaultEllipsoidSize is used for body
    /// nodes that don't have child body node.
    Eigen::Vector3d defaultEllipsoidSize;

    /// Ratio of shorter radii of each ellipsoid to the longest radius where
    /// the longest radius is the distance between a body and its child body
    /// node.
    double thicknessRatio;

    /// Density of each ellipsoid that are used to compute mass.
    double density;

    /// Lower limit of joint position
    double jointPositionLowerLimit;

    /// Upper limit of joint position
    double jointPositionUpperLimit;

    /// Joint damping coefficient
    double jointDampingCoefficient;

    /// Joint Coulomb friction
    double jointFriction;

    /// Remove end BodyNodes with no Shape segment
    bool removeEndBodyNodes;

    /// Constructor
    Options(const common::ResourceRetrieverPtr& retrieverOrNullptr = nullptr,
            const Eigen::Vector3d& defaultEllipsoidSize
                = Eigen::Vector3d::Constant(0.05),
            double thicknessRatio = 0.35,
            double density = 1e+3,
            double jointPositionLowerLimit = -math::constantsd::pi(),
            double jointPositionUpperLimit = +math::constantsd::pi(),
            double jointDampingCoefficient = 0.1,
            double jointFriction = 0.0,
            bool removeEndBodyNodes = false);
  };

  /// Read Skeleton from VSK file
  dynamics::SkeletonPtr readSkeleton(const common::Uri& fileUri,
                                     Options options = Options());

} // namespace VskParser

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_VSKPARSER_H_

