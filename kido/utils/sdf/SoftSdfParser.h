/*
 * Copyright (c) 2014-2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef KIDO_UTILS_SDF_SOFTSDFPARSER_H_
#define KIDO_UTILS_SDF_SOFTSDFPARSER_H_

#include <map>
#include <string>
#include <Eigen/Dense>
#include <Eigen/StdVector>
// TinyXML-2 Library
// http://www.grinninglizard.com/tinyxml2/index.html
#include <tinyxml2.h>

#include "kido/utils/sdf/SdfParser.h"

namespace kido {
namespace dynamics {
class Joint;
class SoftBodyNode;
class Skeleton;
}  // namespace dynamics
namespace simulation {
class World;
}  // namespace simulation
}  // namespace kido

namespace kido {
namespace utils {

class SoftSdfParser : public SdfParser
{
public:
  static simulation::WorldPtr readSoftSdfFile(
      const common::Uri& _fileUri,
      const common::ResourceRetrieverPtr& _retriever = nullptr);

  static dynamics::SkeletonPtr readSkeleton(
      const common::Uri& _fileName,
      const common::ResourceRetrieverPtr& _retriever = nullptr);

  static bool createSoftPair(dynamics::SkeletonPtr skeleton,
                             dynamics::BodyNode* parent,
                             const SDFJoint& newJoint,
                             const SDFBodyNode& newBody);

  /// \brief
  static simulation::WorldPtr readWorld(
      tinyxml2::XMLElement* _worldElement,
      const std::string& _skelPath,
      const common::ResourceRetrieverPtr& _retriever);

  /// \brief
  static dynamics::SkeletonPtr readSkeleton(
      tinyxml2::XMLElement* _skeletonElement,
      const std::string& _skelPath,
      const common::ResourceRetrieverPtr& _retriever);

  /// \brief
  static SDFBodyNode readSoftBodyNode(
      tinyxml2::XMLElement* _softBodyNodeElement,
      const Eigen::Isometry3d& _skeletonFrame,
      const std::string& _skelPath,
      const common::ResourceRetrieverPtr& _retriever);
};

} // namespace utils
} // namespace kido

#endif // #ifndef KIDO_UTILS_SDF_SOFTSDFPARSER_H_
