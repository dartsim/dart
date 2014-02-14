/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
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

#ifndef SOFT_UTILS_SOFTPARSER_H_
#define SOFT_UTILS_SOFTPARSER_H_

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tinyxml2.h>

#include <dart/utils/SkelParser.h>

namespace dart {
namespace dynamics {
class Joint;
class SoftBodyNode;
class SoftSkeleton;
}  // namespace dynamics
namespace simulation {
class SoftWorld;
}  // namespace simulation
}  // namespace dart

namespace dart {
namespace utils {

class SoftSkelParser : public SkelParser
{
public:
  /// \brief
  static simulation::SoftWorld* readSoftFile(const std::string& _filename);

protected:
  /// \brief
  static simulation::SoftWorld* readSoftWorld(tinyxml2::XMLElement* _worldElement);

  /// \brief
  static dynamics::SoftSkeleton* readSoftSkeleton(
      tinyxml2::XMLElement* _softSkeletonElement,
      simulation::World* _world);

  /// \brief
  static SkelBodyNode readSoftBodyNode(
      tinyxml2::XMLElement* _softBodyNodeElement,
      dynamics::SoftSkeleton* _softSkeleton,
      const Eigen::Isometry3d& _skeletonFrame);

  /// \brief
  static dynamics::Joint* readSoftJoint(
      tinyxml2::XMLElement* _jointElement,
      const std::vector<SkelBodyNode,
      Eigen::aligned_allocator<SkelBodyNode> >& _softBodyNodes);
};

}  // namespace utils
}  // namespace dart

#endif  // SOFT_UTILS_SOFTPARSER_H_
