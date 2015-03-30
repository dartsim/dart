/*
 * Copyright (c) 2013-2015, Georgia Tech Research Corporation
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

#ifndef DART_UTILS_SKEL_PARSER_H
#define DART_UTILS_SKEL_PARSER_H

#include <cstddef>

#include <Eigen/StdVector>
#include <Eigen/Dense>
// TinyXML-2 Library
// http://www.grinninglizard.com/tinyxml2/index.html
#include <tinyxml2.h>

#include "dart/common/Deprecated.h"
#include "dart/utils/Parser.h"
#include "dart/dynamics/Skeleton.h"

namespace dart {

namespace dynamics {
class Joint;
class WeldJoint;
class PrismaticJoint;
class RevoluteJoint;
class ScrewJoint;
class UniversalJoint;
class BallJoint;
class EulerXYZJoint;
class EulerJoint;
class TranslationalJoint;
class PlanarJoint;
class FreeJoint;
class Marker;
}

namespace dynamics {
class BodyNode;
class Shape;
class Skeleton;
}

namespace simulation {
class World;
}

namespace utils {

/// SkelParser
class SkelParser
{
public:
  /// Read World from skel file
  static simulation::World* readWorld(const std::string& _filename);

  /// Read Skeleton from skel file
  static dynamics::SkeletonPtr readSkeleton(const std::string& _filename);

protected:
  ///
  struct SkelBodyNode
  {
    dynamics::BodyNode* bodyNode;
    Eigen::Isometry3d initTransform;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  ///
  static simulation::World* readWorld(tinyxml2::XMLElement* _worldElement);

  ///
  static dart::dynamics::SkeletonPtr readSkeleton(
      tinyxml2::XMLElement* _skeletonElement);

  ///
  static SkelBodyNode readBodyNode(tinyxml2::XMLElement* _bodyElement,
      dynamics::SkeletonPtr _skeleton,
      const Eigen::Isometry3d& _skeletonFrame);

  ///
  static SkelBodyNode readSoftBodyNode(tinyxml2::XMLElement* _softBodyNodeElement,
      dynamics::SkeletonPtr _Skeleton,
      const Eigen::Isometry3d& _skeletonFrame);

  ///
  static dynamics::Shape* readShape(tinyxml2::XMLElement* _shapeElement);

  /// Read marker
  static dart::dynamics::Marker* readMarker(
      tinyxml2::XMLElement* _markerElement,
      dynamics::BodyNode* _bodyNode);

  ///
  static dynamics::Joint* readJoint(
      tinyxml2::XMLElement* _jointElement,
      const std::vector<SkelBodyNode,
      Eigen::aligned_allocator<SkelBodyNode> >& _softBodyNodes);

  ///
  static void readDegreeOfFreedom(
      tinyxml2::XMLElement* _dofElement,
      dart::dynamics::Joint* _dartJoint);

  ///
  static dynamics::PrismaticJoint* readPrismaticJoint(
      tinyxml2::XMLElement* _jointElement);

  ///
  static dynamics::RevoluteJoint* readRevoluteJoint(
      tinyxml2::XMLElement* _jointElement);

  ///
  static dynamics::ScrewJoint* readScrewJoint(
      tinyxml2::XMLElement* _jointElement);

  ///
  static dynamics::UniversalJoint* readUniversalJoint(
      tinyxml2::XMLElement* _universalJointElement);

  ///
  static dynamics::BallJoint* readBallJoint(
      tinyxml2::XMLElement* _jointElement);

  ///
  static dart::dynamics::EulerJoint* readEulerJoint(
      tinyxml2::XMLElement* _jointElement);

  ///
  static dynamics::TranslationalJoint* readTranslationalJoint(
      tinyxml2::XMLElement* _jointElement);

  ///
  static dart::dynamics::PlanarJoint* readPlanarJoint(
      tinyxml2::XMLElement* _jointElement);

  ///
  static dynamics::FreeJoint* readFreeJoint(
      tinyxml2::XMLElement* _jointElement);

  ///
  static dart::dynamics::WeldJoint* readWeldJoint(
      tinyxml2::XMLElement* _jointElement);

  /// Read axis
  static void readJointDynamicsAndLimit(
      tinyxml2::XMLElement* _jointElement,
      dart::dynamics::Joint* _joint,
      size_t _numAxis);
};

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_SKEL_PARSER_H
