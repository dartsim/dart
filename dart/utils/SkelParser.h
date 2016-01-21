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
#include "dart/utils/XmlHelpers.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/SingleDofJoint.h"
#include "dart/dynamics/MultiDofJoint.h"
#include "dart/simulation/World.h"

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
  static simulation::WorldPtr readWorld(
    const common::Uri& _uri,
    const common::ResourceRetrieverPtr& _retriever = nullptr);

  /// Read World from an xml-formatted string
  static simulation::WorldPtr readWorldXML(
    const std::string& _xmlString,
    const common::Uri& _baseUri = "",
    const common::ResourceRetrieverPtr& _retriever = nullptr);

  /// Read Skeleton from skel file
  static dynamics::SkeletonPtr readSkeleton(
    const common::Uri& _fileUri,
    const common::ResourceRetrieverPtr& _retriever = nullptr);

  typedef std::shared_ptr<dynamics::BodyNode::Properties> BodyPropPtr;

  ///
  struct SkelBodyNode
  {
    BodyPropPtr properties;
    Eigen::Isometry3d initTransform;
    std::string type;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  // first: BodyNode name | second: BodyNode information
  typedef Eigen::aligned_map<std::string, SkelBodyNode> BodyMap;

  typedef std::shared_ptr<dynamics::Joint::Properties> JointPropPtr;
  struct SkelJoint
  {
    JointPropPtr properties;
    Eigen::VectorXd position;
    Eigen::VectorXd velocity;
    Eigen::VectorXd acceleration;
    Eigen::VectorXd force;
    std::string parentName;
    std::string childName;
    std::string type;
  };

  // first: Child BodyNode name | second: Joint information
  typedef std::map<std::string, SkelJoint> JointMap;

  // first: Order that Joint appears in file | second: Child BodyNode name
  typedef std::map<size_t, std::string> IndexToJoint;

  // first: Child BodyNode name | second: Order that Joint appears in file
  typedef std::map<std::string, size_t> JointToIndex;

protected:

  static simulation::WorldPtr readWorld(
      tinyxml2::XMLElement* _worldElement,
      const common::Uri& _baseUri,
      const common::ResourceRetrieverPtr& _retriever);

  static dart::dynamics::SkeletonPtr readSkeleton(
      tinyxml2::XMLElement* _skeletonElement,
      const common::Uri& _baseUri,
      const common::ResourceRetrieverPtr& _retriever);

  static SkelBodyNode readBodyNode(
      tinyxml2::XMLElement* _bodyElement,
      const Eigen::Isometry3d& _skeletonFrame,
      const common::Uri& _baseUri,
      const common::ResourceRetrieverPtr& _retriever);

  static SkelBodyNode readSoftBodyNode(
      tinyxml2::XMLElement* _softBodyNodeElement,
      const Eigen::Isometry3d& _skeletonFrame,
      const common::Uri& _baseUri,
      const common::ResourceRetrieverPtr& _retriever);

  static dynamics::ShapePtr readShape(
      tinyxml2::XMLElement* _shapeElement,
      const std::string& bodyName,
      const common::Uri& _baseUri,
      const common::ResourceRetrieverPtr& _retriever);

  /// Read marker
  static dynamics::Marker::Properties readMarker(
      tinyxml2::XMLElement* _markerElement);

  ///
  static void readJoint(
      tinyxml2::XMLElement* _jointElement,
      const BodyMap& _bodyNodes,
      JointMap& _joints,
      IndexToJoint& _order,
      JointToIndex& _lookup);

  ///
  static JointPropPtr readRevoluteJoint(
      tinyxml2::XMLElement* _jointElement,
      SkelJoint& _joint,
      const std::string& _name);

  ///
  static JointPropPtr readPrismaticJoint(
      tinyxml2::XMLElement* _jointElement,
      SkelJoint& _joint,
      const std::string& _name);

  ///
  static JointPropPtr readScrewJoint(
      tinyxml2::XMLElement* _jointElement,
      SkelJoint& _joint,
      const std::string& _name);

  ///
  static JointPropPtr readUniversalJoint(
      tinyxml2::XMLElement* _universalJointElement,
      SkelJoint& _joint,
      const std::string& _name);

  ///
  static JointPropPtr readBallJoint(
      tinyxml2::XMLElement* _jointElement,
      SkelJoint& _joint,
      const std::string& _name);

  ///
  static JointPropPtr readEulerJoint(
      tinyxml2::XMLElement* _jointElement,
      SkelJoint& _joint,
      const std::string& _name);

  ///
  static JointPropPtr readTranslationalJoint(
      tinyxml2::XMLElement* _jointElement,
      SkelJoint& _joint,
      const std::string& _name);

  ///
  static JointPropPtr readPlanarJoint(
      tinyxml2::XMLElement* _jointElement,
      SkelJoint& _joint,
      const std::string& _name);

  ///
  static JointPropPtr readFreeJoint(
      tinyxml2::XMLElement* _jointElement,
      SkelJoint& _joint,
      const std::string& _name);

  ///
  static JointPropPtr readWeldJoint(
      tinyxml2::XMLElement* _jointElement,
      SkelJoint& _joint,
      const std::string& _name);

  static common::ResourceRetrieverPtr getRetriever(const common::ResourceRetrieverPtr& _retriever);
};

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_SKEL_PARSER_H
