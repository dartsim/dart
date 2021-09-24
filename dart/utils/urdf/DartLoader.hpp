/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_UTILS_URDF_DARTLOADER_HPP_
#define DART_UTILS_URDF_DARTLOADER_HPP_

#include <map>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/ResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/CompositeResourceRetriever.hpp"
#include "dart/utils/PackageResourceRetriever.hpp"

namespace urdf {
class ModelInterface;
class Link;
class Joint;
class Pose;
class Vector3;
} // namespace urdf

namespace dart {

namespace dynamics {
class Skeleton;
class BodyNode;
class Joint;
class Shape;
} // namespace dynamics
namespace simulation {
class World;
}

namespace utils {

/**
 * @class DartLoader
 */
class DartLoader
{
public:
  /// \deprecated Deprecated in 6.11. Use RootJointType and Options instead.
  ///
  /// Flags for specifying URDF file parsing policies.
  enum Flags
  {
    NONE = 0,

    /// Parser the root link's joint type to be "fixed" joint when not
    /// specified.
    FIXED_BASE_LINK = 1 << 1,

    /// The default flgas
    DEFAULT = NONE,
  };

  /// Root joint type to be used when the parent joint of the root link is not
  /// specified in the URDF file.
  enum class RootJointType
  {
    /// Floating joint type of URDF.
    FLOATING = 0,

    /// Fixed joint type of URDF.
    FIXED = 1,
  };

  /// Options to be used in parsing URDF files.
  struct Options
  {
    /// Resource retriever. LocalResourceRetriever is used if it's nullptr.
    common::ResourceRetrieverPtr mResourceRetriever;

    /// Default root joint type to be used when the parent joint of the root
    /// link is not specified in the URDF file.
    RootJointType mDefaultRootJointType;

    /// Default inertia properties to be used when the inertial element is not
    /// specified in the link element.
    dynamics::Inertia mDefaultInertia;

    /// Default constructor
    Options(
        common::ResourceRetrieverPtr resourceRetriever = nullptr,
        RootJointType defaultRootJointType = RootJointType::FLOATING,
        const dynamics::Inertia& defaultInertia = dynamics::Inertia());
  };

  /// Constructor with the default ResourceRetriever.
  explicit DartLoader(const Options& options = Options());

  /// Sets options
  void setOptions(const Options& options);

  /// Returns options
  const Options& getOptions() const;

  /// Specify the directory of a ROS package. In your URDF files, you may see
  /// strings with a package URI pattern such as:
  ///
  /// @code
  /// "package://my_robot/meshes/mesh_for_my_robot.stl"
  ///  \______/  \______/\___________________________/
  ///      |        |                 |
  ///   package  package   file path with respect to
  ///   keyword   name       the package directory
  /// @endcode
  ///
  /// For us to successfully parse a URDF, we need to be told what the path
  /// to the package directory is, using addPackageDirectory(). In this case,
  /// suppose the path to the my_robot package is /path/to/my_robot. Then you
  /// should use addPackageDirectory("my_robot", "/path/to/my_robot").
  /// Altogether, this implies that a file named
  /// "/path/to/my_robot/meshes/mesh_for_my_robot.stl" exists. Whatever you
  /// specify as the package directory will end up replacing the 'package
  /// keyword' and 'package name' components of the URI string.
  void addPackageDirectory(
      const std::string& packageName, const std::string& packageDirectory);

  /// Parse a file to produce a Skeleton
  DART_DEPRECATED(6.11)
  dynamics::SkeletonPtr parseSkeleton(
      const common::Uri& uri,
      const common::ResourceRetrieverPtr& resourceRetriever,
      unsigned int flags = DEFAULT);

  /// Parse a file to produce a Skeleton
  dynamics::SkeletonPtr parseSkeleton(const common::Uri& uri);

  /// Parse a text string to produce a Skeleton
  DART_DEPRECATED(6.11)
  dynamics::SkeletonPtr parseSkeletonString(
      const std::string& urdfString,
      const common::Uri& baseUri,
      const common::ResourceRetrieverPtr& resourceRetriever,
      unsigned int flags = DEFAULT);

  /// Parse a text string to produce a Skeleton
  dynamics::SkeletonPtr parseSkeletonString(
      const std::string& urdfString, const common::Uri& baseUri);

  /// Parse a file to produce a World
  DART_DEPRECATED(6.11)
  dart::simulation::WorldPtr parseWorld(
      const common::Uri& uri,
      const common::ResourceRetrieverPtr& resourceRetriever,
      unsigned int flags = DEFAULT);

  /// Parse a file to produce a World
  dart::simulation::WorldPtr parseWorld(const common::Uri& uri);

  /// Parse a text string to produce a World
  DART_DEPRECATED(6.11)
  dart::simulation::WorldPtr parseWorldString(
      const std::string& urdfString,
      const common::Uri& baseUri,
      const common::ResourceRetrieverPtr& resourceRetriever,
      unsigned int flags = DEFAULT);

  /// Parse a text string to produce a World
  dart::simulation::WorldPtr parseWorldString(
      const std::string& urdfString, const common::Uri& baseUri);

private:
  typedef std::shared_ptr<dynamics::BodyNode::Properties> BodyPropPtr;
  typedef std::shared_ptr<dynamics::Joint::Properties> JointPropPtr;

  /// Parses the ModelInterface and spits out a Skeleton object
  static dart::dynamics::SkeletonPtr modelInterfaceToSkeleton(
      const urdf::ModelInterface* model,
      const common::Uri& baseUri,
      const common::ResourceRetrieverPtr& resourceRetriever,
      const Options& options);

  static bool createSkeletonRecursive(
      const urdf::ModelInterface* model,
      dynamics::SkeletonPtr skel,
      const urdf::Link* lk,
      dynamics::BodyNode* parent,
      const common::Uri& baseUri,
      const common::ResourceRetrieverPtr& _resourceRetriever,
      const Options& options);

  static bool addMimicJointsRecursive(
      const urdf::ModelInterface* model,
      dynamics::SkeletonPtr _skel,
      const urdf::Link* _lk);

  template <class VisualOrCollision>
  static dynamics::ShapePtr createShape(
      const VisualOrCollision* _vizOrCol,
      const common::Uri& _baseUri,
      const common::ResourceRetrieverPtr& _resourceRetriever);

  static dynamics::BodyNode* createDartJointAndNode(
      const urdf::Joint* _jt,
      const dynamics::BodyNode::Properties& _body,
      dynamics::BodyNode* _parent,
      dynamics::SkeletonPtr _skeleton,
      const Options& options);

  static bool createDartNodeProperties(
      const urdf::Link* _lk,
      dynamics::BodyNode::Properties& properties,
      const common::Uri& _baseUri,
      const common::ResourceRetrieverPtr& _resourceRetriever,
      const Options& options);

  static bool createShapeNodes(
      const urdf::ModelInterface* model,
      const urdf::Link* lk,
      dynamics::BodyNode* bodyNode,
      const common::Uri& baseUri,
      const common::ResourceRetrieverPtr& resourceRetriever);

  common::ResourceRetrieverPtr getResourceRetriever(
      const common::ResourceRetrieverPtr& _resourceRetriever);

  static Eigen::Isometry3d toEigen(const urdf::Pose& _pose);
  static Eigen::Vector3d toEigen(const urdf::Vector3& _vector);

  static bool readFileToString(
      const common::ResourceRetrieverPtr& _resourceRetriever,
      const common::Uri& _uri,
      std::string& _output);

  Options mOptions;
  common::LocalResourceRetrieverPtr mLocalRetriever;
  utils::PackageResourceRetrieverPtr mPackageRetriever;
  utils::CompositeResourceRetrieverPtr mRetriever;
};

} // namespace utils
} // namespace dart

#endif // DART_UTILS_URDF_DARTLOADER_HPP_
