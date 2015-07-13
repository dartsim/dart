/**
 * @file DartLoader.h
 */

#ifndef DART_UTILS_URDF_LOADER_H
#define DART_UTILS_URDF_LOADER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <string>

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/simulation/World.h"

namespace urdf
{
  class ModelInterface;
  class Link;
  class Joint;
  class Pose;
  class Vector3;
}

namespace dart {

namespace dynamics
{
  class Skeleton;
  class BodyNode;
  class Joint;
  class Shape;
}
namespace simulation
{
  class World;
}

namespace utils {

/**
 * @class DartLoader
 */
class DartLoader {
  
public:

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
    void addPackageDirectory(const std::string& _packageName,
                             const std::string& _packageDirectory);

    /// Parse a file to produce a Skeleton
    dynamics::SkeletonPtr parseSkeleton(const std::string& _urdfFileName);

    /// Parse a text string to produce a Skeleton
    dynamics::SkeletonPtr parseSkeletonString(const std::string& _urdfString,
                                              const std::string& _urdfFileDirectory);

    /// Parse a file to produce a World
    dart::simulation::WorldPtr parseWorld(const std::string& _urdfFileName);

    /// Parse a text string to produce a World
    dart::simulation::WorldPtr parseWorldString(const std::string& _urdfString,
                                        const std::string& _urdfFileDirectory);

private:

    typedef std::shared_ptr<dynamics::BodyNode::Properties> BodyPropPtr;
    typedef std::shared_ptr<dynamics::Joint::Properties> JointPropPtr;

    std::string getFullFilePath(const std::string& _filename) const;
    void parseWorldToEntityPaths(const std::string& _xml_string);

    dart::dynamics::SkeletonPtr modelInterfaceToSkeleton(const urdf::ModelInterface* _model);
    bool createSkeletonRecursive(dynamics::SkeletonPtr _skel, const urdf::Link* _lk, dynamics::BodyNode* _parent);

    template <class VisualOrCollision>
    dynamics::ShapePtr createShape(const VisualOrCollision* _vizOrCol);

    dynamics::BodyNode* createDartJointAndNode(
        const urdf::Joint* _jt,
        const dynamics::BodyNode::Properties& _body,
        dynamics::BodyNode* _parent,
        dynamics::SkeletonPtr _skeleton);

    bool createDartNodeProperties(
        const urdf::Link* _lk, dynamics::BodyNode::Properties *properties);

    Eigen::Isometry3d toEigen(const urdf::Pose& _pose);
    Eigen::Vector3d toEigen(const urdf::Vector3& _vector);
    std::string readFileToString(std::string _xmlFile);

    std::map<std::string, std::string> mWorld_To_Entity_Paths;

    std::map<std::string, std::string> mPackageDirectories;
    std::string mRootToSkelPath;
    std::string mRootToWorldPath;
};

}
}

#endif /** DART_UTILS_URDF_LOADER_H */
