/**
 * @file DartLoader.h
 */

#ifndef DART_UTILS_URDF_LOADER_H
#define DART_UTILS_URDF_LOADER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>

namespace urdf {
    class ModelInterface;
    class Link;
    class Joint;
    class Pose;
    class Vector3;
}

namespace dart {

namespace dynamics {
    class Skeleton;
    class BodyNode;
    class Joint;
    class Shape;
}
namespace simulation {
    class World;
}

namespace utils {

/**
 * @class DartLoader
 */
class DartLoader {
  
public:
    dynamics::Skeleton* parseSkeleton(std::string _urdfFileName);
    simulation::World* parseWorld(std::string _urdfFileName);

private:
    void parseWorldToEntityPaths(const std::string &_xml_string);

    dynamics::Skeleton* modelInterfaceToSkeleton(boost::shared_ptr<urdf::ModelInterface> _model, std::string _rootToSkelPath = "");
    void createSkeletonRecursive(dynamics::Skeleton* _skel, boost::shared_ptr<const urdf::Link> _lk, dynamics::BodyNode* _parent, std::string _rootToSkelPath);

    template <class VisualOrCollision>
    dynamics::Shape* createShape(boost::shared_ptr<VisualOrCollision> _vizOrCol, std::string  _rootToSkelPath);

    dynamics::Joint* createDartJoint(boost::shared_ptr<const urdf::Joint> _jt);
    dynamics::BodyNode* createDartNode(boost::shared_ptr<const urdf::Link> _lk, std::string _rootToSkelPath = NULL);

    Eigen::Isometry3d toEigen(const urdf::Pose& _pose);
    Eigen::Vector3d toEigen(const urdf::Vector3& _vector);
    std::string readFileToString(std::string _xmlFile);

    std::map<std::string, std::string> mWorld_To_Entity_Paths;
};

}
}

#endif /** DART_UTILS_URDF_LOADER_H */
