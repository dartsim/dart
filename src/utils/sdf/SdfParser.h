#ifndef DART_UTILS_SDFPARSER_H
#define DART_UTILS_SDFPARSER_H

#include <map>
#include <string>
#include <Eigen/Dense>
#include <Eigen/StdVector>
// TinyXML-2 Library
// http://www.grinninglizard.com/tinyxml2/index.html
#include <tinyxml2.h>

#include "utils/Parser.h"

namespace dart {

namespace dynamics {
class Skeleton;
class BodyNode;
class Shape;
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
class FreeJoint;
}
namespace simulation {
class World;
}

namespace utils {

class SdfParser
{
public:
    /// @brief
    static simulation::World* readSdfFile(const std::string& _filename);

private:
    struct SDFBodyNode
    {
        dynamics::BodyNode* bodyNode;
        Eigen::Isometry3d initTransform;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /// @brief
    static simulation::World* readWorld(tinyxml2::XMLElement* _worldElement);

    /// @brief
    static void readPhysics(
            tinyxml2::XMLElement* _physicsElement,
            simulation::World* _world);

    /// @brief
    static dynamics::Skeleton* readSkeleton(
            tinyxml2::XMLElement* _skeletonElement,
            simulation::World* _world);

    /// @brief
    static SDFBodyNode readBodyNode(
            tinyxml2::XMLElement* _bodyNodeElement,
            dynamics::Skeleton* _skeleton,
            const Eigen::Isometry3d& _skeletonFrame);

    /// @brief
    static dynamics::Shape* readShape(
            tinyxml2::XMLElement* _shapelement);

    /// @brief
    static dynamics::Joint* readJoint(
            tinyxml2::XMLElement* _jointElement,
            const std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> >& _bodies);

    /// @brief
    static dynamics::PrismaticJoint* readPrismaticJoint(
            tinyxml2::XMLElement* _jointElement);

    static dynamics::RevoluteJoint* readRevoluteJoint(
            tinyxml2::XMLElement* _revoluteJointElement);

    static dynamics::ScrewJoint* readScrewJoint(
            tinyxml2::XMLElement* _jointElement);

    static dynamics::UniversalJoint* readUniversalJoint(
            tinyxml2::XMLElement* _jointElement);

    static dynamics::BallJoint* readBallJoint(
            tinyxml2::XMLElement* _jointElement);

    static dart::dynamics::EulerJoint *readEulerJoint(
            tinyxml2::XMLElement* _jointElement);

    static dynamics::TranslationalJoint* readTranslationalJoint(
            tinyxml2::XMLElement* _jointElement);

    static dynamics::FreeJoint* readFreeJoint(
            tinyxml2::XMLElement* _jointElement);

    static dart::dynamics::WeldJoint* readWeldJoint(
            tinyxml2::XMLElement* _jointElement);


};

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_SDFPARSER_H
