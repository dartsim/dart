#ifndef DART_UTILS_SDFPARSER_H
#define DART_UTILS_SDFPARSER_H

#include <map>
#include <string>
#include <Eigen/Dense>
#include <Eigen/StdVector>
// TinyXML-2 Library
// http://www.grinninglizard.com/tinyxml2/index.html
#include <tinyxml2.h>

#include "dart/utils/Parser.h"

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
    /// \brief
    static simulation::World* readSdfFile(const std::string& _filename);

    /// \brief
    static dynamics::Skeleton* readSkeleton(const std::string& _fileName);

public:
    struct SDFBodyNode
    {
        dynamics::BodyNode* bodyNode;
        Eigen::Isometry3d initTransform;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    /// \brief
    static simulation::World* readWorld(tinyxml2::XMLElement* _worldElement,
                                        const std::string& _skelPath);

    /// \brief
    static void readPhysics(
            tinyxml2::XMLElement* _physicsElement,
            simulation::World* _world);

    /// \brief
    static dynamics::Skeleton* readSkeleton(
        tinyxml2::XMLElement* _skeletonElement,
        const std::string& _skelPath);

    /// \brief
    static SDFBodyNode readBodyNode(
            tinyxml2::XMLElement* _bodyNodeElement,
            dynamics::Skeleton* _skeleton,
            const Eigen::Isometry3d& _skeletonFrame,
            const std::string& _skelPath);

    /// \brief
    static dart::dynamics::Shape* readShape(
            tinyxml2::XMLElement* _shapelement,
            const std::string& _skelPath);

    /// \brief
    static dynamics::Joint* readJoint(
            tinyxml2::XMLElement* _jointElement,
            const std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> >& _bodies,
            const Eigen::Isometry3d& _skeletonFrame);

    /// \brief
    static dynamics::PrismaticJoint* readPrismaticJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame);

    static dynamics::RevoluteJoint* readRevoluteJoint(
            tinyxml2::XMLElement* _revoluteJointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame);

    static dynamics::ScrewJoint* readScrewJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame);

    static dynamics::UniversalJoint* readUniversalJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame);

    static dynamics::BallJoint* readBallJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame);

    static dart::dynamics::EulerJoint *readEulerJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame);

    static dynamics::TranslationalJoint* readTranslationalJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame);

    static dynamics::FreeJoint* readFreeJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame);

    static dart::dynamics::WeldJoint* readWeldJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame);


};

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_SDFPARSER_H
