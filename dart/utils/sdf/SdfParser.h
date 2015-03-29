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
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/FreeJoint.h"

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
    static dynamics::SkeletonPtr readSkeleton(const std::string& _fileName);

public:
    struct SDFBodyNode
    {
        dynamics::BodyNode::Properties properties;
        Eigen::Isometry3d initTransform;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::shared_ptr<dynamics::Joint::Properties> JointPropPtr;

    struct SDFJoint
    {
      JointPropPtr properties;
      std::string parentName;
      std::string childName;
      std::string type;
    };

    typedef std::map<std::string, SDFBodyNode> BodyMap;
    typedef std::map<std::string, SDFJoint> JointMap;

    /// \brief
    static simulation::World* readWorld(tinyxml2::XMLElement* _worldElement,
                                        const std::string& _skelPath);

    /// \brief
    static void readPhysics(
            tinyxml2::XMLElement* _physicsElement,
            simulation::World* _world);

    /// \brief
    static dynamics::SkeletonPtr readSkeleton(
        tinyxml2::XMLElement* _skeletonElement,
        const std::string& _skelPath);

    /// \brief
    static SDFBodyNode readBodyNode(tinyxml2::XMLElement* _bodyNodeElement,
            dart::dynamics::SkeletonPtr _skeleton,
            const Eigen::Isometry3d& _skeletonFrame,
            const std::string& _skelPath);

    /// \brief
    static dart::dynamics::Shape* readShape(
            tinyxml2::XMLElement* _shapelement,
            const std::string& _skelPath);

    /// \brief
    static SDFJoint readJoint(tinyxml2::XMLElement* _jointElement,
        const BodyMap& _bodies,
        const Eigen::Isometry3d& _skeletonFrame);

    static dart::dynamics::WeldJoint::Properties readWeldJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame,
        const std::string& _name);

    static dynamics::RevoluteJoint::Properties readRevoluteJoint(
            tinyxml2::XMLElement* _revoluteJointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame,
        const std::string& _name);

    static dynamics::PrismaticJoint::Properties readPrismaticJoint(
        tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame,
        const std::string& _name);

    static dynamics::ScrewJoint::Properties readScrewJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame,
        const std::string& _name);

    static dynamics::UniversalJoint::Properties readUniversalJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame,
        const std::string& _name);

    static dynamics::BallJoint::Properties readBallJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame,
        const std::string& _name);

    static dart::dynamics::EulerJoint* readEulerJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame,
        const std::string& _name);

    static dart::dynamics::TranslationalJoint::Properties readTranslationalJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame,
        const std::string& _name);

    static dynamics::FreeJoint::Properties readFreeJoint(
            tinyxml2::XMLElement* _jointElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const Eigen::Isometry3d& _jointFrame,
        const std::string& _name);


};

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_SDFPARSER_H
