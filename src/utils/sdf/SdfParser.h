#ifndef DART_UTILS_SDFPARSER_H
#define DART_UTILS_SDFPARSER_H

#include <map>
#include <string>
#include <Eigen/Dense>
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

//------------------------------------------------------------------------------
// Parsing Helper Functions
//------------------------------------------------------------------------------
/// @brief
simulation::World* readSdfFile(const std::string& _filename);

/// @brief
simulation::World* readWorld(tinyxml2::XMLElement* _worldElement);

/// @brief
void readPhysics(tinyxml2::XMLElement* _physicsElement,
                 simulation::World* _world);

/// @brief
dynamics::Skeleton* readSkeleton(tinyxml2::XMLElement* _skeletonElement,
                                 simulation::World* _world);

/// @brief
dynamics::BodyNode* readBodyNode(tinyxml2::XMLElement* _bodyElement,
                                     dynamics::Skeleton* _skeleton);

/// @brief
dynamics::Joint* readJoint(tinyxml2::XMLElement* _jointElement,
                             dynamics::Skeleton* _skeleton);

/// @brief
dynamics::PrismaticJoint* readPrismaticJoint(
        tinyxml2::XMLElement* _prismaticJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::RevoluteJoint* readRevoluteJoint(
        tinyxml2::XMLElement* _revoluteJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::ScrewJoint* readScrewJoint(
        tinyxml2::XMLElement* _screwJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::UniversalJoint* readUniversalJoint(
        tinyxml2::XMLElement* _universalJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::BallJoint* readBallJoint(
        tinyxml2::XMLElement* _ballJointElement,
        dynamics::Skeleton* _skeleton);

dart::dynamics::EulerJoint *readEulerJoint(
        tinyxml2::XMLElement* _eulerJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::TranslationalJoint* readTranslationalJoint(
        tinyxml2::XMLElement* _translationalJointElement,
        dynamics::Skeleton* _skeleton);

dynamics::FreeJoint* readFreeJoint(
        tinyxml2::XMLElement* _freeJointElement,
        dynamics::Skeleton* _skeleton);

dart::dynamics::WeldJoint* readWeldJoint(
        tinyxml2::XMLElement* _weldJointElement,
        dynamics::Skeleton* _skeleton);

} // namespace utils
} // namespace dart

#endif // #ifndef DART_UTILS_SDFPARSER_H
