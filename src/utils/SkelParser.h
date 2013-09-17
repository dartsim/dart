/*
 * Copyright (c) 2012, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/24/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#include <vector>
#include <Eigen/Dense>
// TinyXML-2 Library
// http://www.grinninglizard.com/tinyxml2/index.html
#include <tinyxml2.h>

#include "math/Geometry.h"
#include "utils/Parser.h"

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
class FreeJoint;
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

#define DART_OK 0
#define DART_ERROR 1

//------------------------------------------------------------------------------
// Parsing Helper Functions
//------------------------------------------------------------------------------
/// @brief
simulation::World* readSkelFile(const std::string& _filename);

/// @brief
simulation::World* readWorld(tinyxml2::XMLElement* _worldElement);

/// @brief
dynamics::Skeleton* readSkeleton(tinyxml2::XMLElement* _skeletonElement,
                                 simulation::World* _world);

/// @brief
dynamics::BodyNode* readBodyNode(tinyxml2::XMLElement* _bodyElement,
                                 dynamics::Skeleton* _skeleton,
                                 const Eigen::Isometry3d& _skeletonFrame);

/// @brief
dynamics::Shape* readShape(tinyxml2::XMLElement* _shapeElement,
                           dynamics::BodyNode* _bodyNode);

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

#endif // #ifndef DART_UTILS_SKEL_PARSER_H
