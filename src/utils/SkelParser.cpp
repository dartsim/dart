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


// Standard Library
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>
#include <Eigen/Dense>

// Local Files
#include "common/Console.h"
#include "dynamics/BodyNode.h"
#include "dynamics/BoxShape.h"
#include "dynamics/CylinderShape.h"
#include "dynamics/EllipsoidShape.h"
#include "dynamics/WeldJoint.h"
#include "dynamics/PrismaticJoint.h"
#include "dynamics/RevoluteJoint.h"
#include "dynamics/ScrewJoint.h"
#include "dynamics/TranslationalJoint.h"
#include "dynamics/BallJoint.h"
#include "dynamics/FreeJoint.h"
#include "dynamics/EulerJoint.h"
#include "dynamics/UniversalJoint.h"
#include "dynamics/Skeleton.h"
#include "simulation/World.h"
#include "utils/SkelParser.h"

namespace dart {
namespace utils {

simulation::World* readSkelFile(const std::string& _filename)
{
    //--------------------------------------------------------------------------
    // Load xml and create Document
    tinyxml2::XMLDocument _dartFile;
    try
    {
        openXMLFile(_dartFile, _filename.c_str());
    }
    catch(std::exception const& e)
    {
        std::cout << "LoadFile Fails: " << e.what() << std::endl;
        return NULL;
    }

    //--------------------------------------------------------------------------
    // Load DART
    tinyxml2::XMLElement* dartElement = NULL;
    dartElement = _dartFile.FirstChildElement("skel");
    if (dartElement == NULL)
        return NULL;

    //--------------------------------------------------------------------------
    // Load World
    tinyxml2::XMLElement* worldElement = NULL;
    worldElement = dartElement->FirstChildElement("world");
    if (worldElement == NULL)
        return NULL;

    simulation::World* newWorld = readWorld(worldElement);

    return newWorld;
}

simulation::World* readWorld(tinyxml2::XMLElement* _worldElement)
{
    assert(_worldElement != NULL);

    // Create a world
    simulation::World* newWorld = new simulation::World;

    //--------------------------------------------------------------------------
    // Load physics
    tinyxml2::XMLElement* physicsElement = NULL;
    physicsElement = _worldElement->FirstChildElement("physics");
    if (physicsElement != NULL)
    {
        // Time step
        tinyxml2::XMLElement* timeStepElement = NULL;
        timeStepElement = physicsElement->FirstChildElement("time_step");
        if (timeStepElement != NULL)
        {
            std::string strTimeStep = timeStepElement->GetText();
            double timeStep = toDouble(strTimeStep);
            newWorld->setTimeStep(timeStep);
        }

        // Gravity
        tinyxml2::XMLElement* gravityElement = NULL;
        gravityElement = physicsElement->FirstChildElement("gravity");
        if (gravityElement != NULL)
        {
            std::string strGravity = gravityElement->GetText();
            Eigen::Vector3d gravity = toVector3d(strGravity);
            newWorld->setGravity(gravity);
        }
    }

    //--------------------------------------------------------------------------
    // Load skeletons
    ElementEnumerator skeletonElements(_worldElement, "skeleton");
    while (skeletonElements.next())
    {
        dynamics::Skeleton* newSkeleton
                = readSkeleton(skeletonElements.get(), newWorld);

        newWorld->addSkeleton(newSkeleton);
    }

    return newWorld;
}

dynamics::Skeleton* readSkeleton(tinyxml2::XMLElement* _skeletonElement,
                                 simulation::World* _world)
{
    assert(_skeletonElement != NULL);
    assert(_world != NULL);

    dynamics::Skeleton* newSkeleton = new dynamics::Skeleton;
    Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();

    //--------------------------------------------------------------------------
    // Name attribute
    std::string name = getAttribute(_skeletonElement, "name");
    newSkeleton->setName(name);

    //--------------------------------------------------------------------------
    // transformation
    if (hasElement(_skeletonElement, "transformation"))
    {
        Eigen::Isometry3d W = getValueIsometry3d(_skeletonElement, "transformation");
        skeletonFrame = W;
    }

    //--------------------------------------------------------------------------
    // immobile attribute
    tinyxml2::XMLElement* immobileElement = NULL;
    immobileElement = _skeletonElement->FirstChildElement("immobile");
    if (immobileElement != NULL)
    {
        std::string stdImmobile = immobileElement->GetText();
        bool immobile = toBool(stdImmobile);
        newSkeleton->setImmobileState(immobile);
    }

    //--------------------------------------------------------------------------
    // Bodies
    ElementEnumerator bodies(_skeletonElement, "body");
    while (bodies.next())
    {
        dynamics::BodyNode* newBody
                = readBodyNode(bodies.get(), newSkeleton, skeletonFrame);

        newSkeleton->addBodyNode(newBody, false);
    }

    //--------------------------------------------------------------------------
    // Joints
    ElementEnumerator joints(_skeletonElement, "joint");
    while (joints.next())
    {
        dynamics::Joint* newJoint
                = readJoint(joints.get(), newSkeleton);

        newSkeleton->addJoint(newJoint);
    }

    return newSkeleton;
}

dynamics::BodyNode* readBodyNode(tinyxml2::XMLElement* _bodyNodeElement,
                                 dynamics::Skeleton* _skeleton,
                                 const Eigen::Isometry3d& _skeletonFrame)
{
    assert(_bodyNodeElement != NULL);
    assert(_skeleton != NULL);

    dynamics::BodyNode* newBodyNode = new dynamics::BodyNode;

    // Name attribute
    std::string name = getAttribute(_bodyNodeElement, "name");
    newBodyNode->setName(name);

    //--------------------------------------------------------------------------
    // gravity
    if (hasElement(_bodyNodeElement, "gravity"))
    {
        bool gravityMode = getValueBool(_bodyNodeElement, "gravity");
        newBodyNode->setGravityMode(gravityMode);
    }

    //--------------------------------------------------------------------------
    // self_collide
//    if (hasElement(_bodyElement, "self_collide"))
//    {
//        bool gravityMode = getValueBool(_bodyElement, "self_collide");
//    }

    //--------------------------------------------------------------------------
    // transformation
    if (hasElement(_bodyNodeElement, "transformation"))
    {
        Eigen::Isometry3d W = getValueIsometry3d(_bodyNodeElement, "transformation");
        newBodyNode->setWorldTransform(_skeletonFrame * W);
    }

    //--------------------------------------------------------------------------
    // visualization_shape
    ElementEnumerator vizShapes(_bodyNodeElement, "visualization_shape");
    while (vizShapes.next())
    {
        dynamics::Shape* newShape
                = readShape(vizShapes.get(), newBodyNode);

        newBodyNode->addVisualizationShape(newShape);
    }

    //--------------------------------------------------------------------------
    // visualization_shape
    ElementEnumerator collShapes(_bodyNodeElement, "collision_shape");
    while (collShapes.next())
    {
        dynamics::Shape* newShape
                = readShape(collShapes.get(), newBodyNode);

        newBodyNode->addCollisionShape(newShape);
    }

    //--------------------------------------------------------------------------
    // inertia
    if (hasElement(_bodyNodeElement, "inertia"))
    {
        tinyxml2::XMLElement* inertiaElement = getElement(_bodyNodeElement, "inertia");

        // mass
        double mass = getValueDouble(inertiaElement, "mass");
        newBodyNode->setMass(mass);

        // moment of inertia
        if (hasElement(inertiaElement, "moment_of_inertia"))
        {
            tinyxml2::XMLElement* moiElement
                    = getElement(inertiaElement, "moment_of_inertia");

            double ixx = getValueDouble(moiElement, "ixx");
            double iyy = getValueDouble(moiElement, "iyy");
            double izz = getValueDouble(moiElement, "izz");

            double ixy = getValueDouble(moiElement, "ixy");
            double ixz = getValueDouble(moiElement, "ixz");
            double iyz = getValueDouble(moiElement, "iyz");

            newBodyNode->setInertia(ixx, iyy, izz, ixy, ixz, iyz);
        }
        else if (newBodyNode->getVisualizationShape(0) != 0)
        {
            Eigen::Matrix3d Ic = newBodyNode->getVisualizationShape(0)->computeInertia(mass);

            newBodyNode->setInertia(Ic(0,0), Ic(1,1), Ic(2,2),
                                            Ic(0,1), Ic(0,2), Ic(1,2));
        }

        // offset
        if (hasElement(inertiaElement, "offset"))
        {
            Eigen::Vector3d offset = getValueVector3d(inertiaElement, "offset");
            newBodyNode->setLocalCOM(offset);
        }
    }

    return newBodyNode;
}

dynamics::Shape*readShape(tinyxml2::XMLElement* vizElement, dynamics::BodyNode* _bodyNode)
{
    dynamics::Shape* newShape = NULL;

    // type
    assert(hasElement(vizElement, "geometry"));
    tinyxml2::XMLElement* geometryElement = getElement(vizElement, "geometry");

    if (hasElement(geometryElement, "box"))
    {
        tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

        Eigen::Vector3d size = getValueVector3d(boxElement, "size");

        newShape = new dynamics::BoxShape(size);
    }
    else if (hasElement(geometryElement, "ellipsoid"))
    {
        tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement, "ellipsoid");

        Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

        newShape = new dynamics::EllipsoidShape(size);
    }
    else if (hasElement(geometryElement, "cylinder"))
    {
        tinyxml2::XMLElement* cylinderElement = getElement(geometryElement, "cylinder");

        double radius = getValueDouble(cylinderElement, "radius");
        double height = getValueDouble(cylinderElement, "height");

        newShape = new dynamics::CylinderShape(radius, height);
    }
    else
    {
        dterr << "Unknown visualization shape.\n";
        assert(0);
    }

    // transformation
    if (hasElement(vizElement, "transformation"))
    {
        Eigen::Isometry3d W = getValueIsometry3d(vizElement, "transformation");
        newShape->setLocalTransform(W);
    }

    // color
    if (hasElement(vizElement, "color"))
    {
        Eigen::Vector3d color = getValueVector3d(vizElement, "color");
        newShape->setColor(color);
    }

    return newShape;
}

dynamics::Joint* readJoint(tinyxml2::XMLElement* _jointElement,
                            dynamics::Skeleton* _skeleton)
{
    assert(_jointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::Joint* newJoint = NULL;

    //--------------------------------------------------------------------------
    // Type attribute
    std::string type = getAttribute(_jointElement, "type");
    assert(!type.empty());
    if (type == std::string("weld"))
        newJoint = readWeldJoint(_jointElement, _skeleton);
    if (type == std::string("prismatic"))
        newJoint = readPrismaticJoint(_jointElement, _skeleton);
    if (type == std::string("revolute"))
        newJoint = readRevoluteJoint(_jointElement, _skeleton);
    if (type == std::string("universal"))
        newJoint = readUniversalJoint(_jointElement, _skeleton);
    if (type == std::string("ball"))
        newJoint = readBallJoint(_jointElement, _skeleton);
    if (type == std::string("euler"))
        newJoint = readEulerJoint(_jointElement, _skeleton);
    if (type == std::string("translational"))
        newJoint = readTranslationalJoint(_jointElement, _skeleton);
    if (type == std::string("free"))
        newJoint = readFreeJoint(_jointElement, _skeleton);
    assert(newJoint != NULL);

    //--------------------------------------------------------------------------
    // Name attribute
    std::string name = getAttribute(_jointElement, "name");
    newJoint->setName(name);

    //--------------------------------------------------------------------------
    // parent
    dynamics::BodyNode* parentBody = NULL;
    if (hasElement(_jointElement, "parent"))
    {
        std::string strParent = getValueString(_jointElement, "parent");

        if (strParent == std::string("world"))
        {
            newJoint->setParentBodyNode(NULL);
        }
        else
        {
            parentBody = _skeleton->getBodyNode(strParent);
            if (parentBody == NULL)
            {
                dterr << "Can't find the parent body, "
                  << strParent
                  << ", of the joint, "
                  << newJoint->getName()
                  << ", in the skeleton, "
                  << _skeleton->getName()
                  << ". " << std::endl;
                assert(parentBody != NULL);
            }
            newJoint->setParentBodyNode(parentBody);
        }
    }
    else
    {
        dterr << "No parent body.\n";
        assert(0);
    }

    //--------------------------------------------------------------------------
    // child
    dynamics::BodyNode* childBody = NULL;
    if (hasElement(_jointElement, "child"))
    {
        std::string strChild = getValueString(_jointElement, "child");
        childBody = _skeleton->getBodyNode(strChild);
        assert(childBody != NULL && "Dart cannot find child body.");
        newJoint->setChildBodyNode(childBody);
    }
    else
    {
        dterr << "No child body.\n";
        assert(0);
    }

    //--------------------------------------------------------------------------
    // transformation
    Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
    assert(childBody != NULL);
    Eigen::Isometry3d childWorld = childBody->getWorldTransform();
    if (parentBody)
         parentWorld = parentBody->getWorldTransform();
    if (hasElement(_jointElement, "transformation"))
        childToJoint = getValueIsometry3d(_jointElement, "transformation");
    Eigen::Isometry3d parentToJoint = parentWorld.inverse()*childWorld*childToJoint;
    newJoint->setTransformFromChildBodyNode(childToJoint);
    newJoint->setTransformFromParentBodyNode(parentToJoint);

    return newJoint;
}

dynamics::WeldJoint*readWeldJoint(
        tinyxml2::XMLElement* _weldJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_weldJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::WeldJoint* newWeldJoint = new dynamics::WeldJoint;

    return newWeldJoint;
}

dynamics::RevoluteJoint*readRevoluteJoint(
        tinyxml2::XMLElement* _revoluteJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_revoluteJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::RevoluteJoint* newRevoluteJoint = new dynamics::RevoluteJoint;

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_revoluteJointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_revoluteJointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newRevoluteJoint->setAxis(xyz);

        // damping
        if (hasElement(axisElement, "damping"))
        {
            double damping = getValueDouble(axisElement, "damping");
            newRevoluteJoint->setDampingCoefficient(0, damping);
        }

        // limit
        if (hasElement(axisElement, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axisElement, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newRevoluteJoint->getGenCoord(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newRevoluteJoint->getGenCoord(0)->set_qMax(upper);
            }
        }
    }
    else
    {
        assert(0);
    }

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_revoluteJointElement, "init_pos"))
    {
        double init_pos = getValueDouble(_revoluteJointElement, "init_pos");
        Eigen::VectorXd ipos = Eigen::VectorXd(1);
        ipos << init_pos;
        newRevoluteJoint->set_q(ipos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_revoluteJointElement, "init_vel"))
    {
        double init_vel = getValueDouble(_revoluteJointElement, "init_vel");
        Eigen::VectorXd ivel = Eigen::VectorXd(1);
        ivel << init_vel;
        newRevoluteJoint->set_q(ivel);
    }

    return newRevoluteJoint;
}

dynamics::PrismaticJoint* readPrismaticJoint(
        tinyxml2::XMLElement* _prismaticJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_prismaticJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::PrismaticJoint* newPrismaticJoint = new dynamics::PrismaticJoint;

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_prismaticJointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_prismaticJointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newPrismaticJoint->setAxis(xyz);

        // damping
        if (hasElement(axisElement, "damping"))
        {
            double damping = getValueDouble(axisElement, "damping");
            newPrismaticJoint->setDampingCoefficient(0, damping);
        }

        // limit
        if (hasElement(axisElement, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axisElement, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newPrismaticJoint->getGenCoord(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newPrismaticJoint->getGenCoord(0)->set_qMax(upper);
            }
        }
    }
    else
    {
        assert(0);
    }

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_prismaticJointElement, "init_pos"))
    {
        double init_pos = getValueDouble(_prismaticJointElement, "init_pos");
        Eigen::VectorXd ipos = Eigen::VectorXd(1);
        ipos << init_pos;
        newPrismaticJoint->set_q(ipos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_prismaticJointElement, "init_vel"))
    {
        double init_vel = getValueDouble(_prismaticJointElement, "init_vel");
        Eigen::VectorXd ivel = Eigen::VectorXd(1);
        ivel << init_vel;
        newPrismaticJoint->set_q(ivel);
    }

    return newPrismaticJoint;
}

dynamics::ScrewJoint* readScrewJoint(
        tinyxml2::XMLElement* _screwJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_screwJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::ScrewJoint* newScrewJoint = new dynamics::ScrewJoint;

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_screwJointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_screwJointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newScrewJoint->setAxis(xyz);

        // pitch
        if (hasElement(axisElement, "pitch"))
        {
            double pitch = getValueDouble(axisElement, "pitch");
            newScrewJoint->setPitch(pitch);
        }

        // damping
        if (hasElement(axisElement, "damping"))
        {
            double damping = getValueDouble(axisElement, "damping");
            newScrewJoint->setDampingCoefficient(0, damping);
        }

        // limit
        if (hasElement(axisElement, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axisElement, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newScrewJoint->getGenCoord(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newScrewJoint->getGenCoord(0)->set_qMax(upper);
            }
        }
    }
    else
    {
        assert(0);
    }

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_screwJointElement, "init_pos"))
    {
        double init_pos = getValueDouble(_screwJointElement, "init_pos");
        Eigen::VectorXd ipos = Eigen::VectorXd(1);
        ipos << init_pos;
        newScrewJoint->set_q(ipos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_screwJointElement, "init_vel"))
    {
        double init_vel = getValueDouble(_screwJointElement, "init_vel");
        Eigen::VectorXd ivel = Eigen::VectorXd(1);
        ivel << init_vel;
        newScrewJoint->set_q(ivel);
    }

    return newScrewJoint;
}

dynamics::UniversalJoint* readUniversalJoint(
        tinyxml2::XMLElement* _universalJointElement,
        dynamics::Skeleton *_skeleton)
{
    assert(_universalJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::UniversalJoint* newUniversalJoint = new dynamics::UniversalJoint;

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_universalJointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_universalJointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newUniversalJoint->setAxis1(xyz);

        // damping
        if (hasElement(axisElement, "damping"))
        {
            double damping = getValueDouble(axisElement, "damping");
            newUniversalJoint->setDampingCoefficient(0, damping);
        }

        // limit
        if (hasElement(axisElement, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axisElement, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newUniversalJoint->getGenCoord(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newUniversalJoint->getGenCoord(0)->set_qMax(upper);
            }
        }
    }
    else
    {
        assert(0);
    }

    //--------------------------------------------------------------------------
    // axis2
    if (hasElement(_universalJointElement, "axis2"))
    {
        tinyxml2::XMLElement* axis2Element
                = getElement(_universalJointElement, "axis2");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axis2Element, "xyz");
        newUniversalJoint->setAxis2(xyz);

        // damping
        if (hasElement(axis2Element, "damping"))
        {
            double damping = getValueDouble(axis2Element, "damping");
            newUniversalJoint->setDampingCoefficient(1, damping);
        }

        // limit
        if (hasElement(axis2Element, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axis2Element, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newUniversalJoint->getGenCoord(1)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newUniversalJoint->getGenCoord(1)->set_qMax(upper);
            }
        }
    }
    else
    {
        assert(0);
    }

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_universalJointElement, "init_pos"))
    {
        Eigen::Vector2d init_pos = getValueVector2d(_universalJointElement, "init_pos");
        newUniversalJoint->set_q(init_pos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_universalJointElement, "init_vel"))
    {
        Eigen::Vector2d init_vel = getValueVector2d(_universalJointElement, "init_vel");
        newUniversalJoint->set_q(init_vel);
    }

    return newUniversalJoint;
}

dynamics::BallJoint* readBallJoint(
        tinyxml2::XMLElement* _ballJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_ballJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::BallJoint* newBallJoint = new dynamics::BallJoint;

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_ballJointElement, "init_pos"))
    {
        Eigen::Vector3d init_pos = getValueVector3d(_ballJointElement, "init_pos");
        newBallJoint->set_q(init_pos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_ballJointElement, "init_vel"))
    {
        Eigen::Vector3d init_vel = getValueVector3d(_ballJointElement, "init_vel");
        newBallJoint->set_q(init_vel);
    }

    return newBallJoint;
}

dynamics::EulerJoint* readEulerJoint(
        tinyxml2::XMLElement* _eulerJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_eulerJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::EulerJoint* newEulerJoint = new dynamics::EulerJoint;

    //--------------------------------------------------------------------------
    // axis order
    std::string order = getValueString(_eulerJointElement, "axis_order");
    if (order == "xyz")
    {
        newEulerJoint->setAxisOrder(dynamics::EulerJoint::AO_XYZ);
    }
    else if (order == "zyx")
    {
        newEulerJoint->setAxisOrder(dynamics::EulerJoint::AO_ZYX);
    }
    else
    {
        dterr << "Undefined Euler axis order\n";
        assert(0);
    }

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_eulerJointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_eulerJointElement, "axis");

        // damping
        if (hasElement(axisElement, "damping"))
        {
            double damping = getValueDouble(axisElement, "damping");
            newEulerJoint->setDampingCoefficient(0, damping);
        }

        // limit
        if (hasElement(axisElement, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axisElement, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newEulerJoint->getGenCoord(0)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newEulerJoint->getGenCoord(0)->set_qMax(upper);
            }
        }
    }

    //--------------------------------------------------------------------------
    // axis2
    if (hasElement(_eulerJointElement, "axis2"))
    {
        tinyxml2::XMLElement* axis2Element
                = getElement(_eulerJointElement, "axis2");

        // damping
        if (hasElement(axis2Element, "damping"))
        {
            double damping = getValueDouble(axis2Element, "damping");
            newEulerJoint->setDampingCoefficient(1, damping);
        }

        // limit
        if (hasElement(axis2Element, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axis2Element, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newEulerJoint->getGenCoord(1)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newEulerJoint->getGenCoord(1)->set_qMax(upper);
            }
        }
    }

    //--------------------------------------------------------------------------
    // axis3
    if (hasElement(_eulerJointElement, "axis3"))
    {
        tinyxml2::XMLElement* axis3Element
                = getElement(_eulerJointElement, "axis3");

        // damping
        if (hasElement(axis3Element, "damping"))
        {
            double damping = getValueDouble(axis3Element, "damping");
            newEulerJoint->setDampingCoefficient(2, damping);
        }

        // limit
        if (hasElement(axis3Element, "limit"))
        {
            tinyxml2::XMLElement* limitElement
                    = getElement(axis3Element, "limit");

            // lower
            if (hasElement(limitElement, "lower"))
            {
                double lower = getValueDouble(limitElement, "lower");
                newEulerJoint->getGenCoord(2)->set_qMin(lower);
            }

            // upper
            if (hasElement(limitElement, "upper"))
            {
                double upper = getValueDouble(limitElement, "upper");
                newEulerJoint->getGenCoord(2)->set_qMax(upper);
            }
        }
    }

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_eulerJointElement, "init_pos"))
    {
        Eigen::Vector3d init_pos = getValueVector3d(_eulerJointElement, "init_pos");
        newEulerJoint->set_q(init_pos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_eulerJointElement, "init_vel"))
    {
        Eigen::Vector3d init_vel = getValueVector3d(_eulerJointElement, "init_vel");
        newEulerJoint->set_q(init_vel);
    }

    return newEulerJoint;
}

dynamics::TranslationalJoint*readTranslationalJoint(
        tinyxml2::XMLElement* _translationalJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_translationalJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::TranslationalJoint* newTranslationalJoint
            = new dynamics::TranslationalJoint;

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_translationalJointElement, "init_pos"))
    {
        Eigen::Vector3d init_pos = getValueVector3d(_translationalJointElement, "init_pos");
        newTranslationalJoint->set_q(init_pos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_translationalJointElement, "init_vel"))
    {
        Eigen::Vector3d init_vel = getValueVector3d(_translationalJointElement, "init_vel");
        newTranslationalJoint->set_q(init_vel);
    }

    return newTranslationalJoint;
}

dynamics::FreeJoint*readFreeJoint(
        tinyxml2::XMLElement* _freeJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_freeJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

    //--------------------------------------------------------------------------
    // init_pos
    if (hasElement(_freeJointElement, "init_pos"))
    {
        Eigen::Vector6d init_pos = getValueVector6d(_freeJointElement, "init_pos");
        newFreeJoint->set_q(init_pos);
    }

    //--------------------------------------------------------------------------
    // init_vel
    if (hasElement(_freeJointElement, "init_vel"))
    {
        Eigen::Vector6d init_vel = getValueVector6d(_freeJointElement, "init_vel");
        newFreeJoint->set_q(init_vel);
    }

    return newFreeJoint;
}

} // namespace utils
} // namespace dart
