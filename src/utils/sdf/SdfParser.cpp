#include <map>
#include <iostream>
#include <fstream>

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
#include "utils/sdf/SdfParser.h"

namespace dart {
namespace utils {

simulation::World* readSdfFile(const std::string& _filename)
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
    tinyxml2::XMLElement* sdfElement = NULL;
    sdfElement = _dartFile.FirstChildElement("sdf");
    if (sdfElement == NULL)
        return NULL;

    //--------------------------------------------------------------------------
    // version attribute
    std::string version = getAttribute(sdfElement, "version");
    // We support 1.4 only for now.
    if (version != "1.4")
        return NULL;

    //--------------------------------------------------------------------------
    // Load World
    tinyxml2::XMLElement* worldElement = NULL;
    worldElement = sdfElement->FirstChildElement("world");
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
    // Name attribute
    std::string name = getAttribute(_worldElement, "name");
    // World don't have name.
    //newWorld->setName(name);

    //--------------------------------------------------------------------------
    // Load physics
    if (hasElement(_worldElement, "physics"))
    {
        tinyxml2::XMLElement* physicsElement = _worldElement->FirstChildElement("physics");
        readPhysics(physicsElement, newWorld);
    }

    //--------------------------------------------------------------------------
    // Load skeletons
    ElementEnumerator skeletonElements(_worldElement, "model");
    while (skeletonElements.next())
    {
        dynamics::Skeleton* newSkeleton
                = readSkeleton(skeletonElements.get(), newWorld);

        newWorld->addSkeleton(newSkeleton);
    }

    return newWorld;
}

void readPhysics(tinyxml2::XMLElement* _physicsElement,
                 simulation::World* _world)
{
    // Type attribute
//    std::string physicsEngineName = getAttribute(_physicsElement, "type");

    // Time step
    if (hasElement(_physicsElement, "max_step_size"))
    {
        double timeStep = getValueDouble(_physicsElement, "max_step_size");
        _world->setTimeStep(timeStep);
    }

    // Number of max contacts
//    if (hasElement(_physicsElement, "max_contacts"))
//    {
//        int timeStep = getValueInt(_physicsElement, "max_contacts");
//        _world->setMaxNumContacts(timeStep);
//    }

    // Gravity
    if (hasElement(_physicsElement, "gravity"))
    {
        Eigen::Vector3d gravity = getValueVector3d(_physicsElement, "gravity");
        _world->setGravity(gravity);
    }
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
    // immobile attribute
    if (hasElement(_skeletonElement, "static"))
    {
        //bool immobile = getValueBool(_skeletonElement, "static");
        std::string immobile = getValueString(_skeletonElement, "static");

        if (immobile == "true")
        {
            newSkeleton->setImmobileState(true);
        }
        else if (immobile == "false")
        {
            newSkeleton->setImmobileState(false);
        }
        else
        {
            dterr << "Unkn shape.\n";
            assert(0);
        }
    }

    //--------------------------------------------------------------------------
    // transformation
    if (hasElement(_skeletonElement, "pose"))
    {
        Eigen::Isometry3d W = getValueIsometry3d(_skeletonElement, "pose");
        skeletonFrame = W;
    }

    //--------------------------------------------------------------------------
    // Bodies
    ElementEnumerator bodies(_skeletonElement, "link");
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

    //--------------------------------------------------------------------------
    // Add FreeJoint to the body node that doesn't have parent joint
    for (unsigned int i = 0; i < newSkeleton->getNumBodyNodes(); ++i)
    {
      dynamics::BodyNode* bodyNode = newSkeleton->getBodyNode(i);

      if (bodyNode->getParentJoint() == NULL)
      {
        // If this link has no parent joint, then we add 6-dof free joint.
        dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

        newFreeJoint->setParentBodyNode(NULL);
        newFreeJoint->setTransformFromParentBodyNode(bodyNode->getWorldTransform());

        newFreeJoint->setChildBodyNode(bodyNode);
        newFreeJoint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());

        newSkeleton->addJoint(newFreeJoint);
      }
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
    if (hasElement(_bodyNodeElement, "pose"))
    {
        Eigen::Isometry3d W = getValueIsometry3d(_bodyNodeElement, "pose");
        newBodyNode->setWorldTransform(_skeletonFrame * W);
    }

    //--------------------------------------------------------------------------
    // visual
    ElementEnumerator vizShapes(_bodyNodeElement, "visual");
    while (vizShapes.next())
    {
        dynamics::Shape* newShape
                = readShape(vizShapes.get(), newBodyNode);

        newBodyNode->addVisualizationShape(newShape);
    }

    //--------------------------------------------------------------------------
    // collision
    ElementEnumerator collShapes(_bodyNodeElement, "collision");
    while (collShapes.next())
    {
        dynamics::Shape* newShape
                = readShape(collShapes.get(), newBodyNode);

        newBodyNode->addCollisionShape(newShape);
    }

    //--------------------------------------------------------------------------
    // inertia
    if (hasElement(_bodyNodeElement, "inertial"))
    {
        tinyxml2::XMLElement* inertiaElement = getElement(_bodyNodeElement, "inertial");

        // mass
        if (hasElement(inertiaElement, "mass"))
        {
            double mass = getValueDouble(inertiaElement, "mass");
            newBodyNode->setMass(mass);
        }

        // offset
        if (hasElement(inertiaElement, "pose"))
        {
            Eigen::Isometry3d T = getValueIsometry3d(_bodyNodeElement, "pose");
            newBodyNode->setLocalCOM(T.translation());
        }

        // inertia
        if (hasElement(inertiaElement, "inertia"))
        {
            tinyxml2::XMLElement* moiElement
                    = getElement(inertiaElement, "inertia");

            double ixx = getValueDouble(moiElement, "ixx");
            double iyy = getValueDouble(moiElement, "iyy");
            double izz = getValueDouble(moiElement, "izz");

            double ixy = getValueDouble(moiElement, "ixy");
            double ixz = getValueDouble(moiElement, "ixz");
            double iyz = getValueDouble(moiElement, "iyz");

            newBodyNode->setInertia(ixx, iyy, izz, ixy, ixz, iyz);
        }
        else if (newBodyNode->getVisualizationShape(0) != NULL)
        {
            Eigen::Matrix3d Ic =
                    newBodyNode->getVisualizationShape(0)->computeInertia(
                        newBodyNode->getMass());

            newBodyNode->setInertia(Ic(0,0), Ic(1,1), Ic(2,2),
                                    Ic(0,1), Ic(0,2), Ic(1,2));
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
    else if (hasElement(geometryElement, "sphere"))
    {
        tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement, "sphere");

        Eigen::Vector3d size = getValueVector3d(ellipsoidElement, "size");

        newShape = new dynamics::EllipsoidShape(size);
    }
    else if (hasElement(geometryElement, "cylinder"))
    {
        tinyxml2::XMLElement* cylinderElement = getElement(geometryElement, "cylinder");

        double radius = getValueDouble(cylinderElement, "radius");
        double height = getValueDouble(cylinderElement, "length");

        newShape = new dynamics::CylinderShape(radius, height);
    }
    else if (hasElement(geometryElement, "plane"))
    {
        // TODO: Don't support plane shape yet.
        tinyxml2::XMLElement* planeElement = getElement(geometryElement, "plane");

        Eigen::Vector2d visSize = getValueVector2d(planeElement, "size");
        // TODO: Need to use normal for correct orientation of the plane
        //Eigen::Vector3d normal = getValueVector3d(planeElement, "normal");

        Eigen::Vector3d size(visSize(0), visSize(1), 0.001);

        newShape = new dynamics::BoxShape(size);
    }
    else
    {
        assert(0);
    }

    // pose
    if (hasElement(vizElement, "pose"))
    {
        Eigen::Isometry3d W = getValueIsometry3d(vizElement, "pose");
        newShape->setLocalTransform(W);
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
    if (type == std::string("prismatic"))
        newJoint = readPrismaticJoint(_jointElement, _skeleton);
    if (type == std::string("revolute"))
        newJoint = readRevoluteJoint(_jointElement, _skeleton);
//    if (type == std::string("piston"))
//        newJoint = readScrewJoint(_jointElement, _skeleton);
    if (type == std::string("revolute2"))
        newJoint = readUniversalJoint(_jointElement, _skeleton);
    if (type == std::string("ball"))
        newJoint = readBallJoint(_jointElement, _skeleton);
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
    if (hasElement(_jointElement, "pose"))
        childToJoint = getValueIsometry3d(_jointElement, "pose");
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

        // dynamics
        if (hasElement(_revoluteJointElement, "dynamics"))
        {
            tinyxml2::XMLElement* dynamicsElement
                    = getElement(_revoluteJointElement, "dynamics");

            // damping
            if (hasElement(dynamicsElement, "damping"))
            {
                double damping = getValueDouble(dynamicsElement, "damping");
                newRevoluteJoint->setDampingCoefficient(0, damping);
            }
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

        // dynamics
        if (hasElement(_prismaticJointElement, "dynamics"))
        {
            tinyxml2::XMLElement* dynamicsElement
                    = getElement(_prismaticJointElement, "dynamics");

            // damping
            if (hasElement(dynamicsElement, "damping"))
            {
                double damping = getValueDouble(dynamicsElement, "damping");
                newPrismaticJoint->setDampingCoefficient(0, damping);
            }
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

        // dynamics
        if (hasElement(_screwJointElement, "dynamics"))
        {
            tinyxml2::XMLElement* dynamicsElement
                    = getElement(_screwJointElement, "dynamics");

            // damping
            if (hasElement(dynamicsElement, "damping"))
            {
                double damping = getValueDouble(dynamicsElement, "damping");
                newScrewJoint->setDampingCoefficient(0, damping);
            }
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

        // dynamics
        if (hasElement(_universalJointElement, "dynamics"))
        {
            tinyxml2::XMLElement* dynamicsElement
                    = getElement(_universalJointElement, "dynamics");

            // damping
            if (hasElement(dynamicsElement, "damping"))
            {
                double damping = getValueDouble(dynamicsElement, "damping");
                newUniversalJoint->setDampingCoefficient(0, damping);
            }
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

        // dynamics
        if (hasElement(_universalJointElement, "dynamics"))
        {
            tinyxml2::XMLElement* dynamicsElement
                    = getElement(_universalJointElement, "dynamics");

            // damping
            if (hasElement(dynamicsElement, "damping"))
            {
                double damping = getValueDouble(dynamicsElement, "damping");
                newUniversalJoint->setDampingCoefficient(1, damping);
            }
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
                newUniversalJoint->getGenCoord(0)->set_qMin(lower);
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

    return newUniversalJoint;
}

dynamics::BallJoint* readBallJoint(
        tinyxml2::XMLElement* _ballJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_ballJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::BallJoint* newBallJoint = new dynamics::BallJoint;

    return newBallJoint;
}

dynamics::TranslationalJoint*readTranslationalJoint(
        tinyxml2::XMLElement* _translationalJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_translationalJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::TranslationalJoint* newTranslationalJoint
            = new dynamics::TranslationalJoint;

    return newTranslationalJoint;
}

dynamics::FreeJoint*readFreeJoint(
        tinyxml2::XMLElement* _freeJointElement,
        dynamics::Skeleton* _skeleton)
{
    assert(_freeJointElement != NULL);
    assert(_skeleton != NULL);

    dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

    return newFreeJoint;
}

} // namespace utils
} // namespace dart
