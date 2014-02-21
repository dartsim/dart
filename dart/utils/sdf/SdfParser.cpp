#include <map>
#include <iostream>
#include <fstream>

#include "dart/common/Console.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/ScrewJoint.h"
#include "dart/dynamics/TranslationalJoint.h"
#include "dart/dynamics/BallJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/EulerJoint.h"
#include "dart/dynamics/UniversalJoint.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/SkelParser.h"
#include "dart/utils/Paths.h"
#include "dart/utils/sdf/SdfParser.h"

namespace dart {
namespace utils {

simulation::World* SdfParser::readSdfFile(const std::string& _filename)
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
    {
        dterr << "The file format of ["
          << _filename
          << "] is not sdf 1.4. Please try with sdf 1.4." << std::endl;
        return NULL;
    }

    //--------------------------------------------------------------------------
    // Load World
    tinyxml2::XMLElement* worldElement = NULL;
    worldElement = sdfElement->FirstChildElement("world");
    if (worldElement == NULL)
        return NULL;

    // Change path to a Unix-style path if given a Windows one
    // Windows can handle Unix-style paths (apparently)
    std::string unixFileName = _filename;
    std::replace(unixFileName.begin(), unixFileName.end(), '\\' , '/' );
    std::string skelPath = unixFileName.substr(0, unixFileName.rfind("/") + 1);

    simulation::World* newWorld = readWorld(worldElement, skelPath);

    return newWorld;
}

dart::dynamics::Skeleton* SdfParser::readSkeleton(const std::string& _filename)
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
  // Load sdf
  tinyxml2::XMLElement* sdfElement = NULL;
  sdfElement = _dartFile.FirstChildElement("sdf");
  if (sdfElement == NULL)
      return NULL;

  //--------------------------------------------------------------------------
  // version attribute
  std::string version = getAttribute(sdfElement, "version");
  // We support 1.4 only for now.
  if (version != "1.4")
  {
      dterr << "The file format of ["
        << _filename
        << "] is not sdf 1.4. Please try with sdf 1.4." << std::endl;
      return NULL;
  }

  //--------------------------------------------------------------------------
  // Load skeleton
  tinyxml2::XMLElement* skelElement = NULL;
  skelElement = sdfElement->FirstChildElement("model");
  if (skelElement == NULL)
      return NULL;

  // Change path to a Unix-style path if given a Windows one
  // Windows can handle Unix-style paths (apparently)
  std::string unixFileName = _filename;
  std::replace(unixFileName.begin(), unixFileName.end(), '\\' , '/' );
  std::string skelPath = unixFileName.substr(0, unixFileName.rfind("/") + 1);

  dynamics::Skeleton* newSkeleton = readSkeleton(skelElement, skelPath);

  return newSkeleton;
}

simulation::World* SdfParser::readWorld(tinyxml2::XMLElement* _worldElement,
                                        const std::string& _skelPath)
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
                = readSkeleton(skeletonElements.get(), _skelPath);

        newWorld->addSkeleton(newSkeleton);
    }

    return newWorld;
}

void SdfParser::readPhysics(tinyxml2::XMLElement* _physicsElement,
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

dynamics::Skeleton* SdfParser::readSkeleton(
    tinyxml2::XMLElement* _skeletonElement, const std::string& _skelPath)
{
    assert(_skeletonElement != NULL);

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
        bool isStatic= getValueBool(_skeletonElement, "static");
        newSkeleton->setMobile(!isStatic);
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
    std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> > sdfBodyNodes;
    while (bodies.next())
    {
        SDFBodyNode newSDFBodyNode
                = readBodyNode(bodies.get(), newSkeleton, skeletonFrame,
                               _skelPath);
        assert(newSDFBodyNode.bodyNode);
        sdfBodyNodes.push_back(newSDFBodyNode);
    }

    //--------------------------------------------------------------------------
    // Joints
    ElementEnumerator joints(_skeletonElement, "joint");
    while (joints.next())
    {
        readJoint(joints.get(), sdfBodyNodes);
    }

    //--------------------------------------------------------------------------
    // Add FreeJoint to the body node that doesn't have parent joint
    for (unsigned int i = 0; i < sdfBodyNodes.size(); ++i)
    {
        dynamics::BodyNode* bodyNode = sdfBodyNodes[i].bodyNode;

        if (bodyNode->getParentJoint() == NULL)
        {
            // If this link has no parent joint, then we add 6-dof free joint.
            dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

            newFreeJoint->setTransformFromParentBodyNode(
                        bodyNode->getWorldTransform());
            newFreeJoint->setTransformFromChildBodyNode(
                        Eigen::Isometry3d::Identity());

            bodyNode->setParentJoint(newFreeJoint);
        }
    }

    for (std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> >::iterator it = sdfBodyNodes.begin();
         it != sdfBodyNodes.end(); ++it)
        newSkeleton->addBodyNode((*it).bodyNode);

    return newSkeleton;
}

SdfParser::SDFBodyNode SdfParser::readBodyNode(
        tinyxml2::XMLElement* _bodyNodeElement,
        dynamics::Skeleton* _skeleton,
        const Eigen::Isometry3d& _skeletonFrame,
        const std::string& _skelPath)
{
    assert(_bodyNodeElement != NULL);
    assert(_skeleton != NULL);

    dynamics::BodyNode* newBodyNode = new dynamics::BodyNode;
    Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

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
        initTransform = _skeletonFrame * W;
    }
    else
    {
        initTransform = _skeletonFrame;
    }

    //--------------------------------------------------------------------------
    // visual
    ElementEnumerator vizShapes(_bodyNodeElement, "visual");
    while (vizShapes.next())
    {
        dynamics::Shape* newShape
                = readShape(vizShapes.get(), _skelPath);
        if (newShape)
            newBodyNode->addVisualizationShape(newShape);
    }

    //--------------------------------------------------------------------------
    // collision
    ElementEnumerator collShapes(_bodyNodeElement, "collision");
    while (collShapes.next())
    {
        dynamics::Shape* newShape
                = readShape(collShapes.get(), _skelPath);

        if (newShape)
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
            Eigen::Isometry3d T = getValueIsometry3d(inertiaElement, "pose");
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

    SDFBodyNode sdfBodyNode;
    sdfBodyNode.bodyNode = newBodyNode;
    sdfBodyNode.initTransform = initTransform;

    return sdfBodyNode;
}

dynamics::Shape* SdfParser::readShape(tinyxml2::XMLElement* _shapelement,
                                      const std::string& _skelPath)
{
    dynamics::Shape* newShape = NULL;

    // type
    assert(hasElement(_shapelement, "geometry"));
    tinyxml2::XMLElement* geometryElement = getElement(_shapelement, "geometry");

    if (hasElement(geometryElement, "box"))
    {
        tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

        Eigen::Vector3d size = getValueVector3d(boxElement, "size");

        newShape = new dynamics::BoxShape(size);
    }
    else if (hasElement(geometryElement, "sphere"))
    {
        tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement, "sphere");

        double radius = getValueDouble(ellipsoidElement, "radius");
        Eigen::Vector3d size(radius * 2, radius * 2, radius * 2);

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
    else if (hasElement(geometryElement, "mesh"))
    {
      tinyxml2::XMLElement* meshEle = getElement(geometryElement, "mesh");
      // TODO(JS): We assume that uri is just file name for the mesh
      std::string           uri     = getValueString(meshEle, "uri");
      Eigen::Vector3d       scale   = getValueVector3d(meshEle, "scale");
      const aiScene* model = dynamics::MeshShape::loadMesh(_skelPath + uri);
      if (model)
        newShape = new dynamics::MeshShape(scale, model);
      else
        dterr << "Fail to load model[" << uri << "]." << std::endl;
    }
    else
    {
        std::cout << "Invalid shape type." << std::endl;
        return NULL;
    }

    // pose
    if (hasElement(_shapelement, "pose"))
    {
        Eigen::Isometry3d W = getValueIsometry3d(_shapelement, "pose");
        newShape->setLocalTransform(W);
    }

    return newShape;
}

dynamics::Joint* SdfParser::readJoint(tinyxml2::XMLElement* _jointElement,
                            const std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> >& _sdfBodyNodes)
{
    assert(_jointElement != NULL);

    dynamics::Joint* newJoint = NULL;

    //--------------------------------------------------------------------------
    // Type attribute
    std::string type = getAttribute(_jointElement, "type");
    assert(!type.empty());
    if (type == std::string("prismatic"))
        newJoint = readPrismaticJoint(_jointElement);
    if (type == std::string("revolute"))
        newJoint = readRevoluteJoint(_jointElement);
    if (type == std::string("screw"))
        newJoint = readScrewJoint(_jointElement);
    if (type == std::string("revolute2"))
        newJoint = readUniversalJoint(_jointElement);
    if (type == std::string("ball"))
        newJoint = readBallJoint(_jointElement);
    assert(newJoint != NULL);

    //--------------------------------------------------------------------------
    // Name attribute
    std::string name = getAttribute(_jointElement, "name");
    newJoint->setName(name);

    //--------------------------------------------------------------------------
    // parent
    SDFBodyNode sdfParentBodyNode;
    sdfParentBodyNode.bodyNode = NULL;
    sdfParentBodyNode.initTransform = Eigen::Isometry3d::Identity();

    if (hasElement(_jointElement, "parent"))
    {
        std::string strParent = getValueString(_jointElement, "parent");

        if (strParent != std::string("world"))
        {
            for (std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> >::const_iterator it =
                 _sdfBodyNodes.begin(); it != _sdfBodyNodes.end(); ++it)
                if ((*it).bodyNode->getName() == strParent)
                {
                    sdfParentBodyNode = (*it);
                    break;
                }

            if (sdfParentBodyNode.bodyNode == NULL)
            {
                dterr << "Can't find the parent body ["
                  << strParent
                  << "] of the joint ["
                  << newJoint->getName()
                  << "]. " << std::endl;
                assert(0);
            }
        }
    }
    else
    {
        dterr << "Set parent body node for " << newJoint->getName() << "."
              << std::endl;
        assert(0);
    }

    //--------------------------------------------------------------------------
    // child
    SDFBodyNode sdfChildBodyNode;
    sdfChildBodyNode.bodyNode = NULL;
    sdfChildBodyNode.initTransform = Eigen::Isometry3d::Identity();

    if (hasElement(_jointElement, "child"))
    {
        std::string strChild = getValueString(_jointElement, "child");

        for (std::vector<SDFBodyNode, Eigen::aligned_allocator<SDFBodyNode> >::const_iterator it =
             _sdfBodyNodes.begin(); it != _sdfBodyNodes.end(); ++it)
        {
            if ((*it).bodyNode->getName() == strChild)
            {
                sdfChildBodyNode = (*it);
                break;
            }
        }

        if (sdfChildBodyNode.bodyNode == NULL)
        {
            dterr << "Can't find the child body ["
              << strChild
              << "] of the joint ["
              << newJoint->getName()
              << "]. " << std::endl;
            assert(0);
        }
    }
    else
    {
        dterr << "Set child body node for " << newJoint->getName() << "."
              << std::endl;
        assert(0);
    }

    sdfChildBodyNode.bodyNode->setParentJoint(newJoint);

    if (sdfParentBodyNode.bodyNode)
        sdfParentBodyNode.bodyNode->addChildBodyNode(sdfChildBodyNode.bodyNode);

    //--------------------------------------------------------------------------
    // transformation
    Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d childWorld = sdfChildBodyNode.initTransform;
    if (sdfParentBodyNode.bodyNode)
         parentWorld = sdfParentBodyNode.initTransform;
    if (hasElement(_jointElement, "pose"))
        childToJoint = getValueIsometry3d(_jointElement, "pose");
    Eigen::Isometry3d parentToJoint = parentWorld.inverse()*childWorld*childToJoint;
    newJoint->setTransformFromChildBodyNode(childToJoint);
    newJoint->setTransformFromParentBodyNode(parentToJoint);

    return newJoint;
}

dynamics::WeldJoint* SdfParser::readWeldJoint(tinyxml2::XMLElement* _jointElement)
{
    assert(_jointElement != NULL);

    dynamics::WeldJoint* newWeldJoint = new dynamics::WeldJoint;

    return newWeldJoint;
}

dynamics::RevoluteJoint* SdfParser::readRevoluteJoint(
        tinyxml2::XMLElement* _revoluteJointElement)
{
    assert(_revoluteJointElement != NULL);

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
        if (hasElement(axisElement, "dynamics"))
        {
            tinyxml2::XMLElement* dynamicsElement
                    = getElement(axisElement, "dynamics");

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

dynamics::PrismaticJoint* SdfParser::readPrismaticJoint(
        tinyxml2::XMLElement* _jointElement)
{
    assert(_jointElement != NULL);

    dynamics::PrismaticJoint* newPrismaticJoint = new dynamics::PrismaticJoint;

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_jointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_jointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newPrismaticJoint->setAxis(xyz);

        // dynamics
        if (hasElement(_jointElement, "dynamics"))
        {
            tinyxml2::XMLElement* dynamicsElement
                    = getElement(_jointElement, "dynamics");

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

dynamics::ScrewJoint* SdfParser::readScrewJoint(
        tinyxml2::XMLElement* _jointElement)
{
    assert(_jointElement != NULL);

    dynamics::ScrewJoint* newScrewJoint = new dynamics::ScrewJoint;

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_jointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_jointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newScrewJoint->setAxis(xyz);

        // dynamics
        if (hasElement(_jointElement, "dynamics"))
        {
            tinyxml2::XMLElement* dynamicsElement
                    = getElement(_jointElement, "dynamics");

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

    // pitch
    if (hasElement(_jointElement, "thread_pitch"))
    {
        double pitch = getValueDouble(_jointElement, "thread_pitch");
        newScrewJoint->setPitch(pitch);
    }

    return newScrewJoint;
}

dynamics::UniversalJoint* SdfParser::readUniversalJoint(
        tinyxml2::XMLElement* _jointElement)
{
    assert(_jointElement != NULL);

    dynamics::UniversalJoint* newUniversalJoint = new dynamics::UniversalJoint;

    //--------------------------------------------------------------------------
    // axis
    if (hasElement(_jointElement, "axis"))
    {
        tinyxml2::XMLElement* axisElement
                = getElement(_jointElement, "axis");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
        newUniversalJoint->setAxis1(xyz);

        // dynamics
        if (hasElement(_jointElement, "dynamics"))
        {
            tinyxml2::XMLElement* dynamicsElement
                    = getElement(_jointElement, "dynamics");

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
    if (hasElement(_jointElement, "axis2"))
    {
        tinyxml2::XMLElement* axis2Element
                = getElement(_jointElement, "axis2");

        // xyz
        Eigen::Vector3d xyz = getValueVector3d(axis2Element, "xyz");
        newUniversalJoint->setAxis2(xyz);

        // dynamics
        if (hasElement(_jointElement, "dynamics"))
        {
            tinyxml2::XMLElement* dynamicsElement
                    = getElement(_jointElement, "dynamics");

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

dynamics::BallJoint* SdfParser::readBallJoint(
        tinyxml2::XMLElement* _jointElement)
{
    assert(_jointElement != NULL);

    dynamics::BallJoint* newBallJoint = new dynamics::BallJoint;

    return newBallJoint;
}

dynamics::TranslationalJoint* SdfParser::readTranslationalJoint(
        tinyxml2::XMLElement* _jointElement)
{
    assert(_jointElement != NULL);

    dynamics::TranslationalJoint* newTranslationalJoint
            = new dynamics::TranslationalJoint;

    return newTranslationalJoint;
}

dynamics::FreeJoint* SdfParser::readFreeJoint(
        tinyxml2::XMLElement* _jointElement)
{
    assert(_jointElement != NULL);

    dynamics::FreeJoint* newFreeJoint = new dynamics::FreeJoint;

    return newFreeJoint;
}

} // namespace utils
} // namespace dart
