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
    // TODO: We need version aware SDF parser (see #264)
    // We support 1.4 only for now.
    if (version != "1.4" && version != "1.5")
    {
      dterr << "The file format of ["
            << _filename
            << "] is not sdf 1.4 or 1.5."
            << std::endl;
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

dynamics::SkeletonPtr SdfParser::readSkeleton(const std::string& _filename)
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
      return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load sdf
  tinyxml2::XMLElement* sdfElement = nullptr;
  sdfElement = _dartFile.FirstChildElement("sdf");
  if (sdfElement == nullptr)
      return nullptr;

  //--------------------------------------------------------------------------
  // version attribute
  std::string version = getAttribute(sdfElement, "version");
  // TODO: We need version aware SDF parser (see #264)
  // We support 1.4 only for now.
  if (version != "1.4" && version != "1.5")
  {
    dterr << "The file format of ["
          << _filename
          << "] is not sdf 1.4 or 1.5."
          << std::endl;
    return nullptr;
  }
  //--------------------------------------------------------------------------
  // Load skeleton
  tinyxml2::XMLElement* skelElement = nullptr;
  skelElement = sdfElement->FirstChildElement("model");
  if (skelElement == nullptr)
      return nullptr;

  // Change path to a Unix-style path if given a Windows one
  // Windows can handle Unix-style paths (apparently)
  std::string unixFileName = _filename;
  std::replace(unixFileName.begin(), unixFileName.end(), '\\' , '/' );
  std::string skelPath = unixFileName.substr(0, unixFileName.rfind("/") + 1);

  dynamics::SkeletonPtr newSkeleton = readSkeleton(skelElement, skelPath);

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
        dynamics::SkeletonPtr newSkeleton
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

dynamics::SkeletonPtr SdfParser::readSkeleton(
    tinyxml2::XMLElement* _skeletonElement, const std::string& _skelPath)
{
  assert(_skeletonElement != nullptr);

  dynamics::SkeletonPtr newSkeleton(new dynamics::Skeleton);
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
      Eigen::Isometry3d W = getValueIsometry3dWithExtrinsicRotation(_skeletonElement, "pose");
      skeletonFrame = W;
  }

  //--------------------------------------------------------------------------
  // Bodies
  ElementEnumerator bodies(_skeletonElement, "link");
  BodyMap sdfBodyNodes;
  while (bodies.next())
  {
    SDFBodyNode body = readBodyNode(
          bodies.get(), newSkeleton, skeletonFrame, _skelPath);

    BodyMap::iterator it = sdfBodyNodes.find(body.properties.mName);
    if(it != sdfBodyNodes.end())
    {
      dtwarn << "[SdfParser::readSkeleton] Duplicate name in file: "
             << body.properties.mName << "\n"
             << "Every Link must have a unique name!\n";
      continue;
    }

    sdfBodyNodes[body.properties.mName] = body;
  }

  //--------------------------------------------------------------------------
  // Joints
  JointMap sdfJoints;
  ElementEnumerator joints(_skeletonElement, "joint");
  while (joints.next())
  {
    SDFJoint joint = readJoint(joints.get(), sdfBodyNodes, skeletonFrame);

    if(joint.childName.empty())
    {
      dterr << "[SdfParser::readSkeleton] Joint named ["
            << joint.properties->mName << "] does not have a valid child "
            << "Link, so it will not be added to the Skeleton\n";
      continue;
    }

    JointMap::iterator it = sdfJoints.find(joint.childName);
    if(it != sdfJoints.end())
    {
      dterr << "[SdfParser::readSkeleton] Joint named ["
            << joint.properties->mName << "] is claiming Link ["
            << joint.childName << "] as its child, but that is already "
            << "claimed by Joint [" << it->second.properties->mName
            << "]. Joint [" << joint.properties->mName
            << "] will be discarded\n";
      continue;
    }

    sdfJoints[joint.childName] = joint;
  }

  JointMap::iterator it = sdfJoints.begin();
  while(it != sdfJoints.end())
  {
    const SDFJoint& joint = it->second;
    dynamics::BodyNode* parent = newSkeleton->getBodyNode(joint.parentName);
    if(nullptr == parent
       && joint.parentName != "world" && !joint.parentName.empty())
    {
      it = sdfJoints.find(joint.parentName);
      if(it == sdfJoints.end())
      {
        dterr << "[SdfParser::readSkeleton] Searching for parent Joint of "
              << "Link [" << joint.parentName
              << "] yielded no results. This should not "
              << "be possible! We will now quit parsing. "
              << "Please report this bug!\n";
      }
      break;
    }

    BodyMap::iterator child = sdfBodyNodes.find(joint.childName);
    if(child == sdfBodyNodes.end())
    {
      dterr << "[SdfParser::readSkeleton] Could not find Link named ["
            << joint.childName << "] requested as child of Joint ["
            << joint.properties->mName << "]. This should not be possible! "
            << "We will now quit parsing. Please report this bug!\n";
      break;
    }


    const std::string& type = joint.type;
    if (std::string("prismatic") == type)
      newSkeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(parent,
            static_cast<const dynamics::PrismaticJoint::Properties&>(*joint.properties),
            child->second.properties);
    if (std::string("revolute") == type)
      newSkeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(parent,
            static_cast<const dynamics::RevoluteJoint::Properties&>(*joint.properties),
            child->second.properties);
    if (std::string("screw") == type)
      newSkeleton->createJointAndBodyNodePair<dynamics::ScrewJoint>(parent,
            static_cast<const dynamics::ScrewJoint::Properties&>(*joint.properties),
            child->second.properties);
    if (std::string("revolute2") == type)
      newSkeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(parent,
            static_cast<const dynamics::UniversalJoint::Properties&>(*joint.properties),
            child->second.properties);
    if (std::string("ball") == type)
      newSkeleton->createJointAndBodyNodePair<dynamics::BallJoint>(parent,
            static_cast<const dynamics::BallJoint::Properties&>(*joint.properties),
            child->second.properties);
  }

  return newSkeleton;
}

SdfParser::SDFBodyNode SdfParser::readBodyNode(
        tinyxml2::XMLElement* _bodyNodeElement,
        dynamics::SkeletonPtr _skeleton,
        const Eigen::Isometry3d& _skeletonFrame,
        const std::string& _skelPath)
{
  assert(_bodyNodeElement != nullptr);
  assert(_skeleton != nullptr);

  dynamics::BodyNode::Properties properties;
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  std::string name = getAttribute(_bodyNodeElement, "name");
  properties.mName = name;

  //--------------------------------------------------------------------------
  // gravity
  if (hasElement(_bodyNodeElement, "gravity"))
  {
    bool gravityMode = getValueBool(_bodyNodeElement, "gravity");
    properties.mGravityMode = gravityMode;
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
    Eigen::Isometry3d W = getValueIsometry3dWithExtrinsicRotation(_bodyNodeElement, "pose");
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
    dynamics::ShapePtr newShape(readShape(vizShapes.get(), _skelPath));
    if (newShape)
      properties.mVizShapes.push_back(newShape);
  }

  //--------------------------------------------------------------------------
  // collision
  ElementEnumerator collShapes(_bodyNodeElement, "collision");
  while (collShapes.next())
  {
    dynamics::ShapePtr newShape(readShape(collShapes.get(), _skelPath));

    if (newShape)
      properties.mColShapes.push_back(newShape);
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
      properties.mInertia.setMass(mass);
    }

    // offset
    if (hasElement(inertiaElement, "pose"))
    {
      Eigen::Isometry3d T = getValueIsometry3dWithExtrinsicRotation(inertiaElement, "pose");
      properties.mInertia.setLocalCOM(T.translation());
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

      properties.mInertia.setMoment(ixx, iyy, izz, ixy, ixz, iyz);
    }
    else if (properties.mVizShapes.size() > 0
             && properties.mVizShapes[0] != nullptr)
    {
      Eigen::Matrix3d Ic =
          properties.mVizShapes[0]->computeInertia(
            properties.mInertia.getMass());

      properties.mInertia.setMoment(Ic(0,0), Ic(1,1), Ic(2,2),
                                    Ic(0,1), Ic(0,2), Ic(1,2));
    }
  }

  SDFBodyNode sdfBodyNode;
  sdfBodyNode.properties = properties;
  sdfBodyNode.initTransform = initTransform;

  return sdfBodyNode;
}

dynamics::Shape* SdfParser::readShape(tinyxml2::XMLElement* _shapelement,
                                      const std::string& _skelPath)
{
  dynamics::Shape* newShape = nullptr;

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
    return nullptr;
  }

  // pose
  if (hasElement(_shapelement, "pose"))
  {
    Eigen::Isometry3d W = getValueIsometry3dWithExtrinsicRotation(_shapelement, "pose");
    newShape->setLocalTransform(W);
  }

  return newShape;
}

SdfParser::SDFJoint SdfParser::readJoint(tinyxml2::XMLElement* _jointElement,
    const BodyMap& _sdfBodyNodes,
    const Eigen::Isometry3d& _skeletonFrame)
{
  assert(_jointElement != NULL);

  //--------------------------------------------------------------------------
  // Type attribute
  std::string type = getAttribute(_jointElement, "type");
  assert(!type.empty());

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttribute(_jointElement, "name");

  //--------------------------------------------------------------------------
  // parent
  BodyMap::const_iterator parent_it = _sdfBodyNodes.end();

  if (hasElement(_jointElement, "parent"))
  {
    std::string strParent = getValueString(_jointElement, "parent");

    if(strParent != std::string("world"))
    {
      parent_it = _sdfBodyNodes.find(strParent);

      if( parent_it == _sdfBodyNodes.end() )
      {
        dterr << "[SdfParser::readJoint] Cannot find a Link named ["
              << strParent << "] requested as the parent of the Joint named ["
              << name << "]\n";
        assert(0);
      }
    }
  }
  else
  {
    dterr << "[SdfParser::readJoint] You must set parent link for "
          << "the Joint [" << name << "]!\n";
    assert(0);
  }

  //--------------------------------------------------------------------------
  // child
  BodyMap::const_iterator child_it = _sdfBodyNodes.end();

  if (hasElement(_jointElement, "child"))
  {
    std::string strChild = getValueString(_jointElement, "child");

    child_it = _sdfBodyNodes.find(strChild);

    if ( child_it == _sdfBodyNodes.end() )
    {
      dterr << "[SdfParser::readJoint] Cannot find a Link named [" << strChild
            << "] requested as the child of the Joint named [" << name << "]\n";
      assert(0);
    }
  }
  else
  {
    dterr << "[SdfParser::readJoint] You must set the child link for the Joint "
          << "[" << name << "]!\n";
    assert(0);
  }

  SDFJoint newJoint;
  newJoint.parentName = (parent_it == _sdfBodyNodes.end())?
        "" : parent_it->first;
  newJoint.childName = (parent_it == _sdfBodyNodes.end())?
        "" : child_it->first;

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childWorld =  Eigen::Isometry3d::Identity();

  if (parent_it != _sdfBodyNodes.end())
    parentWorld = parent_it->second.initTransform;
  if (child_it != _sdfBodyNodes.end())
    childWorld = child_it->second.initTransform;
  if (hasElement(_jointElement, "pose"))
    childToJoint = getValueIsometry3dWithExtrinsicRotation(_jointElement, "pose");

  Eigen::Isometry3d parentToJoint = parentWorld.inverse()*childWorld*childToJoint;

  // TODO: Workaround!!
  Eigen::Isometry3d jointFrame = childWorld * childToJoint;

  if (type == std::string("prismatic"))
    newJoint.properties = std::make_shared<dynamics::PrismaticJoint::Properties>(
          readPrismaticJoint(_jointElement, _skeletonFrame, jointFrame, name));
  if (type == std::string("revolute"))
    newJoint.properties = std::make_shared<dynamics::RevoluteJoint::Properties>(
          readRevoluteJoint(_jointElement, _skeletonFrame, jointFrame, name));
  if (type == std::string("screw"))
    newJoint.properties = std::make_shared<dynamics::ScrewJoint::Properties>(
          readScrewJoint(_jointElement, _skeletonFrame, jointFrame, name));
  if (type == std::string("revolute2"))
    newJoint.properties = std::make_shared<dynamics::UniversalJoint::Properties>(
          readUniversalJoint(_jointElement, _skeletonFrame, jointFrame, name));
  if (type == std::string("ball"))
    newJoint.properties = std::make_shared<dynamics::BallJoint::Properties>(
          readBallJoint(_jointElement, _skeletonFrame, jointFrame, name));

  newJoint.type = type;

  newJoint.properties->mName = name;

  newJoint.properties->mT_ChildBodyToJoint = childToJoint;
  newJoint.properties->mT_ParentBodyToJoint = parentToJoint;

  return newJoint;
}

static void reportMissingElement(const std::string& functionName,
                                 const std::string& elementName,
                                 const std::string& objectType,
                                 const std::string& objectName)
{
  dterr << "[SdfParser::" << functionName << "] Missing element " << elementName
        << " for " << objectType << " named " << objectName << "\n";
  assert(0);
}

dart::dynamics::WeldJoint::Properties SdfParser::readWeldJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d&,
    const Eigen::Isometry3d&,
    const std::string&)
{
    assert(_jointElement != nullptr);

    return dynamics::WeldJoint::Properties();
}

dynamics::RevoluteJoint::Properties SdfParser::readRevoluteJoint(
    tinyxml2::XMLElement* _revoluteJointElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const Eigen::Isometry3d& _jointFrame,
    const std::string& _name)
{
  assert(_revoluteJointElement != nullptr);

  dynamics::RevoluteJoint::Properties newRevoluteJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_revoluteJointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement
            = getElement(_revoluteJointElement, "axis");

    // use_parent_model_frame
    bool useParentModelFrame = false;
    if (hasElement(axisElement, "use_parent_model_frame"))
      useParentModelFrame = getValueBool(axisElement, "use_parent_model_frame");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    if (useParentModelFrame)
    {
      xyz = _jointFrame.linear().inverse() * _skeletonFrame.linear() * xyz;
    }
    newRevoluteJoint.mAxis = xyz;

    // dynamics
    if (hasElement(axisElement, "dynamics"))
    {
      tinyxml2::XMLElement* dynamicsElement
              = getElement(axisElement, "dynamics");

      // damping
      if (hasElement(dynamicsElement, "damping"))
      {
        double damping = getValueDouble(dynamicsElement, "damping");
        newRevoluteJoint.mDampingCoefficient = damping;
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
        newRevoluteJoint.mPositionLowerLimit = lower;
      }

      // upper
      if (hasElement(limitElement, "upper"))
      {
        double upper = getValueDouble(limitElement, "upper");
        newRevoluteJoint.mPositionUpperLimit = upper;
      }
    }
  }
  else
  {
    reportMissingElement("readRevoluteJoint", "axis", "joint", _name);
  }

  return newRevoluteJoint;
}

dynamics::PrismaticJoint::Properties SdfParser::readPrismaticJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const Eigen::Isometry3d& _jointFrame,
    const std::string& _name)
{
  assert(_jointElement != NULL);

  dynamics::PrismaticJoint::Properties newPrismaticJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement
            = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newPrismaticJoint.mAxis = xyz;

    // dynamics
    if (hasElement(_jointElement, "dynamics"))
    {
      tinyxml2::XMLElement* dynamicsElement
              = getElement(_jointElement, "dynamics");

      // damping
      if (hasElement(dynamicsElement, "damping"))
      {
        double damping = getValueDouble(dynamicsElement, "damping");
        newPrismaticJoint.mDampingCoefficient = damping;
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
        newPrismaticJoint.mPositionLowerLimit = lower;
      }

      // upper
      if (hasElement(limitElement, "upper"))
      {
        double upper = getValueDouble(limitElement, "upper");
        newPrismaticJoint.mPositionUpperLimit = upper;
      }
    }
  }
  else
  {
    reportMissingElement("readPrismaticJoint", "axis", "joint", _name);
  }

  return newPrismaticJoint;
}

dynamics::ScrewJoint::Properties SdfParser::readScrewJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const Eigen::Isometry3d& _jointFrame,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::ScrewJoint::Properties newScrewJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement
            = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newScrewJoint.mAxis = xyz;

    // dynamics
    if (hasElement(_jointElement, "dynamics"))
    {
      tinyxml2::XMLElement* dynamicsElement
              = getElement(_jointElement, "dynamics");

      // damping
      if (hasElement(dynamicsElement, "damping"))
      {
          double damping = getValueDouble(dynamicsElement, "damping");
          newScrewJoint.mDampingCoefficient = damping;
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
          newScrewJoint.mPositionLowerLimit = lower;
      }

      // upper
      if (hasElement(limitElement, "upper"))
      {
          double upper = getValueDouble(limitElement, "upper");
          newScrewJoint.mPositionUpperLimit = upper;
      }
    }
  }
  else
  {
    reportMissingElement("readScrewJoint", "axis", "joint", _name);
  }

  // pitch
  if (hasElement(_jointElement, "thread_pitch"))
  {
      double pitch = getValueDouble(_jointElement, "thread_pitch");
      newScrewJoint.mPitch = pitch;
  }

  return newScrewJoint;
}

dynamics::UniversalJoint::Properties SdfParser::readUniversalJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d& _skeletonFrame,
    const Eigen::Isometry3d& _jointFrame,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::UniversalJoint::Properties newUniversalJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement
            = getElement(_jointElement, "axis");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
    newUniversalJoint.mAxis[0] = xyz;

    // dynamics
    if (hasElement(_jointElement, "dynamics"))
    {
      tinyxml2::XMLElement* dynamicsElement
              = getElement(_jointElement, "dynamics");

      // damping
      if (hasElement(dynamicsElement, "damping"))
      {
        double damping = getValueDouble(dynamicsElement, "damping");
        newUniversalJoint.mDampingCoefficient[0] = damping;
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
        newUniversalJoint.mPositionLowerLimits[0] = lower;
      }

      // upper
      if (hasElement(limitElement, "upper"))
      {
        double upper = getValueDouble(limitElement, "upper");
        newUniversalJoint.mPositionUpperLimits[0] = upper;
      }
    }
  }
  else
  {
    reportMissingElement("readUniversalJoint", "axis", "joint", _name);
  }

  //--------------------------------------------------------------------------
  // axis2
  if (hasElement(_jointElement, "axis2"))
  {
    tinyxml2::XMLElement* axis2Element
            = getElement(_jointElement, "axis2");

    // xyz
    Eigen::Vector3d xyz = getValueVector3d(axis2Element, "xyz");
    newUniversalJoint.mAxis[1] = xyz;

    // dynamics
    if (hasElement(_jointElement, "dynamics"))
    {
      tinyxml2::XMLElement* dynamicsElement
              = getElement(_jointElement, "dynamics");

      // damping
      if (hasElement(dynamicsElement, "damping"))
      {
        double damping = getValueDouble(dynamicsElement, "damping");
        newUniversalJoint.mDampingCoefficient[1] = damping;
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
        newUniversalJoint.mPositionLowerLimits[1] = lower;
      }

      // upper
      if (hasElement(limitElement, "upper"))
      {
        double upper = getValueDouble(limitElement, "upper");
        newUniversalJoint.mPositionUpperLimits[1] = upper;
      }
    }
  }
  else
  {
    reportMissingElement("readUniversalJoint", "axis2", "joint", _name);
  }

  return newUniversalJoint;
}

dynamics::BallJoint::Properties SdfParser::readBallJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d&,
    const Eigen::Isometry3d&,
    const std::string&)
{
  assert(_jointElement != nullptr);

  return dynamics::BallJoint::Properties();
}

dynamics::TranslationalJoint::Properties SdfParser::readTranslationalJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d&,
    const Eigen::Isometry3d&,
    const std::string&)
{
  assert(_jointElement != nullptr);

  return dynamics::TranslationalJoint::Properties();
}

dynamics::FreeJoint::Properties SdfParser::readFreeJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d&,
    const Eigen::Isometry3d&,
    const std::string&)
{
  assert(_jointElement != nullptr);

  return dynamics::FreeJoint::Properties();
}

} // namespace utils
} // namespace dart
