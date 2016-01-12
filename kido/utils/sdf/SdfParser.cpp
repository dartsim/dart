#include <map>
#include <iostream>
#include <fstream>

#include "kido/common/Console.h"
#include "kido/dynamics/BodyNode.h"
#include "kido/dynamics/BoxShape.h"
#include "kido/dynamics/CylinderShape.h"
#include "kido/dynamics/EllipsoidShape.h"
#include "kido/dynamics/MeshShape.h"
#include "kido/dynamics/WeldJoint.h"
#include "kido/dynamics/PrismaticJoint.h"
#include "kido/dynamics/RevoluteJoint.h"
#include "kido/dynamics/ScrewJoint.h"
#include "kido/dynamics/TranslationalJoint.h"
#include "kido/dynamics/BallJoint.h"
#include "kido/dynamics/FreeJoint.h"
#include "kido/dynamics/EulerJoint.h"
#include "kido/dynamics/UniversalJoint.h"
#include "kido/dynamics/Skeleton.h"
#include "kido/simulation/World.h"
#include "kido/utils/SkelParser.h"
#include "kido/common/LocalResourceRetriever.h"
#include "kido/common/Uri.h"
#include "kido/utils/sdf/SdfParser.h"

namespace kido {
namespace utils {

//==============================================================================
simulation::WorldPtr SdfParser::readSdfFile(
  const common::Uri& _fileUri, const common::ResourceRetrieverPtr& _retriever)
{
  return readSdfFile(_fileUri, getResourceRetriever(_retriever),
    static_cast<simulation::WorldPtr (*)(
      tinyxml2::XMLElement*, const std::string&,
      const common::ResourceRetrieverPtr&)>(&SdfParser::readWorld));
}

//==============================================================================
simulation::WorldPtr SdfParser::readSdfFile(
    const common::Uri& _fileUri,
    const common::ResourceRetrieverPtr& _retriever,
    std::function<simulation::WorldPtr (
      tinyxml2::XMLElement*, const std::string&,
      const common::ResourceRetrieverPtr&)> xmlReader)
{
  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _kidoFile;
  try
  {
    openXMLFile(_kidoFile, _fileUri, _retriever);
  }
  catch(std::exception const& e)
  {
    dtwarn << "[SdfParser::readSdfFile] Loading file [" << _fileUri.toString()
           << "] failed: " << e.what() << "\n";
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load KIDO
  tinyxml2::XMLElement* sdfElement = nullptr;
  sdfElement = _kidoFile.FirstChildElement("sdf");
  if (sdfElement == nullptr)
    return nullptr;

  //--------------------------------------------------------------------------
  // version attribute
  std::string version = getAttributeString(sdfElement, "version");
  // TODO: We need version aware SDF parser (see #264)
  // We support 1.4 only for now.
  if (version != "1.4" && version != "1.5")
  {
    dtwarn << "[SdfParser::readSdfFile] The file format of ["
           << _fileUri.toString()
           << "] was found to be [" << version << "], but we only support SDF "
           << "1.4 and 1.5!\n";
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* worldElement = nullptr;
  worldElement = sdfElement->FirstChildElement("world");
  if (worldElement == nullptr)
    return nullptr;

  std::string fileName = _fileUri.getFilesystemPath();  // Uri's path is unix-style path
  std::string skelPath = fileName.substr(0, fileName.rfind("/") + 1);

  return xmlReader(worldElement, skelPath, _retriever);
}

//==============================================================================
dynamics::SkeletonPtr SdfParser::readSkeleton(
  const common::Uri& _fileUri, const common::ResourceRetrieverPtr& _retriever)
{
  return readSkeleton(_fileUri, getResourceRetriever(_retriever),
    static_cast<dynamics::SkeletonPtr (*)(
      tinyxml2::XMLElement*, const std::string&,
      const common::ResourceRetrieverPtr&)>(&SdfParser::readSkeleton));
}

//==============================================================================
dynamics::SkeletonPtr SdfParser::readSkeleton(
    const common::Uri& _fileUri,
    const common::ResourceRetrieverPtr& _retriever,
    std::function<dynamics::SkeletonPtr(
      tinyxml2::XMLElement*, const std::string&,
      const common::ResourceRetrieverPtr&)> xmlReader)
{
  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _kidoFile;
  try
  {
    openXMLFile(_kidoFile, _fileUri, _retriever);
  }
  catch(std::exception const& e)
  {
    dtwarn << "[SdfParser::readSkeleton] Loading file [" << _fileUri.toString()
           << "] failed: " << e.what() << "\n";
    return nullptr;
  }

  //--------------------------------------------------------------------------
  // Load sdf
  tinyxml2::XMLElement* sdfElement = nullptr;
  sdfElement = _kidoFile.FirstChildElement("sdf");
  if (sdfElement == nullptr)
    return nullptr;

  //--------------------------------------------------------------------------
  // version attribute
  std::string version = getAttributeString(sdfElement, "version");
  // TODO: We need version aware SDF parser (see #264)
  // We support 1.4 only for now.
  if (version != "1.4" && version != "1.5")
  {
    dtwarn << "[SdfParser::readSdfFile] The file format of ["
           << _fileUri.toString() << "] was found to be [" << version
           << "], but we only support SDF 1.4 and 1.5!\n";
    return nullptr;
  }
  //--------------------------------------------------------------------------
  // Load skeleton
  tinyxml2::XMLElement* skelElement = nullptr;
  skelElement = sdfElement->FirstChildElement("model");
  if (skelElement == nullptr)
    return nullptr;

  std::string fileName = _fileUri.getFilesystemPath();  // Uri's path is unix-style path
  std::string skelPath = fileName.substr(0, fileName.rfind("/") + 1);

  dynamics::SkeletonPtr newSkeleton = xmlReader(skelElement, skelPath,
                                                _retriever);

  return newSkeleton;
}

simulation::WorldPtr SdfParser::readWorld(
    tinyxml2::XMLElement* _worldElement,
    const std::string& _skelPath,
    const common::ResourceRetrieverPtr& _retriever)
{
  return readWorld(_worldElement, _skelPath, _retriever,
      static_cast<dynamics::SkeletonPtr (*)(
        tinyxml2::XMLElement*, const std::string&,
        const common::ResourceRetrieverPtr&)>(&SdfParser::readSkeleton));
}

simulation::WorldPtr SdfParser::readWorld(
    tinyxml2::XMLElement* _worldElement,
    const std::string& _skelPath,
    const common::ResourceRetrieverPtr& _retriever,
    std::function<dynamics::SkeletonPtr(
      tinyxml2::XMLElement*, const std::string&,
      const common::ResourceRetrieverPtr&)> skeletonReader)
{
  assert(_worldElement != nullptr);

  // Create a world
  simulation::WorldPtr newWorld(new simulation::World);

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttributeString(_worldElement, "name");
  newWorld->setName(name);

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
            = skeletonReader(skeletonElements.get(), _skelPath, _retriever);

    newWorld->addSkeleton(newSkeleton);
  }

  return newWorld;
}

void SdfParser::readPhysics(tinyxml2::XMLElement* _physicsElement,
                 simulation::WorldPtr _world)
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
    tinyxml2::XMLElement* _skeletonElement,
    const std::string& _skelPath,
    const common::ResourceRetrieverPtr& _retriever)
{
  return readSkeleton(_skeletonElement, _skelPath, _retriever, &readBodyNode, &createPair);
}

dynamics::SkeletonPtr SdfParser::readSkeleton(
    tinyxml2::XMLElement* _skeletonElement,
    const std::string& _skelPath,
    const common::ResourceRetrieverPtr& _retriever,
    std::function<SDFBodyNode (tinyxml2::XMLElement*,
                   const Eigen::Isometry3d&,
                   const std::string&,
                   const common::ResourceRetrieverPtr&)> bodyReader,

    std::function<bool(dynamics::SkeletonPtr,
                       dynamics::BodyNode*,
                       const SDFJoint&,
                       const SDFBodyNode&)> pairCreator)
{
  assert(_skeletonElement != nullptr);

  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();
  dynamics::SkeletonPtr newSkeleton = makeSkeleton(
        _skeletonElement, skeletonFrame);

  //--------------------------------------------------------------------------
  // Bodies
  BodyMap sdfBodyNodes = readAllBodyNodes(
    _skeletonElement, _skelPath, _retriever, skeletonFrame, bodyReader);

  //--------------------------------------------------------------------------
  // Joints
  JointMap sdfJoints = readAllJoints(_skeletonElement, skeletonFrame,
                                     sdfBodyNodes);

  // Iterate through the collected properties and construct the Skeleton from
  // the root nodes downward
  BodyMap::iterator body = sdfBodyNodes.begin();
  JointMap::const_iterator parentJoint;
  dynamics::BodyNode* parentBody;
  while(body != sdfBodyNodes.end())
  {
    NextResult result = getNextJointAndNodePair(
          body, parentJoint, parentBody, newSkeleton, sdfBodyNodes, sdfJoints);

    if(BREAK == result)
      break;
    else if(CONTINUE == result)
      continue;
    else if(CREATE_FREEJOINT_ROOT == result)
    {
      // If a root FreeJoint is needed for the parent of the current joint, then
      // create it
      SDFJoint rootJoint;
      rootJoint.properties =
          Eigen::make_aligned_shared<dynamics::FreeJoint::Properties>(
            dynamics::Joint::Properties("root", body->second.initTransform));
      rootJoint.type = "free";

      if(!pairCreator(newSkeleton, nullptr, rootJoint, body->second))
        break;

      sdfBodyNodes.erase(body);
      body = sdfBodyNodes.begin();

      continue;
    }

    if(!pairCreator(newSkeleton, parentBody, parentJoint->second, body->second))
      break;

    sdfBodyNodes.erase(body);
    body = sdfBodyNodes.begin();
  }

  // Set positions to their initial values
  newSkeleton->resetPositions();

  return newSkeleton;
}

bool SdfParser::createPair(dynamics::SkeletonPtr skeleton,
                           dynamics::BodyNode* parent,
                           const SDFJoint& newJoint, const SDFBodyNode& newBody)
{
  std::pair<dynamics::Joint*,dynamics::BodyNode*> pair =
      createJointAndNodePair<dynamics::BodyNode>(
        skeleton, parent, newJoint, newBody);

  if(!pair.first || !pair.second)
    return false;

  return true;
}

SdfParser::NextResult SdfParser::getNextJointAndNodePair(
    BodyMap::iterator& body,
    JointMap::const_iterator& parentJoint,
    dynamics::BodyNode*& parentBody,
    const dynamics::SkeletonPtr skeleton,
    BodyMap& sdfBodyNodes,
    const JointMap& sdfJoints)
{
  parentJoint = sdfJoints.find(body->first);
  if(parentJoint == sdfJoints.end())
  {
    return CREATE_FREEJOINT_ROOT;
  }

  const std::string& parentBodyName = parentJoint->second.parentName;
  const std::string& parentJointName = parentJoint->second.properties->mName;

  // Check if the parent Body is created yet
  parentBody = skeleton->getBodyNode(parentBodyName);
  if(nullptr == parentBody && parentBodyName != "world"
     && !parentBodyName.empty())
  {
    // Find the properties of the parent Joint of the current Joint, because it
    // does not seem to be created yet.
    BodyMap::iterator check_parent_body = sdfBodyNodes.find(parentBodyName);

    if(check_parent_body == sdfBodyNodes.end())
    {
      // The Body does not exist in the file
      dterr << "[SdfParser::getNextJointAndNodePair] Could not find Link "
            << "named [" << parentBodyName << "] requested as parent of "
            << "Joint [" << parentJointName << "]. We will now quit "
            << "parsing.\n";
      return BREAK;
    }
    else
    {
      body = check_parent_body;
      return CONTINUE; // Create the parent before creating the current Joint
    }
  }

  return VALID;
}

dynamics::SkeletonPtr SdfParser::makeSkeleton(
    tinyxml2::XMLElement* _skeletonElement,
    Eigen::Isometry3d& skeletonFrame)
{
  assert(_skeletonElement != nullptr);

  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create();

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttributeString(_skeletonElement, "name");
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

  return newSkeleton;
}

SdfParser::BodyMap SdfParser::readAllBodyNodes(
    tinyxml2::XMLElement* _skeletonElement,
    const std::string& _skelPath,
    const common::ResourceRetrieverPtr& _retriever, 
    const Eigen::Isometry3d& skeletonFrame)
{
  return readAllBodyNodes(_skeletonElement, _skelPath, _retriever,
                          skeletonFrame, &readBodyNode);
}

SdfParser::BodyMap SdfParser::readAllBodyNodes(
    tinyxml2::XMLElement* _skeletonElement,
    const std::string& _skelPath,
    const common::ResourceRetrieverPtr& _retriever, 
    const Eigen::Isometry3d& skeletonFrame,
    std::function<SDFBodyNode (
      tinyxml2::XMLElement*,
      const Eigen::Isometry3d&,
      const std::string&,
      const common::ResourceRetrieverPtr&)> bodyReader)
{
  ElementEnumerator bodies(_skeletonElement, "link");
  BodyMap sdfBodyNodes;
  while (bodies.next())
  {
    SDFBodyNode body = bodyReader(
          bodies.get(), skeletonFrame, _skelPath, _retriever);

    BodyMap::iterator it = sdfBodyNodes.find(body.properties->mName);
    if(it != sdfBodyNodes.end())
    {
      dtwarn << "[SdfParser::readAllBodyNodes] Duplicate name in file: "
             << body.properties->mName << "\n"
             << "Every Link must have a unique name!\n";
      continue;
    }

    sdfBodyNodes[body.properties->mName] = body;
  }

  return sdfBodyNodes;
}

SdfParser::SDFBodyNode SdfParser::readBodyNode(
        tinyxml2::XMLElement* _bodyNodeElement,
        const Eigen::Isometry3d& _skeletonFrame,
        const std::string& _skelPath,
        const common::ResourceRetrieverPtr& _retriever)
{
  assert(_bodyNodeElement != nullptr);

  dynamics::BodyNode::Properties properties;
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  std::string name = getAttributeString(_bodyNodeElement, "name");
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
    dynamics::ShapePtr newShape(readShape(vizShapes.get(), _skelPath, _retriever));
    if (newShape)
      properties.mVizShapes.push_back(newShape);
  }

  //--------------------------------------------------------------------------
  // collision
  ElementEnumerator collShapes(_bodyNodeElement, "collision");
  while (collShapes.next())
  {
    dynamics::ShapePtr newShape(readShape(collShapes.get(), _skelPath, _retriever));

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
  sdfBodyNode.properties =
      Eigen::make_aligned_shared<dynamics::BodyNode::Properties>(properties);
  sdfBodyNode.initTransform = initTransform;

  return sdfBodyNode;
}

dynamics::ShapePtr SdfParser::readShape(
  tinyxml2::XMLElement* _shapelement,
  const std::string& _skelPath,
  const common::ResourceRetrieverPtr& _retriever)
{
  dynamics::ShapePtr newShape;

  // type
  assert(hasElement(_shapelement, "geometry"));
  tinyxml2::XMLElement* geometryElement = getElement(_shapelement, "geometry");

  if (hasElement(geometryElement, "box"))
  {
    tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

    Eigen::Vector3d size = getValueVector3d(boxElement, "size");

    newShape = dynamics::ShapePtr(new dynamics::BoxShape(size));
  }
  else if (hasElement(geometryElement, "sphere"))
  {
    tinyxml2::XMLElement* ellipsoidElement = getElement(geometryElement, "sphere");

    double radius = getValueDouble(ellipsoidElement, "radius");
    Eigen::Vector3d size(radius * 2, radius * 2, radius * 2);

    newShape = dynamics::ShapePtr(new dynamics::EllipsoidShape(size));
  }
  else if (hasElement(geometryElement, "cylinder"))
  {
    tinyxml2::XMLElement* cylinderElement = getElement(geometryElement, "cylinder");

    double radius = getValueDouble(cylinderElement, "radius");
    double height = getValueDouble(cylinderElement, "length");

    newShape = dynamics::ShapePtr(new dynamics::CylinderShape(radius, height));
  }
  else if (hasElement(geometryElement, "plane"))
  {
    // TODO: Don't support plane shape yet.
    tinyxml2::XMLElement* planeElement = getElement(geometryElement, "plane");

    Eigen::Vector2d visSize = getValueVector2d(planeElement, "size");
    // TODO: Need to use normal for correct orientation of the plane
    //Eigen::Vector3d normal = getValueVector3d(planeElement, "normal");

    Eigen::Vector3d size(visSize(0), visSize(1), 0.001);

    newShape = dynamics::ShapePtr(new dynamics::BoxShape(size));
  }
  else if (hasElement(geometryElement, "mesh"))
  {
    tinyxml2::XMLElement* meshEle = getElement(geometryElement, "mesh");
    // TODO(JS): We assume that uri is just file name for the mesh
    if (!hasElement(meshEle, "uri"))
    {
      // TODO(MXG): Figure out how to report the file name and line number of
      dtwarn << "[SdfParser::readShape] Mesh is missing a URI, which is "
             << "required in order to load it\n";
      return nullptr;
    }
    std::string           uri     = getValueString(meshEle, "uri");

    Eigen::Vector3d       scale   = hasElement(meshEle, "scale")?
          getValueVector3d(meshEle, "scale") : Eigen::Vector3d::Ones();

    const std::string meshUri = common::Uri::getRelativeUri(_skelPath, uri);
    const aiScene* model = dynamics::MeshShape::loadMesh(meshUri, _retriever);

    if (model)
      newShape = std::make_shared<dynamics::MeshShape>(
        scale, model, meshUri, _retriever);
    else
    {
      dtwarn << "[SdfParser::readShape] Failed to load mesh model ["
             << meshUri << "].\n";
      return nullptr;
    }
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

SdfParser::JointMap SdfParser::readAllJoints(
    tinyxml2::XMLElement* _skeletonElement,
    const Eigen::Isometry3d& skeletonFrame,
    const BodyMap& sdfBodyNodes)
{
  JointMap sdfJoints;
  ElementEnumerator joints(_skeletonElement, "joint");
  while (joints.next())
  {
    SDFJoint joint = readJoint(joints.get(), sdfBodyNodes, skeletonFrame);

    if(joint.childName.empty())
    {
      dterr << "[SdfParser::readAllJoints] Joint named ["
            << joint.properties->mName << "] does not have a valid child "
            << "Link, so it will not be added to the Skeleton\n";
      continue;
    }

    JointMap::iterator it = sdfJoints.find(joint.childName);
    if(it != sdfJoints.end())
    {
      dterr << "[SdfParser::readAllJoints] Joint named ["
            << joint.properties->mName << "] is claiming Link ["
            << joint.childName << "] as its child, but that is already "
            << "claimed by Joint [" << it->second.properties->mName
            << "]. Joint [" << joint.properties->mName
            << "] will be discarded\n";
      continue;
    }

    sdfJoints[joint.childName] = joint;
  }

  return sdfJoints;
}

SdfParser::SDFJoint SdfParser::readJoint(tinyxml2::XMLElement* _jointElement,
    const BodyMap& _sdfBodyNodes,
    const Eigen::Isometry3d& _skeletonFrame)
{
  assert(_jointElement != nullptr);

  //--------------------------------------------------------------------------
  // Type attribute
  std::string type = getAttributeString(_jointElement, "type");
  assert(!type.empty());

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttributeString(_jointElement, "name");

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
  newJoint.childName = (child_it == _sdfBodyNodes.end())?
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
  Eigen::Isometry3d parentModelFrame =
      (childWorld * childToJoint).inverse() * _skeletonFrame;

  if (type == std::string("prismatic"))
    newJoint.properties =
        Eigen::make_aligned_shared<dynamics::PrismaticJoint::Properties>(
          readPrismaticJoint(_jointElement, parentModelFrame, name));
  if (type == std::string("revolute"))
    newJoint.properties =
        Eigen::make_aligned_shared<dynamics::RevoluteJoint::Properties>(
          readRevoluteJoint(_jointElement, parentModelFrame, name));
  if (type == std::string("screw"))
    newJoint.properties =
        Eigen::make_aligned_shared<dynamics::ScrewJoint::Properties>(
          readScrewJoint(_jointElement, parentModelFrame, name));
  if (type == std::string("revolute2"))
    newJoint.properties =
        Eigen::make_aligned_shared<dynamics::UniversalJoint::Properties>(
          readUniversalJoint(_jointElement, parentModelFrame, name));
  if (type == std::string("ball"))
    newJoint.properties =
        Eigen::make_aligned_shared<dynamics::BallJoint::Properties>(
          readBallJoint(_jointElement, parentModelFrame, name));

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

static void readAxisElement(
    tinyxml2::XMLElement* axisElement,
    const Eigen::Isometry3d& _parentModelFrame,
    Eigen::Vector3d& axis,
    double& lower, double& upper, double& initial, double& rest,
    double& damping)
{
  // use_parent_model_frame
  bool useParentModelFrame = false;
  if (hasElement(axisElement, "use_parent_model_frame"))
    useParentModelFrame = getValueBool(axisElement, "use_parent_model_frame");

  // xyz
  Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
  if (useParentModelFrame)
  {
    xyz = _parentModelFrame * xyz;
  }
  axis = xyz;

  // dynamics
  if (hasElement(axisElement, "dynamics"))
  {
    tinyxml2::XMLElement* dynamicsElement
            = getElement(axisElement, "dynamics");

    // damping
    if (hasElement(dynamicsElement, "damping"))
    {
      damping = getValueDouble(dynamicsElement, "damping");
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
      lower = getValueDouble(limitElement, "lower");
    }

    // upper
    if (hasElement(limitElement, "upper"))
    {
      upper = getValueDouble(limitElement, "upper");
    }
  }

  // If the zero position is out of our limits, we should change the initial
  // position instead of assuming zero
  if( 0.0 < lower || upper < 0.0 )
  {
    if( std::isfinite(lower) && std::isfinite(upper) )
      initial = (lower + upper) / 2.0;
    else if( std::isfinite(lower) )
      initial = lower;
    else if( std::isfinite(upper) )
      initial = upper;

    // Any other case means the limits are both +inf, both -inf, or one is a NaN

    // Apply the same logic to the rest position.
    rest = initial;
  }
}

kido::dynamics::WeldJoint::Properties SdfParser::readWeldJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d&,
    const std::string&)
{
    assert(_jointElement != nullptr);

    return dynamics::WeldJoint::Properties();
}

dynamics::RevoluteJoint::Properties SdfParser::readRevoluteJoint(
    tinyxml2::XMLElement* _revoluteJointElement,
    const Eigen::Isometry3d& _parentModelFrame,
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

    readAxisElement(axisElement, _parentModelFrame,
                    newRevoluteJoint.mAxis,
                    newRevoluteJoint.mPositionLowerLimit,
                    newRevoluteJoint.mPositionUpperLimit,
                    newRevoluteJoint.mInitialPosition,
                    newRevoluteJoint.mRestPosition,
                    newRevoluteJoint.mDampingCoefficient);
  }
  else
  {
    reportMissingElement("readRevoluteJoint", "axis", "joint", _name);
  }

  return newRevoluteJoint;
}

dynamics::PrismaticJoint::Properties SdfParser::readPrismaticJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d& _parentModelFrame,
    const std::string& _name)
{
  assert(_jointElement != nullptr);

  dynamics::PrismaticJoint::Properties newPrismaticJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis"))
  {
    tinyxml2::XMLElement* axisElement
            = getElement(_jointElement, "axis");

    readAxisElement(axisElement, _parentModelFrame,
                    newPrismaticJoint.mAxis,
                    newPrismaticJoint.mPositionLowerLimit,
                    newPrismaticJoint.mPositionUpperLimit,
                    newPrismaticJoint.mInitialPosition,
                    newPrismaticJoint.mRestPosition,
                    newPrismaticJoint.mDampingCoefficient);
  }
  else
  {
    reportMissingElement("readPrismaticJoint", "axis", "joint", _name);
  }

  return newPrismaticJoint;
}

dynamics::ScrewJoint::Properties SdfParser::readScrewJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d& _parentModelFrame,
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

    readAxisElement(axisElement, _parentModelFrame,
                    newScrewJoint.mAxis,
                    newScrewJoint.mPositionLowerLimit,
                    newScrewJoint.mPositionUpperLimit,
                    newScrewJoint.mInitialPosition,
                    newScrewJoint.mRestPosition,
                    newScrewJoint.mDampingCoefficient);
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
    const Eigen::Isometry3d& _parentModelFrame,
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

    readAxisElement(axisElement, _parentModelFrame,
                    newUniversalJoint.mAxis[0],
                    newUniversalJoint.mPositionLowerLimits[0],
                    newUniversalJoint.mPositionUpperLimits[0],
                    newUniversalJoint.mInitialPositions[0],
                    newUniversalJoint.mRestPositions[0],
                    newUniversalJoint.mDampingCoefficients[0]);
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

    readAxisElement(axis2Element, _parentModelFrame,
                    newUniversalJoint.mAxis[1],
                    newUniversalJoint.mPositionLowerLimits[1],
                    newUniversalJoint.mPositionUpperLimits[1],
                    newUniversalJoint.mInitialPositions[1],
                    newUniversalJoint.mRestPositions[1],
                    newUniversalJoint.mDampingCoefficients[1]);
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
    const std::string&)
{
  assert(_jointElement != nullptr);

  return dynamics::BallJoint::Properties();
}

dynamics::TranslationalJoint::Properties SdfParser::readTranslationalJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d&,
    const std::string&)
{
  assert(_jointElement != nullptr);

  return dynamics::TranslationalJoint::Properties();
}

dynamics::FreeJoint::Properties SdfParser::readFreeJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d&,
    const std::string&)
{
  assert(_jointElement != nullptr);

  return dynamics::FreeJoint::Properties();
}

common::ResourceRetrieverPtr SdfParser::getResourceRetriever(
    const common::ResourceRetrieverPtr& _retriever)
{
  if(_retriever)
    return _retriever;
  else
    return std::make_shared<common::LocalResourceRetriever>();
}

} // namespace utils
} // namespace kido
