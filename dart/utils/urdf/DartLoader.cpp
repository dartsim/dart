#include "DartLoader.h"

#include <map>
#include <iostream>
#include <fstream>

#include <urdf_parser/urdf_parser.h>
#include <urdf_world/world.h>

#include "dart/dynamics/Skeleton.h"
#include "dart/dynamics/BodyNode.h"
#include "dart/dynamics/Joint.h"
#include "dart/dynamics/RevoluteJoint.h"
#include "dart/dynamics/PrismaticJoint.h"
#include "dart/dynamics/WeldJoint.h"
#include "dart/dynamics/FreeJoint.h"
#include "dart/dynamics/PlanarJoint.h"
#include "dart/dynamics/Shape.h"
#include "dart/dynamics/BoxShape.h"
#include "dart/dynamics/EllipsoidShape.h"
#include "dart/dynamics/CylinderShape.h"
#include "dart/dynamics/MeshShape.h"
#include "dart/simulation/World.h"
#include "dart/utils/urdf/urdf_world_parser.h"

namespace dart {
namespace utils {

DartLoader::DartLoader()
  : mLocalRetriever(new utils::LocalResourceRetriever)
  , mPackageRetriever(new utils::PackageResourceRetriever(mLocalRetriever))
  , mRetriever(new utils::SchemaResourceRetriever)
{
  using namespace std::placeholders;

  mRetriever->addSchemaRetriever("file", mLocalRetriever);
  mRetriever->addSchemaRetriever("package", mPackageRetriever);
}

void DartLoader::addPackageDirectory(const std::string& _packageName,
                                     const std::string& _packageDirectory)
{
  mPackageRetriever->addPackageDirectory(_packageName, _packageDirectory);
}

dynamics::SkeletonPtr DartLoader::parseSkeleton(const std::string& _urdfFileName) {
  std::string urdfString = readFileToString(_urdfFileName);

  if(urdfString.empty())
  {
    dtwarn << "[DartLoder::parseSkeleton] A blank or nonexistent file cannot "
           << "be parsed into a Skeleton. Returning a nullptr\n";
    return nullptr;
  }

  // Change path to a Unix-style path if given a Windows one
  // Windows can handle Unix-style paths (apparently)
  mRootToSkelPath = _urdfFileName;
  std::replace(mRootToSkelPath.begin(), mRootToSkelPath.end(), '\\' , '/' );
  mRootToSkelPath = mRootToSkelPath.substr(0, mRootToSkelPath.rfind("/") + 1);

  return parseSkeletonString(urdfString, mRootToSkelPath);;
}

dynamics::SkeletonPtr DartLoader::parseSkeletonString(
    const std::string& _urdfString, const std::string& _urdfFileDirectory)
{
  if(_urdfString.empty())
  {
    dtwarn << "[DartLoader::parseSkeletonString] A blank string cannot be "
           << "parsed into a Skeleton. Returning a nullptr\n";
    return nullptr;
  }

  mRootToSkelPath = _urdfFileDirectory;

  boost::shared_ptr<urdf::ModelInterface> skeletonModelPtr = urdf::parseURDF(_urdfString);
  if(!skeletonModelPtr)
      return nullptr;

  return modelInterfaceToSkeleton(skeletonModelPtr.get());
}

simulation::WorldPtr DartLoader::parseWorld(const std::string& _urdfFileName)
{
  std::string urdfString = readFileToString(_urdfFileName);

  if(urdfString.empty())
  {
    dtwarn << "[DartLoader::parseWorld] A blank or nonexistent file cannot "
           << "be parsed into a World. Returning a nullptr\n";
    return nullptr;
  }

  // Change path to a Unix-style path if given a Windows one
  // Windows can handle Unix-style paths (apparently)
  mRootToWorldPath = _urdfFileName;
  std::replace(mRootToWorldPath.begin(), mRootToWorldPath.end(), '\\' , '/');
  mRootToWorldPath = mRootToWorldPath.substr(0, mRootToWorldPath.rfind("/") + 1);

  return parseWorldString(urdfString, mRootToWorldPath);
}

simulation::WorldPtr DartLoader::parseWorldString(
    const std::string& _urdfString, const std::string& _urdfFileDirectory)
{
  if(_urdfString.empty())
  {
    dtwarn << "[DartLoader::parseWorldString] A blank string cannot be "
           << "parsed into a World. Returning a nullptr\n";
    return nullptr;
  }

  mRootToWorldPath = _urdfFileDirectory;

  std::shared_ptr<urdf::World> worldInterface =
      urdf::parseWorldURDF(_urdfString, mRootToWorldPath);

  if(!worldInterface)
      return nullptr;

  // Store paths from world to entities
  parseWorldToEntityPaths(_urdfString);

  simulation::WorldPtr world(new simulation::World());

  for(size_t i = 0; i < worldInterface->models.size(); ++i)
  {
    std::string model_name = worldInterface->models[i].model->getName();
    std::map<std::string, std::string>::const_iterator it =
        mWorld_To_Entity_Paths.find(model_name);

    if(it == mWorld_To_Entity_Paths.end())
    {
      dtwarn << "[DartLoader::parseWorldString] Could not find file path for ["
             << model_name << "]. We will not parse it!\n";
      continue;
    }

    mRootToSkelPath = mRootToWorldPath + it->second;
    dynamics::SkeletonPtr skeleton = modelInterfaceToSkeleton(worldInterface->models[i].model.get());

    if(!skeleton)
    {
      dtwarn << "[DartLoader::parseWorldString] Robot " << worldInterface->models[i].model->getName()
             << " was not correctly parsed!\n";
      continue;
    }

    // Initialize position and RPY
    dynamics::Joint* rootJoint = skeleton->getRootBodyNode()->getParentJoint();
    Eigen::Isometry3d transform = toEigen(worldInterface->models[i].origin);

    if (dynamic_cast<dynamics::FreeJoint*>(rootJoint))
      rootJoint->setPositions(dynamics::FreeJoint::convertToPositions(transform));
    else
      rootJoint->setTransformFromParentBodyNode(transform);

    world->addSkeleton(skeleton);
  }

  return world;
}

/**
 * @function parseWorldToEntityPaths
 */
void DartLoader::parseWorldToEntityPaths(const std::string& _xml_string)
{
  TiXmlDocument xml_doc;
  xml_doc.Parse(_xml_string.c_str());

  TiXmlElement *world_xml = xml_doc.FirstChildElement("world");

  if( !world_xml ) {
    return;
  }

  // Get all include filenames
  std::map<std::string, std::string> includedFiles;

  for( TiXmlElement* include_xml = world_xml->FirstChildElement("include");
    include_xml; include_xml = include_xml->NextSiblingElement("include") ) {

    const char* filename = include_xml->Attribute("filename");
    const char* model_name = include_xml->Attribute("model_name");
    std::string string_filename( filename );
    std::string string_filepath = string_filename.substr( 0, string_filename.rfind("/") + 1 );
    std::string string_model_name( model_name );

    includedFiles[string_model_name] = string_filepath;
  }

  // Get all entities
  for( TiXmlElement* entity_xml = world_xml->FirstChildElement("entity");
    entity_xml; entity_xml = entity_xml->NextSiblingElement("entity") ) {

    // Find model and name for entity, if not, error
    const char* entity_model = entity_xml->Attribute("model");
    const char* entity_name = entity_xml->Attribute("name");

    if( entity_name && entity_model )
    {
      std::string string_entity_model( entity_model );
      std::string string_entity_name( entity_name );
      // Find the model
      if( includedFiles.find( string_entity_model ) == includedFiles.end() )
      {
        dtwarn <<"[DartLoader::parseWorldToEntityPaths] Did not find entity model ["
               << string_entity_model << "] included. We might fail to load some Skeletons!\n";
        return;
      }
      // Add it
      else
      {
        mWorld_To_Entity_Paths[string_entity_name] =
            includedFiles.find( string_entity_model )->second;
      }
    }
    // If no name or model is defined
    else
    {
      dtwarn << "[DartLoader::parseWorldToEntityPaths] Entity was not defined. Weird things will happen" << std::endl;
    }

  } // for all entities

}

/**
 * @function modelInterfaceToSkeleton
 * @brief Read the ModelInterface and spits out a Skeleton object
 */
dynamics::SkeletonPtr DartLoader::modelInterfaceToSkeleton(const urdf::ModelInterface* _model) {

  dynamics::SkeletonPtr skeleton = dynamics::Skeleton::create(_model->getName());

  dynamics::BodyNode* rootNode = nullptr;
  const urdf::Link* root = _model->getRoot().get();
  if(root->name == "world")
  {
    if(_model->getRoot()->child_links.size() != 1)
    {
      dterr << "[DartLoader::modelInterfaceToSkeleton] No unique link attached to world.\n";
    }
    else
    {
      root = root->child_links[0].get();
      dynamics::BodyNode::Properties rootProperties =
          createDartNodeProperties(root);
      rootNode = createDartJointAndNode(
            root->parent_joint.get(), rootProperties, nullptr, skeleton);
      if(nullptr == rootNode)
      {
        dterr << "[DartLoader::modelInterfaceToSkeleton] Failed to create root node!\n";
        return nullptr;
      }
    }
  }
  else
  {
    dynamics::BodyNode::Properties rootProperties =
        createDartNodeProperties(root);
    std::pair<dynamics::Joint*, dynamics::BodyNode*> pair =
        skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          nullptr, dynamics::FreeJoint::Properties(
            dynamics::MultiDofJoint<6>::Properties(
            dynamics::Joint::Properties("rootJoint"))),
          rootProperties);
    rootNode = pair.second;
  }

  for(size_t i = 0; i < root->child_links.size(); i++)
  {
    createSkeletonRecursive(skeleton, root->child_links[i].get(), rootNode);
  }

  return skeleton;
}

void DartLoader::createSkeletonRecursive(
    dynamics::SkeletonPtr _skel,
    const urdf::Link* _lk,
    dynamics::BodyNode* _parentNode)
{
  dynamics::BodyNode::Properties properties = createDartNodeProperties(_lk);
  dynamics::BodyNode* node = createDartJointAndNode(
        _lk->parent_joint.get(), properties, _parentNode, _skel);
  
  for(unsigned int i = 0; i < _lk->child_links.size(); ++i)
  {
      createSkeletonRecursive(_skel, _lk->child_links[i].get(), node);
  }
}


/**
 * @function readXml
 */
std::string  DartLoader::readFileToString(std::string _xmlFile) {
  
  std::string xml_string;
  std::ifstream xml_file(_xmlFile.c_str());

  if(!xml_file.is_open())
  {
    dtwarn << "[DartLoader::readFileToString] Failed to open file '" << _xmlFile << "'! "
           << "Check whether the file exists and has appropriate permissions.\n";
    return xml_string;
  }
  
  // Read xml
  while(xml_file.good()) {
    std::string line;
    std::getline(xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

  if(xml_string.empty())
  {
    dtwarn << "[DartLoader::readFileToString] Opened file '" << _xmlFile << "', but found it to "
           << "be empty. Please make sure you provided the correct filename\n";
  }
  
  return xml_string;
}

/**
 * @function createDartJoint
 */
dynamics::BodyNode* DartLoader::createDartJointAndNode(
    const urdf::Joint* _jt,
    const dynamics::BodyNode::Properties& _body,
    dynamics::BodyNode* _parent,
    dynamics::SkeletonPtr _skeleton)
{
  dynamics::Joint::Properties basicProperties;

  basicProperties.mName = _jt->name;
  basicProperties.mT_ParentBodyToJoint =
      toEigen(_jt->parent_to_joint_origin_transform);

  dynamics::SingleDofJoint::UniqueProperties singleDof;
  if(_jt->limits)
  {
    singleDof.mPositionLowerLimit = _jt->limits->lower;
    singleDof.mPositionUpperLimit = _jt->limits->upper;
    singleDof.mVelocityLowerLimit = -_jt->limits->velocity;
    singleDof.mVelocityUpperLimit =  _jt->limits->velocity;
    singleDof.mForceLowerLimit = -_jt->limits->effort;
    singleDof.mForceUpperLimit =  _jt->limits->effort;
  }

  if(_jt->dynamics)
    singleDof.mDampingCoefficient = _jt->dynamics->damping;

  std::pair<dynamics::Joint*, dynamics::BodyNode*> pair;
  switch(_jt->type)
  {
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:
    {
      dynamics::RevoluteJoint::Properties properties(
            dynamics::SingleDofJoint::Properties(basicProperties, singleDof),
            toEigen(_jt->axis));

      pair = _skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
            _parent, properties, _body);

      break;
    }
    case urdf::Joint::PRISMATIC:
    {
      dynamics::PrismaticJoint::Properties properties(
            dynamics::SingleDofJoint::Properties(basicProperties, singleDof),
            toEigen(_jt->axis));

      pair = _skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
            _parent, properties, _body);

      break;
    }
    case urdf::Joint::FIXED:
    {
      pair = _skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
            _parent, basicProperties, _body);
      break;
    }
    case urdf::Joint::FLOATING:
    {
      dynamics::MultiDofJoint<6>::Properties properties(basicProperties);

      pair = _skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
            _parent, properties, _body);
      break;
    }
    case urdf::Joint::PLANAR:
    {
      pair = _skeleton->createJointAndBodyNodePair<dynamics::PlanarJoint>(
            _parent, dynamics::PlanarJoint::Properties(basicProperties), _body);
      // TODO(MXG): Should we read in position limits? The URDF limits
      // specification only offers one dimension of limits, but a PlanarJoint is
      // three-dimensional. Should we assume that position limits apply to both
      // coordinates equally? Or just don't accept the position limits at all?
      break;
    }
    default:
    {
      dterr << "[DartLoader::createDartJoint] Unsupported joint type ("
            << _jt->type << ")\n";
      assert(false);
      return nullptr;
    }
  }

  return pair.second;
}

/**
 * @function createDartNode
 */
dynamics::BodyNode::Properties DartLoader::createDartNodeProperties(
    const urdf::Link* _lk)
{
  dynamics::BodyNode::Properties node(_lk->name);
  
  // Load Inertial information
  if(_lk->inertial) {
    urdf::Pose origin = _lk->inertial->origin;
    node.mInertia.setLocalCOM(toEigen(origin.position));
    node.mInertia.setMass(_lk->inertial->mass);

    Eigen::Matrix3d J;
    J << _lk->inertial->ixx, _lk->inertial->ixy, _lk->inertial->ixz,
         _lk->inertial->ixy, _lk->inertial->iyy, _lk->inertial->iyz,
         _lk->inertial->ixz, _lk->inertial->iyz, _lk->inertial->izz;
    Eigen::Matrix3d R(Eigen::Quaterniond(origin.rotation.w, origin.rotation.x,
                                         origin.rotation.y, origin.rotation.z));
    J = R * J * R.transpose();

    node.mInertia.setMoment(J(0,0), J(1,1), J(2,2),
                            J(0,1), J(0,2), J(1,2));
  }

  // Set visual information
  for(unsigned int i = 0; i < _lk->visual_array.size(); i++)
  {
    if(dynamics::ShapePtr shape = createShape(_lk->visual_array[i].get()))
    {
      node.mVizShapes.push_back(shape);
    }
  }

  // Set collision information
  for(unsigned int i = 0; i < _lk->collision_array.size(); i++) {
    if(dynamics::ShapePtr shape = createShape(_lk->collision_array[i].get())) {
      node.mColShapes.push_back(shape);
    }
  }

  return node;
}


void setMaterial(dynamics::ShapePtr _shape, const urdf::Visual* _viz) {
  if(_viz->material) {
    _shape->setColor(Eigen::Vector3d(_viz->material->color.r, _viz->material->color.g, _viz->material->color.b));
  }
}

void setMaterial(dynamics::ShapePtr _shape, const urdf::Collision* _col) {
}

/**
 * @function createShape
 */
template <class VisualOrCollision>
dynamics::ShapePtr DartLoader::createShape(const VisualOrCollision* _vizOrCol)
{
  dynamics::ShapePtr shape;

  // Sphere
  if(urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(_vizOrCol->geometry.get()))
  {
    shape = dynamics::ShapePtr(new dynamics::EllipsoidShape(
                  2.0 * sphere->radius * Eigen::Vector3d::Ones()));
  }
  // Box
  else if(urdf::Box* box = dynamic_cast<urdf::Box*>(_vizOrCol->geometry.get()))
  {
    shape = dynamics::ShapePtr(new dynamics::BoxShape(
                  Eigen::Vector3d(box->dim.x, box->dim.y, box->dim.z)));
  }
  // Cylinder
  else if(urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(_vizOrCol->geometry.get()))
  {
    shape = dynamics::ShapePtr(new dynamics::CylinderShape(
                  cylinder->radius, cylinder->length));
  }
  // Mesh
  else if(urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(_vizOrCol->geometry.get()))
  {
    const utils::ConstMemoryResourcePtr resource = mRetriever->retrieve(mesh->filename);
    if (!resource) {
      return nullptr;
    }

    const aiScene* scene = dynamics::MeshShape::loadMesh(*resource);
    if (!scene)
      return nullptr;

    const Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
    shape = std::make_shared<dynamics::MeshShape>(
      scale, scene, mesh->filename, true, mRetriever);
  }
  // Unknown geometry type
  else
  {
    dtwarn << "[DartLoader::createShape] Unknown URDF Shape type "
           << "(we only know of Sphere, Box, Cylinder, and Mesh). "
           << "We are returning a nullptr." << std::endl;
    return nullptr;
  }

  shape->setLocalTransform(toEigen(_vizOrCol->origin));
  setMaterial(shape, _vizOrCol);
  return shape;
}

template dynamics::ShapePtr DartLoader::createShape<urdf::Visual>(const urdf::Visual* _vizOrCol);
template dynamics::ShapePtr DartLoader::createShape<urdf::Collision>(const urdf::Collision* _vizOrCol);

/**
 * @function pose2Affine3d
 */
Eigen::Isometry3d DartLoader::toEigen(const urdf::Pose& _pose) {
    Eigen::Quaterniond quat;
    _pose.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
    Eigen::Isometry3d transform(quat);
    transform.translation() = Eigen::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z);
    return transform;
}

Eigen::Vector3d DartLoader::toEigen(const urdf::Vector3& _vector) {
    return Eigen::Vector3d(_vector.x, _vector.y, _vector.z);
}

} // namespace utils
} // namespace dart
