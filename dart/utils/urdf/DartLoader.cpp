/**
 * @file DartLoader.cpp
 */

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

void DartLoader::addPackageDirectory(const std::string& _packageName,
                                     const std::string& _packageDirectory)
{
  mPackageDirectories[_packageName] = _packageDirectory;
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

simulation::World* DartLoader::parseWorld(const std::string& _urdfFileName)
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

simulation::World* DartLoader::parseWorldString(
    const std::string& _urdfString, const std::string& _urdfFileDirectory)
{
  if(_urdfString.empty())
  {
    dtwarn << "[DartLoader::parseWorldString] A blank string cannot be "
           << "parsed into a World. Returning a nullptr\n";
    return nullptr;
  }

  mRootToWorldPath = _urdfFileDirectory;

  urdf::World* worldInterface = urdf::parseWorldURDF(_urdfString, mRootToWorldPath);
  if(!worldInterface)
      return nullptr;

  // Store paths from world to entities
  parseWorldToEntityPaths(_urdfString);

  simulation::World* world = new simulation::World();

  for(unsigned int i = 0; i < worldInterface->models.size(); ++i) {
    mRootToSkelPath = mRootToWorldPath + mWorld_To_Entity_Paths.find(worldInterface->models[i].model->getName())->second;
    dynamics::SkeletonPtr skeleton = modelInterfaceToSkeleton(worldInterface->models[i].model.get());

    if(!skeleton) {
      std::cout << "[ERROR] Robot " << worldInterface->models[i].model->getName() << " was not correctly parsed. World is not loaded. Exiting!" << std::endl;
      return nullptr;
    }

    // Initialize position and RPY
    dynamics::Joint* rootJoint = skeleton->getRootBodyNode()->getParentJoint();
    Eigen::Isometry3d transform = toEigen(worldInterface->models[i].origin);

    if(dynamic_cast<dynamics::FreeJoint*>(rootJoint)) {
        Eigen::Vector6d coordinates;
        coordinates << math::logMap(transform.linear()), transform.translation();
        rootJoint->setPositions(coordinates);
    }
    else {
        rootJoint->setTransformFromParentBodyNode(transform);
    }

    world->addSkeleton(skeleton);
  }

  return world;
}

/**
 * @function getFullFilePath
 */
std::string DartLoader::getFullFilePath(const std::string& _filename) const
{
  std::string fullpath = _filename;
  size_t scheme = fullpath.find("package://");
  if(scheme < std::string::npos)
  {
    size_t authority_start = scheme+10;
    size_t authority_end = fullpath.find("/", scheme+10);
    size_t authority_length = authority_end - authority_start;

    std::map<std::string, std::string>::const_iterator packageDirectory =
        mPackageDirectories.find(
          fullpath.substr(authority_start, authority_length));

    if(packageDirectory == mPackageDirectories.end())
    {
      dtwarn << "[DartLoader] Trying to load a URDF that uses package '"
             << fullpath.substr(scheme, authority_end-scheme)
             << "' (the full line is '" << fullpath
             << "'), but we do not know the path to that package directory. "
             << "Please use addPackageDirectory(~) to allow us to find the "
             << "package directory.\n";
    }
    else
    {
      fullpath.erase(scheme, authority_end);
      fullpath.insert(scheme, packageDirectory->second);
    }
  }
  else
  {
    fullpath = mRootToSkelPath + fullpath;
  }

  return fullpath;
}

/**
 * @function parseWorldToEntityPaths
 */
void DartLoader::parseWorldToEntityPaths(const std::string& _xml_string) {
    
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

      if( entity_name && entity_model ) {
        std::string string_entity_model( entity_model );  
        std::string string_entity_name( entity_name ); 
        // Find the model
        if( includedFiles.find( string_entity_model ) == includedFiles.end() ) {
          std::cout<<"[!] Did not find entity model included. Weird things may happen"<<std::endl;
          return;
        }
        // Add it
        else {
          std::string string_entity_filepath = includedFiles.find( string_entity_model )->second;
          mWorld_To_Entity_Paths[string_entity_name] = string_entity_filepath;
        }
      }
      // If no name or model is defined
      else {
        std::cout<< "[!] Entity was not defined. Weird things will happen" <<std::endl;
      }

    } // for all entities

}

/**
 * @function modelInterfaceToSkeleton
 * @brief Read the ModelInterface and spits out a Skeleton object
 */
dynamics::SkeletonPtr DartLoader::modelInterfaceToSkeleton(const urdf::ModelInterface* _model) {

    dynamics::SkeletonPtr skeleton(new dynamics::Skeleton(_model->getName()));
    dynamics::BodyNode* rootNode = nullptr;
    dynamics::Joint* rootJoint = nullptr;

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
        rootNode = createDartNode(root);
        rootJoint = createDartJoint(root->parent_joint.get());
        if(!rootJoint)
          return nullptr;
      }
    }
    else {
        rootNode = createDartNode(root);
        rootJoint = new dynamics::FreeJoint();
        rootJoint->setName("rootJoint");
        rootJoint->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());
        rootJoint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
    }

    rootNode->setParentJoint(rootJoint);
    skeleton->addBodyNode(rootNode);

    for(size_t i = 0; i < root->child_links.size(); i++) {
      createSkeletonRecursive(skeleton, root->child_links[i].get(), rootNode);
    }

    return skeleton;
}

void DartLoader::createSkeletonRecursive(dynamics::SkeletonPtr _skel, const urdf::Link* _lk, dynamics::BodyNode* _parentNode) {
  dynamics::BodyNode* node = createDartNode(_lk);
  dynamics::Joint* joint = createDartJoint(_lk->parent_joint.get());
  node->setParentJoint(joint);
  _parentNode->addChildBodyNode(node);
  _skel->addBodyNode(node);
  
  for(unsigned int i = 0; i < _lk->child_links.size(); i++) {
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
    dtwarn << "[DartLoader] Failed to open file '" << _xmlFile << "'! "
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
    dtwarn << "[DartLoader] Opened file '" << _xmlFile << "', but found it to "
           << "be empty. Please make sure you provided the correct filename\n";
  }
  
  return xml_string;
}

/**
 * @function createDartJoint
 */
dynamics::Joint* DartLoader::createDartJoint(const urdf::Joint* _jt)
{ 
  dynamics::Joint* joint;
  switch(_jt->type) {
  case urdf::Joint::REVOLUTE:
      joint = new dynamics::RevoluteJoint(toEigen(_jt->axis));
      joint->setPositionLowerLimit(0, _jt->limits->lower);
      joint->setPositionUpperLimit(0, _jt->limits->upper);
      if (_jt->dynamics)
          joint->setDampingCoefficient(0, _jt->dynamics->damping);
      break;
  case urdf::Joint::CONTINUOUS:
      joint = new dynamics::RevoluteJoint(toEigen(_jt->axis));
      if (_jt->dynamics)
          joint->setDampingCoefficient(0, _jt->dynamics->damping);
      break;
  case urdf::Joint::PRISMATIC:
      joint = new dynamics::PrismaticJoint(toEigen(_jt->axis));
      joint->setPositionLowerLimit(0, _jt->limits->lower);
      joint->setPositionUpperLimit(0, _jt->limits->upper);
      if (_jt->dynamics)
          joint->setDampingCoefficient(0, _jt->dynamics->damping);
      break;
  case urdf::Joint::FIXED:
      joint = new dynamics::WeldJoint();
      break;
  case urdf::Joint::FLOATING:
      joint = new dynamics::FreeJoint();
      break;
  case urdf::Joint::PLANAR:
      joint = new dynamics::PlanarJoint();
      // TODO(MXG): Should we read in position limits? The URDF limits
      // specification only offers one dimension of limits, but a PlanarJoint is
      // three-dimensional. Should we assume that position limits apply to both
      // coordinates equally? Or just don't accept the position limits at all?
      break;
  default:
      std::cout << "Unsupported joint type." << std::endl;
      assert(false);
      return NULL;
  }
  joint->setName(_jt->name);
  joint->setTransformFromParentBodyNode(toEigen(_jt->parent_to_joint_origin_transform));
  joint->setTransformFromChildBodyNode(Eigen::Isometry3d::Identity());
  if(joint->getNumDofs() == 1 && _jt->limits) {
    joint->setVelocityLowerLimit(0, -_jt->limits->velocity);
    joint->setVelocityUpperLimit(0, _jt->limits->velocity);
    joint->setForceLowerLimit(0, -_jt->limits->effort);
    joint->setForceUpperLimit(0, _jt->limits->effort);
  }
  return joint;
}

/**
 * @function createDartNode
 */
dynamics::BodyNode* DartLoader::createDartNode(const urdf::Link* _lk) {

  dynamics::BodyNode* node = new dynamics::BodyNode(_lk->name);
  
  // Load Inertial information
  if(_lk->inertial) {
    urdf::Pose origin = _lk->inertial->origin;
    node->setLocalCOM(toEigen(origin.position));
    node->setMass(_lk->inertial->mass);

    Eigen::Matrix3d J;
    J << _lk->inertial->ixx, _lk->inertial->ixy, _lk->inertial->ixz,
         _lk->inertial->ixy, _lk->inertial->iyy, _lk->inertial->iyz,
         _lk->inertial->ixz, _lk->inertial->iyz, _lk->inertial->izz;
    Eigen::Matrix3d R(Eigen::Quaterniond(origin.rotation.w, origin.rotation.x,
                                         origin.rotation.y, origin.rotation.z));
    J = R * J * R.transpose();

    node->setMomentOfInertia(J(0,0), J(1,1), J(2,2),
                             J(0,1), J(0,2), J(1,2));
  }

  // Set visual information
  for(unsigned int i = 0; i < _lk->visual_array.size(); i++) {
    if(dynamics::Shape* shape = createShape(_lk->visual_array[i].get())) {
      node->addVisualizationShape(std::shared_ptr<dynamics::Shape>(shape));
    }
  }

  // Set collision information
  for(unsigned int i = 0; i < _lk->collision_array.size(); i++) {
    if(dynamics::Shape* shape = createShape(_lk->collision_array[i].get())) {
      node->addCollisionShape(dynamics::ShapePtr(shape));
    }
  }

  return node;
}


void setMaterial(dynamics::Shape* _shape, const urdf::Visual* _viz) {
  if(_viz->material) {
    _shape->setColor(Eigen::Vector3d(_viz->material->color.r, _viz->material->color.g, _viz->material->color.b));
  }
}

void setMaterial(dynamics::Shape* _shape, const urdf::Collision* _col) {
}

/**
 * @function createShape
 */
template <class VisualOrCollision>
dynamics::Shape* DartLoader::createShape(const VisualOrCollision* _vizOrCol)
{
  dynamics::Shape* shape;

  // Sphere
  if(urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(_vizOrCol->geometry.get()))
  {
    shape = new dynamics::EllipsoidShape(2.0 * sphere->radius * Eigen::Vector3d::Ones());
  }
  // Box
  else if(urdf::Box* box = dynamic_cast<urdf::Box*>(_vizOrCol->geometry.get()))
  {
    shape = new dynamics::BoxShape(Eigen::Vector3d(box->dim.x, box->dim.y, box->dim.z));
  }
  // Cylinder
  else if(urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(_vizOrCol->geometry.get()))
  {
    shape = new dynamics::CylinderShape(cylinder->radius, cylinder->length);
  }
  // Mesh
  else if(urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(_vizOrCol->geometry.get()))
  {
    std::string fullPath = getFullFilePath(mesh->filename);
    const aiScene* model = dynamics::MeshShape::loadMesh( fullPath );
    
    if(!model)
    {
      dtwarn << "[DartLoader::createShape] Assimp could not load a model from "
             << "the file '" << fullPath << "'\n";
      shape = nullptr;
    } 
    else
    {
      shape = new dynamics::MeshShape(Eigen::Vector3d(mesh->scale.x, mesh->scale.y, mesh->scale.z), model);
    }
  }
  // Unknown geometry type
  else
  {
    dtwarn << "[DartLoader::createShape] Unknown urdf Shape type "
           << "(we only know of Sphere, Box, Cylinder, and Mesh). "
           << "We are returning a nullptr." << std::endl;
    return nullptr;
  }

  shape->setLocalTransform(toEigen(_vizOrCol->origin));
  setMaterial(shape, _vizOrCol);
  return shape;
}

template dynamics::Shape* DartLoader::createShape<urdf::Visual>(const urdf::Visual* _vizOrCol);
template dynamics::Shape* DartLoader::createShape<urdf::Collision>(const urdf::Collision* _vizOrCol);

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
