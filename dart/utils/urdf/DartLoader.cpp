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
#include "dart/utils/urdf/URDFTypes.h"
#include "dart/utils/urdf/urdf_world_parser.h"

namespace dart {
namespace utils {

using ModelInterfacePtr = urdf_shared_ptr<urdf::ModelInterface>;

DartLoader::DartLoader()
  : mLocalRetriever(new common::LocalResourceRetriever),
    mPackageRetriever(new utils::PackageResourceRetriever(mLocalRetriever)),
    mRetriever(new utils::CompositeResourceRetriever)
{
  mRetriever->addSchemaRetriever("file", mLocalRetriever);
  mRetriever->addSchemaRetriever("package", mPackageRetriever);
}

void DartLoader::addPackageDirectory(const std::string& _packageName,
                                     const std::string& _packageDirectory)
{
  mPackageRetriever->addPackageDirectory(_packageName, _packageDirectory);
}

dynamics::SkeletonPtr DartLoader::parseSkeleton(
  const common::Uri& _uri,
  const common::ResourceRetrieverPtr& _resourceRetriever)
{
  const common::ResourceRetrieverPtr resourceRetriever
    = getResourceRetriever(_resourceRetriever);

  std::string content;
  if (!readFileToString(resourceRetriever, _uri, content))
    return nullptr;

  // Use urdfdom to load the URDF file.
  const ModelInterfacePtr urdfInterface = urdf::parseURDF(content);
  if(!urdfInterface)
  {
    dtwarn << "[DartLoader::readSkeleton] Failed loading URDF file '"
           << _uri.toString() << "'.\n";
    return nullptr;
  }

  return modelInterfaceToSkeleton(urdfInterface.get(), _uri, resourceRetriever);
}

dynamics::SkeletonPtr DartLoader::parseSkeletonString(
    const std::string& _urdfString, const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _resourceRetriever)
{
  if(_urdfString.empty())
  {
    dtwarn << "[DartLoader::parseSkeletonString] A blank string cannot be "
           << "parsed into a Skeleton. Returning a nullptr\n";
    return nullptr;
  }

  ModelInterfacePtr urdfInterface = urdf::parseURDF(_urdfString);
  if(!urdfInterface)
  {
    dtwarn << "[DartLoader::parseSkeletonString] Failed loading URDF.\n";
    return nullptr;
  }

  return modelInterfaceToSkeleton(
    urdfInterface.get(), _baseUri, getResourceRetriever(_resourceRetriever));
}

simulation::WorldPtr DartLoader::parseWorld(
  const common::Uri& _uri,
  const common::ResourceRetrieverPtr& _resourceRetriever)
{
  const common::ResourceRetrieverPtr resourceRetriever
    = getResourceRetriever(_resourceRetriever);

  std::string content;
  if (!readFileToString(resourceRetriever, _uri, content))
    return nullptr;

  return parseWorldString(content, _uri, _resourceRetriever);
}

simulation::WorldPtr DartLoader::parseWorldString(
    const std::string& _urdfString, const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _resourceRetriever)
{
  const common::ResourceRetrieverPtr resourceRetriever
    = getResourceRetriever(_resourceRetriever);

  if(_urdfString.empty())
  {
    dtwarn << "[DartLoader::parseWorldString] A blank string cannot be "
           << "parsed into a World. Returning a nullptr\n";
    return nullptr;
  }

  std::shared_ptr<urdf_parsing::World> worldInterface =
      urdf_parsing::parseWorldURDF(_urdfString, _baseUri);

  if(!worldInterface)
  {
    dtwarn << "[DartLoader::parseWorldString] Failed loading URDF.\n";
    return nullptr;
  }

  simulation::WorldPtr world(new simulation::World);

  for(size_t i = 0; i < worldInterface->models.size(); ++i)
  {
    const urdf_parsing::Entity& entity = worldInterface->models[i];
    dynamics::SkeletonPtr skeleton = modelInterfaceToSkeleton(
      entity.model.get(), entity.uri, resourceRetriever);

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
 * @function modelInterfaceToSkeleton
 * @brief Read the ModelInterface and spits out a Skeleton object
 */
dynamics::SkeletonPtr DartLoader::modelInterfaceToSkeleton(
  const urdf::ModelInterface* _model,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _resourceRetriever)
{
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
      dynamics::BodyNode::Properties rootProperties;
      if (!createDartNodeProperties(root, rootProperties, _baseUri, _resourceRetriever))
        return nullptr;

      rootNode = createDartJointAndNode(
        root->parent_joint.get(), rootProperties, nullptr, skeleton,
        _baseUri, _resourceRetriever);
      if(nullptr == rootNode)
      {
        dterr << "[DartLoader::modelInterfaceToSkeleton] Failed to create root node!\n";
        return nullptr;
      }
    }
  }
  else
  {
    dynamics::BodyNode::Properties rootProperties;
    if (!createDartNodeProperties(root, rootProperties, _baseUri, _resourceRetriever))
      return nullptr;

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
    if (!createSkeletonRecursive(
           skeleton, root->child_links[i].get(), rootNode,
           _baseUri, _resourceRetriever))
      return nullptr;

  }

  return skeleton;
}

bool DartLoader::createSkeletonRecursive(
  dynamics::SkeletonPtr _skel,
  const urdf::Link* _lk,
  dynamics::BodyNode* _parentNode,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _resourceRetriever)
{
  dynamics::BodyNode::Properties properties;
  if (!createDartNodeProperties(_lk, properties, _baseUri, _resourceRetriever))
    return false;

  dynamics::BodyNode* node = createDartJointAndNode(
    _lk->parent_joint.get(), properties, _parentNode, _skel,
    _baseUri, _resourceRetriever);
  if(!node)
    return false;
  
  for(size_t i = 0; i < _lk->child_links.size(); ++i)
  {
    if (!createSkeletonRecursive(_skel, _lk->child_links[i].get(), node,
                                 _baseUri, _resourceRetriever))
      return false;
  }
  return true;
}


/**
 * @function readXml
 */
bool DartLoader::readFileToString(
  const common::ResourceRetrieverPtr& _resourceRetriever,
  const common::Uri& _uri,
  std::string &_output)
{
  const common::ResourcePtr resource = _resourceRetriever->retrieve(_uri);
  if (!resource)
    return false;

  // Safe because std::string is guaranteed to be contiguous in C++11.
  const size_t size = resource->getSize();
  _output.resize(size);
  resource->read(&_output.front(), size, 1);

  return true;
}

/**
 * @function createDartJoint
 */
dynamics::BodyNode* DartLoader::createDartJointAndNode(
    const urdf::Joint* _jt,
    const dynamics::BodyNode::Properties& _body,
    dynamics::BodyNode* _parent,
    dynamics::SkeletonPtr _skeleton,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _resourceRetriever)
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

    // If the zero position is out of our limits, we should change the initial
    // position instead of assuming zero.
    if(_jt->limits->lower > 0 || _jt->limits->upper < 0)
    {
      if(std::isfinite(_jt->limits->lower)
         && std::isfinite(_jt->limits->upper))
        singleDof.mInitialPosition =
            (_jt->limits->lower + _jt->limits->upper) / 2.0;
      else if(std::isfinite(_jt->limits->lower))
        singleDof.mInitialPosition = _jt->limits->lower;
      else if(std::isfinite(_jt->limits->upper))
        singleDof.mInitialPosition = _jt->limits->upper;

      // Any other case means that the limits are both +inf, both -inf, or
      // either of them is NaN. This should generate warnings elsewhere.

      // Apply the same logic to mRestPosition.
      singleDof.mRestPosition = singleDof.mInitialPosition;
    }
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
      return nullptr;
    }
  }

  return pair.second;
}

/**
 * @function createDartNode
 */
bool DartLoader::createDartNodeProperties(
  const urdf::Link* _lk,
  dynamics::BodyNode::Properties &node,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _resourceRetriever)
{
  node.mName = _lk->name;
  
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
  for(size_t i = 0; i < _lk->visual_array.size(); i++)
  {
    dynamics::ShapePtr shape = createShape(
      _lk->visual_array[i].get(), _baseUri, _resourceRetriever);

    if(shape)
      node.mVizShapes.push_back(shape);
    else
      return false;
  }

  // Set collision information
  for(size_t i = 0; i < _lk->collision_array.size(); i++) {
    dynamics::ShapePtr shape = createShape(
      _lk->collision_array[i].get(), _baseUri, _resourceRetriever);

    if (shape)
      node.mColShapes.push_back(shape);
    else
      return false;
  }

  return true;
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
dynamics::ShapePtr DartLoader::createShape(
  const VisualOrCollision* _vizOrCol,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _resourceRetriever)
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
    // Resolve relative URIs.
    common::Uri relativeUri, absoluteUri;
    if(!absoluteUri.fromRelativeUri(_baseUri, mesh->filename))
    {
      dtwarn << "[DartLoader::createShape] Failed resolving mesh URI '"
             << mesh->filename << "' relative to '" << _baseUri.toString()
             << "'.\n";
      return nullptr;
    }

    // Load the mesh.
    const std::string resolvedUri = absoluteUri.toString();
    const aiScene* scene = dynamics::MeshShape::loadMesh(
      resolvedUri, _resourceRetriever);
    if (!scene)
      return nullptr;

    const Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
    shape = Eigen::make_aligned_shared<dynamics::MeshShape>(
      scale, scene, resolvedUri, _resourceRetriever);
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

common::ResourceRetrieverPtr DartLoader::getResourceRetriever(
  const common::ResourceRetrieverPtr& _resourceRetriever)
{
  if (_resourceRetriever)
    return _resourceRetriever;
  else
    return mRetriever;
}

template dynamics::ShapePtr DartLoader::createShape<urdf::Visual>(
  const urdf::Visual* _vizOrCol,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _resourceRetriever);
template dynamics::ShapePtr DartLoader::createShape<urdf::Collision>(
  const urdf::Collision* _vizOrCol,
  const common::Uri& _baseUri,
  const common::ResourceRetrieverPtr& _resourceRetriever);

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
