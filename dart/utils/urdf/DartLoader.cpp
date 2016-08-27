/*
 * Copyright (c) 2011-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2011-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include "dart/utils/urdf/DartLoader.hpp"

#include <map>
#include <iostream>
#include <fstream>

#include <urdf_parser/urdf_parser.h>
#include <urdf_world/world.h>

#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/urdf/URDFTypes.hpp"
#include "dart/utils/urdf/urdf_world_parser.hpp"

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

  for(std::size_t i = 0; i < worldInterface->models.size(); ++i)
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

      const auto result
          = createShapeNodes(root, rootNode, _baseUri, _resourceRetriever);
      if(!result)
        return nullptr;
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
            dynamics::GenericJoint<math::SE3Space>::Properties(
            dynamics::Joint::Properties("rootJoint"))),
          rootProperties);
    rootNode = pair.second;

    const auto result
        = createShapeNodes(root, rootNode, _baseUri, _resourceRetriever);
    if(!result)
      return nullptr;
  }

  for(std::size_t i = 0; i < root->child_links.size(); i++)
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

  const auto result = createShapeNodes(_lk, node, _baseUri, _resourceRetriever);
  if(!result)
    return false;
  
  for(std::size_t i = 0; i < _lk->child_links.size(); ++i)
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
  const std::size_t size = resource->getSize();
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
    const common::Uri& /*_baseUri*/,
    const common::ResourceRetrieverPtr& /*_resourceRetriever*/)
{
  dynamics::Joint::Properties basicProperties;

  basicProperties.mName = _jt->name;
  basicProperties.mT_ParentBodyToJoint =
      toEigen(_jt->parent_to_joint_origin_transform);

  dynamics::GenericJoint<math::R1Space>::UniqueProperties singleDof;
  if(_jt->limits)
  {
    singleDof.mPositionLowerLimits[0] = _jt->limits->lower;
    singleDof.mPositionUpperLimits[0] = _jt->limits->upper;
    singleDof.mVelocityLowerLimits[0] = -_jt->limits->velocity;
    singleDof.mVelocityUpperLimits[0] =  _jt->limits->velocity;
    singleDof.mForceLowerLimits[0] = -_jt->limits->effort;
    singleDof.mForceUpperLimits[0] =  _jt->limits->effort;

    // If the zero position is out of our limits, we should change the initial
    // position instead of assuming zero.
    if(_jt->limits->lower > 0 || _jt->limits->upper < 0)
    {
      if(std::isfinite(_jt->limits->lower)
         && std::isfinite(_jt->limits->upper))
        singleDof.mInitialPositions[0] =
            (_jt->limits->lower + _jt->limits->upper) / 2.0;
      else if(std::isfinite(_jt->limits->lower))
        singleDof.mInitialPositions[0] = _jt->limits->lower;
      else if(std::isfinite(_jt->limits->upper))
        singleDof.mInitialPositions[0] = _jt->limits->upper;

      // Any other case means that the limits are both +inf, both -inf, or
      // either of them is NaN. This should generate warnings elsewhere.

      // Apply the same logic to mRestPosition.
      singleDof.mRestPositions = singleDof.mInitialPositions;
    }
  }

  if(_jt->dynamics)
    singleDof.mDampingCoefficients[0] = _jt->dynamics->damping;

  std::pair<dynamics::Joint*, dynamics::BodyNode*> pair;
  switch(_jt->type)
  {
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:
    {
      dynamics::RevoluteJoint::Properties properties(
            dynamics::GenericJoint<math::R1Space>::Properties(basicProperties, singleDof),
            toEigen(_jt->axis));

      pair = _skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
            _parent, properties, _body);

      break;
    }
    case urdf::Joint::PRISMATIC:
    {
      dynamics::PrismaticJoint::Properties properties(
            dynamics::GenericJoint<math::R1Space>::Properties(basicProperties, singleDof),
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
      dynamics::GenericJoint<math::SE3Space>::Properties properties(
            basicProperties);

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
  const common::Uri& /*_baseUri*/,
  const common::ResourceRetrieverPtr& /*_resourceRetriever*/)
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

  return true;
}

void setMaterial(dynamics::VisualAspect* visualAspect, const urdf::Visual* viz)
{
  if(viz->material)
  {
    visualAspect->setColor(Eigen::Vector3d(viz->material->color.r,
                                          viz->material->color.g,
                                          viz->material->color.b));
  }
}

void setMaterial(dynamics::ShapePtr /*_shape*/,
                 const urdf::Collision* /*_col*/)
{
  // Do nothing
}

//==============================================================================
bool DartLoader::createShapeNodes(
  const urdf::Link* lk,
  dynamics::BodyNode* bodyNode,
  const common::Uri& baseUri,
  const common::ResourceRetrieverPtr& resourceRetriever)
{
  // Set visual information
  for(auto visual : lk->visual_array)
  {
    dynamics::ShapePtr shape
        = createShape(visual.get(), baseUri, resourceRetriever);

    if(shape)
    {
      auto shapeNode = bodyNode->createShapeNodeWith<dynamics::VisualAspect>(shape);
      shapeNode->setRelativeTransform(toEigen(visual->origin));
      setMaterial(shapeNode->getVisualAspect(), visual.get());
    }
    else
    {
      return false;
    }
  }

  // Set collision information
  for(auto collision : lk->collision_array)
  {
    dynamics::ShapePtr shape
        = createShape(collision.get(), baseUri, resourceRetriever);

    if (shape)
    {
      auto shapeNode = bodyNode->createShapeNodeWith<
          dynamics::CollisionAspect, dynamics::DynamicsAspect>(shape);
      shapeNode->setRelativeTransform(toEigen(collision->origin));
    }
    else
      return false;
  }

  return true;
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
    shape = dynamics::ShapePtr(new dynamics::SphereShape(sphere->radius));
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
