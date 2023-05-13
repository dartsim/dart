/*
 * Copyright (c) 2011-2023, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include "dart/io/urdf/DartLoader.hpp"

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Shape.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/io/DartResourceRetriever.hpp"
#include "dart/io/urdf/IncludeUrdf.hpp"
#include "dart/io/urdf/urdf_world_parser.hpp"
#include "dart/simulation/World.hpp"

#include <fstream>
#include <iostream>
#include <map>

namespace dart {
namespace io {

using ModelInterfacePtr = std::shared_ptr<urdf::ModelInterface>;

//==============================================================================
DartLoader::Options::Options(
    common::ResourceRetrieverPtr resourceRetriever,
    RootJointType defaultRootJointType,
    const dynamics::Inertia& defaultInertia)
  : mResourceRetriever(std::move(resourceRetriever)),
    mDefaultRootJointType(defaultRootJointType),
    mDefaultInertia(defaultInertia)
{
  // Do nothing
}

//==============================================================================
DartLoader::DartLoader(const Options& options)
  : mOptions(options),
    mLocalRetriever(new common::LocalResourceRetriever),
    mPackageRetriever(new io::PackageResourceRetriever(mLocalRetriever)),
    mRetriever(new io::CompositeResourceRetriever)
{
  mRetriever->addSchemaRetriever("file", mLocalRetriever);
  mRetriever->addSchemaRetriever("package", mPackageRetriever);
  mRetriever->addSchemaRetriever("dart", DartResourceRetriever::create());
}

//==============================================================================
void DartLoader::setOptions(const Options& options)
{
  mOptions = options;
}

//==============================================================================
const DartLoader::Options& DartLoader::getOptions() const
{
  return mOptions;
}

//==============================================================================
void DartLoader::addPackageDirectory(
    const std::string& _packageName, const std::string& _packageDirectory)
{
  mPackageRetriever->addPackageDirectory(_packageName, _packageDirectory);
}

//==============================================================================
dynamics::SkeletonPtr DartLoader::parseSkeleton(const common::Uri& uri)
{
  const common::ResourceRetrieverPtr resourceRetriever
      = getResourceRetriever(mOptions.mResourceRetriever);

  std::string content;
  if (!readFileToString(resourceRetriever, uri, content))
    return nullptr;

  // Use urdfdom to load the URDF file.
  const ModelInterfacePtr urdfInterface = urdf::parseURDF(content);
  if (!urdfInterface) {
    dtwarn << "[DartLoader::readSkeleton] Failed loading URDF file '"
           << uri.toString() << "'.\n";
    return nullptr;
  }

  return modelInterfaceToSkeleton(
      urdfInterface.get(), uri, resourceRetriever, mOptions);
}

//==============================================================================
dynamics::SkeletonPtr DartLoader::parseSkeletonString(
    const std::string& urdfString, const common::Uri& baseUri)
{
  if (urdfString.empty()) {
    dtwarn << "[DartLoader::parseSkeletonString] A blank string cannot be "
           << "parsed into a Skeleton. Returning a nullptr\n";
    return nullptr;
  }

  ModelInterfacePtr urdfInterface = urdf::parseURDF(urdfString);
  if (!urdfInterface) {
    dtwarn << "[DartLoader::parseSkeletonString] Failed loading URDF.\n";
    return nullptr;
  }

  return modelInterfaceToSkeleton(
      urdfInterface.get(),
      baseUri,
      getResourceRetriever(mOptions.mResourceRetriever),
      mOptions);
}

//==============================================================================
simulation::WorldPtr DartLoader::parseWorld(const common::Uri& uri)
{
  const common::ResourceRetrieverPtr resourceRetriever
      = getResourceRetriever(mOptions.mResourceRetriever);

  std::string content;
  if (!readFileToString(resourceRetriever, uri, content))
    return nullptr;

  return parseWorldString(content, uri);
}

//==============================================================================
simulation::WorldPtr DartLoader::parseWorldString(
    const std::string& urdfString, const common::Uri& baseUri)
{
  const common::ResourceRetrieverPtr resourceRetriever
      = getResourceRetriever(mOptions.mResourceRetriever);

  if (urdfString.empty()) {
    dtwarn << "[DartLoader::parseWorldString] A blank string cannot be "
           << "parsed into a World. Returning a nullptr\n";
    return nullptr;
  }

  std::shared_ptr<urdf_parsing::World> worldInterface
      = urdf_parsing::parseWorldURDF(urdfString, baseUri, resourceRetriever);

  if (!worldInterface) {
    dtwarn << "[DartLoader::parseWorldString] Failed loading URDF.\n";
    return nullptr;
  }

  simulation::WorldPtr world = simulation::World::create();

  for (std::size_t i = 0; i < worldInterface->models.size(); ++i) {
    const urdf_parsing::Entity& entity = worldInterface->models[i];
    dynamics::SkeletonPtr skeleton = modelInterfaceToSkeleton(
        entity.model.get(), entity.uri, resourceRetriever, mOptions);

    if (!skeleton) {
      dtwarn << "[DartLoader::parseWorldString] Robot "
             << worldInterface->models[i].model->getName()
             << " was not correctly parsed!\n";
      continue;
    }

    // Initialize position and RPY
    dynamics::Joint* rootJoint = skeleton->getRootBodyNode()->getParentJoint();
    math::Isometry3d transform = toEigen(worldInterface->models[i].origin);

    if (dynamic_cast<dynamics::FreeJoint*>(rootJoint))
      rootJoint->setPositions(
          dynamics::FreeJoint::convertToPositions(transform));
    else
      rootJoint->setTransformFromParentBodyNode(transform);

    world->addSkeleton(skeleton);
  }

  return world;
}

//==============================================================================
dynamics::SkeletonPtr DartLoader::modelInterfaceToSkeleton(
    const urdf::ModelInterface* model,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& resourceRetriever,
    const Options& options)
{
  dynamics::SkeletonPtr skeleton = dynamics::Skeleton::create(model->getName());

  const urdf::Link* root = model->getRoot().get();

  // If the link name is "world" then the link is regarded as the inertial
  // frame rather than a body in the robot model so we don't create a BodyNode
  // for it. This is not officially specified in the URDF spec, but "world" is
  // practically treated as a keyword.
  if (root->name == "world") {
    if (model->getRoot()->child_links.size() > 1) {
      dtwarn << "[DartLoader::modelInterfaceToSkeleton] The world link has "
             << "more than one child links. This leads to creating a "
             << "multi-tree robot. Multi-tree robot is supported by DART, but "
             << "not the URDF standard. Please consider changing the robot "
             << "model as a single tree robot.\n";
    }

    for (std::size_t i = 0; i < root->child_links.size(); i++) {
      if (!createSkeletonRecursive(
              model,
              skeleton,
              root->child_links[i].get(),
              nullptr,
              baseUri,
              resourceRetriever,
              options)) {
        return nullptr;
      }
    }
  } else {
    if (!createSkeletonRecursive(
            model,
            skeleton,
            root,
            nullptr,
            baseUri,
            resourceRetriever,
            options)) {
      return nullptr;
    }
  }

  // Find mimic joints
  for (std::size_t i = 0; i < root->child_links.size(); i++)
    addMimicJointsRecursive(model, skeleton, root->child_links[i].get());

  return skeleton;
}

//==============================================================================
bool DartLoader::createSkeletonRecursive(
    const urdf::ModelInterface* model,
    dynamics::SkeletonPtr skel,
    const urdf::Link* lk,
    dynamics::BodyNode* parentNode,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& resourceRetriever,
    const Options& options)
{
  assert(lk);

  if (parentNode != nullptr && lk->name == "world") {
    dtwarn << "[DartLoader] Link name 'world' is reserved for the inertial "
           << "frame. Consider changing the name to something else.\n";
  }

  dynamics::BodyNode::Properties properties;
  if (!createDartNodeProperties(
          lk, properties, baseUri, resourceRetriever, options))
    return false;

  dynamics::BodyNode* node = createDartJointAndNode(
      lk->parent_joint.get(), properties, parentNode, skel, options);

  if (!node)
    return false;

  const auto result
      = createShapeNodes(model, lk, node, baseUri, resourceRetriever);

  if (!result)
    return false;

  for (std::size_t i = 0; i < lk->child_links.size(); ++i) {
    if (!createSkeletonRecursive(
            model,
            skel,
            lk->child_links[i].get(),
            node,
            baseUri,
            resourceRetriever,
            options)) {
      return false;
    }
  }
  return true;
}

bool DartLoader::addMimicJointsRecursive(
    const urdf::ModelInterface* model,
    dynamics::SkeletonPtr _skel,
    const urdf::Link* _lk)
{
  const urdf::Joint* jt = _lk->parent_joint.get();
  if (jt->mimic) {
    const std::string& jointName = jt->name;
    const double multiplier = jt->mimic->multiplier;
    const double offset = jt->mimic->offset;
    const std::string& mimicJointName = jt->mimic->joint_name;

    dynamics::Joint* joint = _skel->getJoint(jointName);
    if (!joint) {
      dterr << "Failed to configure a mimic joint [" << jointName
            << "] because the joint isn't found in Skeleton ["
            << _skel->getName() << "]\n.";
      return false;
    }

    const dynamics::Joint* mimicJoint = _skel->getJoint(mimicJointName);
    if (!mimicJoint) {
      dterr << "Failed to configure a mimic joint [" << jointName
            << "] because the joint to mimic [" << mimicJoint
            << "] isn't found in Skeleton [" << _skel->getName() << "]\n.";
      return false;
    }

    joint->setActuatorType(dynamics::Joint::MIMIC);
    joint->setMimicJoint(mimicJoint, multiplier, offset);
  }

  for (std::size_t i = 0; i < _lk->child_links.size(); ++i) {
    if (!addMimicJointsRecursive(model, _skel, _lk->child_links[i].get()))
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
    std::string& _output)
{
  const common::ResourcePtr resource = _resourceRetriever->retrieve(_uri);
  if (!resource)
    return false;

  _output = _resourceRetriever->readAll(_uri);

  return true;
}

dynamics::BodyNode* createDartJointAndNodeForRoot(
    const dynamics::BodyNode::Properties& _body,
    dynamics::BodyNode* _parent,
    dynamics::SkeletonPtr _skeleton,
    const DartLoader::Options& options)
{
  dynamics::Joint::Properties basicProperties("rootJoint");

  dynamics::GenericJoint<math::R1Space>::UniqueProperties singleDof;
  std::pair<dynamics::Joint*, dynamics::BodyNode*> pair;
  switch (options.mDefaultRootJointType) {
    case DartLoader::RootJointType::FLOATING: {
      dynamics::GenericJoint<math::SE3Space>::Properties properties(
          basicProperties);
      pair = _skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          _parent, properties, _body);
      break;
    }
    case DartLoader::RootJointType::FIXED: {
      pair = _skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
          _parent, basicProperties, _body);
      break;
    }
    default: {
      dterr << "Unsupported RootJointType '"
            << static_cast<int>(options.mDefaultRootJointType)
            << "'. Using FLOATING instead.\n";
      dynamics::GenericJoint<math::SE3Space>::Properties properties(
          basicProperties);
      pair = _skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          _parent, properties, _body);
    }
  }

  return pair.second;
}

/**
 * @function createDartJoint
 */
dynamics::BodyNode* DartLoader::createDartJointAndNode(
    const urdf::Joint* _jt,
    const dynamics::BodyNode::Properties& _body,
    dynamics::BodyNode* _parent,
    dynamics::SkeletonPtr _skeleton,
    const Options& options)
{
  // Special case for the root link (A root link doesn't have the parent joint).
  // We don't have sufficient information what the joint type should be for the
  // root link. So we create the root joint by the specified flgas.
  if (!_jt) {
    return createDartJointAndNodeForRoot(_body, _parent, _skeleton, options);
  }

  dynamics::Joint::Properties basicProperties;

  basicProperties.mName = _jt->name;
  basicProperties.mT_ParentBodyToJoint
      = toEigen(_jt->parent_to_joint_origin_transform);

  dynamics::GenericJoint<math::R1Space>::UniqueProperties singleDof;
  if (_jt->limits) {
    singleDof.mPositionLowerLimits[0] = _jt->limits->lower;
    singleDof.mPositionUpperLimits[0] = _jt->limits->upper;
    singleDof.mVelocityLowerLimits[0] = -_jt->limits->velocity;
    singleDof.mVelocityUpperLimits[0] = _jt->limits->velocity;
    singleDof.mForceLowerLimits[0] = -_jt->limits->effort;
    singleDof.mForceUpperLimits[0] = _jt->limits->effort;

    // If the zero position is out of our limits, we should change the initial
    // position instead of assuming zero.
    if (_jt->limits->lower > 0 || _jt->limits->upper < 0) {
      if (std::isfinite(_jt->limits->lower)
          && std::isfinite(_jt->limits->upper))
        singleDof.mInitialPositions[0]
            = (_jt->limits->lower + _jt->limits->upper) / 2.0;
      else if (std::isfinite(_jt->limits->lower))
        singleDof.mInitialPositions[0] = _jt->limits->lower;
      else if (std::isfinite(_jt->limits->upper))
        singleDof.mInitialPositions[0] = _jt->limits->upper;

      // Any other case means that the limits are both +inf, both -inf, or
      // either of them is NaN. This should generate warnings elsewhere.

      // Apply the same logic to mRestPosition.
      singleDof.mRestPositions = singleDof.mInitialPositions;
    }
  }

  if (_jt->dynamics) {
    singleDof.mDampingCoefficients[0] = _jt->dynamics->damping;
    singleDof.mFrictions[0] = _jt->dynamics->friction;
  }

  std::pair<dynamics::Joint*, dynamics::BodyNode*> pair;
  switch (_jt->type) {
    case urdf::Joint::CONTINUOUS: {
      // We overwrite joint position limits to negative/positive infinities
      // for "continuous" joint. The URDF parser, by default, either reads
      // the limits, if specified for this joint, or sets them to 0.
      singleDof.mPositionLowerLimits[0] = -math::inf();
      singleDof.mPositionUpperLimits[0] = math::inf();

      // This joint is still revolute but with no joint limits
      dynamics::RevoluteJoint::Properties properties(
          dynamics::GenericJoint<math::R1Space>::Properties(
              basicProperties, singleDof),
          toEigen(_jt->axis));

      pair = _skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          _parent, properties, _body);

      break;
    }
    case urdf::Joint::REVOLUTE: {
      dynamics::RevoluteJoint::Properties properties(
          dynamics::GenericJoint<math::R1Space>::Properties(
              basicProperties, singleDof),
          toEigen(_jt->axis));

      pair = _skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
          _parent, properties, _body);

      break;
    }
    case urdf::Joint::PRISMATIC: {
      dynamics::PrismaticJoint::Properties properties(
          dynamics::GenericJoint<math::R1Space>::Properties(
              basicProperties, singleDof),
          toEigen(_jt->axis));

      pair = _skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
          _parent, properties, _body);

      break;
    }
    case urdf::Joint::FIXED: {
      pair = _skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
          _parent, basicProperties, _body);
      break;
    }
    case urdf::Joint::FLOATING: {
      dynamics::GenericJoint<math::SE3Space>::Properties properties(
          basicProperties);

      pair = _skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          _parent, properties, _body);
      break;
    }
    case urdf::Joint::PLANAR: {
      pair = _skeleton->createJointAndBodyNodePair<dynamics::PlanarJoint>(
          _parent, dynamics::PlanarJoint::Properties(basicProperties), _body);
      // TODO(MXG): Should we read in position limits? The URDF limits
      // specification only offers one dimension of limits, but a PlanarJoint is
      // three-dimensional. Should we assume that position limits apply to both
      // coordinates equally? Or just don't accept the position limits at all?
      break;
    }
    default: {
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
    dynamics::BodyNode::Properties& node,
    const common::Uri& /*_baseUri*/,
    const common::ResourceRetrieverPtr& /*_resourceRetriever*/,
    const Options& options)
{
  node.mName = _lk->name;

  // Load Inertial information
  if (_lk->inertial) {
    urdf::Pose origin = _lk->inertial->origin;
    node.mInertia.setLocalCOM(toEigen(origin.position));
    node.mInertia.setMass(_lk->inertial->mass);

    math::Matrix3d J;
    J << _lk->inertial->ixx, _lk->inertial->ixy, _lk->inertial->ixz,
        _lk->inertial->ixy, _lk->inertial->iyy, _lk->inertial->iyz,
        _lk->inertial->ixz, _lk->inertial->iyz, _lk->inertial->izz;
    math::Matrix3d R(math::Quaterniond(
        origin.rotation.w,
        origin.rotation.x,
        origin.rotation.y,
        origin.rotation.z));
    J = R * J * R.transpose();

    node.mInertia.setMoment(
        J(0, 0), J(1, 1), J(2, 2), J(0, 1), J(0, 2), J(1, 2));
  } else {
    node.mInertia = options.mDefaultInertia;
  }

  return true;
}

//==============================================================================
void setMaterial(
    const urdf::ModelInterface* model,
    dynamics::VisualAspect* visualAspect,
    const urdf::Visual* viz)
{
  if (viz->material) {
    urdf::Color urdf_color = viz->material->color;

    const auto& m_it = model->materials_.find(viz->material_name);
    if (m_it != model->materials_.end())
      urdf_color = m_it->second->color;

    const math::Vector4d color(
        static_cast<double>(urdf_color.r),
        static_cast<double>(urdf_color.g),
        static_cast<double>(urdf_color.b),
        static_cast<double>(urdf_color.a));
    visualAspect->setColor(color);
    auto shapeFrame = visualAspect->getComposite();
    auto shape = shapeFrame->getShape();
    if (auto mesh = std::dynamic_pointer_cast<dynamics::MeshShape>(shape))
      mesh->setColorMode(dynamics::MeshShape::ColorMode::SHAPE_COLOR);
  }
}

//==============================================================================
bool DartLoader::createShapeNodes(
    const urdf::ModelInterface* model,
    const urdf::Link* lk,
    dynamics::BodyNode* bodyNode,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& resourceRetriever)
{
  // Set visual information
  for (auto visual : lk->visual_array) {
    dynamics::ShapePtr shape
        = createShape(visual.get(), baseUri, resourceRetriever);

    if (shape) {
      auto shapeNode
          = bodyNode->createShapeNodeWith<dynamics::VisualAspect>(shape);
      if (!visual->name.empty())
        shapeNode->setName(visual->name);
      shapeNode->setRelativeTransform(toEigen(visual->origin));
      setMaterial(model, shapeNode->getVisualAspect(), visual.get());
    } else {
      return false;
    }
  }

  // Set collision information
  for (auto collision : lk->collision_array) {
    dynamics::ShapePtr shape
        = createShape(collision.get(), baseUri, resourceRetriever);

    if (shape) {
      auto shapeNode = bodyNode->createShapeNodeWith<
          dynamics::CollisionAspect,
          dynamics::DynamicsAspect>(shape);
      if (!collision->name.empty())
        shapeNode->setName(collision->name);
      shapeNode->setRelativeTransform(toEigen(collision->origin));
    } else
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
  if (urdf::Sphere* sphere
      = dynamic_cast<urdf::Sphere*>(_vizOrCol->geometry.get())) {
    shape = dynamics::ShapePtr(new dynamics::SphereShape(sphere->radius));
  }
  // Box
  else if (
      urdf::Box* box = dynamic_cast<urdf::Box*>(_vizOrCol->geometry.get())) {
    shape = dynamics::ShapePtr(new dynamics::BoxShape(
        math::Vector3d(box->dim.x, box->dim.y, box->dim.z)));
  }
  // Cylinder
  else if (
      urdf::Cylinder* cylinder
      = dynamic_cast<urdf::Cylinder*>(_vizOrCol->geometry.get())) {
    shape = dynamics::ShapePtr(
        new dynamics::CylinderShape(cylinder->radius, cylinder->length));
  }
  // Mesh
  else if (
      urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(_vizOrCol->geometry.get())) {
    // Resolve relative URIs.
    common::Uri relativeUri, absoluteUri;
    if (!absoluteUri.fromRelativeUri(_baseUri, mesh->filename)) {
      dtwarn << "[DartLoader::createShape] Failed resolving mesh URI '"
             << mesh->filename << "' relative to '" << _baseUri.toString()
             << "'.\n";
      return nullptr;
    }

    // Load the mesh.
    const std::string resolvedUri = absoluteUri.toString();
    const aiScene* scene
        = dynamics::MeshShape::loadMesh(resolvedUri, _resourceRetriever);
    if (!scene)
      return nullptr;

    const math::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
    shape = std::make_shared<dynamics::MeshShape>(
        scale, scene, resolvedUri, _resourceRetriever);
  }
  // Unknown geometry type
  else {
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
math::Isometry3d DartLoader::toEigen(const urdf::Pose& _pose)
{
  math::Quaterniond quat;
  _pose.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
  math::Isometry3d transform(quat);
  transform.translation()
      = math::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z);
  return transform;
}

math::Vector3d DartLoader::toEigen(const urdf::Vector3& _vector)
{
  return math::Vector3d(_vector.x, _vector.y, _vector.z);
}

} // namespace io
} // namespace dart
