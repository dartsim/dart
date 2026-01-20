/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/utils/urdf/urdf_parser.hpp"

#include "dart/common/macros.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/planar_joint.hpp"
#include "dart/dynamics/prismatic_joint.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/dynamics/weld_joint.hpp"
#include "dart/simulation/world.hpp"
#include "dart/utils/dart_resource_retriever.hpp"
#include "dart/utils/mesh_loader.hpp"
#include "dart/utils/urdf/backward_compatibility.hpp"
#include "dart/utils/urdf/include_urdf.hpp"
#include "dart/utils/urdf/urdf_world_parser.hpp"

#include <tinyxml2.h>

#include <fstream>
#include <iostream>
#include <map>
#include <unordered_map>

#include <cmath>

namespace dart {
namespace utils {

using ModelInterfacePtr = std::shared_ptr<urdf::ModelInterface>;

//==============================================================================
UrdfParser::Options::Options(
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
UrdfParser::UrdfParser(const Options& options)
  : mOptions(options),
    mLocalRetriever(new common::LocalResourceRetriever),
    mPackageRetriever(new utils::PackageResourceRetriever(mLocalRetriever)),
    mRetriever(new utils::CompositeResourceRetriever)
{
  mRetriever->addSchemaRetriever("file", mLocalRetriever);
  mRetriever->addSchemaRetriever("package", mPackageRetriever);
  mRetriever->addSchemaRetriever("dart", DartResourceRetriever::create());
}

//==============================================================================
void UrdfParser::setOptions(const Options& options)
{
  mOptions = options;
}

//==============================================================================
const UrdfParser::Options& UrdfParser::getOptions() const
{
  return mOptions;
}

//==============================================================================
void UrdfParser::addPackageDirectory(
    std::string_view packageName, std::string_view packageDirectory)
{
  mPackageRetriever->addPackageDirectory(packageName, packageDirectory);
}

//==============================================================================
dynamics::SkeletonPtr UrdfParser::parseSkeleton(const common::Uri& uri)
{
  const common::ResourceRetrieverPtr resourceRetriever
      = getResourceRetriever(mOptions.mResourceRetriever);

  std::string content;
  if (!readFileToString(resourceRetriever, uri, content))
    return nullptr;

  // Use urdfdom to load the URDF file.
  const ModelInterfacePtr urdfInterface = urdf::parseURDF(content);
  if (!urdfInterface) {
    DART_WARN("Failed loading URDF file '{}'.", uri.toString());
    return nullptr;
  }

  ParseContext context;
  context.mTransmissions = parseTransmissions(content);

  return modelInterfaceToSkeleton(
      urdfInterface.get(), uri, resourceRetriever, mOptions, &context);
}

//==============================================================================
dynamics::SkeletonPtr UrdfParser::parseSkeletonString(
    std::string_view urdfString, const common::Uri& baseUri)
{
  if (urdfString.empty()) {
    DART_WARN(
        "A blank string cannot be parsed into a Skeleton. Returning a nullptr");
    return nullptr;
  }

  const std::string urdfStringCopy(urdfString);
  ModelInterfacePtr urdfInterface = urdf::parseURDF(urdfStringCopy);
  if (!urdfInterface) {
    DART_WARN("Failed loading URDF.");
    return nullptr;
  }

  ParseContext context;
  context.mTransmissions = parseTransmissions(urdfStringCopy);

  return modelInterfaceToSkeleton(
      urdfInterface.get(),
      baseUri,
      getResourceRetriever(mOptions.mResourceRetriever),
      mOptions,
      &context);
}

//==============================================================================
simulation::WorldPtr UrdfParser::parseWorld(const common::Uri& uri)
{
  const common::ResourceRetrieverPtr resourceRetriever
      = getResourceRetriever(mOptions.mResourceRetriever);

  std::string content;
  if (!readFileToString(resourceRetriever, uri, content))
    return nullptr;

  return parseWorldString(content, uri);
}

//==============================================================================
simulation::WorldPtr UrdfParser::parseWorldString(
    std::string_view urdfString, const common::Uri& baseUri)
{
  const common::ResourceRetrieverPtr resourceRetriever
      = getResourceRetriever(mOptions.mResourceRetriever);

  if (urdfString.empty()) {
    DART_WARN(
        "A blank string cannot be parsed into a World. Returning a nullptr");
    return nullptr;
  }

  std::shared_ptr<urdf_parsing::World> worldInterface
      = urdf_parsing::parseWorldURDF(urdfString, baseUri, resourceRetriever);

  if (!worldInterface) {
    DART_WARN("Failed loading URDF.");
    return nullptr;
  }

  simulation::WorldPtr world = simulation::World::create();

  for (std::size_t i = 0; i < worldInterface->models.size(); ++i) {
    const urdf_parsing::Entity& entity = worldInterface->models[i];
    std::string modelContent;
    readFileToString(resourceRetriever, entity.uri, modelContent);
    ParseContext context;
    context.mTransmissions = parseTransmissions(modelContent);
    dynamics::SkeletonPtr skeleton = modelInterfaceToSkeleton(
        entity.model.get(), entity.uri, resourceRetriever, mOptions, &context);

    if (!skeleton) {
      DART_WARN(
          "Robot {} was not correctly parsed!",
          worldInterface->models[i].model->getName());
      continue;
    }

    // Initialize position and RPY
    dynamics::Joint* rootJoint = skeleton->getRootBodyNode()->getParentJoint();
    Eigen::Isometry3d transform = toEigen(worldInterface->models[i].origin);

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
dynamics::SkeletonPtr UrdfParser::modelInterfaceToSkeleton(
    const urdf::ModelInterface* model,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& resourceRetriever,
    const Options& options,
    const ParseContext* context)
{
  dynamics::SkeletonPtr skeleton = dynamics::Skeleton::create(model->getName());

  const urdf::Link* root = model->getRoot().get();

  // If the link name is "world" then the link is regarded as the inertial
  // frame rather than a body in the robot model so we don't create a BodyNode
  // for it. This is not officially specified in the URDF spec, but "world" is
  // practically treated as a keyword.
  if (root->name == "world") {
    DART_WARN_IF(
        model->getRoot()->child_links.size() > 1,
        "The world link has more than one child links. This leads to creating "
        "a multi-tree robot. Multi-tree robot is supported by DART, but not "
        "the URDF standard. Please consider changing the robot model as a "
        "single tree robot.");

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

  const std::vector<TransmissionInfo> empty;
  const auto& transmissions
      = (context != nullptr) ? context->mTransmissions : empty;
  applyTransmissions(
      std::span<const TransmissionInfo>{transmissions}, model, skeleton);

  return skeleton;
}

//==============================================================================
bool UrdfParser::createSkeletonRecursive(
    const urdf::ModelInterface* model,
    dynamics::SkeletonPtr skel,
    const urdf::Link* lk,
    dynamics::BodyNode* parentNode,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& resourceRetriever,
    const Options& options)
{
  DART_ASSERT(lk);

  DART_WARN_IF(
      parentNode != nullptr && lk->name == "world",
      "[UrdfParser] Link name 'world' is reserved for the inertial frame. "
      "Consider changing the name to something else.");

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

bool UrdfParser::addMimicJointsRecursive(
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
      DART_ERROR(
          "Failed to configure a mimic joint [{}] because the joint isn't "
          "found in Skeleton [{}]\\n.",
          jointName,
          _skel->getName());
      return false;
    }

    const dynamics::Joint* mimicJoint = _skel->getJoint(mimicJointName);
    if (!mimicJoint) {
      DART_ERROR(
          "Failed to configure a mimic joint [{}] because the joint to mimic "
          "[{}] isn't found in Skeleton [{}]\\n.",
          jointName,
          mimicJoint,
          _skel->getName());
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

//==============================================================================
std::vector<UrdfParser::TransmissionInfo> UrdfParser::parseTransmissions(
    std::string_view urdfString)
{
  std::vector<TransmissionInfo> transmissions;

  if (urdfString.empty())
    return transmissions;

  tinyxml2::XMLDocument doc;
  const auto parseResult = doc.Parse(urdfString.data(), urdfString.size());
  if (parseResult != tinyxml2::XML_SUCCESS) {
    DART_WARN(
        "[UrdfParser] Failed to parse URDF for transmissions: tinyxml2 error "
        "{}."
        " Transmissions will be ignored.",
        parseResult);
    return transmissions;
  }

  const auto* robotElement = doc.FirstChildElement("robot");
  if (!robotElement)
    return transmissions;

  for (auto* tx = robotElement->FirstChildElement("transmission");
       tx != nullptr;
       tx = tx->NextSiblingElement("transmission")) {
    const auto* typeElement = tx->FirstChildElement("type");
    const std::string type
        = typeElement && typeElement->GetText() ? typeElement->GetText() : "";

    if (!type.empty() && type.find("SimpleTransmission") == std::string::npos
        && type.find("simple_transmission") == std::string::npos) {
      DART_WARN(
          "[UrdfParser] Transmission [{}] has unsupported type [{}]; skipping.",
          tx->Attribute("name") ? tx->Attribute("name") : "",
          type);
      continue;
    }

    const auto* jointElement = tx->FirstChildElement("joint");
    const auto* actuatorElement = tx->FirstChildElement("actuator");
    if (!jointElement || !actuatorElement) {
      DART_WARN(
          "[UrdfParser] Transmission [{}] is missing joint or actuator; "
          "skipping.",
          tx->Attribute("name") ? tx->Attribute("name") : "");
      continue;
    }

    const char* jointName = jointElement->Attribute("name");
    const char* actuatorName = actuatorElement->Attribute("name");
    if (!jointName || !actuatorName) {
      DART_WARN(
          "[UrdfParser] Transmission is missing joint or actuator name; "
          "skipping.");
      continue;
    }

    double mechanicalReduction = 1.0;
    const auto* reductionElement
        = actuatorElement->FirstChildElement("mechanicalReduction");
    if (!reductionElement)
      reductionElement = jointElement->FirstChildElement("mechanicalReduction");
    if (reductionElement) {
      const auto queryResult
          = reductionElement->QueryDoubleText(&mechanicalReduction);
      if (queryResult != tinyxml2::XML_SUCCESS) {
        DART_WARN(
            "[UrdfParser] Failed to read mechanicalReduction for transmission "
            "[{}:{}]; defaulting to 1.0.",
            actuatorName,
            jointName);
        mechanicalReduction = 1.0;
      }
    }

    if (mechanicalReduction == 0.0) {
      DART_WARN(
          "[UrdfParser] mechanicalReduction is zero for transmission [{}:{}]; "
          "skipping.",
          actuatorName,
          jointName);
      continue;
    }

    transmissions.push_back(
        {actuatorName, jointName, mechanicalReduction, true});
  }

  return transmissions;
}

//==============================================================================
void UrdfParser::applyTransmissions(
    std::span<const TransmissionInfo> transmissions,
    const urdf::ModelInterface* model,
    dynamics::SkeletonPtr skel)
{
  if (transmissions.empty())
    return;

  const std::string modelName = model ? model->getName() : "";
  std::unordered_map<std::string, std::vector<TransmissionInfo>> grouped;
  for (const auto& tx : transmissions)
    grouped[tx.mActuatorName].push_back(tx);

  for (const auto& actuatorPair : grouped) {
    const auto& entries = actuatorPair.second;
    if (entries.size() < 2)
      continue;

    std::vector<std::pair<const TransmissionInfo*, dynamics::Joint*>> valid;
    for (const auto& tx : entries) {
      auto* joint = skel->getJoint(tx.mJointName);
      if (!joint) {
        DART_WARN(
            "[UrdfParser] Transmission references missing joint [{}] on model "
            "[{}]; skipping.",
            tx.mJointName,
            modelName);
        continue;
      }

      if (joint->getNumDofs() != 1) {
        DART_WARN(
            "[UrdfParser] Transmission [{}] targets joint [{}] with {} DoFs; "
            "only single-DoF joints are supported for transmission coupling.",
            actuatorPair.first,
            tx.mJointName,
            joint->getNumDofs());
        continue;
      }

      if (joint->getActuatorType() == dynamics::Joint::MIMIC) {
        DART_WARN(
            "[UrdfParser] Transmission [{}] targets joint [{}] that already "
            "has a mimic actuator; skipping to avoid conflict.",
            actuatorPair.first,
            tx.mJointName);
        continue;
      }

      valid.emplace_back(&tx, joint);
    }

    if (valid.size() < 2)
      continue;

    const auto* referenceInfo = valid.front().first;
    dynamics::Joint* referenceJoint = valid.front().second;
    const double referenceReduction = referenceInfo->mMechanicalReduction;

    for (std::size_t i = 1; i < valid.size(); ++i) {
      const auto* followerInfo = valid[i].first;
      dynamics::Joint* followerJoint = valid[i].second;
      const double followerReduction = followerInfo->mMechanicalReduction;

      const double multiplier = referenceReduction / followerReduction;
      if (!std::isfinite(multiplier) || multiplier == 0.0) {
        DART_WARN(
            "[UrdfParser] Invalid gear ratio between joints [{}] and [{}] for "
            "actuator [{}]; skipping follower.",
            referenceJoint->getName(),
            followerJoint->getName(),
            actuatorPair.first);
        continue;
      }

      followerJoint->setActuatorType(dynamics::Joint::MIMIC);
      followerJoint->setMimicJoint(referenceJoint, multiplier, 0.0);
      if (followerInfo->mUseCoupler)
        followerJoint->setUseCouplerConstraint(true);
    }
  }
}

/**
 * @function readXml
 */
bool UrdfParser::readFileToString(
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
    const UrdfParser::Options& options)
{
  dynamics::Joint::Properties basicProperties("rootJoint");

  dynamics::GenericJoint<math::R1Space>::UniqueProperties singleDof;
  std::pair<dynamics::Joint*, dynamics::BodyNode*> pair;
  switch (options.mDefaultRootJointType) {
    case UrdfParser::RootJointType::Floating: {
      dynamics::GenericJoint<math::SE3Space>::Properties properties(
          basicProperties);
      pair = _skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          _parent, properties, _body);
      break;
    }
    case UrdfParser::RootJointType::Fixed: {
      pair = _skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
          _parent, basicProperties, _body);
      break;
    }
    default: {
      DART_ERROR(
          "Unsupported RootJointType '{}'. Using Floating instead.",
          static_cast<int>(options.mDefaultRootJointType)); // LCOV_EXCL_LINE
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
dynamics::BodyNode* UrdfParser::createDartJointAndNode(
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
  const Eigen::Vector3d jointAxis = toEigen(_jt->axis);

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
      singleDof.mPositionLowerLimits[0] = -math::inf;
      singleDof.mPositionUpperLimits[0] = math::inf;

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
      if (_jt->limits) {
        DART_WARN(
            "URDF joint '{}' provides 1D limits for a floating joint. Applying "
            "the same limits uniformly to all six degrees of freedom.",
            _jt->name);
        properties.mPositionLowerLimits.setConstant(_jt->limits->lower);
        properties.mPositionUpperLimits.setConstant(_jt->limits->upper);
        properties.mVelocityLowerLimits.setConstant(-_jt->limits->velocity);
        properties.mVelocityUpperLimits.setConstant(_jt->limits->velocity);
        properties.mForceLowerLimits.setConstant(-_jt->limits->effort);
        properties.mForceUpperLimits.setConstant(_jt->limits->effort);

        if (_jt->limits->lower > 0 || _jt->limits->upper < 0) {
          if (std::isfinite(_jt->limits->lower)
              && std::isfinite(_jt->limits->upper)) {
            properties.mInitialPositions.setConstant(
                (_jt->limits->lower + _jt->limits->upper) / 2.0);
          } else if (std::isfinite(_jt->limits->lower)) {
            properties.mInitialPositions.setConstant(_jt->limits->lower);
          } else if (std::isfinite(_jt->limits->upper)) {
            properties.mInitialPositions.setConstant(_jt->limits->upper);
          }

          properties.mRestPositions = properties.mInitialPositions;
        }
      }

      if (_jt->dynamics) {
        properties.mDampingCoefficients.setConstant(_jt->dynamics->damping);
        properties.mFrictions.setConstant(_jt->dynamics->friction);
      }

      pair = _skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
          _parent, properties, _body);
      break;
    }
    case urdf::Joint::PLANAR: {
      dynamics::PlanarJoint::Properties properties{
          dynamics::GenericJoint<math::R3Space>::Properties(basicProperties)};
      if (jointAxis.norm() > 1e-9) {
        const Eigen::Vector3d axis = jointAxis.normalized();
        if (axis.isApprox(Eigen::Vector3d::UnitZ())
            || axis.isApprox(-Eigen::Vector3d::UnitZ())) {
          properties.setXYPlane();
        } else if (
            axis.isApprox(Eigen::Vector3d::UnitX())
            || axis.isApprox(-Eigen::Vector3d::UnitX())) {
          properties.setYZPlane();
        } else if (
            axis.isApprox(Eigen::Vector3d::UnitY())
            || axis.isApprox(-Eigen::Vector3d::UnitY())) {
          properties.setZXPlane();
        } else {
          const Eigen::Vector3d transAxis1 = axis.unitOrthogonal();
          const Eigen::Vector3d transAxis2
              = axis.cross(transAxis1).normalized();
          properties.setArbitraryPlane(transAxis1, transAxis2);
        }
      }

      if (_jt->limits) {
        DART_WARN(
            "URDF joint '{}' provides 1D limits for a planar joint. Applying "
            "the same limits uniformly to all three degrees of freedom.",
            _jt->name);
        properties.mPositionLowerLimits.setConstant(_jt->limits->lower);
        properties.mPositionUpperLimits.setConstant(_jt->limits->upper);
        properties.mVelocityLowerLimits.setConstant(-_jt->limits->velocity);
        properties.mVelocityUpperLimits.setConstant(_jt->limits->velocity);
        properties.mForceLowerLimits.setConstant(-_jt->limits->effort);
        properties.mForceUpperLimits.setConstant(_jt->limits->effort);

        if (_jt->limits->lower > 0 || _jt->limits->upper < 0) {
          if (std::isfinite(_jt->limits->lower)
              && std::isfinite(_jt->limits->upper)) {
            properties.mInitialPositions.setConstant(
                (_jt->limits->lower + _jt->limits->upper) / 2.0);
          } else if (std::isfinite(_jt->limits->lower)) {
            properties.mInitialPositions.setConstant(_jt->limits->lower);
          } else if (std::isfinite(_jt->limits->upper)) {
            properties.mInitialPositions.setConstant(_jt->limits->upper);
          }

          properties.mRestPositions = properties.mInitialPositions;
        }
      }

      if (_jt->dynamics) {
        properties.mDampingCoefficients.setConstant(_jt->dynamics->damping);
        properties.mFrictions.setConstant(_jt->dynamics->friction);
      }

      pair = _skeleton->createJointAndBodyNodePair<dynamics::PlanarJoint>(
          _parent, properties, _body);
      break;
    }
    default: {
      DART_ERROR("Unsupported joint type ({})", _jt->type); // LCOV_EXCL_LINE
      return nullptr;
    }
  }

  return pair.second;
}

/**
 * @function createDartNode
 */
bool UrdfParser::createDartNodeProperties(
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

    Eigen::Matrix3d J;
    J << _lk->inertial->ixx, _lk->inertial->ixy, _lk->inertial->ixz,
        _lk->inertial->ixy, _lk->inertial->iyy, _lk->inertial->iyz,
        _lk->inertial->ixz, _lk->inertial->iyz, _lk->inertial->izz;
    Eigen::Matrix3d R(
        Eigen::Quaterniond(
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

    const Eigen::Vector4d color(
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
bool UrdfParser::createShapeNodes(
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
dynamics::ShapePtr UrdfParser::createShape(
    const VisualOrCollision* _vizOrCol,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _resourceRetriever)
{
  dynamics::ShapePtr shape;
  const auto* geometry = _vizOrCol->geometry.get();
  if (!geometry) {
    DART_WARN("URDF geometry is null. We are returning a nullptr.");
    return nullptr;
  }

  // Use geometry->type enum instead of dynamic_cast for FreeBSD compatibility.
  // dynamic_cast can fail across shared library boundaries on FreeBSD due to
  // RTTI issues.
  switch (geometry->type) {
    case urdf::Geometry::SPHERE: {
      const auto* sphere = static_cast<const urdf::Sphere*>(geometry);
      shape = std::make_shared<dynamics::SphereShape>(sphere->radius);
      break;
    }
    case urdf::Geometry::BOX: {
      const auto* box = static_cast<const urdf::Box*>(geometry);
      shape = std::make_shared<dynamics::BoxShape>(
          Eigen::Vector3d(box->dim.x, box->dim.y, box->dim.z));
      break;
    }
    case urdf::Geometry::CYLINDER: {
      const auto* cylinder = static_cast<const urdf::Cylinder*>(geometry);
      shape = std::make_shared<dynamics::CylinderShape>(
          cylinder->radius, cylinder->length);
      break;
    }
    case urdf::Geometry::MESH: {
      const auto* mesh = static_cast<const urdf::Mesh*>(geometry);
      // Resolve relative URIs.
      common::Uri absoluteUri;
      if (!absoluteUri.fromRelativeUri(
              _baseUri, std::string_view{mesh->filename})) {
        DART_WARN(
            "Failed resolving mesh URI '{}' relative to '{}'.",
            mesh->filename,
            _baseUri.toString());
        return nullptr;
      }

      // Load the mesh.
      const std::string resolvedUri = absoluteUri.toString();

      auto loader = std::make_unique<utils::MeshLoaderd>();
      auto triMeshUnique = loader->load(resolvedUri, _resourceRetriever);

      if (!triMeshUnique)
        return nullptr;

      std::shared_ptr<math::TriMesh<double>> triMesh(std::move(triMeshUnique));

      const Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
      shape = std::make_shared<dynamics::MeshShape>(
          scale,
          std::move(triMesh),
          common::Uri(resolvedUri),
          _resourceRetriever);
      break;
    }
    default:
      DART_WARN(
          "Unknown URDF Shape type (we only know of Sphere, Box, Cylinder, "
          "and Mesh). We are returning a nullptr.");
      return nullptr;
  }

  return shape;
}

common::ResourceRetrieverPtr UrdfParser::getResourceRetriever(
    const common::ResourceRetrieverPtr& _resourceRetriever)
{
  if (_resourceRetriever)
    return _resourceRetriever;
  else
    return mRetriever;
}

template dynamics::ShapePtr UrdfParser::createShape<urdf::Visual>(
    const urdf::Visual* _vizOrCol,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _resourceRetriever);
template dynamics::ShapePtr UrdfParser::createShape<urdf::Collision>(
    const urdf::Collision* _vizOrCol,
    const common::Uri& _baseUri,
    const common::ResourceRetrieverPtr& _resourceRetriever);

/**
 * @function pose2Affine3d
 */
Eigen::Isometry3d UrdfParser::toEigen(const urdf::Pose& _pose)
{
  Eigen::Quaterniond quat;
  _pose.rotation.getQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
  Eigen::Isometry3d transform(quat);
  transform.translation()
      = Eigen::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z);
  return transform;
}

Eigen::Vector3d UrdfParser::toEigen(const urdf::Vector3& _vector)
{
  return Eigen::Vector3d(_vector.x, _vector.y, _vector.z);
}

} // namespace utils
} // namespace dart
