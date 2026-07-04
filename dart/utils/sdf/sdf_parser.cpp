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

#include "dart/utils/sdf/sdf_parser.hpp"

#include "dart/common/local_resource_retriever.hpp"
#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"
#include "dart/common/resource_retriever.hpp"
#include "dart/common/uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/ball_joint.hpp"
#include "dart/dynamics/body_node.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/free_joint.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/mimic_dof_properties.hpp"
#include "dart/dynamics/prismatic_joint.hpp"
#include "dart/dynamics/revolute_joint.hpp"
#include "dart/dynamics/screw_joint.hpp"
#include "dart/dynamics/skeleton.hpp"
#include "dart/dynamics/soft_body_node.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/dynamics/universal_joint.hpp"
#include "dart/dynamics/weld_joint.hpp"
#include "dart/utils/composite_resource_retriever.hpp"
#include "dart/utils/dart_resource_retriever.hpp"
#include "dart/utils/mesh_loader.hpp"
#include "dart/utils/sdf/detail/geometry_parsers.hpp"
#include "dart/utils/sdf/detail/sdf_helpers.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <gz/math/Color.hh>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Model.hh>
#include <sdf/Pbr.hh>
#include <sdf/Surface.hh>
#include <sdf/Visual.hh>
#include <sdf/sdf.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>
#include <ranges>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include <cctype>

namespace dart {
namespace utils {

namespace SdfParser {

//==============================================================================
Options::Options(
    common::ResourceRetrieverPtr resourceRetriever,
    RootJointType defaultRootJointType)
  : mResourceRetriever(std::move(resourceRetriever)),
    mDefaultRootJointType(defaultRootJointType)
{
  // Do nothing
}

namespace {

using ElementPtr = sdf::ElementPtr;
namespace detail = dart::utils::SdfParser::detail;

using detail::getElement;
using detail::getValueDouble;
using detail::getValueIsometry3dWithExtrinsicRotation;
using detail::getValueUInt;
using detail::getValueVector3d;
using detail::getValueVector3i;
using detail::hasAuthoredElement;
using detail::hasElement;
using detail::readGeometryShape;

Eigen::Vector3d toEigenVector3(const gz::math::Vector3d& vector)
{
  return Eigen::Vector3d(vector.X(), vector.Y(), vector.Z());
}

Eigen::Vector4d toEigenColor(const gz::math::Color& color)
{
  return Eigen::Vector4d(color[0], color[1], color[2], color[3]);
}

Eigen::Isometry3d toEigenIsometry3(const gz::math::Pose3d& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = toEigenVector3(pose.Pos());

  const auto& rotation = pose.Rot();
  Eigen::Quaterniond quaternion(
      rotation.W(), rotation.X(), rotation.Y(), rotation.Z());
  quaternion.normalize();
  transform.linear() = quaternion.toRotationMatrix();

  return transform;
}

common::ResourceRetrieverPtr getRetriever(
    const common::ResourceRetrieverPtr& retriever);

struct ResolvedOptions
{
  common::ResourceRetrieverPtr retriever;
  RootJointType defaultRootJointType;
};

struct TemporaryResourceOwner;

ResolvedOptions resolveOptions(const Options& options)
{
  ResolvedOptions resolved;
  resolved.retriever = getRetriever(options.mResourceRetriever);
  resolved.defaultRootJointType = options.mDefaultRootJointType;
  return resolved;
}

bool loadSdfRoot(
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retriever,
    sdf::Root& root,
    TemporaryResourceOwner& resources);

using BodyPropPtr = std::shared_ptr<dynamics::BodyNode::Properties>;

struct SDFBodyNode
{
  BodyPropPtr properties;
  Eigen::Isometry3d initTransform;
  std::string type;
};

using JointPropPtr = std::shared_ptr<dynamics::Joint::Properties>;

struct SDFJoint
{
  JointPropPtr properties;
  std::string parentName;
  std::string childName;
  std::string type;
  sdf::JointType sdfType = sdf::JointType::INVALID;
  struct MimicInfo
  {
    std::string referenceJointName;
    std::size_t referenceDof = 0;
    double multiplier = 1.0;
    double offset = 0.0;
    dynamics::MimicConstraintType constraintType
        = dynamics::MimicConstraintType::Motor;
  };
  std::vector<MimicInfo> mimicInfos;
};

std::vector<SDFJoint::MimicInfo> readMimicElements(const sdf::JointAxis& axis);

// Maps the name of a BodyNode to its properties
using BodyMap = common::aligned_map<std::string, SDFBodyNode>;

// Maps a child BodyNode to the properties of its parent Joint
using JointMap = std::map<std::string, SDFJoint>;

dynamics::SkeletonPtr readSkeleton(
    const sdf::Model& model,
    const common::Uri& baseUri,
    const ResolvedOptions& options);

void applyMimicConstraints(
    const dynamics::SkeletonPtr& skeleton, const JointMap& sdfJoints);

bool createPair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SDFJoint& newJoint,
    const SDFBodyNode& newBody);

enum NextResult
{
  VALID,
  CONTINUE,
  BREAK,
  CREATE_ROOT_JOINT
};

NextResult getNextJointAndNodePair(
    BodyMap::iterator& body,
    JointMap::const_iterator& parentJoint,
    dynamics::BodyNode*& parentBody,
    const dynamics::SkeletonPtr skeleton,
    BodyMap& sdfBodyNodes,
    const JointMap& sdfJoints);

dynamics::SkeletonPtr makeSkeleton(
    const sdf::Model& model, Eigen::Isometry3d& skeletonFrame);

template <class NodeType>
std::pair<dynamics::Joint*, dynamics::BodyNode*> createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SDFJoint& joint,
    const SDFBodyNode& node);

BodyMap readAllBodyNodes(
    const sdf::Model& model,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever,
    const Eigen::Isometry3d& skeletonFrame);

SDFBodyNode readBodyNode(
    const sdf::Link& link,
    const Eigen::Isometry3d& skeletonFrame,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

dynamics::SoftBodyNode::UniqueProperties readSoftBodyProperties(
    const ElementPtr& softBodyNodeElement);

dynamics::ShapePtr readShape(
    const sdf::Geometry* geometry,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& _retriever);

dynamics::ShapeNode* readShapeNode(
    dynamics::BodyNode* bodyNode,
    const sdf::Geometry* geometry,
    const gz::math::Pose3d& rawPose,
    std::string_view shapeNodeName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readMaterial(
    const sdf::Material& material, dynamics::ShapeNode* shapeNode);

void readVisualizationShapeNode(
    dynamics::BodyNode* bodyNode,
    const sdf::Visual& visual,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readCollisionShapeNode(
    dynamics::BodyNode* bodyNode,
    const sdf::Collision& collision,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readAspects(
    const dynamics::SkeletonPtr& skeleton,
    const sdf::Model& model,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

JointMap readAllJoints(
    const sdf::Model& model,
    const Eigen::Isometry3d& skeletonFrame,
    const BodyMap& sdfBodyNodes);

SDFJoint readJoint(
    const sdf::Joint& joint,
    const BodyMap& bodies,
    const Eigen::Isometry3d& skeletonFrame);

dart::dynamics::WeldJoint::Properties readWeldJoint(
    const sdf::Joint& joint,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

dynamics::RevoluteJoint::Properties readRevoluteJoint(
    const sdf::Joint& revoluteJoint,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

dynamics::PrismaticJoint::Properties readPrismaticJoint(
    const sdf::Joint& joint,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

dynamics::ScrewJoint::Properties readScrewJoint(
    const sdf::Joint& joint,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

dynamics::UniversalJoint::Properties readUniversalJoint(
    const sdf::Joint& joint,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

dynamics::BallJoint::Properties readBallJoint(
    const sdf::Joint& joint,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

std::string describeSdformatError(const sdf::Error& error)
{
  std::ostringstream stream;
  stream << error.Message();

  if (const auto file = error.FilePath()) {
    stream << " (" << *file;
    if (const auto line = error.LineNumber()) {
      stream << ':' << *line;
    }
    stream << ')';
  }

  return stream.str();
}

bool isMissingUriError(std::string_view message, std::string& uriValue)
{
  static constexpr std::string_view needle = "Unable to find uri[";
  const auto start = message.find(needle);
  if (start == std::string_view::npos) {
    return false;
  }

  const auto uriStart = start + needle.size();
  const auto uriEnd = message.find(']', uriStart);
  if (uriEnd == std::string_view::npos) {
    return false;
  }

  uriValue = std::string(message.substr(uriStart, uriEnd - uriStart));
  return true;
}

void logSdformatErrors(
    const sdf::Errors& errors, const common::Uri& uri, std::string_view context)
{
  if (errors.empty()) {
    return;
  }

  std::size_t warningCount = 0;
  std::size_t errorCount = 0;
  std::map<std::string, std::size_t> missingUriCounts;
  std::vector<std::string> representativeMessages;

  for (const auto& sdformatError : errors) {
    const bool isWarning = (sdformatError.Code() == sdf::ErrorCode::WARNING);
    if (isWarning) {
      ++warningCount;
    } else {
      ++errorCount;
    }

    const std::string description = describeSdformatError(sdformatError);
    std::string uriValue;
    if (isMissingUriError(description, uriValue)) {
      ++missingUriCounts[uriValue];
      continue;
    }

    if (representativeMessages.size() < 3) {
      representativeMessages.push_back(description);
    }
  }

  std::ostringstream stream;
  stream << "[SdfParser] " << context << " [" << uri.toString()
         << "]: " << errors.size() << " issue(s) detected (" << errorCount
         << " errors, " << warningCount << " warnings).";

  if (!missingUriCounts.empty()) {
    stream << " Missing URIs:";
    for (const auto& [missingUri, count] : missingUriCounts) {
      stream << " [" << missingUri << " x" << count << ']';
    }
  }

  DART_WARN("{}", stream.str());

  for (const auto& message : representativeMessages) {
    DART_WARN("  ↳ {}", message);
  }
}

std::filesystem::path makeTemporaryPath(const common::Uri& uri)
{
  static std::atomic<uint64_t> counter{0};

  const auto tempDir = std::filesystem::temp_directory_path();
  const auto extension = std::filesystem::path(uri.getPath()).extension();

  for (std::size_t attempt = 0; attempt < 64; ++attempt) {
    std::stringstream name;
    name << "dart_sdf_"
         << std::chrono::high_resolution_clock::now().time_since_epoch().count()
         << '_' << counter.fetch_add(1, std::memory_order_relaxed);
    if (attempt > 0) {
      name << '_' << attempt;
    }
    if (!extension.empty()) {
      name << extension.string();
    }

    const auto candidate = tempDir / name.str();
    if (!std::filesystem::exists(candidate)) {
      return candidate;
    }
  }

  throw std::runtime_error(
      "Failed to allocate temporary path for [" + uri.toString() + "]");
}

std::filesystem::path writeTemporaryResource(
    std::string_view data, const common::Uri& uri)
{
  const auto tempPath = makeTemporaryPath(uri);

  std::ofstream output(tempPath.string(), std::ios::binary);
  output.write(data.data(), static_cast<std::streamsize>(data.size()));

  if (!output) {
    throw std::runtime_error(
        "Failed to write temporary file for resource [" + uri.toString()
        + "] at path [" + tempPath.string() + "]");
  }

  return tempPath;
}

std::string resolveWithRetriever(
    std::string_view requested,
    const common::ResourceRetrieverPtr& retriever,
    const std::shared_ptr<std::vector<std::filesystem::path>>& tempFiles)
{
  if (!retriever) {
    return std::string();
  }

  const std::string requestedString(requested);
  common::Uri requestedUri;
  if (!requestedUri.fromStringOrPath(requestedString)) {
    return std::string();
  }

  if (requestedUri.mScheme.get_value_or("file") == "file"
      && requestedUri.mPath) {
    std::error_code ec;
    const auto candidate = requestedUri.getFilesystemPath();
    if (std::filesystem::exists(candidate, ec) && !ec) {
      return candidate;
    }
  }

  if (!retriever->exists(requestedUri)) {
    return std::string();
  }

  try {
    const auto data = retriever->readAll(requestedUri);
    const auto tmp = writeTemporaryResource(data, requestedUri);
    tempFiles->push_back(tmp);
    return tmp.string();
  } catch (const std::exception& e) {
    DART_WARN(
        "[SdfParser] Failed to materialize [{}] for libsdformat: {}",
        requested,
        e.what());
    return std::string();
  }
}

bool hasUriScheme(std::string_view candidate)
{
  return candidate.find("://") != std::string_view::npos;
}

bool isWindowsAbsolutePath(std::string_view path)
{
#ifdef _WIN32
  return path.size() > 1 && path[1] == ':'
         && std::isalpha(static_cast<unsigned char>(path[0]));
#else
  (void)path;
  return false;
#endif
}

bool requiresBaseUriResolution(std::string_view requested)
{
  if (requested.empty()) {
    return false;
  }

  if (hasUriScheme(requested)) {
    return false;
  }

  if (requested.starts_with('/')) {
    return false;
  }

  if (isWindowsAbsolutePath(requested)) {
    return false;
  }

  return true;
}

std::string resolveRequestedUri(
    std::string_view requested, const common::Uri& baseUri)
{
  if (!requiresBaseUriResolution(requested)) {
    return std::string(requested);
  }

  if (!baseUri.mPath) {
    return std::string(requested);
  }

  if (baseUri.mScheme.get_value_or("file") == "file") {
    const auto basePath = std::filesystem::path(baseUri.getFilesystemPath());
    if (basePath.has_parent_path()) {
      return (basePath.parent_path() / std::filesystem::path(requested))
          .string();
    }
  }

  const auto merged = common::Uri::getRelativeUri(baseUri, requested);
  if (!merged.empty()) {
    return merged;
  }

  return std::string(requested);
}

void cleanupTemporaryResources(
    const std::shared_ptr<std::vector<std::filesystem::path>>& tempFiles)
{
  if (!tempFiles) {
    return;
  }

  for (const auto& path : *tempFiles) {
    std::error_code ec;
    std::filesystem::remove(path, ec);
  }
}

struct TemporaryResourceOwner
{
  TemporaryResourceOwner() = default;

  ~TemporaryResourceOwner()
  {
    cleanupTemporaryResources(files);
  }

  std::shared_ptr<std::vector<std::filesystem::path>> files;
};

bool loadSdfRoot(
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retriever,
    sdf::Root& root,
    TemporaryResourceOwner& resources)
{
  if (!retriever) {
    DART_WARN(
        "[SdfParser] Unable to load [{}] because no ResourceRetriever was "
        "provided.",
        uri.toString());
    return false;
  }

  resources.files = std::make_shared<std::vector<std::filesystem::path>>();
  sdf::ParserConfig config = sdf::ParserConfig::GlobalConfig();
  config.SetFindCallback(
      [retriever, tempFiles = resources.files, baseUri = uri](
          const std::string& requested) -> std::string {
        const auto resolvedRequest = resolveRequestedUri(requested, baseUri);
        return resolveWithRetriever(resolvedRequest, retriever, tempFiles);
      });

  sdf::Errors errors;
  std::string localPath;
  if (uri.mScheme.get_value_or("file") == "file" && uri.mPath) {
    const auto candidate = uri.getFilesystemPath();
    std::error_code ec;
    if (std::filesystem::exists(candidate, ec) && !ec) {
      localPath = candidate;
    }
  }
  if (!localPath.empty()) {
    errors = root.Load(localPath, config);
  } else {
    std::string content;
    try {
      content = retriever->readAll(uri);
    } catch (const std::exception& e) {
      DART_WARN(
          "[SdfParser] Failed to read [{}]: {}.", uri.toString(), e.what());
      cleanupTemporaryResources(resources.files);
      resources.files.reset();
      return false;
    }
    errors = root.LoadSdfString(content, config);
  }

  logSdformatErrors(errors, uri, "libsdformat reported an issue while loading");

  if (root.Element() == nullptr) {
    DART_WARN(
        "[SdfParser] [{}] produced an empty SDF document.", uri.toString());
    cleanupTemporaryResources(resources.files);
    resources.files.reset();
    return false;
  }

  return true;
}

dynamics::SkeletonPtr readSkeleton(
    const sdf::Model& model,
    const common::Uri& baseUri,
    const ResolvedOptions& options)
{
  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();
  dynamics::SkeletonPtr newSkeleton = makeSkeleton(model, skeletonFrame);

  //--------------------------------------------------------------------------
  // Bodies
  BodyMap sdfBodyNodes
      = readAllBodyNodes(model, baseUri, options.retriever, skeletonFrame);

  //--------------------------------------------------------------------------
  // Joints
  JointMap sdfJoints = readAllJoints(model, skeletonFrame, sdfBodyNodes);

  // Iterate through the collected properties and construct the Skeleton from
  // the root nodes downward
  BodyMap::iterator body = sdfBodyNodes.begin();
  JointMap::const_iterator parentJoint;
  dynamics::BodyNode* parentBody{nullptr};
  while (body != sdfBodyNodes.end()) {
    NextResult result = getNextJointAndNodePair(
        body, parentJoint, parentBody, newSkeleton, sdfBodyNodes, sdfJoints);

    if (BREAK == result) {
      break;
    } else if (CONTINUE == result) {
      continue;
    } else if (CREATE_ROOT_JOINT == result) {
      SDFJoint rootJoint;
      if (options.defaultRootJointType == RootJointType::Floating) {
        // If a root FreeJoint is needed for the parent of the current joint,
        // then create it
        rootJoint.properties = dynamics::FreeJoint::Properties::createShared(
            dynamics::Joint::Properties("root", body->second.initTransform));
        rootJoint.type = "free";
      } else if (options.defaultRootJointType == RootJointType::Fixed) {
        // If a root WeldJoint is needed for the parent of the current joint,
        // then create it
        rootJoint.properties = dynamics::WeldJoint::Properties::createShared(
            dynamics::Joint::Properties("root", body->second.initTransform));
        rootJoint.type = "fixed";
      } else {
        DART_WARN(
            "Unsupported root joint type [{}]. Using FLOATING by default.",
            std::to_underlying(options.defaultRootJointType));
        rootJoint.properties = dynamics::FreeJoint::Properties::createShared(
            dynamics::Joint::Properties("root", body->second.initTransform));
        rootJoint.type = "free";
      }

      if (!createPair(newSkeleton, nullptr, rootJoint, body->second)) {
        break;
      }

      sdfBodyNodes.erase(body);
      body = sdfBodyNodes.begin();

      continue;
    }

    if (!createPair(
            newSkeleton, parentBody, parentJoint->second, body->second)) {
      break;
    }

    sdfBodyNodes.erase(body);
    body = sdfBodyNodes.begin();
  }

  // Read aspects here since aspects cannot be added if the BodyNodes haven't
  // created yet.
  readAspects(newSkeleton, model, baseUri, options.retriever);

  // Set positions to their initial values
  newSkeleton->resetPositions();

  applyMimicConstraints(newSkeleton, sdfJoints);

  return newSkeleton;
}

//==============================================================================
bool createPair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SDFJoint& newJoint,
    const SDFBodyNode& newBody)
{
  std::pair<dynamics::Joint*, dynamics::BodyNode*> pair;

  if (newBody.type.empty()) {
    pair = createJointAndNodePair<dynamics::BodyNode>(
        skeleton, parent, newJoint, newBody);
  } else if (std::string("soft") == newBody.type) {
    pair = createJointAndNodePair<dynamics::SoftBodyNode>(
        skeleton, parent, newJoint, newBody);
  } else {
    DART_ERROR("Unsupported Link type: {}", newBody.type);
    return false;
  }

  if (!pair.first || !pair.second) {
    return false;
  }

  return true;
}

//==============================================================================
NextResult getNextJointAndNodePair(
    BodyMap::iterator& body,
    JointMap::const_iterator& parentJoint,
    dynamics::BodyNode*& parentBody,
    const dynamics::SkeletonPtr skeleton,
    BodyMap& sdfBodyNodes,
    const JointMap& sdfJoints)
{
  parentJoint = sdfJoints.find(body->first);
  if (parentJoint == sdfJoints.end()) {
    return CREATE_ROOT_JOINT;
  }

  const std::string& parentBodyName = parentJoint->second.parentName;
  const std::string& parentJointName = parentJoint->second.properties->mName;

  // Check if the parent Body is created yet
  parentBody = skeleton->getBodyNode(parentBodyName);
  if (nullptr == parentBody && parentBodyName != "world"
      && !parentBodyName.empty()) {
    // Find the properties of the parent Joint of the current Joint, because it
    // does not seem to be created yet.
    BodyMap::iterator check_parent_body = sdfBodyNodes.find(parentBodyName);

    if (check_parent_body == sdfBodyNodes.end()) {
      // The Body does not exist in the file
      DART_ERROR(
          "Could not find Link named [{}] requested as parent of Joint [{}]. "
          "We will now quit parsing.",
          parentBodyName,
          parentJointName);
      return BREAK;
    } else {
      body = check_parent_body;
      return CONTINUE; // Create the parent before creating the current Joint
    }
  }

  return VALID;
}

//==============================================================================
void applyMimicConstraints(
    const dynamics::SkeletonPtr& skeleton, const JointMap& sdfJoints)
{
  for (const auto& jointInfo : sdfJoints | std::views::values) {
    if (jointInfo.mimicInfos.empty()) {
      continue;
    }

    auto* joint = skeleton->getJoint(jointInfo.properties->mName);
    if (!joint) {
      continue;
    }

    const auto existingProps = joint->getMimicDofProperties();
    std::vector<dynamics::MimicDofProperties> props(joint->getNumDofs());
    std::ranges::copy(
        existingProps | std::views::take(props.size()), props.begin());

    bool applied = false;
    bool useCoupler = false;
    for (const auto& mimic : jointInfo.mimicInfos) {
      auto* ref = skeleton->getJoint(mimic.referenceJointName);
      if (!ref) {
        DART_WARN(
            "[SdfParser] Ignoring mimic joint [{}] referencing missing joint "
            "[{}]",
            jointInfo.properties->mName,
            mimic.referenceJointName);
        continue;
      }

      const std::size_t followerIndex
          = std::min(mimic.referenceDof, joint->getNumDofs() - 1);
      const std::size_t referenceIndex
          = std::min(mimic.referenceDof, ref->getNumDofs() - 1);

      auto& prop = props[followerIndex];
      prop.mReferenceJoint = ref;
      prop.mReferenceDofIndex = referenceIndex;
      prop.mMultiplier = mimic.multiplier;
      prop.mOffset = mimic.offset;
      prop.mConstraintType = mimic.constraintType;
      applied = true;
      useCoupler
          = useCoupler
            || mimic.constraintType == dynamics::MimicConstraintType::Coupler;
    }

    if (!applied) {
      continue;
    }

    joint->setMimicJointDofs(props);
    joint->setActuatorType(dynamics::Joint::MIMIC);
    joint->setUseCouplerConstraint(useCoupler);
  }
}

dynamics::SkeletonPtr makeSkeleton(
    const sdf::Model& model, Eigen::Isometry3d& skeletonFrame)
{
  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create();

  //--------------------------------------------------------------------------
  // Name attribute
  newSkeleton->setName(model.Name());

  //--------------------------------------------------------------------------
  // immobile attribute
  newSkeleton->setMobile(!model.Static());

  //--------------------------------------------------------------------------
  // transformation
  skeletonFrame = toEigenIsometry3(model.RawPose());

  return newSkeleton;
}

//==============================================================================
template <class NodeType>
std::pair<dynamics::Joint*, dynamics::BodyNode*> createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SDFJoint& joint,
    const SDFBodyNode& node)
{
  const std::string& type = joint.type;

  if (joint.sdfType == sdf::JointType::PRISMATIC
      || std::string("prismatic") == type) {
    return skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
        parent,
        static_cast<const dynamics::PrismaticJoint::Properties&>(
            *joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  } else if (
      joint.sdfType == sdf::JointType::REVOLUTE
      || joint.sdfType == sdf::JointType::CONTINUOUS
      || std::string("revolute") == type) {
    return skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
        parent,
        static_cast<const dynamics::RevoluteJoint::Properties&>(
            *joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  } else if (
      joint.sdfType == sdf::JointType::SCREW || std::string("screw") == type) {
    return skeleton->createJointAndBodyNodePair<dynamics::ScrewJoint>(
        parent,
        static_cast<const dynamics::ScrewJoint::Properties&>(*joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  } else if (
      joint.sdfType == sdf::JointType::REVOLUTE2
      || joint.sdfType == sdf::JointType::UNIVERSAL
      || std::string("revolute2") == type || std::string("universal") == type) {
    return skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(
        parent,
        static_cast<const dynamics::UniversalJoint::Properties&>(
            *joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  } else if (
      joint.sdfType == sdf::JointType::BALL || std::string("ball") == type) {
    return skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(
        parent,
        static_cast<const dynamics::BallJoint::Properties&>(*joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  } else if (
      joint.sdfType == sdf::JointType::FIXED || std::string("fixed") == type) {
    return skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
        parent,
        static_cast<const dynamics::WeldJoint::Properties&>(*joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  } else if (std::string("free") == type) {
    return skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
        parent,
        static_cast<const dynamics::FreeJoint::Properties&>(*joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  }

  DART_ERROR(
      "{}{}. Please report this as a bug! We will now quit parsing.",
      "Unsupported Joint type encountered: ",
      type);
  return std::pair<dynamics::Joint*, dynamics::BodyNode*>(nullptr, nullptr);
}

//==============================================================================
BodyMap readAllBodyNodes(
    const sdf::Model& model,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever,
    const Eigen::Isometry3d& skeletonFrame)
{
  BodyMap sdfBodyNodes;
  for (uint64_t linkIndex = 0; linkIndex < model.LinkCount(); ++linkIndex) {
    const sdf::Link* link = model.LinkByIndex(linkIndex);
    if (!link) {
      continue;
    }

    SDFBodyNode body = readBodyNode(*link, skeletonFrame, baseUri, retriever);

    BodyMap::iterator it = sdfBodyNodes.find(body.properties->mName);
    if (it != sdfBodyNodes.end()) {
      DART_WARN(
          "Duplicate name in file: {}\\nEvery Link must have a unique name!",
          body.properties->mName);
      continue;
    }

    sdfBodyNodes[body.properties->mName] = body;
  }

  return sdfBodyNodes;
}

//===============================================================================
SDFBodyNode readBodyNode(
    const sdf::Link& link,
    const Eigen::Isometry3d& skeletonFrame,
    const common::Uri& /*baseUri*/,
    const common::ResourceRetrieverPtr& /*retriever*/)
{
  const ElementPtr bodyNodeElement = link.Element();
  DART_ASSERT(bodyNodeElement != nullptr);

  dynamics::BodyNode::Properties properties;
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  std::string name = link.Name();
  properties.mName = name;
  const std::string bodyName = name;

  //--------------------------------------------------------------------------
  // gravity
  properties.mGravityMode = link.EnableGravity();

  //--------------------------------------------------------------------------
  // transformation
  initTransform = skeletonFrame * toEigenIsometry3(link.RawPose());

  //--------------------------------------------------------------------------
  // inertia
  constexpr double kMinReasonableMass = 1e-9; // 1 microgram
  bool massSpecified = false;
  if (hasAuthoredElement(bodyNodeElement, "inertial")) {
    const ElementPtr& inertiaElement = getElement(bodyNodeElement, "inertial");
    const auto& inertial = link.Inertial();
    const auto& massMatrix = inertial.MassMatrix();

    // mass
    if (hasAuthoredElement(inertiaElement, "mass")) {
      double mass = massMatrix.Mass();
      if (mass <= 0.0) {
        DART_WARN(
            "[SdfParser] Link [{}] has non-positive mass [{}]. Clamping to {} "
            "to continue parsing.",
            bodyName,
            mass,
            kMinReasonableMass);
        mass = kMinReasonableMass;
      } else if (mass < kMinReasonableMass) {
        DART_WARN(
            "[SdfParser] Link [{}] has a very small mass [{} kg]; clamping to "
            "{} to avoid numerical issues.",
            bodyName,
            mass,
            kMinReasonableMass);
        mass = kMinReasonableMass;
      }
      properties.mInertia.setMass(mass);
      massSpecified = true;
    }

    // offset
    if (hasAuthoredElement(inertiaElement, "pose")) {
      properties.mInertia.setLocalCOM(toEigenVector3(inertial.Pose().Pos()));
    }

    // inertia
    if (hasAuthoredElement(inertiaElement, "inertia")) {
      properties.mInertia.setMoment(
          massMatrix.Ixx(),
          massMatrix.Iyy(),
          massMatrix.Izz(),
          massMatrix.Ixy(),
          massMatrix.Ixz(),
          massMatrix.Iyz());
    } else if (massSpecified) {
      // Keep the inertia physically meaningful by matching the moment scale
      // to the specified mass; geometry is unknown, so use an isotropic guess.
      const double mass = properties.mInertia.getMass();
      properties.mInertia.setMoment(Eigen::Matrix3d::Identity() * mass);
      DART_WARN(
          "[SdfParser] Link [{}] defines <mass> but no <inertia>; using an "
          "isotropic inertia tensor (mass * I). Provide <inertia> for "
          "physically correct behavior.",
          bodyName);
    }

    if (!massSpecified) {
      DART_WARN(
          "[SdfParser] Link [{}] is missing <mass>; using default mass of 1 "
          "kg. "
          "Specify <inertial><mass> to avoid unstable simulation.",
          bodyName);
    }
  } else {
    DART_WARN(
        "[SdfParser] Link [{}] is missing <inertial>; using default "
        "mass/inertia "
        "(1 kg, unit inertia). Specify <inertial> to avoid unstable "
        "simulation.",
        bodyName);
  }

  SDFBodyNode sdfBodyNode;
  sdfBodyNode.initTransform = initTransform;
  if (hasElement(bodyNodeElement, "soft_shape")) {
    auto softProperties = readSoftBodyProperties(bodyNodeElement);

    sdfBodyNode.properties = dynamics::SoftBodyNode::Properties::createShared(
        properties, softProperties);
    sdfBodyNode.type = "soft";
  } else {
    sdfBodyNode.properties
        = dynamics::BodyNode::Properties::createShared(properties);
    sdfBodyNode.type = "";
  }

  return sdfBodyNode;
}

//==============================================================================
dynamics::SoftBodyNode::UniqueProperties readSoftBodyProperties(
    const ElementPtr& softBodyNodeElement)
{
  //---------------------------------- Note ------------------------------------
  // SoftBodyNode is created if _softBodyNodeElement has <soft_shape>.
  // Otherwise, BodyNode is created.

  //----------------------------------------------------------------------------
  DART_ASSERT(softBodyNodeElement != nullptr);

  dynamics::SoftBodyNode::UniqueProperties softProperties;

  //----------------------------------------------------------------------------
  // Soft properties
  if (hasElement(softBodyNodeElement, "soft_shape")) {
    const ElementPtr& softShapeEle
        = getElement(softBodyNodeElement, "soft_shape");

    // mass
    double totalMass = getValueDouble(softShapeEle, "total_mass");

    // pose
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (hasElement(softShapeEle, "pose")) {
      T = getValueIsometry3dWithExtrinsicRotation(softShapeEle, "pose");
    }

    // geometry
    const ElementPtr& geometryEle = getElement(softShapeEle, "geometry");
    if (hasElement(geometryEle, "sphere")) {
      const ElementPtr& sphereEle = getElement(geometryEle, "sphere");
      const auto radius = getValueDouble(sphereEle, "radius");
      const auto nSlices = getValueUInt(sphereEle, "num_slices");
      const auto nStacks = getValueUInt(sphereEle, "num_stacks");
      softProperties = dynamics::SoftBodyNodeHelper::makeSphereProperties(
          radius, nSlices, nStacks, totalMass);
    } else if (hasElement(geometryEle, "box")) {
      const ElementPtr& boxEle = getElement(geometryEle, "box");
      Eigen::Vector3d size = getValueVector3d(boxEle, "size");
      Eigen::Vector3i frags = getValueVector3i(boxEle, "frags");
      softProperties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
          size, T, frags, totalMass);
    } else if (hasElement(geometryEle, "ellipsoid")) {
      const ElementPtr& ellipsoidEle = getElement(geometryEle, "ellipsoid");
      Eigen::Vector3d size = getValueVector3d(ellipsoidEle, "size");
      const auto nSlices = getValueUInt(ellipsoidEle, "num_slices");
      const auto nStacks = getValueUInt(ellipsoidEle, "num_stacks");
      softProperties = dynamics::SoftBodyNodeHelper::makeEllipsoidProperties(
          size, nSlices, nStacks, totalMass);
    } else if (hasElement(geometryEle, "cylinder")) {
      const ElementPtr& ellipsoidEle = getElement(geometryEle, "cylinder");
      double radius = getValueDouble(ellipsoidEle, "radius");
      double height = getValueDouble(ellipsoidEle, "height");
      double nSlices = getValueDouble(ellipsoidEle, "num_slices");
      double nStacks = getValueDouble(ellipsoidEle, "num_stacks");
      double nRings = getValueDouble(ellipsoidEle, "num_rings");
      softProperties = dynamics::SoftBodyNodeHelper::makeCylinderProperties(
          radius, height, nSlices, nStacks, nRings, totalMass);
    } else {
      DART_ERROR("Unknown soft shape.");
    }

    // kv
    if (hasElement(softShapeEle, "kv")) {
      softProperties.mKv = getValueDouble(softShapeEle, "kv");
    }

    // ke
    if (hasElement(softShapeEle, "ke")) {
      softProperties.mKe = getValueDouble(softShapeEle, "ke");
    }

    // damp
    if (hasElement(softShapeEle, "damp")) {
      softProperties.mDampCoeff = getValueDouble(softShapeEle, "damp");
    }
  }

  return softProperties;
}

//==============================================================================
dynamics::ShapePtr readShape(
    const sdf::Geometry* geometry,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& _retriever)
{
  if (!geometry) {
    DART_WARN("Shape node is missing required <geometry> element.");
    return nullptr;
  }

  return readGeometryShape(*geometry, baseUri, _retriever);
}

//==============================================================================
dynamics::ShapeNode* readShapeNode(
    dynamics::BodyNode* bodyNode,
    const sdf::Geometry* geometry,
    const gz::math::Pose3d& rawPose,
    std::string_view shapeNodeName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  DART_ASSERT(bodyNode);

  auto shape = readShape(geometry, baseUri, retriever);
  auto shapeNode = bodyNode->createShapeNode(shape, shapeNodeName);

  // Transformation
  shapeNode->setRelativeTransform(toEigenIsometry3(rawPose));

  return shapeNode;
}

//==============================================================================
void readMaterial(const sdf::Material& material, dynamics::ShapeNode* shapeNode)
{
  auto visualAspect = shapeNode->getVisualAspect();
  if (hasAuthoredElement(material.Element(), "diffuse")) {
    visualAspect->setColor(toEigenColor(material.Diffuse()));
  }

  const sdf::Pbr* pbr = material.PbrMaterial();
  if (!pbr) {
    return;
  }

  const sdf::PbrWorkflow* workflow = pbr->Workflow(sdf::PbrWorkflowType::METAL);
  if (!workflow) {
    return;
  }

  visualAspect->setMetallic(workflow->Metalness());
  visualAspect->setRoughness(workflow->Roughness());
}

//==============================================================================
void readCollisionSurface(
    const sdf::Collision& collision, dynamics::ShapeNode* shapeNode)
{
  auto dynamicsAspect = shapeNode->getDynamicsAspect();
  if (!dynamicsAspect) {
    return;
  }

  const sdf::Surface* surface = collision.Surface();
  if (!surface) {
    return;
  }

  const sdf::Friction* friction = surface->Friction();
  if (!friction) {
    return;
  }

  const sdf::ODE* ode = friction->ODE();
  if (!ode) {
    return;
  }

  const ElementPtr odeElement = ode->Element();
  if (hasAuthoredElement(odeElement, "mu")) {
    dynamicsAspect->setPrimaryFrictionCoeff(ode->Mu());
  }
  if (hasAuthoredElement(odeElement, "mu2")) {
    dynamicsAspect->setSecondaryFrictionCoeff(ode->Mu2());
  }
  if (hasAuthoredElement(odeElement, "slip1")) {
    dynamicsAspect->setPrimarySlipCompliance(ode->Slip1());
  }
  if (hasAuthoredElement(odeElement, "slip2")) {
    dynamicsAspect->setSecondarySlipCompliance(ode->Slip2());
  }
  if (hasAuthoredElement(odeElement, "fdir1")) {
    dynamicsAspect->setFirstFrictionDirection(toEigenVector3(ode->Fdir1()));
    dynamicsAspect->setFirstFrictionDirectionFrame(nullptr);
  }
}

//==============================================================================
void readVisualizationShapeNode(
    dynamics::BodyNode* bodyNode,
    const sdf::Visual& visual,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  std::string visualName = visual.Name();
  if (visualName.empty()) {
    visualName = "visual shape";
    DART_WARN(
        "Missing required attribute [name] in <visual> element of <link name = "
        "{}>.",
        bodyNode->getName());
  }

  dynamics::ShapeNode* newShapeNode = readShapeNode(
      bodyNode,
      visual.Geom(),
      visual.RawPose(),
      bodyNode->getName() + " - " + visualName,
      baseUri,
      retriever);

  newShapeNode->createVisualAspect();

  // Material
  if (const auto* material = visual.Material()) {
    readMaterial(*material, newShapeNode);
  }
}

//==============================================================================
void readCollisionShapeNode(
    dynamics::BodyNode* bodyNode,
    const sdf::Collision& collision,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  std::string collName = collision.Name();
  if (collName.empty()) {
    collName = "collision shape";
    DART_WARN(
        "Missing required attribute [name] in <collision> element of <link "
        "name = {}>.",
        bodyNode->getName());
  }

  dynamics::ShapeNode* newShapeNode = readShapeNode(
      bodyNode,
      collision.Geom(),
      collision.RawPose(),
      bodyNode->getName() + " - " + collName,
      baseUri,
      retriever);

  newShapeNode->createCollisionAspect();
  newShapeNode->createDynamicsAspect();
  readCollisionSurface(collision, newShapeNode);
}

//==============================================================================
void readAspects(
    const dynamics::SkeletonPtr& skeleton,
    const sdf::Model& model,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  for (uint64_t linkIndex = 0; linkIndex < model.LinkCount(); ++linkIndex) {
    const sdf::Link* link = model.LinkByIndex(linkIndex);
    if (!link) {
      continue;
    }

    auto bodyNode = skeleton->getBodyNode(link->Name());
    if (!bodyNode) {
      DART_WARN(
          "[SdfParser] Skipping visual/collision aspects for unknown Link "
          "[{}].",
          link->Name());
      continue;
    }

    // visualization_shape
    for (uint64_t visualIndex = 0; visualIndex < link->VisualCount();
         ++visualIndex) {
      const sdf::Visual* visual = link->VisualByIndex(visualIndex);
      if (visual) {
        readVisualizationShapeNode(bodyNode, *visual, baseUri, retriever);
      }
    }

    // collision_shape
    for (uint64_t collisionIndex = 0; collisionIndex < link->CollisionCount();
         ++collisionIndex) {
      const sdf::Collision* collision = link->CollisionByIndex(collisionIndex);
      if (collision) {
        readCollisionShapeNode(bodyNode, *collision, baseUri, retriever);
      }
    }
  }
}

//==============================================================================
JointMap readAllJoints(
    const sdf::Model& model,
    const Eigen::Isometry3d& skeletonFrame,
    const BodyMap& sdfBodyNodes)
{
  JointMap sdfJoints;
  for (uint64_t jointIndex = 0; jointIndex < model.JointCount(); ++jointIndex) {
    const sdf::Joint* jointDom = model.JointByIndex(jointIndex);
    if (!jointDom) {
      continue;
    }

    SDFJoint joint = readJoint(*jointDom, sdfBodyNodes, skeletonFrame);

    if (joint.childName.empty()) {
      DART_ERROR(
          "Joint named [{}] does not have a valid child Link, so it will not "
          "be added to the Skeleton",
          joint.properties->mName);
      continue;
    }

    JointMap::iterator it = sdfJoints.find(joint.childName);
    if (it != sdfJoints.end()) {
      DART_ERROR(
          "Joint named [{}] is claiming Link [{}] as its child, but that is "
          "already claimed by Joint [{}]. Joint [{}] will be discarded",
          joint.properties->mName,
          joint.childName,
          it->second.properties->mName,
          joint.properties->mName);
      continue;
    }

    sdfJoints[joint.childName] = joint;
  }

  return sdfJoints;
}

std::string_view getSdfJointTypeName(const sdf::JointType type)
{
  switch (type) {
    case sdf::JointType::BALL:
      return "ball";
    case sdf::JointType::CONTINUOUS:
      return "continuous";
    case sdf::JointType::FIXED:
      return "fixed";
    case sdf::JointType::GEARBOX:
      return "gearbox";
    case sdf::JointType::PRISMATIC:
      return "prismatic";
    case sdf::JointType::REVOLUTE:
      return "revolute";
    case sdf::JointType::REVOLUTE2:
      return "revolute2";
    case sdf::JointType::SCREW:
      return "screw";
    case sdf::JointType::UNIVERSAL:
      return "universal";
    case sdf::JointType::INVALID:
      break;
  }

  return "invalid";
}

std::vector<SDFJoint::MimicInfo> readMimicElements(const sdf::JointAxis& axis)
{
  std::vector<SDFJoint::MimicInfo> mimics;
  const auto mimic = axis.Mimic();
  if (!mimic) {
    return mimics;
  }

  SDFJoint::MimicInfo info;
  info.referenceJointName = mimic->Joint();
  info.referenceDof = mimic->Axis() == "axis2" ? 1u : 0u;
  info.multiplier = mimic->Multiplier();
  info.offset = mimic->Offset();
  mimics.push_back(info);

  return mimics;
}

SDFJoint readJoint(
    const sdf::Joint& sdfJoint,
    const BodyMap& _sdfBodyNodes,
    const Eigen::Isometry3d& _skeletonFrame)
{
  //--------------------------------------------------------------------------
  // Type attribute
  const sdf::JointType sdfType = sdfJoint.Type();
  std::string type(getSdfJointTypeName(sdfType));
  DART_ASSERT(!type.empty());

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = sdfJoint.Name();

  //--------------------------------------------------------------------------
  // parent
  BodyMap::const_iterator parent_it = _sdfBodyNodes.end();

  if (!sdfJoint.ParentName().empty()) {
    std::string strParent = sdfJoint.ParentName();

    if (strParent != std::string("world")) {
      parent_it = _sdfBodyNodes.find(strParent);

      if (parent_it == _sdfBodyNodes.end()) {
        DART_ERROR(
            "Cannot find a Link named [{}] requested as the parent of the "
            "Joint named [{}]",
            strParent,
            name);
        DART_ASSERT(0);
      }
    }
  } else {
    DART_ERROR("You must set parent link for the Joint [{}]!", name);
    DART_ASSERT(0);
  }

  //--------------------------------------------------------------------------
  // child
  BodyMap::const_iterator child_it = _sdfBodyNodes.end();

  if (!sdfJoint.ChildName().empty()) {
    std::string strChild = sdfJoint.ChildName();

    child_it = _sdfBodyNodes.find(strChild);

    if (child_it == _sdfBodyNodes.end()) {
      DART_ERROR(
          "Cannot find a Link named [{}] requested as the child of the Joint "
          "named [{}]",
          strChild,
          name);
      DART_ASSERT(0);
    }
  } else {
    DART_ERROR("You must set the child link for the Joint [{}]!", name);
    DART_ASSERT(0);
  }

  SDFJoint newJoint;
  newJoint.parentName
      = (parent_it == _sdfBodyNodes.end()) ? "" : parent_it->first;
  newJoint.childName = (child_it == _sdfBodyNodes.end()) ? "" : child_it->first;
  newJoint.sdfType = sdfType;

  //--------------------------------------------------------------------------
  // Mimic metadata (captured before joint creation)
  if (const sdf::JointAxis* axis = sdfJoint.Axis(0)) {
    auto mimics = readMimicElements(*axis);
    std::ranges::move(mimics, std::back_inserter(newJoint.mimicInfos));
  }
  if (const sdf::JointAxis* axis2 = sdfJoint.Axis(1)) {
    auto mimics = readMimicElements(*axis2);
    std::ranges::for_each(mimics, [](auto& mimic) {
      mimic.referenceDof = 1u; // axis2 maps to the second DoF
    });
    std::ranges::move(mimics, std::back_inserter(newJoint.mimicInfos));
  }

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childWorld = Eigen::Isometry3d::Identity();

  if (parent_it != _sdfBodyNodes.end()) {
    parentWorld = parent_it->second.initTransform;
  }
  if (child_it != _sdfBodyNodes.end()) {
    childWorld = child_it->second.initTransform;
  }
  childToJoint = toEigenIsometry3(sdfJoint.RawPose());

  Eigen::Isometry3d parentToJoint
      = parentWorld.inverse() * childWorld * childToJoint;

  // TODO: Workaround!!
  Eigen::Isometry3d parentModelFrame
      = (childWorld * childToJoint).inverse() * _skeletonFrame;

  if (sdfType == sdf::JointType::FIXED || type == std::string("fixed")) {
    newJoint.properties = dynamics::WeldJoint::Properties::createShared(
        readWeldJoint(sdfJoint, parentModelFrame, name));
  } else if (
      sdfType == sdf::JointType::PRISMATIC
      || type == std::string("prismatic")) {
    newJoint.properties = dynamics::PrismaticJoint::Properties::createShared(
        readPrismaticJoint(sdfJoint, parentModelFrame, name));
  } else if (
      sdfType == sdf::JointType::REVOLUTE
      || sdfType == sdf::JointType::CONTINUOUS
      || type == std::string("revolute")) {
    newJoint.properties = dynamics::RevoluteJoint::Properties::createShared(
        readRevoluteJoint(sdfJoint, parentModelFrame, name));
  } else if (sdfType == sdf::JointType::SCREW || type == std::string("screw")) {
    newJoint.properties = dynamics::ScrewJoint::Properties::createShared(
        readScrewJoint(sdfJoint, parentModelFrame, name));
  } else if (
      sdfType == sdf::JointType::REVOLUTE2
      || sdfType == sdf::JointType::UNIVERSAL
      || type == std::string("revolute2") || type == std::string("universal")) {
    newJoint.properties = dynamics::UniversalJoint::Properties::createShared(
        readUniversalJoint(sdfJoint, parentModelFrame, name));
  } else if (sdfType == sdf::JointType::BALL || type == std::string("ball")) {
    auto ballProperties = dynamics::BallJoint::Properties::createShared();
    *ballProperties = readBallJoint(sdfJoint, parentModelFrame, name);
    newJoint.properties = ballProperties;
  } else {
    DART_ERROR(
        "Unsupported joint type [{}]. Using [fixed] joint type instead.", type);
    newJoint.properties = dynamics::WeldJoint::Properties::createShared(
        readWeldJoint(sdfJoint, parentModelFrame, name));
  }

  newJoint.type = type;

  newJoint.properties->mName = name;

  newJoint.properties->mT_ChildBodyToJoint = childToJoint;
  newJoint.properties->mT_ParentBodyToJoint = parentToJoint;

  return newJoint;
}

static void reportMissingElement(
    std::string_view functionName,
    std::string_view elementName,
    std::string_view objectType,
    std::string_view objectName)
{
  DART_ERROR(
      "Missing element {} for {} named {}",
      functionName,
      elementName,
      objectType,
      objectName);
  DART_ASSERT(0);
}

Eigen::Vector3d resolveAxisXyz(
    const sdf::JointAxis& axisElement,
    const Eigen::Isometry3d& _parentModelFrame,
    const ElementPtr& axisXmlElement)
{
  gz::math::Vector3d xyz = axisElement.Xyz();

  if (!axisElement.XyzExpressedIn().empty()) {
    const sdf::Errors errors = axisElement.ResolveXyz(xyz);
    if (errors.empty()) {
      return toEigenVector3(xyz);
    }

    DART_WARN(
        "[SdfParser] Failed to resolve joint axis expressed in frame [{}]; "
        "falling back to the raw <xyz> value.",
        axisElement.XyzExpressedIn());
  }

  // Legacy Gazebo/SDF files used this extension before SDF gained
  // axis/xyz@expressed_in. Keep it as a compatibility fallback only.
  bool useParentModelFrame = false;
  if (hasAuthoredElement(axisXmlElement, "use_parent_model_frame")) {
    sdf::Errors errors;
    const auto [value, found] = axisXmlElement->Get<bool>(
        errors, "use_parent_model_frame", useParentModelFrame);
    if (found && errors.empty()) {
      useParentModelFrame = value;
    } else {
      DART_WARN(
          "[SdfParser] Failed to parse legacy "
          "<use_parent_model_frame> for a joint axis as bool.");
    }
  }

  Eigen::Vector3d axis = toEigenVector3(xyz);
  if (useParentModelFrame) {
    axis = _parentModelFrame.rotation() * axis;
  }

  return axis;
}

static bool readAxisElement(
    const sdf::JointAxis& axisElement,
    const Eigen::Isometry3d& _parentModelFrame,
    Eigen::Vector3d& axis,
    double& lower,
    double& upper,
    double& initial,
    double& rest,
    double& damping,
    double& friction,
    double& spring_stiffness)
{
  bool hasFinitePositionLimit = false;
  const ElementPtr axisXmlElement = axisElement.Element();

  // xyz
  axis = resolveAxisXyz(axisElement, _parentModelFrame, axisXmlElement);

  // dynamics
  if (hasAuthoredElement(axisXmlElement, "dynamics")) {
    const ElementPtr& dynamicsElement = getElement(axisXmlElement, "dynamics");

    // damping
    if (hasAuthoredElement(dynamicsElement, "damping")) {
      damping = axisElement.Damping();
    }

    // friction
    if (hasAuthoredElement(dynamicsElement, "friction")) {
      friction = axisElement.Friction();
    }

    // spring reference
    if (hasAuthoredElement(dynamicsElement, "spring_reference")) {
      rest = axisElement.SpringReference();
    }

    // spring stiffness
    if (hasAuthoredElement(dynamicsElement, "spring_stiffness")) {
      spring_stiffness = axisElement.SpringStiffness();
    }
  }

  // limit
  if (hasAuthoredElement(axisXmlElement, "limit")) {
    const ElementPtr& limitElement = getElement(axisXmlElement, "limit");

    // lower
    if (hasAuthoredElement(limitElement, "lower")) {
      lower = axisElement.Lower();
      hasFinitePositionLimit = hasFinitePositionLimit || std::isfinite(lower);
    }

    // upper
    if (hasAuthoredElement(limitElement, "upper")) {
      upper = axisElement.Upper();
      hasFinitePositionLimit = hasFinitePositionLimit || std::isfinite(upper);
    }
  }

  // If the zero position is out of our limits, we should change the initial
  // position instead of assuming zero
  if (0.0 < lower || upper < 0.0) {
    if (std::isfinite(lower) && std::isfinite(upper)) {
      initial = std::midpoint(lower, upper);
    } else if (std::isfinite(lower)) {
      initial = lower;
    } else if (std::isfinite(upper)) {
      initial = upper;
    }

    // Any other case means the limits are both +inf, both -inf, or one is a NaN

    // Apply the same logic to the rest position.
    rest = initial;
  }

  return hasFinitePositionLimit;
}

dart::dynamics::WeldJoint::Properties readWeldJoint(
    const sdf::Joint& /*joint*/, const Eigen::Isometry3d&, std::string_view)
{
  return dynamics::WeldJoint::Properties();
}

dynamics::RevoluteJoint::Properties readRevoluteJoint(
    const sdf::Joint& _revoluteJoint,
    const Eigen::Isometry3d& _parentModelFrame,
    std::string_view _name)
{
  dynamics::RevoluteJoint::Properties newRevoluteJoint;

  //--------------------------------------------------------------------------
  // axis
  if (const sdf::JointAxis* axisElement = _revoluteJoint.Axis(0)) {
    const bool hasLimitedAxis = readAxisElement(
        *axisElement,
        _parentModelFrame,
        newRevoluteJoint.mAxis,
        newRevoluteJoint.mPositionLowerLimits[0],
        newRevoluteJoint.mPositionUpperLimits[0],
        newRevoluteJoint.mInitialPositions[0],
        newRevoluteJoint.mRestPositions[0],
        newRevoluteJoint.mDampingCoefficients[0],
        newRevoluteJoint.mFrictions[0],
        newRevoluteJoint.mSpringStiffnesses[0]);
    newRevoluteJoint.mIsPositionLimitEnforced = hasLimitedAxis;
  } else {
    reportMissingElement("readRevoluteJoint", "axis", "joint", _name);
  }

  return newRevoluteJoint;
}

dynamics::PrismaticJoint::Properties readPrismaticJoint(
    const sdf::Joint& _joint,
    const Eigen::Isometry3d& _parentModelFrame,
    std::string_view _name)
{
  dynamics::PrismaticJoint::Properties newPrismaticJoint;

  //--------------------------------------------------------------------------
  // axis
  if (const sdf::JointAxis* axisElement = _joint.Axis(0)) {
    const bool hasLimitedAxis = readAxisElement(
        *axisElement,
        _parentModelFrame,
        newPrismaticJoint.mAxis,
        newPrismaticJoint.mPositionLowerLimits[0],
        newPrismaticJoint.mPositionUpperLimits[0],
        newPrismaticJoint.mInitialPositions[0],
        newPrismaticJoint.mRestPositions[0],
        newPrismaticJoint.mDampingCoefficients[0],
        newPrismaticJoint.mFrictions[0],
        newPrismaticJoint.mSpringStiffnesses[0]);
    newPrismaticJoint.mIsPositionLimitEnforced = hasLimitedAxis;
  } else {
    reportMissingElement("readPrismaticJoint", "axis", "joint", _name);
  }

  return newPrismaticJoint;
}

dynamics::ScrewJoint::Properties readScrewJoint(
    const sdf::Joint& _joint,
    const Eigen::Isometry3d& _parentModelFrame,
    std::string_view _name)
{
  dynamics::ScrewJoint::Properties newScrewJoint;

  //--------------------------------------------------------------------------
  // axis
  if (const sdf::JointAxis* axisElement = _joint.Axis(0)) {
    const bool hasLimitedAxis = readAxisElement(
        *axisElement,
        _parentModelFrame,
        newScrewJoint.mAxis,
        newScrewJoint.mPositionLowerLimits[0],
        newScrewJoint.mPositionUpperLimits[0],
        newScrewJoint.mInitialPositions[0],
        newScrewJoint.mRestPositions[0],
        newScrewJoint.mDampingCoefficients[0],
        newScrewJoint.mFrictions[0],
        newScrewJoint.mSpringStiffnesses[0]);
    newScrewJoint.mIsPositionLimitEnforced = hasLimitedAxis;
  } else {
    reportMissingElement("readScrewJoint", "axis", "joint", _name);
  }

  newScrewJoint.mPitch = _joint.ScrewThreadPitch();

  return newScrewJoint;
}

dynamics::UniversalJoint::Properties readUniversalJoint(
    const sdf::Joint& _joint,
    const Eigen::Isometry3d& _parentModelFrame,
    std::string_view _name)
{
  dynamics::UniversalJoint::Properties newUniversalJoint;
  bool hasLimitedAxis1 = false;
  bool hasLimitedAxis2 = false;

  //--------------------------------------------------------------------------
  // axis
  if (const sdf::JointAxis* axisElement = _joint.Axis(0)) {
    hasLimitedAxis1 = readAxisElement(
        *axisElement,
        _parentModelFrame,
        newUniversalJoint.mAxis[0],
        newUniversalJoint.mPositionLowerLimits[0],
        newUniversalJoint.mPositionUpperLimits[0],
        newUniversalJoint.mInitialPositions[0],
        newUniversalJoint.mRestPositions[0],
        newUniversalJoint.mDampingCoefficients[0],
        newUniversalJoint.mFrictions[0],
        newUniversalJoint.mSpringStiffnesses[0]);
  } else {
    reportMissingElement("readUniversalJoint", "axis", "joint", _name);
  }

  //--------------------------------------------------------------------------
  // axis2
  if (const sdf::JointAxis* axis2Element = _joint.Axis(1)) {
    hasLimitedAxis2 = readAxisElement(
        *axis2Element,
        _parentModelFrame,
        newUniversalJoint.mAxis[1],
        newUniversalJoint.mPositionLowerLimits[1],
        newUniversalJoint.mPositionUpperLimits[1],
        newUniversalJoint.mInitialPositions[1],
        newUniversalJoint.mRestPositions[1],
        newUniversalJoint.mDampingCoefficients[1],
        newUniversalJoint.mFrictions[1],
        newUniversalJoint.mSpringStiffnesses[1]);
  } else {
    reportMissingElement("readUniversalJoint", "axis2", "joint", _name);
  }

  newUniversalJoint.mIsPositionLimitEnforced
      = hasLimitedAxis1 || hasLimitedAxis2;

  return newUniversalJoint;
}

dynamics::BallJoint::Properties readBallJoint(
    const sdf::Joint& /*joint*/, const Eigen::Isometry3d&, std::string_view)
{
  return dynamics::BallJoint::Properties();
}

//==============================================================================
common::ResourceRetrieverPtr getRetriever(
    const common::ResourceRetrieverPtr& retriever)
{
  if (retriever) {
    return retriever;
  } else {
    auto newRetriever = std::make_shared<utils::CompositeResourceRetriever>();
    newRetriever->addSchemaRetriever(
        "file", std::make_shared<common::LocalResourceRetriever>());
    newRetriever->addSchemaRetriever("dart", DartResourceRetriever::create());

    return newRetriever;
  }
}

} // anonymous namespace

dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const Options& options)
{
  const auto resolvedOptions = resolveOptions(options);

  sdf::Root root;
  TemporaryResourceOwner tempResources;
  if (!loadSdfRoot(uri, resolvedOptions.retriever, root, tempResources)) {
    return nullptr;
  }

  const sdf::Model* model = root.Model();
  if (!model) {
    DART_WARN(
        "[SdfParser] [{}] does not contain a <model> element.", uri.toString());
    return nullptr;
  }

  return readSkeleton(*model, uri, resolvedOptions);
}

} // namespace SdfParser

} // namespace utils
} // namespace dart
