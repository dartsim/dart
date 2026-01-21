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
#include "dart/simulation/world.hpp"
#include "dart/utils/composite_resource_retriever.hpp"
#include "dart/utils/dart_resource_retriever.hpp"
#include "dart/utils/mesh_loader.hpp"
#include "dart/utils/sdf/detail/geometry_parsers.hpp"
#include "dart/utils/sdf/detail/sdf_helpers.hpp"
#include "dart/utils/skel_parser.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <sdf/sdf.hh>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
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

using detail::ElementEnumerator;
using detail::getAttributeString;
using detail::getElement;
using detail::getValueBool;
using detail::getValueDouble;
using detail::getValueIsometry3dWithExtrinsicRotation;
using detail::getValueString;
using detail::getValueUInt;
using detail::getValueVector2d;
using detail::getValueVector3d;
using detail::getValueVector3i;
using detail::getValueVectorXd;
using detail::hasAttribute;
using detail::hasElement;
using detail::readGeometryShape;

using TempResourceMap = std::unordered_map<std::string, common::Uri>;

thread_local TempResourceMap* gCurrentOriginMap = nullptr;

struct ScopedOriginMap
{
  explicit ScopedOriginMap(const std::shared_ptr<TempResourceMap>& map)
    : mPrevious(gCurrentOriginMap)
  {
    if (map)
      gCurrentOriginMap = map.get();
  }

  ~ScopedOriginMap()
  {
    gCurrentOriginMap = mPrevious;
  }

private:
  TempResourceMap* mPrevious;
};

const common::Uri* findOriginalUri(std::string_view filePath)
{
  if (!gCurrentOriginMap || filePath.empty())
    return nullptr;

  const std::string filePathString(filePath);
  const auto it = gCurrentOriginMap->find(filePathString);
  if (it == gCurrentOriginMap->end())
    return nullptr;

  return &it->second;
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
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using JointPropPtr = std::shared_ptr<dynamics::Joint::Properties>;

struct SDFJoint
{
  JointPropPtr properties;
  std::string parentName;
  std::string childName;
  std::string type;
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

std::vector<SDFJoint::MimicInfo> readMimicElements(
    const ElementPtr& axisElement);

// Maps the name of a BodyNode to its properties
using BodyMap = common::aligned_map<std::string, SDFBodyNode>;

// Maps a child BodyNode to the properties of its parent Joint
using JointMap = std::map<std::string, SDFJoint>;

simulation::WorldPtr readWorld(
    const ElementPtr& worldElement,
    const common::Uri& baseUri,
    const ResolvedOptions& options);

void readPhysics(const ElementPtr& physicsElement, simulation::WorldPtr world);

dynamics::SkeletonPtr readSkeleton(
    const ElementPtr& skeletonElement,
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
    const ElementPtr& skeletonElement, Eigen::Isometry3d& skeletonFrame);

common::Uri getElementBaseUri(
    const ElementPtr& element, const common::Uri& fallbackUri)
{
  if (!element)
    return fallbackUri;

  const auto& filePath = element->FilePath();
  if (filePath.empty())
    return fallbackUri;

  if (const auto* originalUri = findOriginalUri(filePath))
    return *originalUri;

  common::Uri elementUri;
  if (elementUri.fromPath(filePath))
    return elementUri;

  DART_WARN(
      "[SdfParser] Failed to parse file path [{}] for included element. "
      "Falling back to [{}].",
      filePath,
      fallbackUri.toString());
  return fallbackUri;
}

template <class NodeType>
std::pair<dynamics::Joint*, dynamics::BodyNode*> createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SDFJoint& joint,
    const SDFBodyNode& node);

BodyMap readAllBodyNodes(
    const ElementPtr& skeletonElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever,
    const Eigen::Isometry3d& skeletonFrame);

SDFBodyNode readBodyNode(
    const ElementPtr& bodyNodeElement,
    const Eigen::Isometry3d& skeletonFrame,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

dynamics::SoftBodyNode::UniqueProperties readSoftBodyProperties(
    const ElementPtr& softBodyNodeElement);

dynamics::ShapePtr readShape(
    const ElementPtr& _shapelement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& _retriever);

dynamics::ShapeNode* readShapeNode(
    dynamics::BodyNode* bodyNode,
    const ElementPtr& shapeNodeEle,
    std::string_view shapeNodeName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readMaterial(
    const ElementPtr& materialEle, dynamics::ShapeNode* shapeNode);

void readVisualizationShapeNode(
    dynamics::BodyNode* bodyNode,
    const ElementPtr& vizShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readCollisionShapeNode(
    dynamics::BodyNode* bodyNode,
    const ElementPtr& collShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readAspects(
    const dynamics::SkeletonPtr& skeleton,
    const ElementPtr& skeletonElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

JointMap readAllJoints(
    const ElementPtr& skeletonElement,
    const Eigen::Isometry3d& skeletonFrame,
    const BodyMap& sdfBodyNodes);

SDFJoint readJoint(
    const ElementPtr& jointElement,
    const BodyMap& bodies,
    const Eigen::Isometry3d& skeletonFrame);

dart::dynamics::WeldJoint::Properties readWeldJoint(
    const ElementPtr& jointElement,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

dynamics::RevoluteJoint::Properties readRevoluteJoint(
    const ElementPtr& revoluteJointElement,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

dynamics::PrismaticJoint::Properties readPrismaticJoint(
    const ElementPtr& jointElement,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

dynamics::ScrewJoint::Properties readScrewJoint(
    const ElementPtr& jointElement,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

dynamics::UniversalJoint::Properties readUniversalJoint(
    const ElementPtr& jointElement,
    const Eigen::Isometry3d& parentModelFrame,
    std::string_view name);

dynamics::BallJoint::Properties readBallJoint(
    const ElementPtr& jointElement,
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
  if (start == std::string_view::npos)
    return false;

  const auto uriStart = start + needle.size();
  const auto uriEnd = message.find(']', uriStart);
  if (uriEnd == std::string_view::npos)
    return false;

  uriValue = std::string(message.substr(uriStart, uriEnd - uriStart));
  return true;
}

void logSdformatErrors(
    const sdf::Errors& errors, const common::Uri& uri, std::string_view context)
{
  if (errors.empty())
    return;

  std::size_t warningCount = 0;
  std::size_t errorCount = 0;
  std::map<std::string, std::size_t> missingUriCounts;
  std::vector<std::string> representativeMessages;

  for (const auto& sdformatError : errors) {
    const bool isWarning = (sdformatError.Code() == sdf::ErrorCode::WARNING);
    if (isWarning)
      ++warningCount;
    else
      ++errorCount;

    const std::string description = describeSdformatError(sdformatError);
    std::string uriValue;
    if (isMissingUriError(description, uriValue)) {
      ++missingUriCounts[uriValue];
      continue;
    }

    if (representativeMessages.size() < 3)
      representativeMessages.push_back(description);
  }

  std::ostringstream stream;
  stream << "[SdfParser] " << context << " [" << uri.toString()
         << "]: " << errors.size() << " issue(s) detected (" << errorCount
         << " errors, " << warningCount << " warnings).";

  if (!missingUriCounts.empty()) {
    stream << " Missing URIs:";
    for (const auto& [missingUri, count] : missingUriCounts)
      stream << " [" << missingUri << " x" << count << ']';
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
    if (attempt > 0)
      name << '_' << attempt;
    if (!extension.empty())
      name << extension.string();

    const auto candidate = tempDir / name.str();
    if (!std::filesystem::exists(candidate))
      return candidate;
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
    const std::shared_ptr<std::vector<std::filesystem::path>>& tempFiles,
    const std::shared_ptr<TempResourceMap>& origins)
{
  if (!retriever)
    return std::string();

  const std::string requestedString(requested);
  common::Uri requestedUri;
  if (!requestedUri.fromStringOrPath(requestedString))
    return std::string();

  if (requestedUri.mScheme.get_value_or("file") == "file"
      && requestedUri.mPath) {
    std::error_code ec;
    const auto candidate = requestedUri.getFilesystemPath();
    if (std::filesystem::exists(candidate, ec) && !ec)
      return candidate;
  }

  if (!retriever->exists(requestedUri))
    return std::string();

  try {
    const auto data = retriever->readAll(requestedUri);
    const auto tmp = writeTemporaryResource(data, requestedUri);
    tempFiles->push_back(tmp);
    if (origins) {
      auto uri = common::Uri::createFromStringOrPath(requestedString);
      (*origins)[tmp.string()] = uri;
    }
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
  if (requested.empty())
    return false;

  if (hasUriScheme(requested))
    return false;

  if (requested.front() == '/')
    return false;

  if (isWindowsAbsolutePath(requested))
    return false;

  return true;
}

std::string resolveRequestedUri(
    std::string_view requested, const common::Uri& baseUri)
{
  if (!requiresBaseUriResolution(requested))
    return std::string(requested);

  if (!baseUri.mPath)
    return std::string(requested);

  const auto merged = common::Uri::getRelativeUri(baseUri, requested);
  if (!merged.empty())
    return merged;

  return std::string(requested);
}

void cleanupTemporaryResources(
    const std::shared_ptr<std::vector<std::filesystem::path>>& tempFiles)
{
  if (!tempFiles)
    return;

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
  std::shared_ptr<TempResourceMap> origins;
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
  resources.origins = std::make_shared<TempResourceMap>();
  sdf::ParserConfig config = sdf::ParserConfig::GlobalConfig();
  config.SetFindCallback(
      [retriever,
       tempFiles = resources.files,
       origins = resources.origins,
       baseUri = uri](const std::string& requested) -> std::string {
        const auto resolvedRequest = resolveRequestedUri(requested, baseUri);
        return resolveWithRetriever(
            resolvedRequest, retriever, tempFiles, origins);
      });

  sdf::Errors errors;
  std::string localPath;
  if (uri.mScheme.get_value_or("file") == "file" && uri.mPath) {
    const auto candidate = uri.getFilesystemPath();
    std::error_code ec;
    if (std::filesystem::exists(candidate, ec) && !ec)
      localPath = candidate;
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
      resources.origins.reset();
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
    resources.origins.reset();
    return false;
  }

  return true;
}

//==============================================================================
simulation::WorldPtr readWorld(
    const ElementPtr& worldElement,
    const common::Uri& baseUri,
    const ResolvedOptions& options)
{
  DART_ASSERT(worldElement != nullptr);

  // Create a world
  simulation::WorldPtr newWorld = simulation::World::create();

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttributeString(worldElement, "name");
  newWorld->setName(name);

  //--------------------------------------------------------------------------
  // Load physics
  if (hasElement(worldElement, "physics")) {
    const ElementPtr& physicsElement = getElement(worldElement, "physics");
    readPhysics(physicsElement, newWorld);
  }

  //--------------------------------------------------------------------------
  // Load skeletons
  ElementEnumerator skeletonElements(worldElement, "model");
  while (skeletonElements.next()) {
    const ElementPtr skeletonElement = skeletonElements.get();
    const common::Uri skeletonBaseUri
        = getElementBaseUri(skeletonElement, baseUri);
    dynamics::SkeletonPtr newSkeleton
        = readSkeleton(skeletonElement, skeletonBaseUri, options);

    newWorld->addSkeleton(newSkeleton);
  }

  return newWorld;
}

//==============================================================================
void readPhysics(const ElementPtr& physicsElement, simulation::WorldPtr world)
{
  // Type attribute
  // std::string physicsEngineName = getAttribute(_physicsElement, "type");

  // Time step
  if (hasElement(physicsElement, "max_step_size")) {
    double timeStep = getValueDouble(physicsElement, "max_step_size");
    world->setTimeStep(timeStep);
  }

  // Number of max contacts
  // if (hasElement(_physicsElement, "max_contacts"))
  // {
  //   int timeStep = getValueInt(_physicsElement, "max_contacts");
  //   _world->setMaxNumContacts(timeStep);
  // }

  // Gravity
  if (hasElement(physicsElement, "gravity")) {
    Eigen::Vector3d gravity = getValueVector3d(physicsElement, "gravity");
    world->setGravity(gravity);
  }
}

//==============================================================================
dynamics::SkeletonPtr readSkeleton(
    const ElementPtr& skeletonElement,
    const common::Uri& baseUri,
    const ResolvedOptions& options)
{
  DART_ASSERT(skeletonElement != nullptr);

  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();
  dynamics::SkeletonPtr newSkeleton
      = makeSkeleton(skeletonElement, skeletonFrame);

  //--------------------------------------------------------------------------
  // Bodies
  BodyMap sdfBodyNodes = readAllBodyNodes(
      skeletonElement, baseUri, options.retriever, skeletonFrame);

  //--------------------------------------------------------------------------
  // Joints
  JointMap sdfJoints
      = readAllJoints(skeletonElement, skeletonFrame, sdfBodyNodes);

  // Iterate through the collected properties and construct the Skeleton from
  // the root nodes downward
  BodyMap::iterator body = sdfBodyNodes.begin();
  JointMap::const_iterator parentJoint;
  dynamics::BodyNode* parentBody{nullptr};
  while (body != sdfBodyNodes.end()) {
    NextResult result = getNextJointAndNodePair(
        body, parentJoint, parentBody, newSkeleton, sdfBodyNodes, sdfJoints);

    if (BREAK == result)
      break;
    else if (CONTINUE == result)
      continue;
    else if (CREATE_ROOT_JOINT == result) {
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
            static_cast<int>(options.defaultRootJointType));
      }

      if (!createPair(newSkeleton, nullptr, rootJoint, body->second))
        break;

      sdfBodyNodes.erase(body);
      body = sdfBodyNodes.begin();

      continue;
    }

    if (!createPair(newSkeleton, parentBody, parentJoint->second, body->second))
      break;

    sdfBodyNodes.erase(body);
    body = sdfBodyNodes.begin();
  }

  // Read aspects here since aspects cannot be added if the BodyNodes haven't
  // created yet.
  readAspects(newSkeleton, skeletonElement, baseUri, options.retriever);

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

  if (!pair.first || !pair.second)
    return false;

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
  for (const auto& entry : sdfJoints) {
    const auto& jointInfo = entry.second;
    if (jointInfo.mimicInfos.empty())
      continue;

    auto* joint = skeleton->getJoint(jointInfo.properties->mName);
    if (!joint)
      continue;

    const auto existingProps = joint->getMimicDofProperties();
    std::vector<dynamics::MimicDofProperties> props(joint->getNumDofs());
    for (std::size_t i = 0; i < existingProps.size() && i < props.size(); ++i)
      props[i] = existingProps[i];

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

    if (!applied)
      continue;

    joint->setMimicJointDofs(
        std::span<const dynamics::MimicDofProperties>(props));
    joint->setActuatorType(dynamics::Joint::MIMIC);
    joint->setUseCouplerConstraint(useCoupler);
  }
}

dynamics::SkeletonPtr makeSkeleton(
    const ElementPtr& _skeletonElement, Eigen::Isometry3d& skeletonFrame)
{
  DART_ASSERT(_skeletonElement != nullptr);

  dynamics::SkeletonPtr newSkeleton = dynamics::Skeleton::create();

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttributeString(_skeletonElement, "name");
  newSkeleton->setName(name);

  //--------------------------------------------------------------------------
  // immobile attribute
  if (hasElement(_skeletonElement, "static")) {
    bool isStatic = getValueBool(_skeletonElement, "static");
    newSkeleton->setMobile(!isStatic);
  }

  //--------------------------------------------------------------------------
  // transformation
  if (hasElement(_skeletonElement, "pose")) {
    Eigen::Isometry3d W
        = getValueIsometry3dWithExtrinsicRotation(_skeletonElement, "pose");
    skeletonFrame = W;
  }

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

  if (std::string("prismatic") == type)
    return skeleton->createJointAndBodyNodePair<dynamics::PrismaticJoint>(
        parent,
        static_cast<const dynamics::PrismaticJoint::Properties&>(
            *joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  else if (std::string("revolute") == type)
    return skeleton->createJointAndBodyNodePair<dynamics::RevoluteJoint>(
        parent,
        static_cast<const dynamics::RevoluteJoint::Properties&>(
            *joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  else if (std::string("screw") == type)
    return skeleton->createJointAndBodyNodePair<dynamics::ScrewJoint>(
        parent,
        static_cast<const dynamics::ScrewJoint::Properties&>(*joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  else if (std::string("revolute2") == type || std::string("universal") == type)
    return skeleton->createJointAndBodyNodePair<dynamics::UniversalJoint>(
        parent,
        static_cast<const dynamics::UniversalJoint::Properties&>(
            *joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  else if (std::string("ball") == type)
    return skeleton->createJointAndBodyNodePair<dynamics::BallJoint>(
        parent,
        static_cast<const dynamics::BallJoint::Properties&>(*joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  else if (std::string("fixed") == type)
    return skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
        parent,
        static_cast<const dynamics::WeldJoint::Properties&>(*joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));
  else if (std::string("free") == type)
    return skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
        parent,
        static_cast<const dynamics::FreeJoint::Properties&>(*joint.properties),
        static_cast<const typename NodeType::Properties&>(*node.properties));

  DART_ERROR(
      "{}{}. Please report this as a bug! We will now quit parsing.",
      "Unsupported Joint type encountered: ",
      type);
  return std::pair<dynamics::Joint*, dynamics::BodyNode*>(nullptr, nullptr);
}

//==============================================================================
BodyMap readAllBodyNodes(
    const ElementPtr& skeletonElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever,
    const Eigen::Isometry3d& skeletonFrame)
{
  ElementEnumerator bodies(skeletonElement, "link");
  BodyMap sdfBodyNodes;
  while (bodies.next()) {
    SDFBodyNode body
        = readBodyNode(bodies.get(), skeletonFrame, baseUri, retriever);

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
    const ElementPtr& bodyNodeElement,
    const Eigen::Isometry3d& skeletonFrame,
    const common::Uri& /*baseUri*/,
    const common::ResourceRetrieverPtr& /*retriever*/)
{
  DART_ASSERT(bodyNodeElement != nullptr);

  dynamics::BodyNode::Properties properties;
  Eigen::Isometry3d initTransform = Eigen::Isometry3d::Identity();

  // Name attribute
  std::string name = getAttributeString(bodyNodeElement, "name");
  properties.mName = name;
  const std::string bodyName = name;

  //--------------------------------------------------------------------------
  // gravity
  if (hasElement(bodyNodeElement, "gravity")) {
    bool gravityMode = getValueBool(bodyNodeElement, "gravity");
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
  if (hasElement(bodyNodeElement, "pose")) {
    Eigen::Isometry3d W
        = getValueIsometry3dWithExtrinsicRotation(bodyNodeElement, "pose");
    initTransform = skeletonFrame * W;
  } else {
    initTransform = skeletonFrame;
  }

  //--------------------------------------------------------------------------
  // inertia
  constexpr double kMinReasonableMass = 1e-9; // 1 microgram
  bool massSpecified = false;
  if (hasElement(bodyNodeElement, "inertial")) {
    const ElementPtr& inertiaElement = getElement(bodyNodeElement, "inertial");

    // mass
    if (hasElement(inertiaElement, "mass")) {
      double mass = getValueDouble(inertiaElement, "mass");
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
    if (hasElement(inertiaElement, "pose")) {
      Eigen::Isometry3d T
          = getValueIsometry3dWithExtrinsicRotation(inertiaElement, "pose");
      properties.mInertia.setLocalCOM(T.translation());
    }

    // inertia
    if (hasElement(inertiaElement, "inertia")) {
      const ElementPtr& moiElement = getElement(inertiaElement, "inertia");

      double ixx = getValueDouble(moiElement, "ixx");
      double iyy = getValueDouble(moiElement, "iyy");
      double izz = getValueDouble(moiElement, "izz");

      double ixy = getValueDouble(moiElement, "ixy");
      double ixz = getValueDouble(moiElement, "ixz");
      double iyz = getValueDouble(moiElement, "iyz");

      properties.mInertia.setMoment(ixx, iyy, izz, ixy, ixz, iyz);
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
    if (hasElement(softShapeEle, "pose"))
      T = getValueIsometry3dWithExtrinsicRotation(softShapeEle, "pose");

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
    const ElementPtr& _shapelement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& _retriever)
{
  if (!hasElement(_shapelement, "geometry")) {
    DART_WARN("Shape node is missing required <geometry> element.");
    return nullptr;
  }

  const ElementPtr& geometryElement = getElement(_shapelement, "geometry");
  return readGeometryShape(geometryElement, baseUri, _retriever);
}

//==============================================================================
dynamics::ShapeNode* readShapeNode(
    dynamics::BodyNode* bodyNode,
    const ElementPtr& shapeNodeEle,
    std::string_view shapeNodeName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  DART_ASSERT(bodyNode);

  auto shape = readShape(shapeNodeEle, baseUri, retriever);
  auto shapeNode = bodyNode->createShapeNode(shape, shapeNodeName);

  // Transformation
  if (hasElement(shapeNodeEle, "pose")) {
    const Eigen::Isometry3d W
        = getValueIsometry3dWithExtrinsicRotation(shapeNodeEle, "pose");
    shapeNode->setRelativeTransform(W);
  }

  return shapeNode;
}

//==============================================================================
void readMaterial(const ElementPtr& materialEle, dynamics::ShapeNode* shapeNode)
{
  auto visualAspect = shapeNode->getVisualAspect();
  if (hasElement(materialEle, "diffuse")) {
    Eigen::VectorXd color = getValueVectorXd(materialEle, "diffuse");
    if (color.size() == 3) {
      Eigen::Vector3d color3d = color;
      visualAspect->setColor(color3d);
    } else if (color.size() == 4) {
      Eigen::Vector4d color4d = color;
      visualAspect->setColor(color4d);
    } else {
      DART_ERROR("Unsupported color vector size: {}", color.size());
    }
  }
}

//==============================================================================
void readVisualizationShapeNode(
    dynamics::BodyNode* bodyNode,
    const ElementPtr& vizShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  std::string visualName = "visual shape";
  if (hasAttribute(vizShapeNodeEle, "name")) {
    visualName = getAttributeString(vizShapeNodeEle, "name");
  } else {
    DART_WARN(
        "Missing required attribute [name] in <visual> element of <link name = "
        "{}>.",
        bodyNode->getName());
  }

  dynamics::ShapeNode* newShapeNode = readShapeNode(
      bodyNode,
      vizShapeNodeEle,
      bodyNode->getName() + " - " + visualName,
      baseUri,
      retriever);

  newShapeNode->createVisualAspect();

  // Material
  if (hasElement(vizShapeNodeEle, "material")) {
    const ElementPtr& materialEle = getElement(vizShapeNodeEle, "material");
    readMaterial(materialEle, newShapeNode);
  }
}

//==============================================================================
void readCollisionShapeNode(
    dynamics::BodyNode* bodyNode,
    const ElementPtr& collShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  std::string collName = "collision shape";
  if (hasAttribute(collShapeNodeEle, "name")) {
    collName = getAttributeString(collShapeNodeEle, "name");
  } else {
    DART_WARN(
        "Missing required attribute [name] in <collision> element of <link "
        "name = {}>.",
        bodyNode->getName());
  }

  dynamics::ShapeNode* newShapeNode = readShapeNode(
      bodyNode,
      collShapeNodeEle,
      bodyNode->getName() + " - " + collName,
      baseUri,
      retriever);

  newShapeNode->createCollisionAspect();
  newShapeNode->createDynamicsAspect();
}

//==============================================================================
void readAspects(
    const dynamics::SkeletonPtr& skeleton,
    const ElementPtr& skeletonElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever)
{
  ElementEnumerator xmlBodies(skeletonElement, "link");
  while (xmlBodies.next()) {
    auto bodyElement = xmlBodies.get();
    auto bodyNodeName = getAttributeString(bodyElement, "name");
    auto bodyNode = skeleton->getBodyNode(bodyNodeName);

    // visualization_shape
    ElementEnumerator vizShapes(bodyElement, "visual");
    while (vizShapes.next()) {
      readVisualizationShapeNode(bodyNode, vizShapes.get(), baseUri, retriever);
    }

    // collision_shape
    ElementEnumerator collShapes(bodyElement, "collision");
    while (collShapes.next())
      readCollisionShapeNode(bodyNode, collShapes.get(), baseUri, retriever);
  }
}

//==============================================================================
JointMap readAllJoints(
    const ElementPtr& _skeletonElement,
    const Eigen::Isometry3d& skeletonFrame,
    const BodyMap& sdfBodyNodes)
{
  JointMap sdfJoints;
  ElementEnumerator joints(_skeletonElement, "joint");
  while (joints.next()) {
    SDFJoint joint = readJoint(joints.get(), sdfBodyNodes, skeletonFrame);

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

std::vector<SDFJoint::MimicInfo> readMimicElements(
    const ElementPtr& axisElement)
{
  std::vector<SDFJoint::MimicInfo> mimics;
  if (!axisElement || !hasElement(axisElement, "mimic"))
    return mimics;

  const auto mimicElement = getElement(axisElement, "mimic");
  if (!mimicElement)
    return mimics;

  SDFJoint::MimicInfo info;
  info.referenceJointName = getAttributeString(mimicElement, "joint");
  if (hasAttribute(mimicElement, "axis")) {
    const auto axisAttr = getAttributeString(mimicElement, "axis");
    info.referenceDof = axisAttr == "axis2" ? 1u : 0u;
  }
  info.multiplier = hasElement(mimicElement, "multiplier")
                        ? getValueDouble(mimicElement, "multiplier")
                        : 1.0;
  info.offset = hasElement(mimicElement, "offset")
                    ? getValueDouble(mimicElement, "offset")
                    : 0.0;
  mimics.push_back(info);

  return mimics;
}

SDFJoint readJoint(
    const ElementPtr& _jointElement,
    const BodyMap& _sdfBodyNodes,
    const Eigen::Isometry3d& _skeletonFrame)
{
  DART_ASSERT(_jointElement != nullptr);

  //--------------------------------------------------------------------------
  // Type attribute
  std::string type = getAttributeString(_jointElement, "type");
  DART_ASSERT(!type.empty());

  //--------------------------------------------------------------------------
  // Name attribute
  std::string name = getAttributeString(_jointElement, "name");

  //--------------------------------------------------------------------------
  // parent
  BodyMap::const_iterator parent_it = _sdfBodyNodes.end();

  if (hasElement(_jointElement, "parent")) {
    std::string strParent = getValueString(_jointElement, "parent");

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

  if (hasElement(_jointElement, "child")) {
    std::string strChild = getValueString(_jointElement, "child");

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

  //--------------------------------------------------------------------------
  // Mimic metadata (captured before joint creation)
  if (hasElement(_jointElement, "axis")) {
    const ElementPtr& axisElement = getElement(_jointElement, "axis");
    const auto mimics = readMimicElements(axisElement);
    newJoint.mimicInfos.insert(
        newJoint.mimicInfos.end(), mimics.begin(), mimics.end());
  }
  if (hasElement(_jointElement, "axis2")) {
    const ElementPtr& axis2Element = getElement(_jointElement, "axis2");
    auto mimics = readMimicElements(axis2Element);
    for (auto& m : mimics)
      m.referenceDof = 1u; // axis2 maps to the second DoF
    newJoint.mimicInfos.insert(
        newJoint.mimicInfos.end(), mimics.begin(), mimics.end());
  }

  //--------------------------------------------------------------------------
  // transformation
  Eigen::Isometry3d parentWorld = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childToJoint = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d childWorld = Eigen::Isometry3d::Identity();

  if (parent_it != _sdfBodyNodes.end())
    parentWorld = parent_it->second.initTransform;
  if (child_it != _sdfBodyNodes.end())
    childWorld = child_it->second.initTransform;
  if (hasElement(_jointElement, "pose"))
    childToJoint
        = getValueIsometry3dWithExtrinsicRotation(_jointElement, "pose");

  Eigen::Isometry3d parentToJoint
      = parentWorld.inverse() * childWorld * childToJoint;

  // TODO: Workaround!!
  Eigen::Isometry3d parentModelFrame
      = (childWorld * childToJoint).inverse() * _skeletonFrame;

  if (type == std::string("fixed")) {
    newJoint.properties = dynamics::WeldJoint::Properties::createShared(
        readWeldJoint(_jointElement, parentModelFrame, name));
  } else if (type == std::string("prismatic")) {
    newJoint.properties = dynamics::PrismaticJoint::Properties::createShared(
        readPrismaticJoint(_jointElement, parentModelFrame, name));
  } else if (type == std::string("revolute")) {
    newJoint.properties = dynamics::RevoluteJoint::Properties::createShared(
        readRevoluteJoint(_jointElement, parentModelFrame, name));
  } else if (type == std::string("screw")) {
    newJoint.properties = dynamics::ScrewJoint::Properties::createShared(
        readScrewJoint(_jointElement, parentModelFrame, name));
  } else if (
      type == std::string("revolute2") || type == std::string("universal")) {
    newJoint.properties = dynamics::UniversalJoint::Properties::createShared(
        readUniversalJoint(_jointElement, parentModelFrame, name));
  } else if (type == std::string("ball")) {
    newJoint.properties = dynamics::BallJoint::Properties::createShared(
        readBallJoint(_jointElement, parentModelFrame, name));
  } else {
    DART_ERROR(
        "Unsupported joint type [{}]. Using [fixed] joint type instead.", type);
    newJoint.properties = dynamics::WeldJoint::Properties::createShared(
        readWeldJoint(_jointElement, parentModelFrame, name));
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

static bool readAxisElement(
    const ElementPtr& axisElement,
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

  // use_parent_model_frame
  bool useParentModelFrame = false;
  if (hasElement(axisElement, "use_parent_model_frame"))
    useParentModelFrame = getValueBool(axisElement, "use_parent_model_frame");

  // xyz
  Eigen::Vector3d xyz = getValueVector3d(axisElement, "xyz");
  if (useParentModelFrame) {
    xyz = _parentModelFrame.rotation() * xyz;
  }
  axis = xyz;

  // dynamics
  if (hasElement(axisElement, "dynamics")) {
    const ElementPtr& dynamicsElement = getElement(axisElement, "dynamics");

    // damping
    if (hasElement(dynamicsElement, "damping")) {
      damping = getValueDouble(dynamicsElement, "damping");
    }

    // friction
    if (hasElement(dynamicsElement, "friction")) {
      friction = getValueDouble(dynamicsElement, "friction");
    }

    // spring reference
    if (hasElement(dynamicsElement, "spring_reference")) {
      rest = getValueDouble(dynamicsElement, "spring_reference");
    }

    // spring stiffness
    if (hasElement(dynamicsElement, "spring_stiffness")) {
      spring_stiffness = getValueDouble(dynamicsElement, "spring_stiffness");
    }
  }

  // limit
  if (hasElement(axisElement, "limit")) {
    const ElementPtr& limitElement = getElement(axisElement, "limit");

    // lower
    if (hasElement(limitElement, "lower")) {
      lower = getValueDouble(limitElement, "lower");
      hasFinitePositionLimit = hasFinitePositionLimit || std::isfinite(lower);
    }

    // upper
    if (hasElement(limitElement, "upper")) {
      upper = getValueDouble(limitElement, "upper");
      hasFinitePositionLimit = hasFinitePositionLimit || std::isfinite(upper);
    }
  }

  // If the zero position is out of our limits, we should change the initial
  // position instead of assuming zero
  if (0.0 < lower || upper < 0.0) {
    if (std::isfinite(lower) && std::isfinite(upper))
      initial = (lower + upper) / 2.0;
    else if (std::isfinite(lower))
      initial = lower;
    else if (std::isfinite(upper))
      initial = upper;

    // Any other case means the limits are both +inf, both -inf, or one is a NaN

    // Apply the same logic to the rest position.
    rest = initial;
  }

  return hasFinitePositionLimit;
}

dart::dynamics::WeldJoint::Properties readWeldJoint(
    const ElementPtr& /*_jointElement*/,
    const Eigen::Isometry3d&,
    std::string_view)
{
  return dynamics::WeldJoint::Properties();
}

dynamics::RevoluteJoint::Properties readRevoluteJoint(
    const ElementPtr& _revoluteJointElement,
    const Eigen::Isometry3d& _parentModelFrame,
    std::string_view _name)
{
  DART_ASSERT(_revoluteJointElement != nullptr);

  dynamics::RevoluteJoint::Properties newRevoluteJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_revoluteJointElement, "axis")) {
    const ElementPtr& axisElement = getElement(_revoluteJointElement, "axis");

    const bool hasLimitedAxis = readAxisElement(
        axisElement,
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
    const ElementPtr& _jointElement,
    const Eigen::Isometry3d& _parentModelFrame,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::PrismaticJoint::Properties newPrismaticJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    const ElementPtr& axisElement = getElement(_jointElement, "axis");

    const bool hasLimitedAxis = readAxisElement(
        axisElement,
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
    const ElementPtr& _jointElement,
    const Eigen::Isometry3d& _parentModelFrame,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::ScrewJoint::Properties newScrewJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    const ElementPtr& axisElement = getElement(_jointElement, "axis");

    const bool hasLimitedAxis = readAxisElement(
        axisElement,
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

  // pitch
  if (hasElement(_jointElement, "thread_pitch")) {
    double pitch = getValueDouble(_jointElement, "thread_pitch");
    newScrewJoint.mPitch = pitch;
  }

  return newScrewJoint;
}

dynamics::UniversalJoint::Properties readUniversalJoint(
    const ElementPtr& _jointElement,
    const Eigen::Isometry3d& _parentModelFrame,
    std::string_view _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::UniversalJoint::Properties newUniversalJoint;
  bool hasLimitedAxis1 = false;
  bool hasLimitedAxis2 = false;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    const ElementPtr& axisElement = getElement(_jointElement, "axis");

    hasLimitedAxis1 = readAxisElement(
        axisElement,
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
  if (hasElement(_jointElement, "axis2")) {
    const ElementPtr& axis2Element = getElement(_jointElement, "axis2");

    hasLimitedAxis2 = readAxisElement(
        axis2Element,
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
    const ElementPtr& /*_jointElement*/,
    const Eigen::Isometry3d&,
    std::string_view)
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

//==============================================================================
simulation::WorldPtr readWorld(const common::Uri& uri, const Options& options)
{
  const auto resolvedOptions = resolveOptions(options);

  sdf::Root root;
  TemporaryResourceOwner tempResources;
  if (!loadSdfRoot(uri, resolvedOptions.retriever, root, tempResources))
    return nullptr;

  const ElementPtr sdfElement = root.Element();
  if (!sdfElement) {
    DART_WARN(
        "[SdfParser] [{}] does not contain a valid <sdf> root element.",
        uri.toString());
    return nullptr;
  }

  const ElementPtr worldElement = getElement(sdfElement, "world");
  if (!worldElement) {
    DART_WARN(
        "[SdfParser] [{}] does not contain a <world> element.", uri.toString());
    return nullptr;
  }

  ScopedOriginMap originScope(tempResources.origins);
  return readWorld(worldElement, uri, resolvedOptions);
}

//==============================================================================
dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const Options& options)
{
  const auto resolvedOptions = resolveOptions(options);

  sdf::Root root;
  TemporaryResourceOwner tempResources;
  if (!loadSdfRoot(uri, resolvedOptions.retriever, root, tempResources))
    return nullptr;

  const ElementPtr sdfElement = root.Element();
  if (!sdfElement) {
    DART_WARN(
        "[SdfParser] [{}] does not contain a valid <sdf> root element.",
        uri.toString());
    return nullptr;
  }

  const ElementPtr modelElement = getElement(sdfElement, "model");
  if (!modelElement) {
    DART_WARN(
        "[SdfParser] [{}] does not contain a <model> element.", uri.toString());
    return nullptr;
  }

  ScopedOriginMap originScope(tempResources.origins);
  return readSkeleton(modelElement, uri, resolvedOptions);
}

} // namespace SdfParser

} // namespace utils
} // namespace dart
