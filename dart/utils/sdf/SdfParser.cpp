/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#include "dart/utils/sdf/SdfParser.hpp"

#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Logging.hpp"
#include "dart/common/Macros.hpp"
#include "dart/common/ResourceRetriever.hpp"
#include "dart/common/Uri.hpp"
#include "dart/config.hpp"
#include "dart/dynamics/BallJoint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ScrewJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/UniversalJoint.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/simulation/World.hpp"
#include "dart/utils/CompositeResourceRetriever.hpp"
#include "dart/utils/DartResourceRetriever.hpp"
#include "dart/utils/SkelParser.hpp"
#include "dart/utils/XmlHelpers.hpp"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <tinyxml2.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#if HAVE_SDFORMAT
  #include <sdf/Error.hh>
  #include <sdf/ParserConfig.hh>
  #include <sdf/PrintConfig.hh>
  #include <sdf/Root.hh>
#endif

namespace dart {
namespace utils {

namespace SdfParser {

namespace {

bool loadSdfXmlDocument(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retriever,
    bool& normalizedBySdformat);

#if HAVE_SDFORMAT
std::optional<std::string> canonicalizeSdfWithSdformat(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever);
#endif

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
};

// Maps the name of a BodyNode to its properties
using BodyMap = common::aligned_map<std::string, SDFBodyNode>;

// Maps a child BodyNode to the properties of its parent Joint
using JointMap = std::map<std::string, SDFJoint>;

simulation::WorldPtr readWorld(
    tinyxml2::XMLElement* worldElement,
    const common::Uri& baseUri,
    const Options& options);

void readPhysics(
    tinyxml2::XMLElement* physicsElement, simulation::WorldPtr world);

dynamics::SkeletonPtr readSkeleton(
    tinyxml2::XMLElement* skeletonElement,
    const common::Uri& baseUri,
    const Options& options);

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
    tinyxml2::XMLElement* skeletonElement, Eigen::Isometry3d& skeletonFrame);

template <class NodeType>
std::pair<dynamics::Joint*, dynamics::BodyNode*> createJointAndNodePair(
    dynamics::SkeletonPtr skeleton,
    dynamics::BodyNode* parent,
    const SDFJoint& joint,
    const SDFBodyNode& node);

BodyMap readAllBodyNodes(
    tinyxml2::XMLElement* skeletonElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever,
    const Eigen::Isometry3d& skeletonFrame);

SDFBodyNode readBodyNode(
    tinyxml2::XMLElement* bodyNodeElement,
    const Eigen::Isometry3d& skeletonFrame,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

dynamics::SoftBodyNode::UniqueProperties readSoftBodyProperties(
    tinyxml2::XMLElement* softBodyNodeElement);

dynamics::ShapePtr readShape(
    tinyxml2::XMLElement* shapelement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

dynamics::ShapeNode* readShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* shapeNodeEle,
    const std::string& shapeNodeName,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readMaterial(
    tinyxml2::XMLElement* materialEle, dynamics::ShapeNode* shapeNode);

void readVisualizationShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* vizShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readCollisionShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* collShapeNodeEle,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

void readAspects(
    const dynamics::SkeletonPtr& skeleton,
    tinyxml2::XMLElement* skeletonElement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& retriever);

JointMap readAllJoints(
    tinyxml2::XMLElement* skeletonElement,
    const Eigen::Isometry3d& skeletonFrame,
    const BodyMap& sdfBodyNodes);

SDFJoint readJoint(
    tinyxml2::XMLElement* jointElement,
    const BodyMap& bodies,
    const Eigen::Isometry3d& skeletonFrame);

dart::dynamics::WeldJoint::Properties readWeldJoint(
    tinyxml2::XMLElement* jointElement,
    const Eigen::Isometry3d& parentModelFrame,
    const std::string& name);

dynamics::RevoluteJoint::Properties readRevoluteJoint(
    tinyxml2::XMLElement* revoluteJointElement,
    const Eigen::Isometry3d& parentModelFrame,
    const std::string& name);

dynamics::PrismaticJoint::Properties readPrismaticJoint(
    tinyxml2::XMLElement* jointElement,
    const Eigen::Isometry3d& parentModelFrame,
    const std::string& name);

dynamics::ScrewJoint::Properties readScrewJoint(
    tinyxml2::XMLElement* jointElement,
    const Eigen::Isometry3d& parentModelFrame,
    const std::string& name);

dynamics::UniversalJoint::Properties readUniversalJoint(
    tinyxml2::XMLElement* jointElement,
    const Eigen::Isometry3d& parentModelFrame,
    const std::string& name);

dynamics::BallJoint::Properties readBallJoint(
    tinyxml2::XMLElement* jointElement,
    const Eigen::Isometry3d& parentModelFrame,
    const std::string& name);

bool loadSdfXmlDocument(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retriever,
    bool& normalizedBySdformat)
{
  normalizedBySdformat = false;

#if HAVE_SDFORMAT
  if (const auto normalized = canonicalizeSdfWithSdformat(uri, retriever)) {
    const auto result = doc.Parse(normalized->c_str());
    if (result == tinyxml2::XML_SUCCESS) {
      normalizedBySdformat = true;
      return true;
    }

    DART_WARN(
        "[SdfParser] Failed to parse libsdformat-normalized document for [{}]: "
        "{}. Falling back to TinyXML2.",
        uri.toString(),
        toString(result));
    doc.Clear();
  }
#else
  (void)retriever;
#endif

  try {
    openXMLFile(doc, uri, retriever);
    return true;
  } catch (const std::exception& e) {
    DART_WARN("Loading file [{}] failed: {}", uri.toString(), e.what());
    return false;
  }
}

#if HAVE_SDFORMAT
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

bool logSdformatErrors(
    const sdf::Errors& errors,
    const common::Uri& uri,
    const std::string& context)
{
  bool hasCriticalErrors = false;
  if (errors.empty())
    return false;

  for (const auto& error : errors) {
    DART_WARN(
        "[SdfParser] {} [{}]: {}",
        context,
        uri.toString(),
        describeSdformatError(error));
    if (error.Code() != sdf::ErrorCode::WARNING)
      hasCriticalErrors = true;
  }

  return hasCriticalErrors;
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
    const std::string& data, const common::Uri& uri)
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
    const std::string& requested,
    const common::ResourceRetrieverPtr& retriever,
    const std::shared_ptr<std::vector<std::filesystem::path>>& tempFiles)
{
  if (!retriever)
    return std::string();

  common::Uri requestedUri;
  if (!requestedUri.fromStringOrPath(requested))
    return std::string();

  const auto path = retriever->getFilePath(requestedUri);
  if (!path.empty())
    return path;

  if (!retriever->exists(requestedUri))
    return std::string();

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

std::optional<std::string> canonicalizeSdfWithSdformat(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
{
  if (!retriever)
    return std::nullopt;

  auto tempFiles = std::make_shared<std::vector<std::filesystem::path>>();
  sdf::ParserConfig config = sdf::ParserConfig::GlobalConfig();
  config.SetFindCallback(
      [retriever, tempFiles](const std::string& requested) -> std::string {
        return resolveWithRetriever(requested, retriever, tempFiles);
      });

  sdf::Root root;
  sdf::Errors errors;
  const auto localPath = retriever->getFilePath(uri);
  if (!localPath.empty()) {
    errors = root.Load(localPath, config);
  } else {
    std::string content;
    try {
      content = retriever->readAll(uri);
    } catch (const std::exception& e) {
      DART_WARN(
          "[SdfParser] Failed to read [{}] via ResourceRetriever: {}",
          uri.toString(),
          e.what());
      cleanupTemporaryResources(tempFiles);
      return std::nullopt;
    }
    errors = root.LoadSdfString(content, config);
  }

  const bool hasCriticalLoadErrors = logSdformatErrors(
      errors, uri, "libsdformat reported an issue while loading");
  if (hasCriticalLoadErrors) {
    cleanupTemporaryResources(tempFiles);
    return std::nullopt;
  }

  const auto element = root.Element();
  if (element == nullptr) {
    DART_WARN(
        "[SdfParser] libsdformat returned an empty document for [{}].",
        uri.toString());
    cleanupTemporaryResources(tempFiles);
    return std::nullopt;
  }

  sdf::Errors printErrors;
  sdf::PrintConfig printConfig;
  printConfig.SetPreserveIncludes(false);
  const std::string xml
      = element->ToString(printErrors, "", true, false, printConfig);
  const bool hasCriticalPrintErrors = logSdformatErrors(
      printErrors,
      uri,
      "libsdformat reported an issue while serializing canonical XML");
  cleanupTemporaryResources(tempFiles);

  if (hasCriticalPrintErrors)
    return std::nullopt;

  return xml;
}
#endif // HAVE_SDFORMAT

} // anonymous namespace

//==============================================================================
Options::Options(
    common::ResourceRetrieverPtr resourceRetriever,
    RootJointType defaultRootJointType)
  : mResourceRetriever(std::move(resourceRetriever)),
    mDefaultRootJointType(defaultRootJointType)
{
  // Do nothing
}

//==============================================================================
bool checkVersion(
    const tinyxml2::XMLElement& sdfElement,
    const common::Uri& uri,
    bool allowFutureVersions)
{
  const std::string version = getAttributeString(&sdfElement, "version");
  const bool supported
      = (version == "1.4" || version == "1.5" || version == "1.6");

  if (supported)
    return true;

  if (allowFutureVersions) {
    DART_WARN(
        "[SdfParser] The file format of [{}] reports version [{}], which is "
        "newer than the built-in parser understands. Continuing because the "
        "document was normalized by libsdformat.",
        uri.toString(),
        version.empty() ? "unknown" : version);
    return true;
  }

  DART_WARN(
      "[SdfParser] The file format of [{}] was found to be [{}], but we only "
      "support SDF 1.4, 1.5, and 1.6!",
      uri.toString(),
      version);
  return false;
}

//==============================================================================
simulation::WorldPtr readWorld(const common::Uri& uri, const Options& options)
{
  const auto retriever = getRetriever(options.mResourceRetriever);

  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument sdfFile;
  bool normalizedBySdformat = false;
  if (!loadSdfXmlDocument(sdfFile, uri, retriever, normalizedBySdformat))
    return nullptr;

  //--------------------------------------------------------------------------
  // Load DART
  tinyxml2::XMLElement* sdfElement = nullptr;
  sdfElement = sdfFile.FirstChildElement("sdf");
  if (sdfElement == nullptr)
    return nullptr;

  //--------------------------------------------------------------------------
  // version attribute
  if (!checkVersion(*sdfElement, uri, normalizedBySdformat))
    return nullptr;

  //--------------------------------------------------------------------------
  // Load World
  tinyxml2::XMLElement* worldElement = nullptr;
  worldElement = sdfElement->FirstChildElement("world");
  if (worldElement == nullptr)
    return nullptr;

  return readWorld(worldElement, uri, options);
}

//==============================================================================
dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const Options& options)
{
  const auto retriever = getRetriever(options.mResourceRetriever);

  //--------------------------------------------------------------------------
  // Load xml and create Document
  tinyxml2::XMLDocument _dartFile;
  bool normalizedBySdformat = false;
  if (!loadSdfXmlDocument(_dartFile, uri, retriever, normalizedBySdformat))
    return nullptr;

  //--------------------------------------------------------------------------
  // Load sdf
  tinyxml2::XMLElement* sdfElement = nullptr;
  sdfElement = _dartFile.FirstChildElement("sdf");
  if (sdfElement == nullptr)
    return nullptr;

  //--------------------------------------------------------------------------
  // version attribute
  if (!checkVersion(*sdfElement, uri, normalizedBySdformat))
    return nullptr;

  //--------------------------------------------------------------------------
  // Load skeleton
  tinyxml2::XMLElement* skelElement = nullptr;
  skelElement = sdfElement->FirstChildElement("model");
  if (skelElement == nullptr)
    return nullptr;

  dynamics::SkeletonPtr newSkeleton = readSkeleton(skelElement, uri, retriever);

  return newSkeleton;
}

namespace {

//==============================================================================
simulation::WorldPtr readWorld(
    tinyxml2::XMLElement* worldElement,
    const common::Uri& baseUri,
    const Options& options)
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
    tinyxml2::XMLElement* physicsElement
        = worldElement->FirstChildElement("physics");
    readPhysics(physicsElement, newWorld);
  }

  //--------------------------------------------------------------------------
  // Load skeletons
  ElementEnumerator skeletonElements(worldElement, "model");
  while (skeletonElements.next()) {
    dynamics::SkeletonPtr newSkeleton
        = readSkeleton(skeletonElements.get(), baseUri, options);

    newWorld->addSkeleton(newSkeleton);
  }

  return newWorld;
}

//==============================================================================
void readPhysics(
    tinyxml2::XMLElement* physicsElement, simulation::WorldPtr world)
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
    tinyxml2::XMLElement* skeletonElement,
    const common::Uri& baseUri,
    const Options& options)
{
  DART_ASSERT(skeletonElement != nullptr);

  Eigen::Isometry3d skeletonFrame = Eigen::Isometry3d::Identity();
  dynamics::SkeletonPtr newSkeleton
      = makeSkeleton(skeletonElement, skeletonFrame);

  //--------------------------------------------------------------------------
  // Bodies
  BodyMap sdfBodyNodes = readAllBodyNodes(
      skeletonElement, baseUri, options.mResourceRetriever, skeletonFrame);

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
      if (options.mDefaultRootJointType == RootJointType::FLOATING) {
        // If a root FreeJoint is needed for the parent of the current joint,
        // then create it
        rootJoint.properties = dynamics::FreeJoint::Properties::createShared(
            dynamics::Joint::Properties("root", body->second.initTransform));
        rootJoint.type = "free";
      } else if (options.mDefaultRootJointType == RootJointType::FIXED) {
        // If a root WeldJoint is needed for the parent of the current joint,
        // then create it
        rootJoint.properties = dynamics::WeldJoint::Properties::createShared(
            dynamics::Joint::Properties("root", body->second.initTransform));
        rootJoint.type = "fixed";
      } else {
        DART_WARN(
            "Unsupported root joint type [{}]. Using FLOATING by default.",
            static_cast<int>(options.mDefaultRootJointType));
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
  readAspects(
      newSkeleton, skeletonElement, baseUri, options.mResourceRetriever);

  // Set positions to their initial values
  newSkeleton->resetPositions();

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

dynamics::SkeletonPtr makeSkeleton(
    tinyxml2::XMLElement* _skeletonElement, Eigen::Isometry3d& skeletonFrame)
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
    tinyxml2::XMLElement* skeletonElement,
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
    tinyxml2::XMLElement* bodyNodeElement,
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
  if (hasElement(bodyNodeElement, "inertial")) {
    tinyxml2::XMLElement* inertiaElement
        = getElement(bodyNodeElement, "inertial");

    // mass
    if (hasElement(inertiaElement, "mass")) {
      double mass = getValueDouble(inertiaElement, "mass");
      properties.mInertia.setMass(mass);
    }

    // offset
    if (hasElement(inertiaElement, "pose")) {
      Eigen::Isometry3d T
          = getValueIsometry3dWithExtrinsicRotation(inertiaElement, "pose");
      properties.mInertia.setLocalCOM(T.translation());
    }

    // inertia
    if (hasElement(inertiaElement, "inertia")) {
      tinyxml2::XMLElement* moiElement = getElement(inertiaElement, "inertia");

      double ixx = getValueDouble(moiElement, "ixx");
      double iyy = getValueDouble(moiElement, "iyy");
      double izz = getValueDouble(moiElement, "izz");

      double ixy = getValueDouble(moiElement, "ixy");
      double ixz = getValueDouble(moiElement, "ixz");
      double iyz = getValueDouble(moiElement, "iyz");

      properties.mInertia.setMoment(ixx, iyy, izz, ixy, ixz, iyz);
    }
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
    tinyxml2::XMLElement* softBodyNodeElement)
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
    tinyxml2::XMLElement* softShapeEle
        = getElement(softBodyNodeElement, "soft_shape");

    // mass
    double totalMass = getValueDouble(softShapeEle, "total_mass");

    // pose
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    if (hasElement(softShapeEle, "pose"))
      T = getValueIsometry3dWithExtrinsicRotation(softShapeEle, "pose");

    // geometry
    tinyxml2::XMLElement* geometryEle = getElement(softShapeEle, "geometry");
    if (hasElement(geometryEle, "sphere")) {
      tinyxml2::XMLElement* sphereEle = getElement(geometryEle, "sphere");
      const auto radius = getValueDouble(sphereEle, "radius");
      const auto nSlices = getValueUInt(sphereEle, "num_slices");
      const auto nStacks = getValueUInt(sphereEle, "num_stacks");
      softProperties = dynamics::SoftBodyNodeHelper::makeSphereProperties(
          radius, nSlices, nStacks, totalMass);
    } else if (hasElement(geometryEle, "box")) {
      tinyxml2::XMLElement* boxEle = getElement(geometryEle, "box");
      Eigen::Vector3d size = getValueVector3d(boxEle, "size");
      Eigen::Vector3i frags = getValueVector3i(boxEle, "frags");
      softProperties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
          size, T, frags, totalMass);
    } else if (hasElement(geometryEle, "ellipsoid")) {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "ellipsoid");
      Eigen::Vector3d size = getValueVector3d(ellipsoidEle, "size");
      const auto nSlices = getValueUInt(ellipsoidEle, "num_slices");
      const auto nStacks = getValueUInt(ellipsoidEle, "num_stacks");
      softProperties = dynamics::SoftBodyNodeHelper::makeEllipsoidProperties(
          size, nSlices, nStacks, totalMass);
    } else if (hasElement(geometryEle, "cylinder")) {
      tinyxml2::XMLElement* ellipsoidEle = getElement(geometryEle, "cylinder");
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
    tinyxml2::XMLElement* _shapelement,
    const common::Uri& baseUri,
    const common::ResourceRetrieverPtr& _retriever)
{
  dynamics::ShapePtr newShape;

  // type
  DART_ASSERT(hasElement(_shapelement, "geometry"));
  tinyxml2::XMLElement* geometryElement = getElement(_shapelement, "geometry");

  if (hasElement(geometryElement, "sphere")) {
    tinyxml2::XMLElement* sphereElement = getElement(geometryElement, "sphere");

    const auto radius = getValueDouble(sphereElement, "radius");

    newShape = dynamics::ShapePtr(new dynamics::SphereShape(radius));
  } else if (hasElement(geometryElement, "box")) {
    tinyxml2::XMLElement* boxElement = getElement(geometryElement, "box");

    Eigen::Vector3d size = getValueVector3d(boxElement, "size");

    newShape = dynamics::ShapePtr(new dynamics::BoxShape(size));
  } else if (hasElement(geometryElement, "cylinder")) {
    tinyxml2::XMLElement* cylinderElement
        = getElement(geometryElement, "cylinder");

    double radius = getValueDouble(cylinderElement, "radius");
    double height = getValueDouble(cylinderElement, "length");

    newShape = dynamics::ShapePtr(new dynamics::CylinderShape(radius, height));
  } else if (hasElement(geometryElement, "plane")) {
    // TODO: Don't support plane shape yet.
    tinyxml2::XMLElement* planeElement = getElement(geometryElement, "plane");

    Eigen::Vector2d visSize = getValueVector2d(planeElement, "size");
    // TODO: Need to use normal for correct orientation of the plane
    // Eigen::Vector3d normal = getValueVector3d(planeElement, "normal");

    Eigen::Vector3d size(visSize(0), visSize(1), 0.001);

    newShape = dynamics::ShapePtr(new dynamics::BoxShape(size));
  } else if (hasElement(geometryElement, "mesh")) {
    tinyxml2::XMLElement* meshEle = getElement(geometryElement, "mesh");
    // TODO(JS): We assume that uri is just file name for the mesh
    if (!hasElement(meshEle, "uri")) {
      // TODO(MXG): Figure out how to report the file name and line number of
      DART_WARN("Mesh is missing a URI, which is required in order to load it");
      return nullptr;
    }
    std::string uri = getValueString(meshEle, "uri");

    Eigen::Vector3d scale = hasElement(meshEle, "scale")
                                ? getValueVector3d(meshEle, "scale")
                                : Eigen::Vector3d::Ones();

    const std::string meshUri = common::Uri::getRelativeUri(baseUri, uri);
    const aiScene* model = dynamics::MeshShape::loadMesh(meshUri, _retriever);

    if (model)
      newShape = std::make_shared<dynamics::MeshShape>(
          scale, model, meshUri, _retriever);
    else {
      DART_WARN("Failed to load mesh model [{}].", meshUri);
      return nullptr;
    }
  } else {
    std::cout << "Invalid shape type." << std::endl;
    return nullptr;
  }

  return newShape;
}

//==============================================================================
dynamics::ShapeNode* readShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* shapeNodeEle,
    const std::string& shapeNodeName,
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
void readMaterial(
    tinyxml2::XMLElement* materialEle, dynamics::ShapeNode* shapeNode)
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
    tinyxml2::XMLElement* vizShapeNodeEle,
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
    tinyxml2::XMLElement* materialEle = getElement(vizShapeNodeEle, "material");
    readMaterial(materialEle, newShapeNode);
  }
}

//==============================================================================
void readCollisionShapeNode(
    dynamics::BodyNode* bodyNode,
    tinyxml2::XMLElement* collShapeNodeEle,
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
    tinyxml2::XMLElement* skeletonElement,
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
    tinyxml2::XMLElement* _skeletonElement,
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

SDFJoint readJoint(
    tinyxml2::XMLElement* _jointElement,
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
    const std::string& functionName,
    const std::string& elementName,
    const std::string& objectType,
    const std::string& objectName)
{
  DART_ERROR(
      "Missing element {} for {} named {}",
      functionName,
      elementName,
      objectType,
      objectName);
  DART_ASSERT(0);
}

static void readAxisElement(
    tinyxml2::XMLElement* axisElement,
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
    tinyxml2::XMLElement* dynamicsElement = getElement(axisElement, "dynamics");

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
    tinyxml2::XMLElement* limitElement = getElement(axisElement, "limit");

    // lower
    if (hasElement(limitElement, "lower")) {
      lower = getValueDouble(limitElement, "lower");
    }

    // upper
    if (hasElement(limitElement, "upper")) {
      upper = getValueDouble(limitElement, "upper");
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
}

dart::dynamics::WeldJoint::Properties readWeldJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    const Eigen::Isometry3d&,
    const std::string&)
{
  return dynamics::WeldJoint::Properties();
}

dynamics::RevoluteJoint::Properties readRevoluteJoint(
    tinyxml2::XMLElement* _revoluteJointElement,
    const Eigen::Isometry3d& _parentModelFrame,
    const std::string& _name)
{
  DART_ASSERT(_revoluteJointElement != nullptr);

  dynamics::RevoluteJoint::Properties newRevoluteJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_revoluteJointElement, "axis")) {
    tinyxml2::XMLElement* axisElement
        = getElement(_revoluteJointElement, "axis");

    readAxisElement(
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
  } else {
    reportMissingElement("readRevoluteJoint", "axis", "joint", _name);
  }

  return newRevoluteJoint;
}

dynamics::PrismaticJoint::Properties readPrismaticJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d& _parentModelFrame,
    const std::string& _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::PrismaticJoint::Properties newPrismaticJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    readAxisElement(
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
  } else {
    reportMissingElement("readPrismaticJoint", "axis", "joint", _name);
  }

  return newPrismaticJoint;
}

dynamics::ScrewJoint::Properties readScrewJoint(
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d& _parentModelFrame,
    const std::string& _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::ScrewJoint::Properties newScrewJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    readAxisElement(
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
    tinyxml2::XMLElement* _jointElement,
    const Eigen::Isometry3d& _parentModelFrame,
    const std::string& _name)
{
  DART_ASSERT(_jointElement != nullptr);

  dynamics::UniversalJoint::Properties newUniversalJoint;

  //--------------------------------------------------------------------------
  // axis
  if (hasElement(_jointElement, "axis")) {
    tinyxml2::XMLElement* axisElement = getElement(_jointElement, "axis");

    readAxisElement(
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
    tinyxml2::XMLElement* axis2Element = getElement(_jointElement, "axis2");

    readAxisElement(
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

  return newUniversalJoint;
}

dynamics::BallJoint::Properties readBallJoint(
    tinyxml2::XMLElement* /*_jointElement*/,
    const Eigen::Isometry3d&,
    const std::string&)
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

} // namespace SdfParser

} // namespace utils
} // namespace dart
