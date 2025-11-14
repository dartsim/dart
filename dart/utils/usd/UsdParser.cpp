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

#include "dart/utils/usd/UsdParser.hpp"

#include <dart/utils/CompositeResourceRetriever.hpp>
#include <dart/utils/DartResourceRetriever.hpp>
#include <dart/utils/PackageResourceRetriever.hpp>

#include <dart/simulation/World.hpp>

#include <dart/dynamics/FreeJoint.hpp>
#include <dart/dynamics/PrismaticJoint.hpp>
#include <dart/dynamics/RevoluteJoint.hpp>
#include <dart/dynamics/Skeleton.hpp>
#include <dart/dynamics/WeldJoint.hpp>

#include <dart/common/LocalResourceRetriever.hpp>
#include <dart/common/Logging.hpp>
#include <dart/common/Uri.hpp>

#include <Eigen/Geometry>

#if defined(__GNUC__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wcpp"
#endif
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/timeCode.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/scene.h>
#if defined(__GNUC__)
  #pragma GCC diagnostic pop
#endif

#include <algorithm>
#include <functional>
#include <limits>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <cctype>

PXR_NAMESPACE_USING_DIRECTIVE

namespace dart {
namespace utils {
namespace UsdParser {

namespace {

using Path = pxr::SdfPath;
using PathHash = pxr::SdfPath::Hash;

const pxr::TfToken kMassToken("physics:mass");
const pxr::TfToken kDiagonalInertiaToken("physics:diagonalInertia");
const pxr::TfToken kCenterOfMassToken("physics:centerOfMass");

struct BodyInfo
{
  Path path;
  std::string name;
  Eigen::Isometry3d worldTransform{Eigen::Isometry3d::Identity()};
  dynamics::BodyNode::Properties properties;
};

enum class JointType
{
  REVOLUTE,
  PRISMATIC,
  FIXED,
};

struct JointInfo
{
  Path path;
  std::string name;
  Path parent;
  Path child;
  JointType type{JointType::REVOLUTE};
  Eigen::Isometry3d parentToJoint{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d childToJoint{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
  double lowerLimit{-std::numeric_limits<double>::infinity()};
  double upperLimit{std::numeric_limits<double>::infinity()};
};

struct ArticulationDefinition
{
  std::string name;
  Path rootPath;
  std::vector<Path> bodyPaths;
  std::vector<Path> jointPaths;
};

struct StageData
{
  std::unordered_map<Path, BodyInfo, PathHash> bodies;
  std::unordered_map<Path, JointInfo, PathHash> joints;
  std::vector<ArticulationDefinition> articulations;
  Eigen::Vector3d gravity{Eigen::Vector3d(0.0, -9.81, 0.0)};
};

Eigen::Vector3d toEigen(const pxr::GfVec3f& vec)
{
  return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}

Eigen::Quaterniond toEigen(const pxr::GfQuatf& quat)
{
  const auto imag = quat.GetImaginary();
  Eigen::Quaterniond eigen(quat.GetReal(), imag[0], imag[1], imag[2]);
  eigen.normalize();
  return eigen;
}

Eigen::Isometry3d toEigen(const pxr::GfMatrix4d& matrix)
{
  Eigen::Matrix4d eigen = Eigen::Matrix4d::Identity();
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      eigen(row, col) = matrix[row][col];

  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.matrix() = eigen;
  return iso;
}

Eigen::Isometry3d makeTransform(
    const pxr::GfVec3f& pos, const pxr::GfQuatf& rot)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translate(toEigen(pos));
  transform.rotate(toEigen(rot));
  return transform;
}

std::string sanitizeName(const Path& path)
{
  std::string name = path.GetString();
  if (name.empty())
    name = "usd_prim";

  std::replace(name.begin(), name.end(), '/', '_');
  std::replace(name.begin(), name.end(), ':', '_');
  while (!name.empty() && name.front() == '_')
    name.erase(name.begin());
  return name;
}

common::ResourceRetrieverPtr getRetriever(
    const common::ResourceRetrieverPtr& retriever)
{
  if (retriever)
    return retriever;

  auto composite = std::make_shared<utils::CompositeResourceRetriever>();
  composite->addSchemaRetriever(
      "file", std::make_shared<common::LocalResourceRetriever>());
  composite->addSchemaRetriever("dart", DartResourceRetriever::create());
  composite->addSchemaRetriever(
      "package", std::make_shared<PackageResourceRetriever>());
  return composite;
}

pxr::UsdStageRefPtr openStage(
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retriever,
    bool verbose)
{
  if (!retriever || !retriever->exists(uri)) {
    DART_WARN("USD parser could not locate resource '{}'.", uri.toString());
    return nullptr;
  }

  const auto filePath = retriever->getFilePath(uri);
  if (!filePath.empty()) {
    auto stage = pxr::UsdStage::Open(filePath);
    if (!stage)
      DART_WARN(
          "Failed to open USD stage '{}' from path '{}'",
          uri.toString(),
          filePath);
    return stage;
  }

  auto resource = retriever->retrieve(uri);
  if (!resource) {
    DART_WARN(
        "Failed to retrieve USD payload '{}' via retriever.", uri.toString());
    return nullptr;
  }

  const auto buffer = resource->readAll();
  auto layer = pxr::SdfLayer::CreateAnonymous("dart_import.usda");
  if (!layer->ImportFromString(buffer)) {
    DART_WARN(
        "Unable to import USD layer '{}' from memory. Ensure referenced "
        "assets\n"
        "are accessible from the runtime pxr resolver.",
        uri.toString());
    return nullptr;
  }

  if (verbose)
    DART_INFO(
        "Opened in-memory USD layer '{}' for '{}'",
        layer->GetIdentifier(),
        uri.toString());

  return pxr::UsdStage::Open(layer);
}

Eigen::Isometry3d computeWorldTransform(const pxr::UsdPrim& prim)
{
  pxr::UsdGeomXformable xformable(prim);
  if (!xformable)
    return Eigen::Isometry3d::Identity();

  auto matrix
      = xformable.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());
  return toEigen(matrix);
}

BodyInfo parseRigidBody(const pxr::UsdPrim& prim)
{
  BodyInfo info;
  info.path = prim.GetPath();
  info.name = sanitizeName(info.path);
  info.worldTransform = computeWorldTransform(prim);
  info.properties.mName = info.name;

  dynamics::Inertia inertia;
  double mass = 1.0;
  if (auto attr = prim.GetAttribute(kMassToken)) {
    attr.Get(&mass);
  }
  inertia.setMass(mass);
  inertia.setMoment(Eigen::Matrix3d::Identity());

  pxr::GfVec3f diag(0.f);
  if (auto attr = prim.GetAttribute(kDiagonalInertiaToken)) {
    if (attr.Get(&diag) && (diag[0] > 0.f || diag[1] > 0.f || diag[2] > 0.f)) {
      inertia.setMoment(diag[0], diag[1], diag[2], 0.0, 0.0, 0.0);
    }
  }

  pxr::GfVec3f com(0.f);
  if (auto attr = prim.GetAttribute(kCenterOfMassToken)) {
    if (attr.Get(&com))
      inertia.setLocalCOM(toEigen(com));
  }

  info.properties.mInertia = inertia;
  return info;
}

bool isRigidBodyPrim(const pxr::UsdPrim& prim)
{
  if (!prim.IsValid())
    return false;

  pxr::UsdPhysicsRigidBodyAPI rigidBody(prim);
  if (rigidBody)
    return true;

  pxr::UsdAttribute massAttr = prim.GetAttribute(kMassToken);
  return massAttr && massAttr.HasValue();
}

JointInfo parseJoint(const pxr::UsdPrim& prim)
{
  JointInfo info;
  info.path = prim.GetPath();
  info.name = prim.GetName().GetString();
  if (info.name.empty())
    info.name = sanitizeName(info.path);
  info.parentToJoint = Eigen::Isometry3d::Identity();
  info.childToJoint = Eigen::Isometry3d::Identity();

  pxr::UsdPhysicsJoint joint(prim);
  if (joint) {
    std::vector<Path> targets;
    if (joint.GetBody0Rel().GetTargets(&targets) && !targets.empty())
      info.parent = targets.front();
    targets.clear();
    if (joint.GetBody1Rel().GetTargets(&targets) && !targets.empty())
      info.child = targets.front();

    pxr::GfVec3f pos0(0.f);
    pxr::GfVec3f pos1(0.f);
    pxr::GfQuatf rot0(1.f);
    pxr::GfQuatf rot1(1.f);
    joint.GetLocalPos0Attr().Get(&pos0);
    joint.GetLocalPos1Attr().Get(&pos1);
    joint.GetLocalRot0Attr().Get(&rot0);
    joint.GetLocalRot1Attr().Get(&rot1);

    info.parentToJoint = makeTransform(pos0, rot0);
    info.childToJoint = makeTransform(pos1, rot1);
  }

  pxr::UsdPhysicsPrismaticJoint prismatic(prim);
  pxr::UsdPhysicsFixedJoint fixed(prim);
  pxr::UsdPhysicsRevoluteJoint revolute(prim);

  if (prismatic) {
    info.type = JointType::PRISMATIC;
    pxr::GfVec3f axis(0.f, 0.f, 1.f);
    prismatic.GetAxisAttr().Get(&axis);
    info.axis = toEigen(axis);
    float limit = 0.f;
    if (prismatic.GetLowerLimitAttr().Get(&limit))
      info.lowerLimit = limit;
    if (prismatic.GetUpperLimitAttr().Get(&limit))
      info.upperLimit = limit;
  } else if (fixed) {
    info.type = JointType::FIXED;
  } else {
    info.type = JointType::REVOLUTE;
    pxr::GfVec3f axis(0.f, 0.f, 1.f);
    if (revolute)
      revolute.GetAxisAttr().Get(&axis);
    info.axis = toEigen(axis);
    if (info.axis.norm() == 0)
      info.axis = Eigen::Vector3d::UnitZ();
    info.axis.normalize();

    float limit = 0.f;
    if (revolute && revolute.GetLowerLimitAttr().Get(&limit))
      info.lowerLimit = limit;
    if (revolute && revolute.GetUpperLimitAttr().Get(&limit))
      info.upperLimit = limit;
  }

  return info;
}

void collectArticulations(const pxr::UsdStageRefPtr& stage, StageData& data)
{
  std::unordered_set<Path, PathHash> assignedBodies;
  std::vector<Path> rootPaths;

  for (const auto& prim : pxr::UsdPrimRange(stage->GetPseudoRoot())) {
    if (!prim.IsActive())
      continue;
    if (pxr::UsdPhysicsArticulationRootAPI(prim))
      rootPaths.push_back(prim.GetPath());
  }

  for (const auto& rootPath : rootPaths) {
    auto rootPrim = stage->GetPrimAtPath(rootPath);
    if (!rootPrim)
      continue;

    ArticulationDefinition def;
    def.rootPath = rootPath;
    def.name = sanitizeName(rootPath);

    for (const auto& prim : pxr::UsdPrimRange(rootPrim)) {
      const auto path = prim.GetPath();
      if (auto bodyIt = data.bodies.find(path); bodyIt != data.bodies.end()) {
        def.bodyPaths.push_back(path);
        assignedBodies.insert(path);
      }
      if (auto jointIt = data.joints.find(path); jointIt != data.joints.end())
        def.jointPaths.push_back(path);
    }

    if (!def.bodyPaths.empty())
      data.articulations.push_back(def);
  }

  std::vector<Path> looseBodies;
  for (const auto& [path, _] : data.bodies) {
    if (!assignedBodies.count(path))
      looseBodies.push_back(path);
  }

  if (!looseBodies.empty()) {
    ArticulationDefinition fallback;
    fallback.name = "usd_scene_" + std::to_string(data.articulations.size());
    fallback.bodyPaths = looseBodies;
    for (const auto& [path, joint] : data.joints) {
      const bool parentIncluded
          = std::find(looseBodies.begin(), looseBodies.end(), joint.parent)
            != looseBodies.end();
      const bool childIncluded
          = std::find(looseBodies.begin(), looseBodies.end(), joint.child)
            != looseBodies.end();
      if (parentIncluded && childIncluded)
        fallback.jointPaths.push_back(path);
    }

    data.articulations.push_back(fallback);
  }
}

StageData buildStageData(
    const pxr::UsdStageRefPtr& stage, const Options& options)
{
  StageData data;

  for (const auto& prim : pxr::UsdPrimRange(stage->GetPseudoRoot())) {
    if (!prim.IsActive())
      continue;

    if (isRigidBodyPrim(prim)) {
      data.bodies.emplace(prim.GetPath(), parseRigidBody(prim));
      continue;
    }

    if (pxr::UsdPhysicsJoint(prim)) {
      data.joints.emplace(prim.GetPath(), parseJoint(prim));
      continue;
    }

    pxr::UsdPhysicsScene scene(prim);
    if (scene) {
      pxr::GfVec3f gravityDir(0.f, -1.f, 0.f);
      float gravityMag = 9.81f;
      scene.GetGravityDirectionAttr().Get(&gravityDir);
      scene.GetGravityMagnitudeAttr().Get(&gravityMag);
      if (gravityDir.GetLengthSq() == 0.f)
        data.gravity = Eigen::Vector3d(0.0, -gravityMag, 0.0);
      else
        data.gravity = toEigen(gravityDir.GetNormalized()) * gravityMag;
    }
  }

  collectArticulations(stage, data);

  if (data.articulations.empty() && options.mVerbose) {
    DART_INFO(
        "No articulation roots were found in the USD stage, falling back to a\n"
        "single-skeleton import containing all rigid bodies.");
  }

  return data;
}

bool bodySetContains(
    const std::unordered_set<Path, PathHash>& paths, const Path& path)
{
  return paths.find(path) != paths.end();
}

dynamics::BodyNode* createRootBody(
    const BodyInfo& body,
    const Options& options,
    const dynamics::SkeletonPtr& skeleton)
{
  if (options.mDefaultRootJointType == RootJointType::FLOATING) {
    dynamics::FreeJoint::Properties jointProps;
    jointProps.mName = body.name + "_root_joint";
    jointProps.mT_ParentBodyToJoint = body.worldTransform;
    jointProps.mT_ChildBodyToJoint.setIdentity();

    auto pair = skeleton->createJointAndBodyNodePair<dynamics::FreeJoint>(
        nullptr, jointProps, body.properties);
    return pair.second;
  }

  dynamics::WeldJoint::Properties jointProps;
  jointProps.mName = body.name + "_root_joint";
  jointProps.mT_ParentBodyToJoint = body.worldTransform;
  jointProps.mT_ChildBodyToJoint.setIdentity();

  auto pair = skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>(
      nullptr, jointProps, body.properties);
  return pair.second;
}

template <typename JointT>
dynamics::BodyNode* createChildWithJoint(
    const JointInfo& joint,
    const BodyInfo& body,
    dynamics::BodyNode* parent,
    const dynamics::SkeletonPtr& skeleton)
{
  typename JointT::Properties props;
  props.mName = joint.name;
  props.mT_ParentBodyToJoint = joint.parentToJoint;
  props.mT_ChildBodyToJoint = joint.childToJoint;

  if constexpr (std::is_same_v<JointT, dynamics::RevoluteJoint>) {
    props.mAxis = joint.axis;
    props.mPositionLowerLimits[0] = joint.lowerLimit;
    props.mPositionUpperLimits[0] = joint.upperLimit;
  } else if constexpr (std::is_same_v<JointT, dynamics::PrismaticJoint>) {
    props.mAxis = joint.axis;
    props.mPositionLowerLimits[0] = joint.lowerLimit;
    props.mPositionUpperLimits[0] = joint.upperLimit;
  }

  auto pair = skeleton->createJointAndBodyNodePair<JointT>(
      parent, props, body.properties);
  return pair.second;
}

dynamics::BodyNode* attachBody(
    const JointInfo* joint,
    const BodyInfo& body,
    dynamics::BodyNode* parent,
    const dynamics::SkeletonPtr& skeleton,
    const Options& options)
{
  if (!joint)
    return createRootBody(body, options, skeleton);

  switch (joint->type) {
    case JointType::PRISMATIC:
      return createChildWithJoint<dynamics::PrismaticJoint>(
          *joint, body, parent, skeleton);
    case JointType::FIXED:
      return createChildWithJoint<dynamics::WeldJoint>(
          *joint, body, parent, skeleton);
    case JointType::REVOLUTE:
    default:
      return createChildWithJoint<dynamics::RevoluteJoint>(
          *joint, body, parent, skeleton);
  }
}

dynamics::SkeletonPtr buildSkeleton(
    const StageData& data,
    const ArticulationDefinition& articulation,
    const Options& options)
{
  const auto skeleton = dynamics::Skeleton::create(articulation.name);

  std::unordered_set<Path, PathHash> allowedBodies(
      articulation.bodyPaths.begin(), articulation.bodyPaths.end());

  std::unordered_map<Path, const JointInfo*, PathHash> childJointMap;
  std::unordered_set<Path, PathHash> articulatedChildren;
  for (const auto& jointPath : articulation.jointPaths) {
    const auto jointIt = data.joints.find(jointPath);
    if (jointIt == data.joints.end())
      continue;
    const auto& info = jointIt->second;
    if (!bodySetContains(allowedBodies, info.child))
      continue;

    if (!articulatedChildren.insert(info.child).second) {
      DART_WARN(
          "USD joint '{}' targets a child '{}' that is already connected."
          " Skipping duplicate joint.",
          info.name,
          info.child.GetString());
      continue;
    }

    childJointMap[info.child] = &info;
  }

  std::unordered_map<Path, dynamics::BodyNode*, PathHash> created;

  std::function<dynamics::BodyNode*(const Path&)> ensureBody
      = [&](const Path& bodyPath) -> dynamics::BodyNode* {
    if (auto it = created.find(bodyPath); it != created.end())
      return it->second;

    const auto bodyIt = data.bodies.find(bodyPath);
    if (bodyIt == data.bodies.end()) {
      DART_WARN(
          "Rigid body '{}' referenced by USD joint is missing.",
          bodyPath.GetString());
      return nullptr;
    }

    const auto* jointInfo
        = childJointMap.count(bodyPath) ? childJointMap.at(bodyPath) : nullptr;

    dynamics::BodyNode* parentNode = nullptr;
    if (jointInfo && bodySetContains(allowedBodies, jointInfo->parent))
      parentNode = ensureBody(jointInfo->parent);

    if (!parentNode && jointInfo && !jointInfo->parent.IsEmpty()) {
      DART_WARN(
          "USD joint '{}' references parent '{}' outside the articulation root."
          " Treating body as root.",
          jointInfo->name,
          jointInfo->parent.GetString());
    }

    dynamics::BodyNode* node
        = attachBody(jointInfo, bodyIt->second, parentNode, skeleton, options);
    created.emplace(bodyPath, node);
    return node;
  };

  for (const auto& bodyPath : articulation.bodyPaths)
    ensureBody(bodyPath);

  if (skeleton->getNumBodyNodes() == 0)
    return nullptr;

  return skeleton;
}

std::vector<dynamics::SkeletonPtr> importSkeletons(
    const StageData& data, const Options& options)
{
  std::vector<dynamics::SkeletonPtr> skeletons;
  for (const auto& articulation : data.articulations) {
    auto skeleton = buildSkeleton(data, articulation, options);
    if (skeleton)
      skeletons.push_back(skeleton);
  }

  if (skeletons.empty())
    DART_WARN(
        "USD parser did not create any skeletons from the requested stage.");

  return skeletons;
}

} // namespace

Options::Options(
    common::ResourceRetrieverPtr resourceRetriever,
    RootJointType defaultRootJointType,
    bool preserveFixedJoints,
    bool verbose)
  : mResourceRetriever(std::move(resourceRetriever)),
    mDefaultRootJointType(defaultRootJointType),
    mPreserveFixedJoints(preserveFixedJoints),
    mVerbose(verbose)
{
}

simulation::WorldPtr readWorld(const common::Uri& uri, const Options& options)
{
  const auto retriever = getRetriever(options.mResourceRetriever);
  auto stage = openStage(uri, retriever, options.mVerbose);
  if (!stage)
    return nullptr;

  auto data = buildStageData(stage, options);
  auto skeletons = importSkeletons(data, options);
  if (skeletons.empty())
    return nullptr;

  const auto worldName = stage->GetRootLayer()
                             ? stage->GetRootLayer()->GetDisplayName()
                             : std::string();
  auto world
      = simulation::World::create(worldName.empty() ? "usd_world" : worldName);
  world->setGravity(data.gravity);

  for (const auto& skeleton : skeletons)
    world->addSkeleton(skeleton);

  return world;
}

dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const Options& options)
{
  const auto retriever = getRetriever(options.mResourceRetriever);
  auto stage = openStage(uri, retriever, options.mVerbose);
  if (!stage)
    return nullptr;

  auto data = buildStageData(stage, options);
  auto skeletons = importSkeletons(data, options);
  if (skeletons.empty())
    return nullptr;

  return skeletons.front();
}

} // namespace UsdParser
} // namespace utils
} // namespace dart
