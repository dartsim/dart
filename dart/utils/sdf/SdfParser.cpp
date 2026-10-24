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

#include "dart/utils/sdf/SdfParser.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/Macros.hpp"
#include "dart/constraint/ConstraintSolver.hpp"
#include "dart/dynamics/BallJoint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/BoxShape.hpp"
#include "dart/dynamics/CylinderShape.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/MeshShape.hpp"
#include "dart/dynamics/MimicDofProperties.hpp"
#include "dart/dynamics/PlaneShape.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ScrewJoint.hpp"
#include "dart/dynamics/SoftBodyNode.hpp"
#include "dart/dynamics/SphereShape.hpp"
#include "dart/dynamics/UniversalJoint.hpp"
#include "dart/dynamics/WeldJoint.hpp"
#include "dart/utils/CompositeResourceRetriever.hpp"
#include "dart/utils/DartResourceRetriever.hpp"
#include "dart/utils/sdf/detail/SdfResources.hpp"

#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Element.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Physics.hh>
#include <sdf/Plane.hh>
#include <sdf/Root.hh>
#include <sdf/SemanticPose.hh>
#include <sdf/Sphere.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include <algorithm>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <cmath>

namespace dart {
namespace utils {
namespace SdfParser {

namespace {

//==============================================================================
void requireSdf(bool condition, const std::string& message)
{
  if (!condition)
    throw std::invalid_argument(message);
}

//==============================================================================
void checkSdfErrors(const sdf::Errors& errors, const std::string& context)
{
  for (const auto& error : errors) {
    if (error.Code() == sdf::ErrorCode::WARNING)
      dtwarn << "[SdfParser] " << context << ": " << error.Message() << '\n';
    else
      throw std::invalid_argument(context + ": " + error.Message());
  }
}

//==============================================================================
Eigen::Vector3d sdfVector(const gz::math::Vector3d& vector)
{
  return Eigen::Vector3d(vector.X(), vector.Y(), vector.Z());
}

//==============================================================================
Eigen::Isometry3d sdfTransform(const gz::math::Pose3d& pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = sdfVector(pose.Pos());
  const auto& rotation = pose.Rot();
  Eigen::Quaterniond quaternion(
      rotation.W(), rotation.X(), rotation.Y(), rotation.Z());
  const double scale = quaternion.coeffs().cwiseAbs().maxCoeff();
  requireSdf(
      transform.translation().allFinite() && quaternion.coeffs().allFinite()
          && scale > 0.0,
      "Pose must contain a finite position and a finite nonzero quaternion");
  quaternion.coeffs() /= scale;
  transform.linear() = quaternion.normalized().toRotationMatrix();
  return transform;
}

//==============================================================================
Eigen::Isometry3d resolveSdfPose(
    const sdf::SemanticPose& pose, const std::string& frame = "")
{
  gz::math::Pose3d resolved;
  checkSdfErrors(pose.Resolve(resolved, frame), "Resolving pose");
  return sdfTransform(resolved);
}

//==============================================================================
double positiveSdf(double value, const std::string& name)
{
  requireSdf(std::isfinite(value) && value > 0.0, name + " must be positive");
  return value;
}

//==============================================================================
Eigen::Vector3d positiveSdfVector(
    const gz::math::Vector3d& value, const std::string& name)
{
  const Eigen::Vector3d vector = sdfVector(value);
  requireSdf(
      vector.allFinite() && (vector.array() > 0.0).all(),
      name + " must contain positive finite values");
  return vector;
}

//==============================================================================
template <typename T>
T readSdfExtension(const sdf::ElementPtr& element, const std::string& name)
{
  const auto child = element ? element->FindElement(name) : nullptr;
  requireSdf(child != nullptr, "Missing DART extension element <" + name + ">");
  sdf::Errors errors;
  const T value = child->Get<T>(errors);
  checkSdfErrors(errors, "Reading DART extension <" + name + ">");
  return value;
}

//==============================================================================
std::size_t readSdfCount(
    const sdf::ElementPtr& element,
    const std::string& name,
    std::size_t minimum)
{
  const double value = readSdfExtension<double>(element, name);
  requireSdf(
      std::isfinite(value) && value >= minimum && std::floor(value) == value
          && value <= std::numeric_limits<int>::max(),
      "DART extension <" + name + "> has an invalid subdivision count");
  return static_cast<std::size_t>(value);
}

//==============================================================================
dynamics::SoftBodyNode::UniqueProperties readSdfSoftBody(const sdf::Link& link)
{
  const auto soft = link.Element()->FindElement("soft_shape");
  const double mass = positiveSdf(
      readSdfExtension<double>(soft, "total_mass"), "Soft body mass");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  if (soft->FindElement("pose")) {
    pose = sdfTransform(readSdfExtension<gz::math::Pose3d>(soft, "pose"));
    const auto poseElement = soft->FindElement("pose");
    const auto relativeTo = poseElement->GetAttribute("relative_to");
    if (relativeTo) {
      const std::string frame = relativeTo->GetAsString();
      if (!frame.empty() && frame != link.Name())
        pose = resolveSdfPose(link.SemanticPose(), frame).inverse() * pose;
    }
    requireSdf(pose.matrix().allFinite(), "Resolved soft shape pose overflow");
  }

  const auto geometry = soft->FindElement("geometry");
  requireSdf(geometry != nullptr, "Soft shape is missing <geometry>");
  dynamics::SoftBodyNode::UniqueProperties properties;
  if (const auto sphere = geometry->FindElement("sphere")) {
    const auto slices = readSdfCount(sphere, "num_slices", 3);
    const auto stacks = readSdfCount(sphere, "num_stacks", 2);
    requireSdf(
        static_cast<long double>(slices) * (stacks - 1) + 2
            <= std::numeric_limits<int>::max(),
        "Soft sphere point count exceeds DART's index range");
    properties = dynamics::SoftBodyNodeHelper::makeSphereProperties(
        positiveSdf(readSdfExtension<double>(sphere, "radius"), "Soft radius"),
        slices,
        stacks,
        mass);
  } else if (const auto box = geometry->FindElement("box")) {
    const auto fragments
        = sdfVector(readSdfExtension<gz::math::Vector3d>(box, "frags"));
    requireSdf(
        fragments.allFinite() && (fragments.array() >= 2.0).all()
            && (fragments.array() <= std::numeric_limits<int>::max()).all()
            && (fragments.array().floor() == fragments.array()).all(),
        "Soft box subdivisions must be integers of at least two");
    const Eigen::Vector3d edges = fragments.cwiseMax(3.0).array() - 2.0;
    requireSdf(
        8.0 + 4.0 * edges.sum()
                + 2.0
                      * (edges[0] * edges[1] + edges[1] * edges[2]
                         + edges[0] * edges[2])
            <= std::numeric_limits<int>::max(),
        "Soft box point count exceeds DART's index range");
    properties = dynamics::SoftBodyNodeHelper::makeBoxProperties(
        positiveSdfVector(
            readSdfExtension<gz::math::Vector3d>(box, "size"), "Soft box size"),
        pose,
        fragments.cast<int>(),
        mass);
  } else if (const auto ellipsoid = geometry->FindElement("ellipsoid")) {
    const auto slices = readSdfCount(ellipsoid, "num_slices", 3);
    const auto stacks = readSdfCount(ellipsoid, "num_stacks", 2);
    requireSdf(
        static_cast<long double>(slices) * (stacks - 1) + 2
            <= std::numeric_limits<int>::max(),
        "Soft ellipsoid point count exceeds DART's index range");
    properties = dynamics::SoftBodyNodeHelper::makeEllipsoidProperties(
        positiveSdfVector(
            readSdfExtension<gz::math::Vector3d>(ellipsoid, "size"),
            "Soft ellipsoid size"),
        slices,
        stacks,
        mass);
  } else if (const auto cylinder = geometry->FindElement("cylinder")) {
    const auto slices = readSdfCount(cylinder, "num_slices", 3);
    const auto stacks = readSdfCount(cylinder, "num_stacks", 1);
    const auto rings = readSdfCount(cylinder, "num_rings", 1);
    requireSdf(
        static_cast<long double>(slices) * (stacks + 1)
                + 2 * (static_cast<long double>(slices) * (rings - 1) + 1)
            <= std::numeric_limits<int>::max(),
        "Soft cylinder point count exceeds DART's index range");
    properties = dynamics::SoftBodyNodeHelper::makeCylinderProperties(
        positiveSdf(
            readSdfExtension<double>(cylinder, "radius"), "Soft radius"),
        positiveSdf(
            readSdfExtension<double>(cylinder, "height"), "Soft height"),
        slices,
        stacks,
        rings,
        mass);
  } else {
    throw std::invalid_argument("Unsupported DART soft geometry");
  }

  // The box helper already applies the pose to its rest points.
  if (!geometry->FindElement("box")) {
    for (auto& point : properties.mPointProps)
      point.mX0 = pose * point.mX0;
  }
  for (const auto& point : properties.mPointProps)
    requireSdf(point.mX0.allFinite(), "Soft geometry rest point overflow");
  for (const auto& entry : std::vector<std::pair<std::string, double*>>{
           {"kv", &properties.mKv},
           {"ke", &properties.mKe},
           {"damp", &properties.mDampCoeff}}) {
    if (soft->FindElement(entry.first)) {
      const double value = readSdfExtension<double>(soft, entry.first);
      requireSdf(
          std::isfinite(value) && value >= 0.0,
          "Soft <" + entry.first + "> must be nonnegative and finite");
      *entry.second = value;
    }
  }
  return properties;
}

struct SdfBody
{
  std::shared_ptr<dynamics::BodyNode::Properties> properties;
  Eigen::Isometry3d worldTransform;
  bool soft = false;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
using SdfBodies = common::aligned_map<std::string, SdfBody>;

struct SdfJoint
{
  std::shared_ptr<dynamics::Joint::Properties> properties;
  std::string parent;
  std::string child;
  sdf::JointType type;
  struct Mimic
  {
    std::string reference;
    std::size_t followerAxis;
    std::size_t referenceAxis;
    double multiplier;
    double offset;
  };
  std::vector<Mimic> mimics;
};
using SdfJoints = std::map<std::string, SdfJoint>;

//==============================================================================
SdfBody readSdfBody(
    const sdf::Link& link, const Eigen::Isometry3d& modelTransform)
{
  dynamics::BodyNode::Properties properties;
  properties.mName = link.Name();
  properties.mGravityMode = link.EnableGravity();
  gz::math::Inertiald inertial;
  checkSdfErrors(
      link.ResolveInertial(inertial, link.Name()), "Resolving link inertia");
  // DART has no fluid added-mass dynamics; dropping an authored tensor
  // would silently change the link's acceleration response.
  if (const auto& fluidAddedMass = inertial.FluidAddedMass();
      fluidAddedMass.has_value()) {
    bool zero = true;
    for (std::size_t row = 0; row < 6 && zero; ++row)
      for (std::size_t column = 0; column < 6; ++column)
        zero = zero && (*fluidAddedMass)(row, column) == 0.0;
    requireSdf(
        zero,
        "Link [" + link.Name()
            + "] fluid added mass is not supported by DART's SDF parser");
  }
  const auto& matrix = inertial.MassMatrix();
  const auto mass = positiveSdf(matrix.Mass(), "Link mass");
  properties.mInertia.setMass(mass);
  const auto pose = sdfTransform(inertial.Pose());
  properties.mInertia.setLocalCOM(pose.translation());
  const sdf::ElementPtr inertialElement
      = link.Element() ? link.Element()->FindElement("inertial") : nullptr;
  const bool authoredInertia
      = inertialElement && inertialElement->FindElement("inertia");
  Eigen::Matrix3d moment;
  if (authoredInertia) {
    const auto moi = matrix.Moi();
    for (int row = 0; row < 3; ++row)
      for (int column = 0; column < 3; ++column)
        moment(row, column) = moi(row, column);
    requireSdf(moment.allFinite(), "Link inertia must be finite");
    moment = pose.linear() * moment * pose.linear().transpose();
  } else {
    // sdformat resolves an omitted <inertia> to its unit-diagonal default;
    // the legacy DART parser used mass * I, so keep that fallback instead of
    // silently changing the angular response of mass-only links.
    moment = mass * Eigen::Matrix3d::Identity();
  }
  requireSdf(moment.allFinite(), "Resolved link inertia must be finite");
  properties.mInertia.setMoment(moment);

  SdfBody body;
  body.worldTransform
      = modelTransform * resolveSdfPose(link.SemanticPose(), "__model__");
  requireSdf(body.worldTransform.matrix().allFinite(), "Link pose overflow");
  body.soft = link.Element() && link.Element()->FindElement("soft_shape");
  if (body.soft) {
    body.properties = dynamics::SoftBodyNode::Properties::createShared(
        properties, readSdfSoftBody(link));
  } else {
    body.properties = dynamics::BodyNode::Properties::createShared(properties);
  }
  return body;
}

//==============================================================================
template <typename Properties>
void readSdfAxis(
    const sdf::Joint& joint,
    unsigned int index,
    Eigen::Vector3d& direction,
    Properties& properties)
{
  const auto axis = joint.Axis(index);
  requireSdf(
      axis != nullptr, "Joint [" + joint.Name() + "] is missing an axis");
  gz::math::Vector3d resolved;
  checkSdfErrors(axis->ResolveXyz(resolved), "Resolving joint axis");
  direction = sdfVector(resolved);
  const double scale = direction.cwiseAbs().maxCoeff();
  requireSdf(direction.allFinite() && scale > 0.0, "Invalid joint axis");

  const double lower = axis->Lower();
  const double upper = axis->Upper();
  requireSdf(
      !std::isnan(lower) && !std::isnan(upper) && lower <= upper
          && lower != std::numeric_limits<double>::infinity()
          && upper != -std::numeric_limits<double>::infinity(),
      "Joint limits must be ordered and admit a finite position");
  properties.mPositionLowerLimits[index] = lower;
  properties.mPositionUpperLimits[index] = upper;
  properties.mIsPositionLimitEnforced = properties.mIsPositionLimitEnforced
                                        || std::isfinite(lower)
                                        || std::isfinite(upper);
  double initial = 0.0;
  if (lower > 0.0 || upper < 0.0) {
    if (std::isfinite(lower) && std::isfinite(upper))
      initial = lower / 2.0 + upper / 2.0;
    else
      initial = std::isfinite(lower) ? lower : upper;
  }
  properties.mInitialPositions[index] = initial;
  properties.mRestPositions[index] = axis->SpringReference();
  properties.mDampingCoefficients[index] = axis->Damping();
  properties.mFrictions[index] = axis->Friction();
  properties.mSpringStiffnesses[index] = axis->SpringStiffness();
  requireSdf(
      std::isfinite(axis->SpringReference()) && std::isfinite(axis->Damping())
          && axis->Damping() >= 0.0 && std::isfinite(axis->Friction())
          && axis->Friction() >= 0.0 && std::isfinite(axis->SpringStiffness())
          && axis->SpringStiffness() >= 0.0,
      "Joint dynamics must be finite with nonnegative damping, friction and "
      "stiffness");
  for (const auto& limit : std::vector<std::pair<double, bool>>{
           {axis->MaxVelocity(), true}, {axis->Effort(), false}}) {
    requireSdf(
        !std::isnan(limit.first)
            && limit.first != -std::numeric_limits<double>::infinity(),
        "Joint velocity and effort limits must be valid numbers");
    if (limit.first >= 0.0) {
      if (limit.second) {
        properties.mVelocityLowerLimits[index] = -limit.first;
        properties.mVelocityUpperLimits[index] = limit.first;
      } else {
        properties.mForceLowerLimits[index] = -limit.first;
        properties.mForceUpperLimits[index] = limit.first;
      }
    }
  }
}

//==============================================================================
SdfJoint readSdfJoint(
    const sdf::Joint& joint,
    const SdfBodies& bodies,
    const Eigen::Isometry3d& modelTransform)
{
  SdfJoint result;
  result.type = joint.Type();
  checkSdfErrors(
      joint.ResolveParentLink(result.parent), "Resolving joint parent");
  checkSdfErrors(joint.ResolveChildLink(result.child), "Resolving joint child");
  requireSdf(
      result.parent == "world" || bodies.count(result.parent),
      "Joint [" + joint.Name() + "] has a missing parent link");
  requireSdf(
      bodies.count(result.child) && result.parent != result.child,
      "Joint [" + joint.Name() + "] has an invalid child link");
  switch (joint.Type()) {
    case sdf::JointType::FIXED:
      result.properties = dynamics::WeldJoint::Properties::createShared();
      break;
    case sdf::JointType::BALL:
      result.properties = dynamics::BallJoint::Properties::createShared();
      break;
    case sdf::JointType::REVOLUTE: {
      dynamics::RevoluteJoint::Properties properties;
      readSdfAxis(joint, 0, properties.mAxis, properties);
      result.properties
          = dynamics::RevoluteJoint::Properties::createShared(properties);
      break;
    }
    case sdf::JointType::PRISMATIC: {
      dynamics::PrismaticJoint::Properties properties;
      readSdfAxis(joint, 0, properties.mAxis, properties);
      result.properties
          = dynamics::PrismaticJoint::Properties::createShared(properties);
      break;
    }
    case sdf::JointType::SCREW: {
      dynamics::ScrewJoint::Properties properties;
      readSdfAxis(joint, 0, properties.mAxis, properties);
      properties.mPitch = joint.ScrewThreadPitch();
      requireSdf(
          std::isfinite(properties.mPitch), "Invalid screw thread pitch");
      result.properties
          = dynamics::ScrewJoint::Properties::createShared(properties);
      break;
    }
    case sdf::JointType::REVOLUTE2:
    case sdf::JointType::UNIVERSAL: {
      dynamics::UniversalJoint::Properties properties;
      readSdfAxis(joint, 0, properties.mAxis[0], properties);
      readSdfAxis(joint, 1, properties.mAxis[1], properties);
      requireSdf(
          properties.mAxis[0].cross(properties.mAxis[1]).norm() > 1e-12,
          "Universal joint axes must not be parallel");
      result.properties
          = dynamics::UniversalJoint::Properties::createShared(properties);
      break;
    }
    default:
      throw std::invalid_argument(
          "Unsupported joint type in [" + joint.Name() + "]");
  }
  result.properties->mName = joint.Name();
  const Eigen::Isometry3d worldJoint
      = modelTransform * resolveSdfPose(joint.SemanticPose(), "__model__");
  const Eigen::Isometry3d parentWorld
      = result.parent == "world" ? Eigen::Isometry3d::Identity()
                                 : bodies.at(result.parent).worldTransform;
  result.properties->mT_ParentBodyToJoint = parentWorld.inverse() * worldJoint;
  result.properties->mT_ChildBodyToJoint
      = bodies.at(result.child).worldTransform.inverse() * worldJoint;
  requireSdf(
      result.properties->mT_ParentBodyToJoint.matrix().allFinite()
          && result.properties->mT_ChildBodyToJoint.matrix().allFinite(),
      "Resolved joint pose overflow");
  for (unsigned int index = 0; index < 2; ++index) {
    const auto axis = joint.Axis(index);
    const auto mimic = axis ? axis->Mimic() : std::nullopt;
    if (!mimic)
      continue;
    requireSdf(
        mimic->Axis() == "axis" || mimic->Axis() == "axis2",
        "Mimic reference axis must be axis or axis2");
    const double offset
        = mimic->Offset() - mimic->Multiplier() * mimic->Reference();
    requireSdf(
        std::isfinite(mimic->Multiplier()) && std::isfinite(mimic->Offset())
            && std::isfinite(mimic->Reference()) && std::isfinite(offset),
        "Mimic parameters must be finite");
    result.mimics.push_back(
        {mimic->Joint(),
         index,
         mimic->Axis() == "axis2" ? 1u : 0u,
         mimic->Multiplier(),
         offset});
  }
  return result;
}

//==============================================================================
template <class NodeType>
std::pair<dynamics::Joint*, dynamics::BodyNode*> createSdfPair(
    const dynamics::SkeletonPtr& skeleton,
    dynamics::BodyNode* parent,
    const SdfJoint& joint,
    const SdfBody& body)
{
  const auto& properties
      = static_cast<const typename NodeType::Properties&>(*body.properties);
  switch (joint.type) {
    case sdf::JointType::PRISMATIC:
      return skeleton
          ->createJointAndBodyNodePair<dynamics::PrismaticJoint, NodeType>(
              parent,
              static_cast<const dynamics::PrismaticJoint::Properties&>(
                  *joint.properties),
              properties);
    case sdf::JointType::REVOLUTE:
      return skeleton
          ->createJointAndBodyNodePair<dynamics::RevoluteJoint, NodeType>(
              parent,
              static_cast<const dynamics::RevoluteJoint::Properties&>(
                  *joint.properties),
              properties);
    case sdf::JointType::SCREW:
      return skeleton
          ->createJointAndBodyNodePair<dynamics::ScrewJoint, NodeType>(
              parent,
              static_cast<const dynamics::ScrewJoint::Properties&>(
                  *joint.properties),
              properties);
    case sdf::JointType::REVOLUTE2:
    case sdf::JointType::UNIVERSAL:
      return skeleton
          ->createJointAndBodyNodePair<dynamics::UniversalJoint, NodeType>(
              parent,
              static_cast<const dynamics::UniversalJoint::Properties&>(
                  *joint.properties),
              properties);
    case sdf::JointType::BALL:
      return skeleton
          ->createJointAndBodyNodePair<dynamics::BallJoint, NodeType>(
              parent,
              static_cast<const dynamics::BallJoint::Properties&>(
                  *joint.properties),
              properties);
    case sdf::JointType::FIXED:
      return skeleton
          ->createJointAndBodyNodePair<dynamics::WeldJoint, NodeType>(
              parent,
              static_cast<const dynamics::WeldJoint::Properties&>(
                  *joint.properties),
              properties);
    default:
      return skeleton
          ->createJointAndBodyNodePair<dynamics::FreeJoint, NodeType>(
              parent,
              static_cast<const dynamics::FreeJoint::Properties&>(
                  *joint.properties),
              properties);
  }
}

//==============================================================================
dynamics::ShapePtr readSdfShape(
    const sdf::Geometry* geometry,
    const Options& options,
    const detail::SdfResources& resources)
{
  requireSdf(geometry != nullptr, "Missing shape geometry");
  switch (geometry->Type()) {
    case sdf::GeometryType::SPHERE:
      return std::make_shared<dynamics::SphereShape>(
          positiveSdf(geometry->SphereShape()->Radius(), "Sphere radius"));
    case sdf::GeometryType::BOX:
      return std::make_shared<dynamics::BoxShape>(
          positiveSdfVector(geometry->BoxShape()->Size(), "Box size"));
    case sdf::GeometryType::CYLINDER:
      return std::make_shared<dynamics::CylinderShape>(
          positiveSdf(geometry->CylinderShape()->Radius(), "Cylinder radius"),
          positiveSdf(geometry->CylinderShape()->Length(), "Cylinder length"));
    case sdf::GeometryType::PLANE: {
      const auto plane = geometry->PlaneShape();
      Eigen::Vector3d normal = sdfVector(plane->Normal());
      const double scale = normal.cwiseAbs().maxCoeff();
      requireSdf(normal.allFinite() && scale > 0.0, "Invalid plane normal");
      normal /= scale;
      normal.normalize();
      if (options.mUsePlaneShapeForPlane)
        return std::make_shared<dynamics::PlaneShape>(normal, 0.0);
      return std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(
          positiveSdf(plane->Size().X(), "Plane size"),
          positiveSdf(plane->Size().Y(), "Plane size"),
          0.001));
    }
    case sdf::GeometryType::MESH: {
      const auto mesh = geometry->MeshShape();
      requireSdf(!mesh->Uri().empty(), "Mesh is missing a URI");
      requireSdf(
          mesh->Submesh().empty(), "SDF mesh submesh selection is unsupported");
      const Eigen::Vector3d scale = sdfVector(mesh->Scale());
      requireSdf(
          scale.allFinite() && (scale.array() != 0.0).all(),
          "Mesh scale must contain finite, nonzero values");
      const auto uri = resources.resolveUri(mesh->Element(), mesh->Uri());
      DART_SUPPRESS_DEPRECATED_BEGIN
      const aiScene* scene
          = dynamics::MeshShape::loadMesh(uri, options.mResourceRetriever);
      DART_SUPPRESS_DEPRECATED_END
      requireSdf(
          scene != nullptr, "Failed to load mesh [" + uri.toString() + "]");
      DART_SUPPRESS_DEPRECATED_BEGIN
      auto shape = std::shared_ptr<dynamics::MeshShape>(new dynamics::MeshShape(
          scale, scene, uri, options.mResourceRetriever));
      DART_SUPPRESS_DEPRECATED_END
      return shape;
    }
    default:
      throw std::invalid_argument("Unsupported SDF geometry");
  }
}

//==============================================================================
template <typename Shape>
dynamics::ShapeNode* readSdfShapeNode(
    dynamics::BodyNode* body,
    const Shape& shape,
    const Options& options,
    const detail::SdfResources& resources)
{
  auto geometry = readSdfShape(shape.Geom(), options, resources);
  auto transform = resolveSdfPose(shape.SemanticPose(), body->getName());
  if (shape.Geom()->Type() == sdf::GeometryType::PLANE
      && !options.mUsePlaneShapeForPlane) {
    const auto normal
        = sdfVector(shape.Geom()->PlaneShape()->Normal()).normalized();
    // Orient the finite plane approximation along the authored plane normal.
    transform.linear()
        *= Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal)
               .toRotationMatrix();
  }
  auto* node
      = body->createShapeNode(geometry, body->getName() + " - " + shape.Name());
  node->setRelativeTransform(transform);
  return node;
}

//==============================================================================
void readSdfAspects(
    const sdf::Model& model,
    const dynamics::SkeletonPtr& skeleton,
    const Options& options,
    const detail::SdfResources& resources)
{
  for (uint64_t index = 0; index < model.LinkCount(); ++index) {
    const auto link = model.LinkByIndex(index);
    auto* body = skeleton->getBodyNode(link->Name());
    for (uint64_t visualIndex = 0; visualIndex < link->VisualCount();
         ++visualIndex) {
      const auto visual = link->VisualByIndex(visualIndex);
      auto* node = readSdfShapeNode(body, *visual, options, resources);
      auto* aspect = node->createVisualAspect();
      if (visual->Material()) {
        const auto color = visual->Material()->Diffuse();
        const Eigen::Vector4d rgba(color.R(), color.G(), color.B(), color.A());
        requireSdf(rgba.allFinite(), "Material color must be finite");
        aspect->setColor(rgba);
      }
    }
    for (uint64_t collisionIndex = 0; collisionIndex < link->CollisionCount();
         ++collisionIndex) {
      const auto collision = link->CollisionByIndex(collisionIndex);
      auto* node = readSdfShapeNode(body, *collision, options, resources);
      node->createCollisionAspect();
      node->createDynamicsAspect();
    }
  }
}

//==============================================================================
void applySdfMimics(
    const dynamics::SkeletonPtr& skeleton, const SdfJoints& joints)
{
  using Dof = std::pair<std::string, std::size_t>;
  std::map<Dof, Dof> references;
  for (const auto& entry : joints) {
    auto* joint = skeleton->getJoint(entry.second.properties->mName);
    auto properties = joint->getMimicDofProperties();
    properties.resize(joint->getNumDofs());
    for (const auto& mimic : entry.second.mimics) {
      auto* reference = skeleton->getJoint(mimic.reference);
      requireSdf(
          reference && mimic.referenceAxis < reference->getNumDofs()
              && mimic.followerAxis < joint->getNumDofs(),
          "Mimic references a missing joint or axis");
      auto& prop = properties[mimic.followerAxis];
      prop.mReferenceJoint = reference;
      prop.mReferenceDofIndex = mimic.referenceAxis;
      prop.mMultiplier = mimic.multiplier;
      prop.mOffset = mimic.offset;
      references[{joint->getName(), mimic.followerAxis}]
          = {reference->getName(), mimic.referenceAxis};
    }
    if (!entry.second.mimics.empty()) {
      joint->setMimicJointDofs(properties);
      if (entry.second.mimics.size() == joint->getNumDofs())
        joint->setActuatorType(dynamics::Joint::MIMIC);
      else
        for (const auto& mimic : entry.second.mimics)
          joint->setActuatorType(mimic.followerAxis, dynamics::Joint::MIMIC);
    }
  }
  for (const auto& entry : references) {
    std::set<Dof> visiting;
    Dof current = entry.first;
    while (references.count(current)) {
      requireSdf(
          visiting.insert(current).second, "Mimic references form a cycle");
      current = references.at(current);
    }
  }
}

//==============================================================================
dynamics::SkeletonPtr readSdfModel(
    const sdf::Model& model,
    const Options& options,
    const detail::SdfResources& resources)
{
  requireSdf(model.ModelCount() == 0, "Nested SDF models are unsupported");
  const auto modelTransform = resolveSdfPose(model.SemanticPose());
  SdfBodies bodies;
  for (uint64_t index = 0; index < model.LinkCount(); ++index) {
    const auto link = model.LinkByIndex(index);
    requireSdf(
        bodies.emplace(link->Name(), readSdfBody(*link, modelTransform)).second,
        "Duplicate link name [" + link->Name() + "]");
  }
  SdfJoints joints;
  std::set<std::string> jointNames;
  for (uint64_t index = 0; index < model.JointCount(); ++index) {
    auto joint
        = readSdfJoint(*model.JointByIndex(index), bodies, modelTransform);
    jointNames.insert(joint.properties->mName);
    const auto child = joint.child;
    requireSdf(
        joints.emplace(child, std::move(joint)).second,
        "Multiple joints claim child link [" + child + "]");
  }

  // Order links by ancestry before constructing any DART joints.
  std::vector<std::string> order;
  std::set<std::string> ordered;
  for (const auto& entry : bodies) {
    std::vector<std::string> ancestors;
    std::set<std::string> visiting;
    std::string current = entry.first;
    while (current != "world" && !ordered.count(current)) {
      requireSdf(
          visiting.insert(current).second, "Link topology forms a cycle");
      ancestors.push_back(current);
      const auto joint = joints.find(current);
      if (joint == joints.end())
        break;
      current = joint->second.parent;
    }
    for (auto ancestor = ancestors.rbegin(); ancestor != ancestors.rend();
         ++ancestor) {
      ordered.insert(*ancestor);
      order.push_back(*ancestor);
    }
  }

  auto skeleton = dynamics::Skeleton::create(model.Name());
  skeleton->setMobile(!model.Static());
  for (const auto& name : order) {
    const auto& body = bodies.at(name);
    SdfJoint root;
    const SdfJoint* joint = nullptr;
    dynamics::BodyNode* parent = nullptr;
    if (const auto found = joints.find(name); found != joints.end()) {
      joint = &found->second;
      if (joint->parent != "world")
        parent = skeleton->getBodyNode(joint->parent);
    } else {
      root.type = options.mDefaultRootJointType == RootJointType::FIXED
                      ? sdf::JointType::FIXED
                      : sdf::JointType::INVALID;
      std::string rootName = "root";
      for (std::size_t suffix = 1; jointNames.count(rootName); ++suffix)
        rootName = "root(" + std::to_string(suffix) + ")";
      jointNames.insert(rootName);
      const dynamics::Joint::Properties properties(
          rootName, body.worldTransform);
      if (root.type == sdf::JointType::FIXED)
        root.properties
            = dynamics::WeldJoint::Properties::createShared(properties);
      else
        root.properties
            = dynamics::FreeJoint::Properties::createShared(properties);
      joint = &root;
    }
    const auto pair = body.soft ? createSdfPair<dynamics::SoftBodyNode>(
                          skeleton, parent, *joint, body)
                                : createSdfPair<dynamics::BodyNode>(
                                    skeleton, parent, *joint, body);
    requireSdf(pair.first && pair.second, "Failed to create a joint and body");
  }
  readSdfAspects(model, skeleton, options, resources);
  skeleton->resetPositions();
  applySdfMimics(skeleton, joints);
  return skeleton;
}

//==============================================================================
Options resolveSdfOptions(const Options& options)
{
  Options resolved = options;
  if (!resolved.mResourceRetriever) {
    auto retriever = std::make_shared<CompositeResourceRetriever>();
    retriever->addSchemaRetriever(
        "file", std::make_shared<common::LocalResourceRetriever>());
    retriever->addSchemaRetriever("dart", DartResourceRetriever::create());
    resolved.mResourceRetriever = retriever;
  }
  return resolved;
}

} // namespace

//==============================================================================
Options::Options(
    common::ResourceRetrieverPtr resourceRetriever,
    RootJointType defaultRootJointType,
    collision::CollisionDetectorPtr collisionDetector,
    bool usePlaneShapeForPlane)
  : mResourceRetriever(std::move(resourceRetriever)),
    mDefaultRootJointType(defaultRootJointType),
    mCollisionDetector(std::move(collisionDetector)),
    mUsePlaneShapeForPlane(usePlaneShapeForPlane)
{
}

//==============================================================================
simulation::WorldPtr readSdfFile(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
{
  return readWorld(uri, Options(retriever));
}

//==============================================================================
simulation::WorldPtr readWorld(const common::Uri& uri, const Options& options)
{
  try {
    const bool useNativeFileLookup = !options.mResourceRetriever;
    const auto resolved = resolveSdfOptions(options);
    detail::SdfResources resources(
        resolved.mResourceRetriever, useNativeFileLookup);
    sdf::Root root;
    if (!resources.load(uri, root))
      return nullptr;
    requireSdf(root.WorldCount() > 0, "Expected an SDF world");
    if (root.WorldCount() > 1)
      dtwarn << "[SdfParser] Multiple worlds found; loading the first world\n";
    const auto sdfWorld = root.WorldByIndex(0);
    requireSdf(
        sdfWorld->JointCount() == 0, "World-level SDF joints are unsupported");
    const auto gravity = sdfVector(sdfWorld->Gravity());
    requireSdf(gravity.allFinite(), "World gravity must be finite");
    const auto physics = sdfWorld->PhysicsDefault();
    const double timeStep
        = physics ? positiveSdf(physics->MaxStepSize(), "Time step") : 0.001;
    std::vector<dynamics::SkeletonPtr> models;
    for (uint64_t index = 0; index < sdfWorld->ModelCount(); ++index)
      models.push_back(
          readSdfModel(*sdfWorld->ModelByIndex(index), resolved, resources));
    auto world = simulation::World::create(sdfWorld->Name());
    world->setGravity(gravity);
    world->setTimeStep(timeStep);
    if (resolved.mCollisionDetector)
      world->getConstraintSolver()->setCollisionDetector(
          resolved.mCollisionDetector);
    for (const auto& model : models)
      world->addSkeleton(model);
    return world;
  } catch (const std::exception& error) {
    dtwarn << "[SdfParser] Loading world [" << uri.toString()
           << "] failed: " << error.what() << '\n';
    return nullptr;
  }
}

//==============================================================================
simulation::WorldPtr readWorld(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
{
  return readWorld(uri, Options(retriever));
}

//==============================================================================
dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const Options& options)
{
  try {
    const bool useNativeFileLookup = !options.mResourceRetriever;
    const auto resolved = resolveSdfOptions(options);
    detail::SdfResources resources(
        resolved.mResourceRetriever, useNativeFileLookup);
    sdf::Root root;
    if (!resources.load(uri, root))
      return nullptr;
    requireSdf(root.Model() != nullptr, "Expected a standalone SDF model");
    return readSdfModel(*root.Model(), resolved, resources);
  } catch (const std::exception& error) {
    dtwarn << "[SdfParser] Loading model [" << uri.toString()
           << "] failed: " << error.what() << '\n';
    return nullptr;
  }
}

//==============================================================================
dynamics::SkeletonPtr readSkeleton(
    const common::Uri& uri, const common::ResourceRetrieverPtr& retriever)
{
  return readSkeleton(uri, Options(retriever));
}

} // namespace SdfParser
} // namespace utils
} // namespace dart
