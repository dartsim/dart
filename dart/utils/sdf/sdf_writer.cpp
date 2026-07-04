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

#include "dart/utils/sdf/sdf_writer.hpp"

#include "dart/math/geometry.hpp"

#include <dart/dynamics/body_node.hpp>
#include <dart/dynamics/box_shape.hpp>
#include <dart/dynamics/cylinder_shape.hpp>
#include <dart/dynamics/free_joint.hpp>
#include <dart/dynamics/joint.hpp>
#include <dart/dynamics/mesh_shape.hpp>
#include <dart/dynamics/prismatic_joint.hpp>
#include <dart/dynamics/revolute_joint.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/sphere_shape.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <algorithm>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <string_view>

#include <cmath>

namespace dart {
namespace utils {
namespace SdfParser {

namespace {

using WriteError = common::Error;
using WriteResult = common::Result<void, WriteError>;
using StringResult = common::Result<std::string, WriteError>;

std::string indent(int depth)
{
  return std::string(static_cast<std::size_t>(depth) * 2, ' ');
}

std::string escapeXml(std::string_view value)
{
  std::string escaped;
  escaped.reserve(value.size());
  for (const char c : value) {
    switch (c) {
      case '&':
        escaped += "&amp;";
        break;
      case '<':
        escaped += "&lt;";
        break;
      case '>':
        escaped += "&gt;";
        break;
      case '"':
        escaped += "&quot;";
        break;
      case '\'':
        escaped += "&apos;";
        break;
      default:
        escaped += c;
        break;
    }
  }
  return escaped;
}

std::string formatDouble(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

std::string formatVector3(const Eigen::Vector3d& value)
{
  return formatDouble(value.x()) + " " + formatDouble(value.y()) + " "
         + formatDouble(value.z());
}

std::string formatVector4(const Eigen::Vector4d& value)
{
  return formatDouble(value.x()) + " " + formatDouble(value.y()) + " "
         + formatDouble(value.z()) + " " + formatDouble(value.w());
}

bool isFinite(const Eigen::Isometry3d& transform)
{
  return transform.matrix().allFinite();
}

bool isFinite(const Eigen::Matrix3d& matrix)
{
  return matrix.allFinite();
}

bool isFinite(const Eigen::Vector3d& vector)
{
  return vector.allFinite();
}

bool isFinite(const Eigen::Vector4d& vector)
{
  return vector.allFinite();
}

std::string formatPose(const Eigen::Isometry3d& transform)
{
  const Eigen::Vector3d zyx = math::matrixToEulerZYX(transform.linear());
  return formatVector3(transform.translation()) + " " + formatDouble(zyx.z())
         + " " + formatDouble(zyx.y()) + " " + formatDouble(zyx.x());
}

WriteResult fail(std::string message)
{
  return WriteResult::err(WriteError(std::move(message)));
}

WriteResult ok()
{
  return WriteResult::ok();
}

std::string jointTypeName(const dynamics::Joint& joint)
{
  if (dynamic_cast<const dynamics::WeldJoint*>(&joint)) {
    return "fixed";
  }
  if (dynamic_cast<const dynamics::RevoluteJoint*>(&joint)) {
    return "revolute";
  }
  if (dynamic_cast<const dynamics::PrismaticJoint*>(&joint)) {
    return "prismatic";
  }
  return {};
}

std::optional<Eigen::Vector3d> jointAxis(const dynamics::Joint& joint)
{
  if (const auto* revolute
      = dynamic_cast<const dynamics::RevoluteJoint*>(&joint)) {
    return revolute->getAxis();
  }
  if (const auto* prismatic
      = dynamic_cast<const dynamics::PrismaticJoint*>(&joint)) {
    return prismatic->getAxis();
  }
  return std::nullopt;
}

WriteResult writeGeometry(
    std::ostream& stream, const dynamics::Shape& shape, int depth)
{
  stream << indent(depth) << "<geometry>\n";

  if (const auto* box = dynamic_cast<const dynamics::BoxShape*>(&shape)) {
    const Eigen::Vector3d& size = box->getSize();
    if (!isFinite(size)) {
      return fail("Cannot write SDF box geometry with non-finite size.");
    }
    stream << indent(depth + 1) << "<box><size>" << formatVector3(size)
           << "</size></box>\n";
  } else if (
      const auto* sphere = dynamic_cast<const dynamics::SphereShape*>(&shape)) {
    const double radius = sphere->getRadius();
    if (!std::isfinite(radius)) {
      return fail("Cannot write SDF sphere geometry with non-finite radius.");
    }
    stream << indent(depth + 1) << "<sphere><radius>" << formatDouble(radius)
           << "</radius></sphere>\n";
  } else if (
      const auto* cylinder
      = dynamic_cast<const dynamics::CylinderShape*>(&shape)) {
    const double radius = cylinder->getRadius();
    const double height = cylinder->getHeight();
    if (!std::isfinite(radius) || !std::isfinite(height)) {
      return fail(
          "Cannot write SDF cylinder geometry with non-finite dimensions.");
    }
    stream << indent(depth + 1) << "<cylinder><radius>" << formatDouble(radius)
           << "</radius><length>" << formatDouble(height)
           << "</length></cylinder>\n";
  } else if (
      const auto* mesh = dynamic_cast<const dynamics::MeshShape*>(&shape)) {
    const auto uri = mesh->getMeshUri2().toString();
    if (uri.empty()) {
      return fail("Cannot write SDF mesh geometry without a mesh URI.");
    }
    const Eigen::Vector3d& scale = mesh->getScale();
    if (!isFinite(scale)) {
      return fail("Cannot write SDF mesh geometry with non-finite scale.");
    }
    stream << indent(depth + 1) << "<mesh><uri>" << escapeXml(uri)
           << "</uri><scale>" << formatVector3(scale) << "</scale></mesh>\n";
  } else {
    return fail(
        "Unsupported shape type for SDF writing: "
        + std::string(shape.getType()));
  }

  stream << indent(depth) << "</geometry>\n";
  return ok();
}

WriteResult writeMaterial(
    std::ostream& stream, const dynamics::ShapeNode& shapeNode, int depth)
{
  const auto* visualAspect = shapeNode.getVisualAspect();
  if (!visualAspect || visualAspect->usesDefaultColor()) {
    return ok();
  }

  const Eigen::Vector4d color = visualAspect->getRGBA();
  if (!isFinite(color)) {
    return fail(
        "Cannot write SDF visual [" + shapeNode.getName()
        + "] with a non-finite material color.");
  }

  stream << indent(depth) << "<material>\n";
  stream << indent(depth + 1) << "<diffuse>" << formatVector4(color)
         << "</diffuse>\n";
  stream << indent(depth) << "</material>\n";

  return ok();
}

WriteResult writeShapeNode(
    std::ostream& stream,
    const dynamics::ShapeNode& shapeNode,
    std::string_view tag,
    int depth)
{
  const auto shape = shapeNode.getShape();
  if (!shape) {
    return fail(
        "Cannot write SDF " + std::string(tag) + " [" + shapeNode.getName()
        + "] without a Shape.");
  }

  const Eigen::Isometry3d& pose = shapeNode.getRelativeTransform();
  if (!isFinite(pose)) {
    return fail(
        "Cannot write SDF " + std::string(tag) + " [" + shapeNode.getName()
        + "] with a non-finite pose.");
  }

  stream << indent(depth) << "<" << tag << " name=\""
         << escapeXml(shapeNode.getName()) << "\">\n";
  stream << indent(depth + 1) << "<pose>" << formatPose(pose) << "</pose>\n";
  if (auto result = writeGeometry(stream, *shape, depth + 1); result.isErr()) {
    return result;
  }
  if (tag == "visual") {
    if (auto result = writeMaterial(stream, shapeNode, depth + 1);
        result.isErr()) {
      return result;
    }
  }
  stream << indent(depth) << "</" << tag << ">\n";

  return ok();
}

WriteResult writeLink(
    std::ostream& stream,
    const dynamics::BodyNode& bodyNode,
    const Eigen::Isometry3d& modelPose,
    const WriteOptions& options,
    int depth)
{
  if (!isFinite(modelPose)) {
    return fail(
        "Cannot write SDF link [" + bodyNode.getName()
        + "] with a non-finite pose.");
  }

  stream << indent(depth) << "<link name=\"" << escapeXml(bodyNode.getName())
         << "\">\n";
  stream << indent(depth + 1) << "<pose>" << formatPose(modelPose)
         << "</pose>\n";
  if (!bodyNode.getGravityMode()) {
    stream << indent(depth + 1) << "<gravity>false</gravity>\n";
  }

  const auto& inertia = bodyNode.getInertia();
  const Eigen::Matrix3d moment = inertia.getMoment();
  const Eigen::Vector3d& com = inertia.getLocalCOM();
  if (!std::isfinite(inertia.getMass()) || !isFinite(moment)
      || !isFinite(com)) {
    return fail(
        "Cannot write SDF link [" + bodyNode.getName()
        + "] with non-finite inertial data.");
  }

  stream << indent(depth + 1) << "<inertial>\n";
  stream << indent(depth + 2) << "<pose>" << formatVector3(com)
         << " 0 0 0</pose>\n";
  stream << indent(depth + 2) << "<mass>" << formatDouble(inertia.getMass())
         << "</mass>\n";
  stream << indent(depth + 2) << "<inertia>\n";
  stream << indent(depth + 3) << "<ixx>" << formatDouble(moment(0, 0))
         << "</ixx>\n";
  stream << indent(depth + 3) << "<iyy>" << formatDouble(moment(1, 1))
         << "</iyy>\n";
  stream << indent(depth + 3) << "<izz>" << formatDouble(moment(2, 2))
         << "</izz>\n";
  stream << indent(depth + 3) << "<ixy>" << formatDouble(moment(0, 1))
         << "</ixy>\n";
  stream << indent(depth + 3) << "<ixz>" << formatDouble(moment(0, 2))
         << "</ixz>\n";
  stream << indent(depth + 3) << "<iyz>" << formatDouble(moment(1, 2))
         << "</iyz>\n";
  stream << indent(depth + 2) << "</inertia>\n";
  stream << indent(depth + 1) << "</inertial>\n";

  if (options.includeVisuals) {
    WriteResult result = ok();
    bodyNode.eachShapeNodeWith<dynamics::VisualAspect>(
        [&](const dynamics::ShapeNode* shapeNode) {
          if (result.isErr()) {
            return;
          }
          result = writeShapeNode(stream, *shapeNode, "visual", depth + 1);
        });
    if (result.isErr()) {
      return result;
    }
  }

  if (options.includeCollisions) {
    WriteResult result = ok();
    bodyNode.eachShapeNodeWith<dynamics::CollisionAspect>(
        [&](const dynamics::ShapeNode* shapeNode) {
          if (result.isErr()) {
            return;
          }
          result = writeShapeNode(stream, *shapeNode, "collision", depth + 1);
        });
    if (result.isErr()) {
      return result;
    }
  }

  stream << indent(depth) << "</link>\n";
  return ok();
}

WriteResult computeLinkModelPose(
    const dynamics::BodyNode& bodyNode,
    std::map<const dynamics::BodyNode*, Eigen::Isometry3d>& cache,
    Eigen::Isometry3d& pose)
{
  if (const auto it = cache.find(&bodyNode); it != cache.end()) {
    pose = it->second;
    return ok();
  }

  const dynamics::Joint* joint = bodyNode.getParentJoint();
  if (!joint) {
    return fail(
        "Cannot write SDF link [" + bodyNode.getName()
        + "] without a parent joint.");
  }

  Eigen::Isometry3d parentPose = Eigen::Isometry3d::Identity();
  if (const dynamics::BodyNode* parent = joint->getParentBodyNode()) {
    if (auto result = computeLinkModelPose(*parent, cache, parentPose);
        result.isErr()) {
      return result;
    }
  } else if (
      !dynamic_cast<const dynamics::FreeJoint*>(joint)
      && !dynamic_cast<const dynamics::WeldJoint*>(joint)) {
    return fail(
        "Unsupported SDF root joint type [" + std::string(joint->getType())
        + "] for link [" + bodyNode.getName() + "].");
  }

  pose = parentPose * joint->getTransformFromParentBodyNode()
         * joint->getTransformFromChildBodyNode().inverse();
  cache.emplace(&bodyNode, pose);
  return ok();
}

WriteResult writeAxis(
    std::ostream& stream, const dynamics::Joint& joint, int depth)
{
  const auto axis = jointAxis(joint);
  if (!axis) {
    return ok();
  }
  if (!isFinite(*axis)) {
    return fail(
        "Cannot write SDF joint [" + joint.getName()
        + "] with a non-finite axis.");
  }

  stream << indent(depth) << "<axis>\n";
  stream << indent(depth + 1) << "<xyz>" << formatVector3(*axis) << "</xyz>\n";

  bool wroteLimit = false;
  std::ostringstream limit;
  limit << indent(depth + 1) << "<limit>\n";
  const double lower = joint.getPositionLowerLimit(0);
  const double upper = joint.getPositionUpperLimit(0);
  if (std::isfinite(lower)) {
    wroteLimit = true;
    limit << indent(depth + 2) << "<lower>" << formatDouble(lower)
          << "</lower>\n";
  }
  if (std::isfinite(upper)) {
    wroteLimit = true;
    limit << indent(depth + 2) << "<upper>" << formatDouble(upper)
          << "</upper>\n";
  }
  limit << indent(depth + 1) << "</limit>\n";
  if (wroteLimit) {
    stream << limit.str();
  }

  stream << indent(depth) << "</axis>\n";
  return ok();
}

WriteResult writeJoint(
    std::ostream& stream, const dynamics::Joint& joint, int depth)
{
  const dynamics::BodyNode* parent = joint.getParentBodyNode();
  const dynamics::BodyNode* child = joint.getChildBodyNode();
  if (!parent) {
    return ok();
  }
  if (!child) {
    return fail(
        "Cannot write SDF joint [" + joint.getName()
        + "] without a child link.");
  }

  const std::string type = jointTypeName(joint);
  if (type.empty()) {
    return fail(
        "Unsupported joint type for SDF writing: "
        + std::string(joint.getType()) + " [" + joint.getName() + "].");
  }

  const Eigen::Isometry3d& childToJoint = joint.getTransformFromChildBodyNode();
  if (!isFinite(childToJoint)) {
    return fail(
        "Cannot write SDF joint [" + joint.getName()
        + "] with a non-finite pose.");
  }

  stream << indent(depth) << "<joint name=\"" << escapeXml(joint.getName())
         << "\" type=\"" << type << "\">\n";
  stream << indent(depth + 1) << "<parent>" << escapeXml(parent->getName())
         << "</parent>\n";
  stream << indent(depth + 1) << "<child>" << escapeXml(child->getName())
         << "</child>\n";
  stream << indent(depth + 1) << "<pose>" << formatPose(childToJoint)
         << "</pose>\n";
  if (auto result = writeAxis(stream, joint, depth + 1); result.isErr()) {
    return result;
  }
  stream << indent(depth) << "</joint>\n";

  return ok();
}

} // namespace

common::Result<std::string, common::Error> tryWriteSkeletonToString(
    const dynamics::Skeleton& skeleton, const WriteOptions& options)
{
  if (options.version.empty()) {
    return StringResult::err(WriteError("SDF version must not be empty."));
  }

  std::map<const dynamics::BodyNode*, Eigen::Isometry3d> linkModelPoses;
  for (std::size_t i = 0; i < skeleton.getNumBodyNodes(); ++i) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    if (auto result
        = computeLinkModelPose(*skeleton.getBodyNode(i), linkModelPoses, pose);
        result.isErr()) {
      return StringResult::err(result.error());
    }
  }

  std::ostringstream stream;
  stream << "<?xml version=\"1.0\" ?>\n";
  stream << "<sdf version=\"" << escapeXml(options.version) << "\">\n";
  stream << "  <model name=\"" << escapeXml(skeleton.getName()) << "\">\n";
  stream << "    <static>" << (skeleton.isMobile() ? "false" : "true")
         << "</static>\n";

  for (std::size_t i = 0; i < skeleton.getNumBodyNodes(); ++i) {
    const auto* bodyNode = skeleton.getBodyNode(i);
    if (auto result
        = writeLink(stream, *bodyNode, linkModelPoses.at(bodyNode), options, 2);
        result.isErr()) {
      return StringResult::err(result.error());
    }
  }

  for (std::size_t i = 0; i < skeleton.getNumJoints(); ++i) {
    if (auto result = writeJoint(stream, *skeleton.getJoint(i), 2);
        result.isErr()) {
      return StringResult::err(result.error());
    }
  }

  stream << "  </model>\n";
  stream << "</sdf>\n";

  return StringResult::ok(stream.str());
}

} // namespace SdfParser
} // namespace utils
} // namespace dart
