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

#include "dart/gui/vsg/debug_draw.hpp"

#include "dart/gui/vsg/conversions.hpp"
#include "dart/gui/vsg/geometry_builders.hpp"

#include <vsg/utils/Builder.h>
#include <vsg/utils/GraphicsPipelineConfigurator.h>
#include <vsg/utils/ShaderSet.h>

namespace dart::gui::vsg {

namespace {

::vsg::ref_ptr<::vsg::ShaderSet> getLineShaderSet()
{
  static ::vsg::ref_ptr<::vsg::ShaderSet> shaderSet;
  if (!shaderSet) {
    shaderSet = ::vsg::createFlatShadedShaderSet();
  }
  return shaderSet;
}

::vsg::ref_ptr<::vsg::Node> createLineGeometry(
    const ::vsg::ref_ptr<::vsg::vec3Array>& vertices,
    const ::vsg::ref_ptr<::vsg::vec4Array>& vertexColors)
{
  auto shaderSet = getLineShaderSet();

  auto config = ::vsg::GraphicsPipelineConfigurator::create(shaderSet);

  auto inputAssembly
      = ::vsg::InputAssemblyState::create(VK_PRIMITIVE_TOPOLOGY_LINE_LIST);
  config->pipelineStates.push_back(inputAssembly);

  auto normals = ::vsg::vec3Array::create(vertices->size());
  for (size_t i = 0; i < vertices->size(); ++i) {
    (*normals)[i].set(0.0f, 0.0f, 1.0f);
  }

  ::vsg::DataList vertexArrays;
  config->assignArray(
      vertexArrays, "vsg_Vertex", VK_VERTEX_INPUT_RATE_VERTEX, vertices);
  config->assignArray(
      vertexArrays, "vsg_Normal", VK_VERTEX_INPUT_RATE_VERTEX, normals);
  config->assignArray(
      vertexArrays, "vsg_Color", VK_VERTEX_INPUT_RATE_VERTEX, vertexColors);

  config->init();

  auto stateGroup = ::vsg::StateGroup::create();
  config->copyTo(stateGroup);

  auto vertexDraw = ::vsg::VertexDraw::create();
  vertexDraw->assignArrays(vertexArrays);
  vertexDraw->vertexCount = static_cast<uint32_t>(vertices->size());
  vertexDraw->instanceCount = 1;

  stateGroup->addChild(vertexDraw);

  return stateGroup;
}

} // namespace

::vsg::ref_ptr<::vsg::Node> createPoint(
    const Eigen::Vector3d& position, double size, const Eigen::Vector4d& color)
{
  auto transform = createTransform(
      Eigen::Translation3d(position) * Eigen::Isometry3d::Identity());
  transform->addChild(createSphere(size, GeometryOptions{color, false, false}));
  return transform;
}

::vsg::ref_ptr<::vsg::Node> createLine(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Vector4d& color)
{
  auto vertices = ::vsg::vec3Array::create(2);
  (*vertices)[0].set(
      static_cast<float>(start.x()),
      static_cast<float>(start.y()),
      static_cast<float>(start.z()));
  (*vertices)[1].set(
      static_cast<float>(end.x()),
      static_cast<float>(end.y()),
      static_cast<float>(end.z()));

  ::vsg::vec4 vsgColor(
      static_cast<float>(color.x()),
      static_cast<float>(color.y()),
      static_cast<float>(color.z()),
      static_cast<float>(color.w()));

  auto colors = ::vsg::vec4Array::create(2);
  (*colors)[0] = vsgColor;
  (*colors)[1] = vsgColor;

  return createLineGeometry(vertices, colors);
}

::vsg::ref_ptr<::vsg::Node> createArrow(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& direction,
    double length,
    const Eigen::Vector4d& color)
{
  Eigen::Vector3d dir = direction.normalized();
  Eigen::Vector3d end = start + dir * length;

  auto group = ::vsg::Group::create();

  group->addChild(createLine(start, end, color));

  double coneRadius = length * 0.08;
  double coneHeight = length * 0.2;

  Eigen::Vector3d coneBase = end - dir * coneHeight;
  Eigen::Vector3d zAxis = Eigen::Vector3d::UnitZ();

  Eigen::Isometry3d coneTransform = Eigen::Isometry3d::Identity();
  coneTransform.translation() = coneBase;

  if (dir.cross(zAxis).norm() > 1e-6) {
    Eigen::Quaterniond rotation;
    rotation.setFromTwoVectors(zAxis, dir);
    coneTransform.linear() = rotation.toRotationMatrix();
  } else if (dir.dot(zAxis) < 0) {
    coneTransform.linear()
        = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  }

  auto coneTransformNode = createTransform(coneTransform);
  coneTransformNode->addChild(
      createCone(coneRadius, coneHeight, GeometryOptions{color, false, false}));
  group->addChild(coneTransformNode);

  return group;
}

::vsg::ref_ptr<::vsg::Node> createPoints(
    const std::vector<Eigen::Vector3d>& positions,
    double size,
    const Eigen::Vector4d& color)
{
  auto group = ::vsg::Group::create();
  for (const auto& pos : positions) {
    group->addChild(createPoint(pos, size, color));
  }
  return group;
}

::vsg::ref_ptr<::vsg::Node> createLines(
    const std::vector<Eigen::Vector3d>& starts,
    const std::vector<Eigen::Vector3d>& ends,
    const Eigen::Vector4d& color)
{
  if (starts.size() != ends.size() || starts.empty()) {
    return ::vsg::Group::create();
  }

  auto vertices
      = ::vsg::vec3Array::create(static_cast<uint32_t>(starts.size() * 2));
  auto colors
      = ::vsg::vec4Array::create(static_cast<uint32_t>(starts.size() * 2));

  ::vsg::vec4 vsgColor(
      static_cast<float>(color.x()),
      static_cast<float>(color.y()),
      static_cast<float>(color.z()),
      static_cast<float>(color.w()));

  for (size_t i = 0; i < starts.size(); ++i) {
    (*vertices)[i * 2].set(
        static_cast<float>(starts[i].x()),
        static_cast<float>(starts[i].y()),
        static_cast<float>(starts[i].z()));
    (*vertices)[i * 2 + 1].set(
        static_cast<float>(ends[i].x()),
        static_cast<float>(ends[i].y()),
        static_cast<float>(ends[i].z()));
    (*colors)[i * 2] = vsgColor;
    (*colors)[i * 2 + 1] = vsgColor;
  }

  return createLineGeometry(vertices, colors);
}

::vsg::ref_ptr<::vsg::Node> createArrows(
    const std::vector<Eigen::Vector3d>& starts,
    const std::vector<Eigen::Vector3d>& directions,
    double length,
    const Eigen::Vector4d& color)
{
  auto group = ::vsg::Group::create();
  size_t count = std::min(starts.size(), directions.size());
  for (size_t i = 0; i < count; ++i) {
    group->addChild(createArrow(starts[i], directions[i], length, color));
  }
  return group;
}

::vsg::ref_ptr<::vsg::Node> createAxes(double length, double thickness)
{
  auto group = ::vsg::Group::create();

  group->addChild(createArrow(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), length, colors::Red));
  group->addChild(createArrow(
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitY(),
      length,
      colors::Green));
  group->addChild(createArrow(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitZ(), length, colors::Blue));

  (void)thickness;
  return group;
}

::vsg::ref_ptr<::vsg::Node> createGrid(
    double size, double spacing, const Eigen::Vector4d& color)
{
  std::vector<Eigen::Vector3d> starts;
  std::vector<Eigen::Vector3d> ends;

  double halfSize = size / 2.0;
  int numLines = static_cast<int>(size / spacing) + 1;

  for (int i = 0; i < numLines; ++i) {
    double offset = -halfSize + i * spacing;

    starts.push_back(Eigen::Vector3d(offset, -halfSize, 0));
    ends.push_back(Eigen::Vector3d(offset, halfSize, 0));

    starts.push_back(Eigen::Vector3d(-halfSize, offset, 0));
    ends.push_back(Eigen::Vector3d(halfSize, offset, 0));
  }

  return createLines(starts, ends, color);
}

::vsg::ref_ptr<::vsg::Node> createWireframeBox(
    const Eigen::Vector3d& min,
    const Eigen::Vector3d& max,
    const Eigen::Vector4d& color)
{
  // 8 corners of the box
  Eigen::Vector3d c000(min.x(), min.y(), min.z());
  Eigen::Vector3d c001(min.x(), min.y(), max.z());
  Eigen::Vector3d c010(min.x(), max.y(), min.z());
  Eigen::Vector3d c011(min.x(), max.y(), max.z());
  Eigen::Vector3d c100(max.x(), min.y(), min.z());
  Eigen::Vector3d c101(max.x(), min.y(), max.z());
  Eigen::Vector3d c110(max.x(), max.y(), min.z());
  Eigen::Vector3d c111(max.x(), max.y(), max.z());

  // 12 edges of the box
  std::vector<Eigen::Vector3d> starts;
  std::vector<Eigen::Vector3d> ends;

  // Bottom face edges (z = min)
  starts.push_back(c000);
  ends.push_back(c100);
  starts.push_back(c100);
  ends.push_back(c110);
  starts.push_back(c110);
  ends.push_back(c010);
  starts.push_back(c010);
  ends.push_back(c000);

  // Top face edges (z = max)
  starts.push_back(c001);
  ends.push_back(c101);
  starts.push_back(c101);
  ends.push_back(c111);
  starts.push_back(c111);
  ends.push_back(c011);
  starts.push_back(c011);
  ends.push_back(c001);

  // Vertical edges connecting top and bottom
  starts.push_back(c000);
  ends.push_back(c001);
  starts.push_back(c100);
  ends.push_back(c101);
  starts.push_back(c110);
  ends.push_back(c111);
  starts.push_back(c010);
  ends.push_back(c011);

  return createLines(starts, ends, color);
}

} // namespace dart::gui::vsg
