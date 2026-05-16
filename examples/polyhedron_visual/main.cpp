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

#include <dart/gui/application.hpp>

#include <dart/simulation/world.hpp>

#include <dart/dynamics/convex_mesh_shape.hpp>
#include <dart/dynamics/line_segment_shape.hpp>
#include <dart/dynamics/shape_node.hpp>
#include <dart/dynamics/skeleton.hpp>
#include <dart/dynamics/weld_joint.hpp>

#include <dart/math/tri_mesh.hpp>

#include <Eigen/Geometry>

#include <array>
#include <memory>
#include <string>

namespace {

dart::dynamics::SkeletonPtr createStaticVisualSkeleton(
    const std::string& name,
    const dart::dynamics::ShapePtr& shape,
    const Eigen::Vector3d& position,
    const Eigen::Vector3d& color,
    double alpha = 1.0)
{
  auto skeleton = dart::dynamics::Skeleton::create(name);
  auto* body = skeleton->createJointAndBodyNodePair<dart::dynamics::WeldJoint>()
                   .second;
  auto* shapeNode
      = body->createShapeNodeWith<dart::dynamics::VisualAspect>(shape);
  shapeNode->setRelativeTranslation(position);
  shapeNode->getVisualAspect()->setRGBA(
      Eigen::Vector4d(color.x(), color.y(), color.z(), alpha));
  return skeleton;
}

std::array<Eigen::Vector3d, 8> polyhedronVertices()
{
  return {
      Eigen::Vector3d(-0.5, -0.5, 0.0),
      Eigen::Vector3d(0.5, -0.5, 0.2),
      Eigen::Vector3d(0.6, 0.5, 0.1),
      Eigen::Vector3d(-0.4, 0.6, 0.15),
      Eigen::Vector3d(-0.2, -0.2, 0.9),
      Eigen::Vector3d(0.35, -0.3, 0.8),
      Eigen::Vector3d(0.4, 0.35, 0.75),
      Eigen::Vector3d(-0.35, 0.4, 0.7)};
}

std::shared_ptr<dart::math::TriMesh<double>> createPolyhedronMesh()
{
  auto mesh = std::make_shared<dart::math::TriMesh<double>>();
  const auto vertices = polyhedronVertices();
  mesh->reserveVertices(vertices.size());
  mesh->reserveTriangles(12);
  for (const auto& vertex : vertices) {
    mesh->addVertex(vertex);
  }

  mesh->addTriangle(0, 2, 1);
  mesh->addTriangle(0, 3, 2);
  mesh->addTriangle(4, 5, 6);
  mesh->addTriangle(4, 6, 7);
  mesh->addTriangle(0, 1, 5);
  mesh->addTriangle(0, 5, 4);
  mesh->addTriangle(1, 2, 6);
  mesh->addTriangle(1, 6, 5);
  mesh->addTriangle(2, 3, 7);
  mesh->addTriangle(2, 7, 6);
  mesh->addTriangle(3, 0, 4);
  mesh->addTriangle(3, 4, 7);
  mesh->computeVertexNormals();
  return mesh;
}

std::shared_ptr<dart::dynamics::LineSegmentShape> createPolyhedronWireframe()
{
  auto shape = std::make_shared<dart::dynamics::LineSegmentShape>(2.0f);
  const auto vertices = polyhedronVertices();
  for (const auto& vertex : vertices) {
    shape->addVertex(vertex);
  }
  while (!shape->getConnections().empty()) {
    shape->removeConnection(0);
  }

  const std::array<std::pair<std::size_t, std::size_t>, 12> edges{{
      {0, 1},
      {1, 2},
      {2, 3},
      {3, 0},
      {4, 5},
      {5, 6},
      {6, 7},
      {7, 4},
      {0, 4},
      {1, 5},
      {2, 6},
      {3, 7},
  }};
  for (const auto& edge : edges) {
    shape->addConnection(edge.first, edge.second);
  }
  return shape;
}

dart::simulation::WorldPtr createPolyhedronWorld()
{
  auto world = dart::simulation::World::create("dartsim_polyhedron");
  world->setGravity(Eigen::Vector3d::Zero());
  world->addSkeleton(createStaticVisualSkeleton(
      "visual_polyhedron_surface",
      std::make_shared<dart::dynamics::ConvexMeshShape>(createPolyhedronMesh()),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.1, 0.8, 0.6),
      0.5));
  world->addSkeleton(createStaticVisualSkeleton(
      "visual_polyhedron_wireframe",
      createPolyhedronWireframe(),
      Eigen::Vector3d::Zero(),
      Eigen::Vector3d(0.05, 0.05, 0.05)));
  return world;
}

} // namespace

int main(int argc, char* argv[])
{
  dart::gui::ApplicationOptions options;
  options.world = createPolyhedronWorld();
  return dart::gui::runApplication(argc, argv, options);
}
