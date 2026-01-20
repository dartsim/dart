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

#include "math/trimesh.hpp"

#include "dart/math/tri_mesh.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <vector>

#include <cstddef>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

Eigen::Vector3d toVec3(const nb::handle& h)
{
  try {
    return nb::cast<Eigen::Vector3d>(h);
  } catch (const nb::cast_error&) {
    nb::sequence seq = nb::cast<nb::sequence>(h);
    if (nb::len(seq) != 3)
      throw nb::type_error("Expected a length-3 sequence");

    Eigen::Vector3d vec;
    for (nb::ssize_t i = 0; i < 3; ++i)
      vec[i] = nb::cast<double>(seq[i]);
    return vec;
  }
}

} // namespace

void defTriMesh(nb::module_& m)
{
  using TriMeshd = dart::math::TriMesh<double>;

  nb::class_<TriMeshd>(m, "TriMesh")
      .def(
          nb::new_([]() { return std::make_shared<TriMeshd>(); }),
          "Create an empty TriMesh")
      .def(
          "addVertex",
          [](TriMeshd& self, const nb::handle& vertex) {
            self.addVertex(toVec3(vertex));
          },
          nb::arg("vertex"),
          "Add a vertex to the mesh")
      .def(
          "addTriangle",
          [](TriMeshd& self, std::size_t v0, std::size_t v1, std::size_t v2) {
            self.addTriangle(v0, v1, v2);
          },
          nb::arg("v0"),
          nb::arg("v1"),
          nb::arg("v2"),
          "Add a triangle (by vertex indices)")
      .def(
          "addVertexNormal",
          [](TriMeshd& self, const nb::handle& normal) {
            self.addVertexNormal(toVec3(normal));
          },
          nb::arg("normal"),
          "Add a vertex normal")
      .def(
          "reserveVertices",
          &TriMeshd::reserveVertices,
          nb::arg("count"),
          "Reserve space for vertices")
      .def(
          "reserveTriangles",
          &TriMeshd::reserveTriangles,
          nb::arg("count"),
          "Reserve space for triangles")
      .def(
          "reserveVertexNormals",
          &TriMeshd::reserveVertexNormals,
          nb::arg("count"),
          "Reserve space for vertex normals")
      .def(
          "getVertices",
          [](const TriMeshd& self) {
            return std::vector<Eigen::Vector3d>(
                self.getVertices().begin(), self.getVertices().end());
          },
          "Get the list of vertices")
      .def(
          "getTriangles",
          [](const TriMeshd& self) {
            std::vector<Eigen::Vector3i> triangles;
            triangles.reserve(self.getTriangles().size());
            for (const auto& triangle : self.getTriangles()) {
              triangles.emplace_back(
                  static_cast<int>(triangle[0]),
                  static_cast<int>(triangle[1]),
                  static_cast<int>(triangle[2]));
            }
            return triangles;
          },
          "Get the list of triangles (vertex indices)")
      .def(
          "getVertexNormals",
          [](const TriMeshd& self) {
            return std::vector<Eigen::Vector3d>(
                self.getVertexNormals().begin(), self.getVertexNormals().end());
          },
          "Get the list of vertex normals")
      .def(
          "hasVertexNormals",
          &TriMeshd::hasVertexNormals,
          "Check if the mesh has vertex normals")
      .def(
          "computeVertexNormals",
          &TriMeshd::computeVertexNormals,
          "Compute vertex normals")
      .def(
          "getNumVertices",
          [](const TriMeshd& self) { return self.getVertices().size(); },
          "Get the number of vertices")
      .def(
          "getNumTriangles",
          [](const TriMeshd& self) { return self.getTriangles().size(); },
          "Get the number of triangles")
      .def(
          "clear",
          &TriMeshd::clear,
          "Clear all vertices, triangles, and normals");
}

} // namespace dart::python_nb
