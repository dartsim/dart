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

#include "../eigen_geometry_pybind.h"
#include "../eigen_pybind.h"

#include <dart/dart.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace dart {
namespace python {

void TriMesh(py::module& m)
{
  ::py::class_<
      dart::math::TriMesh<double>,
      std::shared_ptr<dart::math::TriMesh<double>>>(m, "TriMesh")
      .def(::py::init<>(), "Create an empty TriMesh")
      .def(
          "addVertex",
          +[](dart::math::TriMesh<double>* self,
              const Eigen::Vector3d& vertex) { self->addVertex(vertex); },
          ::py::arg("vertex"),
          "Add a vertex to the mesh")
      .def(
          "addTriangle",
          +[](dart::math::TriMesh<double>* self,
              std::size_t v0,
              std::size_t v1,
              std::size_t v2) { self->addTriangle(v0, v1, v2); },
          ::py::arg("v0"),
          ::py::arg("v1"),
          ::py::arg("v2"),
          "Add a triangle (by vertex indices)")
      .def(
          "addVertexNormal",
          +[](dart::math::TriMesh<double>* self,
              const Eigen::Vector3d& normal) { self->addVertexNormal(normal); },
          ::py::arg("normal"),
          "Add a vertex normal")
      .def(
          "reserveVertices",
          +[](dart::math::TriMesh<double>* self, std::size_t count) {
            self->reserveVertices(count);
          },
          ::py::arg("count"),
          "Reserve space for vertices")
      .def(
          "reserveTriangles",
          +[](dart::math::TriMesh<double>* self, std::size_t count) {
            self->reserveTriangles(count);
          },
          ::py::arg("count"),
          "Reserve space for triangles")
      .def(
          "reserveVertexNormals",
          +[](dart::math::TriMesh<double>* self, std::size_t count) {
            self->reserveVertexNormals(count);
          },
          ::py::arg("count"),
          "Reserve space for vertex normals")
      .def(
          "getVertices",
          +[](const dart::math::TriMesh<double>* self)
              -> const std::vector<Eigen::Vector3d>& {
            return self->getVertices();
          },
          ::py::return_value_policy::reference_internal,
          "Get the list of vertices")
      .def(
          "getTriangles",
          +[](const dart::math::TriMesh<double>* self) {
            return self->getTriangles();
          },
          ::py::return_value_policy::reference_internal,
          "Get the list of triangles (vertex indices)")
      .def(
          "getVertexNormals",
          +[](const dart::math::TriMesh<double>* self)
              -> const std::vector<Eigen::Vector3d>& {
            return self->getVertexNormals();
          },
          ::py::return_value_policy::reference_internal,
          "Get the list of vertex normals")
      .def(
          "hasVertexNormals",
          +[](const dart::math::TriMesh<double>* self) -> bool {
            return self->hasVertexNormals();
          },
          "Check if the mesh has vertex normals")
      .def(
          "getNumVertices",
          +[](const dart::math::TriMesh<double>* self) -> std::size_t {
            return self->getVertices().size();
          },
          "Get the number of vertices")
      .def(
          "getNumTriangles",
          +[](const dart::math::TriMesh<double>* self) -> std::size_t {
            return self->getTriangles().size();
          },
          "Get the number of triangles")
      .def(
          "clear",
          +[](dart::math::TriMesh<double>* self) { self->clear(); },
          "Clear all vertices, triangles, and normals");
}

} // namespace python
} // namespace dart
