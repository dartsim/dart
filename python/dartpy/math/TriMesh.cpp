// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 * See LICENSE for the BSD-style license governing this file.
 */

#include "detail/eigen.hpp"

#include <dart/math/TriMesh.hpp>

#include <cmath>

namespace dart {
namespace python {

void TriMesh(nb::module_& m)
{
  using Mesh = dart::math::TriMesh<double>;
  using Rows = Eigen::Matrix<double, Eigen::Dynamic, 3>;

  dartnb::dart_class<Mesh>(m, "TriMesh")
      .def(dartnb::init<>())
      .def(
          dartnb::factory([](const Rows& vertices, const Rows& triangles) {
            if (!vertices.allFinite())
              throw nb::value_error("vertices must be finite");
            if (!triangles.allFinite())
              throw nb::value_error("triangle indices must be finite integers");
            auto mesh = std::make_shared<Mesh>();
            for (Eigen::Index i = 0; i < vertices.rows(); ++i)
              mesh->addVertex(vertices.row(i).transpose());
            for (Eigen::Index i = 0; i < triangles.rows(); ++i) {
              Mesh::Triangle triangle;
              for (Eigen::Index j = 0; j < 3; ++j) {
                const double index = triangles(i, j);
                if (index < 0 || index >= vertices.rows()
                    || std::floor(index) != index)
                  throw nb::value_error(
                      "triangle indices must be integers within the vertex "
                      "array");
                triangle[j] = static_cast<Mesh::Index>(index);
              }
              mesh->addTriangle(triangle);
            }
            return mesh;
          }),
          nb::arg("vertices"),
          nb::arg("triangles"))
      .def_prop_ro(
          "vertices",
          [](const Mesh& self) {
            Rows result(self.getVertices().size(), 3);
            for (std::size_t i = 0; i < self.getVertices().size(); ++i)
              result.row(i) = self.getVertices()[i].transpose();
            return result;
          })
      .def_prop_ro(
          "triangles",
          [](const Mesh& self) {
            Eigen::Matrix<Mesh::Index, Eigen::Dynamic, 3> result(
                self.getTriangles().size(), 3);
            for (std::size_t i = 0; i < self.getTriangles().size(); ++i)
              result.row(i) = self.getTriangles()[i].transpose();
            return result;
          })
      .def_prop_ro(
          "num_vertices",
          [](const Mesh& self) { return self.getVertices().size(); })
      .def_prop_ro("num_triangles", [](const Mesh& self) {
        return self.getTriangles().size();
      });
}

} // namespace python
} // namespace dart
