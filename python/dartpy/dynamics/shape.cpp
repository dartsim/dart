#include "dynamics/shape.hpp"

#include "common/eigen_utils.hpp"
#include "common/repr.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/math/tri_mesh.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <memory>

namespace nb = nanobind;

namespace dart::python_nb {

void defShape(nb::module_& m)
{
  using Shape = dart::dynamics::Shape;

  nb::class_<Shape>(m, "Shape")
      .def(
          "getType",
          [](const Shape& self) { return std::string(self.getType()); })
      .def("computeInertia", &Shape::computeInertia, nb::arg("mass"))
      .def("__repr__", [](const Shape& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("type", repr_string(self.getType()));
        return format_repr("Shape", fields);
      });

  nb::class_<dart::dynamics::SphereShape, Shape>(m, "SphereShape")
      .def(
          nb::new_([](double radius) {
            return std::make_shared<dart::dynamics::SphereShape>(radius);
          }),
          nb::arg("radius"))
      .def(
          "setRadius",
          &dart::dynamics::SphereShape::setRadius,
          nb::arg("radius"))
      .def("getRadius", &dart::dynamics::SphereShape::getRadius)
      .def(
          "computeInertia",
          [](const dart::dynamics::SphereShape& self, double mass) {
            return self.computeInertia(mass);
          },
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          []() {
            return std::string(dart::dynamics::SphereShape::getStaticType());
          })
      .def_static(
          "computeVolumeOf",
          [](double radius) {
            return dart::dynamics::SphereShape::computeVolume(radius);
          },
          nb::arg("radius"))
      .def_static(
          "computeInertiaOf",
          [](double radius, double mass) {
            return dart::dynamics::SphereShape::computeInertia(radius, mass);
          },
          nb::arg("radius"),
          nb::arg("mass"))
      .def("__repr__", [](const dart::dynamics::SphereShape& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("radius", repr_double(self.getRadius()));
        return format_repr("SphereShape", fields);
      });

  nb::class_<dart::dynamics::BoxShape, Shape>(m, "BoxShape")
      .def(
          nb::new_([](const Eigen::Vector3d& size) {
            return std::make_shared<dart::dynamics::BoxShape>(size);
          }),
          nb::arg("size"))
      .def(
          nb::new_([](const nb::handle& size) {
            return std::make_shared<dart::dynamics::BoxShape>(toVector3(size));
          }),
          nb::arg("size"))
      .def("setSize", &dart::dynamics::BoxShape::setSize, nb::arg("size"))
      .def(
          "getSize",
          [](const dart::dynamics::BoxShape& self) { return self.getSize(); })
      .def(
          "computeInertia",
          [](const dart::dynamics::BoxShape& self, double mass) {
            return self.computeInertia(mass);
          },
          nb::arg("mass"))
      .def_static(
          "computeVolumeOf",
          [](const Eigen::Vector3d& size) {
            return dart::dynamics::BoxShape::computeVolume(size);
          },
          nb::arg("size"))
      .def_static(
          "computeInertiaOf",
          [](const Eigen::Vector3d& size, double mass) {
            return dart::dynamics::BoxShape::computeInertia(size, mass);
          },
          nb::arg("size"),
          nb::arg("mass"))
      .def("__repr__", [](const dart::dynamics::BoxShape& self) {
        const auto size = self.getSize();
        std::ostringstream size_stream;
        size_stream << "[" << repr_double(size[0]) << ", "
                    << repr_double(size[1]) << ", " << repr_double(size[2])
                    << "]";
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("size", size_stream.str());
        return format_repr("BoxShape", fields);
      });

  nb::class_<dart::dynamics::MeshShape, Shape>(m, "MeshShape")
      .def(
          nb::new_(
              [](const nb::handle& scale,
                 const std::shared_ptr<dart::math::TriMesh<double>>& mesh) {
                nb::sequence seq = nb::cast<nb::sequence>(scale);
                if (nb::len(seq) != 3) {
                  throw nb::type_error("MeshShape scale must have length 3");
                }

                Eigen::Vector3d vec;
                for (size_t i = 0; i < 3; ++i) {
                  vec[i] = nb::cast<double>(seq[i]);
                }

                return std::make_shared<dart::dynamics::MeshShape>(vec, mesh);
              }),
          nb::arg("scale"),
          nb::arg("mesh"))
      .def("getTriMesh", &dart::dynamics::MeshShape::getTriMesh)
      .def(
          "getScale",
          [](const dart::dynamics::MeshShape& self) { return self.getScale(); })
      .def_static("getStaticType", []() {
        return std::string(dart::dynamics::MeshShape::getStaticType());
      });
}

} // namespace dart::python_nb
