#include "dynamics/shape.hpp"

#include "common/eigen_utils.hpp"
#include "common/repr.hpp"
#include "dart/dynamics/box_shape.hpp"
#include "dart/dynamics/capsule_shape.hpp"
#include "dart/dynamics/cone_shape.hpp"
#include "dart/dynamics/cylinder_shape.hpp"
#include "dart/dynamics/ellipsoid_shape.hpp"
#include "dart/dynamics/heightmap_shape.hpp"
#include "dart/dynamics/line_segment_shape.hpp"
#include "dart/dynamics/mesh_shape.hpp"
#include "dart/dynamics/plane_shape.hpp"
#include "dart/dynamics/point_cloud_shape.hpp"
#include "dart/dynamics/pyramid_shape.hpp"
#include "dart/dynamics/shape.hpp"
#include "dart/dynamics/sphere_shape.hpp"
#include "dart/math/tri_mesh.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <span>
#include <vector>

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

  nb::class_<dart::dynamics::CapsuleShape, Shape>(m, "CapsuleShape")
      .def(
          nb::new_([](double radius, double height) {
            return std::make_shared<dart::dynamics::CapsuleShape>(
                radius, height);
          }),
          nb::arg("radius"),
          nb::arg("height"))
      .def("getRadius", &dart::dynamics::CapsuleShape::getRadius)
      .def(
          "setRadius",
          &dart::dynamics::CapsuleShape::setRadius,
          nb::arg("radius"))
      .def("getHeight", &dart::dynamics::CapsuleShape::getHeight)
      .def(
          "setHeight",
          &dart::dynamics::CapsuleShape::setHeight,
          nb::arg("height"))
      .def(
          "computeInertia",
          [](const dart::dynamics::CapsuleShape& self, double mass) {
            return self.computeInertia(mass);
          },
          nb::arg("mass"));

  nb::class_<dart::dynamics::CylinderShape, Shape>(m, "CylinderShape")
      .def(
          nb::new_([](double radius, double height) {
            return std::make_shared<dart::dynamics::CylinderShape>(
                radius, height);
          }),
          nb::arg("radius"),
          nb::arg("height"))
      .def("getRadius", &dart::dynamics::CylinderShape::getRadius)
      .def(
          "setRadius",
          &dart::dynamics::CylinderShape::setRadius,
          nb::arg("radius"))
      .def("getHeight", &dart::dynamics::CylinderShape::getHeight)
      .def(
          "setHeight",
          &dart::dynamics::CylinderShape::setHeight,
          nb::arg("height"))
      .def(
          "computeInertia",
          [](const dart::dynamics::CylinderShape& self, double mass) {
            return self.computeInertia(mass);
          },
          nb::arg("mass"));

  nb::class_<dart::dynamics::EllipsoidShape, Shape>(m, "EllipsoidShape")
      .def(
          nb::new_([](const Eigen::Vector3d& diameters) {
            return std::make_shared<dart::dynamics::EllipsoidShape>(diameters);
          }),
          nb::arg("diameters"))
      .def(
          "getDiameters",
          [](const dart::dynamics::EllipsoidShape& self) {
            return self.getDiameters();
          })
      .def(
          "setDiameters",
          &dart::dynamics::EllipsoidShape::setDiameters,
          nb::arg("diameters"))
      .def(
          "computeInertia",
          [](const dart::dynamics::EllipsoidShape& self, double mass) {
            return self.computeInertia(mass);
          },
          nb::arg("mass"));

  nb::class_<dart::dynamics::ConeShape, Shape>(m, "ConeShape")
      .def(
          nb::new_([](double radius, double height) {
            return std::make_shared<dart::dynamics::ConeShape>(radius, height);
          }),
          nb::arg("radius"),
          nb::arg("height"))
      .def("getRadius", &dart::dynamics::ConeShape::getRadius)
      .def(
          "setRadius", &dart::dynamics::ConeShape::setRadius, nb::arg("radius"))
      .def("getHeight", &dart::dynamics::ConeShape::getHeight)
      .def(
          "setHeight", &dart::dynamics::ConeShape::setHeight, nb::arg("height"))
      .def(
          "computeInertia",
          [](const dart::dynamics::ConeShape& self, double mass) {
            return self.computeInertia(mass);
          },
          nb::arg("mass"));

  nb::class_<dart::dynamics::PyramidShape, Shape>(m, "PyramidShape")
      .def(
          nb::new_([](double base_width, double base_depth, double height) {
            return std::make_shared<dart::dynamics::PyramidShape>(
                base_width, base_depth, height);
          }),
          nb::arg("base_width"),
          nb::arg("base_depth"),
          nb::arg("height"))
      .def("getBaseWidth", &dart::dynamics::PyramidShape::getBaseWidth)
      .def(
          "setBaseWidth",
          &dart::dynamics::PyramidShape::setBaseWidth,
          nb::arg("width"))
      .def("getBaseDepth", &dart::dynamics::PyramidShape::getBaseDepth)
      .def(
          "setBaseDepth",
          &dart::dynamics::PyramidShape::setBaseDepth,
          nb::arg("depth"))
      .def("getHeight", &dart::dynamics::PyramidShape::getHeight)
      .def(
          "setHeight",
          &dart::dynamics::PyramidShape::setHeight,
          nb::arg("height"))
      .def(
          "computeInertia",
          [](const dart::dynamics::PyramidShape& self, double mass) {
            return self.computeInertia(mass);
          },
          nb::arg("mass"));

  nb::class_<dart::dynamics::PlaneShape, Shape>(m, "PlaneShape")
      .def(
          nb::new_([](const Eigen::Vector3d& normal, double offset) {
            return std::make_shared<dart::dynamics::PlaneShape>(normal, offset);
          }),
          nb::arg("normal"),
          nb::arg("offset"))
      .def(
          "setNormal",
          &dart::dynamics::PlaneShape::setNormal,
          nb::arg("normal"))
      .def(
          "getNormal",
          [](const dart::dynamics::PlaneShape& self) -> Eigen::Vector3d {
            return self.getNormal();
          })
      .def(
          "setOffset",
          &dart::dynamics::PlaneShape::setOffset,
          nb::arg("offset"))
      .def("getOffset", &dart::dynamics::PlaneShape::getOffset);

  nb::class_<dart::dynamics::LineSegmentShape, Shape>(m, "LineSegmentShape")
      .def(
          nb::new_([](float thickness) {
            return std::make_shared<dart::dynamics::LineSegmentShape>(
                thickness);
          }),
          nb::arg("thickness") = 1.0f)
      .def(
          "addVertex",
          [](dart::dynamics::LineSegmentShape& self, const Eigen::Vector3d& v) {
            return self.addVertex(v);
          },
          nb::arg("vertex"))
      .def(
          "addConnection",
          &dart::dynamics::LineSegmentShape::addConnection,
          nb::arg("a"),
          nb::arg("b"))
      .def(
          "setVertex",
          &dart::dynamics::LineSegmentShape::setVertex,
          nb::arg("index"),
          nb::arg("vertex"))
      .def("getThickness", &dart::dynamics::LineSegmentShape::getThickness)
      .def(
          "setThickness",
          &dart::dynamics::LineSegmentShape::setThickness,
          nb::arg("thickness"));

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

  using HeightmapShaped = dart::dynamics::HeightmapShape<double>;
  nb::class_<HeightmapShaped, Shape>(m, "HeightmapShape")
      .def(nb::new_([]() { return std::make_shared<HeightmapShaped>(); }))
      .def(
          "setHeightField",
          [](HeightmapShaped& self,
             std::size_t width,
             std::size_t depth,
             const std::vector<double>& heights) {
            self.setHeightField(width, depth, std::span<const double>(heights));
          },
          nb::arg("width"),
          nb::arg("depth"),
          nb::arg("heights"))
      .def(
          "setHeightField",
          [](HeightmapShaped& self,
             const HeightmapShaped::HeightField& heights) {
            self.setHeightField(heights);
          },
          nb::arg("heights"))
      .def(
          "getHeightField",
          [](const HeightmapShaped& self) -> HeightmapShaped::HeightField {
            return self.getHeightField();
          })
      .def("setScale", &HeightmapShaped::setScale, nb::arg("scale"))
      .def(
          "getScale",
          [](const HeightmapShaped& self) { return self.getScale(); })
      .def("getWidth", &HeightmapShaped::getWidth)
      .def("getDepth", &HeightmapShaped::getDepth)
      .def("getMinHeight", &HeightmapShaped::getMinHeight)
      .def("getMaxHeight", &HeightmapShaped::getMaxHeight)
      .def("flipY", &HeightmapShaped::flipY)
      .def_static("getStaticType", []() {
        return std::string(HeightmapShaped::getStaticType());
      });

  using PointCloudShape = dart::dynamics::PointCloudShape;
  auto pointCloudShape
      = nb::class_<PointCloudShape, Shape>(m, "PointCloudShape")
            .def(
                nb::new_([](double visualSize) {
                  return std::make_shared<PointCloudShape>(visualSize);
                }),
                nb::arg("visual_size") = 0.01)
            .def("reserve", &PointCloudShape::reserve, nb::arg("size"))
            .def(
                "addPoint",
                [](PointCloudShape& self, const Eigen::Vector3d& point) {
                  self.addPoint(point);
                },
                nb::arg("point"))
            .def(
                "addPoints",
                [](PointCloudShape& self,
                   const std::vector<Eigen::Vector3d>& points) {
                  self.addPoint(std::span<const Eigen::Vector3d>(points));
                },
                nb::arg("points"))
            .def(
                "setPoints",
                [](PointCloudShape& self,
                   const std::vector<Eigen::Vector3d>& points) {
                  self.setPoint(std::span<const Eigen::Vector3d>(points));
                },
                nb::arg("points"))
            .def(
                "getPoints",
                [](const PointCloudShape& self) {
                  const auto span = self.getPoints();
                  return std::vector<Eigen::Vector3d>(span.begin(), span.end());
                })
            .def("getNumPoints", &PointCloudShape::getNumPoints)
            .def("removeAllPoints", &PointCloudShape::removeAllPoints)
            .def(
                "setPointShapeType",
                &PointCloudShape::setPointShapeType,
                nb::arg("type"))
            .def("getPointShapeType", &PointCloudShape::getPointShapeType)
            .def(
                "setColorMode", &PointCloudShape::setColorMode, nb::arg("mode"))
            .def("getColorMode", &PointCloudShape::getColorMode)
            .def(
                "setOverallColor",
                &PointCloudShape::setOverallColor,
                nb::arg("color"))
            .def("getOverallColor", &PointCloudShape::getOverallColor)
            .def(
                "setVisualSize",
                &PointCloudShape::setVisualSize,
                nb::arg("size"))
            .def("getVisualSize", &PointCloudShape::getVisualSize)
            .def_static("getStaticType", []() {
              return std::string(PointCloudShape::getStaticType());
            });

  nb::enum_<PointCloudShape::ColorMode>(pointCloudShape, "ColorMode")
      .value("USE_SHAPE_COLOR", PointCloudShape::USE_SHAPE_COLOR)
      .value("BIND_OVERALL", PointCloudShape::BIND_OVERALL)
      .value("BIND_PER_POINT", PointCloudShape::BIND_PER_POINT);

  nb::enum_<PointCloudShape::PointShapeType>(pointCloudShape, "PointShapeType")
      .value("BOX", PointCloudShape::BOX)
      .value("BILLBOARD_SQUARE", PointCloudShape::BILLBOARD_SQUARE)
      .value("BILLBOARD_CIRCLE", PointCloudShape::BILLBOARD_CIRCLE)
      .value("POINT", PointCloudShape::POINT);
}

} // namespace dart::python_nb
