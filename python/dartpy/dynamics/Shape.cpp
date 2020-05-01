/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/dart.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace py = pybind11;

namespace dart {
namespace python {

void Shape(py::module& m)
{
  auto shape
      = ::py::class_<
            dart::dynamics::Shape,
            dart::common::Subject,
            std::shared_ptr<dart::dynamics::Shape>>(m, "Shape")
            .def(
                "getType",
                +[](const dart::dynamics::Shape* self) -> const std::string& {
                  return self->getType();
                },
                ::py::return_value_policy::reference_internal)
            .def(
                "getBoundingBox",
                +[](const dart::dynamics::Shape* self)
                    -> const dart::math::BoundingBox& {
                  return self->getBoundingBox();
                },
                ::py::return_value_policy::reference_internal)
            .def(
                "computeInertia",
                +[](const dart::dynamics::Shape* self, double mass)
                    -> Eigen::Matrix3d { return self->computeInertia(mass); },
                ::py::arg("mass"))
            .def(
                "computeInertiaFromDensity",
                +[](const dart::dynamics::Shape* self,
                    double density) -> Eigen::Matrix3d {
                  return self->computeInertiaFromDensity(density);
                },
                ::py::arg("density"))
            .def(
                "computeInertiaFromMass",
                +[](const dart::dynamics::Shape* self,
                    double mass) -> Eigen::Matrix3d {
                  return self->computeInertiaFromMass(mass);
                },
                ::py::arg("mass"))
            .def(
                "getVolume",
                +[](const dart::dynamics::Shape* self) -> double {
                  return self->getVolume();
                })
            .def(
                "getID",
                +[](const dart::dynamics::Shape* self) -> std::size_t {
                  return self->getID();
                })
            .def(
                "setDataVariance",
                +[](dart::dynamics::Shape* self, unsigned int _variance) {
                  self->setDataVariance(_variance);
                },
                ::py::arg("variance"))
            .def(
                "addDataVariance",
                +[](dart::dynamics::Shape* self, unsigned int _variance) {
                  self->addDataVariance(_variance);
                },
                ::py::arg("variance"))
            .def(
                "removeDataVariance",
                +[](dart::dynamics::Shape* self, unsigned int _variance) {
                  self->removeDataVariance(_variance);
                },
                ::py::arg("variance"))
            .def(
                "getDataVariance",
                +[](const dart::dynamics::Shape* self) -> unsigned int {
                  return self->getDataVariance();
                })
            .def(
                "checkDataVariance",
                +[](const dart::dynamics::Shape* self,
                    dart::dynamics::Shape::DataVariance type) -> bool {
                  return self->checkDataVariance(type);
                },
                ::py::arg("type"))
            .def(
                "refreshData",
                +[](dart::dynamics::Shape* self) { self->refreshData(); })
            .def(
                "notifyAlphaUpdated",
                +[](dart::dynamics::Shape* self, double alpha) {
                  self->notifyAlphaUpdated(alpha);
                },
                ::py::arg("alpha"))
            .def(
                "notifyColorUpdated",
                +[](dart::dynamics::Shape* self, const Eigen::Vector4d& color) {
                  self->notifyColorUpdated(color);
                },
                ::py::arg("color"))
            .def(
                "incrementVersion",
                +[](dart::dynamics::Shape* self) -> std::size_t {
                  return self->incrementVersion();
                })
            .def_readonly(
                "onVersionChanged", &dart::dynamics::Shape::onVersionChanged);

#define DARTPY_DEFINE_SHAPE_TYPE(val)                                          \
  .value(#val, dart::dynamics::Shape::ShapeType::val)

  // clang-format off
  ::py::enum_<dart::dynamics::Shape::ShapeType>(shape, "ShapeType")
      DARTPY_DEFINE_SHAPE_TYPE(SPHERE)
      DARTPY_DEFINE_SHAPE_TYPE(BOX)
      DARTPY_DEFINE_SHAPE_TYPE(ELLIPSOID)
      DARTPY_DEFINE_SHAPE_TYPE(CYLINDER)
      DARTPY_DEFINE_SHAPE_TYPE(CAPSULE)
      DARTPY_DEFINE_SHAPE_TYPE(CONE)
      DARTPY_DEFINE_SHAPE_TYPE(PLANE)
      DARTPY_DEFINE_SHAPE_TYPE(MULTISPHERE)
      DARTPY_DEFINE_SHAPE_TYPE(MESH)
      DARTPY_DEFINE_SHAPE_TYPE(SOFT_MESH)
      DARTPY_DEFINE_SHAPE_TYPE(LINE_SEGMENT)
      DARTPY_DEFINE_SHAPE_TYPE(HEIGHTMAP)
      DARTPY_DEFINE_SHAPE_TYPE(UNSUPPORTED)
      .export_values();
  // clang-format on

#define DARTPY_DEFINE_DATA_VARIANCE(val)                                       \
  .value(#val, dart::dynamics::Shape::DataVariance::val)

  // clang-format off
  ::py::enum_<dart::dynamics::Shape::DataVariance>(shape, "DataVariance")
      DARTPY_DEFINE_DATA_VARIANCE(STATIC           )
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC_TRANSFORM)
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC_PRIMITIVE)
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC_COLOR    )
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC_VERTICES )
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC_ELEMENTS )
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC          )
      .export_values();
  // clang-format on

  ::py::class_<
      dart::dynamics::BoxShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::BoxShape>>(m, "BoxShape")
      .def(::py::init<const Eigen::Vector3d&>(), ::py::arg("size"))
      .def(
          "getType",
          +[](const dart::dynamics::BoxShape* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "setSize",
          +[](dart::dynamics::BoxShape* self, const Eigen::Vector3d& _size) {
            self->setSize(_size);
          },
          ::py::arg("size"))
      .def(
          "getSize",
          +[](const dart::dynamics::BoxShape* self) -> const Eigen::Vector3d& {
            return self->getSize();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "computeInertia",
          +[](const dart::dynamics::BoxShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::BoxShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal)
      .def_static(
          "computeVolume",
          +[](const Eigen::Vector3d& size) -> double {
            return dart::dynamics::BoxShape::computeVolume(size);
          },
          ::py::arg("size"))
      .def_static(
          "computeInertiaOf",
          +[](const Eigen::Vector3d& size, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::BoxShape::computeInertia(size, mass);
          },
          ::py::arg("size"),
          ::py::arg("mass"));

  ::py::class_<
      dart::dynamics::ConeShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::ConeShape>>(m, "ConeShape")
      .def(
          ::py::init<double, double>(),
          ::py::arg("radius"),
          ::py::arg("height"))
      .def(
          "getType",
          +[](const dart::dynamics::ConeShape* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "getRadius",
          +[](const dart::dynamics::ConeShape* self) -> double {
            return self->getRadius();
          })
      .def(
          "setRadius",
          +[](dart::dynamics::ConeShape* self, double radius) {
            self->setRadius(radius);
          },
          ::py::arg("radius"))
      .def(
          "getHeight",
          +[](const dart::dynamics::ConeShape* self) -> double {
            return self->getHeight();
          })
      .def(
          "setHeight",
          +[](dart::dynamics::ConeShape* self, double height) {
            self->setHeight(height);
          },
          ::py::arg("height"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::ConeShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::ConeShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal)
      .def_static(
          "computeVolume",
          +[](double radius, double height) -> double {
            return dart::dynamics::ConeShape::computeVolume(radius, height);
          },
          ::py::arg("radius"),
          ::py::arg("height"))
      .def_static(
          "computeInertiaOf",
          +[](double radius, double height, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::ConeShape::computeInertia(
                radius, height, mass);
          },
          ::py::arg("radius"),
          ::py::arg("height"),
          ::py::arg("mass"));

  ::py::class_<
      dart::dynamics::MeshShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::MeshShape>>(m, "MeshShape")
      .def(
          ::py::init<const Eigen::Vector3d&, const aiScene*>(),
          ::py::arg("scale"),
          ::py::arg("mesh"))
      .def(
          ::py::init<
              const Eigen::Vector3d&,
              const aiScene*,
              const dart::common::Uri&>(),
          ::py::arg("scale"),
          ::py::arg("mesh"),
          ::py::arg("uri"))
      .def(
          ::py::init<
              const Eigen::Vector3d&,
              const aiScene*,
              const dart::common::Uri&,
              dart::common::ResourceRetrieverPtr>(),
          ::py::arg("scale"),
          ::py::arg("mesh"),
          ::py::arg("uri"),
          ::py::arg("resourceRetriever"))
      .def(
          "getType",
          +[](const dart::dynamics::MeshShape* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "update", +[](dart::dynamics::MeshShape* self) { self->update(); })
      .def(
          "notifyAlphaUpdated",
          +[](dart::dynamics::MeshShape* self, double alpha) {
            self->notifyAlphaUpdated(alpha);
          },
          ::py::arg("alpha"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self, const aiScene* mesh) {
            self->setMesh(mesh);
          },
          ::py::arg("mesh"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const std::string& path) { self->setMesh(mesh, path); },
          ::py::arg("mesh"),
          ::py::arg("path"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const std::string& path,
              dart::common::ResourceRetrieverPtr resourceRetriever) {
            self->setMesh(mesh, path, resourceRetriever);
          },
          ::py::arg("mesh"),
          ::py::arg("path"),
          ::py::arg("resourceRetriever"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const dart::common::Uri& path) { self->setMesh(mesh, path); },
          ::py::arg("mesh"),
          ::py::arg("path"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const dart::common::Uri& path,
              dart::common::ResourceRetrieverPtr resourceRetriever) {
            self->setMesh(mesh, path, resourceRetriever);
          },
          ::py::arg("mesh"),
          ::py::arg("path"),
          ::py::arg("resourceRetriever"))
      .def(
          "getMeshUri",
          +[](const dart::dynamics::MeshShape* self) -> std::string {
            return self->getMeshUri();
          })
      .def(
          "getMeshUri2",
          +[](const dart::dynamics::MeshShape* self)
              -> const dart::common::Uri& { return self->getMeshUri2(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "getMeshPath",
          +[](const dart::dynamics::MeshShape* self) -> const std::string& {
            return self->getMeshPath();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "getResourceRetriever",
          +[](dart::dynamics::MeshShape* self)
              -> dart::common::ResourceRetrieverPtr {
            return self->getResourceRetriever();
          })
      .def(
          "setScale",
          +[](dart::dynamics::MeshShape* self, const Eigen::Vector3d& scale) {
            self->setScale(scale);
          },
          ::py::arg("scale"))
      .def(
          "getScale",
          +[](const dart::dynamics::MeshShape* self) -> const Eigen::Vector3d& {
            return self->getScale();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "setColorMode",
          +[](dart::dynamics::MeshShape* self,
              dart::dynamics::MeshShape::ColorMode mode) {
            self->setColorMode(mode);
          },
          ::py::arg("mode"))
      .def(
          "getColorMode",
          +[](const dart::dynamics::MeshShape* self)
              -> dart::dynamics::MeshShape::ColorMode {
            return self->getColorMode();
          })
      .def(
          "setColorIndex",
          +[](dart::dynamics::MeshShape* self, int index) {
            self->setColorIndex(index);
          },
          ::py::arg("index"))
      .def(
          "getColorIndex",
          +[](const dart::dynamics::MeshShape* self) -> int {
            return self->getColorIndex();
          })
      .def(
          "getDisplayList",
          +[](const dart::dynamics::MeshShape* self) -> int {
            return self->getDisplayList();
          })
      .def(
          "setDisplayList",
          +[](dart::dynamics::MeshShape* self, int index) {
            self->setDisplayList(index);
          },
          ::py::arg("index"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::MeshShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::MeshShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal);

  auto attr = m.attr("MeshShape");

  ::py::enum_<dart::dynamics::MeshShape::ColorMode>(attr, "ColorMode")
      .value(
          "MATERIAL_COLOR",
          dart::dynamics::MeshShape::ColorMode::MATERIAL_COLOR)
      .value("COLOR_INDEX", dart::dynamics::MeshShape::ColorMode::COLOR_INDEX)
      .value("SHAPE_COLOR", dart::dynamics::MeshShape::ColorMode::SHAPE_COLOR)
      .export_values();

  ::py::class_<
      dart::dynamics::ArrowShape,
      dart::dynamics::MeshShape,
      std::shared_ptr<dart::dynamics::ArrowShape>>(m, "ArrowShape")
      .def(
          ::py::init<const Eigen::Vector3d&, const Eigen::Vector3d&>(),
          ::py::arg("tail"),
          ::py::arg("head"))
      .def(
          ::py::init<
              const Eigen::Vector3d&,
              const Eigen::Vector3d&,
              const dart::dynamics::ArrowShape::Properties&>(),
          ::py::arg("tail"),
          ::py::arg("head"),
          ::py::arg("properties"))
      .def(
          ::py::init<
              const Eigen::Vector3d&,
              const Eigen::Vector3d&,
              const dart::dynamics::ArrowShape::Properties&,
              const Eigen::Vector4d&>(),
          ::py::arg("tail"),
          ::py::arg("head"),
          ::py::arg("properties"),
          ::py::arg("color"))
      .def(
          ::py::init<
              const Eigen::Vector3d&,
              const Eigen::Vector3d&,
              const dart::dynamics::ArrowShape::Properties&,
              const Eigen::Vector4d&,
              std::size_t>(),
          ::py::arg("tail"),
          ::py::arg("head"),
          ::py::arg("properties"),
          ::py::arg("color"),
          ::py::arg("resolution"))
      .def(
          "setPositions",
          +[](dart::dynamics::ArrowShape* self,
              const Eigen::Vector3d& _tail,
              const Eigen::Vector3d& _head) {
            self->setPositions(_tail, _head);
          },
          ::py::arg("tail"),
          ::py::arg("head"))
      .def(
          "getTail",
          +[](const dart::dynamics::ArrowShape* self)
              -> const Eigen::Vector3d& { return self->getTail(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "getHead",
          +[](const dart::dynamics::ArrowShape* self)
              -> const Eigen::Vector3d& { return self->getHead(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "setProperties",
          +[](dart::dynamics::ArrowShape* self,
              const dart::dynamics::ArrowShape::Properties& _properties) {
            self->setProperties(_properties);
          },
          ::py::arg("properties"))
      .def(
          "notifyColorUpdated",
          +[](dart::dynamics::ArrowShape* self, const Eigen::Vector4d& _color) {
            self->notifyColorUpdated(_color);
          },
          ::py::arg("color"))
      .def(
          "getProperties",
          +[](const dart::dynamics::ArrowShape* self)
              -> const dart::dynamics::ArrowShape::Properties& {
            return self->getProperties();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "configureArrow",
          +[](dart::dynamics::ArrowShape* self,
              const Eigen::Vector3d& _tail,
              const Eigen::Vector3d& _head,
              const dart::dynamics::ArrowShape::Properties& _properties) {
            self->configureArrow(_tail, _head, _properties);
          },
          ::py::arg("tail"),
          ::py::arg("head"),
          ::py::arg("properties"));

  ::py::class_<dart::dynamics::ArrowShape::Properties>(
      m, "ArrowShapeProperties")
      .def(::py::init<>())
      .def(::py::init<double>(), ::py::arg("radius"))
      .def(
          ::py::init<double, double>(),
          ::py::arg("radius"),
          ::py::arg("headRadiusScale"))
      .def(
          ::py::init<double, double, double>(),
          ::py::arg("radius"),
          ::py::arg("headRadiusScale"),
          ::py::arg("headLengthScale"))
      .def(
          ::py::init<double, double, double, double>(),
          ::py::arg("radius"),
          ::py::arg("headRadiusScale"),
          ::py::arg("headLengthScale"),
          ::py::arg("minHeadLength"))
      .def(
          ::py::init<double, double, double, double, double>(),
          ::py::arg("radius"),
          ::py::arg("headRadiusScale"),
          ::py::arg("headLengthScale"),
          ::py::arg("minHeadLength"),
          ::py::arg("maxHeadLength"))
      .def(
          ::py::init<double, double, double, double, double, bool>(),
          ::py::arg("radius"),
          ::py::arg("headRadiusScale"),
          ::py::arg("headLengthScale"),
          ::py::arg("minHeadLength"),
          ::py::arg("maxHeadLength"),
          ::py::arg("doubleArrow"))
      .def_readwrite(
          "mRadius", &dart::dynamics::ArrowShape::Properties::mRadius)
      .def_readwrite(
          "mHeadRadiusScale",
          &dart::dynamics::ArrowShape::Properties::mHeadRadiusScale)
      .def_readwrite(
          "mHeadLengthScale",
          &dart::dynamics::ArrowShape::Properties::mHeadLengthScale)
      .def_readwrite(
          "mMinHeadLength",
          &dart::dynamics::ArrowShape::Properties::mMinHeadLength)
      .def_readwrite(
          "mMaxHeadLength",
          &dart::dynamics::ArrowShape::Properties::mMaxHeadLength)
      .def_readwrite(
          "mDoubleArrow",
          &dart::dynamics::ArrowShape::Properties::mDoubleArrow);

  ::py::class_<
      dart::dynamics::PlaneShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::PlaneShape>>(m, "PlaneShape")
      .def(
          ::py::init<const Eigen::Vector3d&, double>(),
          ::py::arg("normal"),
          ::py::arg("offset"))
      .def(
          ::py::init<const Eigen::Vector3d&, const Eigen::Vector3d&>(),
          ::py::arg("normal"),
          ::py::arg("point"))
      .def(
          "getType",
          +[](const dart::dynamics::PlaneShape* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "computeInertia",
          +[](const dart::dynamics::PlaneShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def(
          "setNormal",
          +[](dart::dynamics::PlaneShape* self,
              const Eigen::Vector3d& _normal) { self->setNormal(_normal); },
          ::py::arg("normal"))
      .def(
          "getNormal",
          +[](const dart::dynamics::PlaneShape* self)
              -> const Eigen::Vector3d& { return self->getNormal(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "setOffset",
          +[](dart::dynamics::PlaneShape* self, double _offset) {
            self->setOffset(_offset);
          },
          ::py::arg("offset"))
      .def(
          "getOffset",
          +[](const dart::dynamics::PlaneShape* self) -> double {
            return self->getOffset();
          })
      .def(
          "setNormalAndOffset",
          +[](dart::dynamics::PlaneShape* self,
              const Eigen::Vector3d& _normal,
              double _offset) { self->setNormalAndOffset(_normal, _offset); },
          ::py::arg("normal"),
          ::py::arg("offset"))
      .def(
          "setNormalAndPoint",
          +[](dart::dynamics::PlaneShape* self,
              const Eigen::Vector3d& _normal,
              const Eigen::Vector3d& _point) {
            self->setNormalAndPoint(_normal, _point);
          },
          ::py::arg("normal"),
          ::py::arg("point"))
      .def(
          "computeDistance",
          +[](const dart::dynamics::PlaneShape* self,
              const Eigen::Vector3d& _point) -> double {
            return self->computeDistance(_point);
          },
          ::py::arg("point"))
      .def(
          "computeSignedDistance",
          +[](const dart::dynamics::PlaneShape* self,
              const Eigen::Vector3d& _point) -> double {
            return self->computeSignedDistance(_point);
          },
          ::py::arg("point"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::PlaneShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal);
  ::py::class_<
      dart::dynamics::PointCloudShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::PointCloudShape>>
      pointCloudShape(m, "PointCloudShape");

  pointCloudShape.def(::py::init<double>(), ::py::arg("visualSize") = 0.01)
      .def(
          "getType",
          +[](const dart::dynamics::PointCloudShape* self)
              -> const std::string& { return self->getType(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "computeInertia",
          +[](const dart::dynamics::PointCloudShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def(
          "reserve",
          +[](dart::dynamics::PointCloudShape* self, std::size_t size) -> void {
            return self->reserve(size);
          },
          ::py::arg("size"))
      .def(
          "addPoint",
          +[](dart::dynamics::PointCloudShape* self,
              const Eigen::Vector3d& point) -> void {
            return self->addPoint(point);
          },
          ::py::arg("point"))
      .def(
          "addPoint",
          +[](dart::dynamics::PointCloudShape* self,
              const std::vector<Eigen::Vector3d>& points) -> void {
            return self->addPoint(points);
          },
          ::py::arg("points"))
      .def(
          "setPoint",
          +[](dart::dynamics::PointCloudShape* self,
              const std::vector<Eigen::Vector3d>& points) -> void {
            return self->setPoint(points);
          },
          ::py::arg("points"))
      .def(
          "getPoints",
          +[](const dart::dynamics::PointCloudShape* self)
              -> const std::vector<Eigen::Vector3d>& {
            return self->getPoints();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "getNumPoints",
          +[](const dart::dynamics::PointCloudShape* self) -> std::size_t {
            return self->getNumPoints();
          })
      .def(
          "removeAllPoints",
          +[](dart::dynamics::PointCloudShape* self) -> void {
            return self->removeAllPoints();
          })
      .def(
          "setPointShapeType",
          +[](dart::dynamics::PointCloudShape* self,
              dart::dynamics::PointCloudShape::PointShapeType type) -> void {
            return self->setPointShapeType(type);
          },
          ::py::arg("type"))
      .def(
          "getPointShapeType",
          +[](const dart::dynamics::PointCloudShape* self)
              -> dart::dynamics::PointCloudShape::PointShapeType {
            return self->getPointShapeType();
          })
      .def(
          "setColorMode",
          +[](dart::dynamics::PointCloudShape* self,
              dart::dynamics::PointCloudShape::ColorMode mode) -> void {
            return self->setColorMode(mode);
          },
          ::py::arg("mode"))
      .def(
          "getColorMode",
          +[](const dart::dynamics::PointCloudShape* self)
              -> dart::dynamics::PointCloudShape::ColorMode {
            return self->getColorMode();
          })
      .def(
          "setOverallColor",
          +[](dart::dynamics::PointCloudShape* self,
              const Eigen::Vector4d& color) -> void {
            return self->setOverallColor(color);
          },
          ::py::arg("color"))
      .def(
          "getOverallColor",
          +[](const dart::dynamics::PointCloudShape* self) -> Eigen::Vector4d {
            return self->getOverallColor();
          })
      .def(
          "setColors",
          +[](dart::dynamics::PointCloudShape* self,
              const std::vector<
                  Eigen::Vector4d,
                  Eigen::aligned_allocator<Eigen::Vector4d>>& colors) -> void {
            return self->setColors(colors);
          },
          ::py::arg("colors"))
      .def(
          "getColors",
          +[](const dart::dynamics::PointCloudShape* self)
              -> const std::vector<
                  Eigen::Vector4d,
                  Eigen::aligned_allocator<Eigen::Vector4d>>& {
            return self->getColors();
          })
      .def(
          "setVisualSize",
          +[](dart::dynamics::PointCloudShape* self, double size) -> void {
            return self->setVisualSize(size);
          },
          ::py::arg("size"))
      .def(
          "getVisualSize",
          +[](const dart::dynamics::PointCloudShape* self) -> double {
            return self->getVisualSize();
          })
      .def(
          "notifyColorUpdated",
          +[](dart::dynamics::PointCloudShape* self,
              const Eigen::Vector4d& color) {
            self->notifyColorUpdated(color);
          },
          ::py::arg("color"));

  ::py::enum_<dart::dynamics::PointCloudShape::ColorMode>(
      pointCloudShape, "ColorMode")
      .value(
          "USE_SHAPE_COLOR",
          dart::dynamics::PointCloudShape::ColorMode::USE_SHAPE_COLOR)
      .value(
          "BIND_OVERALL",
          dart::dynamics::PointCloudShape::ColorMode::BIND_OVERALL)
      .value(
          "BIND_PER_POINT",
          dart::dynamics::PointCloudShape::ColorMode::BIND_PER_POINT)
      .export_values();

  ::py::enum_<dart::dynamics::PointCloudShape::PointShapeType>(
      pointCloudShape, "PointShapeType")
      .value("BOX", dart::dynamics::PointCloudShape::PointShapeType::BOX)
      .value(
          "BILLBOARD_SQUARE",
          dart::dynamics::PointCloudShape::PointShapeType::BILLBOARD_SQUARE)
      .value(
          "BILLBOARD_CIRCLE",
          dart::dynamics::PointCloudShape::PointShapeType::BILLBOARD_CIRCLE)
      .export_values();

  ::py::class_<
      dart::dynamics::SphereShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::SphereShape>>(m, "SphereShape")
      .def(::py::init<double>(), ::py::arg("radius"))
      .def(
          "getType",
          +[](const dart::dynamics::SphereShape* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "setRadius",
          +[](dart::dynamics::SphereShape* self, double radius) {
            self->setRadius(radius);
          },
          ::py::arg("radius"))
      .def(
          "getRadius",
          +[](const dart::dynamics::SphereShape* self) -> double {
            return self->getRadius();
          })
      .def(
          "computeInertia",
          +[](const dart::dynamics::SphereShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::SphereShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal)
      .def_static(
          "computeVolumeOf",
          +[](double radius) -> double {
            return dart::dynamics::SphereShape::computeVolume(radius);
          },
          ::py::arg("radius"))
      .def_static(
          "computeInertiaOf",
          +[](double radius, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::SphereShape::computeInertia(radius, mass);
          },
          ::py::arg("radius"),
          ::py::arg("mass"));

  ::py::class_<
      dart::dynamics::CapsuleShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::CapsuleShape>>(m, "CapsuleShape")
      .def(
          ::py::init<double, double>(),
          ::py::arg("radius"),
          ::py::arg("height"))
      .def(
          "getType",
          +[](const dart::dynamics::CapsuleShape* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "getRadius",
          +[](const dart::dynamics::CapsuleShape* self) -> double {
            return self->getRadius();
          })
      .def(
          "setRadius",
          +[](dart::dynamics::CapsuleShape* self, double radius) {
            self->setRadius(radius);
          },
          ::py::arg("radius"))
      .def(
          "getHeight",
          +[](const dart::dynamics::CapsuleShape* self) -> double {
            return self->getHeight();
          })
      .def(
          "setHeight",
          +[](dart::dynamics::CapsuleShape* self, double height) {
            self->setHeight(height);
          },
          ::py::arg("height"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::CapsuleShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::CapsuleShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal)
      .def_static(
          "computeVolumeOf",
          +[](double radius, double height) -> double {
            return dart::dynamics::CapsuleShape::computeVolume(radius, height);
          },
          ::py::arg("radius"),
          ::py::arg("height"))
      .def_static(
          "computeInertiaOf",
          +[](double radius, double height, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::CapsuleShape::computeInertia(
                radius, height, mass);
          },
          ::py::arg("radius"),
          ::py::arg("height"),
          ::py::arg("mass"));

  ::py::class_<
      dart::dynamics::CylinderShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::CylinderShape>>(m, "CylinderShape")
      .def(
          ::py::init<double, double>(),
          ::py::arg("radius"),
          ::py::arg("height"))
      .def(
          "getType",
          +[](const dart::dynamics::CylinderShape* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "getRadius",
          +[](const dart::dynamics::CylinderShape* self) -> double {
            return self->getRadius();
          })
      .def(
          "setRadius",
          +[](dart::dynamics::CylinderShape* self, double _radius) {
            self->setRadius(_radius);
          },
          ::py::arg("radius"))
      .def(
          "getHeight",
          +[](const dart::dynamics::CylinderShape* self) -> double {
            return self->getHeight();
          })
      .def(
          "setHeight",
          +[](dart::dynamics::CylinderShape* self, double _height) {
            self->setHeight(_height);
          },
          ::py::arg("height"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::CylinderShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::CylinderShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal)
      .def_static(
          "computeVolumeOf",
          +[](double radius, double height) -> double {
            return dart::dynamics::CylinderShape::computeVolume(radius, height);
          },
          ::py::arg("radius"),
          ::py::arg("height"))
      .def_static(
          "computeInertiaOf",
          +[](double radius, double height, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::CylinderShape::computeInertia(
                radius, height, mass);
          },
          ::py::arg("radius"),
          ::py::arg("height"),
          ::py::arg("mass"));

  ::py::class_<
      dart::dynamics::SoftMeshShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::SoftMeshShape>>(m, "SoftMeshShape")
      .def(
          ::py::init<dart::dynamics::SoftBodyNode*>(),
          ::py::arg("softBodyNode"))
      .def(
          "getType",
          +[](const dart::dynamics::SoftMeshShape* self) -> const std::string& {
            return self->getType();
          },
          ::py::return_value_policy::reference_internal)
      .def(
          "update",
          +[](dart::dynamics::SoftMeshShape* self) { self->update(); })
      .def(
          "computeInertia",
          +[](const dart::dynamics::SoftMeshShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::SoftMeshShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal);

  ::py::class_<
      dart::dynamics::EllipsoidShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::EllipsoidShape>>(m, "EllipsoidShape")
      .def(::py::init<const Eigen::Vector3d&>(), ::py::arg("diameters"))
      .def(
          "getType",
          +[](const dart::dynamics::EllipsoidShape* self)
              -> const std::string& { return self->getType(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "setDiameters",
          +[](dart::dynamics::EllipsoidShape* self,
              const Eigen::Vector3d& diameters) {
            self->setDiameters(diameters);
          },
          ::py::arg("diameters"))
      .def(
          "getDiameters",
          +[](const dart::dynamics::EllipsoidShape* self)
              -> const Eigen::Vector3d& { return self->getDiameters(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "setRadii",
          +[](dart::dynamics::EllipsoidShape* self,
              const Eigen::Vector3d& radii) { self->setRadii(radii); },
          ::py::arg("radii"))
      .def(
          "getRadii",
          +[](const dart::dynamics::EllipsoidShape* self)
              -> const Eigen::Vector3d { return self->getRadii(); })
      .def(
          "computeInertia",
          +[](const dart::dynamics::EllipsoidShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def(
          "isSphere",
          +[](const dart::dynamics::EllipsoidShape* self) -> bool {
            return self->isSphere();
          })
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::EllipsoidShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal)
      .def_static(
          "computeVolumeOf",
          +[](const Eigen::Vector3d& diameters) -> double {
            return dart::dynamics::EllipsoidShape::computeVolume(diameters);
          },
          ::py::arg("diameters"))
      .def_static(
          "computeInertiaOf",
          +[](const Eigen::Vector3d& diameters,
              double mass) -> Eigen::Matrix3d {
            return dart::dynamics::EllipsoidShape::computeInertia(
                diameters, mass);
          },
          ::py::arg("diameters"),
          ::py::arg("mass"));

  ::py::class_<
      dart::dynamics::LineSegmentShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::LineSegmentShape>>(m, "LineSegmentShape")
      .def(::py::init<>())
      .def(::py::init<float>(), ::py::arg("thickness"))
      .def(
          ::py::init<const Eigen::Vector3d&, const Eigen::Vector3d&>(),
          ::py::arg("v1"),
          ::py::arg("v2"))
      .def(
          ::py::init<const Eigen::Vector3d&, const Eigen::Vector3d&, float>(),
          ::py::arg("v1"),
          ::py::arg("v2"),
          ::py::arg("thickness"))
      .def(
          "getType",
          +[](const dart::dynamics::LineSegmentShape* self)
              -> const std::string& { return self->getType(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "setThickness",
          +[](dart::dynamics::LineSegmentShape* self, float _thickness) {
            self->setThickness(_thickness);
          },
          ::py::arg("thickness"))
      .def(
          "getThickness",
          +[](const dart::dynamics::LineSegmentShape* self) -> float {
            return self->getThickness();
          })
      .def(
          "addVertex",
          +[](dart::dynamics::LineSegmentShape* self, const Eigen::Vector3d& _v)
              -> std::size_t { return self->addVertex(_v); },
          ::py::arg("v"))
      .def(
          "addVertex",
          +[](dart::dynamics::LineSegmentShape* self,
              const Eigen::Vector3d& _v,
              std::size_t _parent) -> std::size_t {
            return self->addVertex(_v, _parent);
          },
          ::py::arg("v"),
          ::py::arg("parent"))
      .def(
          "removeVertex",
          +[](dart::dynamics::LineSegmentShape* self, std::size_t _idx) {
            self->removeVertex(_idx);
          },
          ::py::arg("idx"))
      .def(
          "setVertex",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _idx,
              const Eigen::Vector3d& _v) { self->setVertex(_idx, _v); },
          ::py::arg("idx"),
          ::py::arg("v"))
      .def(
          "getVertex",
          +[](const dart::dynamics::LineSegmentShape* self, std::size_t _idx)
              -> const Eigen::Vector3d& { return self->getVertex(_idx); },
          ::py::return_value_policy::reference_internal,
          ::py::arg("idx"))
      .def(
          "addConnection",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _idx1,
              std::size_t _idx2) { self->addConnection(_idx1, _idx2); },
          ::py::arg("idx1"),
          ::py::arg("idx2"))
      .def(
          "removeConnection",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _vertexIdx1,
              std::size_t _vertexIdx2) {
            self->removeConnection(_vertexIdx1, _vertexIdx2);
          },
          ::py::arg("vertexIdx1"),
          ::py::arg("vertexIdx2"))
      .def(
          "removeConnection",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _connectionIdx) {
            self->removeConnection(_connectionIdx);
          },
          ::py::arg("connectionIdx"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::LineSegmentShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::py::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::LineSegmentShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal);

  ::py::class_<
      dart::dynamics::MultiSphereConvexHullShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::MultiSphereConvexHullShape>>(
      m, "MultiSphereConvexHullShape")
      .def(
          ::py::init<
              const dart::dynamics::MultiSphereConvexHullShape::Spheres&>(),
          ::py::arg("spheres"))
      .def(
          "getType",
          +[](const dart::dynamics::MultiSphereConvexHullShape* self)
              -> const std::string& { return self->getType(); },
          ::py::return_value_policy::reference_internal)
      .def(
          "addSpheres",
          +[](dart::dynamics::MultiSphereConvexHullShape* self,
              const dart::dynamics::MultiSphereConvexHullShape::Spheres&
                  spheres) { self->addSpheres(spheres); },
          ::py::arg("spheres"))
      .def(
          "addSphere",
          +[](dart::dynamics::MultiSphereConvexHullShape* self,
              const dart::dynamics::MultiSphereConvexHullShape::Sphere&
                  sphere) { self->addSphere(sphere); },
          ::py::arg("sphere"))
      .def(
          "addSphere",
          +[](dart::dynamics::MultiSphereConvexHullShape* self,
              double radius,
              const Eigen::Vector3d& position) {
            self->addSphere(radius, position);
          },
          ::py::arg("radius"),
          ::py::arg("position"))
      .def(
          "removeAllSpheres",
          +[](dart::dynamics::MultiSphereConvexHullShape* self) {
            self->removeAllSpheres();
          })
      .def(
          "getNumSpheres",
          +[](const dart::dynamics::MultiSphereConvexHullShape* self)
              -> std::size_t { return self->getNumSpheres(); })
      .def(
          "computeInertia",
          +[](const dart::dynamics::MultiSphereConvexHullShape* self,
              double mass) -> Eigen::Matrix3d {
            return self->computeInertia(mass);
          },
          ::py::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::MultiSphereConvexHullShape::getStaticType();
          },
          ::py::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
