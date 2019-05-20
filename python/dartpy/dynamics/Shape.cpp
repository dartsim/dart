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
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace dart {
namespace python {

void Shape(pybind11::module& m)
{
  ::pybind11::class_<
      dart::dynamics::Shape,
      dart::common::Subject,
      std::shared_ptr<dart::dynamics::Shape>>(m, "Shape")
      .def(
          "getType",
          +[](const dart::dynamics::Shape* self) -> const std::string& {
            return self->getType();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getBoundingBox",
          +[](const dart::dynamics::Shape* self)
              -> const dart::math::BoundingBox& {
            return self->getBoundingBox();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "computeInertia",
          +[](const dart::dynamics::Shape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::pybind11::arg("mass"))
      .def(
          "computeInertiaFromDensity",
          +[](const dart::dynamics::Shape* self,
              double density) -> Eigen::Matrix3d {
            return self->computeInertiaFromDensity(density);
          },
          ::pybind11::arg("density"))
      .def(
          "computeInertiaFromMass",
          +[](const dart::dynamics::Shape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertiaFromMass(mass); },
          ::pybind11::arg("mass"))
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
          ::pybind11::arg("variance"))
      .def(
          "addDataVariance",
          +[](dart::dynamics::Shape* self, unsigned int _variance) {
            self->addDataVariance(_variance);
          },
          ::pybind11::arg("variance"))
      .def(
          "removeDataVariance",
          +[](dart::dynamics::Shape* self, unsigned int _variance) {
            self->removeDataVariance(_variance);
          },
          ::pybind11::arg("variance"))
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
          ::pybind11::arg("type"))
      .def(
          "refreshData",
          +[](dart::dynamics::Shape* self) { self->refreshData(); })
      .def(
          "notifyAlphaUpdated",
          +[](dart::dynamics::Shape* self, double alpha) {
            self->notifyAlphaUpdated(alpha);
          },
          ::pybind11::arg("alpha"))
      .def(
          "notifyColorUpdated",
          +[](dart::dynamics::Shape* self, const Eigen::Vector4d& color) {
            self->notifyColorUpdated(color);
          },
          ::pybind11::arg("color"))
      .def(
          "incrementVersion",
          +[](dart::dynamics::Shape* self) -> std::size_t {
            return self->incrementVersion();
          })
      .def_readonly(
          "onVersionChanged", &dart::dynamics::Shape::onVersionChanged);

  ::pybind11::class_<
      dart::dynamics::BoxShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::BoxShape>>(m, "BoxShape")
      .def(::pybind11::init<const Eigen::Vector3d&>(), ::pybind11::arg("size"))
      .def(
          "getType",
          +[](const dart::dynamics::BoxShape* self) -> const std::string& {
            return self->getType();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setSize",
          +[](dart::dynamics::BoxShape* self, const Eigen::Vector3d& _size) {
            self->setSize(_size);
          },
          ::pybind11::arg("size"))
      .def(
          "getSize",
          +[](const dart::dynamics::BoxShape* self) -> const Eigen::Vector3d& {
            return self->getSize();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "computeInertia",
          +[](const dart::dynamics::BoxShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::pybind11::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::BoxShape::getStaticType();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def_static(
          "computeVolume",
          +[](const Eigen::Vector3d& size) -> double {
            return dart::dynamics::BoxShape::computeVolume(size);
          },
          ::pybind11::arg("size"))
      .def_static(
          "computeInertiaOf",
          +[](const Eigen::Vector3d& size, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::BoxShape::computeInertia(size, mass);
          },
          ::pybind11::arg("size"),
          ::pybind11::arg("mass"));

  ::pybind11::class_<
      dart::dynamics::ConeShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::ConeShape>>(m, "ConeShape")
      .def(
          ::pybind11::init<double, double>(),
          ::pybind11::arg("radius"),
          ::pybind11::arg("height"))
      .def(
          "getType",
          +[](const dart::dynamics::ConeShape* self) -> const std::string& {
            return self->getType();
          },
          ::pybind11::return_value_policy::reference_internal)
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
          ::pybind11::arg("radius"))
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
          ::pybind11::arg("height"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::ConeShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::pybind11::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::ConeShape::getStaticType();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def_static(
          "computeVolume",
          +[](double radius, double height) -> double {
            return dart::dynamics::ConeShape::computeVolume(radius, height);
          },
          ::pybind11::arg("radius"),
          ::pybind11::arg("height"))
      .def_static(
          "computeInertiaOf",
          +[](double radius, double height, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::ConeShape::computeInertia(
                radius, height, mass);
          },
          ::pybind11::arg("radius"),
          ::pybind11::arg("height"),
          ::pybind11::arg("mass"));

  ::pybind11::class_<
      dart::dynamics::MeshShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::MeshShape>>(m, "MeshShape")
      .def(
          ::pybind11::init<const Eigen::Vector3d&, const aiScene*>(),
          ::pybind11::arg("scale"),
          ::pybind11::arg("mesh"))
      .def(
          ::pybind11::init<
              const Eigen::Vector3d&,
              const aiScene*,
              const dart::common::Uri&>(),
          ::pybind11::arg("scale"),
          ::pybind11::arg("mesh"),
          ::pybind11::arg("uri"))
      .def(
          ::pybind11::init<
              const Eigen::Vector3d&,
              const aiScene*,
              const dart::common::Uri&,
              dart::common::ResourceRetrieverPtr>(),
          ::pybind11::arg("scale"),
          ::pybind11::arg("mesh"),
          ::pybind11::arg("uri"),
          ::pybind11::arg("resourceRetriever"))
      .def(
          "getType",
          +[](const dart::dynamics::MeshShape* self) -> const std::string& {
            return self->getType();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def("update", +[](dart::dynamics::MeshShape* self) { self->update(); })
      .def(
          "notifyAlphaUpdated",
          +[](dart::dynamics::MeshShape* self, double alpha) {
            self->notifyAlphaUpdated(alpha);
          },
          ::pybind11::arg("alpha"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self, const aiScene* mesh) {
            self->setMesh(mesh);
          },
          ::pybind11::arg("mesh"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const std::string& path) { self->setMesh(mesh, path); },
          ::pybind11::arg("mesh"),
          ::pybind11::arg("path"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const std::string& path,
              dart::common::ResourceRetrieverPtr resourceRetriever) {
            self->setMesh(mesh, path, resourceRetriever);
          },
          ::pybind11::arg("mesh"),
          ::pybind11::arg("path"),
          ::pybind11::arg("resourceRetriever"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const dart::common::Uri& path) { self->setMesh(mesh, path); },
          ::pybind11::arg("mesh"),
          ::pybind11::arg("path"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const dart::common::Uri& path,
              dart::common::ResourceRetrieverPtr resourceRetriever) {
            self->setMesh(mesh, path, resourceRetriever);
          },
          ::pybind11::arg("mesh"),
          ::pybind11::arg("path"),
          ::pybind11::arg("resourceRetriever"))
      .def(
          "getMeshUri",
          +[](const dart::dynamics::MeshShape* self) -> std::string {
            return self->getMeshUri();
          })
      .def(
          "getMeshUri2",
          +[](const dart::dynamics::MeshShape* self)
              -> const dart::common::Uri& { return self->getMeshUri2(); },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "getMeshPath",
          +[](const dart::dynamics::MeshShape* self) -> const std::string& {
            return self->getMeshPath();
          },
          ::pybind11::return_value_policy::reference_internal)
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
          ::pybind11::arg("scale"))
      .def(
          "getScale",
          +[](const dart::dynamics::MeshShape* self) -> const Eigen::Vector3d& {
            return self->getScale();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setColorMode",
          +[](dart::dynamics::MeshShape* self,
              dart::dynamics::MeshShape::ColorMode mode) {
            self->setColorMode(mode);
          },
          ::pybind11::arg("mode"))
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
          ::pybind11::arg("index"))
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
          ::pybind11::arg("index"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::MeshShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::pybind11::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::MeshShape::getStaticType();
          },
          ::pybind11::return_value_policy::reference_internal);

  auto attr = m.attr("MeshShape");

  ::pybind11::enum_<dart::dynamics::MeshShape::ColorMode>(attr, "ColorMode")
      .value(
          "MATERIAL_COLOR",
          dart::dynamics::MeshShape::ColorMode::MATERIAL_COLOR)
      .value("COLOR_INDEX", dart::dynamics::MeshShape::ColorMode::COLOR_INDEX)
      .value("SHAPE_COLOR", dart::dynamics::MeshShape::ColorMode::SHAPE_COLOR)
      .export_values();

  ::pybind11::class_<
      dart::dynamics::LineSegmentShape,
      dart::dynamics::Shape,
      std::shared_ptr<dart::dynamics::LineSegmentShape>>(m, "LineSegmentShape")
      .def(::pybind11::init<>())
      .def(::pybind11::init<float>(), ::pybind11::arg("thickness"))
      .def(
          ::pybind11::init<const Eigen::Vector3d&, const Eigen::Vector3d&>(),
          ::pybind11::arg("v1"),
          ::pybind11::arg("v2"))
      .def(
          ::pybind11::
              init<const Eigen::Vector3d&, const Eigen::Vector3d&, float>(),
          ::pybind11::arg("v1"),
          ::pybind11::arg("v2"),
          ::pybind11::arg("thickness"))
      .def(
          "getType",
          +[](const dart::dynamics::LineSegmentShape* self)
              -> const std::string& { return self->getType(); },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "setThickness",
          +[](dart::dynamics::LineSegmentShape* self, float _thickness) {
            self->setThickness(_thickness);
          },
          ::pybind11::arg("thickness"))
      .def(
          "getThickness",
          +[](const dart::dynamics::LineSegmentShape* self) -> float {
            return self->getThickness();
          })
      .def(
          "addVertex",
          +[](dart::dynamics::LineSegmentShape* self, const Eigen::Vector3d& _v)
              -> std::size_t { return self->addVertex(_v); },
          ::pybind11::arg("v"))
      .def(
          "addVertex",
          +[](dart::dynamics::LineSegmentShape* self,
              const Eigen::Vector3d& _v,
              std::size_t _parent) -> std::size_t {
            return self->addVertex(_v, _parent);
          },
          ::pybind11::arg("v"),
          ::pybind11::arg("parent"))
      .def(
          "removeVertex",
          +[](dart::dynamics::LineSegmentShape* self, std::size_t _idx) {
            self->removeVertex(_idx);
          },
          ::pybind11::arg("idx"))
      .def(
          "setVertex",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _idx,
              const Eigen::Vector3d& _v) { self->setVertex(_idx, _v); },
          ::pybind11::arg("idx"),
          ::pybind11::arg("v"))
      .def(
          "getVertex",
          +[](const dart::dynamics::LineSegmentShape* self, std::size_t _idx)
              -> const Eigen::Vector3d& { return self->getVertex(_idx); },
          ::pybind11::return_value_policy::reference_internal,
          ::pybind11::arg("idx"))
      .def(
          "addConnection",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _idx1,
              std::size_t _idx2) { self->addConnection(_idx1, _idx2); },
          ::pybind11::arg("idx1"),
          ::pybind11::arg("idx2"))
      .def(
          "removeConnection",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _vertexIdx1,
              std::size_t _vertexIdx2) {
            self->removeConnection(_vertexIdx1, _vertexIdx2);
          },
          ::pybind11::arg("vertexIdx1"),
          ::pybind11::arg("vertexIdx2"))
      .def(
          "removeConnection",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _connectionIdx) {
            self->removeConnection(_connectionIdx);
          },
          ::pybind11::arg("connectionIdx"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::LineSegmentShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          ::pybind11::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::LineSegmentShape::getStaticType();
          },
          ::pybind11::return_value_policy::reference_internal);
}

} // namespace python
} // namespace dart
