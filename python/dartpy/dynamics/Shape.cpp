// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/stl/pair.h>
#include <nanobind/stl/vector.h>

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

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/dynamics/ArrowShape.hpp>
#include <dart/dynamics/BoxShape.hpp>
#include <dart/dynamics/CapsuleShape.hpp>
#include <dart/dynamics/ConeShape.hpp>
#include <dart/dynamics/CylinderShape.hpp>
#include <dart/dynamics/EllipsoidShape.hpp>
#include <dart/dynamics/LineSegmentShape.hpp>
#include <dart/dynamics/MeshShape.hpp>
#include <dart/dynamics/MultiSphereConvexHullShape.hpp>
#include <dart/dynamics/PlaneShape.hpp>
#include <dart/dynamics/PointCloudShape.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/SoftBodyNode.hpp>
#include <dart/dynamics/SoftMeshShape.hpp>
#include <dart/dynamics/SphereShape.hpp>

#include <dart/math/Geometry.hpp>

#include <dart/common/Deprecated.hpp>
#include <dart/common/Macros.hpp>
#include <dart/common/ResourceRetriever.hpp>
#include <dart/common/Subject.hpp>
#include <dart/common/Uri.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

#include <cstddef>

namespace dart {
namespace python {

DART_SUPPRESS_DEPRECATED_BEGIN

void Shape(nb::module_& m)
{
  static_assert(dartnb::GcOwner<dart::dynamics::MeshShape>::value);
  static_assert(dartnb::GcOwner<dart::dynamics::SoftMeshShape>::value);
  auto shape
      = dartnb::dart_class<dart::dynamics::Shape, dart::common::Subject>(
            m, "Shape")
            .def_static(
                "__new__",
                [](nb::handle requested, nb::args, nb::kwargs) {
                  if (!nb::type_check(requested)
                      || nb::type_info(requested)
                             != typeid(dart::dynamics::Shape))
                    throw nb::type_error("incompatible construction type");
                  if (requested.is(nb::type<dart::dynamics::Shape>()))
                    throw nb::type_error("Shape has no constructor");
                  nb::module_::import_("dartpy").attr("_guard_init")(requested);
                  return nb::inst_alloc(requested);
                })
            .def(
                "getType",
                +[](const dart::dynamics::Shape* self) -> const std::string& {
                  return self->getType();
                },
                nb::rv_policy::reference_internal)
            .def(
                "getBoundingBox",
                +[](const dart::dynamics::Shape* self)
                    -> dart::math::BoundingBox {
                  return self->getBoundingBox();
                })
            .def(
                "computeInertia",
                +[](const dart::dynamics::Shape* self, double mass)
                    -> Eigen::Matrix3d { return self->computeInertia(mass); },
                nb::arg("mass"))
            .def(
                "computeInertiaFromDensity",
                +[](const dart::dynamics::Shape* self,
                    double density) -> Eigen::Matrix3d {
                  return self->computeInertiaFromDensity(density);
                },
                nb::arg("density"))
            .def(
                "computeInertiaFromMass",
                +[](const dart::dynamics::Shape* self,
                    double mass) -> Eigen::Matrix3d {
                  return self->computeInertiaFromMass(mass);
                },
                nb::arg("mass"))
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
                nb::arg("variance"))
            .def(
                "addDataVariance",
                +[](dart::dynamics::Shape* self, unsigned int _variance) {
                  self->addDataVariance(_variance);
                },
                nb::arg("variance"))
            .def(
                "removeDataVariance",
                +[](dart::dynamics::Shape* self, unsigned int _variance) {
                  self->removeDataVariance(_variance);
                },
                nb::arg("variance"))
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
                nb::arg("type"))
            .def(
                "refreshData",
                +[](dart::dynamics::Shape* self) { self->refreshData(); })
            .def(
                "notifyAlphaUpdated",
                +[](dart::dynamics::Shape* self, double alpha) {
                  self->notifyAlphaUpdated(alpha);
                },
                nb::arg("alpha"))
            .def(
                "notifyColorUpdated",
                +[](dart::dynamics::Shape* self, const Eigen::Vector4d& color) {
                  self->notifyColorUpdated(color);
                },
                nb::arg("color"))
            .def(
                "incrementVersion",
                +[](dart::dynamics::Shape* self) -> std::size_t {
                  return self->incrementVersion();
                })
            .def_ro(
                "onVersionChanged", &dart::dynamics::Shape::onVersionChanged);

#define DARTPY_DEFINE_SHAPE_TYPE(val)                                          \
  .value(#val, dart::dynamics::Shape::ShapeType::val)

  // clang-format off
  nb::enum_<dart::dynamics::Shape::ShapeType>(shape, "ShapeType", nb::is_arithmetic())
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
  nb::enum_<dart::dynamics::Shape::DataVariance>(shape, "DataVariance", nb::is_arithmetic(), nb::is_flag())
      DARTPY_DEFINE_DATA_VARIANCE(STATIC           )
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC_TRANSFORM)
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC_PRIMITIVE)
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC_COLOR    )
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC_VERTICES )
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC_ELEMENTS )
      DARTPY_DEFINE_DATA_VARIANCE(DYNAMIC          )
      .export_values();
  // clang-format on

  dartnb::dart_class<dart::dynamics::BoxShape, dart::dynamics::Shape>(
      m, "BoxShape")
      .def(dartnb::init<const Eigen::Vector3d&>(), nb::arg("size"))
      .def(
          "getType",
          +[](const dart::dynamics::BoxShape* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setSize",
          +[](dart::dynamics::BoxShape* self, const Eigen::Vector3d& _size) {
            self->setSize(_size);
          },
          nb::arg("size"))
      .def(
          "getSize",
          +[](const dart::dynamics::BoxShape* self) -> const Eigen::Vector3d& {
            return self->getSize();
          },
          nb::rv_policy::reference_internal)
      .def(
          "computeInertia",
          +[](const dart::dynamics::BoxShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::BoxShape::getStaticType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "computeVolume",
          +[](const Eigen::Vector3d& size) -> double {
            return dart::dynamics::BoxShape::computeVolume(size);
          },
          nb::arg("size"))
      .def_static(
          "computeInertiaOf",
          +[](const Eigen::Vector3d& size, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::BoxShape::computeInertia(size, mass);
          },
          nb::arg("size"),
          nb::arg("mass"));

  dartnb::dart_class<dart::dynamics::ConeShape, dart::dynamics::Shape>(
      m, "ConeShape")
      .def(dartnb::init<double, double>(), nb::arg("radius"), nb::arg("height"))
      .def(
          "getType",
          +[](const dart::dynamics::ConeShape* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
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
          nb::arg("radius"))
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
          nb::arg("height"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::ConeShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::ConeShape::getStaticType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "computeVolume",
          +[](double radius, double height) -> double {
            return dart::dynamics::ConeShape::computeVolume(radius, height);
          },
          nb::arg("radius"),
          nb::arg("height"))
      .def_static(
          "computeInertiaOf",
          +[](double radius, double height, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::ConeShape::computeInertia(
                radius, height, mass);
          },
          nb::arg("radius"),
          nb::arg("height"),
          nb::arg("mass"));

  // MeshShape's Assimp (aiScene/aiMesh) constructors and setters are kept as
  // deprecated DART 6 compatibility shims (TriMesh APIs are the replacement).
  // Binding them deliberately uses those deprecated APIs, so suppress
  // -Wdeprecated-declarations here; otherwise -Werror builds (e.g. macOS
  // arm64) fail compiling this binding.
  DART_SUPPRESS_DEPRECATED_BEGIN
  dartnb::dart_class<dart::dynamics::MeshShape, dart::dynamics::Shape>(
      m, "MeshShape")
      .def(
          dartnb::factory(
              +[](const Eigen::Vector3d& scale, const aiScene* mesh) {
                DART_SUPPRESS_DEPRECATED_BEGIN
                auto shape
                    = std::make_shared<dart::dynamics::MeshShape>(scale, mesh);
                DART_SUPPRESS_DEPRECATED_END
                return shape;
              }),
          nb::arg("scale"),
          nb::arg("mesh").none())
      .def(
          dartnb::factory(+[](const Eigen::Vector3d& scale,
                              const aiScene* mesh,
                              const dart::common::Uri& uri) {
            DART_SUPPRESS_DEPRECATED_BEGIN
            auto shape
                = std::make_shared<dart::dynamics::MeshShape>(scale, mesh, uri);
            DART_SUPPRESS_DEPRECATED_END
            return shape;
          }),
          nb::arg("scale"),
          nb::arg("mesh").none(),
          nb::arg("uri"))
      .def(
          dartnb::factory(
              +[](const Eigen::Vector3d& scale,
                  const aiScene* mesh,
                  const dart::common::Uri& uri,
                  dart::common::ResourceRetrieverPtr resourceRetriever) {
                DART_SUPPRESS_DEPRECATED_BEGIN
                auto shape = std::make_shared<dart::dynamics::MeshShape>(
                    scale, mesh, uri, resourceRetriever);
                DART_SUPPRESS_DEPRECATED_END
                return shape;
              }),
          nb::arg("scale"),
          nb::arg("mesh").none(),
          nb::arg("uri"),
          nb::arg("resourceRetriever").none())
      .def(
          "getType",
          +[](const dart::dynamics::MeshShape* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
      .def(
          "update", +[](dart::dynamics::MeshShape* self) { self->update(); })
      .def(
          "notifyAlphaUpdated",
          +[](dart::dynamics::MeshShape* self, double alpha) {
            self->notifyAlphaUpdated(alpha);
          },
          nb::arg("alpha"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self, const aiScene* mesh) {
            self->setMesh(mesh);
          },
          nb::arg("mesh").none())
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const std::string& path) { self->setMesh(mesh, path); },
          nb::arg("mesh").none(),
          nb::arg("path"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const std::string& path,
              dart::common::ResourceRetrieverPtr resourceRetriever) {
            self->setMesh(mesh, path, resourceRetriever);
          },
          nb::arg("mesh").none(),
          nb::arg("path"),
          nb::arg("resourceRetriever").none())
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const dart::common::Uri& path) { self->setMesh(mesh, path); },
          nb::arg("mesh").none(),
          nb::arg("path"))
      .def(
          "setMesh",
          +[](dart::dynamics::MeshShape* self,
              const aiScene* mesh,
              const dart::common::Uri& path,
              dart::common::ResourceRetrieverPtr resourceRetriever) {
            self->setMesh(mesh, path, resourceRetriever);
          },
          nb::arg("mesh").none(),
          nb::arg("path"),
          nb::arg("resourceRetriever").none())
      .def(
          "getMeshUri",
          +[](const dart::dynamics::MeshShape* self) -> std::string {
            return self->getMeshUri();
          })
      .def(
          "getMeshUri2",
          +[](const dart::dynamics::MeshShape* self)
              -> const dart::common::Uri& { return self->getMeshUri2(); },
          nb::rv_policy::reference_internal)
      .def(
          "getMeshPath",
          +[](const dart::dynamics::MeshShape* self) -> const std::string& {
            return self->getMeshPath();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getResourceRetriever",
          +[](dart::dynamics::MeshShape* self)
              -> dart::common::ResourceRetrieverPtr {
            return self->getResourceRetriever();
          })
      .def(
          "setScale",
          nb::overload_cast<const Eigen::Vector3d&>(
              &dart::dynamics::MeshShape::setScale),
          nb::arg("scale"))
      .def(
          "setScale",
          nb::overload_cast<double>(&dart::dynamics::MeshShape::setScale),
          nb::arg("scale"))
      .def(
          "getScale",
          +[](const dart::dynamics::MeshShape* self) -> const Eigen::Vector3d& {
            return self->getScale();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setColorMode",
          +[](dart::dynamics::MeshShape* self,
              dart::dynamics::MeshShape::ColorMode mode) {
            self->setColorMode(mode);
          },
          nb::arg("mode"))
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
          nb::arg("index"))
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
          nb::arg("index"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::MeshShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::MeshShape::getStaticType();
          },
          nb::rv_policy::reference_internal);
  DART_SUPPRESS_DEPRECATED_END

  auto attr = m.attr("MeshShape");

  nb::enum_<dart::dynamics::MeshShape::ColorMode>(
      attr, "ColorMode", nb::is_arithmetic())
      .value(
          "MATERIAL_COLOR",
          dart::dynamics::MeshShape::ColorMode::MATERIAL_COLOR)
      .value("COLOR_INDEX", dart::dynamics::MeshShape::ColorMode::COLOR_INDEX)
      .value("SHAPE_COLOR", dart::dynamics::MeshShape::ColorMode::SHAPE_COLOR)
      .export_values();

  dartnb::dart_class<dart::dynamics::ArrowShape, dart::dynamics::MeshShape>(
      m, "ArrowShape")
      .def(
          dartnb::init<const Eigen::Vector3d&, const Eigen::Vector3d&>(),
          nb::arg("tail"),
          nb::arg("head"))
      .def(
          dartnb::init<
              const Eigen::Vector3d&,
              const Eigen::Vector3d&,
              const dart::dynamics::ArrowShape::Properties&>(),
          nb::arg("tail"),
          nb::arg("head"),
          nb::arg("properties"))
      .def(
          dartnb::init<
              const Eigen::Vector3d&,
              const Eigen::Vector3d&,
              const dart::dynamics::ArrowShape::Properties&,
              const Eigen::Vector4d&>(),
          nb::arg("tail"),
          nb::arg("head"),
          nb::arg("properties"),
          nb::arg("color"))
      .def(
          dartnb::init<
              const Eigen::Vector3d&,
              const Eigen::Vector3d&,
              const dart::dynamics::ArrowShape::Properties&,
              const Eigen::Vector4d&,
              std::size_t>(),
          nb::arg("tail"),
          nb::arg("head"),
          nb::arg("properties"),
          nb::arg("color"),
          nb::arg("resolution"))
      .def(
          "setPositions",
          +[](dart::dynamics::ArrowShape* self,
              const Eigen::Vector3d& _tail,
              const Eigen::Vector3d& _head) {
            self->setPositions(_tail, _head);
          },
          nb::arg("tail"),
          nb::arg("head"))
      .def(
          "getTail",
          +[](const dart::dynamics::ArrowShape* self)
              -> const Eigen::Vector3d& { return self->getTail(); },
          nb::rv_policy::reference_internal)
      .def(
          "getHead",
          +[](const dart::dynamics::ArrowShape* self)
              -> const Eigen::Vector3d& { return self->getHead(); },
          nb::rv_policy::reference_internal)
      .def(
          "setProperties",
          +[](dart::dynamics::ArrowShape* self,
              const dart::dynamics::ArrowShape::Properties& _properties) {
            self->setProperties(_properties);
          },
          nb::arg("properties"))
      .def(
          "notifyColorUpdated",
          +[](dart::dynamics::ArrowShape* self, const Eigen::Vector4d& _color) {
            self->notifyColorUpdated(_color);
          },
          nb::arg("color"))
      .def(
          "getProperties",
          +[](const dart::dynamics::ArrowShape* self)
              -> const dart::dynamics::ArrowShape::Properties& {
            return self->getProperties();
          },
          nb::rv_policy::reference_internal)
      .def(
          "configureArrow",
          +[](dart::dynamics::ArrowShape* self,
              const Eigen::Vector3d& _tail,
              const Eigen::Vector3d& _head,
              const dart::dynamics::ArrowShape::Properties& _properties) {
            self->configureArrow(_tail, _head, _properties);
          },
          nb::arg("tail"),
          nb::arg("head"),
          nb::arg("properties"));

  dartnb::dart_class<dart::dynamics::ArrowShape::Properties>(
      m, "ArrowShapeProperties")
      .def(dartnb::init<>())
      .def(dartnb::init<double>(), nb::arg("radius"))
      .def(
          dartnb::init<double, double>(),
          nb::arg("radius"),
          nb::arg("headRadiusScale"))
      .def(
          dartnb::init<double, double, double>(),
          nb::arg("radius"),
          nb::arg("headRadiusScale"),
          nb::arg("headLengthScale"))
      .def(
          dartnb::init<double, double, double, double>(),
          nb::arg("radius"),
          nb::arg("headRadiusScale"),
          nb::arg("headLengthScale"),
          nb::arg("minHeadLength"))
      .def(
          dartnb::init<double, double, double, double, double>(),
          nb::arg("radius"),
          nb::arg("headRadiusScale"),
          nb::arg("headLengthScale"),
          nb::arg("minHeadLength"),
          nb::arg("maxHeadLength"))
      .def(
          dartnb::init<double, double, double, double, double, bool>(),
          nb::arg("radius"),
          nb::arg("headRadiusScale"),
          nb::arg("headLengthScale"),
          nb::arg("minHeadLength"),
          nb::arg("maxHeadLength"),
          nb::arg("doubleArrow"))
      .def_rw(
          "mRadius",
          &dart::dynamics::ArrowShape::Properties::mRadius,
          dartnb::setterArgument(
              &dart::dynamics::ArrowShape::Properties::mRadius))
      .def_rw(
          "mHeadRadiusScale",
          &dart::dynamics::ArrowShape::Properties::mHeadRadiusScale,
          dartnb::setterArgument(
              &dart::dynamics::ArrowShape::Properties::mHeadRadiusScale))
      .def_rw(
          "mHeadLengthScale",
          &dart::dynamics::ArrowShape::Properties::mHeadLengthScale,
          dartnb::setterArgument(
              &dart::dynamics::ArrowShape::Properties::mHeadLengthScale))
      .def_rw(
          "mMinHeadLength",
          &dart::dynamics::ArrowShape::Properties::mMinHeadLength,
          dartnb::setterArgument(
              &dart::dynamics::ArrowShape::Properties::mMinHeadLength))
      .def_rw(
          "mMaxHeadLength",
          &dart::dynamics::ArrowShape::Properties::mMaxHeadLength,
          dartnb::setterArgument(
              &dart::dynamics::ArrowShape::Properties::mMaxHeadLength))
      .def_rw(
          "mDoubleArrow",
          &dart::dynamics::ArrowShape::Properties::mDoubleArrow,
          dartnb::setterArgument(
              &dart::dynamics::ArrowShape::Properties::mDoubleArrow));

  dartnb::dart_class<dart::dynamics::PlaneShape, dart::dynamics::Shape>(
      m, "PlaneShape")
      .def(
          dartnb::init<const Eigen::Vector3d&, double>(),
          nb::arg("normal"),
          nb::arg("offset"))
      .def(
          dartnb::init<const Eigen::Vector3d&, const Eigen::Vector3d&>(),
          nb::arg("normal"),
          nb::arg("point"))
      .def(
          "getType",
          +[](const dart::dynamics::PlaneShape* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
      .def(
          "computeInertia",
          +[](const dart::dynamics::PlaneShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
      .def(
          "setNormal",
          +[](dart::dynamics::PlaneShape* self,
              const Eigen::Vector3d& _normal) { self->setNormal(_normal); },
          nb::arg("normal"))
      .def(
          "getNormal",
          +[](const dart::dynamics::PlaneShape* self)
              -> const Eigen::Vector3d& { return self->getNormal(); },
          nb::rv_policy::reference_internal)
      .def(
          "setOffset",
          +[](dart::dynamics::PlaneShape* self, double _offset) {
            self->setOffset(_offset);
          },
          nb::arg("offset"))
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
          nb::arg("normal"),
          nb::arg("offset"))
      .def(
          "setNormalAndPoint",
          +[](dart::dynamics::PlaneShape* self,
              const Eigen::Vector3d& _normal,
              const Eigen::Vector3d& _point) {
            self->setNormalAndPoint(_normal, _point);
          },
          nb::arg("normal"),
          nb::arg("point"))
      .def(
          "computeDistance",
          +[](const dart::dynamics::PlaneShape* self,
              const Eigen::Vector3d& _point) -> double {
            return self->computeDistance(_point);
          },
          nb::arg("point"))
      .def(
          "computeSignedDistance",
          +[](const dart::dynamics::PlaneShape* self,
              const Eigen::Vector3d& _point) -> double {
            return self->computeSignedDistance(_point);
          },
          nb::arg("point"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::PlaneShape::getStaticType();
          },
          nb::rv_policy::reference_internal);
  dartnb::dart_class<dart::dynamics::PointCloudShape, dart::dynamics::Shape>
      pointCloudShape(m, "PointCloudShape");

  pointCloudShape.def(dartnb::init<double>(), nb::arg("visualSize") = 0.01)
      .def(
          "getType",
          +[](const dart::dynamics::PointCloudShape* self)
              -> const std::string& { return self->getType(); },
          nb::rv_policy::reference_internal)
      .def(
          "computeInertia",
          +[](const dart::dynamics::PointCloudShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
      .def(
          "reserve",
          +[](dart::dynamics::PointCloudShape* self, std::size_t size) -> void {
            return self->reserve(size);
          },
          nb::arg("size"))
      .def(
          "addPoint",
          +[](dart::dynamics::PointCloudShape* self,
              const Eigen::Vector3d& point) -> void {
            return self->addPoint(point);
          },
          nb::arg("point"))
      .def(
          "addPoint",
          +[](dart::dynamics::PointCloudShape* self,
              const std::vector<Eigen::Vector3d>& points) -> void {
            return self->addPoint(points);
          },
          nb::arg("points"))
      .def(
          "setPoint",
          +[](dart::dynamics::PointCloudShape* self,
              const std::vector<Eigen::Vector3d>& points) -> void {
            return self->setPoint(points);
          },
          nb::arg("points"))
      .def(
          "getPoints",
          +[](const dart::dynamics::PointCloudShape* self)
              -> const std::vector<Eigen::Vector3d>& {
            return self->getPoints();
          },
          nb::rv_policy::reference_internal)
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
          nb::arg("type"))
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
          nb::arg("mode"))
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
          nb::arg("color"))
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
          nb::arg("colors"))
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
          nb::arg("size"))
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
          nb::arg("color"));

  nb::enum_<dart::dynamics::PointCloudShape::ColorMode>(
      pointCloudShape, "ColorMode", nb::is_arithmetic())
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

  nb::enum_<dart::dynamics::PointCloudShape::PointShapeType>(
      pointCloudShape, "PointShapeType", nb::is_arithmetic())
      .value("BOX", dart::dynamics::PointCloudShape::PointShapeType::BOX)
      .value(
          "BILLBOARD_SQUARE",
          dart::dynamics::PointCloudShape::PointShapeType::BILLBOARD_SQUARE)
      .value(
          "BILLBOARD_CIRCLE",
          dart::dynamics::PointCloudShape::PointShapeType::BILLBOARD_CIRCLE)
      .export_values();

  dartnb::dart_class<dart::dynamics::SphereShape, dart::dynamics::Shape>(
      m, "SphereShape")
      .def(dartnb::init<double>(), nb::arg("radius"))
      .def(
          "getType",
          +[](const dart::dynamics::SphereShape* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
      .def(
          "setRadius",
          +[](dart::dynamics::SphereShape* self, double radius) {
            self->setRadius(radius);
          },
          nb::arg("radius"))
      .def(
          "getRadius",
          +[](const dart::dynamics::SphereShape* self) -> double {
            return self->getRadius();
          })
      .def(
          "computeInertia",
          +[](const dart::dynamics::SphereShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::SphereShape::getStaticType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "computeVolumeOf",
          +[](double radius) -> double {
            return dart::dynamics::SphereShape::computeVolume(radius);
          },
          nb::arg("radius"))
      .def_static(
          "computeInertiaOf",
          +[](double radius, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::SphereShape::computeInertia(radius, mass);
          },
          nb::arg("radius"),
          nb::arg("mass"));

  dartnb::dart_class<dart::dynamics::CapsuleShape, dart::dynamics::Shape>(
      m, "CapsuleShape")
      .def(dartnb::init<double, double>(), nb::arg("radius"), nb::arg("height"))
      .def(
          "getType",
          +[](const dart::dynamics::CapsuleShape* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
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
          nb::arg("radius"))
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
          nb::arg("height"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::CapsuleShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::CapsuleShape::getStaticType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "computeVolumeOf",
          +[](double radius, double height) -> double {
            return dart::dynamics::CapsuleShape::computeVolume(radius, height);
          },
          nb::arg("radius"),
          nb::arg("height"))
      .def_static(
          "computeInertiaOf",
          +[](double radius, double height, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::CapsuleShape::computeInertia(
                radius, height, mass);
          },
          nb::arg("radius"),
          nb::arg("height"),
          nb::arg("mass"));

  dartnb::dart_class<dart::dynamics::CylinderShape, dart::dynamics::Shape>(
      m, "CylinderShape")
      .def(dartnb::init<double, double>(), nb::arg("radius"), nb::arg("height"))
      .def(
          "getType",
          +[](const dart::dynamics::CylinderShape* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
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
          nb::arg("radius"))
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
          nb::arg("height"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::CylinderShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::CylinderShape::getStaticType();
          },
          nb::rv_policy::reference_internal)
      .def_static(
          "computeVolumeOf",
          +[](double radius, double height) -> double {
            return dart::dynamics::CylinderShape::computeVolume(radius, height);
          },
          nb::arg("radius"),
          nb::arg("height"))
      .def_static(
          "computeInertiaOf",
          +[](double radius, double height, double mass) -> Eigen::Matrix3d {
            return dart::dynamics::CylinderShape::computeInertia(
                radius, height, mass);
          },
          nb::arg("radius"),
          nb::arg("height"),
          nb::arg("mass"));

  dartnb::dart_class<dart::dynamics::SoftMeshShape, dart::dynamics::Shape>(
      m, "SoftMeshShape")
      .def(
          dartnb::init<dart::dynamics::SoftBodyNode*>(),
          nb::arg("softBodyNode").none())
      .def(
          "getType",
          +[](const dart::dynamics::SoftMeshShape* self) -> const std::string& {
            return self->getType();
          },
          nb::rv_policy::reference_internal)
      .def(
          "update",
          +[](dart::dynamics::SoftMeshShape* self) { self->update(); })
      .def(
          "computeInertia",
          +[](const dart::dynamics::SoftMeshShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::SoftMeshShape::getStaticType();
          },
          nb::rv_policy::reference_internal);

  dartnb::dart_class<dart::dynamics::EllipsoidShape, dart::dynamics::Shape>(
      m, "EllipsoidShape")
      .def(dartnb::init<const Eigen::Vector3d&>(), nb::arg("diameters"))
      .def(
          "getType",
          +[](const dart::dynamics::EllipsoidShape* self)
              -> const std::string& { return self->getType(); },
          nb::rv_policy::reference_internal)
      .def(
          "setDiameters",
          +[](dart::dynamics::EllipsoidShape* self,
              const Eigen::Vector3d& diameters) {
            self->setDiameters(diameters);
          },
          nb::arg("diameters"))
      .def(
          "getDiameters",
          +[](const dart::dynamics::EllipsoidShape* self)
              -> const Eigen::Vector3d& { return self->getDiameters(); },
          nb::rv_policy::reference_internal)
      .def(
          "setRadii",
          +[](dart::dynamics::EllipsoidShape* self,
              const Eigen::Vector3d& radii) { self->setRadii(radii); },
          nb::arg("radii"))
      .def(
          "getRadii",
          +[](const dart::dynamics::EllipsoidShape* self)
              -> const Eigen::Vector3d { return self->getRadii(); })
      .def(
          "computeInertia",
          +[](const dart::dynamics::EllipsoidShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
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
          nb::rv_policy::reference_internal)
      .def_static(
          "computeVolumeOf",
          +[](const Eigen::Vector3d& diameters) -> double {
            return dart::dynamics::EllipsoidShape::computeVolume(diameters);
          },
          nb::arg("diameters"))
      .def_static(
          "computeInertiaOf",
          +[](const Eigen::Vector3d& diameters,
              double mass) -> Eigen::Matrix3d {
            return dart::dynamics::EllipsoidShape::computeInertia(
                diameters, mass);
          },
          nb::arg("diameters"),
          nb::arg("mass"));

  dartnb::dart_class<dart::dynamics::LineSegmentShape, dart::dynamics::Shape>(
      m, "LineSegmentShape")
      .def(dartnb::init<>())
      .def(dartnb::init<float>(), nb::arg("thickness"))
      .def(
          dartnb::init<const Eigen::Vector3d&, const Eigen::Vector3d&>(),
          nb::arg("v1"),
          nb::arg("v2"))
      .def(
          dartnb::init<const Eigen::Vector3d&, const Eigen::Vector3d&, float>(),
          nb::arg("v1"),
          nb::arg("v2"),
          nb::arg("thickness"))
      .def(
          "getType",
          +[](const dart::dynamics::LineSegmentShape* self)
              -> const std::string& { return self->getType(); },
          nb::rv_policy::reference_internal)
      .def(
          "setThickness",
          +[](dart::dynamics::LineSegmentShape* self, float _thickness) {
            self->setThickness(_thickness);
          },
          nb::arg("thickness"))
      .def(
          "getThickness",
          +[](const dart::dynamics::LineSegmentShape* self) -> float {
            return self->getThickness();
          })
      .def(
          "addVertex",
          +[](dart::dynamics::LineSegmentShape* self, const Eigen::Vector3d& _v)
              -> std::size_t { return self->addVertex(_v); },
          nb::arg("v"))
      .def(
          "addVertex",
          +[](dart::dynamics::LineSegmentShape* self,
              const Eigen::Vector3d& _v,
              std::size_t _parent) -> std::size_t {
            return self->addVertex(_v, _parent);
          },
          nb::arg("v"),
          nb::arg("parent"))
      .def(
          "removeVertex",
          +[](dart::dynamics::LineSegmentShape* self, std::size_t _idx) {
            self->removeVertex(_idx);
          },
          nb::arg("idx"))
      .def(
          "setVertex",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _idx,
              const Eigen::Vector3d& _v) { self->setVertex(_idx, _v); },
          nb::arg("idx"),
          nb::arg("v"))
      .def(
          "getVertex",
          +[](const dart::dynamics::LineSegmentShape* self, std::size_t _idx)
              -> const Eigen::Vector3d& { return self->getVertex(_idx); },
          nb::rv_policy::reference_internal,
          nb::arg("idx"))
      .def(
          "addConnection",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _idx1,
              std::size_t _idx2) { self->addConnection(_idx1, _idx2); },
          nb::arg("idx1"),
          nb::arg("idx2"))
      .def(
          "removeConnection",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _vertexIdx1,
              std::size_t _vertexIdx2) {
            self->removeConnection(_vertexIdx1, _vertexIdx2);
          },
          nb::arg("vertexIdx1"),
          nb::arg("vertexIdx2"))
      .def(
          "removeConnection",
          +[](dart::dynamics::LineSegmentShape* self,
              std::size_t _connectionIdx) {
            self->removeConnection(_connectionIdx);
          },
          nb::arg("connectionIdx"))
      .def(
          "computeInertia",
          +[](const dart::dynamics::LineSegmentShape* self, double mass)
              -> Eigen::Matrix3d { return self->computeInertia(mass); },
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::LineSegmentShape::getStaticType();
          },
          nb::rv_policy::reference_internal);

  dartnb::dart_class<
      dart::dynamics::MultiSphereConvexHullShape,
      dart::dynamics::Shape>(m, "MultiSphereConvexHullShape")
      .def(
          dartnb::init<
              const dart::dynamics::MultiSphereConvexHullShape::Spheres&>(),
          nb::arg("spheres"))
      .def(
          "getType",
          +[](const dart::dynamics::MultiSphereConvexHullShape* self)
              -> const std::string& { return self->getType(); },
          nb::rv_policy::reference_internal)
      .def(
          "addSpheres",
          +[](dart::dynamics::MultiSphereConvexHullShape* self,
              const dart::dynamics::MultiSphereConvexHullShape::Spheres&
                  spheres) { self->addSpheres(spheres); },
          nb::arg("spheres"))
      .def(
          "addSphere",
          +[](dart::dynamics::MultiSphereConvexHullShape* self,
              const dart::dynamics::MultiSphereConvexHullShape::Sphere&
                  sphere) { self->addSphere(sphere); },
          nb::arg("sphere"))
      .def(
          "addSphere",
          +[](dart::dynamics::MultiSphereConvexHullShape* self,
              double radius,
              const Eigen::Vector3d& position) {
            self->addSphere(radius, position);
          },
          nb::arg("radius"),
          nb::arg("position"))
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
          nb::arg("mass"))
      .def_static(
          "getStaticType",
          +[]() -> const std::string& {
            return dart::dynamics::MultiSphereConvexHullShape::getStaticType();
          },
          nb::rv_policy::reference_internal);
}

} // namespace python
} // namespace dart

DART_SUPPRESS_DEPRECATED_END

#undef DARTPY_DEFINE_SHAPE_TYPE
#undef DARTPY_DEFINE_DATA_VARIANCE
