// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/secondary_methods.hpp"

#include <nanobind/stl/unique_ptr.h>

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

#include <dart/dynamics/Frame.hpp>
#include <dart/dynamics/Shape.hpp>
#include <dart/dynamics/ShapeFrame.hpp>
#include <dart/dynamics/ShapeNode.hpp>

#include <dart/common/Aspect.hpp>
#include <dart/common/EmbeddedAspect.hpp>
#include <dart/common/SpecializedForAspect.hpp>

#include <Eigen/Core>

#include <memory>

#define DARTPY_DEFINE_SPECIALIZED_ASPECT(name)                                 \
  .def(                                                                        \
      "has" #name,                                                             \
      +[](const dart::dynamics::ShapeFrame* self) -> bool {                    \
        return self->has##name();                                              \
      })                                                                       \
      .def(                                                                    \
          "get" #name,                                                         \
          +[](dart::dynamics::ShapeFrame* self) -> dart::dynamics::name* {     \
            return self->get##name();                                          \
          },                                                                   \
          nb::rv_policy::reference_internal)                                   \
      .def(                                                                    \
          "get" #name,                                                         \
          +[](dart::dynamics::ShapeFrame* self,                                \
              bool createIfNull) -> dart::dynamics::name* {                    \
            return self->get##name(createIfNull);                              \
          },                                                                   \
          nb::rv_policy::reference_internal,                                   \
          nb::arg("createIfNull"))                                             \
      .def(                                                                    \
          "set" #name,                                                         \
          +[](dart::dynamics::ShapeFrame* self,                                \
              const dart::dynamics::name* aspect) {                            \
            self->set##name(aspect);                                           \
          },                                                                   \
          nb::arg("aspect").none())                                            \
      .def(                                                                    \
          "create" #name,                                                      \
          +[](dart::dynamics::ShapeFrame* self) -> dart::dynamics::name* {     \
            return self->create##name();                                       \
          },                                                                   \
          nb::rv_policy::reference_internal)                                   \
      .def(                                                                    \
          "remove" #name,                                                      \
          +[](dart::dynamics::ShapeFrame* self) { self->remove##name(); })     \
      .def(                                                                    \
          "release" #name,                                                     \
          +[](dart::dynamics::ShapeFrame* self)                                \
              -> std::unique_ptr<dart::dynamics::name> {                       \
            return self->release##name();                                      \
          })

namespace dart {
namespace python {

template <class Cls>
void defShapeFrameMethods(Cls& cls)
{
  cls.def(
         "setProperties",
         +[](dart::dynamics::ShapeFrame* self,
             const dart::dynamics::ShapeFrame::UniqueProperties& properties) {
           self->setProperties(properties);
         },
         nb::arg("properties"))
      .def(
          "setAspectProperties",
          +[](dart::dynamics::ShapeFrame* self,
              const dart::common::EmbedPropertiesOnTopOf<
                  dart::dynamics::ShapeFrame,
                  dart::dynamics::detail::ShapeFrameProperties,
                  dart::common::SpecializedForAspect<
                      dart::dynamics::VisualAspect,
                      dart::dynamics::CollisionAspect,
                      dart::dynamics::DynamicsAspect>>::AspectProperties&
                  properties) { self->setAspectProperties(properties); },
          nb::arg("properties"))
      .def(
          "setShape",
          +[](dart::dynamics::ShapeFrame* self,
              const dart::dynamics::ShapePtr& shape) { self->setShape(shape); },
          nb::arg("shape").none())
      .def(
          "getShape",
          +[](dart::dynamics::ShapeFrame* self) -> dart::dynamics::ShapePtr {
            return self->getShape();
          })
      .def(
          "getShape",
          +[](const dart::dynamics::ShapeFrame* self)
              -> dart::dynamics::ConstShapePtr { return self->getShape(); })
      // clang-format off
      DARTPY_DEFINE_SPECIALIZED_ASPECT(VisualAspect)
      DARTPY_DEFINE_SPECIALIZED_ASPECT(CollisionAspect)
      DARTPY_DEFINE_SPECIALIZED_ASPECT(DynamicsAspect)
      // clang-format on
      .def(
          "isShapeNode",
          +[](const dart::dynamics::ShapeFrame* self) -> bool {
            return self->isShapeNode();
          })
      .def(
          "asShapeNode",
          +[](dart::dynamics::ShapeFrame* self) -> dart::dynamics::ShapeNode* {
            return self->asShapeNode();
          },
          nb::rv_policy::reference,
          "Convert to a ShapeNode pointer if ShapeFrame is a ShapeNode, "
          "otherwise return None.")
      .def(
          "asShapeNode",
          +[](const dart::dynamics::ShapeFrame* self)
              -> const dart::dynamics::ShapeNode* {
            return self->asShapeNode();
          },
          nb::rv_policy::reference,
          "Convert to a ShapeNode pointer if ShapeFrame is a ShapeNode, "
          "otherwise return None.");
}

void ShapeFrame(nb::module_& m)
{
  auto cls = dartnb::dart_class<
      dart::dynamics::
          ShapeFrame, // dart::common::EmbedPropertiesOnTopOf<
                      //     dart::dynamics::ShapeFrame,
                      //     dart::dynamics::detail::ShapeFrameProperties,
                      //     dart::common::SpecializedForAspect<
                      //         dart::dynamics::VisualAspect,
                      //         dart::dynamics::CollisionAspect,
                      //         dart::dynamics::DynamicsAspect> >,
      dart::dynamics::Frame>(m, "ShapeFrame");
  defShapeFrameMethods(cls);
  dartnb::register_methods(
      typeid(dart::dynamics::ShapeFrame), [](nb::handle target) {
        dartnb::SecondaryMethods<dart::dynamics::ShapeFrame> rebound(target);
        defShapeFrameMethods(rebound);
      });

  dartnb::dart_class<dart::dynamics::VisualAspect>(m, "VisualAspect")
      .def(dartnb::init<>())
      .def(
          dartnb::init<
              const dart::common::detail::AspectWithVersionedProperties<
                  dart::common::CompositeTrackingAspect<
                      dart::dynamics::ShapeFrame>,
                  dart::dynamics::VisualAspect,
                  dart::dynamics::detail::VisualAspectProperties,
                  dart::dynamics::ShapeFrame,
                  &dart::common::detail::NoOp>::PropertiesData&>(),
          nb::arg("properties"))
      .def(
          "setRGBA",
          +[](dart::dynamics::VisualAspect* self,
              const Eigen::Vector4d& color) { self->setRGBA(color); },
          nb::arg("color"))
      .def(
          "getRGBA",
          +[](dart::dynamics::VisualAspect* self) -> const Eigen::Vector4d& {
            return self->getRGBA();
          })
      .def(
          "setHidden",
          +[](dart::dynamics::VisualAspect* self, const bool& value) {
            self->setHidden(value);
          },
          nb::arg("value"))
      .def(
          "getHidden",
          +[](dart::dynamics::VisualAspect* self) -> bool {
            return self->getHidden();
          })
      .def(
          "setShadowed",
          +[](dart::dynamics::VisualAspect* self, const bool& value) {
            self->setShadowed(value);
          },
          nb::arg("value"))
      .def(
          "getShadowed",
          +[](dart::dynamics::VisualAspect* self) -> bool {
            return self->getShadowed();
          })
      .def(
          "setColor",
          +[](dart::dynamics::VisualAspect* self,
              const Eigen::Vector3d& color) { self->setColor(color); },
          nb::arg("color"))
      .def(
          "setColor",
          +[](dart::dynamics::VisualAspect* self,
              const Eigen::Vector4d& color) { self->setColor(color); },
          nb::arg("color"))
      .def(
          "setRGB",
          +[](dart::dynamics::VisualAspect* self, const Eigen::Vector3d& rgb) {
            self->setRGB(rgb);
          },
          nb::arg("rgb"))
      .def(
          "setAlpha",
          +[](dart::dynamics::VisualAspect* self, const double alpha) {
            self->setAlpha(alpha);
          },
          nb::arg("alpha"))
      .def(
          "getColor",
          +[](const dart::dynamics::VisualAspect* self) -> Eigen::Vector3d {
            return self->getColor();
          })
      .def(
          "getRGB",
          +[](const dart::dynamics::VisualAspect* self) -> Eigen::Vector3d {
            return self->getRGB();
          })
      .def(
          "getAlpha",
          +[](const dart::dynamics::VisualAspect* self) -> double {
            return self->getAlpha();
          })
      .def(
          "hide", +[](dart::dynamics::VisualAspect* self) { self->hide(); })
      .def(
          "show", +[](dart::dynamics::VisualAspect* self) { self->show(); })
      .def(
          "isHidden", +[](const dart::dynamics::VisualAspect* self) -> bool {
            return self->isHidden();
          });

  dartnb::dart_class<dart::dynamics::CollisionAspect>(m, "CollisionAspect")
      .def(dartnb::init<>())
      .def(
          dartnb::init<
              const dart::common::detail::AspectWithVersionedProperties<
                  dart::common::CompositeTrackingAspect<
                      dart::dynamics::ShapeFrame>,
                  dart::dynamics::CollisionAspect,
                  dart::dynamics::detail::CollisionAspectProperties,
                  dart::dynamics::ShapeFrame,
                  &dart::common::detail::NoOp>::PropertiesData&>(),
          nb::arg("properties"))
      .def(
          "setCollidable",
          +[](dart::dynamics::CollisionAspect* self, const bool& value) {
            self->setCollidable(value);
          },
          nb::arg("value"))
      .def(
          "getCollidable",
          +[](const dart::dynamics::CollisionAspect* self) -> bool {
            return self->getCollidable();
          })
      .def(
          "isCollidable",
          +[](const dart::dynamics::CollisionAspect* self) -> bool {
            return self->isCollidable();
          });

  dartnb::dart_class<dart::dynamics::DynamicsAspect>(m, "DynamicsAspect")
      .def(dartnb::init<>())
      .def(
          dartnb::init<
              const dart::common::detail::AspectWithVersionedProperties<
                  dart::common::CompositeTrackingAspect<
                      dart::dynamics::ShapeFrame>,
                  dart::dynamics::DynamicsAspect,
                  dart::dynamics::detail::DynamicsAspectProperties,
                  dart::dynamics::ShapeFrame,
                  &dart::common::detail::NoOp>::PropertiesData&>(),
          nb::arg("properties"))
      .def(
          "setFrictionCoeff",
          +[](dart::dynamics::DynamicsAspect* self, const double& value) {
            self->setFrictionCoeff(value);
          },
          nb::arg("value"))
      .def(
          "getFrictionCoeff",
          +[](const dart::dynamics::DynamicsAspect* self) -> double {
            return self->getFrictionCoeff();
          })
      .def(
          "setRestitutionCoeff",
          +[](dart::dynamics::DynamicsAspect* self, const double& value) {
            self->setRestitutionCoeff(value);
          },
          nb::arg("value"))
      .def(
          "getRestitutionCoeff",
          +[](const dart::dynamics::DynamicsAspect* self) -> double {
            return self->getRestitutionCoeff();
          });
}

} // namespace python
} // namespace dart

#undef DARTPY_DEFINE_SPECIALIZED_ASPECT
