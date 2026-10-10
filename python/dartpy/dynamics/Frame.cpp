// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/secondary_methods.hpp"

#include <nanobind/stl/set.h>

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

#include <dart/dynamics/Entity.hpp>
#include <dart/dynamics/Frame.hpp>

#include <dart/math/MathTypes.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <set>

#include <cstddef>

namespace dart {
namespace python {

template <class Cls>
void defFrameMethods(Cls& cls)
{
  cls.def(
         "getRelativeTransform",
         +[](const dart::dynamics::Frame* self) -> Eigen::Isometry3d {
           return self->getRelativeTransform();
         })
      .def(
          "getWorldTransform",
          +[](const dart::dynamics::Frame* self) -> Eigen::Isometry3d {
            return self->getWorldTransform();
          })
      .def(
          "getTransform",
          +[](const dart::dynamics::Frame* self) -> Eigen::Isometry3d {
            return self->getTransform();
          })
      .def(
          "getTransform",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* withRespectTo) -> Eigen::Isometry3d {
            return self->getTransform(withRespectTo);
          },
          nb::arg("withRespectTo").none())
      .def(
          "getTransform",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* withRespectTo,
              const dart::dynamics::Frame* inCoordinatesOf)
              -> Eigen::Isometry3d {
            return self->getTransform(withRespectTo, inCoordinatesOf);
          },
          nb::arg("withRespectTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getSpatialVelocity",
          +[](const dart::dynamics::Frame* self) -> Eigen::Vector6d {
            return self->getSpatialVelocity();
          })
      .def(
          "getSpatialVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector6d {
            return self->getSpatialVelocity(relativeTo, inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getSpatialVelocity",
          +[](const dart::dynamics::Frame* self, const Eigen::Vector3d& offset)
              -> Eigen::Vector6d { return self->getSpatialVelocity(offset); },
          nb::arg("offset"))
      .def(
          "getSpatialVelocity",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector6d {
            return self->getSpatialVelocity(
                offset, relativeTo, inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self) -> Eigen::Vector3d {
            return self->getLinearVelocity();
          })
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo) -> Eigen::Vector3d {
            return self->getLinearVelocity(relativeTo);
          },
          nb::arg("relativeTo").none())
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getLinearVelocity(relativeTo, inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self, const Eigen::Vector3d& offset)
              -> Eigen::Vector3d { return self->getLinearVelocity(offset); },
          nb::arg("offset"))
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo) -> Eigen::Vector3d {
            return self->getLinearVelocity(offset, relativeTo);
          },
          nb::arg("offset"),
          nb::arg("relativeTo").none())
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getLinearVelocity(offset, relativeTo, inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getAngularVelocity",
          +[](const dart::dynamics::Frame* self) -> Eigen::Vector3d {
            return self->getAngularVelocity();
          })
      .def(
          "getAngularVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo) -> Eigen::Vector3d {
            return self->getAngularVelocity(relativeTo);
          },
          nb::arg("relativeTo").none())
      .def(
          "getAngularVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getAngularVelocity(relativeTo, inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getSpatialAcceleration",
          +[](const dart::dynamics::Frame* self) -> Eigen::Vector6d {
            return self->getSpatialAcceleration();
          })
      .def(
          "getSpatialAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector6d {
            return self->getSpatialAcceleration(relativeTo, inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getSpatialAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset) -> Eigen::Vector6d {
            return self->getSpatialAcceleration(offset);
          },
          nb::arg("offset"))
      .def(
          "getSpatialAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector6d {
            return self->getSpatialAcceleration(
                offset, relativeTo, inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self) -> Eigen::Vector3d {
            return self->getLinearAcceleration();
          })
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo) -> Eigen::Vector3d {
            return self->getLinearAcceleration(relativeTo);
          },
          nb::arg("relativeTo").none())
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getLinearAcceleration(relativeTo, inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset) -> Eigen::Vector3d {
            return self->getLinearAcceleration(offset);
          },
          nb::arg("offset"))
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo) -> Eigen::Vector3d {
            return self->getLinearAcceleration(offset, relativeTo);
          },
          nb::arg("offset"),
          nb::arg("relativeTo").none())
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getLinearAcceleration(
                offset, relativeTo, inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getAngularAcceleration",
          +[](const dart::dynamics::Frame* self) -> Eigen::Vector3d {
            return self->getAngularAcceleration();
          })
      .def(
          "getAngularAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo) -> Eigen::Vector3d {
            return self->getAngularAcceleration(relativeTo);
          },
          nb::arg("relativeTo").none())
      .def(
          "getAngularAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getAngularAcceleration(relativeTo, inCoordinatesOf);
          },
          nb::arg("relativeTo").none(),
          nb::arg("inCoordinatesOf").none())
      .def(
          "getChildEntities",
          +[](const dart::dynamics::Frame* self)
              -> const std::set<const dart::dynamics::Entity*> {
            return self->getChildEntities();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getNumChildEntities",
          +[](const dart::dynamics::Frame* self) -> std::size_t {
            return self->getNumChildEntities();
          })
      .def(
          "getChildFrames",
          +[](const dart::dynamics::Frame* self)
              -> std::set<const dart::dynamics::Frame*> {
            return self->getChildFrames();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getNumChildFrames",
          +[](const dart::dynamics::Frame* self) -> std::size_t {
            return self->getNumChildFrames();
          })
      .def(
          "isShapeFrame",
          +[](const dart::dynamics::Frame* self) -> bool {
            return self->isShapeFrame();
          })
      .def(
          "isWorld",
          +[](const dart::dynamics::Frame* self) -> bool {
            return self->isWorld();
          })
      .def(
          "dirtyTransform",
          +[](dart::dynamics::Frame* self) { self->dirtyTransform(); })
      .def(
          "dirtyVelocity",
          +[](dart::dynamics::Frame* self) { self->dirtyVelocity(); })
      .def(
          "dirtyAcceleration",
          +[](dart::dynamics::Frame* self) { self->dirtyAcceleration(); })
      .def_static(
          "World", +[]() -> std::shared_ptr<dart::dynamics::Frame> {
            return dart::dynamics::Frame::WorldShared();
          });
}

void Frame(nb::module_& m)
{
  auto cls = dartnb::dart_class<dart::dynamics::Frame, dart::dynamics::Entity>(
      m, "Frame");
  defFrameMethods(cls);
  dartnb::register_methods(
      typeid(dart::dynamics::Frame), [](nb::handle target) {
        dartnb::SecondaryMethods<dart::dynamics::Frame> rebound(target);
        defFrameMethods(rebound);
      });
}

} // namespace python
} // namespace dart
