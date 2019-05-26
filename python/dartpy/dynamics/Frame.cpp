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

namespace py = pybind11;

namespace dart {
namespace python {

void Frame(py::module& m)
{
  ::py::class_<
      dart::dynamics::Frame,
      dart::dynamics::Entity,
      std::shared_ptr<dart::dynamics::Frame> >(m, "Frame")
      .def(
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
          ::py::arg("withRespectTo"))
      .def(
          "getTransform",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* withRespectTo,
              const dart::dynamics::Frame* inCoordinatesOf)
              -> Eigen::Isometry3d {
            return self->getTransform(withRespectTo, inCoordinatesOf);
          },
          ::py::arg("withRespectTo"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getSpatialVelocity",
          +[](const dart::dynamics::Frame* self, const Eigen::Vector3d& offset)
              -> Eigen::Vector6d { return self->getSpatialVelocity(offset); },
          ::py::arg("offset"))
      .def(
          "getSpatialVelocity",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector6d {
            return self->getSpatialVelocity(
                offset, relativeTo, inCoordinatesOf);
          },
          ::py::arg("offset"),
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("relativeTo"))
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getLinearVelocity(relativeTo, inCoordinatesOf);
          },
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self, const Eigen::Vector3d& offset)
              -> Eigen::Vector3d { return self->getLinearVelocity(offset); },
          ::py::arg("offset"))
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo) -> Eigen::Vector3d {
            return self->getLinearVelocity(offset, relativeTo);
          },
          ::py::arg("offset"),
          ::py::arg("relativeTo"))
      .def(
          "getLinearVelocity",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getLinearVelocity(offset, relativeTo, inCoordinatesOf);
          },
          ::py::arg("offset"),
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("relativeTo"))
      .def(
          "getAngularVelocity",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getAngularVelocity(relativeTo, inCoordinatesOf);
          },
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getSpatialAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset) -> Eigen::Vector6d {
            return self->getSpatialAcceleration(offset);
          },
          ::py::arg("offset"))
      .def(
          "getSpatialAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector6d {
            return self->getSpatialAcceleration(
                offset, relativeTo, inCoordinatesOf);
          },
          ::py::arg("offset"),
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("relativeTo"))
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getLinearAcceleration(relativeTo, inCoordinatesOf);
          },
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset) -> Eigen::Vector3d {
            return self->getLinearAcceleration(offset);
          },
          ::py::arg("offset"))
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo) -> Eigen::Vector3d {
            return self->getLinearAcceleration(offset, relativeTo);
          },
          ::py::arg("offset"),
          ::py::arg("relativeTo"))
      .def(
          "getLinearAcceleration",
          +[](const dart::dynamics::Frame* self,
              const Eigen::Vector3d& offset,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getLinearAcceleration(
                offset, relativeTo, inCoordinatesOf);
          },
          ::py::arg("offset"),
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
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
          ::py::arg("relativeTo"))
      .def(
          "getAngularAcceleration",
          +[](const dart::dynamics::Frame* self,
              const dart::dynamics::Frame* relativeTo,
              const dart::dynamics::Frame* inCoordinatesOf) -> Eigen::Vector3d {
            return self->getAngularAcceleration(relativeTo, inCoordinatesOf);
          },
          ::py::arg("relativeTo"),
          ::py::arg("inCoordinatesOf"))
      .def(
          "getChildEntities",
          +[](const dart::dynamics::Frame* self)
              -> const std::set<const dart::dynamics::Entity*> {
            return self->getChildEntities();
          })
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
          })
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
      .def_static("World", +[]() -> std::shared_ptr<dart::dynamics::Frame> {
        return dart::dynamics::Frame::WorldShared();
      });
}

} // namespace python
} // namespace dart
