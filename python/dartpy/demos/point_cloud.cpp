// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include <dart/config.hpp>

#if HAVE_OCTOMAP

  #include "demo_models/PointCloudSensor.hpp"

  #include <array>
  #include <memory>

  #include <cmath>

namespace dart::python {
namespace {

struct DemoPointCloudSensor
{
  std::array<nb::object, 4> owners;
  std::unique_ptr<dart_demos::PointCloudSensor> native;

  DemoPointCloudSensor(
      nb::object robot,
      nb::object sensorFrame,
      nb::object pointCloud,
      nb::object voxelGrid)
    : owners{
        std::move(robot),
        std::move(sensorFrame),
        std::move(pointCloud),
        std::move(voxelGrid)}
  {
    for (const auto& owner : owners)
      if (owner.is_none())
        throw nb::value_error(
            "robot, sensor_frame, point_cloud and voxel_grid must not be None");
    if (!nb::isinstance<dart::dynamics::Skeleton>(owners[0])
        || !nb::isinstance<dart::dynamics::SimpleFrame>(owners[1])
        || !nb::isinstance<dart::dynamics::PointCloudShape>(owners[2])
        || !nb::isinstance<dart::dynamics::VoxelGridShape>(owners[3]))
      throw nb::type_error(
          "expected Skeleton, SimpleFrame, PointCloudShape and VoxelGridShape");
    native = std::make_unique<dart_demos::PointCloudSensor>(
        nb::cast<dart::dynamics::SkeletonPtr>(owners[0]),
        nb::cast<dart::dynamics::SimpleFramePtr>(owners[1]),
        nb::cast<std::shared_ptr<dart::dynamics::PointCloudShape>>(owners[2]),
        nb::cast<std::shared_ptr<dart::dynamics::VoxelGridShape>>(owners[3]));
  }

  dart_demos::PointCloudSensor& get() const
  {
    if (!native)
      throw nb::value_error("sensor has been cleared");
    return *native;
  }

  void clear()
  {
    native.reset();
    for (auto& owner : owners)
      owner.reset();
  }
};

int sensorTraverse(PyObject* self, visitproc visit, void* arg) noexcept
{
  Py_VISIT(Py_TYPE(self));
  if (!nb::inst_ready(self))
    return 0;
  try {
    auto* sensor = nb::inst_ptr<DemoPointCloudSensor>(self);
    for (const auto& owner : sensor->owners)
      Py_VISIT(owner.ptr());
    dartnb::GcEdges edges;
    if (sensor->native) {
      edges.add(sensor->native->getRobot(), [] {});
      edges.add(sensor->native->getSensorFrame(), [] {});
      edges.add(sensor->native->getPointCloud(), [] {});
      edges.add(sensor->native->getVoxelGrid(), [] {});
    }
    return edges.traverse(visit, arg);
  } catch (...) {
    PyErr_SetString(
        PyExc_RuntimeError, "point-cloud sensor GC traversal failed");
    return -1;
  }
}

int sensorClear(PyObject* self) noexcept
{
  if (nb::inst_ready(self))
    nb::inst_ptr<DemoPointCloudSensor>(self)->clear();
  return 0;
}

} // namespace
} // namespace dart::python

namespace dartnb {
template <>
const PyType_Slot* gc_slots<dart::python::DemoPointCloudSensor>()
{
  static const PyType_Slot slots[] = {
      {Py_tp_traverse, reinterpret_cast<void*>(&dart::python::sensorTraverse)},
      {Py_tp_clear, reinterpret_cast<void*>(&dart::python::sensorClear)},
      {0, nullptr}};
  return slots;
}
} // namespace dartnb

namespace dart::python {

void bind_point_cloud_sensor(nb::module_& demos)
{
  using Sensor = dart_demos::PointCloudSensor;
  dartnb::dart_class<DemoPointCloudSensor>(
      demos,
      "PointCloudSensor",
      "Native mesh sampling, sensor orbit and voxel occupancy update.")
      .def(
          nb::init<nb::object, nb::object, nb::object, nb::object>(),
          nb::arg("robot").none(),
          nb::arg("sensor_frame").none(),
          nb::arg("point_cloud").none(),
          nb::arg("voxel_grid").none())
      .def_prop_rw(
          "updating",
          [](const DemoPointCloudSensor& self) { return self.get().updating; },
          [](DemoPointCloudSensor& self, bool value) {
            self.get().updating = value;
          })
      .def_prop_rw(
          "sampling_mode",
          [](const DemoPointCloudSensor& self) {
            return static_cast<int>(self.get().samplingMode);
          },
          [](DemoPointCloudSensor& self, int value) {
            if (value < 0 || value > 1)
              throw nb::value_error(
                  "sampling_mode must be 0 (robot) or 1 (box)");
            self.get().samplingMode = static_cast<Sensor::SamplingMode>(value);
          })
      .def_prop_rw(
          "point_shape_type",
          [](const DemoPointCloudSensor& self) {
            return static_cast<int>(
                self.get().getPointCloud()->getPointShapeType());
          },
          [](DemoPointCloudSensor& self, int value) {
            if (value < 0 || value > 3)
              throw nb::value_error("point_shape_type must be between 0 and 3");
            self.get().getPointCloud()->setPointShapeType(
                static_cast<dart::dynamics::PointCloudShape::PointShapeType>(
                    value));
          })
      .def(
          "update",
          [](DemoPointCloudSensor& self, double timeStep) {
            if (!std::isfinite(timeStep) || timeStep <= 0.0)
              throw nb::value_error("time_step must be positive and finite");
            self.get().update(timeStep);
          },
          nb::arg("time_step"));
}

} // namespace dart::python

#endif // HAVE_OCTOMAP
