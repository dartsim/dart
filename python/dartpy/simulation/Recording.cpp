/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 * See LICENSE for the BSD-style license governing this file.
 */

#include "eigen_pybind.h"

#include <dart/simulation/Recording.hpp>

#include <pybind11/pybind11.h>

#include <string>

namespace py = pybind11;

namespace dart {
namespace python {

namespace {

void checkRecordingIndex(int index, int count, const char* kind)
{
  if (index < 0 || index >= count)
    throw py::index_error(std::string(kind) + " index out of range");
}

} // namespace

void Recording(py::module& m)
{
  using Record = dart::simulation::Recording;
  py::class_<Record>(m, "Recording")
      .def("getNumFrames", &Record::getNumFrames)
      .def("getNumSkeletons", &Record::getNumSkeletons)
      .def(
          "getNumDofs",
          [](const Record& self, int skeleton) {
            checkRecordingIndex(skeleton, self.getNumSkeletons(), "skeleton");
            return self.getNumDofs(skeleton);
          },
          py::arg("skeleton"))
      .def(
          "getNumContacts",
          [](const Record& self, int frame) {
            checkRecordingIndex(frame, self.getNumFrames(), "frame");
            return self.getNumContacts(frame);
          },
          py::arg("frame"))
      .def(
          "getConfig",
          [](const Record& self, int frame, int skeleton) {
            checkRecordingIndex(frame, self.getNumFrames(), "frame");
            checkRecordingIndex(skeleton, self.getNumSkeletons(), "skeleton");
            return self.getConfig(frame, skeleton);
          },
          py::arg("frame"),
          py::arg("skeleton"))
      .def(
          "getContactPoint",
          [](const Record& self, int frame, int contact) {
            checkRecordingIndex(frame, self.getNumFrames(), "frame");
            checkRecordingIndex(contact, self.getNumContacts(frame), "contact");
            return self.getContactPoint(frame, contact);
          },
          py::arg("frame"),
          py::arg("contact"))
      .def(
          "getContactForce",
          [](const Record& self, int frame, int contact) {
            checkRecordingIndex(frame, self.getNumFrames(), "frame");
            checkRecordingIndex(contact, self.getNumContacts(frame), "contact");
            return self.getContactForce(frame, contact);
          },
          py::arg("frame"),
          py::arg("contact"))
      .def("clear", &Record::clear);
}

} // namespace python
} // namespace dart
