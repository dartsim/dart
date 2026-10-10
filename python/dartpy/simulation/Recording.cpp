// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 * See LICENSE for the BSD-style license governing this file.
 */

#include "eigen_pybind.h"

#include <dart/simulation/Recording.hpp>

#include <string>

namespace dart {
namespace python {

namespace {

void checkRecordingIndex(int index, int count, const char* kind)
{
  if (index < 0 || index >= count)
    throw nb::index_error((std::string(kind) + " index out of range").c_str());
}

} // namespace

void Recording(nb::module_& m)
{
  using Record = dart::simulation::Recording;
  dartnb::dart_class<Record>(m, "Recording")
      .def("getNumFrames", &Record::getNumFrames)
      .def("getNumSkeletons", &Record::getNumSkeletons)
      .def(
          "getNumDofs",
          [](const Record& self, int skeleton) {
            checkRecordingIndex(skeleton, self.getNumSkeletons(), "skeleton");
            return self.getNumDofs(skeleton);
          },
          nb::arg("skeleton"))
      .def(
          "getNumContacts",
          [](const Record& self, int frame) {
            checkRecordingIndex(frame, self.getNumFrames(), "frame");
            return self.getNumContacts(frame);
          },
          nb::arg("frame"))
      .def(
          "getConfig",
          [](const Record& self, int frame, int skeleton) {
            checkRecordingIndex(frame, self.getNumFrames(), "frame");
            checkRecordingIndex(skeleton, self.getNumSkeletons(), "skeleton");
            return self.getConfig(frame, skeleton);
          },
          nb::arg("frame"),
          nb::arg("skeleton"))
      .def(
          "getContactPoint",
          [](const Record& self, int frame, int contact) {
            checkRecordingIndex(frame, self.getNumFrames(), "frame");
            checkRecordingIndex(contact, self.getNumContacts(frame), "contact");
            return self.getContactPoint(frame, contact);
          },
          nb::arg("frame"),
          nb::arg("contact"))
      .def(
          "getContactForce",
          [](const Record& self, int frame, int contact) {
            checkRecordingIndex(frame, self.getNumFrames(), "frame");
            checkRecordingIndex(contact, self.getNumContacts(frame), "contact");
            return self.getContactForce(frame, contact);
          },
          nb::arg("frame"),
          nb::arg("contact"))
      .def("clear", &Record::clear);
}

} // namespace python
} // namespace dart
