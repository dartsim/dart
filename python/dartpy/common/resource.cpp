#include "common/resource.hpp"

#include "dart/common/local_resource.hpp"
#include "dart/common/resource.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defResource(nb::module_& m)
{
  using Resource = dart::common::Resource;

  nb::enum_<Resource::SeekType>(m, "ResourceSeekType")
      .value("SEEKTYPE_CUR", Resource::SEEKTYPE_CUR)
      .value("SEEKTYPE_END", Resource::SEEKTYPE_END)
      .value("SEEKTYPE_SET", Resource::SEEKTYPE_SET);

  nb::class_<Resource>(m, "Resource")
      .def("getSize", &Resource::getSize)
      .def("tell", &Resource::tell)
      .def("seek", &Resource::seek, nb::arg("offset"), nb::arg("origin"))
      .def(
          "read",
          &Resource::read,
          nb::arg("buffer"),
          nb::arg("size"),
          nb::arg("count"))
      .def("readAll", &Resource::readAll);

  using LocalResource = dart::common::LocalResource;
  nb::class_<LocalResource, Resource>(m, "LocalResource")
      .def(nb::init<const std::string&>(), nb::arg("path"))
      .def("isGood", &LocalResource::isGood);
}

} // namespace dart::python_nb
