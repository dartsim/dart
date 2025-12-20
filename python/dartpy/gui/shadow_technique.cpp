#include "gui/gui.hpp"
#include "gui/utils.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <osgShadow/ShadowMap>
#include <osgShadow/ShadowTechnique>

namespace nb = nanobind;

namespace dart::python_nb {

void defShadowTechnique(nb::module_& m)
{
  nb::class_<osgShadow::ShadowTechnique>(m, "ShadowTechnique");

  nb::class_<osgShadow::ShadowMap, osgShadow::ShadowTechnique>(m, "ShadowMap")
      .def(nb::init<>());
}

} // namespace dart::python_nb
