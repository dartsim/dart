#include "constraint/contact_manifold_cache_options.hpp"

#include "dart/constraint/ContactManifoldCache.hpp"

#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defContactManifoldCacheOptions(nb::module_& m)
{
  using ContactManifoldCacheOptions
      = dart::constraint::ContactManifoldCacheOptions;

  nb::class_<ContactManifoldCacheOptions>(m, "ContactManifoldCacheOptions")
      .def(nb::init<>())
      .def_rw("enabled", &ContactManifoldCacheOptions::enabled)
      .def_rw(
          "maxPointsPerPair", &ContactManifoldCacheOptions::maxPointsPerPair)
      .def_rw("maxPairs", &ContactManifoldCacheOptions::maxPairs)
      .def_rw(
          "maxSeparationFrames",
          &ContactManifoldCacheOptions::maxSeparationFrames)
      .def_rw(
          "positionThreshold", &ContactManifoldCacheOptions::positionThreshold)
      .def_rw("normalThreshold", &ContactManifoldCacheOptions::normalThreshold)
      .def_rw("depthEpsilon", &ContactManifoldCacheOptions::depthEpsilon);
}

} // namespace dart::python_nb
