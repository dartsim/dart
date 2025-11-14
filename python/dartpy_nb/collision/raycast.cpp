#include "collision/raycast.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include "dart/collision/RaycastOption.hpp"
#include "dart/collision/RaycastResult.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defRaycast(nb::module_& m)
{
  using RaycastOption = dart::collision::RaycastOption;
  using RaycastResult = dart::collision::RaycastResult;
  using RayHit = dart::collision::RayHit;

  nb::class_<RaycastOption>(m, "RaycastOption")
      .def(nb::init<>())
      .def_readwrite("mEnableAllHits", &RaycastOption::mEnableAllHits);

  nb::class_<RayHit>(m, "RayHit")
      .def(nb::init<>())
      .def_readwrite("mCollisionObject", &RayHit::mCollisionObject)
      .def_readwrite("mPoint", &RayHit::mPoint)
      .def_readwrite("mFraction", &RayHit::mFraction)
      .def_readwrite("mNormal", &RayHit::mNormal);

  nb::class_<RaycastResult>(m, "RaycastResult")
      .def(nb::init<>())
      .def("clear", &RaycastResult::clear)
      .def("hasHit", &RaycastResult::hasHit)
      .def_readwrite("mRayHits", &RaycastResult::mRayHits);
}

} // namespace dart::python_nb
