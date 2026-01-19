#include "collision/raycast.hpp"

#include "dart/collision/collision_object.hpp"
#include "dart/collision/raycast_option.hpp"
#include "dart/collision/raycast_result.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defRaycast(nb::module_& m)
{
  using RaycastOption = dart::collision::RaycastOption;
  using RaycastResult = dart::collision::RaycastResult;
  using RayHit = dart::collision::RayHit;

  nb::class_<RaycastOption>(m, "RaycastOption")
      .def(nb::init<>())
      .def_rw("mEnableAllHits", &RaycastOption::mEnableAllHits);

  nb::class_<RayHit>(m, "RayHit")
      .def(nb::init<>())
      .def_rw("mCollisionObject", &RayHit::mCollisionObject)
      .def_rw("mPoint", &RayHit::mPoint)
      .def_rw("mFraction", &RayHit::mFraction)
      .def_rw("mNormal", &RayHit::mNormal);

  nb::class_<RaycastResult>(m, "RaycastResult")
      .def(nb::init<>())
      .def("clear", &RaycastResult::clear)
      .def("hasHit", &RaycastResult::hasHit)
      .def_rw("mRayHits", &RaycastResult::mRayHits);
}

} // namespace dart::python_nb
