#include "collision/continuous_collision.hpp"

#include "common/repr.hpp"
#include "dart/collision/collision_object.hpp"
#include "dart/collision/continuous_collision_option.hpp"
#include "dart/collision/continuous_collision_result.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include <string>

namespace nb = nanobind;

namespace dart::python_nb {

void defContinuousCollision(nb::module_& m)
{
  using ContinuousCollisionAdvancement
      = dart::collision::ContinuousCollisionAdvancement;
  using ContinuousCollisionHit = dart::collision::ContinuousCollisionHit;
  using ContinuousCollisionOption = dart::collision::ContinuousCollisionOption;
  using ContinuousCollisionResult = dart::collision::ContinuousCollisionResult;

  nb::enum_<ContinuousCollisionAdvancement>(m, "ContinuousCollisionAdvancement")
      .value("Conservative", ContinuousCollisionAdvancement::Conservative)
      .value("Fast", ContinuousCollisionAdvancement::Fast);

  nb::class_<ContinuousCollisionOption>(m, "ContinuousCollisionOption")
      .def(
          nb::init<bool, bool, double, int, ContinuousCollisionAdvancement>(),
          nb::arg("enable_all_hits") = false,
          nb::arg("sort_by_time_of_impact") = false,
          nb::arg("tolerance") = 1e-4,
          nb::arg("max_iterations") = 32,
          nb::arg("advancement") = ContinuousCollisionAdvancement::Conservative)
      .def_rw("mEnableAllHits", &ContinuousCollisionOption::mEnableAllHits)
      .def_rw(
          "mSortByTimeOfImpact",
          &ContinuousCollisionOption::mSortByTimeOfImpact)
      .def_rw("mTolerance", &ContinuousCollisionOption::mTolerance)
      .def_rw("mMaxIterations", &ContinuousCollisionOption::mMaxIterations)
      .def_rw("mAdvancement", &ContinuousCollisionOption::mAdvancement)
      .def("__repr__", [](const ContinuousCollisionOption& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("all_hits", repr_bool(self.mEnableAllHits));
        fields.emplace_back(
            "sort_by_time", repr_bool(self.mSortByTimeOfImpact));
        fields.emplace_back("tolerance", std::to_string(self.mTolerance));
        fields.emplace_back(
            "max_iterations", std::to_string(self.mMaxIterations));
        fields.emplace_back(
            "advancement",
            self.mAdvancement == ContinuousCollisionAdvancement::Conservative
                ? "Conservative"
                : "Fast");
        return format_repr("ContinuousCollisionOption", fields);
      });

  nb::class_<ContinuousCollisionHit>(m, "ContinuousCollisionHit")
      .def(nb::init<>())
      .def_rw("mCollisionObject", &ContinuousCollisionHit::mCollisionObject)
      .def_rw("mTimeOfImpact", &ContinuousCollisionHit::mTimeOfImpact)
      .def_rw("mPoint", &ContinuousCollisionHit::mPoint)
      .def_rw("mNormal", &ContinuousCollisionHit::mNormal);

  nb::class_<ContinuousCollisionResult>(m, "ContinuousCollisionResult")
      .def(nb::init<>())
      .def("clear", &ContinuousCollisionResult::clear)
      .def("hasHit", &ContinuousCollisionResult::hasHit)
      .def_rw("mHits", &ContinuousCollisionResult::mHits)
      .def("__repr__", [](const ContinuousCollisionResult& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("has_hit", repr_bool(self.hasHit()));
        fields.emplace_back("hits", std::to_string(self.mHits.size()));
        return format_repr("ContinuousCollisionResult", fields);
      });
}

} // namespace dart::python_nb
