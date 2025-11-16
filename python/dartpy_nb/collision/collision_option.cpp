#include "collision/collision_option.hpp"

#include "dart/collision/CollisionFilter.hpp"
#include "dart/collision/CollisionOption.hpp"
#include "dart/dynamics/BodyNode.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defCollisionOption(nb::module_& m)
{
  using CollisionFilter = dart::collision::CollisionFilter;
  using BodyNodeCollisionFilter = dart::collision::BodyNodeCollisionFilter;
  using CollisionOption = dart::collision::CollisionOption;

  nb::class_<CollisionFilter>(m, "CollisionFilter");

  nb::class_<BodyNodeCollisionFilter, CollisionFilter>(
      m, "BodyNodeCollisionFilter")
      .def(nb::init<>())
      .def(
          "addBodyNodePairToBlackList",
          &BodyNodeCollisionFilter::addBodyNodePairToBlackList,
          nb::arg("bodyNode1"),
          nb::arg("bodyNode2"))
      .def(
          "removeBodyNodePairFromBlackList",
          &BodyNodeCollisionFilter::removeBodyNodePairFromBlackList,
          nb::arg("bodyNode1"),
          nb::arg("bodyNode2"))
      .def(
          "removeAllBodyNodePairsFromBlackList",
          &BodyNodeCollisionFilter::removeAllBodyNodePairsFromBlackList);

  nb::class_<CollisionOption>(m, "CollisionOption")
      .def(
          nb::init<
              bool,
              std::size_t,
              const std::shared_ptr<CollisionFilter>&>(),
          nb::arg("enableContact") = true,
          nb::arg("maxNumContacts") = 1000u,
          nb::arg("collisionFilter") = nullptr)
      .def_rw("enableContact", &CollisionOption::enableContact)
      .def_rw("maxNumContacts", &CollisionOption::maxNumContacts)
      .def_rw("collisionFilter", &CollisionOption::collisionFilter);
}

} // namespace dart::python_nb
