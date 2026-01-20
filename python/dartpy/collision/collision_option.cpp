#include "collision/collision_option.hpp"

#include "common/repr.hpp"
#include "dart/collision/collision_filter.hpp"
#include "dart/collision/collision_option.hpp"
#include "dart/dynamics/body_node.hpp"

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
          nb::arg("body_node1"),
          nb::arg("body_node2"))
      .def(
          "removeBodyNodePairFromBlackList",
          &BodyNodeCollisionFilter::removeBodyNodePairFromBlackList,
          nb::arg("body_node1"),
          nb::arg("body_node2"))
      .def(
          "removeAllBodyNodePairsFromBlackList",
          &BodyNodeCollisionFilter::removeAllBodyNodePairsFromBlackList);

  nb::class_<CollisionOption>(m, "CollisionOption")
      .def(
          nb::init<
              bool,
              std::size_t,
              const std::shared_ptr<CollisionFilter>&,
              bool>(),
          nb::arg("enable_contact") = true,
          nb::arg("max_num_contacts") = 1000u,
          nb::arg("collision_filter") = nullptr,
          nb::arg("allow_negative_penetration_depth_contacts") = false)
      .def_rw("enableContact", &CollisionOption::enableContact)
      .def_rw("maxNumContacts", &CollisionOption::maxNumContacts)
      .def_rw(
          "allowNegativePenetrationDepthContacts",
          &CollisionOption::allowNegativePenetrationDepthContacts)
      .def_rw("collisionFilter", &CollisionOption::collisionFilter)
      .def("__repr__", [](const CollisionOption& self) {
        std::vector<std::pair<std::string, std::string>> fields;
        fields.emplace_back("enable_contact", repr_bool(self.enableContact));
        fields.emplace_back(
            "max_contacts", std::to_string(self.maxNumContacts));
        fields.emplace_back(
            "allow_negative_penetration_depth_contacts",
            repr_bool(self.allowNegativePenetrationDepthContacts));
        fields.emplace_back(
            "has_filter", self.collisionFilter ? "True" : "False");
        return format_repr("CollisionOption", fields);
      });
}

} // namespace dart::python_nb
