#include "dynamics/skeleton.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

void defSkeleton(nb::module_& m)
{
  using Skeleton = dart::dynamics::Skeleton;
  using FreeJoint = dart::dynamics::FreeJoint;
  using RevoluteJoint = dart::dynamics::RevoluteJoint;
  using BodyNode = dart::dynamics::BodyNode;

  nb::class_<Skeleton, std::shared_ptr<Skeleton>>(m, "Skeleton")
      .def(nb::init<>())
      .def("getNumBodyNodes", &Skeleton::getNumBodyNodes)
      .def("getNumJoints", &Skeleton::getNumJoints)
      .def("getBodyNode",
          [](Skeleton& self, std::size_t idx) -> BodyNode* {
            return self.getBodyNode(idx);
          },
          nb::rv_policy::reference_internal)
      .def("getRootBodyNode",
          [](Skeleton& self) -> BodyNode* {
            return self.getBodyNode(0);
          },
          nb::rv_policy::reference_internal)
      .def("getBodyNode",
          [](Skeleton& self, const std::string& name) -> BodyNode* {
            return self.getBodyNode(name);
          },
          nb::rv_policy::reference_internal)
      .def("getJoint",
          [](Skeleton& self, std::size_t idx) -> dart::dynamics::Joint* {
            return self.getJoint(idx);
          },
          nb::rv_policy::reference_internal)
      .def("getJoint",
          [](Skeleton& self, const std::string& name) -> dart::dynamics::Joint* {
            return self.getJoint(name);
          },
          nb::rv_policy::reference_internal)
      .def("createFreeJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const FreeJoint::Properties& joint_props) {
            auto pair = self.template createJointAndBodyNodePair<
                FreeJoint,
                BodyNode>(parent, joint_props);
            return nb::make_tuple(pair.first, pair.second);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = FreeJoint::Properties())
      .def("createRevoluteJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const RevoluteJoint::Properties& joint_props) {
            auto pair = self.template createJointAndBodyNodePair<
                RevoluteJoint,
                BodyNode>(parent, joint_props);
            return nb::make_tuple(pair.first, pair.second);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = RevoluteJoint::Properties());
}

} // namespace dart::python_nb
