#include "dynamics/skeleton.hpp"

#include "dart/dynamics/BallJoint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/EulerJoint.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/MetaSkeleton.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ScrewJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/TranslationalJoint.hpp"
#include "dart/dynamics/TranslationalJoint2D.hpp"
#include "dart/dynamics/UniversalJoint.hpp"
#include "dart/dynamics/WeldJoint.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "common/type_casters.hpp"

#include <cstdio>
#include <utility>

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

template <typename JointT>
auto create_pair(
    dart::dynamics::Skeleton& self,
    dart::dynamics::BodyNode* parent,
    const typename JointT::Properties& properties)
{
  auto pair = self.createJointAndBodyNodePair<JointT, dart::dynamics::BodyNode>(
      parent, properties);
  return std::make_pair(
      static_cast<dart::dynamics::Joint*>(pair.first), pair.second);
}

template <typename JointT>
auto create_pair_with_default(
    dart::dynamics::Skeleton& self,
    dart::dynamics::BodyNode* parent,
    const nb::handle& propsHandle)
{
  using Properties = typename JointT::Properties;
  Properties props
      = propsHandle.is_none() ? Properties() : nb::cast<Properties>(propsHandle);
  return create_pair<JointT>(self, parent, props);
}

} // namespace

void defSkeleton(nb::module_& m)
{
  using Skeleton = dart::dynamics::Skeleton;
  using BodyNode [[maybe_unused]] = dart::dynamics::BodyNode;

  const bool trace = std::getenv("DARTPY_NB_TRACE_SKELETON") != nullptr;
  auto log = [&](const char* stage) {
    if (trace)
      std::fprintf(stderr, "[dartpy_nb][skeleton] %s\n", stage);
  };

  log("begin");

  auto skeletonClass
      = nb::class_<Skeleton, dart::dynamics::MetaSkeleton>(m, "Skeleton");

  skeletonClass
      .def(nb::new_([]() { return Skeleton::create(); }))
      .def(
          nb::new_([](const std::string& name) { return Skeleton::create(name); }),
          nb::arg("name"))
      .def("getNumBodyNodes", &Skeleton::getNumBodyNodes)
      .def("getNumJoints", &Skeleton::getNumJoints)
      .def("getNumDofs", &Skeleton::getNumDofs)
      .def(
          "getBodyNode",
          [](Skeleton& self, std::size_t idx) -> BodyNode* {
            return self.getBodyNode(idx);
          },
          nb::rv_policy::reference_internal)
      .def(
          "getBodyNode",
          [](Skeleton& self, const std::string& name) -> BodyNode* {
            return self.getBodyNode(name);
          },
          nb::rv_policy::reference_internal)
      .def(
          "getRootBodyNode",
          [](Skeleton& self) -> BodyNode* { return self.getRootBodyNode(); },
          nb::rv_policy::reference_internal)
      .def(
          "getJoint",
          [](Skeleton& self, std::size_t idx) -> dart::dynamics::Joint* {
            return self.getJoint(idx);
          },
          nb::rv_policy::reference_internal)
      .def(
          "getJoint",
          [](Skeleton& self, const std::string& name)
              -> dart::dynamics::Joint* { return self.getJoint(name); },
          nb::rv_policy::reference_internal)
      .def(
          "getRootJoint",
          [](Skeleton& self) -> dart::dynamics::Joint* {
            return self.getRootJoint();
          },
          nb::rv_policy::reference_internal)
      .def("getPositions", [](Skeleton& self) { return self.getPositions(); })
      .def(
          "setPositions",
          [](Skeleton& self, const Eigen::VectorXd& positions) {
            self.setPositions(positions);
          },
          nb::arg("positions"))
      .def("resetPositions", &Skeleton::resetPositions)
      .def("enableSelfCollisionCheck", &Skeleton::enableSelfCollisionCheck)
      .def("disableSelfCollisionCheck", &Skeleton::disableSelfCollisionCheck)
      .def(
          "isEnabledSelfCollisionCheck", &Skeleton::isEnabledSelfCollisionCheck)
      .def("enableAdjacentBodyCheck", &Skeleton::enableAdjacentBodyCheck)
      .def("disableAdjacentBodyCheck", &Skeleton::disableAdjacentBodyCheck)
      .def("isEnabledAdjacentBodyCheck", &Skeleton::isEnabledAdjacentBodyCheck);

  log("createFreeJointAndBodyNodePair");
  skeletonClass.def(
      "createFreeJointAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::FreeJoint>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("createBallJointAndBodyNodePair");
  skeletonClass.def(
      "createBallJointAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::BallJoint>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("createEulerJointAndBodyNodePair");
  skeletonClass.def(
      "createEulerJointAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::EulerJoint>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("createRevoluteJointAndBodyNodePair");
  skeletonClass.def(
      "createRevoluteJointAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::RevoluteJoint>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("createPrismaticJointAndBodyNodePair");
  skeletonClass.def(
      "createPrismaticJointAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::PrismaticJoint>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("createScrewJointAndBodyNodePair");
  skeletonClass.def(
      "createScrewJointAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::ScrewJoint>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("createPlanarJointAndBodyNodePair");
  skeletonClass.def(
      "createPlanarJointAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::PlanarJoint>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("createUniversalJointAndBodyNodePair");
  skeletonClass.def(
      "createUniversalJointAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::UniversalJoint>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("createTranslationalJointAndBodyNodePair");
  skeletonClass.def(
      "createTranslationalJointAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::TranslationalJoint>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("createTranslationalJoint2DAndBodyNodePair");
  skeletonClass.def(
      "createTranslationalJoint2DAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::TranslationalJoint2D>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("createWeldJointAndBodyNodePair");
  skeletonClass.def(
      "createWeldJointAndBodyNodePair",
      [](Skeleton& self, BodyNode* parent, const nb::handle& props) {
        return create_pair_with_default<dart::dynamics::WeldJoint>(
            self, parent, props);
      },
      nb::rv_policy::reference_internal,
      nb::arg("parent") = nullptr,
      nb::arg("jointProperties") = nb::none());

  log("end");
}

} // namespace dart::python_nb
