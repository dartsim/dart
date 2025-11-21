#include "dynamics/skeleton.hpp"

#include "dart/dynamics/BallJoint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/EulerJoint.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/HierarchicalIK.hpp"
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

#include <cstdlib>
#include <cstdio>
#include <memory>
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
  auto skeletonHandle = self.getPtr();
  if (const char* trace = std::getenv("DARTPY_NB_TRACE_CREATE_PAIR")) {
    (void) trace;
    std::fprintf(
        stderr,
        "[dartpy_nb][skeleton][create_pair] joint=%p body=%p skel=%p use_count=%ld\n",
        static_cast<void*>(pair.first),
        static_cast<void*>(pair.second),
        static_cast<void*>(skeletonHandle.get()),
        skeletonHandle ? skeletonHandle.use_count() : 0);
    std::fflush(stderr);
  }
  auto jointHandle = std::shared_ptr<JointT>(skeletonHandle, pair.first);
  auto bodyHandle = std::shared_ptr<dart::dynamics::BodyNode>(
      skeletonHandle, pair.second);
  auto cleanup = [](void* payload) noexcept {
    delete static_cast<std::shared_ptr<dart::dynamics::Skeleton>*>(payload);
  };
  nb::object jointObj = nb::cast(jointHandle, nb::rv_policy::move);
  nb::detail::keep_alive(
      jointObj.ptr(),
      new std::shared_ptr<dart::dynamics::Skeleton>(skeletonHandle),
      cleanup);
  nb::object bodyObj = nb::cast(bodyHandle, nb::rv_policy::move);
  nb::detail::keep_alive(
      bodyObj.ptr(),
      new std::shared_ptr<dart::dynamics::Skeleton>(skeletonHandle),
      cleanup);
  return nb::make_tuple(jointObj, bodyObj);
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
      .def(
          "getNumEndEffectors",
          [](const Skeleton& self) { return self.getNumEndEffectors(); })
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
      .def(
          "getDof",
          [](Skeleton& self, std::size_t idx) { return self.getDof(idx); },
          nb::rv_policy::reference_internal,
          nb::arg("index"))
      .def(
          "getDof",
          [](Skeleton& self, const std::string& name) {
            return self.getDof(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
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
      .def("isEnabledAdjacentBodyCheck", &Skeleton::isEnabledAdjacentBodyCheck)
      .def(
          "getIK",
          [](Skeleton& self, bool createIfNull) {
            return self.getIK(createIfNull);
          },
          nb::arg("createIfNull") = false);

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

  log("getEndEffector");
  skeletonClass
      .def(
          "getEndEffector",
          [](Skeleton& self, const std::string& name) {
            return self.getEndEffector(name);
          },
          nb::rv_policy::reference_internal,
          nb::arg("name"))
      .def(
          "getEndEffector",
          [](Skeleton& self, std::size_t idx) {
            return self.getEndEffector(idx);
          },
          nb::rv_policy::reference_internal,
          nb::arg("idx"));

  log("end");
}

} // namespace dart::python_nb
