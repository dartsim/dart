#include "dynamics/skeleton.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>

#include "dart/dynamics/BallJoint.hpp"
#include "dart/dynamics/BodyNode.hpp"
#include "dart/dynamics/EulerJoint.hpp"
#include "dart/dynamics/FreeJoint.hpp"
#include "dart/dynamics/PlanarJoint.hpp"
#include "dart/dynamics/PrismaticJoint.hpp"
#include "dart/dynamics/RevoluteJoint.hpp"
#include "dart/dynamics/ScrewJoint.hpp"
#include "dart/dynamics/Skeleton.hpp"
#include "dart/dynamics/TranslationalJoint.hpp"
#include "dart/dynamics/TranslationalJoint2D.hpp"
#include "dart/dynamics/UniversalJoint.hpp"
#include "dart/dynamics/WeldJoint.hpp"

namespace nb = nanobind;

namespace dart::python_nb {

namespace {

template <typename JointT>
auto create_pair(dart::dynamics::Skeleton& self,
    dart::dynamics::BodyNode* parent,
    const typename JointT::Properties& properties)
{
  auto pair = self.createJointAndBodyNodePair<JointT, dart::dynamics::BodyNode>(
      parent, properties);
  return nb::make_tuple(pair.first, pair.second);
}

} // namespace

void defSkeleton(nb::module_& m)
{
  using Skeleton = dart::dynamics::Skeleton;
  using BodyNode = dart::dynamics::BodyNode;

  nb::class_<Skeleton, std::shared_ptr<Skeleton>>(m, "Skeleton")
      .def(nb::init<>())
      .def("getNumBodyNodes", &Skeleton::getNumBodyNodes)
      .def("getNumJoints", &Skeleton::getNumJoints)
      .def("getNumDofs", &Skeleton::getNumDofs)
      .def("getBodyNode",
          [](Skeleton& self, std::size_t idx) -> BodyNode* {
            return self.getBodyNode(idx);
          },
          nb::rv_policy::reference_internal)
      .def("getBodyNode",
          [](Skeleton& self, const std::string& name) -> BodyNode* {
            return self.getBodyNode(name);
          },
          nb::rv_policy::reference_internal)
      .def("getRootBodyNode",
          [](Skeleton& self) -> BodyNode* {
            return self.getRootBodyNode();
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
      .def("getRootJoint",
          [](Skeleton& self) -> dart::dynamics::Joint* {
            return self.getRootJoint();
          },
          nb::rv_policy::reference_internal)
      .def("getPositions",
          [](Skeleton& self) {
            return self.getPositions();
          })
      .def("setPositions",
          [](Skeleton& self, const Eigen::VectorXd& positions) {
            self.setPositions(positions);
          },
          nb::arg("positions"))
      .def("resetPositions", &Skeleton::resetPositions)
      .def("enableSelfCollisionCheck",
          &Skeleton::enableSelfCollisionCheck)
      .def("disableSelfCollisionCheck",
          &Skeleton::disableSelfCollisionCheck)
      .def("isEnabledSelfCollisionCheck",
          &Skeleton::isEnabledSelfCollisionCheck)
      .def("enableAdjacentBodyCheck",
          &Skeleton::enableAdjacentBodyCheck)
      .def("disableAdjacentBodyCheck",
          &Skeleton::disableAdjacentBodyCheck)
      .def("isEnabledAdjacentBodyCheck",
          &Skeleton::isEnabledAdjacentBodyCheck)
      .def("createFreeJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::FreeJoint::Properties& props) {
            return create_pair<dart::dynamics::FreeJoint>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::FreeJoint::Properties())
      .def("createRevoluteJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::RevoluteJoint::Properties& props) {
            return create_pair<dart::dynamics::RevoluteJoint>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::RevoluteJoint::Properties())
      .def("createPrismaticJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::PrismaticJoint::Properties& props) {
            return create_pair<dart::dynamics::PrismaticJoint>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::PrismaticJoint::Properties())
      .def("createWeldJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::WeldJoint::Properties& props) {
            return create_pair<dart::dynamics::WeldJoint>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::WeldJoint::Properties())
      .def("createScrewJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::ScrewJoint::Properties& props) {
            return create_pair<dart::dynamics::ScrewJoint>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::ScrewJoint::Properties())
      .def("createUniversalJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::UniversalJoint::Properties& props) {
            return create_pair<dart::dynamics::UniversalJoint>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::UniversalJoint::Properties())
      .def("createTranslationalJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::TranslationalJoint::Properties& props) {
            return create_pair<dart::dynamics::TranslationalJoint>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::TranslationalJoint::Properties())
      .def("createTranslationalJoint2DAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::TranslationalJoint2D::Properties& props) {
            return create_pair<dart::dynamics::TranslationalJoint2D>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::TranslationalJoint2D::Properties())
      .def("createEulerJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::EulerJoint::Properties& props) {
            return create_pair<dart::dynamics::EulerJoint>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::EulerJoint::Properties())
      .def("createPlanarJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::PlanarJoint::Properties& props) {
            return create_pair<dart::dynamics::PlanarJoint>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::PlanarJoint::Properties())
      .def("createBallJointAndBodyNodePair",
          [](Skeleton& self,
              BodyNode* parent,
              const dart::dynamics::BallJoint::Properties& props) {
            return create_pair<dart::dynamics::BallJoint>(self, parent, props);
          },
          nb::arg("parent") = nullptr,
          nb::arg("jointProperties") = dart::dynamics::BallJoint::Properties());
}

} // namespace dart::python_nb
