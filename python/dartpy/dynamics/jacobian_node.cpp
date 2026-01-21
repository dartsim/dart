#include "dynamics/jacobian_node.hpp"

#include "common/type_casters.hpp"
#include "dart/dynamics/degree_of_freedom.hpp"
#include "dart/dynamics/inverse_kinematics.hpp"
#include "dart/dynamics/jacobian_node.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

namespace dart::python_nb {

void defJacobianNode(nb::module_& m)
{
  using JacobianNode = dart::dynamics::JacobianNode;
  using Frame = dart::dynamics::Frame;

  nb::class_<JacobianNode, Frame>(m, "JacobianNode")
      .def("getIK", [](JacobianNode* self) { return self->getIK(); })
      .def(
          "getIK",
          [](JacobianNode* self, bool createIfNull) {
            return self->getIK(createIfNull);
          },
          nb::arg("create_if_null"))
      .def(
          "getOrCreateIK",
          [](JacobianNode* self) { return self->getOrCreateIK(); })
      .def("clearIK", [](JacobianNode* self) { self->clearIK(); })
      .def(
          "dependsOn",
          [](const JacobianNode* self, std::size_t genCoordIndex) {
            return self->dependsOn(genCoordIndex);
          },
          nb::arg("gen_coord_index"))
      .def(
          "getNumDependentGenCoords",
          [](const JacobianNode* self) {
            return self->getNumDependentGenCoords();
          })
      .def(
          "getDependentGenCoordIndex",
          [](const JacobianNode* self, std::size_t arrayIndex) {
            return self->getDependentGenCoordIndex(arrayIndex);
          },
          nb::arg("array_index"))
      .def(
          "getNumDependentDofs",
          [](const JacobianNode* self) { return self->getNumDependentDofs(); })
      .def(
          "getChainDofs",
          [](const JacobianNode* self) { return self->getChainDofs(); })
      .def(
          "getJacobian",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getJacobian(inCoordinatesOf);
          },
          nb::arg("in_coordinates_of"))
      .def(
          "getJacobian",
          [](const JacobianNode* self, const Eigen::Vector3d& offset) {
            return self->getJacobian(offset);
          },
          nb::arg("offset"))
      .def(
          "getJacobian",
          [](const JacobianNode* self,
             const Eigen::Vector3d& offset,
             const Frame* inCoordinatesOf) {
            return self->getJacobian(offset, inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("in_coordinates_of"))
      .def(
          "getWorldJacobian",
          [](const JacobianNode* self, const Eigen::Vector3d& offset) {
            return self->getWorldJacobian(offset);
          },
          nb::arg("offset"))
      .def(
          "getLinearJacobian",
          [](const JacobianNode* self) { return self->getLinearJacobian(); })
      .def(
          "getLinearJacobian",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getLinearJacobian(inCoordinatesOf);
          },
          nb::arg("in_coordinates_of"))
      .def(
          "getLinearJacobian",
          [](const JacobianNode* self, const Eigen::Vector3d& offset) {
            return self->getLinearJacobian(offset);
          },
          nb::arg("offset"))
      .def(
          "getLinearJacobian",
          [](const JacobianNode* self,
             const Eigen::Vector3d& offset,
             const Frame* inCoordinatesOf) {
            return self->getLinearJacobian(offset, inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("in_coordinates_of"))
      .def(
          "getAngularJacobian",
          [](const JacobianNode* self) { return self->getAngularJacobian(); })
      .def(
          "getAngularJacobian",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getAngularJacobian(inCoordinatesOf);
          },
          nb::arg("in_coordinates_of"))
      .def(
          "getJacobianSpatialDeriv",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getJacobianSpatialDeriv(inCoordinatesOf);
          },
          nb::arg("in_coordinates_of"))
      .def(
          "getJacobianSpatialDeriv",
          [](const JacobianNode* self, const Eigen::Vector3d& offset) {
            return self->getJacobianSpatialDeriv(offset);
          },
          nb::arg("offset"))
      .def(
          "getJacobianSpatialDeriv",
          [](const JacobianNode* self,
             const Eigen::Vector3d& offset,
             const Frame* inCoordinatesOf) {
            return self->getJacobianSpatialDeriv(offset, inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("in_coordinates_of"))
      .def(
          "getJacobianClassicDeriv",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getJacobianClassicDeriv(inCoordinatesOf);
          },
          nb::arg("in_coordinates_of"))
      .def(
          "getJacobianClassicDeriv",
          [](const JacobianNode* self, const Eigen::Vector3d& offset) {
            return self->getJacobianClassicDeriv(offset);
          },
          nb::arg("offset"))
      .def(
          "getJacobianClassicDeriv",
          [](const JacobianNode* self,
             const Eigen::Vector3d& offset,
             const Frame* inCoordinatesOf) {
            return self->getJacobianClassicDeriv(offset, inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("in_coordinates_of"))
      .def(
          "getLinearJacobianDeriv",
          [](const JacobianNode* self) {
            return self->getLinearJacobianDeriv();
          })
      .def(
          "getLinearJacobianDeriv",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getLinearJacobianDeriv(inCoordinatesOf);
          },
          nb::arg("in_coordinates_of"))
      .def(
          "getLinearJacobianDeriv",
          [](const JacobianNode* self, const Eigen::Vector3d& offset) {
            return self->getLinearJacobianDeriv(offset);
          },
          nb::arg("offset"))
      .def(
          "getLinearJacobianDeriv",
          [](const JacobianNode* self,
             const Eigen::Vector3d& offset,
             const Frame* inCoordinatesOf) {
            return self->getLinearJacobianDeriv(offset, inCoordinatesOf);
          },
          nb::arg("offset"),
          nb::arg("in_coordinates_of"))
      .def(
          "getAngularJacobianDeriv",
          [](const JacobianNode* self) {
            return self->getAngularJacobianDeriv();
          })
      .def(
          "getAngularJacobianDeriv",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getAngularJacobianDeriv(inCoordinatesOf);
          },
          nb::arg("in_coordinates_of"))
      .def("dirtyJacobian", [](JacobianNode* self) { self->dirtyJacobian(); })
      .def("dirtyJacobianDeriv", [](JacobianNode* self) {
        self->dirtyJacobianDeriv();
      });

  registerPolymorphicCaster<dart::dynamics::Frame, JacobianNode>();
}

} // namespace dart::python_nb
