#include "dynamics/jacobian_node.hpp"

#include "dart/dynamics/InverseKinematics.hpp"
#include "dart/dynamics/JacobianNode.hpp"
#include "dart/dynamics/DegreeOfFreedom.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "common/type_casters.hpp"

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
          nb::arg("createIfNull"))
      .def("getOrCreateIK", [](JacobianNode* self) {
        return self->getOrCreateIK();
      })
      .def("clearIK", [](JacobianNode* self) { self->clearIK(); })
      .def(
          "dependsOn",
          [](const JacobianNode* self, std::size_t genCoordIndex) {
            return self->dependsOn(genCoordIndex);
          },
          nb::arg("genCoordIndex"))
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
          nb::arg("arrayIndex"))
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
          nb::arg("inCoordinatesOf"))
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
          nb::arg("inCoordinatesOf"))
      .def(
          "getWorldJacobian",
          [](const JacobianNode* self, const Eigen::Vector3d& offset) {
            return self->getWorldJacobian(offset);
          },
          nb::arg("offset"))
      .def("getLinearJacobian", [](const JacobianNode* self) {
        return self->getLinearJacobian();
      })
      .def(
          "getLinearJacobian",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getLinearJacobian(inCoordinatesOf);
          },
          nb::arg("inCoordinatesOf"))
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
          nb::arg("inCoordinatesOf"))
      .def("getAngularJacobian", [](const JacobianNode* self) {
        return self->getAngularJacobian();
      })
      .def(
          "getAngularJacobian",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getAngularJacobian(inCoordinatesOf);
          },
          nb::arg("inCoordinatesOf"))
      .def(
          "getJacobianSpatialDeriv",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getJacobianSpatialDeriv(inCoordinatesOf);
          },
          nb::arg("inCoordinatesOf"))
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
          nb::arg("inCoordinatesOf"))
      .def(
          "getJacobianClassicDeriv",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getJacobianClassicDeriv(inCoordinatesOf);
          },
          nb::arg("inCoordinatesOf"))
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
          nb::arg("inCoordinatesOf"))
      .def("getLinearJacobianDeriv", [](const JacobianNode* self) {
        return self->getLinearJacobianDeriv();
      })
      .def(
          "getLinearJacobianDeriv",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getLinearJacobianDeriv(inCoordinatesOf);
          },
          nb::arg("inCoordinatesOf"))
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
          nb::arg("inCoordinatesOf"))
      .def("getAngularJacobianDeriv", [](const JacobianNode* self) {
        return self->getAngularJacobianDeriv();
      })
      .def(
          "getAngularJacobianDeriv",
          [](const JacobianNode* self, const Frame* inCoordinatesOf) {
            return self->getAngularJacobianDeriv(inCoordinatesOf);
          },
          nb::arg("inCoordinatesOf"))
      .def("dirtyJacobian", [](JacobianNode* self) { self->dirtyJacobian(); })
      .def("dirtyJacobianDeriv", [](JacobianNode* self) {
        self->dirtyJacobianDeriv();
      });

  registerPolymorphicCaster<dart::dynamics::Frame, JacobianNode>();
}

} // namespace dart::python_nb
