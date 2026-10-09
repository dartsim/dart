// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include <nanobind/stl/function.h>
#include <nanobind/trampoline.h>

/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

#include <dart/optimizer/Function.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>

namespace dart {
namespace python {

class PyFunction : public dart::optimizer::Function
{
public:
  // Inherit the constructors
  NB_TRAMPOLINE(Function, 3);

  // Trampoline for virtual function
  double eval(const Eigen::VectorXd& x) override
  {
    NB_OVERRIDE_PURE(eval, x);
  }

  // Trampoline for virtual function
  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    NB_OVERRIDE(evalGradient, x, grad);
  }
};

void Function(nb::module_& m)
{
  dartnb::dart_class<dart::optimizer::Function, PyFunction>(m, "Function")
      .def(dartnb::init<>())
      .def(dartnb::init<const std::string&>(), nb::arg("name"))
      .def(
          "setName",
          +[](dart::optimizer::Function* self, const std::string& newName) {
            self->setName(newName);
          },
          nb::arg("newName"))
      .def(
          "getName",
          +[](const dart::optimizer::Function* self) -> const std::string& {
            return self->getName();
          },
          nb::rv_policy::reference_internal);

  dartnb::dart_class<dart::optimizer::NullFunction, dart::optimizer::Function>(
      m, "NullFunction")
      //      .def(dartnb::init<>())
      //      .def(dartnb::init<const std::string &>(),
      //      nb::arg("name"))
      .def(
          "eval",
          +[](dart::optimizer::NullFunction* self,
              const Eigen::VectorXd& _arg0_) -> double {
            return self->eval(_arg0_);
          },
          nb::arg("arg0_"));

  dartnb::dart_class<dart::optimizer::MultiFunction>(m, "MultiFunction");

  dartnb::dart_class<
      dart::optimizer::ModularFunction,
      dart::optimizer::Function>(m, "ModularFunction")
      .def(dartnb::init<>())
      //      .def(dartnb::init<const std::string &>(),
      //      nb::arg("name"))
      .def(
          "eval",
          +[](dart::optimizer::ModularFunction* self,
              const Eigen::VectorXd& _x) -> double { return self->eval(_x); },
          nb::arg("x"))
      .def(
          "setCostFunction",
          +[](dart::optimizer::ModularFunction* self,
              dart::optimizer::CostFunction _cost) {
            self->setCostFunction(_cost);
          },
          nb::arg("cost"))
      .def(
          "clearCostFunction",
          +[](dart::optimizer::ModularFunction* self) {
            self->clearCostFunction();
          })
      .def(
          "clearCostFunction",
          +[](dart::optimizer::ModularFunction* self, bool _printWarning) {
            self->clearCostFunction(_printWarning);
          },
          nb::arg("printWarning"))
      .def(
          "setGradientFunction",
          +[](dart::optimizer::ModularFunction* self,
              dart::optimizer::GradientFunction _gradient) {
            self->setGradientFunction(_gradient);
          },
          nb::arg("gradient"))
      .def(
          "clearGradientFunction",
          +[](dart::optimizer::ModularFunction* self) {
            self->clearGradientFunction();
          })
      .def(
          "setHessianFunction",
          +[](dart::optimizer::ModularFunction* self,
              dart::optimizer::HessianFunction _hessian) {
            self->setHessianFunction(_hessian);
          },
          nb::arg("hessian"))
      .def(
          "clearHessianFunction", +[](dart::optimizer::ModularFunction* self) {
            self->clearHessianFunction();
          });
}

} // namespace python
} // namespace dart
