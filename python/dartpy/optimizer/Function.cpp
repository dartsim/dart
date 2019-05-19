/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <dart/dart.hpp>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include "eigen_geometry_pybind.h"
#include "eigen_pybind.h"

namespace dart {
namespace python {

class PyFunction : public dart::optimizer::Function
{
public:
  // Inherit the constructors
  using Function::Function;

  // Trampoline for virtual function
  double eval(const Eigen::VectorXd& x) override
  {
    PYBIND11_OVERLOAD_PURE(
        double,   // Return type
        Function, // Parent class
        eval,     // Name of function in C++ (must match Python name)
        x);
  }

  // Trampoline for virtual function
  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    PYBIND11_OVERLOAD(
        void,         // Return type
        Function,     // Parent class
        evalGradient, // Name of function in C++ (must match Python name)
        x,
        grad);
  }
};

void Function(pybind11::module& m)
{
  ::pybind11::class_<
      dart::optimizer::Function,
      PyFunction,
      std::shared_ptr<dart::optimizer::Function>>(m, "Function")
      .def(::pybind11::init<>())
      .def(::pybind11::init<const std::string&>(), ::pybind11::arg("name"))
      .def(
          "setName",
          +[](dart::optimizer::Function* self, const std::string& newName) {
            self->setName(newName);
          },
          ::pybind11::arg("newName"))
      .def(
          "getName",
          +[](const dart::optimizer::Function* self) -> const std::string& {
            return self->getName();
          },
          ::pybind11::return_value_policy::reference_internal)
      .def(
          "eval",
          +[](dart::optimizer::Function* self,
              const Eigen::VectorXd& x) -> double { return self->eval(x); },
          ::pybind11::arg("x"))
      .def(
          "evalGradient",
          +[](dart::optimizer::Function* self,
              const Eigen::VectorXd& _x,
              Eigen::VectorXd& _grad) { self->evalGradient(_x, _grad); },
          ::pybind11::arg("x"),
          ::pybind11::arg("grad"));

  ::pybind11::class_<
      dart::optimizer::NullFunction,
      dart::optimizer::Function,
      std::shared_ptr<dart::optimizer::NullFunction>>(m, "NullFunction")
      //      .def(::pybind11::init<>())
      //      .def(::pybind11::init<const std::string &>(),
      //      ::pybind11::arg("name"))
      .def(
          "eval",
          +[](dart::optimizer::NullFunction* self,
              const Eigen::VectorXd& _arg0_) -> double {
            return self->eval(_arg0_);
          },
          ::pybind11::arg("arg0_"));

  ::pybind11::class_<
      dart::optimizer::MultiFunction,
      std::shared_ptr<dart::optimizer::MultiFunction>>(m, "MultiFunction");

  ::pybind11::class_<
      dart::optimizer::ModularFunction,
      dart::optimizer::Function,
      std::shared_ptr<dart::optimizer::ModularFunction>>(m, "ModularFunction")
      //      .def(::pybind11::init<>())
      //      .def(::pybind11::init<const std::string &>(),
      //      ::pybind11::arg("name"))
      .def(
          "eval",
          +[](dart::optimizer::ModularFunction* self,
              const Eigen::VectorXd& _x) -> double { return self->eval(_x); },
          ::pybind11::arg("x"))
      .def(
          "setCostFunction",
          +[](dart::optimizer::ModularFunction* self,
              dart::optimizer::CostFunction _cost) {
            self->setCostFunction(_cost);
          },
          ::pybind11::arg("cost"))
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
          ::pybind11::arg("printWarning"))
      .def(
          "setGradientFunction",
          +[](dart::optimizer::ModularFunction* self,
              dart::optimizer::GradientFunction _gradient) {
            self->setGradientFunction(_gradient);
          },
          ::pybind11::arg("gradient"))
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
          ::pybind11::arg("hessian"))
      .def("clearHessianFunction", +[](dart::optimizer::ModularFunction* self) {
        self->clearHessianFunction();
      });
}

} // namespace python
} // namespace dart
