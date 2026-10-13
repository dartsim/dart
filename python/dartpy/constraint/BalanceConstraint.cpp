// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/eigen.hpp"

/*
 * Copyright (c) 2011-2026, The DART development contributors
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

#include <dart/constraint/BalanceConstraint.hpp>

#include <dart/dynamics/Skeleton.hpp>

namespace dart {
namespace python {
namespace {
struct BalanceOwner
{
  std::shared_ptr<dart::constraint::BalanceConstraint> constraint;
  std::shared_ptr<dart::dynamics::HierarchicalIK> ik;
  dart::dynamics::SkeletonPtr skeleton;

  auto get_shared() const
  {
    return constraint;
  }

  auto* get() const
  {
    return constraint.get();
  }

  // Never a second owner of the constraint (see dartnb::nativeInstance).
  auto retained() const
  {
    return std::make_pair(ik, skeleton);
  }
};
} // namespace

void BalanceConstraint(nb::module_& m)
{
  using Balance = dart::constraint::BalanceConstraint;
  auto cls = dartnb::dart_class<Balance, dart::optimizer::Function>(
      m, "BalanceConstraint");

  nb::enum_<Balance::ErrorMethod_t>(cls, "ErrorMethod", nb::is_arithmetic())
      .value("FROM_CENTROID", Balance::FROM_CENTROID)
      .value("FROM_EDGE", Balance::FROM_EDGE)
      .value("OPTIMIZE_BALANCE", Balance::OPTIMIZE_BALANCE);
  nb::enum_<Balance::BalanceMethod_t>(cls, "BalanceMethod", nb::is_arithmetic())
      .value("SHIFT_SUPPORT", Balance::SHIFT_SUPPORT)
      .value("SHIFT_COM", Balance::SHIFT_COM);

  cls.def(
         dartnb::factory(+[](std::shared_ptr<dart::dynamics::HierarchicalIK> ik,
                             Balance::BalanceMethod_t balanceMethod,
                             Balance::ErrorMethod_t errorMethod) {
           auto constraint
               = std::make_shared<Balance>(ik, balanceMethod, errorMethod);
           // Both native affiliations are weak; retain them on the wrapper.
           return BalanceOwner{std::move(constraint), ik, ik->getSkeleton()};
         }),
         nb::arg("ik"),
         nb::arg("balanceMethod") = Balance::SHIFT_SUPPORT,
         nb::arg("errorMethod") = Balance::FROM_CENTROID)
      .def(
          "setErrorMethod",
          +[](Balance* self, Balance::ErrorMethod_t method) {
            self->setErrorMethod(method);
          },
          nb::arg("method"))
      .def(
          "getErrorMethod",
          +[](const Balance* self) { return self->getErrorMethod(); })
      .def(
          "setBalanceMethod",
          +[](Balance* self, Balance::BalanceMethod_t method) {
            self->setBalanceMethod(method);
          },
          nb::arg("method"))
      .def(
          "getBalanceMethod",
          +[](const Balance* self) { return self->getBalanceMethod(); })
      .def(
          "setOptimizationTolerance",
          +[](Balance* self, double tolerance) {
            self->setOptimizationTolerance(tolerance);
          },
          nb::arg("tolerance"))
      .def(
          "getOptimizationTolerance",
          +[](const Balance* self) { return self->getOptimizationTolerance(); })
      .def(
          "setPseudoInverseDamping",
          +[](Balance* self, double damping) {
            self->setPseudoInverseDamping(damping);
          },
          nb::arg("damping"))
      .def(
          "getPseudoInverseDamping",
          +[](const Balance* self) { return self->getPseudoInverseDamping(); })
      .def(
          "eval",
          +[](Balance* self, const Eigen::VectorXd& x) {
            return self->eval(x);
          },
          nb::arg("x"));
}

} // namespace python
} // namespace dart
