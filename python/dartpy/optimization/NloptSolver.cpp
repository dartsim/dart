/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#include <dart/config.hpp>
#if DART_HAS_NLOPT

  #include "dartpy/math/eigen_pybind.h"

  #include <dart/optimization/optimization.hpp>

  #include <pybind11/pybind11.h>

namespace py = pybind11;

namespace dart {
namespace python {

  #define DARTPY_DEFINE_ALGORITHM(alg_name)                                    \
    .value(#alg_name, dart::optimization::NloptSolver::Algorithm::alg_name)

void NloptSolver(py::module& m)
{
  ::py::class_<
      dart::optimization::NloptSolver,
      dart::optimization::Solver,
      std::shared_ptr<dart::optimization::NloptSolver>>(m, "NloptSolver")
      .def(::py::init<>())
      .def(
          ::py::init<const dart::optimization::Solver::Properties&>(),
          ::py::arg("properties"))
      .def(
          ::py::init<
              const dart::optimization::Solver::Properties&,
              dart::optimization::NloptSolver::Algorithm>(),
          ::py::arg("properties"),
          ::py::arg("alg"))
      .def(
          ::py::init<std::shared_ptr<dart::optimization::Problem>>(),
          ::py::arg("problem"))
      .def(
          ::py::init<
              std::shared_ptr<dart::optimization::Problem>,
              dart::optimization::NloptSolver::Algorithm>(),
          ::py::arg("problem"),
          ::py::arg("alg"))
      .def(
          "solve",
          +[](dart::optimization::NloptSolver* self) -> bool {
            return self->solve();
          })
      .def(
          "getLastConfiguration",
          +[](const dart::optimization::NloptSolver* self) -> math::VectorXd {
            return self->getLastConfiguration();
          })
      .def(
          "getType",
          +[](const dart::optimization::NloptSolver* self) -> std::string {
            return self->getType();
          })
      .def(
          "clone",
          +[](const dart::optimization::NloptSolver* self)
              -> std::shared_ptr<dart::optimization::Solver> {
            return self->clone();
          })
      .def(
          "setAlgorithm",
          +[](dart::optimization::NloptSolver* self,
              dart::optimization::NloptSolver::Algorithm _alg) {
            self->setAlgorithm(_alg);
          },
          ::py::arg("alg"))
      .def(
          "getAlgorithm",
          +[](const dart::optimization::NloptSolver* self)
              -> dart::optimization::NloptSolver::Algorithm {
            return self->getAlgorithm();
          });

  auto attr = m.attr("NloptSolver");

  // clang-format off
  ::py::enum_<dart::optimization::NloptSolver::Algorithm>(attr, "Algorithm")
      DARTPY_DEFINE_ALGORITHM(GN_DIRECT)
      DARTPY_DEFINE_ALGORITHM(GN_DIRECT_L)
      DARTPY_DEFINE_ALGORITHM(GN_DIRECT_L_RAND)
      DARTPY_DEFINE_ALGORITHM(GN_DIRECT_NOSCAL)
      DARTPY_DEFINE_ALGORITHM(GN_DIRECT_L_NOSCAL)
      DARTPY_DEFINE_ALGORITHM(GN_DIRECT_L_RAND_NOSCAL)
      DARTPY_DEFINE_ALGORITHM(GN_ORIG_DIRECT)
      DARTPY_DEFINE_ALGORITHM(GN_ORIG_DIRECT_L)
      DARTPY_DEFINE_ALGORITHM(GD_STOGO)
      DARTPY_DEFINE_ALGORITHM(GD_STOGO_RAND)
      DARTPY_DEFINE_ALGORITHM(LD_LBFGS_NOCEDAL)
      DARTPY_DEFINE_ALGORITHM(LD_LBFGS)
      DARTPY_DEFINE_ALGORITHM(LN_PRAXIS)
      DARTPY_DEFINE_ALGORITHM(LD_VAR1)
      DARTPY_DEFINE_ALGORITHM(LD_VAR2)
      DARTPY_DEFINE_ALGORITHM(LD_TNEWTON)
      DARTPY_DEFINE_ALGORITHM(LD_TNEWTON_RESTART)
      DARTPY_DEFINE_ALGORITHM(LD_TNEWTON_PRECOND)
      DARTPY_DEFINE_ALGORITHM(LD_TNEWTON_PRECOND_RESTART)
      DARTPY_DEFINE_ALGORITHM(GN_CRS2_LM)
      DARTPY_DEFINE_ALGORITHM(GN_MLSL)
      DARTPY_DEFINE_ALGORITHM(GD_MLSL)
      DARTPY_DEFINE_ALGORITHM(GN_MLSL_LDS)
      DARTPY_DEFINE_ALGORITHM(GD_MLSL_LDS)
      DARTPY_DEFINE_ALGORITHM(LD_MMA)
      DARTPY_DEFINE_ALGORITHM(LN_COBYLA)
      DARTPY_DEFINE_ALGORITHM(LN_NEWUOA)
      DARTPY_DEFINE_ALGORITHM(LN_NEWUOA_BOUND)
      DARTPY_DEFINE_ALGORITHM(LN_NELDERMEAD)
      DARTPY_DEFINE_ALGORITHM(LN_SBPLX)
      DARTPY_DEFINE_ALGORITHM(LN_AUGLAG)
      DARTPY_DEFINE_ALGORITHM(LD_AUGLAG)
      DARTPY_DEFINE_ALGORITHM(LN_AUGLAG_EQ)
      DARTPY_DEFINE_ALGORITHM(LD_AUGLAG_EQ)
      DARTPY_DEFINE_ALGORITHM(LN_BOBYQA)
      DARTPY_DEFINE_ALGORITHM(GN_ISRES)
      DARTPY_DEFINE_ALGORITHM(AUGLAG)
      DARTPY_DEFINE_ALGORITHM(AUGLAG_EQ)
      DARTPY_DEFINE_ALGORITHM(G_MLSL)
      DARTPY_DEFINE_ALGORITHM(G_MLSL_LDS)
      DARTPY_DEFINE_ALGORITHM(LD_SLSQP)
      DARTPY_DEFINE_ALGORITHM(LD_CCSAQ)
      DARTPY_DEFINE_ALGORITHM(GN_ESCH)
  ;
  // clang-format on
}

} // namespace python
} // namespace dart

#endif // DART_HAS_NLOPT
