#include "optimizer/function.hpp"

#include "common/eigen_utils.hpp"
#include "dart/common/diagnostics.hpp"
#include "dart/math/optimization/function.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>

namespace nb = nanobind;

namespace dart::python_nb {

class PyFunction : public dart::math::Function
{
public:
  NB_TRAMPOLINE(dart::math::Function, 2);

  double eval(const Eigen::VectorXd& x) override
  {
    NB_OVERRIDE_PURE(eval, x);
  }

  void evalGradient(
      const Eigen::VectorXd& x, Eigen::Map<Eigen::VectorXd> grad) override
  {
    NB_OVERRIDE(evalGradient, x, grad);
  }
};

void defOptimizerFunction(nb::module_& m)
{
  nb::class_<dart::math::Function, PyFunction>(m, "Function")
      .def(nb::init<>())
      .def(nb::init<const std::string&>(), nb::arg("name"))
      .def(
          "setName",
          [](dart::math::Function& self, const std::string& name) {
            self.setName(name);
          },
          nb::arg("name"))
      .def(
          "getName",
          [](const dart::math::Function& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal);

  nb::class_<dart::math::NullFunction, dart::math::Function>(m, "NullFunction")
      .def(nb::init<const std::string&>(), nb::arg("name"))
      .def(
          "eval",
          [](dart::math::NullFunction& self, const nb::handle& x) {
            return self.eval(toVector(x));
          },
          nb::arg("x"));
}

} // namespace dart::python_nb
