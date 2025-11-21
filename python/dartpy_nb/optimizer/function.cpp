#include "optimizer/function.hpp"

#include "dart/common/Diagnostics.hpp"
#include "dart/optimizer/Function.hpp"

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>

namespace nb = nanobind;

namespace dart::python_nb {

DART_SUPPRESS_DEPRECATED_BEGIN

class PyFunction : public dart::optimizer::Function
{
public:
  NB_TRAMPOLINE(dart::optimizer::Function, 2);

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
  nb::class_<dart::optimizer::Function, PyFunction>(m, "Function")
      .def(nb::init<>())
      .def(nb::init<const std::string&>(), nb::arg("name"))
      .def(
          "setName",
          [](dart::optimizer::Function& self, const std::string& name) {
            self.setName(name);
          },
          nb::arg("name"))
      .def(
          "getName",
          [](const dart::optimizer::Function& self) -> const std::string& {
            return self.getName();
          },
          nb::rv_policy::reference_internal);

  nb::class_<dart::optimizer::NullFunction, dart::optimizer::Function>(
      m, "NullFunction")
      .def(nb::init<const std::string&>(), nb::arg("name"))
      .def(
      "eval",
      [](dart::optimizer::NullFunction& self, const Eigen::VectorXd& x) {
        return self.eval(x);
      },
      nb::arg("x"));
}

DART_SUPPRESS_DEPRECATED_END

} // namespace dart::python_nb
