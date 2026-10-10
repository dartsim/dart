// clang-format off
#include "detail/dart_nb.hpp"
#include "detail/ik_properties.hpp"
#include "detail/optimizer_properties.hpp"
// clang-format on

#include <nanobind/stl/pair.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

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

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/InverseKinematics.hpp>
#include <dart/dynamics/JacobianNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <dart/optimizer/Solver.hpp>

#include <dart/math/MathTypes.hpp>

#include <dart/common/Subject.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <cstddef>

namespace dart {
namespace python {

namespace {

using IK = dart::dynamics::InverseKinematics;

// Allows dartpy callers to plug Python analytical solvers, including ssik
// prebuilt modules, into DART's native Analytical IK machinery.
class PythonAnalyticalIk final : public IK::Analytical
{
public:
  PythonAnalyticalIk(
      IK* ik,
      std::vector<std::size_t> dofs,
      nb::object solve,
      const std::string& methodName,
      const Properties& properties = Properties())
    : Analytical(ik, methodName, properties),
      mDofs(std::move(dofs)),
      mSolve(new nb::object(std::move(solve)))
  {
    if (mSolve->is_none() || !PyCallable_Check(mSolve->ptr()))
      throw nb::value_error("solve must be a callable Python object");

    // Note: constructDofMap() is intentionally not called here. Both
    // construction paths invoke it after the object is registered as the IK's
    // analytical method: InverseKinematics::setGradientMethod() and
    // InverseKinematics::clone(). Calling it in the constructor would run it
    // twice and double-emit the non-dependent-DOF warning.
  }

  ~PythonAnalyticalIk() override
  {
    if (mSolve) {
      nb::gil_scoped_acquire gil;
      mSolve.reset();
    }
  }

  std::unique_ptr<GradientMethod> clone(IK* newIK) const override
  {
    nb::gil_scoped_acquire gil;
    return std::unique_ptr<GradientMethod>(new PythonAnalyticalIk(
        newIK, mDofs, *mSolve, mMethodName, getAnalyticalProperties()));
  }

  const std::vector<Solution>& computeSolutions(
      const Eigen::Isometry3d& desiredBodyTf) override
  {
    mSolutions.clear();

    nb::gil_scoped_acquire gil;
    nb::object rawSolutions = (*mSolve)(desiredBodyTf.matrix());

    if (rawSolutions.is_none()) {
      checkSolutionJointLimits();
      return mSolutions;
    }

    for (nb::handle rawSolution : rawSolutions)
      mSolutions.push_back(castSolution(rawSolution));

    checkSolutionJointLimits();
    return mSolutions;
  }

  const std::vector<std::size_t>& getDofs() const override
  {
    return mDofs;
  }

private:
  // True if the handle is a sequence/array (so it can be a configuration
  // vector), false for a scalar such as a Python or numpy number.
  static bool isArrayLike(nb::handle value)
  {
    return nb::isinstance<nb::list>(value) || nb::isinstance<nb::tuple>(value)
           || nb::hasattr(value, "__len__");
  }

  // True if the handle is an integer scalar (Python int or numpy integer), but
  // not an array, so it can be a validity flag.
  static bool isIntLike(nb::handle value)
  {
    return !isArrayLike(value)
           && (nb::isinstance<nb::int_>(value) || PyIndex_Check(value.ptr()));
  }

  // Reads an integer validity flag, defaulting to VALID when the value is
  // missing or not integer-convertible instead of throwing an opaque cast error
  // back across the Python boundary.
  static int extractValidity(nb::handle value)
  {
    try {
      return nb::cast<int>(value);
    } catch (const nb::cast_error&) {
      return VALID;
    }
  }

  Solution castSolution(nb::handle rawSolution) const
  {
    if (nb::isinstance<Solution>(rawSolution))
      return nb::cast<Solution>(rawSolution);

    nb::object config = nb::borrow<nb::object>(rawSolution);
    int validity = VALID;

    if (nb::hasattr(rawSolution, "mConfig")) {
      config = rawSolution.attr("mConfig");
      if (nb::hasattr(rawSolution, "mValidity"))
        validity = extractValidity(rawSolution.attr("mValidity"));
    } else if (nb::hasattr(rawSolution, "q")) {
      config = rawSolution.attr("q");
      if (nb::hasattr(rawSolution, "validity"))
        validity = extractValidity(rawSolution.attr("validity"));
    } else if (
        nb::isinstance<nb::tuple>(rawSolution)
        || nb::isinstance<nb::list>(rawSolution)) {
      // Treat a 2-element sequence as (config, validity) only when it is
      // unambiguously that shape: the first element is itself array-like and
      // the second is an integer scalar. Otherwise the whole sequence is the
      // configuration, so a bare 2-DOF config such as [a, b] is not misread as
      // a (config, validity) pair.
      nb::sequence sequence = nb::borrow<nb::sequence>(rawSolution);
      if (nb::len(sequence) == 2 && isArrayLike(sequence[0])
          && isIntLike(sequence[1])) {
        config = nb::borrow<nb::object>(sequence[0]);
        validity = extractValidity(sequence[1]);
      }
    }

    Eigen::VectorXd q;
    try {
      q = nb::cast<Eigen::VectorXd>(config);
    } catch (const nb::cast_error&) {
      throw nb::value_error(("Python analytical IK solution could not be "
                             "converted to a 1-D float "
                             "vector of length "
                             + std::to_string(mDofs.size()))
                                .c_str());
    }

    if (q.size() != static_cast<int>(mDofs.size())) {
      throw nb::value_error(("Python analytical IK solution has "
                             + std::to_string(q.size()) + " values, but "
                             + std::to_string(mDofs.size())
                             + " DOFs were registered")
                                .c_str());
    }

    return Solution(q, validity);
  }

  std::vector<std::size_t> mDofs;
  std::unique_ptr<nb::object> mSolve;
};

} // namespace

template <class Cls>
void defTaskSpaceRegionUniquePropertyMethods(Cls& cls)
{
  cls.def_rw(
         "mComputeErrorFromCenter",
         &dart::dynamics::InverseKinematics::TaskSpaceRegion::UniqueProperties::
             mComputeErrorFromCenter,
         dartnb::setterArgument(
             &dart::dynamics::InverseKinematics::TaskSpaceRegion::
                 UniqueProperties::mComputeErrorFromCenter))
      .def_rw(
          "mReferenceFrame",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              UniqueProperties::mReferenceFrame,
          dartnb::setterArgument(
              &dart::dynamics::InverseKinematics::TaskSpaceRegion::
                  UniqueProperties::mReferenceFrame));
}

void InverseKinematics(nb::module_& m)
{
  dartnb::dart_class<
      dart::dynamics::InverseKinematics::ErrorMethod::Properties>(
      m, "InverseKinematicsErrorMethodProperties")
      .def(
          dartnb::init<
              const dart::dynamics::InverseKinematics::ErrorMethod::Bounds&,
              double,
              const Eigen::Vector6d&>(),
          nb::arg("bounds")
          = dart::dynamics::InverseKinematics::ErrorMethod::Bounds(
              Eigen::Vector6d::Constant(-dart::dynamics::DefaultIKTolerance),
              Eigen::Vector6d::Constant(dart::dynamics::DefaultIKTolerance)),
          nb::arg("errorClamp") = dart::dynamics::DefaultIKErrorClamp,
          nb::arg("errorWeights") = Eigen::compose(
              Eigen::Vector3d::Constant(dart::dynamics::DefaultIKAngularWeight),
              Eigen::Vector3d::Constant(dart::dynamics::DefaultIKLinearWeight)))
      .def_rw(
          "mBounds",
          &dart::dynamics::InverseKinematics::ErrorMethod::Properties::mBounds,
          dartnb::setterArgument(&dart::dynamics::InverseKinematics::
                                     ErrorMethod::Properties::mBounds))
      .def_rw(
          "mErrorLengthClamp",
          &dart::dynamics::InverseKinematics::ErrorMethod::Properties::
              mErrorLengthClamp,
          dartnb::setterArgument(
              &dart::dynamics::InverseKinematics::ErrorMethod::Properties::
                  mErrorLengthClamp))
      .def_rw(
          "mErrorWeights",
          &dart::dynamics::InverseKinematics::ErrorMethod::Properties::
              mErrorWeights,
          dartnb::setterArgument(&dart::dynamics::InverseKinematics::
                                     ErrorMethod::Properties::mErrorWeights));

  dartnb::dart_class<
      dart::dynamics::InverseKinematics::ErrorMethod,
      dart::common::Subject>(m, "InverseKinematicsErrorMethod")
      .def(
          "clone",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self,
              dart::dynamics::InverseKinematics* _newIK)
              -> std::unique_ptr<
                  dart::dynamics::InverseKinematics::ErrorMethod> {
            return self->clone(_newIK);
          },
          nb::arg("newIK").none())
      .def(
          "computeError",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> Eigen::Vector6d { return self->computeError(); })
      .def(
          "computeDesiredTransform",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Isometry3d& _currentTf,
              const Eigen::Vector6d& _error) -> Eigen::Isometry3d {
            return self->computeDesiredTransform(_currentTf, _error);
          },
          nb::arg("currentTf"),
          nb::arg("error"))
      .def(
          "getMethodName",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> const std::string& { return self->getMethodName(); },
          nb::rv_policy::reference_internal)
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setBounds();
          })
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector6d& _lower) { self->setBounds(_lower); },
          nb::arg("lower"))
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector6d& _lower,
              const Eigen::Vector6d& _upper) {
            self->setBounds(_lower, _upper);
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def(
          "setBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const std::pair<Eigen::Vector6d, Eigen::Vector6d>& _bounds) {
            self->setBounds(_bounds);
          },
          nb::arg("bounds"))
      .def(
          "getBounds",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> const std::pair<Eigen::Vector6d, Eigen::Vector6d>& {
            return self->getBounds();
          })
      .def(
          "setAngularBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setAngularBounds();
          })
      .def(
          "setAngularBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _lower) {
            self->setAngularBounds(_lower);
          },
          nb::arg("lower"))
      .def(
          "setAngularBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _lower,
              const Eigen::Vector3d& _upper) {
            self->setAngularBounds(_lower, _upper);
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def(
          "setAngularBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds) {
            self->setAngularBounds(_bounds);
          },
          nb::arg("bounds"))
      .def(
          "getAngularBounds",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> std::pair<Eigen::Vector3d, Eigen::Vector3d> {
            return self->getAngularBounds();
          })
      .def(
          "setLinearBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setLinearBounds();
          })
      .def(
          "setLinearBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _lower) { self->setLinearBounds(_lower); },
          nb::arg("lower"))
      .def(
          "setLinearBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _lower,
              const Eigen::Vector3d& _upper) {
            self->setLinearBounds(_lower, _upper);
          },
          nb::arg("lower"),
          nb::arg("upper"))
      .def(
          "setLinearBounds",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const std::pair<Eigen::Vector3d, Eigen::Vector3d>& _bounds) {
            self->setLinearBounds(_bounds);
          },
          nb::arg("bounds"))
      .def(
          "getLinearBounds",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> std::pair<Eigen::Vector3d, Eigen::Vector3d> {
            return self->getLinearBounds();
          })
      .def(
          "setErrorLengthClamp",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setErrorLengthClamp();
          })
      .def(
          "setErrorLengthClamp",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              double _clampSize) { self->setErrorLengthClamp(_clampSize); },
          nb::arg("clampSize"))
      .def(
          "getErrorLengthClamp",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> double { return self->getErrorLengthClamp(); })
      .def(
          "setErrorWeights",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector6d& _weights) {
            self->setErrorWeights(_weights);
          },
          nb::arg("weights"))
      .def(
          "setAngularErrorWeights",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setAngularErrorWeights();
          })
      .def(
          "setAngularErrorWeights",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _weights) {
            self->setAngularErrorWeights(_weights);
          },
          nb::arg("weights"))
      .def(
          "getAngularErrorWeights",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> Eigen::Vector3d { return self->getAngularErrorWeights(); })
      .def(
          "setLinearErrorWeights",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->setLinearErrorWeights();
          })
      .def(
          "setLinearErrorWeights",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self,
              const Eigen::Vector3d& _weights) {
            self->setLinearErrorWeights(_weights);
          },
          nb::arg("weights"))
      .def(
          "getLinearErrorWeights",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> Eigen::Vector3d { return self->getLinearErrorWeights(); })
      .def(
          "getErrorMethodProperties",
          +[](const dart::dynamics::InverseKinematics::ErrorMethod* self)
              -> dart::dynamics::InverseKinematics::ErrorMethod::Properties {
            return self->getErrorMethodProperties();
          })
      .def(
          "clearCache",
          +[](dart::dynamics::InverseKinematics::ErrorMethod* self) {
            self->clearCache();
          });

  auto uniqueProperties
      = dartnb::dart_class<dart::dynamics::InverseKinematics::TaskSpaceRegion::
                               UniqueProperties>(
            m, "InverseKinematicsTaskSpaceRegionUniqueProperties")
            .def(
                dartnb::init<bool, dart::dynamics::SimpleFramePtr>(),
                nb::arg("computeErrorFromCenter") = true,
                nb::arg("referenceFrame").none() = nullptr);
  defTaskSpaceRegionUniquePropertyMethods(uniqueProperties);

  auto properties
      = dartnb::dart_class<
            dart::dynamics::InverseKinematics::TaskSpaceRegion::Properties,
            dart::dynamics::InverseKinematics::ErrorMethod::Properties,
            dart::dynamics::InverseKinematics::TaskSpaceRegion::
                UniqueProperties>(
            m, "InverseKinematicsTaskSpaceRegionProperties")
            .def(
                dartnb::init<
                    const dart::dynamics::InverseKinematics::ErrorMethod::
                        Properties&,
                    const dart::dynamics::InverseKinematics::TaskSpaceRegion::
                        UniqueProperties&>(),
                nb::arg("errorProperties")
                = dart::dynamics::InverseKinematics::ErrorMethod::Properties(),
                nb::arg("taskSpaceProperties") = dart::dynamics::
                    InverseKinematics::TaskSpaceRegion::UniqueProperties());
  defTaskSpaceRegionUniquePropertyMethods(properties);

  dartnb::dart_class<
      dart::dynamics::InverseKinematics::TaskSpaceRegion,
      dart::dynamics::InverseKinematics::ErrorMethod>(
      m, "InverseKinematicsTaskSpaceRegion")
      .def(
          dartnb::init<
              dart::dynamics::InverseKinematics*,
              dart::dynamics::InverseKinematics::TaskSpaceRegion::Properties>(),
          nb::arg("ik").none(),
          nb::arg("properties")
          = dart::dynamics::InverseKinematics::TaskSpaceRegion::Properties())
      .def(
          "setComputeFromCenter",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              setComputeFromCenter,
          nb::arg("computeFromCenter"),
          "Set whether this TaskSpaceRegion should compute its error vector "
          "from the center of the region.")
      .def(
          "isComputingFromCenter",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              isComputingFromCenter,
          "Get whether this TaskSpaceRegion is set to compute its error vector "
          "from the center of the region.")
      .def(
          "setReferenceFrame",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              setReferenceFrame,
          nb::arg("referenceFrame").none(),
          "Set the reference frame that the task space region is expressed. "
          "Pass None to use the parent frame of the target frame instead.")
      .def(
          "getReferenceFrame",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              getReferenceFrame,
          "Get the reference frame that the task space region is expressed.")
      .def(
          "getTaskSpaceRegionProperties",
          &dart::dynamics::InverseKinematics::TaskSpaceRegion::
              getTaskSpaceRegionProperties,
          "Get the Properties of this TaskSpaceRegion.");

  dartnb::dart_class<
      dart::dynamics::InverseKinematics::GradientMethod::Properties>(
      m, "InverseKinematicsGradientMethodProperties")
      .def(
          dartnb::init<double, const Eigen::VectorXd&>(),
          nb::arg("clamp") = dart::dynamics::DefaultIKGradientComponentClamp,
          nb::arg("weights") = Eigen::VectorXd())
      .def_rw(
          "mComponentWiseClamp",
          &dart::dynamics::InverseKinematics::GradientMethod::Properties::
              mComponentWiseClamp,
          dartnb::setterArgument(
              &dart::dynamics::InverseKinematics::GradientMethod::Properties::
                  mComponentWiseClamp))
      .def_rw(
          "mComponentWeights",
          &dart::dynamics::InverseKinematics::GradientMethod::Properties::
              mComponentWeights,
          dartnb::setterArgument(
              &dart::dynamics::InverseKinematics::GradientMethod::Properties::
                  mComponentWeights));

  dartnb::dart_class<
      dart::dynamics::InverseKinematics::GradientMethod,
      dart::common::Subject>(m, "InverseKinematicsGradientMethod")
      .def(
          "getMethodName",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self)
              -> const std::string& { return self->getMethodName(); },
          nb::rv_policy::reference_internal)
      .def(
          "setComponentWiseClamp",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self) {
            self->setComponentWiseClamp();
          })
      .def(
          "setComponentWiseClamp",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self,
              double _clamp) { self->setComponentWiseClamp(_clamp); },
          nb::arg("clamp"))
      .def(
          "getComponentWiseClamp",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self)
              -> double { return self->getComponentWiseClamp(); })
      .def(
          "setComponentWeights",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self,
              const Eigen::VectorXd& _weights) {
            self->setComponentWeights(_weights);
          },
          nb::arg("weights"))
      .def(
          "getComponentWeights",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self)
              -> const Eigen::VectorXd& { return self->getComponentWeights(); },
          nb::rv_policy::reference_internal)
      .def(
          "getGradientMethodProperties",
          +[](const dart::dynamics::InverseKinematics::GradientMethod* self)
              -> dart::dynamics::InverseKinematics::GradientMethod::Properties {
            return self->getGradientMethodProperties();
          })
      .def(
          "clearCache",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self) {
            self->clearCache();
          })
      .def(
          "getIK",
          +[](dart::dynamics::InverseKinematics::GradientMethod* self)
              -> dart::dynamics::InverseKinematics* { return self->getIK(); },
          nb::rv_policy::reference_internal);

  dartnb::dart_class<dart::dynamics::InverseKinematics::Analytical::Solution>(
      m, "InverseKinematicsAnalyticalSolution")
      .def(
          dartnb::init<const Eigen::VectorXd&, int>(),
          nb::arg("config") = Eigen::VectorXd(),
          nb::arg("validity") = static_cast<int>(
              dart::dynamics::InverseKinematics::Analytical::VALID))
      .def_rw(
          "mConfig",
          &dart::dynamics::InverseKinematics::Analytical::Solution::mConfig,
          dartnb::setterArgument(&dart::dynamics::InverseKinematics::
                                     Analytical::Solution::mConfig))
      .def_rw(
          "mValidity",
          &dart::dynamics::InverseKinematics::Analytical::Solution::mValidity,
          dartnb::setterArgument(&dart::dynamics::InverseKinematics::
                                     Analytical::Solution::mValidity));

  auto analytical
      = dartnb::dart_class<
            dart::dynamics::InverseKinematics::Analytical,
            dart::dynamics::InverseKinematics::GradientMethod>(
            m, "InverseKinematicsAnalytical")
            .def(
                "getSolutions",
                +[](dart::dynamics::InverseKinematics::Analytical* self)
                    -> std::vector<dart::dynamics::InverseKinematics::
                                       Analytical::Solution> {
                  return self->getSolutions();
                })
            .def(
                "getSolutions",
                +[](dart::dynamics::InverseKinematics::Analytical* self,
                    const Eigen::Isometry3d& _desiredTf)
                    -> std::vector<dart::dynamics::InverseKinematics::
                                       Analytical::Solution> {
                  return self->getSolutions(_desiredTf);
                },
                nb::arg("desiredTf"))
            .def(
                "getDofs",
                +[](const dart::dynamics::InverseKinematics::Analytical* self)
                    -> std::vector<std::size_t> { return self->getDofs(); })
            .def(
                "setPositions",
                +[](dart::dynamics::InverseKinematics::Analytical* self,
                    const Eigen::VectorXd& _config) {
                  self->setPositions(_config);
                },
                nb::arg("config"))
            .def(
                "getPositions",
                +[](const dart::dynamics::InverseKinematics::Analytical* self)
                    -> Eigen::VectorXd { return self->getPositions(); })
            .def(
                "setExtraDofUtilization",
                +[](dart::dynamics::InverseKinematics::Analytical* self,
                    int _utilization) {
                  self->setExtraDofUtilization(
                      static_cast<dart::dynamics::InverseKinematics::
                                      Analytical::ExtraDofUtilization>(
                          _utilization));
                },
                nb::arg("utilization"))
            .def(
                "getExtraDofUtilization",
                +[](const dart::dynamics::InverseKinematics::Analytical* self)
                    -> int {
                  return static_cast<int>(self->getExtraDofUtilization());
                })
            .def(
                "setExtraErrorLengthClamp",
                +[](dart::dynamics::InverseKinematics::Analytical* self,
                    double _clamp) { self->setExtraErrorLengthClamp(_clamp); },
                nb::arg("clamp"))
            .def(
                "getExtraErrorLengthClamp",
                +[](const dart::dynamics::InverseKinematics::Analytical* self)
                    -> double { return self->getExtraErrorLengthClamp(); });

  analytical.attr("VALID") = nb::int_(
      static_cast<int>(dart::dynamics::InverseKinematics::Analytical::VALID));
  analytical.attr("OUT_OF_REACH") = nb::int_(static_cast<int>(
      dart::dynamics::InverseKinematics::Analytical::OUT_OF_REACH));
  analytical.attr("LIMIT_VIOLATED") = nb::int_(static_cast<int>(
      dart::dynamics::InverseKinematics::Analytical::LIMIT_VIOLATED));
  analytical.attr("UNUSED") = nb::int_(
      static_cast<int>(dart::dynamics::InverseKinematics::Analytical::UNUSED));
  analytical.attr("PRE_ANALYTICAL") = nb::int_(static_cast<int>(
      dart::dynamics::InverseKinematics::Analytical::PRE_ANALYTICAL));
  analytical.attr("POST_ANALYTICAL") = nb::int_(static_cast<int>(
      dart::dynamics::InverseKinematics::Analytical::POST_ANALYTICAL));
  analytical.attr("PRE_AND_POST_ANALYTICAL") = nb::int_(static_cast<int>(
      dart::dynamics::InverseKinematics::Analytical::PRE_AND_POST_ANALYTICAL));

  dartnb::dart_class<dart::dynamics::InverseKinematics, dart::common::Subject>(
      m, "InverseKinematics")
      .def(
          dartnb::factory(
              +[](dart::dynamics::JacobianNode* node)
                  -> dart::dynamics::InverseKinematicsPtr {
                return dart::dynamics::InverseKinematics::create(node);
              }),
          nb::arg("node"))
      .def(
          "findSolution",
          +[](dart::dynamics::InverseKinematics* self,
              Eigen::VectorXd& positions) -> bool {
            return self->findSolution(positions);
          },
          nb::arg("positions"))
      .def(
          "solveAndApply",
          +[](dart::dynamics::InverseKinematics* self) -> bool {
            return self->solveAndApply();
          })
      .def(
          "solveAndApply",
          +[](dart::dynamics::InverseKinematics* self,
              bool allowIncompleteResult) -> bool {
            return self->solveAndApply(allowIncompleteResult);
          },
          nb::arg("allowIncompleteResult"))
      .def(
          "solveAndApply",
          +[](dart::dynamics::InverseKinematics* self,
              Eigen::VectorXd& positions,
              bool allowIncompleteResult) -> bool {
            return self->solveAndApply(positions, allowIncompleteResult);
          },
          nb::arg("positions"),
          nb::arg("allowIncompleteResult"))
      .def(
          "clone",
          +[](const dart::dynamics::InverseKinematics* self,
              dart::dynamics::JacobianNode* _newNode)
              -> dart::dynamics::InverseKinematicsPtr {
            return self->clone(_newNode);
          },
          nb::arg("newNode").none())
      .def(
          "setActive",
          +[](dart::dynamics::InverseKinematics* self) { self->setActive(); })
      .def(
          "setActive",
          +[](dart::dynamics::InverseKinematics* self, bool _active) {
            self->setActive(_active);
          },
          nb::arg("active"))
      .def(
          "setInactive",
          +[](dart::dynamics::InverseKinematics* self) { self->setInactive(); })
      .def(
          "isActive",
          +[](const dart::dynamics::InverseKinematics* self) -> bool {
            return self->isActive();
          })
      .def(
          "setHierarchyLevel",
          +[](dart::dynamics::InverseKinematics* self, std::size_t _level) {
            self->setHierarchyLevel(_level);
          },
          nb::arg("level"))
      .def(
          "getHierarchyLevel",
          +[](const dart::dynamics::InverseKinematics* self) -> std::size_t {
            return self->getHierarchyLevel();
          })
      .def(
          "useChain",
          +[](dart::dynamics::InverseKinematics* self) { self->useChain(); })
      .def(
          "useWholeBody",
          +[](dart::dynamics::InverseKinematics* self) {
            self->useWholeBody();
          })
      .def(
          "setDofs",
          +[](dart::dynamics::InverseKinematics* self,
              const std::vector<std::size_t>& _dofs) { self->setDofs(_dofs); },
          nb::arg("dofs"))
      .def(
          "getDofs",
          +[](const dart::dynamics::InverseKinematics* self)
              -> std::vector<std::size_t> { return self->getDofs(); })
      .def(
          "setPythonAnalytical",
          +[](dart::dynamics::InverseKinematics* self,
              nb::object solve,
              nb::object dofs,
              const std::string& methodName)
              -> dart::dynamics::InverseKinematics::Analytical& {
            std::vector<std::size_t> resolvedDofs;
            if (dofs.is_none())
              resolvedDofs = self->getDofs();
            else
              resolvedDofs = nb::cast<std::vector<std::size_t>>(dofs);

            // Validate the registered DOF indices against the skeleton up
            // front. Otherwise a stale/incorrect index only warns in
            // constructDofMap() and later segfaults in
            // checkSolutionJointLimits(), which dereferences
            // Skeleton::getDof(index) without a null check.
            const dart::dynamics::JacobianNode* node = self->getNode();
            const auto skeleton = node ? node->getSkeleton() : nullptr;
            const std::size_t numDofs = skeleton ? skeleton->getNumDofs() : 0u;
            for (std::size_t dof : resolvedDofs) {
              if (dof >= numDofs) {
                throw nb::value_error(
                    ("setPythonAnalytical: DOF index " + std::to_string(dof)
                     + " is out of range for the skeleton with "
                     + std::to_string(numDofs) + " DOF(s)")
                        .c_str());
              }
            }

            return self->setGradientMethod<PythonAnalyticalIk>(
                resolvedDofs, std::move(solve), methodName);
          },
          "Installs a Python callback as a DART analytical IK gradient method "
          "and returns the created Analytical method.\n"
          "\n"
          "solve: a callable taking the 4x4 desired end-effector transform (as "
          "a numpy array) and returning an iterable of solutions, or None. "
          "Each "
          "solution may be a DART InverseKinematicsAnalytical.Solution, an "
          "object exposing a ``q``/``mConfig`` attribute (optionally with a "
          "``validity``/``mValidity`` flag), a ``(config, validity)`` pair, or "
          "a "
          "plain configuration sequence. Every configuration must contain one "
          "value per registered DOF.\n"
          "dofs: the DOFs the callback solves for; defaults to the IK's DOFs. "
          "DOFs omitted here are handled by DART's analytical extra-DOF "
          "utilization.\n"
          "methodName: the name reported for the analytical method.\n"
          "\n"
          "The returned handle is owned by this InverseKinematics and is "
          "invalidated by a later setPythonAnalytical/setGradientMethod call.",
          nb::arg("solve"),
          nb::arg("dofs") = nb::none(),
          nb::arg("methodName") = "PythonAnalyticalIk",
          nb::rv_policy::reference_internal)
      .def(
          "getGradientMethod",
          +[](dart::dynamics::InverseKinematics* self)
              -> dart::dynamics::InverseKinematics::GradientMethod& {
            return self->getGradientMethod();
          },
          "Returns the active gradient method. The reference is invalidated by "
          "a later setGradientMethod/setPythonAnalytical call.",
          nb::rv_policy::reference_internal)
      .def(
          "getAnalytical",
          +[](dart::dynamics::InverseKinematics* self)
              -> dart::dynamics::InverseKinematics::Analytical* {
            return self->getAnalytical();
          },
          "Returns the active analytical method, or None if the gradient "
          "method "
          "is not analytical. Invalidated by a later setGradientMethod call.",
          nb::rv_policy::reference_internal)
      .def(
          "setObjective",
          +[](dart::dynamics::InverseKinematics* self,
              const std::shared_ptr<dart::optimizer::Function>& _objective) {
            self->setObjective(_objective);
          },
          nb::arg("objective").none())
      .def(
          "getObjective",
          +[](const dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<const dart::optimizer::Function> {
            return self->getObjective();
          })
      .def(
          "setNullSpaceObjective",
          +[](dart::dynamics::InverseKinematics* self,
              const std::shared_ptr<dart::optimizer::Function>& _nsObjective) {
            self->setNullSpaceObjective(_nsObjective);
          },
          nb::arg("nsObjective").none())
      .def(
          "getNullSpaceObjective",
          +[](const dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<const dart::optimizer::Function> {
            return self->getNullSpaceObjective();
          })
      .def(
          "hasNullSpaceObjective",
          +[](const dart::dynamics::InverseKinematics* self) -> bool {
            return self->hasNullSpaceObjective();
          })
      .def(
          "getErrorMethod",
          +[](dart::dynamics::InverseKinematics* self)
              -> dart::dynamics::InverseKinematics::ErrorMethod& {
            return self->getErrorMethod();
          },
          nb::rv_policy::reference_internal)
      .def(
          "getProblem",
          +[](const dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<const dart::optimizer::Problem> {
            return self->getProblem();
          })
      .def(
          "resetProblem",
          +[](dart::dynamics::InverseKinematics* self) {
            self->resetProblem();
          })
      .def(
          "resetProblem",
          +[](dart::dynamics::InverseKinematics* self, bool _clearSeeds) {
            self->resetProblem(_clearSeeds);
          },
          nb::arg("clearSeeds"))
      .def(
          "setSolver",
          +[](dart::dynamics::InverseKinematics* self,
              const std::shared_ptr<dart::optimizer::Solver>& _newSolver) {
            self->setSolver(_newSolver);
          },
          nb::arg("newSolver").none())
      .def(
          "getSolver",
          +[](dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<dart::optimizer::Solver> {
            return self->getSolver();
          })
      .def(
          "setOffset",
          +[](dart::dynamics::InverseKinematics* self) { self->setOffset(); })
      .def(
          "setOffset",
          +[](dart::dynamics::InverseKinematics* self,
              const Eigen::Vector3d& _offset) { self->setOffset(_offset); },
          nb::arg("offset"))
      .def(
          "getOffset",
          +[](const dart::dynamics::InverseKinematics* self)
              -> const Eigen::Vector3d& { return self->getOffset(); },
          nb::rv_policy::reference_internal)
      .def(
          "hasOffset",
          +[](const dart::dynamics::InverseKinematics* self) -> bool {
            return self->hasOffset();
          })
      .def(
          "setTarget",
          +[](dart::dynamics::InverseKinematics* self,
              std::shared_ptr<dart::dynamics::SimpleFrame> _newTarget) {
            self->setTarget(_newTarget);
          },
          nb::arg("newTarget").none())
      .def(
          "getTarget",
          +[](dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<dart::dynamics::SimpleFrame> {
            return self->getTarget();
          })
      .def(
          "getTarget",
          +[](const dart::dynamics::InverseKinematics* self)
              -> std::shared_ptr<const dart::dynamics::SimpleFrame> {
            return self->getTarget();
          })
      .def(
          "getPositions",
          +[](const dart::dynamics::InverseKinematics* self)
              -> Eigen::VectorXd { return self->getPositions(); })
      .def(
          "setPositions",
          +[](dart::dynamics::InverseKinematics* self,
              const Eigen::VectorXd& _q) { self->setPositions(_q); },
          nb::arg("q"))
      .def(
          "clearCaches", +[](dart::dynamics::InverseKinematics* self) {
            self->clearCaches();
          });
}

} // namespace python
} // namespace dart
