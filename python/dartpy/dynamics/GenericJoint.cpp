// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

#include "detail/array.hpp"

#include <nanobind/stl/unique_ptr.h>

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

#include <dart/dynamics/GenericJoint.hpp>
#include <dart/dynamics/Joint.hpp>

#include <dart/math/ConfigurationSpace.hpp>
#include <dart/math/MathTypes.hpp>

#include <dart/common/Aspect.hpp>
#include <dart/common/Composite.hpp>
#include <dart/common/CompositeJoiner.hpp>
#include <dart/common/EmbeddedAspect.hpp>
#include <dart/common/RequiresAspect.hpp>
#include <dart/common/SpecializedForAspect.hpp>

#include <Eigen/Core>
#include <eigen_geometry_pybind.h>

#include <memory>
#include <string>

#include <cstddef>

#define DARTPY_DEFINE_GENERICJOINT(name, space)                                \
  dartnb::dart_class<                                                          \
      dart::dynamics::detail::GenericJointUniqueProperties<space>>(            \
      m, "GenericJointUniqueProperties_" #name)                                \
      .def(dartnb::init<>())                                                   \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&>(),                                  \
          nb::arg("positionLowerLimits"))                                      \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&>(),                                  \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"))                                      \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&>(),                                  \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"))                                         \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"))                                      \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"),                                      \
          nb::arg("velocityUpperLimits"))                                      \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"),                                      \
          nb::arg("velocityUpperLimits"),                                      \
          nb::arg("initialVelocities"))                                        \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"),                                      \
          nb::arg("velocityUpperLimits"),                                      \
          nb::arg("initialVelocities"),                                        \
          nb::arg("accelerationLowerLimits"))                                  \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"),                                      \
          nb::arg("velocityUpperLimits"),                                      \
          nb::arg("initialVelocities"),                                        \
          nb::arg("accelerationLowerLimits"),                                  \
          nb::arg("accelerationUpperLimits"))                                  \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"),                                      \
          nb::arg("velocityUpperLimits"),                                      \
          nb::arg("initialVelocities"),                                        \
          nb::arg("accelerationLowerLimits"),                                  \
          nb::arg("accelerationUpperLimits"),                                  \
          nb::arg("forceLowerLimits"))                                         \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"),                                      \
          nb::arg("velocityUpperLimits"),                                      \
          nb::arg("initialVelocities"),                                        \
          nb::arg("accelerationLowerLimits"),                                  \
          nb::arg("accelerationUpperLimits"),                                  \
          nb::arg("forceLowerLimits"),                                         \
          nb::arg("forceUpperLimits"))                                         \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"),                                      \
          nb::arg("velocityUpperLimits"),                                      \
          nb::arg("initialVelocities"),                                        \
          nb::arg("accelerationLowerLimits"),                                  \
          nb::arg("accelerationUpperLimits"),                                  \
          nb::arg("forceLowerLimits"),                                         \
          nb::arg("forceUpperLimits"),                                         \
          nb::arg("springStiffness"))                                          \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&>(),                                  \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"),                                      \
          nb::arg("velocityUpperLimits"),                                      \
          nb::arg("initialVelocities"),                                        \
          nb::arg("accelerationLowerLimits"),                                  \
          nb::arg("accelerationUpperLimits"),                                  \
          nb::arg("forceLowerLimits"),                                         \
          nb::arg("forceUpperLimits"),                                         \
          nb::arg("springStiffness"),                                          \
          nb::arg("restPosition"))                                             \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"),                                      \
          nb::arg("velocityUpperLimits"),                                      \
          nb::arg("initialVelocities"),                                        \
          nb::arg("accelerationLowerLimits"),                                  \
          nb::arg("accelerationUpperLimits"),                                  \
          nb::arg("forceLowerLimits"),                                         \
          nb::arg("forceUpperLimits"),                                         \
          nb::arg("springStiffness"),                                          \
          nb::arg("restPosition"),                                             \
          nb::arg("dampingCoefficient"))                                       \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&,                                             \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          nb::arg("positionLowerLimits"),                                      \
          nb::arg("positionUpperLimits"),                                      \
          nb::arg("initialPositions"),                                         \
          nb::arg("velocityLowerLimits"),                                      \
          nb::arg("velocityUpperLimits"),                                      \
          nb::arg("initialVelocities"),                                        \
          nb::arg("accelerationLowerLimits"),                                  \
          nb::arg("accelerationUpperLimits"),                                  \
          nb::arg("forceLowerLimits"),                                         \
          nb::arg("forceUpperLimits"),                                         \
          nb::arg("springStiffness"),                                          \
          nb::arg("restPosition"),                                             \
          nb::arg("dampingCoefficient"),                                       \
          nb::arg("coulombFrictions"))                                         \
      .def_rw(                                                                 \
          "mPositionLowerLimits",                                              \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mPositionLowerLimits,                                    \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mPositionLowerLimits))                               \
      .def_rw(                                                                 \
          "mPositionUpperLimits",                                              \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mPositionUpperLimits,                                    \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mPositionUpperLimits))                               \
      .def_rw(                                                                 \
          "mInitialPositions",                                                 \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mInitialPositions,                                       \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mInitialPositions))                                  \
      .def_rw(                                                                 \
          "mVelocityLowerLimits",                                              \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mVelocityLowerLimits,                                    \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mVelocityLowerLimits))                               \
      .def_rw(                                                                 \
          "mVelocityUpperLimits",                                              \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mVelocityUpperLimits,                                    \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mVelocityUpperLimits))                               \
      .def_rw(                                                                 \
          "mInitialVelocities",                                                \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mInitialVelocities,                                      \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mInitialVelocities))                                 \
      .def_rw(                                                                 \
          "mAccelerationLowerLimits",                                          \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mAccelerationLowerLimits,                                \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mAccelerationLowerLimits))                           \
      .def_rw(                                                                 \
          "mAccelerationUpperLimits",                                          \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mAccelerationUpperLimits,                                \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mAccelerationUpperLimits))                           \
      .def_rw(                                                                 \
          "mForceLowerLimits",                                                 \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mForceLowerLimits,                                       \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mForceLowerLimits))                                  \
      .def_rw(                                                                 \
          "mForceUpperLimits",                                                 \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mForceUpperLimits,                                       \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mForceUpperLimits))                                  \
      .def_rw(                                                                 \
          "mSpringStiffnesses",                                                \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mSpringStiffnesses,                                      \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mSpringStiffnesses))                                 \
      .def_rw(                                                                 \
          "mRestPositions",                                                    \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mRestPositions,                                          \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mRestPositions))                                     \
      .def_rw(                                                                 \
          "mDampingCoefficients",                                              \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mDampingCoefficients,                                    \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mDampingCoefficients))                               \
      .def_rw(                                                                 \
          "mFrictions",                                                        \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mFrictions,                                              \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mFrictions))                                         \
      .def_rw(                                                                 \
          "mPreserveDofNames",                                                 \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mPreserveDofNames,                                       \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mPreserveDofNames))                                  \
      .def_rw(                                                                 \
          "mDofNames",                                                         \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mDofNames,                                               \
          dartnb::setterArgument(                                              \
              &dart::dynamics::detail::GenericJointUniqueProperties<           \
                  space>::mDofNames));                                         \
  dartnb::dart_class<                                                          \
      dart::dynamics::detail::GenericJointProperties<space>,                   \
      dart::dynamics::detail::JointProperties,                                 \
      dart::dynamics::detail::GenericJointUniqueProperties<space>>(            \
      m, "GenericJointProperties_" #name)                                      \
      .def(dartnb::init<>())                                                   \
      .def(                                                                    \
          dartnb::init<const dart::dynamics::Joint::Properties&>(),            \
          nb::arg("jointProperties"))                                          \
      .def(                                                                    \
          dartnb::init<                                                        \
              const dart::dynamics::Joint::Properties&,                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>&>(),                                                  \
          nb::arg("jointProperties"),                                          \
          nb::arg("genericProperties"));                                       \
  dartnb::dart_class<                                                          \
      dart::common::SpecializedForAspect<                                      \
          dart::common::EmbeddedStateAndPropertiesAspect<                      \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>>,   \
      dart::common::Composite>(                                                \
      m,                                                                       \
      "SpecializedForAspect_EmbeddedStateAndPropertiesAspect_"                 \
      "GenericJoint_" #name "_GenericJointState_GenericJointUniqueProperties") \
      .def(dartnb::init<>());                                                  \
  dartnb::dart_class<                                                          \
      dart::common::RequiresAspect<                                            \
          dart::common::EmbeddedStateAndPropertiesAspect<                      \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>>,   \
      dart::common::SpecializedForAspect<                                      \
          dart::common::EmbeddedStateAndPropertiesAspect<                      \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>>>(  \
      m,                                                                       \
      "RequiresAspect_EmbeddedStateAndPropertiesAspect_GenericJoint_" #name    \
      "_GenericJointState_GenericJointUniqueProperties")                       \
      .def(dartnb::init<>());                                                  \
  dartnb::dart_class<                                                          \
      dart::common::EmbedStateAndProperties<                                   \
          dart::dynamics::GenericJoint<space>,                                 \
          dart::dynamics::detail::GenericJointState<space>,                    \
          dart::dynamics::detail::GenericJointUniqueProperties<space>>,        \
      dart::common::RequiresAspect<                                            \
          dart::common::EmbeddedStateAndPropertiesAspect<                      \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>>>(  \
      m,                                                                       \
      "EmbedStateAndProperties_GenericJoint_" #name                            \
      "GenericJointState_GenericJointUniqueProperties");                       \
  dartnb::dart_class<                                                          \
      dart::common::CompositeJoiner<                                           \
          dart::common::EmbedStateAndProperties<                               \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>,    \
          dart::dynamics::Joint>,                                              \
      dart::common::EmbedStateAndProperties<                                   \
          dart::dynamics::GenericJoint<space>,                                 \
          dart::dynamics::detail::GenericJointState<space>,                    \
          dart::dynamics::detail::GenericJointUniqueProperties<space>>,        \
      dart::dynamics::Joint>(                                                  \
      m,                                                                       \
      "CompositeJoiner_EmbedStateAndProperties_GenericJoint_" #name            \
      "GenericJointStateGenericJointUniqueProperties_Joint");                  \
  dartnb::dart_class<                                                          \
      dart::common::EmbedStateAndPropertiesOnTopOf<                            \
          dart::dynamics::GenericJoint<space>,                                 \
          dart::dynamics::detail::GenericJointState<space>,                    \
          dart::dynamics::detail::GenericJointUniqueProperties<space>,         \
          dart::dynamics::Joint>,                                              \
      dart::common::CompositeJoiner<                                           \
          dart::common::EmbedStateAndProperties<                               \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>,    \
          dart::dynamics::Joint>>(                                             \
      m,                                                                       \
      "EmbedStateAndPropertiesOnTopOf_GenericJoint_" #name                     \
      "_GenericJointState_GenericJointUniqueProperties_Joint");                \
  dartnb::dart_class<                                                          \
      dart::dynamics::GenericJoint<space>,                                     \
      dart::common::EmbedStateAndPropertiesOnTopOf<                            \
          dart::dynamics::GenericJoint<space>,                                 \
          dart::dynamics::detail::GenericJointState<space>,                    \
          dart::dynamics::detail::GenericJointUniqueProperties<space>,         \
          dart::dynamics::Joint>>(m, "GenericJoint_" #name)                    \
      .def(                                                                    \
          "hasGenericJointAspect",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self) -> bool {       \
            return self->hasGenericJointAspect();                              \
          })                                                                   \
      .def(                                                                    \
          "setGenericJointAspect",                                             \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::Aspect* aspect) {     \
            self->setGenericJointAspect(aspect);                               \
          },                                                                   \
          nb::arg("aspect").none())                                            \
      .def(                                                                    \
          "removeGenericJointAspect",                                          \
          +[](dart::dynamics::GenericJoint<space>* self) {                     \
            self->removeGenericJointAspect();                                  \
          })                                                                   \
      .def(                                                                    \
          "releaseGenericJointAspect",                                         \
          +[](dart::dynamics::GenericJoint<space>* self)                       \
              -> std::unique_ptr<                                              \
                  dart::dynamics::GenericJoint<space>::Aspect> {               \
            return self->releaseGenericJointAspect();                          \
          })                                                                   \
      .def(                                                                    \
          "setProperties",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::Properties&           \
                  properties) { self->setProperties(properties); },            \
          nb::arg("properties"))                                               \
      .def(                                                                    \
          "setProperties",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::UniqueProperties&     \
                  properties) { self->setProperties(properties); },            \
          nb::arg("properties"))                                               \
      .def(                                                                    \
          "setAspectState",                                                    \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::AspectState& state) { \
            self->setAspectState(state);                                       \
          },                                                                   \
          nb::arg("state"))                                                    \
      .def(                                                                    \
          "setAspectProperties",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::AspectProperties&     \
                  properties) { self->setAspectProperties(properties); },      \
          nb::arg("properties"))                                               \
      .def(                                                                    \
          "getGenericJointProperties",                                         \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> dart::dynamics::GenericJoint<space>::Properties {             \
            return self->getGenericJointProperties();                          \
          })                                                                   \
      .def(                                                                    \
          "copy",                                                              \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::ThisClass&            \
                  otherJoint) { self->copy(otherJoint); },                     \
          nb::arg("otherJoint"))                                               \
      .def(                                                                    \
          "copy",                                                              \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::ThisClass*            \
                  otherJoint) { self->copy(otherJoint); },                     \
          nb::arg("otherJoint").none())                                        \
      .def(                                                                    \
          "getNumDofs",                                                        \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> std::size_t { return self->getNumDofs(); })                   \
      .def(                                                                    \
          "setDofName",                                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              const std::string& name) -> const std::string& {                 \
            return self->setDofName(index, name);                              \
          },                                                                   \
          nb::rv_policy::reference_internal,                                   \
          nb::arg("index"),                                                    \
          nb::arg("name"))                                                     \
      .def(                                                                    \
          "setDofName",                                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              const std::string& name,                                         \
              bool preserveName) -> const std::string& {                       \
            return self->setDofName(index, name, preserveName);                \
          },                                                                   \
          nb::rv_policy::reference_internal,                                   \
          nb::arg("index"),                                                    \
          nb::arg("name"),                                                     \
          nb::arg("preserveName"))                                             \
      .def(                                                                    \
          "preserveDofName",                                                   \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              bool preserve) { self->preserveDofName(index, preserve); },      \
          nb::arg("index"),                                                    \
          nb::arg("preserve"))                                                 \
      .def(                                                                    \
          "isDofNamePreserved",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> bool { return self->isDofNamePreserved(index); },             \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "getDofName",                                                        \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> const std::string& { return self->getDofName(index); },       \
          nb::rv_policy::reference_internal,                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "getIndexInSkeleton",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> size_t { return self->getIndexInSkeleton(index); },           \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "getIndexInTree",                                                    \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              size_t index) -> size_t { return self->getIndexInTree(index); }, \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setCommand",                                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double command) { self->setCommand(index, command); },           \
          nb::arg("index"),                                                    \
          nb::arg("command"))                                                  \
      .def(                                                                    \
          "getCommand",                                                        \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getCommand(index);                                    \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setCommands",                                                       \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& commands) {                               \
            self->setCommands(commands);                                       \
          },                                                                   \
          nb::arg("commands"))                                                 \
      .def(                                                                    \
          "getCommands",                                                       \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getCommands(); })              \
      .def(                                                                    \
          "resetCommands",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self) {                     \
            self->resetCommands();                                             \
          })                                                                   \
      .def(                                                                    \
          "setPosition",                                                       \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double position) { self->setPosition(index, position); },        \
          nb::arg("index"),                                                    \
          nb::arg("position"))                                                 \
      .def(                                                                    \
          "getPosition",                                                       \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getPosition(index);                                   \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setPositions",                                                      \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& positions) {                              \
            self->setPositions(positions);                                     \
          },                                                                   \
          nb::arg("positions"))                                                \
      .def(                                                                    \
          "getPositions",                                                      \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getPositions(); })             \
      .def(                                                                    \
          "setPositionLowerLimit",                                             \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double position) {                                               \
            self->setPositionLowerLimit(index, position);                      \
          },                                                                   \
          nb::arg("index"),                                                    \
          nb::arg("position"))                                                 \
      .def(                                                                    \
          "getPositionLowerLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getPositionLowerLimit(index);                         \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setPositionLowerLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setPositionLowerLimits(lowerLimits);                         \
          },                                                                   \
          nb::arg("lowerLimits"))                                              \
      .def(                                                                    \
          "getPositionLowerLimits",                                            \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getPositionLowerLimits(); })   \
      .def(                                                                    \
          "setPositionUpperLimit",                                             \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double position) {                                               \
            self->setPositionUpperLimit(index, position);                      \
          },                                                                   \
          nb::arg("index"),                                                    \
          nb::arg("position"))                                                 \
      .def(                                                                    \
          "getPositionUpperLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getPositionUpperLimit(index);                         \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setPositionUpperLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setPositionUpperLimits(upperLimits);                         \
          },                                                                   \
          nb::arg("upperLimits"))                                              \
      .def(                                                                    \
          "getPositionUpperLimits",                                            \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getPositionUpperLimits(); })   \
      .def(                                                                    \
          "hasPositionLimit",                                                  \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> bool {                                     \
            return self->hasPositionLimit(index);                              \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "resetPosition",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self, std::size_t index) {  \
            self->resetPosition(index);                                        \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "resetPositions",                                                    \
          +[](dart::dynamics::GenericJoint<space>* self) {                     \
            self->resetPositions();                                            \
          })                                                                   \
      .def(                                                                    \
          "setInitialPosition",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double initial) { self->setInitialPosition(index, initial); },   \
          nb::arg("index"),                                                    \
          nb::arg("initial"))                                                  \
      .def(                                                                    \
          "getInitialPosition",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getInitialPosition(index);                            \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setInitialPositions",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& initial) {                                \
            self->setInitialPositions(initial);                                \
          },                                                                   \
          nb::arg("initial"))                                                  \
      .def(                                                                    \
          "getInitialPositions",                                               \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getInitialPositions(); })      \
      .def(                                                                    \
          "setPositionsStatic",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::Vector& positions) {  \
            self->setPositionsStatic(positions);                               \
          },                                                                   \
          nb::arg("positions"))                                                \
      .def(                                                                    \
          "setVelocitiesStatic",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::Vector& velocities) { \
            self->setVelocitiesStatic(velocities);                             \
          },                                                                   \
          nb::arg("velocities"))                                               \
      .def(                                                                    \
          "setAccelerationsStatic",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::Vector& accels) {     \
            self->setAccelerationsStatic(accels);                              \
          },                                                                   \
          nb::arg("accels"))                                                   \
      .def(                                                                    \
          "setVelocity",                                                       \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double velocity) { self->setVelocity(index, velocity); },        \
          nb::arg("index"),                                                    \
          nb::arg("velocity"))                                                 \
      .def(                                                                    \
          "getVelocity",                                                       \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocity(index);                                   \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setVelocities",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& velocities) {                             \
            self->setVelocities(velocities);                                   \
          },                                                                   \
          nb::arg("velocities"))                                               \
      .def(                                                                    \
          "getVelocities",                                                     \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getVelocities(); })            \
      .def(                                                                    \
          "setVelocityLowerLimit",                                             \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double velocity) {                                               \
            self->setVelocityLowerLimit(index, velocity);                      \
          },                                                                   \
          nb::arg("index"),                                                    \
          nb::arg("velocity"))                                                 \
      .def(                                                                    \
          "getVelocityLowerLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocityLowerLimit(index);                         \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setVelocityLowerLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setVelocityLowerLimits(lowerLimits);                         \
          },                                                                   \
          nb::arg("lowerLimits"))                                              \
      .def(                                                                    \
          "getVelocityLowerLimits",                                            \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getVelocityLowerLimits(); })   \
      .def(                                                                    \
          "setVelocityUpperLimit",                                             \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double velocity) {                                               \
            self->setVelocityUpperLimit(index, velocity);                      \
          },                                                                   \
          nb::arg("index"),                                                    \
          nb::arg("velocity"))                                                 \
      .def(                                                                    \
          "getVelocityUpperLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocityUpperLimit(index);                         \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setVelocityUpperLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setVelocityUpperLimits(upperLimits);                         \
          },                                                                   \
          nb::arg("upperLimits"))                                              \
      .def(                                                                    \
          "getVelocityUpperLimits",                                            \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getVelocityUpperLimits(); })   \
      .def(                                                                    \
          "resetVelocity",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self, std::size_t index) {  \
            self->resetVelocity(index);                                        \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "resetVelocities",                                                   \
          +[](dart::dynamics::GenericJoint<space>* self) {                     \
            self->resetVelocities();                                           \
          })                                                                   \
      .def(                                                                    \
          "setInitialVelocity",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double initial) { self->setInitialVelocity(index, initial); },   \
          nb::arg("index"),                                                    \
          nb::arg("initial"))                                                  \
      .def(                                                                    \
          "getInitialVelocity",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getInitialVelocity(index);                            \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setInitialVelocities",                                              \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& initial) {                                \
            self->setInitialVelocities(initial);                               \
          },                                                                   \
          nb::arg("initial"))                                                  \
      .def(                                                                    \
          "getInitialVelocities",                                              \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getInitialVelocities(); })     \
      .def(                                                                    \
          "setAcceleration",                                                   \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double acceleration) {                                           \
            self->setAcceleration(index, acceleration);                        \
          },                                                                   \
          nb::arg("index"),                                                    \
          nb::arg("acceleration"))                                             \
      .def(                                                                    \
          "getAcceleration",                                                   \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getAcceleration(index);                               \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setAccelerations",                                                  \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& accelerations) {                          \
            self->setAccelerations(accelerations);                             \
          },                                                                   \
          nb::arg("accelerations"))                                            \
      .def(                                                                    \
          "getAccelerations",                                                  \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getAccelerations(); })         \
      .def(                                                                    \
          "setAccelerationLowerLimit",                                         \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double acceleration) {                                           \
            self->setAccelerationLowerLimit(index, acceleration);              \
          },                                                                   \
          nb::arg("index"),                                                    \
          nb::arg("acceleration"))                                             \
      .def(                                                                    \
          "getAccelerationLowerLimit",                                         \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getAccelerationLowerLimit(index);                     \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setAccelerationLowerLimits",                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setAccelerationLowerLimits(lowerLimits);                     \
          },                                                                   \
          nb::arg("lowerLimits"))                                              \
      .def(                                                                    \
          "getAccelerationLowerLimits",                                        \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd {                                             \
            return self->getAccelerationLowerLimits();                         \
          })                                                                   \
      .def(                                                                    \
          "setAccelerationUpperLimit",                                         \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double acceleration) {                                           \
            self->setAccelerationUpperLimit(index, acceleration);              \
          },                                                                   \
          nb::arg("index"),                                                    \
          nb::arg("acceleration"))                                             \
      .def(                                                                    \
          "getAccelerationUpperLimit",                                         \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getAccelerationUpperLimit(index);                     \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setAccelerationUpperLimits",                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setAccelerationUpperLimits(upperLimits);                     \
          },                                                                   \
          nb::arg("upperLimits"))                                              \
      .def(                                                                    \
          "getAccelerationUpperLimits",                                        \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd {                                             \
            return self->getAccelerationUpperLimits();                         \
          })                                                                   \
      .def(                                                                    \
          "resetAccelerations",                                                \
          +[](dart::dynamics::GenericJoint<space>* self) {                     \
            self->resetAccelerations();                                        \
          })                                                                   \
      .def(                                                                    \
          "setForce",                                                          \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double force) { self->setForce(index, force); },                 \
          nb::arg("index"),                                                    \
          nb::arg("force"))                                                    \
      .def(                                                                    \
          "getForce",                                                          \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double { return self->getForce(index); },  \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setForces",                                                         \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& forces) { self->setForces(forces); },     \
          nb::arg("forces"))                                                   \
      .def(                                                                    \
          "getForces",                                                         \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getForces(); })                \
      .def(                                                                    \
          "setForceLowerLimit",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double force) { self->setForceLowerLimit(index, force); },       \
          nb::arg("index"),                                                    \
          nb::arg("force"))                                                    \
      .def(                                                                    \
          "getForceLowerLimit",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getForceLowerLimit(index);                            \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setForceLowerLimits",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setForceLowerLimits(lowerLimits);                            \
          },                                                                   \
          nb::arg("lowerLimits"))                                              \
      .def(                                                                    \
          "getForceLowerLimits",                                               \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getForceLowerLimits(); })      \
      .def(                                                                    \
          "setForceUpperLimit",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double force) { self->setForceUpperLimit(index, force); },       \
          nb::arg("index"),                                                    \
          nb::arg("force"))                                                    \
      .def(                                                                    \
          "getForceUpperLimit",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> double { return self->getForceUpperLimit(index); },           \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setForceUpperLimits",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setForceUpperLimits(upperLimits);                            \
          },                                                                   \
          nb::arg("upperLimits"))                                              \
      .def(                                                                    \
          "getForceUpperLimits",                                               \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getForceUpperLimits(); })      \
      .def(                                                                    \
          "resetForces",                                                       \
          +[](dart::dynamics::GenericJoint<space>*                             \
                  self) { self->resetForces(); })                              \
      .def(                                                                    \
          "setVelocityChange",                                                 \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double velocityChange) {                                         \
            self->setVelocityChange(index, velocityChange);                    \
          },                                                                   \
          nb::arg("index"),                                                    \
          nb::arg("velocityChange"))                                           \
      .def(                                                                    \
          "getVelocityChange",                                                 \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocityChange(index);                             \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "resetVelocityChanges",                                              \
          +[](dart::dynamics::GenericJoint<space>* self) {                     \
            self->resetVelocityChanges();                                      \
          })                                                                   \
      .def(                                                                    \
          "setConstraintImpulse",                                              \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double impulse) { self->setConstraintImpulse(index, impulse); }, \
          nb::arg("index"),                                                    \
          nb::arg("impulse"))                                                  \
      .def(                                                                    \
          "getConstraintImpulse",                                              \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getConstraintImpulse(index);                          \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "resetConstraintImpulses",                                           \
          +[](dart::dynamics::GenericJoint<space>* self) {                     \
            self->resetConstraintImpulses();                                   \
          })                                                                   \
      .def(                                                                    \
          "integratePositions",                                                \
          +[](dart::dynamics::GenericJoint<space>* self, double dt) {          \
            self->integratePositions(dt);                                      \
          },                                                                   \
          nb::arg("dt"))                                                       \
      .def(                                                                    \
          "integrateVelocities",                                               \
          +[](dart::dynamics::GenericJoint<space>* self, double dt) {          \
            self->integrateVelocities(dt);                                     \
          },                                                                   \
          nb::arg("dt"))                                                       \
      .def(                                                                    \
          "getPositionDifferences",                                            \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              const Eigen::VectorXd& q2,                                       \
              const Eigen::VectorXd& q1) -> Eigen::VectorXd {                  \
            return self->getPositionDifferences(q2, q1);                       \
          },                                                                   \
          nb::arg("q2"),                                                       \
          nb::arg("q1"))                                                       \
      .def(                                                                    \
          "getPositionDifferencesStatic",                                      \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              const dart::dynamics::GenericJoint<space>::Vector& q2,           \
              const dart::dynamics::GenericJoint<space>::Vector& q1)           \
              -> dart::dynamics::GenericJoint<space>::Vector {                 \
            return self->getPositionDifferencesStatic(q2, q1);                 \
          },                                                                   \
          nb::arg("q2"),                                                       \
          nb::arg("q1"))                                                       \
      .def(                                                                    \
          "setSpringStiffness",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double k) { self->setSpringStiffness(index, k); },               \
          nb::arg("index"),                                                    \
          nb::arg("k"))                                                        \
      .def(                                                                    \
          "getSpringStiffness",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getSpringStiffness(index);                            \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setRestPosition",                                                   \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double q0) { self->setRestPosition(index, q0); },                \
          nb::arg("index"),                                                    \
          nb::arg("q0"))                                                       \
      .def(                                                                    \
          "getRestPosition",                                                   \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getRestPosition(index);                               \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setDampingCoefficient",                                             \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double d) { self->setDampingCoefficient(index, d); },            \
          nb::arg("index"),                                                    \
          nb::arg("d"))                                                        \
      .def(                                                                    \
          "getDampingCoefficient",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getDampingCoefficient(index);                         \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "setCoulombFriction",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double friction) { self->setCoulombFriction(index, friction); }, \
          nb::arg("index"),                                                    \
          nb::arg("friction"))                                                 \
      .def(                                                                    \
          "getCoulombFriction",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getCoulombFriction(index);                            \
          },                                                                   \
          nb::arg("index"))                                                    \
      .def(                                                                    \
          "computePotentialEnergy",                                            \
          +[](const dart::dynamics::GenericJoint<space>* self) -> double {     \
            return self->computePotentialEnergy();                             \
          })                                                                   \
      .def(                                                                    \
          "getBodyConstraintWrench",                                           \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::Vector6d { return self->getBodyConstraintWrench(); })  \
      .def(                                                                    \
          "getRelativeJacobian",                                               \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> const dart::math::Jacobian {                                  \
            return self->getRelativeJacobian();                                \
          })                                                                   \
      .def(                                                                    \
          "getRelativeJacobian",                                               \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              const Eigen::VectorXd& _positions) -> dart::math::Jacobian {     \
            return self->getRelativeJacobian(_positions);                      \
          },                                                                   \
          nb::arg("positions"))                                                \
      .def(                                                                    \
          "getRelativeJacobianStatic",                                         \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              const dart::dynamics::GenericJoint<space>::Vector& positions)    \
              -> dart::dynamics::GenericJoint<space>::JacobianMatrix {         \
            return self->getRelativeJacobianStatic(positions);                 \
          },                                                                   \
          nb::arg("positions"))                                                \
      .def(                                                                    \
          "getRelativeJacobianTimeDeriv",                                      \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> const dart::math::Jacobian {                                  \
            return self->getRelativeJacobianTimeDeriv();                       \
          })                                                                   \
      .def_ro_static(                                                          \
          "NumDofs", &dart::dynamics::GenericJoint<space>::NumDofs);

namespace dart {
namespace python {

void GenericJoint(nb::module_& m)
{
  DARTPY_DEFINE_GENERICJOINT(R1, ::dart::math::RealVectorSpace<1>);
  DARTPY_DEFINE_GENERICJOINT(R2, ::dart::math::RealVectorSpace<2>);
  DARTPY_DEFINE_GENERICJOINT(R3, ::dart::math::RealVectorSpace<3>);
  DARTPY_DEFINE_GENERICJOINT(SO3, ::dart::math::SO3Space);
  DARTPY_DEFINE_GENERICJOINT(SE3, ::dart::math::SE3Space);
}

} // namespace python
} // namespace dart

#undef DARTPY_DEFINE_GENERICJOINT
