/*
 * Copyright (c) 2011-2022, The DART development contributors
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

#include <eigen_geometry_pybind.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

#define DARTPY_DEFINE_GENERICJOINT(name, space)                                \
  ::py::class_<dart::dynamics::detail::GenericJointUniqueProperties<space>>(   \
      m, "GenericJointUniqueProperties_" #name)                                \
      .def(::py::init<>())                                                     \
      .def(                                                                    \
          ::py::init<                                                          \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&>(),                                  \
          ::py::arg("positionLowerLimits"))                                    \
      .def(                                                                    \
          ::py::init<                                                          \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&>(),                                  \
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"))                                    \
      .def(                                                                    \
          ::py::init<                                                          \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&>(),                                  \
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"))                                       \
      .def(                                                                    \
          ::py::init<                                                          \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"))                                    \
      .def(                                                                    \
          ::py::init<                                                          \
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
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"),                                    \
          ::py::arg("velocityUpperLimits"))                                    \
      .def(                                                                    \
          ::py::init<                                                          \
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
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"),                                    \
          ::py::arg("velocityUpperLimits"),                                    \
          ::py::arg("initialVelocities"))                                      \
      .def(                                                                    \
          ::py::init<                                                          \
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
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"),                                    \
          ::py::arg("velocityUpperLimits"),                                    \
          ::py::arg("initialVelocities"),                                      \
          ::py::arg("accelerationLowerLimits"))                                \
      .def(                                                                    \
          ::py::init<                                                          \
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
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"),                                    \
          ::py::arg("velocityUpperLimits"),                                    \
          ::py::arg("initialVelocities"),                                      \
          ::py::arg("accelerationLowerLimits"),                                \
          ::py::arg("accelerationUpperLimits"))                                \
      .def(                                                                    \
          ::py::init<                                                          \
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
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"),                                    \
          ::py::arg("velocityUpperLimits"),                                    \
          ::py::arg("initialVelocities"),                                      \
          ::py::arg("accelerationLowerLimits"),                                \
          ::py::arg("accelerationUpperLimits"),                                \
          ::py::arg("forceLowerLimits"))                                       \
      .def(                                                                    \
          ::py::init<                                                          \
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
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"),                                    \
          ::py::arg("velocityUpperLimits"),                                    \
          ::py::arg("initialVelocities"),                                      \
          ::py::arg("accelerationLowerLimits"),                                \
          ::py::arg("accelerationUpperLimits"),                                \
          ::py::arg("forceLowerLimits"),                                       \
          ::py::arg("forceUpperLimits"))                                       \
      .def(                                                                    \
          ::py::init<                                                          \
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
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"),                                    \
          ::py::arg("velocityUpperLimits"),                                    \
          ::py::arg("initialVelocities"),                                      \
          ::py::arg("accelerationLowerLimits"),                                \
          ::py::arg("accelerationUpperLimits"),                                \
          ::py::arg("forceLowerLimits"),                                       \
          ::py::arg("forceUpperLimits"),                                       \
          ::py::arg("springStiffness"))                                        \
      .def(                                                                    \
          ::py::init<                                                          \
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
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"),                                    \
          ::py::arg("velocityUpperLimits"),                                    \
          ::py::arg("initialVelocities"),                                      \
          ::py::arg("accelerationLowerLimits"),                                \
          ::py::arg("accelerationUpperLimits"),                                \
          ::py::arg("forceLowerLimits"),                                       \
          ::py::arg("forceUpperLimits"),                                       \
          ::py::arg("springStiffness"),                                        \
          ::py::arg("restPosition"))                                           \
      .def(                                                                    \
          ::py::init<                                                          \
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
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"),                                    \
          ::py::arg("velocityUpperLimits"),                                    \
          ::py::arg("initialVelocities"),                                      \
          ::py::arg("accelerationLowerLimits"),                                \
          ::py::arg("accelerationUpperLimits"),                                \
          ::py::arg("forceLowerLimits"),                                       \
          ::py::arg("forceUpperLimits"),                                       \
          ::py::arg("springStiffness"),                                        \
          ::py::arg("restPosition"),                                           \
          ::py::arg("dampingCoefficient"))                                     \
      .def(                                                                    \
          ::py::init<                                                          \
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
          ::py::arg("positionLowerLimits"),                                    \
          ::py::arg("positionUpperLimits"),                                    \
          ::py::arg("initialPositions"),                                       \
          ::py::arg("velocityLowerLimits"),                                    \
          ::py::arg("velocityUpperLimits"),                                    \
          ::py::arg("initialVelocities"),                                      \
          ::py::arg("accelerationLowerLimits"),                                \
          ::py::arg("accelerationUpperLimits"),                                \
          ::py::arg("forceLowerLimits"),                                       \
          ::py::arg("forceUpperLimits"),                                       \
          ::py::arg("springStiffness"),                                        \
          ::py::arg("restPosition"),                                           \
          ::py::arg("dampingCoefficient"),                                     \
          ::py::arg("coulombFrictions"))                                       \
      .def_readwrite(                                                          \
          "mPositionLowerLimits",                                              \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mPositionLowerLimits)                                    \
      .def_readwrite(                                                          \
          "mPositionUpperLimits",                                              \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mPositionUpperLimits)                                    \
      .def_readwrite(                                                          \
          "mInitialPositions",                                                 \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mInitialPositions)                                       \
      .def_readwrite(                                                          \
          "mVelocityLowerLimits",                                              \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mVelocityLowerLimits)                                    \
      .def_readwrite(                                                          \
          "mVelocityUpperLimits",                                              \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mVelocityUpperLimits)                                    \
      .def_readwrite(                                                          \
          "mInitialVelocities",                                                \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mInitialVelocities)                                      \
      .def_readwrite(                                                          \
          "mAccelerationLowerLimits",                                          \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mAccelerationLowerLimits)                                \
      .def_readwrite(                                                          \
          "mAccelerationUpperLimits",                                          \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mAccelerationUpperLimits)                                \
      .def_readwrite(                                                          \
          "mForceLowerLimits",                                                 \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mForceLowerLimits)                                       \
      .def_readwrite(                                                          \
          "mForceUpperLimits",                                                 \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mForceUpperLimits)                                       \
      .def_readwrite(                                                          \
          "mSpringStiffnesses",                                                \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mSpringStiffnesses)                                      \
      .def_readwrite(                                                          \
          "mRestPositions",                                                    \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mRestPositions)                                          \
      .def_readwrite(                                                          \
          "mDampingCoefficients",                                              \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mDampingCoefficients)                                    \
      .def_readwrite(                                                          \
          "mFrictions",                                                        \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mFrictions)                                              \
      .def_readwrite(                                                          \
          "mPreserveDofNames",                                                 \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mPreserveDofNames)                                       \
      .def_readwrite(                                                          \
          "mDofNames",                                                         \
          &dart::dynamics::detail::GenericJointUniqueProperties<               \
              space>::mDofNames);                                              \
                                                                               \
  ::py::class_<                                                                \
      dart::dynamics::detail::GenericJointProperties<space>,                   \
      dart::dynamics::detail::JointProperties,                                 \
      dart::dynamics::detail::GenericJointUniqueProperties<space>>(            \
      m, "GenericJointProperties_" #name)                                      \
      .def(::py::init<>())                                                     \
      .def(                                                                    \
          ::py::init<const dart::dynamics::Joint::Properties&>(),              \
          ::py::arg("jointProperties"))                                        \
      .def(                                                                    \
          ::py::init<                                                          \
              const dart::dynamics::Joint::Properties&,                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>&>(),                                                  \
          ::py::arg("jointProperties"),                                        \
          ::py::arg("genericProperties"));                                     \
                                                                               \
  ::py::class_<                                                                \
      dart::common::SpecializedForAspect<                                      \
          dart::common::EmbeddedStateAndPropertiesAspect<                      \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>>,   \
      dart::common::Composite,                                                 \
      std::shared_ptr<dart::common::SpecializedForAspect<                      \
          dart::common::EmbeddedStateAndPropertiesAspect<                      \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>>>>( \
      m,                                                                       \
      "SpecializedForAspect_EmbeddedStateAndPropertiesAspect_"                 \
      "GenericJoint_" #name "_GenericJointState_GenericJointUniqueProperties") \
      .def(::py::init<>());                                                    \
                                                                               \
  ::py::class_<                                                                \
      dart::common::RequiresAspect<                                            \
          dart::common::EmbeddedStateAndPropertiesAspect<                      \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>>,   \
      dart::common::SpecializedForAspect<                                      \
          dart::common::EmbeddedStateAndPropertiesAspect<                      \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>>,   \
      std::shared_ptr<dart::common::RequiresAspect<                            \
          dart::common::EmbeddedStateAndPropertiesAspect<                      \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>>>>( \
      m,                                                                       \
      "RequiresAspect_EmbeddedStateAndPropertiesAspect_GenericJoint_" #name    \
      "_GenericJointState_GenericJointUniqueProperties")                       \
      .def(::py::init<>());                                                    \
                                                                               \
  ::py::class_<                                                                \
      dart::common::EmbedStateAndProperties<                                   \
          dart::dynamics::GenericJoint<space>,                                 \
          dart::dynamics::detail::GenericJointState<space>,                    \
          dart::dynamics::detail::GenericJointUniqueProperties<space>>,        \
      dart::common::RequiresAspect<                                            \
          dart::common::EmbeddedStateAndPropertiesAspect<                      \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>>,   \
      std::shared_ptr<dart::common::EmbedStateAndProperties<                   \
          dart::dynamics::GenericJoint<space>,                                 \
          dart::dynamics::detail::GenericJointState<space>,                    \
          dart::dynamics::detail::GenericJointUniqueProperties<space>>>>(      \
      m,                                                                       \
      "EmbedStateAndProperties_GenericJoint_" #name                            \
      "GenericJointState_GenericJointUniqueProperties");                       \
                                                                               \
  ::py::class_<                                                                \
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
      dart::dynamics::Joint,                                                   \
      std::shared_ptr<dart::common::CompositeJoiner<                           \
          dart::common::EmbedStateAndProperties<                               \
              dart::dynamics::GenericJoint<space>,                             \
              dart::dynamics::detail::GenericJointState<space>,                \
              dart::dynamics::detail::GenericJointUniqueProperties<space>>,    \
          dart::dynamics::Joint>>>(                                            \
      m,                                                                       \
      "CompositeJoiner_EmbedStateAndProperties_GenericJoint_" #name            \
      "GenericJointStateGenericJointUniqueProperties_Joint");                  \
                                                                               \
  ::py::class_<                                                                \
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
          dart::dynamics::Joint>,                                              \
      std::shared_ptr<dart::common::EmbedStateAndPropertiesOnTopOf<            \
          dart::dynamics::GenericJoint<space>,                                 \
          dart::dynamics::detail::GenericJointState<space>,                    \
          dart::dynamics::detail::GenericJointUniqueProperties<space>,         \
          dart::dynamics::Joint>>>(                                            \
      m,                                                                       \
      "EmbedStateAndPropertiesOnTopOf_GenericJoint_" #name                     \
      "_GenericJointState_GenericJointUniqueProperties_Joint");                \
                                                                               \
  ::py::class_<                                                                \
      dart::dynamics::GenericJoint<space>,                                     \
      dart::common::EmbedStateAndPropertiesOnTopOf<                            \
          dart::dynamics::GenericJoint<space>,                                 \
          dart::dynamics::detail::GenericJointState<space>,                    \
          dart::dynamics::detail::GenericJointUniqueProperties<space>,         \
          dart::dynamics::Joint>,                                              \
      std::shared_ptr<dart::dynamics::GenericJoint<space>>>(                   \
      m, "GenericJoint_" #name)                                                \
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
          ::py::arg("aspect"))                                                 \
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
          ::py::arg("properties"))                                             \
      .def(                                                                    \
          "setProperties",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::UniqueProperties&     \
                  properties) { self->setProperties(properties); },            \
          ::py::arg("properties"))                                             \
      .def(                                                                    \
          "setAspectState",                                                    \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::AspectState& state) { \
            self->setAspectState(state);                                       \
          },                                                                   \
          ::py::arg("state"))                                                  \
      .def(                                                                    \
          "setAspectProperties",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::AspectProperties&     \
                  properties) { self->setAspectProperties(properties); },      \
          ::py::arg("properties"))                                             \
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
          ::py::arg("otherJoint"))                                             \
      .def(                                                                    \
          "copy",                                                              \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::ThisClass*            \
                  otherJoint) { self->copy(otherJoint); },                     \
          ::py::arg("otherJoint"))                                             \
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
          ::py::return_value_policy::reference_internal,                       \
          ::py::arg("index"),                                                  \
          ::py::arg("name"))                                                   \
      .def(                                                                    \
          "setDofName",                                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              const std::string& name,                                         \
              bool preserveName) -> const std::string& {                       \
            return self->setDofName(index, name, preserveName);                \
          },                                                                   \
          ::py::return_value_policy::reference_internal,                       \
          ::py::arg("index"),                                                  \
          ::py::arg("name"),                                                   \
          ::py::arg("preserveName"))                                           \
      .def(                                                                    \
          "preserveDofName",                                                   \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              bool preserve) { self->preserveDofName(index, preserve); },      \
          ::py::arg("index"),                                                  \
          ::py::arg("preserve"))                                               \
      .def(                                                                    \
          "isDofNamePreserved",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> bool { return self->isDofNamePreserved(index); },             \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "getDofName",                                                        \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> const std::string& { return self->getDofName(index); },       \
          ::py::return_value_policy::reference_internal,                       \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "getIndexInSkeleton",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> size_t { return self->getIndexInSkeleton(index); },           \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "getIndexInTree",                                                    \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              size_t index) -> size_t { return self->getIndexInTree(index); }, \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setCommand",                                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double command) { self->setCommand(index, command); },           \
          ::py::arg("index"),                                                  \
          ::py::arg("command"))                                                \
      .def(                                                                    \
          "getCommand",                                                        \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getCommand(index);                                    \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setCommands",                                                       \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& commands) {                               \
            self->setCommands(commands);                                       \
          },                                                                   \
          ::py::arg("commands"))                                               \
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
          ::py::arg("index"),                                                  \
          ::py::arg("position"))                                               \
      .def(                                                                    \
          "getPosition",                                                       \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getPosition(index);                                   \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setPositions",                                                      \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& positions) {                              \
            self->setPositions(positions);                                     \
          },                                                                   \
          ::py::arg("positions"))                                              \
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
          ::py::arg("index"),                                                  \
          ::py::arg("position"))                                               \
      .def(                                                                    \
          "getPositionLowerLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getPositionLowerLimit(index);                         \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setPositionLowerLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setPositionLowerLimits(lowerLimits);                         \
          },                                                                   \
          ::py::arg("lowerLimits"))                                            \
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
          ::py::arg("index"),                                                  \
          ::py::arg("position"))                                               \
      .def(                                                                    \
          "getPositionUpperLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getPositionUpperLimit(index);                         \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setPositionUpperLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setPositionUpperLimits(upperLimits);                         \
          },                                                                   \
          ::py::arg("upperLimits"))                                            \
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
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "resetPosition",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self, std::size_t index) {  \
            self->resetPosition(index);                                        \
          },                                                                   \
          ::py::arg("index"))                                                  \
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
          ::py::arg("index"),                                                  \
          ::py::arg("initial"))                                                \
      .def(                                                                    \
          "getInitialPosition",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getInitialPosition(index);                            \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setInitialPositions",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& initial) {                                \
            self->setInitialPositions(initial);                                \
          },                                                                   \
          ::py::arg("initial"))                                                \
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
          ::py::arg("positions"))                                              \
      .def(                                                                    \
          "setVelocitiesStatic",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::Vector& velocities) { \
            self->setVelocitiesStatic(velocities);                             \
          },                                                                   \
          ::py::arg("velocities"))                                             \
      .def(                                                                    \
          "setAccelerationsStatic",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::Vector& accels) {     \
            self->setAccelerationsStatic(accels);                              \
          },                                                                   \
          ::py::arg("accels"))                                                 \
      .def(                                                                    \
          "setVelocity",                                                       \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double velocity) { self->setVelocity(index, velocity); },        \
          ::py::arg("index"),                                                  \
          ::py::arg("velocity"))                                               \
      .def(                                                                    \
          "getVelocity",                                                       \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocity(index);                                   \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setVelocities",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& velocities) {                             \
            self->setVelocities(velocities);                                   \
          },                                                                   \
          ::py::arg("velocities"))                                             \
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
          ::py::arg("index"),                                                  \
          ::py::arg("velocity"))                                               \
      .def(                                                                    \
          "getVelocityLowerLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocityLowerLimit(index);                         \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setVelocityLowerLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setVelocityLowerLimits(lowerLimits);                         \
          },                                                                   \
          ::py::arg("lowerLimits"))                                            \
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
          ::py::arg("index"),                                                  \
          ::py::arg("velocity"))                                               \
      .def(                                                                    \
          "getVelocityUpperLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocityUpperLimit(index);                         \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setVelocityUpperLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setVelocityUpperLimits(upperLimits);                         \
          },                                                                   \
          ::py::arg("upperLimits"))                                            \
      .def(                                                                    \
          "getVelocityUpperLimits",                                            \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getVelocityUpperLimits(); })   \
      .def(                                                                    \
          "resetVelocity",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self, std::size_t index) {  \
            self->resetVelocity(index);                                        \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "resetVelocities",                                                   \
          +[](dart::dynamics::GenericJoint<space>*                             \
                  self) { self->resetVelocities(); })                          \
      .def(                                                                    \
          "setInitialVelocity",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double initial) { self->setInitialVelocity(index, initial); },   \
          ::py::arg("index"),                                                  \
          ::py::arg("initial"))                                                \
      .def(                                                                    \
          "getInitialVelocity",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getInitialVelocity(index);                            \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setInitialVelocities",                                              \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& initial) {                                \
            self->setInitialVelocities(initial);                               \
          },                                                                   \
          ::py::arg("initial"))                                                \
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
          ::py::arg("index"),                                                  \
          ::py::arg("acceleration"))                                           \
      .def(                                                                    \
          "getAcceleration",                                                   \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getAcceleration(index);                               \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setAccelerations",                                                  \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& accelerations) {                          \
            self->setAccelerations(accelerations);                             \
          },                                                                   \
          ::py::arg("accelerations"))                                          \
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
          ::py::arg("index"),                                                  \
          ::py::arg("acceleration"))                                           \
      .def(                                                                    \
          "getAccelerationLowerLimit",                                         \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getAccelerationLowerLimit(index);                     \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setAccelerationLowerLimits",                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setAccelerationLowerLimits(lowerLimits);                     \
          },                                                                   \
          ::py::arg("lowerLimits"))                                            \
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
          ::py::arg("index"),                                                  \
          ::py::arg("acceleration"))                                           \
      .def(                                                                    \
          "getAccelerationUpperLimit",                                         \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getAccelerationUpperLimit(index);                     \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setAccelerationUpperLimits",                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setAccelerationUpperLimits(upperLimits);                     \
          },                                                                   \
          ::py::arg("upperLimits"))                                            \
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
          ::py::arg("index"),                                                  \
          ::py::arg("force"))                                                  \
      .def(                                                                    \
          "getForce",                                                          \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double { return self->getForce(index); },  \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setForces",                                                         \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& forces) { self->setForces(forces); },     \
          ::py::arg("forces"))                                                 \
      .def(                                                                    \
          "getForces",                                                         \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getForces(); })                \
      .def(                                                                    \
          "setForceLowerLimit",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double force) { self->setForceLowerLimit(index, force); },       \
          ::py::arg("index"),                                                  \
          ::py::arg("force"))                                                  \
      .def(                                                                    \
          "getForceLowerLimit",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getForceLowerLimit(index);                            \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setForceLowerLimits",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setForceLowerLimits(lowerLimits);                            \
          },                                                                   \
          ::py::arg("lowerLimits"))                                            \
      .def(                                                                    \
          "getForceLowerLimits",                                               \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getForceLowerLimits(); })      \
      .def(                                                                    \
          "setForceUpperLimit",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double force) { self->setForceUpperLimit(index, force); },       \
          ::py::arg("index"),                                                  \
          ::py::arg("force"))                                                  \
      .def(                                                                    \
          "getForceUpperLimit",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> double { return self->getForceUpperLimit(index); },           \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setForceUpperLimits",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setForceUpperLimits(upperLimits);                            \
          },                                                                   \
          ::py::arg("upperLimits"))                                            \
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
          ::py::arg("index"),                                                  \
          ::py::arg("velocityChange"))                                         \
      .def(                                                                    \
          "getVelocityChange",                                                 \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocityChange(index);                             \
          },                                                                   \
          ::py::arg("index"))                                                  \
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
          ::py::arg("index"),                                                  \
          ::py::arg("impulse"))                                                \
      .def(                                                                    \
          "getConstraintImpulse",                                              \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getConstraintImpulse(index);                          \
          },                                                                   \
          ::py::arg("index"))                                                  \
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
          ::py::arg("dt"))                                                     \
      .def(                                                                    \
          "integrateVelocities",                                               \
          +[](dart::dynamics::GenericJoint<space>* self, double dt) {          \
            self->integrateVelocities(dt);                                     \
          },                                                                   \
          ::py::arg("dt"))                                                     \
      .def(                                                                    \
          "getPositionDifferences",                                            \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              const Eigen::VectorXd& q2,                                       \
              const Eigen::VectorXd& q1) -> Eigen::VectorXd {                  \
            return self->getPositionDifferences(q2, q1);                       \
          },                                                                   \
          ::py::arg("q2"),                                                     \
          ::py::arg("q1"))                                                     \
      .def(                                                                    \
          "getPositionDifferencesStatic",                                      \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              const dart::dynamics::GenericJoint<space>::Vector& q2,           \
              const dart::dynamics::GenericJoint<space>::Vector& q1)           \
              -> dart::dynamics::GenericJoint<space>::Vector {                 \
            return self->getPositionDifferencesStatic(q2, q1);                 \
          },                                                                   \
          ::py::arg("q2"),                                                     \
          ::py::arg("q1"))                                                     \
      .def(                                                                    \
          "setSpringStiffness",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double k) { self->setSpringStiffness(index, k); },               \
          ::py::arg("index"),                                                  \
          ::py::arg("k"))                                                      \
      .def(                                                                    \
          "getSpringStiffness",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getSpringStiffness(index);                            \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setRestPosition",                                                   \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double q0) { self->setRestPosition(index, q0); },                \
          ::py::arg("index"),                                                  \
          ::py::arg("q0"))                                                     \
      .def(                                                                    \
          "getRestPosition",                                                   \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getRestPosition(index);                               \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setDampingCoefficient",                                             \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double d) { self->setDampingCoefficient(index, d); },            \
          ::py::arg("index"),                                                  \
          ::py::arg("d"))                                                      \
      .def(                                                                    \
          "getDampingCoefficient",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getDampingCoefficient(index);                         \
          },                                                                   \
          ::py::arg("index"))                                                  \
      .def(                                                                    \
          "setCoulombFriction",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double friction) { self->setCoulombFriction(index, friction); }, \
          ::py::arg("index"),                                                  \
          ::py::arg("friction"))                                               \
      .def(                                                                    \
          "getCoulombFriction",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getCoulombFriction(index);                            \
          },                                                                   \
          ::py::arg("index"))                                                  \
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
          ::py::arg("positions"))                                              \
      .def(                                                                    \
          "getRelativeJacobianStatic",                                         \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              const dart::dynamics::GenericJoint<space>::Vector& positions)    \
              -> dart::dynamics::GenericJoint<space>::JacobianMatrix {         \
            return self->getRelativeJacobianStatic(positions);                 \
          },                                                                   \
          ::py::arg("positions"))                                              \
      .def(                                                                    \
          "getRelativeJacobianTimeDeriv",                                      \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> const dart::math::Jacobian {                                  \
            return self->getRelativeJacobianTimeDeriv();                       \
          })                                                                   \
      .def_readonly_static(                                                    \
          "NumDofs", &dart::dynamics::GenericJoint<space>::NumDofs);

namespace dart {
namespace python {

void GenericJoint(py::module& m)
{
  DARTPY_DEFINE_GENERICJOINT(R1, ::dart::math::RealVectorSpace<1>);
  DARTPY_DEFINE_GENERICJOINT(R2, ::dart::math::RealVectorSpace<2>);
  DARTPY_DEFINE_GENERICJOINT(R3, ::dart::math::RealVectorSpace<3>);
  DARTPY_DEFINE_GENERICJOINT(SO3, ::dart::math::SO3Space);
  DARTPY_DEFINE_GENERICJOINT(SE3, ::dart::math::SE3Space);
}

} // namespace python
} // namespace dart
