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
#include <eigen_geometry_pybind.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#define DARTPY_DEFINE_GENERICJOINT(name, space)                                \
  ::pybind11::class_<                                                          \
      dart::dynamics::detail::GenericJointUniqueProperties<space>>(            \
      m, "GenericJointUniqueProperties_" #name)                                \
      .def(::pybind11::init<>())                                               \
      .def(                                                                    \
          ::pybind11::init<                                                    \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&>(),                                  \
          ::pybind11::arg("positionLowerLimits"))                              \
      .def(                                                                    \
          ::pybind11::init<                                                    \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&>(),                                  \
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"))                              \
      .def(                                                                    \
          ::pybind11::init<                                                    \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&>(),                                  \
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"))                                 \
      .def(                                                                    \
          ::pybind11::init<                                                    \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::EuclideanPoint&,                                     \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>::Vector&>(),                                          \
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"))                              \
      .def(                                                                    \
          ::pybind11::init<                                                    \
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
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"),                              \
          ::pybind11::arg("velocityUpperLimits"))                              \
      .def(                                                                    \
          ::pybind11::init<                                                    \
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
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"),                              \
          ::pybind11::arg("velocityUpperLimits"),                              \
          ::pybind11::arg("initialVelocities"))                                \
      .def(                                                                    \
          ::pybind11::init<                                                    \
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
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"),                              \
          ::pybind11::arg("velocityUpperLimits"),                              \
          ::pybind11::arg("initialVelocities"),                                \
          ::pybind11::arg("accelerationLowerLimits"))                          \
      .def(                                                                    \
          ::pybind11::init<                                                    \
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
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"),                              \
          ::pybind11::arg("velocityUpperLimits"),                              \
          ::pybind11::arg("initialVelocities"),                                \
          ::pybind11::arg("accelerationLowerLimits"),                          \
          ::pybind11::arg("accelerationUpperLimits"))                          \
      .def(                                                                    \
          ::pybind11::init<                                                    \
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
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"),                              \
          ::pybind11::arg("velocityUpperLimits"),                              \
          ::pybind11::arg("initialVelocities"),                                \
          ::pybind11::arg("accelerationLowerLimits"),                          \
          ::pybind11::arg("accelerationUpperLimits"),                          \
          ::pybind11::arg("forceLowerLimits"))                                 \
      .def(                                                                    \
          ::pybind11::init<                                                    \
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
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"),                              \
          ::pybind11::arg("velocityUpperLimits"),                              \
          ::pybind11::arg("initialVelocities"),                                \
          ::pybind11::arg("accelerationLowerLimits"),                          \
          ::pybind11::arg("accelerationUpperLimits"),                          \
          ::pybind11::arg("forceLowerLimits"),                                 \
          ::pybind11::arg("forceUpperLimits"))                                 \
      .def(                                                                    \
          ::pybind11::init<                                                    \
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
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"),                              \
          ::pybind11::arg("velocityUpperLimits"),                              \
          ::pybind11::arg("initialVelocities"),                                \
          ::pybind11::arg("accelerationLowerLimits"),                          \
          ::pybind11::arg("accelerationUpperLimits"),                          \
          ::pybind11::arg("forceLowerLimits"),                                 \
          ::pybind11::arg("forceUpperLimits"),                                 \
          ::pybind11::arg("springStiffness"))                                  \
      .def(                                                                    \
          ::pybind11::init<                                                    \
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
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"),                              \
          ::pybind11::arg("velocityUpperLimits"),                              \
          ::pybind11::arg("initialVelocities"),                                \
          ::pybind11::arg("accelerationLowerLimits"),                          \
          ::pybind11::arg("accelerationUpperLimits"),                          \
          ::pybind11::arg("forceLowerLimits"),                                 \
          ::pybind11::arg("forceUpperLimits"),                                 \
          ::pybind11::arg("springStiffness"),                                  \
          ::pybind11::arg("restPosition"))                                     \
      .def(                                                                    \
          ::pybind11::init<                                                    \
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
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"),                              \
          ::pybind11::arg("velocityUpperLimits"),                              \
          ::pybind11::arg("initialVelocities"),                                \
          ::pybind11::arg("accelerationLowerLimits"),                          \
          ::pybind11::arg("accelerationUpperLimits"),                          \
          ::pybind11::arg("forceLowerLimits"),                                 \
          ::pybind11::arg("forceUpperLimits"),                                 \
          ::pybind11::arg("springStiffness"),                                  \
          ::pybind11::arg("restPosition"),                                     \
          ::pybind11::arg("dampingCoefficient"))                               \
      .def(                                                                    \
          ::pybind11::init<                                                    \
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
          ::pybind11::arg("positionLowerLimits"),                              \
          ::pybind11::arg("positionUpperLimits"),                              \
          ::pybind11::arg("initialPositions"),                                 \
          ::pybind11::arg("velocityLowerLimits"),                              \
          ::pybind11::arg("velocityUpperLimits"),                              \
          ::pybind11::arg("initialVelocities"),                                \
          ::pybind11::arg("accelerationLowerLimits"),                          \
          ::pybind11::arg("accelerationUpperLimits"),                          \
          ::pybind11::arg("forceLowerLimits"),                                 \
          ::pybind11::arg("forceUpperLimits"),                                 \
          ::pybind11::arg("springStiffness"),                                  \
          ::pybind11::arg("restPosition"),                                     \
          ::pybind11::arg("dampingCoefficient"),                               \
          ::pybind11::arg("coulombFrictions"))                                 \
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
  ::pybind11::class_<                                                          \
      dart::dynamics::detail::GenericJointProperties<space>,                   \
      dart::dynamics::detail::JointProperties,                                 \
      dart::dynamics::detail::GenericJointUniqueProperties<space>>(            \
      m, "GenericJointProperties_" #name)                                      \
      .def(::pybind11::init<>())                                               \
      .def(                                                                    \
          ::pybind11::init<const dart::dynamics::Joint::Properties&>(),        \
          ::pybind11::arg("jointProperties"))                                  \
      .def(                                                                    \
          ::pybind11::init<                                                    \
              const dart::dynamics::Joint::Properties&,                        \
              const dart::dynamics::detail::GenericJointUniqueProperties<      \
                  space>&>(),                                                  \
          ::pybind11::arg("jointProperties"),                                  \
          ::pybind11::arg("genericProperties"));                               \
                                                                               \
  ::pybind11::class_<                                                          \
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
      .def(::pybind11::init<>());                                              \
                                                                               \
  ::pybind11::class_<                                                          \
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
      .def(::pybind11::init<>());                                              \
                                                                               \
  ::pybind11::class_<                                                          \
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
  ::pybind11::class_<                                                          \
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
  ::pybind11::class_<                                                          \
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
  ::pybind11::class_<                                                          \
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
          ::pybind11::arg("aspect"))                                           \
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
          ::pybind11::arg("properties"))                                       \
      .def(                                                                    \
          "setProperties",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::UniqueProperties&     \
                  properties) { self->setProperties(properties); },            \
          ::pybind11::arg("properties"))                                       \
      .def(                                                                    \
          "setAspectState",                                                    \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::AspectState& state) { \
            self->setAspectState(state);                                       \
          },                                                                   \
          ::pybind11::arg("state"))                                            \
      .def(                                                                    \
          "setAspectProperties",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::AspectProperties&     \
                  properties) { self->setAspectProperties(properties); },      \
          ::pybind11::arg("properties"))                                       \
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
          ::pybind11::arg("otherJoint"))                                       \
      .def(                                                                    \
          "copy",                                                              \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::ThisClass*            \
                  otherJoint) { self->copy(otherJoint); },                     \
          ::pybind11::arg("otherJoint"))                                       \
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
          ::pybind11::return_value_policy::reference_internal,                 \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("name"))                                             \
      .def(                                                                    \
          "setDofName",                                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              const std::string& name,                                         \
              bool preserveName) -> const std::string& {                       \
            return self->setDofName(index, name, preserveName);                \
          },                                                                   \
          ::pybind11::return_value_policy::reference_internal,                 \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("name"),                                             \
          ::pybind11::arg("preserveName"))                                     \
      .def(                                                                    \
          "preserveDofName",                                                   \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              bool preserve) { self->preserveDofName(index, preserve); },      \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("preserve"))                                         \
      .def(                                                                    \
          "isDofNamePreserved",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> bool { return self->isDofNamePreserved(index); },             \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "getDofName",                                                        \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> const std::string& { return self->getDofName(index); },       \
          ::pybind11::return_value_policy::reference_internal,                 \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "getIndexInSkeleton",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> size_t { return self->getIndexInSkeleton(index); },           \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "getIndexInTree",                                                    \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              size_t index) -> size_t { return self->getIndexInTree(index); }, \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setCommand",                                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double command) { self->setCommand(index, command); },           \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("command"))                                          \
      .def(                                                                    \
          "getCommand",                                                        \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getCommand(index);                                    \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setCommands",                                                       \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& commands) {                               \
            self->setCommands(commands);                                       \
          },                                                                   \
          ::pybind11::arg("commands"))                                         \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("position"))                                         \
      .def(                                                                    \
          "getPosition",                                                       \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getPosition(index);                                   \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setPositions",                                                      \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& positions) {                              \
            self->setPositions(positions);                                     \
          },                                                                   \
          ::pybind11::arg("positions"))                                        \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("position"))                                         \
      .def(                                                                    \
          "getPositionLowerLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getPositionLowerLimit(index);                         \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setPositionLowerLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setPositionLowerLimits(lowerLimits);                         \
          },                                                                   \
          ::pybind11::arg("lowerLimits"))                                      \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("position"))                                         \
      .def(                                                                    \
          "getPositionUpperLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getPositionUpperLimit(index);                         \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setPositionUpperLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setPositionUpperLimits(upperLimits);                         \
          },                                                                   \
          ::pybind11::arg("upperLimits"))                                      \
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
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "resetPosition",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self, std::size_t index) {  \
            self->resetPosition(index);                                        \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("initial"))                                          \
      .def(                                                                    \
          "getInitialPosition",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getInitialPosition(index);                            \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setInitialPositions",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& initial) {                                \
            self->setInitialPositions(initial);                                \
          },                                                                   \
          ::pybind11::arg("initial"))                                          \
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
          ::pybind11::arg("positions"))                                        \
      .def(                                                                    \
          "setVelocitiesStatic",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::Vector& velocities) { \
            self->setVelocitiesStatic(velocities);                             \
          },                                                                   \
          ::pybind11::arg("velocities"))                                       \
      .def(                                                                    \
          "setAccelerationsStatic",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const dart::dynamics::GenericJoint<space>::Vector& accels) {     \
            self->setAccelerationsStatic(accels);                              \
          },                                                                   \
          ::pybind11::arg("accels"))                                           \
      .def(                                                                    \
          "setVelocity",                                                       \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              std::size_t index,                                               \
              double velocity) { self->setVelocity(index, velocity); },        \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("velocity"))                                         \
      .def(                                                                    \
          "getVelocity",                                                       \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocity(index);                                   \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setVelocities",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& velocities) {                             \
            self->setVelocities(velocities);                                   \
          },                                                                   \
          ::pybind11::arg("velocities"))                                       \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("velocity"))                                         \
      .def(                                                                    \
          "getVelocityLowerLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocityLowerLimit(index);                         \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setVelocityLowerLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setVelocityLowerLimits(lowerLimits);                         \
          },                                                                   \
          ::pybind11::arg("lowerLimits"))                                      \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("velocity"))                                         \
      .def(                                                                    \
          "getVelocityUpperLimit",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocityUpperLimit(index);                         \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setVelocityUpperLimits",                                            \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setVelocityUpperLimits(upperLimits);                         \
          },                                                                   \
          ::pybind11::arg("upperLimits"))                                      \
      .def(                                                                    \
          "getVelocityUpperLimits",                                            \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getVelocityUpperLimits(); })   \
      .def(                                                                    \
          "resetVelocity",                                                     \
          +[](dart::dynamics::GenericJoint<space>* self, std::size_t index) {  \
            self->resetVelocity(index);                                        \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "resetVelocities",                                                   \
          +[](dart::dynamics::GenericJoint<space>*                             \
                  self) { self->resetVelocities(); })                          \
      .def(                                                                    \
          "setInitialVelocity",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double initial) { self->setInitialVelocity(index, initial); },   \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("initial"))                                          \
      .def(                                                                    \
          "getInitialVelocity",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getInitialVelocity(index);                            \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setInitialVelocities",                                              \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& initial) {                                \
            self->setInitialVelocities(initial);                               \
          },                                                                   \
          ::pybind11::arg("initial"))                                          \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("acceleration"))                                     \
      .def(                                                                    \
          "getAcceleration",                                                   \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getAcceleration(index);                               \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setAccelerations",                                                  \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& accelerations) {                          \
            self->setAccelerations(accelerations);                             \
          },                                                                   \
          ::pybind11::arg("accelerations"))                                    \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("acceleration"))                                     \
      .def(                                                                    \
          "getAccelerationLowerLimit",                                         \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getAccelerationLowerLimit(index);                     \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setAccelerationLowerLimits",                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setAccelerationLowerLimits(lowerLimits);                     \
          },                                                                   \
          ::pybind11::arg("lowerLimits"))                                      \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("acceleration"))                                     \
      .def(                                                                    \
          "getAccelerationUpperLimit",                                         \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getAccelerationUpperLimit(index);                     \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setAccelerationUpperLimits",                                        \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setAccelerationUpperLimits(upperLimits);                     \
          },                                                                   \
          ::pybind11::arg("upperLimits"))                                      \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("force"))                                            \
      .def(                                                                    \
          "getForce",                                                          \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double { return self->getForce(index); },  \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setForces",                                                         \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& forces) { self->setForces(forces); },     \
          ::pybind11::arg("forces"))                                           \
      .def(                                                                    \
          "getForces",                                                         \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getForces(); })                \
      .def(                                                                    \
          "setForceLowerLimit",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double force) { self->setForceLowerLimit(index, force); },       \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("force"))                                            \
      .def(                                                                    \
          "getForceLowerLimit",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getForceLowerLimit(index);                            \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setForceLowerLimits",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& lowerLimits) {                            \
            self->setForceLowerLimits(lowerLimits);                            \
          },                                                                   \
          ::pybind11::arg("lowerLimits"))                                      \
      .def(                                                                    \
          "getForceLowerLimits",                                               \
          +[](const dart::dynamics::GenericJoint<space>* self)                 \
              -> Eigen::VectorXd { return self->getForceLowerLimits(); })      \
      .def(                                                                    \
          "setForceUpperLimit",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double force) { self->setForceUpperLimit(index, force); },       \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("force"))                                            \
      .def(                                                                    \
          "getForceUpperLimit",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self, size_t index)   \
              -> double { return self->getForceUpperLimit(index); },           \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setForceUpperLimits",                                               \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              const Eigen::VectorXd& upperLimits) {                            \
            self->setForceUpperLimits(upperLimits);                            \
          },                                                                   \
          ::pybind11::arg("upperLimits"))                                      \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("velocityChange"))                                   \
      .def(                                                                    \
          "getVelocityChange",                                                 \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getVelocityChange(index);                             \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
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
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("impulse"))                                          \
      .def(                                                                    \
          "getConstraintImpulse",                                              \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getConstraintImpulse(index);                          \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
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
          ::pybind11::arg("dt"))                                               \
      .def(                                                                    \
          "integrateVelocities",                                               \
          +[](dart::dynamics::GenericJoint<space>* self, double dt) {          \
            self->integrateVelocities(dt);                                     \
          },                                                                   \
          ::pybind11::arg("dt"))                                               \
      .def(                                                                    \
          "getPositionDifferences",                                            \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              const Eigen::VectorXd& q2,                                       \
              const Eigen::VectorXd& q1) -> Eigen::VectorXd {                  \
            return self->getPositionDifferences(q2, q1);                       \
          },                                                                   \
          ::pybind11::arg("q2"),                                               \
          ::pybind11::arg("q1"))                                               \
      .def(                                                                    \
          "getPositionDifferencesStatic",                                      \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              const dart::dynamics::GenericJoint<space>::Vector& q2,           \
              const dart::dynamics::GenericJoint<space>::Vector& q1)           \
              -> dart::dynamics::GenericJoint<space>::Vector {                 \
            return self->getPositionDifferencesStatic(q2, q1);                 \
          },                                                                   \
          ::pybind11::arg("q2"),                                               \
          ::pybind11::arg("q1"))                                               \
      .def(                                                                    \
          "setSpringStiffness",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double k) { self->setSpringStiffness(index, k); },               \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("k"))                                                \
      .def(                                                                    \
          "getSpringStiffness",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getSpringStiffness(index);                            \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setRestPosition",                                                   \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double q0) { self->setRestPosition(index, q0); },                \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("q0"))                                               \
      .def(                                                                    \
          "getRestPosition",                                                   \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getRestPosition(index);                               \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setDampingCoefficient",                                             \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double d) { self->setDampingCoefficient(index, d); },            \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("d"))                                                \
      .def(                                                                    \
          "getDampingCoefficient",                                             \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getDampingCoefficient(index);                         \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
      .def(                                                                    \
          "setCoulombFriction",                                                \
          +[](dart::dynamics::GenericJoint<space>* self,                       \
              size_t index,                                                    \
              double friction) { self->setCoulombFriction(index, friction); }, \
          ::pybind11::arg("index"),                                            \
          ::pybind11::arg("friction"))                                         \
      .def(                                                                    \
          "getCoulombFriction",                                                \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              std::size_t index) -> double {                                   \
            return self->getCoulombFriction(index);                            \
          },                                                                   \
          ::pybind11::arg("index"))                                            \
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
          ::pybind11::arg("positions"))                                        \
      .def(                                                                    \
          "getRelativeJacobianStatic",                                         \
          +[](const dart::dynamics::GenericJoint<space>* self,                 \
              const dart::dynamics::GenericJoint<space>::Vector& positions)    \
              -> dart::dynamics::GenericJoint<space>::JacobianMatrix {         \
            return self->getRelativeJacobianStatic(positions);                 \
          },                                                                   \
          ::pybind11::arg("positions"))                                        \
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

void GenericJoint(pybind11::module& m)
{
  DARTPY_DEFINE_GENERICJOINT(R1, ::dart::math::RealVectorSpace<1>);
  DARTPY_DEFINE_GENERICJOINT(R2, ::dart::math::RealVectorSpace<2>);
  DARTPY_DEFINE_GENERICJOINT(R3, ::dart::math::RealVectorSpace<3>);
  DARTPY_DEFINE_GENERICJOINT(SO3, ::dart::math::SO3Space);
  DARTPY_DEFINE_GENERICJOINT(SE3, ::dart::math::SE3Space);
}

} // namespace python
} // namespace dart
