/*
 * Copyright (c) 2011-2021, The DART development contributors
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

namespace dart {
namespace python {

#define DARTPY_DEFINE_JOINT_COMMON_BASE(type, space)                           \
  ::pybind11::class_<                                                          \
      dart::common::SpecializedForAspect<                                      \
          dart::common::EmbeddedPropertiesAspect<                              \
              dart::dynamics::type,                                            \
              dart::dynamics::detail::type##UniqueProperties>>,                \
      dart::common::Composite,                                                 \
      std::shared_ptr<dart::common::SpecializedForAspect<                      \
          dart::common::EmbeddedPropertiesAspect<                              \
              dart::dynamics::type,                                            \
              dart::dynamics::detail::type##UniqueProperties>>>>(              \
      m,                                                                       \
      "SpecializedForAspect_EmbeddedPropertiesAspect_" #type "_" #type         \
      "UniqueProperties")                                                      \
      .def(::pybind11::init<>());                                              \
                                                                               \
  ::pybind11::class_<                                                          \
      dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect<     \
          dart::dynamics::type,                                                \
          dart::dynamics::detail::type##UniqueProperties>>,                    \
      dart::common::SpecializedForAspect<                                      \
          dart::common::EmbeddedPropertiesAspect<                              \
              dart::dynamics::type,                                            \
              dart::dynamics::detail::type##UniqueProperties>>,                \
      std::shared_ptr<                                                         \
          dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect< \
              dart::dynamics::type,                                            \
              dart::dynamics::detail::type##UniqueProperties>>>>(              \
      m,                                                                       \
      "RequiresAspect_EmbeddedPropertiesAspect_" #type "_" #type               \
      "UniqueProperties")                                                      \
      .def(::pybind11::init<>());                                              \
                                                                               \
  ::pybind11::class_<                                                          \
      dart::common::EmbedProperties<                                           \
          dart::dynamics::type,                                                \
          dart::dynamics::detail::type##UniqueProperties>,                     \
      dart::common::RequiresAspect<dart::common::EmbeddedPropertiesAspect<     \
          dart::dynamics::type,                                                \
          dart::dynamics::detail::type##UniqueProperties>>,                    \
      std::shared_ptr<dart::common::EmbedProperties<                           \
          dart::dynamics::type,                                                \
          dart::dynamics::detail::type##UniqueProperties>>>(                   \
      m, "EmbedProperties_" #type "_" #type "UniqueProperties");               \
                                                                               \
  ::pybind11::class_<                                                          \
      dart::common::CompositeJoiner<                                           \
          dart::common::EmbedProperties<                                       \
              dart::dynamics::type,                                            \
              dart::dynamics::detail::type##UniqueProperties>,                 \
          dart::dynamics::GenericJoint<dart::math::space>>,                    \
      dart::common::EmbedProperties<                                           \
          dart::dynamics::type,                                                \
          dart::dynamics::detail::type##UniqueProperties>,                     \
      dart::dynamics::GenericJoint<dart::math::space>,                         \
      std::shared_ptr<dart::common::CompositeJoiner<                           \
          dart::common::EmbedProperties<                                       \
              dart::dynamics::type,                                            \
              dart::dynamics::detail::type##UniqueProperties>,                 \
          dart::dynamics::GenericJoint<dart::math::space>>>>(                  \
      m,                                                                       \
      "CompositeJoiner_EmbedProperties_" #type "_" #type                       \
      "UniqueProperties_GenericJoint_" #space);                                \
                                                                               \
  ::pybind11::class_<                                                          \
      dart::common::EmbedPropertiesOnTopOf<                                    \
          dart::dynamics::type,                                                \
          dart::dynamics::detail::type##UniqueProperties,                      \
          dart::dynamics::GenericJoint<dart::math::space>>,                    \
      dart::common::CompositeJoiner<                                           \
          dart::common::EmbedProperties<                                       \
              dart::dynamics::type,                                            \
              dart::dynamics::detail::type##UniqueProperties>,                 \
          dart::dynamics::GenericJoint<dart::math::space>>,                    \
      std::shared_ptr<dart::common::EmbedPropertiesOnTopOf<                    \
          dart::dynamics::type,                                                \
          dart::dynamics::detail::type##UniqueProperties,                      \
          dart::dynamics::GenericJoint<dart::math::space>>>>(                  \
      m,                                                                       \
      "EmbedPropertiesOnTopOf_" #type "_" #type                                \
      "UniqueProperties_GenericJoint_" #space);

} // namespace python
} // namespace dart
