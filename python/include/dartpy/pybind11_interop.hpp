
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

#ifndef DARTPY_PYBIND11_INTEROP_HPP_
#define DARTPY_PYBIND11_INTEROP_HPP_

// pybind11 casters that exchange DART objects with dartpy through its C API,
// preserving wrapper identity, base-class pointer adjustment, and ownership.
// Include this header before any binding code that uses the listed DART
// types, in every translation unit of the extension, and do not register
// those types with pybind11::class_. Add other polymorphic DART classes that
// dartpy binds with DARTPY_PYBIND11_INTEROP_OBJECT at global scope.

// clang-format off
// First: c_api.hpp includes Python.h, which must precede standard headers.
#include <dartpy/c_api.hpp>
// clang-format on

#include <dart/simulation/World.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/ShapeNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>

#include <Eigen/Geometry>
#include <pybind11/pybind11.h>

#include <memory>
#include <string>
#include <type_traits>

#include <cstring>

namespace dartpy {
namespace pybind11_interop {

/// Returns dartpy's C API, importing dartpy on first use.
inline const c_api::CApi& api()
{
  static const c_api::CApi* table = nullptr;
  if (!table) {
    auto* candidate = static_cast<const c_api::CApi*>(
        PyCapsule_Import(c_api::kCapsuleName, 0));
    if (!candidate)
      throw pybind11::error_already_set();
    if (candidate->version != c_api::kVersion
        || candidate->size < sizeof(c_api::CApi)
        || std::strcmp(candidate->abiTag, c_api::kAbiTag) != 0) {
      throw pybind11::import_error(
          std::string("dartpy C API mismatch: dartpy provides ")
          + candidate->abiTag + " version " + std::to_string(candidate->version)
          + ", this extension expects " + c_api::kAbiTag + " version "
          + std::to_string(c_api::kVersion)
          + "; rebuild the extension against the DART that dartpy uses");
    }
    table = candidate;
  }
  return *table;
}

/// Rejects a wrong-typed argument so pybind11 can try the next overload.
inline bool rejectArgument()
{
  if (PyErr_ExceptionMatches(PyExc_TypeError)) {
    PyErr_Clear();
    return false;
  }
  throw pybind11::error_already_set();
}

inline pybind11::handle checked(PyObject* object)
{
  if (!object)
    throw pybind11::error_already_set();
  return object;
}

/// Python name of a DART type in pybind11 signatures.
template <class T>
struct PythonName;

/// Caster for raw pointers and references to a polymorphic DART class.
template <class T>
class ObjectCaster
{
  static_assert(
      std::is_polymorphic_v<T>,
      "DARTPY_PYBIND11_INTEROP_OBJECT needs a polymorphic DART class; use "
      "DARTPY_PYBIND11_INTEROP_VALUE for value types");

public:
  static constexpr auto name = PythonName<T>::value;

  template <class U>
  using cast_op_type = pybind11::detail::cast_op_type<U>;

  bool load(pybind11::handle src, bool)
  {
    if (src.is_none()) {
      mValue = nullptr;
      return true;
    }
    mValue = static_cast<T*>(api().unwrap(src.ptr(), typeid(T)));
    return mValue || rejectArgument();
  }

  operator T*()
  {
    return mValue;
  }

  operator T&()
  {
    if (!mValue)
      throw pybind11::reference_cast_error();
    return *mValue;
  }

  static pybind11::handle cast(
      const T* pointer,
      pybind11::return_value_policy policy,
      pybind11::handle parent)
  {
    using Policy = pybind11::return_value_policy;
    if (!pointer)
      return pybind11::none().release();
    // dartpy cannot take, copy, or move these objects, which is what the
    // other policies, including the default `automatic`, would request.
    if (policy != Policy::reference && policy != Policy::reference_internal
        && policy != Policy::automatic_reference) {
      throw pybind11::cast_error(
          "dartpy interop returns raw DART pointers and references only with "
          "return_value_policy::reference or reference_internal; return a "
          "std::shared_ptr to transfer ownership");
    }
    auto* object = const_cast<T*>(pointer);
    return checked(api().wrap(
        typeid(T),
        typeid(*object),
        dynamic_cast<void*>(object),
        object,
        policy == Policy::reference_internal ? c_api::kReferenceInternal
                                             : c_api::kReference,
        parent.ptr()));
  }

  static pybind11::handle cast(
      const T& value,
      pybind11::return_value_policy policy,
      pybind11::handle parent)
  {
    return cast(&value, policy, parent);
  }

  // A temporary DART object would leave its wrapper dangling.
  static pybind11::handle cast(
      T&& value, pybind11::return_value_policy policy, pybind11::handle parent)
      = delete;

private:
  T* mValue = nullptr;
};

/// Caster for std::shared_ptr to a polymorphic DART class.
template <class T>
class SharedCaster
{
  static_assert(std::is_polymorphic_v<T>);

public:
  using Value = std::shared_ptr<T>;
  PYBIND11_TYPE_CASTER(Value, PythonName<std::remove_const_t<T>>::value);

  bool load(pybind11::handle src, bool)
  {
    if (src.is_none()) {
      value.reset();
      return true;
    }
    auto* pointer = static_cast<std::remove_const_t<T>*>(
        api().unwrap(src.ptr(), typeid(T)));
    if (!pointer)
      return rejectArgument();
    std::shared_ptr<void> owner;
    if (api().share(src.ptr(), dynamic_cast<void*>(pointer), &owner) != 0)
      return rejectArgument();
    value = Value(std::move(owner), pointer);
    return true;
  }

  static pybind11::handle cast(
      const Value& value, pybind11::return_value_policy, pybind11::handle)
  {
    if (!value)
      return pybind11::none().release();
    auto* object = const_cast<std::remove_const_t<T>*>(value.get());
    const std::shared_ptr<void> owner(value, object);
    return checked(api().wrapShared(
        typeid(T),
        typeid(*object),
        dynamic_cast<void*>(object),
        object,
        &owner));
  }
};

/// Caster for a value type that dartpy binds as a class, such as Isometry3d.
template <class T>
class ValueCaster
{
public:
  PYBIND11_TYPE_CASTER(T, PythonName<T>::value);

  bool load(pybind11::handle src, bool convert)
  {
    const int result = api().loadValue(src.ptr(), typeid(T), &value, convert);
    if (result < 0)
      throw pybind11::error_already_set();
    return result == 0;
  }

  static pybind11::handle cast(
      const T& value, pybind11::return_value_policy, pybind11::handle)
  {
    return checked(api().castValue(typeid(T), &value));
  }
};

} // namespace pybind11_interop
} // namespace dartpy

#define DARTPY_PYBIND11_INTEROP_NAME(Type, Name)                               \
  namespace dartpy::pybind11_interop {                                         \
  template <>                                                                  \
  struct PythonName<Type>                                                      \
  {                                                                            \
    static constexpr auto value = ::pybind11::detail::const_name(Name);        \
  };                                                                           \
  }

/// Exchanges `Type*`, `Type&`, and `std::shared_ptr<Type>` (also to const)
/// with dartpy.
#define DARTPY_PYBIND11_INTEROP_OBJECT(Type, Name)                             \
  DARTPY_PYBIND11_INTEROP_NAME(Type, Name)                                     \
  namespace pybind11::detail {                                                 \
  template <>                                                                  \
  class type_caster<Type>                                                      \
    : public ::dartpy::pybind11_interop::ObjectCaster<Type>                    \
  {                                                                            \
  };                                                                           \
  template <>                                                                  \
  class type_caster<std::shared_ptr<Type>>                                     \
    : public ::dartpy::pybind11_interop::SharedCaster<Type>                    \
  {                                                                            \
  };                                                                           \
  template <>                                                                  \
  class type_caster<std::shared_ptr<const Type>>                               \
    : public ::dartpy::pybind11_interop::SharedCaster<const Type>              \
  {                                                                            \
  };                                                                           \
  }

/// Exchanges copies of `Type`, a value class that dartpy binds, with dartpy.
#define DARTPY_PYBIND11_INTEROP_VALUE(Type, Name)                              \
  DARTPY_PYBIND11_INTEROP_NAME(Type, Name)                                     \
  namespace pybind11::detail {                                                 \
  template <>                                                                  \
  class type_caster<Type>                                                      \
    : public ::dartpy::pybind11_interop::ValueCaster<Type>                     \
  {                                                                            \
  };                                                                           \
  }

DARTPY_PYBIND11_INTEROP_OBJECT(dart::dynamics::Entity, "dartpy.dynamics.Entity")
DARTPY_PYBIND11_INTEROP_OBJECT(dart::dynamics::Frame, "dartpy.dynamics.Frame")
DARTPY_PYBIND11_INTEROP_OBJECT(
    dart::dynamics::SimpleFrame, "dartpy.dynamics.SimpleFrame")
DARTPY_PYBIND11_INTEROP_OBJECT(
    dart::dynamics::JacobianNode, "dartpy.dynamics.JacobianNode")
DARTPY_PYBIND11_INTEROP_OBJECT(
    dart::dynamics::BodyNode, "dartpy.dynamics.BodyNode")
DARTPY_PYBIND11_INTEROP_OBJECT(
    dart::dynamics::ShapeNode, "dartpy.dynamics.ShapeNode")
DARTPY_PYBIND11_INTEROP_OBJECT(dart::dynamics::Joint, "dartpy.dynamics.Joint")
DARTPY_PYBIND11_INTEROP_OBJECT(
    dart::dynamics::DegreeOfFreedom, "dartpy.dynamics.DegreeOfFreedom")
DARTPY_PYBIND11_INTEROP_OBJECT(
    dart::dynamics::MetaSkeleton, "dartpy.dynamics.MetaSkeleton")
DARTPY_PYBIND11_INTEROP_OBJECT(
    dart::dynamics::Skeleton, "dartpy.dynamics.Skeleton")
DARTPY_PYBIND11_INTEROP_OBJECT(
    dart::simulation::World, "dartpy.simulation.World")
DARTPY_PYBIND11_INTEROP_VALUE(Eigen::Isometry3d, "dartpy.math.Isometry3")

#endif // DARTPY_PYBIND11_INTEROP_HPP_
