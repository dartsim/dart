// clang-format off
#include "detail/dart_nb.hpp"
// clang-format on

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

#include <dart/dynamics/Skeleton.hpp>

#include <dartpy/c_api.hpp>

#include <stdexcept>
#include <string>
#include <typeindex>
#include <unordered_map>

namespace dart {
namespace python {
namespace {

namespace c_api = ::dartpy::c_api;

// Keep C++ exceptions inside dartpy: report them as Python exceptions.
void setPythonError()
{
  try {
    throw;
  } catch (nb::python_error& error) {
    error.restore();
  } catch (const nb::builtin_exception& error) {
    // nanobind maps these to TypeError, ValueError, and so on.
    PyErr_SetString(
        error.type() == nb::exception_type::type_error ? PyExc_TypeError
                                                       : PyExc_RuntimeError,
        error.what());
  } catch (const std::exception& error) {
    PyErr_SetString(PyExc_RuntimeError, error.what());
  } catch (...) {
    PyErr_SetString(PyExc_RuntimeError, "dartpy C API: unknown C++ exception");
  }
}

[[noreturn]] void throwTypeError(nb::handle object, const std::type_info& type)
{
  throw nb::type_error(("dartpy C API: cannot convert "
                        + std::string(nb::type_name(object.type()).c_str())
                        + " to C++ " + type.name())
                           .c_str());
}

// Returns the registered C++ type and pointer of a ready dartpy instance.
std::pair<const std::type_info*, void*> instance(nb::handle object)
{
  if (!nb::inst_check(object) || !nb::inst_ready(object)) {
    throw nb::type_error(("dartpy C API: expected a dartpy object, got "
                          + std::string(nb::type_name(object.type()).c_str()))
                             .c_str());
  }
  return {&nb::type_info(object.type()), nb::inst_ptr<void>(object)};
}

struct ValueOps
{
  bool (*load)(nb::handle object, void* out, bool convert);
  nb::object (*cast)(const void* value);
};

template <class T>
ValueOps valueOps()
{
  return {
      [](nb::handle object, void* out, bool convert) {
        return nb::try_cast(object, *static_cast<T*>(out), convert);
      },
      [](const void* value) {
        return nb::cast(*static_cast<const T*>(value), nb::rv_policy::copy);
      }};
}

const ValueOps& valueOps(const std::type_info& type)
{
  static const std::unordered_map<std::type_index, ValueOps> table{
      {typeid(Eigen::Isometry3d), valueOps<Eigen::Isometry3d>()},
      {typeid(Eigen::Quaterniond), valueOps<Eigen::Quaterniond>()},
      {typeid(Eigen::AngleAxisd), valueOps<Eigen::AngleAxisd>()},
  };
  const auto found = table.find(type);
  if (found == table.end()) {
    throw std::invalid_argument(
        std::string("dartpy C API: unsupported value type ") + type.name());
  }
  return found->second;
}

void* unwrap(PyObject* object, const std::type_info& type)
{
  try {
    const nb::handle handle(object);
    const auto [exact, pointer] = instance(handle);
    if (*exact == type)
      return pointer;
    if (void* adjusted = dartnb::upcast(*exact, type, pointer))
      return adjusted;
    throwTypeError(handle, type);
  } catch (...) {
    setPythonError();
  }
  return nullptr;
}

int share(PyObject* object, void* complete, std::shared_ptr<void>* owner)
{
  try {
    const nb::handle handle(object);
    const auto [exact, pointer] = instance(handle);
    // Mirror dartpy's own shared_ptr caster: native owners first.
    if (auto* skeleton = static_cast<dart::dynamics::Skeleton*>(dartnb::upcast(
            *exact, typeid(dart::dynamics::Skeleton), pointer))) {
      if ((*owner = skeleton->getPtr()))
        return 0;
    }
    if ((*owner = dartnb::native_owner(complete)))
      return 0;
    if (!nb::inst_state(handle).second) {
      throw nb::type_error(
          "dartpy C API: a wrapper that borrows its object cannot share "
          "ownership of it");
    }
    handle.inc_ref();
    *owner = std::shared_ptr<void>(complete, dartnb::PythonPin{object});
    return 0;
  } catch (...) {
    setPythonError();
  }
  return -1;
}

PyObject* wrap(
    const std::type_info& type,
    const std::type_info& dynamicType,
    void* complete,
    void* pointer,
    int policy,
    PyObject* parent)
{
  try {
    if (policy != c_api::kReference && policy != c_api::kReferenceInternal)
      throw std::invalid_argument("dartpy C API: unknown wrap policy");
    if (policy == c_api::kReferenceInternal && (!parent || parent == Py_None))
      throw std::invalid_argument(
          "dartpy C API: reference_internal needs a parent");
    nb::rv_policy rvPolicy = nb::rv_policy::reference;
    if (policy == c_api::kReferenceInternal)
      rvPolicy = nb::rv_policy::reference_internal;
    return dartnb::wrap(
               type,
               dynamicType,
               complete,
               pointer,
               rvPolicy,
               nb::handle(parent))
        .ptr();
  } catch (...) {
    setPythonError();
  }
  return nullptr;
}

PyObject* wrapShared(
    const std::type_info& type,
    const std::type_info& dynamicType,
    void* complete,
    void* pointer,
    const std::shared_ptr<void>* owner)
{
  try {
    bool isNew = false;
    auto result = nb::steal(dartnb::wrap(
        type,
        dynamicType,
        complete,
        pointer,
        nb::rv_policy::reference,
        {},
        &isNew));
    if (isNew)
      dartnb::hold_native_owner(result, *owner, complete);
    return result.release().ptr();
  } catch (...) {
    setPythonError();
  }
  return nullptr;
}

int loadValue(
    PyObject* object, const std::type_info& type, void* out, int convert)
{
  try {
    return valueOps(type).load(nb::handle(object), out, convert != 0) ? 0 : 1;
  } catch (...) {
    setPythonError();
  }
  return -1;
}

PyObject* castValue(const std::type_info& type, const void* value)
{
  try {
    return valueOps(type).cast(value).release().ptr();
  } catch (...) {
    setPythonError();
  }
  return nullptr;
}

const c_api::CApi capi{
    c_api::kVersion,
    sizeof(c_api::CApi),
    c_api::kAbiTag,
    unwrap,
    share,
    wrap,
    wrapShared,
    loadValue,
    castValue};

} // namespace

void dart_c_api(nb::module_& m)
{
  m.attr("_C_API") = nb::capsule(&capi, c_api::kCapsuleName);
}

} // namespace python
} // namespace dart
