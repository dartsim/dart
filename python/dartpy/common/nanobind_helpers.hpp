#pragma once

#include <nanobind/nanobind.h>

#include <typeinfo>

#include <cstdint>

namespace dart::python_nb {

// Keep nanobind's domain-aware internal calls isolated from DART's custom
// casters and lifetime helpers.
inline const std::type_info* nanobindTypeInfo(nanobind::handle type) noexcept
{
  if (!nanobind::type_check(type)) {
    return nullptr;
  }

  return &nanobind::type_info(type);
}

inline bool nanobindTypeGet(
    const std::type_info* type,
    PyObject* object,
    std::uint32_t flags,
    nanobind::detail::cleanup_list* cleanup,
    void** output) noexcept
{
  return NB_CALL(nb_type_get)(
      NB_CTX_C(cleanup), type, object, flags, cleanup, output);
}

inline nanobind::handle nanobindTypePut(
    const std::type_info* type,
    const std::type_info* dynamicType,
    void* value,
    nanobind::rv_policy policy,
    nanobind::detail::cleanup_list* cleanup) noexcept
{
  return nanobind::handle(NB_CALL(nb_type_put)(
      NB_CTX_C(cleanup), type, dynamicType, value, policy, cleanup, nullptr));
}

inline void nanobindKeepAlive(PyObject* nurse, PyObject* patient) noexcept
{
  NB_CALL(keep_alive_py)(NB_CTX, nurse, patient);
}

inline void nanobindKeepAlive(
    PyObject* nurse, void* payload, void (*deleter)(void*) noexcept) noexcept
{
  NB_CALL(keep_alive_ptr)(NB_CTX, nurse, payload, deleter);
}

} // namespace dart::python_nb
