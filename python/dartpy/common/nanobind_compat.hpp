#pragma once

#include <nanobind/nanobind.h>

#include <typeinfo>

#include <cstdint>

namespace dart::python_nb {

// nanobind 3 widened caster flags and routes core calls through a domain
// context. Keep that version boundary isolated from DART's custom casters.
#if NB_VERSION_MAJOR >= 3
using NanobindCastFlags = std::uint32_t;
#else
using NanobindCastFlags = std::uint8_t;
#endif

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
    NanobindCastFlags flags,
    nanobind::detail::cleanup_list* cleanup,
    void** output) noexcept
{
#if NB_VERSION_MAJOR >= 3
  return NB_CALL(nb_type_get)(
      NB_CTX_C(cleanup), type, object, flags, cleanup, output);
#else
  return nanobind::detail::nb_type_get(type, object, flags, cleanup, output);
#endif
}

inline nanobind::handle nanobindTypePut(
    const std::type_info* type,
    const std::type_info* dynamicType,
    void* value,
    nanobind::rv_policy policy,
    nanobind::detail::cleanup_list* cleanup) noexcept
{
#if NB_VERSION_MAJOR >= 3
  return nanobind::handle(NB_CALL(nb_type_put)(
      NB_CTX_C(cleanup), type, dynamicType, value, policy, cleanup, nullptr));
#else
  return nanobind::handle(
      nanobind::detail::nb_type_put_p(
          type, dynamicType, value, policy, cleanup, nullptr));
#endif
}

inline void nanobindKeepAlive(PyObject* nurse, PyObject* patient) noexcept
{
#if NB_VERSION_MAJOR >= 3
  NB_CALL(keep_alive_py)(NB_CTX, nurse, patient);
#else
  nanobind::detail::keep_alive(nurse, patient);
#endif
}

inline void nanobindKeepAlive(
    PyObject* nurse, void* payload, void (*deleter)(void*) noexcept) noexcept
{
#if NB_VERSION_MAJOR >= 3
  NB_CALL(keep_alive_ptr)(NB_CTX, nurse, payload, deleter);
#else
  nanobind::detail::keep_alive(nurse, payload, deleter);
#endif
}

} // namespace dart::python_nb

// nanobind 3 derives the trampoline override-cache size automatically.
#if NB_VERSION_MAJOR >= 3
  #define DARTPY_NB_TRAMPOLINE(base, size) NB_TRAMPOLINE(base)
#else
  #define DARTPY_NB_TRAMPOLINE(base, size) NB_TRAMPOLINE(base, size)
#endif
