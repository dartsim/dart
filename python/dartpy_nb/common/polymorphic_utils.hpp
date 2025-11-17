#pragma once

#include "common/polymorphic_caster.hpp"

#include <nanobind/nanobind.h>

namespace dart::python_nb {

template <typename Base>
inline Base* castHandleTo(nanobind::handle h)
{
  PyObject* obj = h.ptr();
  if (!obj)
    return nullptr;

  PyTypeObject* type = Py_TYPE(obj);
  const std::type_info* info
      = nanobind::detail::nb_type_info(reinterpret_cast<PyObject*>(type));
  void* raw = nanobind::detail::nb_inst_ptr(obj);
  if (const char* trace = std::getenv("DARTPY_NB_TRACE_POLY_HANDLE")) {
    std::fprintf(
        stderr,
        "[dartpy_nb][handle] base=%s actual=%s raw=%p has=%d\n",
        typeid(Base).name(),
        info ? info->name() : "(null)",
        raw,
        info ? hasPolymorphicCaster<Base>(*info) : 0);
  }

  if (info && hasPolymorphicCaster<Base>(*info)) {
    Base* adjusted = convertPolymorphicPointer<Base>(raw, *info);
    if (std::getenv("DARTPY_NB_TRACE_POLY_HANDLE")) {
      std::fprintf(stderr, "[dartpy_nb][handle] adjusted=%p\n", adjusted);
    }
    return adjusted;
  }

  return static_cast<Base*>(raw);
}

template <typename Base>
inline Base& refFromHandle(nanobind::handle h)
{
  Base* ptr = castHandleTo<Base>(h);
  if (!ptr)
    throw nanobind::value_error("Unable to extract native object");
  return *ptr;
}

} // namespace dart::python_nb
