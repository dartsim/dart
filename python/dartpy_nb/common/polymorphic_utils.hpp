#pragma once

#include "common/polymorphic_caster.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/nb_class.h>

namespace dart::python_nb {

template <typename Base>
inline Base* castHandleTo(nb::handle h)
{
  PyObject* obj = h.ptr();
  if (!obj)
    return nullptr;

  PyTypeObject* type = Py_TYPE(obj);
  const std::type_info* info
      = nanobind::detail::nb_type_info(reinterpret_cast<PyObject*>(type));
  void* raw = nanobind::detail::nb_inst_ptr(obj);

  if (info && hasPolymorphicCaster<Base>(*info))
    return convertPolymorphicPointer<Base>(raw, *info);

  return static_cast<Base*>(raw);
}

template <typename Base>
inline Base& refFromHandle(nb::handle h)
{
  Base* ptr = castHandleTo<Base>(h);
  if (!ptr)
    throw nanobind::value_error("Unable to extract native object");
  return *ptr;
}

} // namespace dart::python_nb
