#pragma once

#include "common/polymorphic_caster.hpp"

#include <nanobind/nanobind.h>

namespace dart::python_nb {

template <typename Base>
inline Base* castHandleTo(nanobind::handle h)
{
  return nanobind::cast<Base*>(h);
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
