#pragma once

#include "dart_nb.hpp"

#include <dart/common/Composite.hpp>

#include <typeinfo>

#include <cstring>

namespace dartnb {
// Finds an aspect's entry in a composite state by type name. dartpy emits its
// own hidden typeinfo for DART's aspect templates, and Apple arm64 compares
// hidden typeinfo by address, so a typeid lookup misses libdart's keys.
template <class AspectT>
const typename dart::common::detail::GetState<AspectT>::Type* find_state(
    const dart::common::Composite::State& state)
{
  using Aspect = typename dart::common::detail::GetAspect<AspectT>::Type;
  using Data = typename dart::common::detail::GetState<AspectT>::Type;
  for (const auto& [type, data] : state.getMap())
    if (std::strcmp(type.name(), typeid(Aspect).name()) == 0)
      return static_cast<const Data*>(data.get());
  return nullptr;
}
} // namespace dartnb
