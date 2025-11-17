#pragma once

#include "common/polymorphic_caster.hpp"

#include <nanobind/nanobind.h>

#include <cstdio>
#include <cstdlib>

#include "dart/dynamics/Frame.hpp"
#include "dart/dynamics/Joint.hpp"
#include "dart/dynamics/ShapeFrame.hpp"

namespace nanobind::detail {

template <typename Base>
struct polymorphic_type_caster : type_caster_base_tag {
  using Type = Base;
  static constexpr auto Name = const_name<Type>();
  template <typename T>
  using Cast = precise_cast_t<T>;

  NB_INLINE bool from_python(
      handle src, uint8_t flags, cleanup_list* cleanup) noexcept
  {
    PyObject* obj = src.ptr();
    const std::type_info* info = nanobind::detail::nb_type_info(
        reinterpret_cast<PyObject*>(Py_TYPE(obj)));
    if (const char* trace = std::getenv("DARTPY_NB_TRACE_POLY")) {
      (void) trace;
      std::fprintf(
          stderr,
          "[dartpy_nb][poly] base=%s actual=%s has=%d\n",
          typeid(Type).name(),
          info ? info->name() : "(null)",
          (info != nullptr
           && dart::python_nb::hasPolymorphicCaster<Type>(*info)));
    }

    if (info != nullptr
        && dart::python_nb::hasPolymorphicCaster<Type>(*info)) {
      if (!nb_type_get(info, obj, flags, cleanup,
                       reinterpret_cast<void**>(&raw_)))
        return false;
      value_ = dart::python_nb::convertPolymorphicPointer<Type>(
          static_cast<void*>(raw_), *info);
      if (std::getenv("DARTPY_NB_TRACE_POLY")) {
        std::fprintf(
            stderr,
            "[dartpy_nb][poly] raw=%p adjusted=%p\n",
            static_cast<void*>(raw_),
            static_cast<void*>(value_));
      }
    } else {
      if (!nb_type_get(&typeid(Type), obj, flags, cleanup,
                       reinterpret_cast<void**>(&raw_)))
        return false;
      value_ = dart::python_nb::adjustPolymorphicPointer<Type>(obj, raw_);
      if (std::getenv("DARTPY_NB_TRACE_POLY")) {
        std::fprintf(
            stderr,
            "[dartpy_nb][poly][fallback] raw=%p adjusted=%p\n",
            static_cast<void*>(raw_),
            static_cast<void*>(value_));
      }
    }

    return value_ != nullptr;
  }

  template <typename T>
  NB_INLINE static handle from_cpp(
      T&& value, rv_policy policy, cleanup_list* cleanup) noexcept
  {
    return type_caster_base<Type>::from_cpp(
        std::forward<T>(value), policy, cleanup);
  }

  template <typename T_>
  bool can_cast() const noexcept
  {
    return std::is_pointer_v<T_> || value_ != nullptr;
  }

  operator Type*() { return value_; }

  operator Type&()
  {
    raise_next_overload_if_null(value_);
    return *value_;
  }

  operator Type&&()
  {
    raise_next_overload_if_null(value_);
    return static_cast<Type&&>(*value_);
  }

private:
  Type* raw_ = nullptr;
  Type* value_ = nullptr;
};

template <>
struct type_caster<dart::dynamics::Frame>
    : polymorphic_type_caster<dart::dynamics::Frame>
{
};

template <>
struct type_caster<dart::dynamics::ShapeFrame>
    : polymorphic_type_caster<dart::dynamics::ShapeFrame>
{
};

} // namespace nanobind::detail
