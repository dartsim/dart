#pragma once

#include "common/polymorphic_caster.hpp"

#include <nanobind/nanobind.h>

#include "dart/dynamics/frame.hpp"
#include "dart/dynamics/jacobian_node.hpp"
#include "dart/dynamics/joint.hpp"
#include "dart/dynamics/meta_skeleton.hpp"
#include "dart/dynamics/shape_frame.hpp"

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
    if (src.is_none()) {
      raw_ = nullptr;
      value_ = nullptr;
      return true;
    }

    PyObject* obj = src.ptr();
    const std::type_info* info = nanobind::detail::nb_type_info(
        reinterpret_cast<PyObject*>(Py_TYPE(obj)));

    if (info != nullptr
        && dart::python_nb::hasPolymorphicCaster<Type>(*info)) {
      if (!nb_type_get(info, obj, flags, cleanup,
                       reinterpret_cast<void**>(&raw_)))
        return false;
      value_ = dart::python_nb::convertPolymorphicPointer<Type>(
          static_cast<void*>(raw_), *info);
    } else {
      if (!nb_type_get(&typeid(Type), obj, flags, cleanup,
                       reinterpret_cast<void**>(&raw_)))
        return false;
      value_ = dart::python_nb::adjustPolymorphicPointer<Type>(obj, raw_);
    }

    return value_ != nullptr;
  }

  template <typename T>
  NB_INLINE static handle from_cpp(
      T&& value, rv_policy policy, cleanup_list* cleanup) noexcept
  {
    using BareT = std::remove_reference_t<std::remove_cv_t<T>>;

    if constexpr (std::is_pointer_v<BareT>) {
      Type* ptr = (Type*) value;
      policy = infer_policy<T>(policy);
      const std::type_info* actual_type = ptr ? &typeid(*ptr) : nullptr;
      void* adjusted = static_cast<void*>(ptr);
      if (ptr != nullptr && actual_type != nullptr) {
        if (dart::python_nb::hasPolymorphicCaster<Type>(*actual_type)) {
          adjusted = dart::python_nb::restorePolymorphicPointer<Type>(
              ptr, *actual_type);
          if (adjusted == nullptr) {
            actual_type = &typeid(Type);
            adjusted = static_cast<void*>(ptr);
          }
        } else {
          if constexpr (std::is_polymorphic_v<Type>)
            adjusted = dynamic_cast<void*>(ptr);
        }
      }
      return nb_type_put_p(
          &typeid(Type), actual_type, adjusted, policy, cleanup, nullptr);
    } else if constexpr (std::is_lvalue_reference_v<T>) {
      return from_cpp(&value, policy, cleanup);
    } else {
      return type_caster_base<Type>::from_cpp(
          std::forward<T>(value), policy, cleanup);
    }
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

template <>
struct type_caster<dart::dynamics::MetaSkeleton>
    : polymorphic_type_caster<dart::dynamics::MetaSkeleton>
{
};

template <>
struct type_caster<dart::dynamics::BodyNode>
    : polymorphic_type_caster<dart::dynamics::BodyNode>
{
  template <typename T>
  NB_INLINE static handle from_cpp(
      T&& value, rv_policy policy, cleanup_list* cleanup) noexcept
  {
    using BareT = std::remove_cv_t<std::remove_reference_t<T>>;
    if constexpr (std::is_pointer_v<BareT>
                  || std::is_lvalue_reference_v<T>) {
      if (policy != rv_policy::reference_internal)
        policy = rv_policy::reference;
    }
    return polymorphic_type_caster<dart::dynamics::BodyNode>::from_cpp(
        std::forward<T>(value), policy, cleanup);
  }
};

template <>
struct type_caster<dart::dynamics::JacobianNode>
    : polymorphic_type_caster<dart::dynamics::JacobianNode>
{
  template <typename T>
  NB_INLINE static handle from_cpp(
      T&& value, rv_policy policy, cleanup_list* cleanup) noexcept
  {
    using BareT = std::remove_cv_t<std::remove_reference_t<T>>;
    if constexpr (std::is_pointer_v<BareT>
                  || std::is_lvalue_reference_v<T>) {
      if (policy != rv_policy::reference_internal)
        policy = rv_policy::reference;
    }
    return polymorphic_type_caster<dart::dynamics::JacobianNode>::from_cpp(
        std::forward<T>(value), policy, cleanup);
  }
};

template <>
struct type_caster<dart::dynamics::Joint>
    : polymorphic_type_caster<dart::dynamics::Joint>
{
  template <typename T>
  NB_INLINE static handle from_cpp(
      T&& value, rv_policy policy, cleanup_list* cleanup) noexcept
  {
    using BareT = std::remove_cv_t<std::remove_reference_t<T>>;
    if constexpr (std::is_pointer_v<BareT>
                  || std::is_lvalue_reference_v<T>) {
      if (policy != rv_policy::reference_internal)
        policy = rv_policy::reference;
    }
    return polymorphic_type_caster<dart::dynamics::Joint>::from_cpp(
        std::forward<T>(value), policy, cleanup);
  }
};

template <typename JointT>
struct type_caster<
    JointT,
    std::enable_if_t<
        std::is_base_of_v<dart::dynamics::Joint, JointT>
        && !std::is_same_v<dart::dynamics::Joint, JointT>>>
    : polymorphic_type_caster<JointT>
{
  template <typename T>
  NB_INLINE static handle from_cpp(
      T&& value, rv_policy policy, cleanup_list* cleanup) noexcept
  {
    if (policy != rv_policy::reference_internal)
      policy = rv_policy::reference;
    return polymorphic_type_caster<JointT>::from_cpp(
        std::forward<T>(value), policy, cleanup);
  }
};

} // namespace nanobind::detail
