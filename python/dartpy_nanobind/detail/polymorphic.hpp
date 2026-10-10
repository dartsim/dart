#pragma once

#include <functional>
#include <typeindex>
#include <unordered_map>
#include <vector>

namespace dartnb {
namespace nb = nanobind;
using Key = std::type_index;
// Native deleted operations make nanobind's standard traits sufficient.
template <class T>
inline constexpr bool reference_only
    = !std::is_copy_constructible_v<T> && !std::is_move_constructible_v<T>;

template <class T>
inline constexpr bool value_base = false;

template <class T>
void* complete_address(T* pointer)
{
  using Mutable = std::remove_const_t<T>;
  if constexpr (std::is_polymorphic_v<T>)
    return dynamic_cast<void*>(const_cast<Mutable*>(pointer));
  else
    return const_cast<Mutable*>(pointer);
}
using Adjust = void* (*)(void*);
struct BaseEdge
{
  Key base;
  Adjust up;
  Adjust down;
};
struct Entry
{
  nb::handle python_type;
  std::vector<BaseEdge> bases;
  bool graph_owned;
  void (*rebind_methods)(nb::handle) = nullptr;
};
void register_type(Key key, Entry entry);
void register_methods(Key key, void (*methods)(nb::handle));
void rebind_methods(Key key, nb::handle cls);
std::shared_ptr<void> native_owner(void* complete);
void remember_owner(
    void* complete, const std::shared_ptr<void>& owner, const void* token);
void forget_owner(void* complete, const void* token) noexcept;
void remember_wrapper(Key exact, void* complete, nb::handle wrapper);
void* upcast(Key source, Key target, void* pointer);
nb::handle wrap(
    Key source,
    const std::type_info& dynamic_type,
    void* complete,
    void* pointer,
    nb::rv_policy policy,
    nb::handle parent,
    bool* is_new = nullptr);

void hold_body(nb::handle wrapper, Key type, void* pointer);

// `complete` is the most-derived address of the object that `owner` owns.
void hold_native_owner(
    nb::handle wrapper, std::shared_ptr<void> owner, void* complete);

// Keep a shared_ptr result's owner on its wrapper. A reused wrapper also takes
// it when it neither owns its object nor has a native owner, so transferring
// the last owner to Python cannot free a borrowed wrapper's object.
// Python-owned wrappers already keep their object; their owner would be a
// pin on themselves.
void retain_shared_owner(
    nb::handle wrapper,
    bool is_new,
    std::shared_ptr<void> owner,
    void* complete);

template <class T>
void hold_native_owner(nb::handle wrapper, const std::shared_ptr<T>& owner)
{
  using Mutable = std::remove_const_t<T>;
  hold_native_owner(
      wrapper,
      std::const_pointer_cast<Mutable>(owner),
      complete_address(owner.get()));
}

template <class T, class Base>
BaseEdge base_edge()
{
  return {
      typeid(Base),
      [](void* p) -> void* { return static_cast<Base*>(static_cast<T*>(p)); },
      [](void* p) -> void* {
        if constexpr (std::is_polymorphic_v<Base>)
          return dynamic_cast<T*>(static_cast<Base*>(p));
        else
          return nullptr;
      }};
}

// Python has one native base; retain every C++ edge for adjusted conversion.
template <class T, class... Candidates>
struct ClassTraits;
template <class T>
struct ClassTraits<T>
{
  using Base = T;
  using Alias = T;
};
template <class T, class First, class... Rest>
struct ClassTraits<T, First, Rest...>
{
  using Tail = ClassTraits<T, Rest...>;
  using Base = std::
      conditional_t<std::is_base_of_v<First, T>, First, typename Tail::Base>;
  using Alias = std::
      conditional_t<std::is_base_of_v<T, First>, First, typename Tail::Alias>;
};

template <
    class T,
    class Base,
    class Alias,
    bool HasBase = !std::is_same_v<T, Base>,
    bool HasAlias = !std::is_same_v<T, Alias>>
struct NativeClass
{
  using Type = nb::class_<T, Base, Alias>;
};
template <class T, class Base, class Alias>
struct NativeClass<T, Base, Alias, false, false>
{
  using Type = nb::class_<T>;
};
template <class T, class Base, class Alias>
struct NativeClass<T, Base, Alias, true, false>
{
  using Type = nb::class_<T, Base>;
};
template <class T, class Base, class Alias>
struct NativeClass<T, Base, Alias, false, true>
{
  using Type = nb::class_<T, Alias>;
};

template <class T, class... Bases>
struct Primary
{
  using Traits = ClassTraits<T, Bases...>;
  using Type =
      typename NativeClass<T, typename Traits::Base, typename Traits::Alias>::
          Type;
};

template <class Cls>
void defSecondaryMethods(Cls& cls, nb::handle base)
{
  // Snapshot primary names so all overloads resolve as in the oracle's MRO.
  auto names = nb::module_::import_("builtins").attr("dir")(base);
  for (nb::handle name : names) {
    auto key = nb::cast<std::string>(name);
    if (key.rfind("_", 0) == 0 || nb::hasattr(cls, key.c_str()))
      continue;
    cls.attr(key.c_str()) = base.attr(key.c_str());
  }
}

template <class T>
struct polymorphic_caster;

template <class T, class... Bases>
class dart_class : public Primary<T, Bases...>::Type
{
  using Native = typename Primary<T, Bases...>::Type;
  std::set<std::string> rebound_names;
  void override_rebound(const char* name)
  {
    if (rebound_names.erase(name))
      nb::delattr(*this, name);
  }
  template <class Base>
  static void add_edge(std::vector<BaseEdge>& edges)
  {
    if constexpr (std::is_base_of_v<Base, T> && !std::is_same_v<Base, T>)
      edges.push_back(base_edge<T, Base>());
  }

public:
  template <class... Extra>
  dart_class(nb::handle scope, const char* name, const Extra&... extra)
    : Native(
        scope,
        name,
        nb::is_weak_referenceable(),
        nb::type_slots(gc_slots<T>()),
        extra...)
  {
    static_assert(
        (!std::is_polymorphic_v<T> && !value_base<T>)
            || std::
                is_base_of_v<polymorphic_caster<T>, nb::detail::make_caster<T>>,
        "DART caster selection failed: check the specialization SFINAE "
        "parameter");
    if constexpr (
        std::is_polymorphic_v<
            T> || value_base<T> || (value_base<Bases> || ...)) {
      constexpr bool graph
          = std::is_base_of_v<
                dart::dynamics::Entity,
                T> || std::is_base_of_v<dart::dynamics::Node, T> || std::is_base_of_v<dart::dynamics::Joint, T> || std::is_same_v<dart::dynamics::DegreeOfFreedom, T>;
      std::vector<BaseEdge> edges;
      (add_edge<Bases>(edges), ...);
      register_type(typeid(T), {*this, std::move(edges), graph});
    }
    (
        [&] {
          if constexpr (
              std::is_base_of_v<
                  Bases,
                  T> && !std::is_same_v<Bases, typename Primary<T, Bases...>::Traits::Base>) {
            rebind_methods(typeid(Bases), *this);
            defSecondaryMethods(*this, nb::type<Bases>());
          }
        }(),
        ...);
    for (nb::handle name : this->attr("__dict__")) {
      auto key = nb::cast<std::string>(name);
      if (key.rfind("_", 0) != 0)
        rebound_names.insert(key);
    }
  }
  template <class... Args>
  dart_class& def(const char* name, Args&&... args)
  {
    override_rebound(name);
    Native::def(name, std::forward<Args>(args)...);
    return *this;
  }
  template <
      class Visitor,
      class... Args,
      std::enable_if_t<
          !std::is_convertible_v<const Visitor&, const char*>,
          int> = 0>
  dart_class& def(Visitor visitor, const Args&... args)
  {
    Native::def(std::move(visitor), args...);
    return *this;
  }
  template <class... Args>
  dart_class& def_static(const char* name, Args&&... args)
  {
    override_rebound(name);
    Native::def_static(name, std::forward<Args>(args)...);
    return *this;
  }
};

template <class T>
struct polymorphic_caster : nb::detail::type_caster_base<T>
{
  using Base = nb::detail::type_caster_base<T>;
  using Base::Name;
  template <class U>
  using Cast = typename Base::template Cast<U>;
  T* value = nullptr;

  bool from_python(
      nb::handle src, uint32_t flags, nb::detail::cleanup_list*) noexcept
  {
    if (src.is_none())
      return !(flags & nb::detail::cast_flags::none_disallowed);
    if (!nb::inst_check(src))
      return false;
    const bool constructing = flags & nb::detail::cast_flags::construct;
    if (constructing == nb::inst_ready(src))
      return false;
    // No virtual-base adjustment is valid before the C++ constructor runs.
    const auto& exact = nb::type_info(src.type());
    if (constructing && exact != typeid(T))
      return false;
    void* pointer = nb::inst_ptr<void>(src);
    value = static_cast<T*>(
        exact == typeid(T) ? pointer : upcast(exact, typeid(T), pointer));
    return value != nullptr;
  }
  template <class U>
  static nb::handle from_cpp(
      U&& input,
      nb::rv_policy policy,
      nb::detail::cleanup_list* cleanup) noexcept
  {
    static_assert(
        !reference_only<
            T> || std::is_pointer_v<std::decay_t<U>> || std::is_lvalue_reference_v<U>,
        "DART reference-only type cannot be returned by value");
    if constexpr (reference_only<T>) {
      if (policy == nb::rv_policy::copy || policy == nb::rv_policy::move
          || (!std::is_pointer_v<
                  std::decay_t<U>> && policy != nb::rv_policy::reference
              && policy != nb::rv_policy::reference_internal)) {
        PyErr_SetString(
            PyExc_TypeError,
            "DART reference-only type cannot be copied or moved");
        return {};
      }
    }
    if constexpr (!std::is_pointer_v<std::decay_t<U>>) {
      if (policy != nb::rv_policy::reference
          && policy != nb::rv_policy::reference_internal) {
        auto result = Base::from_cpp(std::forward<U>(input), policy, cleanup);
        if (result.is_valid() && !result.is_none()) {
          auto* native = nb::inst_ptr<T>(result);
          remember_wrapper(typeid(T), complete_address(native), result);
        }
        return result;
      }
    }
    const T* p;
    if constexpr (std::is_pointer_v<std::decay_t<U>>)
      p = input;
    else
      p = &input;
    if (!p)
      return nb::none().release();
    if constexpr (!std::is_polymorphic_v<T>) {
      return Base::from_cpp(std::forward<U>(input), policy, cleanup);
    } else
      try {
        return wrap(
            typeid(T),
            typeid(*p),
            dynamic_cast<void*>(const_cast<T*>(p)),
            const_cast<T*>(p),
            policy,
            cleanup ? nb::handle(cleanup->self()) : nb::handle());
      } catch (...) {
        PyErr_SetString(
            PyExc_RuntimeError, "DART nanobind registry conversion failed");
        return {};
      }
  }
  template <class U>
  bool can_cast() const noexcept
  {
    return std::is_pointer_v<U> || value;
  }
  operator T*()
  {
    return value;
  }
  operator T&()
  {
    return *value;
  }
  operator T&&()
  {
    return std::move(*value);
  }
};

// Stock shared_ptr output bypasses the pointee caster and loses base offsets.
// Recover native control blocks on input and adjust dynamic pointers on output.
template <class T>
struct shared_caster
{
  using Value = std::shared_ptr<T>;
  static constexpr auto Name = nb::detail::const_name<T>();
  template <class U>
  using Cast = nb::detail::movable_cast_t<U>;
  Value value;
  template <class U>
  bool can_cast() const noexcept
  {
    return true;
  }
  operator Value*()
  {
    return &value;
  }
  operator Value&()
  {
    return value;
  }
  operator Value&&()
  {
    return std::move(value);
  }
  bool from_python(
      nb::handle src,
      uint32_t flags,
      nb::detail::cleanup_list* cleanup) noexcept
  {
    using Mutable = std::remove_const_t<T>;
    nb::detail::make_caster<Mutable> caster;
    if (!caster.from_python(src, flags, cleanup))
      return false;
    auto* p = caster.operator Mutable*();
    if (!p) {
      value.reset();
      return true;
    }
    if constexpr (std::is_same_v<Mutable, dart::dynamics::Skeleton>) {
      value = p->getPtr();
      return true;
    }
    if (auto owner = native_owner(complete_address(p))) {
      value = std::shared_ptr<T>(std::move(owner), p);
      return true;
    }
    // Raw-reference wrappers cannot manufacture native shared ownership.
    if (!nb::inst_state(src).second)
      return false;
    src.inc_ref();
    value = std::shared_ptr<T>(p, PythonPin{src.ptr()});
    return true;
  }
  static nb::handle from_cpp(
      const std::shared_ptr<T>& value,
      nb::rv_policy,
      nb::detail::cleanup_list* cleanup) noexcept
  {
    if (!value)
      return nb::none().release();
    try {
      using Mutable = std::remove_const_t<T>;
      auto* p = const_cast<Mutable*>(value.get());
      bool is_new = false;
      nb::object result;
      if constexpr (std::is_polymorphic_v<Mutable>) {
        result = nb::steal<nb::object>(wrap(
            typeid(Mutable),
            typeid(*p),
            dynamic_cast<void*>(p),
            p,
            nb::rv_policy::reference,
            {},
            &is_new));
      } else {
        result = nb::steal<nb::object>(
            nb::detail::type_caster_base<Mutable>::from_cpp(
                p, nb::rv_policy::reference, cleanup));
        is_new = true;
      }
      retain_shared_owner(
          result,
          is_new,
          std::const_pointer_cast<Mutable>(value),
          complete_address(p));
      (void)cleanup;
      return result.release();
    } catch (...) {
      PyErr_SetString(
          PyExc_RuntimeError, "DART nanobind shared-owner conversion failed");
      return {};
    }
  }
};
} // namespace dartnb

namespace nanobind::detail {
// All polymorphic DART bindings, including macro-generated aspect classes,
// must use the same pointer registry. Other values retain the native caster.
template <class T>
struct type_caster<
    T,
    std::enable_if_t<std::is_polymorphic_v<T> || dartnb::value_base<T>, int>>
  : dartnb::polymorphic_caster<T>
{
};
template <class T>
struct type_caster<std::shared_ptr<T>> : dartnb::shared_caster<T>
{
};

template <class T>
struct type_caster<dart::dynamics::TemplateBodyNodePtr<T>>
{
  using DartGraphType = T;
  using Mutable = std::remove_const_t<T>;
  static_assert(std::is_base_of_v<
                dartnb::polymorphic_caster<Mutable>,
                make_caster<Mutable>>);
  NB_TYPE_CASTER(dart::dynamics::TemplateBodyNodePtr<T>, make_caster<T>::Name)
  bool from_python(handle src, uint32_t flags, cleanup_list* cleanup) noexcept
  {
    make_caster<T> caster;
    if (!caster.from_python(src, flags, cleanup))
      return false;
    value = Value(caster.operator T*());
    return true;
  }
  static handle from_cpp(
      const Value& value, rv_policy policy, cleanup_list* cleanup) noexcept
  {
    return make_caster<T>::from_cpp(value.get(), policy, cleanup);
  }
};
} // namespace nanobind::detail
