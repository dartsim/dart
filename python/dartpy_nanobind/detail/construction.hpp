#pragma once

#include <nanobind/eval.h>

namespace dartnb {
template <class T>
nb::object nativeInstance(
    nb::handle requested, const std::shared_ptr<T>& owner, bool ready = true)
{
  auto result = nb::inst_reference(requested, owner.get());
  hold_native_owner(result, owner);
  nb::inst_set_state(result, ready, false);
  if (ready)
    remember_wrapper(typeid(T), dynamic_cast<void*>(owner.get()), result);
  return result;
}

inline void bindConstructionGuard(nb::module_& m)
{
  m.def("_instance_ready", [](nb::handle self) {
    return nb::inst_check(self) && nb::inst_ready(self);
  });
  m.attr("__dict__")["__builtins__"]
      = nb::module_::import_("builtins").attr("__dict__");
  nb::exec(
      R"(
from functools import wraps as _wraps

def _guard_init(cls):
    # Install at allocation, so overridden __init_subclass__ cannot bypass it.
    init = next(base.__dict__['__init__'] for base in cls.__mro__
                if '__init__' in base.__dict__)
    if getattr(init, '_dart_nb_checked', False):
        return
    @_wraps(init.__func__ if isinstance(init, (staticmethod, classmethod)) else init)
    def checked(self, *args, **kwargs):
        bound = init.__get__(self, type(self)) if hasattr(init, '__get__') else init
        result = bound(*args, **kwargs)
        if not _instance_ready(self):
            raise TypeError('base __init__ must be called')
        return result
    checked._dart_nb_checked = True
    cls.__init__ = checked
)",
      m.attr("__dict__"));
}

template <class T, class Cls, class Factory, class Subclass, class... Extra>
void defNativeNew(
    Cls& cls, Factory factory, Subclass subclass, const Extra&... extra)
{
  cls.def_static("_native_factory", factory, extra...);
  cls.def_static(
      "__new__",
      [subclass](nb::handle requested, nb::args args, nb::kwargs kwargs) {
        if (!nb::type_check(requested) || nb::type_info(requested) != typeid(T))
          throw nb::type_error("incompatible construction type");
        if (requested.is(nb::type<T>()))
          return requested.attr("_native_factory")(*args, **kwargs);
        nb::module_::import_("dartpy").attr("_guard_init")(requested);
        return subclass(requested);
      });
}

template <class T, class Cls, class Factory, class... Extra>
void defHybridNew(Cls& cls, Factory factory, const Extra&... extra)
{
  defNativeNew<T>(
      cls,
      factory,
      [](nb::handle requested) { return nb::inst_alloc(requested); },
      extra...);
}

template <class T, class... Args>
void initializeInPlace(nb::handle self, Args&&... args)
{
  if (!nb::inst_check(self) || nb::type_info(self.type()) != typeid(T))
    throw nb::type_error("incompatible initialization type");
  if (nb::inst_ready(self))
    return;
  auto* native = new (nb::inst_ptr<T>(self)) T(std::forward<Args>(args)...);
  nb::inst_mark_ready(self);
  remember_wrapper(typeid(T), dynamic_cast<void*>(native), self);
}

template <class T, class... Args, class Cls, class... Extra>
void defHybridInit(Cls& cls, const Extra&... extra)
{
  auto factory = [](Args... args) {
    return nativeInstance(nb::type<T>(), std::make_shared<T>(args...));
  };
  if (nb::cast<bool>(
          cls.attr("__dict__").attr("__contains__")("_native_factory")))
    cls.def_static("_native_factory", factory, extra...);
  else
    defHybridNew<T>(cls, factory, extra...);
  cls.def(
      "__init__",
      [](nb::handle self, Args... args) {
        initializeInPlace<T>(self, args...);
      },
      extra...);
}
} // namespace dartnb

namespace dartnb {
template <class T, class = void>
struct HasDefaultCreate : std::false_type
{
};
template <class T>
struct HasDefaultCreate<T, std::void_t<decltype(T::create())>> : std::true_type
{
};
template <class T>
struct SharedReturn : std::false_type
{
};
template <class T>
struct SharedReturn<std::shared_ptr<T>> : std::true_type
{
};
template <class F>
struct FactorySignature : FactorySignature<decltype(&F::operator())>
{
};
template <class R, class... Args>
struct FactorySignature<R (*)(Args...)>
{
  using Type = R(Args...);
};
template <class R, class C, class... Args>
struct FactorySignature<R (C::*)(Args...) const>
{
  using Type = R(Args...);
};

template <class F, class Sig = typename FactorySignature<F>::Type>
struct factory;
template <class F, class R, class... Args>
struct factory<F, R(Args...)> : nb::def_visitor<factory<F, R(Args...)>>
{
  F func;
  explicit factory(F f) : func(std::move(f)) {}
  template <class Cls, class... Extra>
  void execute(Cls& cls, const Extra&... extra) const
  {
    using T = typename Cls::Type;
    if constexpr (!SharedReturn<R>::value) {
      cls.def(nb::new_(func), extra...);
    } else {
      bool first = !nb::cast<bool>(
          cls.attr("__dict__").attr("__contains__")("_native_factory"));
      cls.def_static(
          "_native_factory",
          [f = func](Args... args) {
            return nativeInstance(nb::type<T>(), f(args...));
          },
          extra...);
      if (first) {
        cls.def_static(
            "__new__",
            [](nb::handle requested, nb::args args, nb::kwargs kwargs) {
              if (!nb::type_check(requested)
                  || nb::type_info(requested) != typeid(T))
                throw nb::type_error("incompatible construction type");
              if (requested.is(nb::type<T>()))
                return requested.attr("_native_factory")(*args, **kwargs);
              nb::module_::import_("dartpy").attr("_guard_init")(requested);
              if constexpr (HasDefaultCreate<T>::value) {
                return nativeInstance(requested, T::create(), false);
              } else {
                auto exact = requested.attr("_native_factory")(*args, **kwargs);
                return nativeInstance(
                    requested, nb::cast<std::shared_ptr<T>>(exact), false);
              }
            });
      }
      cls.def(
          "__init__",
          [f = func](nb::handle self, Args... args) {
            if (!nb::inst_check(self)
                || nb::type_info(self.type()) != typeid(T))
              throw nb::type_error("incompatible initialization type");
            if (nb::inst_ready(self))
              return;
            auto* native = nb::inst_ptr<T>(self);
            if constexpr (std::is_same_v<T, dart::dynamics::Skeleton>) {
              auto prepared = f(args...);
              native->setProperties(prepared->getProperties());
            } else if constexpr (std::is_same_v<T, dart::simulation::World>) {
              auto prepared = f(args...);
              native->setName(prepared->getName());
            }
            nb::inst_set_state(self, true, false);
            remember_wrapper(typeid(T), complete_address(native), self);
          },
          extra...);
    }
  }
};
template <class F>
factory(F) -> factory<F>;
} // namespace dartnb

namespace dartnb {
template <class T>
struct Nullable : std::is_pointer<T>
{
};
template <class T>
struct Nullable<std::shared_ptr<T>> : std::true_type
{
};
template <class T>
struct Nullable<std::unique_ptr<T>> : std::true_type
{
};
template <class T>
struct Nullable<dart::dynamics::TemplateBodyNodePtr<T>> : std::true_type
{
};

template <class T, class Class>
auto setterArgument(T Class::*)
{
  if constexpr (Nullable<T>::value)
    return nb::for_setter(nb::arg("value").none());
  else
    return nb::for_setter(nb::arg("value"));
}

// Mirror init's visitor interface so chains keep their original overload order.
template <class... Args>
struct init : nb::def_visitor<init<Args...>>
{
  template <class Cls, class... Extra>
  void execute(Cls& cls, const Extra&... extra) const
  {
    using T = typename Cls::Type;
    using Alias = typename Cls::Alias;
    if constexpr (!std::is_polymorphic_v<T>) {
      cls.def(nb::init<Args...>(), extra...);
    } else if constexpr (std::is_same_v<T, Alias>) {
      defHybridInit<T, Args...>(cls, extra...);
    } else {
      if (!nb::cast<bool>(
              cls.attr("__dict__").attr("__contains__")("_native_factory"))) {
        cls.def_static(
            "__new__",
            [](nb::handle requested, nb::args args, nb::kwargs kwargs) {
              if (requested.is(nb::type<T>()))
                return requested.attr("_native_factory")(*args, **kwargs);
              nb::module_::import_("dartpy").attr("_guard_init")(requested);
              return nb::inst_alloc(requested);
            });
      }
      cls.def_static(
          "_native_factory",
          [](Args... args) {
            auto result = nb::inst_alloc(nb::type<T>());
            auto* native = new (nb::inst_ptr<T>(result)) Alias(args...);
            nb::inst_mark_ready(result);
            remember_wrapper(typeid(T), complete_address(native), result);
            return result;
          },
          extra...);
      cls.def(
          "__init__",
          [](nb::handle self, Args... args) {
            if (!nb::inst_check(self)
                || nb::type_info(self.type()) != typeid(T))
              throw nb::type_error("incompatible initialization type");
            if (nb::inst_ready(self))
              return;
            auto* native = new (nb::inst_ptr<T>(self)) Alias(args...);
            nb::inst_mark_ready(self);
            remember_wrapper(typeid(T), complete_address(native), self);
          },
          extra...);
    }
  }
};
} // namespace dartnb
