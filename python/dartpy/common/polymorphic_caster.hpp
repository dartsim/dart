#pragma once

#include <nanobind/nanobind.h>

#include <cstdio>
#include <cstdlib>
#include <typeindex>
#include <unordered_map>

namespace dart::python_nb {

namespace detail {

template <typename Base>
class PolymorphicCasterRegistry
{
public:
  using Pointer = Base*;
  using Caster = Pointer (*)(void*);
  using Downcaster = void* (*)(Pointer);

  struct Entry
  {
    Caster upcast = nullptr;
    Downcaster downcast = nullptr;
  };

  static void registerType(
      const std::type_info& info, Caster caster, Downcaster downcaster)
  {
    getMap()[std::type_index(info)] = Entry{caster, downcaster};
  }

  static bool hasType(const std::type_info& info)
  {
    return getMap().contains(std::type_index(info));
  }

  static Pointer convert(void* raw, const std::type_info& info)
  {
    if (raw == nullptr)
      return nullptr;

    auto& map = getMap();
    auto it = map.find(std::type_index(info));
    if (it == map.end())
      return static_cast<Pointer>(raw);
    auto* caster = it->second.upcast;
    if (caster == nullptr)
      return static_cast<Pointer>(raw);
    return caster(raw);
  }

  static void* restore(Pointer raw, const std::type_info& info)
  {
    if (raw == nullptr)
      return static_cast<void*>(raw);

    auto& map = getMap();
    auto it = map.find(std::type_index(info));
    if (it == map.end())
      return static_cast<void*>(raw);
    auto* downcaster = it->second.downcast;
    if (downcaster == nullptr)
      return static_cast<void*>(raw);
    return downcaster(raw);
  }

  static Pointer adjust(PyObject* source, Pointer raw)
  {
    if (source == nullptr || raw == nullptr)
      return raw;

    PyTypeObject* type = Py_TYPE(source);
    if (!nanobind::detail::nb_type_check(reinterpret_cast<PyObject*>(type)))
      return raw;

    const std::type_info* info
        = nanobind::detail::nb_type_info(reinterpret_cast<PyObject*>(type));
    if (info == nullptr)
      return raw;

    return convert(static_cast<void*>(raw), *info);
  }

private:
  static std::unordered_map<std::type_index, Entry>& getMap()
  {
    static std::unordered_map<std::type_index, Entry> map;
    return map;
  }

};

} // namespace detail

template <typename Base, typename Derived>
inline void registerPolymorphicCaster()
{
  const char* trace = std::getenv("DARTPY_TRACE_POLY_REGISTER");
  if (trace) {
    std::fprintf(
        stderr,
        "[dartpy][poly][register] base=%s derived=%s\n",
        typeid(Base).name(),
        typeid(Derived).name());
  }
  detail::PolymorphicCasterRegistry<Base>::registerType(
      typeid(Derived),
      [](void* ptr) -> Base* {
        if (ptr == nullptr)
          return nullptr;
        auto* typed = static_cast<Derived*>(ptr);
        return static_cast<Base*>(typed);
      },
      [](Base* base) -> void* {
        if (base == nullptr)
          return nullptr;
        auto* typed = dynamic_cast<Derived*>(base);
        return typed ? static_cast<void*>(typed) : static_cast<void*>(base);
      });
}

template <typename Base>
inline Base* adjustPolymorphicPointer(PyObject* source, Base* raw)
{
  return detail::PolymorphicCasterRegistry<Base>::adjust(source, raw);
}

template <typename Base>
inline bool hasPolymorphicCaster(const std::type_info& info)
{
  return detail::PolymorphicCasterRegistry<Base>::hasType(info);
}

template <typename Base>
inline Base* convertPolymorphicPointer(void* raw, const std::type_info& info)
{
  return detail::PolymorphicCasterRegistry<Base>::convert(raw, info);
}

template <typename Base>
inline void* restorePolymorphicPointer(Base* raw, const std::type_info& info)
{
  return detail::PolymorphicCasterRegistry<Base>::restore(raw, info);
}

} // namespace dart::python_nb
