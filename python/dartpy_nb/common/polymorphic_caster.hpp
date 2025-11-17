#pragma once

#include <nanobind/nanobind.h>

#include <mutex>
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

  static void registerType(const std::type_info& info, Caster caster)
  {
    std::lock_guard<std::mutex> lock(getMutex());
    getMap()[std::type_index(info)] = caster;
  }

  static bool hasType(const std::type_info& info)
  {
    std::lock_guard<std::mutex> lock(getMutex());
    return getMap().contains(std::type_index(info));
  }

  static Pointer convert(void* raw, const std::type_info& info)
  {
    std::lock_guard<std::mutex> lock(getMutex());
    auto& map = getMap();
    auto it = map.find(std::type_index(info));
    if (it == map.end())
      return static_cast<Pointer>(raw);
    return it->second(raw);
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
  static std::unordered_map<std::type_index, Caster>& getMap()
  {
    static std::unordered_map<std::type_index, Caster> map;
    return map;
  }

  static std::mutex& getMutex()
  {
    static std::mutex mutex;
    return mutex;
  }
};

} // namespace detail

template <typename Base, typename Derived>
inline void registerPolymorphicCaster()
{
  detail::PolymorphicCasterRegistry<Base>::registerType(
      typeid(Derived),
      [](void* ptr) -> Base* {
        auto* typed = static_cast<Derived*>(ptr);
        return static_cast<Base*>(typed);
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

} // namespace dart::python_nb
