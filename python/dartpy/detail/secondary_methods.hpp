#pragma once

namespace dartnb {
template <class T>
struct SecondaryMethods
{
  nb::class_<T> cls;
  std::set<std::string> inherited;
  explicit SecondaryMethods(nb::handle target)
    : cls(nb::borrow<nb::class_<T>>(target))
  {
    for (nb::handle name : nb::module_::import_("builtins").attr("dir")(target))
      inherited.insert(nb::cast<std::string>(name));
  }
  template <class... Args>
  SecondaryMethods& def(const char* name, Args&&... args)
  {
    if (!inherited.count(name))
      cls.def(name, std::forward<Args>(args)...);
    return *this;
  }
  template <
      class Visitor,
      class... Args,
      std::enable_if_t<
          !std::is_convertible_v<const Visitor&, const char*>,
          int> = 0>
  SecondaryMethods& def(const Visitor&, const Args&...)
  {
    return *this; // Constructors belong only to the requested class.
  }
  template <class... Args>
  SecondaryMethods& def_static(const char* name, Args&&... args)
  {
    if (!inherited.count(name))
      cls.def_static(name, std::forward<Args>(args)...);
    return *this;
  }
};
} // namespace dartnb
