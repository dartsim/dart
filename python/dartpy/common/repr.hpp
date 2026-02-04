#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace nb = nanobind;

namespace dart::python_nb {

inline std::string repr_string(const std::string& value)
{
  nb::object repr_obj = nb::repr(nb::cast(value));
  return nb::cast<std::string>(repr_obj);
}

inline std::string repr_string(std::string_view value)
{
  return repr_string(std::string(value));
}

inline std::string repr_bool(bool value)
{
  return value ? "True" : "False";
}

inline std::string repr_double(double value, int precision = 6)
{
  std::ostringstream oss;
  oss << std::setprecision(precision) << value;
  return oss.str();
}

inline std::string format_repr(
    const std::string& type_name,
    const std::vector<std::pair<std::string, std::string>>& fields)
{
  std::ostringstream oss;
  oss << type_name;
  if (!fields.empty()) {
    oss << "(";
    for (std::size_t i = 0; i < fields.size(); ++i) {
      if (i != 0) {
        oss << ", ";
      }
      oss << fields[i].first << "=" << fields[i].second;
    }
    oss << ")";
  }
  return oss.str();
}

} // namespace dart::python_nb
