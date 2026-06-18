/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <dart/simulation/body/rigid_body.hpp>
#include <dart/simulation/body/rigid_body_options.hpp>
#include <dart/simulation/common/exceptions.hpp>
#include <dart/simulation/comps/rigid_body.hpp>
#include <dart/simulation/compute/sequential_executor.hpp>
#include <dart/simulation/compute/world_step_stage.hpp>
#include <dart/simulation/detail/entity_conversion.hpp>
#include <dart/simulation/detail/rigid_ipc/rigid_ipc_ccd.hpp>
#include <dart/simulation/detail/world_registry_access.hpp>
#include <dart/simulation/io/detail/rigid_ipc_fixture.hpp>
#include <dart/simulation/world.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <bit>
#include <filesystem>
#include <format>
#include <fstream>
#include <istream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

namespace dart::simulation::io::detail {
namespace {

namespace rigid_detail = ::dart::simulation::detail;

struct JsonValue
{
  using Array = std::vector<JsonValue>;
  using Object = std::map<std::string, JsonValue>;

  std::variant<std::nullptr_t, bool, double, std::string, Array, Object> value{
      nullptr};

  JsonValue() = default;
  explicit JsonValue(bool input) : value(input) {}
  explicit JsonValue(double input) : value(input) {}
  explicit JsonValue(std::string input) : value(std::move(input)) {}
  explicit JsonValue(Array input) : value(std::move(input)) {}
  explicit JsonValue(Object input) : value(std::move(input)) {}

  [[nodiscard]] const Object* object() const
  {
    return std::get_if<Object>(&value);
  }

  [[nodiscard]] const Array* array() const
  {
    return std::get_if<Array>(&value);
  }

  [[nodiscard]] const std::string* string() const
  {
    return std::get_if<std::string>(&value);
  }

  [[nodiscard]] const double* number() const
  {
    return std::get_if<double>(&value);
  }

  [[nodiscard]] const bool* boolean() const
  {
    return std::get_if<bool>(&value);
  }
};

class JsonParser
{
public:
  explicit JsonParser(std::string input) : m_text(std::move(input)) {}

  [[nodiscard]] JsonValue parse()
  {
    JsonValue value = parseValue();
    skipWhitespace();
    if (m_pos != m_text.size()) {
      fail("Unexpected trailing content");
    }
    return value;
  }

private:
  [[nodiscard]] JsonValue parseValue()
  {
    skipWhitespace();
    if (m_pos == m_text.size()) {
      fail("Unexpected end of input");
    }

    const char c = m_text[m_pos];
    if (c == '{') {
      return JsonValue(parseObject());
    }
    if (c == '[') {
      return JsonValue(parseArray());
    }
    if (c == '"') {
      return JsonValue(parseString());
    }
    if (c == '-' || (c >= '0' && c <= '9')) {
      return JsonValue(parseNumber());
    }
    if (matchLiteral("true")) {
      return JsonValue(true);
    }
    if (matchLiteral("false")) {
      return JsonValue(false);
    }
    if (matchLiteral("null")) {
      return JsonValue();
    }

    fail("Unexpected token");
  }

  [[nodiscard]] JsonValue::Object parseObject()
  {
    expect('{');
    JsonValue::Object object;
    skipWhitespace();
    if (consume('}')) {
      return object;
    }

    while (true) {
      skipWhitespace();
      if (m_pos == m_text.size() || m_text[m_pos] != '"') {
        fail("Expected object key");
      }
      std::string key = parseString();
      skipWhitespace();
      expect(':');
      object.emplace(std::move(key), parseValue());
      skipWhitespace();
      if (consume('}')) {
        break;
      }
      expect(',');
    }

    return object;
  }

  [[nodiscard]] JsonValue::Array parseArray()
  {
    expect('[');
    JsonValue::Array array;
    skipWhitespace();
    if (consume(']')) {
      return array;
    }

    while (true) {
      array.push_back(parseValue());
      skipWhitespace();
      if (consume(']')) {
        break;
      }
      expect(',');
    }

    return array;
  }

  [[nodiscard]] std::string parseString()
  {
    expect('"');
    std::string output;

    while (m_pos < m_text.size()) {
      const char c = m_text[m_pos++];
      if (c == '"') {
        return output;
      }
      if (c != '\\') {
        output.push_back(c);
        continue;
      }

      if (m_pos == m_text.size()) {
        fail("Unterminated escape sequence");
      }

      const char escaped = m_text[m_pos++];
      switch (escaped) {
        case '"':
        case '\\':
        case '/':
          output.push_back(escaped);
          break;
        case 'b':
          output.push_back('\b');
          break;
        case 'f':
          output.push_back('\f');
          break;
        case 'n':
          output.push_back('\n');
          break;
        case 'r':
          output.push_back('\r');
          break;
        case 't':
          output.push_back('\t');
          break;
        case 'u':
          output.push_back(parseAsciiUnicodeEscape());
          break;
        default:
          fail("Invalid escape sequence");
      }
    }

    fail("Unterminated string");
  }

  [[nodiscard]] double parseNumber()
  {
    const char* begin = m_text.c_str() + m_pos;
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(begin, &end);
    if (end == begin || errno == ERANGE) {
      fail("Invalid number");
    }
    m_pos = static_cast<std::size_t>(end - m_text.c_str());
    return value;
  }

  [[nodiscard]] char parseAsciiUnicodeEscape()
  {
    int code = 0;
    for (int i = 0; i < 4; ++i) {
      if (m_pos == m_text.size()) {
        fail("Unterminated unicode escape");
      }
      const char c = m_text[m_pos++];
      code *= 16;
      if (c >= '0' && c <= '9') {
        code += c - '0';
      } else if (c >= 'a' && c <= 'f') {
        code += 10 + c - 'a';
      } else if (c >= 'A' && c <= 'F') {
        code += 10 + c - 'A';
      } else {
        fail("Invalid unicode escape");
      }
    }

    return code <= 0x7f ? static_cast<char>(code) : '?';
  }

  void skipWhitespace()
  {
    while (m_pos < m_text.size()) {
      const char c = m_text[m_pos];
      if (c != ' ' && c != '\n' && c != '\r' && c != '\t') {
        break;
      }
      ++m_pos;
    }
  }

  [[nodiscard]] bool consume(char expected)
  {
    if (m_pos < m_text.size() && m_text[m_pos] == expected) {
      ++m_pos;
      return true;
    }
    return false;
  }

  void expect(char expected)
  {
    if (!consume(expected)) {
      fail(std::string("Expected '") + expected + "'");
    }
  }

  [[nodiscard]] bool matchLiteral(std::string_view literal)
  {
    if (m_text.compare(m_pos, literal.size(), literal) != 0) {
      return false;
    }
    m_pos += literal.size();
    return true;
  }

  [[noreturn]] void fail(std::string_view message) const
  {
    DART_SIMULATION_THROW_T(
        InvalidArgumentException, "{} at byte {}", message, m_pos);
  }

  std::string m_text;
  std::size_t m_pos{0};
};

[[nodiscard]] std::string typeName(const JsonValue& value)
{
  if (std::holds_alternative<std::nullptr_t>(value.value)) {
    return "null";
  }
  if (value.boolean() != nullptr) {
    return "bool";
  }
  if (value.number() != nullptr) {
    return "number";
  }
  if (value.string() != nullptr) {
    return "string";
  }
  if (value.array() != nullptr) {
    return "array";
  }
  return "object";
}

[[nodiscard]] const JsonValue* findMember(
    const JsonValue::Object& object, std::string_view key)
{
  const auto it = object.find(std::string(key));
  if (it == object.end()) {
    return nullptr;
  }
  return &it->second;
}

[[nodiscard]] const JsonValue::Object& requireObject(
    const JsonValue& value, std::string_view path)
{
  const auto* object = value.object();
  if (object == nullptr) {
    DART_SIMULATION_THROW_T(
        InvalidArgumentException,
        "Expected object at {}, got {}",
        path,
        typeName(value));
  }
  return *object;
}

[[nodiscard]] const JsonValue::Array& requireArray(
    const JsonValue& value, std::string_view path)
{
  const auto* array = value.array();
  if (array == nullptr) {
    DART_SIMULATION_THROW_T(
        InvalidArgumentException,
        "Expected array at {}, got {}",
        path,
        typeName(value));
  }
  return *array;
}

template <typename Record>
void addDiagnostic(
    Record& record,
    RigidIpcFixtureDiagnosticSeverity severity,
    std::string path,
    std::string message)
{
  record.diagnostics.push_back(
      RigidIpcFixtureDiagnostic{severity, std::move(path), std::move(message)});
}

template <typename Record>
void addWarning(Record& record, std::string path, std::string message)
{
  addDiagnostic(
      record,
      RigidIpcFixtureDiagnosticSeverity::Warning,
      std::move(path),
      std::move(message));
}

template <typename Record>
void addError(Record& record, std::string path, std::string message)
{
  addDiagnostic(
      record,
      RigidIpcFixtureDiagnosticSeverity::Error,
      std::move(path),
      std::move(message));
}

template <typename Record, std::size_t N>
void warnUnsupportedFields(
    const JsonValue::Object& object,
    const std::array<std::string_view, N>& allowedFields,
    std::string_view path,
    Record& record)
{
  for (const auto& [key, value] : object) {
    (void)value;
    const std::string_view keyView(key);
    const bool allowed
        = std::find(allowedFields.begin(), allowedFields.end(), keyView)
          != allowedFields.end();
    if (!allowed) {
      addWarning(
          record,
          std::string(path) + "." + key,
          "field is not imported by the current fixture slice");
    }
  }
}

template <typename Record>
[[nodiscard]] std::optional<std::string> readStringMember(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    Record& record)
{
  const JsonValue* value = findMember(object, key);
  if (value == nullptr) {
    return std::nullopt;
  }

  const std::string* string = value->string();
  if (string == nullptr) {
    addError(
        record, std::string(path), "expected string, got " + typeName(*value));
    return std::nullopt;
  }

  return *string;
}

template <typename Record>
[[nodiscard]] std::optional<double> readDoubleMember(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    Record& record)
{
  const JsonValue* value = findMember(object, key);
  if (value == nullptr) {
    return std::nullopt;
  }

  const double* number = value->number();
  if (number == nullptr) {
    addError(
        record, std::string(path), "expected number, got " + typeName(*value));
    return std::nullopt;
  }

  return *number;
}

template <typename Record>
[[nodiscard]] std::optional<int> readIntMember(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    Record& record)
{
  const std::optional<double> value
      = readDoubleMember(object, key, path, record);
  if (!value.has_value()) {
    return std::nullopt;
  }

  return static_cast<int>(*value);
}

template <typename Record>
[[nodiscard]] std::optional<bool> readBoolMember(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    Record& record)
{
  const JsonValue* value = findMember(object, key);
  if (value == nullptr) {
    return std::nullopt;
  }

  const bool* boolean = value->boolean();
  if (boolean == nullptr) {
    addError(
        record, std::string(path), "expected bool, got " + typeName(*value));
    return std::nullopt;
  }

  return *boolean;
}

template <typename Record>
[[nodiscard]] std::optional<Eigen::Vector3d> readVector3Member(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    int scalarAxis,
    Record& record)
{
  const JsonValue* value = findMember(object, key);
  if (value == nullptr) {
    return std::nullopt;
  }

  Eigen::Vector3d vector = Eigen::Vector3d::Zero();
  if (const double* scalar = value->number(); scalar != nullptr) {
    vector[scalarAxis] = *scalar;
    return vector;
  }

  const JsonValue::Array* array = value->array();
  if (array == nullptr) {
    addError(
        record,
        std::string(path),
        "expected number array, got " + typeName(*value));
    return std::nullopt;
  }

  if (array->size() > 3) {
    addError(
        record, std::string(path), "expected at most three numeric components");
  }

  if (array->size() == 1) {
    const double* number = (*array)[0].number();
    if (number == nullptr) {
      addError(
          record,
          std::string(path) + "[0]",
          "expected number, got " + typeName((*array)[0]));
      return std::nullopt;
    }
    vector[scalarAxis] = *number;
    return vector;
  }

  const std::size_t count = std::min<std::size_t>(array->size(), 3);
  for (std::size_t i = 0; i < count; ++i) {
    const double* number = (*array)[i].number();
    if (number == nullptr) {
      addError(
          record,
          std::string(path) + "[" + std::to_string(i) + "]",
          "expected number, got " + typeName((*array)[i]));
      return std::nullopt;
    }
    vector[static_cast<Eigen::Index>(i)] = *number;
  }

  return vector;
}

template <typename Record>
[[nodiscard]] std::optional<Eigen::Vector3d> readRequiredVector3Member(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    int scalarAxis,
    Record& record)
{
  if (findMember(object, key) == nullptr) {
    addError(record, std::string(path), "missing required vector field");
    return std::nullopt;
  }

  return readVector3Member(object, key, path, scalarAxis, record);
}

template <typename Record>
[[nodiscard]] std::optional<Eigen::Vector3d> readScaleMember(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    Record& record)
{
  const JsonValue* value = findMember(object, key);
  if (value == nullptr) {
    return std::nullopt;
  }

  if (const double* scalar = value->number(); scalar != nullptr) {
    return Eigen::Vector3d::Constant(*scalar);
  }

  const JsonValue::Array* array = value->array();
  if (array == nullptr) {
    addError(
        record,
        std::string(path),
        "expected number or number array, got " + typeName(*value));
    return std::nullopt;
  }

  if (array->size() == 1) {
    const double* scalar = (*array)[0].number();
    if (scalar == nullptr) {
      addError(
          record,
          std::string(path) + "[0]",
          "expected number, got " + typeName((*array)[0]));
      return std::nullopt;
    }
    return Eigen::Vector3d::Constant(*scalar);
  }

  Eigen::Vector3d scale = Eigen::Vector3d::Ones();
  const std::size_t count = std::min<std::size_t>(array->size(), 3);
  for (std::size_t i = 0; i < count; ++i) {
    const double* number = (*array)[i].number();
    if (number == nullptr) {
      addError(
          record,
          std::string(path) + "[" + std::to_string(i) + "]",
          "expected number, got " + typeName((*array)[i]));
      return std::nullopt;
    }
    scale[static_cast<Eigen::Index>(i)] = *number;
  }

  if (array->size() > 3) {
    addError(
        record, std::string(path), "expected at most three numeric components");
  }

  return scale;
}

[[nodiscard]] std::vector<bool> readFixedDofs(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    RigidIpcFixture& fixture)
{
  const JsonValue* value = findMember(object, key);
  if (value == nullptr) {
    return {};
  }

  if (const bool* fixed = value->boolean(); fixed != nullptr) {
    return std::vector<bool>(6, *fixed);
  }

  const JsonValue::Array* array = value->array();
  if (array == nullptr) {
    addError(
        fixture,
        std::string(path),
        "expected bool or bool array, got " + typeName(*value));
    return {};
  }

  std::vector<bool> fixedDofs;
  fixedDofs.reserve(array->size());
  for (std::size_t i = 0; i < array->size(); ++i) {
    const bool* fixed = (*array)[i].boolean();
    if (fixed == nullptr) {
      addError(
          fixture,
          std::string(path) + "[" + std::to_string(i) + "]",
          "expected bool, got " + typeName((*array)[i]));
      return {};
    }
    fixedDofs.push_back(*fixed);
  }

  return fixedDofs;
}

[[nodiscard]] RigidIpcBodyMode readBodyMode(
    const JsonValue::Object& object,
    std::string_view path,
    RigidIpcFixture& fixture)
{
  const std::optional<std::string> type
      = readStringMember(object, "type", std::string(path) + ".type", fixture);
  if (!type.has_value() || *type == "dynamic") {
    return RigidIpcBodyMode::Dynamic;
  }
  if (*type == "static") {
    return RigidIpcBodyMode::Static;
  }
  if (*type == "kinematic") {
    return RigidIpcBodyMode::Kinematic;
  }

  addWarning(
      fixture,
      std::string(path) + ".type",
      "unknown body type is recorded as dynamic");
  return RigidIpcBodyMode::Dynamic;
}

template <typename Record>
[[nodiscard]] std::optional<Eigen::Vector3d> readInlineVectorValue(
    const JsonValue& value, std::string_view path, Record& record)
{
  const JsonValue::Array* array = value.array();
  if (array == nullptr) {
    addError(
        record,
        std::string(path),
        "expected coordinate array, got " + typeName(value));
    return std::nullopt;
  }

  if (array->size() < 2u || array->size() > 3u) {
    addError(
        record,
        std::string(path),
        "expected two or three coordinate components");
    return std::nullopt;
  }

  Eigen::Vector3d vector = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < array->size(); ++i) {
    const double* number = (*array)[i].number();
    if (number == nullptr) {
      addError(
          record,
          std::string(path) + "[" + std::to_string(i) + "]",
          "expected number, got " + typeName((*array)[i]));
      return std::nullopt;
    }
    vector[static_cast<Eigen::Index>(i)] = *number;
  }

  if (!vector.allFinite()) {
    addError(record, std::string(path), "coordinate must be finite");
    return std::nullopt;
  }

  return vector;
}

template <typename Record>
[[nodiscard]] std::optional<int> readInlineIndexValue(
    const JsonValue& value, std::string_view path, Record& record)
{
  const double* number = value.number();
  if (number == nullptr) {
    addError(
        record, std::string(path), "expected integer, got " + typeName(value));
    return std::nullopt;
  }

  if (!std::isfinite(*number) || *number < 0.0
      || *number > static_cast<double>(std::numeric_limits<int>::max())
      || std::floor(*number) != *number) {
    addError(record, std::string(path), "expected non-negative integer index");
    return std::nullopt;
  }

  return static_cast<int>(*number);
}

[[nodiscard]] std::vector<Eigen::Vector3d> readInlineVertices(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    RigidIpcFixture& fixture)
{
  const JsonValue* value = findMember(object, key);
  if (value == nullptr) {
    return {};
  }

  const JsonValue::Array* array = value->array();
  if (array == nullptr) {
    addError(
        fixture,
        std::string(path),
        "expected coordinate array list, got " + typeName(*value));
    return {};
  }

  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(array->size());
  for (std::size_t i = 0; i < array->size(); ++i) {
    if (const std::optional<Eigen::Vector3d> vertex = readInlineVectorValue(
            (*array)[i],
            std::string(path) + "[" + std::to_string(i) + "]",
            fixture);
        vertex.has_value()) {
      vertices.push_back(*vertex);
    }
  }
  return vertices;
}

[[nodiscard]] std::vector<Eigen::Vector2i> readInlineEdges(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    const std::size_t vertexCount,
    RigidIpcFixture& fixture)
{
  const JsonValue* value = findMember(object, key);
  if (value == nullptr) {
    return {};
  }

  const JsonValue::Array* array = value->array();
  if (array == nullptr) {
    addError(
        fixture,
        std::string(path),
        "expected index pair array list, got " + typeName(*value));
    return {};
  }

  std::vector<Eigen::Vector2i> edges;
  edges.reserve(array->size());
  for (std::size_t i = 0; i < array->size(); ++i) {
    const JsonValue::Array* edge = (*array)[i].array();
    const std::string edgePath
        = std::string(path) + "[" + std::to_string(i) + "]";
    if (edge == nullptr || edge->size() != 2u) {
      addError(fixture, edgePath, "expected two vertex indices");
      continue;
    }

    const std::optional<int> a
        = readInlineIndexValue((*edge)[0], edgePath + "[0]", fixture);
    const std::optional<int> b
        = readInlineIndexValue((*edge)[1], edgePath + "[1]", fixture);
    if (!a.has_value() || !b.has_value()) {
      continue;
    }
    if (static_cast<std::size_t>(*a) >= vertexCount
        || static_cast<std::size_t>(*b) >= vertexCount) {
      addError(fixture, edgePath, "edge index is out of vertex range");
      continue;
    }
    if (*a == *b) {
      addError(fixture, edgePath, "edge must use two distinct vertices");
      continue;
    }
    edges.emplace_back(*a, *b);
  }
  return edges;
}

[[nodiscard]] std::vector<std::vector<Eigen::Vector3d>> readInlinePolygons(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    RigidIpcFixture& fixture)
{
  const JsonValue* value = findMember(object, key);
  if (value == nullptr) {
    return {};
  }

  const JsonValue::Array* array = value->array();
  if (array == nullptr) {
    addError(
        fixture,
        std::string(path),
        "expected polygon coordinate array list, got " + typeName(*value));
    return {};
  }

  std::vector<std::vector<Eigen::Vector3d>> polygons;
  polygons.reserve(array->size());
  for (std::size_t i = 0; i < array->size(); ++i) {
    const JsonValue::Array* polygon = (*array)[i].array();
    const std::string polygonPath
        = std::string(path) + "[" + std::to_string(i) + "]";
    if (polygon == nullptr) {
      addError(fixture, polygonPath, "expected polygon coordinate array");
      continue;
    }
    if (polygon->size() < 3u) {
      addError(fixture, polygonPath, "polygon needs at least three vertices");
      continue;
    }

    std::vector<Eigen::Vector3d> vertices;
    vertices.reserve(polygon->size());
    for (std::size_t j = 0; j < polygon->size(); ++j) {
      if (const std::optional<Eigen::Vector3d> vertex = readInlineVectorValue(
              (*polygon)[j],
              polygonPath + "[" + std::to_string(j) + "]",
              fixture);
          vertex.has_value()) {
        vertices.push_back(*vertex);
      }
    }
    if (vertices.size() >= 3u) {
      polygons.push_back(std::move(vertices));
    }
  }
  return polygons;
}

void readBody(
    const JsonValue& bodyValue, std::string_view path, RigidIpcFixture& fixture)
{
  const JsonValue::Object& body = requireObject(bodyValue, path);
  warnUnsupportedFields(
      body,
      std::array<std::string_view, 21>{
          "mesh",
          "vertices",
          "edges",
          "polygons",
          "oriented",
          "position",
          "rotation",
          "scale",
          "linear_velocity",
          "angular_velocity",
          "force",
          "torque",
          "is_dof_fixed",
          "group_id",
          "density",
          "type",
          "kinematic_max_time",
          "#torque",
          "#force",
          "#kinematic_max_time"},
      path,
      fixture);

  RigidIpcBodyRecord record;
  if (const std::optional<std::string> mesh
      = readStringMember(body, "mesh", std::string(path) + ".mesh", fixture);
      mesh.has_value()) {
    record.meshPath = *mesh;
  }
  if (const std::optional<Eigen::Vector3d> value = readVector3Member(
          body, "position", std::string(path) + ".position", 0, fixture);
      value.has_value()) {
    record.position = *value;
  }
  if (const std::optional<Eigen::Vector3d> value = readVector3Member(
          body, "rotation", std::string(path) + ".rotation", 2, fixture);
      value.has_value()) {
    record.rotationDegrees = *value;
  }
  if (const std::optional<Eigen::Vector3d> value
      = readScaleMember(body, "scale", std::string(path) + ".scale", fixture);
      value.has_value()) {
    record.scale = *value;
  }
  if (const std::optional<Eigen::Vector3d> value = readVector3Member(
          body,
          "linear_velocity",
          std::string(path) + ".linear_velocity",
          0,
          fixture);
      value.has_value()) {
    record.linearVelocity = *value;
  }
  if (const std::optional<Eigen::Vector3d> value = readVector3Member(
          body,
          "angular_velocity",
          std::string(path) + ".angular_velocity",
          2,
          fixture);
      value.has_value()) {
    record.angularVelocity = *value;
  }
  if (const std::optional<Eigen::Vector3d> value = readVector3Member(
          body, "force", std::string(path) + ".force", 0, fixture);
      value.has_value()) {
    record.force = *value;
  }
  if (const std::optional<Eigen::Vector3d> value = readVector3Member(
          body, "torque", std::string(path) + ".torque", 2, fixture);
      value.has_value()) {
    record.torque = *value;
  }
  record.fixedDofs = readFixedDofs(
      body, "is_dof_fixed", std::string(path) + ".is_dof_fixed", fixture);
  record.groupId = readIntMember(
      body, "group_id", std::string(path) + ".group_id", fixture);
  record.density = readDoubleMember(
      body, "density", std::string(path) + ".density", fixture);
  record.kinematicMaxTime = readDoubleMember(
      body,
      "kinematic_max_time",
      std::string(path) + ".kinematic_max_time",
      fixture);
  record.mode = readBodyMode(body, path, fixture);

  if (findMember(body, "vertices") != nullptr
      || findMember(body, "edges") != nullptr
      || findMember(body, "polygons") != nullptr) {
    record.hasInlineGeometry = true;
    record.inlineVertices = readInlineVertices(
        body, "vertices", std::string(path) + ".vertices", fixture);
    record.inlineEdges = readInlineEdges(
        body,
        "edges",
        std::string(path) + ".edges",
        record.inlineVertices.size(),
        fixture);
    record.inlinePolygons = readInlinePolygons(
        body, "polygons", std::string(path) + ".polygons", fixture);
  }
  if (const std::optional<bool> oriented = readBoolMember(
          body, "oriented", std::string(path) + ".oriented", fixture);
      oriented.has_value()) {
    record.inlinePolygonsOriented = *oriented;
  }

  fixture.bodies.push_back(std::move(record));
}

void readRigidBodyProblem(
    const JsonValue::Object& root, RigidIpcFixture& fixture)
{
  const JsonValue* problemValue = findMember(root, "rigid_body_problem");
  if (problemValue == nullptr) {
    DART_SIMULATION_THROW_T(
        InvalidArgumentException, "Missing $.rigid_body_problem");
  }
  const JsonValue::Object& problem
      = requireObject(*problemValue, "$.rigid_body_problem");

  warnUnsupportedFields(
      problem,
      std::array<std::string_view, 4>{
          "coefficient_friction",
          "coefficient_restitution",
          "gravity",
          "rigid_bodies"},
      "$.rigid_body_problem",
      fixture);

  if (const std::optional<double> value = readDoubleMember(
          problem,
          "coefficient_friction",
          "$.rigid_body_problem.coefficient_friction",
          fixture);
      value.has_value()) {
    fixture.coefficientFriction = *value;
  }
  if (const std::optional<double> value = readDoubleMember(
          problem,
          "coefficient_restitution",
          "$.rigid_body_problem.coefficient_restitution",
          fixture);
      value.has_value()) {
    fixture.coefficientRestitution = *value;
  }
  if (const std::optional<Eigen::Vector3d> value = readVector3Member(
          problem, "gravity", "$.rigid_body_problem.gravity", 0, fixture);
      value.has_value()) {
    fixture.gravity = *value;
  }

  const JsonValue* bodiesValue = findMember(problem, "rigid_bodies");
  if (bodiesValue == nullptr) {
    DART_SIMULATION_THROW_T(
        InvalidArgumentException, "Missing $.rigid_body_problem.rigid_bodies");
  }

  const JsonValue::Array& bodies
      = requireArray(*bodiesValue, "$.rigid_body_problem.rigid_bodies");
  for (std::size_t i = 0; i < bodies.size(); ++i) {
    readBody(
        bodies[i],
        "$.rigid_body_problem.rigid_bodies[" + std::to_string(i) + "]",
        fixture);
  }
}

void readDistanceBarrierSettings(
    const JsonValue::Object& root, RigidIpcFixture& fixture)
{
  const JsonValue* value = findMember(root, "distance_barrier_constraint");
  if (value == nullptr) {
    return;
  }

  const JsonValue::Object& settings
      = requireObject(*value, "$.distance_barrier_constraint");
  warnUnsupportedFields(
      settings,
      std::array<std::string_view, 1>{"initial_barrier_activation_distance"},
      "$.distance_barrier_constraint",
      fixture);

  fixture.barrierActivationDistance = readDoubleMember(
      settings,
      "initial_barrier_activation_distance",
      "$.distance_barrier_constraint.initial_barrier_activation_distance",
      fixture);
}

void readSolverSettings(const JsonValue::Object& root, RigidIpcFixture& fixture)
{
  const JsonValue* value = findMember(root, "ipc_solver");
  if (value == nullptr) {
    return;
  }

  const JsonValue::Object& settings = requireObject(*value, "$.ipc_solver");
  warnUnsupportedFields(
      settings,
      std::array<std::string_view, 2>{
          "velocity_conv_tol", "is_velocity_conv_tol_abs"},
      "$.ipc_solver",
      fixture);

  fixture.velocityConvergenceTolerance = readDoubleMember(
      settings, "velocity_conv_tol", "$.ipc_solver.velocity_conv_tol", fixture);
  fixture.velocityConvergenceToleranceIsAbsolute = readBoolMember(
      settings,
      "is_velocity_conv_tol_abs",
      "$.ipc_solver.is_velocity_conv_tol_abs",
      fixture);
}

void readFrictionSettings(
    const JsonValue::Object& root, RigidIpcFixture& fixture)
{
  const JsonValue* value = findMember(root, "friction_constraints");
  if (value == nullptr) {
    return;
  }

  const JsonValue::Object& settings
      = requireObject(*value, "$.friction_constraints");
  warnUnsupportedFields(
      settings,
      std::array<std::string_view, 2>{
          "static_friction_speed_bound", "iterations"},
      "$.friction_constraints",
      fixture);

  fixture.staticFrictionSpeedBound = readDoubleMember(
      settings,
      "static_friction_speed_bound",
      "$.friction_constraints.static_friction_speed_bound",
      fixture);
  fixture.frictionIterations = readIntMember(
      settings, "iterations", "$.friction_constraints.iterations", fixture);
}

[[nodiscard]] RigidIpcPoseRecord readCcdPose(
    const JsonValue::Object& object,
    std::string_view key,
    std::string_view path,
    RigidIpcCcdCase& ccdCase)
{
  RigidIpcPoseRecord pose;

  const JsonValue* value = findMember(object, key);
  if (value == nullptr) {
    addError(ccdCase, std::string(path), "missing required pose field");
    return pose;
  }

  const JsonValue::Object& poseObject = requireObject(*value, path);
  warnUnsupportedFields(
      poseObject,
      std::array<std::string_view, 2>{"position", "rotation"},
      path,
      ccdCase);

  if (const std::optional<Eigen::Vector3d> position = readRequiredVector3Member(
          poseObject, "position", std::string(path) + ".position", 0, ccdCase);
      position.has_value()) {
    pose.position = *position;
  }
  if (const std::optional<Eigen::Vector3d> rotation = readRequiredVector3Member(
          poseObject, "rotation", std::string(path) + ".rotation", 0, ccdCase);
      rotation.has_value()) {
    pose.rotation = *rotation;
  }

  return pose;
}

template <std::size_t N>
[[nodiscard]] RigidIpcCcdBodyRecord readCcdBody(
    const JsonValue::Object& root,
    std::string_view sectionName,
    const std::array<std::string_view, N>& vertexKeys,
    RigidIpcCcdCase& ccdCase)
{
  RigidIpcCcdBodyRecord body;
  const std::string sectionPath = "$." + std::string(sectionName);

  const JsonValue* value = findMember(root, sectionName);
  if (value == nullptr) {
    addError(ccdCase, sectionPath, "missing required CCD primitive section");
    return body;
  }

  const JsonValue::Object& section = requireObject(*value, sectionPath);
  for (const auto& [key, member] : section) {
    (void)member;
    const std::string_view keyView(key);
    const bool allowed
        = keyView == "pose_t0" || keyView == "pose_t1"
          || std::find(vertexKeys.begin(), vertexKeys.end(), keyView)
                 != vertexKeys.end();
    if (!allowed) {
      addWarning(
          ccdCase,
          sectionPath + "." + key,
          "field is not imported by the current CCD data slice");
    }
  }

  body.poseT0
      = readCcdPose(section, "pose_t0", sectionPath + ".pose_t0", ccdCase);
  body.poseT1
      = readCcdPose(section, "pose_t1", sectionPath + ".pose_t1", ccdCase);
  body.vertices.reserve(vertexKeys.size());
  for (const std::string_view vertexKey : vertexKeys) {
    const std::string vertexPath = sectionPath + "." + std::string(vertexKey);
    if (const std::optional<Eigen::Vector3d> vertex
        = readRequiredVector3Member(section, vertexKey, vertexPath, 0, ccdCase);
        vertex.has_value()) {
      body.vertices.push_back(*vertex);
    }
  }

  return body;
}

void readCcdCaseBodyData(
    const JsonValue::Object& root, RigidIpcCcdCase& ccdCase)
{
  const std::optional<std::string> type
      = readStringMember(root, "type", "$.type", ccdCase);
  if (!type.has_value()) {
    addError(ccdCase, "$.type", "missing required CCD case type");
    return;
  }

  if (*type == "ev") {
    ccdCase.type = RigidIpcCcdCaseType::EdgeVertex;
    ccdCase.bodyA = readCcdBody(
        root,
        "edge",
        std::array<std::string_view, 2>{"vertex0", "vertex1"},
        ccdCase);
    ccdCase.bodyB = readCcdBody(
        root, "vertex", std::array<std::string_view, 1>{"vertex"}, ccdCase);
    return;
  }

  if (*type == "ee") {
    ccdCase.type = RigidIpcCcdCaseType::EdgeEdge;
    ccdCase.bodyA = readCcdBody(
        root,
        "edge0",
        std::array<std::string_view, 2>{"vertex0", "vertex1"},
        ccdCase);
    ccdCase.bodyB = readCcdBody(
        root,
        "edge1",
        std::array<std::string_view, 2>{"vertex0", "vertex1"},
        ccdCase);
    return;
  }

  if (*type == "fv") {
    ccdCase.type = RigidIpcCcdCaseType::FaceVertex;
    ccdCase.bodyA = readCcdBody(
        root,
        "face",
        std::array<std::string_view, 3>{"vertex0", "vertex1", "vertex2"},
        ccdCase);
    ccdCase.bodyB = readCcdBody(
        root, "vertex", std::array<std::string_view, 1>{"vertex"}, ccdCase);
    return;
  }

  addError(ccdCase, "$.type", "unsupported direct CCD case type: " + *type);
}

[[nodiscard]] rigid_detail::RigidIpcPose toRigidIpcPose(
    const RigidIpcPoseRecord& pose)
{
  return {pose.position, pose.rotation};
}

void requireCcdVertexCount(
    const RigidIpcCcdBodyRecord& body,
    const std::size_t expected,
    const std::string_view bodyName)
{
  if (body.vertices.size() != expected) {
    DART_SIMULATION_THROW_T(
        InvalidArgumentException,
        "Direct CCD {} primitive expected {} vertices, got {}",
        bodyName,
        expected,
        body.vertices.size());
  }
}

[[nodiscard]] Eigen::Vector3d degreesToRadians(const Eigen::Vector3d& degrees)
{
  constexpr double kDegreesToRadians = 0.017453292519943295;
  return kDegreesToRadians * degrees;
}

[[nodiscard]] bool allFixed(const std::vector<bool>& fixedDofs)
{
  return !fixedDofs.empty()
         && std::all_of(
             fixedDofs.begin(), fixedDofs.end(), [](const bool fixed) {
               return fixed;
             });
}

[[nodiscard]] std::string replayBodyName(
    std::string_view prefix, const std::size_t index)
{
  const std::string_view nonEmptyPrefix
      = prefix.empty() ? std::string_view("rigid_ipc_body") : prefix;
  return std::format("{}_{:03d}", nonEmptyPrefix, index);
}

[[nodiscard]] std::filesystem::path resolveMeshPath(
    const std::filesystem::path& assetRoot, const std::string& meshPath)
{
  if (meshPath.empty()) {
    return {};
  }

  std::filesystem::path resolved(meshPath);
  if (!resolved.is_absolute() && !assetRoot.empty()) {
    resolved = assetRoot / resolved;
  }
  return resolved.lexically_normal();
}

[[nodiscard]] bool pathExists(const std::filesystem::path& path)
{
  if (path.empty()) {
    return false;
  }

  std::error_code error;
  return std::filesystem::exists(path, error);
}

[[nodiscard]] Eigen::Matrix3d fixtureGeometryRotation(
    const Eigen::Vector3d& rotationDegrees)
{
  const Eigen::Vector3d rotation = degreesToRadians(rotationDegrees);
  return (Eigen::AngleAxisd(rotation.z(), Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(rotation.y(), Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(rotation.x(), Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
}

[[nodiscard]] std::string lowerExtension(const std::filesystem::path& path)
{
  std::string extension = path.extension().string();
  std::transform(
      extension.begin(), extension.end(), extension.begin(), [](char c) {
        return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      });
  return extension;
}

struct MeshLoadResult
{
  std::optional<CollisionShape> shape;
  std::size_t vertexCount{0};
  std::size_t triangleCount{0};
  std::string status;
};

[[nodiscard]] MeshLoadResult meshLoadFailure(std::string status)
{
  MeshLoadResult result;
  result.status = std::move(status);
  return result;
}

[[nodiscard]] std::optional<int> parseObjVertexIndex(
    std::string_view token, const int vertexCount)
{
  const std::size_t slash = token.find('/');
  token = token.substr(0, slash);
  if (token.empty()) {
    return std::nullopt;
  }

  const std::string text(token);
  errno = 0;
  char* end = nullptr;
  const long rawIndex = std::strtol(text.c_str(), &end, 10);
  if (errno != 0 || end != text.c_str() + text.size() || rawIndex == 0) {
    return std::nullopt;
  }

  const long zeroBased = rawIndex > 0 ? rawIndex - 1 : vertexCount + rawIndex;
  if (zeroBased < 0 || zeroBased >= vertexCount) {
    return std::nullopt;
  }

  return static_cast<int>(zeroBased);
}

[[nodiscard]] std::optional<std::size_t> parseCountToken(
    const std::string& token)
{
  if (token.empty() || token.front() == '-') {
    return std::nullopt;
  }

  errno = 0;
  char* end = nullptr;
  const unsigned long long value = std::strtoull(token.c_str(), &end, 10);
  if (errno != 0 || end != token.c_str() + token.size()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(value);
}

[[nodiscard]] std::optional<int> parseNonNegativeIndexToken(
    const std::string& token, const std::size_t vertexCount)
{
  errno = 0;
  char* end = nullptr;
  const long value = std::strtol(token.c_str(), &end, 10);
  if (errno != 0 || end != token.c_str() + token.size() || value < 0) {
    return std::nullopt;
  }
  if (static_cast<std::size_t>(value) >= vertexCount) {
    return std::nullopt;
  }
  return static_cast<int>(value);
}

[[nodiscard]] std::optional<std::string> readMeshToken(std::istream& input)
{
  std::string token;
  while (input >> token) {
    if (!token.empty() && token.front() == '#') {
      std::string ignored;
      std::getline(input, ignored);
      continue;
    }
    return token;
  }
  return std::nullopt;
}

void applyFixtureGeometryTransform(
    std::vector<Eigen::Vector3d>& vertices,
    const Eigen::Vector3d& scale,
    const Eigen::Vector3d& rotationDegrees)
{
  const Eigen::Matrix3d rotation = fixtureGeometryRotation(rotationDegrees);
  for (Eigen::Vector3d& vertex : vertices) {
    vertex = rotation * vertex.cwiseProduct(scale);
  }
}

[[nodiscard]] MeshLoadResult makeLoadedMeshResult(
    std::vector<Eigen::Vector3d> vertices,
    std::vector<Eigen::Vector3i> triangles,
    const Eigen::Vector3d& scale,
    const Eigen::Vector3d& rotationDegrees)
{
  applyFixtureGeometryTransform(vertices, scale, rotationDegrees);

  MeshLoadResult result;
  result.vertexCount = vertices.size();
  result.triangleCount = triangles.size();
  result.status = "loaded";
  result.shape
      = CollisionShape::makeMesh(std::move(vertices), std::move(triangles));
  return result;
}

[[nodiscard]] MeshLoadResult loadObjCollisionMesh(
    const std::filesystem::path& path,
    const Eigen::Vector3d& scale,
    const Eigen::Vector3d& rotationDegrees)
{
  std::ifstream input(path);
  if (!input) {
    return meshLoadFailure("could not open mesh file");
  }

  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;

  std::string line;
  std::size_t lineNumber = 0;
  while (std::getline(input, line)) {
    ++lineNumber;
    if (const std::size_t comment = line.find('#');
        comment != std::string::npos) {
      line.erase(comment);
    }

    std::istringstream lineInput(line);
    std::string tag;
    lineInput >> tag;
    if (tag.empty()) {
      continue;
    }

    if (tag == "v") {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      if (!(lineInput >> x >> y >> z)) {
        return meshLoadFailure(
            std::format("invalid OBJ vertex on line {}", lineNumber));
      }

      const Eigen::Vector3d vertex(x, y, z);
      if (!vertex.allFinite()) {
        return meshLoadFailure(
            std::format("non-finite OBJ vertex on line {}", lineNumber));
      }
      vertices.push_back(vertex);
      continue;
    }

    if (tag == "f") {
      std::vector<int> face;
      std::string token;
      while (lineInput >> token) {
        const std::optional<int> index
            = parseObjVertexIndex(token, static_cast<int>(vertices.size()));
        if (!index.has_value()) {
          return meshLoadFailure(
              std::format("invalid OBJ face index on line {}", lineNumber));
        }
        face.push_back(*index);
      }

      if (face.size() < 3u) {
        return meshLoadFailure(
            std::format(
                "OBJ face on line {} has fewer than three vertices",
                lineNumber));
      }
      for (std::size_t i = 1; i + 1 < face.size(); ++i) {
        triangles.emplace_back(face[0], face[i], face[i + 1]);
      }
    }
  }

  if (vertices.empty()) {
    return meshLoadFailure("OBJ mesh contains no vertices");
  }
  if (triangles.empty()) {
    return meshLoadFailure("OBJ mesh contains no triangles");
  }

  return makeLoadedMeshResult(
      std::move(vertices), std::move(triangles), scale, rotationDegrees);
}

[[nodiscard]] std::optional<double> parseDoubleToken(const std::string& token)
{
  errno = 0;
  char* end = nullptr;
  const double value = std::strtod(token.c_str(), &end);
  if (errno != 0 || end != token.c_str() + token.size()) {
    return std::nullopt;
  }
  return value;
}

[[nodiscard]] MeshLoadResult loadOffCollisionMesh(
    const std::filesystem::path& path,
    const Eigen::Vector3d& scale,
    const Eigen::Vector3d& rotationDegrees)
{
  std::ifstream input(path);
  if (!input) {
    return meshLoadFailure("could not open mesh file");
  }

  const std::optional<std::string> magic = readMeshToken(input);
  if (!magic.has_value() || *magic != "OFF") {
    return meshLoadFailure("OFF mesh missing OFF header");
  }

  const std::optional<std::string> vertexCountToken = readMeshToken(input);
  const std::optional<std::string> faceCountToken = readMeshToken(input);
  const std::optional<std::string> edgeCountToken = readMeshToken(input);
  if (!vertexCountToken.has_value() || !faceCountToken.has_value()
      || !edgeCountToken.has_value()) {
    return meshLoadFailure("OFF mesh missing counts");
  }

  const std::optional<std::size_t> vertexCount
      = parseCountToken(*vertexCountToken);
  const std::optional<std::size_t> faceCount = parseCountToken(*faceCountToken);
  const std::optional<std::size_t> edgeCount = parseCountToken(*edgeCountToken);
  if (!vertexCount.has_value() || !faceCount.has_value()
      || !edgeCount.has_value()) {
    return meshLoadFailure("OFF mesh has invalid counts");
  }
  if (*vertexCount == 0u) {
    return meshLoadFailure("OFF mesh contains no vertices");
  }

  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(*vertexCount);
  for (std::size_t i = 0; i < *vertexCount; ++i) {
    const std::optional<std::string> xToken = readMeshToken(input);
    const std::optional<std::string> yToken = readMeshToken(input);
    const std::optional<std::string> zToken = readMeshToken(input);
    if (!xToken.has_value() || !yToken.has_value() || !zToken.has_value()) {
      return meshLoadFailure("OFF mesh ended while reading vertices");
    }

    const std::optional<double> x = parseDoubleToken(*xToken);
    const std::optional<double> y = parseDoubleToken(*yToken);
    const std::optional<double> z = parseDoubleToken(*zToken);
    if (!x.has_value() || !y.has_value() || !z.has_value()) {
      return meshLoadFailure("OFF mesh has an invalid vertex coordinate");
    }

    const Eigen::Vector3d vertex(*x, *y, *z);
    if (!vertex.allFinite()) {
      return meshLoadFailure("OFF mesh has a non-finite vertex");
    }
    vertices.push_back(vertex);
  }

  std::vector<Eigen::Vector3i> triangles;
  for (std::size_t faceIndex = 0; faceIndex < *faceCount; ++faceIndex) {
    const std::optional<std::string> faceSizeToken = readMeshToken(input);
    if (!faceSizeToken.has_value()) {
      return meshLoadFailure("OFF mesh ended while reading faces");
    }
    const std::optional<std::size_t> faceSize = parseCountToken(*faceSizeToken);
    if (!faceSize.has_value()) {
      return meshLoadFailure("OFF mesh has an invalid face size");
    }

    std::vector<int> face;
    face.reserve(*faceSize);
    for (std::size_t i = 0; i < *faceSize; ++i) {
      const std::optional<std::string> indexToken = readMeshToken(input);
      if (!indexToken.has_value()) {
        return meshLoadFailure("OFF mesh ended while reading face indices");
      }
      const std::optional<int> index
          = parseNonNegativeIndexToken(*indexToken, vertices.size());
      if (!index.has_value()) {
        return meshLoadFailure("OFF mesh has an invalid face index");
      }
      face.push_back(*index);
    }

    if (face.size() < 3u) {
      return meshLoadFailure("OFF mesh has a face with fewer than 3 vertices");
    }
    for (std::size_t i = 1; i + 1 < face.size(); ++i) {
      triangles.emplace_back(face[0], face[i], face[i + 1]);
    }
  }

  if (triangles.empty()) {
    return meshLoadFailure("OFF mesh contains no triangles");
  }

  return makeLoadedMeshResult(
      std::move(vertices), std::move(triangles), scale, rotationDegrees);
}

[[nodiscard]] MeshLoadResult loadMshCollisionMesh(
    const std::filesystem::path& path,
    const Eigen::Vector3d& scale,
    const Eigen::Vector3d& rotationDegrees)
{
  std::ifstream input(path);
  if (!input) {
    return meshLoadFailure("could not open mesh file");
  }

  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
  std::map<std::size_t, int> nodeTagToIndex;
  bool loadedNodes = false;
  bool loadedSurface = false;

  std::optional<std::string> token;
  while ((token = readMeshToken(input)).has_value()) {
    if (*token == "$Nodes") {
      const std::optional<std::string> blockCountToken = readMeshToken(input);
      const std::optional<std::string> nodeCountToken = readMeshToken(input);
      if (!blockCountToken.has_value() || !nodeCountToken.has_value()) {
        return meshLoadFailure("MSH mesh missing node counts");
      }

      const std::optional<std::size_t> blockCount
          = parseCountToken(*blockCountToken);
      const std::optional<std::size_t> nodeCount
          = parseCountToken(*nodeCountToken);
      if (!blockCount.has_value() || !nodeCount.has_value()) {
        return meshLoadFailure("MSH mesh has invalid node counts");
      }
      if (*nodeCount == 0u) {
        return meshLoadFailure("MSH mesh contains no vertices");
      }

      vertices.clear();
      nodeTagToIndex.clear();
      vertices.reserve(*nodeCount);
      for (std::size_t blockIndex = 0; blockIndex < *blockCount; ++blockIndex) {
        const std::optional<std::string> entityDimToken = readMeshToken(input);
        const std::optional<std::string> entityTagToken = readMeshToken(input);
        const std::optional<std::string> parametricToken = readMeshToken(input);
        const std::optional<std::string> blockNodeCountToken
            = readMeshToken(input);
        if (!entityDimToken.has_value() || !entityTagToken.has_value()
            || !parametricToken.has_value()
            || !blockNodeCountToken.has_value()) {
          return meshLoadFailure("MSH mesh missing node-block header");
        }

        const std::optional<std::size_t> parametric
            = parseCountToken(*parametricToken);
        const std::optional<std::size_t> blockNodeCount
            = parseCountToken(*blockNodeCountToken);
        if (!parseCountToken(*entityDimToken).has_value()
            || !parseCountToken(*entityTagToken).has_value()
            || !parametric.has_value() || !blockNodeCount.has_value()) {
          return meshLoadFailure("MSH mesh has invalid node-block header");
        }
        if (*parametric != 0u) {
          return meshLoadFailure("MSH parametric nodes are unsupported");
        }

        for (std::size_t i = 0; i < *blockNodeCount; ++i) {
          const std::optional<std::string> tagToken = readMeshToken(input);
          const std::optional<std::string> xToken = readMeshToken(input);
          const std::optional<std::string> yToken = readMeshToken(input);
          const std::optional<std::string> zToken = readMeshToken(input);
          if (!tagToken.has_value() || !xToken.has_value()
              || !yToken.has_value() || !zToken.has_value()) {
            return meshLoadFailure("MSH mesh ended while reading vertices");
          }

          const std::optional<std::size_t> tag = parseCountToken(*tagToken);
          const std::optional<double> x = parseDoubleToken(*xToken);
          const std::optional<double> y = parseDoubleToken(*yToken);
          const std::optional<double> z = parseDoubleToken(*zToken);
          if (!tag.has_value() || *tag == 0u || !x.has_value() || !y.has_value()
              || !z.has_value()) {
            return meshLoadFailure("MSH mesh has an invalid vertex");
          }

          const Eigen::Vector3d vertex(*x, *y, *z);
          if (!vertex.allFinite()) {
            return meshLoadFailure("MSH mesh has a non-finite vertex");
          }

          const int vertexIndex = static_cast<int>(vertices.size());
          if (!nodeTagToIndex.emplace(*tag, vertexIndex).second) {
            return meshLoadFailure("MSH mesh has a duplicate node tag");
          }
          vertices.push_back(vertex);
        }
      }

      if (vertices.size() != *nodeCount) {
        return meshLoadFailure("MSH mesh node count mismatch");
      }

      const std::optional<std::string> endToken = readMeshToken(input);
      if (!endToken.has_value() || *endToken != "$EndNodes") {
        return meshLoadFailure("MSH mesh missing $EndNodes");
      }
      loadedNodes = true;
      continue;
    }

    if (*token == "$Surface") {
      if (!loadedNodes) {
        return meshLoadFailure("MSH surface appears before nodes");
      }

      const std::optional<std::string> surfaceTriangleCountToken
          = readMeshToken(input);
      if (!surfaceTriangleCountToken.has_value()) {
        return meshLoadFailure("MSH mesh missing surface triangle count");
      }

      const std::optional<std::size_t> surfaceTriangleCount
          = parseCountToken(*surfaceTriangleCountToken);
      if (!surfaceTriangleCount.has_value()) {
        return meshLoadFailure("MSH mesh has invalid surface triangle count");
      }

      triangles.clear();
      triangles.reserve(*surfaceTriangleCount);
      for (std::size_t triangleIndex = 0; triangleIndex < *surfaceTriangleCount;
           ++triangleIndex) {
        std::array<int, 3> face{};
        for (int i = 0; i < 3; ++i) {
          const std::optional<std::string> tagToken = readMeshToken(input);
          if (!tagToken.has_value()) {
            return meshLoadFailure(
                "MSH mesh ended while reading surface triangles");
          }

          const std::optional<std::size_t> tag = parseCountToken(*tagToken);
          if (!tag.has_value()) {
            return meshLoadFailure("MSH mesh has an invalid surface node tag");
          }

          const auto node = nodeTagToIndex.find(*tag);
          if (node == nodeTagToIndex.end()) {
            return meshLoadFailure("MSH surface references unknown node tag");
          }
          face[static_cast<std::size_t>(i)] = node->second;
        }
        triangles.emplace_back(face[0], face[1], face[2]);
      }

      const std::optional<std::string> endToken = readMeshToken(input);
      if (!endToken.has_value() || *endToken != "$EndSurface") {
        return meshLoadFailure("MSH mesh missing $EndSurface");
      }
      loadedSurface = true;
      continue;
    }
  }

  if (!loadedNodes) {
    return meshLoadFailure("MSH mesh missing $Nodes");
  }
  if (!loadedSurface) {
    return meshLoadFailure("MSH mesh missing $Surface");
  }
  if (triangles.empty()) {
    return meshLoadFailure("MSH mesh contains no surface triangles");
  }

  return makeLoadedMeshResult(
      std::move(vertices), std::move(triangles), scale, rotationDegrees);
}

[[nodiscard]] std::optional<std::uint32_t> readLittleEndianUInt32(
    std::istream& input)
{
  std::array<unsigned char, 4> bytes{};
  input.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
  if (!input) {
    return std::nullopt;
  }
  return static_cast<std::uint32_t>(bytes[0])
         | (static_cast<std::uint32_t>(bytes[1]) << 8u)
         | (static_cast<std::uint32_t>(bytes[2]) << 16u)
         | (static_cast<std::uint32_t>(bytes[3]) << 24u);
}

[[nodiscard]] std::optional<float> readLittleEndianFloat(std::istream& input)
{
  const std::optional<std::uint32_t> raw = readLittleEndianUInt32(input);
  if (!raw.has_value()) {
    return std::nullopt;
  }
  return std::bit_cast<float>(*raw);
}

[[nodiscard]] MeshLoadResult loadBinaryStlCollisionMesh(
    const std::filesystem::path& path,
    const Eigen::Vector3d& scale,
    const Eigen::Vector3d& rotationDegrees)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return meshLoadFailure("could not open mesh file");
  }

  std::array<char, 80> header{};
  input.read(header.data(), header.size());
  if (!input) {
    return meshLoadFailure("STL mesh missing binary header");
  }

  const std::optional<std::uint32_t> triangleCount
      = readLittleEndianUInt32(input);
  if (!triangleCount.has_value()) {
    return meshLoadFailure("STL mesh missing triangle count");
  }

  std::error_code fileSizeError;
  const std::uintmax_t fileSize
      = std::filesystem::file_size(path, fileSizeError);
  if (!fileSizeError) {
    constexpr std::uintmax_t kBinaryStlHeaderSize = 84u;
    constexpr std::uintmax_t kBinaryStlTriangleSize = 50u;
    const std::uintmax_t expectedSize
        = kBinaryStlHeaderSize
          + (static_cast<std::uintmax_t>(*triangleCount)
             * kBinaryStlTriangleSize);
    if (fileSize != expectedSize) {
      return meshLoadFailure("STL mesh is not exact binary STL");
    }
  }

  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
  vertices.reserve(static_cast<std::size_t>(*triangleCount) * 3u);
  triangles.reserve(*triangleCount);

  for (std::uint32_t triangleIndex = 0; triangleIndex < *triangleCount;
       ++triangleIndex) {
    for (int i = 0; i < 3; ++i) {
      if (!readLittleEndianFloat(input).has_value()) {
        return meshLoadFailure("STL mesh ended while reading normals");
      }
    }

    std::array<Eigen::Vector3d, 3> triangleVertices;
    for (Eigen::Vector3d& vertex : triangleVertices) {
      std::array<float, 3> coordinates{};
      for (float& coordinate : coordinates) {
        const std::optional<float> value = readLittleEndianFloat(input);
        if (!value.has_value()) {
          return meshLoadFailure("STL mesh ended while reading vertices");
        }
        coordinate = *value;
      }
      vertex = Eigen::Vector3d(
          static_cast<double>(coordinates[0]),
          static_cast<double>(coordinates[1]),
          static_cast<double>(coordinates[2]));
      if (!vertex.allFinite()) {
        return meshLoadFailure("STL mesh has a non-finite vertex");
      }
    }

    std::array<char, 2> attributeByteCount{};
    input.read(attributeByteCount.data(), attributeByteCount.size());
    if (!input) {
      return meshLoadFailure("STL mesh ended while reading attributes");
    }

    const int base = static_cast<int>(vertices.size());
    vertices.push_back(triangleVertices[0]);
    vertices.push_back(triangleVertices[1]);
    vertices.push_back(triangleVertices[2]);
    triangles.emplace_back(base, base + 1, base + 2);
  }

  if (triangles.empty()) {
    return meshLoadFailure("STL mesh contains no triangles");
  }

  return makeLoadedMeshResult(
      std::move(vertices), std::move(triangles), scale, rotationDegrees);
}

[[nodiscard]] MeshLoadResult loadAsciiStlCollisionMesh(
    const std::filesystem::path& path,
    const Eigen::Vector3d& scale,
    const Eigen::Vector3d& rotationDegrees)
{
  std::ifstream input(path);
  if (!input) {
    return meshLoadFailure("could not open mesh file");
  }

  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
  std::array<Eigen::Vector3d, 3> triangleVertices;

  std::string line;
  std::size_t lineNumber = 0;
  std::size_t triangleVertexCount = 0;
  while (std::getline(input, line)) {
    ++lineNumber;
    std::istringstream lineInput(line);
    std::string tag;
    lineInput >> tag;
    if (tag != "vertex") {
      continue;
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!(lineInput >> x >> y >> z)) {
      return meshLoadFailure(
          std::format("invalid ASCII STL vertex on line {}", lineNumber));
    }

    const Eigen::Vector3d vertex(x, y, z);
    if (!vertex.allFinite()) {
      return meshLoadFailure(
          std::format("non-finite ASCII STL vertex on line {}", lineNumber));
    }

    triangleVertices[triangleVertexCount] = vertex;
    ++triangleVertexCount;
    if (triangleVertexCount == 3u) {
      const int base = static_cast<int>(vertices.size());
      vertices.push_back(triangleVertices[0]);
      vertices.push_back(triangleVertices[1]);
      vertices.push_back(triangleVertices[2]);
      triangles.emplace_back(base, base + 1, base + 2);
      triangleVertexCount = 0;
    }
  }

  if (triangleVertexCount != 0u) {
    return meshLoadFailure("ASCII STL mesh ended inside a triangle");
  }
  if (triangles.empty()) {
    return meshLoadFailure("ASCII STL mesh contains no triangles");
  }

  return makeLoadedMeshResult(
      std::move(vertices), std::move(triangles), scale, rotationDegrees);
}

[[nodiscard]] MeshLoadResult loadStlCollisionMesh(
    const std::filesystem::path& path,
    const Eigen::Vector3d& scale,
    const Eigen::Vector3d& rotationDegrees)
{
  MeshLoadResult binaryResult
      = loadBinaryStlCollisionMesh(path, scale, rotationDegrees);
  if (binaryResult.shape.has_value()) {
    return binaryResult;
  }

  MeshLoadResult asciiResult
      = loadAsciiStlCollisionMesh(path, scale, rotationDegrees);
  if (asciiResult.shape.has_value()) {
    return asciiResult;
  }

  return binaryResult.status == "STL mesh is not exact binary STL"
             ? std::move(asciiResult)
             : std::move(binaryResult);
}

enum class VtkEncoding
{
  Ascii,
  Binary
};

[[nodiscard]] std::string uppercaseAscii(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(), [](char c) {
    return static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  });
  return text;
}

void consumeVtkBinaryDataDelimiter(std::istream& input)
{
  const int next = input.peek();
  if (next == std::char_traits<char>::eof()
      || !std::isspace(static_cast<unsigned char>(next))) {
    return;
  }

  input.get();
  if (next == '\r' && input.peek() == '\n') {
    input.get();
  }
}

[[nodiscard]] std::optional<std::uint32_t> readBigEndianUInt32(
    std::istream& input)
{
  std::array<unsigned char, 4> bytes{};
  input.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
  if (!input) {
    return std::nullopt;
  }
  return (static_cast<std::uint32_t>(bytes[0]) << 24u)
         | (static_cast<std::uint32_t>(bytes[1]) << 16u)
         | (static_cast<std::uint32_t>(bytes[2]) << 8u)
         | static_cast<std::uint32_t>(bytes[3]);
}

[[nodiscard]] std::optional<std::uint64_t> readBigEndianUInt64(
    std::istream& input)
{
  std::array<unsigned char, 8> bytes{};
  input.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
  if (!input) {
    return std::nullopt;
  }

  std::uint64_t value = 0u;
  for (const unsigned char byte : bytes) {
    value = (value << 8u) | static_cast<std::uint64_t>(byte);
  }
  return value;
}

[[nodiscard]] std::optional<double> readVtkScalar(
    std::istream& input, const VtkEncoding encoding, const std::string& type)
{
  if (encoding == VtkEncoding::Ascii) {
    double value = 0.0;
    if (!(input >> value) || !std::isfinite(value)) {
      return std::nullopt;
    }
    return value;
  }

  if (type == "float") {
    const std::optional<std::uint32_t> raw = readBigEndianUInt32(input);
    if (!raw.has_value()) {
      return std::nullopt;
    }
    const float value = std::bit_cast<float>(*raw);
    if (!std::isfinite(value)) {
      return std::nullopt;
    }
    return static_cast<double>(value);
  }

  if (type == "double") {
    const std::optional<std::uint64_t> raw = readBigEndianUInt64(input);
    if (!raw.has_value()) {
      return std::nullopt;
    }
    const double value = std::bit_cast<double>(*raw);
    if (!std::isfinite(value)) {
      return std::nullopt;
    }
    return value;
  }

  return std::nullopt;
}

[[nodiscard]] std::optional<int> readVtkInt(
    std::istream& input, const VtkEncoding encoding)
{
  if (encoding == VtkEncoding::Ascii) {
    long value = 0;
    if (!(input >> value)
        || value < static_cast<long>(std::numeric_limits<int>::min())
        || value > static_cast<long>(std::numeric_limits<int>::max())) {
      return std::nullopt;
    }
    return static_cast<int>(value);
  }

  const std::optional<std::uint32_t> raw = readBigEndianUInt32(input);
  if (!raw.has_value()) {
    return std::nullopt;
  }
  return std::bit_cast<std::int32_t>(*raw);
}

void appendVtkCellTriangles(
    const std::vector<int>& cell,
    const int cellType,
    const std::size_t vertexCount,
    std::vector<Eigen::Vector3i>& triangles)
{
  constexpr int kVtkTriangle = 5;
  constexpr int kVtkPolygon = 7;
  constexpr int kVtkQuad = 9;

  if (cellType != kVtkTriangle && cellType != kVtkPolygon
      && cellType != kVtkQuad) {
    return;
  }
  if (cellType == kVtkTriangle && cell.size() != 3u) {
    return;
  }
  if (cellType == kVtkQuad && cell.size() != 4u) {
    return;
  }
  if (cell.size() < 3u) {
    return;
  }

  for (const int index : cell) {
    if (index < 0 || static_cast<std::size_t>(index) >= vertexCount) {
      return;
    }
  }

  for (std::size_t i = 1u; i + 1u < cell.size(); ++i) {
    triangles.emplace_back(cell[0], cell[i], cell[i + 1u]);
  }
}

[[nodiscard]] MeshLoadResult loadVtkCollisionMesh(
    const std::filesystem::path& path,
    const Eigen::Vector3d& scale,
    const Eigen::Vector3d& rotationDegrees)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return meshLoadFailure("could not open mesh file");
  }

  std::string header;
  std::string title;
  std::string encodingLine;
  if (!std::getline(input, header) || !std::getline(input, title)
      || !std::getline(input, encodingLine)) {
    return meshLoadFailure("VTK mesh missing legacy header");
  }
  (void)title;
  if (header.find("vtk DataFile") == std::string::npos) {
    return meshLoadFailure("VTK mesh missing legacy signature");
  }

  std::string encodingToken = uppercaseAscii(encodingLine);
  encodingToken.erase(
      std::remove_if(
          encodingToken.begin(),
          encodingToken.end(),
          [](char c) {
            return std::isspace(static_cast<unsigned char>(c)) != 0;
          }),
      encodingToken.end());
  VtkEncoding encoding = VtkEncoding::Ascii;
  if (encodingToken == "ASCII") {
    encoding = VtkEncoding::Ascii;
  } else if (encodingToken == "BINARY") {
    encoding = VtkEncoding::Binary;
  } else {
    return meshLoadFailure("VTK mesh has unsupported encoding");
  }

  std::string datasetKeyword;
  std::string datasetType;
  if (!(input >> datasetKeyword >> datasetType)
      || uppercaseAscii(datasetKeyword) != "DATASET"
      || uppercaseAscii(datasetType) != "UNSTRUCTURED_GRID") {
    return meshLoadFailure("VTK mesh must be an unstructured grid");
  }

  std::string pointsKeyword;
  std::size_t pointCount = 0u;
  std::string scalarType;
  if (!(input >> pointsKeyword >> pointCount >> scalarType)
      || uppercaseAscii(pointsKeyword) != "POINTS") {
    return meshLoadFailure("VTK mesh missing POINTS section");
  }
  scalarType = uppercaseAscii(scalarType);
  std::transform(
      scalarType.begin(), scalarType.end(), scalarType.begin(), [](char c) {
        return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      });
  if (scalarType != "float" && scalarType != "double") {
    return meshLoadFailure("VTK mesh POINTS section must use float or double");
  }

  if (encoding == VtkEncoding::Binary) {
    consumeVtkBinaryDataDelimiter(input);
  }

  std::vector<Eigen::Vector3d> vertices;
  vertices.reserve(pointCount);
  for (std::size_t i = 0u; i < pointCount; ++i) {
    Eigen::Vector3d vertex = Eigen::Vector3d::Zero();
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
      const std::optional<double> value
          = readVtkScalar(input, encoding, scalarType);
      if (!value.has_value()) {
        return meshLoadFailure("VTK mesh ended while reading points");
      }
      vertex[axis] = *value;
    }
    vertices.push_back(vertex);
  }

  std::string cellsKeyword;
  std::size_t cellCount = 0u;
  std::size_t cellIndexCount = 0u;
  if (!(input >> cellsKeyword >> cellCount >> cellIndexCount)
      || uppercaseAscii(cellsKeyword) != "CELLS") {
    return meshLoadFailure("VTK mesh missing CELLS section");
  }
  if (cellIndexCount < cellCount) {
    return meshLoadFailure("VTK mesh has invalid CELLS index count");
  }

  if (encoding == VtkEncoding::Binary) {
    consumeVtkBinaryDataDelimiter(input);
  }

  std::vector<std::vector<int>> cells;
  cells.reserve(cellCount);
  std::size_t consumedCellIndices = 0u;
  for (std::size_t i = 0u; i < cellCount; ++i) {
    const std::optional<int> size = readVtkInt(input, encoding);
    if (!size.has_value() || *size < 0) {
      return meshLoadFailure("VTK mesh has invalid cell size");
    }
    consumedCellIndices += 1u + static_cast<std::size_t>(*size);
    if (consumedCellIndices > cellIndexCount) {
      return meshLoadFailure("VTK mesh has too many cell indices");
    }

    std::vector<int> cell;
    cell.reserve(static_cast<std::size_t>(*size));
    for (int index = 0; index < *size; ++index) {
      const std::optional<int> vertexIndex = readVtkInt(input, encoding);
      if (!vertexIndex.has_value()) {
        return meshLoadFailure("VTK mesh ended while reading cells");
      }
      cell.push_back(*vertexIndex);
    }
    cells.push_back(std::move(cell));
  }
  if (consumedCellIndices != cellIndexCount) {
    return meshLoadFailure("VTK mesh has unused cell indices");
  }

  std::string cellTypesKeyword;
  std::size_t cellTypeCount = 0u;
  if (!(input >> cellTypesKeyword >> cellTypeCount)
      || uppercaseAscii(cellTypesKeyword) != "CELL_TYPES") {
    return meshLoadFailure("VTK mesh missing CELL_TYPES section");
  }
  if (cellTypeCount != cellCount) {
    return meshLoadFailure("VTK mesh CELL_TYPES count mismatch");
  }

  if (encoding == VtkEncoding::Binary) {
    consumeVtkBinaryDataDelimiter(input);
  }

  std::vector<Eigen::Vector3i> triangles;
  for (std::size_t i = 0u; i < cellTypeCount; ++i) {
    const std::optional<int> cellType = readVtkInt(input, encoding);
    if (!cellType.has_value()) {
      return meshLoadFailure("VTK mesh ended while reading cell types");
    }
    appendVtkCellTriangles(cells[i], *cellType, vertices.size(), triangles);
  }

  if (vertices.empty()) {
    return meshLoadFailure("VTK mesh contains no points");
  }
  if (triangles.empty()) {
    return meshLoadFailure("VTK mesh contains no supported surface cells");
  }

  return makeLoadedMeshResult(
      std::move(vertices), std::move(triangles), scale, rotationDegrees);
}

[[nodiscard]] MeshLoadResult loadCollisionMesh(
    const std::filesystem::path& path,
    const Eigen::Vector3d& scale,
    const Eigen::Vector3d& rotationDegrees)
{
  if (path.empty()) {
    return meshLoadFailure("empty mesh path");
  }
  if (!pathExists(path)) {
    return meshLoadFailure("mesh file not found");
  }

  const std::string extension = lowerExtension(path);
  if (extension == ".obj") {
    return loadObjCollisionMesh(path, scale, rotationDegrees);
  }
  if (extension == ".off") {
    return loadOffCollisionMesh(path, scale, rotationDegrees);
  }
  if (extension == ".msh") {
    return loadMshCollisionMesh(path, scale, rotationDegrees);
  }
  if (extension == ".stl") {
    return loadStlCollisionMesh(path, scale, rotationDegrees);
  }
  if (extension == ".vtk") {
    return loadVtkCollisionMesh(path, scale, rotationDegrees);
  }

  return meshLoadFailure(
      std::format("unsupported mesh extension '{}'", extension));
}

[[nodiscard]] std::vector<std::string> splitComparisonScriptLine(
    std::string line)
{
  if (const std::size_t comment = line.find('#');
      comment != std::string::npos) {
    line.erase(comment);
  }

  std::istringstream input(line);
  std::vector<std::string> tokens;
  std::string token;
  while (input >> token) {
    tokens.push_back(std::move(token));
  }
  return tokens;
}

[[nodiscard]] std::optional<double> readComparisonDoubleToken(
    const std::string& token, RigidIpcFixture& fixture, std::string path)
{
  const std::optional<double> value = parseDoubleToken(token);
  if (!value.has_value() || !std::isfinite(*value)) {
    addError(fixture, std::move(path), "expected finite numeric token");
    return std::nullopt;
  }
  return value;
}

[[nodiscard]] std::optional<int> readComparisonIntToken(
    const std::string& token, RigidIpcFixture& fixture, std::string path)
{
  const std::optional<std::size_t> value = parseCountToken(token);
  if (!value.has_value()
      || *value > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    addError(fixture, std::move(path), "expected non-negative integer token");
    return std::nullopt;
  }
  return static_cast<int>(*value);
}

[[nodiscard]] std::optional<bool> readComparisonBoolToken(
    const std::string& token, RigidIpcFixture& fixture, std::string path)
{
  if (token == "true" || token == "on" || token == "1") {
    return true;
  }
  if (token == "false" || token == "off" || token == "0") {
    return false;
  }

  addError(fixture, std::move(path), "expected boolean token");
  return std::nullopt;
}

bool readComparisonVector3(
    const std::vector<std::string>& tokens,
    const std::size_t offset,
    RigidIpcFixture& fixture,
    const std::string& path,
    Eigen::Vector3d& output)
{
  if (tokens.size() < offset + 3u) {
    addError(fixture, path, "expected three numeric components");
    return false;
  }

  Eigen::Vector3d value = Eigen::Vector3d::Zero();
  for (std::size_t i = 0; i < 3u; ++i) {
    const std::optional<double> component = readComparisonDoubleToken(
        tokens[offset + i], fixture, path + "[" + std::to_string(i) + "]");
    if (!component.has_value()) {
      return false;
    }
    value[static_cast<Eigen::Index>(i)] = *component;
  }

  output = value;
  return true;
}

void readComparisonScriptBody(
    const std::vector<std::string>& tokens,
    const std::size_t lineNumber,
    RigidIpcFixture& fixture)
{
  const std::string path = std::format("$.line{}", lineNumber);
  if (tokens.size() < 10u) {
    addError(
        fixture,
        path,
        "comparison script body row needs mesh, pose, rotation, and scale");
    return;
  }

  RigidIpcBodyRecord record;
  record.meshPath = tokens[0];
  readComparisonVector3(
      tokens, 1u, fixture, path + ".position", record.position);
  readComparisonVector3(
      tokens, 4u, fixture, path + ".rotation", record.rotationDegrees);
  readComparisonVector3(tokens, 7u, fixture, path + ".scale", record.scale);

  for (std::size_t i = 10u; i < tokens.size();) {
    const std::string& command = tokens[i];
    const std::string commandPath = path + "." + command;

    if (command == "material") {
      if (tokens.size() < i + 4u) {
        addError(
            fixture, commandPath, "material needs density, Young, and Poisson");
        break;
      }
      record.density
          = readComparisonDoubleToken(tokens[i + 1u], fixture, commandPath);
      record.youngModulus = readComparisonDoubleToken(
          tokens[i + 2u], fixture, commandPath + ".young");
      record.poissonsRatio = readComparisonDoubleToken(
          tokens[i + 3u], fixture, commandPath + ".poisson");
      i += 4u;
      continue;
    }

    if (command == "initVel") {
      if (tokens.size() < i + 7u) {
        addError(
            fixture,
            commandPath,
            "initVel needs linear and angular velocity components");
        break;
      }
      readComparisonVector3(
          tokens,
          i + 1u,
          fixture,
          commandPath + ".linear",
          record.linearVelocity);
      readComparisonVector3(
          tokens,
          i + 4u,
          fixture,
          commandPath + ".angular",
          record.angularVelocity);
      i += 7u;
      continue;
    }

    if (command == "linearVelocity") {
      if (tokens.size() < i + 4u) {
        addError(fixture, commandPath, "linearVelocity needs three components");
        break;
      }
      readComparisonVector3(
          tokens, i + 1u, fixture, commandPath, record.linearVelocity);
      record.mode = RigidIpcBodyMode::Kinematic;
      i += 4u;
      continue;
    }

    if (command == "angularVelocity") {
      if (tokens.size() < i + 4u) {
        addError(
            fixture, commandPath, "angularVelocity needs three components");
        break;
      }
      readComparisonVector3(
          tokens, i + 1u, fixture, commandPath, record.angularVelocity);
      record.mode = RigidIpcBodyMode::Kinematic;
      i += 4u;
      continue;
    }

    if (command == "NBC") {
      if (tokens.size() < i + 10u) {
        addError(
            fixture,
            commandPath,
            "NBC needs min bounds, max bounds, and force components");
        break;
      }
      readComparisonVector3(tokens, i + 7u, fixture, commandPath, record.force);
      i += 10u;
      continue;
    }

    addWarning(
        fixture,
        commandPath,
        "comparison script body option is not imported by the current fixture "
        "slice");
    ++i;
  }

  fixture.bodies.push_back(std::move(record));
}

[[nodiscard]] bool isBareComparisonTolerance(
    const std::vector<std::string>& tokens)
{
  if (tokens.size() != 1u) {
    return false;
  }
  return parseDoubleToken(tokens.front()).has_value();
}

void readComparisonScriptCommand(
    const std::vector<std::string>& tokens,
    const std::size_t lineNumber,
    bool& expectingToleranceValue,
    std::optional<std::size_t>& remainingShapeRows,
    RigidIpcFixture& fixture)
{
  const std::string path = std::format("$.line{}", lineNumber);
  const std::string& command = tokens.front();

  if (expectingToleranceValue || isBareComparisonTolerance(tokens)) {
    if (tokens.size() != 1u) {
      addError(fixture, path, "expected one tolerance value");
      expectingToleranceValue = false;
      return;
    }
    fixture.velocityConvergenceTolerance
        = readComparisonDoubleToken(tokens.front(), fixture, path);
    expectingToleranceValue = false;
    return;
  }

  if (command == "time") {
    if (tokens.size() < 3u) {
      addError(fixture, path, "time needs max time and timestep");
      return;
    }
    if (const std::optional<double> value
        = readComparisonDoubleToken(tokens[1], fixture, path + ".max_time");
        value.has_value()) {
      fixture.maxTime = *value;
    }
    if (const std::optional<double> value
        = readComparisonDoubleToken(tokens[2], fixture, path + ".timestep");
        value.has_value()) {
      fixture.timeStep = *value;
    }
    return;
  }

  if (command == "shapes") {
    if (tokens.size() < 3u || tokens[1] != "input") {
      addError(fixture, path, "shapes command must be 'shapes input <count>'");
      return;
    }
    const std::optional<int> count
        = readComparisonIntToken(tokens[2], fixture, path + ".count");
    if (count.has_value()) {
      remainingShapeRows = static_cast<std::size_t>(*count);
    }
    return;
  }

  if (command == "selfFric") {
    if (tokens.size() < 2u) {
      addError(fixture, path, "selfFric needs one value");
      return;
    }
    if (const std::optional<double> value = readComparisonDoubleToken(
            tokens[1], fixture, path + ".coefficient_friction");
        value.has_value()) {
      fixture.coefficientFriction = *value;
    }
    return;
  }

  if (command == "constraintSolver") {
    if (tokens.size() < 2u) {
      addError(fixture, path, "constraintSolver needs one solver name");
      return;
    }
    fixture.solverName = tokens[1];
    return;
  }

  if (command == "dHat") {
    if (tokens.size() < 2u) {
      addError(fixture, path, "dHat needs one value");
      return;
    }
    fixture.barrierActivationDistance
        = readComparisonDoubleToken(tokens[1], fixture, path + ".dHat");
    return;
  }

  if (command == "epsv") {
    if (tokens.size() < 2u) {
      addError(fixture, path, "epsv needs one value");
      return;
    }
    fixture.staticFrictionSpeedBound
        = readComparisonDoubleToken(tokens[1], fixture, path + ".epsv");
    return;
  }

  if (command == "fricIterAmt") {
    if (tokens.size() < 2u) {
      addError(fixture, path, "fricIterAmt needs one value");
      return;
    }
    fixture.frictionIterations
        = readComparisonIntToken(tokens[1], fixture, path + ".fricIterAmt");
    return;
  }

  if (command == "useAbsParameters") {
    fixture.velocityConvergenceToleranceIsAbsolute = true;
    return;
  }

  if (command == "turnOffGravity") {
    fixture.gravity = Eigen::Vector3d::Zero();
    fixture.gravityDisabled = true;
    return;
  }

  if (command == "tol") {
    expectingToleranceValue = true;
    return;
  }

  if (command == "energy") {
    if (tokens.size() < 2u) {
      addError(fixture, path, "energy needs one model name");
      return;
    }
    fixture.energyModel = tokens[1];
    return;
  }

  if (command == "warmStart") {
    if (tokens.size() < 2u) {
      addError(fixture, path, "warmStart needs one boolean value");
      return;
    }
    fixture.warmStartEnabled
        = readComparisonBoolToken(tokens[1], fixture, path + ".warmStart");
    return;
  }

  if (command == "selfCollisionOn") {
    fixture.selfCollisionEnabled = true;
    return;
  }

  addWarning(
      fixture,
      path,
      "comparison script command is not imported by the current fixture slice");
}

void appendTriangulatedPolygon(
    const std::vector<Eigen::Vector3d>& polygon,
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<Eigen::Vector3i>& triangles)
{
  if (polygon.size() < 3u) {
    return;
  }

  std::vector<Eigen::Vector3d> polygonVertices = polygon;
  if (polygonVertices.size() > 3u
      && polygonVertices.front().isApprox(polygonVertices.back(), 0.0)) {
    polygonVertices.pop_back();
  }
  if (polygonVertices.size() < 3u) {
    return;
  }

  const int base = static_cast<int>(vertices.size());
  vertices.insert(
      vertices.end(), polygonVertices.begin(), polygonVertices.end());
  for (std::size_t i = 1; i + 1 < polygonVertices.size(); ++i) {
    if (polygonVertices[0].isApprox(polygonVertices[i], 0.0)
        || polygonVertices[0].isApprox(polygonVertices[i + 1], 0.0)
        || polygonVertices[i].isApprox(polygonVertices[i + 1], 0.0)) {
      continue;
    }
    triangles.emplace_back(
        base, base + static_cast<int>(i), base + static_cast<int>(i + 1));
  }
}

[[nodiscard]] MeshLoadResult loadInlineCollisionMesh(
    const RigidIpcBodyRecord& source)
{
  if (!source.hasInlineGeometry) {
    return meshLoadFailure("body has no inline geometry");
  }

  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3i> triangles;
  if (!source.inlinePolygons.empty()) {
    for (const std::vector<Eigen::Vector3d>& polygon : source.inlinePolygons) {
      appendTriangulatedPolygon(polygon, vertices, triangles);
    }
  } else {
    appendTriangulatedPolygon(source.inlineVertices, vertices, triangles);
  }

  if (vertices.empty()) {
    return meshLoadFailure("inline geometry contains no vertices");
  }
  if (triangles.empty()) {
    return meshLoadFailure("inline geometry contains no polygonal faces");
  }

  MeshLoadResult result = makeLoadedMeshResult(
      std::move(vertices),
      std::move(triangles),
      source.scale,
      source.rotationDegrees);
  result.status = "loaded inline geometry";
  return result;
}

template <typename Record>
void validateLoadResult(
    const Record& record, const RigidIpcFixtureLoadOptions& options)
{
  const auto fatal = std::find_if(
      record.diagnostics.begin(),
      record.diagnostics.end(),
      [&](const RigidIpcFixtureDiagnostic& diagnostic) {
        return diagnostic.severity == RigidIpcFixtureDiagnosticSeverity::Error
               || options.strict;
      });

  if (fatal != record.diagnostics.end()) {
    DART_SIMULATION_THROW_T(
        InvalidArgumentException,
        "Fixture diagnostic at {}: {}",
        fatal->path,
        fatal->message);
  }
}

} // namespace

bool RigidIpcFixture::hasErrors() const noexcept
{
  return std::any_of(
      diagnostics.begin(),
      diagnostics.end(),
      [](const RigidIpcFixtureDiagnostic& diagnostic) {
        return diagnostic.severity == RigidIpcFixtureDiagnosticSeverity::Error;
      });
}

bool RigidIpcCcdCase::hasErrors() const noexcept
{
  return std::any_of(
      diagnostics.begin(),
      diagnostics.end(),
      [](const RigidIpcFixtureDiagnostic& diagnostic) {
        return diagnostic.severity == RigidIpcFixtureDiagnosticSeverity::Error;
      });
}

RigidIpcFixture loadRigidIpcFixture(
    std::istream& input, const RigidIpcFixtureLoadOptions& options)
{
  std::ostringstream buffer;
  buffer << input.rdbuf();

  const JsonValue root = JsonParser(buffer.str()).parse();
  const JsonValue::Object& rootObject = requireObject(root, "$");

  RigidIpcFixture fixture;
  warnUnsupportedFields(
      rootObject,
      std::array<std::string_view, 8>{
          "scene_type",
          "solver",
          "timestep",
          "max_time",
          "distance_barrier_constraint",
          "ipc_solver",
          "friction_constraints",
          "rigid_body_problem"},
      "$",
      fixture);

  if (const std::optional<std::string> value
      = readStringMember(rootObject, "scene_type", "$.scene_type", fixture);
      value.has_value()) {
    fixture.sceneType = *value;
  }
  if (const std::optional<std::string> value
      = readStringMember(rootObject, "solver", "$.solver", fixture);
      value.has_value()) {
    fixture.solverName = *value;
  }
  if (const std::optional<double> value
      = readDoubleMember(rootObject, "timestep", "$.timestep", fixture);
      value.has_value()) {
    fixture.timeStep = *value;
  }
  if (const std::optional<double> value
      = readDoubleMember(rootObject, "max_time", "$.max_time", fixture);
      value.has_value()) {
    fixture.maxTime = *value;
  }

  readDistanceBarrierSettings(rootObject, fixture);
  readSolverSettings(rootObject, fixture);
  readFrictionSettings(rootObject, fixture);
  readRigidBodyProblem(rootObject, fixture);
  validateLoadResult(fixture, options);

  return fixture;
}

RigidIpcFixture loadRigidIpcFixture(
    const std::filesystem::path& path,
    const RigidIpcFixtureLoadOptions& options)
{
  std::ifstream input(path);
  if (!input) {
    DART_SIMULATION_THROW_T(
        FileNotFoundException, "Could not open fixture {}", path.string());
  }

  RigidIpcFixture fixture = loadRigidIpcFixture(input, options);
  fixture.sourceDirectory = path.parent_path();
  return fixture;
}

RigidIpcFixture loadRigidIpcComparisonScript(
    std::istream& input, const RigidIpcFixtureLoadOptions& options)
{
  RigidIpcFixture fixture;
  fixture.sceneType = "ipc_comparison_script";

  std::optional<std::size_t> remainingShapeRows;
  bool expectingToleranceValue = false;

  std::string line;
  std::size_t lineNumber = 0u;
  while (std::getline(input, line)) {
    ++lineNumber;
    const std::vector<std::string> tokens
        = splitComparisonScriptLine(std::move(line));
    if (tokens.empty()) {
      continue;
    }

    if (remainingShapeRows.has_value() && *remainingShapeRows > 0u) {
      readComparisonScriptBody(tokens, lineNumber, fixture);
      --*remainingShapeRows;
      continue;
    }

    readComparisonScriptCommand(
        tokens,
        lineNumber,
        expectingToleranceValue,
        remainingShapeRows,
        fixture);
  }

  if (remainingShapeRows.has_value() && *remainingShapeRows > 0u) {
    addError(
        fixture,
        "$.shapes",
        std::format(
            "comparison script ended with {} unparsed shape row(s)",
            *remainingShapeRows));
  }
  if (expectingToleranceValue) {
    addError(
        fixture, "$.tol", "comparison script ended before tolerance value");
  }

  validateLoadResult(fixture, options);
  return fixture;
}

RigidIpcFixture loadRigidIpcComparisonScript(
    const std::filesystem::path& path,
    const RigidIpcFixtureLoadOptions& options)
{
  std::ifstream input(path);
  if (!input) {
    DART_SIMULATION_THROW_T(
        FileNotFoundException,
        "Could not open comparison script {}",
        path.string());
  }

  RigidIpcFixture fixture = loadRigidIpcComparisonScript(input, options);
  fixture.sourceDirectory = path.parent_path();
  return fixture;
}

void applyRigidIpcFixtureStageOptions(
    const RigidIpcFixture& fixture,
    compute::RigidIpcContactStageOptions& options)
{
  if (fixture.barrierActivationDistance.has_value()
      && std::isfinite(*fixture.barrierActivationDistance)
      && *fixture.barrierActivationDistance > 0.0) {
    options.activationDistance = *fixture.barrierActivationDistance;
  }
  if (fixture.frictionIterations.has_value()
      && *fixture.frictionIterations >= 0) {
    options.frictionIterations
        = static_cast<std::size_t>(*fixture.frictionIterations);
  }
  if (fixture.staticFrictionSpeedBound.has_value()
      && std::isfinite(*fixture.staticFrictionSpeedBound)
      && *fixture.staticFrictionSpeedBound >= 0.0) {
    options.staticFrictionSpeedBound = *fixture.staticFrictionSpeedBound;
  }
  if (fixture.velocityConvergenceTolerance.has_value()
      && fixture.velocityConvergenceToleranceIsAbsolute.value_or(false)
      && std::isfinite(*fixture.velocityConvergenceTolerance)
      && *fixture.velocityConvergenceTolerance >= 0.0) {
    options.frictionConvergenceTolerance
        = *fixture.velocityConvergenceTolerance;
  }
}

RigidIpcReplayState populateRigidIpcReplayWorld(
    World& world,
    const RigidIpcFixture& fixture,
    const RigidIpcReplayOptions& options)
{
  if (std::isfinite(fixture.timeStep) && fixture.timeStep > 0.0) {
    world.setTimeStep(fixture.timeStep);
  }
  world.setGravity(fixture.gravity);

  RigidIpcReplayState state;
  state.bodies.reserve(fixture.bodies.size());
  const std::filesystem::path assetRoot
      = options.assetRoot.empty() ? fixture.sourceDirectory : options.assetRoot;

  for (std::size_t i = 0; i < fixture.bodies.size(); ++i) {
    const RigidIpcBodyRecord& source = fixture.bodies[i];
    const std::string bodyName = replayBodyName(options.bodyNamePrefix, i);

    RigidBodyOptions bodyOptions;
    bodyOptions.position = source.position;
    bodyOptions.linearVelocity = source.linearVelocity;
    bodyOptions.angularVelocity = degreesToRadians(source.angularVelocity);
    bodyOptions.isStatic
        = source.mode == RigidIpcBodyMode::Static || allFixed(source.fixedDofs);

    RigidBody body = world.addRigidBody(bodyName, bodyOptions);
    if (source.mode == RigidIpcBodyMode::Kinematic) {
      body.setKinematic(true);
      if (source.kinematicMaxTime.has_value()
          && std::isfinite(*source.kinematicMaxTime)
          && *source.kinematicMaxTime >= 0.0) {
        auto& tag = dart::simulation::detail::registryOf(world)
                        .get<comps::KinematicBodyTag>(
                            rigid_detail::toRegistryEntity(body.getEntity()));
        tag.maxTime = *source.kinematicMaxTime;
      }
    }
    body.setForce(source.force);
    body.setTorque(source.torque);
    if (std::isfinite(fixture.coefficientFriction)
        && fixture.coefficientFriction >= 0.0) {
      body.setFriction(fixture.coefficientFriction);
    }
    if (std::isfinite(fixture.coefficientRestitution)
        && fixture.coefficientRestitution >= 0.0
        && fixture.coefficientRestitution <= 1.0) {
      body.setRestitution(fixture.coefficientRestitution);
    }

    RigidIpcReplayBodyState bodyState;
    bodyState.sourceBodyIndex = i;
    bodyState.bodyName = bodyName;
    bodyState.sourceMeshPath = source.meshPath;
    bodyState.resolvedMeshPath = resolveMeshPath(assetRoot, source.meshPath);
    bodyState.resolvedMeshExists = pathExists(bodyState.resolvedMeshPath);
    bodyState.geometryScale = source.scale;
    bodyState.geometryRotationDegrees = source.rotationDegrees;
    bodyState.mode = source.mode;
    bodyState.fixedDofs = source.fixedDofs;
    bodyState.groupId = source.groupId;
    bodyState.density = source.density;
    bodyState.youngModulus = source.youngModulus;
    bodyState.poissonsRatio = source.poissonsRatio;
    bodyState.kinematicMaxTime = source.kinematicMaxTime;
    bodyState.hasInlineGeometry = source.hasInlineGeometry;
    bodyState.inlineGeometryVertexCount = source.inlineVertices.size();
    bodyState.inlineGeometryEdgeCount = source.inlineEdges.size();
    bodyState.inlineGeometryPolygonCount = source.inlinePolygons.size();

    if (!options.loadMeshCollisionShapes) {
      bodyState.collisionMeshStatus = "mesh collision loading disabled";
    } else {
      MeshLoadResult meshLoad = source.meshPath.empty()
                                    ? loadInlineCollisionMesh(source)
                                    : loadCollisionMesh(
                                          bodyState.resolvedMeshPath,
                                          source.scale,
                                          source.rotationDegrees);
      bodyState.collisionMeshStatus = meshLoad.status;
      bodyState.collisionMeshVertexCount = meshLoad.vertexCount;
      bodyState.collisionMeshTriangleCount = meshLoad.triangleCount;
      if (meshLoad.shape.has_value()) {
        body.setCollisionShape(*meshLoad.shape);
        bodyState.collisionMeshLoaded = true;
      }
    }

    state.bodies.push_back(std::move(bodyState));
  }

  return state;
}

RigidIpcReplayState populateAndStepRigidIpcReplayWorld(
    World& world,
    const RigidIpcFixture& fixture,
    const RigidIpcReplayOptions& replayOptions,
    compute::RigidIpcContactStageOptions stageOptions,
    compute::RigidIpcSolverStats* stats)
{
  RigidIpcReplayState state
      = populateRigidIpcReplayWorld(world, fixture, replayOptions);

  applyRigidIpcFixtureStageOptions(fixture, stageOptions);

  compute::SequentialExecutor executor;
  compute::RigidIpcContactStage ipcStage(stageOptions);
  compute::WorldStepPipeline pipeline;
  pipeline.addStage(ipcStage);
  world.step(executor, pipeline);

  if (stats != nullptr) {
    *stats = ipcStage.getLastStats();
  }

  return state;
}

RigidIpcCcdCase loadRigidIpcCcdCase(
    std::istream& input, const RigidIpcFixtureLoadOptions& options)
{
  std::ostringstream buffer;
  buffer << input.rdbuf();

  const JsonValue root = JsonParser(buffer.str()).parse();
  const JsonValue::Object& rootObject = requireObject(root, "$");

  RigidIpcCcdCase ccdCase;
  warnUnsupportedFields(
      rootObject,
      std::array<std::string_view, 6>{
          "type", "edge", "edge0", "edge1", "face", "vertex"},
      "$",
      ccdCase);

  readCcdCaseBodyData(rootObject, ccdCase);
  validateLoadResult(ccdCase, options);

  return ccdCase;
}

RigidIpcCcdCase loadRigidIpcCcdCase(
    const std::filesystem::path& path,
    const RigidIpcFixtureLoadOptions& options)
{
  std::ifstream input(path);
  if (!input) {
    DART_SIMULATION_THROW_T(
        FileNotFoundException,
        "Could not open CCD data file {}",
        path.string());
  }

  return loadRigidIpcCcdCase(input, options);
}

bool evaluateRigidIpcCcdCase(
    const RigidIpcCcdCase& ccdCase,
    const collision::native::CcdOption& option,
    collision::native::CcdPrimitiveResult& result)
{
  switch (ccdCase.type) {
    case RigidIpcCcdCaseType::EdgeVertex:
      requireCcdVertexCount(ccdCase.bodyA, 2, "edge");
      requireCcdVertexCount(ccdCase.bodyB, 1, "vertex");
      return rigid_detail::rigidIpcPointEdgeIntervalCcd(
          ccdCase.bodyB.vertices[0],
          toRigidIpcPose(ccdCase.bodyB.poseT0),
          toRigidIpcPose(ccdCase.bodyB.poseT1),
          ccdCase.bodyA.vertices[0],
          ccdCase.bodyA.vertices[1],
          toRigidIpcPose(ccdCase.bodyA.poseT0),
          toRigidIpcPose(ccdCase.bodyA.poseT1),
          option,
          result);

    case RigidIpcCcdCaseType::EdgeEdge:
      requireCcdVertexCount(ccdCase.bodyA, 2, "edge0");
      requireCcdVertexCount(ccdCase.bodyB, 2, "edge1");
      return rigid_detail::rigidIpcEdgeEdgeIntervalCcd(
          ccdCase.bodyA.vertices[0],
          ccdCase.bodyA.vertices[1],
          toRigidIpcPose(ccdCase.bodyA.poseT0),
          toRigidIpcPose(ccdCase.bodyA.poseT1),
          ccdCase.bodyB.vertices[0],
          ccdCase.bodyB.vertices[1],
          toRigidIpcPose(ccdCase.bodyB.poseT0),
          toRigidIpcPose(ccdCase.bodyB.poseT1),
          option,
          result);

    case RigidIpcCcdCaseType::FaceVertex:
      requireCcdVertexCount(ccdCase.bodyA, 3, "face");
      requireCcdVertexCount(ccdCase.bodyB, 1, "vertex");
      return rigid_detail::rigidIpcPointTriangleIntervalCcd(
          ccdCase.bodyB.vertices[0],
          toRigidIpcPose(ccdCase.bodyB.poseT0),
          toRigidIpcPose(ccdCase.bodyB.poseT1),
          ccdCase.bodyA.vertices[0],
          ccdCase.bodyA.vertices[1],
          ccdCase.bodyA.vertices[2],
          toRigidIpcPose(ccdCase.bodyA.poseT0),
          toRigidIpcPose(ccdCase.bodyA.poseT1),
          option,
          result);
  }

  DART_SIMULATION_THROW_T(
      InvalidArgumentException, "Unsupported direct CCD case type");
}

} // namespace dart::simulation::io::detail
