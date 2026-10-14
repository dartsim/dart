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
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/utils/sdf/detail/SdfResources.hpp"

#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/String.hpp"

#include <gz/math/Quaternion.hh>
#include <sdf/SDFImpl.hh>
#include <sdf/parser.hh>

#include <algorithm>
#include <fstream>
#include <initializer_list>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <cmath>
#include <cstdlib>

namespace dart {
namespace utils {
namespace SdfParser {
namespace detail {
namespace {

//==============================================================================
bool sdfResourceErrorsAcceptable(
    const sdf::Errors& errors, const common::Uri& source)
{
  bool valid = true;
  for (const auto& error : errors) {
    const auto code = error.Code();
    const bool warning = code == sdf::ErrorCode::NONE
                         || code == sdf::ErrorCode::WARNING
                         || code == sdf::ErrorCode::ATTRIBUTE_DEPRECATED
                         || code == sdf::ErrorCode::ELEMENT_DEPRECATED
                         || code == sdf::ErrorCode::VERSION_DEPRECATED;
    valid = valid && warning;
    if (warning)
      dtwarn << "[SdfParser] " << source.toString() << ": " << error.Message()
             << '\n';
    else
      dterr << "[SdfParser] " << source.toString() << ": " << error.Message()
            << '\n';
  }
  return valid;
}

//==============================================================================
std::string sdfResourceText(const sdf::ElementPtr& element)
{
  return element->GetValue() ? element->GetValue()->GetAsString() : "";
}

//==============================================================================
std::string sdfResourceAttribute(
    const sdf::ElementPtr& element,
    const std::string& name,
    const std::string& fallback)
{
  const auto attribute = element->GetAttribute(name);
  return attribute ? attribute->GetAsString() : fallback;
}

//==============================================================================
std::vector<double> sdfResourceNumbers(
    const sdf::ElementPtr& element,
    std::size_t count,
    bool allowInfinity = false)
{
  const auto tokens = common::split(sdfResourceText(element));
  if (tokens.size() != count) {
    throw std::invalid_argument(
        "<" + element->GetName() + "> requires exactly " + std::to_string(count)
        + " numbers");
  }
  std::vector<double> numbers;
  numbers.reserve(count);
  for (const auto& token : tokens) {
    char* end = nullptr;
    const double value = std::strtod(token.c_str(), &end);
    if (end != token.c_str() + token.size() || std::isnan(value)
        || (!allowInfinity && !std::isfinite(value))) {
      throw std::invalid_argument(
          "<" + element->GetName() + "> has invalid number [" + token + "]");
    }
    numbers.push_back(value);
  }
  return numbers;
}

//==============================================================================
void sdfResourceSetText(const sdf::ElementPtr& element, const std::string& text)
{
  if (!element->GetValue())
    element->AddValue("string", "", false);
  if (!element->Set(text))
    throw std::runtime_error(
        "Could not set <" + element->GetName() + "> value");
}

//==============================================================================
void sdfResourceSetNumbers(
    const sdf::ElementPtr& element, const std::vector<double>& values)
{
  std::ostringstream text;
  text.imbue(std::locale::classic());
  text << std::setprecision(std::numeric_limits<double>::max_digits10);
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0)
      text << ' ';
    text << values[i];
  }
  sdfResourceSetText(element, text.str());
}

//==============================================================================
void sdfResourceNormalizeDirection(const sdf::ElementPtr& element)
{
  auto values = sdfResourceNumbers(element, 3);
  double scale = 0.0;
  for (const auto value : values)
    scale = std::max(scale, std::abs(value));
  if (scale == 0.0)
    throw std::invalid_argument("<" + element->GetName() + "> must be nonzero");
  double norm = 0.0;
  for (auto& value : values) {
    value /= scale;
    norm += value * value;
  }
  norm = std::sqrt(norm);
  for (auto& value : values)
    value /= norm;
  sdfResourceSetNumbers(element, values);
}

//==============================================================================
void sdfResourcePreparePose(const sdf::ElementPtr& element, bool extension)
{
  const auto format
      = sdfResourceAttribute(element, "rotation_format", "euler_rpy");
  if (format != "euler_rpy" && format != "quat_xyzw") {
    throw std::invalid_argument(
        "<pose> has unsupported rotation_format [" + format + "]");
  }
  const auto degreesString = common::toUpper(
      common::trim(sdfResourceAttribute(element, "degrees", "false")));
  if (degreesString != "TRUE" && degreesString != "FALSE"
      && degreesString != "1" && degreesString != "0") {
    throw std::invalid_argument("<pose> has invalid degrees attribute");
  }
  const bool degrees = degreesString == "TRUE" || degreesString == "1";
  if (degrees && format == "quat_xyzw")
    throw std::invalid_argument(
        "<pose> degrees=true is invalid with quat_xyzw");

  const bool empty = common::trim(sdfResourceText(element)).empty();
  auto values
      = empty ? std::vector<double>(format == "quat_xyzw" ? 7 : 6, 0.0)
              : sdfResourceNumbers(element, format == "quat_xyzw" ? 7 : 6);
  if (empty && format == "quat_xyzw")
    values[6] = 1.0;
  if (format == "quat_xyzw") {
    double scale = 0.0;
    for (std::size_t i = 3; i < 7; ++i)
      scale = std::max(scale, std::abs(values[i]));
    if (scale == 0.0)
      throw std::invalid_argument(
          "<pose> requires a finite, nonzero quaternion");
    double norm = 0.0;
    for (std::size_t i = 3; i < 7; ++i) {
      values[i] /= scale;
      norm += values[i] * values[i];
    }
    norm = std::sqrt(norm);
    for (std::size_t i = 3; i < 7; ++i)
      values[i] /= norm;
    if (extension) {
      const auto euler
          = gz::math::Quaterniond(values[6], values[3], values[4], values[5])
                .Euler();
      values
          = {values[0], values[1], values[2], euler.X(), euler.Y(), euler.Z()};
    }
  } else if (extension && degrees) {
    for (std::size_t i = 3; i < 6; ++i)
      values[i] *= 0.017453292519943295769;
  }
  if (extension) {
    element->RemoveAttribute("rotation_format");
    element->RemoveAttribute("degrees");
  }
  if (empty || extension || format == "quat_xyzw")
    sdfResourceSetNumbers(element, values);
}

//==============================================================================
void sdfResourceCheckDimensions(const sdf::ElementPtr& geometry)
{
  const auto kind = geometry->GetName();
  for (auto value = geometry->GetFirstElement(); value;
       value = value->GetNextElement()) {
    const auto name = value->GetName();
    std::size_t count = 0;
    if ((kind == "box" || kind == "ellipsoid") && name == "size")
      count = 3;
    else if ((kind == "sphere" || kind == "cylinder") && name == "radius")
      count = 1;
    else if (kind == "cylinder" && (name == "length" || name == "height"))
      count = 1;
    else if (kind == "mesh" && name == "scale")
      count = 3;
    else if (kind == "plane" && name == "size")
      count = 2;
    else if (kind == "plane" && name == "normal")
      sdfResourceNormalizeDirection(value);
    if (count > 0) {
      const auto dimensions = sdfResourceNumbers(value, count);
      if (std::any_of(dimensions.begin(), dimensions.end(), [&kind](double d) {
            return kind == "mesh" ? d == 0.0 : d <= 0.0;
          }))
        throw std::invalid_argument(
            "<" + kind + "><" + name + "> must be "
            + (kind == "mesh" ? "nonzero" : "positive"));
    }
  }
}

} // namespace

//==============================================================================
SdfResources::SdfResources(common::ResourceRetrieverPtr retriever)
  : mRetriever(std::move(retriever)), mConfig(sdf::ParserConfig::GlobalConfig())
{
  mConfig.SetStoreResolvedURIs(false);
  mConfig.SetWarningsPolicy(sdf::EnforcementPolicy::WARN);
  mConfig.SetFindCallback(
      [this](const std::string& alias) { return findInclude(alias); });
}

//==============================================================================
SdfResources::~SdfResources()
{
  if (!mDirectory.empty()) {
    std::error_code error;
    std::filesystem::remove_all(mDirectory, error);
  }
}

//==============================================================================
bool SdfResources::load(const common::Uri& uri, sdf::Root& root)
{
  mRootUri = uri;
  try {
    if (!mRetriever)
      throw std::invalid_argument("No resource retriever");
    std::random_device random;
    const auto temporary = std::filesystem::temp_directory_path();
    for (std::size_t attempt = 0; attempt < 64; ++attempt) {
      std::ostringstream name;
      name << "dart-sdf-" << std::hex << random() << random();
      const auto directory = temporary / name.str();
      if (std::filesystem::create_directory(directory)) {
        mDirectory = directory;
        break;
      }
    }
    if (mDirectory.empty())
      throw std::runtime_error("Could not create an SDF staging directory");
    std::filesystem::permissions(mDirectory, std::filesystem::perms::owner_all);
    const auto staged = stage(uri, {});
    const auto errors = root.Load(staged, mConfig);
    return sdfResourceErrorsAcceptable(errors, uri) && !mFailed;
  } catch (const std::exception& error) {
    dterr << "[SdfParser] " << uri.toString() << ": " << error.what() << '\n';
    mFailed = true;
    return false;
  }
}

//==============================================================================
std::string SdfResources::stage(
    const common::Uri& uri, const std::vector<std::string>& ancestors)
{
  common::Uri source = uri;
  auto path = mRetriever->getFilePath(source);
  if (path.empty() && source.mScheme.get_value_or("file") == "file"
      && std::filesystem::is_directory(source.getFilesystemPath()))
    path = source.getFilesystemPath();
  bool native = false;
  if (path.empty() && !mRetriever->exists(source)) {
    path = sdf::findFile(source.toString(), true, false, mConfig);
    native = !path.empty();
    if (native)
      source = common::Uri::createFromPath(
          std::filesystem::absolute(path).string());
  }
  if (!path.empty() && std::filesystem::is_directory(path)) {
    sdf::Errors errors;
    const auto file = sdf::getModelFilePath(errors, path);
    if (!sdfResourceErrorsAcceptable(errors, source) || file.empty())
      throw std::runtime_error(
          "Could not select included model from " + source.toString());
    const auto relative
        = std::filesystem::relative(file, path).generic_string();
    auto directoryUri = source;
    directoryUri.mPath = directoryUri.getPath() + "/";
    source = common::Uri::createFromRelativeUri(directoryUri, relative);
    path = file;
  }
  const auto identity
      = path.empty()
            ? common::Uri::createFromRelativeUri(source, source).toString()
            : std::filesystem::weakly_canonical(path).string();
  if (std::find(ancestors.begin(), ancestors.end(), identity)
      != ancestors.end())
    throw std::invalid_argument("Cyclic SDF include of " + source.toString());

  common::LocalResourceRetriever local;
  const auto xml = native ? local.readAll(source) : mRetriever->readAll(source);
  auto raw = std::make_shared<sdf::SDF>();
  auto element = std::make_shared<sdf::Element>();
  element->SetName("sdf");
  element->AddAttribute("version", "string", "", true);
  element->SetCopyChildren(true);
  raw->SetRoot(element);
  sdf::Errors errors;
  const bool read = sdf::readStringWithoutConversion(xml, mConfig, raw, errors);
  if (!sdfResourceErrorsAcceptable(errors, source) || !read)
    throw std::invalid_argument(
        "Could not read SDF document " + source.toString());
  auto ancestry = ancestors;
  ancestry.push_back(identity);
  try {
    prepare(raw->Root(), source, ancestry);
  } catch (const std::exception& error) {
    throw std::invalid_argument(source.toString() + ": " + error.what());
  }
  const auto text = raw->Root()->ToString(errors, "", false, false);
  if (!sdfResourceErrorsAcceptable(errors, source))
    throw std::runtime_error(
        "Could not serialize SDF document " + source.toString());
  const auto filename = std::filesystem::absolute(
                            mDirectory / (std::to_string(mNextId++) + ".sdf"))
                            .string();
  std::ofstream output(filename, std::ios::binary);
  output << text;
  output.close();
  if (!output)
    throw std::runtime_error(
        "Could not stage SDF document " + source.toString());
  mOrigins.emplace(filename, source);
  return filename;
}

//==============================================================================
void SdfResources::prepare(
    const sdf::ElementPtr& element,
    const common::Uri& source,
    const std::vector<std::string>& ancestors,
    bool softGeometry)
{
  if (element->GetName() == "plugin")
    return;
  const auto name = element->GetName();
  const auto parent = element->GetParent();
  const auto parentName = parent ? parent->GetName() : "";
  if (name == "pose" && parentName == "inertial") {
    const auto frame
        = common::trim(sdfResourceAttribute(element, "relative_to", ""));
    const auto link = parent->GetParent();
    const auto linkName = link ? sdfResourceAttribute(link, "name", "") : "";
    // Inertial pose relative_to is ignored by the supported sdformat schemas.
    if (!frame.empty() && frame != linkName)
      throw std::invalid_argument(
          "Inertial poses must be expressed in their link frame");
  }
  // sdformat can coerce numeric prefixes or clamp NaNs before typed access.
  const auto is = [&name](std::initializer_list<const char*> names) {
    return std::any_of(names.begin(), names.end(), [&name](const char* value) {
      return name == value;
    });
  };
  if ((parentName == "inertial" && name == "mass")
      || (parentName == "inertia"
          && is({"ixx", "ixy", "ixz", "iyy", "iyz", "izz"}))
      || (parentName == "dynamics"
          && is(
              {"damping", "friction", "spring_reference", "spring_stiffness"}))
      || (parentName == "joint" && is({"thread_pitch", "screw_thread_pitch"}))
      || (parentName == "mimic" && is({"multiplier", "offset", "reference"}))
      || (parentName == "physics" && name == "max_step_size")
      || (parentName == "soft_shape" && is({"total_mass", "kv", "ke", "damp"}))
      || (softGeometry && is({"num_slices", "num_stacks", "num_rings"}))) {
    const auto values = sdfResourceNumbers(element, 1);
    if (is({"mass", "total_mass", "max_step_size"}) && values[0] <= 0.0)
      throw std::invalid_argument("<" + name + "> must be positive");
  }
  if (parentName == "limit" && is({"lower", "upper", "effort", "velocity"}))
    sdfResourceNumbers(element, 1, true);
  if (((parentName == "world" || parentName == "physics") && name == "gravity")
      || (softGeometry && parentName == "box" && name == "frags"))
    sdfResourceNumbers(element, 3);
  if (parentName == "material" && name == "diffuse") {
    const auto count = common::split(sdfResourceText(element)).size();
    if (count != 3 && count != 4)
      throw std::invalid_argument("<diffuse> requires three or four numbers");
    sdfResourceNumbers(element, count);
  }
  if (name == "pose"
      && (parentName == "model" || parentName == "link" || parentName == "joint"
          || parentName == "inertial" || parentName == "visual"
          || parentName == "collision" || parentName == "frame"
          || parentName == "include" || parentName == "soft_shape"
          || softGeometry))
    sdfResourcePreparePose(element, parentName == "soft_shape" || softGeometry);
  if (parentName == "geometry")
    sdfResourceCheckDimensions(element);
  if (name == "xyz" && (parentName == "axis" || parentName == "axis2"))
    sdfResourceNormalizeDirection(element);
  if (name == "include" && (parentName == "world" || parentName == "model")) {
    const auto uri = element->FindElement("uri");
    if (!uri || common::trim(sdfResourceText(uri)).empty())
      throw std::invalid_argument("<include> requires a URI");
    const auto alias = "dart-sdf-stage://load/" + std::to_string(mNextId++);
    mIncludes.emplace(
        alias, Include{source, common::trim(sdfResourceText(uri)), ancestors});
    sdfResourceSetText(uri, alias);
  }
  softGeometry
      = softGeometry || (name == "geometry" && parentName == "soft_shape");
  for (auto child = element->GetFirstElement(); child;
       child = child->GetNextElement())
    prepare(child, source, ancestors, softGeometry);
}

//==============================================================================
std::string SdfResources::findInclude(const std::string& alias)
{
  const auto found = mIncludes.find(alias);
  if (found == mIncludes.end()) {
    dterr << "[SdfParser] Unregistered SDF include " << alias << '\n';
    mFailed = true;
    return "";
  }
  // Copy before staging: child includes extend this table during parsing.
  const auto include = found->second;
  const auto uri
      = common::Uri::createFromRelativeUri(include.owner, include.uri);
  try {
    return stage(uri, include.ancestors);
  } catch (const std::exception& error) {
    dterr << "[SdfParser] " << uri.toString() << ": " << error.what() << '\n';
    mFailed = true;
    return "";
  }
}

//==============================================================================
common::Uri SdfResources::resolveUri(
    const sdf::ElementPtr& element, const std::string& uri) const
{
  auto source = element;
  while (source && source->FilePath().empty())
    source = source->GetParent();
  auto origin = mRootUri;
  if (source) {
    const auto found = mOrigins.find(source->FilePath());
    if (found == mOrigins.end())
      throw std::invalid_argument(
          "Unknown SDF mesh origin " + source->FilePath());
    origin = found->second;
  }
  return common::Uri::createFromRelativeUri(origin, uri);
}

} // namespace detail
} // namespace SdfParser
} // namespace utils
} // namespace dart
