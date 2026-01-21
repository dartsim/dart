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

#include "dart/common/uri.hpp"

#include "dart/common/logging.hpp"
#include "dart/common/macros.hpp"

#include <regex>
#include <sstream>
#include <string_view>

#include <cassert>

static bool startsWith(std::string_view target, std::string_view prefix)
{
  return target.starts_with(prefix);
}

namespace dart {
namespace common {

/*
 * UriComponent
 */

//==============================================================================
UriComponent::UriComponent()
{
  reset();
}

//==============================================================================
UriComponent::UriComponent(reference_const_type _value)
{
  assign(_value);
}

//==============================================================================
UriComponent::operator bool() const
{
  return mExists;
}

//==============================================================================
bool UriComponent::operator!() const
{
  return !mExists;
}

//==============================================================================
auto UriComponent::operator=(reference_const_type _value) -> UriComponent&
{
  assign(_value);
  return *this;
}

//==============================================================================
auto UriComponent::operator*() -> reference_type
{
  return get();
}

//==============================================================================
auto UriComponent::operator*() const -> reference_const_type
{
  return get();
}

//==============================================================================
auto UriComponent::operator->() -> pointer_type
{
  return &get();
}

//==============================================================================
auto UriComponent::operator->() const -> pointer_const_type
{
  return &get();
}

//==============================================================================
void UriComponent::assign(reference_const_type _value)
{
  mExists = true;
  mValue = _value;
}

//==============================================================================
void UriComponent::reset()
{
  mExists = false;
}

//==============================================================================
auto UriComponent::get() -> reference_type
{
  DART_ASSERT(mExists);
  return mValue;
}

//==============================================================================
auto UriComponent::get() const -> reference_const_type
{
  DART_ASSERT(mExists);
  return mValue;
}

//==============================================================================
auto UriComponent::get_value_or(reference_type _default) -> reference_type
{
  if (mExists) {
    return mValue;
  } else {
    return _default;
  }
}

//==============================================================================
auto UriComponent::get_value_or(reference_const_type _default) const
    -> reference_const_type
{
  if (mExists) {
    return mValue;
  } else {
    return _default;
  }
}

/*
 * Uri
 */

//==============================================================================
Uri::Uri(std::string_view input)
{
  DART_WARN_IF(!fromStringOrPath(input), "Failed parsing URI '{}'.", input);

  // We don't need to clear since fromStringOrPath() does not set any component
  // on failure.
}

//==============================================================================
Uri::Uri(const std::string& input) : Uri(std::string_view{input}) {}

//==============================================================================
Uri::Uri(const char* input)
{
  DART_WARN_IF(!fromStringOrPath(input), "Failed parsing URI '{}'.", input);

  // We don't need to clear since fromStringOrPath() does not set any component
  // on failure.
}

//==============================================================================
void Uri::clear()
{
  mScheme.reset();
  mAuthority.reset();
  mPath.reset();
  mQuery.reset();
  mFragment.reset();
}

//==============================================================================
bool Uri::fromString(std::string_view input)
{
  // This is regex is from Appendix B of RFC 3986.
  static std::regex uriRegex(
      R"END(^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\?([^#]*))?(#(.*))?)END",
      std::regex::extended | std::regex::optimize);
  static const std::size_t schemeIndex = 2;
  static const std::size_t authorityIndex = 4;
  static const std::size_t pathIndex = 5;
  static const std::size_t queryIndex = 7;
  static const std::size_t fragmentIndex = 9;

  clear();

  std::match_results<std::string_view::const_iterator> matches;
  if (!std::regex_match(input.begin(), input.end(), matches, uriRegex)) {
    return false;
  }

  DART_ASSERT(matches.size() > schemeIndex);
  const auto& schemeMatch = matches[schemeIndex];
  if (schemeMatch.matched) {
    mScheme = schemeMatch.str();
  }

  DART_ASSERT(matches.size() > authorityIndex);
  const auto& authorityMatch = matches[authorityIndex];
  if (authorityMatch.matched) {
    mAuthority = authorityMatch.str();
  }

  DART_ASSERT(matches.size() > pathIndex);
  const auto& pathMatch = matches[pathIndex];
  if (pathMatch.matched) {
    mPath = pathMatch.str();
  }

  DART_ASSERT(matches.size() > queryIndex);
  const auto& queryMatch = matches[queryIndex];
  if (queryMatch.matched) {
    mQuery = queryMatch.str();
  }

  DART_ASSERT(matches.size() > fragmentIndex);
  const auto& fragmentMatch = matches[fragmentIndex];
  if (fragmentMatch.matched) {
    mFragment = fragmentMatch.str();
  }

  return true;
}

//==============================================================================
bool Uri::fromPath(std::string_view path)
{
  // TODO(JS): We might want to check validity of path.

  static const std::string fileScheme("file://");
  std::string pathString(path);

#ifdef _WIN32
  // Replace backslashes (from Windows paths) with forward slashes.
  std::string unixPath = pathString;
  std::replace(std::begin(unixPath), std::end(unixPath), '\\', '/');

  return fromString(fileScheme + "/" + pathString);
#else
  return fromString(fileScheme + pathString);
#endif
}

//==============================================================================
bool Uri::fromStringOrPath(std::string_view input)
{
  // TODO(JS): Need to check if input is an "absolute" path?

#ifdef _WIN32

  // Assume that any URI begin with pattern [SINGLE_LETTER]:[/ or \\] is an
  // absolute path.
  static std::regex windowsPathRegex(R"END([a-zA-Z]:[/|\\])END");
  const bool isPath = std::regex_search(
      input.begin(),
      input.end(),
      windowsPathRegex,
      std::regex_constants::match_continuous);

  if (isPath) {
    return fromPath(input);
  }

#else

  // Assume that any URI without a scheme is a path.
  static std::regex uriSchemeRegex(R"END(^(([^:/?#]+):))END");
  const bool noScheme = !std::regex_search(
      input.begin(),
      input.end(),
      uriSchemeRegex,
      std::regex_constants::match_continuous);

  if (noScheme) {
    return fromPath(input);
  }

#endif

  return fromString(input);
}

//==============================================================================
bool Uri::fromRelativeUri(
    std::string_view base, std::string_view relative, bool strict)
{
  Uri baseUri;
  if (!baseUri.fromString(base)) {
    DART_WARN("Failed parsing base URI '{}'.", base);
    clear();
    return false;
  }

  return fromRelativeUri(baseUri, relative, strict);
}

//==============================================================================
bool Uri::fromRelativeUri(const char* base, const char* relative, bool strict)
{
  return fromRelativeUri(
      std::string_view(base), std::string_view(relative), strict);
}

//==============================================================================
bool Uri::fromRelativeUri(
    const Uri& base, std::string_view relative, bool strict)
{
  Uri relativeUri;
  if (!relativeUri.fromString(relative)) {
    DART_WARN("Failed parsing relative URI '{}'.", relative);
    clear();
    return false;
  }

  return fromRelativeUri(base, relativeUri, strict);
}

//==============================================================================
bool Uri::fromRelativeUri(const Uri& base, const char* relative, bool strict)
{
  return fromRelativeUri(base, std::string_view(relative), strict);
}

//==============================================================================
bool Uri::fromRelativeUri(const Uri& base, const Uri& relative, bool /*strict*/)
{
  DART_ASSERT(base.mPath && "The path component is always defined.");
  DART_ASSERT(relative.mPath && "The path component is always defined.");

  // TODO If (!strict && relative.mScheme == base.mScheme), then we need to
  // enable backwards compatibility.

  // This directly implements the psueocode in Section 5.2.2. of RFC 3986.
  if (relative.mScheme) {
    mScheme = relative.mScheme;
    mAuthority = relative.mAuthority;
    mPath = removeDotSegments(*relative.mPath);
    mQuery = relative.mQuery;
  } else {
    if (relative.mAuthority) {
      mAuthority = relative.mAuthority;
      mPath = removeDotSegments(*relative.mPath);
      mQuery = relative.mQuery;
    } else {
      if (relative.mPath->empty()) {
        mPath = base.mPath;

        if (relative.mQuery) {
          mQuery = relative.mQuery;
        } else {
          mQuery = base.mQuery;
        }
      } else {
        if (relative.mPath->front() == '/') {
          mPath = removeDotSegments(*relative.mPath);
        } else {
          mPath = removeDotSegments(mergePaths(base, relative));
        }

        mQuery = relative.mQuery;
      }

      mAuthority = base.mAuthority;
    }

    mScheme = base.mScheme;
  }

  mFragment = relative.mFragment;
  return true;
}

//==============================================================================
std::string Uri::toString() const
{
  // This function implements the pseudo-code from Section 5.3 of RFC 3986.
  std::stringstream output;

  if (mScheme) {
    output << *mScheme << ":";
  }

  if (mAuthority) {
    output << "//" << *mAuthority;
  }

  output << mPath.get_value_or("");

  if (mQuery) {
    output << "?" << *mQuery;
  }

  if (mFragment) {
    output << "#" << *mFragment;
  }

  return output.str();
}

//==============================================================================
Uri Uri::createFromString(std::string_view input)
{
  Uri uri;
  DART_WARN_IF(!uri.fromString(input), "Failed parsing URI '{}'.", input);

  // We don't need to clear uri since fromString() does not set any component
  // on failure.

  return uri;
}

//==============================================================================
Uri Uri::createFromPath(std::string_view path)
{
  Uri fileUri;
  DART_WARN_IF(
      !fileUri.fromPath(path), "Failed parsing local path '{}'.", path);

  // We don't need to clear uri since fromString() does not set any component
  // on failure.

  return fileUri;
}

//==============================================================================
Uri Uri::createFromStringOrPath(std::string_view input)
{
  Uri uri;
  DART_WARN_IF(!uri.fromStringOrPath(input), "Failed parsing URI '{}'.", input);

  // We don't need to clear uri since fromString() does not set any component
  // on failure.

  return uri;
}

//==============================================================================
Uri Uri::createFromRelativeUri(
    std::string_view base, std::string_view relative, bool strict)
{
  Uri mergedUri;
  DART_WARN_IF(
      !mergedUri.fromRelativeUri(base, relative, strict),
      "Failed merging URI '{}' with base URI '{}'.",
      relative,
      base);

  // We don't need to clear mergedUri since fromRelativeUri() does not set any
  // component on failure.

  return mergedUri;
}

//==============================================================================
Uri Uri::createFromRelativeUri(
    const Uri& base, std::string_view relative, bool strict)
{
  Uri mergedUri;
  DART_WARN_IF(
      !mergedUri.fromRelativeUri(base, relative, strict),
      "Failed merging URI '{}' with base URI '{}'.",
      relative,
      base.toString());

  // We don't need to clear mergedUri since fromRelativeUri() does not set any
  // component on failure.

  return mergedUri;
}

//==============================================================================
Uri Uri::createFromRelativeUri(
    const Uri& baseUri, const Uri& relativeUri, bool strict)
{
  Uri mergedUri;
  DART_WARN_IF(
      !mergedUri.fromRelativeUri(baseUri, relativeUri, strict),
      "Failed merging URI '{}' with base URI '{}'.",
      relativeUri.toString(),
      baseUri.toString());

  // We don't need to clear mergedUri since fromRelativeUri() does not set any
  // component on failure.

  return mergedUri;
}

//==============================================================================
std::string Uri::getUri(std::string_view input)
{
  Uri uri;
  if (uri.fromStringOrPath(input)) {
    return uri.toString();
  } else {
    return "";
  }
}

//==============================================================================
std::string Uri::getRelativeUri(
    std::string_view base, std::string_view relative, bool strict)
{
  Uri mergedUri;
  if (!mergedUri.fromRelativeUri(base, relative, strict)) {
    return "";
  } else {
    return mergedUri.toString();
  }
}

//==============================================================================
std::string Uri::getRelativeUri(
    const Uri& base, std::string_view relative, bool strict)
{
  Uri mergedUri;
  if (!mergedUri.fromRelativeUri(base, relative, strict)) {
    return "";
  } else {
    return mergedUri.toString();
  }
}

//==============================================================================
std::string Uri::getRelativeUri(
    const Uri& baseUri, const Uri& relativeUri, bool strict)
{
  Uri mergedUri;
  if (!mergedUri.fromRelativeUri(baseUri, relativeUri, strict)) {
    return "";
  } else {
    return mergedUri.toString();
  }
}

//==============================================================================
std::string Uri::getPath() const
{
  return mPath.get_value_or("");
}

//==============================================================================
std::string Uri::getFilesystemPath() const
{
#ifdef _WIN32
  if (mScheme.get_value_or("") == "file") {
    const std::string& filesystemPath = getPath();

    if (!filesystemPath.empty() && filesystemPath[0] == '/') {
      return filesystemPath.substr(1);
    }
  }
#endif

  return getPath();
}

//==============================================================================
std::string Uri::mergePaths(const Uri& base, const Uri& relative)
{
  DART_ASSERT(base.mPath && "The path component is always defined.");
  DART_ASSERT(relative.mPath && "The path component is always defined.");

  // This directly implements the logic from Section 5.2.3. of RFC 3986.
  if (base.mAuthority && base.mPath->empty()) {
    return "/" + *relative.mPath;
  } else {
    const std::size_t index = base.mPath->find_last_of('/');
    if (index != std::string::npos) {
      return base.mPath->substr(0, index + 1) + *relative.mPath;
    } else {
      return *relative.mPath;
    }
  }
}

//==============================================================================
std::string Uri::removeDotSegments(const std::string& path)
{
  // 1.  The input buffer is initialized with the now-appended path
  //     components and the output buffer is initialized to the empty
  //     string.
  std::string input = path;
  std::string output;

  // 2.  While the input buffer is not empty, loop as follows:
  while (!input.empty()) {
    // A.  If the input buffer begins with a prefix of "../" or "./",
    //     then remove that prefix from the input buffer; otherwise,
    if (startsWith(input, "../")) {
      input = input.substr(3);
    } else if (startsWith(input, "./")) {
      input = input.substr(2);
    }
    // B.  if the input buffer begins with a prefix of "/./" or "/.",
    //     where "." is a complete path segment, then replace that
    //     prefix with "/" in the input buffer; otherwise,
    else if (input == "/.") {
      input = "/";
    } else if (startsWith(input, "/./")) {
      input = "/" + input.substr(3);
    }
    // C.  if the input buffer begins with a prefix of "/../" or "/..",
    //     where ".." is a complete path segment, then replace that
    //     prefix with "/" in the input buffer and remove the last
    //     segment and its preceding "/" (if any) from the output
    //     buffer; otherwise,
    else if (input == "/..") {
      input = "/";

      std::size_t index = output.find_last_of('/');
      if (index != std::string::npos) {
        output = output.substr(0, index);
      } else {
        output = "";
      }
    } else if (startsWith(input, "/../")) {
      input = "/" + input.substr(4);

      std::size_t index = output.find_last_of('/');
      if (index != std::string::npos) {
        output = output.substr(0, index);
      } else {
        output = "";
      }
    }
    // D.  if the input buffer consists only of "." or "..", then remove
    //     that from the input buffer; otherwise,
    else if (input == "." || input == "..") {
      input = "";
    }
    // E.  move the first path segment in the input buffer to the end of
    //     the output buffer, including the initial "/" character (if
    //     any) and any subsequent characters up to, but not including,
    //     the next "/" character or the end of the input buffer.
    else {
      // Start at index one so we keep the leading '/'.
      std::size_t index = input.find_first_of('/', 1);
      output += input.substr(0, index);

      if (index != std::string::npos) {
        input = input.substr(index);
      } else {
        input = "";
      }
    }
  }

  // 3.  Finally, the output buffer is returned as the result of
  //     remove_dot_segments.
  return output;
}

} // namespace common
} // namespace dart
