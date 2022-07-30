/*
 * Copyright (c) 2011-2022, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/common/resource/uri.hpp"

#include <cassert>
#include <regex>
#include <sstream>

#include "dart/common/logging.hpp"
#include "dart/common/macro.hpp"

namespace dart::common {

//===============================================================================
static bool starts_with(const std::string& target, const std::string& prefix)
{
  return target.substr(0, prefix.size()) == prefix;
}

//==============================================================================
UriComponent::UriComponent()
{
  reset();
}

//==============================================================================
UriComponent::UriComponent(reference_const_type value)
{
  assign(value);
}

//==============================================================================
UriComponent::operator bool() const
{
  return m_exists;
}

//==============================================================================
bool UriComponent::operator!() const
{
  return !m_exists;
}

//==============================================================================
auto UriComponent::operator=(reference_const_type value) -> UriComponent&
{
  assign(value);
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
auto UriComponent::operator-> () -> pointer_type
{
  return &get();
}

//==============================================================================
auto UriComponent::operator-> () const -> pointer_const_type
{
  return &get();
}

//==============================================================================
void UriComponent::assign(reference_const_type value)
{
  m_exists = true;
  m_value = value;
}

//==============================================================================
void UriComponent::reset()
{
  m_exists = false;
}

//==============================================================================
auto UriComponent::get() -> reference_type
{
  DART_ASSERT(m_exists);
  return m_value;
}

//==============================================================================
auto UriComponent::get() const -> reference_const_type
{
  DART_ASSERT(m_exists);
  return m_value;
}

//==============================================================================
auto UriComponent::get_value_or(reference_type default_value) -> reference_type
{
  if (m_exists) {
    return m_value;
  } else {
    return default_value;
  }
}

//==============================================================================
auto UriComponent::get_value_or(reference_const_type default_value) const
    -> reference_const_type
{
  if (m_exists) {
    return m_value;
  } else {
    return default_value;
  }
}

//==============================================================================
Uri::Uri(const std::string& input)
{
  if (!from_string_or_path(input)) {
    DART_WARN("[Uri::Uri] Failed parsing URI '{}'.", input);
  }

  // We don't need to clear since fromStringOrPath() does not set any component
  // on failure.
}

//==============================================================================
Uri::Uri(const char* input)
{
  if (!from_string_or_path(std::string(input))) {
    DART_WARN("[Uri::Uri] Failed parsing URI '{}'.", input);
  }

  // We don't need to clear since fromStringOrPath() does not set any component
  // on failure.
}

//==============================================================================
void Uri::clear()
{
  scheme.reset();
  authority.reset();
  path.reset();
  query.reset();
  fragment.reset();
}

//==============================================================================
bool Uri::from_string(const std::string& input)
{
  // This is regex is from Appendix B of RFC 3986.
  static std::regex uriRegex(
      R"END(^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\?([^#]*))?(#(.*))?)END",
      std::regex::extended | std::regex::optimize);
  static const std::size_t schemeIndex = 2;
  static const std::size_t authority_index = 4;
  static const std::size_t path_index = 5;
  static const std::size_t query_index = 7;
  static const std::size_t fragment_index = 9;

  clear();

  std::smatch matches;
  if (!regex_match(input, matches, uriRegex)) {
    return false;
  }

  DART_ASSERT(matches.size() > schemeIndex);
  const std::ssub_match& schemeMatch = matches[schemeIndex];
  if (schemeMatch.matched) {
    scheme = schemeMatch;
  }

  DART_ASSERT(matches.size() > authority_index);
  const std::ssub_match& authority_match = matches[authority_index];
  if (authority_match.matched) {
    authority = authority_match;
  }

  DART_ASSERT(matches.size() > path_index);
  const std::ssub_match& path_match = matches[path_index];
  if (path_match.matched) {
    path = path_match;
  }

  DART_ASSERT(matches.size() > query_index);
  const std::ssub_match& query_match = matches[query_index];
  if (query_match.matched) {
    query = query_match;
  }

  DART_ASSERT(matches.size() > fragment_index);
  const std::ssub_match& fragment_match = matches[fragment_index];
  if (fragment_match.matched) {
    fragment = fragment_match;
  }

  return true;
}

//==============================================================================
bool Uri::from_path(const std::string& path)
{
  // TODO(JS): We might want to check validity of path.

  static const std::string fileScheme("file://");

#ifdef WIN32
  // Replace backslashes (from Windows paths) with forward slashes.
  std::string unixPath = path;
  std::replace(std::begin(unixPath), std::end(unixPath), '\\', '/');

  return from_string(fileScheme + "/" + path);
#else
  return from_string(fileScheme + path);
#endif
}

//==============================================================================
bool Uri::from_string_or_path(const std::string& input)
{
  // TODO(JS): Need to check if input is an "absolute" path?

#ifdef WIN32

  // Assume that any URI begin with pattern [SINGLE_LETTER]:[/ or \\] is an
  // absolute path.
  static std::regex windows_path_regex(R"END([a-zA-Z]:[/|\\])END");
  const bool isPath = std::regex_search(
      input, windows_path_regex, std::regex_constants::match_continuous);

  if (isPath) {
    return from_path(input);
  }

#else

  // Assume that any URI without a scheme is a path.
  static std::regex uri_scheme_regex(R"END(^(([^:/?#]+):))END");
  const bool no_scheme = !std::regex_search(
      input, uri_scheme_regex, std::regex_constants::match_continuous);

  if (no_scheme) {
    return from_path(input);
  }

#endif

  return from_string(input);
}

//==============================================================================
bool Uri::from_relative_uri(
    const std::string& base, const std::string& relative, bool strict)
{
  Uri base_uri;
  if (!base_uri.from_string(base)) {
    DART_WARN("Failed parsing base URI '{}'.", base);
    clear();
    return false;
  }

  return from_relative_uri(base_uri, relative, strict);
}

//==============================================================================
bool Uri::from_relative_uri(const char* base, const char* relative, bool strict)
{
  return from_relative_uri(std::string(base), std::string(relative), strict);
}

//==============================================================================
bool Uri::from_relative_uri(
    const Uri& base, const std::string& relative, bool strict)
{
  Uri relative_uri;
  if (!relative_uri.from_string(relative)) {
    DART_WARN("Failed parsing relative URI '{}'.", relative);
    clear();
    return false;
  }

  return from_relative_uri(base, relative_uri, strict);
}

//==============================================================================
bool Uri::from_relative_uri(const Uri& base, const char* relative, bool strict)
{
  return from_relative_uri(base, std::string(relative), strict);
}

//==============================================================================
bool Uri::from_relative_uri(
    const Uri& base, const Uri& relative, bool /*_strict*/)
{
  DART_ASSERT(base.path && "The path component is always defined.");
  DART_ASSERT(relative.path && "The path component is always defined.");

  // TODO If (!_strict && relative.mScheme == base.mScheme), then we need to
  // enable backwards compatability.

  // This directly implements the psueocode in Section 5.2.2. of RFC 3986.
  if (relative.scheme) {
    scheme = relative.scheme;
    authority = relative.authority;
    path = remove_dot_segments(*relative.path);
    query = relative.query;
  } else {
    if (relative.authority) {
      authority = relative.authority;
      path = remove_dot_segments(*relative.path);
      query = relative.query;
    } else {
      if (relative.path->empty()) {
        path = base.path;

        if (relative.query) {
          query = relative.query;
        } else {
          query = base.query;
        }
      } else {
        if (relative.path->front() == '/') {
          path = remove_dot_segments(*relative.path);
        } else {
          path = remove_dot_segments(merge_paths(base, relative));
        }

        query = relative.query;
      }

      authority = base.authority;
    }

    scheme = base.scheme;
  }

  fragment = relative.fragment;
  return true;
}

//==============================================================================
std::string Uri::to_string() const
{
  // This function implements the pseudo-code from Section 5.3 of RFC 3986.
  std::stringstream output;

  if (scheme) {
    output << *scheme << ":";
  }

  if (authority) {
    output << "//" << *authority;
  }

  output << path.get_value_or("");

  if (query) {
    output << "?" << *query;
  }

  if (fragment) {
    output << "#" << *fragment;
  }

  return output.str();
}

//==============================================================================
Uri Uri::CreateFromString(const std::string& input)
{
  Uri uri;
  if (!uri.from_string(input)) {
    DART_WARN("Failed parsing URI '{}'.", input);
  }

  // We don't need to clear uri since fromString() does not set any component
  // on failure.

  return uri;
}

//==============================================================================
Uri Uri::CreateFromPath(const std::string& path)
{
  Uri fileUri;
  if (!fileUri.from_path(path)) {
    DART_WARN("Failed parsing local path '{}'.", path);
  }

  // We don't need to clear uri since fromString() does not set any component
  // on failure.

  return fileUri;
}

//==============================================================================
Uri Uri::CreateFromString_or_path(const std::string& input)
{
  Uri uri;
  if (!uri.from_string_or_path(input)) {
    DART_WARN("Failed parsing URI '{}'.", input);
  }

  // We don't need to clear uri since fromString() does not set any component
  // on failure.

  return uri;
}

//==============================================================================
Uri Uri::CreateFromRelativeUri(
    const std::string& base, const std::string& relative, bool strict)
{
  Uri merged_uri;
  if (!merged_uri.from_relative_uri(base, relative, strict)) {
    DART_WARN("Failed merging URI '{}' with base URI '{}'.", relative, base);
  }

  // We don't need to clear merged_uri since fromRelativeUri() does not set any
  // component on failure.

  return merged_uri;
}

//==============================================================================
Uri Uri::CreateFromRelativeUri(
    const Uri& base, const std::string& relative, bool strict)
{
  Uri merged_uri;
  if (!merged_uri.from_relative_uri(base, relative, strict)) {
    DART_WARN(
        "Failed merging URI '{}' with base URI '{}'.",
        relative,
        base.to_string());
  }

  // We don't need to clear merged_uri since fromRelativeUri() does not set any
  // component on failure.

  return merged_uri;
}

//==============================================================================
Uri Uri::CreateFromRelativeUri(
    const Uri& base_uri, const Uri& relative_uri, bool strict)
{
  Uri merged_uri;
  if (!merged_uri.from_relative_uri(base_uri, relative_uri, strict)) {
    DART_WARN(
        "Failed merging URI '{}' with base URI '{}'.",
        relative_uri.to_string(),
        base_uri.to_string());
  }

  // We don't need to clear merged_uri since fromRelativeUri() does not set any
  // component on failure.

  return merged_uri;
}

//==============================================================================
std::string Uri::GetUri(const std::string& input)
{
  Uri uri;
  if (uri.from_string_or_path(input)) {
    return uri.to_string();
  } else {
    return "";
  }
}

//==============================================================================
std::string Uri::GetRelativeUri(
    const std::string& base, const std::string& relative, bool strict)
{
  Uri merged_uri;
  if (!merged_uri.from_relative_uri(base, relative, strict)) {
    return "";
  } else {
    return merged_uri.to_string();
  }
}

//==============================================================================
std::string Uri::GetRelativeUri(
    const Uri& base, const std::string& relative, bool strict)
{
  Uri merged_uri;
  if (!merged_uri.from_relative_uri(base, relative, strict)) {
    return "";
  } else {
    return merged_uri.to_string();
  }
}

//==============================================================================
std::string Uri::GetRelativeUri(
    const Uri& base_uri, const Uri& relative_uri, bool strict)
{
  Uri merged_uri;
  if (!merged_uri.from_relative_uri(base_uri, relative_uri, strict)) {
    return "";
  } else {
    return merged_uri.to_string();
  }
}

//==============================================================================
std::string Uri::get_path() const
{
  return path.get_value_or("");
}

//==============================================================================
std::string Uri::get_filesystem_path() const
{
#ifdef WIN32
  if (scheme.get_value_or("") == "file") {
    const std::string& filesystem_path = get_path();

    if (!filesystem_path.empty() && filesystem_path[0] == '/') {
      return filesystem_path.substr(1);
    }
  }
#endif

  return get_path();
}

//==============================================================================
std::string Uri::merge_paths(const Uri& base, const Uri& relative)
{
  DART_ASSERT(base.path && "The path component is always defined.");
  DART_ASSERT(relative.path && "The path component is always defined.");

  // This directly implements the logic from Section 5.2.3. of RFC 3986.
  if (base.authority && base.path->empty()) {
    return "/" + *relative.path;
  } else {
    const std::size_t index = base.path->find_last_of('/');
    if (index != std::string::npos) {
      return base.path->substr(0, index + 1) + *relative.path;
    } else {
      return *relative.path;
    }
  }
}

//==============================================================================
std::string Uri::remove_dot_segments(const std::string& path)
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
    if (starts_with(input, "../")) {
      input = input.substr(3);
    } else if (starts_with(input, "./")) {
      input = input.substr(2);
    }
    // B.  if the input buffer begins with a prefix of "/./" or "/.",
    //     where "." is a complete path segment, then replace that
    //     prefix with "/" in the input buffer; otherwise,
    else if (input == "/.") {
      input = "/";
    } else if (starts_with(input, "/./")) {
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
    } else if (starts_with(input, "/../")) {
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

} // namespace dart::common
