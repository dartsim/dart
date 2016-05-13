/*
 * Copyright (c) 2015-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2015-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
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

#include <cassert>
#include <sstream>
#include "dart/common/Console.hpp"
#include "dart/common/Uri.hpp"

// std::regex is only implemented in GCC 4.9 and above; i.e. libstdc++ 6.0.20
// or above. In fact, it contains major bugs in GCC 4.8 [1]. There is no
// reliable way to test the version of libstdc++ when building with Clang [2],
// so we'll fall back on Boost.Regex when using libstdc++.
//
// [1] http://stackoverflow.com/a/12665408/111426
// [2] http://stackoverflow.com/q/31506594/111426
//
#ifdef __GLIBCXX__

#include <boost/regex.hpp>

using boost::regex;
using boost::regex_constants::match_continuous;
using boost::regex_match;
using boost::regex_search;
using boost::smatch;
using boost::ssub_match;

#else

#include <regex>

using std::regex;
using std::regex_constants::match_continuous;
using std::regex_match;
using std::regex_search;
using std::smatch;
using std::ssub_match;

#endif

static bool startsWith(const std::string& _target, const std::string& _prefix)
{
  return _target.substr(0, _prefix.size()) == _prefix;
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
bool UriComponent::operator !() const
{
  return !mExists;
}

//==============================================================================
auto UriComponent::operator =(reference_const_type _value) -> UriComponent&
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
  assert(mExists);
  return mValue;
}

//==============================================================================
auto UriComponent::get() const -> reference_const_type
{
  assert(mExists);
  return mValue;
}

//==============================================================================
auto UriComponent::get_value_or(reference_type _default) -> reference_type
{
  if(mExists)
    return mValue;
  else
    return _default;
}

//==============================================================================
auto UriComponent::get_value_or(reference_const_type _default) const
  -> reference_const_type
{
  if(mExists)
    return mValue;
  else
    return _default;
}


/*
 * Uri
 */

//==============================================================================
Uri::Uri(const std::string& _input)
{
  if (!fromStringOrPath(_input))
    dtwarn << "[Uri::Uri] Failed parsing URI '" << _input << "'.\n";

  // We don't need to clear since fromStringOrPath() does not set any component
  // on failure.
}

//==============================================================================
Uri::Uri(const char* _input)
{
  if (!fromStringOrPath(std::string(_input)))
    dtwarn << "[Uri::Uri] Failed parsing URI '" << _input << "'.\n";

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
bool Uri::fromString(const std::string& _input)
{
  // This is regex is from Appendix B of RFC 3986.
  static regex uriRegex(
    R"END(^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\?([^#]*))?(#(.*))?)END",
    regex::extended | regex::optimize);
  static const std::size_t schemeIndex = 2;
  static const std::size_t authorityIndex = 4;
  static const std::size_t pathIndex = 5;
  static const std::size_t queryIndex = 7;
  static const std::size_t fragmentIndex = 9;

  clear();

  smatch matches;
  if(!regex_match(_input, matches, uriRegex))
    return false;

  assert(matches.size() > schemeIndex);
  const ssub_match& schemeMatch = matches[schemeIndex];
  if(schemeMatch.matched)
    mScheme = schemeMatch;

  assert(matches.size() > authorityIndex);
  const ssub_match& authorityMatch = matches[authorityIndex];
  if(authorityMatch.matched)
    mAuthority = authorityMatch;

  assert(matches.size() > pathIndex);
  const ssub_match& pathMatch = matches[pathIndex];
  if(pathMatch.matched)
    mPath = pathMatch;

  assert(matches.size() > queryIndex);
  const ssub_match& queryMatch = matches[queryIndex];
  if(queryMatch.matched)
    mQuery = queryMatch;

  assert(matches.size() > fragmentIndex);
  const ssub_match& fragmentMatch = matches[fragmentIndex];
  if (fragmentMatch.matched)
    mFragment = fragmentMatch;

  return true;
}

//==============================================================================
bool Uri::fromPath(const std::string& _path)
{
  // TODO(JS): We might want to check validity of _path.

  static const std::string fileScheme("file://");

#ifdef _WIN32
  // Replace backslashes (from Windows paths) with forward slashes.
  std::string unixPath = _path;
  std::replace(std::begin(unixPath), std::end(unixPath), '\\', '/');

  return fromString(fileScheme + "/" + _path);
#else
  return fromString(fileScheme + _path);
#endif
}

//==============================================================================
bool Uri::fromStringOrPath(const std::string& _input)
{
  // TODO(JS): Need to check if _input is an "absolute" path?

#ifdef _WIN32

  // Assume that any URI begin with pattern [SINGLE_LETTER]:[/ or \\] is an
  // absolute path.
  static regex windowsPathRegex(R"END([a-zA-Z]:[/|\\])END");
  bool isPath = regex_search(_input, windowsPathRegex, match_continuous);

  if (isPath)
    return fromPath(_input);

#else

  // Assume that any URI without a scheme is a path.
  static regex uriSchemeRegex(R"END(^(([^:/?#]+):))END");
  bool noScheme = !regex_search(_input, uriSchemeRegex, match_continuous);

  if (noScheme)
    return fromPath(_input);

#endif

  return fromString(_input);
}

//==============================================================================
bool Uri::fromRelativeUri(const std::string& _base,
                          const std::string& _relative, bool _strict)
{
  Uri baseUri;
  if(!baseUri.fromString(_base))
  {
    dtwarn << "[Uri::fromRelativeUri] Failed parsing base URI '"
           << _base << "'.\n";
    clear();
    return false;
  }

  return fromRelativeUri(baseUri, _relative, _strict);
}

//==============================================================================
bool Uri::fromRelativeUri(const char* _base,
                          const char* _relative, bool _strict)
{
  return fromRelativeUri(std::string(_base), std::string(_relative), _strict);
}

//==============================================================================
bool Uri::fromRelativeUri(const Uri& _base, const std::string& _relative,
                          bool _strict)
{
  Uri relativeUri;
  if(!relativeUri.fromString(_relative))
  {
    dtwarn << "[Uri::fromRelativeUri] Failed parsing relative URI '"
           << _relative << "'.\n";
    clear();
    return false;
  }

  return fromRelativeUri(_base, relativeUri, _strict);
}

//==============================================================================
bool Uri::fromRelativeUri(const Uri& _base, const char* _relative, bool _strict)
{
  return fromRelativeUri(_base, std::string(_relative), _strict);
}

//==============================================================================
bool Uri::fromRelativeUri(const Uri& _base, const Uri& _relative,
                          bool /*_strict*/)
{
  assert(_base.mPath && "The path component is always defined.");
  assert(_relative.mPath && "The path component is always defined.");

  // TODO If (!_strict && _relative.mScheme == _base.mScheme), then we need to
  // enable backwards compatability.

  // This directly implements the psueocode in Section 5.2.2. of RFC 3986.
  if(_relative.mScheme)
  {
    mScheme = _relative.mScheme;
    mAuthority = _relative.mAuthority;
    mPath = removeDotSegments(*_relative.mPath);
    mQuery = _relative.mQuery;
  }
  else
  {
    if(_relative.mAuthority)
    {
      mAuthority = _relative.mAuthority;
      mPath = removeDotSegments(*_relative.mPath);
      mQuery = _relative.mQuery;
    }
    else
    {
      if(_relative.mPath->empty())
      {
        mPath = _base.mPath;

        if(_relative.mQuery)
          mQuery = _relative.mQuery;
        else
          mQuery = _base.mQuery;
      }
      else
      {
        if(_relative.mPath->front() == '/')
          mPath = removeDotSegments(*_relative.mPath);
        else
          mPath = removeDotSegments(mergePaths(_base, _relative));

        mQuery = _relative.mQuery;
      }

      mAuthority = _base.mAuthority;
    }

    mScheme = _base.mScheme;
  }

  mFragment = _relative.mFragment;
  return true;
}

//==============================================================================
std::string Uri::toString() const
{
  // This function implements the pseudo-code from Section 5.3 of RFC 3986.
  std::stringstream output;

  if(mScheme)
    output << *mScheme << ":";

  if(mAuthority)
    output << "//" << *mAuthority;

  output << mPath.get_value_or("");

  if(mQuery)
    output << "?" << *mQuery;

  if(mFragment)
    output << "#" << *mFragment;

  return output.str();
}

//==============================================================================
Uri Uri::createFromString(const std::string& _input)
{
  Uri uri;
  if(!uri.fromString(_input))
  {
    dtwarn << "[Uri::createFromString] Failed parsing URI '" << _input
           << "'.\n";
  }

  // We don't need to clear uri since fromString() does not set any component
  // on failure.

  return uri;
}

//==============================================================================
Uri Uri::createFromPath(const std::string& _path)
{
  Uri fileUri;
  if(!fileUri.fromPath(_path))
  {
    dtwarn << "[Uri::createFromPath] Failed parsing local path '" << _path
           << "'.\n";
  }

  // We don't need to clear uri since fromString() does not set any component
  // on failure.

  return fileUri;
}

//==============================================================================
Uri Uri::createFromStringOrPath(const std::string& _input)
{
  Uri uri;
  if(!uri.fromStringOrPath(_input))
  {
    dtwarn << "[Uri::createFromString] Failed parsing URI '" << _input
           << "'.\n";
  }

  // We don't need to clear uri since fromString() does not set any component
  // on failure.

  return uri;
}

//==============================================================================
Uri Uri::createFromRelativeUri(const std::string& _base,
                               const std::string& _relative, bool _strict)
{
  Uri mergedUri;
  if(!mergedUri.fromRelativeUri(_base, _relative, _strict))
  {
    dtwarn << "[Uri::createFromRelativeUri] Failed merging URI '" << _relative
           << "' with base URI '" << _base << "'.\n";
  }

  // We don't need to clear mergedUri since fromRelativeUri() does not set any
  // component on failure.

  return mergedUri;
}

//==============================================================================
Uri Uri::createFromRelativeUri(const Uri& _base,
                               const std::string& _relative, bool _strict)
{
  Uri mergedUri;
  if(!mergedUri.fromRelativeUri(_base, _relative, _strict))
  {
    dtwarn << "[Uri::createFromRelativeUri] Failed merging URI '" << _relative
           << "' with base URI '" << _base.toString() << "'.\n";
  }

  // We don't need to clear mergedUri since fromRelativeUri() does not set any
  // component on failure.

  return mergedUri;
}

//==============================================================================
Uri Uri::createFromRelativeUri(const Uri& _baseUri,
                               const Uri& _relativeUri, bool _strict)
{
  Uri mergedUri;
  if(!mergedUri.fromRelativeUri(_baseUri, _relativeUri, _strict))
  {
    dtwarn << "[Uri::createFromRelativeUri] Failed merging URI '"
           << _relativeUri.toString() << "' with base URI '"
           << _baseUri.toString() << "'.\n";
  }

  // We don't need to clear mergedUri since fromRelativeUri() does not set any
  // component on failure.

  return mergedUri;
}

//==============================================================================
std::string Uri::getUri(const std::string& _input)
{
  Uri uri;
  if(uri.fromStringOrPath(_input))
    return uri.toString();
  else
    return "";
}

//==============================================================================
std::string Uri::getRelativeUri(
  const std::string& _base, const std::string& _relative, bool _strict)
{
  Uri mergedUri;
  if(!mergedUri.fromRelativeUri(_base, _relative, _strict))
    return "";
  else
    return mergedUri.toString();
}

//==============================================================================
std::string Uri::getRelativeUri(
  const Uri& _base, const std::string& _relative, bool _strict)
{
  Uri mergedUri;
  if(!mergedUri.fromRelativeUri(_base, _relative, _strict))
    return "";
  else
    return mergedUri.toString();
}

//==============================================================================
std::string Uri::getRelativeUri(
    const Uri& _baseUri, const Uri& _relativeUri, bool _strict)
{
  Uri mergedUri;
  if(!mergedUri.fromRelativeUri(_baseUri, _relativeUri, _strict))
    return "";
  else
    return mergedUri.toString();
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
  if (mScheme.get_value_or("") == "file")
  {
    const std::string& filesystemPath = getPath();

    if (!filesystemPath.empty() && filesystemPath[0] == '/')
      return filesystemPath.substr(1);
  }
#endif

  return getPath();
}

//==============================================================================
std::string Uri::mergePaths(const Uri& _base, const Uri& _relative)
{
  assert(_base.mPath && "The path component is always defined.");
  assert(_relative.mPath && "The path component is always defined.");

  // This directly implements the logic from Section 5.2.3. of RFC 3986.
  if(_base.mAuthority && _base.mPath->empty())
    return "/" + *_relative.mPath;
  else
  {
    const std::size_t index = _base.mPath->find_last_of('/');
    if(index != std::string::npos)
      return _base.mPath->substr(0, index + 1) + *_relative.mPath;
    else
      return *_relative.mPath;
  }
}

//==============================================================================
std::string Uri::removeDotSegments(const std::string& _path)
{
  // 1.  The input buffer is initialized with the now-appended path
  //     components and the output buffer is initialized to the empty
  //     string.
  std::string input = _path;
  std::string output;

  // 2.  While the input buffer is not empty, loop as follows:
  while(!input.empty())
  {
    // A.  If the input buffer begins with a prefix of "../" or "./",
    //     then remove that prefix from the input buffer; otherwise,
    if(startsWith(input, "../"))
    {
      input = input.substr(3);
    }
    else if(startsWith(input, "./"))
    {
      input = input.substr(2);
    }
    // B.  if the input buffer begins with a prefix of "/./" or "/.",
    //     where "." is a complete path segment, then replace that
    //     prefix with "/" in the input buffer; otherwise,
    else if(input == "/.")
    {
      input = "/";
    }
    else if(startsWith(input, "/./"))
    {
      input = "/" + input.substr(3);
    }
    // C.  if the input buffer begins with a prefix of "/../" or "/..",
    //     where ".." is a complete path segment, then replace that
    //     prefix with "/" in the input buffer and remove the last
    //     segment and its preceding "/" (if any) from the output
    //     buffer; otherwise,
    else if(input  == "/..")
    {
      input = "/";

      std::size_t index = output.find_last_of('/');
      if(index != std::string::npos)
        output = output.substr(0, index);
      else
        output = "";
    }
    else if(startsWith(input, "/../"))
    {
      input = "/" + input.substr(4);

      std::size_t index = output.find_last_of('/');
      if(index != std::string::npos)
        output = output.substr(0, index);
      else
        output = "";
    }
    // D.  if the input buffer consists only of "." or "..", then remove
    //     that from the input buffer; otherwise,
    else if(input == "." || input == "..")
    {
      input = "";
    }
    // E.  move the first path segment in the input buffer to the end of
    //     the output buffer, including the initial "/" character (if
    //     any) and any subsequent characters up to, but not including,
    //     the next "/" character or the end of the input buffer.
    else 
    {
      // Start at index one so we keep the leading '/'.
      std::size_t index = input.find_first_of('/', 1); 
      output += input.substr(0, index);

      if(index != std::string::npos)
        input = input.substr(index);
      else
        input = "";
    }
  }

  // 3.  Finally, the output buffer is returned as the result of
  //     remove_dot_segments.
  return output;
}

} // namespace common
} // namespace dart
