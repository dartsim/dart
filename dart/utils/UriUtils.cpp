#include <cassert>
#include <regex>
#include <sstream>
#include "dart/common/Console.h"
#include "UriUtils.h"

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
using boost::regex_match;
using boost::smatch;
using boost::ssub_match;

#else

#include <regex>

using std::regex;
using std::regex_match;
using std::smatch;
using std::ssub_match;

#endif

namespace dart {
namespace utils {

/*
 * UriComponent
 */
UriComponent::UriComponent()
{
  reset();
}

UriComponent::UriComponent(reference_const_type _value)
{
  assign(_value);  
}

UriComponent::operator bool() const
{
  return mExists;
}

bool UriComponent::operator !() const
{
  return !mExists;
}

auto UriComponent::operator =(reference_const_type _value) -> UriComponent&
{
  assign(_value);
  return *this;
}

auto UriComponent::operator*() -> reference_type
{
  return get();
}

auto UriComponent::operator*() const -> reference_const_type
{
  return get();
}

auto UriComponent::operator->() -> pointer_type
{
  return &get();
}

auto UriComponent::operator->() const -> pointer_const_type
{
  return &get();
}

void UriComponent::assign(reference_const_type _value)
{
  mExists = true;
  mValue = _value;
}

void UriComponent::reset()
{
  mExists = false;
}

auto UriComponent::get() -> reference_type
{
  assert(mExists);
  return mValue;
}

auto UriComponent::get() const -> reference_const_type
{
  assert(mExists);
  return mValue;
}

auto UriComponent::get_value_or(reference_type _default) -> reference_type
{
  if(mExists)
    return mValue;
  else
    return _default;
}

auto UriComponent::get_value_or(reference_const_type _default) const -> reference_const_type
{
  if(mExists)
    return mValue;
  else
    return _default;
}


/*
 * Uri
 */
void Uri::clear()
{
  mScheme.reset();
  mAuthority.reset();
  mPath.reset();
  mQuery.reset();
  mFragment.reset();
}

bool Uri::fromString(const std::string& _input)
{
  // This is regex is from Appendix B of RFC 3986.
  static regex uriRegex(
    R"END(^(([^:/?#]+):)?(//([^/?#]*))?([^?#]*)(\?([^#]*))?(#(.*))?)END",
    regex::extended);
  static const size_t schemeIndex = 2;
  static const size_t authorityIndex = 4;
  static const size_t pathIndex = 5;
  static const size_t queryIndex = 7;
  static const size_t fragmentIndex = 9;

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

bool Uri::fromStringOrPath(const std::string& _input)
{
  if (!fromString(_input))
    return false;

  // Assume that any URI without a scheme is a path.
  if (!mScheme && mPath)
  {
    mScheme = "file";

    // Replace backslashes (from Windows paths) with forward slashes.
    std::replace(std::begin(*mPath), std::end(*mPath), '\\', '/');
  }

  return true;
}

std::string Uri::getUri(const std::string& _input)
{
  Uri uri;
  if(uri.fromStringOrPath(_input))
    return uri.toString();
  else
    return "";
}

} // namespace utils
} // namespace dart
