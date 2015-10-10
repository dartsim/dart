/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael Koval <mkoval@cs.cmu.edu>
 *
 * Georgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
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

#ifndef DART_COMMON_URI_H_
#define DART_COMMON_URI_H_

#include <string>

namespace dart {
namespace common {

class UriComponent final
{
public:
  using value_type = std::string;
  using reference_type = value_type&;
  using reference_const_type = const value_type&;
  using pointer_type = value_type*;
  using pointer_const_type = const value_type*;

  UriComponent();
  UriComponent(reference_const_type _value);

  operator bool() const;

  bool operator !() const;

  UriComponent& operator =(reference_const_type _value);

  reference_type operator *();
  reference_const_type operator *() const;

  pointer_type operator ->();
  pointer_const_type operator ->() const;

  void assign(reference_const_type _value);
  void reset();

  reference_type get();
  reference_const_type get() const;

  reference_type get_value_or(reference_type _default);
  reference_const_type get_value_or(reference_const_type _default) const;

private:
  bool mExists;
  std::string mValue;
};


class Uri final
{
public:
  /// Scheme, e.g. 'http', 'file', 'package'
  UriComponent mScheme;

  /// Authority, e.g. 'google.com', 'en.wikipedia.org'
  UriComponent mAuthority;

  /// Path, e.g. '/index.html', '/foo/bar.txt'
  UriComponent mPath;

  /// Query string, i.e. the part of the URI after the ?
  UriComponent mQuery;

  /// Fragment, e.g. the part of the URI after the #
  UriComponent mFragment;

  /// Constructor
  Uri() = default;

  /// Implicit constructor that takes a string as a filename. Internally, this
  /// is equivalent to calling fromString(_input) after default constructor.
  Uri(const std::string& _input);

  /// Clear the URI by reset()ing all components.
  void clear();

  /// Parse URI from a string; return success.
  bool fromString(const std::string& _input);

  /// Parse a URI or local path (i.e. URI with no schema) from a string.
  bool fromStringOrPath(const std::string& _input);

  /// Resolve a relative path reference; return success.
  bool fromRelativeUri(const Uri& _base, const std::string& _relative,
                       bool _strict = false);

  /// Resolve a relative path reference; return success.
  bool fromRelativeUri(const Uri& _base, const Uri& _relative,
                       bool _strict = false);

  /// Combine the parts of the URI into a string.
  std::string toString() const;

  /// Parse a URI from a string; return an empty string on failure.
  static std::string getUri(const std::string& _input);

  /// Resolve a relative path reference; return an empty string on failure.
  static std::string getRelativeUri(const std::string& _base,
                                    const std::string& _relative,
                                    bool _strict = false);

  /// Resolve a relative path reference; return an empty string on failure.
  static std::string getRelativeUri(const Uri& _base,
                                    const Uri& _relative,
                                    bool _strict = false);

  /// Create URI from a string.
  static Uri createFromString(const std::string& _input);

  /// Create URI from a string.
  static Uri createFromStringOrPath(const std::string& _input);

  /// Create URI from a string.
  static Uri createFromRelativeUri(const std::string& _base,
                                   const std::string& _relative,
                                   bool _strict = false);

  /// Create URI from a string.
  static Uri createFromRelativeUri(const Uri& _base,
                                   const Uri& _relative,
                                   bool _strict = false);

private:
  /// Implement section 5.2.3 of RFC 3986.
  static std::string mergePaths(const Uri& _base, const Uri& _relative);

  /// Implement section 5.2.4 of RFC 3986.
  static std::string removeDotSegments(const std::string& _path);
};

} // namespace common
} // namespace dart

#endif // ifndef DART_COMMON_URI_H_
