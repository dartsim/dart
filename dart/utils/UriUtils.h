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
 * This file is provided under the following "BSD-style" License: *   Redistribution and use in source and binary forms, with or
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
#ifndef DART_UTILS_URIUTILS_H_
#define DART_UTILS_URIUTILS_H_

namespace dart {
namespace utils {

class UriComponent {
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


class Uri {
public:
  UriComponent mScheme;
  UriComponent mAuthority;
  UriComponent mPath;
  UriComponent mQuery;
  UriComponent mFragment;

  void clear();

  bool isPath() const;
  bool isRelativePath() const;

  void append(const std::string& _relativePath);
  void transform();


  bool fromString(const std::string& _input);
  bool fromStringOrPath(const std::string& _input);

  bool fromRelativeUri(const Uri& _base, const std::string& _relative);
  bool fromRelativeUri(const Uri& _base, const Uri& _relative,
                       bool _strict = false);

  std::string toString() const;

  static std::string getUri(const std::string& _input);

private:
  // These are helper functions for implementing transform();

  static std::string mergePaths(const Uri& _base, const Uri& _relative);
  static std::string removeDotSegments(const std::string& _path);
};

} // namespace utils
} // namespace dart

#endif // ifndef DART_UTILS_URIUTILS_H_
