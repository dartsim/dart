/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sumit Jain <sumit@cc.gatech.edu>,
 *            Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/26/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
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

#ifndef DART_COMMON_UTILSCODE_H
#define DART_COMMON_UTILSCODE_H

#include <string>
#include <vector>
#include <iterator>
#include <algorithm> // for copy
#include <iostream>
#include <istream>
#include <fstream>
#include <sstream>

namespace dart {
namespace common {

/// @brief Breaks the _str string into a vector of _tokens using delimiters from
/// input _delimiters
void tokenize(const std::string& _str,
              std::vector<std::string>& _tokens,
              const std::string& _delimiters = " ");    //

/// @brief
template <class T>
inline std::vector<T> stringToVec(const std::string& _str)
{
    std::vector<T> myVec;
    std::istringstream iss (_str, std::istringstream::in);
    copy(std::istream_iterator<T>(iss),
         std::istream_iterator<T>(),
         back_inserter(myVec));
    //check http://www.sgi.com/tech/stl/istream_iterator.html
    return myVec;
}

/// @brief
template <typename T_POINTER>
inline void swapPointers(T_POINTER*& _p1, T_POINTER*& _p2)
{
    T_POINTER* temp = _p2;
    _p2 = _p1;
    _p1 = temp;
}

/// @brief
inline bool isNaN(double _v)
{
    return _v != _v;
}

} // namespace common
} // namespace dart

#endif // #ifndef DART_COMMON_UTILSCODE_H
