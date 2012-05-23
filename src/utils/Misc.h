/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sehoon.ha@gmail.com>
 * Date: 06/12/2011
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

#ifndef UTILS_MISC_H
#define UTILS_MISC_H

// Epsilon definition
#ifndef EPSILON
#define EPSILON 1.0e-6
#endif

// PI definition
#ifndef M_PI
#define M_PI 3.141592653589793
#endif
#ifndef M_PI_2
#define M_PI_2 M_PI*0.5;
#endif

// Define a macro which returns the "pretty" name of function
// It should include names of class and functions
#if defined(__GNUC__) || defined(__clang__) || defined(__llvm__)
#define FUNCTION_NAME() (__PRETTY_FUNCTION__)
#elif defined(_MSC_VER)
#define FUNCTION_NAME() (__FUNCSIG__)
#else
#error "Unrecognized compiler. Could not determine the function name macro for your compiler."
#endif

// Safe Release Ptr
// Warning!!! You should put ";" after this macros!!!
#define SAFE_RELEASE_PTR(x) do{if(x) {delete x; x = NULL;}}while(0)

// For generate getter/setter method
#define GETSET(type, var)                       \
    public:                                     \
    type var() const {                          \
        return var##_;                          \
    }                                           \
    void set_##var(type _val) {                 \
        var##_ = _val;                          \
    }                                           \
private:                                        \
type var##_                 


#endif // #ifndef UTILS_MISC_H


