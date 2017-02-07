/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
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

#ifndef DART_COMMON_MEMORY_HPP_
#define DART_COMMON_MEMORY_HPP_

#include <memory>

namespace dart {
namespace common {

template <typename _Tp, typename... _Args>
std::shared_ptr<_Tp> make_aligned_shared(_Args&&... __args);

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args);

} // namespace common
} // namespace dart

#define DART_SHARED_PTR_CREATE_FUNCTION_NAME createShared
#define DART_UNIQUE_PTR_CREATE_FUNCTION_NAME createUnique

// Define static creator function that returns a smart pointer to an object
#define _DART_DEFINE_OBJECT_CREATOR(                                           \
    class_name, func_name, ptr_type, creator)                                  \
  template <typename... Args>                                                  \
  static ptr_type<class_name> func_name(Args&&... args)                        \
  {                                                                            \
    return creator<class_name>(std::forward<Args>(args)...);                   \
  }

// Define static creator function that returns a smart pointer to an object with
// protected constructor
#define _DART_DEFINE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(                        \
    class_name, func_name, ptr_type, creator)                                  \
private:                                                                       \
  struct private_structure                                                     \
  {                                                                            \
    explicit private_structure()                                               \
    {                                                                          \
    }                                                                          \
  };                                                                           \
                                                                               \
public:                                                                        \
  template <typename... Args>                                                  \
  class_name(const private_structure&, Args&&... args)                         \
    : class_name(std::forward<Args>(args)...)                                  \
  {                                                                            \
  }                                                                            \
  template <typename... Args>                                                  \
  static ptr_type<class_name> func_name(Args&&... args)                        \
  {                                                                            \
    return creator<class_name>(                                                \
        private_structure{}, std::forward<Args>(args)...);                     \
  }

// Define static creator function that returns std::shared_ptr to an object.
// This static functions will be defined: createShared()
#define DART_DEFINE_SHARED_OBJECT_CREATOR(class_name)                          \
  /*! Create shared instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR(                                                 \
      class_name,                                                              \
      DART_SHARED_PTR_CREATE_FUNCTION_NAME,                                    \
      std::shared_ptr,                                                         \
      std::make_shared)

// Define static creator function that returns std::shared_ptr to an object
// where the constructor is protected.
// This static functions will be defined: createShared()
#define DART_DEFINE_SHARED_OBJECT_CREATOR_FOR_PROTECTED_CTOR(class_name)       \
  /*! Create shared instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(                              \
      class_name,                                                              \
      DART_SHARED_PTR_CREATE_FUNCTION_NAME,                                    \
      std::shared_ptr,                                                         \
      std::make_shared)

// Define static creator function that returns std::shared_ptr to an object
// requires aligned memory allocation.
// This static functions will be defined: createShared()
#define DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(class_name)                  \
  /*! Create shared instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR(                                                 \
      class_name,                                                              \
      DART_SHARED_PTR_CREATE_FUNCTION_NAME,                                    \
      std::shared_ptr,                                                         \
      ::dart::common::make_aligned_shared)

// Define static creator function that returns std::shared_ptr to an object
// requires aligned memory allocation where the constructor is protected.
// This static functions will be defined: createShared()
#define DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR_FOR_PROTECTED_CTOR(          \
      class_name)                                                              \
  /*! Create shared instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(                              \
      class_name,                                                              \
      DART_SHARED_PTR_CREATE_FUNCTION_NAME,                                    \
      std::shared_ptr,                                                         \
      ::dart::common::make_aligned_shared)

// Define static creator function that returns std::unique_ptr to an object
#define DART_DEFINE_UNIQUE_OBJECT_CREATOR(class_name)                          \
  /*! Create unique instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR(                                                 \
      class_name,                                                              \
      DART_UNIQUE_PTR_CREATE_FUNCTION_NAME,                                    \
      std::unique_ptr,                                                         \
      ::dart::common::make_unique)

// Define static creator function that returns std::unique_ptr to an object
// where the constructor is protected
#define DART_DEFINE_UNIQUE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(class_name)       \
  /*! Create unique instance of this class */                                  \
  _DART_DEFINE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(                              \
      class_name,                                                              \
      DART_UNIQUE_PTR_CREATE_FUNCTION_NAME,                                    \
      std::unique_ptr,                                                         \
      ::dart::common::make_unique)

// Define two static creator functions that returns std::unique_ptr and
// std::unique_ptr, respectively, to an object
#define DART_DEFINE_OBJECT_CREATORS(class_name)                                \
  DART_DEFINE_SHARED_OBJECT_CREATOR(class_name)                                \
  DART_DEFINE_UNIQUE_OBJECT_CREATOR(class_name)

// Define two static creator functions that returns std::unique_ptr and
// std::unique_ptr, respectively, to an object where the constructor is
// protected
#define DART_DEFINE_OBJECT_CREATORS_FOR_PROTECTED_CTOR(X)                      \
  DART_DEFINE_SHARED_OBJECT_CREATOR_FOR_PROTECTED_CTOR(X)                      \
  DART_DEFINE_UNIQUE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(X)

// Define two static creator functions that returns std::unique_ptr and
// std::unique_ptr, respectively, to an object
#if DART_ENABLE_SIMD
  #define DART_DEFINE_ALIGNED_OBJECT_CREATORS(class_name)                      \
    DART_DEFINE_ALIGNED_SHARED_OBJECT_CREATOR(class_name)                      \
    DART_DEFINE_UNIQUE_OBJECT_CREATOR(class_name)
#else
  #define DART_DEFINE_ALIGNED_OBJECT_CREATORS(class_name)                      \
    DART_DEFINE_OBJECT_CREATORS(class_name)
#endif

// Define two static creator functions that returns std::unique_ptr and
// std::unique_ptr, respectively, to an object where the constructor is
// protected
#if DART_ENABLE_SIMD
  #define DART_DEFINE_ALIGNED_OBJECT_CREATORS_FOR_PROTECTED_CTOR(class_name)   \
    DART_DEFINE_CREATE_ALIGNED_PTR_SHARED_FOR_PROTECTED_CTOR(class_name)       \
    DART_DEFINE_UNIQUE_OBJECT_CREATOR_FOR_PROTECTED_CTOR(class_name)
#else
  #define DART_DEFINE_ALIGNED_OBJECT_CREATORS_FOR_PROTECTED_CTOR(class_name)   \
    DART_DEFINE_OBJECT_CREATORS_FOR_PROTECTED_CTOR(class_name)
#endif

#include "dart/common/detail/Memory-impl.hpp"

#endif // DART_COMMON_MEMORY_HPP_
