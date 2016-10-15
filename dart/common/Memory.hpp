/*
 * Copyright (c) 2014-2016, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
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

#ifndef DART_COMMON_MEMORY_HPP_
#define DART_COMMON_MEMORY_HPP_

#include <memory>

#include <Eigen/Core>

#include "dart/config.hpp"

#ifdef __GNUC__
  #define DART_ALIGNED(v) __attribute__ ((aligned(v)))
#elif defined(_MSC_VER)
  #define DART_ALIGNED(v) __declspec(align(v))
#else
  #define DART_ALIGNED(v)
#endif

#define DART_DEFINE_CREATE_UNIQUE(class_name)\
  /** Create unique instance of this class. */\
  template <typename... Args>\
  static std::unique_ptr<class_name> createUnique(Args&&... args)\
  {\
    return ::dart::common::make_unique<class_name>(std::forward<Args>(args)...);\
  }

#define DART_DEFINE_CREATE_SHARED(class_name)\
  /** Create shared instance of this class. */\
  template <typename... Args>\
  static std::shared_ptr<class_name> createShared(Args&&... args)\
  {\
    return ::std::make_shared<class_name>(std::forward<Args>(args)...);\
  }

#define DART_DEFINE_CREATE_ALIGNED_SHARED(class_name)\
  /** Create shared instance of this class. */\
  template <typename... Args>\
  static std::shared_ptr<class_name> createShared(Args&&... args)\
  {\
    return ::dart::common::make_aligned_shared<class_name>(\
        std::forward<Args>(args)...);\
  }

#define DART_DEFINE_CREATE(class_name)\
    DART_DEFINE_CREATE_SHARED(class_name)\
    DART_DEFINE_CREATE_UNIQUE(class_name)

#if DART_ENABLE_SIMD
  #define DART_DEFINE_CREATE_ALIGNED(class_name)\
      DART_DEFINE_CREATE_ALIGNED_SHARED(class_name)\
      DART_DEFINE_CREATE_UNIQUE(class_name)
#else
  #define DART_DEFINE_CREATE_ALIGNED(class_name)\
      DART_DEFINE_CREATE_SHARED(class_name)\
      DART_DEFINE_CREATE_UNIQUE(class_name)
#endif

namespace dart {
namespace common {

#if EIGEN_VERSION_AT_LEAST(3,2,1)

namespace detail {

/// Aligned allocator that is compatible with c++11
// Ref: https://bitbucket.org/eigen/eigen/commits/f5b7700
// TODO: Remove this and use Eigen::aligned_allocator once new version of Eigen
// is released with above commit.
template <class T>
class aligned_allocator_cpp11 : public std::allocator<T>
{
public:
  typedef std::size_t     size_type;
  typedef std::ptrdiff_t  difference_type;
  typedef T*              pointer;
  typedef const T*        const_pointer;
  typedef T&              reference;
  typedef const T&        const_reference;
  typedef T               value_type;

  template <class U>
  struct rebind
  {
    typedef aligned_allocator_cpp11<U> other;
  };

  aligned_allocator_cpp11()
    : std::allocator<T>() {}

  aligned_allocator_cpp11(const aligned_allocator_cpp11& other)
    : std::allocator<T>(other) {}

  template <class U>
  aligned_allocator_cpp11(const aligned_allocator_cpp11<U>& other)
    : std::allocator<T>(other) {}

  ~aligned_allocator_cpp11() {}

  pointer allocate(size_type num, const void* /*hint*/ = 0)
  {
    Eigen::internal::check_size_for_overflow<T>(num);
    return static_cast<pointer>( Eigen::internal::aligned_malloc(num * sizeof(T)) );
  }

  void deallocate(pointer p, size_type /*num*/)
  {
    Eigen::internal::aligned_free(p);
  }
};

} // namespace detail

template <typename _Tp, typename... _Args>
inline std::shared_ptr<_Tp> make_aligned_shared(_Args&&... __args)
{
  typedef typename std::remove_const<_Tp>::type _Tp_nc;
  return std::allocate_shared<_Tp>(detail::aligned_allocator_cpp11<_Tp_nc>(),
                                   std::forward<_Args>(__args)...);
}

#else

template <typename _Tp, typename... _Args>
inline std::shared_ptr<_Tp> make_aligned_shared(_Args&&... __args)
{
  typedef typename std::remove_const<_Tp>::type _Tp_nc;
  return std::allocate_shared<_Tp>(Eigen::aligned_allocator<_Tp_nc>(),
                                   std::forward<_Args>(__args)...);
}

#endif // EIGEN_VERSION_AT_LEAST(3,2,1)

//==============================================================================
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
// TODO(JS): This is a stopgap solution as it was omitted from C++11 as "partly
// an oversight". This can be replaced by std::make_unique<T> of the standard
// library when we migrate to using C++14.

} // namespace common
} // namespace dart

#endif // DART_COMMON_MEMORY_HPP_
