/*
 * Copyright (c) 2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2017, Personal Robotics Lab, Carnegie Mellon University
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

#ifndef DART_COMMON_CREATE_HPP_
#define DART_COMMON_CREATE_HPP_

#include <memory>
#include "dart/common/detail/Create.hpp"

#define DART_DEFINE_GENERIC_CREATOR(class_name)                                \
  template <template <typename...> class Container, typename... Args>          \
  static Container<class_name> create(Args&&... args)                          \
  {                                                                            \
    return ::dart::common::detail::CreateImpl<                                 \
       Container, class_name, Args...>::run(std::forward<Args>(args)...);      \
  }

#define DART_DEFINE_STD_UNIQUE_PTR_CREATOR(class_name, func_name)              \
  template <typename... Args>                                                  \
  static std::unique_ptr<class_name> func_name(Args&&... args)                 \
  {                                                                            \
    return create<std::unique_ptr>(std::forward<Args>(args)...);               \
  }

#define DART_DEFINE_STD_SHARED_PTR_CREATOR(class_name, func_name)              \
  template <typename... Args>                                                  \
  static std::shared_ptr<class_name> func_name(Args&&... args)                 \
  {                                                                            \
    return create<std::shared_ptr>(std::forward<Args>(args)...);               \
  }

#endif // DART_COMMON_CREATE_HPP_
