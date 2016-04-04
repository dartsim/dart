/*
 * Copyright (c) 2016, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
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

#ifndef DART_COMMON_STLHELPERS_H_
#define DART_COMMON_STLHELPERS_H_

#include <cassert>
#include <cstddef>
#include <memory>
#include <vector>
#include "dart/common/Console.h"

// Macro to suppress -Wunused-parameter and -Wunused-variable warnings in
// release mode when a variable is only used in assertions.
#define DART_UNUSED(x) do { (void)(x); } while (0)

namespace dart {
namespace common {

//==============================================================================
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
// TODO(JS): This is a stopgap solution as it was omitted from C++11 as "partly
// an oversight". This can be replaced by std::make_unique<T> of the standard
// library when we migrate to using C++14.

//==============================================================================
template <typename T>
static T getVectorObjectIfAvailable(size_t index, const std::vector<T>& vec)
{
  assert(index < vec.size());
  if(index < vec.size())
    return vec[index];

  return nullptr;
}

//==============================================================================
/// static_if_else allows the compiler to choose between two different possible
/// types at runtime based on a runtime boolean.
template <bool B, typename T_if, typename T_else>
struct static_if_else
{
  using type = T_else;
};

template <typename T_if, typename T_else>
struct static_if_else<true, T_if, T_else>
{
  using type = T_if;
};

//==============================================================================
/// Templated function for passing each entry in a std::vector<Data> into each
/// member of an array of Objects belonging to some Owner class.
///
/// The ObjectBase argument should be the base class of Object in which the
/// setData function is defined. In many cases, ObjectBase may be the same as
/// Object, but it is not always.
template <class Owner, class Object, class ObjectBase, class Data,
          size_t (Owner::*getNumObjects)() const,
          Object* (Owner::*getObject)(size_t),
          void (ObjectBase::*setData)(const Data&)>
void setAllMemberObjectData(Owner* owner, const std::vector<Data>& data)
{
  if(!owner)
  {
    dterr << "[setAllMemberObjectData] Attempting to set ["
          << typeid(Data).name() << "] of every [" << typeid(Object).name()
          << "] in a nullptr [" << typeid(Owner).name() << "]. Please report "
          << "this as a bug!\n";
    assert(false);
    return;
  }

  size_t numObjects = (owner->*getNumObjects)();

  if(data.size() != numObjects)
  {
    dtwarn << "[setAllMemberObjectData] Mismatch between the number of ["
           << typeid(Object).name() << "] member objects (" << numObjects
           << ") in the [" << typeid(Owner).name() << "] named ["
           << owner->getName() << "] (" << owner << ") and the number of ["
           << typeid(Object).name() << "] which is (" << data.size()
           << ") while setting [" << typeid(Data).name() << "]\n"
           << " -- We will set (" << std::min(numObjects, data.size())
           << ") of them.\n";
    numObjects = std::min(numObjects, data.size());
  }

  for(size_t i=0; i < numObjects; ++i)
    ((owner->*getObject)(i)->*setData)(data[i]);
}

//==============================================================================
/// Templated function for aggregating a std::vector<Data> out of each member of
/// an array of Objects belonging to some Owner class.
///
/// The ObjectBase argument should be the base class of Object in which the
/// getData function is defined. In many cases, ObjectBase may be the same as
/// Object, but it is not always.
template <class Owner, class Object, class ObjectBase, class Data,
          size_t (Owner::*getNumObjects)() const,
          const Object* (Owner::*getObject)(size_t) const,
          Data (ObjectBase::*getData)() const>
std::vector<Data> getAllMemberObjectData(const Owner* owner)
{
  if(!owner)
  {
    dterr << "[getAllMemberObjectData] Attempting to get the ["
          << typeid(Data).name() << "] from every [" << typeid(Object).name()
          << "] in a nullptr [" << typeid(Owner).name() << "]. Please report "
          << "this as a bug!\n";
    assert(false);
    return std::vector<Data>();
  }

  const size_t numObjects = (owner->*getNumObjects)();
  std::vector<Data> data;
  data.reserve(numObjects);

  for(size_t i=0; i < numObjects; ++i)
    data.push_back(((owner->*getObject)(i)->*getData)());

  return data;
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_EMPTY_H_
