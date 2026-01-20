/*
 * Copyright (c) 2011, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#ifndef DART_COMMON_DETAIL_NAMEMANAGER_HPP_
#define DART_COMMON_DETAIL_NAMEMANAGER_HPP_

#include "dart/common/macros.hpp"

#include <dart/common/logging.hpp>
#include <dart/common/name_manager.hpp>

#include <sstream>

#include <cassert>

namespace dart {
namespace common {

//==============================================================================
template <class T>
NameManager<T>::NameManager(
    std::string_view managerName, std::string_view defaultName)
  : mManagerName(managerName),
    mDefaultName(defaultName),
    mNameBeforeNumber(true),
    mPrefix(""),
    mInfix("("),
    mAffix(")")
{
  // Do nothing
}

//==============================================================================
template <class T>
bool NameManager<T>::setPattern(std::string_view newPattern)
{
  std::size_t name_start = newPattern.find("%s");
  std::size_t number_start = newPattern.find("%d");

  if (name_start == std::string_view::npos
      || number_start == std::string_view::npos)
    return false;

  if (name_start < number_start)
    mNameBeforeNumber = true;
  else
    mNameBeforeNumber = false;

  std::size_t prefix_end = std::min(name_start, number_start);
  std::size_t infix_end = std::max(name_start, number_start);

  mPrefix = std::string(newPattern.substr(0, prefix_end));
  mInfix = std::string(
      newPattern.substr(prefix_end + 2, infix_end - prefix_end - 2));
  mAffix = std::string(newPattern.substr(infix_end + 2));

  return true;
}

//==============================================================================
template <class T>
std::string NameManager<T>::issueNewName(std::string_view name) const
{
  if (!hasName(name))
    return std::string(name);

  int count = 1;
  std::string newName;
  do {
    std::stringstream ss;
    if (mNameBeforeNumber)
      ss << mPrefix << name << mInfix << count++ << mAffix;
    else
      ss << mPrefix << count++ << mInfix << name << mAffix;
    newName = ss.str();
  } while (hasName(newName));

  DART_INFO(
      "({}) The name [{}] is a duplicate, so it has been renamed to [{}]",
      mManagerName,
      name,
      newName);

  return newName;
}

//==============================================================================
template <class T>
std::string NameManager<T>::issueNewNameAndAdd(
    std::string_view name, const T& obj)
{
  const std::string_view checkEmpty
      = name.empty() ? std::string_view(mDefaultName) : name;
  const std::string newName = issueNewName(checkEmpty);
  addName(newName, obj);

  return newName;
}

//==============================================================================
template <class T>
bool NameManager<T>::addName(std::string_view name, const T& obj)
{
  if (name.empty()) {
    DART_WARN("({}) Empty name is not allowed!", mManagerName);
    return false;
  }

  if (hasName(name)) {
    DART_WARN("({}) The name [{}] already exists!", mManagerName, name);
    return false;
  }

  const std::string nameString(name);
  mMap.insert(std::pair<std::string, T>(nameString, obj));
  mReverseMap.insert(std::pair<T, std::string>(obj, nameString));

  DART_ASSERT(mReverseMap.size() == mMap.size());

  return true;
}

//==============================================================================
template <class T>
bool NameManager<T>::removeName(std::string_view name)
{
  DART_ASSERT(mReverseMap.size() == mMap.size());

  auto it = mMap.find(name);

  if (it == mMap.end())
    return false;

  auto rit = mReverseMap.find(it->second);

  if (rit != mReverseMap.end())
    mReverseMap.erase(rit);

  mMap.erase(it);

  return true;
}

//==============================================================================
template <class T>
bool NameManager<T>::removeObject(const T& obj)
{
  DART_ASSERT(mReverseMap.size() == mMap.size());

  typename std::map<T, std::string>::iterator rit = mReverseMap.find(obj);

  if (rit == mReverseMap.end())
    return false;

  auto it = mMap.find(rit->second);
  if (it != mMap.end())
    mMap.erase(it);

  mReverseMap.erase(rit);

  return true;
}

//==============================================================================
template <class T>
void NameManager<T>::removeEntries(std::string_view name, const T& obj)
{
  removeObject(obj);
  removeName(name);
}

//==============================================================================
template <class T>
void NameManager<T>::clear()
{
  mMap.clear();
  mReverseMap.clear();
}

//==============================================================================
template <class T>
bool NameManager<T>::hasName(std::string_view name) const
{
  return mMap.contains(name);
}

//==============================================================================
template <class T>
bool NameManager<T>::hasObject(const T& obj) const
{
  return mReverseMap.contains(obj);
}

//==============================================================================
template <class T>
std::size_t NameManager<T>::getCount() const
{
  return mMap.size();
}

//==============================================================================
template <class T>
T NameManager<T>::getObject(std::string_view name) const
{
  auto result = mMap.find(name);

  if (result != mMap.end())
    return result->second;
  else
    return nullptr;
}

//==============================================================================
template <class T>
std::string NameManager<T>::getName(const T& obj) const
{
  DART_ASSERT(mReverseMap.size() == mMap.size());

  typename std::map<T, std::string>::const_iterator result
      = mReverseMap.find(obj);

  if (result != mReverseMap.end())
    return result->second;
  else
    return "";
}

//==============================================================================
template <class T>
std::string NameManager<T>::changeObjectName(
    const T& obj, std::string_view newName)
{
  DART_ASSERT(mReverseMap.size() == mMap.size());

  typename std::map<T, std::string>::iterator rit = mReverseMap.find(obj);
  if (rit == mReverseMap.end())
    return std::string(newName);

  if (rit->second == newName)
    return rit->second;

  removeName(rit->second);
  return issueNewNameAndAdd(newName, obj);
}

//==============================================================================
template <class T>
void NameManager<T>::setDefaultName(std::string_view defaultName)
{
  mDefaultName = std::string(defaultName);
}

//==============================================================================
template <class T>
const std::string& NameManager<T>::getDefaultName() const
{
  return mDefaultName;
}

//==============================================================================
template <class T>
void NameManager<T>::setManagerName(std::string_view managerName)
{
  mManagerName = std::string(managerName);
}

//==============================================================================
template <class T>
const std::string& NameManager<T>::getManagerName() const
{
  return mManagerName;
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_NAMEMANAGER_HPP_
