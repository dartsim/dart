/*
 * Copyright (c) 2011-2019, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
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

#include <sstream>
#include <cassert>
#include "dart/common/Console.hpp"
#include "dart/common/NameManager.hpp"

namespace dart {
namespace common {

//==============================================================================
template <class T>
NameManager<T>::NameManager(const std::string& _managerName,
                            const std::string& _defaultName)
  : mManagerName(_managerName),
    mDefaultName(_defaultName),
    mNameBeforeNumber(true),
    mPrefix(""), mInfix("("), mAffix(")")
{
  // Do nothing
}

//==============================================================================
template <class T>
bool NameManager<T>::setPattern(const std::string& _newPattern)
{
  std::size_t name_start = _newPattern.find("%s");
  std::size_t number_start = _newPattern.find("%d");

  if(name_start == std::string::npos || number_start == std::string::npos)
    return false;

  if(name_start < number_start)
    mNameBeforeNumber = true;
  else
    mNameBeforeNumber = false;

  std::size_t prefix_end = std::min(name_start, number_start);
  std::size_t infix_end = std::max(name_start, number_start);

  mPrefix = _newPattern.substr(0, prefix_end);
  mInfix = _newPattern.substr(prefix_end+2, infix_end-prefix_end-2);
  mAffix = _newPattern.substr(infix_end+2);

  return true;
}

//==============================================================================
template <class T>
std::string NameManager<T>::issueNewName(const std::string& _name) const
{
  if(!hasName(_name))
    return _name;

  int count = 1;
  std::string newName;
  do
  {
    std::stringstream ss;
    if(mNameBeforeNumber)
      ss << mPrefix << _name << mInfix << count++ << mAffix;
    else
      ss << mPrefix << count++ << mInfix << _name << mAffix;
    newName = ss.str();
  } while (hasName(newName));

  dtmsg << "[NameManager::issueNewName] (" << mManagerName << ") The name ["
        << _name << "] is a duplicate, so it has been renamed to ["
        << newName << "]\n";

  return newName;
}

//==============================================================================
template <class T>
std::string NameManager<T>::issueNewNameAndAdd(const std::string& _name,
                                               const T& _obj)
{
  const std::string& checkEmpty = _name.empty()? mDefaultName : _name;
  const std::string& newName = issueNewName(checkEmpty);
  addName(newName, _obj);

  return newName;
}

//==============================================================================
template <class T>
bool NameManager<T>::addName(const std::string& _name, const T& _obj)
{
  if (_name.empty())
  {
    dtwarn << "[NameManager::addName] (" << mManagerName
           << ") Empty name is not allowed!\n";
    return false;
  }

  if (hasName(_name))
  {
    dtwarn << "[NameManager::addName] (" << mManagerName << ") The name ["
           << _name << "] already exists!\n";
    return false;
  }

  mMap.insert(std::pair<std::string, T>(_name, _obj));
  mReverseMap.insert(std::pair<T, std::string>(_obj, _name));

  assert(mReverseMap.size() == mMap.size());

  return true;
}

//==============================================================================
template <class T>
bool NameManager<T>::removeName(const std::string& _name)
{
  assert(mReverseMap.size() == mMap.size());

  typename std::map<std::string, T>::iterator it = mMap.find(_name);

  if (it == mMap.end())
    return false;

  typename std::map<T, std::string>::iterator rit =
      mReverseMap.find(it->second);

  if (rit != mReverseMap.end())
    mReverseMap.erase(rit);

  mMap.erase(it);

  return true;
}

//==============================================================================
template <class T>
bool NameManager<T>::removeObject(const T& _obj)
{
  assert(mReverseMap.size() == mMap.size());

  typename std::map<T, std::string>::iterator rit = mReverseMap.find(_obj);

  if (rit == mReverseMap.end())
    return false;

  typename std::map<std::string, T>::iterator it = mMap.find(rit->second);
  if (it != mMap.end())
    mMap.erase(it);

  mReverseMap.erase(rit);

  return true;
}

//==============================================================================
template <class T>
void NameManager<T>::removeEntries(const std::string& _name, const T& _obj)
{
  removeObject(_obj);
  removeName(_name);
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
bool NameManager<T>::hasName(const std::string& _name) const
{
  return (mMap.find(_name) != mMap.end());
}

//==============================================================================
template <class T>
bool NameManager<T>::hasObject(const T& _obj) const
{
  return (mReverseMap.find(_obj) != mReverseMap.end());
}

//==============================================================================
template <class T>
std::size_t NameManager<T>::getCount() const
{
  return mMap.size();
}

//==============================================================================
template <class T>
T NameManager<T>::getObject(const std::string& _name) const
{
  typename std::map<std::string, T>::const_iterator result =
      mMap.find(_name);

  if (result != mMap.end())
    return result->second;
  else
    return nullptr;
}

//==============================================================================
template <class T>
std::string NameManager<T>::getName(const T& _obj) const
{
  assert(mReverseMap.size() == mMap.size());

  typename std::map<T, std::string>::const_iterator result =
      mReverseMap.find(_obj);

  if (result != mReverseMap.end())
    return result->second;
  else
    return "";
}

//==============================================================================
template <class T>
std::string NameManager<T>::changeObjectName(const T& _obj,
                                             const std::string& _newName)
{
  assert(mReverseMap.size() == mMap.size());

  typename std::map<T, std::string>::iterator rit = mReverseMap.find(_obj);
  if(rit == mReverseMap.end())
    return _newName;

  if(rit->second == _newName)
    return rit->second;

  removeName(rit->second);
  return issueNewNameAndAdd(_newName, _obj);
}

//==============================================================================
template <class T>
void NameManager<T>::setDefaultName(const std::string& _defaultName)
{
  mDefaultName = _defaultName;
}

//==============================================================================
template <class T>
const std::string& NameManager<T>::getDefaultName() const
{
  return mDefaultName;
}

//==============================================================================
template <class T>
void NameManager<T>::setManagerName(const std::string& _managerName)
{
  mManagerName = _managerName;
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
