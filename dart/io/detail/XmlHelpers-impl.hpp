/*
 * Copyright (c) 2011-2023, The DART development contributors
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

#ifndef DART_UTILS_DETAIL_XMLHELPERS_IMPL_HPP_
#define DART_UTILS_DETAIL_XMLHELPERS_IMPL_HPP_

#include "dart/common/String.hpp"
#include "dart/io/XmlHelpers.hpp"

namespace dart::io {

//==============================================================================
template <typename S, int N>
std::string toString(const Eigen::Matrix<S, N, 1>& v)
{
  std::stringstream ss;
  ss << v.transpose();
  return ss.str();
}

//==============================================================================
template <typename S>
std::string toString(
    const Eigen::Transform<S, 3, Eigen::Isometry>& v,
    const std::string& rotationType)
{
  Eigen::Matrix<S, 3, 1> angles;
  if (rotationType == "intrinsic")
  {
    angles = math::matrixToEulerXYZ(v.rotation());
  }
  else if (rotationType == "extrinsic")
  {
    angles = math::matrixToEulerZYX(v.rotation()).reverse();
  }
  else
  {
    DART_ERROR(
        "Unsupported rotation type [{}]. Assuming intrinsic.", rotationType);
    angles = math::matrixToEulerXYZ(v.rotation());
  }

  std::stringstream ss;
  ss.precision(6);
  ss << v.translation().transpose() << " ";
  ss << angles;

  return ss.str();
}

//==============================================================================
template <std::size_t N>
Eigen::Matrix<double, N, 1> toVectorNd(const std::string& str)
{
  const std::vector<std::string> pieces = common::split(common::trim(str));
  const std::size_t sizeToRead = std::min(N, pieces.size());
  if (pieces.size() < N)
  {
    dterr << "Failed to read a vector because the dimension '" << pieces.size()
          << "' is less than the expectation '" << N << "'.\n";
  }
  else if (pieces.size() > N)
  {
    dterr << "Failed to read a vector because the dimension '" << pieces.size()
          << "' is greater than the expectation '" << N << "'.\n";
  }

  Eigen::Matrix<double, N, 1> ret = Eigen::Matrix<double, N, 1>::Zero();

  for (std::size_t i = 0; i < sizeToRead; ++i)
  {
    if (pieces[i] != "")
    {
      try
      {
        ret[i] = toDouble(pieces[i]);
      }
      catch (std::exception& e)
      {
        dterr << "value [" << pieces[i]
              << "] is not a valid double for Eigen::Vector" << N << "d[" << i
              << "]: " << e.what() << "\n";
      }
    }
  }

  return ret;
}

//==============================================================================
template <std::size_t N>
Eigen::Matrix<double, N, 1> getAttributeVectorNd(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);
  return toVectorNd<N>(val);
}

//==============================================================================
template <typename ElementType>
TemplatedElementEnumerator<ElementType>::TemplatedElementEnumerator(
    ElementPtr parentElement, const std::string& childElementName)
  : mParentElement(parentElement),
    mChildElementName(childElementName),
    mCurrentElement(nullptr)
{
  // Do nothing
}

//==============================================================================
template <typename ElementType>
TemplatedElementEnumerator<ElementType>::~TemplatedElementEnumerator()
{
  // Do nothing
}

//==============================================================================
template <typename ElementType>
bool TemplatedElementEnumerator<ElementType>::next()
{
  if (!mParentElement)
    return false;

  if (mCurrentElement)
  {
    mCurrentElement
        = mCurrentElement->NextSiblingElement(mChildElementName.c_str());
  }
  else
  {
    mCurrentElement
        = mParentElement->FirstChildElement(mChildElementName.c_str());
  }

  if (!valid())
    mParentElement = nullptr;

  return valid();
}

//==============================================================================
template <typename ElementType>
typename TemplatedElementEnumerator<ElementType>::ElementPtr
TemplatedElementEnumerator<ElementType>::get() const
{
  return mCurrentElement;
}

//==============================================================================
template <typename ElementType>
typename TemplatedElementEnumerator<ElementType>::ElementPtr
TemplatedElementEnumerator<ElementType>::operator->() const
{
  return mCurrentElement;
}

//==============================================================================
template <typename ElementType>
typename TemplatedElementEnumerator<ElementType>::ElementRef
TemplatedElementEnumerator<ElementType>::operator*() const
{
  return *mCurrentElement;
}

//==============================================================================
template <typename ElementType>
bool TemplatedElementEnumerator<ElementType>::operator==(
    const TemplatedElementEnumerator<ElementType>& rhs) const
{
  // If they point at the same node, then the names must match
  return (this->mParentElement == rhs.mParentElement)
         && (this->mCurrentElement == rhs.mCurrentElement)
         && (this->mCurrentElement != nullptr
             || (this->mChildElementName == rhs.mChildElementName));
}

//==============================================================================
template <typename ElementType>
TemplatedElementEnumerator<ElementType>&
TemplatedElementEnumerator<ElementType>::operator=(
    const TemplatedElementEnumerator<ElementType>& rhs)
{
  this->mParentElement = rhs.mParentElement;
  this->mChildElementName = rhs.mChildElementName;
  this->mCurrentElement = rhs.mCurrentElement;

  return *this;
}

//==============================================================================
template <typename ElementType>
bool TemplatedElementEnumerator<ElementType>::valid() const
{
  return mCurrentElement != nullptr;
}

} // namespace dart::io

#endif // #ifndef DART_UTILS_DETAIL_XMLHELPERS_IMPL_HPP_
