/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Michael X. Grey <mxgrey@gatech.edu>
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

#ifndef DART_COMMON_DETAIL_SUB_PTR_HPP_
#define DART_COMMON_DETAIL_SUB_PTR_HPP_

//==============================================================================
template <class T>
sub_ptr<T>::sub_ptr()
  : mSubject(nullptr)
{
  // Do nothing
}

//==============================================================================
template <class T>
sub_ptr<T>::sub_ptr(T* _ptr) : mSubject(nullptr)
{
  set(_ptr);
}

//==============================================================================
template <class T>
sub_ptr<T>& sub_ptr<T>::operator =(const sub_ptr<T>& _sp)
{
  set(_sp.get());
  return *this;
}

//==============================================================================
template <class T>
sub_ptr<T>& sub_ptr<T>::operator =(T* _ptr)
{
  set(_ptr);
  return *this;
}

//==============================================================================
template <class T>
sub_ptr<T>::operator T*() const
{
  return mSubject;
}

//==============================================================================
template <class T>
T& sub_ptr<T>::operator*() const
{
  return *mSubject;
}

//==============================================================================
template <class T>
T* sub_ptr<T>::operator->() const
{
  return mSubject;
}

//==============================================================================
template <class T>
T* sub_ptr<T>::get() const
{
  return mSubject;
}

//==============================================================================
template <class T>
void sub_ptr<T>::set(T* _ptr)
{
  if(mSubject == _ptr)
    return;

  removeSubject(mSubject);
  mSubject = _ptr;
  addSubject(mSubject);
}

//==============================================================================
template <class T>
bool sub_ptr<T>::valid()
{
  return mSubject != nullptr;
}

//==============================================================================
template <class T>
void sub_ptr<T>::handleDestructionNotification(const Subject* _subject)
{
  if(_subject == mSubject)
    mSubject = nullptr;
}

#endif // DART_COMMON_DETAIL_SUB_PTR_HPP_
