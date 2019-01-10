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

#ifndef DART_COMMON_DETAIL_CONNECTIONBODY_HPP_
#define DART_COMMON_DETAIL_CONNECTIONBODY_HPP_

#include <memory>

namespace dart {
namespace common {

namespace signal {
namespace detail {

/// class ConnectionBodyBase
class ConnectionBodyBase
{
public:
  /// Constructor
  ConnectionBodyBase();

  /// Destructor
  virtual ~ConnectionBodyBase();

  /// Disconnect
  void disconnect();

  /// Get true if this connection body is connected to the signal
  bool isConnected() const;

protected:
  /// Connection flag
  bool mIsConnected;
};

/// class ConnectionBody
template <typename SlotType>
class ConnectionBody : public ConnectionBodyBase
{
public:
  /// Constructor given slot
  ConnectionBody(const SlotType& _slot);

  /// Move constructor given slot
  ConnectionBody(SlotType&& _slot);

  /// Destructor
  virtual ~ConnectionBody();

  /// Get slot
  const SlotType& getSlot();

private:
  /// Slot
  SlotType mSlot;
};

//==============================================================================
template <typename SlotType>
ConnectionBody<SlotType>::ConnectionBody(const SlotType& _slot)
  : ConnectionBodyBase(), mSlot(_slot)
{
  // Do nothing
}

//==============================================================================
template <typename SlotType>
ConnectionBody<SlotType>::ConnectionBody(SlotType&& _slot)
  : ConnectionBodyBase(), mSlot(std::forward<SlotType>(_slot))
{
  // Do nothing
}

//==============================================================================
template <typename SlotType>
ConnectionBody<SlotType>::~ConnectionBody()
{
  // Do nothing
}

//==============================================================================
template <typename SlotType>
const SlotType& ConnectionBody<SlotType>::getSlot()
{
  return mSlot;
}

/// DefaultCombiner -- return the last result
template <typename T>
struct DefaultCombiner
{
  typedef T result_type;

  template <typename InputIterator>
  static T process(InputIterator first, InputIterator last)
  {
    // If there are no slots to call, just return the
    // default-constructed value
    if (first == last)
      return T();

    return *(--last);
  }
};

}  // namespace detail
}  // namespace signal

}  // namespace common
}  // namespace dart

#endif  // DART_COMMON_DETAIL_CONNECTIONBODY_HPP_

