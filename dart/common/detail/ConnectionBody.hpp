/*
 * Copyright (c) 2011-2021, The DART development contributors
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

#ifndef DART_COMMON_DETAIL_CONNECTIONBODY_HPP_
#define DART_COMMON_DETAIL_CONNECTIONBODY_HPP_

#include <memory>

namespace dart {
namespace common {

namespace signal {
namespace detail {

/// class ConnectionBodyBase
class ConnectionBodyBase {
public:
  /// Constructor
  ConnectionBodyBase() = default;

  /// Destructor
  virtual ~ConnectionBodyBase();

  /// Disconnect
  virtual void disconnect() = 0;
};

/// class ConnectionBody
template <typename SignalType>
class ConnectionBody final
  : public ConnectionBodyBase,
    public std::enable_shared_from_this<ConnectionBody<SignalType>> {
public:
  using SlotType = typename SignalType::SlotType;

  /// Constructor given slot
  ConnectionBody(SignalType& signal, const SlotType& _slot);

  /// Move constructor given slot
  ConnectionBody(SignalType& signal, SlotType&& _slot);

  /// Destructor
  virtual ~ConnectionBody();

  /// Disconnect
  void disconnect() override;

  /// Get slot
  const SlotType& getSlot() const;

private:
  /// Signal of this connection
  SignalType& mSignal;
  // Holding the reference of the Signal is safe because the lifetime of this
  // ConnectionBody is always shorter than the Signal since the Signal has the
  // ownership of this ConnectionBody.

  /// Slot
  SlotType mSlot;
};

//==============================================================================
template <typename SignalType>
ConnectionBody<SignalType>::ConnectionBody(
    SignalType& signal, const SlotType& _slot)
  : ConnectionBodyBase(), mSignal(signal), mSlot(_slot) {
  // Do nothing
}

//==============================================================================
template <typename SignalType>
ConnectionBody<SignalType>::ConnectionBody(SignalType& signal, SlotType&& _slot)
  : ConnectionBodyBase(),
    mSignal(signal),
    mSlot(std::forward<SlotType>(_slot)) {
  // Do nothing
}

//==============================================================================
template <typename SignalType>
ConnectionBody<SignalType>::~ConnectionBody() {
  // Do nothing
}

//==============================================================================
template <typename SignalType>
void ConnectionBody<SignalType>::disconnect() {
  mSignal.disconnect(this->shared_from_this());
}

//==============================================================================
template <typename SignalType>
const typename ConnectionBody<SignalType>::SlotType&
ConnectionBody<SignalType>::getSlot() const {
  return mSlot;
}

/// DefaultCombiner -- return the last result
template <typename T>
struct DefaultCombiner {
  typedef T result_type;

  template <typename InputIterator>
  static T process(InputIterator first, InputIterator last) {
    // If there are no slots to call, just return the
    // default-constructed value
    if (first == last)
      return T();

    return *(--last);
  }
};

} // namespace detail
} // namespace signal

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_CONNECTIONBODY_HPP_
