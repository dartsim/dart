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

#ifndef DART_COMMON_DETAIL_SIGNAL_HPP_
#define DART_COMMON_DETAIL_SIGNAL_HPP_

#include <vector>

namespace dart {
namespace common {

//==============================================================================
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
Signal<_Res(_ArgTypes...), Combiner>::Signal()
{
  // Do nothing
}

//==============================================================================
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
Signal<_Res(_ArgTypes...), Combiner>::~Signal()
{
  disconnectAll();
}

//==============================================================================
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
Connection Signal<_Res(_ArgTypes...), Combiner>::connect(const SlotType& slot)
{
  auto newConnectionBody = std::make_shared<ConnectionBodyType>(*this, slot);
  mConnectionBodies.insert(newConnectionBody);

  return Connection(std::move(newConnectionBody));
}

//==============================================================================
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
Connection Signal<_Res(_ArgTypes...), Combiner>::connect(SlotType&& slot)
{
  auto newConnectionBody = std::make_shared<ConnectionBodyType>(
      *this, std::forward<SlotType>(slot));
  mConnectionBodies.insert(newConnectionBody);

  return Connection(std::move(newConnectionBody));
}

//==============================================================================
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
void Signal<_Res(_ArgTypes...), Combiner>::disconnect(
    const Connection& connection) const
{
  connection.disconnect();
}

//==============================================================================
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
void Signal<_Res(_ArgTypes...), Combiner>::disconnect(
    const std::shared_ptr<Signal::ConnectionBodyType>& connectionBody)
{
  mConnectionBodies.erase(connectionBody);
}

//==============================================================================
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
void Signal<_Res(_ArgTypes...), Combiner>::disconnectAll()
{
  mConnectionBodies.clear();
}

//==============================================================================
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
std::size_t Signal<_Res(_ArgTypes...), Combiner>::getNumConnections() const
{
  return mConnectionBodies.size();
}

//==============================================================================
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
template <typename... ArgTypes>
_Res Signal<_Res(_ArgTypes...), Combiner>::raise(ArgTypes&&... args)
{
  std::vector<ResultType> res(mConnectionBodies.size());
  auto resIt = res.begin();

  for (const auto& connectionBody : mConnectionBodies)
  {
    *(resIt++) = connectionBody->getSlot()(std::forward<ArgTypes>(args)...);
  }

  return Combiner<ResultType>::process(res.begin(), resIt);
}

//==============================================================================
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
template <typename... Args>
_Res Signal<_Res(_ArgTypes...), Combiner>::operator()(Args&&... args)
{
  return raise(std::forward<Args>(args)...);
}

//==============================================================================
template <typename... _ArgTypes>
Signal<void(_ArgTypes...)>::Signal()
{
  // Do nothing
}

//==============================================================================
template <typename... _ArgTypes>
Signal<void(_ArgTypes...)>::~Signal()
{
  disconnectAll();
}

//==============================================================================
template <typename... _ArgTypes>
Connection Signal<void(_ArgTypes...)>::connect(const SlotType& slot)
{
  auto newConnectionBody = std::make_shared<ConnectionBodyType>(*this, slot);
  mConnectionBodies.insert(newConnectionBody);

  return Connection(std::move(newConnectionBody));
}

//==============================================================================
template <typename... _ArgTypes>
Connection Signal<void(_ArgTypes...)>::connect(SlotType&& slot)
{
  auto newConnectionBody = std::make_shared<ConnectionBodyType>(
      *this, std::forward<SlotType>(slot));
  mConnectionBodies.insert(newConnectionBody);

  return Connection(std::move(newConnectionBody));
}

//==============================================================================
template <typename... _ArgTypes>
void Signal<void(_ArgTypes...)>::disconnect(const Connection& connection) const
{
  connection.disconnect();
}

//==============================================================================
template <typename... _ArgTypes>
void Signal<void(_ArgTypes...)>::disconnect(
    const std::shared_ptr<Signal::ConnectionBodyType>& connectionBody)
{
  mConnectionBodies.erase(connectionBody);
}

//==============================================================================
template <typename... _ArgTypes>
void Signal<void(_ArgTypes...)>::disconnectAll()
{
  mConnectionBodies.clear();
}

//==============================================================================
template <typename... _ArgTypes>
std::size_t Signal<void(_ArgTypes...)>::getNumConnections() const
{
  return mConnectionBodies.size();
}

//==============================================================================
template <typename... _ArgTypes>
template <typename... Args>
void Signal<void(_ArgTypes...)>::raise(Args&&... args)
{
  for (const auto& connectionBody : mConnectionBodies)
  {
    connectionBody->getSlot()(std::forward<Args>(args)...);
  }
}

//==============================================================================
template <typename... _ArgTypes>
template <typename... Args>
void Signal<void(_ArgTypes...)>::operator()(Args&&... args)
{
  raise(std::forward<Args>(args)...);
}

//==============================================================================
template <typename T>
SlotRegister<T>::SlotRegister(SignalType& signal) : mSignal(signal)
{
  // Do nothing
}

//==============================================================================
template <typename T>
Connection SlotRegister<T>::connect(const SlotType& slot)
{
  return mSignal.connect(slot);
}

} // namespace common
} // namespace dart

#endif // DART_COMMON_DETAIL_SIGNAL_HPP_
