/*
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#ifndef DART_COMMON_SIGNAL_H_
#define DART_COMMON_SIGNAL_H_

#include <algorithm>
#include <functional>
#include <memory>
#include <set>
#include <utility>
#include <vector>

namespace dart {
namespace common {

namespace signal {
namespace detail {

/// class ConnectionBodyBase
class ConnectionBodyBase
{
public:
  /// Constructor
  ConnectionBodyBase() : mIsConnected(true) {}

  /// Destructor
  virtual ~ConnectionBodyBase() {}

  /// Disconnect
  void disconnect() { mIsConnected = false; }

  /// Get true if this connection body is connected to the signal
  bool isConnected() const { return mIsConnected; }

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
  ConnectionBody(const SlotType& _slot) : ConnectionBodyBase(), mSlot(_slot) {}

  /// Move constructor given slot
  ConnectionBody(SlotType&& _slot)
    : ConnectionBodyBase(), mSlot(std::forward<SlotType>(_slot)) {}

  /// Destructor
  virtual ~ConnectionBody() {}

  /// Get slot
  const SlotType& getSlot() { return mSlot; }

private:
  /// Slot
  SlotType mSlot;
};

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

/// class Connection
class Connection
{
public:
  /// Default constructor
  Connection() {}

  /// Copy constructor
  Connection(const Connection& _other)
    : mWeakConnectionBody(_other.mWeakConnectionBody) {}

  /// Move constructor
  Connection(Connection&& _other)
    : mWeakConnectionBody(std::move(_other.mWeakConnectionBody)) {}

  /// Assignment operator
  Connection& operator=(const Connection& _other)
  {
    mWeakConnectionBody = _other.mWeakConnectionBody;
    return *this;
  }

  /// Move assignment operator
  Connection& operator=(Connection&& _other)
  {
    mWeakConnectionBody = std::move(_other.mWeakConnectionBody);
    return *this;
  }

  /// Constructor given connection body
  Connection(
      const std::weak_ptr<signal::detail::ConnectionBodyBase>& _connectionBody)
    : mWeakConnectionBody(_connectionBody) {}

  /// Move constructor given connection body
  Connection(
      std::weak_ptr<signal::detail::ConnectionBodyBase>&& _connectionBody)
    : mWeakConnectionBody(std::move(_connectionBody)) {}

  /// Destructor
  virtual ~Connection() {}

  /// Get true if the slot is connected
  bool isConnected() const
  {
    std::shared_ptr<signal::detail::ConnectionBodyBase>
        connectionBody(mWeakConnectionBody.lock());

    if (nullptr == connectionBody)
      return false;

    return connectionBody->isConnected();
  }

  /// Disconnect the connection
  void disconnect() const
  {
    std::shared_ptr<signal::detail::ConnectionBodyBase>
        connectionBody(mWeakConnectionBody.lock());

    if (nullptr == connectionBody)
      return;

    connectionBody->disconnect();
  }

private:
  /// Weak pointer to connection body in the signal
  std::weak_ptr<signal::detail::ConnectionBodyBase> mWeakConnectionBody;
};

/// class ScopedConnection
class ScopedConnection : public Connection
{
public:
  /// Default constructor
  ScopedConnection(const Connection& _other) : Connection(_other) {}

  /// Move constructor
  ScopedConnection(Connection&& _other) : Connection(std::move(_other)) {}

  /// Destructor
  virtual ~ScopedConnection() { disconnect(); }
};

template <typename _Signature,
          template<class> class Combiner = signal::detail::DefaultCombiner>
class Signal;

/// Signal implements a signal/slot mechanism
template <typename _Res, typename... _ArgTypes, template<class> class Combiner>
class Signal<_Res(_ArgTypes...), Combiner>
{
public:
  using ResultType    = _Res;
  using SlotType      = std::function<ResultType(_ArgTypes...)>;
  using SignalType    = Signal<_Res(_ArgTypes...), Combiner>;

  using ConnectionBodyType = signal::detail::ConnectionBody<SlotType>;
  using ConnectionSetType
    = std::set<std::shared_ptr<ConnectionBodyType>,
               std::owner_less<std::shared_ptr<ConnectionBodyType>>>;

  /// Constructor
  Signal() {}

  /// Destructor
  virtual ~Signal() { disconnectAll(); }

  /// Connect a slot to the signal
  Connection connect(const SlotType& _slot)
  {
    auto newConnectionBody = std::make_shared<ConnectionBodyType>(_slot);
    mConnectionBodies.insert(newConnectionBody);

    return Connection(std::move(newConnectionBody));
  }

  /// Connect a slot to the signal
  Connection connect(SlotType&& _slot)
  {
    auto newConnectionBody
        = std::make_shared<ConnectionBodyType>(std::forward<SlotType>(_slot));
    mConnectionBodies.insert(newConnectionBody);

    return Connection(std::move(newConnectionBody));
  }

  /// Disconnect given connection
  void disconnect(const Connection& _connection) const
  {
    _connection.disconnect();
  }

  /// Disconnect all the connections
  void disconnectAll() { mConnectionBodies.clear(); }

  /// Cleanup all the disconnected connections
  void clenaupConnections()
  {
    // Counts all the connected conection bodies
    for (const auto& connectionBody : mConnectionBodies)
    {
      if (!connectionBody->isConnected())
        mConnectionBodies.erase(connectionBody);
    }
  }

  /// Get the number of connections
  size_t getNumConnections() const
  {
    size_t numConnections = 0;

    // Counts all the connected conection bodies
    for (const auto& connectionBody : mConnectionBodies)
    {
      if (connectionBody->isConnected())
        ++numConnections;
    }

    return numConnections;
  }

  /// Raise the signal
  template <typename... ArgTypes>
  ResultType raise(ArgTypes&&... _args)
  {
    std::vector<ResultType> res(mConnectionBodies.size());
    auto resIt = res.begin();

    for (auto itr = mConnectionBodies.begin(); itr != mConnectionBodies.end(); )
    {
      if ((*itr)->isConnected())
      {
        *(resIt++) = (*itr)->getSlot()(std::forward<ArgTypes>(_args)...);
        ++itr;
      }
      else
      {
        mConnectionBodies.erase(itr++);
      }
    }

    return Combiner<ResultType>::process(res.begin(), resIt);
  }

  /// Raise the signal
  template <typename... ArgTypes>
  ResultType operator()(ArgTypes&&... _args)
  {
    return raise(std::forward<ArgTypes>(_args)...);
  }

private:
  /// Connection set
  ConnectionSetType mConnectionBodies;
};

/// Signal implements a signal/slot mechanism for the slots don't return a value
template <typename... _ArgTypes>
class Signal<void(_ArgTypes...)>
{
public:
  using SlotType   = std::function<void(_ArgTypes...)>;
  using SignalType = Signal<void(_ArgTypes...)>;

  using ConnectionBodyType = signal::detail::ConnectionBody<SlotType>;
  using ConnectionSetType
    = std::set<std::shared_ptr<ConnectionBodyType>,
               std::owner_less<std::shared_ptr<ConnectionBodyType>>>;

  /// Constructor
  Signal() {}

  /// Destructor
  virtual ~Signal() { disconnectAll(); }

  /// Connect a slot to the signal
  Connection connect(const SlotType& _slot)
  {
    auto newConnectionBody = std::make_shared<ConnectionBodyType>(_slot);
    mConnectionBodies.insert(newConnectionBody);

    return Connection(std::move(newConnectionBody));
  }

  /// Connect a slot to the signal
  Connection connect(SlotType&& _slot)
  {
    auto newConnectionBody
        = std::make_shared<ConnectionBodyType>(std::forward<SlotType>(_slot));
    mConnectionBodies.insert(newConnectionBody);

    return Connection(std::move(newConnectionBody));
  }

  /// Disconnect given connection
  void disconnect(const Connection& _connection) const
  {
    _connection.disconnect();
  }

  /// Disconnect all the connections
  void disconnectAll() { mConnectionBodies.clear(); }

  /// Cleanup all the disconnected connections
  void clenaupConnections()
  {
    // Counts all the connected conection bodies
    for (const auto& connectionBody : mConnectionBodies)
    {
      if (!connectionBody->isConnected())
        mConnectionBodies.erase(connectionBody);
    }
  }

  /// Get the number of connections
  size_t getNumConnections() const
  {
    size_t numConnections = 0;

    // Counts all the connected conection bodies
    for (const auto& connectionBody : mConnectionBodies)
    {
      if (connectionBody->isConnected())
        ++numConnections;
    }

    return numConnections;
  }

  /// Raise the signal
  template <typename... ArgTypes>
  void raise(ArgTypes&&... _args)
  {
    for (auto itr = mConnectionBodies.begin(); itr != mConnectionBodies.end(); )
    {
      if ((*itr)->isConnected())
      {
        (*itr)->getSlot()(std::forward<ArgTypes>(_args)...);
        ++itr;
      }
      else
      {
        mConnectionBodies.erase(itr++);
      }
    }
  }

  /// Raise the signal
  template <typename... ArgTypes>
  void operator()(ArgTypes&&... _args)
  {
    raise(std::forward<ArgTypes>(_args)...);
  }

private:
  /// Connection set
  ConnectionSetType mConnectionBodies;
};

/// SlotRegister can be used as a public member for connecting slots to a
/// private Signal member. In this way you won't have to write forwarding
/// connect/disconnect boilerplate for your classes.
template <typename T>
class SlotRegister
{
public:
  using SlotType   = typename T::SlotType;
  using SignalType = typename T::SignalType;

  /// Constructor given signal
  SlotRegister(typename T::SignalType& _signal) : mSignal(_signal) {}

  /// Connect a slot to the signal
  Connection connect(const SlotType& _slot) { return mSignal.connect(_slot); }

private:
  /// Signal
  typename T::SignalType& mSignal;
};

}  // namespace common
}  // namespace dart

#endif  // DART_COMMON_SIGNAL_H_

