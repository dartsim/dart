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
#include <cassert>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>
#include <map>
#include <queue>

namespace dart {
namespace common {

namespace signal {
namespace detail {

template <typename T> struct DefaultCombiner;

}  // namespace detail
}  // namespace signal

class SignalBase;

/// class Connection
class Connection
{
public:
  /// Default constructor
  Connection();

  /// Copy constructor
  Connection(const Connection& _other);

  /// Constructor
  Connection(SignalBase* _signal, size_t _id);

  /// Destructor
  virtual ~Connection();

  /// Get id
  size_t getId() const;

  /// Get true if the slot is connected
  bool isConnected() const;

  /// Disconnect the connection
  void disconnect();

protected:
  /// True if disconnect() has not been called
  bool mIsConnected;

  /// Signal
  SignalBase* mSignal;

  /// Unique ID of the connection
  size_t mId;
};

/// class ScopedConnection
class ScopedConnection : public Connection
{
public:
  /// Default constructor
  ScopedConnection(const Connection& _other);

  /// Destructor
  virtual ~ScopedConnection();
};

/// class SignalBase
class SignalBase
{
public:
  friend class Connection;

  /// Default constructor
  SignalBase();

  /// Destructor
  virtual ~SignalBase();

  /// Disconnect given connection
  void disconnect(const Connection& _connection);

protected:
  /// Disconnect given connection ID
  virtual void disconnect(size_t _id) = 0;
};

template <typename _Signature,
          template<class> class Combiner = signal::detail::DefaultCombiner>
class Signal;

/// Signal implements a signal/slot mechanism
template <typename _Res, typename... _ArgTypes, template<class> class Combiner>
class Signal<_Res(_ArgTypes...), Combiner> : public SignalBase
{
public:
  using ResultType    = _Res;
  using SlotType      = std::function<ResultType(_ArgTypes...)>;
  using ConnectionMap = std::map<size_t, SlotType>;
  using SignalType    = Signal<_Res(_ArgTypes...), Combiner>;

  /// Constructor
  Signal() : SignalBase() {}

  /// Destructor
  virtual ~Signal() {}

  /// Connect a slot to the signal
  Connection connect(const SlotType& _slot)
  {
    const size_t index = issueUniqueId();

    mConnections[index] = _slot;

    return Connection(this, index);
  }

  /// Connect another signal
  /// \warning Be cautious to cyclic connections
  Connection connect(SignalType* _signal)
  {
    // Check self connection
    if (this == _signal)
      return Connection();

    using std::placeholders::_1;
    return connect(std::bind(std::mem_fn(&SignalType::raise), _signal, _1));
  }

  /// Connect another signal
  /// \warning Be cautious to cyclic connections
  Connection connect(const SignalType& _signal) { return connect(&_signal); }

  // Note: To implement void disconnect(const SlotType&) function,
  // std::function should provide == operator, which is not included in C++11
  // In the meantime, we use Connection to disconnect slots.

  /// Disconnect all the connections
  void disconnectAll()
  {
    flushDisconnections();

    for (const auto& connection : mConnections)
      disconnect(connection.first);
  }

  /// Flush all the disconnected connections
  void flushDisconnections()
  {
    if (mConnectionsToBeDisconnected.empty())
      return;

    for (const auto& id : mConnectionsToBeDisconnected)
    {
      const auto& connectionPair = mConnections.find(id);

      // If the connection of the ID exists in the list of connections
      if (mConnections.end() != connectionPair)
      {
        // Save the ID as free ID
        mFreeIds.push(connectionPair->first);
        mConnections.erase(connectionPair);
      }
    }

    mConnectionsToBeDisconnected.clear();
  }

  /// Get the number of connections
  size_t getNumConnections() const { return mConnections.size(); }

  /// Raise the signal
  ResultType raise(const _ArgTypes&... _args)
  {
    flushDisconnections();

    std::vector<ResultType> res(mConnections.size());

    size_t index = 0;
    for (const auto& connection : mConnections)
      res[index++] = (connection.second)(_args...);

    Combiner<ResultType> getResult;
    return getResult(res.begin(), res.end());
  }

  /// Raise the signal
  ResultType operator()(const _ArgTypes&... _args)
  {
    return raise(_args...);
  }

protected:
  // Documentation inherited
  virtual void disconnect(size_t _id) override
  {
    mConnectionsToBeDisconnected.push_back(_id);
  }

private:
  /// Issue unique ID for new connection
  size_t issueUniqueId()
  {
    // Check the capacity
    assert(mConnections.size() <= std::numeric_limits<size_t>::max());

    if (!mFreeIds.empty())
    {
      const size_t uniqueId = mFreeIds.front();
      mFreeIds.pop();
      return uniqueId;
    }

    size_t uniqueId = 0;

    if (!mConnections.empty())
    {
      const auto& it = mConnections.rbegin();
      uniqueId = it->first + 1;
    }

    return uniqueId;
  }

private:
  /// Connection map
  ConnectionMap mConnections;

  /// List of IDs whose connection will be disconnedted in the next raise
  std::vector<size_t> mConnectionsToBeDisconnected;

  /// Free IDs
  std::queue<size_t> mFreeIds;
};

/// Signal implements a signal/slot mechanism for the slots don't return a value
template <typename... _ArgTypes>
class Signal<void(_ArgTypes...)> : public SignalBase
{
public:
  using SlotType = std::function<void(_ArgTypes...)>;
  using ConnectionMap = std::map<size_t, SlotType>;
  using SignalType = Signal<void(_ArgTypes...)>;

  /// Constructor
  Signal() : SignalBase() {}

  /// Destructor
  virtual ~Signal() {}

  /// Connect a slot to the signal
  Connection connect(const SlotType& _slot)
  {
    const size_t index = issueUniqueId();

    mConnections[index] = _slot;

    return Connection(this, index);
  }

  /// Connect another signal
  /// \warning Be cautious to cyclic connections
  Connection connect(SignalType* _signal)
  {
    // Check self connection
    if (this == _signal)
      return Connection();

    using std::placeholders::_1;
    return connect(std::bind(std::mem_fn(&SignalType::raise), _signal, _1));
  }

  /// Connect another signal
  /// \warning Be cautious to cyclic connections
  Connection connect(const SignalType& _signal) { return connect(&_signal); }

  // Note: To implement void disconnect(const SlotType&) function,
  // std::function should provide == operator, which is not included in C++11
  // In the meantime, we use Connection to disconnect slots.

  /// Disconnect all the connections
  void disconnectAll()
  {
    flushDisconnections();

    for (const auto& connection : mConnections)
      disconnect(connection.first);
  }

  /// Flush all the disconnected connections
  void flushDisconnections()
  {
    if (mConnectionsToBeDisconnected.empty())
      return;

    for (const auto& id : mConnectionsToBeDisconnected)
    {
      const auto& connectionPair = mConnections.find(id);

      // If the connection of the ID exists in the list of connections
      if (mConnections.end() != connectionPair)
      {
        // Save the ID as free ID
        mFreeIds.push(connectionPair->first);
        mConnections.erase(connectionPair);
      }
    }

    mConnectionsToBeDisconnected.clear();
  }

  /// Get the number of connections
  size_t getNumConnections() const { return mConnections.size(); }

  /// Raise the signal
  void raise(const _ArgTypes&... _args)
  {
    flushDisconnections();

    for (const auto& connection : mConnections)
      (connection.second)(_args...);
  }

  /// Raise the signal
  void operator()(const _ArgTypes&... _args)
  {
    raise(_args...);
  }

protected:
  // Documentation inherited
  virtual void disconnect(size_t _id) override
  {
    mConnectionsToBeDisconnected.push_back(_id);
  }

private:
  /// Issue unique ID for new connection
  size_t issueUniqueId()
  {
    // Check the capacity
    assert(mConnections.size() <= std::numeric_limits<size_t>::max());

    if (!mFreeIds.empty())
    {
      const size_t uniqueId = mFreeIds.front();
      mFreeIds.pop();
      return uniqueId;
    }

    size_t uniqueId = 0;

    if (!mConnections.empty())
    {
      const auto& it = mConnections.rbegin();
      uniqueId = it->first + 1;
    }

    return uniqueId;
  }

private:
  /// Connection map
  ConnectionMap mConnections;

  /// List of IDs whose connection will be disconnedted in the next raise
  std::vector<size_t> mConnectionsToBeDisconnected;

  /// Free IDs
  std::queue<size_t> mFreeIds;
};

namespace signal {
namespace detail {

/// DefaultCombiner -- return the last result
template <typename T>
struct DefaultCombiner
{
  typedef T result_type;

  template <typename InputIterator>
  T operator()(InputIterator first, InputIterator last) const
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

#endif  // DART_COMMON_SIGNAL_H_
