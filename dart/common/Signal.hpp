/*
 * Copyright (c) 2011-2025, The DART development contributors
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

#ifndef DART_COMMON_SIGNAL_HPP_
#define DART_COMMON_SIGNAL_HPP_

#include <dart/common/Deprecated.hpp>
#include <dart/common/detail/ConnectionBody.hpp>

#include <functional>
#include <memory>
#include <set>

namespace dart {
namespace common {

/// class Connection
class Connection
{
public:
  /// Default constructor
  Connection();

  /// Copy constructor
  Connection(const Connection& other);

  /// Move constructor
  Connection(Connection&& other);

  /// Assignment operator
  Connection& operator=(const Connection& other);

  /// Move assignment operator
  Connection& operator=(Connection&& other);

  /// Destructor
  virtual ~Connection();

  /// Get true if the slot is connected
  [[nodiscard]] bool isConnected() const;

  /// Disconnect the connection
  void disconnect() const;
  // TODO(JS): Make this non-const in the next major release

  template <typename Signature, template <class> class Combiner>
  friend class Signal;

protected:
  /// Constructor given connection body
  Connection(
      const std::weak_ptr<signal::detail::ConnectionBodyBase>& connectionBody);

  /// Move constructor given connection body
  Connection(
      std::weak_ptr<signal::detail::ConnectionBodyBase>&& connectionBody);

private:
  /// Weak pointer to connection body in the signal
  std::weak_ptr<signal::detail::ConnectionBodyBase> mWeakConnectionBody;
};

/// class ScopedConnection
class ScopedConnection : public Connection
{
public:
  /// Default constructor
  ScopedConnection(const Connection& other);

  /// Move constructor
  ScopedConnection(Connection&& other);

  /// Destructor
  virtual ~ScopedConnection();
};

template <
    typename _Signature,
    template <class> class Combiner = signal::detail::DefaultCombiner>
class Signal;

/// Signal implements a signal/slot mechanism
template <typename _Res, typename... _ArgTypes, template <class> class Combiner>
class Signal<_Res(_ArgTypes...), Combiner>
{
public:
  using ResultType = _Res;
  using SlotType = std::function<ResultType(_ArgTypes...)>;
  using SignalType = Signal<_Res(_ArgTypes...), Combiner>;

  using ConnectionBodyType = signal::detail::ConnectionBody<Signal>;
  using ConnectionSetType = std::set<
      std::shared_ptr<ConnectionBodyType>,
      std::owner_less<std::shared_ptr<ConnectionBodyType>>>;

  /// Constructor
  Signal();

  /// Destructor
  virtual ~Signal();

  /// Connect a slot to the signal
  Connection connect(const SlotType& slot);

  /// Connect a slot to the signal
  Connection connect(SlotType&& slot);

  /// Disconnect given connection
  void disconnect(const Connection& connection) const;

  /// Disconnect a connection
  void disconnect(const std::shared_ptr<ConnectionBodyType>& connectionBody);

  /// Disconnect all the connections
  void disconnectAll();

  /// Cleanup all the disconnected connections
  DART_DEPRECATED(6.10)
  void cleanupConnections();
  // This explicit connection cleaning is no longer necessary because now a
  // connection gets removed when it's disconnected.

  /// Get the number of connections
  [[nodiscard]] std::size_t getNumConnections() const;

  /// Raise the signal
  template <typename... ArgTypes>
  ResultType raise(ArgTypes&&... _args);

  /// Raise the signal
  template <typename... ArgTypes>
  ResultType operator()(ArgTypes&&... _args);

private:
  /// Connection set
  ConnectionSetType mConnectionBodies;
};

/// Signal implements a signal/slot mechanism for the slots don't return a value
template <typename... _ArgTypes>
class Signal<void(_ArgTypes...)>
{
public:
  using SlotType = std::function<void(_ArgTypes...)>;
  using SignalType = Signal<void(_ArgTypes...)>;

  using ConnectionBodyType = signal::detail::ConnectionBody<Signal>;
  using ConnectionSetType = std::set<
      std::shared_ptr<ConnectionBodyType>,
      std::owner_less<std::shared_ptr<ConnectionBodyType>>>;

  /// Constructor
  Signal();

  /// Destructor
  virtual ~Signal();

  /// Connect a slot to the signal
  Connection connect(const SlotType& slot);

  /// Connect a slot to the signal
  Connection connect(SlotType&& slot);

  /// Disconnect given connection
  void disconnect(const Connection& connection) const;

  /// Disconnect given connection
  void disconnect(const std::shared_ptr<ConnectionBodyType>& connectionBody);

  /// Disconnect all the connections
  void disconnectAll();

  /// Cleanup all the disconnected connections
  DART_DEPRECATED(6.10)
  void cleanupConnections();
  // This explicit connection cleaning is no longer necessary because now a
  // connection gets removed when it's disconnected.

  /// Get the number of connections
  [[nodiscard]] std::size_t getNumConnections() const;

  /// Raise the signal
  template <typename... Args>
  void raise(Args&&... args);

  /// Raise the signal
  template <typename... Args>
  void operator()(Args&&... args);

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
  using SlotType = typename T::SlotType;
  using SignalType = typename T::SignalType;

  /// Constructor given signal
  SlotRegister(SignalType& signal);

  /// Connect a slot to the signal
  Connection connect(const SlotType& slot);

private:
  /// Signal
  SignalType& mSignal;
};

} // namespace common
} // namespace dart

#include <dart/common/detail/Signal.hpp>

#endif // DART_COMMON_SIGNAL_HPP_
