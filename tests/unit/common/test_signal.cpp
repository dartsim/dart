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

#include <dart/common/signal.hpp>

#include <gtest/gtest.h>

#include <string>
#include <vector>

using namespace dart;
using namespace dart::common;

TEST(Signal, DefaultConstruction)
{
  Signal<void()> signal;
  EXPECT_EQ(signal.getNumConnections(), 0u);
}

TEST(Signal, ConnectSlot)
{
  Signal<void()> signal;
  bool called = false;

  Connection conn = signal.connect([&called]() { called = true; });

  EXPECT_TRUE(conn.isConnected());
  EXPECT_EQ(signal.getNumConnections(), 1u);
}

TEST(Signal, RaiseSignal)
{
  Signal<void()> signal;
  int callCount = 0;

  signal.connect([&callCount]() { ++callCount; });
  signal.raise();

  EXPECT_EQ(callCount, 1);
}

TEST(Signal, RaiseWithOperator)
{
  Signal<void()> signal;
  int callCount = 0;

  signal.connect([&callCount]() { ++callCount; });
  signal();

  EXPECT_EQ(callCount, 1);
}

TEST(Signal, RaiseWithArguments)
{
  Signal<void(int, const std::string&)> signal;
  int receivedInt = 0;
  std::string receivedStr;

  signal.connect([&](int i, const std::string& s) {
    receivedInt = i;
    receivedStr = s;
  });
  signal.raise(42, "hello");

  EXPECT_EQ(receivedInt, 42);
  EXPECT_EQ(receivedStr, "hello");
}

TEST(Signal, MultipleSlots)
{
  Signal<void()> signal;
  std::vector<int> callOrder;

  signal.connect([&callOrder]() { callOrder.push_back(1); });
  signal.connect([&callOrder]() { callOrder.push_back(2); });
  signal.connect([&callOrder]() { callOrder.push_back(3); });

  EXPECT_EQ(signal.getNumConnections(), 3u);

  signal.raise();

  EXPECT_EQ(callOrder.size(), 3u);
}

TEST(Signal, DisconnectViaConnection)
{
  Signal<void()> signal;
  int callCount = 0;

  Connection conn = signal.connect([&callCount]() { ++callCount; });
  EXPECT_EQ(signal.getNumConnections(), 1u);

  conn.disconnect();

  EXPECT_FALSE(conn.isConnected());
  EXPECT_EQ(signal.getNumConnections(), 0u);

  signal.raise();
  EXPECT_EQ(callCount, 0);
}

TEST(Signal, DisconnectViaSignal)
{
  Signal<void()> signal;
  int callCount = 0;

  Connection conn = signal.connect([&callCount]() { ++callCount; });

  signal.disconnect(conn);

  EXPECT_EQ(signal.getNumConnections(), 0u);
}

TEST(Signal, DisconnectAll)
{
  Signal<void()> signal;
  int callCount = 0;

  signal.connect([&callCount]() { ++callCount; });
  signal.connect([&callCount]() { ++callCount; });
  signal.connect([&callCount]() { ++callCount; });

  EXPECT_EQ(signal.getNumConnections(), 3u);

  signal.disconnectAll();

  EXPECT_EQ(signal.getNumConnections(), 0u);

  signal.raise();
  EXPECT_EQ(callCount, 0);
}

TEST(Signal, ConnectionDefaultConstruction)
{
  Connection conn;
  EXPECT_FALSE(conn.isConnected());
}

TEST(Signal, ConnectionCopyConstruction)
{
  Signal<void()> signal;
  Connection conn1 = signal.connect([]() {});

  Connection conn2(conn1);

  EXPECT_TRUE(conn2.isConnected());
  EXPECT_EQ(signal.getNumConnections(), 1u);
}

TEST(Signal, ConnectionMoveConstruction)
{
  Signal<void()> signal;
  Connection conn1 = signal.connect([]() {});

  Connection conn2(std::move(conn1));

  EXPECT_TRUE(conn2.isConnected());
  EXPECT_EQ(signal.getNumConnections(), 1u);
}

TEST(Signal, ConnectionAssignment)
{
  Signal<void()> signal;
  Connection conn1 = signal.connect([]() {});
  Connection conn2;

  conn2 = conn1;

  EXPECT_TRUE(conn2.isConnected());
}

TEST(Signal, ConnectionMoveAssignment)
{
  Signal<void()> signal;
  Connection conn1 = signal.connect([]() {});
  Connection conn2;

  conn2 = std::move(conn1);

  EXPECT_TRUE(conn2.isConnected());
}

TEST(Signal, ScopedConnectionDisconnectsOnDestruction)
{
  Signal<void()> signal;
  int callCount = 0;

  {
    ScopedConnection scoped = signal.connect([&callCount]() { ++callCount; });
    EXPECT_EQ(signal.getNumConnections(), 1u);
    signal.raise();
    EXPECT_EQ(callCount, 1);
  }

  EXPECT_EQ(signal.getNumConnections(), 0u);

  signal.raise();
  EXPECT_EQ(callCount, 1);
}

TEST(Signal, SlotRegister)
{
  Signal<void(int)> signal;
  SlotRegister<Signal<void(int)>> reg(signal);

  int receivedValue = 0;
  Connection conn
      = reg.connect([&receivedValue](int val) { receivedValue = val; });

  EXPECT_TRUE(conn.isConnected());
  EXPECT_EQ(signal.getNumConnections(), 1u);

  signal.raise(100);
  EXPECT_EQ(receivedValue, 100);
}

TEST(Signal, SignalDestruction)
{
  Connection conn;
  {
    Signal<void()> signal;
    conn = signal.connect([]() {});
    EXPECT_TRUE(conn.isConnected());
  }
  EXPECT_FALSE(conn.isConnected());
}

TEST(Signal, ConnectMovedSlot)
{
  Signal<void()> signal;
  int callCount = 0;

  auto slot = [&callCount]() {
    ++callCount;
  };
  signal.connect(std::move(slot));

  signal.raise();
  EXPECT_EQ(callCount, 1);
}

TEST(Signal, ScopedConnectionMoveConstruction)
{
  Signal<void()> signal;
  int callCount = 0;

  {
    Connection conn = signal.connect([&callCount]() { ++callCount; });
    ScopedConnection scoped(std::move(conn));
    EXPECT_TRUE(scoped.isConnected());
    EXPECT_EQ(signal.getNumConnections(), 1u);
    signal.raise();
    EXPECT_EQ(callCount, 1);
  }

  // ScopedConnection should have disconnected on destruction
  EXPECT_EQ(signal.getNumConnections(), 0u);
  signal.raise();
  EXPECT_EQ(callCount, 1);
}

TEST(Signal, DisconnectDuringEmission)
{
  Signal<void()> signal;
  int callCount1 = 0;
  int callCount2 = 0;
  Connection conn2;

  // First slot disconnects the second slot during emission
  signal.connect([&]() {
    ++callCount1;
    conn2.disconnect();
  });

  conn2 = signal.connect([&callCount2]() { ++callCount2; });

  EXPECT_EQ(signal.getNumConnections(), 2u);

  // Raise signal - first slot disconnects second, but second should still
  // be called because raise() copies the connection list before iterating
  signal.raise();

  EXPECT_EQ(callCount1, 1);
  // The second slot is still called because the signal copies connections
  // before iterating
  EXPECT_EQ(callCount2, 1);

  // After emission, second connection should be disconnected
  EXPECT_EQ(signal.getNumConnections(), 1u);
  EXPECT_FALSE(conn2.isConnected());
}

TEST(Signal, SelfDisconnectDuringEmission)
{
  Signal<void()> signal;
  int callCount = 0;
  Connection conn;

  conn = signal.connect([&]() {
    ++callCount;
    conn.disconnect();
  });

  EXPECT_EQ(signal.getNumConnections(), 1u);

  signal.raise();

  EXPECT_EQ(callCount, 1);
  EXPECT_EQ(signal.getNumConnections(), 0u);
  EXPECT_FALSE(conn.isConnected());

  // Raising again should not call the slot
  signal.raise();
  EXPECT_EQ(callCount, 1);
}

TEST(Signal, SignalWithReturnValue)
{
  Signal<int()> signal;

  signal.connect([]() { return 1; });
  signal.connect([]() { return 2; });
  signal.connect([]() { return 3; });

  // DefaultCombiner returns the last result
  int result = signal.raise();
  EXPECT_EQ(result, 3);
}

TEST(Signal, SignalWithReturnValueOperator)
{
  Signal<int(int)> signal;

  signal.connect([](int x) { return x * 2; });
  signal.connect([](int x) { return x * 3; });

  // DefaultCombiner returns the last result
  int result = signal(5);
  EXPECT_EQ(result, 15);
}

TEST(Signal, SignalWithReturnValueNoConnections)
{
  Signal<int()> signal;

  // DefaultCombiner returns default-constructed value when no slots
  int result = signal.raise();
  EXPECT_EQ(result, 0);
}

TEST(Signal, MultipleConnectionsSameSlot)
{
  Signal<void()> signal;
  int callCount = 0;

  auto slot = [&callCount]() {
    ++callCount;
  };

  Connection conn1 = signal.connect(slot);
  Connection conn2 = signal.connect(slot);
  Connection conn3 = signal.connect(slot);

  EXPECT_EQ(signal.getNumConnections(), 3u);

  signal.raise();
  EXPECT_EQ(callCount, 3);

  // Disconnect one connection
  conn2.disconnect();
  EXPECT_EQ(signal.getNumConnections(), 2u);

  signal.raise();
  EXPECT_EQ(callCount, 5);
}

TEST(Signal, ConnectionDisconnectIdempotent)
{
  Signal<void()> signal;
  int callCount = 0;

  Connection conn = signal.connect([&callCount]() { ++callCount; });
  EXPECT_EQ(signal.getNumConnections(), 1u);

  // Disconnect multiple times should be safe
  conn.disconnect();
  EXPECT_EQ(signal.getNumConnections(), 0u);
  EXPECT_FALSE(conn.isConnected());

  conn.disconnect();
  EXPECT_EQ(signal.getNumConnections(), 0u);
  EXPECT_FALSE(conn.isConnected());

  conn.disconnect();
  EXPECT_EQ(signal.getNumConnections(), 0u);
}

TEST(Signal, DefaultConnectionDisconnect)
{
  Connection conn;
  EXPECT_FALSE(conn.isConnected());

  // Disconnecting a default-constructed connection should be safe
  conn.disconnect();
  EXPECT_FALSE(conn.isConnected());
}

TEST(Signal, SlotRegisterMultipleConnections)
{
  Signal<void(int)> signal;
  SlotRegister<Signal<void(int)>> reg(signal);

  std::vector<int> values1;
  std::vector<int> values2;

  Connection conn1 = reg.connect([&values1](int v) { values1.push_back(v); });
  Connection conn2 = reg.connect([&values2](int v) { values2.push_back(v); });

  EXPECT_EQ(signal.getNumConnections(), 2u);

  signal.raise(42);

  EXPECT_EQ(values1.size(), 1u);
  EXPECT_EQ(values1[0], 42);
  EXPECT_EQ(values2.size(), 1u);
  EXPECT_EQ(values2[0], 42);

  conn1.disconnect();
  EXPECT_EQ(signal.getNumConnections(), 1u);

  signal.raise(100);

  EXPECT_EQ(values1.size(), 1u);
  EXPECT_EQ(values2.size(), 2u);
  EXPECT_EQ(values2[1], 100);
}

TEST(Signal, RaiseMultipleTimes)
{
  Signal<void()> signal;
  int callCount = 0;

  signal.connect([&callCount]() { ++callCount; });

  signal.raise();
  signal.raise();
  signal.raise();

  EXPECT_EQ(callCount, 3);
}

TEST(Signal, ConnectAfterRaise)
{
  Signal<void()> signal;
  int callCount1 = 0;
  int callCount2 = 0;

  signal.connect([&callCount1]() { ++callCount1; });
  signal.raise();

  EXPECT_EQ(callCount1, 1);
  EXPECT_EQ(callCount2, 0);

  signal.connect([&callCount2]() { ++callCount2; });
  signal.raise();

  EXPECT_EQ(callCount1, 2);
  EXPECT_EQ(callCount2, 1);
}

TEST(Signal, DisconnectAllDuringEmission)
{
  Signal<void()> signal;
  int callCount1 = 0;
  int callCount2 = 0;

  signal.connect([&]() {
    ++callCount1;
    signal.disconnectAll();
  });

  signal.connect([&callCount2]() { ++callCount2; });

  EXPECT_EQ(signal.getNumConnections(), 2u);

  signal.raise();

  // First slot called, then disconnectAll, but second slot still called
  // because raise() copies the connection list
  EXPECT_EQ(callCount1, 1);
  EXPECT_EQ(callCount2, 1);
  EXPECT_EQ(signal.getNumConnections(), 0u);
}
