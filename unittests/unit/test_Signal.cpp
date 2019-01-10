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

#include <numeric>
#include <gtest/gtest.h>

#include "dart/dart.hpp"

using namespace std;
using namespace Eigen;
using namespace dart;
using namespace common;
using namespace dynamics;
using namespace simulation;

static int callCount0 = 0;
static int callCount1 = 0;
static int callCount2 = 0;

//==============================================================================
void foo0() { callCount0++; }
void foo1(int /*_val*/) { callCount1++; }
void foo2(int /*_val1*/, float /*_val2*/) { callCount2++; }
double foo3() { return 10.0; }

//==============================================================================
class Viewer
{
public:
  static void onSignal1Static(int /*_val*/) { callCount1++; }
  static void onSignal2Static(int /*_val1*/, float /*_val2*/) { callCount2++; }
  void onSignal1(int /*_val*/) { callCount1++; }
  void onSignal2(int /*_val1*/, float /*_val2*/) { callCount2++; }
};

//==============================================================================
TEST(Signal, Basic)
{
  Signal<void()> signal0;
  Signal<void(int)> signal1;
  Signal<void(int, float)> signal2;
  Signal<double()> signal3;

  Connection connection0 = signal0.connect(&foo0);
  Connection connection1 = signal1.connect(&foo1);
  Connection connection2 = signal2.connect(&foo2);
  Connection connection3 = signal3.connect(&foo3);

  EXPECT_EQ(static_cast<int>(signal0.getNumConnections()), 1);
  EXPECT_EQ(static_cast<int>(signal1.getNumConnections()), 1);
  EXPECT_EQ(static_cast<int>(signal2.getNumConnections()), 1);
  EXPECT_EQ(static_cast<int>(signal3.getNumConnections()), 1);

  EXPECT_TRUE(connection0.isConnected());
  EXPECT_TRUE(connection1.isConnected());
  EXPECT_TRUE(connection2.isConnected());
  EXPECT_TRUE(connection3.isConnected());

  connection0.disconnect();
  connection1.disconnect();
  connection2.disconnect();
  connection3.disconnect();

  EXPECT_EQ(static_cast<int>(signal0.getNumConnections()), 0);
  EXPECT_EQ(static_cast<int>(signal1.getNumConnections()), 0);
  EXPECT_EQ(static_cast<int>(signal2.getNumConnections()), 0);
  EXPECT_EQ(static_cast<int>(signal3.getNumConnections()), 0);

  signal0();

  EXPECT_FALSE(connection0.isConnected());
  EXPECT_FALSE(connection1.isConnected());
  EXPECT_FALSE(connection2.isConnected());
  EXPECT_FALSE(connection3.isConnected());
}

//==============================================================================
TEST(Signal, NonStaticMemberFunction)
{
  Signal<void(int)> signal1;
  Signal<void(int, float)> signal2;
  Viewer viewer;

  // Connect static member function
  signal1.connect(&Viewer::onSignal1Static);
  signal2.connect(&Viewer::onSignal2Static);

  // Connect non-static member function
  using placeholders::_1;
  using placeholders::_2;
  signal1.connect(bind(&Viewer::onSignal1, &viewer, _1));
  signal2.connect(bind(&Viewer::onSignal2, &viewer, _1, _2));

  // The signal should have the maximum number of listeners
  EXPECT_EQ(static_cast<int>(signal1.getNumConnections()), 2);
  EXPECT_EQ(static_cast<int>(signal2.getNumConnections()), 2);

  // Check the number of calls
  callCount1 = 0;
  callCount2 = 0;
  signal1.raise(0);
  signal2.raise(0, 0);
  EXPECT_EQ(callCount1, 2);
  EXPECT_EQ(callCount2, 2);
}

//==============================================================================
TEST(Signal, ScopedConnection)
{
  Signal<void(int)> signal;
  Connection c = signal.connect(foo1);
  EXPECT_EQ(static_cast<int>(signal.getNumConnections()), 1);

  {
    ScopedConnection sc(signal.connect(foo1));
    EXPECT_EQ(static_cast<int>(signal.getNumConnections()), 2);
  }

  EXPECT_EQ(static_cast<int>(signal.getNumConnections()), 1);
}

//==============================================================================
TEST(Signal, ConnectionLifeTime)
{
  Signal<void(int)>* signal = new Signal<void(int)>();

  Connection connection1 = signal->connect(foo1);
  EXPECT_TRUE(connection1.isConnected());

  Connection connection2 = signal->connect(foo1);
  EXPECT_TRUE(connection2.isConnected());

  {
    ScopedConnection scopedConnection(connection2);
    EXPECT_TRUE(scopedConnection.isConnected());
    EXPECT_TRUE(connection2.isConnected());
    // scopedConnection disconnected connection2 when it was destroyed.
  }
  EXPECT_FALSE(connection2.isConnected());

  delete signal;

  // Although the signal is destroyed, its connection still works. For those
  // connections, isConnected() returns false.
  EXPECT_FALSE(connection1.isConnected());
}

//==============================================================================
float product(float x, float y) { return x * y; }
float quotient(float x, float y) { return x / y; }
float sum(float x, float y) { return x + y; }
float difference(float x, float y) { return x - y; }

// combiner which returns the maximum value returned by all slots
template <typename T>
struct signal_maximum
{
  typedef T result_type;

  template <typename InputIterator>
  static T process(InputIterator first, InputIterator last)
  {
    // If there are no slots to call, just return the
    // default-constructed value
    if (first == last)
      return T();

    T max_value = *first++;

    while (first != last)
    {
      if (max_value < *first)
        max_value = *first;
      ++first;
    }

    return max_value;
  }
};

// combiner which returns the maximum value returned by all slots
template <typename T>
struct signal_sum
{
  typedef T result_type;

  template <typename InputIterator>
  static T process(InputIterator first, InputIterator last)
  {
    // If there are no slots to call, just return the
    // default-constructed value
    if (first == last)
      return T();

    T sum = *first;
    first++;

    while (first != last)
    {
      sum += *first;
      ++first;
    }

    return sum;
  }
};

//==============================================================================
TEST(Signal, ReturnValues)
{
  const float tol = 1.5e-6;

  const float a = 5.0f;
  const float b = 3.0f;

  std::vector<float> res(4);
  res[0] = product(a, b);
  res[1] = quotient(a, b);
  res[2] = sum(a, b);
  res[3] = difference(a, b);

  // signal_maximum
  Signal<float(float, float), signal_maximum> signal1;
  signal1.connect(&product);
  signal1.connect(&quotient);
  signal1.connect(&sum);
  signal1.connect(&difference);
  EXPECT_EQ(signal1(5, 3), *std::max_element(res.begin(), res.end()));

  // signal_sum
  Signal<float(float, float), signal_sum> signal2;
  signal2.connect(&product);
  signal2.connect(&quotient);
  signal2.connect(&sum);
  signal2.connect(&difference);
  EXPECT_NEAR(signal2(5, 3), std::accumulate(res.begin(), res.end(), 0.0), tol);
}

//==============================================================================
void frameChangeCallback(const Entity* _entity,
                         const Frame* _oldParentFrame,
                         const Frame* _newParentFrame)
{
  assert(_entity);

  std::string oldFrameName
      = _oldParentFrame == nullptr ? "(empty)" : _oldParentFrame->getName();
  std::string newFrameName
      = _newParentFrame == nullptr ? "(empty)" : _newParentFrame->getName();

  if(_newParentFrame)
    std::cout << "[" << _entity->getName() << "]: "
              << oldFrameName << " --> " << newFrameName << std::endl;
  else
    std::cout << "Entity (" << _entity << ") has been destroyed" << std::endl;
}

//==============================================================================
void nameChangedCallback(const Entity* entity,
                         const std::string& oldName,
                         const std::string& newName)
{
  assert(entity);

  std::cout << "[" << entity->getName() << "]: Name changed: '"
            << oldName << "' --> '" << newName << "'.\n";
}

//==============================================================================
TEST(Signal, FrameSignals)
{
  Isometry3d tf1(Isometry3d::Identity());
  tf1.translate(Vector3d(0.1,-0.1,0));

  Isometry3d tf2(Isometry3d::Identity());
  tf2.translate(Vector3d(0,0.1,0));
  tf2.rotate(AngleAxisd(45.0*M_PI/180.0, Vector3d(1,0,0)));

  Isometry3d tf3(Isometry3d::Identity());
  tf3.translate(Vector3d(0,0,0.1));
  tf3.rotate(AngleAxisd(60*M_PI/180.0, Vector3d(0,1,0)));

  SimpleFrame F1(Frame::World(), "F1", tf1);
  SimpleFrame F2(&F1, "F2", tf2);
  SimpleFrame F3(&F2, "F3", tf3);

  Connection cf1 = F1.onFrameChanged.connect(frameChangeCallback);
  Connection cf2 = F2.onFrameChanged.connect(frameChangeCallback);
  ScopedConnection cf3(F3.onFrameChanged.connect(frameChangeCallback));

  Connection cv1 = F1.onNameChanged.connect(nameChangedCallback);
  Connection cv2 = F2.onNameChanged.connect(nameChangedCallback);
  ScopedConnection cv3(F3.onNameChanged.connect(nameChangedCallback));

  F1.setName("new " + F1.getName());
  F2.setName("new " + F2.getName());
  F3.setName("new " + F3.getName());

  F3.setParentFrame(&F1);
}
