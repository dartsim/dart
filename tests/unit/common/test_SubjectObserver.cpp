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

#include "dart/common/Observer.hpp"
#include "dart/common/Subject.hpp"
#include "dart/common/sub_ptr.hpp"

#include <gtest/gtest.h>

#include <memory>

using namespace dart;
using namespace dart::common;

namespace {

class TestSubject : public Subject
{
public:
  void notify()
  {
    sendDestructionNotification();
  }

  std::size_t observerCount() const
  {
    return mObservers.size();
  }
};

class TestObserver : public Observer
{
public:
  void track(TestSubject* subject)
  {
    addSubject(subject);
  }

  std::size_t subjectCount() const
  {
    return mSubjects.size();
  }

  int notificationCount() const
  {
    return mNotificationCount;
  }

protected:
  void handleDestructionNotification(const Subject*) override
  {
    ++mNotificationCount;
  }

private:
  int mNotificationCount = 0;
};

} // namespace

//==============================================================================
TEST(SubjectObserverTests, BasicNotification)
{
  TestObserver observer;
  TestSubject subject;

  observer.track(&subject);
  EXPECT_EQ(observer.subjectCount(), 1u);
  EXPECT_EQ(subject.observerCount(), 1u);

  subject.notify();
  EXPECT_EQ(observer.subjectCount(), 0u);
  EXPECT_EQ(subject.observerCount(), 0u);
  EXPECT_EQ(observer.notificationCount(), 1);
}

//==============================================================================
TEST(SubjectObserverTests, ObserverDestructorDetaches)
{
  TestSubject subject;
  {
    TestObserver observer;
    observer.track(&subject);
    EXPECT_EQ(subject.observerCount(), 1u);
  }
  EXPECT_EQ(subject.observerCount(), 0u);
}

//==============================================================================
TEST(SubjectObserverTests, SubjectDestructorNotifiesObserver)
{
  TestObserver observer;
  {
    auto subject = std::make_unique<TestSubject>();
    observer.track(subject.get());
    EXPECT_EQ(observer.subjectCount(), 1u);
  }
  EXPECT_EQ(observer.subjectCount(), 0u);
  EXPECT_EQ(observer.notificationCount(), 1);
}

//==============================================================================
TEST(SubjectObserverTests, MultipleObserversReceiveNotifications)
{
  TestObserver observer1;
  TestObserver observer2;
  TestObserver observer3;

  {
    TestSubject subject;
    observer1.track(&subject);
    observer2.track(&subject);
    observer3.track(&subject);
    EXPECT_EQ(subject.observerCount(), 3u);
  }

  EXPECT_EQ(observer1.notificationCount(), 1);
  EXPECT_EQ(observer2.notificationCount(), 1);
  EXPECT_EQ(observer3.notificationCount(), 1);
}

//==============================================================================
// Regression: destroying a sibling observer during sendDestructionNotification
// invalidated the iterator (SEGFAULT on macOS arm64 Debug). The swap-into-
// local fix makes this safe.
//
// Both observers are heap-allocated and each holds a destroy-handle to the
// other. Whichever is notified first destroys its peer, exercising the
// iterator-invalidation path regardless of std::set pointer ordering.
class PeerDestroyingObserver : public Observer
{
public:
  void track(TestSubject* subject)
  {
    addSubject(subject);
  }

  void setPeerToDestroy(std::unique_ptr<PeerDestroyingObserver>* peerOwner)
  {
    mPeerOwner = peerOwner;
  }

  std::size_t subjectCount() const
  {
    return mSubjects.size();
  }

  int notificationCount() const
  {
    return mNotificationCount;
  }

protected:
  void handleDestructionNotification(const Subject*) override
  {
    ++mNotificationCount;
    if (mPeerOwner && *mPeerOwner) {
      mPeerOwner->reset();
    }
  }

private:
  std::unique_ptr<PeerDestroyingObserver>* mPeerOwner = nullptr;
  int mNotificationCount = 0;
};

//==============================================================================
TEST(SubjectObserverTests, ObserverDestroysPeerDuringNotification)
{
  TestSubject subject;
  auto observerA = std::make_unique<PeerDestroyingObserver>();
  auto observerB = std::make_unique<PeerDestroyingObserver>();

  observerA->track(&subject);
  observerB->track(&subject);
  EXPECT_EQ(subject.observerCount(), 2u);

  // Each observer holds a destroy-handle to the other, so whichever is
  // notified first will destroy its peer — exercising the invalidation
  // path regardless of pointer ordering in std::set.
  observerA->setPeerToDestroy(&observerB);
  observerB->setPeerToDestroy(&observerA);

  subject.notify();

  EXPECT_EQ(subject.observerCount(), 0u);
  // Exactly one was destroyed by the other's callback
  EXPECT_TRUE(!observerA || !observerB);
}
