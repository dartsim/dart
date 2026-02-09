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

#include "dart/common/observer.hpp"
#include "dart/common/sub_ptr.hpp"
#include "dart/common/subject.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <vector>

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

class RecordingObserver : public Observer
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

  const std::vector<const Subject*>& notifications() const
  {
    return mNotifications;
  }

protected:
  void handleDestructionNotification(const Subject* subject) override
  {
    mNotifications.push_back(subject);
  }

private:
  std::vector<const Subject*> mNotifications;
};

} // namespace

//==============================================================================
TEST(SubjectObserverTests, SubjectNotifiesObserversOnce)
{
  RecordingObserver observer;
  TestSubject subject;

  observer.track(&subject);
  EXPECT_EQ(observer.subjectCount(), 1u);
  EXPECT_EQ(subject.observerCount(), 1u);

  subject.notify();
  EXPECT_EQ(observer.subjectCount(), 0u);
  EXPECT_EQ(subject.observerCount(), 0u);
  ASSERT_EQ(observer.notifications().size(), 1u);
  EXPECT_EQ(observer.notifications().front(), &subject);
}

//==============================================================================
TEST(SubjectObserverTests, ObserverDestructorDetachesFromSubjects)
{
  TestSubject subject;
  {
    RecordingObserver observer;
    observer.track(&subject);
    EXPECT_EQ(subject.observerCount(), 1u);
  }
  EXPECT_EQ(subject.observerCount(), 0u);
}

//==============================================================================
TEST(SubjectObserverTests, SubjectDestructorDetachesFromObserver)
{
  RecordingObserver observer;
  {
    auto subject = std::make_unique<TestSubject>();
    observer.track(subject.get());
    EXPECT_EQ(observer.subjectCount(), 1u);
  }
  EXPECT_EQ(observer.subjectCount(), 0u);
  ASSERT_EQ(observer.notifications().size(), 1u);
  EXPECT_NE(observer.notifications().front(), nullptr);
}

//==============================================================================
TEST(SubjectObserverTests, MultipleObserversReceiveNotifications)
{
  RecordingObserver observer1;
  RecordingObserver observer2;
  RecordingObserver observer3;

  {
    TestSubject subject;
    observer1.track(&subject);
    observer2.track(&subject);
    observer3.track(&subject);
    EXPECT_EQ(subject.observerCount(), 3u);
  }

  EXPECT_EQ(observer1.notifications().size(), 1u);
  EXPECT_EQ(observer2.notifications().size(), 1u);
  EXPECT_EQ(observer3.notifications().size(), 1u);
}

//==============================================================================
TEST(SubjectObserverTests, ObserverTracksMultipleSubjects)
{
  RecordingObserver observer;

  {
    TestSubject subject1;
    TestSubject subject2;
    observer.track(&subject1);
    observer.track(&subject2);
    EXPECT_EQ(observer.subjectCount(), 2u);
  }

  EXPECT_EQ(observer.subjectCount(), 0u);
  EXPECT_EQ(observer.notifications().size(), 2u);
}

//==============================================================================
TEST(SubPtr, DefaultConstruction)
{
  common::sub_ptr<TestSubject> ptr;

  EXPECT_EQ(ptr.get(), nullptr);
  EXPECT_FALSE(ptr.valid());
}

//==============================================================================
TEST(SubPtr, ConstructWithPointer)
{
  TestSubject subject;
  common::sub_ptr<TestSubject> ptr(&subject);

  EXPECT_EQ(ptr.get(), &subject);
  EXPECT_TRUE(ptr.valid());
}

//==============================================================================
TEST(SubPtr, ImplicitConversion)
{
  TestSubject subject;
  common::sub_ptr<TestSubject> ptr(&subject);

  TestSubject* rawPtr = ptr;
  EXPECT_EQ(rawPtr, &subject);
}

//==============================================================================
TEST(SubPtr, DereferenceOperator)
{
  TestSubject subject;
  common::sub_ptr<TestSubject> ptr(&subject);

  TestSubject& ref = *ptr;
  EXPECT_EQ(&ref, &subject);
}

//==============================================================================
TEST(SubPtr, ArrowOperator)
{
  TestSubject subject;
  common::sub_ptr<TestSubject> ptr(&subject);

  EXPECT_EQ(ptr->observerCount(), 1u);
}

//==============================================================================
TEST(SubPtr, BecomesNullOnSubjectDestruction)
{
  common::sub_ptr<TestSubject> ptr;

  {
    TestSubject subject;
    ptr = &subject;
    EXPECT_TRUE(ptr.valid());
    EXPECT_EQ(ptr.get(), &subject);
  }

  EXPECT_FALSE(ptr.valid());
  EXPECT_EQ(ptr.get(), nullptr);
}

//==============================================================================
TEST(SubPtr, SetMethod)
{
  TestSubject subject1;
  TestSubject subject2;
  common::sub_ptr<TestSubject> ptr;

  ptr.set(&subject1);
  EXPECT_EQ(ptr.get(), &subject1);

  ptr.set(&subject2);
  EXPECT_EQ(ptr.get(), &subject2);

  ptr.set(nullptr);
  EXPECT_EQ(ptr.get(), nullptr);
}

//==============================================================================
TEST(SubPtr, AssignmentFromSubPtr)
{
  TestSubject subject;
  common::sub_ptr<TestSubject> ptr1(&subject);
  common::sub_ptr<TestSubject> ptr2;

  ptr2 = ptr1;

  EXPECT_EQ(ptr2.get(), &subject);
  EXPECT_TRUE(ptr2.valid());
}

//==============================================================================
TEST(SubPtr, CopyConstruction)
{
  TestSubject subject;
  common::sub_ptr<TestSubject> ptr1(&subject);
  common::sub_ptr<TestSubject> ptr2(ptr1);

  EXPECT_EQ(ptr1.get(), &subject);
  EXPECT_EQ(ptr2.get(), &subject);
}

//==============================================================================
TEST(SubPtr, AssignPointerDirectly)
{
  TestSubject subject1;
  TestSubject subject2;
  common::sub_ptr<TestSubject> ptr;

  ptr = &subject1;
  EXPECT_EQ(ptr.get(), &subject1);

  ptr = &subject2;
  EXPECT_EQ(ptr.get(), &subject2);
}

//==============================================================================
TEST(SubPtr, MultiplePointersToSameSubject)
{
  common::sub_ptr<TestSubject> ptr1;
  common::sub_ptr<TestSubject> ptr2;
  common::sub_ptr<TestSubject> ptr3;

  {
    TestSubject subject;
    ptr1 = &subject;
    ptr2 = &subject;
    ptr3 = &subject;

    EXPECT_TRUE(ptr1.valid());
    EXPECT_TRUE(ptr2.valid());
    EXPECT_TRUE(ptr3.valid());
    EXPECT_EQ(subject.observerCount(), 3u);
  }

  EXPECT_FALSE(ptr1.valid());
  EXPECT_FALSE(ptr2.valid());
  EXPECT_FALSE(ptr3.valid());
}

//==============================================================================
TEST(SubPtr, ReassignAfterSubjectDestruction)
{
  common::sub_ptr<TestSubject> ptr;

  {
    TestSubject subject1;
    ptr = &subject1;
  }

  EXPECT_FALSE(ptr.valid());

  TestSubject subject2;
  ptr = &subject2;

  EXPECT_TRUE(ptr.valid());
  EXPECT_EQ(ptr.get(), &subject2);
}

//==============================================================================
TEST(SubPtr, ReassignToNewSubjectBeforeDestruction)
{
  TestSubject subject1;
  TestSubject subject2;
  common::sub_ptr<TestSubject> ptr(&subject1);

  EXPECT_EQ(subject1.observerCount(), 1u);
  EXPECT_EQ(subject2.observerCount(), 0u);

  ptr = &subject2;

  EXPECT_EQ(subject1.observerCount(), 0u);
  EXPECT_EQ(subject2.observerCount(), 1u);
}

//==============================================================================
class TestObserver : public Observer
{
public:
  void track(TestSubject* subject)
  {
    addSubject(subject);
  }

  void untrack(const Subject* subject)
  {
    removeSubject(subject);
  }

  void untrackAll()
  {
    removeAllSubjects();
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

//==============================================================================
TEST(SubjectObserverTests, RemoveAllSubjects)
{
  TestSubject subject1;
  TestSubject subject2;
  TestSubject subject3;
  TestObserver observer;

  observer.track(&subject1);
  observer.track(&subject2);
  observer.track(&subject3);

  EXPECT_EQ(observer.subjectCount(), 3u);
  EXPECT_EQ(subject1.observerCount(), 1u);
  EXPECT_EQ(subject2.observerCount(), 1u);
  EXPECT_EQ(subject3.observerCount(), 1u);

  observer.untrackAll();

  EXPECT_EQ(observer.subjectCount(), 0u);
  EXPECT_EQ(subject1.observerCount(), 0u);
  EXPECT_EQ(subject2.observerCount(), 0u);
  EXPECT_EQ(subject3.observerCount(), 0u);
}

//==============================================================================
TEST(SubjectObserverTests, AddNullSubjectIsIgnored)
{
  TestObserver observer;

  observer.track(nullptr);

  EXPECT_EQ(observer.subjectCount(), 0u);
}

//==============================================================================
TEST(SubjectObserverTests, RemoveNullSubjectIsIgnored)
{
  TestSubject subject;
  TestObserver observer;

  observer.track(&subject);
  EXPECT_EQ(observer.subjectCount(), 1u);

  observer.untrack(nullptr);

  EXPECT_EQ(observer.subjectCount(), 1u);
}

//==============================================================================
TEST(SubjectObserverTests, AddDuplicateSubjectIsIgnored)
{
  TestSubject subject;
  TestObserver observer;

  observer.track(&subject);
  observer.track(&subject);
  observer.track(&subject);

  EXPECT_EQ(observer.subjectCount(), 1u);
  EXPECT_EQ(subject.observerCount(), 1u);
}

//==============================================================================
TEST(SubjectObserverTests, RemoveNonTrackedSubjectIsIgnored)
{
  TestSubject subject1;
  TestSubject subject2;
  TestObserver observer;

  observer.track(&subject1);
  EXPECT_EQ(observer.subjectCount(), 1u);

  observer.untrack(&subject2);

  EXPECT_EQ(observer.subjectCount(), 1u);
  EXPECT_EQ(subject1.observerCount(), 1u);
}

//==============================================================================
TEST(SubjectObserverTests, MultipleObserversOnSingleSubjectAllNotified)
{
  TestSubject subject;
  TestObserver observer1;
  TestObserver observer2;
  TestObserver observer3;

  observer1.track(&subject);
  observer2.track(&subject);
  observer3.track(&subject);

  EXPECT_EQ(subject.observerCount(), 3u);

  subject.notify();

  EXPECT_EQ(observer1.notificationCount(), 1);
  EXPECT_EQ(observer2.notificationCount(), 1);
  EXPECT_EQ(observer3.notificationCount(), 1);
  EXPECT_EQ(observer1.subjectCount(), 0u);
  EXPECT_EQ(observer2.subjectCount(), 0u);
  EXPECT_EQ(observer3.subjectCount(), 0u);
}

//==============================================================================
TEST(
    SubjectObserverTests,
    ObserverReceivesMultipleNotificationsFromDifferentSubjects)
{
  TestObserver observer;

  {
    TestSubject subject1;
    TestSubject subject2;
    observer.track(&subject1);
    observer.track(&subject2);

    EXPECT_EQ(observer.subjectCount(), 2u);

    subject1.notify();
    EXPECT_EQ(observer.notificationCount(), 1);
    EXPECT_EQ(observer.subjectCount(), 1u);

    subject2.notify();
    EXPECT_EQ(observer.notificationCount(), 2);
    EXPECT_EQ(observer.subjectCount(), 0u);
  }
}

//==============================================================================
TEST(SubjectObserverTests, ManualDisconnectBeforeDestruction)
{
  TestSubject subject;
  TestObserver observer;

  observer.track(&subject);
  EXPECT_EQ(observer.subjectCount(), 1u);
  EXPECT_EQ(subject.observerCount(), 1u);

  observer.untrack(&subject);
  EXPECT_EQ(observer.subjectCount(), 0u);
  EXPECT_EQ(subject.observerCount(), 0u);

  subject.notify();
  EXPECT_EQ(observer.notificationCount(), 0);
}

//==============================================================================
TEST(SubjectObserverTests, ObserverDestructorCleansUpAllConnections)
{
  TestSubject subject1;
  TestSubject subject2;

  {
    TestObserver observer;
    observer.track(&subject1);
    observer.track(&subject2);

    EXPECT_EQ(subject1.observerCount(), 1u);
    EXPECT_EQ(subject2.observerCount(), 1u);
  }

  EXPECT_EQ(subject1.observerCount(), 0u);
  EXPECT_EQ(subject2.observerCount(), 0u);
}

//==============================================================================
class SelfRemovingObserver : public Observer
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
    removeAllSubjects();
  }

private:
  int mNotificationCount = 0;
};

//==============================================================================
TEST(SubjectObserverTests, ObserverRemovesAllSubjectsDuringNotification)
{
  TestSubject subject1;
  TestSubject subject2;
  SelfRemovingObserver observer;

  observer.track(&subject1);
  observer.track(&subject2);

  EXPECT_EQ(observer.subjectCount(), 2u);

  subject1.notify();

  EXPECT_EQ(observer.notificationCount(), 1);
  EXPECT_EQ(observer.subjectCount(), 0u);
  EXPECT_EQ(subject2.observerCount(), 0u);
}

//==============================================================================
TEST(SubjectObserverTests, NotifySignalClearsObserverList)
{
  TestSubject subject;
  TestObserver observer1;
  TestObserver observer2;

  observer1.track(&subject);
  observer2.track(&subject);

  EXPECT_EQ(subject.observerCount(), 2u);

  subject.notify();

  EXPECT_EQ(subject.observerCount(), 0u);
}

//==============================================================================
TEST(SubjectObserverTests, ReconnectAfterNotification)
{
  TestSubject subject;
  TestObserver observer;

  observer.track(&subject);
  EXPECT_EQ(observer.subjectCount(), 1u);

  subject.notify();
  EXPECT_EQ(observer.subjectCount(), 0u);
  EXPECT_EQ(observer.notificationCount(), 1);

  observer.track(&subject);
  EXPECT_EQ(observer.subjectCount(), 1u);
  EXPECT_EQ(subject.observerCount(), 1u);

  subject.notify();
  EXPECT_EQ(observer.notificationCount(), 2);
}
