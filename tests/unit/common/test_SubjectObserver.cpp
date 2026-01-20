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
