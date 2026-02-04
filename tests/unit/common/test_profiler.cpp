// Copyright (c) 2011, The DART development contributors

#include <dart/common/detail/profiler.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <sstream>
#include <string>
#include <thread>

#include <cstdlib>

namespace dart::test {
namespace {

class ScopedEnvVar
{
public:
  ScopedEnvVar(const char* name, const char* value) : mName(name)
  {
    const char* current = std::getenv(name);
    if (current) {
      mHadValue = true;
      mOldValue = current;
    }
    set(value);
  }

  ~ScopedEnvVar()
  {
    if (mHadValue) {
      set(mOldValue.c_str());
    } else {
      unset();
    }
  }

private:
  void set(const char* value)
  {
#ifdef _WIN32
    _putenv_s(mName.c_str(), value ? value : "");
#else
    if (value) {
      setenv(mName.c_str(), value, 1);
    } else {
      unsetenv(mName.c_str());
    }
#endif
  }

  void unset()
  {
#ifdef _WIN32
    _putenv_s(mName.c_str(), "");
#else
    unsetenv(mName.c_str());
#endif
  }

  std::string mName;
  std::string mOldValue;
  bool mHadValue{false};
};

} // namespace

TEST(Profiler, EmptySummary)
{
  auto& profiler = common::profile::Profiler::instance();
  profiler.reset();

  std::ostringstream oss;
  profiler.printSummary(oss);

  const std::string output = oss.str();
  EXPECT_NE(output.find("no scoped regions were recorded"), std::string::npos);
}

TEST(Profiler, RecordsScopesAndFramesWithoutColor)
{
  auto& profiler = common::profile::Profiler::instance();
  profiler.reset();

  ScopedEnvVar env("DART_PROFILE_COLOR", "0");

  {
    common::profile::ProfileScope outer("OuterScope", __FILE__, __LINE__);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    {
      common::profile::ProfileScope inner("InnerScope", __FILE__, __LINE__);
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
  }

  profiler.markFrame();
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  profiler.markFrame();

  std::ostringstream oss;
  profiler.printSummary(oss);

  const std::string output = oss.str();
  EXPECT_NE(output.find("DART profiler (text backend)"), std::string::npos);
  EXPECT_NE(output.find("OuterScope"), std::string::npos);
  EXPECT_NE(output.find("InnerScope"), std::string::npos);
  EXPECT_EQ(output.find("\033["), std::string::npos);
}

TEST(Profiler, ColorizedOutput)
{
  auto& profiler = common::profile::Profiler::instance();
  profiler.reset();

  ScopedEnvVar env("DART_PROFILE_COLOR", "1");

  {
    common::profile::ProfileScope scope("ColorScope", __FILE__, __LINE__);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  profiler.markFrame();
  profiler.markFrame();

  std::ostringstream oss;
  profiler.printSummary(oss);

  const std::string output = oss.str();
  EXPECT_NE(output.find("ColorScope"), std::string::npos);
  EXPECT_NE(output.find("\033["), std::string::npos);
}

TEST(Profiler, UseColorEnvVarVariants)
{
  {
    ScopedEnvVar env("DART_PROFILE_COLOR", "TRUE");
    auto& profiler = common::profile::Profiler::instance();
    profiler.reset();
    {
      common::profile::ProfileScope scope("TrueScope", __FILE__, __LINE__);
    }
    profiler.markFrame();
    std::ostringstream oss;
    profiler.printSummary(oss);
    EXPECT_NE(oss.str().find("\033["), std::string::npos);
  }

  {
    ScopedEnvVar env("DART_PROFILE_COLOR", "YES");
    auto& profiler = common::profile::Profiler::instance();
    profiler.reset();
    {
      common::profile::ProfileScope scope("YesScope", __FILE__, __LINE__);
    }
    profiler.markFrame();
    std::ostringstream oss;
    profiler.printSummary(oss);
    EXPECT_NE(oss.str().find("\033["), std::string::npos);
  }

  {
    ScopedEnvVar env("DART_PROFILE_COLOR", "ON");
    auto& profiler = common::profile::Profiler::instance();
    profiler.reset();
    {
      common::profile::ProfileScope scope("OnScope", __FILE__, __LINE__);
    }
    profiler.markFrame();
    std::ostringstream oss;
    profiler.printSummary(oss);
    EXPECT_NE(oss.str().find("\033["), std::string::npos);
  }
}

TEST(Profiler, MultipleFrameTiming)
{
  auto& profiler = common::profile::Profiler::instance();
  profiler.reset();

  ScopedEnvVar env("DART_PROFILE_COLOR", "0");

  for (int i = 0; i < 5; ++i) {
    {
      common::profile::ProfileScope scope("FrameWork", __FILE__, __LINE__);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    profiler.markFrame();
  }

  std::ostringstream oss;
  profiler.printSummary(oss);

  const std::string output = oss.str();
  EXPECT_NE(output.find("FrameWork"), std::string::npos);
  EXPECT_NE(output.find("Frames marked: 5"), std::string::npos);
}

TEST(Profiler, NestedScopes)
{
  auto& profiler = common::profile::Profiler::instance();
  profiler.reset();

  ScopedEnvVar env("DART_PROFILE_COLOR", "0");

  {
    common::profile::ProfileScope outer("OuterNested", __FILE__, __LINE__);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    {
      common::profile::ProfileScope middle("MiddleNested", __FILE__, __LINE__);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      {
        common::profile::ProfileScope inner("InnerNested", __FILE__, __LINE__);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  profiler.markFrame();

  std::ostringstream oss;
  profiler.printSummary(oss);

  const std::string output = oss.str();
  EXPECT_NE(output.find("OuterNested"), std::string::npos);
  EXPECT_NE(output.find("MiddleNested"), std::string::npos);
  EXPECT_NE(output.find("InnerNested"), std::string::npos);
}

TEST(Profiler, FormatsAndReportGeneration)
{
  using common::profile::Profiler;

  auto& profiler = Profiler::instance();
  profiler.reset();

  ScopedEnvVar env("DART_PROFILE_COLOR", "0");

  {
    common::profile::ProfileScope scope("ReportScope", __FILE__, __LINE__);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  profiler.markFrame();
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  profiler.markFrame();

  std::ostringstream oss;
  profiler.printSummary(oss);

  const std::string output = oss.str();
  EXPECT_NE(output.find("ReportScope"), std::string::npos);
  EXPECT_NE(output.find("Frames marked: 2"), std::string::npos);
  EXPECT_NE(output.find("Total scoped time:"), std::string::npos);
  EXPECT_NE(output.find("Avg FPS:"), std::string::npos);
}

TEST(Profiler, SummaryMultipleThreads)
{
  auto& profiler = common::profile::Profiler::instance();
  profiler.reset();

  ScopedEnvVar env("DART_PROFILE_COLOR", "0");

  std::thread worker([]() {
    common::profile::ProfileScope scope("WorkerScope", __FILE__, __LINE__);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  });

  {
    common::profile::ProfileScope scope("MainScope", __FILE__, __LINE__);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  profiler.markFrame();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  profiler.markFrame();

  worker.join();

  std::ostringstream oss;
  profiler.printSummary(oss);

  const std::string output = oss.str();
  EXPECT_NE(output.find("Per-thread breakdown"), std::string::npos);
  EXPECT_NE(output.find("thread"), std::string::npos);
}

} // namespace dart::test

namespace dart::common::profile {

class ProfilerTestAccess
{
public:
  static std::string fd(std::uint64_t ns)
  {
    return Profiler::formatDuration(ns);
  }
  static std::string fda(std::uint64_t ns)
  {
    return Profiler::formatDurationAligned(ns);
  }
  static std::string ff(double fps)
  {
    return Profiler::formatFps(fps);
  }
  static std::string fc(std::uint64_t v)
  {
    return Profiler::formatCount(v);
  }
  static void popEmpty(Profiler& p)
  {
    auto rec = p.registerThread();
    p.popScope(*rec);
  }
};

} // namespace dart::common::profile

using PTA = dart::common::profile::ProfilerTestAccess;

TEST(Profiler, FormatDurationScales)
{
  EXPECT_NE(PTA::fd(2'500'000'000ULL).find("s"), std::string::npos);
  EXPECT_NE(PTA::fd(5'000'000ULL).find("ms"), std::string::npos);
  EXPECT_NE(PTA::fd(500ULL).find("s"), std::string::npos);
}

TEST(Profiler, FormatDurationAlignedScales)
{
  EXPECT_NE(PTA::fda(2'500'000'000ULL).find("s"), std::string::npos);
  EXPECT_NE(PTA::fda(5'000'000ULL).find("ms"), std::string::npos);
  EXPECT_NE(PTA::fda(5'000ULL).find("s"), std::string::npos);
  EXPECT_NE(PTA::fda(42ULL).find("ns"), std::string::npos);
}

TEST(Profiler, FormatDurationAlignedThresholds)
{
  EXPECT_NE(PTA::fda(1'200'000'000ULL).find("s"), std::string::npos);
  EXPECT_NE(PTA::fda(1'200'000ULL).find("ms"), std::string::npos);
  EXPECT_NE(PTA::fda(1'200ULL).find("µs"), std::string::npos);
}

TEST(Profiler, FormatFpsScales)
{
  EXPECT_NE(PTA::ff(2'500'000.0).find("M"), std::string::npos);
  EXPECT_NE(PTA::ff(5'000.0).find("k"), std::string::npos);
  EXPECT_FALSE(PTA::ff(60.0).empty());
}

TEST(Profiler, FormatCountScales)
{
  EXPECT_NE(PTA::fc(5'000'000'000ULL).find("B"), std::string::npos);
  EXPECT_NE(PTA::fc(5'000'000ULL).find("M"), std::string::npos);
  EXPECT_NE(PTA::fc(5'000ULL).find("K"), std::string::npos);
  EXPECT_EQ(PTA::fc(42ULL), "42");
}

TEST(Profiler, PopScopeOnEmptyStack)
{
  auto& profiler = dart::common::profile::Profiler::instance();
  PTA::popEmpty(profiler);
}
