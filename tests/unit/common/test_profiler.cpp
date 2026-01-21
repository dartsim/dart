// Copyright (c) 2011-2025, The DART development contributors

#include <dart/common/detail/Profiler.hpp>

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

} // namespace dart::test
