#pragma once

#include <string>

#include <gtest/internal/gtest-internal.h>

namespace testing {
namespace internal {

// gz-physics currently vendors an older copy of GoogleTest whose implementation
// exports `MakeAndRegisterTestInfo(const char*, ...)`. Newer GoogleTest headers
// (including the conda-forge `gtest` package) declare a `std::string` overload,
// which can lead to link errors when both versions are present in the include
// path. Provide a small ABI bridge by forwarding the `std::string` overload to
// the legacy implementation.
TestInfo* MakeAndRegisterTestInfo(
    const char* test_suite_name,
    const char* name,
    const char* type_param,
    const char* value_param,
    CodeLocation code_location,
    TypeId fixture_class_id,
    SetUpTestSuiteFunc set_up_tc,
    TearDownTestSuiteFunc tear_down_tc,
    TestFactoryBase* factory);

inline TestInfo* MakeAndRegisterTestInfo(
    std::string test_suite_name,
    const char* name,
    const char* type_param,
    const char* value_param,
    CodeLocation code_location,
    TypeId fixture_class_id,
    SetUpTestSuiteFunc set_up_tc,
    TearDownTestSuiteFunc tear_down_tc,
    TestFactoryBase* factory)
{
  return MakeAndRegisterTestInfo(
      test_suite_name.c_str(),
      name,
      type_param,
      value_param,
      code_location,
      fixture_class_id,
      set_up_tc,
      tear_down_tc,
      factory);
}

} // namespace internal
} // namespace testing
