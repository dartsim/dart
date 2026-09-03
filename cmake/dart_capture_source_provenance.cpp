// Copyright (c) 2011, The DART development contributors
// All rights reserved.

#include <dart/capture_source_provenance.hpp>

#if defined(_WIN32)
#define DART_CAPTURE_IDENTITY_EXPORT __declspec(dllexport)
#else
#define DART_CAPTURE_IDENTITY_EXPORT __attribute__((visibility("default")))
#endif

extern "C" DART_CAPTURE_IDENTITY_EXPORT const char*
dart_capture_source_provenance_digest_v1() noexcept
{
  return DART_CAPTURE_SOURCE_PROVENANCE_DIGEST;
}

extern "C" DART_CAPTURE_IDENTITY_EXPORT const char*
dart_capture_source_git_head_v1() noexcept
{
  return DART_CAPTURE_SOURCE_GIT_HEAD;
}

extern "C" DART_CAPTURE_IDENTITY_EXPORT const char*
dart_capture_build_configuration_digest_v1() noexcept
{
  return DART_CAPTURE_BUILD_CONFIGURATION_DIGEST;
}

extern "C" DART_CAPTURE_IDENTITY_EXPORT const char*
dart_capture_build_target_v1() noexcept
{
  return DART_CAPTURE_BUILD_TARGET;
}

extern "C" DART_CAPTURE_IDENTITY_EXPORT const char*
dart_capture_cmake_build_type_v1() noexcept
{
  return DART_CAPTURE_CMAKE_BUILD_TYPE;
}

extern "C" DART_CAPTURE_IDENTITY_EXPORT const char*
dart_capture_compiler_id_v1() noexcept
{
  return DART_CAPTURE_COMPILER_ID;
}

extern "C" DART_CAPTURE_IDENTITY_EXPORT const char*
dart_capture_compiler_version_v1() noexcept
{
  return DART_CAPTURE_COMPILER_VERSION;
}

extern "C" DART_CAPTURE_IDENTITY_EXPORT int dart_capture_ndebug_v1() noexcept
{
#if defined(NDEBUG)
  return 1;
#else
  return 0;
#endif
}

extern "C" DART_CAPTURE_IDENTITY_EXPORT int
dart_capture_optimization_enabled_v1() noexcept
{
#if defined(__OPTIMIZE__) || (defined(_MSC_VER) && !defined(_DEBUG))
  return 1;
#else
  return 0;
#endif
}
