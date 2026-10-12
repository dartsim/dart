// gz-physics main 4e2af8c733ac8396a9d2b4dcae1f7f6aa28c5668:
// dartsim/CMakeLists.txt checks this header with CMake's CheckIncludeFileCXX
// and CMAKE_REQUIRED_FLAGS /std:c++14 (MSVC) or -std=c++14 (other compilers).
// Whether CMake also applies the project's C++17 flag depends on its policies
// and generator, so this include tree stays valid C++14. Refresh by checking
// DART_HAS_CONTACT_SURFACE_HEADER and CMake's probe template.
// The ContactSurfaceProbe CTest checks syntax with only DART and Eigen include
// directories. GCC uses -pedantic-errors and Clang 14 or newer uses
// -Werror=c++17-extensions, so C++17 syntax that MSVC rejects in C++14 mode
// fails on every platform; see tests/CMakeLists.txt for the exact flags.
#include <dart/constraint/ContactSurface.hpp>

int main()
{
  return 0;
}
