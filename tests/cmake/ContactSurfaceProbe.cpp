// gz-physics main 4e2af8c733ac8396a9d2b4dcae1f7f6aa28c5668:
// dartsim/CMakeLists.txt uses CMake's CheckIncludeFileCXX with this source.
// Refresh by checking DART_HAS_CONTACT_SURFACE_HEADER and CMake's probe
// template.
// The CTest uses gz-physics's -std=c++14 with GCC and Clang. With MSVC it uses
// /std:c++17, because DART's headers need C++17 there, so it checks include
// hygiene only.
#include <dart/constraint/ContactSurface.hpp>

int main()
{
  return 0;
}
