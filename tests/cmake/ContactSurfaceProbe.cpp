// gz-physics main 4e2af8c733ac8396a9d2b4dcae1f7f6aa28c5668:
// dartsim/CMakeLists.txt uses CMake's CheckIncludeFileCXX with this source and
// CMAKE_REQUIRED_FLAGS /std:c++14 (MSVC) or -std=c++14 (other compilers).
// gz-physics 7 keeps CMake 3.5 policies, so its check compiles as C++14. From
// gz-physics 8, policy CMP0067 also applies CMAKE_CXX_STANDARD 17, whose flag
// came after -std=c++14 with GCC and Ninja (CMake 3.22 to 4.1).
// Refresh by checking DART_HAS_CONTACT_SURFACE_HEADER and CMake's probe
// template.
// The ContactSurfaceProbe CTest checks syntax with only DART and Eigen include
// directories: /std:c++14 /Zs with MSVC, -std=c++14 -pedantic-errors
// -fsyntax-only with GCC, and -std=c++14 -fsyntax-only with other compilers.
// GCC is pedantic so C++17 syntax, which MSVC rejects in C++14 mode, fails
// there too; Clang's pedantic mode also rejects accepted extensions.
#include <dart/constraint/ContactSurface.hpp>

int main()
{
  return 0;
}
