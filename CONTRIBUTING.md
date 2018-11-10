## How to Contribute to DART?

[Opening a GitHub pull request](https://help.github.com/articles/about-pull-requests/) would be the best way. Please make sure that your code follows DART conventions.

The code doesn't need to be perfect right away, feel free to post work-in-progress versions to get the discussion started.

## DART Contributors

 Name                                               | Contributions
----------------------------------------------------|---------------
 [C. Karen Liu](https://github.com/karenliu)        | project creator, multibody dynamics, constraint resolution, tutorials
 [Mike Stilman](https://github.com/mstilman)        | project creator
 [Siddhartha S. Srinivasa](https://github.com/siddhss5) | project advisor
 [Jeongseok Lee](https://github.com/jslee02)        | project director, multibody dynamics, constraint resolution, collision detection, tutorials
 [Michael X. Grey](https://github.com/mxgrey)       | project director, extensive API improvements, inverse kinematics, gui::osg, tutorials
 [Tobias Kunz](https://github.com/tobiaskunz)       | former project director, motion planner
 [Sumit Jain](http://www.cc.gatech.edu/graphics/projects/Sumit/homepage/) | multibody dynamics
 [Yuting Ye](https://github.com/yutingye)           | multibody dynamics, GUI
 [Michael Koval](https://github.com/mkoval)         | uri, resource retriever, bug fixes
 [Ana C. Huamán Quispe](https://github.com/ana-GT)  | urdf parser
 [Chen Tang](https://github.com/chentang)           | collision detection
 [Matthew Dutton](https://github.com/mdutton3)      | build and bug fixes
 [Eric Huang](https://github.com/ehuang3)           | build and bug fixes
 [Pushkar Kolhe](https://github.com/pushkar)        | early DART build system design
 [Saul Reynolds-Haertle](https://github.com/saulrh) | examples, bug fixes
 [Arash Rouhani](https://github.com/Tarrasch)       | build fixes
 [Kristin Siu](https://github.com/kasiu)            | integrators, bug fixes
 [Steven Peters](https://github.com/scpeters)       | build improvements and fixes
 [Can Erdogan](https://github.com/cerdogan)         | planning, examples
 [Jie Tan](https://github.com/jietan)               | lcp solver, renderer
 [Yunfei Bai](https://github.com/YunfeiBai)         | build and bug fixes
 [Konstantinos Chatzilygeroudis](https://github.com/costashatz) | mimic joint, OSG shadows, build and bug fixes
 [Sehoon Ha](https://github.com/sehoonha)           | early DART data structure design, [pydart](https://github.com/sehoonha/pydart)
 [Donny Ward](https://github.com/donnyward)         | build fix
 [Andrew Price](https://github.com/a-price)         | build fix
 [Eric Tobis](https://github.com/tobis)             | build fix
 [Jonathan Martin](https://github.com/nybblr)       | build fix
 [Jia Ye Li](https://github.com/JiaYeLi)            | fix typo of tutorials
 [Benjamin Chrétien](https://github.com/bchretien)  | bug fix
 [Olzhas Adiyatov](https://github.com/olzhas)       | bug fix
 [José Luis Rivero](https://github.com/j-rivero)    | build, especially for Debian
 [Jonathan Scholz](https://github.com/jscholz)      | build fix
 [John Turgeson](https://github.com/JohnTurgeson)   | mesh model
 [Jennifer Buehler](https://github.com/JenniferBuehler) | heightmap, bug fix
 [Dong Xu](https://github.com/hxbloom)              | motion blur renderer
 [Donghyun Kim](https://github.com/dhkim0821)       | Atlas texture images
 [Aditya Vamsikrishna](https://github.com/aditya-vk) | bug fix
 [pchorak](https://github.com/pchorak)              | bug fixes

You can find the complete contribution history in [here](https://github.com/dartsim/dart/graphs/contributors).

## DART Style Guide

* [C++ Style](#c-style)
  * [C++ Header Style](#header-style)
  * [C++ Source Style](#source-style)
  * [Autoformatting using ClangFormat](#autoformatting-using-clangformat)
* [CMake Style](#cmake-style)

### C++ Style

#### Header Style

C++ headers should be contained in a subdirectory of `include/` that matches
their namespace, with the extension `.hpp`.

* Use **two-space** indentation
* Use **camelCase** function names
* Use **PascalCase** class names
* No "cuddled" braces!

```c++
#ifndef DART_EXAMPLE_EXAMPLECLASS_HPP_  // Header guards must include the library, namespace, and source file names.
#define DART_EXAMPLE_EXAMPLECLASS_HPP_

// Place all dependency includes at the top of the file.
// Use absolute paths, and place these at the top.
#include <stl_headers>
#include <library/headers.hpp>
#include "dart/common/pointers.hpp"
#include "dart/component_name/ExampleInterface.hpp"
#include "dart/component_name/ExampleOtherInterface.hpp"

// Namespaces scopes should be one line each with "cuddled" braces.
namespace dart {
namespace example {

// Use the following macro (defined in dart/common/SmartPointer.hpp) to declare
// STL "smart" pointers. Pointers should be declared ahead of the class so
// that the class itself can use the pointers.
DART_COMMON_MAKE_SHARED_WEAK(ExampleClass)

/// A required Doxygen comment description for this class. This can be extended
/// to include various useful details about the class, and can use the standard
/// Doxygen tag set to refer to other classes or documentation. It should use
/// the '///' style of block comment.
class ExampleClass : public ExampleInterface, public ExampleOtherInterface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // Many classes that require Eigen will also need this macro

  /// Required brief description of constructor. This will often be as simple as:
  /// "Creates an instance of ExampleClass."
  ///
  /// \param[in] foo This is an example parameter description.
  /// \param[in] bar This is a longer example parameter description that needs
  /// to wrap across multiple lines.
  ExampleClass(std::unique_ptr<util::RNG> foo,
               const Eigen::Isometry3d& bar = Eigen::Isometry3d::Identity());

  ExampleClass(const ExampleClass& other);
  ExampleClass(ExampleClass&& other);

  ExampleClass& operator=(const ExampleClass& other);
  ExampleClass& operator=(ExampleClass&& other);

  // If a class should be non-copyable, it should explicitly delete the following:
  ExampleClass(const ExampleClass&) = delete;
  ExampleClass(ExampleClass&& other) = delete;
  ExampleClass& operator=(const ExampleClass& other) = delete;
  ExampleClass& operator=(ExampleClass&& other) = delete;
  
  // Classes should explicitly declare a default virtual destructor
  // if they do not declare one (unless marking a class as final).
  virtual ~ExampleClass() = default; 

  // Documentation inherited.  <-- Use this comment to indicate that the docstring of the interface method applies
  int exampleInterfaceFunction() const override;  // <-- Always explicitly `override` interface functions without `virtual`

  /// Required brief description of method.
  /// \note If a method has output parameters, they should be the last
  /// arguments.
  ///
  /// \param[in] a A description of a
  /// \param[in] b A description of b
  /// \param[out] out A description of out
  int exampleMethod(int a, int b, int* out) const;

private:
  std::unique_ptr<util::RNG> mExampleMember; // Member variables are prefixed with "m"
};

} // namespace example
} // namespace dart

// In certain cases, such as heavily templated code, implementations must be included
// in headers. In this case, a "detail" header should be created in the "./detail"
// subdirectory with the same name as the main header file, but an "-impl" suffix.
// Private declarations in this header can use a "detail" sub-namespace.
#include "dart/component_name/detail/ExampleClass-impl.hpp"

#endif // DART_EXAMPLE_EXAMPLECLASS_HPP_
```

#### Source Style

C++ sources should be contained in a subdirectory of `src/` that matches their
namespace, with the extension `.cpp`.

* Use **two-space** indentation
* Use **camelCase** function names
* Use **PascalCase** class names
* No "cuddled" braces!

```c++
// Includes should be at the top of the file.
// The first include in a class source file should be the matching `.hpp` header file.
#include "dart/example/ExampleClass.hpp"

#include <stl_headers>
#include <library/headers.hpp>
#include "dart/example/OtherHeaders.hpp"

// Namespace nesting is preferred to "using namespace" directives.
// Namespaces scopes should be one line each with "cuddled" braces.
namespace dart {
namespace example {

// Each function is separated by an 80 column line of "=" characters.
//==============================================================================
int ExampleClass::exampleInterfaceFunction() const
{
  if (mExampleMember)
    return 3;

  return -1;
}

//==============================================================================
int ExampleClass::exampleMethod(int a, int b, int* out) const
{
  int result = a + b;
  if (out)
    *out = result;
  return result;
}

} // namespace example
} // namespace dart
```

#### Smart Pointers

> These guidelines are based on [this article][sutter-smart-pointers]. Consider
> looking at the full article for the details.

[sutter-smart-pointers]: https://herbsutter.com/2013/06/05/gotw-91-solution-smart-pointer-parameters/

* General Rules
  * Use a by-value `std::shared_ptr` as a parameter if the function surely takes
    the shared ownership.
  * Use a `const std::shared_ptr&` as a parameter only if you're not sure
    whether or not you'll take a copy and share ownership.
  * Use a non-const `std::shared_ptr&` parameter only to modify the
    `std::shared_ptr`.
  * Use `std::unique_ptr` anytime you want to use a `std::shared_ptr` but don't
    need to share ownership.
  * Otherwise use `Object*` instead, or `Object&` if not nullable.

#### Autoformatting using ClangFormat

You can automatically format all DART code
using [ClangFormat](https://clang.llvm.org/docs/ClangFormat.html) through
CMake. Make sure `clang-format 3.8` is installed.

##### Using CMake

```bash
$ cd to/dart/root/
$ mkdir build
$ cd build
$ make check-format # to check the code without formatting
$ make format       # to format the code
```

### CMake Style

* Use **two-space** indentation
* Use **lowercase** function names
* Use **all-caps** variables except when referring to target names
* Use `target_VARIABLE` when naming target-specific variables
* **ALWAYS** quote singleton variables (e.g. `"${MY_VARIABLE}"` but not `${MY_LIST_VARIABLE}`) 

```cmake
cmake_minimum_required(VERSION 2.8.11)  # Always declare a minimum version in the top-level CMakeLists.txt.

project(dart)  # Only declare a project name in the top-level CMakeLists.txt.

# Put in comments liberally!  CMake is complicated!
if(SOME_VARIABLE)
  message(STATUS "Variable was set to '${SOME_VARIABLE}'.")
endif()

# Prefer using LIST functions to SET functions when working with list variables
list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")   # ALWAYS quote around singleton variables
list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

set(MY_INCLUDE_DIR include)  # Use all-caps for variables.

find_package(SomeLibrary REQUIRED)  # Use REQUIRED keyword when appropriate.

# For now, `include_directories` is necessary, but later we will switch to `target_include_directories`.
include_directories(
  "${MY_INCUDE_DIR}"  # This should be quoted.
)

# Complex commands should be split into one line for each semantic group (with two-space indentation).
# It is OK to put a target or output on the first line.
include_directories(SYSTEM 
  ${SomeLibrary_INCLUDE_DIRS}  # This should NOT be quoted, because it is a list.
)

add_library("${PROJECT_NAME}" SHARED  # This target name is generated from a variable, so it should be quoted.
  src/MySourceCode.cpp
)

# Always prefer `target_link_directories` to `link_directories` or other global functions.
target_link_libraries("${PROJECT_NAME}"
  ${SomeLibrary_LIBRARIES}
)

# Tests will typically be added to the end of the time from a `tests` subdirectory like the following.
enable_testing()
add_subdirectory(tests)
```
