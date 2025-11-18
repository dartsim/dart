# Code Style Guide

This document describes the code style conventions used in the DART project.

## Table of Contents

- [C++ Style](#c-style)
  - [Header Style](#header-style)
  - [Source Style](#source-style)
  - [Smart Pointers](#smart-pointers)
  - [Macro Conventions](#macro-conventions)
- [Python Style](#python-style-dartpy-bindings)
- [CMake Style](#cmake-style)
- [Auto-formatting](#auto-formatting)

## C++ Style

C++ headers and sources should be contained in the same subdirectory of `dart/` that matches their namespace, with the extension `.hpp` and `.cpp`, respectively.

### C++20 Modern Patterns

DART uses C++20 features to improve code clarity, maintainability, and performance. Use these patterns when appropriate:

**Concepts for Template Constraints:**

```cpp
// Prefer concepts over SFINAE
template <std::floating_point T>
T normalize(T value);

// Custom concepts for domain-specific constraints
template <typename T>
concept UniformIntCompatible = std::integral<T> && !std::same_as<T, bool>;
```

**std::span for Function Parameters:**

```cpp
// Prefer std::span over const std::vector<T>&
void addSkeletons(std::span<const SkeletonPtr> skeletons);

// Accepts vectors, arrays, subranges without temporary allocations
```

**Spaceship Operator for Comparisons:**

```cpp
// Replace 6 operators with 2
auto operator<=>(const Ptr& other) const {
  return std::tie(mT1, mT2) <=> std::tie(other.mT1, other.mT2);
}
bool operator==(const Ptr& other) const = default;
```

**Branch Prediction Attributes:**

```cpp
// Use [[likely]] / [[unlikely]] for hot paths
if (objects.empty()) [[unlikely]]
  return false;

if (!result.isCollision()) [[likely]]
  return;
```

**When NOT to use C++20 features:**

- Avoid `std::format` until Ubuntu 24.04 LTS (GCC 13+)
- Don't replace clear index-based loops with complex range expressions
- Keep SFINAE for Eigen compile-time traits (not proper constexpr)

### File Naming Conventions

DART now maintains two naming schemes in parallel:

- **Legacy modules (`dart/`, `python/dartpy/`, `tests/`, etc.)**: Keep the established **PascalCase** names for headers and sources to avoid churn in the long-lived API surface (e.g., `MyClass.hpp`, `MyClass.cpp`).
- **Next-gen modules (`dart8/`, `tests_dart8/`, and other dart8 dependents)**: Use all **snake_case** file names (e.g., `rigid_body.hpp`, `free_joint.cpp`, `test_multi_body.py`). Build-system files such as `CMakeLists.txt` keep their canonical capitalization.
- **Implementation details**: Continue to use the `-impl` suffix for template implementations, matching the surrounding style (e.g., `MyClass-impl.hpp` in legacy code, `rigid_body-impl.hpp` in dart8).
- **Legacy tests (`tests/`)**: Use the `test_` prefix followed by **PascalCase** (e.g., `test_SkeletonState.cpp`) to align with historic targets while still grouping test binaries by prefix.

**Rationale**: We preserve PascalCase in legacy code to minimise disruption for downstream users while making dart8 code more Pythonic and consistent with modern C++ projects. The `test_` prefix keeps legacy test binaries grouped while allowing PascalCase identifiers, whereas dart8-era tests favour fully snake_case names for readability.

### Quick Reference

- **Indentation**: 2 spaces
- **Functions**: camelCase
- **Classes**: PascalCase
- **Enum members**: PascalCase (e.g., `enum class Mode { Disabled, Enabled };`)
- **Member variables**: Prefixed with `m` (e.g., `mExampleMember`)
- **File extensions**: `.hpp` for headers, `.cpp` for sources
- **File naming**: PascalCase in `dart/` and `python/dartpy/`; snake*case everywhere in `dart8/` and its dependents; legacy `tests/` use `test*` + PascalCase while dart8-era tests stay fully snake_case
- **Header guards**: `DART_NAMESPACE_CLASSNAME_HPP_`
- **Braces**: No "cuddled" braces (except namespaces)
- **Documentation**: Doxygen-style comments (`///`)

### Parameter Naming

- **New/modified code**: Use camelCase names for function parameters (e.g., `void setMass(double mass)`).
- **Legacy code**: Many functions in `dart/dynamics` still use `_param` conventions. Avoid mass renames—update these opportunistically when touching the surrounding code, and prefer camelCase for any new APIs.

### Control Flow Braces

Always wrap the bodies of `if`/`else`, loop, and switch clauses in braces, even
when the body is a single statement. This prevents dangling statements and makes
future edits safer.

```cpp
if (!skeleton) {
  return;
}
```

### Header Style

**Rules:**

- Use **two-space** indentation
- Use **camelCase** function names
- Use **PascalCase** class names
- No "cuddled" braces (except namespaces)!
- Header guards must include the library, namespace, and source file names

**Example:**

```cpp
#ifndef DART_EXAMPLE_EXAMPLECLASS_HPP_
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
DART_COMMON_DECLARE_SHARED_WEAK(ExampleClass)

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
  /// @param[in] foo This is an example parameter description.
  /// @param[in] bar This is a longer example parameter description that needs
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
  /// @note If a method has output parameters, they should be the last
  /// arguments.
  ///
  /// @param[in] a A description of a
  /// @param[in] b A description of b
  /// @param[out] out A description of out
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

### Source Style

**Rules:**

- Use **two-space** indentation
- Use **camelCase** function names
- Use **PascalCase** class names
- No "cuddled" braces!
- Each function is separated by an 80 column line of "=" characters

**Example:**

```cpp
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

### Smart Pointers

These guidelines are based on [Herb Sutter's article](https://herbsutter.com/2013/06/05/gotw-91-solution-smart-pointer-parameters/). Consider looking at the full article for details.

**General Rules:**

- Use a by-value `std::shared_ptr` as a parameter if the function surely takes the shared ownership.
- Use a `const std::shared_ptr&` as a parameter only if you're not sure whether or not you'll take a copy and share ownership.
- Use a non-const `std::shared_ptr&` parameter only to modify the `std::shared_ptr`.
- Use `std::unique_ptr` anytime you want to use a `std::shared_ptr` but don't need to share ownership.
- Otherwise use `Object*` instead, or `Object&` if not nullable.

### Macro Conventions

All macros in DART are prefixed with `DART_` to distinguish them from other identifiers:

- `DART_HAS_<optional_dep>`: Boolean set to true when an optional dependency is detected
- `DART_ENABLE_<optional_feature>`: Boolean set to true if the optional feature should be enabled when requirements are met
- `DART_ENABLED_<optional_feature>`: Boolean set to true if the optional feature is enabled

Use all-caps for all macro names to ensure consistency and to visually distinguish macros from other variables.

## Python Style (dartpy bindings)

### Naming Conventions

The Python bindings use different naming conventions than the C++ code to follow Python community standards:

| Element            | C++ Style  | Python Style | Example (C++)    | Example (Python) |
| ------------------ | ---------- | ------------ | ---------------- | ---------------- |
| Functions          | camelCase  | snake_case   | `isIdentity()`   | `is_identity()`  |
| Classes            | PascalCase | PascalCase   | `MyClass`        | `MyClass`        |
| Variables          | snake_case | snake_case   | `my_variable`    | `my_variable`    |
| Constants          | ALL_CAPS   | ALL_CAPS     | `MY_CONSTANT`    | `MY_CONSTANT`    |
| Namespaces/Modules | N/A        | snake_case   | `dart::dynamics` | `dart.dynamics`  |

### Example

C++ code:

```cpp
auto so3 = SO3();
bool is_identity = so3.isIdentity();
```

Python code:

```python
so3 = SO3()
is_identity = so3.is_identity()
```

### Rationale

Using different naming conventions allows each API to follow its language's community standards:

- **camelCase** is more common for function names in the C++ community
- **snake_case** is more common for function names in the Python community

This makes the code more readable and easier to integrate with other projects in each ecosystem, while maintaining consistency within each language.

## CMake Style Summary

- **Indentation**: 2 spaces
- **Functions**: lowercase
- **Variables**: ALL_CAPS (except target names)
- **Target variables**: `target_VARIABLE` format
- **Quoting**: Always quote singleton variables (`"${MY_VAR}"`), but not lists

See [CONTRIBUTING.md](../../CONTRIBUTING.md#cmake-style) for detailed examples.

## Auto-formatting

Use ClangFormat to automatically format C++ code:

```bash
cd build/
make check-format  # Check without modifying
make format        # Apply formatting
```

## See Also

- [CONTRIBUTING.md](../../CONTRIBUTING.md) - Full style guide with detailed examples
- [building.md](building.md) - Build instructions and CMake configuration
- [python-bindings.md](python-bindings.md) - Python bindings architecture
