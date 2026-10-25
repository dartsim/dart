Section: Build

  * Allocators and other classes declared with `DART_STRING_TYPE` no longer
    report empty type names, which also made `is<T>()` match the wrong type,
    when DART is built as C++20 and the code that uses it as C++17.
    [#3953](https://github.com/dartsim/dart/pull/3953)
