# Agent Guidelines for Python Bindings

This module provides Python bindings (`dartpy`) for DART using nanobind. Agents working here must understand C++/Python interop, memory management across language boundaries, and nanobind-specific patterns.

## Read First

- **Base Guidelines**: Read `/AGENTS.md` for overall DART patterns
- **Nanobind Technology**: Uses nanobind for modern Python/C++ bindings
- **C++/Python Sync**: When C++ APIs change, bindings must be updated
- **Testing**: Python tests in `python/tests/`, integration in `tests/python/`

## Module-Specific Commands

```bash
# Build Python bindings specifically
pixi run build --target dartpy
pixi run build --config DART_BUILD_DARTPY=ON

# Run Python-specific tests
pixi run test --filter python
pixi run test --filter integration/python

# Test Python import and functionality
python -c "import dartpy; print(dartpy.__version__)"
python examples/tutorials/hello_world.py

# Debug binding issues
python -m pdb examples/tutorials/hello_world.py
python -c "import dartpy; dartpy.setLogLevel('DEBUG')"

# Install in development mode
pip install -e python/
python -m build --wheel python/
pip install dist/dartpy-*.whl
```

## Critical Architecture Patterns

### 1. Nanobind Binding Structure

```cpp
// Correct: Modern nanobind pattern
#include <nanobind/nanobind.h>
#include <nanobind/eigen/sparse.h>

namespace nb = nanobind;

NB_MODULE(dartpy, m) {
    // Bind class with documentation
    nb::class_<Skeleton>(m, "Skeleton")
        .def(nb::init<>())
        .def("getName", &Skeleton::getName)
        .def("getNumDofs", &Skeleton::getNumDofs)
        .def_prop("positions", &Skeleton::getPositions, &Skeleton::setPositions);
}
```

### 2. Memory Management and Lifetime

```cpp
// Correct: Smart pointer handling
.def("createSkeleton", []() {
    return std::make_shared<Skeleton>();  // Python gets shared ownership
})

// Wrong: Raw pointer return
.def("createSkeleton", []() {
    return new Skeleton();  // Memory leak!
})
```

### 3. Eigen Integration

```cpp
// Correct: Use nanobind's Eigen integration
.def("getPositions", &Skeleton::getPositions)  // Automatically handles Eigen->NumPy

// Wrong: Manual NumPy array creation
.def("getPositions", [](Skeleton* skel) {
    auto positions = skel->getPositions();
    return PyArray_FromData(positions.data());  // Dangerous!
})
```

## When to Modify This Module

### Add New C++ Class to Python

1. **Include header**: Add `#include <new_class.hpp>`
2. **Add binding**: Use nanobind syntax in appropriate module file
3. **Handle constructors**: Bind all relevant constructors
4. **Bind methods**: Include public interface methods
5. **Handle properties**: Use `.def_prop()` for getters/setters
6. **Update tests**: Add Python tests for new functionality

### Update for C++ API Changes

1. **Check breaking changes**: Method signatures, class hierarchy
2. **Update bindings**: Modify nanobind calls to match new API
3. **Handle deprecated methods**: Maintain backward compatibility if needed
4. **Run binding tests**: Ensure all Python functionality works
5. **Update documentation**: Python docstrings and examples

### Add New Python Module

1. **Create source file**: `python/dartpy/src/new_module.cpp`
2. **Add to CMake**: Update `python/dartpy/CMakeLists.txt`
3. **Register module**: Add to `NB_MODULE` declarations
4. **Create tests**: Add test file in `python/tests/`
5. **Update setup**: Include in package configuration

## Common Pitfalls to Avoid

### ❌ Memory Management Errors

```cpp
// Wrong: Returning references to temporary objects
.def("getJoint", [](Skeleton* skel, size_t index) {
    return &skel->getJoint(index);  // Dangling reference!
})

// Correct: Returning shared ownership
.def("getJoint", [](Skeleton* skel, size_t index) {
    return skel->getJoint(index);  // Returns shared_ptr if available
})
```

### ❌ Incorrect Exception Handling

```cpp
// Wrong: C++ exceptions not translated
.def("riskyOperation", [](Object* obj) {
    if (!obj) throw std::runtime_error("Null object");  // Crashes Python
    return obj->doSomething();
})

// Correct: Proper exception translation
.def("riskyOperation", [](Object* obj) {
    if (!obj) nb::raise("Null object provided");
    return obj->doSomething();
})
```

### ❌ Type Conversion Issues

```cpp
// Wrong: Manual type handling
.def("setMatrix", [](Skeleton* skel, py::handle obj) {
    // Complex manual conversion from Python object
})

// Correct: Let nanobind handle conversion
.def("setMatrix", [](Skeleton* skel, const Eigen::MatrixXd& matrix) {
    // nanobind converts NumPy array to Eigen automatically
    skel->setMatrix(matrix);
})
```

## Nanobind-Specific Features

### 1. Automatic Eigen Integration

```cpp
// Eigen dense matrices/vectors
.def("getPosition", &Skeleton::getPosition)  // Returns NumPy array
.def("setPosition", &Skeleton::setPosition)  // Accepts NumPy array

// Eigen sparse matrices (with special header)
.def("getSparseMatrix", &Skeleton::getSparseMatrix)
```

### 2. Enum Handling

```cpp
// Correct enum binding
nb::enum_<Joint::Type>(m, "JointType")
    .value("REVOLUTE", Joint::Type::REVOLUTE)
    .value("PRISMATIC", Joint::Type::PRISMATIC)
    .value("BALL", Joint::Type::BALL);
```

### 3. STL Container Support

```cpp
// Automatic STL to Python list conversion
.def("getJointNames", &Skeleton::getJointNames)  // Returns list of strings
.def("addForce", [](Skeleton* skel, const std::vector<double>& forces) {
    skel->addForces(forces);  // Accepts Python list
})
```

## Testing Requirements

### Before Submitting Changes:

1. **Python Unit Tests**: `pixi run test --filter python`
2. **Integration Tests**: `pixi run test --filter integration/python`
3. **Import Tests**: Verify `import dartpy` works
4. **Memory Tests**: Check for memory leaks with Python usage
5. **Documentation Tests**: Ensure docstrings are accurate

### Test Categories:

- **Basic functionality**: Import, create objects, call methods
- **Type conversions**: NumPy/Eigen handling
- **Error handling**: Python exceptions vs C++ crashes
- **Memory management**: No memory leaks with Python GC
- **Performance**: Acceptable overhead for Python layer

## Integration Points

### With C++ Modules:

- **API synchronization**: Changes in `dart/` require binding updates
- **Template instantiation**: Some templates need explicit Python bindings
- **Aspect system**: Expose Aspect-related functionality

### With Build System:

- **CMake integration**: Proper linking with DART libraries
- **Dependency management**: Handle nanobind and Eigen integration
- **Platform differences**: Windows/Linux/macOS specific issues

### With Package Distribution:

- **PyPI packaging**: Build wheels for distribution
- **Conda packaging**: Integration with conda-forge
- **Development install**: `pip install -e` for development

## Performance Considerations

### 1. Minimize Python/C++ Transitions

```python
# Wrong: Many small calls
for i in range(1000):
    skel.setJointPosition(i, value)  # 1000x C++ calls

# Correct: Batch operations
positions = np.array([values])  # Single array setup
skel.setJointPositions(positions)  # Single C++ call
```

### 2. Use Numpy Arrays Efficiently

```python
# Wrong: Unnecessary copying
positions = skel.getPositions()  # Copies array
positions[0] = new_value
skel.setPositions(positions)  # Copies back

# Correct: Direct manipulation
positions = skel.getPositions()
positions[0] = new_value  # Modifies in-place if supported
```

### 3. Memory Concurrency

```python
# Wrong: Long-running operation blocks GIL
result = skel.computationHeavyOperation()  # Blocks all Python threads

# Correct: Release GIL for long operations
result = skel.computationHeavyOperation nogil=True)  # If supported
```

## Getting Help

- **Nanobind issues**: Use Librarian agent to research nanobind patterns
- **C++/Python interop**: Use Oracle agent for architecture decisions
- **Performance optimization**: Use Explore agent to find efficient patterns
- **Memory issues**: Use multi-model orchestration for debugging

## Files to Understand First

1. `python/dartpy/src/` - Main binding implementation files
2. `python/dartpy/CMakeLists.txt` - Build configuration
3. `python/tests/` - Python-specific test suite
4. `python/examples/` - Usage examples and tutorials
5. `tests/python/` - Integration tests with C++ components

---

_Remember: Python bindings are the primary interface for many DART users. Maintain API consistency and robust error handling._
