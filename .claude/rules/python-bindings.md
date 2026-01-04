---
globs: ["dart/**/*.hpp", "dart/**/*.cpp"]
description: "Python binding update requirements for C++ API changes"
alwaysApply: false
---

## Python Binding Update Requirements

### When C++ Changes Require Python Updates

Any public API change in `dart/` requires corresponding updates to `python/dartpy/`. This includes:

1. **New public classes/structs**
2. **Modified public interfaces**
3. **Changed template parameters**
4. **Updated enum values**
5. **Modified constructor signatures**
6. **New public methods**

### Python Binding Locations

- **Main bindings**: `python/dartpy/src/`
- **CMake configuration**: `python/dartpy/CMakeLists.txt`
- **Testing**: `tests/python/`
- **Documentation updates**: `docs/tutorials/python/`

### Binding Update Workflow

1. **After C++ API changes**:

   ```bash
   # Build Python bindings
   pixi run build --target dartpy

   # Run Python tests
   pixi run test --filter python
   ```

2. **Template metaprogramming changes**:
   - Aspect system changes often require binding regeneration
   - Check `python/dartpy/src/common/` for Aspect-specific bindings
   - Template instantiation may need explicit binding declarations

3. **New classes**:

   ```cpp
   // In python/dartpy/src/[module].cpp
   #include <[new_class].hpp>

   void init_[module](py::module_& m) {
       py::class_<NewClass>(m, "NewClass")
           .def(py::init<>())
           .def("publicMethod", &NewClass::publicMethod);
   }
   ```

### Common Pitfalls to Avoid

❌ **WRONG**: Forgetting to update bindings after API change

```cpp
// Changed this in C++ but forgot Python binding
class Skeleton {
public:
  void setMass(double mass);  // Added parameter
};
```

✅ **CORRECT**: Update both C++ and Python

```cpp
// C++ side
class Skeleton {
public:
  void setMass(double mass, bool preserveMomentum = false);  // Updated signature
};

// Python side
.def("setMass", &Skeleton::setMass,
      py::arg("mass"), py::arg("preserveMomentum") = false)
```

### Testing Requirements

After binding updates, always run:

```bash
# Full Python test suite
pixi run test --filter python

# Integration tests that use Python
pixi run test --filter integration/python

# Documentation build (tests examples)
pixi run docs
```

### Special Cases

**Aspect System**: When modifying Aspect classes, check if the Aspect is exposed in Python:

- `AspectizedEntity` and related classes
- `createAspect()` factory functions
- Type casting utilities for Aspects

**Template Classes**: Some DART templates are explicitly instantiated for Python:

- `Eigen` types are handled by `nanobind-eigen`
- `std::vector`, `std::map` are automatically supported
- Custom templates need explicit binding

### Performance Considerations

- Python bindings have zero overhead for C++-only usage
- Use `py::return_value_policy::reference` for objects that should not be copied
- Prefer `const` references in Python-exposed APIs
- Consider `py::gil_scoped_release` for long-running operations

### Documentation Updates

When updating bindings:

1. Update API documentation if interface changed
2. Update Python tutorials and examples
3. Update `dartpy` README if major changes
4. Add new examples to `examples/python/`

### Validation Checklist

After binding changes:

- [ ] `pixi run test --filter python` passes
- [ ] All new methods accessible from Python
- [ ] Example code still works
- [ ] Documentation builds without errors
- [ ] No memory leaks in Python usage
- [ ] Import statements work correctly
