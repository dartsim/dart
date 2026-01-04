---
globs: ["dart/common/**/*.hpp", "dart/common/**/*.cpp"]
description: "C++ template metaprogramming patterns for Aspect system"
alwaysApply: false
---

## Aspect System Template Guidelines

### CRTP Pattern Usage

```cpp
// Correct Aspect pattern
template <class Base>
class CollisionAspect : public Base {
public:
  using ThisType = CollisionAspect<Base>;

  // Aspect-specific methods
  void enableCollision(bool enable) { /* ... */ }

protected:
  // Always forward to Base
  template<class... Args>
  CollisionAspect(Args&&... args) : Base(std::forward<Args>(args)...) {}
};
```

### Template Metaprogramming Rules

1. **Never use dynamic_cast** - Aspect system provides compile-time type safety
2. **Preserve perfect forwarding** - All constructors must forward arguments
3. **Use type traits** - `std::is_base_of`, `std::enable_if_t` for SFINAE

### Common Patterns to Avoid

❌ **WRONG**: Mixing Aspect inheritance with traditional inheritance

```cpp
class BadDesign : public CollisionAspect<Skeleton>, public SomeOtherClass {
  // This breaks the Aspect chain
};
```

✅ **CORRECT**: Aspect composition

```cpp
using MyWorld = CollisionAspect<AspectizedWorld>;
```

### When Editing Aspect Classes

- Always run `pixi run test` - template compile errors can be subtle
- Check Python binding updates - Aspect changes often require binding regeneration
- Consult documentation in `docs/onboarding/architecture/aspect-system.md`

### Aspect System Location

- **Core**: `dart/common/Aspect.hpp`
- **Examples**: Look at existing aspects in `dart/collision/`, `dart/dynamics/`
- **Testing**: `tests/unit/common/aspect/`

### Performance Considerations

- Aspect composition has zero runtime overhead (compile-time only)
- Template instantiation can increase compile time for complex hierarchies
- Use `static_assert` for compile-time validation when possible

### DART-Specific Patterns

```cpp
// Standard DART aspect pattern
template <typename Base>
class MyFeature : public Base {
public:
  using Base::Base;  // Inherit constructors

  // New functionality
  void enableMyFeature(bool flag) { mMyFeatureEnabled = flag; }

private:
  bool mMyFeatureEnabled{false};
};

// Usage
using EnhancedWorld = MyFeature<CollisionAspect<World>>;
```
