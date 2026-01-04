# Agent Guidelines for DART Collision Module

This module provides collision detection capabilities using multiple backends (Bullet, ODE, FCL). Agents working here should understand performance-critical collision algorithms and maintain compatibility with different physics engines.

## Read First

- **Base Guidelines**: Read `/AGENTS.md` for overall DART patterns
- **Aspect System**: This module uses Aspect for runtime collision features
- **Multi-Backend**: Bullet, ODE, FCL backends have different performance characteristics
- **Testing**: Collision tests are in `tests/unit/collision/` and `tests/integration/collision/`

## Module-Specific Commands

```bash
# Build collision with specific backends
pixi run build --config DART_BUILD_COLLISION_BULLET=ON
pixi run build --config DART_BUILD_COLLISION_ODE=ON
pixi run build --config DART_BUILD_COLLISION_FCL=ON

# Run collision-specific tests
pixi run test --filter collision
pixi run test --filter integration/collision

# Performance benchmarking
pixi run benchmark --filter collision

# Debug collision issues
pixi run -e debug build --config DART_BUILD_COLLISION_BULLET=ON
```

## Critical Patterns

### 1. Backend Selection

```cpp
// Correct: Use factory pattern for backend selection
auto detector = CollisionDetector::create(CollisionDetector::Type::FCL);

// Wrong: Hardcode backend dependencies
auto detector = std::make_unique<FCLCollisionDetector>(); // Avoid
```

### 2. Collision Object Management

```cpp
// Correct: Share collision geometry when possible
auto collObj = std::make_shared<CollisionObject>(shape);
detector->registerObject(collObj);

// Wrong: Create duplicate geometries
auto collObj1 = std::make_unique<CollisionObject>(shape);
auto collObj2 = std::make_unique<CollisionObject>(shape); // Duplicate shape
```

### 3. Performance Considerations

- **Broad-phase filtering**: Use `CollisionFilter` to cull obvious non-collisions
- **Spatial partitioning**: Let backends handle spatial data structures
- **Cache results**: Collision results are valid until geometry changes
- **Update strategically**: Only update collision objects when necessary

### 4. Backend-Specific Behavior

Each backend has different characteristics:

**Bullet**:

- Best for: Complex convex shapes, continuous collision detection
- Limitations: Less precise for thin meshes
- Performance: Good for moderate scene complexity

**ODE**:

- Best for: Simple shapes, stable simulation
- Limitations: Limited geometric primitive support
- Performance: Fast for simple scenes

**FCL**:

- Best for: High-precision collision, complex meshes
- Limitations: Slower for very large scenes
- Performance: Excellent with spatial culling

## When to Modify This Module

### Add New Collision Backend

1. **Implement interface** in `CollisionDetector` inheritance hierarchy
2. **Add factory method** in `CollisionDetector::create()`
3. **Update CMake** to include new backend
4. **Add comprehensive tests** in `tests/unit/collision/`
5. **Update documentation** with backend characteristics

### Optimize Performance

1. **Profile with benchmarks**: `pixi run benchmark --filter collision`
2. **Focus on hotspots**: Broad-phase filtering, narrow-phase algorithms
3. **Maintain API compatibility**: Don't change public interfaces
4. **Test all backends**: Ensure optimizations work across implementations

### Fix Collision Accuracy Issues

1. **Reproduce consistently**: Create minimal test case
2. **Check backend differences**: Compare Bullet/ODE/FCL results
3. **Verify geometry**: Ensure shapes are correctly defined
4. **Test edge cases**: Coincident faces, thin objects, etc.

## Common Pitfalls to Avoid

### ❌ Performance Anti-Patterns

```cpp
// Wrong: Updating collision objects every frame
for (auto& obj : objects) {
    obj->updateTransform(); // Expensive if unchanged
}

// Correct: Track updates and update only when needed
for (auto& obj : objects) {
    if (obj->transformChanged()) {
        obj->updateTransform();
    }
}
```

### ❌ Memory Management Errors

```cpp
// Wrong: Stale pointers to collision objects
CollisionObject* ptr = detector->getCollisionObject(id);
detector->removeObject(id);
ptr->updateShape(); // dangling pointer!

// Correct: Use shared_ptr or get copy before removal
auto obj = detector->getCollisionObject(id);
detector->removeObject(id);
if (obj) obj->updateShape(); // Safe
```

### ❌ Backend Assumptions

```cpp
// Wrong: Assume specific backend behavior
auto detector = CollisionDetector::get();
dynamic_cast<FCLCollisionDetector*>(detector)->specialMethod(); // Unsafe

// Correct: Check backend type or use common interface
if (detector->getType() == CollisionDetector::Type::FCL) {
    // Safe to use FCL-specific features
}
```

## Testing Requirements

### Before Submitting Changes:

1. **Unit Tests**: `pixi run test --filter unit/collision`
2. **Integration Tests**: `pixi run test --filter integration/collision`
3. **Performance Tests**: `pixi run benchmark --filter collision`
4. **Memory Tests**: Check for leaks in collision scenarios
5. **Backend Tests**: Verify changes work with all enabled backends

### Test Categories to Consider:

- **Accuracy**: Known collision scenarios
- **Performance**: Timing with scene complexity scaling
- **Memory**: Resource usage with many objects
- **Edge Cases**: Degenerate geometries, coincident objects
- **Multi-threading**: Concurrent collision queries

## Integration Points

### With Dynamics Module:

- **Contact forces**: `Contact` objects feed into constraint solver
- **Body relationships**: `CollisionObject` maps to `BodyNode`
- **Update cycles**: Synchronize with physics timestep

### With GUI Module:

- **Visualization**: Debug rendering of collision geometry
- **Interactive debugging**: Select/highlight colliding objects
- **Performance profiling**: Visualize collision hotspots

### With Python Bindings:

- **API exposure**: Make collision functionality available in Python
- **Testing**: Python tests should mirror C++ test coverage
- **Examples**: Provide Python usage examples

## Getting Help

- **Architecture decisions**: Use Oracle agent for collision algorithm design
- **Performance optimization**: Use Librarian agent to research collision techniques
- **Cross-backend compatibility**: Use Explore agent to find existing patterns
- **Debugging complex issues**: Use multi-model orchestration (Oracle + Librarian)

## Files to Understand First

1. `CollisionDetector.hpp` - Main interface and factory pattern
2. `CollisionObject.hpp` - Collision geometry management
3. `Contact.hpp` - Contact representation for dynamics
4. `CollisionFilter.hpp` - Performance optimization for broad-phase
5. `Fwd.hpp` - Forward declarations and type aliases

---

_Remember: Collision detection is performance-critical. Always benchmark optimizations and test across all backends._
