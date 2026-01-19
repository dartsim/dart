# Resume: Experimental Collision Module

## Last Session Summary

Completed comparative benchmarks against FCL, Bullet, and ODE. **Results are excellent:**

| Shape Pair      | Experimental | Best Alternative | Speedup |
| --------------- | ------------ | ---------------- | ------- |
| Sphere-Sphere   | **41 ns**    | 411 ns (Bullet)  | **10x** |
| Box-Box         | **210 ns**   | 1094 ns (Bullet) | **5x**  |
| Capsule-Capsule | **41 ns**    | 242 ns (FCL)     | **6x**  |
| Distance Sphere | **7 ns**     | 288 ns (FCL)     | **40x** |

Accuracy verification: **PASSED** (sphere-sphere, box-box, distance queries match FCL within tolerance)

## ✅ Performance Goal: ACHIEVED

The experimental module demonstrates **5-40x speedup** over existing backends in narrow-phase collision detection while maintaining accuracy parity. The brute-force broad-phase is O(N²) which is slower than FCL/Bullet at large N, but this is intentional - narrow-phase optimization was the priority.

## Current Branch

`feature/new_coll` — has uncommitted changes (bm_comparative.cpp)

## Immediate Next Step

**Commit and push comparative benchmark results**, then consider:

1. Add raycast support
2. Add visual verification tool
3. Optimize broad-phase with spatial data structures (BVH, spatial hash)

## Context That Would Be Lost

- **Performance validated**: 5-40x faster than FCL/Bullet/ODE in narrow-phase
- **Accuracy verified**: Matches FCL results within tolerance
- **213 tests passing**: All primitive shapes complete
- **N-body scaling**: O(N²) brute-force - acceptable for now, optimize later

## How to Resume

```bash
git checkout feature/new_coll
git status && git log -5 --oneline
```

## Key Constraints

1. **Use `Aabb` not `AABB`** (PascalCase for abbreviations)
2. **BSD copyright headers only**
3. **snake_case for file names** in experimental modules
4. **Determinism required** - bit-exact results
