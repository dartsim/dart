# Renderer Real-Time Loop And Large-Scene Scalability

Durable design rationale and follow-up roadmap for the DART Filament viewer's
real-time behavior and its performance at large scene scale. Roadmap priority
and status live in `docs/plans/dashboard.md`; user-facing usage lives in
`docs/onboarding/gui-rendering.md`. This doc records the investigation findings
and the recommended direction, not active task state.

## Real-Time Loop (sim stepping vs rendering)

### Current state

`dart/gui/detail/simulation_stepper.cpp` already implements the canonical
fixed-`dt` accumulator with a spiral-of-death clamp: it accumulates wall time,
clamps the accumulator to `dt * 64`, and runs `0..64` `world->step()` calls per
rendered frame. What was missing for a smooth, clearly-real-time feel:

- a **real-time-factor readout** so users can tell whether playback keeps up —
  now provided in the perf HUD (`real-time %`, EMA of simulated-vs-wall time);
- **state interpolation** between physics states for smooth display;
- optional **render/sim decoupling** so a slow render frame never stalls physics.

### What other simulators do (survey)

The same pattern recurs everywhere: fixed physics `dt`, an accumulator running
`0..N` steps per rendered frame, a clamp to avoid the spiral of death, render
decoupled from sim (often a separate physics thread), and state interpolation
for smoothness; robotics stacks also surface a real-time factor.

- Glenn Fiedler, "Fix Your Timestep!" — accumulator + `dt`, clamp `frameTime`,
  interpolate with `alpha = accumulator/dt`. https://gafferongames.com/post/fix_your_timestep/
- MuJoCo `simulate` — physics on a background thread with sync points; sleeps to
  hit real time. https://mujoco.readthedocs.io/en/stable/programming/visualization.html
- Gazebo — `max_step_size` x `real_time_update_rate` = real-time factor.
  https://classic.gazebosim.org/tutorials?tut=physics_params
- Isaac Sim — separate `physics_dt` and `rendering_dt`, multiple physics steps
  per frame. https://docs.isaacsim.omniverse.nvidia.com/4.5.0/physics/simulation_fundamentals.html
- PyBullet — fixed `1/240` step; GUI server can auto-step in real time on its own
  thread. https://github.com/bulletphysics/bullet3
- Drake — `set_target_realtime_rate`; visualizer publishes on a fixed period;
  Meshcat shows a real-time-rate readout. https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_meshcat.html
- Unity `FixedUpdate` (fixed dt, `maximumDeltaTime` clamp, Rigidbody
  interpolation); Unreal/PhysX sub-stepping with a max-substeps clamp + async
  physics thread.

### Recommendation (follow-up)

1. **State interpolation** — keep previous/current poses and render the blend by
   `alpha = accumulator/dt`. This is the single biggest fix for perceived
   lag/stutter at a fixed sim rate. Requires exposing the leftover accumulator
   and a prev-pose snapshot to the extraction/render path.
2. **Render/sim decoupling** — optionally move `world->step()` to a background
   thread with a double-buffered state the render thread interpolates, so a slow
   render never blocks physics (MuJoCo/Bullet/Unreal model).
3. **Speed controls** — pause / 0.5x / 1x / 2x and frame-skip by scaling the
   wall time fed into the accumulator (MuJoCo viewer model).

The RTF readout (done) makes it observable; interpolation + decoupling make it
smooth. These are deferred to a focused follow-up because they touch the
extraction/render data path and frame ordering.

## Large-Scene Scalability

### Finding: do not build a DART-side scene graph

Filament already frustum-culls every renderable by AABB automatically (culling
is on by default, and DART supplies object-space bounds), so an application-side
BVH/octree for culling would duplicate the GPU-facing work while adding per-frame
maintenance cost. Spatial trees pay off mainly for mostly-static, many-object
scenes or multi-query-per-frame cases (e.g. many shadow lights) — not DART's
case where transforms change every step.

- Filament culling default + AABB: https://github.com/google/filament/blob/main/filament/include/filament/RenderableManager.h
- OGRE/RViz flat-vs-octree (octree helps CPU broad-phase, not GPU draw cost):
  https://wiki.ogre3d.org/SceneManagersFAQ

### Recommendation (follow-up), highest ROI first

1. **GPU instancing for identical shapes.** DART's worst case (thousands of
   identical spheres/boxes/capsules, each currently its own mesh + material +
   draw call) is the instancing sweet spot. Group renderables by shape signature
   and render each group via Filament `RenderableManager::Builder::instances()`
   with a per-instance transform buffer, collapsing N draw calls to one. Mind the
   shared-bounding-box caveat (size the group AABB to enclose all instances).
   Filament instancing: https://github.com/google/filament/discussions/6739 ;
   Isaac Sim "scenegraph instancing":
   https://docs.isaacsim.omniverse.nvidia.com/latest/reference_material/sim_performance_optimization_handbook.html
2. **Fix the per-frame CPU sync.** `updateSceneRenderablesFromDescriptors`
   (`dart/gui/detail/renderable_sync.cpp`) previously matched descriptors to
   renderables with an O(N^2) `find_if`; this is now an id-keyed map lookup
   (O(N), landed). Remaining follow-up: skip `setTransform`/material updates when
   the transform and version are unchanged (extend the version-based skipping the
   geometry cache already uses) so static/sleeping bodies cost nothing per frame.
3. **Defer LOD.** Filament has no built-in LOD; fill-rate is rarely DART's
   bottleneck (draw-call count and CPU sync dominate at scale). Use
   `View::setVisibleLayers` for category toggles (visual/collision/debug), not as
   a performance scheme.

## Already Landed (this initiative)

- Per-shape geometry extraction cache keyed by stable shape id
  (`dart/gui/renderable.cpp`, `RenderableExtractor`).
- Live perf HUD with CPU/GPU frame time, FPS, per-phase breakdown, fixed-scale
  history plots, and a real-time-factor readout (`dart/gui/detail/perf_hud.cpp`).
- Runtime rendering-backend selection (`render_context.cpp`).
- O(N) id-keyed descriptor-to-renderable sync, replacing the O(N^2) `find_if`
  (`dart/gui/detail/renderable_sync.cpp`).
