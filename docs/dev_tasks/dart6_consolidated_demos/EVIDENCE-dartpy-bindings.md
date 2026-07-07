# Evidence: dartpy Binding Coverage for py-demos (release-6.20)

Collected 2026-07-04 by recon agent (file:line evidence in agent transcript).

## What works from pure Python (bound + proven in existing examples)

- `ImGuiViewer` instantiation; `Viewer.addWorldNode/removeWorldNode/
  setWorldNodeActive` (all overloads) → scene switching fully supported.
- `WorldNode`/`RealTimeWorldNode` trampolines: `customPreStep/PostStep/
  PreRefresh/PostRefresh` overridable in Python (hello_world_gui proves it);
  RTF getters bound.
- `GUIEventHandler` trampoline for keyboard input (atlas_puppet pattern);
  full key/event enums. BUT `GUIEventAdapter` binds only `getEventType()` +
  `getKey()` — no mouse coords/buttons/modifiers.
- `InteractiveFrame` + all 5 `enableDragAndDrop` overloads; `BodyNode.
  addExtForce`; `GridVisual`; ShadowMap/setShadowTechnique; `captureScreen`,
  `setUpViewInWindow`, `setCameraHomePosition`, `record`.

## What is NOT available (and directive says: no new bindings for demos)

- **ImGui entirely unusable from Python**: `ImGuiWidget` has no trampoline/
  init (pure-virtual `render()` unreachable), no raw `imgui` draw-function
  module, `setGuiScale` unbound. → py-demos navigator must be
  keyboard-driven + `addInstructionText` overlay + terminal catalog listing.
- `DefaultEventHandler` picks / `MouseEventHandler`: unbound → no custom
  mouse picking (built-in DnD still works).
- `setThreadingModel`: unbound (Viewer.run() default loop is fine).
- `SupportPolygonVisual`: unbound — and `python/examples/atlas_puppet/
  main.py:552` already calls it → **that example is broken today**
  (AttributeError). Pre-existing bug, independent of this task; candidate
  small fix: bind the class OR drop the call. Track as side-fix, decide at
  Phase 4.

## Wider binding gaps found during Phase 4 implementation (2026-07-05)

The atlas_puppet port surfaced that the unbound surface is broader than the
first audit captured (which listed only `SupportPolygonVisual`):

- **`dart::dynamics::EndEffector` is entirely unbound** — no
  `BodyNode::createEndEffector`, no class. Since `BalanceConstraint` derives
  its support polygon from active EndEffectors, it is unusable from Python.
  Skeleton-level `WholeBodyIK` also has no Python creation path.
- Also unbound (worked around with bound equivalents in the py port):
  `createShapeNodeWith<>`, generic `createJointAndBodyNodePair(props)`,
  `WeldJoint::Properties`, `dart::math::eulerToMatrix`.
- Consequence for py atlas_puppet: reworked to attach whole-body IK directly
  to the four hand/foot `BodyNode`s via `BodyNode::getIK(true)` (bound),
  keeping draggable targets + F1-F4 constrain/release, but **dropping**
  foot-support toggle, `BalanceConstraint`, and `RelaxedPosture`.
- `--shot` capture race: `SaveScreen` writes the PNG inside an OSG draw
  callback that can run on a separate draw thread; dartpy has no
  `setThreadingModel` binding to force single-threaded, so the py runner
  polls for the output file (up to ~2s) rather than trusting one extra
  `frame()`. Local-only debug capture, not CI.

These are candidate follow-up binding additions (out of scope here: the task
forbids new bindings for demos), highest-leverage being EndEffector +
mouse accessors on GUIEventAdapter.

## Phase 4 design consequence

py-demos = dartpy runner with: scene registry (same ids/categories as C++),
RealTimeWorldNode-subclass scenes with pre-step controllers, keyboard
navigator (number/±keys cycle scenes, world-node swap via
setWorldNodeActive), instruction-text overlay showing catalog + active
scene + keys, drag/force interactions where bound, `--shot` capture mode
reusing `captureScreen`. No ImGui panels in Python (best-effort per task
directive).
