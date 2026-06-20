# Differentiable Simulation — Dev Task

Implementation tracking for **PLAN-110**: opt-in analytic differentiable
simulation in the DART 7 `World` (the Nimble method, arXiv:2103.16021).

- Plan: [`docs/plans/110-differentiable-simulation.md`](../../plans/110-differentiable-simulation.md)
- Design (source of truth): [`docs/design/differentiable_simulation.md`](../../design/differentiable_simulation.md)
- Gap audit + cross-engine survey: [`docs/plans/110-differentiable-simulation/nimble-gap-audit.md`](../../plans/110-differentiable-simulation/nimble-gap-audit.md)
- Roadmap (slice detail): [`02-roadmap.md`](02-roadmap.md)

## Current Status — all workstreams implemented & verified (first slices)

- [x] **WS0 (prereq, PLAN-080 WS4)**: opt-in boxed-LCP rigid-body contact path +
      `BoxedLcpContactSnapshot{A,b,lo,hi,findex,f,J}` (normal contact + Coulomb
      friction).
- [x] **WS1**: build-time + runtime opt-in seam (`DART_BUILD_DIFF` +
      `WorldOptions.differentiable`); contact-free `state_jacobian`/
      `control_jacobian` for ALL joint types (fixed/revolute/prismatic/screw/
      universal/planar/ball-SO(3)/free-SE(3)); FD checker; zero-cost parity.
- [x] **WS2**: analytic contact gradient (clamping `A_CC⁻¹`) — normal contact,
      Coulomb friction (upper-bound mapping), rotational/off-COM, multi-contact.
- [x] **WS3**: `applyStepVjp` + dartpy `sx.diff.timestep` (`torch.autograd.Function`,
      lazy torch) + `sx.World(differentiable=True)` + `get_step_derivatives`.
- [x] **Public facade**: `World::getStepDerivatives()` contact-aware; out-of-scope
      throws (never a wrong gradient).
- [x] **Rollout**: framework-neutral `diff::rollout` + `sx.diff.rollout` (chained
      VJP, FD-checked); torch rollout via `timestep`-chaining.
- [x] **WS4**: parameter derivatives — MASS, INERTIA (3 diag), FRICTION
      (COM excluded: no effect on the rigid-body step → identically-zero gradient).
- [x] **WS5**: refinement modes — `ContactGradientMode{ANALYTIC, COMPLEMENTARITY_AWARE,
PRE_CONTACT_SURROGATE}`; 5a elastic/restitution FD-verified; 5b/5c documented
      non-true-gradient opt-ins.

The WS1–WS5 first-slice surface landed on `main` via PR #2761. Experimental
suite evidence from that path: ON **30/30**, default OFF **23/23**. All
contact/parameter/joint gradients were FD-of-step validated (~1e-11; rollout VJP
<1e-4 over a chain). `differentiable=false` is bitwise zero-cost. dartpy
`test-py` was green (12 diff tests pass / 1 torch-skipped).
`check-api-boundaries`, `lint`, and stub-sync were green; the default build was
untouched (`#ifdef DART_HAS_DIFF`), and the cache remained OFF. Current active
work is hardening/examples/promotion plus the Dojo-style evaluation tracked in
PLAN-110, not landing the original WS1–WS5 surface.

### Deferred / honest follow-ups (none block the plan's workstreams)

- The end-to-end **torch autograd** gradient test is skipped (torch not installed
  in-env); `pip install torch` to run it. Non-torch numerics fully verified.
- CENTER_OF_MASS parameter, articulated multibody-link contact gradients, and
  contact-point-migration scenes are out of scope (documented; throw or noted).
- A benign `LCP internal error, s<=0` `DART_WARN_ONCE` in the static-friction
  Dantzig degenerate-pivot path (shared `dart/math/lcp`) — recovers correctly,
  all assertions pass; harden later (touches shared LCP code, warrants its own PR).
- Binary save/load preserves the World-level differentiability/contact policy
  flags, and replay restores registered differentiable parameters. Parameter
  registrations are not yet serialized in the binary format; promotion must
  either serialize them or document them as a replay-only/runtime registration.
- A standalone trajectory-optimization example program. The
  system-identification standalone example now ships as
  `python/examples/diff_system_identification.py` (recovers an unknown mass via
  `add_differentiable_parameter` + `get_step_derivatives`); GUI trajopt scenes
  already ship, so only a standalone trajopt script remains.

## Goal

A researcher enables one flag (`differentiable=True`) and gets exact, fast,
finite-difference-verified gradients of a physics step w.r.t. state, control, and
physical parameters — via a typed C++ surface and an optional PyTorch bridge —
with zero cost when the flag is off.

## Non-Goals (for early phases)

- GPU / accelerator differentiable kernels (deferred; coordinate with PLAN-030).
- Batched differentiable rollouts beyond a documented leading-dimension contract.
- Differentiable deformable / IPC contact (coordinate with PLAN-081).
- Always-on differentiability (DART is deliberately opt-in).
- Whole-pipeline autodiff-scalar templating as the contact-gradient mechanism.
- A public third-party custom-gradient plugin API.

## Key Decisions

- **Analytic LCP gradients, not autodiff-through-the-solver** — exact within a
  contact mode; DART already owns the inputs. (design D1)
- **Opt-in, default off, zero cost off** — matches the stated overhead
  requirement and Newton/Genesis. (design D2)
- **Framework-neutral C++ core + thin PyTorch `autograd.Function` in `sx.diff`** —
  lowest-friction Python UX without a core tensor dependency. (design D4)
- **Expose explicit Jacobians _and_ an efficient VJP** — serves optimal control
  and learning; beats reverse-only tape engines. (design D5)
- **Finite-difference checking is the correctness gate**; mode-switch
  subgradients and elastic approximation are documented; saddle-escape and
  pre-contact surrogates are opt-in only. (design D6)
- **It is a rigid-body-solver-internal reverse pass** (distinct from the
  cross-domain coupler reverse pass), not a new engine or a parallel `DiffWorld`.
  (design D7)

## Immediate Next Steps

All PLAN-110 workstreams (WS1–WS5) + the PLAN-080-WS4 contact prerequisite are
implemented and verified (see Current Status). What remains is hardening,
examples, promotion, and landing:

1. **Run the torch path**: `pip install torch` in the pixi env and unskip the
   `sx.diff.timestep` autograd gradient test (the only test skipped in-env).
2. **Worked examples**: the system-identification standalone example (recovers
   mass via `add_differentiable_parameter`) now ships as
   `python/examples/diff_system_identification.py`. A standalone
   trajectory-optimization example (control via `sx.diff.timestep` rollout)
   remains a follow-up (three GUI trajopt scenes already ship) — both are part
   of the DART 8 promotion contract.
3. **Promotion prep**: when promoting, document the contact-point-migration and
   make/break-instant subgradient limits, and the CENTER_OF_MASS exclusion.
4. **Robustness**: harden the static-friction Dantzig degenerate-pivot
   `s<=0` warning (shared `dart/math/lcp` — its own PR).
5. **Dojo de-risking spike**: use the PLAN-110 Dojo gap audit before any public
   API, dependency, or runtime-path promise for a second differentiable solver
   family.
