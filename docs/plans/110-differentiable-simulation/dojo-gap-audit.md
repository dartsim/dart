# Dojo Solver Gap Audit + Integration Notes

Evidence sidecar for [`PLAN-110`](../110-differentiable-simulation.md), covering
Dojo as an additional differentiable rigid-body solver reference. This does not
replace the active Nimble-style boxed-LCP reverse-pass work; it records the
separate Dojo-style path that must be evaluated before DART commits public API or
runtime behavior.

## Sources

- Howell, Le Cleac'h, Bruedigam, Chen, Sun, Kolter, Schwager, Manchester. "Dojo:
  A Differentiable Physics Engine for Robotics." arXiv:2203.00806.
- Project site: <https://sites.google.com/view/dojo-sim/home>
- Reference implementation: <https://github.com/dojo-sim/Dojo.jl>

## Method Summary

Dojo is a differentiable rigid-body simulator for robotics. The method combines:

- a maximal-coordinate rigid-body representation;
- variational integration for energy and momentum behavior in non-contact
  regimes;
- hard contact/friction as a nonlinear complementarity problem with
  second-order cone constraints and nonlinear friction cones;
- a custom primal-dual interior-point method for the contact solve;
- implicit differentiation of the relaxed interior-point solve, with the central
  path parameter acting as a user-facing smoothness/accuracy knob;
- examples in trajectory optimization, policy optimization, system
  identification, and hardware sim-to-real evaluation.

The Dojo.jl repository is open source and Julia-based, with a Python interface,
but its README says the project has not been actively developed since April 2023. DART should treat it as method evidence and a comparison baseline, not as a
dependency or source to vendor.

## DART Mapping

| Dojo capability                         | DART state today                                      | Initial plan stance                                                                          |
| --------------------------------------- | ----------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| Maximal-coordinate rigid bodies         | Experimental multibodies are articulated/minimal      | GAP; requires a distinct model/state path or a constrained maximal-coordinate layer          |
| Variational integration                 | PLAN-084 tracks a variational-integrator solver       | Coordinate with PLAN-084 instead of duplicating its integration rationale                    |
| Nonlinear hard-contact NCP/SOC friction | Current diff path is boxed LCP with linearized cone   | GAP; separate contact model, solver residuals, and diagnostic surface                        |
| Primal-dual interior-point solver       | Existing LCP path uses a pivoting Dantzig-style solve | GAP; new internal solver, convergence criteria, central-path controls, warm-starting         |
| Implicit gradients through contact      | Nimble-style LCP active-set gradient implemented      | GAP; needs KKT/IFT derivation and finite-difference validation                               |
| Smoothness/accuracy knob                | Contact refinement modes are backward aids only       | GAP; must be labeled as a solver accuracy/smoothness tradeoff, not a true-gradient guarantee |
| Examples and baselines                  | PLAN-110 examples remain Nimble-style                 | GAP; reproduce one Dojo paper example before any parity claim                                |

## DART 7 World Architecture Fit

Dojo-style work belongs to the DART 7 `World` multi-solver architecture as
an alternate rigid-body solver family, not as a separate public engine and not as
a deformable-domain solver. The intended high-level placement is:

- entities remain ordinary DART 7 `World` rigid/articulated entities at the
  public facade;
- a domain-scoped solver option requests the method capabilities
  (variational integration, hard-contact NCP/SOC friction, interior-point solve,
  analytic differentiability) using DART-owned names rather than the Dojo project
  name;
- the `World` maps that option to an internal solver when available, or reports
  an unsupported-capability error when the build or model content cannot support
  it;
- the solver owns any maximal-coordinate or constrained-coordinate runtime state,
  IPM work buffers, central-path tolerance state, KKT factors, and reverse-pass
  cache;
- the public differentiability surface remains the PLAN-110 one:
  state/control/parameter derivative value objects and optional framework
  bridges, not solver rows or Dojo.jl types.

Relative to existing work:

- the boxed-LCP/Nimble-style path is the first implemented differentiable rigid
  path and differentiates the current generalized-coordinate LCP solve;
- the Dojo-style path would be a second rigid path with a different forward solve
  and gradient derivation, so it needs a separate spike and finite-difference
  gate;
- PLAN-084 supplies relevant variational-integration rationale, but Dojo adds the
  hard-contact NCP/IPM and implicit-gradient requirements;
- deformable IPC/VBD paths remain separate domains and would couple to this rigid
  solver only through the pairwise coupler architecture.

## Sequencing

1. Keep PLAN-110's active Nimble-style PR hardening path unchanged.
2. Run a Dojo de-risking spike after the current differentiable surface lands:
   implement an internal single-body or planar multibody NCP/IPM toy solve,
   expose no public API, and compare the implicit-gradient result with central
   finite differences.
3. If the spike succeeds, decide whether the Dojo path stays under PLAN-110 as a
   second opt-in rigid solver or splits into a new initiative. The split decision
   should depend on how much maximal-coordinate model/state machinery overlaps
   with PLAN-080 and PLAN-082.

## Acceptance Gate Before Promotion

The Dojo-style path is not promotable until DART has:

- an internal maximal-coordinate or constrained-coordinate state model that
  coexists with the current DART 7 `World`;
- a documented NCP/SOC contact formulation, solver residual definition, and
  central-path smoothness/accuracy contract;
- finite-difference agreement for state/control/parameter derivatives on named
  contact and non-contact scenes;
- one reproduced Dojo paper/tutorial example packet;
- benchmark/profiling JSON for solver iterations, residuals, and gradient cost;
- no Dojo.jl runtime dependency and no public solver/backend/cache types.

## Open Questions

- Does DART need a maximal-coordinate representation to gain the Dojo benefits,
  or can the variational/NCP/IPM pieces be adapted to articulated coordinates?
- Should the central-path parameter become a public differentiability control, or
  remain an internal solver tolerance until examples prove useful gradients?
- Which first comparison scene is smallest while still exercising contact,
  friction, and implicit gradients: cartpole contact, falling block, or a simple
  legged mechanism?
