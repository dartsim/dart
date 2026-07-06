# Orchestration dashboard — DART 6 performance generalization

Status board only; packet definitions live in the lane docs. Update this
file (and RESUME.md) in every PR that claims or completes a packet.

Baseline: `origin/release-6.20` @ `70b92010311` (2026-07-04).
Related open queue at last refresh: none blocking (enablers merged:
#3209, #3226, #3229, #3230, #3234, #3281). WP-PG.40 standalone PR
#3270 was closed by maintainer direction and its D1/D2 evidence now rides
with the first real SIMD-kernel PR.

## Lane status

| Lane | Owner doc | Packets | Status |
| --- | --- | --- | --- |
| WS-A constraint/LCP | 02-constraint-lcp-lane.md | PG.10–PG.15 | open (PG.13 evidence-gated; PG.14 blocked D3; PG.15 blocked D7) |
| WS-B ODE backend | 03-ode-backend-lane.md | PG.20–PG.23 | open (PG.23 blocked D8; lane re-review at WS-F phase 5) |
| WS-C dynamics batching | 04-dynamics-batching-lane.md | PG.30–PG.33 | open (PG.33 gated) |
| WS-D SIMD enablement | 05-simd-enablement-lane.md | PG.40–PG.42 | active (PG.40 folded into PG.42; PG.41 waits for PG.10 seam evidence) |
| WS-E infra/evidence | 06-infra-evidence-lane.md | PG.01–PG.04 | open (**PG.01 first**) |
| WS-F native collision port | ../dart6_dependency_minimization/03-native-collision-port-scoping.md | phases 0–7 | external owner; phase 1 internal math core merged (#3281); no DART 6 detector adapter or phase-4 broadphase SIMD yet |

## Packet board

| Packet | Lane | Status | Branch / PR | Evidence |
| --- | --- | --- | --- | --- |
| WP-PG.01 baseline packet | WS-E | open | — | — |
| WP-PG.02 benchmark matrix | WS-E | open | — | — |
| WP-PG.03 profiling doc | WS-E | open | — | — |
| WP-PG.04 executor tooling | WS-E | blocked (D4) | — | — |
| WP-PG.10 LCP instrumentation | WS-A | open | — | — |
| WP-PG.11 solver RTTI removal | WS-A | open | — | — |
| WP-PG.12 direct assembly | WS-A | open | — | — |
| WP-PG.13 row islanding | WS-A | evidence-gated (PG.10 census) | — | — |
| WP-PG.14 matrix-free path | WS-A | blocked (D3) | — | — |
| WP-PG.15 creep vs rest-veto | WS-A | blocked (D7) | — | — |
| WP-PG.20 history spans | WS-B | open | — | — |
| WP-PG.21 history map/pruning | WS-B | open | — | — |
| WP-PG.22 version-gated pose push | WS-B | open | — | — |
| WP-PG.23 ODE manifold reduction | WS-B | blocked (D8) | — | — |
| WP-PG.30 free-body cache + FD path | WS-C | open | — | — |
| WP-PG.31 shallow-support scratch | WS-C | open | — | — |
| WP-PG.32 frame arena + alloc gate | WS-C | open | — | — |
| WP-PG.33 SoA integration | WS-C | gated | — | — |
| WP-PG.40 FP/ISA contracts | WS-D | folded into WP-PG.42 | #3270 closed | maintainer direction: carry D1/D2 evidence with actual SIMD kernel PR |
| WP-PG.41 batch math seam | WS-D | blocked (PG.10 seam evidence) | — | — |
| WP-PG.42 SoA broadphase | WS-D | claimed — local | `wp-pg-42-soa-broadphase-simd` | AVX-width finite sweep SIMD screen, scalar/SSE/NEON fallback, SIMD CI consumer coverage, contact-container macro rows, finite-finite profile scope |

Claim flow: set the packet row to `claimed — <who/session>` with the
`wp-pg-<nn>-<slug>` branch name, update RESUME.md, and open the packet PR
titled `WP-PG.<nn>: <title>`. On completion set `done — <PR#>` and add the
evidence link. Author never self-approves; an independent reviewer accepts
against the packet's acceptance evidence.

## Cross-lane coordination notes

- WS-F consumes WS-D kernels in its phase 4; WS-D's WP-PG.42 checked live
  phase status before claiming. As of #3281, WS-F has internal native
  collision math only, with no DART 6 detector adapter or broadphase SIMD.
- WS-B investment is re-reviewed when WS-F reaches phase 5 (facade
  decision) — see D5 in the README.
- WS-A WP-PG.12 and WS-C WP-PG.30 share the single-free-body
  classification; land WP-PG.30 first when possible.
- Deactivation-gate work (#3226) is merged; any packet touching sleeping
  behavior must include deactivation counters in its evidence table.
- Six round-1 experiment branches live on origin (`perf/dart6-*`, pushed
  2026-07-03, 21–28 commits behind); prior-art inventory + rejected list
  in 01-baseline-evidence.md. WP-PG.01 triages them; packets overlapping
  them (PG.30, PG.42) must read the inventory before claiming.
