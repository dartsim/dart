# CCD Milestones

Each milestone is independently shippable and must pass `pixi run lint` +
relevant `pixi run test-unit` before being considered done.

## M1 — Primitive CCD (point–triangle, edge–edge) ← active

- `narrow_phase/primitive_ccd.{hpp,cpp}`: ACCD PT/EE, closest-distance
  primitives, conservative TOI, minimum-separation support.
- `narrow_phase/primitive_ccd_exact.{hpp,cpp}` (or same TU): coplanarity cubic
  for test cross-validation.
- Unit tests: hits, misses, grazing, degenerate (parallel edges, point on
  vertex), conservativeness assertion (`τ_ACCD ≤ τ_cubic`), determinism.
- Benchmarks: PT/EE ACCD rows in `bm_ccd.cpp`.
- **Exit**: tests green; ACCD never overshoots cubic on the cross-check corpus.

## M2 — Unified rigid CCD motion model

- `CcdMotion` (linear, screw/interpolated) feeding conservative advancement so a
  single call covers what the reference engines call translational / interp /
  screw continuous collision.
- Reuse existing `conservativeAdvancement` + casts; no behavioral regression.
- **Exit**: parity tests vs. existing per-shape casts; one entry point.

## M3 — Swept broad-phase

- Swept-AABB (union of start/end AABBs, optionally inflated by motion) feeding
  the existing broad-phase so world-level CCD only narrow-tests plausible pairs.
- **Exit**: world-level `*CastAll` returns identical hits to brute force at lower
  cost on a populated scene.

## M4 — Comparative benchmarks & manifest

- Add CCD comparison rows to the comparative benchmark suite where a reference
  equivalent exists (sphere/capsule cast, convex advancement).
- Regenerate `docs/plans/035-native-collision/benchmark-manifest.md`; drive
  every comparable CCD row to `lead`.
- **Exit**: zero `behind` CCD rows in the manifest.

## M5 — Public API + bindings

- Detector-level CCD entry (DART-semantic options/results) + dartpy.
- **Exit**: Python smoke test; `dart/collision/AGENTS.md` compatibility checklist
  satisfied.

## Definition of done (whole task)

- Feature superset of the three reference engines (rigid casts + advancement)
  **plus** primitive PT/EE — verified by the superset matrix in `01-design.md`.
- No comparable CCD benchmark row is slower than the strongest reference engine.
- Durable design notes promoted to `docs/onboarding/`; this folder removed in the
  completing PR.
