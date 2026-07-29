# M2.2 — elastic element forces

Second increment of the Kim/Pollard half of the full-parity program
(`10-full-parity-execution-plan.md` §7/§11), building on the M2.1 foundation in
`13-fem-foundation.md`. This is the increment that turns a falling cloud of
lumped masses into a body that actually deforms, stores energy, and springs
back.

Branch `wp-db-fem-elastic`, based on `release-6.20` after #3382 merged
(`6c88ac1d774`).

## What landed

- `ElasticModel` — `Linear` (small strain) and `Corotational` (co-rotated
  linear, the default).
- Lame parameters derived from the material's Young's modulus and Poisson's
  ratio, with both now validated on construction.
- Per-element deformation gradient `F = Ds · Dm⁻¹`, using the inverse rest shape
  matrix M2.1 already cached for exactly this purpose.
- First Piola-Kirchhoff stress for both models, nodal force assembly, and
  elastic potential energy.
- Polar decomposition through a 3x3 SVD, with the reflection fix that keeps an
  inverted element mapping to the nearest true rotation.
- Node pinning (`setNodeFixed`), which anchors a body to the world without
  needing a coupling constraint and makes hanging/cantilever scenes possible.

## Math

Lame parameters: `mu = E / (2(1+nu))`, `lambda = E·nu / ((1+nu)(1-2·nu))`.

**Linear** (small strain): `strain = ½(F + Fᵀ) - I`, and
`P = 2·mu·strain + lambda·tr(strain)·I`. Cheap, and correct only while
rotations stay small — it cannot distinguish a rotation from a stretch.

**Co-rotated**: factor the element rotation out first, `F = R·S` by polar
decomposition, then
`P = 2·mu·(F - R) + lambda·(tr(Rᵀ F) - 3)·R`. Under a rigid rotation `F == R`
exactly, so both terms vanish identically and the model produces no force at
all. This is why it is the default.

Nodal forces of one element are `H = -volume · P · Dm⁻ᵀ`, whose columns are the
forces on nodes one through three; the force on node zero follows from the
element exerting no net force on itself, which is also what makes the global
force balance exact.

Integration extends the M2.1 update with the elastic acceleration, keeping the
mass-proportional implicit damping:
`v <- (v + (f/m + g)·dt) / (1 + c·dt)`, then `x += v·dt`. With no deformation
`f` is exactly zero, so every M2.1 closed-form oracle still holds unchanged —
and it does: all fourteen M2.1 gates pass untouched, because a freely falling or
uniformly translating body keeps `F == I`.

## Acceptance gates

Seven new gates, twenty-one in total. The new ones:

1. **Rest has no force and no energy**, for both models.
2. **Rigid translation produces no force** — `F` is translation invariant.
3. **Co-rotated is rotation invariant while linear is not.** This is the gate
   that justifies the whole co-rotated model: under a rigid rotation the
   co-rotated peak nodal force and stored energy are both below `1e-8`, while
   small-strain elasticity manufactures a peak force and stored energy above
   `1.0` out of a motion that did not deform the body at all.
4. **Elastic forces are internally balanced** — under a deliberately non-uniform
   deformation with peak nodal force above 1 N, the summed force over all nodes
   is below `1e-9` of that peak. Internal forces cannot accelerate the body as a
   whole.
5. **Energy is exactly quadratic in strain and exactly linear in stiffness.**
   A uniform stretch leaves the co-rotated rotation at identity, so the energy
   density reduces to `(mu + lambda/2)·s²`. Doubling the stretch multiplies
   energy by exactly 4, and doubling Young's modulus by exactly 2, both to
   `1e-9`. These are ratio checks, so they test the constitutive law rather than
   restating the implementation.
6. **A stretched body relaxes without blowing up** — released from a 5% stretch
   with damping, the energy never exceeds 1.5x its initial value and falls below
   5% of it within 2000 steps. This is the explicit-integration stability gate.
7. **Fixed nodes anchor the body, and a softer material sags further.** With one
   face pinned, anchored nodes stay put to `1e-12`, the free end hangs instead of
   falling, and a body four times softer sags measurably more.

## Stability limit — a real constraint, stated plainly

Elastic forces are integrated explicitly, matching the papers' integration
choice. That carries a CFL-style bound: the step must satisfy roughly
`dt < h·sqrt(rho/E)` for element size `h`. The default material (E = 1e5 Pa,
rho = 1000 kg/m³) on the test meshes is comfortably stable at `dt = 1e-3`, but a
stiffer material or a finer mesh will need a smaller step, and **nothing warns
about it today** — an over-large step simply diverges. Gate 6 covers the
configurations in use; it does not make the bound go away. Options for later are
an implicit or semi-implicit elastic solve, or a computed CFL warning at
construction. Damping is unaffected: it remains unconditionally stable.

## Cost, and what M2.3 targets

The co-rotated model runs one 3x3 SVD per element per step, which dominates the
elastic cost. That is precisely what Kim and Pollard's selective diagonalization
addresses, and it is the M2.3 target along with the reduced/modal basis. No
performance claim is made here; this increment is correctness first.

## Compatibility

Unchanged from M2.1, and re-verified: this increment touches **no pre-existing
runtime source file**, so rigid-body simulation runs identical instructions, and
no pre-existing object references `dart::dynamics::fem::`. The subsystem remains
internal-only — no installed header, not in the generated aggregate, excluded
from Doxygen — so the release branch's public API and ABI are untouched. The
gz-physics and gz-sim gates passed on this base (199/199 functional, 4/4
performance, 1/1 gz-sim integration).

## Next

- **M2.3** reduced/modal coordinates and selective diagonalization, with the
  performance rows and an accelerated embedded-surface binding.
- **M2.4** skeleton coupling, one-way then two-way.
- **M2.5** collision, the four paper characters, demos, and the parity gates.
