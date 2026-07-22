# Figure 8 101-Stone Blocker Diagnosis

## Scope

This note records post-capture diagnosis for the source-pinned
`fbf_author_masonry_arch_101_standing_current_source` scene. It does not change
the sealed `fig08_arch101_author_current_v1` packet, relax the strict standing
oracle, or promote the current DART result into paper-parity evidence.

## Strongest Current Finding

The DART standing gate fails before the step-209 iteration cap: the unsupported
crown has already crossed the displacement limit at step 188. The first
confirmed source/DART modeling mismatch that can affect this physical result is
contact activation. The pinned source assigns a `0.005` gap to each stone and
cube. Its Newton builder supplies the ground's otherwise-unspecified default
gap of `0.1`, and collision activation sums the two shape gaps. The effective
source thresholds are therefore:

- stone to stone: `0.005 + 0.005 = 0.010` raw units;
- stone to ground: `0.005 + 0.100 = 0.105` raw units.

A collision-only source probe at the initial 101-stone pose reports eight
contacts: four manifold points for each fixed springer against the ground. The
current Native DART adapter records `0.005` as source metadata but implements
zero-gap collision and begins with zero contacts. Native's
`allowNegativePenetrationDepthContacts` option cannot close this gap by itself:
the Native narrow phase does not generate separated convex contacts, and the
constraint solver intentionally ignores negative-depth contacts.

The DART crown follows semi-implicit free fall exactly while unsupported:

```text
displacement(n) = 0.5 * 9.81 * (1 / 240)^2 * n * (n + 1)
```

There are zero DART contacts through step 38; contacts begin at step 39 and
grow inward in eight-contact increments. The crown reaches the preregistered
three-unit displacement limit at step 188 (`3.025771875` raw units), before the
strict exact solve reaches its first cap. At step 209 the analytic free-fall
displacement is `3.7375078125`, exactly the recorded maximum displacement.
Clearing the solver cap alone therefore cannot establish standing.

## Pinned-Source Short Probe

An isolated CPU run of the pinned source used its explicit public command:

```text
python -u paper_examples/masonry-arch/run.py --solvers fbf --stones 101 \
  --frames 30 --drop-frame 400 --device cpu
```

It completed 120 substeps in 28.04 seconds. The eight initial fixed-springer to
ground contacts remained through step 35; the first dynamic contacts appeared
at step 36, and the count reached 80 at step 119. The source recorded 27
nonconverged history steps at its 200-iteration cap but deliberately continued.
The crown fell from `66.138565` to `64.902092`, a `1.236473`-unit drop at 0.5
simulated seconds. That is still inside the three-unit local DART threshold but
covers only 7.5 percent of the full horizon.

This short probe confirms different contact onset and fail-through policies,
but it also shows that the source crown initially falls by essentially the
same analytic amount. Source-style gaps are required for a faithful comparison;
they are not yet proven sufficient to make the 101-stone arch stand. A full
400-frame source run is scheduled separately because the measured runtime is
approximately 50 to 85 minutes.

## Step-209 Solver Diagnosis

The exact run forms two chain-like contact groups. At step 209 one group
converges in 4,501 outer iterations and the other reaches the 5,000-iteration
limit at residual `1.2582804496066107e-6`, only above the unchanged `1e-6`
tolerance. The capped solve remains finite and is accepted internally; the
strict evidence runner then fails closed. There are no local exact failures,
boxed fallbacks, non-finite values, or line-search shrink events at this state.

This is a valid current-DART strict negative. It is not evidence that the
pinned source solver fails: the source uses different solver policies and does
not apply DART's fail-fast evidence gate.

## Rejected One-Factor Solver Trials

All trials preserved the source-pinned geometry and schedule, `1e-6` outer
tolerance, 5,000-iteration cap, no boxed fallback, and strict fail-fast
interpretation. They are diagnostics only and are not evidence packets.

| Trial | First cap | Capped residual | Wall time | Decision |
| --- | ---: | ---: | ---: | --- |
| Baseline | 209 | `1.25828045e-6` | 156.51 s | Reference negative |
| Step-size scale `35 -> 50` | 181 | `4.16529873e-6` | 46.33 s | Reject: earlier and worse |
| Outer relaxation `1.1 -> 1.5` | 186 | `1.41504369e-6` | 55.00 s | Reject: earlier cap |
| Serial block Gauss-Seidel | 205 | `1.12625911e-6`, `1.02764187e-6` | 132.97 s | Reject: both groups cap |
| Inner sweeps `30 -> 60` | 235 | `1.65823432e-6`, `1.68193538e-6` | 433.74 s | Reject: still caps and costs about 2.8x |

The 60-sweep trial delays the solver cap but reaches step 235 with maximum
mobile displacement `4.722765625`; the standing oracle had already failed.
Do not combine these rejected knobs or weaken the tolerance/cap/oracle.

## Next Faithful Experiment

Before further exact-solver tuning, isolate source-style summed proximity gaps
as one factor. Any implementation must be default-off and leave ordinary
Native collision bit-identical. It must expand broad-phase candidates as well
as generate finite convex-convex and plane-convex proximity contacts; merely
retaining negative-depth backend contacts is insufficient. The first gate is a
collision-only comparison of pair identities, points, normals, signed
separations, and multiplicity at matched poses. Only after that matches closely
enough should a 1,600-substep exact/boxed dynamics run be captured.

The public pinned source's complete `--stones 101` outcome is still being
validated independently. Until that completes, do not claim that its supported
101-stone selection stands, reproduces historical Figure 8, or supplies a
golden trajectory.
