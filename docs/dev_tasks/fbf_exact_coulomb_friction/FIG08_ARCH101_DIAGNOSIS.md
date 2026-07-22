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

## Pinned-Source Control

### Short probe

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
they are not proven sufficient to make the 101-stone arch stand.

### Full current-source run

The same pinned public command completed all 400 frames / 1,600 substeps and
exited zero in 1,071.20 seconds. That completion is not convergence evidence:
only 127 substeps converged, while 1,473 (`92.0625%`) exhausted the source's
200-outer-iteration cap and continued. Contacts ranged from 8 to 432 and ended
at 428. The final step was nonconverged at residual `0.002282420096486425`.

The saved keystone (`stone-51`) fell from `66.13856506347656` to
`58.90357971191406`, or `7.2349853515625` raw units. Fifty-seven of 99 mobile
stones changed height by more than the local three-unit standing limit. All
saved trajectory arrays and all 7,190,132 numeric history values are finite.
The audit JSON SHA-256 is
`56844eee3d908a1078fe7e76c6a92e31f2de79eb6e1e503ca7173d4e078c6cd4`;
history, result, and trajectory SHA-256 values are respectively
`e5ec24fd156ef40919785096ccac1ceb7fdfb981b1d3e3ba1fb0356346a1c9ac`,
`97bf1609df85707cc810631a9315c0efb2b4c15d9344a56628c10f66dbf08eba`,
and `ed7b151f491146a0b02161dbeddcdb86a07ddf780a48cf82b23eb58598cdf634`.

The public runner persists only initial/final stone heights and the keystone
height timeline. It does not save final x/y positions, quaternions, or media.
This proves failure of the local standing criterion by vertical displacement
alone, but it does not support a visual-collapse, full 3D displacement, or
rotation claim.

## Step-209 Solver Diagnosis

The exact run forms two chain-like contact groups. At step 209 one group
converges in 4,501 outer iterations and the other reaches the 5,000-iteration
limit at residual `1.2582804496066107e-6`, only above the unchanged `1e-6`
tolerance. The capped solve remains finite and is accepted internally; the
strict evidence runner then fails closed. There are no local exact failures,
boxed fallbacks, non-finite values, or line-search shrink events at this state.

This is a valid current-DART strict negative. The DART step-209 cap alone does
not establish failure at a corresponding source state because the source uses
different solver policies and no fail-fast evidence gate. The independent
source nonconvergence result above is a separate full-run observation, not a
step-to-step trajectory match.

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
| Source-sized contact gaps (`0.005` shapes, `0.1` ground) | 161 | `1.51681505e-6` | about 30 s | Reject: contact starts earlier, but cap is 48 steps earlier |

The 60-sweep trial delays the solver cap but reaches step 235 with maximum
mobile displacement `4.722765625`; the standing oracle had already failed.
Do not combine these rejected knobs or weaken the tolerance/cap/oracle.

## Rejected Contact-Gap Dynamics Trial

The default-off Native implementation expands broad-phase candidates, emits
finite signed convex-convex and plane-convex proximity manifolds strictly
inside the summed gap, and admits only explicitly configured negative-depth
contacts without penetration bias. Its focused collision and solver suites
pass 50/50 and 65/65. The sealed source-pinned scene remains unchanged.

An external one-factor harness then applied the source-sized gaps: `0.005` to
every stone/cube collision shape and `0.1` to the ground. First contact and
first negative contact move from baseline step 39 to step 37, one step later
than the current source's step-36 dynamic-contact onset. The trial nevertheless
caps at step 161 instead of 209. At that state it has 135 contacts, including
95 negative-depth contacts; one 67-contact group reaches 5,000 iterations at
residual `1.5168150500676777e-6`, while the companion 68-contact group solves
in 460 iterations at residual `9.989170207559752e-7`. Maximum mobile
displacement is `2.2210453125000313`.

This is a dynamics-fix reject. The shorter wall time is not a speed result
because the harness stops 48 steps earlier. Do not wire the gaps into the
sealed scene, combine them with rejected tuning knobs, or capture a full
exact/boxed pair from this variant. Revisit only with a matched historical
contact oracle or recovered historical Figure 8 invocation/backend.

The complete public `--stones 101` control therefore cannot serve as a
converged golden trajectory and does not preserve the initial configuration
under the local standing criterion. The historical Figure 8 invocation and
backend remain unrecovered. Do not claim source visual collapse, historical
Figure 8 parity, or DART-versus-source solver superiority.
