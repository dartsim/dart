# Literal-Wedge Crown-Impact v1 Preregistration

Status: frozen before the first diagnostic run on 2026-07-12.

This file preregisters the separately named
`masonry_arch_25_literal_wedge_crown_impact_v1` trace contract. It is a DART
reconstruction intended to test whether the already-validated literal-wedge
arch has a localized, finite response to a small crown impact. It is not the
paper's Fig. 7 scene, does not establish paper parity, and must retain the scene
contract label
`reconstructed_literal_wedge_crown_impact_v1_nonpaper_native_collision_frontend`.

## Frozen scene and timeline

- Reuse `masonry_arch_25_literal_wedge` without changing its 25 wedge meshes,
  mass or inertia calculation, one-micrometer interface closure, 1.001 mm
  downward shift, pinned endpoint stones, friction 0.8, Native collision
  frontend, FourPointPlanar manifolds, split impulse, zero contact ERP, exact
  FBF options, or deterministic colored-inner-BGS option.
- Complete steps 1 through 600 with no projectile skeleton present. This is the
  standing prefix and corresponds to 10 seconds at `dt = 1/60 s`.
- After completed step 600 and before stepping 601, snapshot all 25 stone
  poses, then add exactly three dynamic cubes.
- Projectile x coordinates are `-0.045`, `0`, and `0.045 m`; y is `0`; z is
  `0.95 m`. Each cube has edge length `0.035 m`, density `1000 kg/m^3`, mass
  `0.042875 kg`, friction `0.8`, linear velocity `(0, 0, -3) m/s`, and zero
  angular velocity.
- Run 120 post-launch steps, for a default total of 720 completed steps.
- Do not change these parameters or the thresholds below in response to the
  result. A miss is a scientific negative for this frozen v1 reconstruction.

## Standing-prefix contract

At completed step 600, before projectile insertion:

- exactly 25 arch bodies exist; only stones 0 and 24 are immobile;
- no projectile exists;
- all states are finite;
- exact failures, boxed-LCP fallbacks, and accepted iteration caps are zero;
- every comparable exact residual is finite and at most `1e-6`;
- the maximum stone displacement from constructed t0 is at most `0.001 m` and
  minimum orientation alignment is at least `0.999`;
- the standing contact manifold has 96 contacts over 24 unique colliding body
  pairs.

Impact rows must retain the ordinary performance fields needed for an external
field-by-field comparison with `masonry_arch_25_literal_wedge`. The impact-only
extension records the reference scenario and whether each row belongs to the
600-step comparable prefix. The comparison excludes only the scenario name,
scene-contract label, wall time, CPU-residency fields, and the impact-only
extension; all other physical and exact-solver fields are eligible for exact
standing-prefix comparison.

## Frozen impact acceptance gates

The impact phase must satisfy all of the following:

- at least one projectile contacts an arch stone, and the first projectile-arch
  contact occurs before any projectile-ground contact;
- all body states remain finite;
- exact failures, boxed-LCP fallbacks, and accepted iteration caps remain zero;
- every comparable post-launch exact residual is finite and at most `1e-6`;
- localized crown response, defined as the maximum translation of stones 9
  through 15 from their pre-impact poses, is at least `0.0001 m`;
- final maximum translation of any arch stone from its pre-impact pose is at
  most `0.07 m`;
- final minimum stone orientation alignment from its pre-impact pose is at
  least `cos(30 degrees) = 0.8660254037844386`;
- final maximum translation outside stones 9 through 15 is at most `0.007 m`;
- pinned springers 0 and 24 remain unchanged within `1e-12 m` translation and
  `1e-12` orientation-alignment error;
- at the final step, all 16 adjacent far-field interfaces are present in the
  collision result: `(0,1)` through `(7,8)` and `(16,17)` through `(23,24)`.

The trace must expose step and cumulative projectile-arch and
projectile-ground contact counts, first-contact step/time, finite-state and
exact-solver counters, maximum comparable residual, pose metrics, final
far-field adjacency count, and explicit gate booleans. Gates that depend on
the final state are diagnostic before step 720 and authoritative only on the
final row. The process fails closed on exact-solver failure during stepping and
returns failure after step 720 if any preregistered gate is false.

## Scope exclusions

- No visual capture is part of this numeric lane.
- The existing standing visual bundle and capture script are immutable here.
- The existing 83-column default and 95-column colored standing schemas must
  remain byte-identical. Impact fields are appended only for the separately
  named impact scenario.
- No paper timing, paper trajectory, or author-source claim may be inferred
  from this v1 result.

## Frozen v1 result

The first run of the preregistered contract completed all 720 steps and
returned failure, as required for a scientific negative. The 136-column
impact trace had SHA-256
`b076f375071043a4a36fb3af0c9a2fbf06b795127892634eb1aa983d4f0c7677`.
The result must not be used as paper parity or as a passing impact example.

The standing-prefix gate passed. A separate 600-step
`masonry_arch_25_literal_wedge` reference run was compared across all 88
eligible non-timing, non-label, non-residency fields, with zero mismatches.
The impact trace also recorded no projectile through completed step 600.

The impact produced a real localized response and remained finite:

- first projectile-arch contact: step 607 (`10.116666666666667 s`);
- first projectile-ground contact: step 616 (`10.266666666666666 s`), so the
  preregistered contact-order gate passed;
- maximum crown displacement from the pre-impact snapshot:
  `0.070939644312156866 m`, above the `0.0001 m` response floor;
- minimum final orientation alignment: `0.99194711995436347`, above
  `cos(30 degrees)`;
- springer displacement/alignment: `0 m` / `1`, and all 16 final far-field
  adjacent interfaces remained present;
- exact failures and boxed-LCP fallbacks: zero; all body states stayed finite.

The final acceptance gate failed for three independent preregistered reasons:

- five exact solves were accepted at the outer-iteration cap, while the
  contract requires zero;
- worst exact residual was `9.1545317042653963e-05`, above `1e-6`;
- final maximum arch displacement was `0.070939644312156866 m`, above
  `0.07 m`, and final far-field displacement was `0.060523747030465196 m`,
  above `0.007 m`.

No parameter or threshold was changed after observing this result.

The final lint-clean current-source locally sealed bundle is stored under
`assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/` and can
be reproduced with:

```bash
.pixi/envs/default/bin/python \
  scripts/run_fbf_literal_crown_impact_negative.py --cpu 8 \
  --output-dir /tmp/fbf_literal_crown_impact_v1_recheck
```

The runner treats child exit 1 plus the exact frozen failed-gate pattern as a
valid scientific-negative artifact. It independently reruns the standing
scenario and requires 600 by 88 exact field matches, recomputes every final
gate from finite metrics, requires post-launch exact-solve progress, pins the
normalized trace fingerprint, binds the frozen preregistration-contract hash,
and records and rechecks the resolved `taskset` executable after both child
runs. It rejects child exit 0, a passing or internally inconsistent impact
gate, schema drift, prefix projectile activity, runtime-tool identity drift,
or any frozen-result drift.
Bundle SHA-256 values from the recorded run are:

- frozen preregistration contract: `4cdea674f366fc2d18eadf11ef4333d491786d5d85e3fc16fa611ea7dede3f37`
- normalized trace fingerprint: `86b37ec37d28259514453c9ecc9e7d5f12afe08118a0b04abd40e72acd147384`
- runner source: `622b388142fed881191dcb6efb103266eb2ea1e3ffbadaf402f63b89c558bb67`
- executed `/usr/bin/taskset`: `3ac0b42cbe242b78c63b3ecef3023f3bcc17432b9d4731ac874848df6a87c914`
  (`31184` bytes)
- trace source: `b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`
- trace binary: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`
- `raw.csv`: `42cc94f5f111442da42b03af33722f713d66b853d3ac21f21083f28ba24e7ba4`
- `stderr.txt`: `7964b07fd5396f10013ff8d9100d2b36e7dfc151764210d8c60bd0ae853a2d94`
- `standing-reference.csv`: `22fabfe2e8f35bf329f32873845a60599ed2df171c15728f14311b91d888c387`
- `standing-reference.stderr.txt`: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`
- `summary.json`: `e3baa479111a999d03359a0642533cad1baa3a48c415609bafb9a7a7cfa18e7c`
- `metadata.json`: `0bd19797953a10223bbd7183f6e8002e490b21c29b2304befee7e6618ff9ee73`
- `REPORT.md`: `c5711b0fe06e679f889f6a7ca3812aa955c3b8c61270ef038def06b2a3f173ff`

Superseded local iterations through v8 are historical and are not canonical
evidence. V9 reran the unchanged preregistered command after runtime-tool
closure hardening; the scientific result is unchanged and was not
retroactively relabeled.
