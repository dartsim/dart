# FBF GUI And Video Evidence Report

## Current Status

Visual evidence is partial. The small-scene capture system now produces
auditable, solver-gated media, and a complete post-review small matrix exists.
The card, 101-stone arch, and crown-impact examples do not have valid current
long-run media. The current-source nine-row numeric small matrix is now a
repository-finalized evidence bundle. Five visual bundles are
repository-finalized: literal-wedge standing, the two-cell incline threshold,
the two-cell Painleve proxy, backspin, and the author-pinned four-cell
turntable matrix. They remain bounded
DART evidence, not paper-golden or paper-timing evidence. The current visual
workflow inventory declares 19 schedules. The task-wide manifest remains 29
requirements = 24 partial + 5 blocked + 0 complete, so the overall task is
still incomplete.

## Evidence Predicates

The visual workflow and this report use five separate predicates:

- `artifact_valid`: the declared files, JSON schema, hashes, ordering,
  dimensions, frame rate/count, duration, and full decode are valid;
- `solver_contract_valid`: every completed requested step and every exact
  solver group passes its status, residual, cap, exact-failure, and fallback
  gates;
- `physical_outcome_valid`: a separate scenario-specific quantitative trace or
  regression passes;
- `manual_inspected`: a person or agent actually viewed the media and recorded
  the qualitative observation; and
- `claim_valid`: every predicate required for the specific claim, including
  final build/source identity and comparability, passes.

None implies another. A decoded video can show the wrong physics; a
solver-valid clip can differ from the paper; manual inspection cannot override
a failed quantitative outcome.

## Literal-Wedge Numeric And Visual Evidence

The durable bundle
`assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore`
records one warmup and three measured 600-step trajectories at each of one and
four threads for an explicit non-paper exact-inertia literal-wedge
reconstruction. Every measured step remains at 96 contacts and 24 colliding
pairs, exact FBF succeeds with maximum residual
`9.999807145410957e-7`, no accepted caps, failures, or fallbacks occur, and the
standing physical outcome passes for all 25 stones. Maximum displacement from
the constructed initial state is `5.431169776791696e-6 m`; minimum orientation
alignment is `0.9999999999111284`. Mean timings are `6.122883 ms` at one thread
and `4.269397 ms` at four threads, with validated speedup `1.434133x`. The
one-thread p95 and both maxima exceed the 60 Hz budget; the four-thread p95 is
`14.396602 ms` and does not. Both maxima still rule out an every-step deadline,
so this proves mean throughput only.

Those facts authorize a numeric standing/outcome/scaling claim for the named
Native `FourPointPlanar`, `1 um`-closure reconstruction.

The matching standing bundle under
`assets/paper_evidence/fig07_arch25_literal/` is finalized with
`claim_valid=true` for that current-source, no-projectile, non-paper scope. It
contains five durable 1280x720 stills, a fully decoded 61-frame H.264 schedule,
a five-panel timeline, the full trajectory, and an independently generated
reference trace. All 600 rows have zero differences in the nine mapped integer
and five mapped floating-point fields, with fixed reference/scene-contract
checks. The five zero-iteration warm-start rows retain the required
24-manifold/3-color/width-8 schedule and zero colored work.

The five durable stills, timeline, and a separately decoded 3.0-second video
midpoint were manually inspected. The retained pending hash DAG passed before
final metadata/index writes, and the final index covers 19 artifacts in a
21-file physical directory. The 70-file raw capture staging directory is
pruned after sealing, and clean-checkout verify-only needs no ignored files. The
10-second simulation is sampled into 61 frames and played for 6.1 seconds at
10 fps, or `1.639344x` time-lapse. This is not a real-time playback claim.

| Artifact | SHA-256 |
| --- | --- |
| `metadata.json` | `b217b0a43200ca558f09776e464d50f0f9dc5c5a175bcfa66ae1b656076871d1` |
| `provenance.json` | `eca349842e4121584145cd039a554aa13c51e5242ff924b37f5f49a9dee0ac2f` |
| `artifact-index.json` | `4d173c78e2c6584d6d8a85867160cc0c0687fe0f35e6878f66b0cfcd7c1c15ee` |
| `manual-inspection.json` | `4b52bcd26ed88d184bd695d77070420aeec2c95edfb203efda7ae6241150c343` |
| `pending-metadata.json` | `e300103dbb950b97e217ca0bea6f1c1dc78598ddf485cdaca26cc7ad58835b3c` |
| `fig07_literal_wedge_stability.mp4` | `e0c111f96511bbda41af145787bd9f68782b8c9e8a909d45eb03d3c3d647feb1` |
| `fig07_literal_wedge_timeline.png` | `926380c5c5768cc9ca1e414cbfe2631584daa99387090926d5a1f035d62b6ec9` |
| `decoded/video_midpoint_t3.0.png` | `75a88bb317441ed71803f784b2eaa099211c4e026f8547d6fe5f2ff3ee95909f` |

Fresh revalidation against the final current trace and Native source binds the
trace source/binary to
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`
and
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`,
while the capture source
`c3efeac52d02a0c373f733598db81e545d062195ba6e96c2a65bcb607cd0207f`,
capture binary
`8b3cad15220c8fdb69c3ebdf7fa3923fda6fd812a49d0e54c8aeb07e62f0a7e9`,
and durable media remain unchanged. Finalizer driver
`0f4e27b0c58e9dd3774c6be48ad4c70a857e2956fd81f11710837561f08f7243`
regenerated reference trace
`0e41af230fb4c27143966c3f8522702280e6746b31e84155a9a047326e46dbb3`
from the current binary and again passed all 600 zero-difference rows. Its
test source is
`f06ba295006cf7e1f0fb692fc5eacf62e4e7f3be89bd166c041752d915779bdd`;
the full visual revalidation module passes 16/16.

The valid claim is a visually and numerically stable standing reconstruction.
The scene has no projectile, and no current capture applies or validates the
paper crown impact, pinned public author scene, or paper timing.

## Crown-Impact v1 Is A Numeric Negative, Not Media

The separately preregistered 720-step numeric run at
`assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/`,
reproduced by `scripts/run_fbf_literal_crown_impact_negative.py`, is a valid
scientific negative. Its projectile-free standing prefix matches 88 eligible
fields with zero differences. Projectile-arch contact begins at step 607
before ground contact at step 616; all bodies remain finite with zero exact
failures or fallbacks. The impact claim still fails: five caps are accepted,
worst residual is `9.1545317042653963e-5`, all-arch displacement is
`0.07093964431215687 m > 0.07 m`, and far-field displacement is
`0.060523747030465196 m > 0.007 m`.

No parameter or threshold was tuned after this first frozen v1 result. The
lane has no visual capture, and the valid standing video must not be relabeled
as impact media. The final runner SHA-256 is
`622b388142fed881191dcb6efb103266eb2ea1e3ffbadaf402f63b89c558bb67`.
Bundle SHA-256 values are raw
`42cc94f5f111442da42b03af33722f713d66b853d3ac21f21083f28ba24e7ba4`,
stderr `7964b07fd5396f10013ff8d9100d2b36e7dfc151764210d8c60bd0ae853a2d94`,
standing reference
`22fabfe2e8f35bf329f32873845a60599ed2df171c15728f14311b91d888c387`,
summary `e3baa479111a999d03359a0642533cad1baa3a48c415609bafb9a7a7cfa18e7c`,
metadata `0bd19797953a10223bbd7183f6e8002e490b21c29b2304befee7e6618ff9ee73`,
and report `c5711b0fe06e679f889f6a7ca3812aa955c3b8c61270ef038def06b2a3f173ff`.
Its normalized trace fingerprint is
`86b37ec37d28259514453c9ecc9e7d5f12afe08118a0b04abd40e72acd147384`.
The v6, v7, and v8 paths are historical current-at-capture evidence; v9
preserves the same negative semantics and records the executed `taskset`
identity in its runtime closure.

## Card-Manifold v2 Is Numeric Sensitivity, Not Media

The current-source bundle
`assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/` compares Native
`Compact` and `FourPointPlanar` as the only intended factor in the reconstructed
26-card scene. Both modes emit all 600 requested rows with zero exact failures
and fallbacks, but both have zero strict-success rows and accepted capped groups
on every row. `Compact` records 3,495 capped groups across 5,757 attempts and a
successful terminal last group at residual `8.525678738415048e-7`.
`FourPointPlanar` records 682 capped groups across 745 attempts and exits through
the represented terminal convergence gate at residual
`0.016582575623909489`.

The directional contact hypothesis is supported: `FourPointPlanar` raises mean
contacts by `93.7983333333` and mean pair multiplicity by `1.9548548971`.
Strict convergence is not improved, and both physical and timing verdicts are
null. Raw wall time is diagnostic-only. This numeric comparison neither
validates the bounded step-6 card capture nor authorizes card media, physical
outcome, real-time, or paper-parity claims.
The unsuffixed v2 bundle is historical current-at-capture evidence. The
current v2_r3 report/index/invocation/tree hashes are
`0429b12f90e4fef534c9b707324a180eaac78829a3b3667505111228aba58d2e`,
`1703f995bd5d2c6edeb12ba936010a80609c3e952f2c782b554bd0be7ca40627`,
`6e48eb835ab9aafca0b8d4bd29acb5ec932645fb37ca6124ce3c23b3265847f0`,
and `953d8efd4d43ee2a74cede6b5e3a0a766ecd1fd8914485b50b340f515b7ecb77`;
diagnostic semantics are unchanged.

## Post-Review Small-Scene Matrix

The current output is
`/tmp/fbf_visual_evidence_postreview_20260712`. It contains eight dynamic schedules:
one combined incline scene, backspin, four turntable cells, and two Painleve
proxies. It also contains a single static frame for 10-level card-house
construction; that frame is not dynamic evidence.

For every completed step in all eight dynamic schedules, cumulative
all-group diagnostics report residual `<=1e-6`, zero accepted maximum-iteration
caps, zero exact failures, and zero boxed-LCP fallbacks. This establishes
`solver_contract_valid=true` for those captured schedules only.

| Surface | Automated artifact/solver result | Manual observation | Claim boundary |
| --- | --- | --- | --- |
| Combined incline | Finalized 21-indexed/23-physical current-source bundle; five durable stills, a 61-frame decoded schedule, and 240/240 exact attempts/solves with zero caps/failures/fallbacks | Reconstructed `mu=.4` slide and `mu=.5` stick-threshold separation visible | Partial: independent traces corroborate the threshold, but capture contacts 8 differ from aggregate trace contacts 6; strict `paper_cpu` `mu=.5` still fails at residual `1.439e-6`, and external/paper/golden/timing/real-time proof is missing |
| Backspin | Finalized 18-indexed/20-physical current-source bundle; three durable stills, MP4/GIF media, and 129/129 exact attempts/solves | Translational reversal, checker-cell changes, and the coral registration tile are visible | Partial: trace owns reversal; no signed angular direction, continuous contact, rest, landing, full-state equivalence, or external/paper/golden/timing/real-time parity |
| Turntable 2x2 | Finalized 58-indexed/60-physical author-pinned bundle with four outcome stills; all four 360-step current visual cells are solver-valid | Segmented disc/coral wedge are legible; three cells eject and `mu=.5, omega=2` remains on support through 6 s | Partial: finite-horizon result only; no zero-slip/co-rotation, full-state, approved-golden, paper-timing, real-time, or paper-parity claim, and the separate strict lane fails the retained cell at step 40 |
| Painleve pair | Historical post-review group media valid; both proxies solver-valid | `mu=0.5` upright and `mu=0.55` tumble visible | Superseded as the durable source by the finalized repository proxy bundle below; no author parity |
| 10-level construction | Static frame only | Construction layout visible | No dynamic or solver-outcome claim |
| Author five-level construction | Finalized 12-indexed/14-physical current-source bundle; step-zero capture and bound manual inspection | Public-author default five-level, 40-card configuration and four suspended cubes visible | Construction only: zero simulation substeps; no release, standing, trajectory, solver, dynamics, physical-outcome, Fig. 6/video, timing, performance, or parity claim |

The finalized current-source incline bundle is
[`assets/paper_evidence/fig01_02_incline_current_v1/`](assets/paper_evidence/fig01_02_incline_current_v1/).
Finalization and clean-checkout verify-only pass with status
`valid_current_source_nonpaper_incline`. The directory contains 23 physical
files, of which the exact-membership index binds 21. The capture retains five
durable 660x506 stills and a 61-frame decoded 30 fps H.264 schedule. Its
70-file raw capture staging directory is pruned after sealing, so verify-only
needs no ignored staging. It records 240 exact attempts/solves,
zero accepted caps, exact failures, or boxed-LCP fallbacks, maximum residual
`9.999836962261359e-7`, and eight contacts per post-initial step.

The independent `mu=.4` and `mu=.5` tracked traces each contain 121 rows, 120
exact solves, 119 warm starts, zero fallbacks, three contacts per post-initial
step, and continuous post-initial tracked contact. The `mu=.4` box travels
`1.7686892884927794 m` downhill versus analytical
`1.7548661487418349 m`, ends at `1.7544655347780056 m/s` downhill, and has
maximum residual `9.986952135669881e-7`. The `mu=.5` box travels
`0.0008905412965980523 m`, has maximum/final stick speed
`0.001116442058867632 m/s`, and maximum residual
`9.997210606407098e-7`. Their downhill displacement separation is
`1.7677987471961814 m`.

Capture and aggregate traces are byte-identical only for the projected fields
`step`, cumulative exact solves, and cumulative boxed-LCP fallbacks, SHA-256
`f03dff0aaec5f0fa6615609c2ea97aa31f072c14c6212fffe15858ea969d88c2`.
The capture reports 8 contacts while the traces report 6 in aggregate;
`contact_count_match=false` and `contact_counts_compared=false`. Because the
combined renderer and independent traces also use different placements, no
state, residual, status, warm-start, per-cell, or full-trace equivalence is
claimed.

Finalized SHA-256 values are metadata
`7a5f973a9b7264911058ec91e253dfcf5d72a7ec46fa7020df0020af1a259b7d`,
index `b758bd28965bf9a96be7668c0dbb738b72c1493d83f195a3e726ae891f8f6e85`,
manual inspection
`3c3af65d62c629ae836302910a2fe7f928ab398628f280221f6d0b5d94d5a848`,
trace summary
`4df130e878f1e58d478870c8f132ee165061a752520e926f44c331d32f14f20d`,
verification
`2681073ee44f7fecd2782081826b414ee6541bb930149ced91f60a17dc2416d5`,
report `f75efcd40bd0452bcbdc7bbc82eee0fdbd78d7a1059f974e83999467b1688fa5`,
panel `f9f211fb376c97d98bccc67806ba3e1c9905d7d27764c794e1c480af7b4df9d3`,
MP4 `ad6d00ae6a614b0edbc836396c30621b589295a0eec826d060663f6782cee3f9`,
and traces
`449acf19feef2e0aa7fb04bb9f45f865727ba59f626b3964114cd900169ecd8a`
and
`2b30e8033b123876ad1cdea755741fd230a4d72d3747e55b907e6427962659c5`.
Bound finalizer/test, runner/test, demo source/binary, trace source/binary,
fixture source, and libdart hashes are
`705da2a308697b4b4b923894d10f23622310e024aeab49862a24593c79142e23`,
`02f3800cf7cf5df5d85b1950512e78df6e413fea42cfa89880775f010d208e1c`,
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`,
`6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`,
`84fd1330a13d548760e537f9734790ab1b5c91fcda3e446bf76b9b36f3e1aa99`,
`d838f23e9fb04224ff194658bd6e53d8cbb95ac94ce6676e54c49b8ea25916e4`,
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`,
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`,
`a4bed7372213d544fef62923b88bc9accee7086165d7b3cb20245bee8bade05f`,
and
`8fae2320858e49fdda309d89df8cb1158c1cc5dc11d345e14f5adca0ff63cf3d`.

Keep this `dart_best` capture/trace lane separate from strict
`paper_cpu`/Native evidence. Strict `mu=.5` moves only `8.63436433e-7 m`, but
one accepted cap per repetition raises maximum residual to
`1.4392081500753078e-6`; its strict solver/local-real-time contract remains
failed. Fig. 1, Fig. 2, and video.03 remain `partial` without the full friction
sweep/plot, matched external rows, approved source golden/diff, paper contact
count, full 11 s semantic edit, paper timing, and real-time parity.

The finalized current-source backspin bundle is
[`assets/paper_evidence/fig03_backspin_current_v3/`](assets/paper_evidence/fig03_backspin_current_v3/).
Its index binds 18 artifacts in a 20-file physical directory. The MP4/GIF
preserve the full motion schedule, three durable stills retain steps 0, 1, and
2, and the 140-file raw capture staging directory is pruned after sealing.
Clean-checkout verify-only needs no ignored staging. The capture has 129 exact
attempts/solves, with zero accepted caps, exact failures, or boxed-LCP
fallbacks and maximum residual `9.96497154974839e-7`. The separate trace
maximum is `9.964971544991853e-7`; maximum forward travel is
`x=1.5959314363310166` at step 48, `vx` first becomes negative at step 49,
and the final state is `x=-2.9362508912363654`,
`vx=-6.628158971623909`. Step 120 is the sole contact-free post-initial step.

The renderer applies a high-contrast 6x4 ivory/charcoal checker texture with
one coral registration tile through a visual-only UV `MeshShape` under
`VisualAspect`. The physical `SphereShape` remains unchanged and continues to
own collision, dynamics, inertia, and friction.

Trace and capture solver/contact projections are byte-identical at SHA-256
`973d544311bac3b5927cc73b335b1a375d0339403a2f713707bb928076aa2b22`.
Finalized SHA-256 values are metadata
`a42ce0521a7c2af31662eff6a000ef5e68fe8bddf631d4f323be1ea8230c25a7`,
index `429a0888fa7a002cbc0c93e708569e4bf8e18195421c9663d5ce3b3a1968ab7f`,
manual inspection `b86c596da631503a3831fd55adeb934505cf758dfc8a612d0a0f32b2a55067cb`,
trace summary `0f6221fd32742b849e9ac79ec750de71b46ec24ba66b8d6e5a0e25048223ab48`,
verification `fb9de609e52df648465dc0dbe73af7fbbd1b98c1c4671cde504200ff72c15c01`,
trace `dc205297fa4cbffa1b497f12507b919f344bbee45a5820ae46145e0ed91bbd98`,
panel `72bf8d6098e8fb17b98bbedeaa00cc539866cf570d91d725f77dc75f1971b067`,
MP4 `7d4606f4da0a57ffbdfa0528906b21a20d7e1a4e47a6e7eb5387242aecc71928`,
GIF `773365f624ba1326855f2ad99c0196f761ab776001da568f7cdaa84054adacc8`,
and report `f9178c14c1930361afc1d97cb5bf08afe04d3fd451e8a94144b02bb0b46e61cf`.
Manual inspection passes the checker texture and coral registration tile as
legible.

At `-200 rad/s`, 30/15 fps sampling can alias and cannot prove signed angular
direction. The contact-free step rules out continuous contact; neither rest
nor an airborne landing phase is proven. The rendered demo and CSV exporter
are separate scenes and are not full-state trace-equivalent. External-solver,
paper, approved-golden, timing, and real-time parity remain unproven. Both
`fig.03` and `video.02_backspin` remain `partial`.

The finalized author-pinned turntable bundle is
[`assets/paper_evidence/fig04_turntable_author_current_v1/`](assets/paper_evidence/fig04_turntable_author_current_v1/).
It pins author commit `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, binds 58
indexed artifacts in a 60-file physical directory, and preserves the source
order `mu=.2/omega=2`,
`mu=.2/omega=5`, `mu=.5/omega=2`, `mu=.5/omega=5`. The synchronized 2x2
clip is 1320x1060 at 30 fps with 181 decoded frames over 6.033333 s.

All four current `dart_best`/Native `FourPointPlanar` visual runs complete 360
steps with valid solver contracts and no fallbacks. Their exact six-second
outcomes are ejected, ejected, retained on support through 6 s, ejected.
Retention does not prove zero slip, perfect sticking, co-rotation, or behavior
beyond 6 s. Manual inspection passes the segmented visual disc, one coral
registration wedge, labels, and order. The physical cylinder remains the sole
collision and dynamics geometry.
Four durable timeline-selected outcome stills bind steps 136, 120, 360, and
90 in source order. Capture staging is pruned after sealing;
clean-checkout verify-only needs no ignored files.

All four capture/trace projections are byte-identical over `step`, `contacts`,
`exact_solves`, `warm_starts`, `boxed_lcp_fallbacks`, and `status`; this is not
full-state equivalence. The separate strict `paper_cpu_native` lane has no
capture comparison and cannot be substituted: `mu=.5, omega=2` fails at step
40 with residual `7.407835021099202e-6`, while the other three strict rows
pass.

Status is `valid_author_source_pinned_nonpaper_turntable_matrix`, with current
visual artifact, solver, physical, manual-inspection, and pass gates true.
Core SHA-256 values are report
`930cc12b95ab78c6e61d084064b584f5872633f2432e434c68d071818cec7fb1`,
index `209b677ce35e2b9c11248ab414727016394831084881a5e9c2866f68b51f6cdf`,
metadata `854ef0c8aad75b4b200f512951d56b4dbef7aa82c46cc5e82123f0583dcc6ac5`,
verification `455da7686eaeeb989e0d122c4724ea66ff161e8d652230eff9fb8ad20590bacd`,
and group clip `b241463658e6b48dfb2c74815e85317c2f0eccd46a7f7ab978b8f2701ce80d6d`.
Author-spec/visual-OBJ/visual-MTL/manual-inspection hashes are
`1680cd8351fa62937c0318826f7abc75917234cb3888f983acce06f13698bc6c`,
`bc86f1ef1f5fae1510f23b1586ae20efe788c499373370a66af81b06818f1b14`,
`619352b9ac14e89a4d467dde867019e0d01540b6f11852df565f23fb26a01752`,
and `095652d3df70a144be31be49b0e25b3265df54a3815df21742333d5fdfb4529a`.
Current visual runner/test bindings are
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`
and `6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`.
This is not paper-comparable, approved-golden, timing, or real-time evidence.

The finalized author card-house construction bundle is
[`assets/paper_evidence/card_house_author_5_construction_current_v1/`](assets/paper_evidence/card_house_author_5_construction_current_v1/).
It has 12 indexed artifacts / 14 physical files and shows the public-author
default five-level, 40-card construction with four suspended cubes at step
zero. Its index, metadata, and manual-inspection hashes are
`d6cbc6f9600b8bc5c3094dd85974eae8e71d64a9e3d6e99c1783ace36be9741d`,
`b97ac795c9368f2632fe422f975914f412e8d1cb0667023e8d83c6224547df00`,
and `7bc672e9dd95b52853c5c7e56680190d564fc9514add9b263de0c33c3f94e2a4`.
It executes zero simulation substeps and is construction-only: no release,
standing, trajectory, solver, contact-dynamics, physical-outcome, historical
four-level/26-card trajectory, Fig. 6/video, timing, performance, or parity
claim.

The quantitative CPU evidence is authoritative for
`physical_outcome_valid`. The visual observations above are recorded under
`manual_inspected`; they do not promote a failing row.

## Synchronized Group Outputs

Every listed group video fully decodes and has exact probed dimensions, frame
rate, and frame count:

| Group | Layout | Video contract | SHA-256 |
| --- | --- | --- | --- |
| Turntable | 2x2 | 1320x1060, 30 fps, 121 frames | `e5bf08986d6e70bdf25797e66c958c9ac947f9b2dc863b30d37afe1efd29f11e` |
| Turntable, finalized author-pinned v1 | 2x2 | 1320x1060, 30 fps, 181 decoded frames | `b241463658e6b48dfb2c74815e85317c2f0eccd46a7f7ab978b8f2701ce80d6d` |
| Painleve, historical `/tmp` matrix | Side by side | 1320x530, 30 fps, 76 frames | `d25a93abf964df707e3c10c30202bf75d76e0b216b9f2301b393dca63322afd0` |
| Painleve, finalized repository v1 | Side by side | 1320x530, 30 fps, 76 frames | `dd4cdda2410b71f8e86035f1c0ff278f9dda77133c702e409ee5d533da443a4b` |
| Incline, finalized repository v1 | Combined view | 660x506, 30 fps, 61 frames | `ad6d00ae6a614b0edbc836396c30621b589295a0eec826d060663f6782cee3f9` |
| Backspin, finalized repository v3 | Single view | 1300x506, 30 fps, 66 decoded frames | `7d4606f4da0a57ffbdfa0528906b21a20d7e1a4e47a6e7eb5387242aecc71928` |

The member order, labels, synchronized duration, source-frame hashes, and
composite hashes are recorded in the group metadata. These integrity checks
support `artifact_valid`; they do not establish author-scene or paper parity.

The historical matrix was captured from the post-review binary, then fully
decoded and manually inspected. Its incline row is superseded by the finalized
repository v1 bundle, which binds the current-source combined clip/panel,
manual inspection, and two independent tracked traces while explicitly
excluding mismatched contact counts from equivalence. The finalized Painleve
v1 bundle separately binds both current-source member clips, hardened per-step
sidecars, the paired clip and panel, manual inspection, and separate tracked
physical traces in a 27-artifact exact-membership index within 29 physical
files. The finalized backspin v3 bundle similarly binds its current-source
trace, three durable stills, panel, MP4/GIF, manual inspection, and 18-artifact
index within 20 physical files. The author-pinned turntable
bundle separately binds its four source-ordered cells, synchronized media,
manual inspection, and separated visual/strict lanes. In the Painleve and
backspin cases the rendered demo and tracked trace are different
implementations and are not trace-equivalent.

On that matrix's recorded `dart-demos` binary
(`6ac1b6fb167bdcdbfbf2fea831eac7755a751ed72bb6f1d55e4f80a4d4e25165`),
the selected nine retained schedules and their exact group outputs pass the
workflow verifier. The broader `all-runnable` verification fails closed at the
missing `card_house_26/timeline.json`, so the session remains partial and is
not promoted to a repository deliverable.

## Bounded Long-Scene Attempts

Each long schedule was attempted with a 120 s bound. None produced a valid
completed sidecar/media bundle:

| Schedule | Requested duration | Furthest observation | Result predicates |
| --- | ---: | --- | --- |
| 26-card settle/projectile | 600 steps | Step 6/600 | `artifact_valid=false`; incomplete |
| 25-stone oriented-box reconstructed arch | 360 steps | Step 24/360; visibly collapsed at 0.4 s | `artifact_valid=false`; `physical_outcome_valid=false` |
| 101-stone reconstructed arch | 600 steps | Step 120/600; visibly collapsed at 2 s | `artifact_valid=false`; `physical_outcome_valid=false` |
| Dynamic 10-level card house | Long schedule | No completed step 1 | `artifact_valid=false`; no outcome evidence |

There are no valid long sidecars or media for these four bounded schedules.
The attempts are negative evidence and must not be replaced by one-step,
reduced-scene, or boxed-LCP captures under the same label. The separate
literal-wedge standing capture follows a different declared scene contract.

The positive literal-wedge result is a separate scene contract and does not
relabel or invalidate the oriented-box visual failure. Its standing capture
also cannot be substituted for missing crown-impact media.

The retained prior-source strict CPU card trajectory, repository-archived at
`assets/dart_cpu_evidence/2026-07-12_prior_source_paper_cpu_card600_negative/`,
independently fails at step 89/600 with terminal residual `1.8612e-2`, one
exact failure, and zero fallbacks. The current-source card-v2 diagnostic emits
600 rows in both modes,
but neither trajectory is strict and both physical verdicts remain null. A card
video cannot be paper-valid while those trajectory contracts remain failed or
non-strict.

The current exact-inertia literal 101-stone v1 artifact is
`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v7/`. It fails
closed on step 1 after 5,000 outer iterations with residual
`0.7815364614352474`, one exact failure, and zero fallbacks. Its dynamic
`FourPointPlanar` row reports 400 contacts and 100 constraint pairs; a separate
Compact collision-only probe proves the constructed time-zero graph of 100
adjacent-stone pairs plus two springer-ground pairs. The v7 one-step
FourPointPlanar companion resolves the failed step-1 pre-solve graph as exactly
100 adjacent pairs and 400 contacts, four contacts per pair, with zero
non-adjacent and zero ground pairs. Its aggregates and residual match the
frozen trace. The companion accepts the capped iterate and does not follow the
frozen trace participant-affinity contract, so solver-taxonomy and affinity
equivalence remain false. This numeric identity result is limited to the
already-failed prefix and does not validate the oriented-box media attempt or
supply source equivalence, a valid trajectory, standing/physical outcome,
timing, visual evidence, long-run behavior, or paper parity. Raw, metadata,
and report hashes are
`fc0705ff497b44658dfe186ed27c50b2ab9d19a268f20cf50875a3155d1dc07d`,
`770d662f7021320982700fe699266bc94d2448679c8c4dc60bd127c660facb8a`,
and `1ba3987af896f68401faa1153f6b388aed38b087214d07592f5a060cab429e2a`.
The current v7 whole-tree hash is
`e65107a6d2e2880742e02d01949bc4b3f9adb2f48b6be552a720fa83c9bcc4b3`.
The v3 bundle was superseded after additive instrumentation, and v4 is
historical current-at-capture evidence. The unchanged command was rebaselined
as v5 and again as v6 after the current-build libdart identity advanced. V7
adds the identity-resolved one-step dynamics companion; the frozen trace and
scientific result are unchanged.

## Source Identity Audit

The source workflow pins:

| Source | SHA-256 |
| --- | --- |
| Combined 82 s paper video | `d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794` |
| Project teaser | `99527da7a84f7b9ac0031f794d9b16adadfba846d2165e7da22fd51d986c8db0` |
| Paper PDF | `af7cb8df58288f4323fa4340e1590b09643b8702116a6c47cefe7ffa9a51e2f4` |

It validates nine contiguous semantic segments across all 82 s:

| Segment | Time | Source content |
| --- | ---: | --- |
| Title | 0-2 s | Title and authors |
| Backspin | 2-24 s | Real time, slow motion, and solver comparison |
| Incline | 24-35 s | `mu=0.4` slide and `mu=0.5` stick |
| Turntable | 35-50 s | Four simultaneous cells |
| Painleve | 50-60 s | Two coefficient outcomes |
| Four-level card house | 60-67 s | Settle and projectile impact |
| 25-stone arch | 67-74 s | Settle and crown impact |
| 101-stone arch | 74-80 s | Long-run comparison |
| Closing | 80-82 s | Closing card |

This proves source identity, decodeability, and segment coverage. It does not
prove that a DART rendering matches the source geometry, camera, parameters,
trajectory, solver convergence, or physical outcome.

The public author solver, six current scene/configuration sources,
dependencies, and MuJoCo/Kamino runners are available and pinned; source-port
and matched-run work are internal. Current author invocations were
independently run and preserved. Only masonry-arch meshes are shipped, so the
historical renderer/cameras/materials/approved goldens, original
invocation/timing logs and warmup/aggregation attestation, and exact Apple
silicon host remain unavailable.

## Geometry Boundary

The production arch visuals use weighted-catenary oriented `BoxShape` stones,
not literal tapered voussoirs. The 25-stone natural production manifold has 96
contacts and the GUI-reduced profile has 48. The 101-stone full profile reaches
the 512 cap and its GUI-reduced profile has 38. The exact-inertia literal-wedge
collision probe remains collision-only. A distinct exact-inertia dynamic trace
now supplies a valid 600-step numeric standing claim, and the finalized
current-source capture validates its standing presentation. Neither artifact
supplies a crown-impact or author-parity claim. The separate impact v1 lane is
numeric-only negative evidence and has no media. The literal 101-stone v7 lane
is an identity-resolved step-1 failed-prefix negative, and the card-v2 lane is
non-strict numeric sensitivity; neither supplies valid long media.

## Reproduction Entry Points

Use the current built executable and retain negative exit status:

```bash
./build/default/cpp/Release/bin/dart-demos --list-scenes
./build/default/cpp/Release/bin/dart-demos --verify-fbf-scene-docs
./build/default/cpp/Release/bin/dart-demos --cycle-scenes --frames 1

.pixi/envs/default/bin/python scripts/run_fbf_visual_evidence.py --help

.pixi/envs/default/bin/python scripts/finalize_fbf_incline_visual.py \
  --bundle \
    docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/fig01_02_incline_current_v1 \
  --verify-only
```

The source-audit mode requires the pinned local video, teaser, and paper paths
shown by `--help` and rejects hash or segment-boundary drift.

## Remaining Visual Gates

- Preserve the finalized incline, Painleve, backspin, and turntable bundles
  within their declared lane-specific boundaries; promote only the remaining
  small-matrix rows through validated immutable publication bundles.
- Keep manual observations separate from quantitative physical-outcome status.
- Preserve the current 9/9 physical-classifier versus 7/9 strict-solver
  distinction. The finalized incline visual/trace lane does not promote the
  strict `paper_cpu` `mu=.5` residual failure, and the turntable visual lane
  does not erase its separate strict failure.
- Produce valid full-duration card/arch media only after every requested step
  satisfies the strict exact-solver and physical-outcome contracts.
- Preserve card-manifold v2 as diagnostic-only sensitivity evidence; its two
  non-strict trajectories and raw wall times cannot promote media.
- Keep the frozen crown-impact v1 negative unchanged. Any revised impact
  hypothesis must be separately preregistered, pass its numeric gates, and then
  receive separately labeled media; do not relabel the finalized no-projectile
  standing bundle as impact evidence.
- Port and capture the pinned public author scenes and define an approved
  source-golden/diff policy. The author repository supports USD export but does
  not ship the historical cameras, materials, or approved frame goldens, so
  keep that visual-comparison gate explicitly blocked until replacements are
  approved.

Until those gates close, the visual result is useful reconstructed evidence,
not final paper parity.
