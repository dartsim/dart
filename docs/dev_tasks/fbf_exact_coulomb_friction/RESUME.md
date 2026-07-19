# Resume: FBF exact Coulomb friction

## Current Checkpoint

This task is active and incomplete as of 2026-07-19. Start with
[AGENT_CONTINUATION.md](AGENT_CONTINUATION.md); it is the authoritative truth
ledger.

Mandatory status answer:

```text
No, the DART reconstruction does not yet cover or match every paper test,
benchmark, GUI example, physical outcome, or performance result.
```

Do not retire the task folder. Do not commit, push, update a PR, rerun remote
CI, or mutate GitHub without explicit user approval. Never add AI or tool
attribution to a commit or PR.

The newest numeric scientific addition is the pinned-author incline sweep.
It preserves all three current public runner lanes over the Figure 1 friction
grid, but proves neither DART/source parity nor historical paper parity. The
separate sealed author masonry-arch negative also remains current. Neither
packet changes the overall 29-row audit: 24 partial, 5 blocked, 0 complete,
and the visual bundle count remains six.

## Read First

1. `AGENTS.md`
2. `docs/dev_tasks/fbf_exact_coulomb_friction/AGENT_CONTINUATION.md`
3. `docs/dev_tasks/fbf_exact_coulomb_friction/README.md`
4. `docs/dev_tasks/fbf_exact_coulomb_friction/paper-parity-matrix.md`
5. `docs/dev_tasks/fbf_exact_coulomb_friction/PR_REPORT.md`
6. `docs/dev_tasks/fbf_exact_coulomb_friction/gui-capture-report.md`
7. `docs/dev_tasks/fbf_exact_coulomb_friction/residual-history-report.md`
8. `docs/dev_tasks/fbf_exact_coulomb_friction/paper-evidence-manifest.json`

Inspect and preserve any existing worktree changes. The durable branch state is:

```text
branch: research/fbf-friction-release620
target synchronization verified through: 75306efe770
topic HEAD/divergence/PR/CI/review state: verify live
```

Run `git fetch origin`, verify `origin/release-6.20` is an ancestor of the
topic, and query PR #3377 before reporting mutable state. Do not assume a clean
branch or silently switch checkouts.

## Source And Historical Comparison Boundary

The earlier 2026-07-13 observation that the project page said
`Code (coming soon)` and the named repository returned 404 is superseded. The
MIT-licensed author reference is public and pinned at
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`. Its solver, six runnable scenes,
current configuration defaults, exact local kernel, warm-start policy, gamma
controller, pinned dependencies, and optional MuJoCo/Kamino runners are now
source-auditable.

Source-port and matched-run work are therefore internal, not externally
blocked. Current author invocations were independently run and preserved.
They do not fill the historical gap: only masonry-arch mesh assets are shipped,
and the public repository still does not provide the historical renderer,
cameras, materials, approved goldens, original invocation/timing logs and
warmup/aggregation attestation, or exact Apple-silicon host. Local work is
reconstructed float64 DART on x86-64 Linux, so every paper timing target and
verdict remains null.

## What Is Now Proven Locally

The current-source small `paper_cpu`/Native matrix is repository-finalized at
[`assets/dart_cpu_evidence/2026-07-19_current_source_paper_cpu_small_r7/`](assets/dart_cpu_evidence/2026-07-19_current_source_paper_cpu_small_r7/).
It binds 60 indexed artifacts, 27 complete CPU-4 invocations, and 5,220 raw
rows. All 9 reconstructed physical classifiers pass, while 7/9 scenarios pass
the strict solver and local real-time contracts. Incline `mu=.5` is physically
valid but reaches maximum residual `1.4392081500753078e-6`; turntable `mu=.5,
omega=5` passes the physical ejection classifier in all three repetitions, but
all three measured processes fail and the maximum residual is
`3.050386527672585e-4`. Both strict negatives contain one
`max_iterations_accepted` row per repetition. The bundle used zero warmups and
is not paper-comparable, so its paper timing targets and verdicts remain null.
Core r7 SHA-256 values are report
`008bc94667893cd26bfc04a720caca3d3a5703601c8739bc06856f93818c63d5`,
artifact index
`06594c1e1cc3c6858bc78a80630aefb0e85eeef92204b0a09b99425411c3ebeb`,
summary JSON
`9137f1b2db0909a96897632a66617f2cfa58476a369d2adfe232b73e46cd0fa2`,
and metadata
`e227d7aef3b273ff81385f227f9e6766a7e40a622bc4298e1807b7c2068bc417`.

The authoritative performance/scaling bundle for the reconstructed literal
arch is
[`assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore/`](assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore/).
For the reconstructed literal 25-stone Native FourPointPlanar wedge arch it
records one warmup plus three measured 600-step trajectories at each of one
and four threads.

- Each thread count has 1,800 measured steps.
- All measured steps have 96 contacts and 24 colliding body pairs.
- The schedule remains 24 manifolds, 3 colors, and width 8.
- Exact FBF succeeds every step; maximum residual is
  `9.999807145410957e-7`; exact failures, accepted caps, and fallbacks are
  zero.
- Physical outcomes pass and the one- and four-thread trajectories are
  identical.
- One thread: mean `6.122883343333333 ms`, median `2.4966535 ms`, p95
  `21.663236899999994 ms`, max `287.473818 ms`.
- Four threads: mean `4.26939745 ms`, median `1.9047965 ms`, p95
  `14.396602399999995 ms`, max `180.504588 ms`.
- Validated matched-work speedup is `1.4341328993236115x`.

Both means meet the local 60 Hz target. Neither thread count meets an
every-step 60 Hz deadline. This is a non-paper, reconstructed float64 x86-64
result: it proves local mean-real-time throughput and multicore scaling for
the declared scene, not apples-to-apples paper timing, an impact result, or
media parity.

Schema v8 preserves the default 83-column trace. Its newline-terminated header
SHA-256 remains
`396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50`.
The colored contract uses a separate 95-column trace. Its newline-terminated
header SHA-256 is
`424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5`.
Its two added invariant columns record maximum all-stone displacement and
minimum all-stone orientation alignment from constructed t0, closing the old
tracked-crown-only outcome blind spot.

Both thread counts have measured-work fingerprint
`9d8df2edba609314432ff17f63768fded23577703537040d75ce082ab4233a36`.
The trace binary SHA-256 is
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`.
Bundle SHA-256 values are:

- `artifact-index.json`: `a60899cc12a53f03424c02c2647f233d5c75f3ccce367ea2a604f9a7ee18bf11`;
- `metadata.json`: `5507ed80140a146d4247c4f0b05fd9503879ce79856189a15759b101c2cab789`;
- `invocations.json`: `719ba3491fad8ac12aa290faa401d0c46b10af52da7eb7a28487a5b2aac44812`;
- `raw.csv`: `91a379f832ca52bbce7011308640012d9f199b98e39cba4eaf661cf17fb0f017`;
- `summary.csv`: `3c08b251c340aa9ae7909d715ebcc2985ba34350a6e03b0b6b351d82d1ec82a2`;
- `summary.json`: `304736d6b871c4498a6c0de4c4448635e712fb3a8a455dd54d9ffefeef2ec170`; and
- `REPORT.md`: `4e55d7f4dc0532ab15b86cfeb72ea9526d3f9b63194dc5a25f3974186b1a7ba7`.

The current-source visual bundle is
[`assets/paper_evidence/fig07_arch25_literal/`](assets/paper_evidence/fig07_arch25_literal/).
It records 600 exact steps with zero failures/fallbacks and worst residual
`9.9998071454109575e-7`. The independently rendered capture and current trace
compare equal across 600 rows with no integer mismatches and zero difference
in every compared floating-point field. Maximum all-stone displacement from
constructed t0 is `5.431169776791696e-6 m`, and minimum orientation alignment
is `0.9999999999111284`.

The finalized bundle has 19 indexed artifacts / 21 physical files and five
durable 1280x720 stills. Its five-panel timeline, durable stills, 61-frame
decoded clip schedule, and separately decoded video midpoint passed manual
inspection. The H.264 clip compresses 10 simulation seconds into 6.1 playback
seconds at 10 fps, a `1.639344262295082x` time-lapse rather than a real-time
playback claim. Finalization retained the immutable pending metadata and
verified its hash DAG before writing final provenance, index, and metadata.
The capture source, binary, numeric trajectory, and decoded media remain
unchanged. The 70-file raw capture staging directory was pruned after sealing;
provenance binds its schedule and hashes, and clean-checkout verify-only needs
no ignored staging. The bundle was freshly revalidated against the final
current trace and Native source after later additive scenario work.
Revalidation regenerated the standing reference with the current executable
and again proved all 600 standing rows zero-difference; original trace hashes
remain separate provenance bindings.

Final visual SHA-256 identity:

- 12-function/16-case visual unit file: `f06ba295006cf7e1f0fb692fc5eacf62e4e7f3be89bd166c041752d915779bdd`;
- driver: `0f4e27b0c58e9dd3774c6be48ad4c70a857e2956fd81f11710837561f08f7243`;
- C++ capture source: `c3efeac52d02a0c373f733598db81e545d062195ba6e96c2a65bcb607cd0207f`;
- capture binary: `8b3cad15220c8fdb69c3ebdf7fa3923fda6fd812a49d0e54c8aeb07e62f0a7e9`;
- current trace source: `b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`;
- current trace binary: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`;
- original trace source: `2e55298496d76a3a3fe002fcfaf5391332fdcf2da813b5df400affbea431e7cd`;
- original trace binary: `0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`;
- final metadata: `b217b0a43200ca558f09776e464d50f0f9dc5c5a175bcfa66ae1b656076871d1`;
- retained pending metadata: `e300103dbb950b97e217ca0bea6f1c1dc78598ddf485cdaca26cc7ad58835b3c`;
- manual inspection: `4b52bcd26ed88d184bd695d77070420aeec2c95edfb203efda7ae6241150c343`;
- final provenance: `eca349842e4121584145cd039a554aa13c51e5242ff924b37f5f49a9dee0ac2f`;
- final artifact index: `4d173c78e2c6584d6d8a85867160cc0c0687fe0f35e6878f66b0cfcd7c1c15ee`;
- current reference trace: `0e41af230fb4c27143966c3f8522702280e6746b31e84155a9a047326e46dbb3`;
- trace equivalence: `537f6e497b6fb6810240f006db47c91304581741370e319787635e6bcbfdd2e3`;
- frame validation: `a00dd7f4971756589ea834ac1a04f302ba8b7b7af96300d98e9cbc17413b3330`;
- video: `e0c111f96511bbda41af145787bd9f68782b8c9e8a909d45eb03d3c3d647feb1`;
- timeline: `926380c5c5768cc9ca1e414cbfe2631584daa99387090926d5a1f035d62b6ec9`; and
- decoded midpoint: `75a88bb317441ed71803f784b2eaa099211c4e026f8547d6fe5f2ff3ee95909f`.

This proves only that the reconstructed float64 x86-64 literal arch remains
standing and visually stable in the declared no-projectile run. It does not
prove projectile impact, paper or author-scene parity, a 101-stone result, or
an author-golden image match.

### Finalized Fig. 01/02 And Video.03 Incline Evidence

Preserve
[`assets/paper_evidence/fig01_02_incline_current_v1/`](assets/paper_evidence/fig01_02_incline_current_v1/).
Finalization and clean-checkout verify-only pass with status
`valid_current_source_nonpaper_incline`. The directory has 23 physical files
and a 21-artifact exact-membership index; the index deliberately excludes only
itself and `metadata.json`.

- The combined 660x506, 30 fps capture retains five durable stills and a
  61-frame decoded H.264 schedule. Its 70-file raw capture staging directory
  is pruned after sealing, so verify-only needs no ignored staging. It records
  240 exact attempts/solves with zero caps, failures, or fallbacks. Its worst
  residual is `9.999836962261359e-7`, and it reports eight contacts per
  post-initial step.
- Each independent tracked trace has 121 rows, 120 exact solves, 119 warm
  starts, zero fallbacks, three contacts per post-initial step, and continuous
  post-initial tracked contact.
- `mu=.4` moves downhill `1.7686892884927794 m` against analytical
  `1.7548661487418349 m`, reaches final downhill speed
  `1.7544655347780056 m/s`, and has maximum residual
  `9.986952135669881e-7`.
- `mu=.5` moves downhill `0.0008905412965980523 m`, has maximum/final stick
  speed `0.001116442058867632 m/s`, and has maximum residual
  `9.997210606407098e-7`. The tracked displacement separation is
  `1.7677987471961814 m`.
- Only the aggregate `step`/exact-solve/fallback projections are
  byte-identical, at
  `f03dff0aaec5f0fa6615609c2ea97aa31f072c14c6212fffe15858ea969d88c2`.
  Capture contacts 8 versus aggregate trace contacts 6 are an explicit
  mismatch: `contact_count_match=false` and
  `contact_counts_compared=false`. State, residual, status, warm-start,
  per-cell, and full-trace equivalence are not claimed because the combined
  render and independent traces use different placements.

Core SHA-256 identity is metadata
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
and clip `ad6d00ae6a614b0edbc836396c30621b589295a0eec826d060663f6782cee3f9`.
The two trace hashes are
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

Keep this lane separate from the current strict `paper_cpu`/Native matrix.
There, `mu=.5` passes its physical classifier with only `8.63436433e-7 m`
displacement, but one accepted cap per repetition yields maximum residual
`1.4392081500753078e-6`, so its strict solver/local-real-time contract remains
failed. Fig. 1, Fig. 2, and video.03 remain `partial` pending the full friction
sweep/plot, matched external rows, approved source golden/diff, paper contact
count, full 11 s semantic edit, paper timing, and real-time parity.

### Pinned-Author Incline Sweep Scientific Negative

Preserve
[`assets/paper_evidence/author_incline_sweep_reference_v1/`](assets/paper_evidence/author_incline_sweep_reference_v1/).
It is a numeric packet from three independent current-source CPU invocations
at pinned author commit `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, not a
new visual bundle. FBF, MuJoCo, and Kamino each run the exact grid
`mu=.3,.4,.45,.5,.55,.6,.8`, with 120 steps per cell: seven cells and 840
rows in each lane. The retained FBF histories record four contacts per FBF
step; the MuJoCo and Kamino result records contain no contact-count field.

FBF has 839/840 configured convergence flags. Its sole false row is
`mu=.55`, step 1, after 200 outer iterations, so that cell is 119/120. Of the
839 true flags, 235 are initial natural-residual shortcuts and 604 satisfy the
configured outer nonnegative `coulomb_rel < 1e-6` gate. Natural
`final_residual` is separate: 456 configured-true rows are at or below
`1e-6`, 383 configured-true rows are above it, and the configured-false row
has natural residual `3.273267262002487e-8` while its terminal
`r_coulomb` is `1.5311460572898186e-6`.

The current-run displacement projection places FBF and Kamino close together
and shows a nonmonotone MuJoCo result, but this is not full-state equivalence
or a parity verdict. First-use JIT work, always-on history collection,
ineffective warmup exclusion, and lane-dependent timer boundaries exclude all
recorded timings. No DART match, historical invocation, approved golden,
media, paper timing, or performance claim follows; Fig. 1, Fig. 2, and
video.03 remain partial.

### Finalized Painleve Proxy Visual Evidence

Use
[`assets/paper_evidence/fig05_painleve_proxy_current_v1/`](assets/paper_evidence/fig05_painleve_proxy_current_v1/)
for the durable current-source Painleve DART-side proxy bundle. It supersedes the
Painleve cells in the older session-local `/tmp` visual matrix. Finalization
and clean-checkout verify-only both pass with status
`valid_current_source_nonpaper_proxy`, 27 indexed artifacts, and 29 physical
files; ignored capture staging is not required.

- Its fail-closed index binds 27 artifacts. Each durable member clip fully
  decodes to 76 frames, and the synchronized pair is 1320x530 at 30 fps with
  76 frames. Raw capture frames are pruned after sealing.
- Each separate tracked fixture completes 150 steps and writes 151 rows with
  zero exact failures and zero boxed-LCP fallbacks; the maximum tracked
  residual is `9.998574150559113e-7`.
- The `mu=.50` trace remains upright and reaches its settled proxy at
  `x=1.298081699724907 m`, with final `up_z=0.999998178998452`.
- The `mu=.55` trace first crosses the fixture tumble threshold at step 36,
  `t=0.6000000000000002 s`, and `x=1.2597198197697048 m`, or
  `0.03836187995520213 m` before the `mu=.50` rest distance.
- Manual inspection records the `mu=.50` upright return and the `mu=.55`
  tumble/horizontal final presentation. The separately tracked traces
  corroborate the classifier; they are not trace-equivalent to the rendered
  demos.

Verified current-source source/binary SHA-256 identity: finalizer
`31b2b560a3a6a7f06e514a8bc3dce9f4b766b3c4e62fe520435bfaa1e3ba77a9`,
visual runner
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`,
visual runner test
`6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`,
demo source
`84fd1330a13d548760e537f9734790ab1b5c91fcda3e446bf76b9b36f3e1aa99`,
demo binary
`d838f23e9fb04224ff194658bd6e53d8cbb95ac94ce6676e54c49b8ea25916e4`,
trace source
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76`,
trace binary
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`,
and fixture source
`a4bed7372213d544fef62923b88bc9accee7086165d7b3cb20245bee8bade05f`.

Final bundle SHA-256 identity: metadata
`0845988dd05b18c965eba5fb5163d43dafc901ca8f30f66b01b4f37163d72f30`,
artifact index
`7880c62d39c3f47109b50394cd84e20919565b70af19c1473c618841f682ff43`,
manual inspection
`27c2632e427ec83b3612de6003cd16523fd483ab066d6618f9cdee38efdd2d0c`,
trace summary
`115b50d92338477022435911df38d53a54e880c1daed4004dedd7d8507336164`,
verification
`6fd34e54f7958c45777731ab888fe3a5e8e24d06b9fdddb96bdace269fcb515e`,
paired panel
`bef97d04d59e2937195151cbf36a0b713bb2c2bef43d111513c05885595546e2`,
paired clip
`dd4cdda2410b71f8e86035f1c0ff278f9dda77133c702e409ee5d533da443a4b`,
and report
`a04c37f9f3db428994252c8486030aa6b97b367a627fc3a1a22106dfa8a9f51f`.

The only allowed scientific claim is a manually classified, physically
corroborated DART-side proxy transition. The rendered demo and tracked fixture
are not trace-equivalent. Angular velocity is not exported, so strict
rigid-body rest is unproven. The bundle does not establish author-scene/paper
parity, faithful external-solver parity, an approved golden/diff, paper timing,
or real-time performance.

### Finalized Fig. 03 And Video.02 Backspin Evidence

Preserve the repository-finalized current-source bundle at
[`assets/paper_evidence/fig03_backspin_current_v3/`](assets/paper_evidence/fig03_backspin_current_v3/).
It binds 18 indexed artifacts in a 20-file physical directory. The MP4/GIF
preserve the full motion schedule, three durable stills retain steps 0, 1,
and 2, and the 140-file raw capture staging directory is pruned after sealing.
Clean-checkout verify-only needs no ignored staging. It records 129 exact
attempts/solves with zero accepted caps, exact failures, or boxed-LCP
fallbacks. Trace and capture maximum residuals are respectively
`9.964971544991853e-7` and `9.96497154974839e-7`. Their 131-row
solver/contact projections are byte-identical at SHA-256
`973d544311bac3b5927cc73b335b1a375d0339403a2f713707bb928076aa2b22`.

The physical trace reaches maximum `x=1.5959314363310166` at step 48, first
records negative `vx` at step 49, and ends at `x=-2.9362508912363654`,
`vx=-6.628158971623909`. Step 120 is the sole contact-free post-initial step.
The sphere uses a renderer-applied high-contrast 6x4 ivory/charcoal checker
texture with a coral registration tile. Its UV `MeshShape` is
`VisualAspect`-only; collision, inertia, friction, and dynamics still use the
unchanged physical `SphereShape`. Manual inspection passes only the texture
and registration tile's legibility. Finalized SHA-256 values are metadata
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

Do not broaden the result: `-200 rad/s` can alias at 30/15 fps, so the media
do not prove signed angular direction. The step-120 gap rules out continuous
contact; rest and an airborne landing phase are not proven. Separate demo and
CSV scenes are not full-state trace-equivalent. External-solver, paper,
approved-golden, timing, and real-time parity remain unproven. Both
`fig.03` and `video.02_backspin` remain `partial`.

### Finalized Author-Pinned Fig. 04 Turntable Evidence

Preserve
[`assets/paper_evidence/fig04_turntable_author_current_v1/`](assets/paper_evidence/fig04_turntable_author_current_v1/).
It pins the public author configuration at commit
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`, binds 58 indexed artifacts in a
60-file physical directory, and keeps the source order `mu=.2, omega=2`,
`mu=.2, omega=5`, `mu=.5, omega=2`,
`mu=.5, omega=5`. The current `dart_best`/Native `FourPointPlanar` visual lane
completes all four 360-step runs with valid solver contracts and no fallbacks.
Its exact six-second classification is ejected, ejected, retained on support
through 6 s, ejected. The retained cell does not prove zero slip, perfect
sticking, co-rotation, or an infinite-horizon outcome.

Manual inspection passes the segmented disc, one coral registration wedge,
labels, and source order. The disc and wedge are visual-only; the physical
cylinder remains the sole collision/dynamics geometry. All four six-field
solver/contact projections are byte-identical between capture and trace, but
that is not full-state equivalence. Keep the strict `paper_cpu_native` lane
separate: it has no capture comparison, and `mu=.5, omega=2` fails at step 40
with residual `7.407835021099202e-6` while its other three rows pass.
Four durable, timeline-selected outcome stills bind steps 136, 120, 360, and
90 in source order. Capture staging is pruned after sealing;
clean-checkout verify-only needs no ignored files.

Status is `valid_author_source_pinned_nonpaper_turntable_matrix`, with the
current visual lane artifact-, solver-, physical-, and manual-inspection gates
passing. Core SHA-256 values are report
`930cc12b95ab78c6e61d084064b584f5872633f2432e434c68d071818cec7fb1`,
index `209b677ce35e2b9c11248ab414727016394831084881a5e9c2866f68b51f6cdf`,
metadata `854ef0c8aad75b4b200f512951d56b4dbef7aa82c46cc5e82123f0583dcc6ac5`,
verification `455da7686eaeeb989e0d122c4724ea66ff161e8d652230eff9fb8ad20590bacd`,
group clip `b241463658e6b48dfb2c74815e85317c2f0eccd46a7f7ab978b8f2701ce80d6d`.
Author-spec/visual-OBJ/visual-MTL/manual-inspection hashes are
`1680cd8351fa62937c0318826f7abc75917234cb3888f983acce06f13698bc6c`,
`bc86f1ef1f5fae1510f23b1586ae20efe788c499373370a66af81b06818f1b14`,
`619352b9ac14e89a4d467dde867019e0d01540b6f11852df565f23fb26a01752`,
and `095652d3df70a144be31be49b0e25b3265df54a3815df21742333d5fdfb4529a`.
Current visual runner/test bindings are
`d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432`
and `6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e`.
This is not paper-comparable, approved-golden, paper-timing, or real-time
evidence.

### Finalized Author Card-House Construction Evidence

Preserve
[`assets/paper_evidence/card_house_author_5_construction_current_v1/`](assets/paper_evidence/card_house_author_5_construction_current_v1/).
It has 12 indexed artifacts / 14 physical files and shows the public-author
default five-level, 40-card construction with four suspended cubes at step
zero. Index, metadata, and manual-inspection SHA-256 values are
`d6cbc6f9600b8bc5c3094dd85974eae8e71d64a9e3d6e99c1783ace36be9741d`,
`b97ac795c9368f2632fe422f975914f412e8d1cb0667023e8d83c6224547df00`,
and `7bc672e9dd95b52853c5c7e56680190d564fc9514add9b263de0c33c3f94e2a4`.

The boundary is construction-only: zero simulation substeps, with no release,
standing, trajectory, solver, contact-dynamics, physical-outcome, historical
four-level/26-card trajectory, Fig. 6/video, timing, performance, or parity
claim.

### Pinned-Author Masonry-Arch Scientific Negative

Preserve
[`assets/paper_evidence/author_masonry_arch_reference_v1/`](assets/paper_evidence/author_masonry_arch_reference_v1/).
The pinned author run uses 500 frames, four substeps per frame, and releases
three cubes at frame 400 / substep 1,600. The source default is 400 frames with
`drop_frame=400`, which never releases the cubes; this is a newly declared
current-source diagnostic, not a historical or paper invocation.

A deterministic projection represents every one of the 2,000 substeps and is
lossless with respect to the declared claim fields. The 382,753,953-byte raw
source history (SHA-256 `cec0e4b86837e7542c498c7ddad40538983ec023332b88ebddee7766997e3ac1`)
is size/hash-bound but omitted. In the projection, 40 author convergence flags
are true through the initial natural-residual shortcut, 117 are true through
the configured outer nonnegative `coulomb_rel < 1e-6` gate, and 1,843 outer
solves are false. The separate `final_residual` field is a natural residual;
only 47 values are at or below `1e-6`. Release substep 1,600 is
nonconverged with 100 contacts and natural residual
`0.017456069692858667`; final substep 1,999 is nonconverged with 108 contacts
and residual `0.5161195175386001`. Contact-count growth after release is not
pair-identified contact evidence because the projection contains no pair
identities.

Exit zero proves only that the author diagnostic completed and its artifacts
were preserved. The DART shared spec is configuration-only and executes no
dynamics; source collision/contact-gap/backend/float32 semantics are not
implemented. Do not infer DART or cross-solver dynamics, trajectory, outcome,
Fig. 7/video.07, timing, repeatability, contact-pair, or media parity.

The separate frozen crown-impact v1 contract and result are in
[LITERAL_CROWN_IMPACT_V1.md](LITERAL_CROWN_IMPACT_V1.md) and
[`assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/`](assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/).
It completes the unchanged 600-step standing prefix, injects three fixed
35 mm cubes, and runs 120 post-launch steps. The prefix matches the standalone
standing trace across 88 eligible fields with zero mismatches; first arch
contact occurs at step 607 before first ground contact at 616. The run stays
finite with zero exact failures/fallbacks, but fails closed with five accepted
caps, worst residual `9.154531704265396e-5`, final arch displacement
`0.07093964431215687 m > 0.07 m`, and far-field displacement
`0.060523747030465196 m > 0.007 m`. This is durable scientific-negative
evidence with `impact_claim_passed=false`; no parameter or threshold was
tuned. The current runner recomputes all finite gates, requires post-launch exact
progress, pins the normalized fingerprint and frozen preregistration hash, and
stores the independent 600-step by 88-field standing reference.

Final negative-bundle hashes: runner
`622b388142fed881191dcb6efb103266eb2ea1e3ffbadaf402f63b89c558bb67`,
preregistration `4cdea674f366fc2d18eadf11ef4333d491786d5d85e3fc16fa611ea7dede3f37`,
fingerprint `86b37ec37d28259514453c9ecc9e7d5f12afe08118a0b04abd40e72acd147384`,
reference `22fabfe2e8f35bf329f32873845a60599ed2df171c15728f14311b91d888c387`,
raw `42cc94f5f111442da42b03af33722f713d66b853d3ac21f21083f28ba24e7ba4`,
stderr `7964b07fd5396f10013ff8d9100d2b36e7dfc151764210d8c60bd0ae853a2d94`,
summary `e3baa479111a999d03359a0642533cad1baa3a48c415609bafb9a7a7cfa18e7c`,
metadata `0bd19797953a10223bbd7183f6e8002e490b21c29b2304befee7e6618ff9ee73`,
report `c5711b0fe06e679f889f6a7ca3812aa955c3b8c61270ef038def06b2a3f173ff`.
The v6, v7, and v8 paths are historical current-at-capture evidence; v9
preserves the same frozen negative semantics and records the executed
`taskset` identity in its runtime closure.

The frozen 101-stone literal reconstruction is documented in
[LITERAL_ARCH_101_V1.md](LITERAL_ARCH_101_V1.md), with the current
source/executable/shared-library-bound result at
[`assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v6/`](assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v6/).
It fails closed on step 1 after 5,000 outers at residual
`0.78153646143524735`: one exact failure, zero fallbacks, and no valid standing
or timing trajectory. Its dynamic `FourPointPlanar` aggregate fields report
400 contacts, 100 constraint pairs, three colors, width 34, and four-P-core
colored execution. The 95-column CSV does not expose dynamic pair identities.
The independent repeat-2 Compact collision probe is collision-only and proves
the constructed time-zero graph has 102 pairs: 100 adjacent-stone pairs and two
springer-ground pairs. Preserve the separation between these scopes and keep
the result as an untuned scientific negative.

The runner binds and rechecks the protocol, runner, trace/probe sources,
executables, `taskset`, `ldd`, and resolved regular shared-library files. This
does not bind all host runtime state. Current hashes are runner
`e3e3b7c998e038f6bf1499b1b4a8eea04261dc0aa7a6e4cf668486482c56a6f5`,
fingerprint `8d275edcaa82a2f628fdd5d9f846b2daed7e6864fdc4b7e80eca9a8c93bcf527`,
raw `03fba1d83f9209b0ade3698bdada42ca3927c176397a5ddee8ce0ca175e82947`,
summary `2e03653745a1e8d94c7b18d446004d16bfd4e176d7b9315aa198e3c08538bbfa`,
metadata `5e0b166171e08349847664b504b0a9b15cc1f582514388b532259ad6edad2bec`,
report `52e5592ba5ad32a919f221e1a8e98ecb4b4f82c0228e16de44318b436bf58958`,
and tree `ce5731fa6e4f967845d664341761eca7b0ce9ffb4cb93d55ac71df79f9243947`.
The earlier `fig08_arch101_literal_v1_negative_final/` bundle is invalid
historical evidence only; it lacked the shared-library and independent graph
bindings introduced in v2 and carried by v3/v4/v5/v6.
The v2 bundle is provenance-complete historical evidence, but a
clang-format-only collision-probe source identity change superseded it for
current-source claims. The v3 bundle was superseded after additive
card-sensitivity instrumentation, and v4 is historical current-at-capture
evidence. The unchanged command was rebaselined as v5 and again as v6 after the
current-build libdart identity advanced; the scientific result is unchanged.

The current card-manifold sensitivity contract is
[CARD_HOUSE_MANIFOLD_SENSITIVITY_V2.md](CARD_HOUSE_MANIFOLD_SENSITIVITY_V2.md),
with its provenance-complete bundle at
[`assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/`](assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/).
It changes only Native `Compact` versus `FourPointPlanar` in the reconstructed
26-card scene while holding every `paper_cpu` solver knob and one-core affinity
fixed.

Both modes emit 600/600 rows with zero exact failures and fallbacks, but both
have zero strict-success rows and accepted capped groups on every row.
`Compact` records 3,495 capped groups across 5,757 attempts and a successful
terminal last group at residual `8.525678738415048e-7`. `FourPointPlanar`
records 682 capped groups across 745 attempts and exits through the represented
terminal convergence gate at residual `0.016582575623909489`. It increases
mean contacts by `93.7983333333` and mean pair multiplicity by
`1.9548548971`, supporting the directional multiplicity hypothesis without
improving strict convergence. Both physical and timing verdicts are null, raw
wall time is diagnostic-only, and there is no paper-parity claim.

Core SHA-256 values are protocol
`eeb1c8c1d09a29e197a3c402217ecd0af9a9878749cb4756cf94bc714f2b60e9`,
runner
`e03356c772560f061e9b90fb4cd9f5df0c569631cd5e9fdd0857c337ff840562`,
trace source/executable
`b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76` /
`0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff`,
summary
`52a082ab15e8b9c314d706474cc7be557ddfc58c4961faad0d3da9d347f59f4f`,
comparison
`051605c25ccd5aa4de2f243c4dafe547c82f7298f8018cee53a8701d018ff297`,
metadata
`5890ab138179f4d7aaee6cd04c63799086439bae58fbde4f994048785dd0b8ac`,
artifact index
`1703f995bd5d2c6edeb12ba936010a80609c3e952f2c782b554bd0be7ca40627`,
invocation
`6e48eb835ab9aafca0b8d4bd29acb5ec932645fb37ca6124ce3c23b3265847f0`,
report
`0429b12f90e4fef534c9b707324a180eaac78829a3b3667505111228aba58d2e`,
and tree
`953d8efd4d43ee2a74cede6b5e3a0a766ecd1fd8914485b50b340f515b7ecb77`.
The unsuffixed v2 path is historical current-at-capture evidence; v2_r3 is the
current-source rebaseline with unchanged diagnostic semantics.
The prior-source strict paper-profile artifact remains a separate step-89
fail-closed negative, repository-archived at
`assets/dart_cpu_evidence/2026-07-12_prior_source_paper_cpu_card600_negative/`.

## What Remains Unproven

- The reconstructed impact-v1 numeric trajectory is complete only as a
  preregistered negative. A passing source-equivalent impact outcome and
  inspected impact media remain missing; do not tune or relabel v1.
- The pinned-author masonry-arch run is complete only as a current-source
  scientific negative. A source-equivalent DART dynamics port and matched
  trajectory/outcome remain missing; the configuration-only spec is not that
  port.
- The pinned-author incline sweep is complete only as numeric current-source
  scientific-negative/reference evidence. Matched DART trajectories,
  historical invocation/timing attestation, full-state external-solver
  equivalence, approved goldens, and media remain missing.
- The 101-stone exact-inertia reconstruction is precisely blocked at step 1;
  a source-pinned DART scene and matched trajectory, valid
  long-run/media/external baselines, the 10-level card house, remaining smaller
  figures beyond the finalized incline, Painleve, backspin, and turntable
  bundles, and the full paper video/GUI matrix remain missing.
- Card-manifold v2 is complete only as current-source non-strict sensitivity
  evidence. A strict full-card trajectory, physical outcome, valid long media,
  and paper parity remain missing; the prior-source step-89 strict negative is
  retained separately.
- No local timing is apples-to-apples with the paper; paper timing remains
  unevaluated rather than failed or passed.
- The current manifest audit has 29 rows: 24 partial, 5 blocked, and 0
  complete. The local visual inventory has six finalized bundles, and the
  visual workflow declares 17 schedules. Its validator fail-closed hashes
  repository artifacts, materializes
  the current bundle indexes, binds process/taskset/topology/residency and
  archived-prior-source provenance, recomputes CPU claims from raw rows, and
  rejects promotion-boundary drift.
- Final local validation completed the focused
  manifest/backspin/incline/author-masonry/author-incline suite with 777 passed
  post-merge in 213.46 s
  and the full no-cache dartpy suite with 1,555 passed in 165.09 s. The author
  masonry-arch focused CTest passed 1/1, all four current sealed bundles passed
  verify-only, and the manifest validator covered all 29 canonical
  requirements with intentional status `partial`. Live mode performed 102
  file-identity rechecks with none skipped; explicit archive mode reported all
  102 as skipped. A
  final full build/test closeout, two clean independent final-state reviews,
  current-head PR wording, and CI remain pending.

## Historical Negative Diagnostics

Preserve these as older solver evidence, not as current performance results:

- Scale-1 literal arch: 28,981-outer cold solve; the standard 200-outer
  continuation passed steps 60/180, capped near 240, and collapsed by 300/600.
- A 2,000-outer continuation was stable through 300, capped near 420/480, and
  was too slow.
- Local-diagonal seeding reduced the cold count to 22,726; dense-global
  seeding did not help materially; ERP `0.01` and `0.1` worsened or failed.
- Strict paper-parameter card bootstrap: step 1 had 119 contacts, 30,000
  outers, `15.167 s`, residual `0.004311`; step 2 had 119 contacts, 200 outers,
  `0.136 s`, residual `0.026233`, and warm start 87/119. Both failed `1e-6`
  and the trajectory was incomplete despite zero exact failures/fallbacks.

Mark26's scale 35, relaxation 1.1, literal closure, and colored schedule are
explicit DART reconstruction choices, not author parameters.

## Current Recorded Gates

- exact-Coulomb math: 47/47 in Release;
- exact constraint solver: 25/25;
- `ConstraintSolver` integration: 64/64;
- Native collision: 42/42;
- masonry wedge dynamics: 3/3;
- default paper fixtures: 19 passed, 3 explicit stress cases skipped;
- focused Release and Debug CTest matrices: 9/9 in each configuration;
- schema-v8 CPU evidence tests: 230/230;
- literal-wedge visual finalization tests: 16/16;
- crown-impact trace and negative-runner tests: 25/25;
- literal 101-stone trace/probe/runner tests: 35/35;
- finalized incline finalizer unit tests: 62/62; clean-checkout verify-only
  passes with 21 indexed artifacts and no ignored staging dependency;
- author-incline reference finalizer unit tests: 64/64; verify-only passes with
  37 indexed artifacts and 39 physical files;
- focused manifest/backspin/incline/author-masonry/author-incline evidence
  suite: 777 passed post-merge in 213.46 s;
- full no-cache dartpy Python suite: 1,555 passed in 165.09 s;
- author masonry-arch C++ specification: 1/1 focused CTest passed;
- manifest live mode: 102 file-identity rechecks, zero skipped; explicit
  archive mode: zero live rechecks, 102 skipped; and
- deterministic colored-scheduler stress: 1,000 runs passed.

## Immediate Work Order

1. Reinspect current diffs and active agents before taking file or shared-build
   ownership.
2. Preserve the P-core, standing-visual, finalized current-source Painleve-proxy,
   finalized incline and backspin, pinned-author numeric incline sweep, and
   frozen impact-v1 negative bundles
   alongside the arch101-v6 and card-v2 blockers. Continue strict card/media,
   remaining
   smaller-figure, and
   separately declared source-equivalent impact work; manually inspect decoded
   media and do not promote v2 raw timings.
3. Keep all 29 manifest rows, sidecars, hashes, and report entries synchronized
   from one identified build.
4. Synchronize the parity matrix, GUI report, residual report, and PR-facing
   report with the same evidence.
5. Preserve the final integrated gate results and obtain two clean independent
   reviews.
6. With explicit approval only: merge the latest target base, create coherent
   commits, push, replace the #3377 body, and obtain current-head CI.

## Useful Commands

Focused local tests after source stabilizes:

```bash
cmake --build build/default/cpp/Release \
  --target fbf_paper_trace fbf_paper_arch_wedge_dynamics_probe \
  test_ExactCoulombFbfPaperFixtures test_ConstraintSolver \
  test_NativeCollisionDetector \
  --parallel 4

.pixi/envs/default/bin/python -m pytest -q \
  python/tests/unit/test_run_fbf_cpu_evidence.py
```

Mark26 reproduction shape, with a fresh output directory:

```bash
.pixi/envs/default/bin/python scripts/run_fbf_cpu_evidence.py \
  --binary \
    build/default/cpp/Release/tests/benchmark/integration/fbf_paper_trace \
  --output-dir /tmp/fbf-mark26-native25-colored-v9-archwide-pcore-rerun \
  --case masonry_arch_25_literal_wedge:600 \
  --threads 1,4 \
  --cpu-list-for 1:8 \
  --cpu-list-for 4:8,10,12,14 \
  --repetitions 3 \
  --warmup-repetitions 1 \
  --solver exact_fbf \
  --contract dart_best_colored_bgs \
  --collision-frontend native
```

Demo/documentation smokes after rebuilding:

```bash
./build/default/cpp/Release/bin/dart-demos --verify-fbf-scene-docs
./build/default/cpp/Release/bin/dart-demos --cycle-scenes --frames 1
```

Revalidate the sealed incline bundle without modifying it:

```bash
.pixi/envs/default/bin/python scripts/finalize_fbf_incline_visual.py \
  --bundle \
    docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/fig01_02_incline_current_v1 \
  --verify-only
```

Verify the pinned-author numeric incline sweep without rewriting it:

```bash
python3 scripts/finalize_fbf_author_incline_reference.py --verify-only
```

## Guardrails

- `max_iterations_accepted` is not convergence.
- Zero local-kernel failures and zero fallbacks do not imply a valid global
  residual.
- Do not advance evidence after an unresolved exact group when fallback is
  off.
- Do not infer a physical outcome from nonblank or decoded media.
- Do not call a mean-real-time result an every-step deadline guarantee.
- Do not infer paper parity from local trajectory validity or speedup.
- Do not compare the incline capture's eight contacts with the independent
  traces' six aggregate contacts or claim full-trace equivalence.
- Do not use stale media, stale manifest bindings, or uncommitted diagnostics
  as current evidence.
- Preserve boxed LCP as DART's default.
