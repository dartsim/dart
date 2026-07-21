# Fresh Session Handoff Prompt

This prompt reflects the 2026-07-19 checkpoint in
[RESUME.md](RESUME.md). Copy it into a fresh session without appending an older
cached status summary.

```text
Continue the active DART 6.20 exact-Coulomb FBF task in:

  /home/js/dev/dartsim/dart/task_6

Project home:

  docs/dev_tasks/fbf_exact_coulomb_friction/

Do not call the task complete or retire its folder. The answer to whether all
paper tests, benchmarks, GUI examples, outcomes, and performance results are
covered is "No."

Read AGENTS.md, then AGENT_CONTINUATION.md, README.md, RESUME.md,
paper-parity-matrix.md, PR_REPORT.md, gui-capture-report.md,
residual-history-report.md, and PAPER_DEMO_VIDEO_MATRIX.md. Treat
AGENT_CONTINUATION.md as the authoritative truth ledger.

Inspect and preserve any existing worktree changes. Durable branch state:

  branch research/fbf-friction-release620
  target synchronization verified through 75306efe770
  topic HEAD/divergence/PR/CI/review state must be verified live

Fetch origin and verify origin/release-6.20 is an ancestor before publishing.
Do not switch trees, revert unrelated changes, commit, push, edit a PR, rerun
CI, or mutate GitHub without explicit approval. Never add AI/tool attribution.

External truth:

- The earlier 2026-07-13 `Code (coming soon)`/404 observation is superseded.
  The MIT-licensed matthcsong/fbf-sca-2026 reference is public and pinned at
  b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0.
- Its solver, six runnable scene/configuration sources, local kernel,
  warm-start and gamma policies, pinned dependencies, and current
  MuJoCo/Kamino runners are source-auditable. Source-port and matched-run work
  are internal.
- Current author invocations were independently run and preserved. Only the
  masonry-arch mesh assets are shipped; historical renderer
  cameras/materials/goldens, the exact Apple-silicon host, and the original
  paper invocation/timing logs and warmup/aggregation attestation remain
  unavailable.
- Local work is reconstructed float64 DART on x86-64 Linux. Paper timing
  targets and verdicts remain null; no local time is apples-to-apples with the
  paper.
- The sealed author masonry-arch run is a current-source scientific negative,
  not a historical paper invocation or DART parity evidence.
- The numeric author_incline_sweep_reference_v1 packet preserves independent
  current-source FBF, MuJoCo, and Kamino CPU runs on
  mu=.3,.4,.45,.5,.55,.6,.8. Each lane has seven 120-step cells; the retained
  FBF histories record four contacts per FBF step, while the MuJoCo and Kamino
  result records contain no contact-count field. It is
  scientific-negative/reference evidence, not a DART match, historical run,
  golden, media, timing, performance, or parity result, and it does not change
  the six-bundle visual inventory.

PR truth:

- PR #3374 is merged at fa17fad.
- The #3377 topic contains origin/release-6.20 through 75306efe770. Its topic
  head, divergence, merge state, checks, and reviews are mutable and must be
  queried live rather than copied from this handoff.
- PR #3377 is work in progress and its body overclaims the task's completion
  and paper coverage. It is not completion evidence.
- Use PR_REPORT.md as the source for a truthful replacement body only after
  explicit approval to mutate GitHub.

Exact-math truth:

- Contact-row signs and conventions are covered by focused tests.
- Matrix-free row W matches impulse-test W at relative error 1.33e-16.
- Spectral-nullspace regressions cover the captured false local KKT rejection.
- Deterministic manifold-colored inner BGS groups 24 literal-arch manifolds
  into 3 colors of maximum width 8 and records actual phase CPU residency.

Authoritative schema-v8 P-core evidence:

- Use assets/dart_cpu_evidence/2026-07-19_mark26_native25_colored_v10_archwide_pcore/.
- Workload: reconstructed literal 25-stone wedge arch, Native
  FourPointPlanar, 1 um closure, exact FBF, float64 DART on x86-64 Linux.
- One warmup plus three measured 600-step trajectories completed for each of
  one and four threads: 1,800 measured steps per thread count.
- Contacts remain 96, colliding pairs/manifolds remain 24, colors remain 3,
  and maximum color width remains 8.
- Exact FBF succeeds every step. Maximum residual is
  9.999807145410957e-7. Exact failures, accepted caps, and fallbacks are zero.
- Physical outcomes pass and one-/four-thread trajectories are identical.
- One thread: mean 6.122883343333333 ms, median 2.4966535 ms, p95
  21.663236899999994 ms, max 287.473818 ms.
- Four threads: mean 4.26939745 ms, median 1.9047965 ms, p95
  14.396602399999995 ms, max 180.504588 ms.
- Validated matched-work speedup is 1.4341328993236115x.
- Both means meet the local 60 Hz target. Neither thread count meets an
  every-step 60 Hz deadline. Say "mean-real-time throughput," not "real-time
  deadline guarantee."
- This is explicitly non-paper reconstructed evidence. Paper time is blank,
  the paper timing verdict is null, and no impact/media parity follows.

Schema truth:

- Schema v8 leaves the default trace at 83 columns. Its newline-terminated
  header SHA-256 is
  396c866d782e626b5c5b52e74c392de1175f5e46dfd90b809d1f219417092e50.
- The non-paper colored contract uses a separate 95-column trace. Its
  newline-terminated header SHA-256 is
  424195336cb42753179130c9f6fcba3a8ddea9bf669cfa70c0f77fcb4c6335a5.
- Its two added invariant columns record maximum all-stone displacement and
  minimum all-stone orientation alignment from constructed t0; the literal
  outcome no longer relies only on the tracked crown.
- Both thread counts have measured-work fingerprint
  9d8df2edba609314432ff17f63768fded23577703537040d75ce082ab4233a36.
- The trace binary SHA-256 is
  0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff.
- Bundle SHA-256 values are:
  - artifact-index.json:
    a60899cc12a53f03424c02c2647f233d5c75f3ccce367ea2a604f9a7ee18bf11
  - metadata.json:
    5507ed80140a146d4247c4f0b05fd9503879ce79856189a15759b101c2cab789
  - invocations.json:
    719ba3491fad8ac12aa290faa401d0c46b10af52da7eb7a28487a5b2aac44812
  - raw.csv:
    91a379f832ca52bbce7011308640012d9f199b98e39cba4eaf661cf17fb0f017
  - summary.csv:
    3c08b251c340aa9ae7909d715ebcc2985ba34350a6e03b0b6b351d82d1ec82a2
  - summary.json:
    304736d6b871c4498a6c0de4c4448635e712fb3a8a455dd54d9ffefeef2ec170
  - REPORT.md:
    4e55d7f4dc0532ab15b86cfeb72ea9526d3f9b63194dc5a25f3974186b1a7ba7
- The four-thread result passes colored dispatch, phase residency, physical
  core mapping, matched-work fingerprint, nested baseline, physical outcome,
  and scaling-pair gates.
- Treat the P-core bundle path, timings, speedup, fingerprint, and hashes above
  as authoritative. The separate high-load E-core run is diagnostic only.

Current literal-arch visual truth:

- Use assets/paper_evidence/fig07_arch25_literal/.
- The separate session-local small visual matrix at
  `/tmp/fbf_visual_evidence_postreview_20260712` revalidates its selected nine
  schedules and exact group outputs against its recorded `dart-demos` SHA-256
  `6ac1b6fb167bdcdbfbf2fea831eac7755a751ed72bb6f1d55e4f80a4d4e25165`.
  Its `all-runnable` check fails closed at the absent
  `card_house_26/timeline.json`; keep that matrix partial and outside
  repository deliverables. Its incline, Painleve, and backspin cells are
  superseded for durable use by the finalized repository bundles below; do
  not treat those `/tmp` media as the current artifacts.
- It completes 600 exact steps with zero exact failures, zero boxed-LCP
  fallbacks, and worst residual 9.9998071454109575e-7.
- Its independently rendered capture and current fbf_paper_trace reference
  compare equal over 600 rows: no integer mismatches and zero difference in
  every compared floating-point field.
- Maximum all-stone displacement from constructed t0 is
  5.431169776791696e-6 m; minimum orientation alignment is
  0.9999999999111284.
- It has 19 indexed artifacts / 21 physical files, including five durable
  1280x720 stills. The five-panel timeline, the durable stills, and the
  separately decoded video midpoint passed manual inspection; the H.264 clip
  retains a 61-frame decoded schedule.
- The H.264 clip compresses 10 simulation seconds into 6.1 playback seconds at
  10 fps, a 1.639344262295082x time-lapse, not real-time playback.
- Finalization retained immutable pending metadata and verified its hash DAG
  before writing final provenance, artifact index, and metadata.
- Capture source, binary, numeric trajectory, and decoded media are unchanged.
  The 70-file raw capture staging directory was pruned after sealing;
  provenance binds its schedule and hashes, and clean-checkout verify-only
  needs no ignored staging. The current trace changed additively for the
  card-manifold sensitivity lane, so finalization regenerated the standing
  reference with the current executable and freshly proved all 600 standing
  rows zero-difference. Original trace hashes remain separate capture
  provenance bindings.
- Final visual SHA-256 identity:
  - 12-function/16-case visual unit file:
    f06ba295006cf7e1f0fb692fc5eacf62e4e7f3be89bd166c041752d915779bdd
  - driver:
    0f4e27b0c58e9dd3774c6be48ad4c70a857e2956fd81f11710837561f08f7243
  - C++ capture source:
    c3efeac52d02a0c373f733598db81e545d062195ba6e96c2a65bcb607cd0207f
  - capture binary:
    8b3cad15220c8fdb69c3ebdf7fa3923fda6fd812a49d0e54c8aeb07e62f0a7e9
  - current trace source:
    b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76
  - current trace binary:
    0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff
  - original trace source:
    2e55298496d76a3a3fe002fcfaf5391332fdcf2da813b5df400affbea431e7cd
  - original trace binary:
    0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff
  - final metadata:
    b217b0a43200ca558f09776e464d50f0f9dc5c5a175bcfa66ae1b656076871d1
  - retained pending metadata:
    e300103dbb950b97e217ca0bea6f1c1dc78598ddf485cdaca26cc7ad58835b3c
  - manual inspection:
    4b52bcd26ed88d184bd695d77070420aeec2c95edfb203efda7ae6241150c343
  - final provenance:
    eca349842e4121584145cd039a554aa13c51e5242ff924b37f5f49a9dee0ac2f
  - final artifact index:
    4d173c78e2c6584d6d8a85867160cc0c0687fe0f35e6878f66b0cfcd7c1c15ee
  - current reference trace:
    0e41af230fb4c27143966c3f8522702280e6746b31e84155a9a047326e46dbb3
  - trace equivalence:
    537f6e497b6fb6810240f006db47c91304581741370e319787635e6bcbfdd2e3
  - frame validation:
    a00dd7f4971756589ea834ac1a04f302ba8b7b7af96300d98e9cbc17413b3330
  - video:
    e0c111f96511bbda41af145787bd9f68782b8c9e8a909d45eb03d3c3d647feb1
  - timeline:
    926380c5c5768cc9ca1e414cbfe2631584daa99387090926d5a1f035d62b6ec9
  - decoded midpoint:
    75a88bb317441ed71803f784b2eaa099211c4e026f8547d6fe5f2ff3ee95909f
- The allowed claim is only that this reconstructed float64 x86-64 literal
  arch remains standing and visually stable in the declared 600-step
  no-projectile run.
- It is not projectile-impact evidence, paper/author-scene parity, a
  101-stone result, paper video/GUI parity, or an author-golden comparison.

Finalized Fig. 01/02 and video.03 incline truth:

- Use assets/paper_evidence/fig01_02_incline_current_v1/.
- Finalization and clean-checkout verify-only pass with status
  valid_current_source_nonpaper_incline. The directory contains 23 physical
  files; its exact-membership index binds 21 and excludes only
  artifact-index.json and metadata.json.
- The combined capture retains five durable 660x506 stills and a 61-frame
  decoded H.264 schedule at 30 fps. Its 70-file raw capture staging directory
  is pruned after sealing, so verify-only needs no ignored staging. It records
  240 exact attempts/solves, zero caps, failures,
  or fallbacks, maximum residual 9.999836962261359e-7, and eight contacts per
  post-initial step.
- Each independent tracked trace has 121 rows, 120 exact solves, 119 warm
  starts, zero fallbacks, three contacts per post-initial step, and continuous
  post-initial tracked contact.
- mu=.4 travels 1.7686892884927794 m downhill versus analytical
  1.7548661487418349 m, ends at 1.7544655347780056 m/s downhill, and has
  maximum residual 9.986952135669881e-7.
- mu=.5 travels 0.0008905412965980523 m downhill, has maximum/final stick
  speed 0.001116442058867632 m/s, and maximum residual
  9.997210606407098e-7. The tracked displacement separation is
  1.7677987471961814 m.
- Only aggregate step/exact-solve/fallback projections are byte-identical, at
  f03dff0aaec5f0fa6615609c2ea97aa31f072c14c6212fffe15858ea969d88c2.
  Capture contacts 8 differ from aggregate trace contacts 6;
  contact_count_match=false and contact_counts_compared=false. State,
  residual, status, warm-start, per-cell, and full-trace equivalence are not
  claimed because the combined renderer and independent traces use different
  placements.
- Core bundle SHA-256 identity:
  - metadata: 7a5f973a9b7264911058ec91e253dfcf5d72a7ec46fa7020df0020af1a259b7d
  - artifact index: b758bd28965bf9a96be7668c0dbb738b72c1493d83f195a3e726ae891f8f6e85
  - manual inspection: 3c3af65d62c629ae836302910a2fe7f928ab398628f280221f6d0b5d94d5a848
  - trace summary: 4df130e878f1e58d478870c8f132ee165061a752520e926f44c331d32f14f20d
  - verification: 2681073ee44f7fecd2782081826b414ee6541bb930149ced91f60a17dc2416d5
  - invocations: b4e199a8e06ac6dd0c638b261a4d969fa973fb0bdbb376e0579f19fb2f6c56c3
  - run summary: cfed42cb3b09ab8559d84a1dae4041a26d2e0937374787f34193c8363795b5d1
  - report: f75efcd40bd0452bcbdc7bbc82eee0fdbd78d7a1059f974e83999467b1688fa5
  - panel: f9f211fb376c97d98bccc67806ba3e1c9905d7d27764c794e1c480af7b4df9d3
  - clip: ad6d00ae6a614b0edbc836396c30621b589295a0eec826d060663f6782cee3f9
  - mu=.4 trace: 449acf19feef2e0aa7fb04bb9f45f865727ba59f626b3964114cd900169ecd8a
  - mu=.5 trace: 2b30e8033b123876ad1cdea755741fd230a4d72d3747e55b907e6427962659c5
- Bound implementation SHA-256 identity:
  - finalizer: 705da2a308697b4b4b923894d10f23622310e024aeab49862a24593c79142e23
  - finalizer test: 02f3800cf7cf5df5d85b1950512e78df6e413fea42cfa89880775f010d208e1c
  - visual runner: d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432
  - visual runner test: 6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e
  - demo source: 84fd1330a13d548760e537f9734790ab1b5c91fcda3e446bf76b9b36f3e1aa99
  - demo binary: d838f23e9fb04224ff194658bd6e53d8cbb95ac94ce6676e54c49b8ea25916e4
  - trace source: b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76
  - trace binary: 0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff
  - fixture source: a4bed7372213d544fef62923b88bc9accee7086165d7b3cb20245bee8bade05f
  - libdart: 8fae2320858e49fdda309d89df8cb1158c1cc5dc11d345e14f5adca0ff63cf3d
- Keep this lane separate from the strict paper_cpu/Native matrix. Its mu=.5
  classifier passes at 8.63436433e-7 m displacement, but one accepted cap per
  repetition raises the maximum residual to 1.4392081500753078e-6, so strict
  solver/local-real-time remains failed.
- Fig. 1, Fig. 2, and video.03 remain partial. Missing proof includes the full
  matched DART sweep, full-state external equivalence, approved source
  golden/diff, paper contact-count match, full 11 s semantic edit, historical
  paper timing, and real-time parity.

Pinned-author incline sweep truth:

- Preserve
  assets/paper_evidence/author_incline_sweep_reference_v1/.
- FBF, MuJoCo, and Kamino each use the exact seven-cell grid
  mu=.3,.4,.45,.5,.55,.6,.8 and 120 steps per cell. The retained FBF histories
  record four contacts per FBF step; the MuJoCo and Kamino result records
  contain no contact-count field.
- FBF records 839/840 configured convergence flags; mu=.55 is 119/120 and
  step 1 is the sole false flag after the 200-outer cap.
- Of the 839 true flags, 235 use the initial natural-residual shortcut and
  604 satisfy the configured outer nonnegative coulomb_rel < 1e-6 gate.
- Natural final_residual is a separate metric: 456 configured-true rows are
  <=1e-6 and 383 are above it. The sole configured-false row has natural
  residual 3.273267262002487e-8 but terminal
  r_coulomb=1.5311460572898186e-6. Never substitute one metric for the other.
- Current displacements place FBF and Kamino close and show a nonmonotone
  MuJoCo curve. This is not full-state or cross-solver parity.
- First-use JIT, always-on history collection, ineffective warmup exclusion,
  and lane-dependent timer boundaries exclude all timing claims.
- This packet is numeric only. It establishes no DART/source trajectory
  match, historical paper invocation, approved golden, media, paper timing,
  performance, or parity claim. Fig. 1, Fig. 2, and video.03 remain partial.

Finalized Painleve proxy visual truth:

- Use assets/paper_evidence/fig05_painleve_proxy_current_v1/.
- Finalization and clean-checkout verify-only both pass with status
  `valid_current_source_nonpaper_proxy`, 27 indexed artifacts, and 29 physical
  files; ignored capture staging is not required.
- The fail-closed index binds 27 artifacts. Each durable member clip fully
  decodes to 76 frames; the paired clip is 1320x530 at 30 fps with 76 frames.
  Raw capture frames are pruned after sealing.
- Each separate tracked fixture completes 150 steps and writes 151 rows with
  zero exact failures and zero boxed-LCP fallbacks. Maximum tracked residual is
  9.998574150559113e-7.
- The mu=.50 tracked result remains upright and reaches the settled proxy at
  x=1.298081699724907 m with final up_z=0.999998178998452.
- The mu=.55 tracked result first crosses the fixture tumble threshold at step
  36, t=0.6000000000000002 s, x=1.2597198197697048 m, which is
  0.03836187995520213 m before the mu=.50 rest distance.
- Manual inspection records the mu=.50 upright return and the mu=.55
  tumble/horizontal final presentation.
- Verified current-source source/binary SHA-256 identity:
  - finalizer: 31b2b560a3a6a7f06e514a8bc3dce9f4b766b3c4e62fe520435bfaa1e3ba77a9
  - visual runner: d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432
  - visual runner test: 6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e
  - demo source: 84fd1330a13d548760e537f9734790ab1b5c91fcda3e446bf76b9b36f3e1aa99
  - demo binary: d838f23e9fb04224ff194658bd6e53d8cbb95ac94ce6676e54c49b8ea25916e4
  - trace source: b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76
  - trace binary: 0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff
  - fixture source: a4bed7372213d544fef62923b88bc9accee7086165d7b3cb20245bee8bade05f
- Final bundle SHA-256 identity:
  - metadata: 0845988dd05b18c965eba5fb5163d43dafc901ca8f30f66b01b4f37163d72f30
  - artifact index: 7880c62d39c3f47109b50394cd84e20919565b70af19c1473c618841f682ff43
  - manual inspection: 27c2632e427ec83b3612de6003cd16523fd483ab066d6618f9cdee38efdd2d0c
  - trace summary: 115b50d92338477022435911df38d53a54e880c1daed4004dedd7d8507336164
  - verification: 6fd34e54f7958c45777731ab888fe3a5e8e24d06b9fdddb96bdace269fcb515e
  - paired panel: bef97d04d59e2937195151cbf36a0b713bb2c2bef43d111513c05885595546e2
  - paired clip: dd4cdda2410b71f8e86035f1c0ff278f9dda77133c702e409ee5d533da443a4b
  - report: a04c37f9f3db428994252c8486030aa6b97b367a627fc3a1a22106dfa8a9f51f
- The rendered demos and tracked fixtures are separate proxies and are not
  trace-equivalent. Angular velocity is absent from the tracked CSV, so strict
  rigid-body rest is not proven. The bundle does not establish author-scene or
  paper parity, faithful external-solver parity, an approved golden/diff,
  paper timing, or real-time performance.

Finalized Fig. 03 and video.02 backspin truth:

- Preserve
  assets/paper_evidence/fig03_backspin_current_v3/. Its exact-membership index
  binds 18 artifacts in a 20-file physical directory.
- The renderer applies a high-contrast 6x4 ivory/charcoal checker texture with
  one coral registration tile through a visual-only UV MeshShape under
  VisualAspect. The physical SphereShape remains the unchanged source of
  collision, dynamics, inertia, and friction.
- The MP4/GIF preserve the motion schedule, and three durable stills retain
  steps 0, 1, and 2. The 140-file raw capture staging directory is pruned after
  sealing, so clean-checkout verify-only needs no ignored staging. The capture
  has 129 exact attempts/solves, with zero accepted caps, exact failures, or
  boxed-LCP fallbacks. Capture maximum
  residual is 9.96497154974839e-7; separate trace maximum residual is
  9.964971544991853e-7.
- The trace reaches maximum x=1.5959314363310166 at step 48, first records
  negative vx at step 49, and ends at x=-2.9362508912363654 with
  vx=-6.628158971623909. Step 120 is the sole contact-free post-initial step.
- Trace and capture solver/contact projections are byte-identical at SHA-256
  973d544311bac3b5927cc73b335b1a375d0339403a2f713707bb928076aa2b22.
- Finalized SHA-256 values are metadata
  a42ce0521a7c2af31662eff6a000ef5e68fe8bddf631d4f323be1ea8230c25a7,
  index 429a0888fa7a002cbc0c93e708569e4bf8e18195421c9663d5ce3b3a1968ab7f,
  manual inspection
  b86c596da631503a3831fd55adeb934505cf758dfc8a612d0a0f32b2a55067cb,
  trace summary
  0f6221fd32742b849e9ac79ec750de71b46ec24ba66b8d6e5a0e25048223ab48,
  verification
  fb9de609e52df648465dc0dbe73af7fbbd1b98c1c4671cde504200ff72c15c01,
  trace
  dc205297fa4cbffa1b497f12507b919f344bbee45a5820ae46145e0ed91bbd98,
  panel 72bf8d6098e8fb17b98bbedeaa00cc539866cf570d91d725f77dc75f1971b067,
  MP4 7d4606f4da0a57ffbdfa0528906b21a20d7e1a4e47a6e7eb5387242aecc71928,
  GIF 773365f624ba1326855f2ad99c0196f761ab776001da568f7cdaa84054adacc8,
  and report f9178c14c1930361afc1d97cb5bf08afe04d3fd451e8a94144b02bb0b46e61cf.
- Manual inspection passes checker-texture and coral-registration-tile
  legibility only.
  At -200 rad/s, 30/15 fps sampling can alias and does not prove signed
  angular direction. Step 120 rules out continuous contact; neither rest nor
  an airborne landing phase is proven. Separate rendered and CSV scenes are
  not full-state trace-equivalent. External-solver, paper, approved-golden,
  timing, and real-time parity remain unproven. Both fig.03 and
  video.02_backspin remain partial.

Author-pinned Fig. 4 turntable truth:

- Preserve assets/paper_evidence/fig04_turntable_author_current_v1/. It pins
  author commit b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0 and binds 58
  indexed artifacts in a 60-file physical directory, in source order
  mu=.2/omega=2, mu=.2/omega=5,
  mu=.5/omega=2, mu=.5/omega=5.
- The current dart_best/Native FourPointPlanar visual lane completes all four
  360-step cells with valid solver contracts and zero fallbacks. The exact
  finite-horizon classification is ejected, ejected, retained on support
  through 6 s, ejected.
- Do not relabel the retained cell as captured, zero-slip, perfectly stuck,
  co-rotating, or stable beyond 6 s.
- Manual inspection passes the segmented disc, one coral registration wedge,
  labels, and source order. The disc and wedge are visual-only; the physical
  cylinder remains the sole collision/dynamics geometry.
- Four durable timeline-selected outcome stills bind steps 136, 120, 360, and
  90 in source order. Capture staging is pruned after sealing;
  clean-checkout verify-only needs no ignored files.
- All four capture/trace projections are byte-identical over step, contacts,
  exact_solves, warm_starts, boxed_lcp_fallbacks, and status. This is not
  full-state equivalence.
- Keep the strict paper_cpu_native lane separate. It has no capture comparison;
  mu=.5/omega=2 fails at step 40 with residual 7.407835021099202e-6 while
  its other three rows pass. Never use one lane to erase the other's result.
- Bundle status is valid_author_source_pinned_nonpaper_turntable_matrix with
  the current visual artifact, solver, physical, manual-inspection, and pass
  gates true.
- Core SHA-256 values:
  - report: 930cc12b95ab78c6e61d084064b584f5872633f2432e434c68d071818cec7fb1
  - artifact index: 209b677ce35e2b9c11248ab414727016394831084881a5e9c2866f68b51f6cdf
  - metadata: 854ef0c8aad75b4b200f512951d56b4dbef7aa82c46cc5e82123f0583dcc6ac5
  - manual inspection: 095652d3df70a144be31be49b0e25b3265df54a3815df21742333d5fdfb4529a
  - trace summary: c50ad532d1c95564c2dc7d236ebb263e83932cafcd0ec77013ed8a433336ab22
  - verification: 455da7686eaeeb989e0d122c4724ea66ff161e8d652230eff9fb8ad20590bacd
  - invocations: ed0bcce48f4c70ef6fb0a6ca37dda736e0023aae2278d29fc121290d4c249ff8
  - run summary: 3970da8be868c512ff0e1aafe4eb8add2814e3a6394b4cbfe2422ad870cc7829
  - capture provenance: b38c9adccd6de4d6dcd90d8fdadde9cd61234b4822dfb3eb444bba0787198e4b
  - group metadata: a8b7f0e6d76fa1a6993aec2038b0f0bda403e79024ef8a559d9ff938f031cfc3
  - visual runner: d848afa53caf14b9fb3ea061d658eef274e8d917151937bd6340283b79ab5432
  - visual runner test: 6e378252fa6a7cb51c6813c9d5a2b30b8c8129eacdb01df0ee19a58a270cbc5e
  - group clip: b241463658e6b48dfb2c74815e85317c2f0eccd46a7f7ab978b8f2701ce80d6d
  - author spec: 1680cd8351fa62937c0318826f7abc75917234cb3888f983acce06f13698bc6c
  - visual OBJ: bc86f1ef1f5fae1510f23b1586ae20efe788c499373370a66af81b06818f1b14
  - visual MTL: 619352b9ac14e89a4d467dde867019e0d01540b6f11852df565f23fb26a01752
- Member metadata SHA-256 values in source order are
  cf3fa8fed649adc7958f6307a4471d9e1af5651a65c2f48ec74b29066334eff8,
  c140c26ef2504a26d7b2349e2a8e76d58e302c0e19c5e0a3111da3a844747634,
  07a6a2ead3311483f81ec9a3f22c5bb77c17bbb31e4cc2916c55f03a45367c5a,
  and 7ee013065d81fe5e2f7beefa9e20efd981fab3e01abd805f01dd37fb386adaa6.
- Demo/trace CMake source SHA-256 values are
  3f4c9ac29ac5bd2c7eee07e970d5ad03a9f4b3c7fce88a5582c5027c00cfa0e5
  and 4574bc5b1243c4722c2d8f1e3849715b566c5b7ab2d6b853f3659c3fc653e2d9.
- This is finite-horizon author-source-pinned non-paper DART evidence, not
  paper-comparable, approved-golden, paper-timing, or real-time evidence.

Author card-house construction truth:

- Preserve
  assets/paper_evidence/card_house_author_5_construction_current_v1/.
- It has 12 indexed artifacts / 14 physical files and shows the public-author
  default five-level, 40-card construction with four suspended cubes at step
  zero.
- Index, metadata, and manual-inspection SHA-256 values are
  d6cbc6f9600b8bc5c3094dd85974eae8e71d64a9e3d6e99c1783ace36be9741d,
  b97ac795c9368f2632fe422f975914f412e8d1cb0667023e8d83c6224547df00,
  and 7bc672e9dd95b52853c5c7e56680190d564fc9514add9b263de0c33c3f94e2a4.
- This is construction-only evidence. The capture runs zero simulation
  substeps and proves no release, standing, trajectory, solver, contact
  dynamics, physical outcome, historical four-level/26-card trajectory,
  Fig. 6/video parity, timing, or performance claim.

Pinned-author masonry-arch truth:

- Preserve
  assets/paper_evidence/author_masonry_arch_reference_v1/.
- The pinned author invocation uses 500 frames and four substeps per frame,
  releasing three cubes at frame 400 / substep 1600. The source default is
  400 frames with drop_frame=400 and never releases the cubes; this is a new
  current-source diagnostic, not a historical or paper invocation.
- A deterministic projection represents every one of the 2000 substeps and is
  lossless with respect to the declared claim fields. The 382,753,953-byte raw
  source history (SHA-256
  `cec0e4b86837e7542c498c7ddad40538983ec023332b88ebddee7766997e3ac1`)
  is size/hash-bound but omitted. The projection records 157 true and 1843
  false author convergence flags: 142/1458 before release and 15/385 after
  release.
- Of the true flags, 40 are initial natural-residual shortcut accepts and 117
  are configured outer nonnegative `coulomb_rel < 1e-6` accepts. All 40
  shortcut accepts occur before release; the outer-gate split is 102 before
  and 15 after release. `final_residual` is the separate natural residual, and
  only 47 values are at or below 1e-6.
  Release substep 1600 is nonconverged at 100 contacts and residual
  0.017456069692858667; final substep 1999 is nonconverged at 108 contacts and
  residual 0.5161195175386001.
- A post-release contact-count increase is not pair-identified cube-arch
  contact evidence because the projection stores no pair identities.
- Exit zero proves completion and bundle preservation only. The DART shared
  spec is configuration-only, executes no dynamics, and does not implement
  the source collision/contact-gap/backend/float32 semantics. Claim no DART or
  cross-solver dynamics/trajectory/outcome equivalence, Fig. 7/video.07
  parity, timing, repeatability, pair-contact, or visual/golden evidence.

Current reconstructed crown-impact truth:

- Use LITERAL_CROWN_IMPACT_V1.md and
  assets/paper_evidence/fig07_arch25_literal_impact_v1_negative_final_v9/.
- Frozen v1 keeps the 600-step standing prefix, then injects three fixed
  35 mm cubes for 120 impact steps. The prefix matches 88 eligible standing
  fields with zero mismatches.
- First arch contact is step 607, before first ground contact at 616. The run
  stays finite with zero exact failures/fallbacks.
- It is a scientific negative: five accepted caps, residual
  9.154531704265396e-5, final arch displacement 0.07093964431215687 m above
  0.07 m, and far-field displacement 0.060523747030465196 m above 0.007 m.
- `impact_claim_passed=false`; do not tune v1 or present it as impact parity.
- The current runner recomputes all finite gates, requires post-launch exact
  progress, pins the fingerprint/preregistration contract, and stores the
  600x88 reference.
- Final hashes:
  - runner: 622b388142fed881191dcb6efb103266eb2ea1e3ffbadaf402f63b89c558bb67
  - preregistration: 4cdea674f366fc2d18eadf11ef4333d491786d5d85e3fc16fa611ea7dede3f37
  - fingerprint: 86b37ec37d28259514453c9ecc9e7d5f12afe08118a0b04abd40e72acd147384
  - standing reference: 22fabfe2e8f35bf329f32873845a60599ed2df171c15728f14311b91d888c387
  - raw: 42cc94f5f111442da42b03af33722f713d66b853d3ac21f21083f28ba24e7ba4
  - stderr: 7964b07fd5396f10013ff8d9100d2b36e7dfc151764210d8c60bd0ae853a2d94
  - summary: e3baa479111a999d03359a0642533cad1baa3a48c415609bafb9a7a7cfa18e7c
  - metadata: 0bd19797953a10223bbd7183f6e8002e490b21c29b2304befee7e6618ff9ee73
  - report: c5711b0fe06e679f889f6a7ca3812aa955c3b8c61270ef038def06b2a3f173ff
- The v6, v7, and v8 paths are historical current-at-capture evidence; v9
  preserves the same frozen negative semantics and records the executed
  `taskset` identity in its runtime closure.

Current reconstructed 101-stone truth:

- Use LITERAL_ARCH_101_V1.md and
  assets/paper_evidence/fig08_arch101_literal_v1_negative_final_v7/.
- The frozen Native exact-inertia run fails closed at step 1 after 5,000
  outers: residual 0.78153646143524735, one exact failure, zero fallbacks.
- The dynamic FourPointPlanar trace records aggregate fields of 400 contacts,
  100 constraint pairs, 3 colors, width 34, and four colored participants on
  logical CPUs 8,10,12,14.
- A separate collision-only Compact repeat-2 probe proves the constructed
  time-zero graph: 102 pairs are 100 adjacent-stone pairs plus two
  springer-ground pairs.
- The v7 one-step FourPointPlanar companion resolves the failed step-1
  pre-solve graph as exactly 100 adjacent-stone pairs and 400 contacts, four
  contacts per pair, with zero non-adjacent and zero ground pairs. Its
  aggregates and residual match the frozen trace. The companion accepts the
  capped iterate and does not follow the frozen trace participant-affinity
  contract, so solver-taxonomy and affinity equivalence remain false. This
  narrow identity result supplies no source equivalence, valid trajectory,
  standing/physical outcome, timing, media, long-run behavior, or paper
  parity.
- Runner artifact schema v2 seals the dynamics-companion validator to this
  failed-step taxonomy and exact residual. Preserve v7 as valid immutable
  negative evidence; any future positive-standing candidate requires a newly
  versioned runner and bundle rather than reusing or relabeling v7.
- `artifact_valid=false`, `standing_claim_passed=false`, and
  `timing_evidence_eligible=false`; do not promote media or timing.
- Provenance binds and rechecks the protocol, runner,
  trace/collision/dynamics-probe sources and executables, `taskset`, `ldd`, and
  resolved regular shared-library files. It does not bind all host runtime
  state.
- Fingerprint:
  8d275edcaa82a2f628fdd5d9f846b2daed7e6864fdc4b7e80eca9a8c93bcf527
- Runner:
  7155b9bc6082e79aca317be6626fea587b58538df2f34e94f865cd54c15eb993
- Raw/summary/metadata/report hashes:
  fc0705ff497b44658dfe186ed27c50b2ab9d19a268f20cf50875a3155d1dc07d,
  2cae961048b776c069caeccda2d95f2f0fd0969cae9e3de3782f0e5e5b7b640d,
  770d662f7021320982700fe699266bc94d2448679c8c4dc60bd127c660facb8a,
  1ba3987af896f68401faa1153f6b388aed38b087214d07592f5a060cab429e2a.
- The current v7 whole-tree hash is
  e65107a6d2e2880742e02d01949bc4b3f9adb2f48b6be552a720fa83c9bcc4b3.
- Preserve `fig08_arch101_literal_v1_negative_final/` only as invalid
  historical evidence; it omitted the shared-library and independent graph
  provenance introduced in v2 and carried forward through v7.
- The v2 bundle is provenance-complete historical evidence, but a
  clang-format-only collision-probe source identity change superseded it for
  current-source claims. The v3 bundle was superseded after additive
  card-sensitivity instrumentation, and v4 is historical current-at-capture
  evidence. The unchanged command was rebaselined as v5 and again as v6 after
  the current-build libdart identity advanced. V7 adds the identity-resolved
  one-step dynamics companion; the frozen trace and scientific result are
  unchanged.

Current card-manifold sensitivity truth:

- Use CARD_HOUSE_MANIFOLD_SENSITIVITY_V2.md and
  assets/paper_evidence/card_house_26_manifold_sensitivity_v2_r3/.
- The one intended factor is Native Compact versus FourPointPlanar; the
  reconstructed 26-card scene, paper_cpu solver knobs, run order, and one-core
  affinity remain fixed.
- Both modes emit 600/600 rows with zero exact failures/fallbacks, but both
  have zero strict-success rows and accepted capped groups on every row.
- Compact records 3,495 capped groups across 5,757 attempts and a successful
  terminal last group at residual 8.525678738415048e-7.
- FourPointPlanar records 682 capped groups across 745 attempts and exits
  through the represented terminal convergence gate at residual
  0.016582575623909489.
- FourPointPlanar increases mean contacts by 93.7983333333 and mean pair
  multiplicity by 1.9548548971, supporting the directional hypothesis without
  improving strict convergence.
- Both physical and timing verdicts are null, raw wall time is diagnostic-only,
  and there is no real-time or paper-parity claim.
- The prior-source strict paper-profile artifact remains a separate step-89
  fail-closed negative, repository-archived at
  assets/dart_cpu_evidence/2026-07-12_prior_source_paper_cpu_card600_negative/;
  v2 does not replace or relabel it.
- Protocol/runner/trace-source/trace-executable hashes:
  eeb1c8c1d09a29e197a3c402217ecd0af9a9878749cb4756cf94bc714f2b60e9,
  e03356c772560f061e9b90fb4cd9f5df0c569631cd5e9fdd0857c337ff840562,
  b00eea0c87f75f17259fac433bafd98c20a961d14389a80e91742d3c8a678f76,
  0923bf7df1eaa518f9a2ffacd0a42fd7330f5cbf6c35cb5749fbc122db1310ff.
- Summary/comparison/metadata/index/report hashes:
  52a082ab15e8b9c314d706474cc7be557ddfc58c4961faad0d3da9d347f59f4f,
  051605c25ccd5aa4de2f243c4dafe547c82f7298f8018cee53a8701d018ff297,
  5890ab138179f4d7aaee6cd04c63799086439bae58fbde4f994048785dd0b8ac,
  1703f995bd5d2c6edeb12ba936010a80609c3e952f2c782b554bd0be7ca40627,
  0429b12f90e4fef534c9b707324a180eaac78829a3b3667505111228aba58d2e.
- The unsuffixed v2 path is historical current-at-capture evidence. The
  current v2_r3 invocation/tree hashes are
  6e48eb835ab9aafca0b8d4bd29acb5ec932645fb37ca6124ce3c23b3265847f0 and
  953d8efd4d43ee2a74cede6b5e3a0a766ecd1fd8914485b50b340f515b7ecb77;
  diagnostic semantics are unchanged.

Historical negative diagnostics to preserve:

- The old scale-1 literal profile needed 28,981 cold outer iterations. Its
  200-outer continuation passed steps 60/180, capped near 240, and collapsed
  by 300/600. A 2,000-outer continuation was stable through 300, capped near
  420/480, and was too slow.
- Local-diagonal seeding reduced that cold count to 22,726. Dense-global
  seeding did not help materially. ERP 0.01/0.1 worsened or failed.
- The strict paper-parameter card bootstrap had step 1: 119 contacts, 30,000
  outers, 15.167 s, residual 0.004311; step 2: 119 contacts, 200 outers,
  0.136 s, residual 0.026233, warm start 87/119. Both failed 1e-6 and the
  trajectory was incomplete despite zero exact failures/fallbacks.
- Keep these as historical negatives. Do not confuse them with mark26 or
  present reconstruction parameters as author parameters.

Evidence truth:

- Current manifest audit: 29 rows = 24 partial + 5 blocked + 0 complete. The
  local visual inventory has six finalized bundles, and the visual workflow
  declares 17 schedules; validator passes and fail-closed hashes repository artifacts, materializes
  the current bundle indexes, binds process/taskset/topology/residency and
  archived-prior-source provenance, recomputes CPU claims from raw rows, and
  enforces current-truth promotion boundaries.
- The P-core mark26 bundle is the authoritative performance/scaling evidence
  for the reconstructed literal arch.
- The repository-finalized current-source small paper_cpu/Native matrix is
  assets/dart_cpu_evidence/2026-07-19_current_source_paper_cpu_small_r7/. It
  binds 60 artifacts, 27 complete CPU-4 invocations, and 5,220 rows: 9/9
  physical classifiers pass and 7/9 strict-solver/local-real-time contracts
  pass. It used zero warmups and is not paper-comparable.
  Its report/index/metadata/summary/raw SHA-256 values are
  008bc94667893cd26bfc04a720caca3d3a5703601c8739bc06856f93818c63d5,
  06594c1e1cc3c6858bc78a80630aefb0e85eeef92204b0a09b99425411c3ebeb,
  e227d7aef3b273ff81385f227f9e6766a7e40a622bc4298e1807b7c2068bc417,
  9137f1b2db0909a96897632a66617f2cfa58476a369d2adfe232b73e46cd0fa2,
  and ba062cf359da85d21b5ea83b722d26375267212dfc44a4ce9f48701ad8a79a5a.
- Older small CPU bundles and the high-load E-core diagnostic remain
  historical.
- The fig07 literal stable/standing bundle is current, trace-equivalent, and
  manually inspected within its narrow scope.
- The finalized incline `current_v1` bundle passes within its tracked
  stick/slide threshold boundary. Preserve the capture-8 versus trace-6
  contact mismatch and the separate strict `paper_cpu` mu=.5 negative.
- The pinned-author incline sweep preserves the current seven-cell FBF,
  MuJoCo, and Kamino source-run projection within its numeric
  scientific-negative boundary. It is not another visual bundle.
- The Painleve `current_v1` bundle is repository-finalized and verified
  current-source within its DART-side proxy boundary. Its separate visual and
  trace contracts must not be conflated with the current small paper_cpu
  matrix.
- The finalized backspin `current_v3` repository bundle passes within its
  translational-trace and checker-texture-legibility boundary; both
  requirement rows remain partial.
- Impact v1 is complete only as a preregistered numeric negative. A passing
  source-equivalent impact outcome/media and remaining paper-media artifacts
  are pending.
- Arch101 v7 is an identity-resolved step-1 failed-prefix blocker, and
  card-manifold v2 is
  complete non-strict sensitivity evidence. Neither promotes a physical,
  timing, media, real-time, or paper-parity claim.

Latest recorded focused gates:

- exact math 47/47 Release;
- exact constraint solver 25/25;
- ConstraintSolver 64/64;
- Native collision 42/42;
- masonry wedge dynamics 3/3;
- default paper fixtures 19 pass / 3 explicit stress skip;
- focused Release and Debug CTest matrices 9/9 in each configuration;
- schema-v8 CPU runner tests 230/230;
- literal-wedge visual finalization tests 16/16;
- crown-impact trace and negative-runner tests 25/25;
- literal 101-stone trace/probe/runner tests 41/41;
- finalized incline finalizer unit tests passed 62/62; clean-checkout
  verify-only passed with 21 indexed artifacts and no ignored staging
  dependency;
- author-incline reference finalizer unit tests passed 64/64; verify-only
  reports 37 indexed artifacts and 39 physical files;
- focused manifest/backspin/incline/author-masonry/author-incline evidence
  suite: 859 passed post-merge in 163.95 s;
- full no-cache dartpy Python suite: 1,555 passed in 165.09 s;
- author masonry-arch focused CTest: 1/1 passed;
- all four current sealed bundles pass verify-only; under the sealed producer
  closure, the manifest validator passes all 29 canonical requirements with
  intentional status `partial`, 118 live file-identity rechecks, and zero
  skipped, while explicit archive mode reports zero live rechecks and 118
  skipped; and
- deterministic colored-scheduler stress 1,000 runs passed.

That sealed live closure resolved `libdart.so.6.19` to the recorded
`libdart.so.6.19.3`. The normal development symlink is restored to
`libdart.so.6.19.4`; the files are byte-identical, but resolved path is part of
the fail-closed identity contract, so the current default live run reports five
path mismatches. Archive mode remains clean; only recreate the historical
symlink for an explicit live-closure recheck.

Immediate order:

1. Inspect current diffs and active agents before taking file/build ownership.
2. Preserve the P-core, standing-visual, finalized incline, pinned-author
   numeric incline sweep, verified current-source Painleve-proxy, finalized
   backspin, and frozen impact-v1 negative bundles alongside the
   arch101-v7 and card-v2
   blockers. Continue strict card/media, remaining smaller-figure, and
   separately declared source-equivalent impact work; inspect decoded media
   and do not promote v2 raw timings.
3. Keep all 29 manifest rows, sidecars, hashes, semantic verdicts, and report
   entries synchronized from one identified build.
4. Synchronize the parity matrix, GUI report, residual report, and PR-facing
   report with the same evidence.
5. Preserve the final integrated gate results and obtain two clean independent
   reviews.
6. With explicit approval only, merge the latest target base, commit/push,
   replace the overclaiming PR body, and obtain current-head CI.

Mark26 reproduction shape, using a fresh output directory:

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

Finalized incline clean-checkout verify-only:

  .pixi/envs/default/bin/python scripts/finalize_fbf_incline_visual.py \
    --bundle \
      docs/dev_tasks/fbf_exact_coulomb_friction/assets/paper_evidence/fig01_02_incline_current_v1 \
    --verify-only

Pinned-author numeric incline sweep verify-only:

  python3 scripts/finalize_fbf_author_incline_reference.py --verify-only

Never convert a capped or finite solve into convergence, never infer physics
from decoded media, never turn mean throughput into a worst-case deadline,
never infer paper parity from local speedup, and never use stale artifacts as
current evidence.
```
