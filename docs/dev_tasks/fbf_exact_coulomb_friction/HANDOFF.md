# Fresh Session Handoff Prompt

This prompt reflects the 2026-07-22 checkpoint in
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
  implementation checkpoint 67073f4f575
  docs checkpoint 967e961421c was published before the latest evidence sync
  backspin source-contract CI fix c0364afd390
  target 6a1d377f616 is an ancestor; verify again before the next push
  topic HEAD/divergence/PR/CI/review state must be verified live

The topic tip tracks zero files under
docs/dev_tasks/fbf_exact_coulomb_friction/assets/, but topic history still has
493 unique task-asset blob IDs: 492 topic-added nonempty blobs totaling
96,573,227 bytes plus the shared empty blob already reachable from the base. PR
#3377 must be squash-merged so the 492 topic-added blobs do not enter
release-6.20, unless the topic history is explicitly rewritten first. Do not
use a merge commit or rebase merge.

The role-based task ignore keeps generated evidence out of Git. Ignored
`assets/pr_upload_3377/` stages 16 independently audited H.264/yuv420p MP4s and
`SHA256SUMS`; all browser-composer uploads and resulting user-attachment URLs
remain pending. The intentional P3 checker PPM is a runtime input, not generated
evidence. Keep all generated task evidence under an `assets/` output root; do
not replace that contract with global extension ignores that can hide runtime
inputs.

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
- Separate strict and source-continuation DART adapters now bind the current
  public no-argument five-level card-house default. Strict fails before
  release after completed step 31; final exact-with-continuation and boxed
  members complete 3,200/3,200 and their policy-labeled group-v3 clip fully
  decodes. This is not strict success, a solver-only A/B, superiority, or
  historical/paper parity evidence. Upload only group-v3 through the PR
  browser composer; v1/v2 are superseded framing probes.
- A separate source-pinned 101-stone DART adapter now runs the current
  source-supported 400-frame no-release schedule. Exact stops at step 209 on an
  iteration cap; boxed completes but collapses. This is precise current-DART
  negative evidence, not Fig. 8/video.08 or solver-superiority evidence.
- Full current-source FBF and Kamino controls also fail the same local standing
  criterion. Kamino completes all 400 frames with finite arrays, but 98/99
  mobile stones cross the height-change gate and the keystone drops
  82.03050136566162 raw units. Neither control is a historical Figure 8
  invocation or supplies parity-eligible media/full-pose evidence.
- The numeric author_incline_sweep_reference_v1 packet preserves independent
  current-source FBF, MuJoCo, and Kamino CPU runs on
  mu=.3,.4,.45,.5,.55,.6,.8. Each lane has seven 120-step cells; the retained
  FBF histories record four contacts per FBF step, while the MuJoCo and Kamino
  result records contain no contact-count field. It is
  scientific-negative/reference evidence, not a DART match, historical run,
  golden, media, timing, performance, or parity result, and it does not change
  the six-bundle visual inventory.
- A separately registered DART demo,
  fbf_author_incline_sweep_current_source, presents that operator-selected
  seven-value grid in labeled simultaneous Y lanes. Its exact and boxed
  120-step traces pass the supported/upright/in-lane/contact and
  first-three-slide/last-four-stick gates. Maximum retained-source-FBF endpoint
  deltas are 0.002426469449185232 m / 0.0011201728594518558 m/s for exact and
  0.0011521317667995284 m / 0.00030012480388411203 m/s for boxed.
- The local ignored upload candidate is
  assets/pr_media_author_incline_final_candidate_v6/groups/
  incline_author_sweep_current_source__exact_vs_boxed/clip.mp4. Capture and
  independent reuse verification pass; it is 61-frame H.264/yuv420p at 30 fps,
  2600x890, 2.033333 s, SHA-256
  a750350c7f210953bf3292f79faef2bdacb160c9652676a9f98695165357f723.
  The copied demo binary, capture summary, and independent verification summary
  SHA-256 values are
  67d399eee85ffd286984a877b8f4181b9ce3030acf5f9b2bc03886e54e7a5f20,
  243ba16ef500fc8d3bb71e1b264e0bd2e99dbd23257a3bb9cfa809a4fbeaacba,
  and 02305f4faeeb792198dc7e85cf4348ffa3f3d52a742fa4e63f8b01e52bd27b4c.
  Group metadata sets the narrow
  automated_current_source_fbf_terminal_outcome_slice_validated flag true and
  leaves generic automated_semantic_outcome_validated false.
  Source raw/projection/mu=.55-history hashes are respectively
  f5cc26d2b0ca542b2b98f7fe94a8e2f7f7c9b7cccb3d23c35234ebe45d0d9d12,
  e8b3b5c93a543480bae5c2f50106ecc1b137f65337cc1e725ef8c840efdb8921,
  and c0aa2d65cbbee24447e7ece9aa97bf83da4cc666ccf16da7edd6874abc22422f.
  The retained source has 839/840 configured convergence flags; mu=.55 step 1
  reaches 200/200, so source_reference_strictly_converged=false. This proves
  only a current-source terminal/outcome slice, not source trajectory/backend,
  solver/full-physical equivalence, timing/video/paper parity, or superiority.

PR truth:

- PR #3374 is merged at fa17fad.
- PR #3377 remains open and draft. Its implementation checkpoint is
  67073f4f575 and docs checkpoint 967e961421c was published before the latest
  evidence sync. It targets 6a1d377f616. That base is an ancestor, so no merge
  was needed. Checks and
  reviews remain mutable and must be queried live. It is not completion
  evidence.
- Checkpoint c0364afd390 fixes the observed macOS backspin-finalizer failure by
  updating the obsolete one-argument checker-helper contract to the current
  two-argument helper and call site; fresh current-head CI is still required.
- Checkpoint 67073f4f575 adds a generic no-render scene-physics query and
  fail-closed semantic provenance for 10 provider schemas. Provider-backed
  captures use v2 only after exact canonical live/sidecar equality; one broad
  implementation hash is separated into its own field, while full binary and
  source gates remain mandatory. Card-house construction and author-turntable
  consumers validate v2, the binary is hashed before and after capture, and
  legacy/no-provider v1 remains supported. Historical v1 media is not made
  current-head-reusable.
- Ignored `assets/pr_media_current_head_67073/` is the first current-head v2
  reseal: exact, boxed, and the synchronized checker-texture Figure 3 group all
  pass capture and independent verification against demo SHA-256 `69879e77...`.
  Exact/group clip SHA-256 values are `b2c268aa...` / `e321c711...`, matching
  the audited upload-staging copies. Both lanes pass the bounded 240-step
  outcome, so retain the no-superiority/no-paper-parity boundary; browser URLs
  remain pending.
- The same root now contains the current-head Figures 1-2/4-5 compact source
  rows. Capture and independent verification pass with 10 members, 6 groups,
  zero failures, and five expected exact-only author-turntable boxed skips.
  Run/verify summary SHA-256 values are 8a8b83bb... / 7599ecf8...; all ten
  members use schema v2, bind demo SHA-256 69879e77..., and have exact
  live/sidecar contract matches. The four minimum clips are byte-identical to
  their audited staging copies. Manual panels clearly show the seven incline
  lanes, author turntable 2x2, and Painleve mu=.5 agreement / mu=.55
  exact-fallen versus boxed-upright observation. Keep every non-parity and
  no-superiority boundary.
- Five-level implementation/media is bound to 0e3937e6294 and demo SHA-256
  74d989f2419734c1767d60fedf7961935e78fbf42ed33f69b68d71699a9b4067.
  This is historical capture identity. Its metadata points at the mutable
  build path, which now resolves to a later binary, and current-head reuse also
  fails the source-hash gate. Reuse validation is pending a stable
  capture-binary archive or a current-head recapture/reseal. The older small-row
  reseal remains separately bound to c95bd5fb916 and is superseded for the five
  minimum source-row uploads.
- Use PR_REPORT.md as the source for truthful body updates; keep the PR draft
  until the documented completion gates are actually met.

C95-bound supplemental proxy-row media truth:

- The historical ignored Figures 1-5 root is
  assets/pr_media_current_head_c95_small_rows/ at implementation head
  c95bd5fb916. Its reconstructed Figure 3 member/group are historical and
  superseded by the source-pinned author capture, and its minimum Figures
  1-2/4-5 source rows are superseded by the current-head v2 reseal. Retain only
  the four reconstructed Figure 4 proxy comparisons and historical diagnostics.
  Demo SHA-256 is
  5725672a0305fb6e2d824533f7e28b2a779074cc88e7f24ffa164c14cdb78149.
- Capture passes with 20 member results, 13 groups, zero failures, and five
  expected boxed skips for exact-only author-turntable schedules/group. Run
  summary SHA-256 is
  8f227ab567c4d4b3a871cdaf29336e40b3ebb6732aa37cf401ce8d01025a18af.
- Independent reuse verification passes with the same 20 members, 13 groups,
  and five expected skips. /tmp/fbf_small_rows_c95_verify.json SHA-256 is
  264ac6ebdb461c99218070571900ee0b49e1a0925ffdb8101fdcf86f117b5f1e.
- Historical c95 temporal panels show the reconstructed Figure 3 checker
  texture and coral registration tile rotating in both lanes; the Painleve
  mu=.55 group shows
  exact-tumbled versus boxed-upright. The c95 turntable group hashes are
  byte-identical to the previously audited clips/timelines, which retain the
  narrow ejected/ejected/retained/ejected author classification and proxy
  outcomes. These are manual current-DART observations, not paper parity
  or general solver-superiority evidence.
- No attachment URL is recorded. Upload accepted MP4s through GitHub's browser
  composer and record the resulting user-attachment URLs.
- The consolidated browser handoff in PAPER_DEMO_VIDEO_MATRIX.md contains 16
  independently audited H.264/yuv420p clips: nine minimum source-row clips and
  seven supplemental direct comparisons. Every listed SHA matches and every
  clip passes a full `ffmpeg -xerror` decode. Retain the mandatory non-strict,
  frozen-prefix, exact-only, proxy, and continuation-policy caption boundaries.

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

- The ignored c95-bound exact/boxed root is
  assets/pr_media_current_head_fig07/. Capture and independent reuse
  verification pass for two 600-step members and one group, with external
  summary SHA-256 values
  5a1de1f915d75c373f06aeb48978b92a540bc245427c55584f68ad178ea491bb and
  e4b3d44d5f2afebef9f79bcb92b38ee282f5635c38fa4be2f4264a5f961acce5.
- Exact records 600/600 solves, 96 contacts, zero accepted
  caps/failures/fallbacks, final/worst residual 9.778093504499096e-7 /
  9.999807145410957e-7, 599 warm starts, and 7,933 iterations. Its
  timeline/clip/panel/metadata hashes are
  6041addd27a79a747cdbcdaafb495f787d4e90a906ec0616aa16e5a33d9c9b74,
  24c110421572500bec9f43a431061ae5a386e7e59940e86030e3059cc90d9676,
  cd3498c90fd549365dadc9cf96908c4c3d86da81cbd5dd64ff4d891407b4ee6b,
  and 614704cc1ed70065d81b789c019e618ac54d7013f706b26a346ea236ef876802.
- Boxed completes 600/600 with BoxedLcpConstraintSolver. Its corresponding
  hashes are
  809ca91a475fdd0ebe3ad6b5cba73115c9ec2b3dd4a98e478beca229abf62321,
  80e79fa6b356f951e9615dd94aad2de2f55d0bbab07d7b116cb07c1b3bef686c,
  078990e4d7a950ba9e207102624d651d948f546f1153bf85077e8084c01b040a,
  and 944a6636bd6febb79ab9d6abda0a92165acd34ab3aa96b42f55f1c36731e6d45.
- The group clip/panel/metadata hashes are
  89c4d7372f68c6c9ad1a5d0e0e0388ffa1f198c2446e04fe30b9bc66325d8f9e,
  5ce6efcebcb5f6a2385f6aea6de7933cccfa7446979b7ea0e46ef2aab5199633,
  and 5cc2513eb16db191454b27126783df70adbe3ad2617d3faa3f51597ab43966bb.
  The 1320x530 H.264/yuv420p group clip has 301 frames at 30 fps over
  10.033333 s and fully decodes.
- Manual endpoint/group inspection finds both visibly standing and the labels
  clear. This is qualitative only: every metadata record sets
  automated_semantic_outcome_validated=false. It is not a physical oracle,
  source/paper trajectory or outcome, timing, superiority, crown-impact, or
  Fig. 7 parity result. The separate crown-impact adapter remains blocked at
  strict step 142. Upload remains pending through the PR browser composer.

Current source-continuation crown-impact truth:

- Implementation checkpoint 34d9b66e97c adds a separately named bounded
  continuation schedule; its ignored root is
  assets/pr_media_current_head_fig07_crown_continuation/.
- The paired run and independent reuse verification pass. Their SHA-256 values
  are f0e45526d648d7c8d6052c3a4f32ec47a29033e4ed687b89fecc52c1ce04396f
  and 969ef6143185716e8441704829d4643a1d804fc5580288dae28fe47257fce0f3;
  verification reports two results and one group.
- Exact and boxed complete 2,000/2,000 steps and release the cubes successfully
  at step 1,600. Exact records 2,122/2,122 attempts/solves, zero
  failures/fallbacks, 1,940 plateau accepts, 98 max-iteration accepts, and
  final residual 0.004493046465992133.
- Group metadata/panel/clip hashes are
  4229307f7d6d91f4b347fecc53db9f290061c6dc76482e684a94064f764601d7,
  f3bdb5a20ad57ee20e1a2cf6508a701f9bbd79bf0532ffc303d87989b4dfa802,
  and c4ffe2488520a5c22608c9117443cf9ff5de5396f4353d4bced5d1afff6bf0c8.
- Manual inspection finds both arches standing and cubes reaching the crown,
  with nearly identical visible outcomes. Metadata sets paper_comparable=false
  and automated_semantic_outcome_validated=false. This is bounded non-strict
  continuation evidence only, not strict convergence, superiority, physical
  outcome, source/paper trajectory or Figure 7 parity, timing, or backend
  evidence. It does not clear the strict step-142 blocker.
- Upload the group clip through the GitHub browser composer and record its
  user-attachment URL; do not commit the generated bundle.

- Keep the earlier trace-equivalent bundle at
  assets/paper_evidence/fig07_arch25_literal/ as distinct provenance.
- The separate session-local small visual matrix at
  `/tmp/fbf_visual_evidence_postreview_20260712` revalidates its selected nine
  schedules and exact group outputs against its recorded `dart-demos` SHA-256
  `6ac1b6fb167bdcdbfbf2fea831eac7755a751ed72bb6f1d55e4f80a4d4e25165`.
  Its `all-runnable` check fails closed at the absent
  `card_house_26/timeline.json`; keep that matrix partial and outside
  repository deliverables. Its incline, Painleve, and backspin cells are
  superseded for local use by the locally finalized bundles below; do not
  treat those `/tmp` media as the current artifacts.
- It completes 600 exact steps with zero exact failures, zero boxed-LCP
  fallbacks, and worst residual 9.9998071454109575e-7.
- Its independently rendered capture and current fbf_paper_trace reference
  compare equal over 600 rows: no integer mismatches and zero difference in
  every compared floating-point field.
- Maximum all-stone displacement from constructed t0 is
  5.431169776791696e-6 m; minimum orientation alignment is
  0.9999999999111284.
- It has 19 indexed artifacts / 21 physical files, including five selected
  local 1280x720 stills. The five-panel timeline, those stills, and the
  separately decoded video midpoint passed manual inspection; the H.264 clip
  retains a 61-frame decoded schedule.
- The H.264 clip compresses 10 simulation seconds into 6.1 playback seconds at
  10 fps, a 1.639344262295082x time-lapse, not real-time playback.
- Finalization retained immutable pending metadata and verified its hash DAG
  before writing final provenance, artifact index, and metadata.
- Capture source, binary, numeric trajectory, and decoded media are unchanged.
  The 70-file raw capture staging directory was pruned after local sealing.
  With the compact local bundle present, verify-only does not require that raw
  staging directory. The current trace changed additively for the
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
- With the compact local bundle present, finalization and verify-only pass with status
  valid_current_source_nonpaper_incline. The directory contains 23 physical
  files; its exact-membership index binds 21 and excludes only
  artifact-index.json and metadata.json.
- The combined capture retains five selected local 660x506 stills and a 61-frame
  decoded H.264 schedule at 30 fps. Its 70-file raw capture staging directory
  is pruned after local sealing, so verify-only does not require that raw
  staging directory. It records
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

Source-pinned Figure 5 Painleve truth:

- `painleve_author_mu05` and `painleve_author_mu055` are the primary Fig. 5
  exact/boxed DART schedules, pinned to public author commit
  b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0.
- The ignored durable bundle is
  assets/paper_evidence/fig05_painleve_author_current_v1/. Capture summary and
  independent verify both pass with four member results and four groups; every
  member and composite MP4 is 61-frame H.264/yuv420p, fully decodes, and has
  manually audited panels/keyframes.
- Outcomes/travel are: mu=.5 exact upright_near_rest / 1.5986787381 m; mu=.5
  boxed upright_near_rest / 1.5977005918 m; mu=.55 exact tumbled_near_rest /
  1.5399225956 m; mu=.55 boxed upright_near_rest / 1.6623056217 m.
- Exact mu=.5 has 119 attempts/solves, zero failures/fallbacks, final residual
  5.2255077e-7, and worst residual 9.7391465e-7. Exact mu=.55 has 108
  attempts/solves, zero failures/fallbacks, final residual 9.1964345e-7, and
  worst residual 9.9977460e-7. The adapter binds the exact-options header hash.
- The defensible claim is only that under the pinned current DART adapter,
  exact and boxed lanes diverge at mu=.55. Source-backend equivalence,
  trajectory equivalence, paper Figure 5 parity, timing comparability, and
  solver superiority remain false. GitHub attachment URLs are pending manual
  browser-composer upload; do not check the local bundle into Git. See
  PAPER_DEMO_VIDEO_MATRIX.md for the complete configuration and gates.

Historical finalized Painleve proxy visual truth:

- Use assets/paper_evidence/fig05_painleve_proxy_current_v1/.
- With the compact local bundle present, finalization and verify-only both pass with status
  `valid_current_source_nonpaper_proxy`, 27 indexed artifacts, and 29 physical
  files; pruned raw capture staging is not required.
- The fail-closed index binds 27 artifacts. Each selected local member clip fully
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
- Capture-time SHA-256 identity for this current-source bundle:
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

Source-pinned author Fig. 03 backspin truth:

- Prefer `fbf_author_backspin_current_source` with schedule
  `backspin_author_current_source` and ignored capture root
  `assets/pr_media_current_head_67073/`. They bind author commit
  `b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0` and sealed source-reference
  manifest SHA-256
  `7d4248f6431a902571b569b3477e61b4fa8ad0897f6c957e10a223cf32eb0b98`.
- Capture and independent reuse verification pass for two members and one
  group; summary SHA-256 values are
  `e2709dcc8aedb7c8deb52d4fd9e5ccf881cf64b92490b85d6760bffb1ae929c7` and
  `f82e74fb762906a525ee442f878c489aa833820fa910e3c137f54341b94242fc`.
- Exact and boxed both complete 240 steps / 241 states and pass the bounded
  rolling, contiguous-support, left-edge roll-off, planar-motion, airborne
  terminal, and source-terminal-tolerance slice. Exact records 205/205 solves,
  zero accepted caps/failures/fallbacks, and worst residual
  `9.990141261260073e-7`.
- Prefer the 121-frame exact member for browser upload, SHA-256
  `b2c268aa337f8d4e753408c1bbf17ca29dc4300597b64782fcb7344f6c676b30`;
  retain the labeled exact/boxed group as a supplement, SHA-256
  `e321c711eae7daf8e2a289df71f4d08c0d813d6c84e204c0930594d4a561e15b`.
- The visual-only checker is hash-bound before/after capture and visibly
  changes orientation. At 30 fps the initial `-200 rad/s` aliases, so signed
  spin is trace-backed. Both solvers pass; no equivalence/superiority,
  source-backend/full-trajectory/video/timing, historical Figure 3, or paper
  parity claim follows. GitHub attachment URLs remain pending.

Historical reconstructed Fig. 03 and video.02 backspin truth:

- Preserve
  assets/paper_evidence/fig03_backspin_current_v3/. Its exact-membership index
  binds 18 artifacts in a 20-file physical directory.
- The renderer applies a high-contrast 6x4 ivory/charcoal checker texture with
  one coral registration tile through a visual-only UV MeshShape under
  VisualAspect. The physical SphereShape remains the unchanged source of
  collision, dynamics, inertia, and friction.
- The MP4/GIF preserve the motion schedule, and three selected local stills retain
  steps 0, 1, and 2. The 140-file raw capture staging directory is pruned after
  local sealing. With the compact local bundle present, verify-only does not
  require that raw staging directory. The capture
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
- Four selected local, timeline-bound outcome stills cover steps 136, 120, 360,
  and 90 in source order. With the compact local bundle present, verify-only
  does not require the pruned raw capture staging.
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

Source-default five-level card-house truth:

- Keep strict scene/schedule `fbf_author_card_house_5_impact_current_source` /
  `card_house_author_5_impact_current_source` and continuation scene/schedule
  `fbf_author_card_house_5_impact_source_continuation_current_source` /
  `card_house_author_5_impact_source_continuation_current_source` distinct from
  the construction-only, four-level, and ten-level lanes.
- They bind the current public no-argument source default through DART: five
  levels, 40 cards, four cubes, 800 source frames / 3,200 DART substeps at
  dt=1/240 s, with release after completed step 1,600.
- Strict exact fails closed before release after completed step 31. The
  retained failed group has 39 contacts, reaches 200 iterations, and ends at
  residual 9.022404720646783e-6 against tolerance 1e-6. Aggregate telemetry is
  248 attempts, 247 solves, one failure, zero accepted caps, and zero boxed
  fallbacks.
- Final exact-with-continuation and boxed members both complete 3,200/3,200,
  and both release actions succeed at step 1,600. Exact records 7,337/7,337
  attempts/solves, zero failures/fallbacks, 2,245 plateau accepts, 836
  max-iteration accepts, zero shrink caps, 7,298 warm starts, 303,900 total
  iterations, 266 maximum and 248 final contacts, final residual
  1.2757511844995566e-7, and worst residual 0.6378480998790657.
- Final ignored roots are
  assets/pr_media_card5_source_default_exact_v3/,
  assets/pr_media_card5_source_default_boxed_v3/, and
  assets/pr_media_card5_source_default_group_v3/.
- Exact timeline/metadata/clip SHA-256 values are
  35c7fddedc2dbdb6f2b00323f19dcc6df98ac1e4188d246f06ef562ad44aea80,
  3afe8c7a3bc795827ff4318438b150d176699dc9dd55057bb48dd025684ffbdf,
  and 956ca7c32fcc23501a863d0e7ec2668fe7ffe5986343a5dcaf49ba6886be8816.
- Boxed timeline/metadata/clip SHA-256 values are
  ba771601affe997b07a66ddc161f4a110ea4eced0b4d94773b219270b63a322f,
  f77af90deac3f740d06ba0bec2eca17d837cf83974b253c35f3b76dbca35113e,
  and 319747d1a24a8a735ab4b4485b44a39905e3f6d40fe00257f78ba9b9451fcaa7.
- Independent exact and boxed verification summaries hash to
  1b7c42c0836aa17fd55f952e6335167a5786d37e54236f9088fa5d1a6a1885fd and
  078212b68ae07e234c94b2537d41158cd5c944492e930de7e1fc2b329f5a6453.
  Exact has 401 captured / 399 unique frames because two settled-frame pairs
  duplicate; boxed has 401 captured/unique frames.
- The synchronized presentation labels the lanes `EXACT COULOMB FBF + SOURCE
  CONTINUATION` and `EXISTING BOXED LCP (NO SOURCE CONTINUATION)`. Its
  H.264/yuv420p clip is 1320x530, 401 frames at 30 fps over 13.366667 s, and
  passes full decode. Clip/panel/manifest SHA-256 values are
  b46aeb3d9f09e95151e26fef4838432b6b071a5d3c39c3c9a489c6f1d42e875b,
  484fdc35aed15ba06e253be63e5ff9bb46f88bc6c273150d8f78a646da0dc7f8,
  and 2c80a8cca4cb3a0a11f49a1747bb5cc90092f884e05f0b30d3959e8e2a3eb3cf.
- This is current-public-source-default DART-adapter and qualitative
  continuation-policy presentation evidence only. It is not historical
  Tables 6-7/paper-video recovery, strict success, a solver-only A/B, solver
  superiority, trajectory/physical-outcome parity, source-backend equivalence,
  timing parity, or paper parity. Automated semantic-outcome validation stays
  false.
- Preserve v1/v2 only as superseded framing probes and never upload them.
  Upload only final group-v3 through the PR browser composer and record the
  user-attachment URL; none exists. It is the sixth supplemental clip and does
  not change the six formally finalized visual-bundle count.

Current-source four-level Figure 6 adapter truth:

- Keep the new `fbf_author_card_house_4_impact_current_source` scene and
  `card_house_author_4_impact_current_source` schedule distinct from the older
  reconstructed `fbf_paper_card_house_26` scene.
- The adapter pins author commit
  b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0 and uses the source-supported
  selection `--solvers fbf --levels 4 --frames 600 --drop-frame 400
  --num-cubes 4 --mu 0.8 --cube-size 0.4 --cube-density 500 --drop-height 1.0
  --device cpu --profile --usd`.
  This is not the no-argument five-level/800-frame source default or a known
  historical paper command.
- Source `ke=1e4`, `kd=1e3`, and `gap=.005` are recorded source semantics;
  the DART adapter does not implement equivalent source contact semantics.
- It contains 20 leaning plus 6 bridge cards and four initially kinematic
  0.8 m, 256 kg cubes. Interactive `p` releases them immediately; the evidence
  runner invokes `p` after completed substep 1600 of a 2400-step run at
  dt=1/240 s.
- Exact and boxed lanes share Native FourPointPlanar, contact capacity 4096,
  and manifold subdivision 4.
- The demo build, 13 headless/continuation C++ tests, 454 visual-runner Python
  tests, and exact/boxed contract-smoke validators pass.
- The strict exact 100-step request fails closed at completed step 35 when
  contacts jump 44 to 68. Steps through 34 are clean with prior worst residual
  9.826274595482653e-7; the failed prefix records 103 attempts, 102 solves,
  one failure, zero fallbacks, zero accepted caps, and worst residual
  4.1039190451256334e-4. Timeline:
  /tmp/fbf_author_card_house_4_exact100_last_failure_current_source_20260721/timeline.json;
  SHA-256 2d04d31134426ac2c4fc87b1774d5285b77740acaeb3ec3a005557b85944bb9d.
- The pinned source defaults `project_after_correction=false`. The current
  checkpoint adds an ABI-neutral default-on DART policy and disables it only
  for this source-selected exact adapter. Strict 36- and 100-step replays still
  fail at completed step 35 with identical diagnostics: 56 contacts, 200
  iterations, residual/best/dual 4.0845653576327421e-4, primal
  3.9380158679450451e-6, complementarity 2.3818176330330057e-4, zero accepted
  caps, and zero fallbacks. Timelines and SHA-256:
  /tmp/fbf_author_card_house_4_source_correction_exact36_20260721/timeline.json,
  686be7170e3c217bfa917698a449e7ecde40e500a2c87d073ed58ba2ac833bfb;
  /tmp/fbf_author_card_house_4_source_correction_exact100_20260721/timeline.json,
  1a76b71fc4558c7cb978eab410a95948ae50e66522e45dbded07dd36aeb11a77.
  The residual improvement is only 0.471590382%, and the new small primal
  violation means the strict claim boundary is unchanged.
- The exact adapter also opts into the pinned source's inner initialization:
  every inner solve and rejected step-size trial starts from the current outer
  reaction without projecting the seed. The ABI-neutral option defaults off,
  so DART's carried, projected seed remains unchanged elsewhere. Strict 36-
  and 100-step v3 replays with both source policies active still fail at
  completed step 35 with byte-identical diagnostics: 56 contacts, 200
  iterations, residual/best/dual 4.0844850280896461e-4, primal
  3.9375947649884479e-6, complementarity 2.3815426453852184e-4, zero accepted
  caps, zero boxed fallbacks, and zero line-search shrinks. Timelines and
  SHA-256:
  /tmp/fbf_author_card_house_4_source_inner_exact36_v3_20260721/timeline.json,
  8909e915b63bb2c412a5c5289a5aa690dc1a9ef1d712fe531d12a38d626f0d2e;
  /tmp/fbf_author_card_house_4_source_inner_exact100_v3_20260721/timeline.json,
  3e379747bac636c259fe7e9bbd711bb57d5a719d5a1d8d6b9e6317e20b639f73.
  The Figure 6 adapter and strict replay disable colored block Gauss-Seidel. A
  one-factor c95-bound probe exercises the colored ordering/path for 200
  solves with one participant and zero parallel dispatches, but changes the
  failed residual by only 2.19e-14 relative. Reject it only as the next Figure
  6 step-35 discriminator, not as a multicore or general colored-BGS result.
  A subsequent isolated c95-bound one-global-group probe also fails at
  completed step 35. Its global 68-contact residual is
  4.0848243204467147e-4, its sliced 56-contact residual is
  4.0848243204472058e-4, and every off-block `W` coefficient under the native
  partition is zero. Native and global pass generation 28 from identical
  fingerprints with different within-tolerance reactions, then first diverge
  in contact fingerprints at generation 29. Reject native-island scope only
  as this blocker hypothesis, not as a general equivalence, source, outcome,
  performance, superiority, video, Figure 6, or paper result. The isolated
  report and manifest under /tmp/fbf_fig06_global_scope_c95.TSfONI/ hash to
  633828adbe08577b6d0973ca817194530ed8a08cbe27e85d2bcb004689919fe9 and
  90d72452c6b3ed09e0bc1e408b56e70092557784fd2089e6895d7a31a0c809d3.
- A c95-bound one-factor source-gap diagnostic enables Native predictive
  negative-depth closure with 0.1 m on the ground and 0.005 m on all 30 dynamic
  shapes while preserving the strict source-inner serial contract. It fails
  earlier, after completed step 31 on a 31-contact group at 200 iterations and
  residual 1.0006073317077885e-5: 186 attempts, 185 solves, one failure, zero
  accepted caps, and zero fallback. The compared contact streams differ from
  step 1, and the stock sidecar embeds ancestor 844c9c3 rather than a fresh
  c95 binary. Reject only this gap representation as the strict 36-step fix;
  do not infer general harm/benefit, trajectory, outcome, source-backend,
  float32, performance, superiority, video, Figure 6, or paper parity. Keep
  the checked-in scene unchanged. Package /tmp/fbf_fig06_gap_c95.m6bsif/ has
  RESULTS.md SHA-256
  3b0948c80871d19cbe29495a8abc57ac4f3e92dc518a9ae6551238a9aad9b17a and
  SHA256SUMS SHA-256
  11888f98a24175f50c09ce95509d754d0bbc1963e5d2294ad982ece280292119.
- A separate c95-bound one-factor candidate changes the internal exact-FBF
  residual cadence from 1 to the source value 5. Its single 36-step strict run
  still fails at completed step 35 on a 56-contact group at 200 iterations and
  residual 4.0845024466967225e-4: 103 attempts, 102 solves, one failure, 3450
  total iterations, zero accepted caps, and zero fallback. All nonzero
  successful-iteration sums are divisible by five. The copied stock sidecar is
  ancestor-bound to 844c9c3 rather than freshly built at c95, and neither
  source-binding hash covers the patched math header. Treat every stock delta
  as context only; this establishes only that cadence five does not clear the
  prefix. The global-default patch is unshippable because two legacy-default
  tests fail, despite both cadence-focused tests passing. Keep the main tree
  unchanged. Package /tmp/fbf_fig06_residual_cadence_c95.0QXC5c/ has
  RESULTS.md SHA-256
  1f57c569f7feacb2c681cb17a70743782f07822abcbc1eb13d7822d81e9df18f and
  SHA256SUMS SHA-256
  69db5e8915fadc31aae34d94c5f484928841b286566e172eea9535ee262d7645.
- A same-binary c95 terminal spectral-estimate A/B changes only stock
  rayleigh11 to last_norm10. The control exactly reproduces step 35 / attempt
  101 / 56 contacts, residual 4.0844850280896461e-4, 103 attempts, 102 solves,
  and one failure. The single recorded variant passes all 103
  ten-product/no-Rayleigh trace invariants but still fails at those same gate
  coordinates with residual 4.07679549813362e-4. Residuals first diverge at
  attempt 57 / step 29; recorded contact-frame/reduced-state hashes and
  product-norm sequences first diverge at attempt 67 / step 30. The
  reduced-state hash covers only contact count, freeVelocity, and coefficients;
  product_norms stores norms only. Those summaries exclude W, operator
  identity, the initial reaction, the complete reduced problem, and product
  vectors, so final-state
  deltas are contextual. Reject only last_norm10 as the 36-step fix; do not
  infer source-estimator, trajectory, performance, superiority, video, Figure
  6, or paper parity. The sealed marker/timeline/trace triplets are internally
  consistent with the preregistered protocol, and the guard refuses output-path
  reuse, but neither is external proof of no discarded run. Package
  /tmp/fbf_fig06_spectral_terminal_c95.OjNIB4/evidence/ has RESULTS.md SHA-256
  e33894ab0b771544209d48724641716c491b04073ec5bec533c07df653e54cda,
  comparison.json SHA-256
  8b7af123ccaa42fd9c6bbeb0916c5b691ed3234c428ae62e404e6f26449227f6,
  and SHA256SUMS SHA-256
  f18efba2ffb1f7f8ee0f88798c9bcd38103b571210949de5c0cc625fed3fd553.
- The sixth bounded four-level strict-prefix diagnostic is a same-binary c95
  source-seed-values A/B. It changes only the initial-vector selector from
  stock `ones64` to `rs42_f32_values_dart_norm64`; both arms retain
  `rayleigh11`, DART `[n,t1,t2]` order, float64 Eigen normalization and power
  products, ten configured products plus the terminal Rayleigh product, and
  the frozen scene and strict policies. The variant promotes the raw
  NumPy-2.4.4 `RandomState(42).randn(4096 * 3).astype(float32)` values to
  double before DART normalization. The registered 168-value raw prefix has
  SHA-256
  7506d5e093b6e3787fccb4c91aee3a26feffd8548637a9a76825ad1a9f3ccfe1 and
  aborts above that dimension. The control exactly reproduces step 35 /
  attempt 101 / 56 contacts / 200 iterations, residual
  4.0844850280896461e-4, 103 attempts, 102 solves, and one failure. The sole
  variant also fails there with residual 4.1638905763175730e-4 and best
  residual 4.1593800452634807e-4 at iteration 199. Seed/product-norm/estimate
  telemetry differs from attempt 1; residual and iteration count first differ
  at attempt 57 / step 29, and contact-frame/reduced-state hashes first differ
  at attempt 67 / step 30. The reduced-state hash omits W and the complete
  solver input, and product_norms omits product vectors, so post-divergence
  residual/gamma deltas are contextual. Reject only these raw source values,
  promoted and DART-normalized in float64, as sufficient for the frozen
  36-step gate. Do not infer source-estimator or coordinate-order parity, a
  root cause, a longer trajectory, Figure 6/video parity, timing, performance,
  or superiority. No visual verdict applies. The one-shot artifacts are
  internal protocol evidence, not external proof of no discarded invocation.
  Package
  /tmp/fbf_fig06_source_seed_c95.Uemp3S/evidence/ has RESULTS.md SHA-256
  07b2f08f55bcb0210149e441c1886601d2a1f1d60d4f094b53f475ceaec88da3,
  comparison.json SHA-256
  8897b3d826789baaba11ec9c1fea47569f108f82937b41978445f51aad028aeb,
  and SHA256SUMS SHA-256
  b2ecc0cf5c84a58448b8a1eafbb03ecda05e4f9935be193d3cd79ded87676a41.
- The pinned author control completes all 2,400 substeps but marks only 1,455
  converged and 945 unconverged: 632 caps and 313 plateaus. Pre-release is
  1,332/268; release-and-after is 123/677. First false/cap indices are 33/35;
  worst natural `final_residual` is 2.59804445965485 and worst per-step final
  checked `r_coulomb` is 7.597910320688573. History:
  /tmp/fbf-sca-2026-author/paper_examples/card-house/results/20260721T175341Z/fbf/history.json;
  SHA-256 b67d3c86f106171008dfbb0aca0a2ca72a9d3747c1a7a6694f57f211d3f83afd.
  Zero-cap completion is a strict scientific DART gate, not source-equivalent
  continuation semantics. The separate telemetry-rich continuation lane below
  is implemented and captured; it does not alter the strict failure verdict.
- The boxed control completes 100 steps. Timeline:
  /tmp/fbf_author_card_house_4_boxed100_20260721_contract_v2/timeline.json;
  SHA-256 fdd3d9e96058176faa51b148d1bcf5a4c0a7f1c4e7da64e15490dcae4ce6fafc.
- The failure is now precisely characterized as the 56-contact island at
  attempt 101: `fbf_failed` / build `success` / `max_iterations`, 200 outer
  iterations, residual and dual feasibility 4.1039190451256334e-4,
  complementarity 2.4220067503580449e-4, and worst dual/complementarity local
  contact 11. The additive sidecar `last_failure` object retains that truth
  after the later 8- and 4-contact groups succeed. See
  FIGURE6_CONVERGENCE_BLOCKER.md.
- A bounded option matrix did not produce a strict 100-step completion.
  Raising the outer budget as high as 50,000 only moved the first failure to
  step 48; regularization failed earlier.
- An unsealed GDB-mutated accepted-cap preview reached release and all 2,400
  steps, but accepted 1,106/3,231 solves at the cap and reached worst residual
  0.61608914241359314. It is finite runtime continuation only, not exact
  trajectory, physical-outcome, media, or parity evidence.
- The strict exact lane still does not reach release, and the boxed control is
  bounded to 100 steps. There is no valid strict 2,400-step outcome,
  source-backend or timing equivalence, final media or PR video upload,
  Fig. 6/paper parity, or exact-versus-boxed superiority claim.

Source-continuation Figure 6 capture truth:

- Keep `fbf_author_card_house_4_impact_source_continuation_current_source` and
  `card_house_author_4_impact_source_continuation_current_source` separate from
  both the strict adapter and the older reconstruction.
- Exact and boxed use the same 26-card/four-cube scene, Native FourPointPlanar
  frontend, 2,400-step clock, and successful `p` release at step 1,600. Only
  exact requests source continuation.
- Both lanes complete 2,400/2,400 steps. Exact records 3,351/3,351
  attempts/solves, zero failures/fallbacks, 2,605 successes, 113 plateau
  accepts, 633 max-iteration accepts, and zero shrink caps. The 746 accepts are
  22.262% of 3,351 solves and occur across 723 steps. Worst residual is
  0.91712002943322535, first reached at step 2,101.
- Independent inspection finds both houses standing through release. Exact and
  boxed are identical only at step 0; viewport difference is 0.165% at step
  1,600 and 11.985% at the endpoint. In this DART source-parameterized scene,
  exact completes without exact-solver failures/fallbacks and visibly retains
  more upright structure after impact than boxed. The official MuJoCo panel
  degrades while settling but DART boxed remains upright until impact, so do not
  map the DART lanes to paper lanes or infer a mechanism. This is not
  quantitative trajectory/outcome parity, an approved golden, source-backend
  equivalence, timing evidence, solver superiority, or paper parity.
- Ignored durable bundle:
  assets/paper_evidence/fig06_card_house_source_continuation_current_v1/.
  Current-contract reseal source:
  /tmp/fbf_fig6_source_continuation_pair_20260721T1414_v2/.
  Summary SHA-256:
  6888f4729c99d41753c9c8ec9a1ec2ec9e2367c71da76aab973f8f8c5e8674cc.
  Exact timeline:
  a9eb12711419b7801037d17059560559893be2898e07d14425a5f572175482ff.
  Boxed timeline:
  1618e284f97ff7ed49e3288636269f5bea6131faa3bae45428e42e23de660bd8.
  Paired clip:
  282aebfb9e2e38fe3741db28e2ce909fb548d7aa46d048302a3b0e0bea9e1786.
- Fresh official-video SHA-256 is
  d5356e1b31487be62b75af05efbfecdb70ad5d98501a8efd378fcedf066e4794,
  exactly the audited value.
- Generated capture assets stay outside Git. The clip is only a local
  attachment candidate until its GitHub user-attachment URL is recorded.
- This is source-continuation evidence, not strict convergence. Preserve the
  strict step-35 negative unchanged.

Source-supported ten-level card-house truth:

- Keep `fbf_author_card_house_10_impact_current_source` and
  `card_house_author_10_impact_current_source` distinct from the five-level
  construction bundle, both four-level Figure 6 lanes, and the older
  `fbf_paper_card_house_10_dynamic` reconstruction.
- The adapter pins author commit
  b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0 and its public `--levels 10`
  parameterization: 155 cards, four initially kinematic cubes, 800 display
  frames / 3,200 DART substeps, and release after completed step 1,600.
  Historical Tables 6-7 invocation, trajectory, renderer, and source-video
  segment are unavailable.
- The pinned source inspection resolves one ground gap at 0.1 m and 159
  card/cube gaps at 0.005 m. The DART Native lane represents those values and
  configures all 160 collision ShapeFrames, but does not claim equivalent
  Newton/Warp collision, stiffness/damping, float32, or backend semantics.
- The one-frame pinned-source control converges all four substeps with 424
  contacts. Its 30-frame / 120-substep control stays finite but reports only 33
  converged and 87 non-converged entries, first false at `step_idx=33`. Its
  Warp/Newton backend and contact counts differ from DART, so the nearby index
  is continuation evidence, not a strict DART trajectory or physical oracle.
- The pushed Release demo and focused fixtures build; checkpoint `ffe23d347b0`
  passed the three then-current ten-level C++ tests, both C++ lint gates, and
  the then-current 331-test visual runner.
- Pushed checkpoint `ffe23d347b0` retains the historical completed-step-1
  failure: 264 final contacts, 18 attempts, 17 solves, one failure, and a
  39-contact group at residual 8.891154359157548e-6 after 200 iterations.
  Boxed completes that one-step control. Historical exact timeline SHA-256 is
  c0af3c2b03d38b68bd30374394bebb83286b08e91414641796f8ff58ec202bbf;
  historical boxed timeline SHA-256 is
  059d8d8c21db86df9b8708cf0da9b8bd63e024f2a9723d5a491c0bee2d3e78b0.
- Predictive checkpoint `3647959a188` matches the
  source's scalar `separation / dt` velocity allowance narrowly, clears exact
  step 1 with 18/18 solves and zero failures, then reaches completed step 31
  before a 79-contact group fails after 200 iterations at residual
  1.072805023427092e-5. Boxed completes all 40 requested steps.
- Final scoped local timelines are
  `/tmp/card10-predictive-scoped-exact1.0VwT5s/timeline.json` (SHA-256
  bb1c352a3a2e35b7ee0796899dfbffb59358790fe8871cc8a936cbf62404066f),
  `/tmp/card10-predictive-scoped-boxed1.Kye74m/timeline.json` (SHA-256
  8947284c4719212722a67ef920d9e5e60892ae1ecf5195f4312703cb2061fbc7),
  `/tmp/card10-predictive-scoped-exact40.YMlQ9q/timeline.json` (SHA-256
  8154d5e4eeeec934e717717f9381dd1e5f300f2691702143881a6bcf047a2495),
  and `/tmp/card10-predictive-scoped-boxed40.JE0tj6/timeline.json` (SHA-256
  b08bedc459bd0ea946c9b7a0bedf8030215c847b3377ebc3e127861ca5096b94).
- Final local gates pass: `ConstraintSolver` 66/66, Native collision detector
  50/50, `SplitImpulse` 13/13, exact solver 38/38, paper fixtures 36 passed
  with 3 explicit opt-in skips, visual runner 332/332, and independent
  post-fix re-review `ALLOW`.
- Comparable source `step_idx=30` converges 422 contacts in one global
  workspace at Coulomb-relative `7.59e-7`; DART completed step 31 has 304
  contacts across 18 islands and fails one 79-contact group. Source uses
  colored BGS with convergence checked every five sweeps; current DART uses
  sequential BGS with ten fixed sweeps. These are not the same operator or
  contact problem.
- A separately labeled ten-level continuation lane is now implemented:
  `fbf_author_card_house_10_impact_source_continuation_current_source` with
  schedule `card_house_author_10_impact_source_continuation_current_source`.
  It preserves the same geometry, Native frontend, 3,200-step clock, and
  step-1,600 release; only exact requests source continuation.
- The final exact reseal passes 3,200/3,200, and its release action succeeds.
  It records 7,702/7,702 attempts/solves, zero failures/fallbacks,
  2,427 plateau accepts, 763 max-iteration accepts, zero shrink caps, 310,880
  total iterations, 7,630 warm starts, 1,071 maximum/987 final contacts, final
  residual 7.709159985211234e-8, and worst residual
  0.59964511064890469. Timeline validation passes with 3,201 represented
  states and 401 captured/unique frames.
- The decoded exact member is H.264/yuv420p, 660x506, 401 frames, 30 fps,
  13.366667 s, and full decode passes. Metadata records
  `paper_comparable=false` and `automated_semantic_outcome_validated=false`.
  Final panel/keyframe inspection checks only legibility, release, visible
  post-release evolution, and lower structure remaining; it is not a physical
  oracle.
- Final timeline, clip, panel, and metadata SHA-256 values are
  edddf5bab098f655f6fa6a0adf50bc236474f987fa99f630a1b18d15d6d232ce,
  19637c4255c890f1f32383e7e7e680169688e5d8b071168bc6b4ffdebf33061d,
  e5ed0d63ca9818292c5a373f476f2841f280f3e01492e0065b2aec8eb95a74d6,
  and 23fe61063c024d3e93466395798951b4942755ef6bd0c4b3650f5ee00c48c84d.
  The ignored root is
  assets/pr_media_current_head_c95_card10_same_binary_exact_v2/card_house_author_10_impact_source_continuation_current_source/.
- The separate run summary
  `/tmp/card10_same_binary_exact_c95_v2_summary.json` has SHA-256
  ebf02723ab30875204bed78ebcffe1ef53bebfee8d25e84c5e5649aeb4b0ebf1
  and reports pass. Independent reuse verification passes. Its
  separate `/tmp/card10_same_binary_exact_c95_v2_verify.json` summary has
  SHA-256
  6701bcdea5664d095380e7fa5870972965dec76fdf1595d2e3ca3d8038463055,
  kind verification, one result, no skips or groups, full-decode success, and
  the matching metadata hash. The exact schedule has no blockers only within
  the narrow continuation-evidence boundary.
- A clean boxed control completes 80/80 in about 4 minutes 46 seconds with
  BoxedLcpConstraintSolver. Timeline SHA-256 is
  ccbdc322791a06d5a8858818acae63e8540ca7770e635545e3c017d84bf96d7d.
  This is a bounded solver-identity/completion control, not a full outcome.
- The full boxed member completes 3,200/3,200 with the same scene, action,
  clock, cadence, and byte-identical demo SHA-256 5725672a... as exact. Capture
  and independent reuse verification pass; timeline, clip, panel, and metadata
  SHA-256 values are 7d1d272913f4bb72bb0f98bff3d8417668ed86d2522fe913ca3f0bbfca658b43,
  c3bf391fafa0913e53ce857c497e6411a2810d71f8201a5cffb56e4dd6eb2f20,
  918eec24dbb1c30876a6d6f4a38fbb209100fe0e2fc7728d8518d233ac19db76,
  and 54414a7ab170569a1645bfaace87ea08b8d7f0fb5ce1ae51b9df87da75c19aae.
- The presentation-only same-binary labeled pair fully decodes. Its clip,
  panel, and manifest SHA-256 values are
  d09d8a4b6c962eef84620f5fc4aebd709c8631f4c274a302217c56e9163547b2,
  848805bece727c73e35e51261edd9a02a655cefdb2facd75affdd4667b972794,
  and 800d03fcf8ca5c461b9ce18bbef0ea948a30864fa2bdb739774cf20ca0b333dc.
  Manual endpoint inspection finds retained upright multi-level structure in
  exact and a largely collapsed boxed endpoint, but exact requests source
  continuation while boxed does not. This is not a solver-only A/B, automated
  physical outcome, strict convergence, performance/superiority, or parity
  result.
- The 2026-07-22 manually paired, same-binary, one-thread colored-BGS A/B is a
  completed bounded reject. Serial and colored both stop after completed step
  31 on a 79-contact group in each run after 200 iterations. No stable
  group-membership fingerprint or state-prefix hash was retained, so this is a
  same-sized-group proxy rather than proof of identical group identity. The residuals are
  1.072805023427092e-5 and 1.0728050229273756e-5. The absolute delta is
  4.997164045950943e-15 and the improvement is only
  1.0000000004658036x, far below the preregistered 10x threshold. Retained
  telemetry proves 200 colored solves, one participant, 39 manifolds, five
  colors, maximum width 10, zero dispatches, disabled affinity, and no CPU
  IDs. Do not extend this candidate.
- The ignored durable colored sidecar/text root is
  `assets/sealed_diagnostics/card10_colored_bgs_2026-07-22`.
  `RESULTS.md` and `SHA256SUMS` have SHA-256
  c5eebed4feb84b5756e42a4b70404fb2ef3c24cfc88709d0d47958eeb2fc4e2a
  and 2f20edd5cc4baca2f98f2719d0c275c3f066abecde05ac7e717529c1e05b0e9c;
  all five manifest entries verify.
- The 2026-07-22 detached one-global-group diagnostic is also a completed
  bounded reject. Clean, disabled, and native-observe controls reproduce the
  step-31 failure exactly. Candidate and native inputs retain identical
  contacts, native partition, every `mu`, `q`, post-warm-start `lambda0`, and
  per-island `W` through generation 27; off-block `W` maximum, Frobenius, and
  relative Frobenius norms are exactly zero in every candidate generation.
  The global candidate instead stops after completed step 28. Its 264-contact
  production solve accepts 9.783085822289067e-7 after 83 iterations, while
  the independent native-slice audit finds the 39-contact island at
  2.120936044948513e-6. Stock native solves that same island at
  9.487211884987307e-7 after 38 iterations. Reject native grouping as a
  sufficient cause: global scaling/stopping masked a strict native-slice
  violation. The audit invalidated that acceptance, so later trajectory is
  unknown.
- The ignored durable global evidence root is
  `assets/sealed_diagnostics/card10_global_scope_82877`.
  `RESULTS.md`, `analysis/RESULTS.json`, the final patch, and `SHA256SUMS` have
  SHA-256
  af3052c38594049adc3c266449e0c13f655cf92752a0ab85ac32bd82d1b3ee62,
  c1724ac1dfb30550ed38400e83bc59e68bbb23df6a3510fbb900fc4dbad4b160,
  04316087130d87558546129a36d21aef70e5930215e3cf70f56b2999bfb6ac7b,
  and 99dff645e0d1f55e7e6519da0557d3b385afe8c554f09d34417642439b785b90;
  all 52 manifest entries verify.
- No source/paper physical parity, Tables 6-7 parity, strict convergence,
  trajectory parity, or solver superiority follows. The labeled pair still
  requires manual PR-browser-composer upload and a recorded user-attachment
  URL. Both completed diagnostics are numerical bounded rejects only: neither
  ships default behavior, permits a 100-step/3,200-step extension, or supports
  visual, performance, source/backend, trajectory, physical, Tables 6-7,
  superiority, or paper-parity claims. A new strict A/B requires a new
  source-backed preregistered mismatch; do not loosen tolerance, caps,
  fallback, accepted-cap, or fail-fast policy. See
  CARD_HOUSE_10_CURRENT_SOURCE_DIAGNOSIS.md.

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
- A separate current-source DART adapter exposes exact/boxed lanes and the
  runner's 2,000-step release schedule. The current local-QP change keeps the
  ordinary KKT fast path and primal/dual tolerances, then certifies rejected
  positive-friction boundary candidates with fused long-double normal-cone
  stationarity. A clean exact run completes 100/100 steps with 124/124 local
  solves, zero accepted caps/failures/fallbacks, and worst residual
  `9.9936331058309156e-7`. The next fail-closed blocker is outer convergence at
  step 142: 211/211 local solves succeed, but 5,000 outer iterations end at
  residual `8.6992951837150444e-4` as contacts rise from 88 to 96. This is
  local bounded evidence, not release/impact/media evidence.
- Do not retry global tolerance/gamma widening, ULP-neighbor searches, or the
  existing projected-gradient retry: they broadened acceptance or merely moved
  the failure. FMA-only and long-double scalar-gap-only experiments still
  failed after clean rebuild. Earlier apparent successes came from a stale
  rejected gamma-bound build and are non-evidence. A boundary-multiplier Newton
  prototype was unnecessary and removed; the current path does not mutate the
  candidate or loosen the ordinary scalar-gap predicate.

Source-pinned 101-stone DART truth:

- Read FIG08_ARCH101_DIAGNOSIS.md before changing this lane. The source sums
  per-shape collision gaps (`0.010` stone-to-stone and `0.105`
  stone-to-ground), while the Native adapter currently uses zero-gap
  activation. The unsupported crown follows analytic free fall, crosses the
  standing displacement gate at step 188, and only then reaches the exact
  iteration cap at step 209. Step-size-scale, relaxation, serial-BGS, and
  doubled-inner-sweep trials are rejected; the last delays the cap to step 235
  at about 2.8x cost after physical failure. The isolated source-sized gap
  trial is also rejected: first contact moves from step 39 to 37, but the first
  exact cap moves from step 209 to 161 at residual 1.5168150500676777e-6. Do
  not wire it into the sealed scene or combine the rejected knobs.
- The independent public-source control completes 1600/1600 but only 127 steps
  converge; 1473 hit `max_outer=200` and continue. The saved keystone drops
  7.2349853515625 raw units and 57/99 mobile stones exceed the local
  three-unit height-change limit. Audit SHA-256 is
  56844eee3d908a1078fe7e76c6a92e31f2de79eb6e1e503ca7173d4e078c6cd4.
  No final x/y, rotation, or media is saved, so claim no source visual collapse,
  converged golden, historical Figure 8 parity, or solver superiority.
- Preserve
  assets/paper_evidence/fig08_arch101_author_current_v1/.
- Scene `fbf_author_masonry_arch_101_standing_current_source` binds author
  `--stones 101`, mesh tree e0c209235673d2f69c3c5de7708ab1dfadec96e3,
  path-manifest SHA-256
  7198f71730d06dd70af8703065541765bd6b6f5da137f28f9befdf7acc5f96bf,
  101 stones/99 mobile/two fixed springers/three pinned cubes, and 400 frames x
  four substeps. `drop_frame=400` is the endpoint, so no release occurs.
- Strict exact fails closed after 209/1600 completed steps on an iteration cap:
  208 contacts, 5000 iterations, residual 1.2582804496066107e-6, 342/342
  attempts/solves, one accepted cap, zero exact failures/fallbacks. The
  incomplete state trace fails standing; crown Z falls 66.1385625 ->
  62.4010546875 and maximum mobile displacement is 3.7375078125.
- Boxed completes 1600/1600 with all 1601 inventory/finite/cube-pinning samples
  valid but fails standing: maximum displacement 21.2188459736, maximum
  rotation 3.14152663339 rad, crown minimum/final 58.3806809854 /
  61.1013192467. Manual inspection shows crown loss by step 800 and collapse
  by steps 1200/1600.
- Exact/boxed timeline SHA-256 values are
  df1ed4afc9ef5aa74f7c0b6da0560ae0d1b63fca28f45051ed27c5dfb3632889 and
  a8caee71c9356a72fa65210207d7b4209d9e305363974ec07c81f19ec14bfa1e.
  The fully decoded boxed clip SHA-256 is
  7635c2722b20fb8bcb0255054cc9172153d1dd640fd8e81df4df52c0e515d3c0.
  The diagnostic hstack freezes exact at its final step-208 frame while boxed
  continues; SHA-256
  d6f5f658e4fb027edb23e0911acd34b74dfd749daace41b5d9c9204af3163b94.
- Capture and independent boxed reuse verification pass. Compact summary
  SHA-256 is
  1c19c6c3c36171a5e85f330b2863b429956652fb894aae0aa0b82d68291e3481.
  GitHub URLs remain pending.
- The current-source FBF control continues after 1,473/1,600 capped substeps
  and 57/99 mobile stones cross the local standing threshold. The separate
  Kamino control completes all 400 frames with finite arrays but 98/99 cross
  the threshold; its result/trajectory SHA-256 values are
  86b351c212c0e69df371fa66c67c53d6c3421575afbafae3a6b8d339f574dfb3 and
  63c47426019a218942afe3edae31cdf5dbbbd8d8926732ea59120e23bb6cf1a4.
  It saves no convergence/contact history, full poses, rotations, cube
  trajectory, or media.
- Keep this current-DART negative distinct from the literal v7 reconstruction
  below. Claim no source/backend/float32 trajectory or outcome equivalence,
  historical source/Kamino golden, timing, performance, Fig. 8/video.08
  parity, or solver superiority.

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
  fail-closed negative, retained in the ignored local evidence cache at
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
  local visual inventory has six locally finalized bundles, and the visual
  workflow declares 33 runnable schedules, 30 of which encode MP4. The added
  schedule is a DART-only numeric diagnostic, not a required paper/video row;
  the validator passes and fail-closed hashes local
  bundle artifacts, materializes
  the current bundle indexes, binds process/taskset/topology/residency and
  archived-prior-source provenance, recomputes CPU claims from raw rows, and
  enforces current-truth promotion boundaries.
- The browser-upload handoff contains 16 independently audited clips: nine
  minimum source-row uploads and seven supplemental comparisons. The source-
  pinned Figure 3 group is the seventh supplemental clip; adding it changes
  neither the canonical 29-row manifest status nor the six formally finalized
  visual-bundle count.
- The P-core mark26 bundle is the authoritative performance/scaling evidence
  for the reconstructed literal arch.
- The locally finalized current-source small paper_cpu/Native matrix is
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
- The Painleve proxy `current_v1` bundle remains valid only within its
  historical DART-side proxy boundary. It does not replace the separately
  finalized source-pinned Figure 5 adapter bundle, and neither visual/trace
  contract may be conflated with the current small paper_cpu matrix.
- The locally finalized backspin `current_v3` bundle passes within its
  translational-trace and checker-texture-legibility boundary; both
  requirement rows remain partial.
- Impact v1 is complete only as a preregistered numeric negative. A passing
  source-equivalent impact outcome/media and remaining paper-media artifacts
  are pending.
- Arch101 v7 is an identity-resolved step-1 failed-prefix blocker. The distinct
  source-pinned 101-stone DART lane adds a step-209 exact blocker and an
  independently verified boxed-collapse clip, but no standing or parity claim.
  Card-manifold v2 is complete non-strict sensitivity evidence.

Latest recorded focused gates:

- exact math 56/56 Release;
- exact constraint solver 38/38;
- ConstraintSolver 66/66;
- Native collision 50/50;
- SplitImpulse 13/13;
- masonry wedge dynamics 3/3;
- default paper fixtures 42 pass / 3 explicit opt-in skips;
- source-pinned author-backspin visual-runner/finalizer Python suites 463/463;
- author-backspin numeric reference verify-only passes with four artifacts,
  240 configured convergence flags, 240 trajectory states, and status
  `valid_current_source_numeric_reference`;
- focused Release and Debug CTest matrices 9/9 in each configuration;
- schema-v8 CPU runner tests 230/230;
- literal-wedge visual finalization tests 16/16;
- crown-impact trace and negative-runner tests 25/25;
- literal 101-stone trace/probe/runner tests 41/41;
- finalized incline finalizer unit tests passed 62/62; with the compact local
  bundle present, verify-only passed with 21 indexed artifacts and no raw
  capture-staging dependency;
- author-incline reference finalizer unit tests passed 64/64; verify-only
  reports 37 indexed artifacts and 39 physical files;
- focused manifest/backspin/incline/author-masonry/author-incline evidence
  suite: 859 passed post-merge in 163.95 s;
- full no-cache dartpy Python suite: 1,555 passed in 165.09 s;
- current-source four-level author-card demo build passed;
- current-source four-level headless/continuation C++ fixtures passed 13/13;
- source-supported ten-level card-house demo/test build passed; all five
  `AuthorCardHouseTenLevel*` C++ tests now pass, including the continuation and
  colored-diagnostic contracts. The older pushed checkpoint's step-1 failure
  remains historical. Predictive checkpoint `3647959a188` clears exact step 1
  and has the separate completed-step-31 / boxed-step-40 evidence above;
- colored `ConstraintSolver` filter: 3/3 passed, comprising the two
  `ExactCoulombColored*` tests and the existing default-off/copy test;
- source-default five-level demo/test build passed and both focused strict and
  continuation fixtures pass. The historical capture binary at checkpoint
  `0e3937e6294` had SHA-256
  `74d989f2419734c1767d60fedf7961935e78fbf42ed33f69b68d71699a9b4067`;
  current-head media reuse is pending recapture/reseal or a stable archive of
  that binary. V2 semantic provenance now separates one broad implementation
  identity, but full source/binary gates remain mandatory; archive and rebind
  the exact `0e3937e6294` binary or recapture at current head, then rerun exact,
  boxed, and group reuse verification before calling current reuse green;
- author-incline shared-specification and production-world C++ contract: 5/5
  passed for exact/boxed solver wiring, finite stepped state, and contact
  inventory;
- predictive-checkpoint gates: `ConstraintSolver` 66/66, Native
  collision detector 50/50, `SplitImpulse` 13/13, exact solver 38/38, and
  paper fixtures 36 passed with 3 explicit opt-in skips;
- semantic-provenance, visual-runner, card-house-construction consumer,
  author-turntable consumer, and backspin-finalizer suites passed 665/665;
- shared-library symbol inspection retained the existing nine-argument
  failure-record method and correction-policy methods, and found the additive
  source-inner setter/getter plus all 12 nonvirtual colored-diagnostic
  accessors. Colored state remains in implementation-side maps, with no public
  data-member or vtable change;
- four-level exact and boxed adapter contract smoke passed;
- the earlier Clang/FreeBSD polymorphic-`typeid` compile failure was fixed
  before implementation/media head c95bd5fb916; a Clang 22 warning-as-error
  syntax check passes, while current remote confirmation must be read live;
- author masonry-arch focused CTest: 1/1 target and 8/8 contained tests passed;
- demo scene documentation verifier: 34 scenes passed; exact/boxed real
  step-zero JSON cross-check passed;
- all four locally sealed bundles pass verify-only; under the local sealed producer
  closure, the manifest validator passes all 29 canonical requirements with
  intentional status `partial`, 118 live file-identity rechecks, and zero
  skipped, while explicit archive mode reports zero live rechecks and 118
  skipped; and
- deterministic colored-scheduler stress 1,000 runs passed; and
- semantic-provenance/current-implementation reviews: two independent
  read-only passes `CLEAN`; future provider schemas and consumers remain a
  fail-closed review watch; and
- independent post-fix re-review of checkpoint `3647959a188`: `ALLOW`.

That sealed live closure resolved `libdart.so.6.19` to the recorded
`libdart.so.6.19.3`. The normal development symlink is restored to
`libdart.so.6.19.4`; the files are byte-identical, but resolved path is part of
the fail-closed identity contract, so the current default live run reports five
path mismatches. Archive mode remains clean; only recreate the historical
symlink for an explicit live-closure recheck.

Immediate order:

1. Inspect current diffs and active agents before taking file/build ownership.
2. Preserve the P-core, standing-visual, finalized incline, pinned-author
   numeric incline sweep, historical verified Painleve proxy, finalized
   backspin, and frozen impact-v1 negative bundles alongside the
   arch101-v7, source-pinned 101-stone DART, and card-v2 blockers. Preserve the
   finalized source-pinned
   Painleve exact/boxed capture, complete state-trace outcome audit, independent
   replay, and manual inspection; upload its two exact-vs-boxed clips only
   through the PR browser composer and record the resulting URLs. Upload the
   101-stone hstack, if selected, only as a frozen-prefix blocker with no
   superiority/parity claim and record its URL. The
   source-selected four-level card
   adapter's completed-step-35 exact failure is precisely characterized but
   not corrected. Continue one-factor-at-a-time strict work without changing
   tolerance, caps, fallback, fail-fast, or accepted-cap policy. Diagnose the
   separate ten-level lane's corrected 79-contact post-step-31 failure under
   the same strict policies. Preserve the distinction between previous
   checkpoint `ffe23d347b0` and predictive checkpoint `3647959a188`. Preserve
   the independently reverified ten-level exact and boxed continuation members
   and their same-binary presentation-only labeled pair. Keep the superseded
   interrupted step-112 boxed run and partial frames as non-evidence. Preserve
   the source-default five-level exact-v3, boxed-v3, and labeled group-v3 roots
   plus the strict completed-step-31 failure. Keep v1/v2 as superseded framing
   probes and never upload them. Add only group-v3 to the browser handoff as
   the sixth supplemental clip, preserve both policy labels and all claim
   boundaries, and record its user-attachment URL after browser-composer
   upload. The
   one-participant colored-ordering result is
   rejected only for the four-level Figure 6 strict-prefix blocker, as are the
   one-global-group, source-sized-gap, residual-cadence, terminal
   spectral-estimate, and source-seed-values results. Preserve all six bounded
   rejects. Require a new
   source-backed, preregistered mismatch
   before another four-level strict solver A/B. Preserve the completed
   ten-level colored and global-scope bounded rejects; do not rerun or extend
   either without a new source-backed preregistered mismatch, promote colored
   ordering to the default, or ship the detached global patch, and do not
   loosen tolerance or caps. Use `assets/pr_media_current_head_67073/` as the
   retained five minimum Figures 1-5 source-row upload source. Keep the ignored
   c95-bound c95bd5fb916 reseal only for the four supplemental reconstructed
   Figure 4 proxy comparisons and historical diagnostics. Upload any
   final labeled pair only through the PR browser composer and
   record the URL.
   Independently review the completed telemetry-rich source-continuation
   capture without calling accepted finite iterates strict success; publish it
   only through the PR editor after explicit approval and record the resulting
   URL. Then continue strict full-duration card work, remaining smaller-figure, and
   separately declared source-equivalent impact work; inspect decoded media
   and do not promote v2 raw timings.
3. Keep all 29 manifest rows, sidecars, hashes, semantic verdicts, and report
   entries synchronized from one identified build.
4. Synchronize the parity matrix, GUI report, residual report, and PR-facing
   report with the same evidence.
5. Preserve the final integrated gate results and obtain two clean independent
   reviews.
6. With explicit approval only, merge the latest target base, commit/push,
   synchronize the truthful draft PR body, and obtain current-head CI.

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

Finalized incline local-bundle verify-only:

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
