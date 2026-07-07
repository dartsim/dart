# WS-B — ODE backend lane (gz-sim default detector)

gz-physics uses the ODE collision detector by default (issue #3056,
@azeey), and round-1 evidence plus code reading show the DART-side ODE
wrapper's *contact-history* bookkeeping is quadratic: per-pair full copies
of the accumulated contact vector (`OdeCollisionDetector.cpp:536`), a
linear-scan pair-history lookup (`:150`), and O(history × contacts)
pruning (`:1074`). After the round-1 fix (#3203) the ODE backend is still
~4.5x slower than DART-native on the settled 3k scene (RTF 18.1 vs ~81)
and ~1.5–2x slower on the active container rows; WP-PG.01 re-derives the
ratios on the round-2 branch point.

**Coordination (D5)**: WS-F may eventually flip the default detector to a
ported native engine and turn ODE into a facade. Until that flip is
*decided and green* (WS-F phases 5–6), ODE is the downstream default and
bounded algorithmic fixes here pay off immediately. D5's bounded set is
WP-PG.20/21/22 (behavior-preserving bookkeeping); WP-PG.23 is
behavior-changing and governed by D8. Re-review this lane when WS-F
reaches phase 5.

Hard behavioral constraint for the whole lane: the history machinery
exists to supplement resting contacts (capsule/manifold stability —
DART #1654, gz-physics#692, and the #3203 cylinder tangent fix depend on
it). **Contact emission order and content must be preserved exactly**;
every packet must show bit-identical ODE hashes/contacts on all guard
scenes, plus the gz gate.

#### WP-PG.20 — Contact-history span tracking (kill per-pair full copies)

- Status: done — PR pending (`wp-pg-20-ode-history-spans`)
- Objective: replace the per-pair full copy of `result.getContacts()` with
  per-pair index spans recorded as contacts are appended.
- Value: removes the dominant O(pairs × total-contacts) copy cost on
  contact-rich scenes for the gz default backend; pure bookkeeping change.
- Scope: `dart/collision/ode/OdeCollisionDetector.cpp` (+ group/object
  files if span state lives there); no header layout changes to
  gz-subclassed classes (`OdeCollisionDetector` vtable frozen).
- Non-goals: changing which contacts are emitted or their order.
- Acceptance evidence: bit-identical ODE hashes/contacts/resting on all
  five guard scenes; ODE rows of the active matrix improve measurably;
  other detectors untouched (hash-identical).
- Completion evidence: base vs WP-PG.20 hashes stayed bit-identical for
  `S2_ode`, `S4_ode`, and `S3_dart`. Dense ODE `S3_ode` improved from
  194.2 to 17.6 ms/step; Google Benchmark `BM_ContactContainerActive`
  ODE 120/1/1 improved from 16.5 to 6.18 s and 120/1/16 improved from
  6.84 to 6.02 s. Controlled-load crash gate was 30/30 clean on both
  base and WP-PG.20.
- Dependencies: WP-PG.01.

#### WP-PG.21 — Pair-keyed history lookup + generation-based pruning

- Status: open
- Objective: replace the `FindPairInHist` linear scan with an
  unordered_map keyed on the canonical object pair, and replace
  O(history × contacts) pruning with a generation/stamp sweep.
- Value: removes the remaining quadratic terms in history maintenance.
- Scope: same files as WP-PG.20; data-structure change only, iteration
  order over emitted contacts unchanged (iterate emission-ordered spans,
  not the map).
- Non-goals: history semantics changes; capacity/eviction policy changes.
- Acceptance evidence: same bar as WP-PG.20 (bit-identical ODE outcomes;
  measurable win at 900/3000 objects; gz gate green).
- Dependencies: WP-PG.20 (shares span structure).
- Re-scope note: WP-PG.20's executed implementation already includes the
  pair-keyed history map and current-frame pruning sweep. After WP-PG.20
  lands, re-evaluate whether any generation-based pruning work remains
  evidence-backed before claiming this packet.

#### WP-PG.22 — Version-gated ODE pose push

- Status: open
- Objective: skip per-step ODE geom pose writes for objects whose
  kinematic version is unchanged (`CollisionGroup.cpp:379`,
  `OdeCollisionObject.cpp:321`).
- Value: settled/mixed scenes stop paying O(all objects) pose-push per
  step on the gz default path.
- Scope: version plumbing mirrors the round-1 version-keyed cache pattern
  (#3192); cpp-only.
- Non-goals: broadphase changes inside ODE itself.
- Assumptions: BodyNode/ShapeNode version counters already tick on every
  pose-affecting mutation (verified pattern from round 1); sub-1e-6 pose
  deltas must still propagate (gz exact-pose expectation) — version
  granularity, not epsilon filtering.
- Acceptance evidence: bit-identical ODE outcomes on guard scenes
  (including the #3227 joint-detach regression scene); settled-scene ODE
  step time reduction.
- Dependencies: WP-PG.01; independent of 20/21.

#### WP-PG.23 — ODE wrapper-level per-pair manifold reduction

- Status: blocked on D8 — behavior-changing PR class
- Objective: the follow-up named by #3209 root-cause finding 3: when the
  ODE trimesh cylinder fallback (or any pair) emits excessive contacts,
  reduce to a representative per-pair manifold at the wrapper level
  (deepest + spread selection, mining round 1's #3135 native
  implementation, issue #2366, and DART 7 native contact reduction)
  instead of relying on scene-level caps.
- Value: round 1 recorded 2.6x RTF on FCL from per-pair capping alone;
  removes the silent-contact-drop failure mode (bodies tunneling out)
  that today makes uncapped ODE scenes invalid.
- Scope: `dart/collision/ode/*` wrapper (no ODE library changes); honors
  `CollisionOption.maxNumContactsPerPair` semantics gz relies on
  (`GzCollisionDetector::LimitCollisionPairMaxContacts`).
- Non-goals: changing native/FCL/Bullet manifold behavior; altering the
  contact-history machinery (WP-PG.20/21 territory).
- Acceptance evidence: behavior-changing bar — tolerance rationale,
  old/new ODE guard rows (contacts, resting, hash) with maintainer
  sign-off; the #3209 tunneling reproducer stays bounded WITHOUT the
  scene-level cap; gz gate green (gz per-pair capping still honored).
- Dependencies: D8; WP-PG.01; sequenced after WP-PG.20/21 to avoid
  history/manifold interaction churn.
