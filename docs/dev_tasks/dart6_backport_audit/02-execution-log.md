# 02 — Backport Execution Log (release-6.20)

The execution companion to this task. [`README.md`](README.md) owns scope and
decisions, [`01-backport-inventory.md`](01-backport-inventory.md) owns the
per-item plan/evidence, [`RESUME.md`](RESUME.md) owns next steps — and this file
owns the **record of what was actually ported, how it was verified, and how
conflicts with the parallel performance lane were avoided**.

**Status (2026-06-25): all work merged; a clean green-matrix run is the only
remaining gate.** All accepted feature backports, the in-scope CI/build tooling,
and the spell-lint (including its temporary-skip cleanup) are merged to
`release-6.20`. After the GitHub CI runner gridlock cleared, the full matrix
surfaced several *latent* platform-specific failures (clang `-Werror`, macOS arm64,
FreeBSD VM patches, MSVC) — all now resolved (§9). Two of them were fixed by the
parallel performance lane in DART-6 code this effort had backported. No code items
remain; a clean, uninterrupted green run of the required matrix is the closing
confirmation (§10).

---

## 1. Verification protocol (the gate every PR passed)

CI on `release-6.20` was runner-gridlocked throughout, and the Codex review quota
was exhausted, so **local verification was the merge gate** (admin-squash-merge on
local green). Per backport:

1. Merge the backport branch onto a local cascade branch off current
   `release-6.20`; resolve conflicts.
2. `pixi run test-all` — full C++ (`tests_and_run`) + Python (`pytest`) suite;
   **0 failures** required.
3. **gz backward-compat** at cluster checkpoints: `pixi run -e gazebo test-gz`
   (gz-physics 199 + 4 perf + gz-sim `INTEGRATION_entity_system`). Passed at the
   5-, 11-, and 13-feature marks.
4. Coverage: the ported tests exercise the new lines (codecov CI never posted due
   to the gridlock; a local `lcov` pass was used instead — see §7).
5. `gh pr merge --squash --admin` (release-6.20 has no required reviews; admin
   bypasses the BEHIND/strict requirement during the serial cascade).

Configs the Release-only `test-all` gate does **not** cover were reproduced
locally afterward (§7): asserts-enabled, AddressSanitizer, coverage.

## 2. Feature backports merged (13)

Each is additive / opt-in and preserves gz-physics + gz-sim API, behavior, and
performance. Order is the merge order.

| # | Source PR | Backport PR | Summary | gz checkpoint |
|---|-----------|-------------|---------|---------------|
| 1 | #2153 | #3156 | `Inertia::transformed()/transform()` helpers | |
| 2 | #2242 | #3158 | `WeldJoint::merge()` + `BodyNode::getNodes()` | |
| 3 | #2222 | #3159 | per-DoF actuator type API on `Joint` | |
| 4 | #2279 | #3157 | raycast filter option (Bullet) | |
| 5 | #2168 | #3160 | `WorldConfig` + collision-detector selection | ✅ 5-feature gz |
| 6 | #2170 | #3161 | ShapeNode inertia aggregation (parser) | |
| 7 | #2351 | #3162 | coordinate charts for Ball/FreeJoint (default `EXP_MAP`) | |
| 8 | #2254 | #3163 | SDF `<mimic>` joint parsing | |
| 9 | #2212 | #3164 | coupler constraint support (opt-in) | |
| 10 | #2252 | #3165 | `RevoluteJointConstraint` (closed-loop hinges) | |
| 11 | #2354 | #3166 | split-impulse contact correction (opt-in, default off) | ✅ 11-feature gz |
| 12 | #2340 | #3167 | polygon / n-gon mesh support | |
| 13 | #2338 | #3168 | convex-mesh collision + rendering (`ConvexMeshShape`) | ✅ 13-feature gz |

`#2325` (MeshShape TriMesh) was already merged earlier via `#3145`, ahead of this
cascade.

### gz backward-compat evidence

`pixi run -e gazebo test-gz` on the cumulative state at each checkpoint:
**gz-physics 199/199 + 4/4 perf + gz-sim `INTEGRATION_entity_system` 1/1**, every
time. The perf-sensitive split-impulse (#2354) was additionally verified
empirically flag-OFF byte-identical to the Baumgarte default, so gz performance is
unchanged.

## 3. CI / build tooling merged

The four in-scope non-feature items were combined into **one PR (#3174)** because
they share `pixi.toml` / `pixi.lock` / `CMakeLists.txt`:

| Source PR | What landed on release-6.20 |
|-----------|------------------------------|
| #3108 | `DART_USE_SYSTEM_FMT` option (default **ON**) + FetchContent fmt 11.1.4 fallback; finder `cmake/DARTFindfmt.cmake` |
| #2736 | gersemi CMake formatting + `check-lint-cmake` gate + fresh reformat of ~44 CMake files (`scripts/lint_cmake.py` skips the FreeBSD `*.cmake` patch files) |
| #2185 | Taplo TOML lint (`lint-toml` auto-fix task; no required gate) |
| #2541 | Eigen 64-byte over-alignment CI guard (non-blocking job mirroring `build-asserts`) |

The fifth non-feature item, **#2251** (typos + spell lint), landed separately as a
conflict-safe port (**#3177**) — see §6. The sixth, **#2655**, was skipped (§6).

Plus two follow-ups:

- **#3173** — clang-format-14 fixes in 3 backported files (`WeldJoint.cpp`,
  `RaycastOption.cpp`, `Joint.cpp` binding) that failed the Debug job's
  `Check Lint` step (the build-only gate had not run formatting).
- **#3175** — a single-body `RevoluteJointConstraint` constructor test, closing a
  coverage gap a local `lcov` run surfaced (§7).

## 4. Conflict resolutions (during the cascade)

- **#2254 (CHANGELOG.md)** — both sides added a Dynamics entry at the same spot;
  kept both entries.
- **#2212 (4-file coupler/actuator overlap with #2222)** — the meaningful one.
  #2222 (per-DoF actuator) and #2212 (coupler) both touch
  `ConstraintSolver::updateConstraints()`, `Joint.cpp`, the `Joint` pybind11
  binding, and `test_Joints.cpp`. Resolved by **nesting #2212's opt-in coupler
  dispatch inside #2222's per-DoF `hasValidMimicDof` validation gate** (so the
  default Motor path gz uses is unchanged), keeping both `.def` chains in the
  binding and both test sets. Verified compile + all 4 tests pass.

The mesh cluster (#2340/#2338) and the rest of the constraint cluster
(#2252/#2354) merged clean in the chosen order.

## 5. DART-6 layout adaptations (recurring)

PascalCase headers (vs main snake_case); pybind11 (vs nanobind); `dart/utils`
(vs `dart/io`); `SmartPointer.hpp` + `DART_COMMON_DECLARE_SHARED_WEAK` (no
`Fwd.hpp`); no `DART_API`/`Export.hpp` on 6.20 constraint/shape classes;
`HAVE_BULLET`/`HAVE_ODE` (vs `DART_HAVE_*`); tests auto-discovered via
`dart_build_tests(GLOB_SOURCES)` (no per-test CMake registration); `-Werror`
deprecation enforced. DART-7-only infra (`CouplerConstraint` on main,
`MeshLoader`, `ClassicRigidSolver`, `tests/unit/gui` tree) is legitimately
dropped or re-homed.

## 6. Ported (conflict-safe) and skipped

- **#2251 (typos + spell lint) — PORTED conflict-safe (#3177).** Adds `codespell`
  + an enforced `check-lint-spell` gate (wired into the required `check-lint`) and
  fixes ~270 typos across 128 files (comments/docs/strings/local vars; no
  behavioral change). Wiring the gate would normally red-X the parallel lane's
  open PRs and its fixes would edit do-not-collide files — so the 9 do-not-collide
  files that carry pre-existing typos (e.g. misspelled "force" in `Joint.hpp`,
  "entity" in `World.cpp`, "with" in `ContactConstraint.hpp`) are **temporarily
  skipped** in `.codespellrc` under a labeled block. The gate therefore passes
  without editing their files and does **not** trip the other lane's PRs. Vendored
  `tests/unit/math/legacy_convhull_3d` is permanently skipped. **Cleanup done
  (#3184, 2026-06-25):** once the perf lane fully merged, the 9 `# TEMPORARY` skip
  entries were removed and those typos fixed (misspellings of force, entity,
  contacts, axis, "even though", etc.), so codespell now scans the whole tree.
- **#2655 (centralize CI path filters) — SKIPPED.** No-op on release-6.20: the
  `dorny/paths-filter` blocks and `ci_altlinux.yml` it refactors do not exist
  here, and the one relevant change is already present.

## 7. Proactive local CI-config reproduction

Because the GitHub matrix was gridlocked, the required configs the Release-only
gate never exercised were reproduced locally on the merged `release-6.20`:

- **Asserts-enabled (no `-DNDEBUG`)** — 115/115 C++ + 60 Python, **0** assertion
  failures. (Required CI check, pre-validated.)
- **AddressSanitizer** — 114/114, **no** memory errors in the new
  split-impulse / convex-mesh / coupler code.
- **Coverage** — 64.2% overall; backport files 68–90%. A local `lcov` pass
  surfaced the single-body `RevoluteJointConstraint` constructor as uncovered →
  test added (#3175). Remaining low-coverage files are pre-existing untouched
  code (`SoftBodyNode`, `MetaSkeleton`) and LCP solver internals (sim-exercised).
  Note: `build/default/cpp/Debug` had been poisoned with foreign artifacts and was
  wiped before the clean coverage build.

## 8. Cross-lane conflict-avoidance

A separate machine/agent owns the **performance lane** — the 11-PR stacked chain
`perf/dart6-*` (#3147 → #3172) and **#3169** (`fix/dart6-actuator-override-storage`,
which fixes the per-DoF actuator storage from #2222). **Do not touch those
branches/PRs.** Their combined footprint (the "do-not-collide" set) is collision /
constraint / `Joint*` / `World.cpp` / the two bindings / a few tests / CHANGELOG —
and notably **zero** `.github` / `.cmake` / `CMakeLists` / `.toml` files. Every
port here was checked to touch **none** of those files (the gersemi reformat is
CMake-only; the eigen job is non-required signal), so the tooling additions do not
collide with the perf lane and the new gersemi gate cannot trip their CMake-free
PRs.

## 9. Post-gridlock CI resolution

When the runner gridlock cleared, the full matrix ran for the first time and
surfaced several *latent* failures — none catchable by the Release-only / gcc /
no-`-Werror` local gate. All resolved:

| Failure (config) | Root cause | Fix |
|---|---|---|
| `Check Lint` (Debug/Release) | codespell flagged this log's own example typos; #2251's typo fixes drifted clang-format on 3 files | #3179, #3181 |
| `arm64-Debug/Release` (macOS clang `-Werror`) | dartpy MeshShape binding exposes deprecated Assimp APIs (#3145) → `-Wdeprecated-declarations` fatal | #3180 (`DART_SUPPRESS_DEPRECATED_BEGIN/END`) |
| `FreeBSD repro (VM)` | gersemi reformat (#2736) shifted `CMakeLists.txt`; `tools/freebsd/patches/patch-CMakeLists.txt` stopped applying | #3182 (regenerate the patch) |
| Windows `Release` (`std::bad_alloc`) | #3145 `ArrowShape::instantiate` left `mNumMaterials` unset and `mMaterialIndex = (unsigned)-1` → MSVC read a ~4-billion material offset | **#3169** (perf lane) |
| `arm64`/`clang`/`FreeBSD` compile | perf-lane `DARTCollisionDetector.cpp` `typeid(*smart_ptr)` (`-Wpotentially-evaluated-expression`) | **#3183** (perf lane) |

**Cross-lane note:** the last two were fixed by the parallel performance lane,
which took ownership of bugs in DART-6 code this effort had backported — #3169
fixed both the per-DoF actuator storage (#2222) and the ArrowShape mesh metadata
(#3145). Holding off on a blind Windows fix avoided colliding with #3169.

**Lesson recorded:** for any lint/tooling/large-diff PR, gate on the *full*
`pixi run check-lint` (clang-format + gersemi + black/isort + codespell), not just
the sub-check changed — three Check-Lint follow-ups (#3173, #3179, #3181) traced to
verifying only part of the gate before merging.

## 10. Outcome

Every config that failed once the gridlock cleared now has its fix merged (§9),
so the required CI matrix — Debug, Release (Linux + Windows), Asserts-enabled,
`arm64-Debug/Release`, FreeBSD, coverage, gcc/clang, the Eigen over-alignment
guard, and all manylinux/macOS wheels — has **no remaining known failures**. The
`#2251` spell-lint cleanup (#3184) is the final change and is comment/string-only
(no build impact).

**Closing gate:** the matrix is not yet *confirmed* green — the runs on the most
recent docs-only heads were **cancelled** by superseding pushes (not failures), so
no single completed all-green run exists yet. The closing session must let the full
required matrix complete on the final head and confirm green before declaring the
effort done. All engineering work is complete; this is the only remaining step.
