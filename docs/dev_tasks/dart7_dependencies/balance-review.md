# DART 7 dependencies: a balanced recommendation

For the maintainer, 2026-10-19. This is the second draft, revised after a critique of the first. Compared with the first draft, three things changed:
- The fmt and ImGui-patch recommendations are reversed (Q1, Q5).
- Q4 now says plainly that it re-opens DD-8.
- DEP-14 is treated as committed, and every packet is tagged by whether it must land before API freeze F.

Critique points this draft did not adopt, or adopted differently, are listed at the end.

**Basis.** This document is based on:
- `main` at `91275ea29`.
- DEP-14 as committed at `8dbc91568` on `refactor/dart-log-sink`: 2026-10-19 03:25, after four review rounds. Its Gazebo lanes on the Wave 1 integration tree are still owed.
- DEP-1 at `8e9b462b4` on `build/remove-boost`, and DEP-12(a) at `c476df42c` on `build/ubuntu-2404-toolchain-floor`.
- The follow-up study.
- A balance review of each item. Each item was argued once for self-containment and once for productivity, then judged against re-checked evidence under DD-9.
- A critique of the first draft. Its facts were re-checked read-only for this revision; no builds were run.

**Labels.**
- "Measured" means a build, a binary or a file on this machine.
- "Read" means code at the cited line.
- "Estimate" means a projected line count.
- Judgments are labeled as judgments.
- Confidence is high unless stated.

**Recommendation.**
- **Remove five packages in 7.0:**
  - Boost (DEP-1);
  - spdlog (DEP-14);
  - fmt (DEP-15, the small standard-library formatter that DD-4 records);
  - libcurl and OpenSSL, which go with the unreleased C++ asset retriever (DEP-19).

  If you keep fmt hidden instead (Q1(b)), the count is four.
- **Keep the rest, but narrow how it reaches users.**
  - urdfdom leaves the installed headers.
  - SDF becomes an optional part of `dart-io`, using the optional-dependency mechanism that Bullet and ODE already use.
  - The collision libraries get four trims.
- **Keep acquisition simple.**
  - Keep the per-dependency switches and add one umbrella switch.
  - Fetch only what DART keeps private or must pin exactly.
  - Libraries that Gazebo also loads come from the system or from pixi. This re-opens DD-8 for those libraries (Q4).
- **Stop patching Dear ImGui** once DART's own font path no longer needs the patch (Q5).

**Changes to the recorded plan.**
- DEP-15 stays, but without byte-for-byte parity with fmt. Under Q1(b) it becomes "Confine fmt" instead.
- DEP-2 and DEP-8(d) are dropped. DEP-4(b) is split into one PR per dependency.
- DEP-17 to DEP-20 are re-scoped.
- From the follow-up study, DEP-25, DEP-27 and DEP-28 are kept, DEP-25 in a smaller form.
- DEP-8(f) and DEP-30 to DEP-33 are added. DEP-30's form depends on Q5.
- DEP-26 and DEP-29 are not created.

Five questions need your preference (§4). §4 also lists the approvals and labeled changes the plan carries.

## 1. Summary

The basis for every number is in §2.

| Item | Today | Recommendation | Verdict | Net benefit | Confidence |
|---|---|---|---|---|---|
| spdlog | Public header-only backend of `Logging.hpp`. It reaches 369 of 721 DART TUs and every gz-physics dartsim TU | Remove it. A DART-owned sink built on the standard library replaces it (DEP-14, committed locally) | replace in-house | The compiled boundary, which every option needs, cuts `ContactSurface.hpp` parse memory by 33% and libdart `.text` by 25%. Removal itself takes spdlog out of every channel and takes DART's build out of the spdlog × fmt version matrix. It also exports 0 spdlog symbols (was 731) with no hiding convention. Costs: about 330 owned lines, a labeled log-format change, a narrower `DART_*` argument set (a source break) and `Threads` in DART's package config | High for removal. The margin over keeping spdlog privately is moderate (judgment) |
| fmt | After DEP-14: private and header-only in `Logging.cpp` and `XmlHelpers.cpp`. 85 + 75 symbols are still exported | Replace it with DEP-15's formatter for `{}`, `{:.N[efg]}` and escaped braces, as DD-4 records, without byte parity with fmt. Alternative, Q1(b): keep fmt hidden in one file until `std::format` is allowed | replace in-house (close call) | Owned code ends slightly lower: about +100-150 lines of formatter and tests, against about 165 lines of fmt calls and glue deleted. fmt leaves every channel, with no configure-time download and no hiding convention. Costs: the `DART_*` grammar narrows to the forms DART uses (142 `{}`, two `{:.Ne}`); one more packet before F; conda and pixi macOS builds need one define | Medium |
| Asset networking | Unreleased C++ `ModelResourceRetriever` in `dart-io` (default OFF, ON in pixi). libcurl downloads, OpenSSL hashes | Delete it before F. A Python helper that uses only the standard library replaces it and later becomes `dart.data` | replace with stdlib (Python) | Two packages leave. Production code: −757 C++ and CMake lines for +150-250 Python lines. Tests: −327 C++ lines and a rewritten Python suite. Nothing is frozen in 7.x. "One fetcher instead of two" depends on reading D22's "a fetcher" as covering the humanoids; Q2 asks you to confirm that | Medium-high |
| SDF (sdformat kept) | sdformat is required by all of `dart-io`; configure stops without it | Optional part of `dart-io`, using the optional-dependency mechanism Bullet and ODE use (warn and skip; `DART_SKIP_SDFORMAT`). An `io-sdf` forwarding component; sdformat exported only for static builds | make optional | `dart-io` builds where sdformat cannot exist: Homebrew core, Debian, the Ubuntu archive and `gz_dartsim_vendor`. About 40 CMake lines, no new SONAME and no new kind of switch. Cost: on SDF-off builds, legacy `utils` lacks `SdfParser`, which DART 6 always had | High |
| urdfdom (kept) | Floor 3.0.0; linked PUBLIC; two installed headers expose its types | Floor 4.0.0, glue cleanup, wheels on 6.0.1, and the urdfdom-typed headers retired before F | keep private | TinyXML v1 leaves; about 60 fewer production lines; one undefined-behavior bug fixed. No shared consumer needs three dev packages to configure any more. The format-string risk stays documented; release-6.20 wheels carry it too (a backport candidate) | High; header retirement medium-high |
| Acquisition (DD-8) | Nine BOOL `DART_USE_SYSTEM_<X>` switches; offline configures fail late | Keep the BOOL switches. Add one umbrella switch and an offline fail-fast as a second shared mechanical step. Fetch only private or exact-pin dependencies. For libraries shared with Gazebo, this re-opens DD-8 (Q4) | simplify; defer fallbacks | About 40-50 new CMake lines and one new frozen name, against an unprototyped estimate of 600-1,500 lines and 5-7 more names. No second copy of a library Gazebo loads | High for the direction; the alternative's size is an estimate |
| GLFW | System 3.4 required; noble ships 3.3.10 | System only, floor 3.3.10, with the X11 hint applied at run time on 3.4 and later | keep | Closes the noble gap in about 20 lines. A fetch would cost 100-200 lines and more X11 `-dev` packages, for a default that no CI lane would run | Medium-high |
| Dear ImGui | Git clone of v1.92.8-docking, then `git apply` of a 74-line patch | Fix DART's own font path (DEP-31). Then drop the patch (Q5) and fetch a SHA256-pinned tag archive with TLS verification (DEP-30). If you keep the patch, apply it without Git, into the build tree | keep the library; drop the patch | About 300 fewer owned lines and no Git. Fetched, pre-seeded and system ImGui behave the same. Cost: a labeled 7.0 change for third-party misuse of DART's re-exported ImGui | Medium; the patch is your call |
| libpng, libjpeg-turbo | Private system libraries of `dart-gui`; decode sizes unbounded | Keep them. Cap decodes with the existing 256 MiB budget. No switch. libpng's floor stays at its API minimum | keep | About 20 lines close the real gap: a 65500² JPEG header asks for about 17 GB. Wuffs would add an unpackaged alpha dependency and about 150-200 owned lines. stb_image still has unfixed 2023 CVEs in Debian | High |
| In-tree derived code | Code from ODE, libccd, convhull_3d, nanobind and Drake ships without third-party notices; 9 dead files | Keep and attribute what DART owns. Delete the dead files. Withdraw ImGui vendoring. Keep both test oracles | other: attribute, delete dead | −4,242 dead lines. Notices travel with every binary. The oracles stay as the only independent reference for the solver while the quiet window is open | High |
| octomap | Optional voxel backend, required in practice by FCL; leaks into `dynamics.hpp` | Keep it for 7.x. Forward-declare its types (one hunk goes through the PLAN-624 owner), pin 1.10.1, and fix a C++ ray overflow. A native grid only with an 8.0 build that has no FCL | keep | Takes octomap and FCL headers out of two dynamics headers and closes a heap overflow, for tens of lines. A native grid would save 0 packages | High; the 8.0 part medium |
| FCL, assimp, Bullet, ODE | Kept in core; gz-physics binds all four | Keep them in 7.0 and do four trims with corrected specs. assimp stays the 8.0 importer | keep | The allowlist compiles out about 74% of recent assimp CVEs (estimated −2.5-3.5 MB on the compressed Linux wheel), as a labeled difference between platforms. The Bullet trim takes 1.33 MB off the unpacked macOS wheel. About 125 dead lines go | High; allowlist size low-medium |

**Where the reviews disagreed.** This plan follows the more specific evidence.
- **GLFW.** The acquisition review made GLFW fetch-first. The GLFW review showed two problems with that:
  - on noble, a fetched X11 GLFW needs more apt packages than `libglfw3-dev`;
  - no CI lane would run the fetched default.

  A hidden copy would also force a rework of `test_FilamentBackend`'s shared-window seam, but that seam is internal, so the point weighs little. GLFW stays system-only, and `DART_USE_SYSTEM_GLFW` is not added. In DEP-25 the umbrella switch covers GoogleTest, Benchmark and Tracy. Filament joins in DEP-20(c), and fmt only under Q1(b). Dropping GLFW also takes about 30 lines off the acquisition estimate (60-80 → 40-50).
- **ImGui patch.** The in-tree review recommended dropping the patch. The ImGui review kept it and applied it without Git (DEP-30). The first draft followed the ImGui review because that changes no behavior. That was a status-quo tie-break:
  - the patch only guards against third-party misuse;
  - DD-7, DD-8 and DD-9 all favor dropping it once DART's own path is fixed.

  This draft recommends dropping it (Q5(a)) and keeps the Git-free edits as the fallback.
- **fmt.** The fmt review kept fmt on a tie-break, against a DEP-15 costed with byte parity against fmt. Costed like-for-like, a DEP-15 without parity ends with slightly less owned code and takes fmt out of every channel. DD-4's recorded decision therefore stands (Q1).
- **fmt in `XmlHelpers.cpp`.** It is removed under either Q1 answer, so `dart-io` drops fmt. Two overloads need care:
  - `toString(bool)` needs `v ? "true" : "false"`;
  - `toString(char)` needs `std::string(1, v)`.

  `std::to_string` would print `1` for `true` and `120` for `'x'`. Read: `XmlHelpers.cpp:104-107` and `:134-137` at `8dbc91568`. Both overloads are declared in the installed `XmlHelpers.hpp:53, 58`, and no test covers either.
- **Package count.** The urdfdom review's "5 fewer" holds under Q1(a): Boost, spdlog, fmt, libcurl and OpenSSL. Under Q1(b) it is four.
- **DEP-26.** The octomap and libpng reviews mention it only conditionally. It is not created, so octomap stays with the system FCL group.

**Re-checked for this revision.** All read-only, on `91275ea29`, `8dbc91568` or `origin/release-6.20` unless noted.
- **DEP-14 as committed:**
  - `Threads::Threads` PUBLIC (`dart/CMakeLists.txt:154`);
  - `Threads` in the core component's packages (`:238-239`);
  - thread flags on the `Libs` line of `cmake/dart.pc.in`;
  - the installed `cmake/DARTFindThreads.cmake`;
  - the `DART_*` argument restriction (`Logging-impl.hpp:119-134`) and its CHANGELOG "Breaking Changes" entry.

  Its build exports 85 + 75 fmt symbols and 0 spdlog symbols (`nm -DC`), with `.text` of 9,639,599 and 1,615,047 B (`size`).
- **The `DART_*` format strings.** There are 112 call sites in `dart/` and `python/`: 142 `{}`, one `{:.4e}` (`DantzigLcp.cpp:533`) and one `{:.6e}` (`DantzigLcp-impl.hpp:262`). This comes from the critique's scan script, run on the DEP-14 tree.
- **`XmlHelpers.cpp`:** all five fmt calls, including `toString(char)` at `:134-137`. The `dart/utils` fmt hunk is at `:64-76`.
- **macOS and floating-point `std::to_chars`:**
  - libc++'s availability marker requires macOS 13.3 (`_LIBCPP_AVAILABILITY_TO_CHARS_FLOATING_POINT` maps to `introduced = 13.3`);
  - conda-forge's macOS baseline is 11.0 (conda-forge news, 2026-02-06);
  - conda-forge documents the `-D_LIBCPP_DISABLE_AVAILABILITY` workaround (knowledge base, "Newer C++ features with old SDK").
- **Switch defaults and the optional-dependency macro.** The `DART_USE_SYSTEM_*` defaults are at `CMakeLists.txt:224, 274-298`. The optional-dependency macro is at `cmake/DARTMacros.cmake:142-182`; Bullet uses it at `dart/collision/bullet/CMakeLists.txt:7` and ODE at `dart/collision/ode/CMakeLists.txt:3`.
- **Quiet window.** `NativeShapeConversion.cpp` uses the octree directly (`:51-53`, `:308-318`). The quiet-window rule is at `dart7_python_first/README.md:192-194` and `decisions.md:129-131`.
- **release-6.20's legacy `utils`** builds `SdfParser` on tinyxml2 alone (`dart/utils/CMakeLists.txt:2-3, 21`). #3789 (`f630c91c8`, 2026-10-17) is in no tag.
- **urdfdom and console_bridge:**
  - release-6.20 pins urdfdom at `pixi.toml:50`, and its wheel image builds it at `Dockerfile.manylinux_2_28:69, 83`;
  - noble's `libconsole-bridge1.0` 1.0.1+dfsg2-3build1 imports `__vsnprintf_chk` and `__fprintf_chk` (`nm -D` on the extracted `.deb`).
- **main's wheel image** has 17 `git clone`s, and only the ODE download is hash-checked (`Dockerfile.manylinux_2_28:23-169`).
- **`gz_dartsim_vendor`, default branch** (`gh api`). It clones DART at build time through `ament_vendor`'s `VCS_URL`. It declares `fmt` but no sdformat, and requests gz-physics' four components.
- **gz-physics' component request:** `.deps/gz-physics/CMakeLists.txt:75-83`.
- **`libglfw3-dev` 3.3.10-1build1:** `libx11-dev` and `libxrandr-dev` are Recommends, not Depends (packages.ubuntu.com).
- **`CMAKE_TLS_VERIFY`** has defaulted to on only since CMake 3.31 (the CMake 4.3 docs in pixi).
- **`test_DantzigNative`** compares native and baseline solutions to `1e-10` (`test_DantzigNative.cpp:187-195`).
- **stb_image.** Debian's tracker lists CVE-2023-45661 as unfixed in sid's libstb 0.0~git20250907, and as postponed on bookworm and trixie.
- **CMake overlap between packets.** DEP-19 PR2 touches `dart/utils/CMakeLists.txt:103-105` and `assets/CMakeLists.txt:25`. DEP-18 and DEP-17 PR4 share `dart/utils/CMakeLists.txt:53-68`.
- **GitHub code search** (2026-10-19). Outside DART and its forks, no `DART_*` call uses a spec other than `{}`; the index is partial, so this is weak evidence. Outside projects do include DART's re-exported ImGui, for example KavrakiLab/robowflex and DIAG-Robotics-Lab/joint_level_ismpc.
- **Checked for the first draft and still current:**
  - D22's "sample data and a fetcher", and the `dart.data` must-have;
  - wheels building with assets OFF;
  - the `utils-assets` forwarding pattern;
  - the sdformat FATAL;
  - the ImGui patch hunks, and the single reporter of #2516, #2668 and #2671;
  - the broken `uninstall` target;
  - the dead files, and IKFast's removal in #3824;
  - the unchecked ray insertion;
  - the include path from `urdf.hpp` to `BackwardCompatibility.hpp`.

## 2. Per item

### 2.1 spdlog → DART log sink (DEP-14)

**Recommendation.** Merge DEP-14 as committed (`8dbc91568`) once two things are done: its Gazebo lanes pass on the Wave 1 integration tree, and its migration note is written. The first draft folded several follow-ups into "one small round". They now move out, so the reviewed commit is not reopened:
- the fmt work goes to DEP-15;
- the Windows console check goes to DEP-16.

**Gains from the compiled boundary.** Every option gets these, including keeping spdlog behind the boundary. They were measured with GCC 15.2 and the flags from `build.ninja`, comparing the DEP-14 build with its `main`-based base.
- **Parse memory.** `ContactSurface.hpp` 500 → 337 MB and `Skeleton.hpp` 583 → 388 MB, both −33% (`c++ -fsyntax-only`). Parse time 3.4 → 1.7 s, at medium confidence because the host load was about 52 on 64 cores.
- **Code size.** `libdart` `.text` 12.86 → 9.64 MB (−25%); `libdart-io` 2.40 → 1.62 MB (−33%) (`size`).
- **Fixes:**
  - the pkg-config ODR split: the log level was a compile definition that `dart.pc` never carried;
  - dartpy mangling braces in log text;
  - the GUI log panel never seeing `DART_*` output.
- **The M5 sink**, which M5's Python bridge requires.

**Gains from removal itself.**
- **spdlog leaves every packaging channel:** pixi, the Brewfile, FreeBSD CI and the manylinux image now, and at their next bumps vcpkg's `spdlog` feature and conda's host dependency.
- **DART's build leaves the spdlog × fmt version matrix** (noble's spdlog 1.12 to 1.17, against fmt 9.1 to 12). If spdlog stayed behind the boundary, `Logging.cpp` would still compile spdlog. A break like dart#2538 (spdlog 1.17 with fmt 12.1 rejecting runtime format strings) would then still stop DART's own build, though no longer its consumers' builds.
- **Exported spdlog symbols go from 731 to 0** (`nm -D`) with no hiding convention. This matters because Gazebo loads its own `libspdlog.so.1.15`, and spdlog has no versioned namespace. Keeping spdlog would require hiding it. Hiding would also cut users' `spdlog::set_level` off from DART's private registry, which is that option's one user-visible advantage besides timestamps.
- **Nothing left to break:** no registry shutdown ordering to manage, and no finder like the one dart#3503 broke.

**Costs.**
- **Code.** Logging code grows from about 352 to about 682 lines (`Logging.hpp` 146 → 188, `Logging-impl.hpp` 206 → 233, new `Logging.cpp` 261; `wc -l` at `548ff24ad` and `8dbc91568`). Tests grow from 4 to 19.
- **Ownership.** DART now owns colors, TTY detection, thread safety and shutdown. The first two review rounds produced 15 and 8 fixes; rounds 3 and 4 found nothing substantive.
- **Labeled output change:**
  - no timestamps;
  - `warn`/`fatal` replace `warning`/`critical`;
  - warn and above go to stderr.
- **Lost routing.** `spdlog::set_level` no longer reaches DART output. A 5-line `setLogSink` adapter restores the routing.
- **Source break.** `DART_*` arguments must now be integers, floating-point values, strings, pointers or enums. A type with only an `fmt::formatter` or an `operator<<` stops compiling (`Logging-impl.hpp:119-134`). Projects that got fmt through `find_package(DART)` must now find it themselves. DEP-14's CHANGELOG lists both under "Breaking Changes", and §4 asks you to approve them.
- **Package config.** Removing spdlog showed that DART never linked Threads itself. spdlog's target had supplied it, and the conda-toolchain Gazebo lane failed with an undefined `pthread_create`. DEP-14 now:
  - links `Threads::Threads` PUBLIC (`dart/CMakeLists.txt:154`);
  - installs `cmake/DARTFindThreads.cmake`, which runs `find_package(Threads REQUIRED)` in every core consumer, gz-physics included;
  - adds `Threads` to the core component's packages (`:239`);
  - adds the thread flags to `dart.pc`.

  PLAN-624 asks for "zero new core dependencies" (`dart7_python_first/README.md:187`). Threads is a platform library that replaces what spdlog supplied, so this probably fits, but it is a labeled package-config change (§4). The conda-toolchain DART build, and a gz-physics build against it, both pass (`verification.md`, DEP-14).
- gz-physics and gz-sim use none of the logging API.

**Why.** Kept behind the same boundary, spdlog would replace only about 30 lines (the default sink and colors), and it would add timestamps and native Windows colors. In exchange it would:
- keep a row in every packaging channel;
- keep the version matrix in DART's own build;
- need a hiding convention, which cancels its user-visible advantage.

Under Q1(a) it would also block removing fmt, because spdlog needs fmt until `std::format` is allowed. Under Q1(b) that point does not distinguish the options, since fmt stays anyway.

There were three spdlog/fmt incidents in five months:
- dart#2538 and gz-physics#1018 reached consumers because DART's headers instantiated spdlog and fmt. The boundary alone fixes that part.
- dart#3503, and the build-side half of dart#2538, are what removal fixes.

Removal still wins, but by a moderate margin rather than the wide one the first draft showed (judgment, medium-high).

### 2.2 fmt after DEP-14 (DEP-15)

**Recommendation (Q1(a)).** Do DEP-15 as recorded (`packets.md:233-241`), but without byte parity with fmt:
- **Formatter.** Replace the fmt call in `Logging.cpp` with a formatter for `{}`, `{:.Ne}`, `{:.Nf}`, `{:.Ng}`, `{{` and `}}`, over DEP-14's closed set of argument types. Anything else renders as today's error marker plus the raw text (`Logging.cpp:242-243`).
- **Tests.** Its table test pins expected strings. float and double follow fmt's layout for `{}`. long double does not need to: the prototype switches to exponent form at 1e19, where fmt switches at 1e16.
- **`XmlHelpers.cpp`.** Move its five calls to plain `std::string` code, with tests for `toString(bool)` and `toString(char)`.
- **Deletions.** Delete `DARTFindfmt.cmake`, its fetch, every fmt pin, the image step and the `dart/utils` fmt hunk (`:64-76`). Keep `DART_USE_SYSTEM_FMT` as a no-op through 7.x.
- **Follow-ups.** Take on the two DEP-14 nits that `verification.md` moved here.
- **Gate:** `nm -DC` shows 0 `fmt::` and 0 `spdlog` symbols in `libdart` and `libdart-io`.

DEP-15 changes behavior, so it must land before F. If it misses F, fmt stays hidden through 7.x, as in Q1(b).

**Like-for-like comparison.** The first draft costed DEP-15 with byte parity against fmt, and left two costs of keeping fmt out. Costed on the same basis:

| | (a) DEP-15 without parity | (b) Keep fmt hidden ("Confine fmt") |
|---|---|---|
| Owned code | About 100-150 lines of formatter and tests (estimate). A local prototype is 84 lines and covers the grammar and every argument type. It deletes about 35 lines of fmt calls in `Logging.cpp` and about 130 net lines of find, fetch, pin, image and test glue | A 6-line pragma, nm and include gates, and a URL-and-hash fetch: about 20 lines in all. The 130 lines of glue stay |
| Packaging | fmt leaves pixi, Homebrew, FreeBSD, the manylinux image, vcpkg and conda | fmt stays in every channel. Under DD-8's fetch-first default, every source build without the umbrella downloads fmt at configure, core-only builds included. Today they use system fmt (`DART_USE_SYSTEM_FMT` defaults to ON, `CMakeLists.txt:274-278`), so this is a labeled default change. Keeping it system-first instead adds one more DD-8 exception |
| Behavior | The `DART_*` grammar narrows to the forms DART uses: 142 `{}` and two `{:.Ne}` across 112 call sites. `{:>8}`, `{:x}` and `{0}` print the error marker. This is a labeled change: one more line in DEP-14's existing logging breaking-change entry. Before DEP-14, builds without spdlog did no formatting at all | Unchanged |
| Platforms | The first floating-point `std::to_chars` in core. libstdc++ has it at the GCC 13.2 floor. libc++'s availability markers require macOS 13.3. Wheels target 15.0 (`publish_dartpy.yml:125`), but conda-forge's baseline is 11.0, so conda and pixi macOS builds add `-D_LIBCPP_DISABLE_AVAILABILITY`. conda-forge documents that define for its own libc++ | None |
| Evidence | Table tests of expected strings | The hiding is a convention. It has been shown only on a probe TU with fmt 11.2 on Linux (GCC 15.2: 110 → 0 symbols; Clang 21.1: 151 → 0). It still needs the nm gate on the real `libdart` with noble's fmt 9.1, with fmt 12 and on macOS (`nm -gU`), plus a gate that only `Logging.cpp` includes `<fmt/...>` |
| Static builds | Nothing left over | fmt's weak or COMDAT instantiations can still merge with an application's own fmt |
| Record | As DD-4 records (`decisions.md:24-27`) | Re-opens DD-4 (`decisions.md:3`) |
| Schedule | One more gated packet before F | Nothing tied to F except the default change |
| Exit | None needed | A port to `std::format` once the GCC floor reaches 16, about 40 lines. That is not expected in 7.x: libstdc++ still exports 62 `std::__format` symbols under the pragma (measured), and Ubuntu 26.04 and 26.10 default to GCC 15 |

**Why (a).** The fmt review broke its tie on three criteria: less owned code, simpler, and no public behavior change. Costed like-for-like:
- (a) now has less owned code and one dependency fewer;
- (b) keeps the grammar unchanged;
- (b) also brings a configure-time download, and a hiding convention not yet verified on the real library.

DD-9 allows keeping a private, well-maintained dependency, but here it shows no net benefit for keeping it. `decisions.md:3` reopens a decision only on new evidence, and the new evidence makes keeping fmt possible, not better. So DD-4's recorded decision stands, in its cheaper form.

Judgment, medium: this is a close call. It flips to (b) if you value an unchanged `DART_*` grammar, or if the conda macOS define is unwelcome.

**Outside users.** A GitHub code search found `DART_*` calls outside DART only in dartsim/dart-optimization and in DART forks, all using `{}`. GitHub's index is partial, so this is weak evidence (low-medium).

### 2.3 Asset networking (DD-6, DEP-19)

**Recommendation.** Option (e), in three PRs.
- **PR1.** A helper built only on `urllib`, `ssl` and `hashlib`, which does not import dartpy. It keeps today's policy:
  - HTTPS-only redirects, at most 5;
  - a byte cap;
  - connect and low-speed timeouts;
  - CA-bundle overrides;
  - atomic publication;
  - SHA-256 and size checks on fetch and again before load.

  It also keeps today's cache layout. Codex writes it, because the routing table gives security work to Codex, and Claude reviews it.
- **PR2.** Delete the unreleased C++ feature. This needs your approval. It removes curl and OpenSSL in one PR. Both exist only for this feature and are added in one file (`dart/utils/assets/CMakeLists.txt:2-25`), so this is the deletion of one feature. It is stated as an exception to one-dependency-per-PR (§4).
- **PR3.** The helper becomes `dart.data` in M5a step 2.

**Gains.**
- **One fetcher instead of two, if D22 means what this plan reads it to mean.**
  - For this reading: D22's conventions bundle lists "sample data and a fetcher" (`dart7_python_first/decisions.md:42`). Wheels build with assets OFF (`CMakeLists.txt:179`; `setup.py` passes no assets flag), so a fetcher for wheel users has to be Python.
  - Against it: the PLAN-624 must-have is "the `dart.data` sample data" (README `:127`), and its acceptance test loads in-repo KR5 data (`test_target_shape.py:168`). No PLAN-624 document names the humanoid bundles.

  Reading "a fetcher" as covering the humanoids is an interpretation, and Q2 asks you to confirm it. Without that reading, (e) still wins on the points below, and PR3 becomes additive work that may slip to 7.1.
- **Less code.**
  - Production: −757 C++ and CMake lines (85 + 618 + 36 + 9 + 9; `wc -l`), for about +150-250 Python lines (estimate).
  - Tests: −327 C++ lines. The 566-line Python suite is rewritten against the helper, and it gains ports of 11 C++ path-safety tests (estimate: +100-150 lines).
- **Two packages out.** libcurl and OpenSSL leave DART.
- **Nothing frozen.** No networked C++ API is locked in for 7.x. The feature never shipped: `git tag --contains 08085d737` is empty.
- **More users get the scenes.** After PR3, wheel and conda users get the two humanoids through `dart.data`; today only pixi builds with assets ON do. C++-only installs get no installed fetch path. They can run the repository script, or use CMake's `file(DOWNLOAD ... EXPECTED_HASH ... TLS_VERIFY ON)`.

**Costs.**
- **Effort M:**
  - rewrite the assertions of the 566-line Python suite;
  - port 11 C++ path-safety tests;
  - edit about 31 referencing files.
- **Download policy.** DART owns about 30 lines of download policy. By default, Python's `urllib` follows https→http redirects, up to 10 of them.
- **No `model://` in C++.** It has no caller today, because #3780 and #3824 moved the demos to Python.
- **Python import.** "Does not import dartpy" holds for PR1's repository script only. After PR3, importing `dart.data` imports dartpy.
- **It reverses your #3653.**

**Not claimed.** These gains are small:
- **Footprint.** The curl closure maps 45.6 MiB, but only 5.4 MiB stays resident after import, and only in pixi builds with assets ON (`/proc/self/smaps`).
- **CVEs.** None of curl's 2026 High or Medium CVEs affects an HTTPS GET over OpenSSL.
- **The pixi environment does not shrink.** cmake, hdf5 and python still need curl or OpenSSL.

**Why.** Keeping C++ means:
- two fetchers sharing one cache contract, if D22's fetcher covers the humanoids;
- a frozen C++ online API with no caller;
- two dependencies that DD-8 cannot sensibly fetch.

(e) can be reversed in 7.x by adding a C++ resolver later. Keeping the C++ API cannot be reversed after 7.0.

### 2.4 SDF packaging (DD-5 follow-up, DEP-18)

**Recommendation.** Make SDF optional inside `dart-io`, the pattern #3672 used for `utils-assets` and the collision backends. Use the optional-dependency mechanism that Bullet and ODE already use, not a new tri-state switch.
- **Detection.** `dart/utils/sdf/CMakeLists.txt` calls `dart_check_optional_package(SDFORMAT ...)` (`cmake/DARTMacros.cmake:142-182`), as `dart/collision/bullet/CMakeLists.txt:7` does.
  - With sdformat 14.8 to 16, SDF is built.
  - Without it, configure warns and skips SDF.
  - `DART_SKIP_SDFORMAT=ON` skips it on purpose.
- **Strictness.** DEP-4(e) adds an opt-in strict switch that turns a missing Bullet or ODE into an error; it also covers sdformat. DART CI (pixi `config`), the wheels, the gz lanes and packagers that ship sdformat set it, so SDF cannot drop out silently.
- **When SDF is built:**
  - an INTERFACE `dart-io-sdf` target and an `io-sdf` component forward to `dart-io` (copy `dart/utils/assets/CMakeLists.txt:27-33`);
  - `config.hpp` gets `DART_HAVE_SDFORMAT`.
- **Exports.** sdformat is exported only for static builds.
- **Unchanged.** The legacy edges stay: `utils` → `io` and `utils-urdf` → `utils`.
- **Never fetched.** sdformat stays system-only.

**Gains.**
- **`dart-io` builds where sdformat cannot exist.** Today a missing sdformat stops configure for all of IO (`dart/utils/CMakeLists.txt:16-24`). sdformat cannot exist in:
  - Homebrew core, which may not depend on the osrf tap (`formula_auditor.rb:402-412`), even though osrf's gz-physics8 depends on core `dartsim`;
  - `gz_dartsim_vendor`, which declares no sdformat;
  - Debian trixie and sid, and the Ubuntu 24.04 archive.
- **Faster consumer configures.** Shared consumers, every gz-physics configure included, stop looking up DART's exact sdformat major.
- **Lighter builds without SDF.** They skip about 4.9 MB of libraries and 6.8 MB of schemas (pixi, `stat`).
- **One mechanism.** No new kind of switch; optional SDF behaves like optional Bullet and ODE.

**Costs.**
- **Code.** About 35-45 CMake lines (estimate), plus gating about 22 consumer files. A separate library would need the same gating.
- **CI.** One more configuration: IO on, sdformat lookups off.
- **Symbols.** `libdart-io`'s symbols vary by build. `libdart` already does this for Bullet, ODE and octomap.
- **Compared with DART 6, SDF-off builds lose `SdfParser` from legacy `utils`.** release-6.20's `utils` always built `SdfParser` on tinyxml2 alone (`origin/release-6.20:dart/utils/CMakeLists.txt:2-3, 21`). #3789 (`f630c91c8`, 2026-10-17, unreleased) moved it onto sdformat. Homebrew core, Debian, the Ubuntu archive and `gz_dartsim_vendor` would ship `utils` without it. That changes a legacy component's contents, so it is part of Q3. It is still the right trade under DD-5, which keeps sdformat as the SDF parser (judgment).
- **Package contents depend on what is installed** for builders who set no switch, such as local builds of distro recipes (judgment, medium). Mitigations:
  - the warning and the summary line;
  - `DART_HAVE_SDFORMAT`;
  - the `io-sdf` component;
  - DEP-23's per-channel table.

  Q3(b) avoids the problem by failing instead.
- **Labeled package-config changes** (Q3).

**Why.** A separate library would add:
- a fourth SONAME that every channel carries through 7.x;
- new link edges for dartpy and `tools/package`;
- a legacy-edge rewiring that needs approval.

Its isolation would help only C++ processes that load just `dart-io`. Gazebo loads its own sdformat, and `--as-needed` and `-dead_strip_dylibs` drop unused entries on Ubuntu and conda. A real library can still come in 8.0, behind the same `io-sdf` name.

### 2.5 urdfdom kept (DEP-17)

**Recommendation.**
- urdfdom stays system-only, with a floor of 4.0.0.
- Clean up DART's URDF glue.
- Move the wheels to urdfdom 6.0.1. This needs your approval (§4).
- Retire `IncludeUrdf.hpp` and `urdf_world_parser.hpp` from the installed headers before F. This needs approval, and it waits for no other packet except DEP-18, which edits the same lines.
- Document the format-string bug in 4.0.0-5.0.4 rather than patching around it.

**Gains.**
- TinyXML v1 (last updated 2011) leaves: `FindTinyXML.cmake` (45 lines) and two lines in `DARTFindurdfdom.cmake`.
- The duplicate joint-metadata parser goes, −34 lines. `diff` of `DartLoader.cpp:70-103` against `urdf_world_parser.cpp:84-117` is empty.
- A null-attribute undefined-behavior bug is fixed (`urdf_world_parser.cpp:172-175, 192-193`).
- Net production code shrinks by about 60 lines (85 deleted, 24 added).
- After PR4, no shared consumer needs liburdfdom-dev, liburdfdom-headers-dev or libconsole-bridge-dev to configure.

**Costs.**
- **Ports.** The vcpkg (3.1.1) and FreeBSD (3.0.2) ports must be bumped. Neither builds DART 7 today.
- **World parser.** DART's 351-line world parser stays, because urdfdom's `parseWorld` is a stub through 6.0.1.
- **Residual risk.** urdfdom 4.0.0-5.0.4 passes URDF text to printf as a format string at about 11 sites. noble's console_bridge is built fortified: `libconsole-bridge1.0` 1.0.1+dfsg2-3build1 imports `__vsnprintf_chk` and `__fprintf_chk` (`nm -D` on the extracted `.deb`). On Ubuntu the impact is therefore a crash or a leak into the log.
- **Exposure that has already shipped.**
  - release-6.20 pins `urdfdom >=4.0.1,<5` (`pixi.toml:50`).
  - Its wheel image builds urdfdom 4.0.1 with console_bridge 1.0.2 from source (`Dockerfile.manylinux_2_28:69, 83`).
  - The urdfdom review found that main's nightly wheel bundles an unfortified console_bridge.

  PR3's wheel bump is a backport candidate for release-6.20 through `dart-backport-pr`. That needs approval, and it would widen 6.20 wheels to accept URDF 1.1 and 1.2.

**Why.**
- The floor follows DD-1.
- PR4 is the only step that lowers what urdfdom costs DART's consumers. Under D5 it is now or 8.0, so it should not queue behind unrelated packets.
- A version-gated `%` guard would be 20-40 lines that duplicate an upstream fix. It would protect only 24.04-era source builds that load untrusted URDF. Those builds already face assimp's larger attack surface (70 CVEs since 2023-10).
- The deprecation message for `BackwardCompatibility.hpp` says there is no replacement. `URDFTypes.hpp`, deprecated since 6.2, still tells users to include `BackwardCompatibility.hpp` instead (`URDFTypes.hpp:36-38`), so its message gets the same wording.

### 2.6 Acquisition mechanics (DD-8)

**Recommendation.**
- **Keep the BOOL `DART_USE_SYSTEM_<X>` switches.**
- **Add an umbrella switch (DEP-25).** `DART_USE_SYSTEM_DEPENDENCIES` (BOOL, OFF) becomes the default value of the fetch-first switches that a distro can satisfy. An explicit `-D` always wins, because `option()` keeps an existing cache value.
  - DEP-25 wires only GoogleTest, Benchmark and Tracy. Their defaults are already OFF (`CMakeLists.txt:290-298`), so nothing changes while the umbrella is OFF.
  - Filament joins in DEP-20(c), and fmt only under Q1(b), each in its own PR, because each changes a default that is ON today (`:224`, `:274-278`).
- **Offline behavior.**
  - Set CMP0170 to NEW where CMake has it (3.30 and later).
  - Add a check of 20 lines or fewer that stops a disconnected configure at once and names the switch. DEP-25 calls it at every existing fetch site.
- **A second shared mechanical step.** DEP-25 touches several dependencies' fetch sites but changes no version, source or default. `README.md:104-105` allows only one shared step today, the toolchain floors, so this needs your OK and a README edit (§4).
- **Fetch first, by URL and hash, each in its own packet:**
  - ImGui (DEP-30);
  - nanobind (DEP-10);
  - GoogleTest and Benchmark (DEP-6);
  - Tracy (DEP-22);
  - fmt, only under Q1(b).
- **System only, with a documented install command:** Eigen, libccd, octomap, FCL, assimp, tinyxml2, the urdfdom group, sdformat, GLFW, libpng and libjpeg-turbo.
- **ODE and Bullet.** They keep system-first, with a warning and a drop when missing. A DART configured without them lacks `collision-ode` and `collision-bullet`, which gz-physics requires (`.deps/gz-physics/CMakeLists.txt:75-83`). Gazebo-facing packagers therefore set DEP-4(e)'s strict switch. The opt-in fetch gets a minimal repair (DEP-4(b)).

**This re-opens DD-8 for the libraries Gazebo also loads.** The history is:
1. The original Q4 proposed system-first with fetch fallbacks for these libraries (`decisions.md:98-102`).
2. You answered "fetch first for build from source" (DD-8).
3. This plan now proposes system-only.

Under `decisions.md:3`, that needs new evidence and your approval. The new evidence:
1. **Fallbacks would serve one case.** Every builder the study lists except a bare `cmake` passes the umbrella (`synthesize.md:276-286`), so fallbacks would save that one case one apt line. Stock noble has every one of these libraries at the floors; S3 built core and `dart-io` from them.
2. **They need sub-builds.** FCL 0.7.0, urdfdom 4.0.0, urdfdom_headers 2.1.0 and ODE 0.16.6 define unguarded `uninstall` targets (verified by the acquisition review), so fallbacks need configure-time sub-builds. The one heavy fallback DART has, the ODE fetch, has never had a CI caller. It cannot configure today, because its `uninstall` target clashes with DART's (`CMakeLists.txt:726-737`).
3. **They would duplicate Gazebo's libraries.** Fallbacks would install about 26.9 MB of shared libraries with upstream SONAMEs into DART's prefix (pixi, `stat`). These would be second copies of libraries Gazebo loads, with no design yet for Windows DLLs or macOS install names.
4. **Their cost is an estimate.** The 600-1,500 lines were not prototyped. If you lean towards fallbacks, a one-dependency prototype (tinyxml2) should bound the cost first.

**Gains.**
- **Code.** About 40-50 new CMake lines (estimate), against the 600-1,500-line estimate for the study's tri-state, provider-record and sub-build framework. Both acquisition verifiers rated that framework high effort.
- **Frozen interface.** One new packager name, the umbrella, instead of the study's 7-9.
- **No migration.** No BOOL-to-STRING change across about 40 CMake references and 39 in `pixi.toml`.
- **No duplicate libraries** in DART's prefix.
- **CVE duty does not spread.** DART already owns pins for the Linux wheel: its image builds assimp v6.0.5, FCL 0.7.0, tinyxml2 11.0.0, urdfdom 4.0.1, octomap, libccd, Bullet 3.25 and others from 17 `git clone`s. Only the ODE download is hash-checked (`Dockerfile.manylinux_2_28:23-169`). Fallbacks would extend that duty from one image to every source build that takes them. DEP-8(f) adds hash checks to the image's sources.

**Costs.**
- A bare `cmake` build on stock noble still needs one apt line of about nine `-dev` packages. All of them are in the archive at the floors.
- Windows and macOS source builds still need a package manager. pixi is the documented route.
- It re-opens DD-8 for the shared libraries (Q4).

**Why.** With find-or-fetch, every builder except a bare `cmake` would set the umbrella anyway. The fallbacks would serve that one case, and only after the expensive FCL, assimp and urdfdom sub-builds land. Find-or-fetch stays possible as an addition in 7.x if a need appears.

### 2.7 GLFW

**Recommendation.**
- System only, private to `dart-gui`, floor 3.3.10.
- One `dart/gui/detail` helper applies `glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11)` only when `glfwGetVersion` reports 3.4 or later at run time. The call sites are `Viewer.cpp:107` and `test_FilamentBackend.cpp:1509, 1610`.
- pixi keeps 3.4. Wheels keep their static, X11-only 3.4, now fetched as a hash-checked release archive (DEP-8(f)).
- DEP-20(b) is additive. It lands with the DEP-20(a) spike and is not tied to F.

**Gains.**
- About 20 changed lines (estimate).
- **Fewer apt packages on noble.** `libglfw3-dev` brings `libx11-dev` and `libxrandr-dev`, which `glfw3native.h` needs, as Recommends rather than Depends (packages.ubuntu.com/noble/libglfw3-dev). With `--no-install-recommends`, list them explicitly. A fetched X11 GLFW 3.4 needs six X11 `-dev` packages.
- No new switch or pin.
- 0 NVD CVEs for GLFW.

**Costs.**
- Two GLFW minor versions to support on Linux through 7.x.
- The 3.3 path has no CI until the noble Tier 2 lane (DEP-20(a)) exists, so land it with that spike.
- A guard of about 8 lines.
- A recorded DD-8 exception (Q4).

**Why the runtime check, not `#if`.** A DART built against 3.3 can load a 3.4 `libglfw.so.3`, which has the same SONAME. That library would pick Wayland and break Filament's GLX path.

**Why not a fetched GLFW.** The costs that decide it:
- more X11 `-dev` packages on Linux;
- 100-200 lines of CMake, including a static-export rule;
- a fetched default that no CI lane would run, because pixi and the wheels supply GLFW.

A copy hidden in `libdart-gui` would also be a second GLFW instance for `test_FilamentBackend`, which passes windows from its own GLFW to `getFilamentNativeWindow`. That function is an internal seam: it is declared in `dart/gui/detail/native_window.hpp`, which is not installed (only `dart/gui/*.hpp` is; `dart/gui/CMakeLists.txt:96, 161-165`). The test could therefore be reworked, so this point weighs little.

### 2.8 Dear ImGui (DEP-30, DEP-31)

**Recommendation.**
1. **Now: DEP-31.** Reject an empty font file in `OverlayRenderer::prepareFonts` (`overlay_adapter.cpp:177-191`). DART's own `setFont` path then stops relying on the patch. System-ImGui builds already lack the patch today.
2. **Then, as a labeled 7.0 change you approve (Q5):** drop `dart-compat.patch`, the CMake that applies it and refreshes its tag stamp, and the three regression tests that exercise it. Those are `test_Issue2516`, `test_Issue2668` and the hidden-symbol half of `test_Issue2671`.
3. **DEP-30.** Fetch ImGui as GitHub's v1.92.8-docking tag archive, with:
   - `URL_HASH SHA256`;
   - `TLS_VERIFY ON`;
   - `DOWNLOAD_EXTRACT_TIMESTAMP`, with no version guard needed after DEP-12(a)'s CMake 3.28 floor.

   Keep `dart_imgui`, `DART_USE_SYSTEM_IMGUI` and pre-seeding. Do not vendor.
- **Fallback if you keep the patch (Q5(b)).** DEP-30 applies the patch's edits without Git. It writes the edited files into the build tree instead of editing a pre-seeded source in place, and configure fails unless every marker is present.

**Gains.**
- **No Git for a GUI-only packager configure.** `dart/gui/imgui/CMakeLists.txt:160` is DART's only unconditional Git requirement. Default developer configures still clone nanobind, GoogleTest and Benchmark until DEP-6 and DEP-10 land.
- **A content-addressed source.** A moved tag can no longer change what is built. Git verifies TLS by default, but CMake verifies TLS for URL downloads by default only from 3.31 (`CMAKE_TLS_VERIFY` docs), and noble ships 3.28. The switch to a URL download therefore passes `TLS_VERIFY ON`, so transport security does not regress.
- **Less owned code.** About 300 fewer owned lines with the patch dropped: the 74-line patch, about 105 lines of apply and stamp CMake, and three regression tests. About 100 fewer if the patch is kept (−177, +60-80).
- **The same behavior everywhere.** Fetched, pre-seeded and system ImGui behave the same. The system-ImGui lanes (asan, coverage, install, dart-gz) already run without the guards.
- **A latent silent skip goes away.** Inside DART's work tree, `git apply` on a tree without `.git` returns 0 and skips every hunk (reproduced).
- **About 6x less download.** A 2,252,145-byte archive instead of a 13.76 MiB shallow pack.
- **Identical sources** to a checkout of the tag (`diff -rq`).

**Costs.**
- **A labeled 7.0 change (Q5(a)).** Third-party code that misuses DART's re-exported ImGui gets upstream's assert or crash. Misuse here means null font data, or calls without a context. ImFormatString is also exported again. This reverts the 6.16.x fixes for #2516, #2668 and #2671, all from one reporter in 2026-02 and 2026-05. Outside projects do include DART's ImGui; a GitHub search finds KavrakiLab/robowflex and DIAG-Robotics-Lab/joint_level_ismpc, mostly from the DART 6 OSG era. The change affects only misuse, which a search cannot show.
- **Archive bytes.** GitHub does not promise archive bytes forever. A changed archive fails closed, and DART already accepts this risk for Bullet and the Filament source.
- **With the patch kept:** 60-80 lines of edit logic, and five anchors to re-check on each bump.

**Why.** All three relevant decisions favor dropping the patch:
- DD-7 prefers a clean fix over a compatibility shim, and gz never requests the gui component.
- DD-8 removes bundled code whenever possible, and a patched copy is code DART owns.
- DD-9 asks for a net benefit, and the patch protects only against third-party misuse once DEP-31 fixes DART's own path.

The first draft kept the patch on a status-quo tie-break. Confidence is medium, and it is your call (Q5).

The rejected alternatives:
- Vendoring re-adds 63,160 lines; DART deleted the vendored tree twice, in #2056 and #3081.
- System-first would silently drop docking for most users.
- Plain `git apply`, or a `PATCH_COMMAND` (which never runs on a pre-seeded source), would ship an unpatched ImGui with no error. This matters only if the patch is kept.

### 2.9 libpng and libjpeg-turbo

**Recommendation.**
- Keep both as private system libraries with no switch. This is a recorded DD-8 exception.
- Re-scope DEP-20(d):
  - move `loadPngImage` and `loadJpegImage` into `image_decoders.cpp`;
  - allocate through the existing `makeImage()` and `kMaximumDecodedBytes` (256 MiB);
  - check JPEG dimensions before `jpeg_start_decompress`, and PNG dimensions before `resize`.

  Codex writes it, because it is security work, and Claude reviews it.
- Drop DEP-8(d).
- Keep libpng's floor at its API minimum, 1.6.0.

**Gains.**
- About 20 lines and two tests close the real gap. Today a 65500×65500 JPEG header asks for about 17 GB (`textures.cpp:183-184`).
- Security fixes arrive without DART. noble and AlmaLinux 8 shipped every 2025-26 libpng fix that reaches DART's calls, and the nightly wheel already bundles 1.6.34-11.el8_10.
- The GUI test images stay unchanged.

**Costs.**
- DART keeps 114 lines of decode glue and a 22-line PNG writer.
- libpng's CVE stream continues: 11 in Red Hat's feed in Jan-Sep 2026, and CVE-2025-65018 reaches DART's decode path.

**Why.** Both libraries meet DD-9's test for staying. The alternatives are worse:
- **Wuffs:**
  - no package manager ships it;
  - it has been alpha since 2023-10;
  - it would add about 150-200 owned lines;
  - it drops arithmetic-coded and 12-bit JPEGs.
- **stb_image.** Filament's SDK already ships it as libstb, but it is not a better swap. Debian sid's libstb 0.0~git20250907 still lists CVE-2023-45661, from the GHSL-2023-145..151 batch (CVE-2023-45661 to 45667), as unfixed. bookworm and trixie mark it "Minor issue, revisit when fixed upstream".

libpng's floor cannot rise to noble's 1.6.43, because no RHEL-based manylinux image ships it: AlmaLinux 8 has 1.6.34, RHEL 9 has 1.6.37 and RHEL 10 has 1.6.40.

### 2.10 In-tree derived and copied code

**Recommendation.**
- Keep and attribute the code DART owns:
  - Dantzig (from ODE), ConvexHull (convhull_3d) and MPR (libccd);
  - the URDF world parser;
  - the Drake helpers;
  - `Components.cmake`;
  - the nanobind caster fork.
- Delete the dead files.
- Withdraw "ImGui vendored in-tree in 7.x".
- Keep both test oracles.
- Do not open a packet to retire the caster fork.
- IKFast is settled: #3824 removed it.

**Gains.**
- −4,242 lines with no effect on the build: 7 `dart/utils/*_` files (3,918 lines, including GPL-2+ Bison output) and `tests/.deprecated` (324 lines).
- Notices for code that is in every binary:
  - MPR (677 lines) is a port of libccd's `mpr.c`, with no attribution anywhere in the repository.
  - The nightly wheel bundles 16 libraries, but `dist-info/licenses` holds only `LICENSE` and Filament's notices.
- A root `NOTICE` reaches the wheel through the default license globs, with no `setup.py` edit.

**Costs.**
- About 300-350 lines of license text, and a drift test of about 30 lines.
- The derived code gets no upstream fix stream, and no library can replace any of it: libode does not export its LCP solver, and urdfdom's `parseWorld` is a stub.
- The oracles keep 6,283 test-only lines.

**Why.**
- Owning this derived code is the choice with the fewest dependencies.
- The oracles are the only independent reference for the Dantzig solver and the convex hull while the PLAN-624 quiet window is open. `test_DantzigNative` compares native and ODE-baseline solutions to `1e-10` (`test_DantzigNative.cpp:187-195`), and `bm_dantzig_lcp` benchmarks the ODE arm. They do not back PLAN-624's bit-identical gates, which compare DART before and after a change. Keeping them is a test-only exception to DD-8's "bundled removed whenever possible", for that reason.
- M5a step 3 has to edit the caster fork anyway.

### 2.11 octomap

**Recommendation.** Keep octomap for all of 7.x:
- **DEP-4(a):** remove the dead shim; pin pixi to 1.10.1 or later; move the wheel to 1.10.1.
- **DEP-4(d):** forward-declare the octomap types, and remove the FCL include.
- **DEP-11 PR3:** suppress the warning only around the FCL includes.
- **DEP-32:** a ray-length check inside `VoxelGridShape.cpp`.

DEP-4(d) has one hunk inside the PLAN-624 quiet window. `NativeShapeConversion.cpp`, part of the native collision detector, uses the octree directly (`:51-53`, `:308-318`). Once `VoxelGridShape.hpp` forward-declares the types, that file needs its own `<octomap/OcTree.h>`, and DEP-11 PR3 then wraps that include. Both hunks go through the PLAN-624 owner or wait for M1d (`decisions.md:129-131`). The edits are include-only and bit-identical by construction.

A native voxel grid comes only if 8.0 makes FCL optional and a supported build ships without FCL.

**Gains.**
- Code that includes `VoxelGridShape.hpp` or `PointCloudShape.hpp` stops parsing about 361 FCL headers and 7,845 lines of octomap headers. Both headers are in `dynamics.hpp`. `dart.hpp` still reaches octomap through FCL until 8.0.
- 1.10.1 drops the `<ciso646>` warning.
- About 17 dead CMake lines go.
- A heap overflow closes:
  - octomap bounds its 100,000-key ray buffer only with `assert`;
  - `VoxelGridShape.cpp:161-186` inserts rays and point clouds unchecked;
  - for example, a 0.01 m grid and a ray from (−200,−200,−200) to (200,200,200) cross about 120,000 keys (arithmetic).

**Costs.**
- About 40-80 changed lines (estimate).
- Code that relied on the transitive includes breaks. This was approved under DD-7, and no gz-physics or gz-sim use was found.
- DEP-32 adds no installed API, because a helper in an installed header would be frozen at F. The dartpy and `PointCloudSensor.cpp` pre-checks become redundant and can go in a later cleanup.

**Why.** In 7.x a native grid saves 0 packages, because FCL requires octomap both on conda-forge (`fcl-config.cmake:40`) and on Ubuntu (`libfcl-dev` depends on `liboctomap-dev`). It would also add an estimated 600-1,000 owned lines, and results that are not bit-identical.

### 2.12 FCL, assimp, Bullet, ODE

**Recommendation.** Keep all four in 7.0 and do all four trims, with corrected specs. In 8.0, assimp stays DART's private, optional mesh importer; there are no in-house format readers.

- **DEP-3(a), Bullet link trim.**
  - The macOS wheel bundles `libBulletDynamics` and `libBulletSoftBody`, which DART never uses. They take 1,325,376 B unpacked and 521,171 B compressed, about 3.5% of the wheel (measured).
  - Linux and the static-Bullet wheels gain nothing, so the report's "2.0 MB" claim is withdrawn.
  - It narrows the exported `Bullet` link interface from four libraries to two, so it lands before F.
- **DEP-3(b), usage requirements.**
  - With DEP-14 committed, the core component's dependency packages drop from six to four, plus octomap. The six are assimp, Eigen3, fcl, Threads, Bullet and ODE (`dart/CMakeLists.txt:238-239`, plus the backends' `DART_CORE_DEPENDENCY_PACKAGES`); Bullet and ODE leave. This restores the release-6.20 contract that the unreleased #3672 dropped, plus Threads.
  - Spec fix: `collision-bullet` and `collision-ode` must carry the link in every build type, not only in static ones. gz-physics calls the raw ODE API and gets ODE only through `DART_LIBRARIES`. A static-only link would therefore fail on macOS and Windows while the Linux lanes stay green.
- **DEP-8(e), assimp allowlist.**
  - Collada, STL, OBJ, PLY and glTF 2.0/GLB. glTF feeds DART 7's PBR material design.
  - About 52 of the 70 assimp CVEs published since 2023-10 sit in code this compiles out (NVD, reclassified).
  - Estimated 60-70% smaller library, and 2.5-3.5 MB off the 19.8 MB compressed Linux wheel (symbol attribution, low-medium confidence; measure in the PR).
  - **Linux wheels only.** Linux wheels would reject FBX, 3DS, Blend, X, IFC and the other dropped formats. The macOS and Windows wheels bundle conda-forge's full assimp and still load them, and assimp reports only "No suitable reader found". DD-7's approved batch included an allowlist in Linux wheels, but this per-platform difference is listed as a labeled change (§4), with a documented format list. Applying the allowlist everywhere would mean DART building assimp for the macOS and Windows wheels too.
- **DEP-13(b)(c).**
  - About 125 dead `aiScene` lines go.
  - Bullet keeps a live `aiMesh` path, so `<assimp/scene.h>` moves from the installed header into the `.cpp`.
  - DART's own callers move off the deprecated APIs first.
- **Why keep.** gz-physics binds all four on every line:
  - the `fcl` detector key;
  - `new aiScene` in `CustomMeshShape.cc`;
  - raw ODE calls and `btCollisionWorld::rayTest` in `GzCollisionDetector.cc`;
  - the `OdeCollisionDetector` subclass.

## 3. Updated packet plan

Each packet keeps two reviews from engines other than its writer. Writers follow the team-mode routing: Codex for security, ABI and solver work, and Claude Sonnet for bounded build, CI and docs packets.

**[F]** marks a packet bound by API freeze F: it breaks something or changes a default, so it lands before F or waits for 8.0. Unmarked packets are additive and may slip to 7.1 if they miss F (`dart7_python_first/README.md:287-289, 324-329`).

### Existing packets

| Packet | Edit | Owner | Target |
|---|---|---|---|
| DEP-0 | Unchanged. Finish review round 2 and the `config-gz` re-check | Claude Sonnet | 7.0, first |
| DEP-1 | [F] Unchanged. Committed (`8e9b462b4`) with two reviews; the Gazebo lanes are owed. Land it before DEP-4(a) and DEP-19 PR2, which edit nearby `pixi.toml` and Dockerfile lines | Claude Sonnet | 7.0 |
| DEP-2 | **Drop.** Under Q2(a) it would guard deleted code. Revive it as DEP-19's first PR only under Q2(b) | — | — |
| DEP-3 | **(a)** [F] Restate the gain as the macOS-wheel figure. Replace the `readelf` gate with two checks: the wheel's dylib listing, and a two-library `Bullet` link interface. **(b)** [F] Re-spec as in §2.12: the core packages go from six to four, plus octomap. `python/dartpy/CMakeLists.txt:81`, `tools/perf` and `tools/friction_eval` link the components. New gate: the gz plugin still NEEDs libode. Drop the DEP-25 dependency, and drop (b) if it grows past one CMake PR | Claude Sonnet | 7.0; (b) after DEP-14 |
| DEP-4 | **(a)** Unchanged, after DEP-1. **(b)** Narrowed to one PR per dependency (`packets.md:65`). The ODE PR guards DART's `uninstall` target and fixes its script name: `CMakeLists.txt:726-737` configures `uninstall_target.cmake` but runs `cmake_uninstall.cmake`. It also sets a scoped `CMAKE_POLICY_VERSION_MINIMUM`. The Bullet PR does the same scoping and adds one scheduled lane that fetches both. No sub-build. **(c)** Unchanged. **(d)** [F] Add explicit octomap includes where code relied on the transitive one, and keep the link PUBLIC. Its `NativeShapeConversion.cpp` hunk goes through the PLAN-624 owner or waits for M1d. **(e)** Unchanged, landing before DEP-18, which adds sdformat to its strict switch | Claude Sonnet | 7.0 |
| DEP-5, DEP-7 | Unchanged | Codex high; Muse | 7.0 |
| DEP-6 | Add: fetch by URL and hash, using release assets where they exist. DEP-25 adds the fail-fast call at this fetch site; DEP-6 does not wait for it | Claude Sonnet | 7.0 |
| DEP-8 | **(d)** **Drop.** AlmaLinux 8's libpng 1.6.34-9, -10 and -11.el8_10 already carry the fixes. **(e)** [F] Allowlist Collada, STL, OBJ, PLY and glTF 2.0/GLB. Exporters off, but never `ASSIMP_NO_EXPORT`. Generate `.ply` and `.glb` fixtures, record the sizes, and add a labeled CHANGELOG entry plus a docs format list that names the per-platform difference. **(f)** **New.** Fetch the wheel image's sources as release archives checked with `sha256sum`, as the ODE block does (`Dockerfile.manylinux_2_28:168-169`), instead of the 17 unchecked `git clone`s (16 after DEP-14). No version changes. A shared mechanical step (§4) | Claude Sonnet | 7.0 |
| DEP-9 | [F] Unchanged | Claude Sonnet | 7.0 |
| DEP-10 | Add: fetch nanobind from its PyPI sdist by hash. The bump checklist re-diffs `dart_eigen_dense.h` against the stock `dense.h`. Record that `construction.cpp:31`, not the fork, pins system nanobind to 3.1.x | Claude Sonnet | 7.0 |
| DEP-11 | PR3 lands after DEP-4(d) and wraps only the FCL includes and the `.cpp` files that include octomap. Its `NativeShapeConversion.cpp` hunk goes through the PLAN-624 owner or waits for M1d. PRs 1-2 are committed; their Gazebo lanes are owed | Codex high + Claude Sonnet | 7.0 |
| DEP-12 | **(b)** [F] Raise the tinyxml2 floor to 10.0.0. libpng stays at its API minimum (approval needed, §4). Raise the fmt floor to 9.1.0 only under Q1(b). The urdfdom floor moves to DEP-17 PR1, and curl and OpenSSL are moot. The apt lane is added once the distro FCL assertion is triaged. It passes the per-dependency system switches (the umbrella, once DEP-25 lands) and `FETCHCONTENT_FULLY_DISCONNECTED=ON`, so it also serves as the `dis` gate | Orchestrator + Claude Sonnet | 7.0 |
| DEP-13 | **(a)** [F] Unchanged. **(b)** [F] Move `<assimp/scene.h>` from the installed `BulletCollisionDetector.hpp` into `BulletCollisionDetector.cpp`. **(c)** Move DART's own callers off the APIs before adding the deprecation attributes: `BulletCollisionDetector.cpp:703`, `FCLCollisionDetector.cpp:1245`, `FCLCollisionObject.cpp:155`, `MeshShape.cpp:1116, 1196`, plus tests | Codex high + Claude Sonnet | 7.0 |
| DEP-14 | [F] Confirm as committed (`8dbc91568`). Owed: the Gazebo lanes on the Wave 1 integration tree, and the migration note. The note covers the log format, the narrower `DART_*` argument set, `Threads` in the package config, fmt no longer being found through `find_package(DART)`, and the 5-line `setLogSink` recipe for spdlog users. Nothing else reopens the commit | Codex (writer); orchestrator | 7.0 |
| DEP-15 | **Narrowed** (Q1). **Under Q1(a)** [F]: the formatter without byte parity (§2.2); `XmlHelpers.cpp` off fmt, with `toString(bool)` and `toString(char)` tests; deletion of the `dart/utils` fmt hunk (`:64-76`), every fmt pin, the image step and `DARTFindfmt.cmake`; the two DEP-14 nits; the nm gate. Gates: base on Linux, macOS, Windows and FreeBSD; wheel; pkg; gz. **Under Q1(b), "Confine fmt":** the pragma, with the standard headers included before fmt; the same `XmlHelpers.cpp` change and hunk deletion; fmt fetched by URL and hash, as a labeled default change ([F]); the floor at 9.1.0; the nm gate on the apt lane (fmt 9.1), on fmt 12 and on macOS (`nm -gU`); and a check that `git grep '<fmt/' -- dart` lists only `Logging.cpp` | Codex high | 7.0, after DEP-14 |
| DEP-16 | [F] The dartpy logging bridge is a PLAN-624 must-have (README `:124-125`). Scope unchanged. Add: LogCapture subscribes through `setLogSink` and restores the previous sink. The Python bridge queues messages and drains them under the GIL, because the sink holds a recursive mutex. `Console.hpp` text reaches the sink as data, never as a format string. **Moved here from DEP-14:** on Windows, the default sink colors output only when the console has VT processing; it enables it, or prints plain text. **Optional:** drop `resetLogSink()`, since `setLogSink({})` already restores the default, so 7.x freezes one fewer API | Claude Sonnet + orchestrator | 7.0, after DEP-14 |
| DEP-17 | **PR1** [F]: floor 4.0.0; delete the TinyXML v1 code; keep the MSVC `/include` strip until a vcpkg 4.0+ port is checked; drop ros-urdfdom from the FreeBSD lane while IO is off. **PR2** (no decision): null-attribute fixes plus a malformed-file test; one copy of `collectJointXmlElementMetadata`; remove the includes of `BackwardCompatibility.hpp` at `DartLoader.cpp:52` and `urdf_world_parser.hpp:40`. **PR2b** (approval): the 7.0 deprecation message on `BackwardCompatibility.hpp`, saying there is no replacement, with `URDFTypes.hpp`'s message reworded to match. Today `urdf.hpp` reaches that header, so PR2b must follow PR2. **PR3** (approval): move the wheel image to urdfdom 6.0.1 with headers 3.0.x, and rebuild the image's sdformat on top; add the URDF acceptance table and "trusted input below 5.1"; propose the release-6.20 backport through `dart-backport-pr`. **PR4** [F] (approval): move `IncludeUrdf.hpp` and `urdf_world_parser.*` to a non-installed `detail/`; link urdfdom PRIVATE and export it only for static builds. PR4 waits for no packet except DEP-18, which edits the same lines. **Dropped:** deferring to urdfdom's world parser, the `%` guard, console_bridge gating, any fetch | Claude Sonnet; PR4 Codex high | 7.0 |
| DEP-18 | [F] **Re-scope** (Q3) to optional SDF inside `dart-io` through `dart_check_optional_package` (§2.4), after DEP-4(e). Drop the separate library, the `utils-urdf` rewiring, the dartpy link edge and the NEEDED gate. **Gates:** a shared `io` consumer configures without sdformat, and a static one requires it; `COMPONENTS io-sdf` fails cleanly on an SDF-off install; with the strict switch, a missing sdformat fails configure; one lane with IO on and every sdformat lookup off runs the URDF, SKEL and MJCF tests; on SDF-on builds, the exported `SdfParser` symbols match today's. It lands before or after DEP-17 PR4, whichever is approved first, and does not wait for DEP-19 | Claude Sonnet or Codex high | 7.0 |
| DEP-19 | **Rewrite** (Q2). **PR1** [F, as PR2's prerequisite]: the helper, with tests against a local TLS server for: https→http rejected; a 6th redirect rejected; the byte cap; a trickle abort; a bad CA and hostname; a corrupted cache; concurrent fetches. **PR2** [F]: delete the C++ feature; curl and OpenSSL go together as one feature's deletion (§4). `DART_BUILD_UTILS_ASSETS` is removed, not turned into a no-op, and only the CHANGELOG assets entry is replaced. **PR3**: `dart.data`, inside M5a step 2; [F] only if Q2 confirms the fetcher belongs to the must-have | PR1 Codex high, Claude reviews; PR2-3 Claude Sonnet | 7.0 |
| DEP-20 | **(a)** The spike uses apt `libglfw3-dev` 3.3.10, and also decides the GUI's skip-or-stop default. **(b)** **Rewrite:** floor 3.3.10 plus the run-time hint guard; no fetch, no switch. It is additive, lands with the spike and is not tied to F. **(c)** Make the prebuilt Filament SDK the default only on macOS arm64 and Windows x64, after the spike. Its PR wires `DART_USE_SYSTEM_FILAMENT` to the umbrella there; [F] as a default change if it lands in 7.0. **(d)** **Re-scope** to the existing decode budget (§2.9). **(e)** [F] Unchanged | (a) Claude; (b)(e) Claude Sonnet; (c) orchestrator; (d) Codex high, Claude reviews | 7.0; (c) as the spike decides |
| DEP-21 | Unchanged. Set the C++ standard per DART target, so fetched subprojects are not compiled as C++20 | Orchestrator + Codex ultra | 7.0 or 7.1, after M1d |
| DEP-22 | Add: fetch Tracy by URL and hash | Muse / Codex high | 7.x |
| DEP-23 | **Add documentation:** the switch table; the umbrella switch's first-configure semantics; the stable FetchContent names (`dart_imgui`, `nanobind`, `googletest`, `benchmark`, `tracy`, `dart_filament_sdk`, `ode`, `bullet`, and `fmt` only under Q1(b)); the offline recipe; the noble apt line (`libglfw3-dev`, plus `libx11-dev` and `libxrandr-dev` under `--no-install-recommends`); SDF per channel (strict for conda-forge, wheels and FreeBSD with sdformat 16; skipped for Debian and Homebrew core; a new vcpkg feature). For `gz_dartsim_vendor`, which clones DART at build time and declares `fmt` but no sdformat: pass the umbrella and expect SDF to be skipped, or add `sdformat_vendor`. **Add drafts, each needing approval:** the vcpkg `spdlog` feature and the conda spdlog host dependency become no-ops; the vcpkg and FreeBSD urdfdom port bumps and their header lists. **Drop:** the proposal that gz-physics request only `utils-urdf` | Orchestrator | 7.0 / 7.x |
| DEP-24 | **Add:** assimp stays a private, optional importer with no in-house readers; a real `dart-io-sdf` library only on evidence; remove `BackwardCompatibility.hpp`, its `.in` file and `URDFTypes.hpp`; a native voxel grid only with an FCL-free supported build, and only after a benchmark; retire the ODE and Bullet opt-in fetch. **Correct:** the native grid's 7.x placement | Orchestrator | 8.0 |

### New packets, and packets not created

| Packet | Scope | Owner | Target |
|---|---|---|---|
| DEP-25 (smaller) | The umbrella switch, CMP0170 and the fail-fast check (§2.6), with the check called at every existing fetch site. It wires only GoogleTest, Benchmark and Tracy, whose defaults are already OFF, so nothing changes while the umbrella is OFF. Filament (DEP-20(c)) and fmt (Q1(b) only) join in their own PRs. A second shared mechanical step that needs your OK and a README edit (§4). **Out of scope:** AUTO/STRING values, a provider record, a private sub-prefix and RUNPATH, a per-dependency summary, a compiler-only lane, GLFW/PNG/JPEG switches, the ImGui archive. **Gates:** defaults unchanged with the umbrella OFF; a disconnected configure with the umbrella; a Homebrew-style trap provider sees no FetchContent call; installed configs are byte-identical | Claude Sonnet | 7.0, after Q4 and DEP-12(a); additive |
| DEP-26 | **Not created.** Revisit find-or-fetch only if someone reports a need, and make the dependency private first | — | — |
| DEP-27a | A root `NOTICE` covering ODE's BSD option, libccd, FCL, convhull_3d, Willow Garage, nanobind, Drake, CMU/aikido, ImGui and tsl::robin_map, plus a pointer to Eigen's MPL-2.0 source. Install `LICENSE` and `NOTICE` next to Filament's notices. Add a test that maps every file with a foreign copyright to an entry. The wheel `METADATA` lists `License-File: NOTICE`. The `Mpr*` attribution is a separate, comment-only commit. **Gates:** base and one wheel build | Claude Sonnet | 7.0 now; the `Mpr` comment through the PLAN-624 owner or after M1d |
| DEP-27b | License texts for the 16 libraries bundled in the wheel, installed into the wheel metadata through CMake, with a test | Claude Sonnet | 7.0, with M5a step 2 |
| DEP-28 (smaller) | Delete the 7 `dart/utils/*_` files and `tests/.deprecated`. **Gates:** base, and an unchanged install manifest. Nothing built or installed changes, so the Gazebo gates add nothing | Orchestrator or Claude Sonnet | 7.0, now |
| DEP-29 | **Not created.** M5a step 3 owns the caster fork, and DEP-10 re-diffs it on each bump | — | — |
| DEP-30 | ImGui as a SHA256-pinned tag archive with `TLS_VERIFY ON` and `DOWNLOAD_EXTRACT_TIMESTAMP`, in the form Q5 picks. **Under Q5(a):** one PR, one dependency. It also deletes `dart-compat.patch`, its apply and stamp CMake (`dart/gui/imgui/CMakeLists.txt:115-142, 160-236`), `test_Issue2516`, `test_Issue2668` and the hidden-symbol half of `test_Issue2671`, with a labeled CHANGELOG entry. **Under Q5(b):** the patch's edits are applied without Git to copies in the build tree, and configure fails unless every marker is present. **Gates (both):** a configure with `GIT_EXECUTABLE=/nonexistent` and system GoogleTest succeeds; a pre-seeded, disconnected configure succeeds; a tree populated by today's clone reconfigures cleanly; `pixi run test-gui`. **Also under Q5(b):** the edited files are byte-identical to today's patch output, the three regression tests pass, and a read-only pre-seeded source works | Claude Sonnet; Codex and Muse review | 7.0, after Q5 and DEP-31; [F] under Q5(a) |
| DEP-31 | Reject an empty font file in `OverlayRenderer::prepareFonts` (`overlay_adapter.cpp:177-191`), with a `setFont` test. DART's own path then stops depending on the ImGui patch, which system-ImGui builds already lack. **Gates:** base, gui, test-py | Claude Sonnet | 7.0, now |
| DEP-32 (bug fix) | A ray-length check inside `VoxelGridShape.cpp`'s update paths, with no new installed API, plus a regression test with a ray of more than 100,000 cells. Valid inputs stay bit-identical. The dartpy and `PointCloudSensor.cpp` pre-checks become redundant. **Gates:** base, py, the voxel tests, gz. PLAN-624 runs the Gazebo gates for every core change (README `:188-189`) | Codex high writes; Claude reviews | 7.0, now |
| DEP-33 | Sample-data provenance. Record the source and license of every installed `data/` subtree. Move the 37 Atlas files to a pinned location DART owns; 34 sit on a personal fork and 3 on a commit that is on no main-line branch. The move needs approval. This gates `dart.data` in wheels | Orchestrator | 7.0; [F] for the data `dart.data` ships |

### Order

1. **Finish Wave 1:** DEP-0, DEP-1 [F], DEP-11 PRs 1-2 and DEP-12(a). DEP-14 [F] needs only its Gazebo lanes and its migration note.
2. **Now, with no decision needed.** F-bound packets first:
   - [F]:
     - DEP-3(a), plus DEP-3(b) and DEP-16 after DEP-14;
     - DEP-4(d), with its quiet-window hunk through the PLAN-624 owner;
     - DEP-8(e); DEP-9; DEP-12(b)'s tinyxml2 floor; DEP-13(a)(b); DEP-17 PR1; DEP-20(e).
   - Additive:
     - DEP-28, DEP-27a, DEP-31, DEP-32;
     - DEP-4(a)(b)(c) after DEP-1, and DEP-4(e);
     - DEP-5, DEP-6, DEP-7; DEP-8(a)(b)(c); DEP-10;
     - DEP-11 PR3 after DEP-4(d);
     - DEP-13(c); DEP-17 PR2; DEP-20(d);
     - DEP-20(b), with the DEP-20(a) spike.
3. **After your answers.**
   - [F]:
     - DEP-15 (Q1), after DEP-14;
     - DEP-19 PR1, then PR2 (Q2);
     - DEP-17 PR4 (approval) and DEP-18 (Q3), one after the other, in whichever order they are approved;
     - DEP-30 with the patch dropped (Q5(a)), after DEP-31.
   - Additive:
     - DEP-25 and DEP-8(f) (Q4 and the README edit);
     - DEP-17 PR2b and PR3 (approvals);
     - DEP-30 under Q5(b).
4. **With PLAN-624 M5a step 2:** DEP-19 PR3 ([F] if Q2 confirms the premise), DEP-27b and DEP-33.
5. **After M1d:** DEP-21, the `World.cpp` captures from DEP-11, the `Mpr*` comment, and any quiet-window hunk the PLAN-624 owner did not clear earlier.
6. **Later:**
   - 7.x: DEP-22, and DEP-20(c) if the spike defers it.
   - 8.0: DEP-24.
   - Only under Q1(b), once the GCC floor reaches 16: the fmt → `std::format` port.

### Project-home corrections, after your answers

- **`decisions.md`:**
  - record the answers;
  - rewrite DD-4's fmt line only under Q1(b);
  - record DD-8's per-dependency matrix and its exceptions as the answer to the re-opened DD-8 (Q4), keeping the original Q4 text as history;
  - replace the libpng default (`:121-122`) and the ImGui vendoring default (`:123`) with the answers;
  - record the residual urdfdom format-string risk and the release-6.20 backport decision.
- **`inventory.md`:**
  - urdfdom rows: kept, floor 4.0.0;
  - sdformat: an optional part of `dart-io`;
  - GLFW: 3.3.10, system-only;
  - ImGui: fetched and hashed, unpatched under Q5(a);
  - fmt: removed under Q1(a), or private and hidden under Q1(b);
  - libcurl and OpenSSL: removed with an unreleased feature;
  - row 130: spdlog 731 → 0; fmt 174 and 172 on `main`, 85 and 75 after DEP-14, and 0 after DEP-15;
  - row 131: five fewer packages (four under Q1(b));
  - add a section on in-tree third-party code.
- **`README.md`:**
  - the north star says source builds fetch what DART keeps private or pins exactly;
  - spdlog and fmt go, or fmt stays private until `std::format` (Q1(b));
  - Constraints names DEP-25 and DEP-8(f) as shared mechanical steps, and the DEP-19 PR2 one-feature exception, if approved;
  - the `dis` gate passes the umbrella;
  - the risk "Fetch-first defaults versus ODR" is marked resolved.
- **`packets.md`:** the DEP-4(b) split, DEP-15's new scope, and the re-scoped packets above.

## 4. Questions for the maintainer

**Q1. fmt (DD-4).** DD-4 records a DART-owned formatter built on the standard library (DEP-15). After DEP-14, fmt is a private, header-only detail of one file, and a 6-line pragma would hide its symbols.
- **(a) Recommended: DEP-15 as recorded, without byte parity with fmt.**
  - It supports `{}`, `{:.N[efg]}` and escaped braces.
  - It costs about 100-150 lines of formatter and tests, and deletes about 165 lines of fmt calls and glue. fmt leaves every channel.
  - The `DART_*` grammar narrows to those forms. This is a labeled change before F. Every in-tree use fits, and a code search found no outside use that doesn't.
  - conda and pixi macOS builds add `-D_LIBCPP_DISABLE_AVAILABILITY`.
- **(b) Keep fmt hidden in `Logging.cpp` until a GCC 16 floor allows `std::format` (DD-3).** This re-opens DD-4.
  - fmt stays in every channel, with about 130 lines of glue.
  - With fetch-first, every source build without the umbrella downloads fmt at configure; today system fmt is the default.
  - The hiding needs nm gates on fmt 9.1, fmt 12 and macOS.

**Q2. C++ asset fetching in the 7.0 API (DD-6).** Do you want C++ users to load the two humanoids through `model://` URIs, as part of the frozen 7.0 API? This is the CloudResourceRetriever idea from your #464 comment, which you shipped in #3653.
- **(a) Recommended: no.** Delete the unreleased `utils-assets` header, component, option and binding before F. A Python helper built only on the standard library fetches and verifies the files, and later becomes `dart.data`. A C++ resolver can still be added in 7.x.
- **(b) Yes.** Keep it as it is (default OFF, with system libcurl and OpenSSL) and add DEP-2.

Please also confirm one premise: should D22's "a fetcher" fetch the humanoid bundles for wheel users? The must-have acceptance test uses only in-repo data.
- If yes, (b) means DART maintains two fetchers, because `dart.data` needs a Python one for wheels.
- If no, (a) is still recommended, and DEP-19 PR3 becomes additive work that may slip to 7.1.

Not offered: a separate `dart-io-assets` library, or an in-house SHA-256. Both add owned surface for a feature with no caller.

**Q3. SDF packaging (DEP-18).** Should SDF become an optional part of `dart-io`, and what should a plain configure do when sdformat is missing? "Optional part" means:
- the optional-dependency mechanism Bullet and ODE use;
- an `io-sdf` forwarding component;
- sdformat exported only for static builds;
- the legacy `utils` edges unchanged.

Under any of these options, SDF-off builds ship legacy `utils` without `SdfParser`. DART 6's `utils` always had it, built on tinyxml2, so this is a legacy-component change you would be approving.
- **(a) Recommended.** Without sdformat, configure warns and skips SDF. DART CI, wheels, the gz lanes and packagers that ship sdformat set DEP-4(e)'s strict switch. Cost: for builders who set nothing, what the package contains depends on what is installed.
- **(b)** A missing sdformat stops configure, unless `DART_SKIP_SDFORMAT=ON` is set.
- **(c)** A separate `dart-io-sdf` library, the earlier plan. It adds:
  - a fourth SONAME through 7.x;
  - new link edges for dartpy and `tools/package`;
  - a `utils-urdf` rewiring that needs your approval.

  Its isolation helps only C++ processes that load just `dart-io`.

Keeping SDF required, as today, leaves Homebrew core and `gz_dartsim_vendor` unable to build DART 7's IO.

**Q4. "Fetch first" for the libraries Gazebo also loads (re-opens DD-8).** You answered the original Q4 with "fetch first for build from source". For the libraries Gazebo also loads, this plan proposes system-only, which goes further than the original Q4, which had fetch fallbacks. Under `decisions.md:3` this re-opens DD-8 for those libraries. The new evidence, detailed in §2.6:
- every builder except a bare `cmake` passes the umbrella;
- four upstreams' unguarded `uninstall` targets force sub-builds;
- DART's only heavy fallback has never run in CI;
- fallbacks would put second copies of Gazebo's libraries in DART's prefix;
- one apt line covers stock noble.

The options:
- **(a) Recommended.**
  - **Fetch first:** ImGui, nanobind, GoogleTest, Benchmark and Tracy. Also the prebuilt Filament SDK on macOS arm64 and Windows if the spike agrees, and fmt only under Q1(b).
  - **System only:** Eigen, libccd, octomap, FCL, assimp, tinyxml2, the urdfdom group and sdformat, plus GLFW at noble's 3.3.10, libpng and libjpeg-turbo.
  - **No-install route:** pixi needs no installs, and the docs give one apt line for noble.
  - **ODE and Bullet** stay system-first with warn-and-drop. A DART built without them lacks `collision-ode` and `collision-bullet`, which gz-physics requires, so Gazebo-facing packagers set DEP-4(e)'s strict switch.
- **(b)** As (a), plus a fetched static GLFW (X11 on Linux, Cocoa and Win32 elsewhere), so plain source builds need no glfw package. That costs:
  - 100-200 more lines;
  - more X11 `-dev` packages on Linux than `libglfw3-dev`;
  - a default that neither pixi nor the wheels would run.
- **(c)** As (a), plus find-or-fetch fallbacks in 7.x for the libraries shared with Gazebo. That costs:
  - an unprototyped estimate of 600-1,500 lines of CMake;
  - 5-7 more switches frozen through 7.x;
  - configure-time sub-builds for four upstreams with unguarded `uninstall` targets.

  DART already pins assimp, FCL, tinyxml2 and urdfdom for the Linux wheel, and fallbacks would extend that CVE duty to every source build that takes them. If you lean towards (c), a one-dependency prototype (tinyxml2) should bound the cost first.

**Q5. The Dear ImGui patch.** `dart-compat.patch` guards DART's re-exported ImGui against third-party misuse: null font data, calls without a context, and an exported ImFormatString. These came from #2516, #2668 and #2671, all from one reporter in 2026. DART's own code calls no patched function except through `setFont`, which DEP-31 fixes. System-ImGui builds and lanes already run without the patch.
- **(a) Recommended.** Drop it in 7.0 once DEP-31 lands, as a labeled change.
  - Gains: about 300 fewer owned lines (the 74-line patch, about 105 lines of apply and stamp CMake, three regression tests) and no Git. Fetched, pre-seeded and system ImGui behave the same.
  - Cost: third-party code that misuses DART's ImGui gets upstream's assert or crash. Outside projects do include DART's ImGui, but a search cannot show misuse.
- **(b)** Keep the guards, applied without Git to copies in the build tree. That is 60-80 lines of edit logic, and five anchors to re-check on each bump.

**Approvals the plan needs.** These are not preference calls; object to any of them if you disagree.
- **DEP-17 PR4.** Retire `IncludeUrdf.hpp` and `urdf_world_parser.hpp` (`dart::utils::urdf_parsing`) from the installed headers before F.
  - gz-physics compiles unchanged: `DartLoader.hpp` only forward-declares the urdf types, and gz-physics' test uses only `DartLoader`.
  - Code search found only packaging file lists and forks.
  - If you decline, urdfdom, urdfdom_headers and console_bridge stay public dependencies of `io`, `utils` and `utils-urdf` through 7.x.
- **DEP-12(b).** libpng keeps its 1.6.0 API-minimum floor instead of noble's 1.6.43, a DD-1 exception, so Linux wheels keep RHEL's patched copy. The alternative is for DART to build libpng 1.6.57 or later in the wheel image and own its CVE bumps.
- **Shared mechanical steps.** `README.md:104-105` allows one shared step today, the toolchain floors. Two packets each touch several dependencies but change no version, source tree or default:
  - DEP-25: the umbrella, CMP0170, and the fail-fast call at every fetch site;
  - DEP-8(f): hash-checked wheel-image sources.

  Approve them as shared steps, with a README edit.
- **DEP-19 PR2** removes curl and OpenSSL in one PR, as the deletion of the one feature they serve.
- **release-6.20.** A backport of DEP-17 PR3's wheel bump through `dart-backport-pr`. It also widens 6.20 wheels to accept URDF 1.1 and 1.2.
- **Labeled changes this plan adds or carries:**
  - **DEP-14 (committed):**
    - the log format;
    - `DART_*` arguments limited to integers, floating-point values, strings, pointers and enums (a source break);
    - `Threads::Threads` PUBLIC, with `DARTFindThreads.cmake`, `Threads` in the core component and thread flags in `dart.pc`. This is a platform library replacing the one spdlog supplied, which reads PLAN-624's "zero new core dependencies" accordingly;
    - fmt no longer found through `find_package(DART)`.
  - **DEP-15:** the narrower `DART_*` grammar under Q1(a), or fmt's default changing from system to fetched under Q1(b).
  - **DEP-18:** sdformat exported only for static builds, and `utils` without `SdfParser` on SDF-off builds (Q3).
  - **DEP-8(e):** Linux wheels load only Collada, STL, OBJ, PLY and glTF, while macOS and Windows wheels load every assimp format. DD-7's approved batch included a Linux-wheel allowlist; this names the per-platform difference it creates.
  - **DEP-17 PR3:** wheels accept URDF 1.1 and 1.2.
  - **DEP-17 PR2b:** a 7.0 deprecation message on `BackwardCompatibility.hpp`, after PR2 removes the internal includes. It says there is no replacement.
  - **DEP-19 PR2:** `DART_BUILD_UTILS_ASSETS` is removed instead of becoming a no-op, because it never shipped.
  - **Q5(a):** the ImGui patch is dropped.

## Critique points not adopted, or adopted differently

1. **"`std::to_chars`'s macOS availability does not apply."** Not adopted. The AppleClang floor does not set the deployment target.
   - libc++ marks floating-point `to_chars` as introduced in macOS 13.3: `_LIBCPP_AVAILABILITY_TO_CHARS_FLOATING_POINT` maps to the LLVM 15 marker, `introduced = 13.3`.
   - conda-forge builds for macOS 11.0 (conda-forge news, 2026-02-06).

   Wheels (15.0) and Homebrew are unaffected. conda and pixi macOS builds need the `-D_LIBCPP_DISABLE_AVAILABILITY` define that conda-forge's knowledge base documents. This is kept as a small cost of Q1(a).
2. **"Remove 'mandatory symbol hiding' and 'fmt for good' from §2.1's comparison."** Adopted in part. Neither appears as a standalone cost any more. Two narrower points stay, because they still distinguish keeping spdlog:
   - hiding spdlog would cut users' `spdlog::set_level` off from DART, which cancels that option's advantage;
   - under Q1(a), keeping spdlog would also keep fmt.

   §2.1 states that under Q1(b) the second point does not distinguish the options.
3. **"The boundary alone fixes two of the three incidents."** Adopted with a correction. dart#2538 also broke DART's own build: spdlog 1.17 with fmt 12.1 rejected runtime format strings in `Logging-impl.hpp`. With spdlog behind the boundary, `Logging.cpp` would hit the same break, so the boundary fixes only the consumer half.
4. **"Put the Windows VT check in the separate fmt PR."** Adopted differently. The check changes the default sink, not fmt, so it goes to DEP-16, which already reworks sink output. DEP-14 is not reopened either way.
5. **"DEP-17 PR3 is a breaking change bound to F."** Not adopted. Moving wheels to urdfdom 6.0.1 widens what loads (URDF 1.1 and 1.2), so it is treated as an additive labeled change that may slip to 7.1. If PR3's acceptance table shows urdfdom 6 rejecting a file that 4.0.1 accepted, it becomes F-bound.
6. **"Write the edited ImGui files into the binary directory."** Adopted for the Q5(b) fallback only; under Q5(a) no edits remain.
7. **"A tri-state `DART_BUILD_IO_SDF` adds a third mechanism."** Adopted differently: DEP-18 reuses the existing optional-dependency macro instead of aligning a new switch with DEP-4(e). This also removes a STRING switch of the kind §2.6 rejects for acquisition.
8. **Evidence not gathered here, because this revision ran no builds:**
   - a prototype of one find-or-fetch fallback. It is proposed before any choice of Q4(c).
   - the nm gate on the real `libdart` with fmt 9.1, fmt 12 and macOS. It is a DEP-15 gate under Q1(b), and moot under Q1(a).