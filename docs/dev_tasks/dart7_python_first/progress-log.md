# Progress log

- **2026-10-10**
  - Read-only research and the decision interview: D0–D29.
- **2026-10-11**
  - The decision interview completed: D30–D32 and D31a.
  - Three independent plan reviews ran, and their verified findings became
    design contracts.
  - The plan was approved.
  - The Filament backend effort agreed to own M6a, porting onto a fresh branch
    from main with interim GUI bindings.
  - M0 started: this project home, the design doc, and two bug-fix packets
    (XML value parsing; GIL acquisition during finalization).
  - Both fixes were committed locally. The GIL fix was verified against a
    Python 3.15 build of dartpy.
  - The rollover's policy and validator halves were combined on one branch,
    based on main `1b5722176`.
  - The project home gained the approved contracts after a plan-conformance
    review.
  - The fixes were revised after review. The GIL fix gained a Python 3.15
    regression test that crashes without it; the XML fix gained a log-level
    build fix and an MJCF regression test.
  - The rollover was stacked on the project home and narrowed after review.
  - The milestone "DART 6.21.0" was retitled "DART 7.0.0", after archiving the
    earlier milestone of that name.
  - #3697 (program home and release-line rollover), #3698 (XML value
    parsing) and #3699 (GIL finalization) merged; the 6.20 backport of #3698
    opened as #3717 on the 6.20.1 patch line.
  - WP-7.0.3b and WP-7.0.5 started.
  - #3717 (the 6.20 backport of #3698), #3719 (SDF pose attributes) and #3720
    (many-World and dartpy loop rows) merged. Automated reviews raised four
    findings on #3719 that it did not address.
  - #3723 (ODE collision-callback exceptions) merged and was backported in
    #3727; #3722 (analytical-IK clone crashes) opened.
  - #3725 (Gazebo compatibility gates and version rules) merged; #3729 (gz API
    inventory) and #3728 (WP-7.0.7) opened.
  - On the 6.20.x line, #3704 added wheels for Python 3.12 to 3.14 and #3730
    Linux wheels for Python 3.15.
- **2026-10-12**
  - #3731 (C++14 contact-surface header tree, a follow-up to #3725) opened as
    a draft.
  - The maintainer settled the API freeze: `F` waits for every must-have and
    C2, and only additive work that is not a must-have and misses `F` moves to
    7.1.
  - The maintainer amended D13: FCL stays the C++ default in 7.0, and dartpy's
    `dart.World` and the MJCF world loader default to the DART detector.
  - The maintainer set M0's exit and the steps after it, the M1a PR shape, the
    `dart7-base` ref and the step-path Gazebo gates.
- **2026-10-13**
  - Reconciled the detached `dart7-base` checkout with fetched main:
    M1a (#3775) and demo consolidation (#3780) are merged; scope/acceptance
    (#3728) and the gz API inventory (#3729) remain on their existing branches.
  - Confirmed the completed baseline's 78 passing rows and recorded its
    pinned source, host and record digest in the verification owner.
  - Reproduced and fixed four remaining SDF pose gaps on
    `fix/sdf-pose-followup`, including strict admission boundaries for modern
    schema versions. Added a self-contained fixture and native viewer example.
  - Final core aggregate passes: 195 CTest and 1,596 Python cases. Matched
    native Filament captures agree with independent numeric model oracles;
    two independent source reviews are clean. GUI and the patched forward
    Gazebo gate, lint, check-lint and public-path checks also pass. The offline
    PR body and selected visual attachment are prepared for approval.
  - Kept the full program active: missing implementation, integration and
    release acceptance still prevent project-home retirement.
  - An independent M1b preparation spike confirms that editing unrelated
    World B incorrectly invalidates prepared World A. The shared-Skeleton
    protection case passes. Production implementation waits for the recorded
    sequencing to be reconciled; the evidence is retained in the task owner.
  - The SDF follow-up merged in #3782 as `79a87e6ed`, completing WP-7.0.3b.
    Scope/acceptance #3728 integrates that evidence with the reviewed freeze
    contract and records M1a and the baseline as already completed.
