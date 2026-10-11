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
