# C++20 Modernization Follow-ups Progress (01)

## Status

- Current phase: Phase 2 (container membership cleanup)
- Code changes: Phase 2 in progress.

## Phase checklist

- Phase 0 - Discovery and scoping: Complete
- Phase 1 - Internal non-owning views: Complete
- Phase 2 - Container membership cleanup: In progress
- Phase 3 - Algorithm cleanups: Not started
- Phase 4 - Validation and wrap-up: Not started

## Notes

- This task continues C++20 modernization after the initial cpp20 effort.
- Phase 1: adopt `std::span` for `.cpp`-local helpers in
  `dart/math/optimization/Problem.cpp` and lambda indices in
  `dart/math/lcp/other/StaggeringSolver.cpp`.
- Phase 2: start replacing associative membership checks with `contains` in
  `dart/gui/Viewer.cpp` and `dart/dynamics/Linkage.cpp`.
