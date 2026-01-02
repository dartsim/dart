# C++20 Modernization Follow-ups Progress (01)

## Status

- Current phase: Phase 8 (container membership cleanup)
- Code changes: Phase 8 in progress.

## Phase checklist

- Phase 6 - Discovery and scoping: Complete
- Phase 7 - Internal non-owning views: Complete
- Phase 8 - Container membership cleanup: In progress
- Phase 9 - Algorithm cleanups: Not started
- Phase 10 - Validation and wrap-up: Not started

## Notes

- This task continues C++20 modernization after the initial cpp20 effort.
- Phase 7: adopt `std::span` for `.cpp`-local helpers in
  `dart/math/optimization/Problem.cpp` and lambda indices in
  `dart/math/lcp/other/StaggeringSolver.cpp`.
- Phase 8: start replacing associative membership checks with `contains` in
  `dart/gui/Viewer.cpp`, `dart/dynamics/Linkage.cpp`,
  `dart/utils/SkelParser.cpp`, and `dart/utils/urdf/urdf_world_parser.cpp`.
