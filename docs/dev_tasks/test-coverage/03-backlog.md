# Test Coverage — Backlog (Future Work)

Items deferred from the main roadmap. Review and prioritize when triggers are met.

---

## Backlog 1: GUI Headless Testing

**Description**: Investigate headless rendering (Mesa/Xvfb) to enable `dart/gui/` coverage in CI.

**Current state**: `dart/gui/` is excluded from coverage metrics.

**Trigger**: CI infrastructure investigation complete, headless rendering proven feasible.

**Potential impact**: +5-10% overall coverage if feasible.

**Tasks when activated**:

1. Investigate Mesa/OSMesa for software rendering
2. Set up Xvfb in CI workflow
3. Create smoke tests for `Viewer`, `WorldNode`, `ImGuiViewer`
4. Add basic rendering verification tests

**Risks**:

- Headless rendering may not work with all OSG features
- CI time increase
- Platform-specific issues

---

## Backlog 2: Python Full API Parity

**Description**: Expand Python tests to cover full API surface matching C++ coverage.

**Current state**: Focus on most-used APIs only.

**Trigger**: Phase 3 complete, Python infrastructure mature.

**Tasks when activated**:

1. Audit Python bindings for untested methods
2. Auto-generate test stubs from nanobind signatures
3. Systematically test all bound classes/methods
4. Add edge case and error handling tests

**Scope estimate**: 40-60 additional Python test cases.

---

## Backlog 3: Experimental Module Testing

**Description**: Add test coverage for `dart/simulation/experimental/` once APIs stabilize.

**Current state**: Excluded from coverage metrics (heavy development).

**Trigger**: Experimental APIs promoted to stable (or marked for DART 7.0 release).

**Tasks when activated**:

1. Review API surface in `dart/simulation/experimental/`
2. Create unit tests for core components
3. Add integration tests for typical workflows
4. Update coverage targets to include experimental

**Notes**: This is the future API direction - should have excellent coverage (90%+) before promotion to stable.

---

## Backlog 4: Windows-Specific Test Coverage

**Description**: Ensure `#ifdef _WIN32` code paths are tested.

**Current state**: Primary CI runs on Linux; Windows paths may be untested.

**Trigger**: Coverage analysis shows significant untested Windows branches.

**Tasks when activated**:

1. Audit platform-specific code (`Uri.cpp`, `Lcp.cpp`, etc.)
2. Add conditional tests or mocks for Windows paths
3. Consider Windows CI coverage job

---

## Review Schedule

- Review backlog items at the start of each phase
- Promote items when triggers are met
- Archive completed items (move to "Completed" section below)

## Completed Backlog Items

(None yet)
