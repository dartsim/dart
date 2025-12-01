# Other / Specialized LCP Methods

Use this folder for methods that do not fit pivoting, projection, or Newton categories (e.g., interior point, staggering, shock propagation wrappers).

Tips:

- Keep implementations self-contained and derive from `LcpSolver`.
- Document assumptions (matrix properties, bounds) in the header.
- Add regression tests in `tests/unit/math/lcp`.
