# Newton / NCP Methods

This folder is reserved for Newton-style LCP/NCP solvers (e.g., minimum map, Fischer-Burmeister, penalized FB).

Guidelines:

- Derive from `LCPSolver` and reuse `LCPOptions`/`LCPResult`.
- Keep solver-specific configuration in an options struct referenced via `LCPOptions::customOptions`.
- Add comprehensive unit tests in `tests/unit/math/lcp`.
