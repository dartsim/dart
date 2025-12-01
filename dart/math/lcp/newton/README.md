# Newton / NCP Methods

This folder is reserved for Newton-style LCP/NCP solvers (e.g., minimum map, Fischer-Burmeister, penalized FB).

Guidelines:

- Derive from `LcpSolver` and reuse `LcpOptions`/`LcpResult`.
- Keep solver-specific configuration in an options struct referenced via `LcpOptions::customOptions`.
- Add comprehensive unit tests in `tests/unit/math/lcp`.
