# Agent Guidelines for Background Documentation

Guidelines for AI agents working with `docs/background/` theoretical documentation.

## Purpose of This Directory

`docs/background/` contains **theoretical background** derived from academic PDFs written by
DART's original authors. This is reference material explaining the mathematics behind the code.

| Directory   | Content                                         | Original Source     |
| ----------- | ----------------------------------------------- | ------------------- |
| `dynamics/` | Lagrangian mechanics, articulated body dynamics | `docs/dynamics.pdf` |
| `lcp/`      | Linear Complementarity Problem solvers          | `docs/lcp.pdf`      |

## Key Rules

### 1. Attribution is Non-Negotiable

Every page derived from the original PDFs **MUST** include the attribution header:

```markdown
> **Attribution**: This content is derived from "[Title]" by [Authors].
> The original PDF is preserved at `docs/[file].pdf`.
```

**Never remove or obscure original authorship.**

### 2. Original PDFs are Authoritative

When in doubt about mathematical correctness, **defer to the original PDF**.
The markdown versions are for maintainability; the PDFs are the authoritative source.

```bash
# To view original PDFs:
# docs/dynamics.pdf - Multibody dynamics tutorial
# docs/lcp.pdf - LCP contact handling
```

### 3. Notation Consistency

Use the notation glossary to map between PDF symbols and DART code:

| PDF Notation | DART API          | Notes                  |
| ------------ | ----------------- | ---------------------- |
| $q$          | `getPositions()`  | Generalized positions  |
| $\dot{q}$    | `getVelocities()` | Generalized velocities |
| $M(q)$       | `getMassMatrix()` | Mass matrix            |
| $\tau$       | `getForces()`     | Generalized forces     |

See [`dynamics/notation-glossary.md`](dynamics/notation-glossary.md) for the complete mapping.

### 4. LaTeX Math for Equations

These docs use **LaTeX math syntax** for equations (GitHub renders this natively):

```markdown
✅ Correct: Inline math uses single dollars: $M(q)\ddot{q} + C(q,\dot{q}) = \tau$
✅ Correct: Display equations use double dollars:
$$M(q)\ddot{q} + C(q,\dot{q}) + g(q) = \tau$$

❌ Avoid: Unicode math like `M(q)q̈ + C(q,q̇) = τ` (harder to read/search)
❌ Avoid: Code blocks for equations (no rendering)
```

GitHub renders LaTeX math natively since 2022. This provides better readability for complex equations.

### 5. Cross-Reference to Code

When explaining concepts, link to the implementing code:

```markdown
The mass matrix is computed by [`Skeleton::getMassMatrix()`][1].

[1]: ../../dart/dynamics/Skeleton.hpp
```

Or use inline references:

```markdown
See `dart/dynamics/Skeleton.hpp` for the implementation.
```

## When to Edit These Docs

### DO Edit When:

- Fixing typos or formatting errors
- Adding cross-references to DART code
- Updating notation glossary for new APIs
- Adding navigation links
- Clarifying content (mark additions clearly)

### DO NOT Edit When:

- The math seems wrong → Check the original PDF first
- You want to "improve" explanations → Add commentary separately
- API names changed → Update the notation glossary, not the derivations

## File Organization

```
docs/background/
├── README.md           # Index and attribution summary
├── ATTRIBUTION.md      # Attribution template
├── AGENTS.md           # This file
├── dynamics/           # From dynamics.pdf
│   ├── 01_introduction.md
│   ├── 02_lagrangian-dynamics.md
│   ├── ...
│   └── notation-glossary.md
└── lcp/                # From lcp.pdf (already migrated)
    ├── 01_problem-statement.md
    ├── 02_overview.md
    └── ...
```

## Migration Status

| Source              | Target                      | Status                   |
| ------------------- | --------------------------- | ------------------------ |
| `docs/lcp.pdf`      | `docs/background/lcp/`      | ✅ Complete (LaTeX math) |
| `docs/dynamics.pdf` | `docs/background/dynamics/` | ✅ Complete (LaTeX math) |

## Related Documentation

- [`docs/onboarding/dynamics.md`](../onboarding/dynamics.md) — Code-level dynamics exploration
- [`docs/onboarding/constraints.md`](../onboarding/constraints.md) — Constraint system internals

## Quick Reference: Adding New Content

1. **New page from PDF**: Copy structure, add attribution header, use LaTeX math syntax
2. **New code reference**: Add to notation glossary, link to source file
3. **Correction**: Note "Correction from original: [reason]" in a comment
4. **Addition**: Mark clearly as "[Added for DART context]"
