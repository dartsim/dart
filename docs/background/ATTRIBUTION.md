# Attribution Template

Use this template when adding attribution headers to migrated documentation.

## Standard Header (for each migrated page)

```markdown
> **Attribution**: This content is derived from "[Document Title]" by [Authors]
> ([Affiliation]). The original PDF is preserved at `docs/[filename].pdf`.
>
> **Navigation**: [← Previous](previous.md) | [Index](../README.md) | [Next →](next.md)

---
```

## Document-Specific Attribution

### For Dynamics Pages

```markdown
> **Attribution**: This content is derived from "A Quick Tutorial on Multibody
> Dynamics" by C. Karen Liu and Sumit Jain (Georgia Institute of Technology).
> The original PDF is preserved at `docs/dynamics.pdf`.
```

### For LCP Pages

```markdown
> **Attribution**: This content is derived from "Contact Handling for Articulated
> Rigid Bodies Using LCP" by Jie Tan, Kristin Siu, and C. Karen Liu.
> The original PDF is preserved at `docs/lcp.pdf`.
```

## Guidelines

### What to Preserve

- ✅ Original mathematical content and derivations
- ✅ Author names and affiliations
- ✅ Document structure (sections, ordering)
- ✅ Original notation (with mapping to DART APIs)

### What to Add

- ✅ Navigation links between pages
- ✅ Cross-references to DART code (`dart/dynamics/`, `dart/math/lcp/`)
- ✅ Notation glossary entries mapping PDF symbols → DART APIs
- ✅ Code examples showing DART usage

### What NOT to Do

- ❌ Remove or obscure original authorship
- ❌ Claim migrated content as new original work
- ❌ Delete original PDF files
- ❌ Substantially rewrite original explanations (add commentary separately if needed)

## Provenance Tracking

Each background document should be traceable to its source:

| Markdown File                        | Source PDF     | Section     |
| ------------------------------------ | -------------- | ----------- |
| `dynamics/01_introduction.md`        | `dynamics.pdf` | Chapter 1   |
| `dynamics/02_lagrangian-dynamics.md` | `dynamics.pdf` | Chapter 2   |
| `lcp/01_problem-statement.md`        | `lcp.pdf`      | Section 1-2 |
| ...                                  | ...            | ...         |

When making changes:

- **Corrections**: Note "Correction from original" in a comment
- **Additions**: Clearly mark as "[Added for DART context]" or similar
- **Clarifications**: Use blockquotes or callouts to distinguish from original
