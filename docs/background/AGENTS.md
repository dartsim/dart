# docs/background/

Agent rules for the theory pages derived from DART's original academic PDFs.
`README.md` owns the index and attribution table.

## Rules

1. **Attribution is required.** Every page derived from a PDF keeps the
   attribution header from [`ATTRIBUTION.md`](ATTRIBUTION.md) naming the title,
   authors, and preserved PDF path. Never remove or obscure authorship.
2. **The PDFs are authoritative.** When the math looks wrong, check the original
   PDF first. Record a verified correction as a clearly marked note
   ("Correction from original: ...") rather than silently rewriting the
   derivation; mark DART-specific additions as "[Added for DART context]".
3. **Use LaTeX math** (`$...$` inline, `$$...$$` display), which GitHub renders
   natively. Do not use Unicode math or code blocks for equations.
4. **Map notation to code through the glossary.** When an API name changes,
   update [`dynamics/notation-glossary.md`](dynamics/notation-glossary.md), not
   the derivations. Link concepts to implementing headers with repo-relative
   links.
5. **Keep theory here and decisions elsewhere.** DART design decisions go to
   `docs/design/`; code walkthroughs go to `docs/onboarding/`.

Add a new topic as a peer section in `README.md` pointing to its own
subdirectory or page.
