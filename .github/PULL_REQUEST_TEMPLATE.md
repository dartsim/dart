## Summary

-

<!-- 1–3 short outcome bullets, normally under 50 words total, most important first.
PR-writing guidance: docs/onboarding/contributing.md#submitting-a-pull-request.
Put material risks or Breaking Changes immediately after Summary. Add context,
Key Changes, or Before / After only when useful; omit empty sections. -->

## Testing

- Commands or manual checks and their results; name pending or skipped checks.

<!-- Keep failures and limitations visible. Link or collapse lengthy supporting logs.
For 3D structure or behavior changes, add Visual verification here using
dart-verify-sim and docs/onboarding/agent-sim-verification.md. Keep assessed
media/comparisons and their supporting evidence visible; do not shorten away
required evidence. Include related issues, backports, or follow-ups when relevant. -->

---

<details>
<summary>Checklist</summary>

<!-- Mark non-applicable items N/A with a short reason. -->

- [ ] Milestone set (DART 7.0 for `main`, branch-matching DART 6.x release
      milestone for the active DART 6 LTS branch)
- [ ] CHANGELOG.md updated per `docs/onboarding/changelog.md` if required
- [ ] Add unit tests for new functionality
- [ ] Document new methods and classes
- [ ] Add Python bindings (dartpy) if applicable

</details>
