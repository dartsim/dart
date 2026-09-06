## Summary

-

<!-- TL;DR: concrete problem or missing capability + principal change.
Use one or two short sentences, as prose or bullets (roughly 30–50 words total).
Keep secondary changes below; a short but generic opening is not enough.
PR-writing guidance: docs/onboarding/contributing.md#submitting-a-pull-request.
Keep material risks/migration beside the affected claim. Add rationale, Key Changes,
or a comparison only when it helps review. -->

## Testing

- Relevant check or evidence and result.

<!-- Keep failed, skipped, pending, or unavailable validation visible when it limits
a claim or blocks required readiness/merge gates, even if unrelated to this diff.
Full gate inventories and process/audit logs belong in task/session evidence, not this body.
For 3D structure or behavior changes, include Visual verification with the claim
it explains (before Testing when central), using dart-verify-sim and
docs/onboarding/agent-sim-verification.md. Keep required media and evidence visible.
Include relevant issues, backports, or follow-ups. -->

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
- [ ] Architecture map views (`docs/assets/architecture/`) updated when
      `dart/simulation/**` structure, step-stage slots, or solver families changed

</details>
