# Figures 1-2 incline current-source DART evidence

Status: valid current-source DART incline evidence; not paper parity.

Two fresh 121-row exact-FBF traces independently validate the reconstructed
two-second threshold outcomes. At `mu=.4`, the downhill displacement is
1.7686892884927794 m versus the analytical
1.7548661487418349 m target with a
0.20000000000000001 m fixture tolerance. At `mu=.5`, the
downhill displacement is 0.00089054129659805234 m with a
0.02 m stick tolerance. The slide-minus-stick
separation is 1.7677987471961814 m and exceeds the frozen
0.5 m threshold. Both traces retain
three DART contacts at every post-initial step, complete without boxed-LCP
fallback, and keep every solved-step residual at or below
1.0e-06. Full-trajectory gates also bound cross-slope motion,
incline-normal center-offset drift, and the exported `up_z` alignment; the
exported velocity components must match step-to-step position differences,
cross-slope and incline-normal speeds remain bounded, the stick cell stays in
both its displacement and speed bands, and the slide finishes with positive
downhill speed without regressing beyond its frozen numerical tolerance.

The combined actual-simulator capture completes 120 steps with
61 distinct world-view frames, zero accepted
caps, zero exact failures, zero boxed-LCP fallbacks, and worst residual
9.9998369622613588e-07. Its MP4 is 660x
506 at 30/1 with 61 fully
decoded frames. Recorded manual inspection binds the five-time panel, durable
source-frame copies, and clip to the visible low-friction slide versus
near-threshold stick.
The combined timeline records
8 contacts per
post-initial step, whereas the two independent traces record six in aggregate
(three per trace). The capture does not export a per-cell contact split.

The traces and combined capture have a byte-identical
121-row additive count projection over
step, exact_solves, boxed_lcp_fallbacks. Contact counts are explicitly excluded from
that projection because the measured eight-versus-six counts differ. This is
explicitly not state or full-trace equivalence: the combined renderer offsets
the two cells for presentation, while each tracked CSV is an independent
single-cell run. Residual, status, warm-start, state, and per-cell capture
equivalence are not compared.

Claim boundary: this is current-source DART reconstruction evidence. The DART
tracked traces report three contacts per cell, the combined capture reports
eight contacts in aggregate without a per-cell split, and the paper timing row
reports four. The raw runner schedule
retains a legacy three-contacts-per-cell mismatch note; the measured aggregate
timeline and traces reported above supersede that note for this bundle. The
tracked CSV does not directly export maximum penetration. No full friction
sweep, source-matched analytical plot, faithful external-solver media, approved
source golden/diff, paper timing, or real-time verdict is supplied. The
validated runner timeline remains internal bundle evidence and is not promoted
as a standalone manifest capture-sidecar deliverable.
