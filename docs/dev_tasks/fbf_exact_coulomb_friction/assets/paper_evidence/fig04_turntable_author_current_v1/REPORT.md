# Figure 4 author-source-pinned DART turntable evidence

Status: valid DART port of the public author turntable configuration at
`b3f3c5ca646b39a1bc4fbd8c3ebfb6810fee4bd0`; not Warp/Newton trace parity or historical paper-render parity.

The capture contains the source-ordered 2x2 matrix: top row mu=.2 at omega=2
and 5 rad/s, then bottom row mu=.5 at omega=2 and 5 rad/s. The synchronized
group clip is 1320x1060 at 30/1 with
181 decoded frames. Manual inspection records the three
ejections and the mu=.5, omega=2 cell retained on the support through 6 s.
Retention is not a zero-slip or co-rotation claim.

Four fresh current_visual traces use exact FBF with the dart_best contract and
the Native FourPointPlanar collision frontend. They complete 360 steps with no boxed-LCP
fallbacks, all solved residuals at or below 1.0e-06, and the
expected ejected/ejected/retained-through-6s/ejected physical outcome order.
Their four complete
core projections (contacts, exact solves, fallbacks, and status) equal the
corresponding capture sidecars. Full six-field projection byte identity holds
for 4 of four cells. The
retained warm-start-only divergence is: none. This is not
full-state trace equivalence.

Four additional paper_cpu/Native traces are retained in a separate directory,
invocation list, and trace-summary lane. Their strict-invalid scenarios are:
turntable_author_mu_0_5_omega_2. They are not used to validate the rendered trajectory,
cannot replace a current_visual trace, and do not make this reconstruction
paper-comparable. The current visual lane is solver-valid=true;
the separate strict lane is solver-valid=false.

Claim boundary: geometry, mass/density, friction cells, drop height, settle,
smoothstep ramp, timestep, and six-second horizon are pinned to the public author
source. DART represents the author's 5 mm collision gap as initial geometric
separation; backend margin semantics are not equivalent. The rendered demo and
CSV exporter are separate DART implementations. No Warp/Newton state-trace
equivalence, historical camera/material/lighting golden, synchronized external-
solver row, historical Apple timing, or real-time verdict is supplied.
