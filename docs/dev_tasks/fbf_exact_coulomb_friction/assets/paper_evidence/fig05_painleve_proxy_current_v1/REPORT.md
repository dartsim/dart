# Figure 5 Painleve proxy current-source evidence

Status: valid current-source DART proxy evidence; not paper parity.

The paired 2.5-second exact-FBF captures and fresh 151-row traces independently
support the qualitative proxy transition. At `mu=.50`, the box remains upright
and is translationally and tilt-settled over the final 0.5-second trace window
at x=1.2980816997249069 m. At `mu=.55`, the first
fixture-defined tumble threshold occurs at step 36
(t=0.6000000000000002 s,
x=1.2597198197697048 m),
0.038361879955202127 m before the `mu=.50` rest distance. The `mu=.55`
box is visually horizontal and passes the same translational/tilt-settled tail
gates; its later final x position is not the definition of the shorter-travel
claim. Angular velocity is not exported, so strict rigid-body rest is not
established.

Both traces complete 150 steps without boxed-LCP fallback and keep
every solved-step residual at or below 1.0e-06. Both capture
streams contain 76 distinct frames and decode fully. The synchronized group
clip is 1320x530 at 30/1 with
76 frames.

The panels use a deliberately dark palette. Their generic contrast heuristic
is false and contrast is not a required automated gate; nonblank, motion,
hash, composition, stream, full-decode, physical-trace, and recorded manual
inspection gates are the evidence used here.

Claim boundary: this is DART-side proxy evidence. The rendered demo and tracked
trace are separate scene implementations and are not trace-equivalent; the
trace corroborates the classifier but does not automate the visual semantic
verdict. The paper does not publish the box dimensions, mass, launch state, or
absolute timestamps. No faithful external-solver panels, approved source
golden/diff, paper timing, or real-time performance verdict is supplied.
