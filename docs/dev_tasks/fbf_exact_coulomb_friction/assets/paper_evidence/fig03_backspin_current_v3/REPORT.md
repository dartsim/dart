# Figure 3 backspin current-source DART evidence

Status: valid current-source DART backspin evidence; not paper parity.

The fresh 131-row exact-FBF trace advances to x=1.5959314363310166 m at
step 48, first records negative translational velocity at step
49 (t=0.81666666666666754 s), and reaches
x=-2.9362508912363654 m with vx=-6.6281589716239093 m/s at step 130.
This establishes translational advance and reversal for the reconstructed DART
fixture. It does not establish rest.

The capture completes 130 steps with
131 distinct world-view frames, zero accepted
caps, zero exact failures, zero boxed-LCP fallbacks, and worst residual
9.9649715497483895e-07. The MP4 is 1300x506
at 30/1 with 66 decoded frames; the GIF is
960x374 at 15/1 with
33 decoded frames.

The high-contrast 6x4 ivory/charcoal checker texture and coral registration tile
are renderer-applied through a UV MeshShape attached as a VisualAspect-only
shape node. The camera is nearly perpendicular to the Y spin axis, so the
checker cells remain large and quadrilateral in the captured viewport. They
make orientation changes legible without adding collision or dynamics aspects.
The physical SphereShape retains only CollisionAspect and DynamicsAspect.
The trace and capture have byte-identical 131-row
solver/contact projections over step, contacts, exact_solves, warm_starts, boxed_lcp_fallbacks, status. That is not
full-state trace equivalence.

Claim boundary: the rendered demo and CSV exporter are separate scene
implementations. The trace exports translation and pose alignment but not
angular velocity. At the encoded 30/15 fps rates, the -200 rad/s motion can
alias, so sampled media do not prove signed angular direction. A contact-free
post-initial step is retained and uninterrupted contact is not claimed. No
landing phase, faithful external-solver row, approved source golden/diff,
author-scene parity, paper timing, or real-time performance verdict is supplied.
