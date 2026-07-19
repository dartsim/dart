# Card-House Native Manifold Sensitivity v2

Comparison artifact integrity valid: **yes**.
Paper parity: **no**. Timing verdict: **none**.

## compact

- process exit class: complete_exit_zero
- emitted steps: 600/600
- first failed step: None
- strict successes / accepted caps / failures: 0 / 600 / 0
- terminal wrapper/internal status: success / success
- terminal residual: 8.5256787384150482e-07
- contact range: 39..155
- unique-pair range: 30..57

## four_point_planar

- process exit class: complete_terminal_convergence_gate_failure
- emitted steps: 600/600
- first failed step: None
- strict successes / accepted caps / failures: 0 / 600 / 0
- terminal wrapper/internal status: max_iterations_accepted / max_iterations
- terminal residual: 0.016582575623909489
- contact range: 124..200
- unique-pair range: 35..54

The only intended factor is Native contact-manifold mode. Raw wall time is
retained for transparency but is excluded from every scientific verdict.

Comparison delta object: `{"accepted_cap_exact_groups": -2813, "accepted_cap_rows": 0, "emitted_steps": 0, "exact_attempts": -5012, "exact_failure_rows": 0, "exact_failures": 0, "first_failed_step": null, "maximum_card_displacement": -1.5867747630610847, "mean_complementarity_residual": -0.0032151849872528865, "mean_contacts": 93.79833333333335, "mean_dual_residual": 0.009986555035270892, "mean_pair_multiplicity": 1.9548548971156625, "mean_primal_residual": -5.645637323328349e-18, "mean_unique_pairs": 2.375, "mean_warm_start_matched_contacts": 136.81833333333336, "mean_warm_start_matched_fraction": 0.7706870466702074, "minimum_card_orientation_alignment": -0.36316279674589536, "pair_identity_transition_rows": -398, "pair_identity_union_count": 18, "pair_multiplicity_transition_rows": -376, "step_size_persistence_used_rows": 213, "strict_success_rows": 0, "terminal_best_iteration": 200, "terminal_best_residual": 0.016581723056035648, "terminal_iterations": -2664, "terminal_residual": 0.016581723056035648}`
