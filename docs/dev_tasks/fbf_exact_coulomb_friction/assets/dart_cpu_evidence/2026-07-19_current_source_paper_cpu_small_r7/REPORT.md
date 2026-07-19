# FBF CPU evidence summary

Raw timings cover `World::step()` only. A realtime or paper-target verdict is emitted only when its corresponding evidence contract is valid. Blank target cells mean unevaluated, not failed.

`Mean realtime met` is a throughput-average verdict. `Every-step deadline met` is the stricter latency verdict: every measured `World::step()` must finish below 16.6666667 ms.

A requested case/thread group with no measured rows remains visible and is marked incomplete; `--accept-nonzero` never turns a failed process into successful evidence.

| Scenario | Frontend | Threads | Affinity | Samples | Failed measured | Failed warmups | Complete | Outcome | Controlled affinity | Mean ms | p95 ms | RTF | Realtime contract valid | Mean realtime met | Every-step deadline met | Residual pass | Fallbacks | Paper ref ms | Paper evaluated/met |
| --- | --- | ---: | --- | ---: | ---: | ---: | --- | --- | --- | ---: | ---: | ---: | --- | --- | --- | ---: | ---: | ---: | --- |
| backspin | native | 1 | 4 | 720 | 0 | 0 | True | True | True | 0.388122 | 0.813278 | 42.9419 | True | True | True | 1.000 | 0 | 6 | False/None |
| incline_mu_0_4 | native | 1 | 4 | 360 | 0 | 0 | True | True | True | 0.149147 | 0.209554 | 111.747 | True | True | True | 1.000 | 0 | 5.3 | False/None |
| incline_mu_0_5 | native | 1 | 4 | 360 | 0 | 0 | True | True | True | 0.314663 | 0.410601 | 52.9667 | False | None | None | 0.992 | 0 | 5.5 | False/None |
| painleve_mu_0_5 | native | 1 | 4 | 450 | 0 | 0 | True | True | True | 0.105293 | 0.346149 | 158.288 | True | True | True | 1.000 | 0 | 7 | False/None |
| painleve_mu_0_55 | native | 1 | 4 | 450 | 0 | 0 | True | True | True | 0.119171 | 0.632581 | 139.856 | True | True | True | 1.000 | 0 | 6.4 | False/None |
| turntable_mu_0_2_omega_2 | native | 1 | 4 | 720 | 0 | 0 | True | True | True | 0.22362 | 0.587558 | 74.5311 | True | True | True | 1.000 | 0 | 3.7 | False/None |
| turntable_mu_0_2_omega_5 | native | 1 | 4 | 720 | 0 | 0 | True | True | True | 0.198401 | 0.606793 | 84.0049 | True | True | True | 1.000 | 0 | 3.1 | False/None |
| turntable_mu_0_5_omega_2 | native | 1 | 4 | 720 | 0 | 0 | True | True | True | 0.923083 | 1.94742 | 18.0554 | True | True | True | 1.000 | 0 | 6.8 | False/None |
| turntable_mu_0_5_omega_5 | native | 1 | 4 | 720 | 3 | 0 | True | True | True | 0.463364 | 1.7557 | 35.9688 | False | None | None | 0.989 | 0 | 3.1 | False/None |

## Physical outcome audit

| Scenario | Threads | Valid | Reasons | Details |
| --- | ---: | --- | --- | --- |
| backspin | 1 | True | valid | rep1:passed=true,vx=-11.5674647,expected_vx=-11.4285714,vx_tolerance=0.5,height=0.248827785,height_expected=0.25,height_tolerance=0.03,contacts=1<br>rep2:passed=true,vx=-11.5674647,expected_vx=-11.4285714,vx_tolerance=0.5,height=0.248827785,height_expected=0.25,height_tolerance=0.03,contacts=1<br>rep3:passed=true,vx=-11.5674647,expected_vx=-11.4285714,vx_tolerance=0.5,height=0.248827785,height_expected=0.25,height_tolerance=0.03,contacts=1 |
| incline_mu_0_4 | 1 | True | valid | rep1:passed=true,displacement=1.7686901,expected=1.75486615,tolerance=0.2,sustained_contact=true,max_penetration=0.01,max_penetration_limit=0.02<br>rep2:passed=true,displacement=1.7686901,expected=1.75486615,tolerance=0.2,sustained_contact=true,max_penetration=0.01,max_penetration_limit=0.02<br>rep3:passed=true,displacement=1.7686901,expected=1.75486615,tolerance=0.2,sustained_contact=true,max_penetration=0.01,max_penetration_limit=0.02 |
| incline_mu_0_5 | 1 | True | valid | rep1:passed=true,displacement=8.63436433e-07,expected=0,tolerance=0.02,sustained_contact=true,max_penetration=0.01,max_penetration_limit=0.02<br>rep2:passed=true,displacement=8.63436433e-07,expected=0,tolerance=0.02,sustained_contact=true,max_penetration=0.01,max_penetration_limit=0.02<br>rep3:passed=true,displacement=8.63436433e-07,expected=0,tolerance=0.02,sustained_contact=true,max_penetration=0.01,max_penetration_limit=0.02 |
| painleve_mu_0_5 | 1 | True | valid | rep1:passed=true,expected=upright,up_z=0.999999995,height=0.47585413<br>rep2:passed=true,expected=upright,up_z=0.999999995,height=0.47585413<br>rep3:passed=true,expected=upright,up_z=0.999999995,height=0.47585413 |
| painleve_mu_0_55 | 1 | True | valid | rep1:passed=true,expected=tumbled,up_z=0.000839850453,height=0.02801739<br>rep2:passed=true,expected=tumbled,up_z=0.000839850453,height=0.02801739<br>rep3:passed=true,expected=tumbled,up_z=0.000839850453,height=0.02801739 |
| turntable_mu_0_2_omega_2 | 1 | True | valid | rep1:passed=true,expected=ejected,radius=6.34473784,height=-12.8000392,contacts=0,radial_velocity=nan,tangential_velocity=nan,corotation_speed_error=nan<br>rep2:passed=true,expected=ejected,radius=6.34473784,height=-12.8000392,contacts=0,radial_velocity=nan,tangential_velocity=nan,corotation_speed_error=nan<br>rep3:passed=true,expected=ejected,radius=6.34473784,height=-12.8000392,contacts=0,radial_velocity=nan,tangential_velocity=nan,corotation_speed_error=nan |
| turntable_mu_0_2_omega_5 | 1 | True | valid | rep1:passed=true,expected=ejected,radius=7.86699773,height=-27.9045884,contacts=0,radial_velocity=nan,tangential_velocity=nan,corotation_speed_error=nan<br>rep2:passed=true,expected=ejected,radius=7.86699773,height=-27.9045884,contacts=0,radial_velocity=nan,tangential_velocity=nan,corotation_speed_error=nan<br>rep3:passed=true,expected=ejected,radius=7.86699773,height=-27.9045884,contacts=0,radial_velocity=nan,tangential_velocity=nan,corotation_speed_error=nan |
| turntable_mu_0_5_omega_2 | 1 | True | valid | rep1:passed=true,expected=captured,radius=1.1171459,height=0.123884823,contacts=4,radial_velocity=0.074393968,tangential_velocity=2.23181191,corotation_speed_error=0.00247989669<br>rep2:passed=true,expected=captured,radius=1.1171459,height=0.123884823,contacts=4,radial_velocity=0.074393968,tangential_velocity=2.23181191,corotation_speed_error=0.00247989669<br>rep3:passed=true,expected=captured,radius=1.1171459,height=0.123884823,contacts=4,radial_velocity=0.074393968,tangential_velocity=2.23181191,corotation_speed_error=0.00247989669 |
| turntable_mu_0_5_omega_5 | 1 | True | valid | rep1:passed=true,expected=ejected,radius=23.473496,height=-31.4016276,contacts=0,radial_velocity=nan,tangential_velocity=nan,corotation_speed_error=nan<br>rep2:passed=true,expected=ejected,radius=23.473496,height=-31.4016276,contacts=0,radial_velocity=nan,tangential_velocity=nan,corotation_speed_error=nan<br>rep3:passed=true,expected=ejected,radius=23.473496,height=-31.4016276,contacts=0,radial_velocity=nan,tangential_velocity=nan,corotation_speed_error=nan |

## Exact-kernel parallelism audit

| Scenario | Threads | Contract | Row products | Parallel products | Parallel fraction | Dispatch valid | Lifetime CPU residency | Per-phase-best CPUs | Phase residency valid/reasons | Multicore valid | Scaling pair valid/reasons | Raw/validated speedup |
| --- | ---: | --- | ---: | ---: | ---: | --- | --- | --- | --- | --- | --- | --- |
| backspin | 1 | single_simulation_thread | 121188 | 0 | 0.000 | False | none (0 cores) | rep1=none<br>rep2=none<br>rep3=none | False<br>not_applicable_single_thread | False | None<br>not_applicable_one_thread_baseline | 1/1 |
| incline_mu_0_4 | 1 | single_simulation_thread | 11016 | 0 | 0.000 | False | none (0 cores) | rep1=none<br>rep2=none<br>rep3=none | False<br>not_applicable_single_thread | False | None<br>not_applicable_one_thread_baseline | 1/1 |
| incline_mu_0_5 | 1 | single_simulation_thread | 36660 | 0 | 0.000 | False | none (0 cores) | rep1=none<br>rep2=none<br>rep3=none | False<br>not_applicable_single_thread | False | None<br>not_applicable_one_thread_baseline | 1/ |
| painleve_mu_0_5 | 1 | single_simulation_thread | 73698 | 0 | 0.000 | False | none (0 cores) | rep1=none<br>rep2=none<br>rep3=none | False<br>not_applicable_single_thread | False | None<br>not_applicable_one_thread_baseline | 1/1 |
| painleve_mu_0_55 | 1 | single_simulation_thread | 43626 | 0 | 0.000 | False | none (0 cores) | rep1=none<br>rep2=none<br>rep3=none | False<br>not_applicable_single_thread | False | None<br>not_applicable_one_thread_baseline | 1/1 |
| turntable_mu_0_2_omega_2 | 1 | single_simulation_thread | 28311 | 0 | 0.000 | False | none (0 cores) | rep1=none<br>rep2=none<br>rep3=none | False<br>not_applicable_single_thread | False | None<br>not_applicable_one_thread_baseline | 1/1 |
| turntable_mu_0_2_omega_5 | 1 | single_simulation_thread | 15744 | 0 | 0.000 | False | none (0 cores) | rep1=none<br>rep2=none<br>rep3=none | False<br>not_applicable_single_thread | False | None<br>not_applicable_one_thread_baseline | 1/1 |
| turntable_mu_0_5_omega_2 | 1 | single_simulation_thread | 274272 | 0 | 0.000 | False | none (0 cores) | rep1=none<br>rep2=none<br>rep3=none | False<br>not_applicable_single_thread | False | None<br>not_applicable_one_thread_baseline | 1/1 |
| turntable_mu_0_5_omega_5 | 1 | single_simulation_thread | 47019 | 0 | 0.000 | False | none (0 cores) | rep1=none<br>rep2=none<br>rep3=none | False<br>not_applicable_single_thread | False | None<br>not_applicable_one_thread_baseline | 1/ |

Contract rejection reasons are preserved in `summary.csv` and `summary.json`. The legacy dart_best multicore verdict requires measured parallel contact-row products. The explicitly non-paper colored contract instead requires colored path use, exactly one persistent dispatch per eligible multi-thread exact attempt, per-positive-work-step schedule width at least equal to the requested threads, no legacy parallel contact-row activity, and colored-dispatch CPU residency. In both cases every repetition's largest within-phase CPU-ID set must map to the requested number of pinned physical cores. Lifetime CPU-ID unions are audit-only because migration across phases can populate them. Runtime CPU IDs are residency observations, not proof of perfect simultaneity.

Validated scaling additionally requires a nested one-thread CPU list and physical mapping, matching package, SMT topology, cpuinfo maximum-frequency core class, scaling governor, solver/workload options, and an exact measured-work trajectory fingerprint. Every configured warmup must also complete successfully. Raw speedup remains visible when a scaling-pair gate fails.

Schema v8 preserves the 83-column v7 default trace byte-for-byte and selects a 95-column extended trace only for the non-paper colored contract. The paper's local block formula, gamma-recovery rule, and DART split-impulse equivalence are not published, so those gaps keep an apples-to-apples paper verdict unevaluated.

See `metadata.json`, `invocations.json`, `raw.csv`, and the per-process files under `raw/` for the complete evidence chain.
