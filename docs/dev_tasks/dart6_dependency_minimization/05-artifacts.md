# Phase-0 baseline packet — raw evidence appendix

> Companion to [05-phase0-baseline-packet.md](05-phase0-baseline-packet.md).
> Committed so the packet's row summaries, commands, host metadata, and
> analyzer are auditable from a clean checkout. Full JSONL scene dumps are
> not committed; their SHA-256 digests below pin the exact local artifacts
> used for the tolerance table. To reuse the per-body tolerance gate later,
> retrieve a dump archive matching these digests or recapture dumps on the
> phase-6/default-flip PR parent and compare within that same recapture.

## Host metadata

```
captured_at_utc: 2026-07-05T17:21:55Z
head: 1e6a8332a730a994450c14ee8a780780c5e069bb
origin/release-6.20: 949a9c2ff5ed6309beef0aa1345101d36c813f02
cpu: 13th Gen Intel(R) Core(TM) i9-13950HX
governor: powersave
nproc: 32
compiler(pixi): c++ (Ubuntu 15.2.0-16ubuntu1) 15.2.0
pixi: pixi 0.72.0
```
## Scene-dump digests (SHA-256)

```
e217a13015f9ba920abe1a46c4d29f36b19ff8c91dd828cfd23ad8115f64ae82  g120-bullet.jsonl
e49f88af35a1cf86ea27cdac4f48c3093f1f133f511f21a42e0ee48e1cb63d75  g120-dart.jsonl
8c9b6d68a6f8916370e9bb15f38e3e52bf695bdd1e1b90672dfb4936cd6af4b2  g120-default.jsonl
8c9b6d68a6f8916370e9bb15f38e3e52bf695bdd1e1b90672dfb4936cd6af4b2  g120-fcl.jsonl
2f55e6965452f7a07416a2fe965e0b941bd84f25366e9c5b7d1d294d123da931  g120-ode.jsonl
8d6c253964f711314c7edbfc50554f20852f9d628f82e17e1b95794a75851805  g3000-bullet.jsonl
5df8a61ed2746561fc4c93b56be61933bbdb37f16891a231628393daf62f8a66  g3000-dart.jsonl
3883d37c8e2117f3830a95cbb050766d232382e93a8e052af70a383ed97085b7  g3000-fcl.jsonl
58f18677e5988921f14614296a089f4551a3d9e00f6089574df7970c0b7889ef  g3000-ode.jsonl
b87c2bcdb1e542a969ebbc41d79be566d491ca392bd4b8e2d36461c271531ede  s2-3ksettled-bullet.jsonl
4f9e6e4c3689b19301f002089426d765fe5bd1f50f72a0bef4b93dddef310eb6  s2-3ksettled-dart.jsonl
ab9100a4add7f062506fa760cc0ec4318bb6ae51ae47db25bf3599a6d8c708b2  s2-3ksettled-fcl.jsonl
552c4cb978853647103b93c59657f34fd3b4572b0d427075c6009649ba4b5e7d  s2-3ksettled-ode.jsonl
eb8c1eca5ed5673b05833e768b81d64c8d7439737cc757a57d2d44e38fdfea56  s4-900-bullet.jsonl
62be3b522a96b4f7c9b445a7130b2458eada17c17c4e1fd066acaaa70113e2da  s4-900-dart.jsonl
caee295d33590e05e78ac767fbc32048eb1e5716d6f55908471e8e7016d82ef6  s4-900-fcl.jsonl
89e9b139b31f00b2ccd958f602828ef8191c26da051dbcb0101c5fec217199a0  s4-900-ode.jsonl
```
## Per-row final summaries (raw benchmark output)

### g120-default

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    1.43337 s
Real-Time Factor:   0.209297
Avg Step Time:      4.77789 ms/step
Final Contacts:     240
Final Contact Cap Hit: false
Final Contact Pairs: 120
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0.000751709
Final Resting:      60 / 120 mobile skeletons
Final Candidates:   60 / 120 mobile skeletons
Final Islanded:     120 / 120 mobile skeletons
Final Disturbed:    0 / 120 mobile skeletons
Final Max Smoothed Linear Speed: 0.0117064
Final Max Smoothed Angular Speed: 0.0235642
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0xe0fe5d6773190ca1
Final State Sums:   position_l1 2721.98 velocity_l1 1.13604 body_translation_l1 2721.97 max_abs_velocity 0.0216858
```

### g120-dart

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    0.618194 s
Real-Time Factor:   0.485285
Avg Step Time:      2.06065 ms/step
Final Contacts:     240
Final Contact Cap Hit: false
Final Contact Pairs: 120
Final Over Sleep Tol: 1
Final Zero Normals:  0
Final Max Penetration: 0.00100019
Final Resting:      80 / 120 mobile skeletons
Final Candidates:   80 / 120 mobile skeletons
Final Islanded:     120 / 120 mobile skeletons
Final Disturbed:    0 / 120 mobile skeletons
Final Max Smoothed Linear Speed: 0.0119544
Final Max Smoothed Angular Speed: 0.0240463
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0xbd8a15e877f70822
Final State Sums:   position_l1 2721.97 velocity_l1 1.19619 body_translation_l1 2721.97 max_abs_velocity 0.0221249
```

### g120-fcl

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    0.560896 s
Real-Time Factor:   0.534858
Avg Step Time:      1.86965 ms/step
Final Contacts:     240
Final Contact Cap Hit: false
Final Contact Pairs: 120
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0.000751709
Final Resting:      60 / 120 mobile skeletons
Final Candidates:   60 / 120 mobile skeletons
Final Islanded:     120 / 120 mobile skeletons
Final Disturbed:    0 / 120 mobile skeletons
Final Max Smoothed Linear Speed: 0.0117064
Final Max Smoothed Angular Speed: 0.0235642
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0xe0fe5d6773190ca1
Final State Sums:   position_l1 2721.98 velocity_l1 1.13604 body_translation_l1 2721.97 max_abs_velocity 0.0216858
```

### g120-bullet

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    0.612872 s
Real-Time Factor:   0.489499
Avg Step Time:      2.04291 ms/step
Final Contacts:     268
Final Contact Cap Hit: false
Final Contact Pairs: 120
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 9.99123e-06
Final Resting:      78 / 120 mobile skeletons
Final Candidates:   78 / 120 mobile skeletons
Final Islanded:     120 / 120 mobile skeletons
Final Disturbed:    0 / 120 mobile skeletons
Final Max Smoothed Linear Speed: 5.08999e-06
Final Max Smoothed Angular Speed: 1.24728e-13
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x3508439b0c95d6be
Final State Sums:   position_l1 2722 velocity_l1 0.000195453 body_translation_l1 2722 max_abs_velocity 4.67027e-06
```

### g120-ode

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    0.0264457 s
Real-Time Factor:   11.344
Avg Step Time:      0.0881525 ms/step
Final Contacts:     0
Final Contact Cap Hit: false
Final Contact Pairs: 0
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0
Final Resting:      120 / 120 mobile skeletons
Final Candidates:   120 / 120 mobile skeletons
Final Islanded:     120 / 120 mobile skeletons
Final Disturbed:    0 / 120 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0xe52ba4524e3c5db2
Final State Sums:   position_l1 2722 velocity_l1 0 body_translation_l1 2722 max_abs_velocity 0
```

### s5-90-dart

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    0.254206 s
Real-Time Factor:   1.18015
Avg Step Time:      0.847353 ms/step
Final Contacts:     180
Final Contact Cap Hit: false
Final Contact Pairs: 90
Final Over Sleep Tol: 1
Final Zero Normals:  0
Final Max Penetration: 0.00100129
Final Resting:      60 / 90 mobile skeletons
Final Candidates:   60 / 90 mobile skeletons
Final Islanded:     90 / 90 mobile skeletons
Final Disturbed:    0 / 90 mobile skeletons
Final Max Smoothed Linear Speed: 0.00882222
Final Max Smoothed Angular Speed: 0.0175163
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x726d1ff51bdb717
Final State Sums:   position_l1 1546.48 velocity_l1 0.726261 body_translation_l1 1546.48 max_abs_velocity 0.0136637
```

### s5-90-fcl

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    1.38653 s
Real-Time Factor:   0.216367
Avg Step Time:      4.62178 ms/step
Final Contacts:     180
Final Contact Cap Hit: false
Final Contact Pairs: 90
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0.000758781
Final Resting:      45 / 90 mobile skeletons
Final Candidates:   45 / 90 mobile skeletons
Final Islanded:     90 / 90 mobile skeletons
Final Disturbed:    0 / 90 mobile skeletons
Final Max Smoothed Linear Speed: 0.00890269
Final Max Smoothed Angular Speed: 0.0177148
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x99bfaef49c254203
Final State Sums:   position_l1 1546.48 velocity_l1 0.752144 body_translation_l1 1546.48 max_abs_velocity 0.0159782
```

### s5-90-bullet

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    0.616232 s
Real-Time Factor:   0.486829
Avg Step Time:      2.05411 ms/step
Final Contacts:     210
Final Contact Cap Hit: false
Final Contact Pairs: 90
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 9.99123e-06
Final Resting:      56 / 90 mobile skeletons
Final Candidates:   56 / 90 mobile skeletons
Final Islanded:     90 / 90 mobile skeletons
Final Disturbed:    0 / 90 mobile skeletons
Final Max Smoothed Linear Speed: 8.76543e-05
Final Max Smoothed Angular Speed: 0.000120772
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0xf78e3bd075780c83
Final State Sums:   position_l1 1546.5 velocity_l1 0.000204603 body_translation_l1 1546.5 max_abs_velocity 2.62356e-05
```

### s5-90-ode

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    0.00665041 s
Real-Time Factor:   45.11
Avg Step Time:      0.022168 ms/step
Final Contacts:     0
Final Contact Cap Hit: false
Final Contact Pairs: 0
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0
Final Resting:      90 / 90 mobile skeletons
Final Candidates:   90 / 90 mobile skeletons
Final Islanded:     90 / 90 mobile skeletons
Final Disturbed:    0 / 90 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x5f2afc7230ee8d10
Final State Sums:   position_l1 1546.5 velocity_l1 0 body_translation_l1 1546.5 max_abs_velocity 0
```

### s4-900-dart

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    7.70722 s
Real-Time Factor:   0.0389245
Avg Step Time:      25.6907 ms/step
Final Contacts:     1800
Final Contact Cap Hit: false
Final Contact Pairs: 900
Final Over Sleep Tol: 3
Final Zero Normals:  0
Final Max Penetration: 0.00104808
Final Resting:      600 / 900 mobile skeletons
Final Candidates:   600 / 900 mobile skeletons
Final Islanded:     900 / 900 mobile skeletons
Final Disturbed:    0 / 900 mobile skeletons
Final Max Smoothed Linear Speed: 0.0122954
Final Max Smoothed Angular Speed: 0.0248171
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x76205ad68f4293bb
Final State Sums:   position_l1 149115 velocity_l1 7.65769 body_translation_l1 149115 max_abs_velocity 0.0235974
```

### s4-900-fcl

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    10.4133 s
Real-Time Factor:   0.0288092
Avg Step Time:      34.7111 ms/step
Final Contacts:     1800
Final Contact Cap Hit: false
Final Contact Pairs: 900
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0.000976287
Final Resting:      450 / 900 mobile skeletons
Final Candidates:   450 / 900 mobile skeletons
Final Islanded:     900 / 900 mobile skeletons
Final Disturbed:    0 / 900 mobile skeletons
Final Max Smoothed Linear Speed: 0.0122622
Final Max Smoothed Angular Speed: 0.0246168
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x7a0974e837912472
Final State Sums:   position_l1 149115 velocity_l1 8.22511 body_translation_l1 149115 max_abs_velocity 0.0229626
```

### s4-900-bullet

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    9.22561 s
Real-Time Factor:   0.0325182
Avg Step Time:      30.752 ms/step
Final Contacts:     2353
Final Contact Cap Hit: false
Final Contact Pairs: 900
Final Over Sleep Tol: 26
Final Zero Normals:  0
Final Max Penetration: 0.011297
Final Resting:      269 / 900 mobile skeletons
Final Candidates:   269 / 900 mobile skeletons
Final Islanded:     900 / 900 mobile skeletons
Final Disturbed:    0 / 900 mobile skeletons
Final Max Smoothed Linear Speed: 0.0349737
Final Max Smoothed Angular Speed: 0.0703123
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x2a5577952e2de925
Final State Sums:   position_l1 149115 velocity_l1 0.685046 body_translation_l1 149115 max_abs_velocity 0.0667275
```

### s4-900-ode

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    0.0954347 s
Real-Time Factor:   3.14351
Avg Step Time:      0.318116 ms/step
Final Contacts:     0
Final Contact Cap Hit: false
Final Contact Pairs: 0
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0
Final Resting:      900 / 900 mobile skeletons
Final Candidates:   900 / 900 mobile skeletons
Final Islanded:     900 / 900 mobile skeletons
Final Disturbed:    0 / 900 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x429b65bc5c4a14b6
Final State Sums:   position_l1 149115 velocity_l1 0 body_translation_l1 149115 max_abs_velocity 0
```

### g3000-dart

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    29.8716 s
Real-Time Factor:   0.010043
Avg Step Time:      99.5719 ms/step
Final Contacts:     6000
Final Contact Cap Hit: false
Final Contact Pairs: 3000
Final Over Sleep Tol: 4
Final Zero Normals:  0
Final Max Penetration: 0.00106022
Final Resting:      2000 / 3000 mobile skeletons
Final Candidates:   2000 / 3000 mobile skeletons
Final Islanded:     3000 / 3000 mobile skeletons
Final Disturbed:    0 / 3000 mobile skeletons
Final Max Smoothed Linear Speed: 0.0132607
Final Max Smoothed Angular Speed: 0.0268575
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x48680325dda0fd3f
Final State Sums:   position_l1 1.65205e+06 velocity_l1 27.7945 body_translation_l1 1.65205e+06 max_abs_velocity 0.0258923
```

### g3000-fcl

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    40.1363 s
Real-Time Factor:   0.00747453
Avg Step Time:      133.788 ms/step
Final Contacts:     6000
Final Contact Cap Hit: false
Final Contact Pairs: 3000
Final Over Sleep Tol: 6
Final Zero Normals:  0
Final Max Penetration: 0.00104462
Final Resting:      1500 / 3000 mobile skeletons
Final Candidates:   1500 / 3000 mobile skeletons
Final Islanded:     3000 / 3000 mobile skeletons
Final Disturbed:    0 / 3000 mobile skeletons
Final Max Smoothed Linear Speed: 0.0136196
Final Max Smoothed Angular Speed: 0.0275731
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0xe1873605e7be1a9
Final State Sums:   position_l1 1.65205e+06 velocity_l1 28.3271 body_translation_l1 1.65205e+06 max_abs_velocity 0.0258908
```

### g3000-bullet

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    21.6445 s
Real-Time Factor:   0.0138603
Avg Step Time:      72.1484 ms/step
Final Contacts:     8633
Final Contact Cap Hit: false
Final Contact Pairs: 3000
Final Over Sleep Tol: 2252
Final Zero Normals:  0
Final Max Penetration: 0.439973
Final Resting:      635 / 3000 mobile skeletons
Final Candidates:   635 / 3000 mobile skeletons
Final Islanded:     3000 / 3000 mobile skeletons
Final Disturbed:    0 / 3000 mobile skeletons
Final Max Smoothed Linear Speed: 2.90376
Final Max Smoothed Angular Speed: 0.884198
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0xd49e015847f0a8f
Final State Sums:   position_l1 1.65194e+06 velocity_l1 819.95 body_translation_l1 1.65193e+06 max_abs_velocity 2.943
```

### g3000-ode

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    0.580585 s
Real-Time Factor:   0.51672
Avg Step Time:      1.93528 ms/step
Final Contacts:     0
Final Contact Cap Hit: false
Final Contact Pairs: 0
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0
Final Resting:      3000 / 3000 mobile skeletons
Final Candidates:   3000 / 3000 mobile skeletons
Final Islanded:     3000 / 3000 mobile skeletons
Final Disturbed:    0 / 3000 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x559483fad1bd256f
Final State Sums:   position_l1 1.65205e+06 velocity_l1 0 body_translation_l1 1.65205e+06 max_abs_velocity 0
```

### s2-3ksettled-dart

```
Simulated Time:     3 s
Expected Sim Time:  3 s
World Time Before:  0 s
World Time After:   3 s
Frame Delta:        3000 / 3000
Time Advanced:      true
Wall-Clock Time:    0.422625 s
Real-Time Factor:   7.0985
Avg Step Time:      0.140875 ms/step
Final Contacts:     0
Final Contact Cap Hit: false
Final Contact Pairs: 0
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0
Final Resting:      3003 / 3003 mobile skeletons
Final Candidates:   3003 / 3003 mobile skeletons
Final Islanded:     3003 / 3003 mobile skeletons
Final Disturbed:    0 / 3003 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x8ddc9a81f2d28a7f
Final State Sums:   position_l1 2.70041e-07 velocity_l1 0 body_translation_l1 1.13175e+06 max_abs_velocity 0
```

### s2-3ksettled-fcl

```
Simulated Time:     3 s
Expected Sim Time:  3 s
World Time Before:  0 s
World Time After:   3 s
Frame Delta:        3000 / 3000
Time Advanced:      true
Wall-Clock Time:    0.444688 s
Real-Time Factor:   6.74631
Avg Step Time:      0.148229 ms/step
Final Contacts:     0
Final Contact Cap Hit: false
Final Contact Pairs: 0
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0
Final Resting:      3003 / 3003 mobile skeletons
Final Candidates:   3003 / 3003 mobile skeletons
Final Islanded:     3003 / 3003 mobile skeletons
Final Disturbed:    0 / 3003 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x266da31836a314a6
Final State Sums:   position_l1 2.94591e-07 velocity_l1 0 body_translation_l1 1.13175e+06 max_abs_velocity 0
```

### s2-3ksettled-bullet

```
Simulated Time:     3 s
Expected Sim Time:  3 s
World Time Before:  0 s
World Time After:   3 s
Frame Delta:        3000 / 3000
Time Advanced:      true
Wall-Clock Time:    0.71217 s
Real-Time Factor:   4.21248
Avg Step Time:      0.23739 ms/step
Final Contacts:     0
Final Contact Cap Hit: false
Final Contact Pairs: 0
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0
Final Resting:      3003 / 3003 mobile skeletons
Final Candidates:   3003 / 3003 mobile skeletons
Final Islanded:     3003 / 3003 mobile skeletons
Final Disturbed:    0 / 3003 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x2375f1927218cd43
Final State Sums:   position_l1 0.0172802 velocity_l1 0 body_translation_l1 1.13175e+06 max_abs_velocity 0
```

### s2-3ksettled-ode

```
Simulated Time:     3 s
Expected Sim Time:  3 s
World Time Before:  0 s
World Time After:   3 s
Frame Delta:        3000 / 3000
Time Advanced:      true
Wall-Clock Time:    0.703499 s
Real-Time Factor:   4.2644
Avg Step Time:      0.2345 ms/step
Final Contacts:     0
Final Contact Cap Hit: false
Final Contact Pairs: 0
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 0
Final Resting:      3003 / 3003 mobile skeletons
Final Candidates:   3003 / 3003 mobile skeletons
Final Islanded:     3003 / 3003 mobile skeletons
Final Disturbed:    0 / 3003 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0.5
Final State Finite: true
Final State Hash:   0x10f80b0408cede90
Final State Sums:   position_l1 2.70041e-07 velocity_l1 0 body_translation_l1 1.13175e+06 max_abs_velocity 0
```

### s3-3kactive-dart

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    24.4713 s
Real-Time Factor:   0.0122593
Avg Step Time:      81.571 ms/step
Final Contacts:     5005
Final Contact Cap Hit: false
Final Contact Pairs: 3003
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 9.32405e-09
Final Resting:      0 / 3003 mobile skeletons
Final Candidates:   0 / 3003 mobile skeletons
Final Islanded:     0 / 3003 mobile skeletons
Final Disturbed:    0 / 3003 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0
Final State Finite: true
Final State Hash:   0xcf0ba6eaa97be038
Final State Sums:   position_l1 2.568e-05 velocity_l1 1.33769e-05 body_translation_l1 1.13175e+06 max_abs_velocity 4.8595e-09
```

### s3-3kactive-fcl

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    25.3179 s
Real-Time Factor:   0.0118493
Avg Step Time:      84.3931 ms/step
Final Contacts:     3003
Final Contact Cap Hit: false
Final Contact Pairs: 3003
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 9.32405e-09
Final Resting:      0 / 3003 mobile skeletons
Final Candidates:   0 / 3003 mobile skeletons
Final Islanded:     0 / 3003 mobile skeletons
Final Disturbed:    0 / 3003 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0
Final State Finite: true
Final State Hash:   0x6088ea0177efa6a
Final State Sums:   position_l1 2.80147e-05 velocity_l1 1.45931e-05 body_translation_l1 1.13175e+06 max_abs_velocity 4.8595e-09
```

### s3-3kactive-bullet

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    13.152 s
Real-Time Factor:   0.0228102
Avg Step Time:      43.84 ms/step
Final Contacts:     5005
Final Contact Cap Hit: false
Final Contact Pairs: 3003
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 2.98023e-07
Final Resting:      0 / 3003 mobile skeletons
Final Candidates:   0 / 3003 mobile skeletons
Final Islanded:     0 / 3003 mobile skeletons
Final Disturbed:    0 / 3003 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0
Final State Finite: true
Final State Hash:   0x22e27960cbabe83e
Final State Sums:   position_l1 0.000543338 velocity_l1 0.00443988 body_translation_l1 1.13175e+06 max_abs_velocity 2.90666e-06
```

### s3-3kactive-ode

```
Simulated Time:     0.3 s
Expected Sim Time:  0.3 s
World Time Before:  0 s
World Time After:   0.3 s
Frame Delta:        300 / 300
Time Advanced:      true
Wall-Clock Time:    62.0369 s
Real-Time Factor:   0.00483583
Avg Step Time:      206.79 ms/step
Final Contacts:     9009
Final Contact Cap Hit: false
Final Contact Pairs: 3003
Final Over Sleep Tol: 0
Final Zero Normals:  0
Final Max Penetration: 9.32405e-09
Final Resting:      0 / 3003 mobile skeletons
Final Candidates:   0 / 3003 mobile skeletons
Final Islanded:     0 / 3003 mobile skeletons
Final Disturbed:    0 / 3003 mobile skeletons
Final Max Smoothed Linear Speed: 0
Final Max Smoothed Angular Speed: 0
Final Max Rest Dwell Time: 0
Final State Finite: true
Final State Hash:   0x4904c09a93a36442
Final State Sums:   position_l1 2.21797e-05 velocity_l1 1.15343e-05 body_translation_l1 1.13175e+06 max_abs_velocity 4.8595e-09
```

## Capture driver

```bash
#!/usr/bin/env bash
# Phase-0 baseline matrix for the native-collision port
# (docs/dev_tasks/dart6_dependency_minimization). Reuses the canonical
# guard-scene protocol captured by the retired performance round's WP-PG.01
# evidence (S2/S3/S4/S5) and adds the scoping-doc delta rows (g120 incl. `default`
# detector, g3000). ODE rows valid only with --max-contacts-per-pair 4.
set -uo pipefail
cd "$(git rev-parse --show-toplevel)"
ART=${1:-.omc/artifacts/native-collision-phase0}
mkdir -p "$ART"
CB=./build/default/cpp/Release/bin/contact_benchmark
SDF=.deps/gz-sim/examples/worlds/3k_shapes.sdf

{
  echo "captured_at_utc: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "head: $(git rev-parse HEAD)"
  echo "origin/release-6.20: $(git rev-parse origin/release-6.20)"
  echo "cpu: $(lscpu | rg 'Model name' | sed 's/.*: *//')"
  echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
  echo "nproc: $(nproc)"
  echo "compiler(pixi): $(pixi run c++ --version 2>/dev/null | head -1)"
  echo "pixi: $(pixi --version)"
} > "$ART/metadata.txt"

FAILED_ROWS=""
ROW_FAILURE=0
run_row() {
  local name=$1; shift
  echo "=== ROW $name START $(date -u +%H:%M:%SZ) ==="
  timeout 3600 pixi run "$CB" "$@" > "$ART/$name.out" 2>&1
  local rc=$?
  echo "=== ROW $name EXIT $rc $(date -u +%H:%M:%SZ) ==="
  if [ "$rc" -ne 0 ]; then
    FAILED_ROWS="$FAILED_ROWS $name(rc=$rc)"
    ROW_FAILURE=1
  fi
  return 0
}

# Continuity row (scoping-doc 2026-07-03 probe rerun, now under the canonical
# per-pair cap), including `default` to prove default==fcl at head.
for det in default dart fcl bullet ode; do
  run_row "g120-$det" --generate-objects 120 --steps 300 --warmup 0 \
    --checkpoint 0 --quiet --collision "$det" --world-threads 1 \
    --max-contacts 20000 --max-contacts-per-pair 4 \
    --dump-final-scene "$ART/g120-$det.jsonl"
done

for det in dart fcl bullet ode; do
  # S5 — serial 90 (cross-check against WP-PG.01 cells)
  run_row "s5-90-$det" --generate-objects 90 --steps 300 --warmup 0 \
    --checkpoint 0 --quiet --collision "$det" --world-threads 1 \
    --max-contacts 20000 --max-contacts-per-pair 4
  # S4 — generated 900
  run_row "s4-900-$det" --generate-objects 900 --steps 300 --warmup 0 \
    --checkpoint 0 --quiet --collision "$det" --world-threads 16 \
    --max-contacts 20000 --max-contacts-per-pair 4 \
    --dump-final-scene "$ART/s4-900-$det.jsonl"
  # Scoping delta — generated 3000
  run_row "g3000-$det" --generate-objects 3000 --steps 300 --warmup 0 \
    --checkpoint 0 --quiet --collision "$det" --world-threads 16 \
    --max-contacts 60000 --max-contacts-per-pair 4 \
    --dump-final-scene "$ART/g3000-$det.jsonl"
  # S2 — settled 3k (deactivation on)
  run_row "s2-3ksettled-$det" "$SDF" --steps 3000 --sdf-plane-shapes --quiet \
    --checkpoint 0 --collision "$det" --world-threads 1 \
    --max-contacts 12000 --max-contacts-per-pair 4 \
    --dump-final-scene "$ART/s2-3ksettled-$det.jsonl"
  # S3 — active 3k
  run_row "s3-3kactive-$det" "$SDF" --steps 300 --disable-deactivation \
    --world-threads 16 --max-contacts 12000 --max-contacts-per-pair 4 \
    --quiet --checkpoint 0 --sdf-plane-shapes --collision "$det"
done

# Keep collecting rows so MATRIX_FAILED reports every failure, but propagate
# failure at the end and never print MATRIX_DONE for an incomplete packet.
if [ "$ROW_FAILURE" -ne 0 ]; then
  echo "MATRIX_FAILED rows:$FAILED_ROWS"
  exit 1
fi
echo "MATRIX_DONE"
```

## Tolerance analyzer

```python
#!/usr/bin/env python3
"""Cross-detector scene-dump tolerance analysis for the phase-0 packet."""
import json
import math
import subprocess
import sys
from pathlib import Path

if len(sys.argv) > 1:
    ART = Path(sys.argv[1])
else:
    repo = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    ART = Path(repo) / ".omc/artifacts/native-collision-phase0"
SCENES = {
    "g120": ["fcl", "default", "dart", "bullet", "ode"],
    "s4-900": ["fcl", "dart", "bullet", "ode"],
    "g3000": ["fcl", "dart", "bullet", "ode"],
    "s2-3ksettled": ["fcl", "dart", "bullet", "ode"],
}


def load(path):
    bodies = {}
    with open(path) as f:
        for line in f:
            row = json.loads(line)
            if row.get("kind") != "shape" or not row.get("mobile"):
                continue
            key = (row["skeleton"], row["shape_node"])
            bodies[key] = (row["position"], row.get("resting"))
    return bodies


for scene, dets in SCENES.items():
    ref_name = dets[0]
    ref = load(ART / f"{scene}-{ref_name}.jsonl")
    print(f"\n## {scene} (reference: {ref_name}, {len(ref)} mobile shapes)")
    for det in dets[1:]:
        path = ART / f"{scene}-{det}.jsonl"
        if not path.exists():
            print(f"  {det}: MISSING DUMP")
            continue
        other = load(path)
        common = set(ref) & set(other)
        missing = len(ref) - len(common)
        deltas = []
        rest_mismatch = 0
        for key in common:
            p1, r1 = ref[key]
            p2, r2 = other[key]
            d = math.dist(p1, p2)
            deltas.append(d)
            if r1 != r2:
                rest_mismatch += 1
        deltas.sort()
        n = len(deltas)
        mean = sum(deltas) / n if n else float("nan")
        p95 = deltas[int(0.95 * (n - 1))] if n else float("nan")
        dmax = deltas[-1] if n else float("nan")
        print(
            f"  {det}: n={n} missing={missing} mean={mean:.4g} "
            f"p95={p95:.4g} max={dmax:.4g} resting_mismatch={rest_mismatch}"
        )
```
