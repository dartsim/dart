# DART main canonical behaviour guards

Generated at 2026-10-08T17:25:58.931498+00:00 for `e4cd718cc1666682730a5d5b4d9437d3b40ba264`.
Environment fingerprint: `07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0`.

Commands and scene definitions: [baseline evidence](https://github.com/dartsim/dart/blob/main/docs/dev_tasks/dart6_performance_generalization/01-baseline-evidence.md).
This table is generated evidence, not a fixed reference. Wall time is advisory.
S3 and S6 drift belongs to the #3056 / D7 owners.

| Row | Detector | Threads | Warm-up / steps | Status | Hash | Contacts | Pairs | Resting | Finite | Cap hit | Max penetration | Allocs / step |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1-60-t1 | dart | 1 | 0 / 200 | ok | 0x5fd4858cc9f34efb | 80 | 72 | 0/60 | True | False | 0.182088 | 1.465 |
| S1-60-t16 | dart | 16 | 0 / 200 | ok | 0x5fd4858cc9f34efb | 80 | 72 | 0/60 | True | False | 0.182088 | 10.765 |
| S1-60-t1 | ode | 1 | 0 / 200 | ok | 0xf9ac6baa42cedb3e | 142 | 75 | 0/60 | True | False | 0.192214 | 342.885 |
| S1-60-t16 | ode | 16 | 0 / 200 | ok | 0xf9ac6baa42cedb3e | 142 | 75 | 0/60 | True | False | 0.192214 | 342.885 |
| S1-120-t1 | dart | 1 | 0 / 200 | ok | 0x341871ecb68bbe26 | 251 | 177 | 0/120 | True | False | 0.312885 | 2.105 |
| S1-120-t16 | dart | 16 | 0 / 200 | ok | 0x341871ecb68bbe26 | 251 | 177 | 0/120 | True | False | 0.312885 | 11.5 |
| S1-120-t1 | ode | 1 | 0 / 200 | ok | 0x8f8d77d6d8ae91df | 274 | 182 | 0/120 | True | False | 0.302429 | 509.79 |
| S1-120-t16 | ode | 16 | 0 / 200 | ok | 0x8f8d77d6d8ae91df | 274 | 182 | 0/120 | True | False | 0.302429 | 509.79 |
| S2 | dart | 1 | 0 / 3000 | ok | 0x9308c8e0b3367e8f | 0 | 0 | 3003/3003 | True | False | 0.0 | 3.0676666666666668 |
| S2 | fcl | 1 | 0 / 3000 | ok | 0x9308c8e0b3367e8f | 0 | 0 | 3003/3003 | True | False | 0.0 | 9.058 |
| S2 | bullet | 1 | 0 / 3000 | ok | 0xe5f943c127942b5b | 0 | 0 | 3003/3003 | True | False | 0.0 | 3.7403333333333335 |
| S2 | ode | 1 | 0 / 3000 | ok | 0x1b08c3c93face3fa | 0 | 0 | 3003/3003 | True | False | 0.0 | 28.979333333333333 |
| S3-t1 | dart | 1 | 0 / 300 | ok | 0xe86faf4474573fd1 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 30.676666666666666 |
| S3-t4 | dart | 4 | 0 / 300 | ok | 0xe86faf4474573fd1 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 36.806666666666665 |
| S3-t16 | dart | 16 | 0 / 300 | ok | 0xe86faf4474573fd1 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 37.486666666666665 |
| S3-t1 | fcl | 1 | 0 / 300 | ok | 0xe7ff6160a083e067 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 3064.54 |
| S3-t4 | fcl | 4 | 0 / 300 | ok | 0xe7ff6160a083e067 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 3064.69 |
| S3-t16 | fcl | 16 | 0 / 300 | ok | 0xe7ff6160a083e067 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 3065.37 |
| S3-t1 | bullet | 1 | 0 / 300 | ok | 0x236a7ec3e4ff4149 | 5005 | 3003 | 0/3003 | True | False | 2.68221e-07 | 37.31333333333333 |
| S3-t4 | bullet | 4 | 0 / 300 | ok | 0x236a7ec3e4ff4149 | 5005 | 3003 | 0/3003 | True | False | 2.68221e-07 | 37.583333333333336 |
| S3-t16 | bullet | 16 | 0 / 300 | ok | 0x236a7ec3e4ff4149 | 5005 | 3003 | 0/3003 | True | False | 2.68221e-07 | 38.74333333333333 |
| S3-t1 | ode | 1 | 0 / 300 | ok | 0x1409272bff2b1bc4 | 9009 | 3003 | 0/3003 | True | False | 9.32405e-09 | 15870.963333333333 |
| S3-t4 | ode | 4 | 0 / 300 | ok | 0x1409272bff2b1bc4 | 9009 | 3003 | 0/3003 | True | False | 9.32405e-09 | 15871.25 |
| S3-t16 | ode | 16 | 0 / 300 | ok | 0x1409272bff2b1bc4 | 9009 | 3003 | 0/3003 | True | False | 9.32405e-09 | 15872.49 |
| S4-t1 | dart | 1 | 0 / 300 | ok | 0xc88ddc668bf1c585 | 1500 | 900 | 600/900 | True | False | 0.00159871 | 15.663333333333334 |
| S4-t4 | dart | 4 | 0 / 300 | ok | 0xc88ddc668bf1c585 | 1500 | 900 | 600/900 | True | False | 0.00159871 | 21.953333333333333 |
| S4-t16 | dart | 16 | 0 / 300 | ok | 0xc88ddc668bf1c585 | 1500 | 900 | 600/900 | True | False | 0.00159871 | 23.35333333333333 |
| S4-t1 | fcl | 1 | 0 / 300 | ok | 0x71113f65d68d16da | 1800 | 900 | 450/900 | True | False | 0.000976287 | 6004.596666666666 |
| S4-t4 | fcl | 4 | 0 / 300 | ok | 0x71113f65d68d16da | 1800 | 900 | 450/900 | True | False | 0.000976287 | 6004.886666666666 |
| S4-t16 | fcl | 16 | 0 / 300 | ok | 0x71113f65d68d16da | 1800 | 900 | 450/900 | True | False | 0.000976287 | 6006.126666666667 |
| S4-t1 | bullet | 1 | 0 / 300 | ok | 0x941e59b136d56511 | 2617 | 899 | 0/900 | True | False | 0.011297 | 15.736666666666666 |
| S4-t4 | bullet | 4 | 0 / 300 | ok | 0x941e59b136d56511 | 2617 | 899 | 0/900 | True | False | 0.011297 | 16.04 |
| S4-t16 | bullet | 16 | 0 / 300 | ok | 0x941e59b136d56511 | 2617 | 899 | 0/900 | True | False | 0.011297 | 17.2 |
| S4-t1 | ode | 1 | 0 / 300 | ok | 0x3d9054bf94b309 | 2696 | 899 | 600/900 | True | False | 0.000477682 | 3730.4733333333334 |
| S4-t4 | ode | 4 | 0 / 300 | ok | 0x3d9054bf94b309 | 2696 | 899 | 600/900 | True | False | 0.000477682 | 3730.62 |
| S4-t16 | ode | 16 | 0 / 300 | ok | 0x3d9054bf94b309 | 2696 | 899 | 600/900 | True | False | 0.000477682 | 3731.3 |
| S5 | dart | 1 | 0 / 300 | ok | 0x307783ef2271c56a | 150 | 90 | 60/90 | True | False | 0.00135091 | 2.026666666666667 |
| S5 | fcl | 1 | 0 / 300 | ok | 0xd7cbdc24aff5c247 | 180 | 90 | 44/90 | True | False | 0.000758781 | 594.9133333333333 |
| S5 | bullet | 1 | 0 / 300 | ok | 0xf84350bca10909b2 | 268 | 90 | 1/90 | True | False | 2.09436e-05 | 2.0433333333333334 |
| S5 | ode | 1 | 0 / 300 | ok | 0x7410a7102191606c | 270 | 90 | 60/90 | True | False | 8.24071e-05 | 397.22333333333336 |
| S6 | dart | 1 | 0 / 20000 | ok | 0x38aa2d7b26c4bba8 | 155 | 119 | 0/71 | True | False | 0.00363641 | 0.02805 |

`S6/dart` checkpoints: `[{"max_penetration": 0.00397687, "resting": 0, "step": 5000}, {"max_penetration": 0.00398381, "resting": 0, "step": 10000}, {"max_penetration": 0.00385061, "resting": 0, "step": 15000}, {"max_penetration": 0.00363641, "resting": 0, "step": 20000}]`

