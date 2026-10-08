# DART main canonical behaviour guards

Generated at 2026-10-08T14:49:38.706869+00:00 for `044ec3378cbc09f28808dca2fff40d2b953c2892`.
Environment fingerprint: `07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0`.

Commands and scene definitions: [baseline evidence](https://github.com/dartsim/dart/blob/main/docs/dev_tasks/dart6_performance_generalization/01-baseline-evidence.md).
This table is generated evidence, not a fixed reference. Wall time is advisory.
S3 and S6 drift belongs to the #3056 / D7 owners.

| Row | Detector | Threads | Warm-up / steps | Status | Hash | Contacts | Pairs | Resting | Finite | Cap hit | Max penetration | Allocs / step |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S1-60-t1 | dart | 1 | 0 / 200 | ok | 0x5fd4858cc9f34efb | 80 | 72 | 0/60 | True | False | 0.182088 | 1.485 |
| S1-60-t16 | dart | 16 | 0 / 200 | ok | 0x5fd4858cc9f34efb | 80 | 72 | 0/60 | True | False | 0.182088 | 10.785 |
| S1-60-t1 | ode | 1 | 0 / 200 | ok | 0xf9ac6baa42cedb3e | 142 | 75 | 0/60 | True | False | 0.192214 | 342.905 |
| S1-60-t16 | ode | 16 | 0 / 200 | ok | 0xf9ac6baa42cedb3e | 142 | 75 | 0/60 | True | False | 0.192214 | 342.905 |
| S1-120-t1 | dart | 1 | 0 / 200 | ok | 0x341871ecb68bbe26 | 251 | 177 | 0/120 | True | False | 0.312885 | 2.125 |
| S1-120-t16 | dart | 16 | 0 / 200 | ok | 0x341871ecb68bbe26 | 251 | 177 | 0/120 | True | False | 0.312885 | 11.52 |
| S1-120-t1 | ode | 1 | 0 / 200 | ok | 0x8f8d77d6d8ae91df | 274 | 182 | 0/120 | True | False | 0.302429 | 509.81 |
| S1-120-t16 | ode | 16 | 0 / 200 | ok | 0x8f8d77d6d8ae91df | 274 | 182 | 0/120 | True | False | 0.302429 | 509.81 |
| S2 | dart | 1 | 0 / 3000 | ok | 0x9308c8e0b3367e8f | 0 | 0 | 3003/3003 | True | False | 0.0 | 3.069 |
| S2 | fcl | 1 | 0 / 3000 | ok | 0x9308c8e0b3367e8f | 0 | 0 | 3003/3003 | True | False | 0.0 | 9.059333333333333 |
| S2 | bullet | 1 | 0 / 3000 | ok | 0xe5f943c127942b5b | 0 | 0 | 3003/3003 | True | False | 0.0 | 3.7416666666666667 |
| S2 | ode | 1 | 0 / 3000 | ok | 0x1b08c3c93face3fa | 0 | 0 | 3003/3003 | True | False | 0.0 | 28.980666666666668 |
| S3-t1 | dart | 1 | 0 / 300 | ok | 0xe86faf4474573fd1 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 30.69 |
| S3-t4 | dart | 4 | 0 / 300 | ok | 0xe86faf4474573fd1 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 36.82 |
| S3-t16 | dart | 16 | 0 / 300 | ok | 0xe86faf4474573fd1 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 37.5 |
| S3-t1 | fcl | 1 | 0 / 300 | ok | 0xe7ff6160a083e067 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 3064.5533333333333 |
| S3-t4 | fcl | 4 | 0 / 300 | ok | 0xe7ff6160a083e067 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 3064.7033333333334 |
| S3-t16 | fcl | 16 | 0 / 300 | ok | 0xe7ff6160a083e067 | 3003 | 3003 | 0/3003 | True | False | 9.32405e-09 | 3065.383333333333 |
| S3-t1 | bullet | 1 | 0 / 300 | ok | 0x236a7ec3e4ff4149 | 5005 | 3003 | 0/3003 | True | False | 2.68221e-07 | 37.32666666666667 |
| S3-t4 | bullet | 4 | 0 / 300 | ok | 0x236a7ec3e4ff4149 | 5005 | 3003 | 0/3003 | True | False | 2.68221e-07 | 37.596666666666664 |
| S3-t16 | bullet | 16 | 0 / 300 | ok | 0x236a7ec3e4ff4149 | 5005 | 3003 | 0/3003 | True | False | 2.68221e-07 | 38.75666666666667 |
| S3-t1 | ode | 1 | 0 / 300 | ok | 0x1409272bff2b1bc4 | 9009 | 3003 | 0/3003 | True | False | 9.32405e-09 | 15870.976666666667 |
| S3-t4 | ode | 4 | 0 / 300 | ok | 0x1409272bff2b1bc4 | 9009 | 3003 | 0/3003 | True | False | 9.32405e-09 | 15871.263333333334 |
| S3-t16 | ode | 16 | 0 / 300 | ok | 0x1409272bff2b1bc4 | 9009 | 3003 | 0/3003 | True | False | 9.32405e-09 | 15872.503333333334 |
| S4-t1 | dart | 1 | 0 / 300 | ok | 0xc88ddc668bf1c585 | 1500 | 900 | 600/900 | True | False | 0.00159871 | 15.676666666666666 |
| S4-t4 | dart | 4 | 0 / 300 | ok | 0xc88ddc668bf1c585 | 1500 | 900 | 600/900 | True | False | 0.00159871 | 21.966666666666665 |
| S4-t16 | dart | 16 | 0 / 300 | ok | 0xc88ddc668bf1c585 | 1500 | 900 | 600/900 | True | False | 0.00159871 | 23.366666666666667 |
| S4-t1 | fcl | 1 | 0 / 300 | ok | 0x71113f65d68d16da | 1800 | 900 | 450/900 | True | False | 0.000976287 | 6004.61 |
| S4-t4 | fcl | 4 | 0 / 300 | ok | 0x71113f65d68d16da | 1800 | 900 | 450/900 | True | False | 0.000976287 | 6004.9 |
| S4-t16 | fcl | 16 | 0 / 300 | ok | 0x71113f65d68d16da | 1800 | 900 | 450/900 | True | False | 0.000976287 | 6006.14 |
| S4-t1 | bullet | 1 | 0 / 300 | ok | 0x941e59b136d56511 | 2617 | 899 | 0/900 | True | False | 0.011297 | 15.75 |
| S4-t4 | bullet | 4 | 0 / 300 | ok | 0x941e59b136d56511 | 2617 | 899 | 0/900 | True | False | 0.011297 | 16.053333333333335 |
| S4-t16 | bullet | 16 | 0 / 300 | ok | 0x941e59b136d56511 | 2617 | 899 | 0/900 | True | False | 0.011297 | 17.213333333333335 |
| S4-t1 | ode | 1 | 0 / 300 | ok | 0x3d9054bf94b309 | 2696 | 899 | 600/900 | True | False | 0.000477682 | 3730.4866666666667 |
| S4-t4 | ode | 4 | 0 / 300 | ok | 0x3d9054bf94b309 | 2696 | 899 | 600/900 | True | False | 0.000477682 | 3730.633333333333 |
| S4-t16 | ode | 16 | 0 / 300 | ok | 0x3d9054bf94b309 | 2696 | 899 | 600/900 | True | False | 0.000477682 | 3731.3133333333335 |
| S5 | dart | 1 | 0 / 300 | ok | 0x307783ef2271c56a | 150 | 90 | 60/90 | True | False | 0.00135091 | 2.04 |
| S5 | fcl | 1 | 0 / 300 | ok | 0xd7cbdc24aff5c247 | 180 | 90 | 44/90 | True | False | 0.000758781 | 594.9266666666666 |
| S5 | bullet | 1 | 0 / 300 | ok | 0xf84350bca10909b2 | 268 | 90 | 1/90 | True | False | 2.09436e-05 | 2.0566666666666666 |
| S5 | ode | 1 | 0 / 300 | ok | 0x7410a7102191606c | 270 | 90 | 60/90 | True | False | 8.24071e-05 | 397.2366666666667 |
| S6 | dart | 1 | 0 / 20000 | ok | 0x38aa2d7b26c4bba8 | 155 | 119 | 0/71 | True | False | 0.00363641 | 0.02825 |

`S6/dart` checkpoints: `[{"max_penetration": 0.00397687, "resting": 0, "step": 5000}, {"max_penetration": 0.00398381, "resting": 0, "step": 10000}, {"max_penetration": 0.00385061, "resting": 0, "step": 15000}, {"max_penetration": 0.00363641, "resting": 0, "step": 20000}]`

