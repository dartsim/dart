window.BENCHMARK_DATA = {
  "lastUpdate": 1780690794185,
  "repoUrl": "https://github.com/dartsim/dart",
  "entries": {
    "DART Performance": [
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e7acde1afb576ca04289ca0e02f44e9c74a49ed1",
          "message": "Fix performance dashboard: configure build before benchmarks (#2696)",
          "timestamp": "2026-05-23T10:54:49-07:00",
          "tree_id": "cad01130c2493f3dd878661ab4006928d69dea6f",
          "url": "https://github.com/dartsim/dart/commit/e7acde1afb576ca04289ca0e02f44e9c74a49ed1"
        },
        "date": 1779561257023,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_LCP_COMPARE_SMOKE",
            "value": 2221.2415022762093,
            "unit": "ns/iter",
            "extra": "iterations: 1\ncpu: 2221.124014464211 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 83748.95672500762,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 83745.26408535124 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1828953.9596354486,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1828840.6927083333 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 23103797.967214406,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23103574.868852448 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 164244.1409903716,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 119830.67598005495 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1457747.4481842667,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1248035.7564216137 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 13096167.02631625,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 12285878.956140349 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_KR5_WorldStep/min_time:0.100",
            "value": 35645.04107338097,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 35645.473986856516 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_Atlas_WorldStep/min_time:0.100",
            "value": 135563.9542394847,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 135543.3283983847 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32_Baseline/1024/repeats:5",
            "value": 46.37261151978983,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 46.37103808032624 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32/1024/repeats:5",
            "value": 42.131653871690595,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 42.129297758457014 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 449456.8413858993,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 449433.59853782575 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5827252.96250004,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5826410.708333333 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 75984012.83333562,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 75979556.27777776 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 513435.61665518844,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 485637.5791465933 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4146594.529069669,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4062049.0988372075 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48683276.51723999,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48425362.79310346 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7babc5769c54cfc6439f04ede65b802cf6715af5",
          "message": "Add Filament renderer profiling, backend selection, scene-sync cache (#2694)",
          "timestamp": "2026-05-23T18:27:37-07:00",
          "tree_id": "30cd675630ba47d0760de659acc59c9e227afbbc",
          "url": "https://github.com/dartsim/dart/commit/7babc5769c54cfc6439f04ede65b802cf6715af5"
        },
        "date": 1779589293897,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_LCP_COMPARE_SMOKE",
            "value": 2216.2949464270696,
            "unit": "ns/iter",
            "extra": "iterations: 1\ncpu: 2216.229108528124 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 83659.44350110136,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 83655.2867700953 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1830573.6367187167,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1830422.7539062502 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 23120348.04917969,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23119096.00000002 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 163860.77771026813,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 120352.76390938263 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1473133.734109172,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1257539.83169203 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 13116178.333333256,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 12299916.894736856 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_KR5_WorldStep/min_time:0.100",
            "value": 35832.85706535806,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 35830.059940330895 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_Atlas_WorldStep/min_time:0.100",
            "value": 131968.29795521064,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 131959.9259980526 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32_Baseline/1024/repeats:5",
            "value": 46.36326643337647,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 46.355770073704754 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32/1024/repeats:5",
            "value": 42.09658202349575,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 42.09353764970119 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 448165.2152181042,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 448133.994587711 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5796062.152263709,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5795608.049382716 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 75810648.27777733,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 75804656.44444443 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 514042.0864807393,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 486552.14719378785 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4166208.0555557776,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4074370.6023391825 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48805791.551726855,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48533598.965517215 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02",
            "email": "jslee02@users.noreply.github.com"
          },
          "committer": {
            "name": "GitHub",
            "username": "web-flow",
            "email": "noreply@github.com"
          },
          "id": "6fe445c0213e84f1d931e41102b7fe6a1736df8d",
          "message": "Renderer docs: software-render gotcha and interpolation caveat (#2706)",
          "timestamp": "2026-05-24T01:44:10Z",
          "url": "https://github.com/dartsim/dart/commit/6fe445c0213e84f1d931e41102b7fe6a1736df8d"
        },
        "date": 1779608931129,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_LCP_COMPARE_SMOKE",
            "value": 2231.0411017112733,
            "unit": "ns/iter",
            "extra": "iterations: 1\ncpu: 2230.8941325876135 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 83102.83434995354,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 83096.02816648725 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1822875.1805194607,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1822743.8727272728 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 23103309.533333536,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23101865.98333335 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 167167.84324834123,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 122922.19881534901 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1460737.2675046306,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1246558.8186714523 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 13116595.01754404,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 12310587.385964928 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_KR5_WorldStep/min_time:0.100",
            "value": 35625.186949318704,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 35624.79983480177 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_Atlas_WorldStep/min_time:0.100",
            "value": 131215.21603133742,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 131212.70087976524 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32_Baseline/1024/repeats:5",
            "value": 49.58886173405769,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 49.58485047233705 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32/1024/repeats:5",
            "value": 45.892182040391546,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 45.89047341619594 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 449286.6941591703,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 449262.45731707313 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5838477.68200818,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5837986.949790792 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 75861836.77777727,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 75855770.61111109 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 512802.14708955673,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 484397.39072847605 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4147016.208823703,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4067677.6470588227 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48713851.965517126,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48452473.7931034 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0c0d63c3d5ca11c5b7adfa1733cd0d1c82eee91d",
          "message": "Add typed Lie group API (SO3/SE3) with batch ops and dartpy bindings (#2697)",
          "timestamp": "2026-05-24T07:20:44-07:00",
          "tree_id": "ffe82dcd6881ce1bbec2e1128c54f406cb3de717",
          "url": "https://github.com/dartsim/dart/commit/0c0d63c3d5ca11c5b7adfa1733cd0d1c82eee91d"
        },
        "date": 1779634030926,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_LCP_COMPARE_SMOKE",
            "value": 2387.0186944372203,
            "unit": "ns/iter",
            "extra": "iterations: 1\ncpu: 2386.78179836456 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 89380.11810720491,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 89370.131572198 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 2038141.1807579354,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2037896.4402332364 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 26031474.314815264,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 26027323.518518526 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 163473.23928662407,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 117179.81185022382 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1556280.0923077064,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1347616.5653846154 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14460097.203882955,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13650393.262135908 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_KR5_WorldStep/min_time:0.100",
            "value": 38278.879018145795,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 38278.50905902982 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_Atlas_WorldStep/min_time:0.100",
            "value": 141033.8008342902,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 141030.7184567258 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32_Baseline/1024/repeats:5",
            "value": 50.37563151212641,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 50.36878120198881 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32/1024/repeats:5",
            "value": 50.71774244650031,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 50.712328039052025 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 480738.31463745603,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 480633.7510259918 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 6274770.923766683,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6273792.529147986 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 82923119.41176303,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 82916441.11764704 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 528055.0081879493,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 498116.1484514069 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4419228.518404704,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4316694.184049082 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 51636356.74074174,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 51320497.18518522 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3982b72e8fa52c7b4e1791c23765abbc4cf67fae",
          "message": "Add native continuous collision detection (CCD) (#2700)",
          "timestamp": "2026-05-24T07:43:04-07:00",
          "tree_id": "5c568912c47cc77ed55d7d99b3445760739776bd",
          "url": "https://github.com/dartsim/dart/commit/3982b72e8fa52c7b4e1791c23765abbc4cf67fae"
        },
        "date": 1779635462831,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_LCP_COMPARE_SMOKE",
            "value": 2358.466328340242,
            "unit": "ns/iter",
            "extra": "iterations: 1\ncpu: 2358.286895669927 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 110433.94874642884,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 110426.89503332275 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1558180.0188678512,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1557998.9822419528 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 15429915.945055706,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15427630.351648333 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 211495.71039222638,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 165707.08662163749 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1852768.221831059,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1635116.8157276988 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 15486406.041665882,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14629517.020833299 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_KR5_WorldStep/min_time:0.100",
            "value": 37280.15925396804,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 37270.988809182214 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_Atlas_WorldStep/min_time:0.100",
            "value": 138737.37422038362,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 138704.4667359666 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32_Baseline/1024/repeats:5",
            "value": 49.041358120511404,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 49.03822682321411 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32/1024/repeats:5",
            "value": 49.116768854792774,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 49.11330189995562 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 491385.2008441998,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 491284.21948645794 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4948938.003521184,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4948649.908450705 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 59628827.26086975,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 59624428.99999996 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 632627.8229166821,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 602202.1367187505 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4764050.026666761,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4663258.263333331 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 53487632.34615297,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53183506.88461547 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7e885b547cc4bebebdfc66401f7cf4b20825888d",
          "message": "Add standalone dartsim GUI simulator with ImGui docking (#2701)",
          "timestamp": "2026-05-25T08:16:48-07:00",
          "tree_id": "bc64f084bee6fe869093a356bb8c83c8f209da33",
          "url": "https://github.com/dartsim/dart/commit/7e885b547cc4bebebdfc66401f7cf4b20825888d"
        },
        "date": 1779724976132,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_LCP_COMPARE_SMOKE",
            "value": 2235.1746505845695,
            "unit": "ns/iter",
            "extra": "iterations: 1\ncpu: 2235.0260703739173 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 105284.72601562201,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 105277.21093749999 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1444117.9104016975,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1444071.113285273 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 13813791.287128707,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13812730.178217834 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 218654.2326945644,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 180117.53200365 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1703492.3496280415,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1486952.7141339008 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 13964855.839622848,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13170494.160377359 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_KR5_WorldStep/min_time:0.100",
            "value": 35932.69050218337,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 35933.17221615721 ns\nthreads: 1"
          },
          {
            "name": "BM_Robot_Atlas_WorldStep/min_time:0.100",
            "value": 135925.6008024971,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 135917.27482447348 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32_Baseline/1024/repeats:5",
            "value": 46.39784839503223,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 46.391054433308604 ns\nthreads: 1"
          },
          {
            "name": "BM_Add_DART_f32/1024/repeats:5",
            "value": 42.15826734873071,
            "unit": "ns/iter",
            "extra": "iterations: 5\ncpu: 42.11011507839168 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 471865.7205239865,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 471844.2976150487 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4691615.043624408,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4691478.2114093965 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 55200614.11999905,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55197173.48 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 641381.0254720902,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 610275.9920948616 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4612188.585760708,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4513580.851132676 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 50502324.035712875,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50229898.03571421 ns\nthreads: 1"
          }
        ]
      }
    ],
    "DART Experimental World Performance": [
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "003c2d0e39dcf72b4603cc254d1e7a2d37d79a22",
          "message": "Experimental World: rigid-body dynamics MVP (all joint types, contacts, floating base) (#2705)",
          "timestamp": "2026-05-25T08:57:02-07:00",
          "tree_id": "c309ba2ad4718e89dc78092248922f7734cae592",
          "url": "https://github.com/dartsim/dart/commit/003c2d0e39dcf72b4603cc254d1e7a2d37d79a22"
        },
        "date": 1779727069031,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 479964.58898015245,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 479853.6505817934 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4907617.719298511,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4906955.333333329 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 60263272.86956925,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60255257.652173944 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 484182.80612948636,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 484099.5561294761 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4927897.325088394,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4927051.837455829 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 60063973.26086699,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60051661.608695574 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 638565.4228595957,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 608255.0830073884 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4732551.893333342,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4635526.906666669 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 54633556.19230452,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54335685.92307672 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 116070.62731787958,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 116055.75190397348 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1567320.683856433,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1567074.1535874445 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 15424220.222222276,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15422659.588888856 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 217507.19603048335,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 203109.70990939194 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1874464.789072181,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1785646.709021598 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 15857951.177779088,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15531830.422222173 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a4845e034ee331f5f8008bfe258b01ed7ea8bed0",
          "message": "Add Lie group product API and robot coverage (#2708)",
          "timestamp": "2026-05-25T11:54:34-07:00",
          "tree_id": "6212da6bcd3725f06944e2ea0e06c173f20fe4ca",
          "url": "https://github.com/dartsim/dart/commit/a4845e034ee331f5f8008bfe258b01ed7ea8bed0"
        },
        "date": 1779743496130,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 478150.6136986511,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 478057.1910958905 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4946467.9542248165,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4945773.76408451 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 60305343.043479376,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60297330.47826082 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 481290.1507692658,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 481222.3370940173 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4940633.690140863,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4940123.130281684 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 60365230.739129715,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60354605.82608697 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 635732.9047013725,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 605669.5069885628 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4743127.534883331,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4644480.986710969 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 54380571.076929025,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54079243.49999997 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 115072.81659280515,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 115066.89422287862 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1571331.0269358575,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1571095.5297418663 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 15584163.711110301,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15582853.977777787 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 215906.83530753807,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 202471.8681318676 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1873475.909207018,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1785518.7020460283 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 15962623.528088279,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15604589.640449451 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "23372bf19429f1aecc9f846fbef3e39493bb00d1",
          "message": "Add opt-in CUDA smoke path for experimental simulation (#2710)",
          "timestamp": "2026-05-25T12:20:27-07:00",
          "tree_id": "19a6a74901ba5008abb8c734b650a2ce4ac7ad17",
          "url": "https://github.com/dartsim/dart/commit/23372bf19429f1aecc9f846fbef3e39493bb00d1"
        },
        "date": 1779746404841,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 478655.86421016976,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 478606.09177755023 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4951079.336879595,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4950410.439716317 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 60146797.69565073,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60144187.08695656 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 480386.59931624617,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 480326.986324786 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4924082.989399137,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4923678.583038871 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 60156383.69565394,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60153155.3478261 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 629030.9277003511,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 598649.9503484321 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4826665.172297576,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4725080.912162166 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 54339205.65384544,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54039269.76923073 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 115178.28635842357,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 115170.470978849 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1570788.25842697,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1570626.0584269622 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 15427856.978021735,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15425894.4175824 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 216864.37045552966,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 203049.208363271 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1866457.6645490204,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1778785.9618805621 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 15865451.933332982,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15536164.922222262 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@gmail.com",
            "name": "Jeongseok Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "jslee02@gmail.com",
            "name": "Jeongseok Lee",
            "username": "jslee02"
          },
          "distinct": true,
          "id": "7f2097d821c4e60898abf3490d2ef6fd874c044d",
          "message": "Advance dartsim workbench interactions",
          "timestamp": "2026-05-26T11:49:58-07:00",
          "tree_id": "6c2dc84ca9313cdd7a416f6ca2e2b9a5b01a404c",
          "url": "https://github.com/dartsim/dart/commit/7f2097d821c4e60898abf3490d2ef6fd874c044d"
        },
        "date": 1779822809190,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 470185.699227937,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 470138.1597851627 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4699755.845637425,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4699471.083892616 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 55603702.36000381,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55598602.6 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 469702.6148693838,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 469624.84695244447 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4736052.575757523,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4735498.127946123 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 55543023.760001235,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55535858.679999985 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 651254.6572959202,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 618753.7804551513 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4572199.175718981,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4482560.741214062 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 51040653.92857494,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50739698.21428571 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 113327.72333710168,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 113314.04045962113 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1470635.4511040123,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1470545.7392218693 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 13993038.267325671,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13992023.287128689 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 208808.56615811016,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 195165.59459084613 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1736499.3231848713,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1640763.1604215505 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14566939.515150512,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14191686.141414104 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@gmail.com",
            "name": "Jeongseok Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "jslee02@gmail.com",
            "name": "Jeongseok Lee",
            "username": "jslee02"
          },
          "distinct": true,
          "id": "ac0bf48036170dec83de8d9682b7dc9c0fec5da9",
          "message": "Add dartsim viewport camera controls",
          "timestamp": "2026-05-26T14:38:43-07:00",
          "tree_id": "5e6e4b30f5fb03a096a61cb61323a4f10637d2f4",
          "url": "https://github.com/dartsim/dart/commit/ac0bf48036170dec83de8d9682b7dc9c0fec5da9"
        },
        "date": 1779833151497,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 465368.6438812656,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 465333.28376125376 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4683391.568561886,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4683154.588628761 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 55422488.96000274,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55418351.79999999 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 469131.45050160866,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 469093.3719063544 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4696837.033556611,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4696360.4228188 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 55504310.240003176,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55499495.800000034 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 639679.3017315896,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 608059.9904761909 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4581470.98058294,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4482993.31067962 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 50527314.03571898,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50252163.32142841 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 114543.61021767867,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 114528.9584145551 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1474353.417281334,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1474229.5237091628 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 13979775.01000014,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13978895.790000012 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 209545.22333948035,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 196892.38068262278 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1735801.455082599,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1646628.6382978726 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14465718.75757482,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14111079.383838363 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f6de9e057c5af5f8c50537014b7ea8297494f5ad",
          "message": "Make GUI scaling DPI-aware (#2713)",
          "timestamp": "2026-05-26T18:05:39-07:00",
          "tree_id": "516ae80dd2dd03dc60ad44dc89294bc772f2f32c",
          "url": "https://github.com/dartsim/dart/commit/f6de9e057c5af5f8c50537014b7ea8297494f5ad"
        },
        "date": 1779845921450,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 432900.4556726525,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 432852.6258524489 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4161463.6607143376,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4161053.6815476217 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 49310285.99999974,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49304111.82142854 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 434461.96861404,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 434433.6995027967 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4168658.973214062,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4168447.1011904753 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 49519266.39285731,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49515937.78571426 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 591968.3307483576,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 547631.9154711118 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4178586.4655172117,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4026094.3074712455 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 45230516.812502235,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 44737438.18750009 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 105663.46218804175,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 105652.30935685754 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1388592.8779761533,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1388451.061507941 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 13074264.254717551,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13072820.53773588 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 208445.19912591865,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 182665.95206227864 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1700154.295604404,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1540006.5791208835 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 13875398.451923197,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13248545.70192303 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "86c51dfe8bcda7ab2133849616339cee5244040e",
          "message": "Add experimental deformable body dynamics (#2711)",
          "timestamp": "2026-05-26T18:22:44-07:00",
          "tree_id": "1f70d19daa0a65c77455af9f29508481ee418170",
          "url": "https://github.com/dartsim/dart/commit/86c51dfe8bcda7ab2133849616339cee5244040e"
        },
        "date": 1779849954740,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 459439.8617194919,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 459385.36253677675 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4547671.143322779,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4547436.527687296 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52779170.6666676,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52778811.111111104 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 460168.2149901197,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 460156.445101906 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4553605.924342051,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4553339.417763164 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52912731.629631974,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52910055.74074077 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 630710.4130434546,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 599647.6773231045 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4423239.8215381885,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4330078.63384616 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47870216.93333221,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47600001.633333504 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 114991.12931674026,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 114985.82741284058 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1477762.6642105707,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1477074.131578948 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 13965835.579999747,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13964553.910000034 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 210003.29657268676,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 197034.4467938085 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1729304.1995305044,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1640599.2406103204 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14414368.777778177,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14074339.090909103 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "cebe8cb0d74a60f33c0d4a6b9555e1adb7457c96",
          "message": "Add experimental scalable compute CUDA gates (#2712)",
          "timestamp": "2026-05-26T21:30:46-07:00",
          "tree_id": "53897bde22947a21af822cf85a86b113ab5e5da7",
          "url": "https://github.com/dartsim/dart/commit/cebe8cb0d74a60f33c0d4a6b9555e1adb7457c96"
        },
        "date": 1779858363621,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 478205.49538464076,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 478056.6492307692 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4779581.023972365,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4779093.352739727 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 57146122.08333373,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 57142751.37500002 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 482707.2866438334,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 482657.74897260265 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4810758.412969154,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4810322.566552904 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 56933661.79166522,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 56925233 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 629908.9969995933,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 599905.5773681946 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4638208.78778156,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4537055.62700965 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 52008543.370372236,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 51709251.14814835 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 115704.95006581627,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 115691.2180816058 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1570333.1372328799,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1570159.1642294733 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 15540359.511111192,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15538415.111111052 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 215303.2874032381,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 202061.73019000652 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1860123.6725440703,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1772905.0554156164 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 15990062.077777466,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15630918.100000061 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55505.733491873376,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55499.51736028547 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 219975.8821866043,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 219953.05592208504 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 224154.7591673231,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 224129.93626901504 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77144.31846835364,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11815.72924439903 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257756.69773384786,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14119.40373084609 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 263845.76556169434,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21554.66077348062 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433377.794026122,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433344.1023646511 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 866917.5012376608,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 866788.3168316853 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1736683.2344913278,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1736479.992555826 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 135366.42153000002,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5280.439099999939 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 251659.83812999915,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6325.538600000015 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 488148.02914999967,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8845.55840999994 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 157071626.1250027,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 157053962.6249996 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02",
            "email": "jslee02@users.noreply.github.com"
          },
          "committer": {
            "name": "GitHub",
            "username": "web-flow",
            "email": "noreply@github.com"
          },
          "id": "8333f8131313ec5c1d596f95874cee1f930edbd6",
          "message": "Consolidate research papers catalog (#2717)",
          "timestamp": "2026-05-27T04:34:44Z",
          "url": "https://github.com/dartsim/dart/commit/8333f8131313ec5c1d596f95874cee1f930edbd6"
        },
        "date": 1779869556194,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 453887.32309184177,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 453794.25420439854 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4507118.183870819,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4506709.335483869 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52778777.92592435,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52773306.96296295 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 455271.3212081936,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 455221.424813251 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4510173.883870867,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4509789.6451612925 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52754753.11110834,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52749170.96296295 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 630242.7079228682,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 600721.175588865 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4408934.330246826,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4322957.256172848 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47797629.65517483,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47522472.551724024 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 114536.29923000703,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 114528.75139252981 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1476738.0126315737,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1476654.2189473708 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 13936743.435643205,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13936173.999999978 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 205793.44644795288,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 191838.5006778737 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1744964.2668240385,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1651866.6540732014 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14484770.49494944,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14112476.151515147 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50052.97514483958,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50049.71089335532 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198547.68407606208,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198534.45926767017 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202780.781707483,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202761.9230323229 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 66058.77748533493,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11805.741265417759 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 217430.29419326552,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13323.095768459681 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 223873.4522150196,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 20264.37276855851 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388542.99445215845,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388517.3425797505 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776967.6272221206,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776913.7027777774 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1553457.4750277505,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1553378.466148724 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 120545.3128499994,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 4883.154200000064 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 229179.8853699993,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6143.435900000043 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 430846.680730001,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8260.134459999905 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 134383112.99998733,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 134366381.66666624 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2266269803393d9146eea2e412966826ecc73b0a",
          "message": "Add opt-in pixi abi-check task for libabigail ABI diffs (#2722)",
          "timestamp": "2026-05-28T07:45:41-07:00",
          "tree_id": "0b8204bfa4568d8991bece156cbd9cc1da035041",
          "url": "https://github.com/dartsim/dart/commit/2266269803393d9146eea2e412966826ecc73b0a"
        },
        "date": 1779982284482,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 458057.3019237241,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 458005.8640365177 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4530583.708609335,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4530144.913907284 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52787696.91999969,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52782540.15999999 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 455395.5416803088,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 455359.0869565219 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4518108.879870163,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4517771.444805188 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52927880.576922126,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52923520.80769219 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 592571.2651273818,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 561426.9263535021 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4446845.102803604,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4350111.947040492 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48027849.75861963,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47715216.06896556 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 114610.63066830464,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 114601.90036900355 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1479589.7771910718,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1479482.4107708577 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 14054487.880000578,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14052915.319999995 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 214524.34890491725,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 196667.25187136114 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1765313.399524302,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1668256.1843044038 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14506217.09090908,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14147572.96969703 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50130.30917424562,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50127.168624220874 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198590.352682372,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198576.78768095226 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 203309.0046350044,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 203295.8166280403 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 67548.53836834799,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12678.669842231357 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218890.6120590948,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15760.250993511198 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 225160.7397229029,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23093.671295843553 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388723.3112160429,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388693.48001110693 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 777072.7081022103,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 777003.3895671443 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1560064.1336302797,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1559921.8385300692 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 124483.31714000006,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5096.607459999945 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 231029.66453999898,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6631.383339999957 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 431564.7373799993,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8499.885869999986 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 140112796.77778372,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 140092157.44444406 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f96e0fa9ab816aa9851573239d02445804aca8be",
          "message": "Add IPC deformable scene replay scaffolding (#2719)",
          "timestamp": "2026-05-28T08:15:28-07:00",
          "tree_id": "5fd7c13e91bd83d74da94afc7546ba1d2b0fec58",
          "url": "https://github.com/dartsim/dart/commit/f96e0fa9ab816aa9851573239d02445804aca8be"
        },
        "date": 1779986540686,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 362345.49466562737,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 362306.970856102 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 3599673.147286933,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3599256.0542635676 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 43327592.7187502,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 43323827.50000002 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 362855.24819025677,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 362812.9097724923 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 3624508.057142615,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3624143.8467532475 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 43127656.59375017,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 43122777.62500005 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 452123.82193863863,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 427625.11759343656 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 3496552.1299020164,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3414452.2843137286 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 38678344.69444472,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 38432270.944444604 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 84245.12863245422,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 84231.06359424836 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 943364.041527165,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 943285.8352310756 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 7490246.11764679,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7489323.556149713 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 163408.29301784784,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 148418.3935776909 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1175071.4091627093,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1102893.5687203773 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 7829376.448648873,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7559891.470270327 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 43098.85266785678,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 43093.11899193776 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 170713.90962312126,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 170698.1959995119 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 174130.02943733006,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 174111.59408769128 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 60067.431559999706,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 9154.364560000091 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 197028.52280845412,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 10726.637665991668 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 202967.62272193388,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 16563.976432597614 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 336565.69133092096,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 336533.15360195446 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 672728.9595571157,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 672665.1198844442 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1346133.862367709,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1346011.7921078 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 105614.40137000091,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 4178.880059999983 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 197566.8847199995,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5128.057670000033 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 379595.8080200012,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 7053.569090000025 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 117106698.4000163,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 117085095.80000027 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d9276efa6db1f74edefe3fd1f7b0799e719aa3ee",
          "message": "Add IPC barrier kernel scaffolding (#2721)",
          "timestamp": "2026-05-28T09:27:54-07:00",
          "tree_id": "daefb37762af0cdf6750baaf5aa3daaf159d3159",
          "url": "https://github.com/dartsim/dart/commit/d9276efa6db1f74edefe3fd1f7b0799e719aa3ee"
        },
        "date": 1779989571117,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 449329.95569822605,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 449303.13322632416 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4449411.6190478625,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4448860.42222222 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52596058.074072964,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52592333.25925925 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 452066.37005278934,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 452032.6850263847 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4480098.012944892,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4479861.294498387 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52380682.5185181,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52376784.25925932 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 592547.878751048,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 560805.345904298 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4387524.935185077,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4291988.404320983 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47656310.466667645,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47366933.13333333 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111072.96185200698,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111064.4588049071 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1164584.9724309018,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1164536.2230576465 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8805632.503143834,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8804814.974842789 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 204262.2696020189,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 186747.90850191723 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1439000.9194312582,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1341406.1127962058 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9306012.282051003,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8970934.90384612 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50091.788609458396,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50090.22909884417 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198500.19719307453,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198489.5740005671 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 203655.41448517866,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 203643.18673647512 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 67346.43885383464,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12576.003259672025 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218871.15572136547,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15762.458882497003 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 225042.71058726145,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23024.953449131386 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388518.2669997275,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388497.8712184287 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776897.1453940375,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776870.4966703588 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1556024.7977777587,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1555933.0322222328 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 120823.29088000052,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 4992.412620000124 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 229273.7330199998,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6397.758509999961 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 431207.49532999983,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8354.51316000004 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 136462198.66667604,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 136448886.22222278 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "92a0ff8b92a3c9bd01a798739223e6e5eda399b3",
          "message": "Add IPC tangent stencil scaffolding (#2724)",
          "timestamp": "2026-05-28T11:43:31-07:00",
          "tree_id": "acc833900f178d33ee18efbd4348146f0bbf6f31",
          "url": "https://github.com/dartsim/dart/commit/92a0ff8b92a3c9bd01a798739223e6e5eda399b3"
        },
        "date": 1779996654233,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 420513.2148636484,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 420472.79112975736 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4036096.4641833566,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4035672.507163326 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 46742594.23333448,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 46739953.69999998 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 421878.8514044232,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 421854.9317426761 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4013435.8514287025,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4013178.6571428515 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 46778920.76666316,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 46774356.60000005 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 567430.2144732036,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 526731.3547056622 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 3964391.620879074,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3837116.2252747137 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 42319046.121213324,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 41868823.03030292 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 101403.10496638139,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 101395.25641581712 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1092500.2255286565,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1092441.597494124 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8334739.458333506,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8334049.940476176 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 202301.61961899555,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 176344.57459178555 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1403364.3057040998,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1250784.18983957 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9133536.335366165,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8557810.408536598 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 61205.43916732855,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 61203.61825417638 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 243487.65895652116,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 243468.83286956523 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 246589.64197533834,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 246573.51022927577 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77628.30806589422,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 10025.666961225956 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 261252.14762125895,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 10689.063217893106 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 270271.24795905873,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 19627.14816348891 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 452800.95077718113,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 452766.7244170953 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 902958.8212903152,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 902905.8406451619 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1805810.5128866911,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1805688.136597923 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 184385.27718999924,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6237.162439999935 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 361052.4614999986,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9448.935279999943 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 696249.5112000168,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 11727.658800000994 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 127902623.69999254,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 127893758.70000015 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "bff7ae75554cf514986aa03fd4c9712f0c42cf07",
          "message": "Add IPC motion-aware candidate culling (#2725)",
          "timestamp": "2026-05-28T12:02:10-07:00",
          "tree_id": "c624655150b83796b951abfeab9a796ee61ecb90",
          "url": "https://github.com/dartsim/dart/commit/bff7ae75554cf514986aa03fd4c9712f0c42cf07"
        },
        "date": 1779998815802,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 450313.5258261173,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 450222.6406801411 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4451921.450793865,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4451619.330158732 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52316689.518517226,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52313174.81481477 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 450269.9627249258,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 450232.1908740368 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4441251.739682692,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4440815.453968252 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52457689.74073902,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52452796.18518529 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 589312.573011472,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 559605.1978630781 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4396084.690183894,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4299594.141104284 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47502628.50000126,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47201074.73333334 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 108679.50898064737,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 108674.23061970291 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1154218.0977011828,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1154100.294745484 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8745322.23125044,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8744495.506250028 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 201180.02247337298,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 183612.37529228337 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1412935.7648724692,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1323481.9981114331 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9235929.961538987,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8900831.98717953 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50067.17331902746,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50063.8201359084 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198489.2993052669,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198472.214943995 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202722.55385059756,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202708.37463810106 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 66130.51843855747,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11973.15631088411 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 217890.8829444887,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13708.231609206598 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224089.23261518395,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 20715.295534474124 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388369.7854565982,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388347.0732722701 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776651.112035514,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776601.9866888463 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1554609.6725861272,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1554517.338512755 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 123567.15966000139,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 4985.15304999998 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 224029.92367000025,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6251.1725199999555 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 440210.4665699994,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8575.676840000027 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 135798574.11112547,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 135785393.88888878 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1d91242e27cd4bfaf0bf5eb843cc76bd5a02c725",
          "message": "Reuse IPC contact candidate buffers (#2726)",
          "timestamp": "2026-05-28T14:15:36-07:00",
          "tree_id": "609b5fb7502d2122fe2850aca1771f35f5649a08",
          "url": "https://github.com/dartsim/dart/commit/1d91242e27cd4bfaf0bf5eb843cc76bd5a02c725"
        },
        "date": 1780004553504,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 449784.5842986346,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 449749.3452380953 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4453390.8222222505,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4451470.060317461 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52750600.923078455,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52744489.46153847 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 450241.1734234057,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 450206.9214929217 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4473458.945686926,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4473174.750798729 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52622506.884622805,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52620067.3846154 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 591523.5239629162,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 562110.3922674194 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4380191.975231963,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4284137.275541808 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48011845.20689777,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47697122.000000216 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 110032.03954052112,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 110013.85965460642 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1156778.4021380963,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1156576.445723686 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8855777.075949317,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8855117.53797468 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 201216.98002872337,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 182966.6283774966 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1412566.238410792,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1323580.4361400267 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9546787.487013942,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9167857.987013018 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50034.347491866174,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50030.098930959015 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198476.49851128526,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198457.16262583225 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202638.17227923757,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202619.37187454788 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 65776.33640013765,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11828.566171194252 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 217601.57556341545,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13482.31032316669 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 223703.77095108223,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 20293.84761550468 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388377.07021924696,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388336.66583402496 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776569.433166905,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776515.4242928498 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1556068.3055554768,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1555855.4844444457 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 121028.92127000132,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 4970.599589999978 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 224215.01834000085,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6128.942859999995 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 431109.32801999984,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8325.252199999937 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 139718116.4444545,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 139708840.8888894 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1c516c2154127cac026fe555a60249900964bc12",
          "message": "Add IPC inter-body surface CCD limiter (#2729)",
          "timestamp": "2026-05-28T14:41:04-07:00",
          "tree_id": "95969904c5df4560a0087a0baa8ece2ef28e7cd8",
          "url": "https://github.com/dartsim/dart/commit/1c516c2154127cac026fe555a60249900964bc12"
        },
        "date": 1780006260742,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 459546.36073659366,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 459412.59914501803 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4541630.351791399,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4540887.169381109 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 53342268.807691306,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53336103.15384618 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 462823.00398801354,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 462783.98238617537 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4537223.042071117,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4536907.417475724 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 53128909.69230466,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53125702.19230764 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 599351.3663165957,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 568069.69184168 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4456310.147335705,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4356100.047021947 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48433130.034482025,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48075540.79310361 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 114352.48655038013,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 114346.1746820999 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1477862.50474164,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1477743.2845100088 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 14212753.639176017,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14211007.402061889 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 212995.01205823632,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 195869.1530145527 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1749877.0094676493,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1650880.7502958605 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14767104.649484552,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14352669.164948499 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50041.430630052666,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50038.41900879619 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198679.6472092246,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198663.38602471285 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202421.86405796572,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202402.60971014525 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 68369.03711255369,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12949.292794477964 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 219326.04847715818,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15970.907371942718 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 225491.40555318605,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23100.044464877956 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388485.8001110347,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388451.7232093257 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 777075.0421753833,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 777000.1132075489 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1555396.2166664961,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1555234.6988888942 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 121473.51048000019,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5135.589490000001 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 230849.7986800012,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6542.641890000027 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 452124.53170000116,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8887.802600000045 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 147673592.5000128,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 147651608.6250008 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7f88f2a29086503a549f70719dfd11deae2af518",
          "message": "Add static rigid surface CCD limiter (#2732)",
          "timestamp": "2026-05-28T14:52:36-07:00",
          "tree_id": "edb30e5840ea2391cc69b107f4bfae7cb01252c5",
          "url": "https://github.com/dartsim/dart/commit/7f88f2a29086503a549f70719dfd11deae2af518"
        },
        "date": 1780011570391,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 488272.1886331725,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 488229.57635983266 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5004144.917857697,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5003800.892857141 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 60648199.26086784,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60645198.652173914 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 490597.2115250095,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 490531.36226282525 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5021909.74193549,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5021530.440860213 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 60646606.652169235,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60638604.1304348 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 613031.7621034522,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 582763.9382303852 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4584985.432692963,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4485642.839743589 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 50719683.42856994,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50409937.92857152 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 109365.96977999108,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 109355.97486886421 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1154184.027914661,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1154116.4384236417 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8779260.531250088,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8778482.650000008 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 207255.32048192687,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 189574.2239625168 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1430594.0057034388,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1335672.519961972 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9319198.36538479,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8974062.801282018 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50188.1344983121,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50182.65143593778 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198778.97998294525,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198753.90743895422 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 203552.1452294495,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 203528.76970138474 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 68035.35144842877,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12808.953566263976 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218700.92586920096,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15619.865120178833 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 225193.1103501851,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22703.92222384128 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388418.8789225256,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388362.43571230076 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 778242.9766408178,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 778095.6607341468 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1557735.098998813,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1557599.768631804 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 127292.39092999934,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5203.99490999992 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 232730.4116100004,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6461.006159999983 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 432090.38856999995,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8728.357359999989 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 143185425.2222264,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 143163822.4444435 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ff50f0a771db1ea43b170aa2ec33dc28a6d28352",
          "message": "Add gersemi CMake formatting (lint-cmake) and format CMake files (#2736)",
          "timestamp": "2026-05-28T19:11:57-07:00",
          "tree_id": "67e15ad87be905ea82dc53cc3e09ae22092d39ce",
          "url": "https://github.com/dartsim/dart/commit/ff50f0a771db1ea43b170aa2ec33dc28a6d28352"
        },
        "date": 1780027396007,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 488164.5632424665,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 488136.59364081064 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5006011.4444443,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5005893.870967745 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 60483337.47825698,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60479405.304347835 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 490204.1588523569,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 490184.7620713792 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5012470.446428097,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5012234.35 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 60506425.04348379,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60499883.65217405 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 618355.4619587889,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 588013.7898276594 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4608325.935275084,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4511617.673139174 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 50495967.53571806,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50190599.96428586 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 108990.67112112408,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 108986.80623106181 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1156540.521232357,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1156511.1423813528 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8793357.607595382,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8793144.658227852 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 209356.60555031427,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 191438.47164219295 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1430250.6413663775,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1337420.9193548327 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9496141.55844266,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9116633.37662336 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50175.51708452788,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50173.241117478545 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198583.05204938466,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198567.53510140473 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 203441.3594182049,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 203425.71185454456 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 68406.61631561992,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12954.746117793175 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 220499.96628708218,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15931.120523072832 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 226616.9993575427,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23159.58434654115 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388411.2901165798,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388394.00249861326 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776767.8508042762,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776748.080421525 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1554945.1100001635,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1554861.5299999982 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 123980.3933600001,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5195.454100000064 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 231282.7925900001,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6556.381160000058 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 432194.00941,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8519.742449999938 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 136147433.1111165,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 136134450.33333308 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e842bcbc0759d1c25e42a83edf44dcdbfcad6c89",
          "message": "Add IPC moving rigid surface CCD limiter (#2738)",
          "timestamp": "2026-05-29T07:01:14-07:00",
          "tree_id": "b3bc788ff007e1f8b32b7e2c8c7b9ac2a747ec88",
          "url": "https://github.com/dartsim/dart/commit/e842bcbc0759d1c25e42a83edf44dcdbfcad6c89"
        },
        "date": 1780074339557,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 521095.67755102,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 520877.0233766234 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5368239.88888946,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5367414.302681995 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 65287431.52381095,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 65282230.42857138 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 519971.8403580833,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 519929.31107795605 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5390003.693486735,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5388912.337164749 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 65345000.57142351,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 65338631.52380955 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 621704.0368487763,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 591381.8254976714 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4834801.938983157,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4718565.050847457 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 53429364.500000425,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53096300.92307699 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 108859.0477850095,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 108843.77548017334 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1197336.4456895886,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1197188.2112069016 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9826502.661971522,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9824348.457746442 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 214455.72643774774,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 193054.43295181586 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1517356.3687943455,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1427739.0688956396 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10222373.415493,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9848487.936619716 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55625.950013906004,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55618.77808240959 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 220147.3474296513,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 220121.42540481125 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 225749.7657207024,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 225733.5807803945 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 78000.96683969235,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11986.740081321832 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257419.02112825998,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14007.984970722906 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 265300.42745145987,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21886.597155019186 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433680.6258903688,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433649.2771755937 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 868872.4786112859,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 868770.229386237 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1740625.1387858335,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1740466.2292441167 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 136583.51554000092,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5411.365589999946 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 254137.27092000048,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6704.429299999931 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 488594.8897200001,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9025.582109999988 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 151921259.9999946,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 151897428.25000075 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "21f12fc6aff114d4e322a889a094dbd06edc1fda",
          "message": "Add IPC friction diagnostics to the deformable solver stats (#2754)",
          "timestamp": "2026-05-29T09:56:24-07:00",
          "tree_id": "ecb4b0484b2d63862c02d2abc9f3348e501710de",
          "url": "https://github.com/dartsim/dart/commit/21f12fc6aff114d4e322a889a094dbd06edc1fda"
        },
        "date": 1780086594161,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 460198.20269824617,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 460151.60644948995 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4538384.055016101,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4537839.061488671 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 53212423.34615415,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53205779.6153846 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 463082.85054439725,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 463020.0567469489 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4553383.9385114005,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4552821.909385111 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 53309825.84615204,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53301409.00000001 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 613092.2325487016,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 580804.9042207802 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4475713.812500004,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4373093.540625006 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48537240.72413492,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48200791.37931029 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 113220.0716941512,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 113203.39290333043 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1470564.7970401125,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1470457.0274841462 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 14208172.606059276,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14206065.51515154 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 212996.4489934609,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 194665.97633296993 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1757524.2914690683,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1658488.0829383882 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14670475.775509268,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14281930.499999953 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50069.51741276026,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50064.35018592671 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198496.18556846993,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198473.09441451752 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202901.9852216679,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202875.5324543597 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 68090.04169739939,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12795.987675645463 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218664.00579148202,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15936.297460484544 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 225175.81664354875,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23047.171249081937 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388410.4127115817,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388371.84179850155 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776856.485302228,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776776.5429839155 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1553404.9113082543,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1553258.3702882577 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 124438.9747299988,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5156.595969999955 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 230401.10341999936,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6481.889309999929 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 432120.31776000006,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8463.643040000052 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 145318643.11111853,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 145305616.88888904 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "41898282+github-actions[bot]@users.noreply.github.com",
            "name": "github-actions[bot]",
            "username": "github-actions[bot]"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5088f0cd7426de2f5d2687d73c3953621c849f42",
          "message": "Update pixi lockfile (#2768)\n\nCo-authored-by: jslee02 <4038467+jslee02@users.noreply.github.com>",
          "timestamp": "2026-05-29T12:23:20-07:00",
          "tree_id": "a01b094f24984bea2d41d5a37a4124d8533e0058",
          "url": "https://github.com/dartsim/dart/commit/5088f0cd7426de2f5d2687d73c3953621c849f42"
        },
        "date": 1780100140857,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 456721.69134993287,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 456630.13794233283 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4440855.0126985265,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4440715.561904762 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52418536.666664295,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52391884.703703694 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 459376.7912986526,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 459344.5027805036 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4452613.17142886,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4452301.555555562 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52359876.777784474,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52355992.51851857 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 593130.491136169,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 562441.508460919 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4374447.401840321,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4279315.208588965 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47622266.033332996,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47324122.1333332 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 109170.09070365515,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 109166.13738019152 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1151500.3212818762,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1151432.2670501212 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8761755.20754814,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8761443.22641512 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 207793.72195119667,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 191110.42601625968 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1435152.9730250363,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1342051.4585741844 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9295983.75159333,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8947293.165605081 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50066.96770150374,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50063.97514949678 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198816.78677823572,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198807.8693431693 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202741.95358953616,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202734.17200870134 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 68244.76642994136,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12864.884328214988 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 219296.00187405187,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15966.19113025789 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 225257.86074117452,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22906.402735092935 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388362.96727673314,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388342.3910149767 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 778114.0277932811,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 778071.6798221184 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1553662.1809102278,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1553579.988901224 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 123823.86566000151,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5196.3121 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 230991.97853000078,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6430.391279999982 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 442809.84760999895,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8855.702560000083 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 137487161.00000516,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 137464379.11111146 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0ce98b3e1f830d138d914bb5f0096d04981671a0",
          "message": "Redirect `pixi run ex py-demos` to the Python demos task (#2772)",
          "timestamp": "2026-05-29T15:55:13-07:00",
          "tree_id": "1bb3b4941894bcd91e602e11be5286c281c0674e",
          "url": "https://github.com/dartsim/dart/commit/0ce98b3e1f830d138d914bb5f0096d04981671a0"
        },
        "date": 1780108538512,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 467343.1963746283,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 467279.01107754273 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4567113.525806396,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4566779.954838709 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 53921929.2692311,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53913504.80769233 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 472057.52988583484,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 472033.1484217591 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4548410.840908889,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4547811.386363641 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 54155987.57692281,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54151831.61538455 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 580738.0364500849,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 550511.9231378769 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4403435.333333366,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4302974.18518519 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48701114.34482968,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48406255.58620692 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 110550.77358341737,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 110531.763968792 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1219039.7106195658,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1218872.1230088507 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9592325.821917137,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9591613.054794492 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 212258.03523559085,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 193398.31877850005 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1530145.8551020308,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1441706.8887755123 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10050383.3194444,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9723616.88194439 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55588.74388694803,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55582.57792156239 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 220429.35096834853,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 220405.7121713118 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 224349.52795129784,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 224319.43472689367 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77735.15326254473,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12104.976254975954 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257950.40211769196,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14107.240801146305 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 265181.62409809005,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21793.74381932582 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433507.89473688364,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433452.20990711945 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 867163.5442724352,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 867059.2303405587 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1734197.9999999795,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1734013.4993804202 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 137547.25063000023,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5610.109570000077 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 253800.55918000152,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6785.992160000092 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 487599.52547000017,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9109.38079999994 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 143555805.87499616,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 143542489.25000057 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8c7d8fe584f76caea46989457b7a7b49410761d0",
          "message": "Add Filament offscreen render-to-texture parity gate (#2774)",
          "timestamp": "2026-05-29T18:19:48-07:00",
          "tree_id": "53d63dacd97f87257dce4e0f03d67feabb9ef547",
          "url": "https://github.com/dartsim/dart/commit/8c7d8fe584f76caea46989457b7a7b49410761d0"
        },
        "date": 1780116123331,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 473816.33739012416,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 473773.14841108856 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4543038.918566794,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4542407.198697068 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 54118752.73077053,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54113518.07692305 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 472465.998991269,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 472402.3301950232 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4565298.247557073,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4564621.570032571 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52961212.961537726,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52954124.153846174 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 586896.0736341964,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 556472.9493269993 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4393008.496932465,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4290129.383435567 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48715248.34482772,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48391925.89655161 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 110726.92915335279,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 110717.18905750838 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1219034.7927689261,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1218992.9955908274 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9642353.386207419,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9641322.027586194 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 213114.08420611883,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 193400.64809384095 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1529330.3650465785,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1437072.1365046604 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10145396.818181446,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9795044.496503446 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55555.1065020611,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55549.00797872373 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 220003.25294951556,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 219959.93094226834 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 224182.6667735217,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 224155.9774002241 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77604.15890178012,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12000.049062399703 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257035.2881713385,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14047.20571587739 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 265063.4082884475,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21655.17878450346 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433527.4916409042,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433476.7569659448 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 867192.0359355384,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 867082.3091697666 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1733845.4826731004,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1733670.6584158337 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 137655.65546000062,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5261.078690000005 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 253542.60759,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6613.377720000102 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 490585.1133700003,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9074.559550000033 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 152207316.00001612,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 152183938.12500075 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "910e619cd822de4492f4183644916c53bfa80da4",
          "message": "Add a radial barrier force for static sphere obstacles (#2776)",
          "timestamp": "2026-05-29T21:39:48-07:00",
          "tree_id": "191e860374791762b4f414623d867c68a62bbf3f",
          "url": "https://github.com/dartsim/dart/commit/910e619cd822de4492f4183644916c53bfa80da4"
        },
        "date": 1780121411581,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 468118.00802138407,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 468062.76036096254 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4678973.5418058075,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4678552.859531774 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 55245776.520000614,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55203076.359999985 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 470804.4341085215,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 470766.96461071813 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4693795.862416108,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4693237.372483222 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 55301459.47999699,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55296114.71999999 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 616129.6408010096,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 585746.5949103053 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4602397.637254815,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4509687.500000001 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 50465794.428565204,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50169696.321428545 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 114528.09383159294,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 114521.62173629284 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1484141.391073237,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1484086.6323060582 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 13949232.809998192,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13948307.450000001 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 213738.0425266534,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 196114.80163457536 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1743564.4654161038,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1654959.8933177043 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14432814.646464279,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14102095.91919191 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50023.759021490296,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50020.4190479598 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198442.89922040547,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198426.18497519472 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202622.8006370175,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202595.7382365721 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 67965.93497572976,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12824.678730063533 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218508.21614159443,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15683.178251158817 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224654.64238085214,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22610.839824508697 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388514.6503884511,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388470.74750277353 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776916.443951263,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776863.6842397383 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1556327.4527250726,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1556224.1468298116 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 125088.30123000052,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5262.447019999996 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 224776.46278000064,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6113.959219999998 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 445228.99305000016,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8813.814260000045 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 137409788.77778313,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 137391847.00000033 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "56ca5a027aeb21a5d157181fe835b288c11d7201",
          "message": "Add a linear-time variational integrator to the experimental World (PLAN-082) (#2764)",
          "timestamp": "2026-05-29T22:00:11-07:00",
          "tree_id": "2beb04bc48540247fcd1e66efd6af628338a876a",
          "url": "https://github.com/dartsim/dart/commit/56ca5a027aeb21a5d157181fe835b288c11d7201"
        },
        "date": 1780132787199,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 494905.4260375995,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 494756.3568641362 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5049921.36330925,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5049314.575539566 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 61161101.3913044,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 61156915.30434775 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 500136.81927711907,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 499502.14138908684 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5069304.123188378,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5069004.49637681 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 61011014.608697146,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 61008377.69565221 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 631714.9578392869,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 600687.9411506356 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4652180.0032568965,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4549493.934853419 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 51179132.78571352,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50866252.321428716 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 112444.79241803997,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 112436.04087521024 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1178881.4793944415,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1178758.858704794 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8812374.377357742,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8811575.169811344 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 207815.3331530447,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 190270.15183883175 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1433946.4292542262,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1339963.1959846988 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9340673.846153526,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8982841.807692284 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50122.72515750236,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50118.544100802064 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198442.16600510944,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198431.45392685055 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 203331.9798980422,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 203319.7270211205 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 68142.36584840906,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12880.035282116454 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 219177.59257847964,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 16221.159837437755 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224865.8515960772,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23035.706833765184 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388538.89703024674,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388515.7607549233 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776877.605993328,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776842.5577136548 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1553891.571587263,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1553814.1487236416 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 127199.08068999984,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5181.611479999901 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 232646.04010000083,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6489.7383199999595 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 431971.43912,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8529.631459999933 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 139148233.777784,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 139136782.33333352 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e6ecc730ce5704dc5a77370f845f153ee97a7a95",
          "message": "Link Filament's macOS system frameworks so the dartpy wheel builds (#2785)",
          "timestamp": "2026-05-30T05:09:52-07:00",
          "tree_id": "913f988fd7d4306b283c44e53bcaa0c57da524b3",
          "url": "https://github.com/dartsim/dart/commit/e6ecc730ce5704dc5a77370f845f153ee97a7a95"
        },
        "date": 1780146217882,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 461358.37603168905,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 461325.2994387587 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4637004.28196724,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4636611.400000001 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 55894892.69231343,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55890839.49999996 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 463712.24508164823,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 463668.70390130085 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4647142.973509525,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4646658.0695364205 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 55584650.80000133,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55577401.7600001 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 593123.297721903,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 550499.3067556962 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4423927.401813092,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4288905.703927492 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 46500270.7741948,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 46043626.67741943 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 102802.08130799537,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 102792.29533068211 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1097780.3322833735,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1097652.61653543 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8342341.273808935,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8341440.81547619 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 210061.67217875345,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 183047.22948750373 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1427184.359821508,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1267686.5508928562 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9153803.282208698,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8557134.503067441 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 61310.11521463299,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 61306.92933094178 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 243853.80982237344,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 243840.17328456903 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 247800.77433628868,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 247788.06690265468 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 80237.8246249133,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 10449.022537472854 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 270294.2642614981,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12008.868450035254 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 279191.08857822634,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 20926.08024985132 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 451519.99483866146,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 451489.17548387195 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 902970.7375887568,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 902898.7743391342 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1806656.8709676384,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1806596.454193548 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 207861.2707100001,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 7393.186410000112 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 333323.0158099991,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8554.547979999968 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 692643.6905999935,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 12287.88190000074 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 136948007.1111108,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 136926084.55555603 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a22d1d16126c4b0319c4b1fb496e7a8312511b5e",
          "message": "Add the projected-Newton Hessian for the sphere obstacle barrier (#2790)",
          "timestamp": "2026-05-30T06:00:20-07:00",
          "tree_id": "e421880b8428614bbb809e133b8ffe1a63890579",
          "url": "https://github.com/dartsim/dart/commit/a22d1d16126c4b0319c4b1fb496e7a8312511b5e"
        },
        "date": 1780159789527,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 468709.46518665814,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 468640.8183652877 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4658895.820598158,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4658233.441860464 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 55630676.280002266,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55623993.840000026 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 473402.3648054244,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 473334.3421319796 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4686629.100334254,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4686017.963210703 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 55725686.23999814,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55717920.680000074 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 597780.5992632267,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 566878.6729431024 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4567003.587859369,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4463758.875399373 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 50239540.392855786,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49918932.642857075 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111005.45743420151,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 110993.56288040498 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1220658.4600858248,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1220515.691845498 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9608927.705479402,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9607355.36986298 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 213212.94763536364,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 193962.59602467518 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1548000.0759878366,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1460332.2765957352 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10070892.937499644,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9751749.451388866 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55511.149226495734,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55504.22324474382 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 219918.86380773623,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 219897.38375746328 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 224369.97180842664,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 224344.8532756682 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77710.88122605234,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12097.745611089538 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257965.04366489392,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14142.740218529912 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 264754.4040369548,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21978.848272649688 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433349.41362224496,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433300.2631578956 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 866668.0953560068,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 866569.1479876173 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1735918.6373763047,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1735604.7945544573 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 139081.68280999915,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5730.481260000033 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 256081.92625999893,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6813.805409999958 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 484103.0562299988,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9046.197029999945 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 147776646.24998012,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 147752290.5000015 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "75c18c534f0c221462856d2a89232e6765529357",
          "message": "Add the fixed-corotational FEM material to the deformable solver (#2794)",
          "timestamp": "2026-05-30T08:00:37-07:00",
          "tree_id": "1cff506437012cc8fa1ffbfa3401f18101ff209e",
          "url": "https://github.com/dartsim/dart/commit/75c18c534f0c221462856d2a89232e6765529357"
        },
        "date": 1780168897825,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 500252.89488734404,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 500225.3385770468 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5074318.210145593,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5073929.710144927 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 61004332.130432494,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60998646.73913047 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 500745.4429337836,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 500727.5506261189 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5144741.847272433,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5144166.5672727255 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 61169149.043478124,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 61166604.82608685 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 631926.0577175104,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 602549.7498906857 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4645619.684039545,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4550025.794788268 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 51030347.4642829,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50723222.50000002 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 110879.17111497668,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 110873.96936197123 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1164130.091735598,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1163980.1256198336 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8784833.693749761,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8784366.587499991 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 209231.52642933757,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 191432.68217367865 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1422873.272023199,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1330170.6118102593 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9476105.948718984,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9127686.878205128 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50101.0887723742,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50098.81233050045 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198646.49680985836,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198626.64653339004 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 203584.68546857362,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 203574.8519166302 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 67931.59581330948,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12751.268909426988 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218425.23557883524,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15631.042887768976 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224605.51244416143,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22598.01266751734 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388552.05882348376,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388529.7061598249 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776873.9766925741,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776803.9467258607 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1553995.2064371838,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1553929.5715871186 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 123359.23868999998,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5085.816750000021 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 224459.6743000011,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6365.9756900000275 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 445447.16717000026,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8714.000079999949 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 140033535.88891008,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 140001981.3333336 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "af917bbafc6a50492613a11ab44b5afdcea8f745",
          "message": "Add a capsule rod obstacle barrier for deformable bodies (#2804)",
          "timestamp": "2026-05-30T16:11:48-07:00",
          "tree_id": "8feac7011b22a2c9725826286dfff096de9722b9",
          "url": "https://github.com/dartsim/dart/commit/af917bbafc6a50492613a11ab44b5afdcea8f745"
        },
        "date": 1780184790797,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 576695.1106940944,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 576449.4469507101 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5149973.847453065,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5146363.550847458 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 74564690.47381113,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 74266309.26315786 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 582198.5581399313,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 582099.9622641496 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 6114627.895911387,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6102524.676579929 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 61559396.27272097,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 61542603.681818195 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1310924.567828614,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1005007.8480355825 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 7339571.811311509,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6908475.216981123 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 61507027.40904472,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60341005.99999997 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 134622.77314577234,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 134543.5380513808 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1671510.815510847,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1671373.7508813234 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 17726320.675946124,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 17724542.824074082 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 654658.6530102126,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 372188.16312594735 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3671787.6588805225,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3187422.2009345633 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 25353964.027797494,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23756232.05555557 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 43602.17105346414,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 43590.893526351916 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 168584.7848601078,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 168347.91987562823 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 178995.49700766255,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 178955.50611501397 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 80787.94089882732,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 17564.468594961698 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 232465.06245627798,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 20278.349716752662 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 251053.09597640487,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 40928.51195673741 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 327270.27764742484,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 326430.7051764691 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 662163.9555651011,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 659029.1459307767 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1320711.7277053678,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1319496.0208728563 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 261467.50415251768,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 14897.087506070957 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 487602.6138460541,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 20733.660284152404 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 660552.5703998863,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 26123.431700001733 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 256330227.00058064,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 255761145.0000024 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8f9b850d382161b6822f0aa735482ea33eddcde8",
          "message": "Make C++ demos scenes Z-up; add robot-agnostic Python SIMBICON (Atlas + G1) (#2786)",
          "timestamp": "2026-05-30T16:29:24-07:00",
          "tree_id": "ff1a79a7412522ae914a8e1f7597a320d1a45cdc",
          "url": "https://github.com/dartsim/dart/commit/8f9b850d382161b6822f0aa735482ea33eddcde8"
        },
        "date": 1780187342500,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 475045.06954769365,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 475012.48818365973 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4722126.738249165,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4719733.221476506 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 57033093.99996215,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 57027288.87500001 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 474719.06852048455,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 474658.575644505 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4734455.824375973,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4732455.842293907 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 56305897.083348095,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 56297448.041666694 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1728863.610180232,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 885504.9413659791 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 5141585.066917371,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5078108.408921931 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 49928241.88467729,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49641298.38461554 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 110852.13936242207,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 110849.79513944213 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1327859.7697415303,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1327827.6831588985 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 11947346.393155277,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 11946558.145299107 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 1020653.3888141691,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 386156.7685309066 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 2775431.3631160622,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2677189.2547528404 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 15573478.249972962,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14626755.999999931 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 41829.058119843605,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 41825.76159866702 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 162533.92142608206,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 162523.09499237625 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 171531.33280326377,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 171523.93006480107 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 69191.89829837128,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 16595.522317744446 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 213774.99326137293,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 19290.27560374449 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 207331.77675575193,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 32425.632067948845 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 316899.17294095474,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 316890.29340634285 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 634034.4345070281,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 634018.0334236693 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1271794.6128767894,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1271543.8068903072 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 177447.7039982398,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 12306.116522549391 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 254164.20883883644,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 15860.962952856658 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 740816.0248000057,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 24241.88640000011 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 211510267.49976155,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 211496474.9999994 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "71a1140b286d46ab80060eb2ca4675fef8f50bb1",
          "message": "Add opt-in adaptive barrier stiffness for deformable contact (#2805)",
          "timestamp": "2026-05-30T16:48:25-07:00",
          "tree_id": "a6b139d57618a6c86cb7b3e75ee07765034f15e3",
          "url": "https://github.com/dartsim/dart/commit/71a1140b286d46ab80060eb2ca4675fef8f50bb1"
        },
        "date": 1780189490096,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 454404.22081741603,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 454331.34866093984 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5337940.459991972,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5337604.200000002 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 79650110.83332582,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 79547082.44444446 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 686567.0364610698,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 686221.333009237 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 7447061.617793948,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7443908.089005228 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 85518674.1177664,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 85178167.0588235 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1105471.0398223298,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 931485.8502949869 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 7725117.881512694,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6664867.431279627 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 57243886.75010535,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54468816.541666836 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 132261.16293656043,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 132244.5988388425 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1608229.527973763,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1608059.950174824 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 14270596.782610126,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14264508.793478252 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 637051.2049773402,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 404134.17980984366 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3290915.687748751,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2789356.097959199 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 18260145.934099067,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 16020336.340659313 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 43777.5731632332,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 43732.59531607229 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 167728.09090916679,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 167722.52026143915 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 176816.4212142809,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 176186.4765564958 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 78160.30142811128,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 18081.659732058244 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 229523.38450488469,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 20982.23959577981 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 249952.6238897751,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 44229.33416064418 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 329717.98144217196,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 328128.18252290285 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 659964.3388381383,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 659396.0778669192 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1306678.3317766765,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1306576.837383183 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 244888.2461364887,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 16722.165017351686 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 462024.7834776587,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 24463.28869101206 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 742421.6330000489,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 37977.36459999896 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 2119732785.9990766,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2105615948.0000076 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "23c33125910a7d429ef367b8380504adceba2ac1",
          "message": "Add Coulomb friction against the capsule rod obstacle (#2808)",
          "timestamp": "2026-05-30T19:08:13-07:00",
          "tree_id": "f87ff5173ac94ad4ce81a17aafef49d57c83f38e",
          "url": "https://github.com/dartsim/dart/commit/23c33125910a7d429ef367b8380504adceba2ac1"
        },
        "date": 1780195383295,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 466493.4344919829,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 466424.34391711233 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4521241.299999606,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4520696.609677418 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 54011048.961534955,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54003490.15384614 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 467211.46677798155,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 467150.21936560975 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4540087.506536283,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4539430.009803924 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 53629632.76922983,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53621524.076923095 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 585299.4896825309,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 553378.3579365077 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4399929.285714223,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4295708.565217386 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47790856.20000387,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47467202.46666669 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111150.45332378811,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111135.55327608182 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1222687.1096490948,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1222477.3070175431 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9624912.724138053,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9623295.193103464 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 214327.68500897486,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 194501.51055026893 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1510730.753048991,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1420744.456300813 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10040213.40972214,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9708281.368055498 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55572.34518895301,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55564.4102889809 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 220313.0594620097,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 220298.0442032404 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 225127.0881361861,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 225094.70239299163 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77732.01562350018,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12041.275706767088 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 256905.62348178195,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13996.683751618906 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 264815.83183503954,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21842.882018956177 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433707.16950728,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433652.5751471987 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 867257.0532507679,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 867144.4111455098 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1735614.1908302268,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1735447.5885997468 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 138066.20796000064,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5402.947510000047 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 254786.2466699985,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6625.312279999918 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 492474.45779000147,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9100.731469999913 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 146665287.50001362,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 146645865.75000092 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f178240be1228c4d08c09575f02dd594edc1bcd1",
          "message": "Add barrier-only obstacle mode to complete sphere/box obstacle friction (#2809)",
          "timestamp": "2026-05-30T19:25:20-07:00",
          "tree_id": "332d6771bb066645f131674ba7067a1d05374b44",
          "url": "https://github.com/dartsim/dart/commit/f178240be1228c4d08c09575f02dd594edc1bcd1"
        },
        "date": 1780197255358,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 511886.85290888237,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 511859.80607391155 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5264801.89772716,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5264448.928030305 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 63379983.50000423,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 63365335.22727271 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 516710.58228779776,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 516680.8822878227 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5288334.286792462,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5288066.094339629 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 63463433.49999663,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 63460428.54545462 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 648292.9068350209,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 616846.2154984758 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4814925.023569314,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4713076.050505057 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 53382055.57692478,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53076395.11538463 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 116152.7762852419,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 116145.24054726347 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1485771.3538786422,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1485600.3188097773 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 13972275.700000407,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13970838.000000043 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 213465.3572821021,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 196407.53731551036 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1749136.344665923,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1654769.9062133534 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14490127.050505474,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14138511.717171634 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50127.01493767289,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50123.77389310807 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198660.7493617035,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198653.8415602839 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 204056.2474526977,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 204040.23740902616 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 68057.72138479051,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12973.374622214358 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218758.23205772226,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15978.980005771971 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 225486.76712753007,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23130.919261746993 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 390446.4557313451,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 390421.07882320276 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 777422.8862375544,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 777382.1992230836 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1553696.6570476573,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1553612.5260821448 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 124475.43493000012,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5175.3695000000735 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 224632.25961000033,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6497.845789999986 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 431687.09848999925,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8581.634049999991 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 141158813.1111077,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 141147761.3333337 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "31646f69933038a4cc6deac8b1f9f562a2c8d207",
          "message": "Strengthen the deformable iterative CG preconditioner to incomplete-Cholesky (#2811)",
          "timestamp": "2026-05-30T20:09:59-07:00",
          "tree_id": "e08c98373f4e0c7a02a0c948410d85dd642105f9",
          "url": "https://github.com/dartsim/dart/commit/31646f69933038a4cc6deac8b1f9f562a2c8d207"
        },
        "date": 1780200667403,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 469327.89713319414,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 469301.6364249577 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4653022.313531326,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4652751.082508247 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 55495548.360004246,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55492130.00000002 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 474369.5842583271,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 474329.7571476614 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4677449.880794646,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4677250.834437078 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 55606994.03999933,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55605523.83999991 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 593084.7866449289,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 563124.6298859931 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4535155.774603107,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4433299.917460315 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 49976960.392858215,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49681582.75000002 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111954.25020099692,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111942.3191027493 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1209329.2326387228,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1209223.4696180604 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9584629.726026371,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9583775.705479475 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 214155.80462153585,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 195017.48042064506 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1533381.8862705946,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1442486.7407786858 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10043995.569443747,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9724902.80555554 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55520.45769566164,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55518.004480215925 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 220030.78359266443,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 220012.60757504287 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 224483.7439513126,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 224473.24867809497 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77734.08469543184,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12026.019210790595 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257023.64056975147,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14039.968146805122 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 264624.2478954012,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21691.513676030678 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433733.66284478595,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433706.9959714915 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 866830.3597523534,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 866743.367801854 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1733358.7821779605,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1733271.6955445574 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 137519.45322999972,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5318.114180000037 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 253559.2404199997,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6544.156190000052 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 489850.54750000016,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9081.78625000005 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 148529426.00001028,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 148516867.62500015 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8a2ee7aad5060f089a314aeeb3eeeabf7e5542c3",
          "message": "Extend the World VBD solver: FEM materials, static obstacles, surface self-collision, and the TinyVBD strand demo (#2801)",
          "timestamp": "2026-05-30T21:46:12-07:00",
          "tree_id": "ac1ce759729c76e148a0b8631b7101cb42295368",
          "url": "https://github.com/dartsim/dart/commit/8a2ee7aad5060f089a314aeeb3eeeabf7e5542c3"
        },
        "date": 1780204213187,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 525295.1068247821,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 525268.2639138241 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4966991.211602753,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4966118.7030716725 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 60100643.04333038,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60096956.95652169 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 624677.7801862705,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 624649.2273821819 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5941203.841813353,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5940831.061224497 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 67583047.941084,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 67574966.35294114 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1165608.209772081,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 953803.8755700308 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 6895166.830502804,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6700771.665254254 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 59272244.68445945,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 58170718.84210521 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 165913.24608785342,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 165907.9588127015 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1327922.9554178633,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1326310.6789808916 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9434760.476812175,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9428531.450331131 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 604500.7095003895,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 358858.4091888012 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3174567.1925450806,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2747786.7681728876 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 15877059.852259248,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14439362.931818165 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 41823.907949637534,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 41822.424490532794 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 161880.55669146948,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 161875.21375464706 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 168175.88302153867,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 168147.46479885175 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 64291.923124917375,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13789.869363559452 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 209410.72500469486,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 18407.011997750382 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 215824.92064356295,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 35562.25924545649 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 326602.7976247082,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 326587.8360503017 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 652323.6837152852,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 652306.0549037984 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1290871.132720983,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1290828.9465437785 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 236525.43427213145,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 14533.910930249453 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 319031.7882040635,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 18134.3601494507 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 738358.9481003582,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 24083.692300000333 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 191652563.14278975,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 191645477.4285715 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "6bbf424658c2671f97bcefbec8bb8a1477bcfde4",
          "message": "Document Dojo differentiable solver planning (#2807)",
          "timestamp": "2026-05-30T21:55:36-07:00",
          "tree_id": "20653b53c9ac8e97533985d1d5b35998e2d31453",
          "url": "https://github.com/dartsim/dart/commit/6bbf424658c2671f97bcefbec8bb8a1477bcfde4"
        },
        "date": 1780206485085,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 430836.4580089823,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 430498.01685919444 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 3973427.925495375,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3973126.091690546 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 47740324.133337714,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47737009.23333338 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 450915.679167482,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 450885.60984594724 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4190958.1842122204,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4190654.880116951 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 46571972.379250176,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 46567957.93103454 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1674775.8026855316,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 901005.0031928503 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 6729548.16095071,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6598156.887804893 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 58175734.59088717,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 57676837.863636486 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111195.14233816294,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111107.40228730398 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1133493.0238275323,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1130507.6417791864 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9422003.336427037,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9421721.037383216 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 840853.947252921,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 399486.656868132 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3097723.359276356,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2828446.794411177 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 17452473.963380836,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 16174312.780487848 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42700.88694606144,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42698.09299950478 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 166565.78671241924,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 166502.56525343427 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 174554.78289409095,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 174550.1262772811 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 71081.3830308455,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 16996.95279580406 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 205523.2549489943,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 18595.350406334645 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 220084.78182097615,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 36878.097277181834 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 322217.4451111238,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 322110.16143911454 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 657763.9811225047,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 657692.3397790034 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1304995.2431415387,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1304681.353831608 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 166395.99316590503,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 14962.61204675455 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 350049.51446650067,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 19904.57572508362 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 1020799.8466998106,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 29439.7662999998 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 464540629.99934345,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 464502683.666666 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02",
            "email": "jslee02@users.noreply.github.com"
          },
          "committer": {
            "name": "GitHub",
            "username": "web-flow",
            "email": "noreply@github.com"
          },
          "id": "6bbf424658c2671f97bcefbec8bb8a1477bcfde4",
          "message": "Document Dojo differentiable solver planning (#2807)",
          "timestamp": "2026-05-31T04:55:36Z",
          "url": "https://github.com/dartsim/dart/commit/6bbf424658c2671f97bcefbec8bb8a1477bcfde4"
        },
        "date": 1780212521113,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 416318.63731152465,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 416220.97074626875 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 3837608.934255725,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3837255.5890410966 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 43296718.71869323,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 43294484.37499999 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 416872.5436336011,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 416821.4551703523 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 3784522.6216115803,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3784181.818918914 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 46028466.233353056,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 46027157.23333333 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1780039.7511739435,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 750832.4241001573 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4407300.359064372,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4363648.040268445 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 41981414.852831654,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 41628835.35294108 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 109021.12645694935,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 109004.5886056273 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1047450.2852877663,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1047116.3741598205 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8040872.954297811,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8040182.885714315 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 1273612.6100544592,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 391463.52076038066 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 2571132.840594319,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1860537.5858310706 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 11389224.860281771,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 10218894.647058824 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 40794.481995530215,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 40793.53860601611 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 157132.23451368496,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 157127.80054458798 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 167298.92978074288,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 167294.62368265344 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 92037.04528492312,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 16491.371370726858 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 189611.0924583102,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14202.14338269397 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 196828.09893473054,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 33352.257425086136 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 302936.9525967954,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 302904.40173160407 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 607044.4125808632,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 606940.720173539 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1215241.525125332,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1215176.6603119585 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 223038.5990518008,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 13258.489942431404 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 315781.8053177886,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 16895.263438037946 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 922390.7594001502,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 24756.396799999435 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 156565840.49975207,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 156536796.1249997 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7e548cbe2991bbd3001433a97116fa632dba9046",
          "message": "Variational integrator roadmap follow-ups (PLAN-082) (#2779)",
          "timestamp": "2026-05-31T10:23:01-07:00",
          "tree_id": "d6d50ed9f2e99038c017a21577b7622665771575",
          "url": "https://github.com/dartsim/dart/commit/7e548cbe2991bbd3001433a97116fa632dba9046"
        },
        "date": 1780249836347,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 471491.0619320152,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 471452.81016492756 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4684055.636666547,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4683511.196666664 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 55503829.720000796,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55497212.960000016 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 473691.3604336201,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 473633.8448509483 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4710608.929293175,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4710066.575757577 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 55524834.87999779,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55522864.600000046 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 613207.4097539972,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 582747.4922884543 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4572156.575080133,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4477438.830670926 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 50803188.392860316,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50467508.64285723 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 116755.9443522893,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 116744.14786863503 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1491966.4376994933,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1491869.9467518611 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 14005000.009999549,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14003941.81000003 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 214523.1654269907,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 197159.617906336 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1755674.5469679022,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1664123.149821641 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14516723.949495232,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14170194.262626203 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50045.14015083775,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50041.73853522502 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198414.62851461823,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198400.15450156192 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202900.05797100413,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202882.47275362315 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 68031.58901650202,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12850.631865843608 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218333.86969124913,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15791.74464069028 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224556.33910782446,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22644.73138864062 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388516.67721340136,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388479.49458784214 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 777185.0260821432,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 777095.8129855719 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1555394.242222216,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1555230.4533333306 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 126274.69956999902,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5061.343140000076 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 224380.71951999972,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6230.369490000102 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 440998.49489000015,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8560.632889999908 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 143629393.55553153,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 143611585.99999946 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "94e039ca643855eeb492f2d82baa53d72a08b439",
          "message": "Adopt DART 7 clean-break policy and parity gate (#2817)",
          "timestamp": "2026-05-31T12:05:58-07:00",
          "tree_id": "a61368820358ca9303344f2d7efca60ee3356756",
          "url": "https://github.com/dartsim/dart/commit/94e039ca643855eeb492f2d82baa53d72a08b439"
        },
        "date": 1780257187014,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 453679.6480318959,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 453539.71270718233 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4206357.059435967,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4205709.19330855 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 49419663.89274707,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49408066.53571426 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 498951.23564812704,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 498915.0027008772 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4861583.812299356,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4857770.218430028 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 50609957.03657951,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50608814.59259253 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1713729.3268237992,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 986927.266846363 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 7604252.7407143675,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7443737.162037038 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 55302009.81857806,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54851837.72727289 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111638.56180127319,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111620.62855783709 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1441570.3904731863,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1441453.7476190503 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 14008869.684179792,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14008501.526315803 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 847655.2311365673,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 399084.5276301797 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3378691.948085979,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3194700.846501135 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 25657072.937519845,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 24322622.31249993 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42438.45099382179,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42436.81205196426 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 166616.9450011997,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 166612.78678571514 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 176343.7979507496,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 176156.1272208543 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 65540.44670947896,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13404.318083362077 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 206546.81168356957,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 18247.19566423248 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 217056.1146862793,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 33107.53279058303 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 322482.624409234,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 322391.34490084945 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 641948.8028554685,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 641928.4209558821 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1305626.984416229,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1305149.563703019 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 207587.94464397334,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 14257.409807686554 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 323757.6057581694,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 19722.992569749982 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 697036.6028996068,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 25628.188400000337 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 181644668.74986827,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 181627289.62499842 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "359be01f1c91109b3662b5646b5ff2c3d782a62a",
          "message": "Run the CUDA CI job on the self-hosted GPU runner (#2803)",
          "timestamp": "2026-05-31T12:57:23-07:00",
          "tree_id": "9b1c43c836fe3611769cae694c7962783ac32235",
          "url": "https://github.com/dartsim/dart/commit/359be01f1c91109b3662b5646b5ff2c3d782a62a"
        },
        "date": 1780260517500,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 439503.4008175126,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 439454.54143126175 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4192198.598242872,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4191753.764880954 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 51217052.42813083,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 51212283.82142862 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 444019.039107645,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 443976.7414944352 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4255061.045202078,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4254777.499999993 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 50121140.64291901,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50115731.571428485 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1733216.0540883758,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 906973.2757255934 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 6175535.254378541,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6067241.592105269 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48000343.85698382,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47728147.464285694 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 110295.42245135528,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 110244.60300395277 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1321453.9341458364,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1320835.5578551255 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 11963614.974828327,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 11958533.37815124 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 1048359.0651087329,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 392025.06126373686 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 2818185.505649789,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2748444.710526318 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 15945785.700104252,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14854247.95555556 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42302.71005612878,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42299.71379816182 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 162526.67633966883,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 162502.19735314537 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 169929.60812058297,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 169903.27854545505 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 64301.96795291335,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11479.19798636996 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 197150.78534718044,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 16355.857057654111 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 191940.34297895175,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21779.33281272595 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 320232.7346079452,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 320221.5389876878 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 641327.5876134279,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 641271.9981651393 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1280647.7902930242,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1280625.2261904802 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 208610.07549675205,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 13929.569388609285 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 327776.6189292432,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 19343.09166240649 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 791879.7931008156,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 26289.215500000293 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 234197052.16656438,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 234160675.83333227 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "adc131ea53be68dc6d2d2a1d9bf03fc81783f30c",
          "message": "Honor ImGui resize cursors in GUI backend (#2822)",
          "timestamp": "2026-05-31T13:18:47-07:00",
          "tree_id": "b526382652e88bd5f4efce01dadb6e69be15709a",
          "url": "https://github.com/dartsim/dart/commit/adc131ea53be68dc6d2d2a1d9bf03fc81783f30c"
        },
        "date": 1780265107997,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 471483.35443780775,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 471416.05594639893 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4240934.596820144,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4240186.130158729 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 50023834.536010064,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50017763.57142854 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 444820.4408420515,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 444785.3487844414 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4178372.8059937265,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4178256.2537313416 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 50004614.03584007,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49960974.928571336 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1834150.3829894473,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 879206.6719948855 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 5499797.208778407,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5427038.823293177 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47841387.068760455,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47534649.068965524 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111275.61543396719,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111263.11109341397 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1329441.5793955787,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1329302.9527410145 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 12221530.687514,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 12219392.107142797 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 828481.6255349492,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 388039.9621836792 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3179353.1862682793,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3068936.957871376 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 21718692.220447876,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 20578941.867647 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42442.643560083845,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42439.11794996817 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 161120.5338158077,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 161112.06763284936 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 171711.29653697167,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 171704.7241463706 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 66822.50268623173,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15361.376519288344 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 193544.6898480309,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14570.470851050413 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 211175.6644705782,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 29000.69659399477 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 325298.79059739213,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 321059.55458715564 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 656427.7784946171,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 643139.7587316218 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1284423.226690034,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1284377.0457038356 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 190260.03750802972,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 12961.135743208337 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 277445.455699791,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 16572.5458387059 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 790718.1684000533,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 27628.226699999912 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 270279703.00114244,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 270266585.60000104 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "7bed463f311a7e5c8313d203f9cae47df3bc0cd7",
          "message": "Add experimental skeleton loading bridge (#2818)",
          "timestamp": "2026-05-31T14:48:10-07:00",
          "tree_id": "219826ea2de51e113e2ab19279353751d169b364",
          "url": "https://github.com/dartsim/dart/commit/7bed463f311a7e5c8313d203f9cae47df3bc0cd7"
        },
        "date": 1780266468426,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 437660.3500340488,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 437644.80632145243 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4063009.1959338263,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4062889.710526317 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 57288189.72012959,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 57271594.99999999 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 519240.67306240485,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 519224.84800602007 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4983619.064296363,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4983514.000000001 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 57027014.36006465,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 57025514.319999926 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1551340.5293674807,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 930499.2349650349 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 6861648.383635148,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6615194.068965533 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 63238887.14978238,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 62373369.650000036 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 129534.58749669866,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 129529.6361591284 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1300240.9455120468,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1297288.064053532 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9156536.561298003,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9156174.638709692 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 875859.2047964804,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 385000.4024775966 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3005393.297649614,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2714006.9437386454 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 16783926.431487903,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15375830.821052643 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42707.274507826616,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42703.76340916747 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 166174.59594251483,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 166165.81967213083 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 176796.41594815126,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 176786.8146911517 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 71916.05612613734,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 17424.520614635756 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 215167.94635378753,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 20062.144169899853 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 218879.27940137932,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 37931.35073240801 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 326954.16497209476,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 326854.3408177738 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 648759.1375751424,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 648682.2461394471 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1300288.7103421788,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1300226.237084871 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 252583.42499965243,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 15849.640386724985 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 351006.01357741066,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 18723.720080477706 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 731356.7798002623,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 24884.8291999991 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 242535989.99930546,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 242524299.20000228 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "74338577982dd296d443fdfb47decc550bf9af05",
          "message": "Add deformable CG diagnostics and matrix-free solve path (#2821)",
          "timestamp": "2026-05-31T15:10:44-07:00",
          "tree_id": "97e0f2fe412e316509bff00e3e792fa49f467c6b",
          "url": "https://github.com/dartsim/dart/commit/74338577982dd296d443fdfb47decc550bf9af05"
        },
        "date": 1780267916757,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 607763.9973532134,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 605988.4599118942 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 6672451.364086122,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6672021.3834951455 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 66265063.10529197,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 66197699.05263152 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 534980.0481629176,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 534047.3587223582 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5060301.670301419,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5057582.916666661 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 59371871.962664746,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 59308770.92592591 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1322216.0722814077,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 951484.7115021966 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 7205341.930872942,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6738389.179723504 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 61577517.72711477,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60616217.22727265 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 129829.3829716681,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 129825.23798488644 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1340006.9936085152,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1339937.4151459844 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 11194669.559055308,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 11194034.842519699 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 560252.7193842648,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 368714.32907425216 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3117869.904475399,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2665289.9473684025 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 16943704.73809873,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15397215.440476231 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 43426.49643265478,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 43390.6853549021 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 168056.0966015232,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 168049.17340489084 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 175845.44269865027,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 175837.82841585416 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 75796.43699369009,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 17035.900706722008 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 214246.03501170292,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 19353.975319699948 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 229188.04329607947,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 41419.59458637477 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 329302.2023687503,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 329284.5810650883 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 650257.3395647946,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 650193.2573563753 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1310014.583723523,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1309783.9335827949 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 255975.12491250387,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 16395.695839467127 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 362820.59917668323,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 19985.4965132291 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 737489.2738997004,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 27352.222499999352 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 278527810.2528537,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 278501984.24999917 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "554a13f33b618406e444fbf9e0a5e23ab6a22e97",
          "message": "Fix demo search keyboard input (#2823)",
          "timestamp": "2026-05-31T15:54:38-07:00",
          "tree_id": "ebb954a066e489d6a96b5e7d101ac1bdce5710a5",
          "url": "https://github.com/dartsim/dart/commit/554a13f33b618406e444fbf9e0a5e23ab6a22e97"
        },
        "date": 1780269709464,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 602153.3846535695,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 599891.0231660231 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 6006590.229666315,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6005228.306220096 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 54827869.458677016,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54801177.75 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 431112.65077577834,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 431096.17364341085 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4155892.8119812687,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4155699.4045584057 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 57493008.94967746,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 57490910.74999998 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1579163.6236451229,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 917947.978260872 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 6495305.197233326,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6372208.756880757 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47202370.99999476,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 46662676.63636377 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 106942.5116440725,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 106859.86318369822 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1080067.146807153,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1080020.379707914 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9544626.032302698,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9544035.187096806 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 789644.186912255,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 379683.7486685277 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3264149.6887267022,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2861277.289883271 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 18193748.68931651,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 16381404.708737949 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42858.808410867314,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42821.06288019877 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 165345.3639594939,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 165331.8142315891 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 173457.14133021786,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 173442.59602161133 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 74589.26049567603,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 19086.882916181374 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 229027.25838930628,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 21675.155513156074 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 235365.85089947423,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 43782.64649641447 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 322158.98322654783,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 322149.14741318376 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 644522.8747093049,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 644503.1192787789 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1308531.9038852577,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1308448.0831793013 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 279029.1593236797,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 18353.40857477764 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 462617.6966010281,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 25987.017327598438 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 731187.645799946,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 35975.812099999865 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 504032517.0048163,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 503791302.9999998 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "dbd25f94ac28c0a3734c988d97da9f722600d81a",
          "message": "Enable CUDA-aware test-all validation (#2827)",
          "timestamp": "2026-05-31T17:09:35-07:00",
          "tree_id": "feeb37a9eceaac5a629a0d8644831d267378ad57",
          "url": "https://github.com/dartsim/dart/commit/dbd25f94ac28c0a3734c988d97da9f722600d81a"
        },
        "date": 1780275070571,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 466797.7591917343,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 466685.30440543237 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4521774.493548125,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4521401.909677422 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 53552270.38460971,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53547138.269230746 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 472442.47865544347,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 472397.02756302425 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4538298.931818604,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4537645.886363638 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 53702520.65384088,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53699041.11538455 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 579014.7358417,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 549156.3968192396 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4387652.753048702,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4283455.868902425 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47462143.51724058,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47132466.06896529 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111344.6657905106,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111335.6552767819 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1211303.2864675939,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1211244.0817223203 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9614764.931034345,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9614005.26206894 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 213016.70302614907,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 194181.13548830847 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1530693.6071794135,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1441844.601025643 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10132383.104894495,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9781222.622377573 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55620.78558666196,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55615.7053007743 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 220325.62155782516,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 220314.52446892302 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 224632.67790625108,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 224617.3394348107 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77576.6626900419,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11870.01187081322 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257642.58281813495,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14000.796397574568 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 265368.43360061664,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21746.60368542792 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433616.34717921057,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433564.57377557206 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 866855.7653250595,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 866767.5770897898 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1734314.8042130629,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1734118.1078066772 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 136253.56807000117,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5199.923849999948 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 255570.82160999926,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6555.637439999914 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 482907.0837099994,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8737.586109999995 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 153621938.62500817,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 153605036.75000104 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "efec8706ae6ca3577c228645fc7014c865cf18c1",
          "message": "Fail CUDA benchmark smoke when the filter matches nothing (#2828)",
          "timestamp": "2026-05-31T17:46:55-07:00",
          "tree_id": "fcd4153f893c1ee6637af947900f92c686d1f9ac",
          "url": "https://github.com/dartsim/dart/commit/efec8706ae6ca3577c228645fc7014c865cf18c1"
        },
        "date": 1780276948163,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 474271.07167288894,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 473687.57335257315 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4253939.917423709,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4250538.209439528 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 48911731.1376974,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48883779.96551722 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 676168.4182987693,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 675741.8003913896 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 7037268.0787067665,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7027040.078740157 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 72096922.50035004,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 72089784.16666667 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1781846.4437214027,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1553589.3327935233 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 10178246.229770593,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9552191.658385087 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 81894697.37499167,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 80269552.93750016 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 139505.7066483744,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 139502.40181191807 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1790759.6476207904,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1790647.466666667 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 22109566.014504697,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 22108511.5362319 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 690097.4345626308,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 549029.2796833748 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 4856715.154585943,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4138818.802631601 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 24030308.507612225,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 21694987.074626926 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 44644.638629773464,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 44629.42039823981 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 216089.36734983485,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 215585.42100296507 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 209789.66782245794,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 209710.91942945367 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 80618.06509585968,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 20832.566860030685 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 237145.69346443738,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 21307.54783358762 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 250440.26126741475,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 44209.05026544668 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 332008.274080807,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 331782.805712938 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 658914.8275074951,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 657484.8358974337 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1307713.4424811902,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1307183.5018552812 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 259930.11874342465,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 15626.052686902221 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 397909.17545332963,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 19753.11194504144 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 926205.4845996316,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 26857.800900000937 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 296706384.2515927,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 296696323.2500035 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "96385c4f02d31bad56a4336ddcb9efbc06fef87e",
          "message": "Prevalidate experimental world imports (#2825)",
          "timestamp": "2026-05-31T20:15:04-07:00",
          "tree_id": "57a75ca1f8db9119bf7249c9c14cde8284ffef54",
          "url": "https://github.com/dartsim/dart/commit/96385c4f02d31bad56a4336ddcb9efbc06fef87e"
        },
        "date": 1780285007209,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 475811.4477395527,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 475795.60525036446 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 6304651.606189683,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6303941.486301366 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 72842242.63199385,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 72831228.5789474 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 535366.4718611187,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 534703.3482860764 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5203653.364202391,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5202985.3067092635 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 61813328.908597626,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 61802603.04545451 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1219047.0792040147,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 984539.0048209379 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 7547080.275130201,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6977287.624338631 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 60072328.954860985,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 58651654.136363484 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 142022.64923503346,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 142019.37343599586 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1460604.0261269575,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1460557.266457686 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 11387379.103922285,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 11367983.664000006 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 570763.8748851286,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 365169.1098173514 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3065300.7250445634,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2678286.5641547837 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 15762731.50001523,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14571526.966666682 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 41973.06907071718,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 41931.83460460067 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 163778.85216670597,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 163774.73951537846 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 170777.17562068746,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 170706.45869538622 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 64039.669782179095,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13928.320279594982 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 184298.13965142416,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12174.07159027853 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 193309.65563041918,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 24052.11240208488 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 313382.4616761664,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 313375.46952039463 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 629400.0971745582,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 629355.006269596 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1262112.867560653,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1262016.1693693595 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 167700.40622010131,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 10540.459019999986 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 302688.1553293574,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 17306.063392042946 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 682095.6512994599,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23346.71469999989 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 195701591.16847208,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 195684147.00000146 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "bb5aa9bd711ae22304b0fa4aa9e1e64a455921b2",
          "message": "Plan official simulation API promotion (#2830)",
          "timestamp": "2026-06-01T08:14:45-07:00",
          "tree_id": "1731c770b6e1f2c6795b674876730da7e86bce85",
          "url": "https://github.com/dartsim/dart/commit/bb5aa9bd711ae22304b0fa4aa9e1e64a455921b2"
        },
        "date": 1780328730750,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 461722.05482172425,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 461686.5247688243 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4529043.122977438,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4528116.106796114 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52934270.153846584,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52922505.84615379 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 465393.0724011823,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 465359.23912321514 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4528884.864077383,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4528545.000000003 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52922413.57692372,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52918989.69230762 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 616324.5777310975,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 585841.7827731116 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4465938.363636467,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4368689.025078366 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48146576.65517565,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47846699.68965519 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111692.87885778169,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111688.22263760724 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1164853.0827759118,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1164790.652173917 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8786864.63522126,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8786362.100628907 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 209331.10595754362,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 191353.82113710628 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1428286.5175687547,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1334289.6733143332 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9290275.23357633,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8940583.364963531 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50036.55180301275,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50034.18240949241 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198424.2683479574,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198410.50708416093 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202519.54853175147,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202509.43570085205 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 67813.68918757648,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12555.613720035564 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218568.9803579381,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15655.775255669172 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224687.43214221715,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22571.327521045958 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388456.23046985484,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388433.35501807043 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776967.338700689,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776930.5324819536 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1553720.0910099123,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1553649.1165371782 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 124334.86232999939,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5069.962740000023 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 224723.74679000044,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6179.2899499999985 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 437940.24126000074,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8590.869929999912 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 138061371.99998778,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 138054187.55555522 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "80f5c5b93916bcb9d6010dc79bb075eb1138b010",
          "message": "Add AVBD rigid block foundation (#2832)",
          "timestamp": "2026-06-01T14:43:41-07:00",
          "tree_id": "1e252840553fbefae7d88a1ec0b0b3a10dc89ce2",
          "url": "https://github.com/dartsim/dart/commit/80f5c5b93916bcb9d6010dc79bb075eb1138b010"
        },
        "date": 1780352261365,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 468274.0895473144,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 468236.3428761651 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4665958.916666568,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4665245.280000002 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 56286144.63999838,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 56279349.20000002 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 468955.72827173636,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 468903.95704295713 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4696636.0903008785,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4695959.250836124 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 56104316.04000041,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 56098985.60000005 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 587413.8885375309,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 556700.6102766796 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4613302.0888884775,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4514014.523809509 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 50636997.00000208,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50238118.99999992 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 110776.51609577057,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 110768.25182365991 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1202485.1372211627,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1202338.6843910806 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9610371.687075507,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9609324.517006828 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 213988.2726014159,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 194517.55350962208 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1548430.8753926333,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1459267.7853403138 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10083343.853146836,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9758224.461538473 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 56045.07600063405,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 56040.83215533684 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 220197.3039308421,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 220173.53773584825 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 224527.07925559164,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 224510.56521739112 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77661.7972837537,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11855.701493171155 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257138.94349308196,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13958.483807032055 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 266303.96448503603,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21523.865996340977 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433556.1514870091,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433529.6087360584 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 868631.1700807735,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 868530.9466170064 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1737948.1898261511,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1737786.539702221 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 137239.0478300008,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5267.936449999979 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 256263.01721999884,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6759.442709999917 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 490284.95875000086,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9270.466090000014 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 150374206.12497953,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 150349321.37500158 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "a8d34b9b5bc341afa8f12720d180cd31979e3c9c",
          "message": "Add AVBD rigid contact joints and feature keys (#2837)",
          "timestamp": "2026-06-01T18:33:31-07:00",
          "tree_id": "25e116861370aaa983b9ccd2d65855e354d8971f",
          "url": "https://github.com/dartsim/dart/commit/a8d34b9b5bc341afa8f12720d180cd31979e3c9c"
        },
        "date": 1780367997189,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 459570.79986814753,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 459545.2868447082 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4518624.590322097,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4518371.570967742 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52958364.61538365,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52953611.115384616 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 464457.5871437849,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 464421.0791915175 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4515500.935691056,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4515289.726688106 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52875904.03845745,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52872405.15384621 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 607502.1272114933,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 577008.7236731258 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4453173.164596231,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4357062.440993774 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48235314.44827526,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47938732.27586222 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111639.63829616073,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111634.85495085067 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1162414.0488815268,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1162328.7315658673 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8798390.823900184,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8797822.194968568 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 209440.47862313854,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 192352.94331375518 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1426229.4245014547,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1337986.598290599 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9314372.673203085,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8978577.248366075 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50082.004077545746,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50079.31912153951 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198675.30398977012,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198659.0251313361 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202747.98143041428,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202739.53619614092 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 68074.6709642455,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12811.113157132628 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218718.63160595283,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15792.477680861908 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224766.85610859768,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22682.608678086628 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388615.52872606204,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388584.928670553 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 781339.4871652842,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 781304.9704241125 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1554317.8590455877,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1554223.280799102 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 121195.97827999997,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5001.631830000121 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 228270.07685999887,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6309.195799999969 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 431831.40889000014,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8685.962839999916 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 136685015.3333164,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 136669741.3333324 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b5922f4ddecc08684a57cc3de4a9cf95adc5b74b",
          "message": "Modernize the dart::gui viewer theme and add GPU (CUDA) deformable solve to py-demos (#2836)",
          "timestamp": "2026-06-01T20:13:22-07:00",
          "tree_id": "61a206fa55aeeb59286fa2a29f211b37106feeae",
          "url": "https://github.com/dartsim/dart/commit/b5922f4ddecc08684a57cc3de4a9cf95adc5b74b"
        },
        "date": 1780372248563,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 477218.24889341975,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 477124.4048348656 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4682181.00671102,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4681637.493288593 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 55658832.39999494,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55650083.6 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 473659.8437076904,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 473594.2066982414 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4705709.269359683,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4705224.21212121 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 55627243.44000343,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55619912.59999999 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 587995.1071713176,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 557545.7844621509 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4546717.636076123,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4444422.335443028 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 50201413.89285624,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49899456.8214286 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 114233.93408477877,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 114220.59201176174 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1229551.7921846237,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1229396.232682062 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9681520.006897071,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9680276.758620685 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 213967.02354571456,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 195347.7516620501 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1541639.1175869338,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1452430.652351737 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10097256.951048559,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9779053.076923067 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55554.70411524508,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55547.02627088394 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 220332.54624097035,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 220314.06023906995 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 224662.49079853995,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 224641.4583133304 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77646.55344057629,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12063.78204796629 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257001.0099290069,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13961.531506849336 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 264486.0800643879,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21795.33335417137 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433581.84479552135,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433529.4730483258 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 867365.9219814573,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 867248.7405572747 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1734369.8190829644,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1734219.8054522842 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 136521.96121000088,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5524.808529999916 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 251689.4176899996,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6689.015939999906 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 482637.5866099988,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8984.312070000016 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 144127614.88890484,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 144106142.3333344 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c24545c22bc0bf99447af78714e18b19cede0bd1",
          "message": "Add AVBD self-contact friction rows (PLAN-104) (#2845)",
          "timestamp": "2026-06-01T20:27:15-07:00",
          "tree_id": "ad7c9e83702f749f57904634c1708e55262a4790",
          "url": "https://github.com/dartsim/dart/commit/c24545c22bc0bf99447af78714e18b19cede0bd1"
        },
        "date": 1780374324616,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 464358.40867377777,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 464343.6540735873 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4516362.562056727,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4512752.8759124065 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 55595579.000655554,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55589735.24000002 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 463951.4217751789,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 463530.9254125412 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5291554.857156799,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5290435.930232563 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 55444557.41148662,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55441079.23529416 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1747992.9889255113,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 926769.1147327245 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 7493754.639374226,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7296784.413461556 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 62654488.33316138,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 62296342.458333306 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 115919.3881238001,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 115858.77222222208 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1082706.653083173,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1080586.5676923078 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9225416.133642955,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9208913.617834361 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 941274.8727237586,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 387162.1546853162 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 2829511.2543771025,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2667252.0097087245 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 15752241.978924232,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 14417868.031579038 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42393.08127830547,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42373.41321910927 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 164851.53167938374,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 164845.79505796995 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 172984.98028067016,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 172976.9493468078 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 64676.42209638208,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14255.38422908423 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 186876.9078465012,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11516.185008014429 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 209531.7756721022,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 35277.06003283831 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 319778.64498002914,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 319561.04260192276 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 636546.1160155103,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 636519.308917195 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1279540.7550226224,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1279489.8524590174 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 185239.11266513087,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 12391.13548356635 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 240966.60612617506,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 15649.226149180768 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 628878.6940014688,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21645.672299999318 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 219388385.60017756,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 219209906.99999833 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8b87e1dba0a8c00b457418241f5eb730254f6de1",
          "message": "Fix dartpy autodoc stub imports and clang-format cache staleness (#2851)",
          "timestamp": "2026-06-02T07:24:50-07:00",
          "tree_id": "6bfd8e2014004ee4ac1d65d81320314cb7e6e2dd",
          "url": "https://github.com/dartsim/dart/commit/8b87e1dba0a8c00b457418241f5eb730254f6de1"
        },
        "date": 1780413038871,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 508779.01385349844,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 508727.697411593 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5301619.417624592,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5299747.873563219 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 64943195.380952336,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 64938114.714285694 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 519017.0855988317,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 518961.984937546 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5313582.692015154,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5312702.20152091 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 65321930.1904777,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 65317249.80952378 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 615771.7936108091,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 584179.7065994112 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4803552.016835243,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4698545.215488211 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 53038949.49999561,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52735697.3461538 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 112860.82353415508,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 112851.10084917121 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1220473.9275108597,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1220369.2174672498 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9620217.130136615,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9618999.554794537 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 215815.94947218694,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 195792.20365939452 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1538531.6676952802,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1445941.8930041117 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10068618.664334891,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9742746.986013977 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55690.69201430952,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55686.22193087004 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 220050.35641228265,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 220023.6283241536 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 225773.05989668358,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 225750.19825637803 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77774.53300085907,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11934.221607910496 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257211.1449047155,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14178.417800983241 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 265006.7041624556,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21957.80569516847 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433986.6184129167,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433952.6773093609 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 867250.1493185073,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 867161.3587360559 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1738973.9006211925,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1738823.3291925464 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 138910.79743999854,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5510.5999200000615 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 251031.31938999923,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6616.368309999957 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 487327.83111000125,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9185.05910999997 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 145904826.22222427,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 145891777.66666713 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e8484fc3e788364d6ac09312036536d265bd6790",
          "message": "Add DART 7 rigid open-chain dynamics parity harness (PLAN-080 B2) (#2842)",
          "timestamp": "2026-06-02T07:26:25-07:00",
          "tree_id": "e5ef36ef6ff719b43177a416ba3a61e77f09446a",
          "url": "https://github.com/dartsim/dart/commit/e8484fc3e788364d6ac09312036536d265bd6790"
        },
        "date": 1780417727975,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 519716.08742157044,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 519662.1257838436 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5353101.098859465,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5352395.8517110245 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 65991490.50000299,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 65982884.090909064 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 517647.5273466629,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 517586.449371767 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5383912.874046077,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5383210.671755734 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 66205290.19047743,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 66197663.80952371 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 622864.8192872141,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 593377.0989517829 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4829919.436425649,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4727393.144329892 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 53696869.461541265,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53388829.846153766 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 113550.53996147467,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 113536.49662975482 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1229846.1001757768,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1229685.4077328667 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9658869.855171548,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9657735.158620687 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 213848.60917150564,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 195436.73455250703 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1526370.011235933,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1435379.9325842725 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 10067421.374999855,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9745049.298611086 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 55633.45766618286,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55628.32496324878 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 220133.83065784466,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 220098.30311614796 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 225858.5387590814,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 225833.7869460126 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 78116.65687654506,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12012.739829373828 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 257279.19753372076,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13965.800947485945 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 264848.954303182,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21697.797628373362 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 433640.91047086264,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433578.73203222017 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 869072.2054624519,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 868976.0111731859 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1734366.2391574185,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1734132.6356877228 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 137460.37515000126,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5228.856789999981 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 252707.90231999857,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6514.863839999947 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 489286.98981000023,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9029.449479999983 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 157882338.50000212,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 157849936.8749995 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "18a4ce558fceb54fd298baec77306478178c4d5e",
          "message": "DART 7 PLAN-041: de-ECS the experimental handle surface + promotion-surface audit (WS1/WS5) (#2844)",
          "timestamp": "2026-06-02T09:28:44-07:00",
          "tree_id": "9f8367f7ab43eb908d7698168b95d40ae5c32858",
          "url": "https://github.com/dartsim/dart/commit/18a4ce558fceb54fd298baec77306478178c4d5e"
        },
        "date": 1780425370527,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 459592.0647600557,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 459584.13083497697 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4480830.02250788,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4480518.983922832 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52617450.88461549,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52616374.730769254 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 460428.53241193265,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 460398.31358999753 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4493076.442307653,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4492662.628205124 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52781487.629629366,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52780345.2222223 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 594817.4779620565,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 564173.6643752528 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4399053.560000539,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4302631.556923083 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47853464.00000169,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47539106.172413826 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 113956.3598702996,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 113952.82845561426 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1189064.5668084656,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1188963.424680849 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8896767.775641551,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8896681.070512822 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 209172.0930619299,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 192362.21229746516 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1455832.632038737,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1361283.8378640737 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9469853.600000622,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9125231.316129008 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 49972.44300574309,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 49969.3202880629 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 199509.63987461312,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 199480.65412569433 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202063.6968822323,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202038.78565242558 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 67105.19386708862,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12789.644887687611 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 219083.52359014878,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 16089.039957403367 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224839.6707627863,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22977.974351898414 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388633.96502917144,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388592.59644740407 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776958.2119866902,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776916.058268595 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1554215.4066666728,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1554123.475555558 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 126519.47948999806,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5302.108120000071 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 224548.902319998,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6252.227009999984 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 440559.84609999997,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8674.618149999986 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 148701691.250011,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 148687940.49999857 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "88d90d7d2190530a76bfac46cab5446e809e3a4f",
          "message": "Add AVBD rigid-body fixed-joint facade (#2847)",
          "timestamp": "2026-06-02T12:33:49-07:00",
          "tree_id": "9a53b111bb5866ba6c330d54a92a263494fde345",
          "url": "https://github.com/dartsim/dart/commit/88d90d7d2190530a76bfac46cab5446e809e3a4f"
        },
        "date": 1780437147235,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 448552.95125082624,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 448407.0955740859 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4357697.532710472,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4357289.872274145 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 50310368.28571749,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50305031.24999999 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 451613.35242995154,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 451571.2925651754 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4376265.358255055,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4376016.859813077 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 50392408.214284904,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50389194.035714366 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 579390.7815820948,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 549732.9795356157 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4266028.284866522,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4171888.0000000023 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 45546569.387096815,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 45251610.2580645 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 113023.48645162304,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 113016.1200806449 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1180464.3813914459,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1180328.9170159257 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8863804.183543904,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8863097.063291168 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 208406.61642733394,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 190652.53241175666 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1430298.5766634115,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1341254.8437801262 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9356154.335483229,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9021360.116129024 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 49907.08871542375,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 49904.15526090653 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198370.0338718869,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198348.44926303733 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202035.2778098885,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202018.492776655 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 67018.88604987164,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12489.403794723463 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218140.2267488153,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15487.792774656029 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224031.97151101186,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22150.36793125855 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388513.87402879633,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388483.2719200889 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776856.2347391899,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776805.9728079918 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1555149.9899998412,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1555059.084444442 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 124202.8710800014,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 4946.482839999931 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 229521.72436999946,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6382.747100000045 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 440406.28314000065,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8451.150689999879 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 136508940.6666458,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 136491102.9999989 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "5673519338e2bfa2294f0c67c003813c6cc552e2",
          "message": "Expose rigid fixed-joint endpoint handles (#2864)",
          "timestamp": "2026-06-02T14:19:09-07:00",
          "tree_id": "bf9d791ff1d00f0ed47de92a501691bcfbbc6d0f",
          "url": "https://github.com/dartsim/dart/commit/5673519338e2bfa2294f0c67c003813c6cc552e2"
        },
        "date": 1780441683976,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 470612.2065374526,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 460717.12528326316 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4500282.4749501785,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4490689.751572327 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 54919237.806014,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53291809.61538461 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 479076.94887585077,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 464771.2423638774 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4561552.334470069,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4430210.031847136 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 53771369.29719537,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52095380.8518518 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1679231.5410196942,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 929795.5251322767 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 6095427.25323837,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5760361.902953584 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 51158318.80054126,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49443088.399999864 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 118895.39817187679,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 115677.02385959245 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1172821.1030084565,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1167637.0547833191 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9682977.013289928,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9681972.820000008 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 998119.3111215484,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 453457.2038776122 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 2793073.582195276,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2595928.561922373 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 14224567.981590606,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 12842751.01886793 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42454.44998138376,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42447.88764113167 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 163662.1254231581,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 163643.46774005375 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 171212.09365411804,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 171187.52137376642 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 65493.73451224854,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14858.958207487552 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 202160.54738222013,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 16528.056573551552 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 213277.04903157885,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 31372.499072742274 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 326154.04664517293,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 322864.58729792084 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 643692.9206084759,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 643671.6154199219 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1285478.7282186088,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1285389.9489795952 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 202052.6192988543,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 14013.797732103738 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 308668.1545819933,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 17809.12356908618 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 729285.7826978433,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 24963.85400000065 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 197000414.99648556,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 196995015.33333337 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "64e61f1f04bf1e8e4e67726702f0f920c24b2aa2",
          "message": "DART 7 PLAN-041 WS5: de-ECS world.hpp (registry pimpl) + reintroduce internal component-access validation (#2863)",
          "timestamp": "2026-06-02T16:44:00-07:00",
          "tree_id": "fcc349533e9c4094abc8a281e457aee285da97cf",
          "url": "https://github.com/dartsim/dart/commit/64e61f1f04bf1e8e4e67726702f0f920c24b2aa2"
        },
        "date": 1780449948228,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 506092.90788912796,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 506045.53874268994 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4757890.259448134,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4757777.23208191 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 56952948.50011123,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 56951380.62500001 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 475157.4726055644,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 475144.303504593 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4744370.55091665,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4744287.527210887 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 56816044.70980043,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 56813451.166666694 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1828447.7306397527,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 945251.6145833327 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 6318683.448883772,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6241006.811023627 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 53320746.6661999,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52984910.66666678 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 112219.5650573315,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 112203.19269501309 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1550656.8601532478,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1549187.4712643698 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 12505667.662480846,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 12501742.310810855 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 955012.9769289511,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 395527.8964546989 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 3256949.2410142887,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3103955.7008928508 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 19521806.923545558,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18295093.575757578 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42210.14447642737,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42208.784483552954 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 163668.0357663308,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 163662.81484511963 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 173412.2504655445,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 173407.8082377606 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 61458.029840141535,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12632.784820000043 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 194865.37821008827,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14318.027820000054 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 218851.03322635902,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 39000.09828530017 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 320980.11092664016,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 320957.2565287727 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 645412.08467781,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 645363.2811780915 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1293635.0497876375,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1293546.0792626727 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 220074.96098303128,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 13598.223165099087 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 319685.33918338205,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 18959.14274688226 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 878614.2708035186,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 27702.431299999826 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 244326547.60312286,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 244310144.59999856 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02",
            "email": "jslee02@users.noreply.github.com"
          },
          "committer": {
            "name": "GitHub",
            "username": "web-flow",
            "email": "noreply@github.com"
          },
          "id": "64e61f1f04bf1e8e4e67726702f0f920c24b2aa2",
          "message": "DART 7 PLAN-041 WS5: de-ECS world.hpp (registry pimpl) + reintroduce internal component-access validation (#2863)",
          "timestamp": "2026-06-02T23:44:00Z",
          "url": "https://github.com/dartsim/dart/commit/64e61f1f04bf1e8e4e67726702f0f920c24b2aa2"
        },
        "date": 1780477362071,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 462373.64515062957,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 462286.57000993047 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4716874.627118127,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4716511.583050846 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 56862786.32000721,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 56857559.83999997 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 463753.3229752004,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 463721.37355371827 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4717520.841750505,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4717189.171717168 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 56984449.43999675,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 56979547.88000003 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 594052.2050880586,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 552984.5772994139 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4385749.603029993,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4254789.76363637 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47789924.766660385,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47318974.56666682 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 107568.6610430674,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 107559.86734458077 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1396200.4924923822,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1396096.5975976002 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 13045127.495326817,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13044122.915887853 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 207198.07058072,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 182464.86419910635 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1699911.5631929848,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1551061.5066518842 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 13678487.245284006,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13200345.971698022 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 61266.78361215845,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 61263.95677498467 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 244027.69513684302,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 244017.95834059504 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 247751.8398229898,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 247734.84761061936 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 77185.56822999971,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 9649.690330000029 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 260756.84213788624,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 10397.59735788752 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 268873.8313815177,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 18585.319853613877 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 451266.84875847556,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 451238.9887133173 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 902613.9013540271,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 902575.0773694405 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1806604.5058065122,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1806504.5935483773 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 190257.40370999984,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5896.615249999967 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 353203.6951299983,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8863.865269999991 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 672401.6695000045,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 11915.182900000333 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 130699387.49999891,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 130684689.9000007 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "35692e6513e8619118c3ee2b9c6972d937fb3f06",
          "message": "Add experimental World memory manager diagnostics (#2869)",
          "timestamp": "2026-06-03T07:31:58-07:00",
          "tree_id": "24d88a2acf585869a67a0ed9d712c09b6a5d44a5",
          "url": "https://github.com/dartsim/dart/commit/35692e6513e8619118c3ee2b9c6972d937fb3f06"
        },
        "date": 1780499334705,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 454127.31093802676,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 454105.0749756572 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4347311.209374993,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4346934.000000002 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 50038996.21428738,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50032500.357142806 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 455226.19095644454,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 455201.90728692216 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4343194.216049697,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4342938.68518518 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 49988405.10713925,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49985171.107142806 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 595163.7521092832,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 565529.8461229407 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4273066.225225096,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4179112.1471471353 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 44864520.61290389,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 44544659.70967741 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 112230.82919427085,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 112223.85698315111 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1171796.6894253304,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1171731.7735220653 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8832589.993710157,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8832102.779874234 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 205293.96057871313,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 187447.58786833033 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1455071.425242776,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1360373.3077669868 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9346476.845160896,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9001601.929032272 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 49929.69113029363,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 49926.26776794618 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 199272.14761568222,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 199263.74120996482 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 202145.3448973738,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 202134.39693553213 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 65068.527676266975,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 11962.014375032459 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 217926.65788934744,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13566.064074431568 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 223910.45467882743,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 20605.94959472396 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388563.85488345963,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388550.20172031154 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 776950.945615993,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 776915.5482796851 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1554905.406666573,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1554836.1177777776 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 120947.13147999982,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 4900.829630000061 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 228762.54964999817,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6278.4736900000835 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 442946.4064700005,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8667.007359999985 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 137170467.77777693,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 137152521.6666672 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "702eb6832df1aff24d7d19b919e6ac233310d7d1",
          "message": "Reset stale CMake compiler cache before CUDA py-demos configure (#2873)\n\nThe config-py task pins the host and CUDA compilers for the CUDA-enabled\n`pixi run -e cuda py-demos` build. When a build directory already holds a\nCMakeCache.txt from an earlier configure that used different compilers, CMake\nrefuses to change the compiler in place and silently re-runs with default\noptions, dropping the DART_BUILD_DARTPY target so the demos can no longer\nimport dartpy.\n\nDetect when the cached CMAKE_C/CXX/CUDA/CUDA_HOST compilers diverge from the\ndesired CUDA host toolchain and remove the stale CMakeCache.txt and CMakeFiles/\nbefore invoking cmake, so the reconfigure adopts the pinned compilers and keeps\nthe dartpy target. Add a regression test locking in the dartpy=ON honoring, the\nstale-cache reset ordering, and the py-demos -> build-py-dev-docking wiring.",
          "timestamp": "2026-06-03T15:11:42-07:00",
          "tree_id": "1fbf551b7391e60632203a5591b4bbe3c05c768e",
          "url": "https://github.com/dartsim/dart/commit/702eb6832df1aff24d7d19b919e6ac233310d7d1"
        },
        "date": 1780527791605,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 426748.40926685487,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 426458.48307410796 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4188411.54564039,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4179594.94857143 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 49092023.82051652,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49090358.21428574 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 427989.31037431286,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 427975.62530339864 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4053374.5471000723,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4051374.2336182315 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 45945345.59873864,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 45932790.23333329 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1761779.8847205269,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 855264.4235588963 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4795665.923797903,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4741362.217391298 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 42981279.96697722,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 42765417.54838712 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 107323.15619811742,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 107315.60372299676 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1082488.0061522212,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1082427.8247895956 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 9200729.869895741,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9200487.485207096 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 1000673.3943446231,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 404102.13791295217 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 2333915.609737661,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2170877.7739837244 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 12018909.6110321,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 11111239.351851897 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42086.162141813766,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42079.55101733705 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 160427.90899758745,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 160412.4935897443 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 170314.85687060063,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 170260.741462824 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 58392.00935427658,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 10309.054051637922 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 184340.73574026118,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14342.336420000094 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 211540.3444027244,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 32662.147307673553 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 322812.10967173346,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 322804.42132456705 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 645256.1843733506,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 645228.8790804599 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1296139.0277400075,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1296034.1109057253 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 137597.26331234293,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 12710.185678601176 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 324026.9311579574,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 17768.00498516163 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 758434.7410011104,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 26040.013499999757 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 225198255.17277542,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 225191341.66666532 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "861bcdc65671fe534464c0a344169f2e2be10266",
          "message": "Add alignment-aware common allocators (#2871)",
          "timestamp": "2026-06-04T03:05:08Z",
          "tree_id": "a4778be1b390fb436e54145dc333c5fbae86a071",
          "url": "https://github.com/dartsim/dart/commit/861bcdc65671fe534464c0a344169f2e2be10266"
        },
        "date": 1780544430095,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 702752.0628629676,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 702398.7524366471 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 3974839.7633993053,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3972380.206896551 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 48914499.64114665,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48890356.74999998 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 445066.6825552446,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 444889.9022452505 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 3979337.89521651,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3977890.9490085 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 48381423.43408739,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48362103.80000002 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 1651880.6837091725,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 915517.2213490497 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 6769747.00007122,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6644902.054999981 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 48624609.319958836,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48180889.79999999 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 107914.63435633702,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 107910.93980552546 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1077770.4691586953,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1077737.3587962983 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8700296.347845139,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8700053.198757777 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 862141.6469053914,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 386117.950171822 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 2996500.202847834,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2826816.1311188913 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 18357385.35208458,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 17010484.636363737 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 42558.09407235083,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42534.440524382146 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 164472.62408654334,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 164464.74239924655 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 172106.02270379514,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 172101.0281307826 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 61055.974632483085,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12206.171558879207 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 204481.7468720339,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 16566.156293242788 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 205450.1903564356,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 27575.70686174612 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 322717.129297487,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 322702.0124823356 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 645018.1644363137,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 644999.9912482735 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1304809.6311087667,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1302524.7752808987 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 234077.9394232744,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 15632.309059074578 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 471670.5918021326,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22498.099881763395 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 933159.630402224,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 46082.823300000084 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 1251674476.021435,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1251536719.0000007 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "0ed989c33389a0278c39d1e21b397ff8fa56303d",
          "message": "Share a device-runtime substrate across experimental CUDA solvers (PLAN-031) (#2875)\n\nConsolidate the three experimental CUDA solver modules (rigid-body batch, vertex block descent, deformable PSD projection) onto a shared device-runtime substrate so new GPU solvers reuse common blocks instead of reinventing them (PLAN-031): one isCudaRuntimeAvailable/throwIfCudaError/checkLastError/launchGrid1D (cuda_runtime.cuh/.cpp), one owning DeviceBuffer<T> (device_buffer.cuh), and a single-sourced per-body __host__ __device__ orientation core (detail/rigid_integration_core.hpp) shared by the CPU batch kernel and the CUDA kernel (deleting a divergent device re-derivation). Backend-neutral public API, build-only -cuda static library, and no-GPU-runtime-dependency packaging unchanged. Adds docs/design/shared_cuda_device_substrate.md and the PLAN-031 dashboard entry.\n\nVerified locally: pixi run test-all and pixi run -e cuda test-all both green; CUDA parity 3/3 within the Phase-5 1.78e-15 tolerance. CI had 0 failures; merged via admin override while the remaining required checks were stuck in the Actions runner queue (infra congestion, not test failures).",
          "timestamp": "2026-06-03T23:58:59-07:00",
          "tree_id": "0d862f36d6c07fdfa11c7ad6804203316162e178",
          "url": "https://github.com/dartsim/dart/commit/0ed989c33389a0278c39d1e21b397ff8fa56303d"
        },
        "date": 1780559100992,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 457472.96212119964,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 457422.2404479578 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 4491483.527330701,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4490937.327974276 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 52487850.66666844,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52481685.51851848 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 459435.0444079216,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 459393.4749999999 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 4505326.925806413,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4504798.412903226 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 52556115.555557035,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52551295.66666659 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 585543.9646628448,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 556066.9638505273 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4369526.140243906,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4273461.417682906 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 47483916.29999939,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47176137.83333334 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111725.44424058689,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111716.25701978302 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1175086.625209947,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1174948.3428571417 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8876451.051282158,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8875847.160256403 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 209063.59686221494,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 192005.75811732662 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1437445.659287831,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1345265.1424446616 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9380894.012904339,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9044300.741935503 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 49923.02948832276,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 49918.240256730416 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198514.57883551772,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198491.23827737803 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 201830.51861470306,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 201816.4724386717 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 66394.0011182679,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12743.229614705762 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218238.58869893287,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15915.61505390789 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224072.6520620204,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22692.43931351729 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 391045.1689943984,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 391020.5041899446 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 777195.3398112017,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 777134.1510272095 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1555704.484444252,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1555577.6166666627 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 121090.85348000006,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5092.697279999925 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 231766.38853999975,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6599.29949000002 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 439124.7219100001,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8887.567120000029 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 140699854.33333284,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 140683900.99999955 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "2d4eec61774e371d5f34ab0553ca5e827b2cc455",
          "message": "Add fixed-joint World lookup API (#2866)",
          "timestamp": "2026-06-04T10:52:13Z",
          "tree_id": "8fee52cf1807c67cd8def44f79bb948005804f7e",
          "url": "https://github.com/dartsim/dart/commit/2d4eec61774e371d5f34ab0553ca5e827b2cc455"
        },
        "date": 1780572213424,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "BM_WorldUpdateKinematics/32/8",
            "value": 500173.34321426327,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 500096.4289285715 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/8",
            "value": 5044402.7617325345,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5043023.685920576 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldUpdateKinematics/128/32",
            "value": 60907740.56521854,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60900295.21739135 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/32/8",
            "value": 499949.88607141166,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 499886.75714285584 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/8",
            "value": 5048227.147482204,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5047747.334532378 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepSequential/128/32",
            "value": 60951569.78261437,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60943681.521739185 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/32/8",
            "value": 621381.7090606625,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 589262.6500415636 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/8",
            "value": 4578947.74760357,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4482044.888178915 ns\nthreads: 1"
          },
          {
            "name": "BM_WorldStepParallel/128/32",
            "value": 50592885.2142899,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50286526.85714289 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/128",
            "value": 111034.68886336325,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111027.29735935682 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/1024",
            "value": 1176943.0358333467,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1176849.743333334 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepSequential/4096",
            "value": 8896993.873418933,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8896527.620253123 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/128",
            "value": 206760.1589466482,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 189313.0428089127 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/1024",
            "value": 1453151.6238096065,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1359172.550476187 ns\nthreads: 1"
          },
          {
            "name": "BM_RigidBodyStepParallel/4096",
            "value": 9390773.116129206,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9038432.548387086 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/16",
            "value": 50043.559381484134,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50041.07026272413 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/4096/16",
            "value": 198501.86119383018,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198491.17141641898 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedSequential/1024/64",
            "value": 203659.17518568374,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 203641.88087956866 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/16",
            "value": 66525.37991153992,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12601.614460822566 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/4096/16",
            "value": 218229.96271998718,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15855.111142129217 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactShapedParallel/1024/64",
            "value": 224886.43409693273,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23172.88413432118 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/4/512/64",
            "value": 388586.9195115387,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388546.20344157674 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/8/512/64",
            "value": 778668.534482724,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 778546.7836484988 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedSequential/16/512/64",
            "value": 1555604.9911111183,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1555467.0222222141 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/4/512/64",
            "value": 120957.5456000016,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 4958.28545000009 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/8/512/64",
            "value": 228994.30069999883,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6325.3464699999995 ns\nthreads: 1"
          },
          {
            "name": "BM_ContactIslandShapedParallel/16/512/64",
            "value": 441402.20235000015,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8548.216620000063 ns\nthreads: 1"
          },
          {
            "name": "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
            "value": 142889500.11112118,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 142880303.22222134 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "4567d9d360c3e33ce2e7849dbb802005b9df0585",
          "message": "Expand performance dashboard with DART 7 solver surfaces and readable labels (#2878)",
          "timestamp": "2026-06-04T06:12:30-07:00",
          "tree_id": "1ce07b78af754cde67cb9334319e7130e94c1387",
          "url": "https://github.com/dartsim/dart/commit/4567d9d360c3e33ce2e7849dbb802005b9df0585"
        },
        "date": 1780580361091,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "AVBD fixed-joint step · 1 links",
            "value": 19436.7668353928,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19434.87467462992 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 8 links",
            "value": 398806.39863633265,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 398775.10637813207 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 32 links",
            "value": 3983003.2172145885,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3968861.050139277 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 2 cells",
            "value": 454664.5215477054,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 454582.63458466466 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 8 cells",
            "value": 6079935.093727359,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6077040.229787235 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 24 cells",
            "value": 18898564.86054163,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18895642.52777778 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 48 cells",
            "value": 52103747.09956038,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52096989.933333404 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 32 parents · 8 children/parent",
            "value": 479951.43689734,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 479639.2062269835 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 8 children/parent",
            "value": 4626526.435128074,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4624548.876623374 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 32 children/parent",
            "value": 54983188.75906989,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54892908.40000003 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 32 parents · 8 children/parent",
            "value": 468765.7941513993,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 468276.5926919214 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 8 children/parent",
            "value": 4584093.619573128,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4582805.432786877 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 32 children/parent",
            "value": 54626760.23831591,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54606772.99999986 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 32 parents · 8 children/parent",
            "value": 1786504.4783605032,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 902767.82102462 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 8 children/parent",
            "value": 5479427.702586439,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5412385.267241369 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 32 children/parent",
            "value": 47610535.689806245,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47282818.72413794 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 128 bodies",
            "value": 108198.71279536611,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 108195.23684004044 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 1024 bodies",
            "value": 1086603.0479589736,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1086537.6133024015 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 4096 bodies",
            "value": 8263294.192913331,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8262781.619883032 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 128 bodies",
            "value": 1058288.4654454326,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 399458.50918975 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 1024 bodies",
            "value": 2486215.739146523,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2346537.8330434593 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 4096 bodies",
            "value": 13150435.585532807,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 12146161.729729712 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 16 iterations",
            "value": 41872.959193465605,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 41870.13476345969 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 4096 bodies · 16 iterations",
            "value": 162982.00967278282,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 162976.13320125718 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 64 iterations",
            "value": 171460.43952521044,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 171456.21969974364 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 16 iterations",
            "value": 60931.39099946711,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12546.066510000031 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 4096 bodies · 16 iterations",
            "value": 199399.19180993456,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 17573.370130000116 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 64 iterations",
            "value": 207845.30730377804,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 30876.358832143607 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 321653.18178113323,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 321271.3213867413 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 642011.6017527339,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 641697.1076993627 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 1286972.565657679,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1286462.6382001752 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 141959.3450933219,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 13143.989317953055 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 270524.412167448,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 17342.241322137794 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 810947.0834024249,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 25899.319900000246 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body batch (CPU baseline) · 1024 worlds · 128 bodies · 10 steps",
            "value": 200400154.99883643,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 200395173.66666594 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 1 boxes",
            "value": 3852.4272945308117,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3886.3360928315046 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 2 boxes",
            "value": 4970.402411727091,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5005.348433843406 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 4 boxes",
            "value": 7335.345892303959,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7372.897689717299 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 1 boxes",
            "value": 633676757.4830446,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 633663585.0000025 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 2 boxes",
            "value": 533881865.01494294,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 533693998.66666836 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 4 boxes",
            "value": 1064635690.0106183,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1064601688.9999999 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 8×8 grid",
            "value": 3106981.524979347,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3106879.2604166674 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 16×16 grid",
            "value": 22459180.880105123,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 22458340.779999997 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 24×24 grid",
            "value": 40812757.16360489,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 40810377.43636367 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 8×8 grid",
            "value": 41749.871861351276,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 41746.50180204337 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 16×16 grid",
            "value": 187631.47759026592,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 187618.56763285052 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 24×24 grid",
            "value": 435975.1921028413,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 435943.34939384524 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "78698081d5d3fad4ac22c948fb2ece43c070b786",
          "message": "Add allocator comparative benchmark gate (#2870)",
          "timestamp": "2026-06-04T06:21:54-07:00",
          "tree_id": "3ffdf26fa21ec7ccd368ef9fcb9c064d37633d8c",
          "url": "https://github.com/dartsim/dart/commit/78698081d5d3fad4ac22c948fb2ece43c070b786"
        },
        "date": 1780582425769,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "AVBD fixed-joint step · 1 links",
            "value": 21324.249542710382,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 21321.899533565025 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 8 links",
            "value": 457564.51309038285,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 457540.66747638327 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 32 links",
            "value": 4694884.298657672,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4694663.030201341 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 2 cells",
            "value": 528645.7541667271,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 528586.32125 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 8 cells",
            "value": 6682414.586854245,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6681968.971830987 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 24 cells",
            "value": 19707019.557141133,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19705529.771428563 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 48 cells",
            "value": 50164252.68750169,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50161182.21874999 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 32 parents · 8 children/parent",
            "value": 500774.8963175307,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 500726.338577047 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 8 children/parent",
            "value": 5049063.682142625,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5048809.485714284 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 32 children/parent",
            "value": 60457249.17391018,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60453449.260869496 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 32 parents · 8 children/parent",
            "value": 500531.6533238162,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 500498.41136526153 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 8 children/parent",
            "value": 5032872.741007228,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5032366.974820145 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 32 children/parent",
            "value": 60450482.130432725,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 60447927.347826116 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 32 parents · 8 children/parent",
            "value": 621918.5277540785,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 591490.5764304033 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 8 children/parent",
            "value": 4572620.958064392,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4477417.641935485 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 32 children/parent",
            "value": 50122686.85714259,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49823267.57142859 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 128 bodies",
            "value": 111732.54814695397,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 111720.42335707993 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 1024 bodies",
            "value": 1173210.0184562856,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1173114.3531879222 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 4096 bodies",
            "value": 8829808.088607127,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8829289.06329117 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 128 bodies",
            "value": 209010.64396284625,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 191211.0808991784 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 1024 bodies",
            "value": 1449560.3294797717,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1355497.7552986594 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 4096 bodies",
            "value": 9363969.27096769,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9022400.690322513 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 16 iterations",
            "value": 50029.40198813041,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 50027.73042265606 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 4096 bodies · 16 iterations",
            "value": 198595.44058423847,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 198587.8004821333 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 64 iterations",
            "value": 203119.91270304177,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 203115.9911542907 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 16 iterations",
            "value": 66326.06721713247,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12538.537572465868 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 4096 bodies · 16 iterations",
            "value": 218147.90805536532,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15912.723564783346 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 64 iterations",
            "value": 224229.47217681332,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 22714.462956908523 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 388481.8285158432,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 388456.74874930753 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 778560.4196776409,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 778532.3168426884 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 1553863.0422222216,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1553773.6022222268 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 121213.86342000052,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5049.779559999905 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 230419.80950999915,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6375.881700000008 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 439583.82223999937,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8698.978280000063 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body batch (CPU baseline) · 1024 worlds · 128 bodies · 10 steps",
            "value": 138043876.2222285,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 138027400.11111167 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 1 boxes",
            "value": 3826.2252248454683,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3879.5386410209417 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 2 boxes",
            "value": 4951.475494975423,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5000.653045562162 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 4 boxes",
            "value": 7543.319408012993,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7596.172903578435 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 1 boxes",
            "value": 708147440.9999373,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 708120821.4999996 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 2 boxes",
            "value": 581234599.9999025,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 581189297.9999982 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 4 boxes",
            "value": 1131746375.000148,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1131682694.999995 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 8×8 grid",
            "value": 3268421.0549447653,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3268285.635164835 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 16×16 grid",
            "value": 19609589.46000119,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19608667.399999995 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 24×24 grid",
            "value": 44287503.89873337,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 44276853.4556962 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 8×8 grid",
            "value": 44486.07809214274,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 44484.4471420378 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 16×16 grid",
            "value": 199109.40051311252,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 199098.6661915623 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 24×24 grid",
            "value": 465506.54140129784,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 465496.21186724707 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "76ba4f97872242e0a090916cda93187bc9776501",
          "message": "DART 7 PLAN-041 WS2: experimental package shape (install allowlist + private EnTT/Taskflow + default-on) (#2867)",
          "timestamp": "2026-06-04T08:26:35-07:00",
          "tree_id": "9b7d66c1422122a1c6b083de164e9d64c44006a7",
          "url": "https://github.com/dartsim/dart/commit/76ba4f97872242e0a090916cda93187bc9776501"
        },
        "date": 1780589646783,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "AVBD fixed-joint step · 1 links",
            "value": 19519.487618030296,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19513.88930910851 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 8 links",
            "value": 400462.3435836376,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 400427.5121681416 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 32 links",
            "value": 4259697.515996831,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4247636.3903743345 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 2 cells",
            "value": 481153.98193139985,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 481099.1875000002 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 8 cells",
            "value": 6404399.166617972,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6403824.632478632 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 24 cells",
            "value": 23791632.135556538,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23787734.847457636 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 48 cells",
            "value": 53365666.74980401,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53286172.74999996 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 32 parents · 8 children/parent",
            "value": 471298.6774782446,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 471253.82854278083 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 8 children/parent",
            "value": 4589077.236023962,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4588528.295081965 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 32 children/parent",
            "value": 55851828.34497223,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55840133.230769284 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 32 parents · 8 children/parent",
            "value": 487768.0898534614,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 487507.4771941945 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 8 children/parent",
            "value": 4555137.148967309,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4554729.847682112 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 32 children/parent",
            "value": 54810783.681459725,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54804840.91999998 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 32 parents · 8 children/parent",
            "value": 1819727.0026214619,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 919028.157615894 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 8 children/parent",
            "value": 6578547.22273906,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6495956.917030563 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 32 children/parent",
            "value": 61578553.18054896,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 61242385.18181842 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 128 bodies",
            "value": 109690.28477072933,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 109686.41590187036 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 1024 bodies",
            "value": 1095196.6321061107,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1095149.2941653158 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 4096 bodies",
            "value": 8717316.20353796,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8716335.734567925 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 128 bodies",
            "value": 896258.7482504257,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 396090.85096828523 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 1024 bodies",
            "value": 2747404.14840835,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2621722.9062499846 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 4096 bodies",
            "value": 14176794.56062615,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 13047122.604395578 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 16 iterations",
            "value": 42010.5827621703,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42007.64950096222 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 4096 bodies · 16 iterations",
            "value": 164410.31772238633,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 164368.8215504064 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 64 iterations",
            "value": 173116.75737250017,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 173080.6806121196 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 16 iterations",
            "value": 63515.546582851755,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 13587.08524936963 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 4096 bodies · 16 iterations",
            "value": 204318.6646467994,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 17460.586535047285 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 64 iterations",
            "value": 215414.8854094206,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 35052.31157087064 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 324615.0289634552,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 324605.40640452004 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 650321.8099666637,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 650303.5691662821 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 1297664.3702467612,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1297624.1532779348 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 202392.69892350168,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 13883.016905849146 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 416640.4541775328,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 19644.18551499758 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 820111.6509968415,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 26347.20369999996 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body batch (CPU baseline) · 1024 worlds · 128 bodies · 10 steps",
            "value": 268387297.59911072,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 268295216.00000134 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 1 boxes",
            "value": 4238.5163461514185,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4271.7400928242005 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 2 boxes",
            "value": 4943.542818307388,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4977.809774798702 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 4 boxes",
            "value": 8430.53365709708,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8466.701733523949 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 1 boxes",
            "value": 731806716.9981078,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 731575260.9999996 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 2 boxes",
            "value": 543528192.6572012,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 543507643.3333338 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 4 boxes",
            "value": 1089236269.9960358,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1089186355.999999 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 8×8 grid",
            "value": 3639201.661237443,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3638750.203252034 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 16×16 grid",
            "value": 18904461.40023414,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18898644.220000003 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 24×24 grid",
            "value": 51726084.12279148,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 51697727.76712329 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 8×8 grid",
            "value": 42845.7349846292,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 42835.84792944516 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 16×16 grid",
            "value": 188076.09763957313,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 188046.62160702256 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 24×24 grid",
            "value": 439137.38294824166,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 439121.2926982129 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ef082df6f68cd3e53aa8a5033010a075a7436f86",
          "message": "Add rigid-body one-DOF joint facades (#2891)",
          "timestamp": "2026-06-04T23:54:02Z",
          "tree_id": "473a625e8d74ee6c14e9b3154f32142755c2cfe4",
          "url": "https://github.com/dartsim/dart/commit/ef082df6f68cd3e53aa8a5033010a075a7436f86"
        },
        "date": 1780621765603,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "AVBD fixed-joint step · 1 links",
            "value": 23613.937139471735,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23612.343618370975 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 8 links",
            "value": 489704.97509196936,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 489633.28955561854 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 32 links",
            "value": 4833935.772413842,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4833251.444827587 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 2 cells",
            "value": 509827.9081258083,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 509773.07962433645 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 8 cells",
            "value": 6723944.745283246,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6723070.179245285 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 24 cells",
            "value": 19965001.6714291,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19962016.171428587 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 48 cells",
            "value": 54196650.82758754,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 54191671.137931116 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 32 parents · 8 children/parent",
            "value": 486563.9131643164,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 486491.56477943715 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 8 children/parent",
            "value": 4755853.377550735,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4754972.374149661 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 32 children/parent",
            "value": 56272430.440003514,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 56263657.76 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 32 parents · 8 children/parent",
            "value": 477236.8962810926,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 477194.8785397478 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 8 children/parent",
            "value": 4713299.033783403,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4712839.233108102 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 32 children/parent",
            "value": 55867274.000002,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 55862483.55999998 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 32 parents · 8 children/parent",
            "value": 598843.7368421423,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 567992.1864202487 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 8 children/parent",
            "value": 4554674.6242039865,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4451549.296178357 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 32 children/parent",
            "value": 49857333.28571281,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49554651.14285699 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 128 bodies",
            "value": 114791.69243314756,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 114777.6060013043 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 1024 bodies",
            "value": 1253580.014159259,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1253401.4327433652 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 4096 bodies",
            "value": 9757598.895832743,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9756741.826388884 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 128 bodies",
            "value": 214052.15100853948,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 195320.31873445687 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 1024 bodies",
            "value": 1554666.174418567,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1464961.653276956 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 4096 bodies",
            "value": 10187372.158273863,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 9861348.007194305 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 16 iterations",
            "value": 55532.72598556395,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 55527.84242880952 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 4096 bodies · 16 iterations",
            "value": 220072.1039798486,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 220050.21409469898 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 64 iterations",
            "value": 224441.30000000785,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 224421.40577849012 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 16 iterations",
            "value": 77946.18279785274,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12205.595111463163 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 4096 bodies · 16 iterations",
            "value": 257835.53981985766,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14100.376245483742 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 64 iterations",
            "value": 264314.21845788887,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21781.29220629216 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 433679.0201300828,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433632.5236915464 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 867681.6317419858,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 867640.605083697 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 1734356.4733580363,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1734165.8847583588 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 138436.1501400008,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5363.580850000034 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 253777.812620001,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6783.363050000019 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 482792.1996400005,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9138.937470000032 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body batch (CPU baseline) · 1024 worlds · 128 bodies · 10 steps",
            "value": 151393292.2500203,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 151387119.62500116 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 1 boxes",
            "value": 4010.9631170626744,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4054.127705659399 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 2 boxes",
            "value": 5113.7547326377835,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5169.758548105267 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 4 boxes",
            "value": 7390.756731922134,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7443.4967488089505 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 1 boxes",
            "value": 775192088.9999156,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 775128815.0000005 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 2 boxes",
            "value": 631029121.499978,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 630914330.5000049 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 4 boxes",
            "value": 1244206753.9998333,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1244068972.000001 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 8×8 grid",
            "value": 3444976.542529218,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3444576.606896552 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 16×16 grid",
            "value": 20625799.39999978,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 20623438.719999995 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 24×24 grid",
            "value": 45657399.46052884,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 45651411.28947366 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 8×8 grid",
            "value": 45744.410137345636,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 45739.4156638326 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 16×16 grid",
            "value": 205184.1553895597,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 205153.19566490894 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 24×24 grid",
            "value": 478726.6129142232,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 478676.89921421226 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "be525675a0cec39cb63c0e8a90d66da42aefb31f",
          "message": "Validate HeightmapShape scale and height field inputs (DART 7) (#2886)",
          "timestamp": "2026-06-05T10:04:13Z",
          "tree_id": "493d3b0cf9a2f19c8c66a64a75fcb9ef872fcd9a",
          "url": "https://github.com/dartsim/dart/commit/be525675a0cec39cb63c0e8a90d66da42aefb31f"
        },
        "date": 1780659336332,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "AVBD fixed-joint step · 1 links",
            "value": 19340.52028928175,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19340.085533829617 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 8 links",
            "value": 404429.4369387903,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 404372.4651675485 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 32 links",
            "value": 3749746.9920024113,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3748984.9516129014 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 2 cells",
            "value": 459215.77968374593,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 458737.95843422116 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 8 cells",
            "value": 5999063.814795528,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5997558.064655169 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 24 cells",
            "value": 18611057.123252228,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18606514.068493146 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 48 cells",
            "value": 52070609.838401355,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 52063494.67741933 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 32 parents · 8 children/parent",
            "value": 435465.1564109864,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 435353.5071517413 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 8 children/parent",
            "value": 4046626.418971623,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4046222.089595374 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 32 children/parent",
            "value": 47291593.24076668,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47284998.86206897 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 32 parents · 8 children/parent",
            "value": 437145.38569327124,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 437127.5146783267 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 8 children/parent",
            "value": 4080612.199934389,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4078899.443478259 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 32 children/parent",
            "value": 47253400.82799888,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47246375.06896556 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 32 parents · 8 children/parent",
            "value": 1815861.9871231627,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 895653.8414948466 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 8 children/parent",
            "value": 5562007.553592611,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5500732.671140936 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 32 children/parent",
            "value": 44485675.12554292,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 44257874.96874989 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 128 bodies",
            "value": 109064.65770329906,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 109019.71830654673 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 1024 bodies",
            "value": 1083488.8770113203,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1083283.8623356505 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 4096 bodies",
            "value": 8468904.545544786,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8468293.17575757 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 128 bodies",
            "value": 990328.4473263008,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 386382.1315489744 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 1024 bodies",
            "value": 2550020.422481836,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2372546.3084112094 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 4096 bodies",
            "value": 12410860.752312409,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 11409557.36082472 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 16 iterations",
            "value": 42197.87472673949,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 42186.76997274071 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 4096 bodies · 16 iterations",
            "value": 164127.671468391,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 164115.12328767238 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 64 iterations",
            "value": 171357.31916687422,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 171352.25844759998 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 16 iterations",
            "value": 58364.818989648484,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 9696.170120000004 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 4096 bodies · 16 iterations",
            "value": 190553.54943387487,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14310.155292051772 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 64 iterations",
            "value": 200735.51630998668,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 26876.120872758882 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 320662.6605683901,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 320654.41297396057 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 640166.6437347446,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 640154.2031178351 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 1279611.0282660571,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1279581.136736551 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 122536.79586171378,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 12023.69122898071 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 285122.04254115093,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 17204.692154093496 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 706463.0312961525,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23170.503999999422 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body batch (CPU baseline) · 1024 worlds · 128 bodies · 10 steps",
            "value": 180121567.14186338,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 180116120.57142863 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 1 boxes",
            "value": 3816.231766474529,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3841.3191542958516 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 2 boxes",
            "value": 4848.950667282202,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4883.616908581941 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 4 boxes",
            "value": 7172.454411344656,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7207.535721290061 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 1 boxes",
            "value": 624878797.4764127,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 624865044.9999999 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 2 boxes",
            "value": 518683407.9989543,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 518610441.33333206 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 4 boxes",
            "value": 1017566714.0283622,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1017533927.0000024 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 8×8 grid",
            "value": 3077866.0553461704,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3077661.102459017 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 16×16 grid",
            "value": 18569493.98003053,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18567115.770000003 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 24×24 grid",
            "value": 42633516.07990141,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 42631123.293333344 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 8×8 grid",
            "value": 41518.379786396785,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 41515.56878746508 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 16×16 grid",
            "value": 186991.12213543776,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 186986.40374581958 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 24×24 grid",
            "value": 434543.7293459761,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 434523.61049037805 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "1a5469960c2703accb2762e03fe8a6bb1156dc08",
          "message": "Add opt-in experimental World simulation replay recording (#2876)",
          "timestamp": "2026-06-05T04:32:24-07:00",
          "tree_id": "fd4dbadadcce3bfb6ab17b1f0ed34581b14bc40d",
          "url": "https://github.com/dartsim/dart/commit/1a5469960c2703accb2762e03fe8a6bb1156dc08"
        },
        "date": 1780672197342,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "AVBD fixed-joint step · 1 links",
            "value": 19092.501325215075,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19091.931145167455 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 8 links",
            "value": 402740.51531672623,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 402715.92895937414 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 32 links",
            "value": 3643424.6223204047,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3642957.9062500033 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 2 cells",
            "value": 445583.62195304787,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 445546.3639215685 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 8 cells",
            "value": 5928156.945318031,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5928015.151898733 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 24 cells",
            "value": 18321459.379040863,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18320930.918918908 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 48 cells",
            "value": 46769055.52656131,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 46763368.86111117 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 32 parents · 8 children/parent",
            "value": 446273.6869337642,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 446037.87420382164 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 8 children/parent",
            "value": 4195536.192270225,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4194130.1411411376 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 32 children/parent",
            "value": 49326183.310517214,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49314280.44827586 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 32 parents · 8 children/parent",
            "value": 444884.91673048754,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 444755.44843049394 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 8 children/parent",
            "value": 4200987.946025994,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4200343.242514967 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 32 children/parent",
            "value": 49353521.06835565,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49337942.068965495 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 32 parents · 8 children/parent",
            "value": 1794862.2482161,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 893918.2302250805 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 8 children/parent",
            "value": 5566809.656546574,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5493331.426415102 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 32 children/parent",
            "value": 46698603.23192552,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 46487917.06666652 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 128 bodies",
            "value": 112982.8970193197,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 112929.13252818036 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 1024 bodies",
            "value": 1352712.9777899536,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1352678.6245173768 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 4096 bodies",
            "value": 11980308.129468225,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 11979829.017241368 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 128 bodies",
            "value": 1091625.7318159975,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 422707.79627507227 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 1024 bodies",
            "value": 3040903.0639086417,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2954986.4020618335 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 4096 bodies",
            "value": 16459718.775149668,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 15401289.82500004 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 16 iterations",
            "value": 41737.04773947503,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 41723.62645197733 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 4096 bodies · 16 iterations",
            "value": 162672.9820680366,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 162665.79203539735 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 64 iterations",
            "value": 170103.86289281095,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 170099.2322839434 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 16 iterations",
            "value": 53502.20092176875,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 8303.710457948251 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 4096 bodies · 16 iterations",
            "value": 182701.58853255433,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12194.535767543479 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 64 iterations",
            "value": 186412.135501566,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 19312.272926146146 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 319208.70707160863,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 319146.94450761715 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 635563.1630339039,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 635415.3165304281 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 1277259.7770398997,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1277104.4049135651 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 166009.70862605507,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 12849.538883402538 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 232697.96768539923,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 14944.968794213943 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 603156.8203994539,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 20596.3085999997 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body batch (CPU baseline) · 1024 worlds · 128 bodies · 10 steps",
            "value": 172796018.86236507,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 172789367.1428567 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 1 boxes",
            "value": 3693.33383505359,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3729.0428079357007 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 2 boxes",
            "value": 4780.522297415284,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4813.474569315204 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 4 boxes",
            "value": 7111.832656568182,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7150.990500874826 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 1 boxes",
            "value": 618737967.510242,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 618681373.4999994 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 2 boxes",
            "value": 518724577.65663666,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 518712713.3333327 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 4 boxes",
            "value": 1032607368.0189438,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1032571941.0000076 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 8×8 grid",
            "value": 3087331.40734032,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3087206.530864198 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 16×16 grid",
            "value": 18433051.63027253,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18432290.01 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 24×24 grid",
            "value": 44646250.53128369,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 44637863.215189874 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 8×8 grid",
            "value": 41379.159606379515,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 41375.63827717341 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 16×16 grid",
            "value": 186019.7254893578,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 186007.64878637527 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 24×24 grid",
            "value": 432044.1243864912,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 431880.91367574286 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "52683364ae3c89895ab30863419682d4c53cf0d3",
          "message": "Add fixed-capacity free-list allocator mode (#2892)",
          "timestamp": "2026-06-05T06:58:29-07:00",
          "tree_id": "1517c683bfcd5f42c0dd2d629eb5be39ef1dde51",
          "url": "https://github.com/dartsim/dart/commit/52683364ae3c89895ab30863419682d4c53cf0d3"
        },
        "date": 1780677582166,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "AVBD fixed-joint step · 1 links",
            "value": 19127.80178982468,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19127.135268600112 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 8 links",
            "value": 401528.00195966335,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 401173.16550218355 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 32 links",
            "value": 3654399.2286027926,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3653371.007792208 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 2 cells",
            "value": 447590.8794525989,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 447577.5964774951 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 8 cells",
            "value": 5962451.759535222,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5962211.561181436 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 24 cells",
            "value": 18572549.177780237,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18570736.561643828 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 48 cells",
            "value": 47862805.48432842,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 47835450.8571429 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 32 parents · 8 children/parent",
            "value": 426766.80751291144,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 426440.5743965782 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 8 children/parent",
            "value": 3896092.245138632,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3893445.635097491 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 32 children/parent",
            "value": 44654131.54778822,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 44625886.77419355 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 32 parents · 8 children/parent",
            "value": 424068.3265202168,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 423995.141158537 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 8 children/parent",
            "value": 3885592.538664035,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3885417.3397790077 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 32 children/parent",
            "value": 44218471.937711,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 44217070.531250015 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 32 parents · 8 children/parent",
            "value": 1761835.8887789892,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 847446.4284023661 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 8 children/parent",
            "value": 4546129.236853843,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4507244.172932333 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 32 children/parent",
            "value": 42166110.81783688,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 41972838.727272734 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 128 bodies",
            "value": 109079.2321976787,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 109072.16471871611 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 1024 bodies",
            "value": 1096204.120875592,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1096090.5423861842 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 4096 bodies",
            "value": 8244843.941236682,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8244531.194117646 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 128 bodies",
            "value": 989054.0363575411,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 371110.7327966764 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 1024 bodies",
            "value": 2412325.8356393273,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2222824.863321798 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 4096 bodies",
            "value": 11829056.95670092,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 10797684.69827587 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 16 iterations",
            "value": 42022.491518096685,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 41996.335980580036 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 4096 bodies · 16 iterations",
            "value": 162799.02221624533,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 162795.00068704877 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 64 iterations",
            "value": 168647.23380716844,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 168636.77448469383 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 16 iterations",
            "value": 56362.52226948273,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 9683.12449999999 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 4096 bodies · 16 iterations",
            "value": 173509.19716001954,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 8565.28188999988 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 64 iterations",
            "value": 181869.85515417438,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 17081.700139259327 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 316534.7444482855,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 316365.665609423 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 641498.5446753411,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 641270.5300045774 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 1266533.0253619922,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1266501.0181159535 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 110379.39299631909,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 10319.56331293029 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 266420.81302980304,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 14574.061594239522 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 592539.798299549,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 20528.584299999864 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body batch (CPU baseline) · 1024 worlds · 128 bodies · 10 steps",
            "value": 173519367.87599698,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 173505765.9999999 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 1 boxes",
            "value": 4148.292409948969,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4180.974928634498 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 2 boxes",
            "value": 4813.62357550332,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4850.72955727645 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 4 boxes",
            "value": 7157.511762811598,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7191.688715771826 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 1 boxes",
            "value": 624858563.0091839,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 624763444.4999992 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 2 boxes",
            "value": 524906266.98515064,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 524888451.3333323 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 4 boxes",
            "value": 1073268509.9937953,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1073232916.000002 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 8×8 grid",
            "value": 3105287.205958547,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3105012.0343249426 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 16×16 grid",
            "value": 18441334.110102616,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18440424.069999997 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 24×24 grid",
            "value": 43955293.63108308,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 43927069.736842126 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 8×8 grid",
            "value": 41704.363271058246,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 41693.945266360424 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 16×16 grid",
            "value": 188011.98787799085,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 187934.96188552174 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 24×24 grid",
            "value": 439344.105358487,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 439151.5989338355 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "24e4830070befda0c2cdf59646746adf4898b473",
          "message": "Add experimental World step profiling (#2883)",
          "timestamp": "2026-06-05T09:37:40-07:00",
          "tree_id": "2a7c344eaf97ffc69252e0f6d52fb475226fb641",
          "url": "https://github.com/dartsim/dart/commit/24e4830070befda0c2cdf59646746adf4898b473"
        },
        "date": 1780683413502,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "AVBD fixed-joint step · 1 links",
            "value": 20891.184309030756,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 20888.91459174767 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 8 links",
            "value": 398621.4639265158,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 398576.75136533496 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 32 links",
            "value": 4849002.3299313625,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4848513.836734691 ns\nthreads: 1"
          },
          {
            "name": "AVBD revolute-motor step · 1 motors",
            "value": 18061.936730409227,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18060.9894529185 ns\nthreads: 1"
          },
          {
            "name": "AVBD revolute-motor step · 8 motors",
            "value": 386941.9567987076,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 386927.4270285245 ns\nthreads: 1"
          },
          {
            "name": "AVBD revolute-motor step · 32 motors",
            "value": 4768345.594595144,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4767843.969594591 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 2 cells",
            "value": 519426.261190838,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 519329.2924024642 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 8 cells",
            "value": 6684488.785046775,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6683944.116822429 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 24 cells",
            "value": 19669546.676057916,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19667069.112676073 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 48 cells",
            "value": 50017610.96875157,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50011973.24999995 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 32 parents · 8 children/parent",
            "value": 458641.2074049839,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 458565.1382699869 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 8 children/parent",
            "value": 4371557.578124908,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4371388.637500001 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 32 children/parent",
            "value": 50653824.071430005,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 50647751.24999997 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 32 parents · 8 children/parent",
            "value": 145117.00529374715,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 145106.61677392563 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 8 children/parent",
            "value": 2088073.5827124796,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2087857.383010433 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 32 children/parent",
            "value": 27442530.99999964,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 27440291.68627451 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 32 parents · 8 children/parent",
            "value": 294112.77745993965,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 262428.6174675824 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 8 children/parent",
            "value": 2473348.1050847233,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2379222.611864406 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 32 children/parent",
            "value": 28470219.44000062,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 28118388.54000001 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 128 bodies",
            "value": 45804.68499159228,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 45800.38020644404 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 1024 bodies",
            "value": 362332.75712806405,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 362286.00414722675 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 4096 bodies",
            "value": 1463025.6781250257,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1462873.1354166649 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 128 bodies",
            "value": 112480.4109255257,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 95851.1904662489 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 1024 bodies",
            "value": 678342.3472786815,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 580246.2546710029 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 4096 bodies",
            "value": 2379837.617688908,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2007380.914407993 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 16 iterations",
            "value": 48833.18805179099,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 48831.118958718405 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 4096 bodies · 16 iterations",
            "value": 197153.8583098715,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 197139.24140845053 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 64 iterations",
            "value": 195539.81198489657,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 195522.6158681373 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 16 iterations",
            "value": 67288.11278840862,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12652.345241316238 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 4096 bodies · 16 iterations",
            "value": 218958.6552987661,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 15864.811596866362 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 64 iterations",
            "value": 225751.69070711828,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 23629.10683961475 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 387900.36796891916,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 387861.75865890604 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 777457.5288568028,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 777380.3157602703 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 1551691.9922309124,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1551553.1731409451 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 121234.01864000015,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 4826.542169999897 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 229142.79069000029,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6370.789289999976 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 432512.09840999986,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 8493.292289999914 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body batch (CPU baseline) · 1024 worlds · 128 bodies · 10 steps",
            "value": 144974255.29998508,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 144962119.60000095 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 1 boxes",
            "value": 2572.8544778303726,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2617.427820261871 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 2 boxes",
            "value": 3325.3146917352055,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3367.649179640169 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 4 boxes",
            "value": 4835.541902061631,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4880.141881821869 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 1 boxes",
            "value": 709957224.4999307,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 709879040.9999989 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 2 boxes",
            "value": 584690664.9999255,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 584665020.9999993 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 4 boxes",
            "value": 1160763132.9998186,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1160619945.9999971 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 8×8 grid",
            "value": 3366903.253932383,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3366603.932584271 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 16×16 grid",
            "value": 19894906.499998797,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19893346.999999996 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 24×24 grid",
            "value": 45087831.37974586,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 45081960.84810126 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 8×8 grid",
            "value": 44496.78155585422,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 44494.92026820887 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 16×16 grid",
            "value": 199268.0484882859,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 199248.5787221902 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 24×24 grid",
            "value": 466970.1564013997,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 466905.2633217993 ns\nthreads: 1"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "jslee02@users.noreply.github.com",
            "name": "Jeongseok (JS) Lee",
            "username": "jslee02"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "894e278f51ff5f8110cb3ed2b11ed1b311dad165",
          "message": "Preflight zero-DOF multibody contact fallback (#2906)",
          "timestamp": "2026-06-05T11:07:28-07:00",
          "tree_id": "87806dfc6eaba48b35b6ac6f319899d051f9a043",
          "url": "https://github.com/dartsim/dart/commit/894e278f51ff5f8110cb3ed2b11ed1b311dad165"
        },
        "date": 1780690687276,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "AVBD fixed-joint step · 1 links",
            "value": 22747.125708608877,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 22744.066185183998 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 8 links",
            "value": 419706.46756350476,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 419671.27742899844 ns\nthreads: 1"
          },
          {
            "name": "AVBD fixed-joint step · 32 links",
            "value": 4875462.679310345,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4875129.093103448 ns\nthreads: 1"
          },
          {
            "name": "AVBD revolute-motor step · 1 motors",
            "value": 18757.79554406408,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 18755.88795852202 ns\nthreads: 1"
          },
          {
            "name": "AVBD revolute-motor step · 8 motors",
            "value": 402270.0572167849,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 402220.94680851046 ns\nthreads: 1"
          },
          {
            "name": "AVBD revolute-motor step · 32 motors",
            "value": 4847514.451724018,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4846800.041379311 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 2 cells",
            "value": 502251.224448894,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 502218.4501002005 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 8 cells",
            "value": 6664603.869767445,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6663377.9674418615 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 24 cells",
            "value": 19733960.04285795,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 19732356.17142858 ns\nthreads: 1"
          },
          {
            "name": "FEM bar step · 48 cells",
            "value": 49367429.617652364,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 49360730.99999995 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 32 parents · 8 children/parent",
            "value": 464811.02641094034,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 464731.14218523883 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 8 children/parent",
            "value": 4502513.93870956,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 4501957.667741938 ns\nthreads: 1"
          },
          {
            "name": "Kinematics update · 128 parents · 32 children/parent",
            "value": 53361519.42307985,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 53356368.34615388 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 32 parents · 8 children/parent",
            "value": 145482.0464319101,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 145463.86537862272 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 8 children/parent",
            "value": 2097062.8847304604,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2096872.6541916153 ns\nthreads: 1"
          },
          {
            "name": "World step (sequential) · 128 parents · 32 children/parent",
            "value": 27443367.509802416,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 27441205.058823522 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 32 parents · 8 children/parent",
            "value": 299126.80223461025,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 268902.9301675979 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 8 children/parent",
            "value": 2509300.271944783,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2421530.4371772804 ns\nthreads: 1"
          },
          {
            "name": "World step (parallel) · 128 parents · 32 children/parent",
            "value": 28292585.619997222,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 28007662.160000134 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 128 bodies",
            "value": 48261.86184005869,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 48256.41537876434 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 1024 bodies",
            "value": 381883.634199692,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 381812.1956224346 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (sequential) · 4096 bodies",
            "value": 1544193.5263158202,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1543977.338815786 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 128 bodies",
            "value": 109589.20602850521,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 90554.06512357773 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 1024 bodies",
            "value": 701446.0169639116,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 611391.2031317983 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body step (parallel) · 4096 bodies",
            "value": 2411465.215976402,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2084287.6568047337 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 16 iterations",
            "value": 54340.54303811566,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 54336.479617611214 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 4096 bodies · 16 iterations",
            "value": 219229.33557676015,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 219211.3994365309 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (sequential) · 1024 bodies · 64 iterations",
            "value": 217401.3829787273,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 217383.81208262217 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 16 iterations",
            "value": 77851.05615996722,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 12062.687912276917 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 4096 bodies · 16 iterations",
            "value": 257715.58196754844,
            "unit": "ns/iter",
            "extra": "iterations: 16\ncpu: 14116.286233289073 ns\nthreads: 1"
          },
          {
            "name": "Contact-shaped proxy (parallel) · 1024 bodies · 64 iterations",
            "value": 265635.02231805155,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 21473.922605537842 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 433189.3944925689,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 433155.9186262391 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 865465.7206426823,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 865383.9140914773 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (sequential) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 1731138.9999999034,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 1730998.1559406014 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 4 islands · 512 bodies/island · 64 iterations",
            "value": 138464.1591200011,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 5285.040059999915 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 8 islands · 512 bodies/island · 64 iterations",
            "value": 254831.31039999987,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 6574.532270000048 ns\nthreads: 1"
          },
          {
            "name": "Contact-island proxy (parallel) · 16 islands · 512 bodies/island · 64 iterations",
            "value": 491311.0324599983,
            "unit": "ns/iter",
            "extra": "iterations: 64\ncpu: 9191.565059999932 ns\nthreads: 1"
          },
          {
            "name": "Rigid-body batch (CPU baseline) · 1024 worlds · 128 bodies · 10 steps",
            "value": 143056330.29998717,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 143037394.09999993 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 1 boxes",
            "value": 2227.6401639430696,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2270.202396688054 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 2 boxes",
            "value": 2786.791285069063,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2837.650164390976 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (sequential impulse) · 4 boxes",
            "value": 3839.5625625926677,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3877.8983250965443 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 1 boxes",
            "value": 774897531.5000734,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 774820464.0000011 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 2 boxes",
            "value": 629983252.9999776,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 629886754.5000011 ns\nthreads: 1"
          },
          {
            "name": "Rigid world step (IPC barrier) · 4 boxes",
            "value": 1234610500.0001442,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1234485150.999994 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 8×8 grid",
            "value": 3432220.735631985,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3431912.434482759 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 16×16 grid",
            "value": 20638702.78999957,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 20636779.339999992 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (default solver) · 24×24 grid",
            "value": 45664492.14473778,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 45659722.05263158 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 8×8 grid",
            "value": 46097.38089444457,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 46093.09917790204 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 16×16 grid",
            "value": 204961.52354491907,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 204942.18587306255 ns\nthreads: 1"
          },
          {
            "name": "Deformable world step (VBD) · 24×24 grid",
            "value": 479721.2001372887,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 479682.6869207006 ns\nthreads: 1"
          }
        ]
      }
    ]
  }
}