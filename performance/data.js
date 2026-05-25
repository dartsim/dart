window.BENCHMARK_DATA = {
  "lastUpdate": 1779727173934,
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
      }
    ]
  }
}