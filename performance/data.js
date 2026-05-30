window.BENCHMARK_DATA = {
  "lastUpdate": 1780100249421,
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
      }
    ]
  }
}