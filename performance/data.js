window.BENCHMARK_DATA = {
  "lastUpdate": 1780267924570,
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
      }
    ]
  }
}