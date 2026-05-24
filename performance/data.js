window.BENCHMARK_DATA = {
  "lastUpdate": 1779589400026,
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
      }
    ]
  }
}