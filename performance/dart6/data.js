window.BENCHMARK_DATA = {
  "lastUpdate": 1783133462901,
  "repoUrl": "https://github.com/dartsim/dart",
  "entries": {
    "DART 6 Performance": [
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
          "id": "b1a127e303e959bc02f200dddf17be51ebf6e0c1",
          "message": "Add DART 6 performance dashboard (#3230)",
          "timestamp": "2026-07-03T18:58:41-07:00",
          "tree_id": "4074815c32c166e0060121451aa7b8eb9dcea50c",
          "url": "https://github.com/dartsim/dart/commit/b1a127e303e959bc02f200dddf17be51ebf6e0c1"
        },
        "date": 1783133357768,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 66.0859964761931,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 66.07930628571428 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1190.5598770000552,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1190.5147519999994 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 13119.016774999976,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 13118.192047000002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 483.09105033335226,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 483.0658880000001 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 487.7036086666446,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 487.6253246666676 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 2003.7438279998696,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2003.588143 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 2006.9321219998528,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2006.7763429999986 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5929.284028999973,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5928.955062 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5924.692628000116,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5924.340424999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8053.910529000063,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8053.571784000013 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8046.164895000061,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8045.82489100001 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 7.420000045499365e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7.009999999996534e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.975752652561237,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.9756992649801526 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 6.101859698405192,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 6.101581089194389 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 13.015614275732247,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.01528921045857 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.6020863907063,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.601589618410404 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 34.201117523865584,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 34.20024520261059 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 129.52007382975103,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 129.51052177842044 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 25.05741737591236,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 25.056526516235163 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.64927754831742,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.648944094840964 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 18.67978561480457,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.67894966119402 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.696652677582593,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.696325041657893 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 18.222435711200607,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.22199234423462 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 31.174240056850426,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 31.173200861628583 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 89941.43368164496,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 89912.41528303099 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 712393.0983273793,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 712332.8185504308 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 6916908.882352439,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6916743.946078432 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 337839.2568740866,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 337828.98866377247 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2664544.2791586346,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2664393.8948374735 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 30350604.586954966,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 30349104.891304363 ns\nthreads: 1"
          }
        ]
      }
    ]
  }
}