window.BENCHMARK_DATA = {
  "lastUpdate": 1783149186514,
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
          "id": "a10786732ba5391d7c3c767862bb86a6874cabf0",
          "message": "Preserve release CI runs under concurrency (#3233)",
          "timestamp": "2026-07-03T19:33:15-07:00",
          "tree_id": "bd87cfac0b11cfc6d6e7aa290e4b58e1a012f331",
          "url": "https://github.com/dartsim/dart/commit/a10786732ba5391d7c3c767862bb86a6874cabf0"
        },
        "date": 1783148464671,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 71.3881485265263,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 71.3854124736842 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1318.1427590025123,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1318.1005829999997 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 18442.284329998074,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 18441.14281 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 454.51549933447194,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 454.300147666667 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 451.97491299768444,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 451.8548453333337 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1904.4250390143134,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1904.313403999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 2025.0518480170285,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2024.8372729999992 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5572.823635986424,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5572.680730999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5469.418766006129,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5468.391584000002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8882.541011014837,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8882.341132000007 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8859.507264991407,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8858.713619 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 6.659975042566658e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6.459999999999799e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.953446162873601,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.95293935709576 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.728073488216679,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.727014866907129 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.680243476263184,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.67673686681141 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.439163239729439,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.436549648946842 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.047660730174876,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.03147085409945 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 115.97422447355665,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 115.96355462324523 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 24.585655955895206,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 24.58100660606703 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.27772412639341,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.274706562176021 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.481222586846222,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.47926281619834 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.338667811030044,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.336200323957634 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 18.67047071536745,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.66845786640659 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 31.101020193513794,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 31.099226970119332 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 81997.1893597918,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 81941.84149143974 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 726482.0036380682,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 725832.832640333 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7247166.422624189,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7243284.551546394 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 343385.542076098,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 343269.8988681102 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2938319.2283293842,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2937666.211416489 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33651397.11899073,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33646850.83333338 ns\nthreads: 1"
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
          "id": "70b92010311f19a9363003bb699d3f1996004069",
          "message": "Scale deactivation final-quiet gate with configured thresholds (#3226)",
          "timestamp": "2026-07-03T19:09:19-07:00",
          "tree_id": "1fb8f1fc27d188e246cf97bef96b1d2efb91b63e",
          "url": "https://github.com/dartsim/dart/commit/70b92010311f19a9363003bb699d3f1996004069"
        },
        "date": 1783149021797,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 70.6307398948794,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 70.62877984210526 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1284.3641590006882,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1284.26799 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 18553.331233008066,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 18550.918771 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 457.5870219996432,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 457.46427200000016 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 459.7242733240516,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 459.37941766666677 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1944.5492849918082,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1943.7732700000013 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1951.7440980125684,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1951.2016249999995 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5504.9305699940305,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5504.556236000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5457.631487006438,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5456.786892000004 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8772.87974698993,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8771.371314000007 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8843.684278006549,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8825.445187000014 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 6.560003384947778e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6.459999999999799e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.9835561475621515,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.9830335451602914 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.643286162837271,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.643191146684889 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.439486043648216,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.438676897197178 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.17242108614123,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.172200373231474 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 32.574814392663846,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 32.57426009829164 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 117.20339667571824,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 117.20143385932523 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 24.478625536738452,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 24.478068642413515 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.098434880796672,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.09820841842987 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.423257656158295,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.422845345052973 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.184232209698022,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.183994460591235 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 19.250982620165527,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.250267027662755 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 31.54036371733814,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 31.53983462020271 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 81043.09051775308,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 81040.75074712643 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 715798.8461908406,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 715560.9699637498 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7246007.675292407,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7245440.8195876265 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 349167.57881300215,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 349112.3087011347 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2891958.7863106336,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2891505.4709543567 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33151404.071499456,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33146611.642857086 ns\nthreads: 1"
          }
        ]
      }
    ]
  }
}