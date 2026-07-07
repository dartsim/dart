window.BENCHMARK_DATA = {
  "lastUpdate": 1783445451084,
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
          "id": "a450f918cfa2ce46ad22bcba0a5d46adddabe97e",
          "message": "Fix release SIMD trait on MSVC (#3245)",
          "timestamp": "2026-07-03T23:48:12-07:00",
          "tree_id": "d987bc9197dc0bd1e32b6267542395a7a9540a3a",
          "url": "https://github.com/dartsim/dart/commit/a450f918cfa2ce46ad22bcba0a5d46adddabe97e"
        },
        "date": 1783161239754,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 70.92197178947475,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 70.9185659473684 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1288.9214020001418,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1288.818172 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 14373.935450999852,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 14372.444629999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 523.3475079999153,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 523.3187770000001 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 525.0503953332858,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 524.982468666666 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 2135.2589709999847,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2135.0714189999976 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 2133.4430330000487,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2132.470131999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 6421.804456000017,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6421.276313999996 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 6427.953980000212,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6427.382803 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8906.018119999999,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8904.713550000011 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 9063.218455999959,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 9062.018992000007 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 9.009999928366597e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8.510000000000634e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 3.2104022236242624,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 3.2098423704176167 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 6.601628078302347,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 6.6010948388201385 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 14.086128266343435,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.08516568195724 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 12.855342576654872,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 12.854287973250095 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 43.00919440585315,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 43.00588993856311 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 156.13165799798387,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 156.1193961439302 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 26.81800785952505,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 26.815032990836958 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.940680896851669,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.939280992261237 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 20.213384671173824,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 20.21199863653511 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.91297643359533,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.912033852057101 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 19.76844604225214,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.766771397524803 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 33.64197320846977,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.63825389201318 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 95257.5308451839,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 95237.68762691687 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 773362.1200885727,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 773324.8289983397 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7512875.7486635065,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7512336.28877006 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 363071.3098775276,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 363038.45608548325 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2876551.760736209,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2876213.055214727 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 32851371.046514183,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 32848522.976744164 ns\nthreads: 1"
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
          "id": "f84ed016a8a83c75930fb4ec77dee482275e28c5",
          "message": "Top up the release-branch AI infra with main's enforcement stack (#3239)",
          "timestamp": "2026-07-04T00:26:59-07:00",
          "tree_id": "1f41a6a110a7ee0d806bce4819d66ce8bdf5c8c4",
          "url": "https://github.com/dartsim/dart/commit/f84ed016a8a83c75930fb4ec77dee482275e28c5"
        },
        "date": 1783166043603,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 71.31143173684282,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 71.30105015789474 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1298.2596599999852,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1298.0872809999998 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 15961.888060999969,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 15960.266646999999 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 538.7155363333326,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 538.6654013333333 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 539.6988213333316,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 539.4696986666671 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 2357.5826820001566,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2357.2524769999977 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 2333.3120539996344,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2332.9229809999993 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 6619.187543000179,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6618.452026 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 6659.690514000431,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6658.791271999988 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 9412.251536999975,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 9410.979933999983 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 9390.009133000149,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 9388.762036999991 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 8.610000037378996e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8.41000000000123e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 3.203784323372703,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 3.203354730619907 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 6.587263425905998,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 6.586327723419974 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 14.062667655965392,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.061171663638197 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 12.845074852802787,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 12.844349049048164 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 43.3009355478902,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 43.29708719811925 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 158.50048254780606,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 158.48303334815475 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 26.743684375597454,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 26.742374182444053 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.643814902868824,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.642778740576192 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 20.19631106765999,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 20.195328737383914 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.641341399060089,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.639838085166538 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 19.63378882602541,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.632048656294184 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 33.49067021559096,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.48762719393541 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 97829.80511205072,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 97745.61381280211 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 784237.9047085188,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 784133.468049327 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7551962.627026975,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7551090.935135137 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 370601.83672936354,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 370555.0013061646 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2891558.5000000168,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2891233.403292185 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33157033.785715025,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33154040.571428567 ns\nthreads: 1"
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
          "id": "c1832e12dd2f7999984fc1f919825bc739d55501",
          "message": "Update pixi lockfile (#3261)\n\nCo-authored-by: jslee02 <4038467+jslee02@users.noreply.github.com>",
          "timestamp": "2026-07-04T09:03:22-07:00",
          "tree_id": "2d4234879a5e1d65b807682578886672d76ee684",
          "url": "https://github.com/dartsim/dart/commit/c1832e12dd2f7999984fc1f919825bc739d55501"
        },
        "date": 1783185901671,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 70.3019549997407,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 70.29145955 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1282.7537370030768,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1282.5073730000006 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 18177.661893016193,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 18177.179943 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 451.97176132933237,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 451.8562923333332 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 460.44258065133664,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 460.3848143333335 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1899.3676859536208,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1899.251061000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1890.9528279909864,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1890.8852620000014 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5421.876015985617,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5421.579991999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5419.4680050131865,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5419.296848000009 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8742.06373799825,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8739.714171000009 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8673.069882031996,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8672.737618 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 7.060007192194462e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6.95999999999683e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.8991815046278,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.897857586608568 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.62777669506048,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.6268595664278465 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.402321970842847,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.400584279761434 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.32886991689027,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.328076017366987 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.2402495999304,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.23472788482198 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 117.33041180503238,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 117.32669423472892 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 24.33899998285253,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 24.335769820883105 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.621832507141356,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.619227219270414 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.444102256291252,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.442558001164382 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.072160678868244,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.071844495098231 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 18.585382823888654,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.58507792973988 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 30.90584700257563,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 30.90533244716195 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 81544.29596372918,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 81506.38116223045 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 712220.2172350596,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 711812.4637096776 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7071903.884386287,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7069376.4070351785 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 342176.696831561,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 342160.255580083 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2987569.5042165765,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2986946.320675103 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33137889.21444765,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33136551.83333336 ns\nthreads: 1"
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
          "id": "9904ea2b213f6045acb42ae6a4274b82eab1e0d9",
          "message": "Isolate Pixi binary paths in CI (#3275)",
          "timestamp": "2026-07-04T15:43:10-07:00",
          "tree_id": "8486e76ab1cb9284f2bb5dfde10a99f70c69cf20",
          "url": "https://github.com/dartsim/dart/commit/9904ea2b213f6045acb42ae6a4274b82eab1e0d9"
        },
        "date": 1783206993218,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 71.75409678946882,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 71.74577515789476 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1293.1952239999873,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1292.9390859999996 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 14960.643690999974,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 14958.867405999996 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 529.8984360000153,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 529.855748 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 529.0333559999377,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 528.9776006666666 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 2233.156851999979,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2232.7706769999995 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 2220.504238999979,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2220.2794479999993 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 6616.7353770000545,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6616.108915000005 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 6519.81860099977,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6519.176983000001 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8780.112986000177,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8779.199144000004 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 9232.613936999996,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 9231.681184999985 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 9.210000015968945e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8.620000000003451e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 3.22063045238161,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 3.2199457942239103 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 6.6728655936339125,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 6.672154177045461 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 14.067616244212813,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.066193315524373 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 12.795249118600827,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 12.793968780848965 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 43.147128211415115,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 43.1430124471151 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 158.34681928786395,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 158.33149603750428 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 26.444138511082173,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 26.44214347414284 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.6922440492091,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.690335552966564 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 20.137274688688787,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 20.13518409143323 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 15.015931441654653,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 15.014042780068449 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 19.578701059165812,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.576110327583326 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 33.673179347696056,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.668459177071036 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 99128.399928421,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 99113.78568360773 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 784877.3440551404,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 784792.3130384837 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7624229.657608956,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7623527.603260871 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 362970.22940563696,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 362931.6830031282 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2878584.5534978337,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2878249.4876543176 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33401439.238094185,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33397869.119047653 ns\nthreads: 1"
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
          "id": "6cb6535744593d8ea0f2053b88f8020d676e83fa",
          "message": "Fix FreeBSD split impulse validation (#3280)",
          "timestamp": "2026-07-05T03:12:23-07:00",
          "tree_id": "a7b137ddeec32f9dcd514ff3aac19f428de77256",
          "url": "https://github.com/dartsim/dart/commit/6cb6535744593d8ea0f2053b88f8020d676e83fa"
        },
        "date": 1783250046959,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 71.23146863121195,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 71.16586273684212 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1293.7962759970105,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1292.6937570000002 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 18722.941288004222,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 18714.195416000002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 472.4374273355352,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 472.16537399999993 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 465.2876506637161,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 464.3164906666672 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1890.474774998438,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1890.1683160000005 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1847.4282660026802,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1847.3695560000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5371.40452399035,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5370.968791999999 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5405.768466000154,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5405.100811000004 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8600.246841000626,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8599.593005999992 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8642.482445007772,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8641.841720000017 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 6.369955372065306e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6.259999999992314e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.930144433593028,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.9288992413431107 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.743367226603488,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.740613442483434 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.489616650844807,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.486845300169398 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.370773583266926,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.36544731382998 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.18325240585414,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.17633187384398 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 116.81407242521213,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 116.78631885389682 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 24.55510360987135,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 24.548776301704 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.152131996078294,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.148455134303552 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.39968877767305,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.397471026042602 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.112022499942547,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.11055251665359 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 18.578981664303136,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.57792752616036 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 30.488507417135008,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 30.48530028619489 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 79761.25925103099,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 79685.69695033402 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 709650.1656544874,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 709350.2360688951 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7070911.8071168205,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7066579.695431464 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 344763.20708220755,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 344638.5592720119 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2928934.5873679086,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2928197.2589473743 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33420331.095147133,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33409733.92857143 ns\nthreads: 1"
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
          "id": "272a4ecd612192748f75530f86a75b81101336e3",
          "message": "Replace dartpy wheel Docker flow with Pixi (#3279)",
          "timestamp": "2026-07-05T11:21:50-07:00",
          "tree_id": "d11e070bb9dd259433a08a387e3efe6f0930b0d3",
          "url": "https://github.com/dartsim/dart/commit/272a4ecd612192748f75530f86a75b81101336e3"
        },
        "date": 1783286614559,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 71.66376189439018,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 71.66125257894736 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1298.9808179991087,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1298.8443070000014 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 18754.65308400453,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 18752.846881999994 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 454.46697366908967,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 454.2041096666666 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 455.75749132937443,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 455.7040276666667 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1893.372284976067,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1893.2132790000012 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1905.497284984449,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1905.3663439999973 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5463.218033997691,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5461.680074 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5428.644908985007,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5428.227309000001 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8679.404793001595,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8678.024925999993 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8741.823978998582,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8741.075834000001 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 5.860056262463332e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5.659999999995877e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.92617218227938,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.926074200070873 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.653668606884692,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.653051141219596 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.44264596329568,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.441862371083227 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.231269690619875,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.230567955930612 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.45344490230135,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.44948055443084 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 119.86035558630273,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 119.85710331216738 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 24.4343914605741,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 24.432564464668612 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.159752695481167,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.159424942239868 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.381618640646803,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.38119616992992 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.097726200686546,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.09591642433813 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 18.59062313165731,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.588456694582835 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 30.685422157440858,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 30.68462658352909 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 80968.59660083427,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 80965.16001607623 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 721769.9617555952,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 721729.8583979332 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7266483.290140261,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7265601.264248711 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 351491.83060067194,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 351448.4118231495 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2944191.6271395516,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2943579.4194915253 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33591241.21440497,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33588190.92857144 ns\nthreads: 1"
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
          "id": "135cc8f20765d39040e3e3ae87536dabac8f5401",
          "message": "Port native collision math core (phase 1, internal-only) (#3281)",
          "timestamp": "2026-07-05T11:30:49-07:00",
          "tree_id": "44ba7662be3537e32bbf7afb4a7ee94c142ceaf1",
          "url": "https://github.com/dartsim/dart/commit/135cc8f20765d39040e3e3ae87536dabac8f5401"
        },
        "date": 1783288006816,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 70.51268029972562,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 70.4777 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1304.4941420084797,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1304.4006139999985 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 19421.728990011616,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 19418.28259 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 453.527850661582,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 453.25462033333315 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 471.77009666726616,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 471.68267433333386 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1961.1182880034903,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1961.0428289999984 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1966.2265150109306,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1966.0871260000033 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5560.75798200618,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5560.421062000009 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5701.653325013467,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5700.938479999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 9264.868091995595,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 9263.760137000007 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 9287.166761991102,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 9286.462466000003 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 7.159978849813343e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6.860000000001761e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.9712910087843176,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.9710122901016347 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.7467682554160495,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.746586892410313 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.538406325318109,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.537522426488671 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.251215914505705,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.250559395482751 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.72959091961683,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.72634785247431 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 122.91884989526032,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 122.8990559229802 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 24.240599510681076,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 24.24021246984918 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.02978435294388,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.027998079003547 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.26152559928083,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.261114232570964 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.031656685952422,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.031324890418928 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 18.4712561036205,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.46938766519829 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 30.95418168136082,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 30.94255193898147 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 80776.29186866347,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 80736.39625260235 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 716279.2046702552,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 716120.1283116885 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7189606.082064506,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7189439.523076922 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 344288.1276439754,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 344278.69232661114 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2956321.1652603676,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2956132.2012711815 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33126838.928659532,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33116266.428571433 ns\nthreads: 1"
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
          "id": "95c226d4044f4449b44bc4e18beea061e2cc0975",
          "message": "WP-PG.42: SIMD-screen DART broadphase candidates (#3299)",
          "timestamp": "2026-07-05T20:25:24-07:00",
          "tree_id": "8b1e87b7454672a04ba497e119434c5445d3dadf",
          "url": "https://github.com/dartsim/dart/commit/95c226d4044f4449b44bc4e18beea061e2cc0975"
        },
        "date": 1783309480444,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 54.85140300000239,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 54.846066400000005 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1048.1541810000863,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1047.9533070000002 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 12041.320827999925,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 12039.571173999997 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 428.0124256666795,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 427.97297700000036 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 430.05283299999064,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 429.99684966666626 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1676.152565999928,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1676.0431360000005 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1678.1436479999456,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1677.9549789999974 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5439.523498999961,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5439.019965999996 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5432.51829999997,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5432.041362999996 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 7112.74939100008,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 7111.93234000001 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 7119.371830999967,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 7118.567680000012 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 8.509999958050686e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8.109999999998674e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.69717305482745,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.6962522497686616 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.534867253957525,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.533715799823031 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.97476920716373,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.973249290359979 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 8.439027283393708,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 8.437831525821446 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 28.55779284006752,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 28.554308886097488 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 105.961750317765,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 105.94835676835073 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 20.94342348328399,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 20.94064430876063 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 11.94833644819974,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.94703419525513 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 16.3532080192906,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 16.351819694092708 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 12.041762641331317,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 12.040182055801546 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 15.883601468310415,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 15.881539592734795 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 27.67694540667924,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 27.67315948455028 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 71804.33025572218,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 71786.60267648738 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 648301.2971564404,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 648161.4374407589 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 6313040.197309289,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6312119.88340807 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 278866.72636815725,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 278822.65174129367 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2235962.3210862214,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2235720.1421725247 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 25670442.090908676,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 25667873.472727235 ns\nthreads: 1"
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
          "id": "a073d7a6a0d69c246c310cc15bdbf8bcdd02e866",
          "message": "Fix native sphere-sphere zero-contact-limit short-circuit (#3298)",
          "timestamp": "2026-07-05T19:28:56-07:00",
          "tree_id": "8be2f96f7c12ed912ab2122cce67052b8b2db4b7",
          "url": "https://github.com/dartsim/dart/commit/a073d7a6a0d69c246c310cc15bdbf8bcdd02e866"
        },
        "date": 1783309794374,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 70.78410989997792,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 70.78193045 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1292.9462210013298,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1292.9048140000007 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 19659.56056700088,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 19657.974218999996 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 478.02100866586744,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 477.8664616666668 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 460.1022283362302,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 460.0271529999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1955.2573039836716,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1955.1269530000006 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1938.0987720069243,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1937.9944800000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5635.02576299652,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5634.522273000002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5718.373159994371,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5717.9393070000015 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8900.874484985252,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8900.030513999993 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8912.314471992431,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8911.424161999988 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 6.960035534575583e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7.250000000000312e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 3.053270257985385,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 3.0507134891431296 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.830179684384497,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.829616044927505 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 12.109174840611056,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 12.10755054676401 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.446858033810305,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.444558215475586 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.72326581953927,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.722761528863536 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 121.22787759463334,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 121.22479430434042 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 24.898610199926775,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 24.897964471358332 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.365031857693197,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.364670257866663 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.914679533819818,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.9129952901386 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.368469676144022,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.367757459514129 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 19.06234577949483,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.06189404554591 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 32.287898435310886,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 32.286837571471494 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 83252.61660677406,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 83240.309566787 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 741307.9506390622,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 741050.5461783436 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7239960.424676512,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7228628.602150534 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 347651.450189562,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 347588.6120451697 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2897360.175738411,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2896888.4518828443 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 34414384.100091405,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 34413110.699999996 ns\nthreads: 1"
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
          "id": "736d116b7311693f3ec1068e915b4b67d66e876b",
          "message": "Fix dart-gui-osg mouse-handler unregistration and InteractiveFrameDnD tool ownership (#3300)",
          "timestamp": "2026-07-05T20:47:38-07:00",
          "tree_id": "a4befe8e1a041dea2158ae52ba20f6d4e55279a5",
          "url": "https://github.com/dartsim/dart/commit/736d116b7311693f3ec1068e915b4b67d66e876b"
        },
        "date": 1783313449952,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 71.71276352631283,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 71.70572978947368 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1314.0729689999944,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1313.9814329999995 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 16953.222793000008,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 16951.557233000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 539.0535240000341,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 538.8547096666667 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 541.941192666665,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 541.8145543333338 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 2268.422413999815,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2268.0876360000007 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 2244.146141999863,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2243.804313000002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 6710.498764000248,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6709.606298999993 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 6698.356475000083,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6697.651800000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 9084.769244999961,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 9083.91192400002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 9081.460972000514,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 9080.56644700001 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 8.920000027501374e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8.620000000003451e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 3.284857511358606,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 3.2844643538469103 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 6.662724772596728,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 6.662091484283853 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 14.180351854653304,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.178343824713243 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 12.77590519967283,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 12.7748344570291 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 42.76540077482654,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 42.761561353573626 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 155.80604158045182,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 155.79849147699278 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 26.59144361831747,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 26.58905054385033 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.885347332869399,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.883890795046389 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 20.647582045175074,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 20.64601224797722 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.865560188135197,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.863957913959323 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 20.00819289572829,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 20.00686109558809 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 34.24292123841942,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 34.23962131686023 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 97948.1045615091,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 97862.67788426894 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 782870.6655499735,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 782801.8749302073 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7529432.3279569,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7529046.543010748 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 367814.3107180687,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 367768.01715945126 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2882173.774327051,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2881908.7577639767 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33300567.99999933,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33297298.166666728 ns\nthreads: 1"
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
          "id": "3559967cef86c09b09ca503fbdd6ae301e887b82",
          "message": "Fix scalar macOS broadphase build (DART 6 LTS) (#3309)",
          "timestamp": "2026-07-06T09:09:48-07:00",
          "tree_id": "8a97c105b5495c01d152edcd68bb35566717f077",
          "url": "https://github.com/dartsim/dart/commit/3559967cef86c09b09ca503fbdd6ae301e887b82"
        },
        "date": 1783355943444,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 72.10553715776769,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 72.0893859473684 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1296.7258820026473,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1295.820526 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 17827.96325399977,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 17825.237750000004 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 456.74945933448424,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 456.6210920000001 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 460.57911933530704,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 460.5319766666665 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1891.8683620031516,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1891.8028519999996 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1891.7274400009774,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1891.6665150000008 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5428.816335996089,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5428.623359000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5397.172590997798,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5396.964482999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8704.677102996357,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8704.037787000005 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8718.233786003111,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8717.971095999985 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 7.249982445500792e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7.060000000004911e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.9859659306065462,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.98513315299132 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.847190329448075,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.842308678065052 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.613468772993862,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.61277368381885 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.021126559986442,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.020285109999989 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 32.438783498453866,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 32.436784126319715 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 115.43068276755093,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 115.42196523821305 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 24.45501269358219,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 24.45433703781731 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.99306578881825,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.99263733414054 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.090298211816872,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.089951764271976 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.935965153102744,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.935713155674682 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 18.44055463355414,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.44022605631021 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 30.035833487310516,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 30.03529991189811 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 81484.18436744109,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 81409.33672157965 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 733300.2081793766,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 732967.3602517041 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7305973.58333777,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7303873.104166664 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 343668.4318071383,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 343496.5108321022 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2935245.673686854,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2934320.454736844 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33306094.166619003,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33295885.738095164 ns\nthreads: 1"
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
          "id": "f91e6d7a3e9512bf9f82f3203daa94adf1d105e9",
          "message": "Improve soft_bodies visualization capture (#3304)",
          "timestamp": "2026-07-06T09:13:16-07:00",
          "tree_id": "d06e629346aed31d356e069c02559d1005ab4f26",
          "url": "https://github.com/dartsim/dart/commit/f91e6d7a3e9512bf9f82f3203daa94adf1d105e9"
        },
        "date": 1783356110703,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 74.71614105260845,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 74.69961084210526 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1326.3144810007361,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1326.2581910000001 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 18867.004299001565,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 18864.434408999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 454.2311470019437,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 454.14983733333315 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 457.61804300006287,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 457.5329873333332 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1936.7931410015444,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1936.7365419999985 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1944.7943129998748,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1944.724821999994 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5481.091878002189,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5480.805979000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5513.554958004534,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5513.319289000002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8894.017086997337,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8893.46300199999 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8934.878166997805,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8934.450216000003 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 6.76001945976168e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6.260000000000987e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 3.04666226013573,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 3.0459011431861165 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.741539730153323,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.74104522887706 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.554169611652844,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.544171173416403 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.158936198033336,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.157390261560005 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.50905442143392,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.502009038443184 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 121.0089967980805,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 120.97873978885427 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 24.507060919208488,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 24.506176889932046 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.192978868611803,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.19265678920113 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.351107039881416,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.350592316863516 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.199243286063444,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.198987222199625 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 18.614881892056783,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.614277201865452 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 30.375733954714324,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 30.37506867857292 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 81870.86671775367,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 81850.89239648978 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 738211.5021158749,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 738041.6592592592 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7336179.312498339,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7334430.244791672 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 348851.45243479364,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 348829.7113607987 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2939315.150419812,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2939186.7966101747 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 33761389.02435408,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 33760307.097560935 ns\nthreads: 1"
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
          "id": "93afb066c665acf1f207aec69ea71169120ca87b",
          "message": "Add DART 6 simulation allocation hardening (#3297)",
          "timestamp": "2026-07-06T12:23:43-07:00",
          "tree_id": "8b37fac09069da5d205c5b7682146e88ce924067",
          "url": "https://github.com/dartsim/dart/commit/93afb066c665acf1f207aec69ea71169120ca87b"
        },
        "date": 1783371469871,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 58.17528162500215,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 58.17105891666666 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1060.6009050000011,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1060.4805394999994 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 13006.778757000007,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 13005.737764000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 495.46152333338495,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 495.0791826666668 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 500.512838333331,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 500.44194500000043 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 2019.4968089999747,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2019.383773000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 2024.028656999917,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2023.8094390000008 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 7426.719298999841,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 7425.722873999995 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 7458.5135170000285,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 7457.109201000002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 11646.862525999948,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 11645.816154999991 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 11638.813490000075,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 11637.378658000003 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 3.7799999574872343e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 3.469999999999342e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 3.0379402220049005,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 3.0373403284396314 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 6.3653898492928205,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 6.365036159736924 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 13.923863749261166,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.923290163611268 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.539727836162795,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.539488487679261 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 34.33124101267118,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 34.329551345643885 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 124.36808588735067,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 124.36277660247727 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 23.5459508870199,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 23.54519246118245 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.45140894903193,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.45065809115763 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.205974488465667,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.204906956318855 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.493512557199452,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.492131930061149 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 18.474698770681414,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.473092667661415 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 33.0432122423882,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.0409899675839 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 78300.47868779588,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 78294.39661905824 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 720184.8854110938,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 720123.2673740046 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7083127.369230916,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7082804.066666664 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 283249.24872474844,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 283217.1789430727 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2099503.236641128,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2099223.0717557254 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 24279074.263156936,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 24275866.59649119 ns\nthreads: 1"
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
          "id": "38656f3dbea4934445cf1dc4130e50b56d2cec2b",
          "message": "WP-PG.30: Cache single-free-body root-joint classification (#3310)",
          "timestamp": "2026-07-06T12:38:28-07:00",
          "tree_id": "b41052c3b81da3c2e6acd93f8de9710739c62d21",
          "url": "https://github.com/dartsim/dart/commit/38656f3dbea4934445cf1dc4130e50b56d2cec2b"
        },
        "date": 1783374442365,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 56.875089640088845,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 56.783633880000004 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1030.3091985006176,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1029.8971155000002 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 12712.343719998898,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 12701.913288000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 415.58696433882386,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 415.4779896666666 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 416.60142966672237,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 416.5389113333336 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1734.795356001996,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1734.7633670000012 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1714.8889469972346,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1714.8336549999997 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5207.64193800278,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5207.449531000002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5213.531035995402,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5213.178757999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8327.289704997384,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8325.888052999993 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8398.884255992016,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8398.61871299999 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 5.769979907199741e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5.55999999999647e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.7615613575479396,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.761080459502464 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.507964305438083,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.507251555025814 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.256854062143768,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.256418985204562 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.594668145364862,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.593126345315413 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.95360109914859,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.944514762949254 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 119.17921796844026,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 119.1574830695933 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 23.14275968339739,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 23.142371796994 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.253076655851421,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.252188979001277 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 18.575750971602684,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.57170562253783 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.249046987073747,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.247187948200535 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 17.691088423653223,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 17.690627680658654 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 30.034435744957506,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 30.033902760296755 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 81044.5992607022,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 81016.50228152257 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 715814.2073263843,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 715780.9226405361 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7082804.954559907,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7081004.611111113 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 253571.2428156646,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 253540.12713714092 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 1994612.1559280155,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1993649.7682403428 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 23282316.23323518,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23281680.716666684 ns\nthreads: 1"
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
          "id": "c2404438c63118faa6e7a4f275598356fa53f750",
          "message": "Port native narrowphase dispatcher, sphere/box only (phase-2 P2, internal-only) (#3306)",
          "timestamp": "2026-07-06T14:12:50-07:00",
          "tree_id": "522f1f9b20b967dd3a202d5eb80b460cb9f15292",
          "url": "https://github.com/dartsim/dart/commit/c2404438c63118faa6e7a4f275598356fa53f750"
        },
        "date": 1783376738392,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 56.32193411991465,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 56.29606512000001 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1037.4558815019554,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1037.3235815 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 11075.723262998508,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 11074.187061999999 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 411.4116176594204,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 411.09569833333336 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 412.1508353346144,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 412.09672366666706 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1611.3260959973559,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1611.0713769999984 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1630.0429830007488,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1629.4384669999963 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5104.916476004291,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5104.299378000001 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5099.708075998933,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5099.433362999996 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 7965.931687002012,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 7964.990855000004 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 7974.1856699984055,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 7973.869712999999 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 5.670008249580861e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5.56999999999988e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.651380525810554,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.6510317558742433 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.383005772430586,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.380819600954273 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.259798299632635,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.258608680713694 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.495670188012491,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.495199040982977 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.89728161495926,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.89636429674683 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 119.02337567147383,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 119.02012778299066 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 23.113151118115297,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 23.112276293423534 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.066017059882528,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.065621466497111 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 18.400898933000338,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.40031769326441 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.090114112776561,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.086650567702014 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 17.49878408385749,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 17.49469353951552 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 29.859732618816064,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 29.85774574393292 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 79689.5899485901,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 79666.33180350161 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 708304.4710366895,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 708061.700711382 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 6958087.10397704,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6956709.173267326 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 251967.6315699519,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 251943.53059755196 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 1983442.542489609,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1983355.689801702 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 23165569.11867692,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23164588.576271202 ns\nthreads: 1"
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
          "id": "361c34401d4b471c0ae48892fc6eb5b99d538a04",
          "message": "Add headless capture and image verification to DART 6 (#3314)",
          "timestamp": "2026-07-06T14:28:41-07:00",
          "tree_id": "14f1003def346bd0958775e492c3352b8a3f17b1",
          "url": "https://github.com/dartsim/dart/commit/361c34401d4b471c0ae48892fc6eb5b99d538a04"
        },
        "date": 1783377468166,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 56.83528292021947,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 56.83359152 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1045.428946501488,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1045.3925935000004 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 14430.509616002382,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 14429.847963000006 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 418.7687800003914,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 418.71774166666665 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 421.10061633138685,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 421.0509126666668 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1807.8030699980445,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1806.8505470000016 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1816.3016119942768,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1816.1691619999995 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5329.978450994531,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5327.406845999999 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5363.413065992063,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5363.118688000007 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8375.158843999088,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8374.241242999993 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8379.852863996348,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8379.506049 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 5.769979907199741e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5.56999999999988e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.7343979797869076,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.7343151159649595 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.530526294952701,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.527803824506948 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.311325136081045,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.311014892765686 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.600459518530798,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.59259753827997 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 34.125000412530234,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 34.12429522677081 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 119.17087216523964,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 119.15884341576124 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 23.191826203456475,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 23.190883534800676 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.139954395001917,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.139614273117791 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 18.605488941850354,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.603873923983585 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.175455363083655,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.174143450419095 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 17.701415006436402,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 17.700783891130822 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 29.95691776209195,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 29.95632149476316 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 80876.87007108262,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 80855.76111468131 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 722278.8160639076,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 722201.0917098437 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7168271.3575159935,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7167478.528497409 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 252156.61115165675,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 252145.28478023966 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2015733.3644212375,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2015649.5265423248 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 23690044.474556167,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23683660.983050834 ns\nthreads: 1"
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
          "id": "2e11928288c9ac695839ebd2cb708d160c3f859f",
          "message": "Port native BruteForce broadphase (phase-2 P1, internal-only) (#3303)",
          "timestamp": "2026-07-06T15:10:48-07:00",
          "tree_id": "666ed7335aeb052f696be410c62b843eed932995",
          "url": "https://github.com/dartsim/dart/commit/2e11928288c9ac695839ebd2cb708d160c3f859f"
        },
        "date": 1783383528480,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 57.853485666782944,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 57.84109316666666 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1054.174504501134,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1054.1060364999994 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 14273.457940995286,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 14271.502055000004 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 419.3673303331404,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 419.29600833333353 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 422.7081133309791,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 422.6308213333339 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1787.7993470028741,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1787.6332290000025 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1792.8693540015956,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1792.611517000001 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5318.333921008161,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5317.1499589999985 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5322.384435014101,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5322.189035000001 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8367.334830996697,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8366.866887 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8427.80793899874,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8427.204971000008 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 6.459958967752755e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6.370000000003803e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.7220975805359306,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.7217651961653018 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.4291508706924025,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.4233136534107125 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.397151702281002,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.387978860961788 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.53889947816527,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.527708721218763 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.70261967272736,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.69724351345491 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 117.30261804857588,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 117.28739054600591 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 23.073658479448046,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 23.072102152841897 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.112865379394835,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.112049971452924 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 18.128210017722648,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.126233765223073 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.023379758770984,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.02242815071052 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 17.41191262122972,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 17.411495200550625 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 29.16071284344427,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 29.157895146118076 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 80570.56765011093,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 80565.0593985395 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 720879.6868871079,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 720818.9161953727 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7120649.9695311645,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7118548.939086295 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 264371.21402551356,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 264343.03278688475 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2016966.826840412,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2016859.4357864351 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 23822063.881306026,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23811747.779661026 ns\nthreads: 1"
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
          "id": "66d35920cf685b401fcb4653bd951127efa5079c",
          "message": "Add native collision detector adapter skeleton (#3318)",
          "timestamp": "2026-07-06T17:26:00-07:00",
          "tree_id": "4f56243b4e4abf95fcd788980d3de1ef5979a8b5",
          "url": "https://github.com/dartsim/dart/commit/66d35920cf685b401fcb4653bd951127efa5079c"
        },
        "date": 1783388146262,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 56.840187160123605,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 56.837172319999986 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1045.6070535001345,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1045.5002585000002 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 13468.22097599943,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 13467.803332999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 421.10813700128347,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 421.0428163333334 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 424.85857200032723,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 424.7371990000002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1771.417502008262,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1771.3600510000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1812.423253999441,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1812.3310649999985 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5413.6697279900545,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5413.497971999995 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5258.201316995837,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5258.023430999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8235.08281699469,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8233.803128000005 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8208.95828200446,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8208.590889000006 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 5.869951564818621e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5.660000000004551e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.713113007396028,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.7129435793531838 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.495022100035947,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.494907727710501 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.324957851549968,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.324151561248344 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.539593115917032,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.538771274801555 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.73993096182598,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.73922674753085 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 118.70262367966002,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 118.6725945485522 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 23.243224994621382,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 23.242738291133946 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.164030039632765,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.163222173153128 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 18.22782619445349,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.226814678130633 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.145948839583086,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.14568692329202 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 17.508749620340964,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 17.50687468464059 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 29.18561482814678,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 29.185135827060602 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 80679.36713874679,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 80648.47921802444 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 718841.8975393226,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 718680.1726434422 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7148072.682022572,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7147340.31794872 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 254272.0454703553,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 254126.98478260884 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2029111.8729854494,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2029007.6686131358 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 23828305.033976763,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23827520.3898305 ns\nthreads: 1"
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
          "id": "8af37f7de45c0576d6ae28b48a35220bbb09c301",
          "message": "Consolidate DART 6 GUI examples into dart-demos + best-effort py-demos (#3301)",
          "timestamp": "2026-07-06T18:57:04-07:00",
          "tree_id": "5d88be5a488d0784dcdf006bb9a951187a85ee61",
          "url": "https://github.com/dartsim/dart/commit/8af37f7de45c0576d6ae28b48a35220bbb09c301"
        },
        "date": 1783392873637,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 57.15213491988833,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 57.147562719999996 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1053.5876730027667,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1053.4932944999998 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 14425.433892000001,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 14424.673204 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 422.0684656659917,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 421.94773833333346 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 427.5661576660544,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 427.505993333333 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1817.2244650049834,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1817.0844689999992 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1796.177053001884,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1796.1038490000012 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5339.585884998087,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5338.661323 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5331.891474997974,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5331.486624 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8345.105213011266,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8344.452801000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8382.461157001671,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8382.002752000006 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 7.060007192194462e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6.95999999999683e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.677783303277046,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.677701853610523 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.401940489345125,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.401797737470574 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.317602131442635,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.316772218633927 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.46916867949971,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.468408634130961 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.57675084899275,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.572354119999055 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 117.95065075187911,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 117.94786526208486 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 23.05287969589719,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 23.05207666485657 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.14509688440137,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.14414226894604 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 18.286754712094687,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.284822870773457 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.134506557230452,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.13322962112512 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 17.468926051649607,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 17.467630834157813 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 29.193650873194,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 29.19300073021635 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 80713.78763174497,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 80708.38475217989 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 717241.0529257633,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 717158.4010178121 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7069359.076137566,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7069177.010152285 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 253101.122768649,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 253069.19379844939 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2003311.4985633744,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2003262.2607449878 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 23688591.254201237,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23686866.016949136 ns\nthreads: 1"
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
          "id": "a1871c31f759152d72b17838e670e1379db97a6f",
          "message": "Bridge native detector to narrow phase (#3319)",
          "timestamp": "2026-07-06T20:58:05-07:00",
          "tree_id": "0c67acf6adc52ea02c0ddb82789e943b10cc326e",
          "url": "https://github.com/dartsim/dart/commit/a1871c31f759152d72b17838e670e1379db97a6f"
        },
        "date": 1783405931471,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 61.04970608695883,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 61.04172986956521 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1129.1846370000371,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1129.0582545000004 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 11360.294895000037,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 11359.093345 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 498.8274060000701,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 498.76137999999975 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 501.95055533322375,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 501.9052206666667 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 2105.299728999853,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2105.155612999999 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 2106.671394000159,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2106.535733999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 6460.713265999857,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6459.502225999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 6495.059674000232,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6494.624486000007 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8652.14236900033,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8651.557225000006 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8673.252649999768,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8672.308197000008 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 9.219999981269213e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8.719999999998521e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 3.0539391480333347,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 3.0537008345574232 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 6.417509067351894,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 6.416833694506667 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 13.956283767602557,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.95502060345766 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 12.898285807802807,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 12.896821218469059 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 42.76873424741905,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 42.76442329379134 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 154.8652544290737,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 154.85191576180114 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 25.519052459614816,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 25.516261348909378 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.99826243661908,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.996376811594173 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.531386440251534,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.52906841420603 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.907010665554267,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.906301465768903 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 18.714603791877014,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.712013223096015 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 32.700858499613474,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 32.69903887810247 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 97817.31403496357,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 97810.00983792204 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 775347.8857795479,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 775280.279268989 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7481310.5698923515,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7480504.913978498 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 284994.4668335405,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 284962.06508135126 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2186004.976525796,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2185781.0563380285 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 25320064.581817612,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 25317592.618181806 ns\nthreads: 1"
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
          "id": "6a273599873b74675cf7b481e4dd6454e7c3f52b",
          "message": "Enable compiler cache discovery in DART 6 Pixi builds (#3326)",
          "timestamp": "2026-07-07T08:39:01-07:00",
          "tree_id": "115997d6be9d42b59923cc24f4b2253eafce886d",
          "url": "https://github.com/dartsim/dart/commit/6a273599873b74675cf7b481e4dd6454e7c3f52b"
        },
        "date": 1783440402631,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 59.35764908676705,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 59.35031999999998 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 1081.03099049913,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1080.9492289999998 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 15974.538786002086,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 15972.532929000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 420.5022029976438,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 420.4567553333334 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 423.8209713221295,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 423.765699333333 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1922.2181109944358,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1921.8214480000013 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1918.7789799907478,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1918.5479239999986 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5446.027538011549,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5445.73455299999 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5420.274363015778,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5419.623710999999 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8774.197725986596,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8773.549315000011 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8954.08844499616,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8953.320569999987 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 7.049966370686889e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 6.860000000006098e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.678592602574447,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.6784319754372703 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.403527656423235,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.40309708326757 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.297242433661005,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.295893907554564 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.43297267142831,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.43072497718285 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 33.314184299728666,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.312409527888796 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 117.49768254278074,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 117.49381007881938 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 23.3026325092632,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 23.30195473628096 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.225616177844344,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.224945555534504 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 18.467441585531358,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.466765851004535 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.352852702869418,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.349713411039914 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 17.777853968205843,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 17.775995517460313 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 30.275249660859128,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 30.27442420061854 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 81946.83604630195,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 81930.63224225452 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 723185.5407366581,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 723125.3976578413 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7066374.90914389,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7066171.565656568 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 256610.35659706904,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 256563.71464921834 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 2020335.8526171704,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 2020258.4768786142 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 23890871.172248993,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 23890171.81034482 ns\nthreads: 1"
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
          "id": "b6e6a0d8778a13c2d43d5b75553738e84014baea",
          "message": "Optimize DART 6 deformable simulation and native soft collision (#3307)",
          "timestamp": "2026-07-07T09:05:56-07:00",
          "tree_id": "5bbb19961d09bbe6c4c8a8f333295230a4cafeb0",
          "url": "https://github.com/dartsim/dart/commit/b6e6a0d8778a13c2d43d5b75553738e84014baea"
        },
        "date": 1783442639556,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 52.48712425883341,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 52.46780407407406 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 884.8526464935276,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 884.8142105000001 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 13857.569277010043,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 13855.964797000002 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 404.5436196611263,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 404.28258966666664 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 405.34452666179277,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 405.2681079999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 1833.9547629875597,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1833.1065019999996 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 1812.578498997027,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 1812.0446819999997 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 5321.615130014834,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5320.712030999999 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 5334.067620002315,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 5333.373233000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8708.78249600355,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8708.015372000005 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8686.375749995932,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8685.379709999977 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 5.869951564818621e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 5.669999999999287e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 2.7257064048169544,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 2.7251276212486175 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 5.410651324139997,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 5.410474674673907 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 11.377418852589253,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 11.375780941588706 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 10.138245135321709,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 10.136992570782736 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 32.818602104362675,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 32.814534314181955 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 116.17228790131732,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 116.14952625514381 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 23.414050034284188,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 23.409879262195048 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 13.70956407954973,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.706354198286201 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 18.287205981013116,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 18.28655001370593 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 13.349681584118885,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.348625970579091 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 17.813603658735268,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 17.811409612535503 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 29.512643972859824,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 29.509422067294395 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 81695.43721648298,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 81689.8082239517 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 731551.6098338267,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 731358.3028242677 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7092320.219401569,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7090510.704081637 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 214994.53080049576,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 214974.18851320105 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 1666641.5560875824,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1666368.9761336541 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 20166862.217382107,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 20166214.869565222 ns\nthreads: 1"
          },
          {
            "name": "Soft-body world step - 0 scene - 1 threads - 200 steps",
            "value": 25.39737528258109,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 25.39717883018867 ms\nthreads: 1"
          },
          {
            "name": "Soft-body world step - 0 scene - 16 threads - 200 steps",
            "value": 25.639119872519384,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 25.59057105555546 ms\nthreads: 16"
          },
          {
            "name": "Soft-body world step - 1 scene - 1 threads - 200 steps",
            "value": 21.04996600067422,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 21.050676830769152 ms\nthreads: 1"
          },
          {
            "name": "Soft-body world step - 1 scene - 16 threads - 200 steps",
            "value": 21.31652951599487,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 21.266144484848223 ms\nthreads: 16"
          },
          {
            "name": "Soft-body world step - 2 scene - 1 threads - 200 steps",
            "value": 84.1467411211103,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 84.1360896470592 ms\nthreads: 1"
          },
          {
            "name": "Soft-body world step - 2 scene - 16 threads - 200 steps",
            "value": 84.2781631183564,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 84.26746852941179 ms\nthreads: 16"
          },
          {
            "name": "Soft-body world step - 3 scene - 1 threads - 200 steps",
            "value": 45.1968862879796,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 45.19706216128926 ms\nthreads: 1"
          },
          {
            "name": "Soft-body world step - 3 scene - 16 threads - 200 steps",
            "value": 45.262303901836276,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 45.20197383871024 ms\nthreads: 16"
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
          "id": "9ff8b1d77a1d0c694d45754995245a05c52a4c0e",
          "message": "WP-PG.02: Extend contact-container benchmark coverage (#3327)",
          "timestamp": "2026-07-07T09:54:48-07:00",
          "tree_id": "47c745a0bc920b3e80efcb274731467787ea8f97",
          "url": "https://github.com/dartsim/dart/commit/9ff8b1d77a1d0c694d45754995245a05c52a4c0e"
        },
        "date": 1783445341373,
        "tool": "googlecpp",
        "benches": [
          {
            "name": "Stacked boxes world step - 2 grid side",
            "value": 54.08568969230251,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 54.081549499999994 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 4 grid side",
            "value": 932.4867619999395,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 932.4007895 ms\nthreads: 1"
          },
          {
            "name": "Stacked boxes world step - 8 grid side",
            "value": 9797.796402000131,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 9796.685671000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 1 threads",
            "value": 485.1422473333666,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 485.0448903333338 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 0 engine - 16 threads",
            "value": 487.754223999976,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 487.7085983333336 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 1 threads",
            "value": 2014.907638000068,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2014.640874999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 1 engine - 16 threads",
            "value": 2017.861502000187,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 2017.6583540000017 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 1 threads",
            "value": 6322.627997999916,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6321.9787039999965 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 16 threads",
            "value": 6301.678168000081,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6300.830358000013 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 1 threads",
            "value": 8525.814064000087,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8524.812933999996 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 16 threads",
            "value": 8511.487994999925,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8510.477973000006 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 2 engine - 1 threads",
            "value": 524.2438653334224,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 524.2006129999908 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 2 engine - 16 threads",
            "value": 526.4195809999516,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 526.3738003333268 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 2 engine - 1 threads",
            "value": 6392.179873000033,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6391.651839000019 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 2 engine - 16 threads",
            "value": 6388.207749000004,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6387.605553999975 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 3 engine - 1 threads",
            "value": 446.183721999887,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 446.1341113333219 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 60 objects - 3 engine - 16 threads",
            "value": 446.79149533340023,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 446.70330766667615 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 3 engine - 1 threads",
            "value": 6636.798294000073,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6636.239570999976 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 3 engine - 16 threads",
            "value": 6650.222361000033,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6649.494457000003 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 0 engine - 4 threads",
            "value": 6357.18692599994,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6356.556894000022 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 1 engine - 4 threads",
            "value": 8526.983873999825,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 8526.074877999974 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 2 engine - 4 threads",
            "value": 6378.036262999786,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6377.463613999993 ms\nthreads: 1"
          },
          {
            "name": "Contact container active step - 120 objects - 3 engine - 4 threads",
            "value": 6632.15368300007,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 6631.691482999998 ms\nthreads: 1"
          },
          {
            "name": "Contact container deactivation-enabled step - 60 objects - 0 engine - 16 threads",
            "value": 56.103806999999506,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 56.03255100010074 ms\nthreads: 1"
          },
          {
            "name": "Contact container deactivation-enabled step - 60 objects - 1 engine - 16 threads",
            "value": 234.39018900012343,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 234.37249099993096 ms\nthreads: 1"
          },
          {
            "name": "Google Benchmark empty baseline",
            "value": 9.310000450568623e-10,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 8.920000000006008e-10 ns\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 10 links",
            "value": 3.0425803615566096,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 3.042329082362736 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 20 links",
            "value": 6.4033248652705135,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 6.402585064920295 us\nthreads: 1"
          },
          {
            "name": "Recursive inverse dynamics - 40 links",
            "value": 13.890683277482372,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 13.889556834150596 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 10 links",
            "value": 12.95187015186234,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 12.950808159437704 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 20 links",
            "value": 43.704293651287394,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 43.701371777264434 us\nthreads: 1"
          },
          {
            "name": "Dense mass-matrix inverse dynamics - 40 links",
            "value": 154.32490610225688,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 154.3143619571193 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 2 contacts",
            "value": 25.544266785866398,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 25.542558622487928 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 4 contacts",
            "value": 14.145119501695445,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.143844978984186 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics - 8 contacts",
            "value": 19.583781163748856,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.582352602377068 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 4 friction bases",
            "value": 14.196369947407966,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 14.19501522996947 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 8 friction bases",
            "value": 19.028030738151827,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 19.026373211945852 us\nthreads: 1"
          },
          {
            "name": "Contact inverse dynamics basis sweep - 16 friction bases",
            "value": 33.25866586647172,
            "unit": "us/iter",
            "extra": "iterations: 3\ncpu: 33.254851947805015 us\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 1 iterations",
            "value": 100817.99248229752,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 100791.34851470696 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 10 iterations",
            "value": 794764.3493906143,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 794697.164248404 ns\nthreads: 1"
          },
          {
            "name": "Skel kinematics update corpus - 100 iterations",
            "value": 7661907.615385474,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 7661039.461538465 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 1 steps",
            "value": 244736.38293024476,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 244711.49877322093 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 10 steps",
            "value": 1767037.4885204982,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 1766821.692602039 ns\nthreads: 1"
          },
          {
            "name": "Skel dynamics step corpus - 100 steps",
            "value": 20944621.428570252,
            "unit": "ns/iter",
            "extra": "iterations: 3\ncpu: 20942511.22222222 ns\nthreads: 1"
          },
          {
            "name": "Soft-body world step - 0 scene - 1 threads - 200 steps",
            "value": 27.042277769256685,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 27.03968048076925 ms\nthreads: 1"
          },
          {
            "name": "Soft-body world step - 0 scene - 16 threads - 200 steps",
            "value": 27.458317490242077,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 27.4238467843136 ms\nthreads: 16"
          },
          {
            "name": "Soft-body world step - 1 scene - 1 threads - 200 steps",
            "value": 23.695947813555875,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 23.693026559321925 ms\nthreads: 1"
          },
          {
            "name": "Soft-body world step - 1 scene - 16 threads - 200 steps",
            "value": 24.07326977585603,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 24.048582758620658 ms\nthreads: 16"
          },
          {
            "name": "Soft-body world step - 2 scene - 1 threads - 200 steps",
            "value": 93.49713833333531,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 93.48777993333357 ms\nthreads: 1"
          },
          {
            "name": "Soft-body world step - 2 scene - 16 threads - 200 steps",
            "value": 93.582727866639,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 93.57315933333321 ms\nthreads: 16"
          },
          {
            "name": "Soft-body world step - 3 scene - 1 threads - 200 steps",
            "value": 51.96880514814371,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 51.96451059259217 ms\nthreads: 1"
          },
          {
            "name": "Soft-body world step - 3 scene - 16 threads - 200 steps",
            "value": 52.15348529633991,
            "unit": "ms/iter",
            "extra": "iterations: 3\ncpu: 52.13857659259228 ms\nthreads: 16"
          }
        ]
      }
    ]
  }
}