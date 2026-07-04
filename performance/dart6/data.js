window.BENCHMARK_DATA = {
  "lastUpdate": 1783207111232,
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
      }
    ]
  }
}