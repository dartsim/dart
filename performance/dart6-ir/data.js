window.BENCHMARK_DATA = {
  "lastUpdate": 1791472614029,
  "repoUrl": "https://github.com/dartsim/dart",
  "entries": {
    "DART 6 deterministic counts": [
      {
        "commit": {
          "id": "789d3662c599a462ef1f9e95d969344416c80e63",
          "message": "v6.19.5-310-g789d3662c",
          "timestamp": "2026-10-08T11:14:41.549161+00:00",
          "committer": {
            "username": "github-actions[bot]"
          },
          "url": "https://github.com/dartsim/dart/commit/789d3662c599a462ef1f9e95d969344416c80e63"
        },
        "date": 1791458081549,
        "tool": "customSmallerIsBetter",
        "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
        "head_fingerprints": {
          "gzb/ode": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
          "robot/dart": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120"
        },
        "measurement": {
          "s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 92808902.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s3w/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 150207179.8,
              "allocs_per_step": 15750.0,
              "bytes_per_step": 2454390.4,
              "guards": {
                "hash": "0xfd39a7f4ed106476",
                "finite": true,
                "contacts": 9009,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 78750,
              "bytes": 12271952,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 1664.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x8b01e88923a385c",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xad6647b622395f5d",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/dart": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 11834811.16,
              "allocs_per_step": 573.38,
              "bytes_per_step": 399859.84,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 28669,
              "bytes": 19992992,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/ode": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 15632952.34,
              "allocs_per_step": 335.28,
              "bytes_per_step": 31914.32,
              "guards": {
                "hash": "0x4840d43f1b43c877",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 16764,
              "bytes": 1595716,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/dart": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 3962484.17,
              "allocs_per_step": 195.9,
              "bytes_per_step": 319468.8,
              "guards": {
                "hash": "0xb3ff9fa44c9a37aa",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 19590,
              "bytes": 31946880,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/fcl": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4118188.03,
              "allocs_per_step": 594.0,
              "bytes_per_step": 60528.0,
              "guards": {
                "hash": "0xeea3ab6aa3f85419",
                "finite": true,
                "contacts": 180,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 59400,
              "bytes": 6052800,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/bullet": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 5323248.66,
              "allocs_per_step": 0.25,
              "bytes_per_step": 12.48,
              "guards": {
                "hash": "0xcef699981debcdd9",
                "finite": true,
                "contacts": 268,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 25,
              "bytes": 1248,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/ode": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4857982.02,
              "allocs_per_step": 394.43,
              "bytes_per_step": 36928.16,
              "guards": {
                "hash": "0xd2fc4b3d63700bb",
                "finite": true,
                "contacts": 270,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 39443,
              "bytes": 3692816,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "pend/dart": {
            "version": 1,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 383391.262,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x4d0cffc02a6ceb19",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "0/22",
                "pairs": 0
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "gzb/ode": {
            "version": 1,
            "input_sha": "653c2c929d55103d9965d409e0fc6a8e42594ceecd6536129d3b558c471827d8",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": false,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 177609980.66666666,
              "allocs_per_step": 16707.666666666668,
              "bytes_per_step": 2405840.0,
              "guards": {
                "hash": "0xb72854800aade88b",
                "finite": true,
                "contacts": 10000,
                "cap_hit": true,
                "resting": "2002/3003"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 50123,
              "bytes": 7217520,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "robot/dart": {
            "version": 1,
            "input_sha": "e0d9d55b30c325159aa9c782348c105f694cf933389fc4b3b0f61475588004c2",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": false,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 13652376.61,
              "allocs_per_step": 22.0,
              "bytes_per_step": 226523.2,
              "guards": {
                "hash": "0xd8971b5a222b0da0",
                "finite": true,
                "contacts": 12,
                "cap_hit": false,
                "resting": "0/2"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2200,
              "bytes": 22652320,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "dyn": {
            "version": 1,
            "input_sha": "80b6ee9bea0966d0c570725a21a11f1af4a632dddaaf7d19000e83fdeb3d0c3e",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 14130818.8,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "BM_Dynamics/10": "0x28b48987064eb5de"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "BM_Dynamics/10"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "lcp": {
            "version": 1,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 1566397.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                  "solveNative/friction_32": "0xcacc7e802fd71fc7"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "solveNative/boxed_coupled_96",
                "solveNative/friction_32"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "mt4-s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "mt4-s1p/dart": {
            "version": 1,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 573.38,
              "bytes_per_step": 399859.84,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 28669,
              "bytes": 19992992,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          }
        },
        "benches": [
          {
            "name": "s3w/dart@1:cded8ae3 Ir",
            "value": 92808902.0,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 Ir",
            "value": 150207179.8,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 allocations",
            "value": 15750.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 Ir",
            "value": 1664.0,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/ode@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "s1p/dart@1:5bb85e65 Ir",
            "value": 11834811.16,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/dart@1:5bb85e65 allocations",
            "value": 573.38,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 Ir",
            "value": 15632952.34,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 allocations",
            "value": 335.28,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 Ir",
            "value": 3962484.17,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 allocations",
            "value": 195.9,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 Ir",
            "value": 4118188.03,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 allocations",
            "value": 594.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 Ir",
            "value": 5323248.66,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 allocations",
            "value": 0.25,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 Ir",
            "value": 4857982.02,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 allocations",
            "value": 394.43,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc Ir",
            "value": 383391.262,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "gzb/ode@1:653c2c92 Ir",
            "value": 177609980.66666666,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3608",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "653c2c929d55103d9965d409e0fc6a8e42594ceecd6536129d3b558c471827d8",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "gzb/ode@1:653c2c92 allocations",
            "value": 16707.666666666668,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3608",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "653c2c929d55103d9965d409e0fc6a8e42594ceecd6536129d3b558c471827d8",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:e0d9d55b Ir",
            "value": 13652376.61,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3608",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "e0d9d55b30c325159aa9c782348c105f694cf933389fc4b3b0f61475588004c2",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:e0d9d55b allocations",
            "value": 22.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3608",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "e0d9d55b30c325159aa9c782348c105f694cf933389fc4b3b0f61475588004c2",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "dyn@1:80b6ee9b Ir",
            "value": 14130818.8,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": true,
            "input_sha": "80b6ee9bea0966d0c570725a21a11f1af4a632dddaaf7d19000e83fdeb3d0c3e",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "dyn@1:80b6ee9b allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": true,
            "input_sha": "80b6ee9bea0966d0c570725a21a11f1af4a632dddaaf7d19000e83fdeb3d0c3e",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "lcp@1:c5fbcf1d Ir",
            "value": 1566397.0,
            "unit": "instructions / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "lcp@1:c5fbcf1d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "mt4-s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "mt4-s1p/dart@1:103d9699 allocations",
            "value": 573.38,
            "unit": "allocations / step",
            "extra": "fingerprint: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33\nPR #3608",
            "fingerprint": "3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33",
            "micro_instrumented": null,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null
          }
        ]
      },
      {
        "commit": {
          "id": "789d3662c599a462ef1f9e95d969344416c80e63",
          "message": "v6.19.5-310-g789d3662c",
          "timestamp": "2026-10-08T12:12:13.715794+00:00",
          "committer": {
            "username": "github-actions[bot]"
          },
          "url": "https://github.com/dartsim/dart/commit/789d3662c599a462ef1f9e95d969344416c80e63"
        },
        "date": 1791461533715,
        "tool": "customSmallerIsBetter",
        "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
        "head_fingerprints": {
          "gzb/ode": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
          "robot/dart": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120"
        },
        "measurement": {
          "s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 92808902.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s3w/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 150207179.8,
              "allocs_per_step": 15750.0,
              "bytes_per_step": 2454390.4,
              "guards": {
                "hash": "0xfd39a7f4ed106476",
                "finite": true,
                "contacts": 9009,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 78750,
              "bytes": 12271952,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 1664.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x8b01e88923a385c",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xad6647b622395f5d",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/dart": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 11834811.16,
              "allocs_per_step": 573.38,
              "bytes_per_step": 399859.84,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 28669,
              "bytes": 19992992,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/ode": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 15632952.34,
              "allocs_per_step": 335.28,
              "bytes_per_step": 31914.32,
              "guards": {
                "hash": "0x4840d43f1b43c877",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 16764,
              "bytes": 1595716,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/dart": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 19590,
                "bytes": 31946880,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 3962484.17,
              "allocs_per_step": 195.9,
              "bytes_per_step": 319468.8,
              "guards": {
                "hash": "0xb3ff9fa44c9a37aa",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 19590,
              "bytes": 31946880,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/fcl": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4118188.03,
              "allocs_per_step": 594.0,
              "bytes_per_step": 60528.0,
              "guards": {
                "hash": "0xeea3ab6aa3f85419",
                "finite": true,
                "contacts": 180,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 59400,
              "bytes": 6052800,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/bullet": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 5323248.66,
              "allocs_per_step": 0.25,
              "bytes_per_step": 12.48,
              "guards": {
                "hash": "0xcef699981debcdd9",
                "finite": true,
                "contacts": 268,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 25,
              "bytes": 1248,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/ode": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4857982.02,
              "allocs_per_step": 394.43,
              "bytes_per_step": 36928.16,
              "guards": {
                "hash": "0xd2fc4b3d63700bb",
                "finite": true,
                "contacts": 270,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 39443,
              "bytes": 3692816,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "pend/dart": {
            "version": 1,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 383391.262,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x4d0cffc02a6ceb19",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "0/22",
                "pairs": 0
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "gzb/ode": {
            "version": 1,
            "input_sha": "653c2c929d55103d9965d409e0fc6a8e42594ceecd6536129d3b558c471827d8",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": false,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 177609980.66666666,
              "allocs_per_step": 16707.666666666668,
              "bytes_per_step": 2405840.0,
              "guards": {
                "hash": "0xb72854800aade88b",
                "finite": true,
                "contacts": 10000,
                "cap_hit": true,
                "resting": "2002/3003"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 50123,
              "bytes": 7217520,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "robot/dart": {
            "version": 1,
            "input_sha": "e0d9d55b30c325159aa9c782348c105f694cf933389fc4b3b0f61475588004c2",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": false,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 2200,
                "bytes": 22652320,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 13652376.61,
              "allocs_per_step": 22.0,
              "bytes_per_step": 226523.2,
              "guards": {
                "hash": "0xd8971b5a222b0da0",
                "finite": true,
                "contacts": 12,
                "cap_hit": false,
                "resting": "0/2"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2200,
              "bytes": 22652320,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "dyn": {
            "version": 1,
            "input_sha": "80b6ee9bea0966d0c570725a21a11f1af4a632dddaaf7d19000e83fdeb3d0c3e",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 14130818.8,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "BM_Dynamics/10": "0x28b48987064eb5de"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "BM_Dynamics/10"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "lcp": {
            "version": 1,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)",
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 1566397.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                  "solveNative/friction_32": "0xcacc7e802fd71fc7"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "solveNative/boxed_coupled_96",
                "solveNative/friction_32"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "mt4-s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "mt4-s1p/dart": {
            "version": 1,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": null,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "allocs": 28669,
                "bytes": 19992992,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 573.38,
              "bytes_per_step": 399859.84,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 28669,
              "bytes": 19992992,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          }
        },
        "benches": [
          {
            "name": "s3w/dart@1:cded8ae3 Ir",
            "value": 92808902.0,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 Ir",
            "value": 150207179.8,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 allocations",
            "value": 15750.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 Ir",
            "value": 1664.0,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/ode@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "s1p/dart@1:5bb85e65 Ir",
            "value": 11834811.16,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/dart@1:5bb85e65 allocations",
            "value": 573.38,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 Ir",
            "value": 15632952.34,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 allocations",
            "value": 335.28,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 Ir",
            "value": 3962484.17,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 allocations",
            "value": 195.9,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 Ir",
            "value": 4118188.03,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 allocations",
            "value": 594.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 Ir",
            "value": 5323248.66,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 allocations",
            "value": 0.25,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 Ir",
            "value": 4857982.02,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 allocations",
            "value": 394.43,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc Ir",
            "value": 383391.262,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "gzb/ode@1:653c2c92 Ir",
            "value": 177609980.66666666,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3608",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "653c2c929d55103d9965d409e0fc6a8e42594ceecd6536129d3b558c471827d8",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "gzb/ode@1:653c2c92 allocations",
            "value": 16707.666666666668,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3608",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "653c2c929d55103d9965d409e0fc6a8e42594ceecd6536129d3b558c471827d8",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:e0d9d55b Ir",
            "value": 13652376.61,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3608",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "e0d9d55b30c325159aa9c782348c105f694cf933389fc4b3b0f61475588004c2",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:e0d9d55b allocations",
            "value": 22.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3608",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "e0d9d55b30c325159aa9c782348c105f694cf933389fc4b3b0f61475588004c2",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "dyn@1:80b6ee9b Ir",
            "value": 14130818.8,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": true,
            "input_sha": "80b6ee9bea0966d0c570725a21a11f1af4a632dddaaf7d19000e83fdeb3d0c3e",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "dyn@1:80b6ee9b allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": true,
            "input_sha": "80b6ee9bea0966d0c570725a21a11f1af4a632dddaaf7d19000e83fdeb3d0c3e",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "lcp@1:c5fbcf1d Ir",
            "value": 1566397.0,
            "unit": "instructions / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "lcp@1:c5fbcf1d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "mt4-s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "mt4-s1p/dart@1:103d9699 allocations",
            "value": 573.38,
            "unit": "allocations / step",
            "extra": "fingerprint: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96\nPR #3608\nfingerprint changed: 3c5512bba7b06165f11970fa7dfeae9e98db7eea0da5323758b2c28cd6381a33 -> 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "fingerprint": "1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96",
            "micro_instrumented": null,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null
          }
        ]
      },
      {
        "commit": {
          "id": "2b43b47fb15083e3fd8da4e2e197f8132716499b",
          "message": "v6.19.5-313-g2b43b47fb",
          "timestamp": "2026-10-08T12:24:52.794055+00:00",
          "committer": {
            "username": "github-actions[bot]"
          },
          "url": "https://github.com/dartsim/dart/commit/2b43b47fb15083e3fd8da4e2e197f8132716499b"
        },
        "date": 1791462292794,
        "tool": "customSmallerIsBetter",
        "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
        "head_fingerprints": {},
        "measurement": {
          "s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 92805044.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s3w/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 150207184.8,
              "allocs_per_step": 15750.0,
              "bytes_per_step": 2454390.4,
              "guards": {
                "hash": "0xfd39a7f4ed106476",
                "finite": true,
                "contacts": 9009,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 78750,
              "bytes": 12271952,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 1664.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x8b01e88923a385c",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xad6647b622395f5d",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/dart": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 11728869.36,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/ode": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 15632952.34,
              "allocs_per_step": 335.28,
              "bytes_per_step": 31914.32,
              "guards": {
                "hash": "0x4840d43f1b43c877",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.197723,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 16764,
              "bytes": 1595716,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/dart": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 3916006.7,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xb3ff9fa44c9a37aa",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.00113193,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/fcl": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4123468.03,
              "allocs_per_step": 594.0,
              "bytes_per_step": 60528.0,
              "guards": {
                "hash": "0xeea3ab6aa3f85419",
                "finite": true,
                "contacts": 180,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000536389,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 59400,
              "bytes": 6052800,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/bullet": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 5323248.66,
              "allocs_per_step": 0.25,
              "bytes_per_step": 12.48,
              "guards": {
                "hash": "0xcef699981debcdd9",
                "finite": true,
                "contacts": 268,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 2.10181e-05,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 25,
              "bytes": 1248,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/ode": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4857982.02,
              "allocs_per_step": 394.43,
              "bytes_per_step": 36928.16,
              "guards": {
                "hash": "0xd2fc4b3d63700bb",
                "finite": true,
                "contacts": 270,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000105695,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 39443,
              "bytes": 3692816,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "pend/dart": {
            "version": 1,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 383391.262,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x4d0cffc02a6ceb19",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "0/22",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "gzb/ode": {
            "version": 1,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 177631147.0,
              "allocs_per_step": 16707.666666666668,
              "bytes_per_step": 2405840.0,
              "guards": {
                "hash": "0xb72854800aade88b",
                "finite": true,
                "contacts": 10000,
                "cap_hit": true,
                "resting": "2002/3003"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 50123,
              "bytes": 7217520,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "robot/dart": {
            "version": 1,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 12653564.08,
              "allocs_per_step": 19.98,
              "bytes_per_step": 226200.0,
              "guards": {
                "hash": "0xd8971b5a222b0da0",
                "finite": true,
                "contacts": 12,
                "cap_hit": false,
                "resting": "0/2"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 1998,
              "bytes": 22620000,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "dyn": {
            "version": 1,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 14130898.8,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "BM_Dynamics/10": "0x28b48987064eb5de"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "BM_Dynamics/10"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "lcp": {
            "version": 1,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 1566397.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                  "solveNative/friction_32": "0xcacc7e802fd71fc7"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "solveNative/boxed_coupled_96",
                "solveNative/friction_32"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "mt4-s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "mt4-s1p/dart": {
            "version": 1,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          }
        },
        "benches": [
          {
            "name": "s3w/dart@1:cded8ae3 Ir",
            "value": 92805044.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 Ir",
            "value": 150207184.8,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 allocations",
            "value": 15750.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 Ir",
            "value": 1664.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/ode@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "s1p/dart@1:5bb85e65 Ir",
            "value": 11728869.36,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/dart@1:5bb85e65 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 Ir",
            "value": 15632952.34,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 allocations",
            "value": 335.28,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 Ir",
            "value": 3916006.7,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 Ir",
            "value": 4123468.03,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 allocations",
            "value": 594.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 Ir",
            "value": 5323248.66,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 allocations",
            "value": 0.25,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 Ir",
            "value": 4857982.02,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 allocations",
            "value": 394.43,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc Ir",
            "value": 383391.262,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "gzb/ode@1:80bf1c23 Ir",
            "value": 177631147.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "gzb/ode@1:80bf1c23 allocations",
            "value": 16707.666666666668,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:1ce2e8d9 Ir",
            "value": 12653564.08,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:1ce2e8d9 allocations",
            "value": 19.98,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "dyn@1:cf10671b Ir",
            "value": 14130898.8,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "dyn@1:cf10671b allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "lcp@1:c5fbcf1d Ir",
            "value": 1566397.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "lcp@1:c5fbcf1d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "mt4-s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "mt4-s1p/dart@1:103d9699 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3605\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null
          }
        ]
      },
      {
        "commit": {
          "id": "973c2b70dcb8b15abfc34795093bac8b319d394a",
          "message": "v6.19.5-311-g973c2b70d",
          "timestamp": "2026-10-08T12:27:46.974982+00:00",
          "committer": {
            "username": "github-actions[bot]"
          },
          "url": "https://github.com/dartsim/dart/commit/973c2b70dcb8b15abfc34795093bac8b319d394a"
        },
        "date": 1791462466974,
        "tool": "customSmallerIsBetter",
        "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
        "head_fingerprints": {},
        "measurement": {
          "s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 92805044.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s3w/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 150207184.8,
              "allocs_per_step": 15750.0,
              "bytes_per_step": 2454390.4,
              "guards": {
                "hash": "0xfd39a7f4ed106476",
                "finite": true,
                "contacts": 9009,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 78750,
              "bytes": 12271952,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 1664.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x8b01e88923a385c",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xad6647b622395f5d",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/dart": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 11728869.36,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/ode": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 15632952.34,
              "allocs_per_step": 335.28,
              "bytes_per_step": 31914.32,
              "guards": {
                "hash": "0x4840d43f1b43c877",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.197723,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 16764,
              "bytes": 1595716,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/dart": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 3916006.7,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xb3ff9fa44c9a37aa",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.00113193,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/fcl": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4123468.03,
              "allocs_per_step": 594.0,
              "bytes_per_step": 60528.0,
              "guards": {
                "hash": "0xeea3ab6aa3f85419",
                "finite": true,
                "contacts": 180,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000536389,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 59400,
              "bytes": 6052800,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/bullet": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 5323248.66,
              "allocs_per_step": 0.25,
              "bytes_per_step": 12.48,
              "guards": {
                "hash": "0xcef699981debcdd9",
                "finite": true,
                "contacts": 268,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 2.10181e-05,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 25,
              "bytes": 1248,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/ode": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4857982.02,
              "allocs_per_step": 394.43,
              "bytes_per_step": 36928.16,
              "guards": {
                "hash": "0xd2fc4b3d63700bb",
                "finite": true,
                "contacts": 270,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000105695,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 39443,
              "bytes": 3692816,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "pend/dart": {
            "version": 1,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 383391.262,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x4d0cffc02a6ceb19",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "0/22",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "gzb/ode": {
            "version": 1,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 177631147.0,
              "allocs_per_step": 16707.666666666668,
              "bytes_per_step": 2405840.0,
              "guards": {
                "hash": "0xb72854800aade88b",
                "finite": true,
                "contacts": 10000,
                "cap_hit": true,
                "resting": "2002/3003"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 50123,
              "bytes": 7217520,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "robot/dart": {
            "version": 1,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 12653564.08,
              "allocs_per_step": 19.98,
              "bytes_per_step": 226200.0,
              "guards": {
                "hash": "0xd8971b5a222b0da0",
                "finite": true,
                "contacts": 12,
                "cap_hit": false,
                "resting": "0/2"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 1998,
              "bytes": 22620000,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "dyn": {
            "version": 1,
            "input_sha": "80b6ee9bea0966d0c570725a21a11f1af4a632dddaaf7d19000e83fdeb3d0c3e",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 14130818.8,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "BM_Dynamics/10": "0x28b48987064eb5de"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "BM_Dynamics/10"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "lcp": {
            "version": 1,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 1566397.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                  "solveNative/friction_32": "0xcacc7e802fd71fc7"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "solveNative/boxed_coupled_96",
                "solveNative/friction_32"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "mt4-s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "mt4-s1p/dart": {
            "version": 1,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          }
        },
        "benches": [
          {
            "name": "s3w/dart@1:cded8ae3 Ir",
            "value": 92805044.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 Ir",
            "value": 150207184.8,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 allocations",
            "value": 15750.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 Ir",
            "value": 1664.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/ode@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "s1p/dart@1:5bb85e65 Ir",
            "value": 11728869.36,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/dart@1:5bb85e65 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 Ir",
            "value": 15632952.34,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 allocations",
            "value": 335.28,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 Ir",
            "value": 3916006.7,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 Ir",
            "value": 4123468.03,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 allocations",
            "value": 594.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 Ir",
            "value": 5323248.66,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 allocations",
            "value": 0.25,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 Ir",
            "value": 4857982.02,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 allocations",
            "value": 394.43,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc Ir",
            "value": 383391.262,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "gzb/ode@1:80bf1c23 Ir",
            "value": 177631147.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "gzb/ode@1:80bf1c23 allocations",
            "value": 16707.666666666668,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:1ce2e8d9 Ir",
            "value": 12653564.08,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:1ce2e8d9 allocations",
            "value": 19.98,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "dyn@1:80b6ee9b Ir",
            "value": 14130818.8,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "80b6ee9bea0966d0c570725a21a11f1af4a632dddaaf7d19000e83fdeb3d0c3e",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "dyn@1:80b6ee9b allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594\nfingerprint changed: 1cf0157b225bfa1a5fdb9385eede484699082beabcbd9dab24b69db42ba0fb96 -> a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "80b6ee9bea0966d0c570725a21a11f1af4a632dddaaf7d19000e83fdeb3d0c3e",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "lcp@1:c5fbcf1d Ir",
            "value": 1566397.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "lcp@1:c5fbcf1d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "mt4-s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "mt4-s1p/dart@1:103d9699 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3594",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null
          }
        ]
      },
      {
        "commit": {
          "id": "215989dfb2399e9574ee26ee25d210ec53dce95c",
          "message": "v6.19.5-314-g215989dfb",
          "timestamp": "2026-10-08T12:54:35.877404+00:00",
          "committer": {
            "username": "github-actions[bot]"
          },
          "url": "https://github.com/dartsim/dart/commit/215989dfb2399e9574ee26ee25d210ec53dce95c"
        },
        "date": 1791464075877,
        "tool": "customSmallerIsBetter",
        "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
        "head_fingerprints": {},
        "measurement": {
          "s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 92910517.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s3w/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 150431791.2,
              "allocs_per_step": 15750.0,
              "bytes_per_step": 2454390.4,
              "guards": {
                "hash": "0xfd39a7f4ed106476",
                "finite": true,
                "contacts": 9009,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 78750,
              "bytes": 12271952,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 1664.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x8b01e88923a385c",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xad6647b622395f5d",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/dart": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 11734008.04,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/ode": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 15635962.98,
              "allocs_per_step": 335.28,
              "bytes_per_step": 31914.32,
              "guards": {
                "hash": "0x4840d43f1b43c877",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.197723,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 16764,
              "bytes": 1595716,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/dart": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 3920015.86,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xb3ff9fa44c9a37aa",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.00113193,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/fcl": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4127825.03,
              "allocs_per_step": 594.0,
              "bytes_per_step": 60528.0,
              "guards": {
                "hash": "0xeea3ab6aa3f85419",
                "finite": true,
                "contacts": 180,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000536389,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 59400,
              "bytes": 6052800,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/bullet": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 5327451.17,
              "allocs_per_step": 0.25,
              "bytes_per_step": 12.48,
              "guards": {
                "hash": "0xcef699981debcdd9",
                "finite": true,
                "contacts": 268,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 2.10181e-05,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 25,
              "bytes": 1248,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/ode": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4861698.33,
              "allocs_per_step": 394.43,
              "bytes_per_step": 36928.16,
              "guards": {
                "hash": "0xd2fc4b3d63700bb",
                "finite": true,
                "contacts": 270,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000105695,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 39443,
              "bytes": 3692816,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "pend/dart": {
            "version": 1,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 384494.262,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x4d0cffc02a6ceb19",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "0/22",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "gzb/ode": {
            "version": 1,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 177748624.0,
              "allocs_per_step": 16707.666666666668,
              "bytes_per_step": 2405840.0,
              "guards": {
                "hash": "0xb72854800aade88b",
                "finite": true,
                "contacts": 10000,
                "cap_hit": true,
                "resting": "2002/3003"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 50123,
              "bytes": 7217520,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "robot/dart": {
            "version": 1,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 12653346.43,
              "allocs_per_step": 19.98,
              "bytes_per_step": 226200.0,
              "guards": {
                "hash": "0xd8971b5a222b0da0",
                "finite": true,
                "contacts": 12,
                "cap_hit": false,
                "resting": "0/2"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 1998,
              "bytes": 22620000,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "dyn": {
            "version": 1,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 14142470.8,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "BM_Dynamics/10": "0x28b48987064eb5de"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "BM_Dynamics/10"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "lcp": {
            "version": 1,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 1566397.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                  "solveNative/friction_32": "0xcacc7e802fd71fc7"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "solveNative/boxed_coupled_96",
                "solveNative/friction_32"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "mt4-s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "mt4-s1p/dart": {
            "version": 1,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          }
        },
        "benches": [
          {
            "name": "s3w/dart@1:cded8ae3 Ir",
            "value": 92910517.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 Ir",
            "value": 150431791.2,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 allocations",
            "value": 15750.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 Ir",
            "value": 1664.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/ode@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "s1p/dart@1:5bb85e65 Ir",
            "value": 11734008.04,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/dart@1:5bb85e65 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 Ir",
            "value": 15635962.98,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 allocations",
            "value": 335.28,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 Ir",
            "value": 3920015.86,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 Ir",
            "value": 4127825.03,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 allocations",
            "value": 594.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 Ir",
            "value": 5327451.17,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 allocations",
            "value": 0.25,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 Ir",
            "value": 4861698.33,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 allocations",
            "value": 394.43,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc Ir",
            "value": 384494.262,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "gzb/ode@1:80bf1c23 Ir",
            "value": 177748624.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "gzb/ode@1:80bf1c23 allocations",
            "value": 16707.666666666668,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:1ce2e8d9 Ir",
            "value": 12653346.43,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:1ce2e8d9 allocations",
            "value": 19.98,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "dyn@1:cf10671b Ir",
            "value": 14142470.8,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "dyn@1:cf10671b allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "lcp@1:c5fbcf1d Ir",
            "value": 1566397.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "lcp@1:c5fbcf1d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "mt4-s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "mt4-s1p/dart@1:103d9699 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3607",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null
          }
        ]
      },
      {
        "commit": {
          "id": "6e942a87291742e163fd1782bd69aac71df0f81f",
          "message": "v6.19.5-317-g6e942a872",
          "timestamp": "2026-10-08T14:22:34.889121+00:00",
          "committer": {
            "username": "github-actions[bot]"
          },
          "url": "https://github.com/dartsim/dart/commit/6e942a87291742e163fd1782bd69aac71df0f81f"
        },
        "date": 1791469354889,
        "tool": "customSmallerIsBetter",
        "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
        "head_fingerprints": {},
        "measurement": {
          "s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 92910517.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s3w/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 150431791.2,
              "allocs_per_step": 15750.0,
              "bytes_per_step": 2454390.4,
              "guards": {
                "hash": "0xfd39a7f4ed106476",
                "finite": true,
                "contacts": 9009,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 78750,
              "bytes": 12271952,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 1664.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x8b01e88923a385c",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xad6647b622395f5d",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/dart": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 11734008.04,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/ode": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 15635962.98,
              "allocs_per_step": 335.28,
              "bytes_per_step": 31914.32,
              "guards": {
                "hash": "0x4840d43f1b43c877",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.197723,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 16764,
              "bytes": 1595716,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/dart": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 3920015.86,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xb3ff9fa44c9a37aa",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.00113193,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/fcl": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4127825.03,
              "allocs_per_step": 594.0,
              "bytes_per_step": 60528.0,
              "guards": {
                "hash": "0xeea3ab6aa3f85419",
                "finite": true,
                "contacts": 180,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000536389,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 59400,
              "bytes": 6052800,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/bullet": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 5327451.17,
              "allocs_per_step": 0.25,
              "bytes_per_step": 12.48,
              "guards": {
                "hash": "0xcef699981debcdd9",
                "finite": true,
                "contacts": 268,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 2.10181e-05,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 25,
              "bytes": 1248,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/ode": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4861698.33,
              "allocs_per_step": 394.43,
              "bytes_per_step": 36928.16,
              "guards": {
                "hash": "0xd2fc4b3d63700bb",
                "finite": true,
                "contacts": 270,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000105695,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 39443,
              "bytes": 3692816,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "pend/dart": {
            "version": 1,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 384494.262,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x4d0cffc02a6ceb19",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "0/22",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "gzb/ode": {
            "version": 1,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 177748624.0,
              "allocs_per_step": 16707.666666666668,
              "bytes_per_step": 2405840.0,
              "guards": {
                "hash": "0xb72854800aade88b",
                "finite": true,
                "contacts": 10000,
                "cap_hit": true,
                "resting": "2002/3003"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 50123,
              "bytes": 7217520,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "robot/dart": {
            "version": 1,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 12653346.43,
              "allocs_per_step": 19.98,
              "bytes_per_step": 226200.0,
              "guards": {
                "hash": "0xd8971b5a222b0da0",
                "finite": true,
                "contacts": 12,
                "cap_hit": false,
                "resting": "0/2"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 1998,
              "bytes": 22620000,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "dyn": {
            "version": 1,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 14142470.8,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "BM_Dynamics/10": "0x28b48987064eb5de"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "BM_Dynamics/10"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "lcp": {
            "version": 1,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 1566397.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                  "solveNative/friction_32": "0xcacc7e802fd71fc7"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "solveNative/boxed_coupled_96",
                "solveNative/friction_32"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "mt4-s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "mt4-s1p/dart": {
            "version": 1,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          }
        },
        "benches": [
          {
            "name": "s3w/dart@1:cded8ae3 Ir",
            "value": 92910517.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 Ir",
            "value": 150431791.2,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 allocations",
            "value": 15750.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 Ir",
            "value": 1664.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/ode@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "s1p/dart@1:5bb85e65 Ir",
            "value": 11734008.04,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/dart@1:5bb85e65 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 Ir",
            "value": 15635962.98,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 allocations",
            "value": 335.28,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 Ir",
            "value": 3920015.86,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 Ir",
            "value": 4127825.03,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 allocations",
            "value": 594.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 Ir",
            "value": 5327451.17,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 allocations",
            "value": 0.25,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 Ir",
            "value": 4861698.33,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 allocations",
            "value": 394.43,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc Ir",
            "value": 384494.262,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "gzb/ode@1:80bf1c23 Ir",
            "value": 177748624.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "gzb/ode@1:80bf1c23 allocations",
            "value": 16707.666666666668,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:1ce2e8d9 Ir",
            "value": 12653346.43,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:1ce2e8d9 allocations",
            "value": 19.98,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "dyn@1:cf10671b Ir",
            "value": 14142470.8,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "dyn@1:cf10671b allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "lcp@1:c5fbcf1d Ir",
            "value": 1566397.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "lcp@1:c5fbcf1d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "mt4-s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "mt4-s1p/dart@1:103d9699 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3610",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null
          }
        ]
      },
      {
        "commit": {
          "id": "044ec3378cbc09f28808dca2fff40d2b953c2892",
          "message": "v6.19.5-318-g044ec3378",
          "timestamp": "2026-10-08T14:52:48.598406+00:00",
          "committer": {
            "username": "github-actions[bot]"
          },
          "url": "https://github.com/dartsim/dart/commit/044ec3378cbc09f28808dca2fff40d2b953c2892"
        },
        "date": 1791471168598,
        "tool": "customSmallerIsBetter",
        "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
        "head_fingerprints": {
          "gzb/ode": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
          "robot/dart": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0"
        },
        "measurement": {
          "s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 92910517.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s3w/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 150431791.2,
              "allocs_per_step": 15750.0,
              "bytes_per_step": 2454390.4,
              "guards": {
                "hash": "0xfd39a7f4ed106476",
                "finite": true,
                "contacts": 9009,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 78750,
              "bytes": 12271952,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 1664.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x8b01e88923a385c",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xad6647b622395f5d",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/dart": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 11734008.04,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/ode": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 15635962.98,
              "allocs_per_step": 335.28,
              "bytes_per_step": 31914.32,
              "guards": {
                "hash": "0x4840d43f1b43c877",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.197723,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 16764,
              "bytes": 1595716,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/dart": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 3920015.86,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xb3ff9fa44c9a37aa",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.00113193,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/fcl": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4127825.03,
              "allocs_per_step": 594.0,
              "bytes_per_step": 60528.0,
              "guards": {
                "hash": "0xeea3ab6aa3f85419",
                "finite": true,
                "contacts": 180,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000536389,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 59400,
              "bytes": 6052800,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/bullet": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 5327451.17,
              "allocs_per_step": 0.25,
              "bytes_per_step": 12.48,
              "guards": {
                "hash": "0xcef699981debcdd9",
                "finite": true,
                "contacts": 268,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 2.10181e-05,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 25,
              "bytes": 1248,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/ode": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4861698.33,
              "allocs_per_step": 394.43,
              "bytes_per_step": 36928.16,
              "guards": {
                "hash": "0xd2fc4b3d63700bb",
                "finite": true,
                "contacts": 270,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000105695,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 39443,
              "bytes": 3692816,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "pend/dart": {
            "version": 1,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 384494.262,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x4d0cffc02a6ceb19",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "0/22",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "gzb/ode": {
            "version": 1,
            "input_sha": "653c2c929d55103d9965d409e0fc6a8e42594ceecd6536129d3b558c471827d8",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": false,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 177737993.66666666,
              "allocs_per_step": 16707.666666666668,
              "bytes_per_step": 2405840.0,
              "guards": {
                "hash": "0xb72854800aade88b",
                "finite": true,
                "contacts": 10000,
                "cap_hit": true,
                "resting": "2002/3003"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 50123,
              "bytes": 7217520,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "robot/dart": {
            "version": 1,
            "input_sha": "e0d9d55b30c325159aa9c782348c105f694cf933389fc4b3b0f61475588004c2",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": false,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 12653443.52,
              "allocs_per_step": 19.98,
              "bytes_per_step": 226200.0,
              "guards": {
                "hash": "0xd8971b5a222b0da0",
                "finite": true,
                "contacts": 12,
                "cap_hit": false,
                "resting": "0/2"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 1998,
              "bytes": 22620000,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "dyn": {
            "version": 1,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 14142470.8,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "BM_Dynamics/10": "0x28b48987064eb5de"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "BM_Dynamics/10"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "lcp": {
            "version": 1,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 1566397.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                  "solveNative/friction_32": "0xcacc7e802fd71fc7"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "solveNative/boxed_coupled_96",
                "solveNative/friction_32"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "mt4-s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "mt4-s1p/dart": {
            "version": 1,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          }
        },
        "benches": [
          {
            "name": "s3w/dart@1:cded8ae3 Ir",
            "value": 92910517.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 Ir",
            "value": 150431791.2,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 allocations",
            "value": 15750.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 Ir",
            "value": 1664.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/ode@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "s1p/dart@1:5bb85e65 Ir",
            "value": 11734008.04,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/dart@1:5bb85e65 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 Ir",
            "value": 15635962.98,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 allocations",
            "value": 335.28,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 Ir",
            "value": 3920015.86,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 Ir",
            "value": 4127825.03,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 allocations",
            "value": 594.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 Ir",
            "value": 5327451.17,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 allocations",
            "value": 0.25,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 Ir",
            "value": 4861698.33,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 allocations",
            "value": 394.43,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc Ir",
            "value": 384494.262,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "gzb/ode@1:653c2c92 Ir",
            "value": 177737993.66666666,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3616\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "653c2c929d55103d9965d409e0fc6a8e42594ceecd6536129d3b558c471827d8",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "gzb/ode@1:653c2c92 allocations",
            "value": 16707.666666666668,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3616\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "653c2c929d55103d9965d409e0fc6a8e42594ceecd6536129d3b558c471827d8",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:e0d9d55b Ir",
            "value": 12653443.52,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3616\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "e0d9d55b30c325159aa9c782348c105f694cf933389fc4b3b0f61475588004c2",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:e0d9d55b allocations",
            "value": 19.98,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3616\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "e0d9d55b30c325159aa9c782348c105f694cf933389fc4b3b0f61475588004c2",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "dyn@1:cf10671b Ir",
            "value": 14142470.8,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "dyn@1:cf10671b allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "lcp@1:c5fbcf1d Ir",
            "value": 1566397.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "lcp@1:c5fbcf1d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "mt4-s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "mt4-s1p/dart@1:103d9699 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120\nPR #3616",
            "fingerprint": "a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120",
            "micro_instrumented": null,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null
          }
        ]
      },
      {
        "commit": {
          "id": "55fb991b57782dc7ec59c12d02d6ca7b8432dbaa",
          "message": "v6.19.5-321-g55fb991b5",
          "timestamp": "2026-10-08T15:16:54.029027+00:00",
          "committer": {
            "username": "github-actions[bot]"
          },
          "url": "https://github.com/dartsim/dart/commit/55fb991b57782dc7ec59c12d02d6ca7b8432dbaa"
        },
        "date": 1791472614029,
        "tool": "customSmallerIsBetter",
        "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
        "head_fingerprints": {},
        "measurement": {
          "s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 92950750.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s3w/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xfd39a7f4ed106476",
                  "finite": true,
                  "contacts": 9009,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 78750,
                "bytes": 12271952,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 150431681.2,
              "allocs_per_step": 15750.0,
              "bytes_per_step": 2454390.4,
              "guards": {
                "hash": "0xfd39a7f4ed106476",
                "finite": true,
                "contacts": 9009,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 78750,
              "bytes": 12271952,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x8b01e88923a385c",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 1664.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x8b01e88923a385c",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s2r/ode": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xad6647b622395f5d",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "3003/3003",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xad6647b622395f5d",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "3003/3003",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/dart": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 11732202.88,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/ode": {
            "version": 1,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4840d43f1b43c877",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.197723,
                "checkpoints": null,
                "allocs": 16764,
                "bytes": 1595716,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 15636155.58,
              "allocs_per_step": 335.28,
              "bytes_per_step": 31914.32,
              "guards": {
                "hash": "0x4840d43f1b43c877",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.197723,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 16764,
              "bytes": 1595716,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/dart": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb3ff9fa44c9a37aa",
                  "finite": true,
                  "contacts": 150,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.00113193,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 3919408.82,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xb3ff9fa44c9a37aa",
                "finite": true,
                "contacts": 150,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.00113193,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/fcl": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xeea3ab6aa3f85419",
                  "finite": true,
                  "contacts": 180,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000536389,
                "checkpoints": null,
                "allocs": 59400,
                "bytes": 6052800,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4127791.03,
              "allocs_per_step": 594.0,
              "bytes_per_step": 60528.0,
              "guards": {
                "hash": "0xeea3ab6aa3f85419",
                "finite": true,
                "contacts": 180,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000536389,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 59400,
              "bytes": 6052800,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/bullet": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xcef699981debcdd9",
                  "finite": true,
                  "contacts": 268,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 2.10181e-05,
                "checkpoints": null,
                "allocs": 25,
                "bytes": 1248,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 5327377.05,
              "allocs_per_step": 0.25,
              "bytes_per_step": 12.48,
              "guards": {
                "hash": "0xcef699981debcdd9",
                "finite": true,
                "contacts": 268,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 2.10181e-05,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 25,
              "bytes": 1248,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s5a/ode": {
            "version": 1,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd2fc4b3d63700bb",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 0.000105695,
                "checkpoints": null,
                "allocs": 39443,
                "bytes": 3692816,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 4862170.15,
              "allocs_per_step": 394.43,
              "bytes_per_step": 36928.16,
              "guards": {
                "hash": "0xd2fc4b3d63700bb",
                "finite": true,
                "contacts": 270,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 0.000105695,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 39443,
              "bytes": 3692816,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "pend/dart": {
            "version": 1,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x4d0cffc02a6ceb19",
                  "finite": true,
                  "contacts": 0,
                  "cap_hit": false,
                  "resting": "0/22",
                  "pairs": 0
                },
                "max_penetration": 0.0,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 384395.262,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x4d0cffc02a6ceb19",
                "finite": true,
                "contacts": 0,
                "cap_hit": false,
                "resting": "0/22",
                "pairs": 0
              },
              "max_penetration": 0.0,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "gzb/ode": {
            "version": 1,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xb72854800aade88b",
                  "finite": true,
                  "contacts": 10000,
                  "cap_hit": true,
                  "resting": "2002/3003"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 50123,
                "bytes": 7217520,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 177752177.66666666,
              "allocs_per_step": 16707.666666666668,
              "bytes_per_step": 2405840.0,
              "guards": {
                "hash": "0xb72854800aade88b",
                "finite": true,
                "contacts": 10000,
                "cap_hit": true,
                "resting": "2002/3003"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 50123,
              "bytes": 7217520,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "robot/dart": {
            "version": 1,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xd8971b5a222b0da0",
                  "finite": true,
                  "contacts": 12,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1998,
                "bytes": 22620000,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 12654512.08,
              "allocs_per_step": 19.98,
              "bytes_per_step": 226200.0,
              "guards": {
                "hash": "0xd8971b5a222b0da0",
                "finite": true,
                "contacts": 12,
                "cap_hit": false,
                "resting": "0/2"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 1998,
              "bytes": 22620000,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "dyn": {
            "version": 1,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "BM_Dynamics/10": "0x28b48987064eb5de"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 14141500.8,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "BM_Dynamics/10": "0x28b48987064eb5de"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "BM_Dynamics/10"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "lcp": {
            "version": 1,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)",
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": {
                    "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                    "solveNative/friction_32": "0xcacc7e802fd71fc7"
                  },
                  "finite": true
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": null
              }
            },
            "head": {
              "ir_per_step": 1566397.0,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": {
                  "solveNative/boxed_coupled_96": "0x7bd94fa4ccbc20e8",
                  "solveNative/friction_32": "0xcacc7e802fd71fc7"
                },
                "finite": true
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": null,
              "allocs": 0,
              "bytes": 0,
              "cases": [
                "solveNative/boxed_coupled_96",
                "solveNative/friction_32"
              ],
              "micro_instrumented": true,
              "est_cycles_per_step": null
            }
          },
          "mt4-s3w/dart": {
            "version": 1,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xbd65cadfdfbc8eb7",
                  "finite": true,
                  "contacts": 3003,
                  "cap_hit": false,
                  "resting": "0/3003",
                  "pairs": 3003
                },
                "max_penetration": 8.48395e-10,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xbd65cadfdfbc8eb7",
                "finite": true,
                "contacts": 3003,
                "cap_hit": false,
                "resting": "0/3003",
                "pairs": 3003
              },
              "max_penetration": 8.48395e-10,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "mt4-s1p/dart": {
            "version": 1,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": true,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x1c42939f04cb569a",
                  "finite": true,
                  "contacts": 89,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 79
                },
                "max_penetration": 0.18711,
                "checkpoints": null,
                "allocs": 2,
                "bytes": 14848,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": null,
              "allocs_per_step": 0.04,
              "bytes_per_step": 296.96,
              "guards": {
                "hash": "0x1c42939f04cb569a",
                "finite": true,
                "contacts": 89,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 79
              },
              "max_penetration": 0.18711,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 2,
              "bytes": 14848,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          }
        },
        "benches": [
          {
            "name": "s3w/dart@1:cded8ae3 Ir",
            "value": 92950750.0,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 Ir",
            "value": 150431681.2,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:cded8ae3 allocations",
            "value": 15750.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 Ir",
            "value": 1664.0,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/ode@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "s1p/dart@1:5bb85e65 Ir",
            "value": 11732202.88,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/dart@1:5bb85e65 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 Ir",
            "value": 15636155.58,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:5bb85e65 allocations",
            "value": 335.28,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "5bb85e65d3f31297857af83ce0bf14ced69bf494302a8aa02e3930ed6189e9bf",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 Ir",
            "value": 3919408.82,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:78353708 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 Ir",
            "value": 4127791.03,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:78353708 allocations",
            "value": 594.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 Ir",
            "value": 5327377.05,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:78353708 allocations",
            "value": 0.25,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 Ir",
            "value": 4862170.15,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:78353708 allocations",
            "value": 394.43,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "7835370825a61d4b881ffd510d55bbfb9ef5d40d8de30037ec3ffeddae236a3a",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc Ir",
            "value": 384395.262,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:9ff386cc allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "9ff386ccaeddef0831308c3de558617324e3608ece6716806b109ac7dd9062ae",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "gzb/ode@1:80bf1c23 Ir",
            "value": 177752177.66666666,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "gzb/ode@1:80bf1c23 allocations",
            "value": 16707.666666666668,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "80bf1c239bf045b612f37692f72a2e5f297c1c7bf5103240a6f4ba1dea8eae39",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:1ce2e8d9 Ir",
            "value": 12654512.08,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:1ce2e8d9 allocations",
            "value": 19.98,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "1ce2e8d940989322a00d3ac3f3fec6ede9c59bd52abed0a6244f5411b8350617",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "dyn@1:cf10671b Ir",
            "value": 14141500.8,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": true,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "dyn@1:cf10671b allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": true,
            "input_sha": "cf10671bac3b81b10b358b3d7d9d7d328abe29a918578ddd7f9c9bcc8141e867",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "lcp@1:c5fbcf1d Ir",
            "value": 1566397.0,
            "unit": "instructions / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "lcp@1:c5fbcf1d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": true,
            "input_sha": "c5fbcf1d112548f4f3531417ff2111d76543be03e63da2ed0967e15b7d5f8096",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "mt4-s3w/dart@1:cded8ae3 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "cded8ae351c45b42d65243fd3ab3ee28272c1e39243bb405f967400380e73ce0",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "mt4-s1p/dart@1:103d9699 allocations",
            "value": 0.04,
            "unit": "allocations / step",
            "extra": "fingerprint: 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0\nPR #3615\nfingerprint changed: a405f899efe1c6c29dc72958f0cb5ccd5650ec12d0364dfc5b93a6b3208a4120 -> 07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "fingerprint": "07fe2c8bea6456e1fcd12b4c29dd123048a081a58ae8337fc8c98062e96c2bc0",
            "micro_instrumented": null,
            "input_sha": "103d9699d9db9315527c198c3d5f98eac7cd318b90cc41629ae8c2dab48ea275",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null
          }
        ]
      }
    ]
  }
}
