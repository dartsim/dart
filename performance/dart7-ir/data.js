window.BENCHMARK_DATA = {
  "lastUpdate": 1791760022146,
  "repoUrl": "https://github.com/dartsim/dart",
  "entries": {
    "DART 7 deterministic counts": [
      {
        "commit": {
          "id": "354e46c65e37e5b6fb77044e951dcacc81ca7082",
          "message": "v6.20.0-35-g354e46c65",
          "timestamp": "2026-10-11T22:58:15.715530+00:00",
          "committer": {
            "username": "github-actions[bot]"
          },
          "url": "https://github.com/dartsim/dart/commit/354e46c65e37e5b6fb77044e951dcacc81ca7082"
        },
        "date": 1791759495715,
        "tool": "customSmallerIsBetter",
        "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
        "head_fingerprints": {},
        "measurement": {
          "s3w/dart": {
            "version": 1,
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
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
              "ir_per_step": 93033935.0,
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
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
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
              "ir_per_step": 150658901.2,
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
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
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
              "ir_per_step": 1607.0,
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
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
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
            "input_sha": "f381f8a4e35376fce479416bb6b29a996075a0fcb3366143d8f3639dc9f56d60",
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
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 11834947.2,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x3f41cf22f5a9f2ff",
                "finite": true,
                "contacts": 92,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 78
              },
              "max_penetration": 0.18712,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/ode": {
            "version": 1,
            "input_sha": "f381f8a4e35376fce479416bb6b29a996075a0fcb3366143d8f3639dc9f56d60",
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
              "ir_per_step": 15634301.28,
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
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
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
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 5079137.94,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xfa5f6b7a91e0df5e",
                "finite": true,
                "contacts": 270,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 8.48238e-09,
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
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
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
              "ir_per_step": 4128841.03,
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
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
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
              "ir_per_step": 5327739.29,
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
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
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
              "ir_per_step": 4861929.1,
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
            "input_sha": "de429022c113432100a3e265606855cdbebfec96fe60c9f70cf4177adeb0845f",
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
              "ir_per_step": 361642.15,
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
            "input_sha": "9c5a64498b78952b8d532441489d434491c3397e127bbfc455e60e6e5480eee5",
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
              "ir_per_step": 174072683.0,
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
            "input_sha": "343fc4cb6879164ccfe46f5d0b0a41657fd0cc303217912551c7dbaed41c4afc",
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
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 12631413.49,
              "allocs_per_step": 16.0,
              "bytes_per_step": 223104.0,
              "guards": {
                "hash": "0xf3016c0eb10fdb15",
                "finite": true,
                "contacts": 8,
                "cap_hit": false,
                "resting": "0/2"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 1600,
              "bytes": 22310400,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "dyn": {
            "version": 1,
            "input_sha": "64f3174f2ca3dad684382054ac8dfb74c93466b50dda3be3f93b673a11a9f3ff",
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
              "ir_per_step": 12320997.8,
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
            "input_sha": "449e0b2f49ee644d60c4dbaece4bffb0bd9761e503575c6111a06bc6e996bd4b",
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
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
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
            "input_sha": "d1003a9e7a6abe0238b22ae747d87fc110da8ae288117b3937f3edfd64f65f5d",
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
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
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
                "hash": "0x3f41cf22f5a9f2ff",
                "finite": true,
                "contacts": 92,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 78
              },
              "max_penetration": 0.18712,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          }
        },
        "benches": [
          {
            "name": "s3w/dart@1:7401797d Ir",
            "value": 93033935.0,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/dart@1:7401797d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:7401797d Ir",
            "value": 150658901.2,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:7401797d allocations",
            "value": 15750.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:7401797d Ir",
            "value": 1607.0,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:7401797d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/ode@1:7401797d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "s1p/dart@1:f381f8a4 Ir",
            "value": 11834947.2,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "f381f8a4e35376fce479416bb6b29a996075a0fcb3366143d8f3639dc9f56d60",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/dart@1:f381f8a4 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "f381f8a4e35376fce479416bb6b29a996075a0fcb3366143d8f3639dc9f56d60",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:f381f8a4 Ir",
            "value": 15634301.28,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "f381f8a4e35376fce479416bb6b29a996075a0fcb3366143d8f3639dc9f56d60",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:f381f8a4 allocations",
            "value": 335.28,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "f381f8a4e35376fce479416bb6b29a996075a0fcb3366143d8f3639dc9f56d60",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:0aa85496 Ir",
            "value": 5079137.94,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:0aa85496 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:0aa85496 Ir",
            "value": 4128841.03,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:0aa85496 allocations",
            "value": 594.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:0aa85496 Ir",
            "value": 5327739.29,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:0aa85496 allocations",
            "value": 0.25,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:0aa85496 Ir",
            "value": 4861929.1,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:0aa85496 allocations",
            "value": 394.43,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "0aa8549695f32d8d17f85e6edf9bc6cef8c2c7f544e4e2688980fcfbc03389f4",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:de429022 Ir",
            "value": 361642.15,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "de429022c113432100a3e265606855cdbebfec96fe60c9f70cf4177adeb0845f",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:de429022 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "de429022c113432100a3e265606855cdbebfec96fe60c9f70cf4177adeb0845f",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "gzb/ode@1:9c5a6449 Ir",
            "value": 174072683.0,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "9c5a64498b78952b8d532441489d434491c3397e127bbfc455e60e6e5480eee5",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "gzb/ode@1:9c5a6449 allocations",
            "value": 16707.666666666668,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "9c5a64498b78952b8d532441489d434491c3397e127bbfc455e60e6e5480eee5",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:343fc4cb Ir",
            "value": 12631413.49,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "343fc4cb6879164ccfe46f5d0b0a41657fd0cc303217912551c7dbaed41c4afc",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:343fc4cb allocations",
            "value": 16.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "343fc4cb6879164ccfe46f5d0b0a41657fd0cc303217912551c7dbaed41c4afc",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "dyn@1:64f3174f Ir",
            "value": 12320997.8,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": true,
            "input_sha": "64f3174f2ca3dad684382054ac8dfb74c93466b50dda3be3f93b673a11a9f3ff",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "dyn@1:64f3174f allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": true,
            "input_sha": "64f3174f2ca3dad684382054ac8dfb74c93466b50dda3be3f93b673a11a9f3ff",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "lcp@1:449e0b2f Ir",
            "value": 1566397.0,
            "unit": "instructions / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": true,
            "input_sha": "449e0b2f49ee644d60c4dbaece4bffb0bd9761e503575c6111a06bc6e996bd4b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "lcp@1:449e0b2f allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": true,
            "input_sha": "449e0b2f49ee644d60c4dbaece4bffb0bd9761e503575c6111a06bc6e996bd4b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "mt4-s3w/dart@1:7401797d allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "7401797d73822d7c6f3223b638eddf96ed69361334284bda432cadcf1086ea1f",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "mt4-s1p/dart@1:d1003a9e allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959\nPR #3697",
            "fingerprint": "c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959",
            "micro_instrumented": null,
            "input_sha": "d1003a9e7a6abe0238b22ae747d87fc110da8ae288117b3937f3edfd64f65f5d",
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
          "id": "427188fe0b891fa143edad9f8a45332c771981e4",
          "message": "v6.20.0-38-g427188fe0",
          "timestamp": "2026-10-11T23:07:02.146776+00:00",
          "committer": {
            "username": "github-actions[bot]"
          },
          "url": "https://github.com/dartsim/dart/commit/427188fe0b891fa143edad9f8a45332c771981e4"
        },
        "date": 1791760022146,
        "tool": "customSmallerIsBetter",
        "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
        "head_fingerprints": {},
        "measurement": {
          "s3w/dart": {
            "version": 1,
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
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
              "ir_per_step": 93033935.0,
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
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
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
              "ir_per_step": 150658901.2,
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
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
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
              "ir_per_step": 1607.0,
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
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
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
            "input_sha": "ea7cc6c5859ed60bcfc8cea0feb6ddbebff9ff79394e4a7748bf5bd1a12a8355",
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
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 11834947.2,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0x3f41cf22f5a9f2ff",
                "finite": true,
                "contacts": 92,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 78
              },
              "max_penetration": 0.18712,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "s1p/ode": {
            "version": 1,
            "input_sha": "ea7cc6c5859ed60bcfc8cea0feb6ddbebff9ff79394e4a7748bf5bd1a12a8355",
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
              "ir_per_step": 15634301.28,
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
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
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
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xfa5f6b7a91e0df5e",
                  "finite": true,
                  "contacts": 270,
                  "cap_hit": false,
                  "resting": "0/90",
                  "pairs": 90
                },
                "max_penetration": 8.48238e-09,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 5079137.94,
              "allocs_per_step": 0.0,
              "bytes_per_step": 0.0,
              "guards": {
                "hash": "0xfa5f6b7a91e0df5e",
                "finite": true,
                "contacts": 270,
                "cap_hit": false,
                "resting": "0/90",
                "pairs": 90
              },
              "max_penetration": 8.48238e-09,
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
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
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
              "ir_per_step": 4128841.03,
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
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
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
              "ir_per_step": 5327739.29,
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
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
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
              "ir_per_step": 4861929.1,
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
            "input_sha": "8aeb4ab8f238f513905d605bcb7358d5d8b18ece1d7ac164d79b2655bead28ba",
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
              "ir_per_step": 361642.15,
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
            "input_sha": "9c5a64498b78952b8d532441489d434491c3397e127bbfc455e60e6e5480eee5",
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
              "ir_per_step": 174072683.0,
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
            "input_sha": "343fc4cb6879164ccfe46f5d0b0a41657fd0cc303217912551c7dbaed41c4afc",
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
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "random2": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0xf3016c0eb10fdb15",
                  "finite": true,
                  "contacts": 8,
                  "cap_hit": false,
                  "resting": "0/2"
                },
                "max_penetration": null,
                "checkpoints": null,
                "allocs": 1600,
                "bytes": 22310400,
                "time_advanced": true
              }
            },
            "head": {
              "ir_per_step": 12631413.49,
              "allocs_per_step": 16.0,
              "bytes_per_step": 223104.0,
              "guards": {
                "hash": "0xf3016c0eb10fdb15",
                "finite": true,
                "contacts": 8,
                "cap_hit": false,
                "resting": "0/2"
              },
              "max_penetration": null,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 1600,
              "bytes": 22310400,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          },
          "dyn": {
            "version": 1,
            "input_sha": "64f3174f2ca3dad684382054ac8dfb74c93466b50dda3be3f93b673a11a9f3ff",
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
              "ir_per_step": 12320997.8,
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
            "input_sha": "449e0b2f49ee644d60c4dbaece4bffb0bd9761e503575c6111a06bc6e996bd4b",
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
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
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
            "input_sha": "6bef24e6d8b608e6f5956900833e1aa7a5e78e77193fd7ca0cab0fb5fbc1df51",
            "threads": 4,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "native",
            "collection_signature": null,
            "status": "ok",
            "gated": false,
            "qualification_required": true,
            "perturbations": {
              "start4k": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "start100k": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size16": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "size48": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random1": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 0,
                "bytes": 0,
                "time_advanced": true
              },
              "random2": {
                "stable": false,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
                "checkpoints": null,
                "allocs": 1,
                "bytes": 7168,
                "time_advanced": true
              },
              "tcache0": {
                "stable": true,
                "guards": {
                  "hash": "0x3f41cf22f5a9f2ff",
                  "finite": true,
                  "contacts": 92,
                  "cap_hit": false,
                  "resting": "0/60",
                  "pairs": 78
                },
                "max_penetration": 0.18712,
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
                "hash": "0x3f41cf22f5a9f2ff",
                "finite": true,
                "contacts": 92,
                "cap_hit": false,
                "resting": "0/60",
                "pairs": 78
              },
              "max_penetration": 0.18712,
              "checkpoints": null,
              "time_advanced": true,
              "allocs": 0,
              "bytes": 0,
              "cases": null,
              "micro_instrumented": null,
              "est_cycles_per_step": null
            }
          }
        },
        "benches": [
          {
            "name": "s3w/dart@1:c5a17253 Ir",
            "value": 93033935.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/dart@1:c5a17253 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:c5a17253 Ir",
            "value": 150658901.2,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s3w/ode@1:c5a17253 allocations",
            "value": 15750.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
            "threads": 1,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:c5a17253 Ir",
            "value": 1607.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/dart@1:c5a17253 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s2r/ode@1:c5a17253 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
            "threads": 1,
            "window": {
              "warmup": 1000,
              "steps": 200
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "s1p/dart@1:ea7cc6c5 Ir",
            "value": 11834947.2,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "ea7cc6c5859ed60bcfc8cea0feb6ddbebff9ff79394e4a7748bf5bd1a12a8355",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/dart@1:ea7cc6c5 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "ea7cc6c5859ed60bcfc8cea0feb6ddbebff9ff79394e4a7748bf5bd1a12a8355",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:ea7cc6c5 Ir",
            "value": 15634301.28,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "ea7cc6c5859ed60bcfc8cea0feb6ddbebff9ff79394e4a7748bf5bd1a12a8355",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s1p/ode@1:ea7cc6c5 allocations",
            "value": 335.28,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "ea7cc6c5859ed60bcfc8cea0feb6ddbebff9ff79394e4a7748bf5bd1a12a8355",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 50
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:63ce0de2 Ir",
            "value": 5079137.94,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/dart@1:63ce0de2 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:63ce0de2 Ir",
            "value": 4128841.03,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/fcl@1:63ce0de2 allocations",
            "value": 594.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:63ce0de2 Ir",
            "value": 5327739.29,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/bullet@1:63ce0de2 allocations",
            "value": 0.25,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:63ce0de2 Ir",
            "value": 4861929.1,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "s5a/ode@1:63ce0de2 allocations",
            "value": 394.43,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "63ce0de21c4bcda8a174cb6e48d1a220fd70b4672a0d175c20b57b53e2e7318b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:8aeb4ab8 Ir",
            "value": 361642.15,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "8aeb4ab8f238f513905d605bcb7358d5d8b18ece1d7ac164d79b2655bead28ba",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "pend/dart@1:8aeb4ab8 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "8aeb4ab8f238f513905d605bcb7358d5d8b18ece1d7ac164d79b2655bead28ba",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 1000
            },
            "method": "slope",
            "collection_signature": "dart::simulation::World::step(bool)"
          },
          {
            "name": "gzb/ode@1:9c5a6449 Ir",
            "value": 174072683.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698\nfingerprint changed: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959 -> a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "9c5a64498b78952b8d532441489d434491c3397e127bbfc455e60e6e5480eee5",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "gzb/ode@1:9c5a6449 allocations",
            "value": 16707.666666666668,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698\nfingerprint changed: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959 -> a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "9c5a64498b78952b8d532441489d434491c3397e127bbfc455e60e6e5480eee5",
            "threads": 1,
            "window": {
              "warmup": 2,
              "steps": 3
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:343fc4cb Ir",
            "value": 12631413.49,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698\nfingerprint changed: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959 -> a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "343fc4cb6879164ccfe46f5d0b0a41657fd0cc303217912551c7dbaed41c4afc",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "robot/dart@1:343fc4cb allocations",
            "value": 16.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698\nfingerprint changed: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959 -> a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "343fc4cb6879164ccfe46f5d0b0a41657fd0cc303217912551c7dbaed41c4afc",
            "threads": 1,
            "window": {
              "warmup": 300,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "stepAndRead(dart::simulation::World*)"
          },
          {
            "name": "dyn@1:64f3174f Ir",
            "value": 12320997.8,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698\nfingerprint changed: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959 -> a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": true,
            "input_sha": "64f3174f2ca3dad684382054ac8dfb74c93466b50dda3be3f93b673a11a9f3ff",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "dyn@1:64f3174f allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698\nfingerprint changed: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959 -> a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": true,
            "input_sha": "64f3174f2ca3dad684382054ac8dfb74c93466b50dda3be3f93b673a11a9f3ff",
            "threads": 1,
            "window": {
              "warmup": 20,
              "steps": 20
            },
            "method": "slope",
            "collection_signature": "BM_Dynamics(benchmark::State&)"
          },
          {
            "name": "lcp@1:449e0b2f Ir",
            "value": 1566397.0,
            "unit": "instructions / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698\nfingerprint changed: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959 -> a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": true,
            "input_sha": "449e0b2f49ee644d60c4dbaece4bffb0bd9761e503575c6111a06bc6e996bd4b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "lcp@1:449e0b2f allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698\nfingerprint changed: c6ae090a9ffc3c2650345911dfc43478cda7ddc56afd958b7cd91c18f8466959 -> a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": true,
            "input_sha": "449e0b2f49ee644d60c4dbaece4bffb0bd9761e503575c6111a06bc6e996bd4b",
            "threads": 1,
            "window": {
              "warmup": 100,
              "steps": 100
            },
            "method": "slope",
            "collection_signature": "(anonymous namespace)::solveNative(benchmark::State&, int)"
          },
          {
            "name": "mt4-s3w/dart@1:c5a17253 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "c5a17253d0daa8ff952e243de752073029a90bb166bfa5b4a7266d4739507adc",
            "threads": 4,
            "window": {
              "warmup": 5,
              "steps": 5
            },
            "method": "native",
            "collection_signature": null
          },
          {
            "name": "mt4-s1p/dart@1:6bef24e6 allocations",
            "value": 0.0,
            "unit": "allocations / step",
            "extra": "fingerprint: a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c\nPR #3698",
            "fingerprint": "a09ee7c264409d5cbf4f1f8c45f5e1fe62a36eb6c74f75d528b540131611407c",
            "micro_instrumented": null,
            "input_sha": "6bef24e6d8b608e6f5956900833e1aa7a5e78e77193fd7ca0cab0fb5fbc1df51",
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
