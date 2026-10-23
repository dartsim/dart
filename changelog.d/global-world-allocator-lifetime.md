Section: Simulation

  * Keep the default allocator alive until process exit so globally held Worlds
    and memory managers can release their storage safely during shutdown.
