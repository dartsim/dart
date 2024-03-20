#!/bin/bash
# entrypoint.sh

# Use the host's display server for the GUI
export DISPLAY=host.docker.internal:0

# Start Tracy Profiler
./tracy/profiler/build/unix/Tracy-release
