# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

# Deprecated compatibility shim for older installed-consumer configurations.
# DART no longer uses spdlog.
# Satisfy dependency checks in older component configurations without discovery.
message(DEPRECATION "DARTFindspdlog is deprecated; DART no longer uses spdlog.")
set(spdlog_FOUND TRUE)
