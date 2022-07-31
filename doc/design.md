# DART Design

## Requirements

* Support Ubuntu, macOS, and Windows
  * Ubuntu >= 22.04 LTS
  * macOS >= 12
  * Windows 64-bit with Visual Studio >= 2022
* Provide Python binding
* Minimum required dependencies (i.e., Eigen)

## Design Philosophy

* Prefer vectorization (using SIMD) when applicable
* Prefer parallelization when applicable

## Features

## Style Guide

### C/C++

* Use CamelCase for type names
* Use snake_case for function and variable names
