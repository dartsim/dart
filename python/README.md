# dartpy

> :warning: **Warning:** `dartpy` is under heavy development. See the open
> issues on [`dartpy`](https://github.com/dartsim/dart/issues) and
> [Chimera](https://github.com/personalrobotics/chimera/issues) for insight
> into the current state of the project. Please report any issues you
> encounter on the appropriate repository. This repo will be mreged into the
> main repository of DART once becomes stable.

Python bindings for [DART][dart], the Dynamic Animation and Robotics Toolkit.

## Installation

### On Ubuntu 14.04 / 16.04 / 18.04

You can install `dartpy` using `apt-get` as:

**14.04**

```shell
$ sudo add-apt-repository ppa:libccd-debs/ppa
$ sudo add-apt-repository ppa:fcl-debs/ppa
$ sudo add-apt-repository ppa:dartsim/ppa
$ sudo add-apt-repository ppa:personalrobotics/ppa
$ sudo apt-get update

$ sudo apt-get install python-dartpy  # for Python 2
$ sudo apt-get install python3-dartpy # for Python 3
```

**16.04 / 18.04**

```shell
$ sudo add-apt-repository ppa:dartsim/ppa
$ sudo add-apt-repository ppa:personalrobotics/ppa
$ sudo apt-get update

$ sudo apt-get install python-dartpy  # for Python 2
$ sudo apt-get install python3-dartpy # for Python 3
```

All set! Import `dartpy` in Python and enjoy! Please see [Usage](#usage) section for more information.

If you want to build `dartpy` from source, please see this [wiki page](https://github.com/personalrobotics/dartpy/wiki/Building-from-Source).

### On macOS

```shell
$ brew install personalrobotics/tap/dartpy
```

## Usage

Once `dartpy` is installed, you should be able to open a Python terminal and
`import dartpy`. Since `dartpy` is mostly auto-generated using Chimera, the
DART Python API mostly matches the DART C++ API.

There are a few exceptions:

### Template Functions

Template functions take the template parameters as regular arguments, e.g. the
C++ code:
```c++
auto joint = bodynode.moveTo<dart::dynamics::FreeJoint>(newParent);
```
becomes the Python code
```python
joint = bodynode.moveTo(dartpy.dynamics.FreeJoint, newParent)
```

Due to limitations of C++, this functionality requires the template arguments
to be registered with `dartpy`. Follow [the instructions](https://github.com/personalrobotics/dartpy/wiki/Bindings-for-Extension-Libraries#template-member-functions)
to register your custom types for use as template arguments.

## License

`dartpy` is licensed under [the BSD-2-Clause license](https://opensource.org/licenses/BSD-2-Clause). See [LICENSE](https://github.com/personalrobotics/dartpy/blob/master/LICENSE) for more information.

## Authors

`dartpy` is developed by the [Personal Robotics Lab][prl] in the [Robotics
Institute][ri] at [Carnegie Mellon University][cmu] by [Michael Koval][mkoval]
([**@mkoval**][mkoval_github]) and [Pras Velagapudi][psigen]
([**@psigen**][psigen_github]).


[chimera]: https://github.com/personalrobotics/chimera
[cmake]: https://cmake.org/
[cmu]: http://www.cmu.edu
[dart]: http://dartsim.github.io/
[mkoval]: http://mkoval.org
[mkoval_github]: https://github.com/mkoval
[prl]: https://personalrobotics.ri.cmu.edu
[prl_dev]: https://personalrobotics.cs.washington.edu/software/development-environment/
[psigen]: http://www.snowbotic.com/
[psigen_github]: http://github.com/psigen
[ri]: https://www.ri.cmu.edu
[ubuntu1404]: http://releases.ubuntu.com/14.04/
