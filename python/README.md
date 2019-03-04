# dartpy

> :warning: **Warning:** `dartpy` is under heavy development. See the open
> issues on [`dartpy`](https://github.com/dartsim/dart/issues) and
> [Chimera](https://github.com/personalrobotics/chimera/issues) for insight
> into the current state of the project. Please report any issues you
> encounter on the appropriate repository. This repo will be mreged into the
> main repository of DART once becomes stable.

Python bindings for [DART][dart], the Dynamic Animation and Robotics Toolkit.

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

### Template Member Functions

Due to limitations of C++, this functionality requires the template arguments
to be registered with `dartpy`.
DART uses template member functions to construct `Addon`, `BodyNode`, and
`Joint` instances. `dartpy` works around this limitation by wrapping these
functions for a *predefined set* of template arguments. You need to *register*
your classes with `dartpy` for these methods to work on custom types.

For custom `Joint`s:
```c++
JointTemplateRegistry::register_type<MyJoint1>();
JointTemplateRegistry::register_type<MyJoint2>();
// ...
```

For custom `BodyNode`s:
```c++
BodyNodeTemplateRegistry::register_type<MyBodyNode1>();
BodyNodeTemplateRegistry::register_type<MyBodyNode2>();
BodyNodeTemplateRegistry::register_type<MyBodyNode3>();
// ...
```

If you want to use the `createJointAndBodyNodePair()` method on `Skeleton`,
then you also need to register *all pairs* of `BodyNode` and `Joint` subclasses
that you intend to pass as template arguments. Typically, this includes:

1. each custom `BodyNode` paired with each custom `Joint`
2. each custom `BodyNode` paired with each default `Joint`
3. each default `BodyNode` paired with each custom` Joint`

`dartpy` provides the `register_all_types` helper function to register the
cartesian product of two lists of types. You can register all of the above
combinations using the three lines of code:
```c++
using MyJointTypes = typelist<MyJoint1, MyJoint2 /* ... */>;
using MyBodyNodeTypes = typelist<MyBodyNode1, MyBodyNode2, MyBodyNode3 /* ... */>;

JointAndNodeTemplateRegistry::register_all_types<MyJointTypes, AllNodeTypes>();
JointAndNodeTemplateRegistry::register_all_types<AllJointTypes, MyBodyNodeTypes>();
JointAndNodeTemplateRegistry::register_all_types<MyJointTypes, MyBodyNodeTypes>();
```

Note that this approach means that is is not generally possible to call
`createJointAndBodyNodePair` with `BodyNode` and `Joint` types defined in two
different extension libraries. If this is necessary, you need to modify one of
the libraries to call `register_type` on that pair of `BodyNode` and `Joint`
types.

## License

`dartpy` is licensed under [the BSD-2-Clause license](https://opensource.org/licenses/BSD-2-Clause). See [LICENSE](https://github.com/dartsim/dart/blob/master/LICENSE) for more information.

## Authors

`dartpy` is initiated by the [Personal Robotics Lab][prl] in the [Robotics
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
