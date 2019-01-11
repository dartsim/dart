## DART 7

### DART 7.0.0 (201X-XX-XX)

* Collision detection

  * Added FCL 0.6 support: [#873](https://github.com/dartsim/dart/pull/873)

* Optimizer

  * Changed Function::eval\[Gradient/Hessian\](...) to const function: [#928](https://github.com/dartsim/dart/pull/928)

* IO

  * Renamed namespace `utils` to `io`: [#887](https://github.com/dartsim/dart/pull/887), [#953](https://github.com/dartsim/dart/pull/953)

## DART 6

### [DART 6.8.0 (20XX-XX-XX)](https://github.com/dartsim/dart/milestone/48?closed=1)

#### Changes

* Planning

  * Fixed linking error of FLANN by explicitly linking to lz4: [#1221](https://github.com/dartsim/dart/pull/1221)

#### Compilers Tested

  * Linux

    * GCC (C++11): 5.4.0, 7.3.0, 8.2.0

  * Linux (32-bit)

    * GCC (C++11): 5.4.0

  * macOS

    * AppleClang (C++11): 9.1.0

### [DART 6.7.0 (2019-01-10)](https://github.com/dartsim/dart/milestone/45?closed=1)

#### Changes

* Build system

  * Fixed compilation warnings for newer versions of compilers: [#1177](https://github.com/dartsim/dart/pull/1177)
  * Changed to generate namespace headers without requiring *.hpp.in files: [#1192](https://github.com/dartsim/dart/pull/1192)
  * Dropped supporting Ubuntu Trusty and started using imported targets of dependencies: [#1212](https://github.com/dartsim/dart/pull/1212)

* Collision Detection

  * CollisionGroups will automatically update their objects when any changes occur to Skeletons or BodyNodes that they are subscribed to: [#1112](https://github.com/dartsim/dart/pull/1112)
  * Contact points with negative penetration depth will be ignored: [#1185](https://github.com/dartsim/dart/pull/1185)

* Math

  * Consolidated random functions into Random class: [#1109](https://github.com/dartsim/dart/pull/1109)

* Dynamics

  * Refactor constraint solver: [#1099](https://github.com/dartsim/dart/pull/1099), [#1101](https://github.com/dartsim/dart/pull/1101)
  * Added mimic joint functionality as a new actuator type: [#1178](https://github.com/dartsim/dart/pull/1178)
  * Added clone function to MetaSkeleton: [#1201](https://github.com/dartsim/dart/pull/1201)

* Optimization

  * Added multi-objective optimization with pagmo2 support: [#1106](https://github.com/dartsim/dart/pull/1106)

* GUI

  * Reorganized OpenGL and GLUT files: [#1088](https://github.com/dartsim/dart/pull/1088)
  * Added the RealTimeWorldNode to display simulations at real-time rates: [#1216](https://github.com/dartsim/dart/pull/1216)

* Misc

  * Updated Googletest to version 1.8.1: [#1214](https://github.com/dartsim/dart/pull/1214)

#### Compilers Tested

  * Linux

    * GCC (C++11): 5.4.0, 7.3.0, 8.2.0

  * Linux (32-bit)

    * GCC (C++11): 5.4.0

  * macOS

    * AppleClang (C++11): 9.1.0

### [DART 6.6.2 (2018-09-03)](https://github.com/dartsim/dart/milestone/47?closed=1)

* Utils

  * Fixed checking file existence in DartResourceRetriever: [#1107](https://github.com/dartsim/dart/pull/1107)

### [DART 6.6.1 (2018-08-04)](https://github.com/dartsim/dart/milestone/46?closed=1)

* Utils

  * Added option to DartResourceRetriever to search from environment variable DART_DATA_PATH: [#1095](https://github.com/dartsim/dart/pull/1095)

* Examples

  * Fixed CMakeLists.txt of humanJointLimits: [#1094](https://github.com/dartsim/dart/pull/1094)

### [DART 6.6.0 (2018-08-02)](https://github.com/dartsim/dart/milestone/44?closed=1)

* Collision detection

  * Added voxel grid map: [#1076](https://github.com/dartsim/dart/pull/1076), [#1083](https://github.com/dartsim/dart/pull/1083)
  * Added heightmap support: [#1069](https://github.com/dartsim/dart/pull/1069)

### [DART 6.5.0 (2018-05-12)](https://github.com/dartsim/dart/milestone/41?closed=1)

* Common

  * Added LockableReference classes: [#1011](https://github.com/dartsim/dart/pull/1011)
  * Added missing \<vector\> to Memory.hpp: [#1057](https://github.com/dartsim/dart/pull/1057)

* GUI

  * Added FOV API to OSG viewer: [#1048](https://github.com/dartsim/dart/pull/1048)

* Parsers

  * Fixed incorrect parsing of continuous joints specified in URDF [#1064](https://github.com/dartsim/dart/pull/1064)

* Simulation

  * Added World::hasSkeleton(): [#1050](https://github.com/dartsim/dart/pull/1050)

* Misc

  * Fixed memory leaks in mesh loading: [#1066](https://github.com/dartsim/dart/pull/1066)

### [DART 6.4.0 (2018-03-26)](https://github.com/dartsim/dart/milestone/39?closed=1)

* Common

  * Added DART_COMMON_DECLARE_SMART_POINTERS macro: [#1022](https://github.com/dartsim/dart/pull/1022)
  * Added ResourceRetriever::getFilePath(): [#972](https://github.com/dartsim/dart/pull/972)

* Kinematics/Dynamics

  * Added relative Jacobian functions to MetaSkeleton: [#997](https://github.com/dartsim/dart/pull/997)
  * Added vectorized joint limit functions: [#996](https://github.com/dartsim/dart/pull/996)
  * Added lazy evaluation for shape's volume and bounding-box computation: [#959](https://github.com/dartsim/dart/pull/959)
  * Added IkFast support as analytic IK solver: [#887](https://github.com/dartsim/dart/pull/887)
  * Added TranslationalJoint2D: [#1003](https://github.com/dartsim/dart/pull/1003)
  * Fixed NaN values caused by zero-length normals in ContactConstraint: [#881](https://github.com/dartsim/dart/pull/881)
  * Extended BodyNode::createShapeNode() to accept more types of arguments: [#986](https://github.com/dartsim/dart/pull/986)

* Collision detection

  * Added FCL 0.6 support (backport of #873): [#936](https://github.com/dartsim/dart/pull/936)

* GUI

  * Added support of rendering texture images: [#973](https://github.com/dartsim/dart/pull/973)
  * Added OSG shadows: [#978](https://github.com/dartsim/dart/pull/978)

* Examples

  * Added humanJointLimits: [#1016](https://github.com/dartsim/dart/pull/1016)

* License

  * Added Personal Robotics Lab and Open Source Robotics Foundation as contributors: [#929](https://github.com/dartsim/dart/pull/929)

* Misc

  * Added World::create(): [#962](https://github.com/dartsim/dart/pull/962)
  * Added MetaSkeleton::hasBodyNode() and MetaSkeleton::hasJoint(): [#1000](https://github.com/dartsim/dart/pull/1000)
  * Suppressed -Winjected-class-name warnings from Clang 5.0.0: [#964](https://github.com/dartsim/dart/pull/964)
  * Suppressed -Wdangling-else warnings from GCC 7.2.0: [#937](https://github.com/dartsim/dart/pull/937)
  * Changed console macros to use global namespace resolutions: [#1010](https://github.com/dartsim/dart/pull/1010)
  * Fixed build with Eigen 3.2.1-3.2.8: [#1042](https://github.com/dartsim/dart/pull/1042)
  * Fixed various build issues with Visual Studio: [#956](https://github.com/dartsim/dart/pull/956)
  * Removed TinyXML dependency: [#993](https://github.com/dartsim/dart/pull/993)

### [DART 6.3.1 (2018-03-21)](https://github.com/dartsim/dart/milestone/42?closed=1)

* Build system

  * Removed an undefined cmake macro/function: [#1036](https://github.com/dartsim/dart/pull/1036)

* ROS support

  * Tweaked package.xml for catkin support: [#1027](https://github.com/dartsim/dart/pull/1027), [#1029](https://github.com/dartsim/dart/pull/1029), [#1031](https://github.com/dartsim/dart/pull/1031), [#1032](https://github.com/dartsim/dart/pull/1031), [#1033](https://github.com/dartsim/dart/pull/1033)

### [DART 6.3.0 (2017-10-04)](https://github.com/dartsim/dart/milestone/36?closed=1)

* Collision detection

  * Added a feature of disabling body node pairs to BodyNodeCollisionFilter: [#911](https://github.com/dartsim/dart/pull/911)

* Kinematics/Dynamics

  * Added setter and getter for WeldJointConstraint::mRelativeTransform: [#910](https://github.com/dartsim/dart/pull/910)

* Parsers

  * Improved SkelParser to read alpha value: [#914](https://github.com/dartsim/dart/pull/914)

* Misc

  * Changed not to use lambda function as an workaround for DART python binding: [#916](https://github.com/dartsim/dart/pull/916)

### [DART 6.2.1 (2017-08-08)](https://github.com/dartsim/dart/milestone/37?closed=1)

* Collision detection

  * Fixed collision checking between objects from the same body node: [#894](https://github.com/dartsim/dart/pull/894)

* Kinematics/Dynamics

  * Fixed transform of ScrewJoint with thread pitch: [#855](https://github.com/dartsim/dart/pull/855)

* Parsers

  * Fixed incorrect reading of <use_parent_model_frame> from SDF: [#893](https://github.com/dartsim/dart/pull/893)
  * Fixed missing reading of joint friction from URDF: [#891](https://github.com/dartsim/dart/pull/891)

* Testing

  * Fixed testing ODE collision detector on macOS: [#884](https://github.com/dartsim/dart/pull/884)
  * Removed redundant main body for each test source file: [#856](https://github.com/dartsim/dart/pull/856)

* Misc

  * Fixed build of dart-gui-osg that depends on the presence of OSG: [#898](https://github.com/dartsim/dart/pull/898)
  * Fixed build of examples and tutorials on macOS: [#889](https://github.com/dartsim/dart/pull/889)
  * Fixed missing overriding method OdePlane::isPlaceable(): [#886](https://github.com/dartsim/dart/pull/886)
  * Replaced use of enum by static constexpr: [#852](https://github.com/dartsim/dart/pull/852), [#904](https://github.com/dartsim/dart/pull/904)

### [DART 6.2.0 (2017-05-15)](https://github.com/dartsim/dart/milestone/30?closed=1)

* Common

  * Added Factory class and applied it to collision detection creation: [#864](https://github.com/dartsim/dart/pull/864)
  * Added readAll() to Resource and ResourceRetriever: [#875](https://github.com/dartsim/dart/pull/875)

* Math

  * Added accessors for diameters and radii of EllipsoidShape, and deprecated EllipsoidShape::get/setSize(): [#829](https://github.com/dartsim/dart/pull/829)
  * Fixed Lemke LCP solver (#808 for DART 6): [#812](https://github.com/dartsim/dart/pull/812)

* Collision Detection

  * Added support of ODE collision detector: [#861](https://github.com/dartsim/dart/pull/861)
  * Fixed incorrect collision filtering of BulletCollisionDetector: [#859](https://github.com/dartsim/dart/pull/859)

* Simulation

  * Fixed World didn't clear collision results on reset: [#863](https://github.com/dartsim/dart/pull/863)

* Parsers

  * Fixed incorrect creation of resource retriever in SkelParser and SdfParser: [#847](https://github.com/dartsim/dart/pull/847), [#849](https://github.com/dartsim/dart/pull/849)

* GUI

  * Added MotionBlurSimWindow: [#840](https://github.com/dartsim/dart/pull/840)
  * Improved MultiSphereShape rendering in GLUT renderer: [#862](https://github.com/dartsim/dart/pull/862)
  * Fixed incorrect parsing of materials and normal scaling from URDF: [#851](https://github.com/dartsim/dart/pull/851)
  * Fixed the OSG renderer not rendering collision geometries: [#851](https://github.com/dartsim/dart/pull/851)
  * Fixed that GUI was rendering white lines with nvidia drivers: [#805](https://github.com/dartsim/dart/pull/805)

* Misc

  * Added createShared() and createUnique() pattern: [#844](https://github.com/dartsim/dart/pull/844)
  * Added Skeleton::getRootJoint(): [#832](https://github.com/dartsim/dart/pull/832)
  * Added CMake targets for code formatting using clang-format: [#811](https://github.com/dartsim/dart/pull/811), [#817](https://github.com/dartsim/dart/pull/817)
  * Renamed MultiSphereShape to MultiSphereConvexHullShape: [#865](https://github.com/dartsim/dart/pull/865)
  * Modified the member function names pertain to lazy evaluation to be more relevant to their functionalities: [#833](https://github.com/dartsim/dart/pull/833)

* Tutorials & Examples

  * Allowed tutorials and examples to be built out of DART source tree: [#842](https://github.com/dartsim/dart/pull/842)
  * Fixed tutorialDominoes-Finished that didn't work with the latest DART: [#807](https://github.com/dartsim/dart/pull/807)

### DART 6.1.2 (2017-01-13)

* Dynamics

  * Fixed bug of ContactConstraint with kinematic joints: [#809](https://github.com/dartsim/dart/pull/809)

* Misc

  * Fixed that ZeroDofJoint::getIndexInTree was called: [#818](https://github.com/dartsim/dart/pull/818)

### DART 6.1.1 (2016-10-14)

* Build

  * Modified to build DART without SIMD options by default: [#790](https://github.com/dartsim/dart/pull/790)
  * Modified to build external libraries as separately build targets: [#787](https://github.com/dartsim/dart/pull/787)
  * Modified to export CMake target files separately per target: [#786](https://github.com/dartsim/dart/pull/786)

* Misc

  * Updated lodepng up to version 20160501: [#791](https://github.com/dartsim/dart/pull/791)

### DART 6.1.0 (2016-10-07)

* Collision detection

  * Added distance API: [#744](https://github.com/dartsim/dart/pull/744)
  * Fixed direction of contact normal of BulletCollisionDetector: [#763](https://github.com/dartsim/dart/pull/763)

* Dynamics

  * Added `computeLagrangian()` to `MetaSkeleton` and `BodyNode`: [#746](https://github.com/dartsim/dart/pull/746)
  * Added new shapes: sphere, capsule, cone, and multi-sphere: [#770](https://github.com/dartsim/dart/pull/770), [#769](https://github.com/dartsim/dart/pull/769), [#745](https://github.com/dartsim/dart/pull/745)
  * Changed base class of joint from SingleDofJoint/MultiDofJoint to GenericJoint: [#747](https://github.com/dartsim/dart/pull/747)

* Planning

  * Fixed incorrect linking to flann library: [#761](https://github.com/dartsim/dart/pull/761)

* Parsers

  * Added `sdf` parsing for `fixed` joint and `material` tag of visual shape: [#775](https://github.com/dartsim/dart/pull/775)
  * Added support of urdfdom_headers 1.0: [#766](https://github.com/dartsim/dart/pull/766)

* GUI

  * Added ImGui for 2D graphical interface: [#781](https://github.com/dartsim/dart/pull/781)

* Examples

  * Added osgAtlasSimbicon and osgTinkertoy: [#781](https://github.com/dartsim/dart/pull/781)

* Misc improvements and bug fixes

  * Added `virtual Shape::getType()` and deprecated `ShapeType Shape::getShapeType()`: [#725](https://github.com/dartsim/dart/pull/725)
  * Changed building with SIMD optional: [#765](https://github.com/dartsim/dart/pull/765), [#760](https://github.com/dartsim/dart/pull/760)
  * Fixed minor build and install issues: [#773](https://github.com/dartsim/dart/pull/773), [#772](https://github.com/dartsim/dart/pull/772)
  * Fixed Doxyfile to show missing member functions in API documentation: [#768](https://github.com/dartsim/dart/pull/768)
  * Fixed typo: [#756](https://github.com/dartsim/dart/pull/756), [#755](https://github.com/dartsim/dart/pull/755)

### DART 6.0.1 (2016-06-29)

* Collision detection

  * Added support of FCL 0.5 and tinyxml2 4.0: [#749](https://github.com/dartsim/dart/pull/749)
  * Added warnings for unsupported shape pairs of DARTCollisionDetector: [#722](https://github.com/dartsim/dart/pull/722)

* Dynamics

  * Fixed total mass is not being updated when bodies removed from Skeleton: [#731](https://github.com/dartsim/dart/pull/731)

* Misc improvements and bug fixes

  * Renamed `DEPRECATED` and `FORCEINLINE` to `DART_DEPRECATED` and `DART_FORCEINLINE` to avoid name conflicts: [#742](https://github.com/dartsim/dart/pull/742)
  * Updated copyright: added CMU to copyright holder, moved individual contributors to CONTRIBUTING.md: [#723](https://github.com/dartsim/dart/pull/723)

### DART 6.0.0 (2016-05-10)

* Common data structures

  * Added `Node`, `Aspect`, `State`, and `Properties`: [#713](https://github.com/dartsim/dart/pull/713), [#712](https://github.com/dartsim/dart/issues/712), [#708](https://github.com/dartsim/dart/pull/708), [#707](https://github.com/dartsim/dart/pull/707), [#659](https://github.com/dartsim/dart/pull/659), [#649](https://github.com/dartsim/dart/pull/649), [#645](https://github.com/dartsim/dart/issues/645), [#607](https://github.com/dartsim/dart/pull/607), [#598](https://github.com/dartsim/dart/pull/598), [#591](https://github.com/dartsim/dart/pull/591), [#531](https://github.com/dartsim/dart/pull/531)
  * Added mathematical constants and user-defined literals for radian, degree, and pi: [#669](https://github.com/dartsim/dart/pull/669), [#314](https://github.com/dartsim/dart/issues/314)
  * Added `ShapeFrame` and `ShapeNode`: [#608](https://github.com/dartsim/dart/pull/608)
  * Added `BoundingBox`: [#547](https://github.com/dartsim/dart/pull/547), [#546](https://github.com/dartsim/dart/issues/546)

* Kinematics

  * Added convenient functions for setting joint limits: [#703](https://github.com/dartsim/dart/pull/703)
  * Added more description on `InverseKinematics::solve()`: [#624](https://github.com/dartsim/dart/pull/624)
  * Added API for utilizing analytical inverse kinematics: [#530](https://github.com/dartsim/dart/pull/530), [#463](https://github.com/dartsim/dart/issues/463)
  * Added color property to `Marker`: [#187](https://github.com/dartsim/dart/issues/187)
  * Improved `Skeleton` to clone `State` as well: [#691](https://github.com/dartsim/dart/pull/691)
  * Improved `ReferentialSkeleton` to be able to add and remove `BodyNode`s and `DegreeOfFreedom`s to/from `Group`s freely: [#557](https://github.com/dartsim/dart/pull/557), [#556](https://github.com/dartsim/dart/issues/556), [#548](https://github.com/dartsim/dart/issues/548)
  * Changed `Marker` into `Node`: [#692](https://github.com/dartsim/dart/pull/692), [#609](https://github.com/dartsim/dart/issues/609)
  * Renamed `Joint::get/setLocal[~]` to `Joint::get/setRelative[~]`: [#715](https://github.com/dartsim/dart/pull/715), [#714](https://github.com/dartsim/dart/issues/714)
  * Renamed `PositionLimited` to `PositionLimitEnforced`: [#447](https://github.com/dartsim/dart/issues/447)
  * Fixed initialization of joint position and velocity: [#691](https://github.com/dartsim/dart/pull/691), [#621](https://github.com/dartsim/dart/pull/621)
  * Fixed `InverseKinematics` when it's used with `FreeJoint` and `BallJoint`: [#683](https://github.com/dartsim/dart/pull/683)
  * Fixed ambiguous overload on `MetaSkeleton::getLinearJacobianDeriv`: [#628](https://github.com/dartsim/dart/pull/628), [#626](https://github.com/dartsim/dart/issues/626)

* Dynamics

  * Added `get/setLCPSolver` functions to `ConstraintSolver`: [#633](https://github.com/dartsim/dart/pull/633)
  * Added `ServoMotorConstraint` as a preliminary implementation for `SERVO` actuator type: [#566](https://github.com/dartsim/dart/pull/566)
  * Improved `ConstraintSolver` to obey C++11 ownership conventions: [#616](https://github.com/dartsim/dart/pull/616)
  * Fixed segfualting of `DantzigLCPSolver` when the constraint dimension is zero: [#634](https://github.com/dartsim/dart/pull/634)
  * Fixed missing implementations in ConstrainedGroup: [#586](https://github.com/dartsim/dart/pull/586)
  * Fixed incorrect applying of joint constraint impulses: [#317](https://github.com/dartsim/dart/issues/317)
  * Deprecated `draw()` functions of dynamics classes: [#654](https://github.com/dartsim/dart/pull/654)

* Collision detection

  * Added `CollisionGroup` and refactored `CollisionDetector` to be more versatile: [#711](https://github.com/dartsim/dart/pull/711), [#704](https://github.com/dartsim/dart/pull/704), [#689](https://github.com/dartsim/dart/pull/689), [#631](https://github.com/dartsim/dart/pull/631), [#642](https://github.com/dartsim/dart/issues/642), [#20](https://github.com/dartsim/dart/issues/20)
  * Improved API for self collision checking options: [#718](https://github.com/dartsim/dart/pull/718), [#702](https://github.com/dartsim/dart/issues/702)
  * Deprecated `BodyNode::isColliding`; collision sets are moved to `CollisionResult`: [#694](https://github.com/dartsim/dart/pull/694), [#670](https://github.com/dartsim/dart/pull/670), [#668](https://github.com/dartsim/dart/pull/668), [#666](https://github.com/dartsim/dart/issues/666)

* Parsers

  * Added back VSK parser: [#602](https://github.com/dartsim/dart/pull/602), [#561](https://github.com/dartsim/dart/pull/561), [#254](https://github.com/dartsim/dart/issues/254)
  * Fixed segfault of `SdfParser` when `nullptr` `ResourceRetriever` is passed: [#663](https://github.com/dartsim/dart/pull/663)

* GUI features

  * Merged `renderer` namespace into `gui` namespace: [#652](https://github.com/dartsim/dart/pull/652), [#589](https://github.com/dartsim/dart/issues/589)
  * Moved `osgDart` under `dart::gui` namespace as `dart::gui::osg`: [#651](https://github.com/dartsim/dart/pull/651)
  * Fixed GlutWindow::screenshot(): [#623](https://github.com/dartsim/dart/pull/623), [#395](https://github.com/dartsim/dart/issues/395)

* Simulation

  * Fixed `World::clone()` didn't clone the collision detector: [#658](https://github.com/dartsim/dart/pull/658)
  * Fixed bug of `World` concurrency: [#577](https://github.com/dartsim/dart/pull/577), [#576](https://github.com/dartsim/dart/issues/576)

* Misc improvements and bug fixes

  * Added `make_unique<T>` that was omitted from C++11: [#639](https://github.com/dartsim/dart/pull/639)
  * Added missing `override` keywords: [#617](https://github.com/dartsim/dart/pull/617), [#535](https://github.com/dartsim/dart/pull/535)
  * Added gcc warning flag `-Wextra`: [#600](https://github.com/dartsim/dart/pull/600)
  * Improved memory management of `constraint` namespace: [#584](https://github.com/dartsim/dart/pull/584), [#583](https://github.com/dartsim/dart/issues/583)
  * Changed the extension of headers from `.h` to `.hpp`: [#709](https://github.com/dartsim/dart/pull/709), [#693](https://github.com/dartsim/dart/pull/693), [#568](https://github.com/dartsim/dart/issues/568)
  * Changed Doxyfile to gnerate tag file: [#690](https://github.com/dartsim/dart/pull/690)
  * Changed the convention to use `std::size_t` over `size_t`: [#681](https://github.com/dartsim/dart/pull/681), [#656](https://github.com/dartsim/dart/issues/656)
  * Changed CMake to configure preprocessors using `#cmakedefine`: [#648](https://github.com/dartsim/dart/pull/648), [#641](https://github.com/dartsim/dart/pull/641)
  * Updated copyright years: [#679](https://github.com/dartsim/dart/pull/679), [#160](https://github.com/dartsim/dart/issues/160)
  * Renamed directory name `apps` to `examples`: [#685](https://github.com/dartsim/dart/pull/685)
  * Fixed warnings of unused variables in release mode: [#646](https://github.com/dartsim/dart/pull/646)
  * Fixed typo of `getNumPluralAddoName` in utility macro: [#615](https://github.com/dartsim/dart/issues/615)
  * Fixed linker error by adding namespace-scope definitions for `constexpr static` members: [#603](https://github.com/dartsim/dart/pull/603)
  * Fixed segfault from nullptr meshes: [#585](https://github.com/dartsim/dart/pull/585)
  * Fixed typo of tutorial with minor improvements: [#573](https://github.com/dartsim/dart/pull/573)
  * Fixed `NameManager<T>::removeEntries(~)` called a function that does not exist: [#564](https://github.com/dartsim/dart/pull/564), [#554](https://github.com/dartsim/dart/issues/554)
  * Fixed missing definitions for various functions: [#558](https://github.com/dartsim/dart/pull/558), [#555](https://github.com/dartsim/dart/issues/555)
  * Fixed const correctness of `BodyNode::getMomentsOfInertia()`: [#541](https://github.com/dartsim/dart/pull/541), [#540](https://github.com/dartsim/dart/issues/540)
  * Fixed `ftel` bug in Linux with an workaround: [#533](https://github.com/dartsim/dart/pull/533)
  * Removed unnecessary `virtual` keyword for overriding functions: [#680](https://github.com/dartsim/dart/pull/680)
  * Removed deprecated APIs in DART 5: [#678](https://github.com/dartsim/dart/pull/678)

* Build and test issues

  * Added CMake target for code coverage testing, and automatic reporting: [#688](https://github.com/dartsim/dart/pull/688), [#687](https://github.com/dartsim/dart/issues/687), [#638](https://github.com/dartsim/dart/pull/638), [#632](https://github.com/dartsim/dart/pull/632)
  * Added missing `liburdfdom-dev` dependency in Ubuntu package: [#574](https://github.com/dartsim/dart/pull/574)
  * Modulized DART libraries: [#706](https://github.com/dartsim/dart/pull/706), [#675](https://github.com/dartsim/dart/pull/675), [#652](https://github.com/dartsim/dart/pull/652), [#477](https://github.com/dartsim/dart/issues/477)
  * Improved Travis-CI script: [#655](https://github.com/dartsim/dart/pull/655)
  * Improved CMake script by splitting tutorials, examples, and tests into separate targets: [#644](https://github.com/dartsim/dart/pull/644)
  * Improved wording of the cmake warning messages for ASSIMP: [#553](https://github.com/dartsim/dart/pull/553)
  * Changed Travis-CI to treat warning as errors using `-Werror` flags: [#682](https://github.com/dartsim/dart/pull/682), [#677](https://github.com/dartsim/dart/issues/677)
  * Changed Travis-CI to test DART with bullet collision detector: [#650](https://github.com/dartsim/dart/pull/650), [#376](https://github.com/dartsim/dart/issues/376)
  * Changed the minimum requirement of Visual Studio version to 2015: [#592](https://github.com/dartsim/dart/issues/592)
  * Changed CMake to build gui::osg examples when `DART_BUILD_EXAMPLES` is on: [#536](https://github.com/dartsim/dart/pull/536)
  * Simplfied Travis-CI tests for general pushes: [#700](https://github.com/dartsim/dart/pull/700)
  * Fixed Eigen memory alignment issue in testCollision.cpp: [#719](https://github.com/dartsim/dart/pull/719)
  * Fixed `BULLET_INCLUDE_DIRS` in `DARTConfig.cmake`: [#697](https://github.com/dartsim/dart/pull/697)
  * Fixed linking with Bullet on OS X El Capitan by supporting for Bullet built with double precision: [#660](https://github.com/dartsim/dart/pull/660), [#657](https://github.com/dartsim/dart/issues/657)
  * Fixed FCL version check logic in the main `CMakeLists.txt`: [#640](https://github.com/dartsim/dart/pull/640)
  * Fixed `find_package(DART)` on optimizer components: [#637](https://github.com/dartsim/dart/pull/637)
  * Fixed linking against `${DART_LIBRARIES}` not working in Ubuntu 14.04: [#630](https://github.com/dartsim/dart/pull/630), [#629](https://github.com/dartsim/dart/issues/629)
  * Fixed Visual Studio 2015 build errors: [#580](https://github.com/dartsim/dart/pull/580)
  * Removed OpenGL dependency from `dart` library: [#667](https://github.com/dartsim/dart/pull/667)
  * Removed version check for Bullet: [#636](https://github.com/dartsim/dart/pull/636), [#625](https://github.com/dartsim/dart/issues/625)

## DART 5

### Version 5.1.6 (2017-08-08)

1. Improved camera movement of OpenGL GUI: smooth zooming and translation
    * [Pull request #843](https://github.com/dartsim/dart/pull/843)

2. Removed debian meta files from the main DART repository
    * [Pull request #853](https://github.com/dartsim/dart/pull/853)

### Version 5.1.5 (2017-01-20)

1. Fixed Lemke LCP solver for several failing cases
    * [Pull request #808](https://github.com/dartsim/dart/pull/808)

1. Increase minimum required Ipopt version to 3.11.9
    * [Pull request #800](https://github.com/dartsim/dart/pull/800)

1. Added support of urdfdom_headers 1.0 for DART 5.1 (backport of [#766](https://github.com/dartsim/dart/pull/766))
    * [Pull request #799](https://github.com/dartsim/dart/pull/799)

### Version 5.1.4 (2016-10-14)

1. Fixed inconsistent frame rate of GlutWindow
    * [Pull request #794](https://github.com/dartsim/dart/pull/794)

### Version 5.1.3 (2016-10-07)

1. Updated to support Bullet built with double precision (backport of [#660](https://github.com/dartsim/dart/pull/660))
    * [Pull request #777](https://github.com/dartsim/dart/pull/777)

1. Modified to use btGImpactMeshShape instead of btConvexTriangleMeshShape for mesh
    * [Pull request #764](https://github.com/dartsim/dart/pull/764)

1. Updated to support FCL 0.5 and tinyxml 4.0 (backport of [#749](https://github.com/dartsim/dart/pull/749))
    * [Pull request #759](https://github.com/dartsim/dart/pull/759)

### Version 5.1.2 (2016-04-25)

1. Fixed inverse kinematics (backporting)
    * [Pull request #684](https://github.com/dartsim/dart/pull/684)

1. Fixed aligned memory allocation with Eigen objects in loading meshes
    * [Pull request #606](https://github.com/dartsim/dart/pull/606)

1. Fixed incorrect applying joint constraint impulses (backporting)
    * [Pull request #579](https://github.com/dartsim/dart/pull/579)

1. Fixed some build and packaging issues
    * [Pull request #559](https://github.com/dartsim/dart/pull/559)
    * [Pull request #595](https://github.com/dartsim/dart/pull/595)
    * [Pull request #696](https://github.com/dartsim/dart/pull/696)

### Version 5.1.1 (2015-11-06)

1. Add bullet dependency to package.xml
    * [Pull request #523](https://github.com/dartsim/dart/pull/523)

1. Improved handling of missing symbols of Assimp package
    * [Pull request #542](https://github.com/dartsim/dart/pull/542)

1. Improved travis-ci build log for Mac
    * [Pull request #529](https://github.com/dartsim/dart/pull/529)

1. Fixed warnings in Function.cpp
    * [Pull request #550](https://github.com/dartsim/dart/pull/550)

1. Fixed build failures on AppVeyor
    * [Pull request #543](https://github.com/dartsim/dart/pull/543)

1. Fixed const qualification of ResourceRetriever
    * [Pull request #534](https://github.com/dartsim/dart/pull/534)
    * [Issue #532](https://github.com/dartsim/dart/issues/532)

1. Fixed aligned memory allocation with Eigen objects
    * [Pull request #527](https://github.com/dartsim/dart/pull/527)

1. Fixed copy safety for various classes
    * [Pull request #526](https://github.com/dartsim/dart/pull/526)
    * [Pull request #539](https://github.com/dartsim/dart/pull/539)
    * [Issue #524](https://github.com/dartsim/dart/issues/524)

### Version 5.1.0 (2015-10-15)

1. Fixed incorrect rotational motion of BallJoint and FreeJoint
    * [Pull request #518](https://github.com/dartsim/dart/pull/518)

1. Removed old documents: dart-tutorial, programmingGuide
    * [Pull request #515](https://github.com/dartsim/dart/pull/515)

1. Fixed aligned memory allocation with Eigen objects
    * [Pull request #513](https://github.com/dartsim/dart/pull/513)

1. Fixed segfault in Linkage::Criteria
    * [Pull request #491](https://github.com/dartsim/dart/pull/491)
    * [Issue #489](https://github.com/dartsim/dart/issues/489)

1. Improved sdf/urdf parser
    * [Pull request #497](https://github.com/dartsim/dart/pull/497)
    * [Pull request #485](https://github.com/dartsim/dart/pull/485)

1. Fixed CMake warnings
    * [Pull request #483](https://github.com/dartsim/dart/pull/483)

1. Fixed build issues on Windows
    * [Pull request #516](https://github.com/dartsim/dart/pull/516)
    * [Pull request #509](https://github.com/dartsim/dart/pull/509)
    * [Pull request #486](https://github.com/dartsim/dart/pull/486)
    * [Pull request #482](https://github.com/dartsim/dart/pull/482)
    * [Issue #487](https://github.com/dartsim/dart/issues/487)

1. Fixed IpoptSolver bugs
    * [Pull request #481](https://github.com/dartsim/dart/pull/481)

1. Added Frame::getTransform(withRespecTo, inCoordinatesOf)
    * [Pull request #475](https://github.com/dartsim/dart/pull/475)
    * [Issue #471](https://github.com/dartsim/dart/issues/471)

1. Improved API documentation -- set the SHOW_USED_FILES tag to NO
    * [Pull request #474](https://github.com/dartsim/dart/pull/474)

1. Added convenience setters for generalized coordinates of FreeJoint
    * [Pull request #470](https://github.com/dartsim/dart/pull/470)
    * [Pull request #507](https://github.com/dartsim/dart/pull/507)

1. Fixed compilation warnings
    * [Pull request #480](https://github.com/dartsim/dart/pull/480)
    * [Pull request #469](https://github.com/dartsim/dart/pull/469)
    * [Issue #418](https://github.com/dartsim/dart/issues/418)

1. Added a mutex to Skeleton
    * [Pull request #466](https://github.com/dartsim/dart/pull/466)

1. Added generic URIs support
    * [Pull request #464](https://github.com/dartsim/dart/pull/464)
    * [Pull request #517](https://github.com/dartsim/dart/pull/517)

1. Added End Effector, Inverse Kinematics, and osgDart
    * [Pull request #461](https://github.com/dartsim/dart/pull/461)
    * [Pull request #495](https://github.com/dartsim/dart/pull/495)
    * [Pull request #502](https://github.com/dartsim/dart/pull/502)
    * [Pull request #506](https://github.com/dartsim/dart/pull/506)
    * [Pull request #514](https://github.com/dartsim/dart/pull/514)
    * [Issue #381](https://github.com/dartsim/dart/issues/381)
    * [Issue #454](https://github.com/dartsim/dart/issues/454)
    * [Issue #478](https://github.com/dartsim/dart/issues/478)

1. Removed outdated packaging scripts
    * [Pull request #456](https://github.com/dartsim/dart/pull/456)

1. Added initial position and initial velocity properties
    * [Pull request #449](https://github.com/dartsim/dart/pull/449)

1. Added a package.xml file for REP-136 support
    * [Pull request #446](https://github.com/dartsim/dart/pull/446)

1. Improved Linkage and Chain Criteria
    * [Pull request #443](https://github.com/dartsim/dart/pull/443)
    * [Issue #437](https://github.com/dartsim/dart/issues/437)

1. Added Joint::isCyclic to mark SO(2) topology
    * [Pull request #441](https://github.com/dartsim/dart/pull/441)

1. Fixed SEGFAULTs in DartLoader
    * [Pull request #439](https://github.com/dartsim/dart/pull/439)

1. Added the SYSTEM flag to include_directories
    * [Pull request #435](https://github.com/dartsim/dart/pull/435)

1. Improved Joint warning
    * [Pull request #430](https://github.com/dartsim/dart/pull/430)

1. Added tutorials (http://dart.readthedocs.org/)
    * [Pull request #504](https://github.com/dartsim/dart/pull/504)
    * [Pull request #484](https://github.com/dartsim/dart/pull/484)
    * [Pull request #423](https://github.com/dartsim/dart/pull/423)
    * [Pull request #511](https://github.com/dartsim/dart/pull/511)

### Version 5.0.2 (2015-09-28)

1. Fixed bug in Jacobian update notifications
    * [Pull request #500](https://github.com/dartsim/dart/pull/500)
    * [Issue #499](https://github.com/dartsim/dart/issues/499)

### Version 5.0.1 (2015-07-28)

1. Improved app indexing for bipedStand and atlasSimbicon
    * [Pull request #417](https://github.com/dartsim/dart/pull/417)

1. Added clipping command when it exceeds the limits
    * [Pull request #419](https://github.com/dartsim/dart/pull/419)

1. Improved CollisionNode's index validity check
    * [Pull request #421](https://github.com/dartsim/dart/pull/421)

1. Standardized warning messages for Joints
    * [Pull request #425](https://github.com/dartsim/dart/pull/425)
    * [Pull request #429](https://github.com/dartsim/dart/pull/429)

1. Fixed bug in SDF parser -- correct child for a joint
    * [Pull request #431](https://github.com/dartsim/dart/pull/431)

1. Fixed SDF parsing for single link model without joint
    * [Pull request #444](https://github.com/dartsim/dart/pull/444)

1. Added missing virtual destructors to Properties in Entity and [Soft]BodyNode
    * [Pull request #458](https://github.com/dartsim/dart/pull/458)

1. Limited maximum required version of Assimp less than 3.0~dfsg-4
    * [Pull request #459](https://github.com/dartsim/dart/pull/459)

1. Fixed SEGFAULTs in DartLoader
    * [Pull request #472](https://github.com/dartsim/dart/pull/472)

### Version 5.0.0 (2015-06-15)

1. Fixed aligned memory allocation with Eigen objects
    * [Pull request #414](https://github.com/dartsim/dart/pull/414)

1. Added some missing API for DegreeOfFreedom
    * [Pull request #408](https://github.com/dartsim/dart/pull/408)

1. Replaced logMaps with Eigen::AngleAxisd
    * [Pull request #407](https://github.com/dartsim/dart/pull/407)

1. Improved FCL collision detector
    * [Pull request #405](https://github.com/dartsim/dart/pull/405)

1. Removed deprecated API and suppressed warnings
    * [Pull request #404](https://github.com/dartsim/dart/pull/404)

1. Added use of OpenGL's multisample anti-aliasing
    * [Pull request #402](https://github.com/dartsim/dart/pull/402)

1. Added computation of differences of generalized coordinates
    * [Pull request #389](https://github.com/dartsim/dart/pull/389)
    * [Issue #290](https://github.com/dartsim/dart/issues/290)

1. Added deprecated and force-linline definitions for clang
    * [Pull request #384](https://github.com/dartsim/dart/pull/384)
    * [Issue #379](https://github.com/dartsim/dart/issues/379)

1. Eradicated memory leaks and maked classes copy-safe and clonable
    * [Pull request #369](https://github.com/dartsim/dart/pull/369)
    * [Pull request #390](https://github.com/dartsim/dart/pull/390)
    * [Pull request #391](https://github.com/dartsim/dart/pull/391)
    * [Pull request #392](https://github.com/dartsim/dart/pull/392)
    * [Pull request #397](https://github.com/dartsim/dart/pull/397)
    * [Pull request #415](https://github.com/dartsim/dart/pull/415)
    * [Issue #280](https://github.com/dartsim/dart/issues/280)
    * [Issue #339](https://github.com/dartsim/dart/issues/339)
    * [Issue #370](https://github.com/dartsim/dart/issues/370)
    * [Issue #383](https://github.com/dartsim/dart/issues/383)

1. Improved PlaneShape constructors
    * [Pull request #366](https://github.com/dartsim/dart/pull/366)
    * [Pull request #377](https://github.com/dartsim/dart/pull/377)
    * [Issue #373](https://github.com/dartsim/dart/issues/373)

1. Added appveyor options for parallel build and detailed log
    * [Pull request #365](https://github.com/dartsim/dart/pull/365)

1. Improved robustness and package handling for URDF parsing
    * [Pull request #364](https://github.com/dartsim/dart/pull/364)

1. Fixed bug in BodyNode::_updateBodyJacobianSpatialDeriv()
    * [Pull request #363](https://github.com/dartsim/dart/pull/363)

1. Added alpha channel and Color functions
    * [Pull request #359](https://github.com/dartsim/dart/pull/359)
    * [Issue #358](https://github.com/dartsim/dart/issues/358)

1. Added Jacobian getters to Skeleton
    * [Pull request #357](https://github.com/dartsim/dart/pull/357)

1. Added ArrowShape for visualizing arrows
    * [Pull request #356](https://github.com/dartsim/dart/pull/356)

1. Fixed matrix dimension bug in operationalSpaceControl app
    * [Pull request #354](https://github.com/dartsim/dart/pull/354)

1. Added build type definitions
    * [Pull request #353](https://github.com/dartsim/dart/pull/353)

1. Added Signal class
    * [Pull request #350](https://github.com/dartsim/dart/pull/350)

1. Added LineSegmentShape for visualizing line segments
    * [Pull request #349](https://github.com/dartsim/dart/pull/349)
    * [Issue #346](https://github.com/dartsim/dart/issues/346)

1. Fixed segfault in SoftSdfParser
    * [Pull request #345](https://github.com/dartsim/dart/pull/345)

1. Added subscriptions for destructions and notifications
    * [Pull request #343](https://github.com/dartsim/dart/pull/343)

1. Added NloptSolver::[get/set]NumMaxEvaluations()
    * [Pull request #342](https://github.com/dartsim/dart/pull/342)

1. Added support of Eigen::VectorXd in parser
    * [Pull request #341](https://github.com/dartsim/dart/pull/341)

1. Added Skeleton::getNumJoints()
    * [Pull request #335](https://github.com/dartsim/dart/pull/335)

1. Fixed bug in DARTCollide for sphere-sphere collision
    * [Pull request #332](https://github.com/dartsim/dart/pull/332)

1. Fixed naming issues for Skeletons in World
    * [Pull request #331](https://github.com/dartsim/dart/pull/331)
    * [Issue #330](https://github.com/dartsim/dart/issues/330)

1. Added PlanarJoint support for URDF loader
    * [Pull request #326](https://github.com/dartsim/dart/pull/326)

1. Fixed rotation of the inertia reference frame for URDF loader
    * [Pull request #326](https://github.com/dartsim/dart/pull/326)
    * [Issue #47](https://github.com/dartsim/dart/issues/47)

1. Fixed bug in loading WorldFile
    * [Pull request #325](https://github.com/dartsim/dart/pull/325)

1. Added plotting of 2D trajectories
    * [Pull request #324](https://github.com/dartsim/dart/pull/324)

1. Removed unsupported axis orders of EulerJoint
    * [Pull request #323](https://github.com/dartsim/dart/pull/323)
    * [Issue #321](https://github.com/dartsim/dart/issues/321)

1. Added convenience functions to help with setting joint positions
    * [Pull request #322](https://github.com/dartsim/dart/pull/322)
    * [Pull request #338](https://github.com/dartsim/dart/pull/338)

1. Added Frame class and auto-updating for forward kinematics
    * [Pull request #319](https://github.com/dartsim/dart/pull/319)
    * [Pull request #344](https://github.com/dartsim/dart/pull/344)
    * [Pull request #367](https://github.com/dartsim/dart/pull/367)
    * [Pull request #380](https://github.com/dartsim/dart/pull/380)
    * [Issue #289](https://github.com/dartsim/dart/issues/289)
    * [Issue #294](https://github.com/dartsim/dart/issues/294)
    * [Issue #305](https://github.com/dartsim/dart/issues/305)

1. Added Travis-CI build test for OSX
    * [Pull request #313](https://github.com/dartsim/dart/pull/313)
    * [Issue #258](https://github.com/dartsim/dart/issues/258)

1. Added specification of minimum dependency version
    * [Pull request #306](https://github.com/dartsim/dart/pull/306)

## DART 4

### Version 4.3.7 (2018-01-05)

1. Updated DART 4.3 to be compatible with urdf 1.0/tinyxml2 6/flann 1.9.1
    * [Pull request #955](https://github.com/dartsim/dart/pull/955)

### Version 4.3.6 (2016-04-16)

1. Fixed duplicate entries in Skeleton::mBodyNodes causing segfault in destructor
    * [Issue #671](https://github.com/dartsim/dart/issues/671)
    * [Pull request #672](https://github.com/dartsim/dart/pull/672)

### Version 4.3.5 (2016-01-09)

1. Fixed incorrect applying of joint constraint impulses (backported from 6.0.0)
    * [Pull request #578](https://github.com/dartsim/dart/pull/578)

### Version 4.3.4 (2015-01-24)

1. Fixed build issue with gtest on Mac
    * [Pull request #315](https://github.com/dartsim/dart/pull/315)

### Version 4.3.3 (2015-01-23)

1. Fixed joint Coulomb friction
    * [Pull request #311](https://github.com/dartsim/dart/pull/311)

### Version 4.3.2 (2015-01-22)

1. Fixed installation -- missing headers (utils/urdf, utils/sdf)

### Version 4.3.1 (2015-01-21)

1. Fixed API incompatibility introduced by dart-4.3.0
    * [Issue #303](https://github.com/dartsim/dart/issues/303)
    * [Pull request #309](https://github.com/dartsim/dart/pull/309)

### Version 4.3.0 (2015-01-19)

1. Added name manager for efficient name look-up and unique naming
    * [Pull request #277](https://github.com/dartsim/dart/pull/277)
1. Added all-inclusive header and namespace headers
    * [Pull request #278](https://github.com/dartsim/dart/pull/278)
1. Added DegreeOfFreedom class for getting/setting data of individual generalized coordinates
    * [Pull request #288](https://github.com/dartsim/dart/pull/288)
1. Added hybrid dynamics
    * [Pull request #298](https://github.com/dartsim/dart/pull/298)
1. Added joint actuator types
    * [Pull request #298](https://github.com/dartsim/dart/pull/298)
1. Added Coulomb joint friction
    * [Pull request #301](https://github.com/dartsim/dart/pull/301)
1. Migrated to C++11
    * [Pull request #268](https://github.com/dartsim/dart/pull/268)
    * [Pull request #299](https://github.com/dartsim/dart/pull/299)
1. Improved readability of CMake output messages
    * [Pull request #272](https://github.com/dartsim/dart/pull/272)
1. Fixed const-correctneess of member functions
    * [Pull request #277](https://github.com/dartsim/dart/pull/277)
1. Added handling use of 'package:/' in URDF
    * [Pull request #273](https://github.com/dartsim/dart/pull/273)
    * [Issue #271](https://github.com/dartsim/dart/issues/271)

### Version 4.2.1 (2015-01-07)

1. Fixed version numbering of shared libraries in debian packages
    * [Pull request #286](https://github.com/dartsim/dart/pull/286)
1. Fixed Jacobian and its derivatives of FreeJoint/BallJoint
    * [Pull request #284](https://github.com/dartsim/dart/pull/284)

### Version 4.2.0 (2014-11-22)

1. Added reset functions for Simulation and Recording class
    * [Pull request #231](https://github.com/dartsim/dart/pull/231)
1. Added operational space control example
    * [Pull request #257](https://github.com/dartsim/dart/pull/257)
1. Fixed misuse of Bullet collision shapes
    * [Pull request #228](https://github.com/dartsim/dart/pull/228)
1. Fixed adjacent body pair check for Bullet collision detector
    * [Pull request #246](https://github.com/dartsim/dart/pull/246)
1. Fixed incorrect computation of constraint impulse for BallJointConstraint and WeldJointContraint
    * [Pull request #247](https://github.com/dartsim/dart/pull/247)
1. Improved generation of soft box shape for soft body
    * [Commit ec31f44](https://github.com/dartsim/dart/commit/ec31f44)

### Version 4.1.1 (2014-07-17)

1. Added ABI check script
    * [Pull request #226](https://github.com/dartsim/dart/pull/226)
    * [Pull request #227](https://github.com/dartsim/dart/pull/227)
1. Fixed build issues on Linux
    * [Pull request #214](https://github.com/dartsim/dart/pull/214)
    * [Pull request #219](https://github.com/dartsim/dart/pull/219)
1. Fixed build issues on Windows
    * [Pull request #215](https://github.com/dartsim/dart/pull/215)
    * [Pull request #217](https://github.com/dartsim/dart/pull/217)
1. Fixed unintended warning messages
    * [Pull request #220](https://github.com/dartsim/dart/pull/220)

### Version 4.1.0 (2014-07-02)

1. Fixed bug in switching collision detectors
    * [Issue #127](https://github.com/dartsim/dart/issues/127)
    * [Pull request #195](https://github.com/dartsim/dart/pull/195)
1. Fixed kinematics and dynamics when a skeleton has multiple parent-less bodies
    * [Pull request #196](https://github.com/dartsim/dart/pull/196)
1. Fixed issue on installing DART 4 alongside DART 3 on Linux
    * [Issue #122](https://github.com/dartsim/dart/issues/122)
    * [Pull request #203](https://github.com/dartsim/dart/pull/203)
1. Fixed warnings on gcc
    * [Pull request #206](https://github.com/dartsim/dart/pull/206)
1. Renamed getDof() to getNumDofs()
    * [Pull request #209](https://github.com/dartsim/dart/pull/209)
1. Added cylinder shape for soft body
    * [Pull request #210](https://github.com/dartsim/dart/pull/210)

### Version 4.0.0 (2014-06-02)

1. Added implicit joint spring force and damping force
1. Added planar joint
1. Added soft body dynamics
1. Added computation of velocity and acceleration of COM
1. Added bullet collision detector
  * [Pull request #156](https://github.com/dartsim/dart/pull/156)
1. Improved performance of forward dynamics algorithm
  * [Pull request #188](https://github.com/dartsim/dart/pull/188)
1. Improved dynamics API for Skeleton and Joint
  * [Pull request #161](https://github.com/dartsim/dart/pull/161)
  * [Pull request #192](https://github.com/dartsim/dart/pull/192)
  * [Pull request #193](https://github.com/dartsim/dart/pull/193)
1. Improved constraint dynamics solver
  * [Pull request #184](https://github.com/dartsim/dart/pull/184)
1. Improved calculation of equations of motion using Featherstone algorithm
  * [Issue #85](https://github.com/dartsim/dart/issues/87)
1. Improved optimizer interface and added nlopt solver
  * [Pull request #152](https://github.com/dartsim/dart/pull/152)
1. Fixed self collision bug
  * [Issue #125](https://github.com/dartsim/dart/issues/125)
1. Fixed incorrect integration of BallJoint and FreeJoint
  * [Issue #122](https://github.com/dartsim/dart/issues/122)
  * [Pull request #168](https://github.com/dartsim/dart/pull/168)

## DART 3

### Version 3.0 (2013-11-04)

1. Removed Transformation classes. Their functionality is now included in joint classes.
1. Added Featherstone algorithm. Can currently only be used without collision handling. The old algortihm is still present and used for that case.
1. Removed kinematics namespace. Functionality is moved to dynamics classes.
1. Added dart root namespace
1. A lot of function and variable renames
1. Added constraint namespace
1. Added "common" namespace

## DART 2

### Version 2.6 (2013-09-07)

1. Clean-up of build system:
  * Renamed DART_INCLUDEDIR to the standard-compliant DART_INCLUDE_DIRS in CMake files. Users need to adapt their CMake files for this change.
  * Users no longer need to call find_package(DARTExt) in the CMake files. A call to find_package(DART) also finds its dependencies now.
  * Allow user to overwrite installation prefix
  * Add possibility to include DART header files as '#include \<dart/dynamics/Skeleton.h\>' in addition to '#include \<dynamics/Skeleton.h\>'
  * Allow out-of-source builds
1. URDF loader:
  * Major clean-up
  * Consider mesh scaling factor

### Version 2.5 (2013-07-16)

1. Replaced robotics::World with simulation::World
1. Removed robotics::Robot
1. Added simulation::SimWindow
1. Some speed-up of Eigen calculations
1. Added abstract trajectory interface
1. ConstraintDynamics handles contact, joint limit and other constraint forces simultaneously
1. Improved Lemke algorithm for solving LCP
1. Renamed skeletonDynamics::getQDotVector() to getPoseVelocity()
1. Added abstract CollisionDetector interface allowing for multiple different collision detector implementations.
1. Created math namespace
1. Added System class as base class to Skeleton and Joint
1. URDF loader: Removed ability to load nonstandard URDF files with an object tag
1. Added support for multiple shapes per BodyNode
1. Made urdfdom a dependency instead of including it in the DART source
1. Added function to CollisionDetector to let user check a specific pair of BodyNodes for collision

### Version 2.4 (2013-03-05)

1. Mass and inertia are no longer stored in Shape but in BodyNode.
1. Different shapes for collision and visualization (not just different meshes)
1. Shapes are no longer centered at the COM but can be transformed independently relative to the link frame.
1. Improved URDF support
  * Support for non-mesh shapes
  * Does not create dummy root nodes anymore
  * Support for continuous joints
  * Support for arbitrary joint axes for revolute joints (but not for prismatic joints) instead of only axis-aligned joint axes
  * Support for relative mesh paths even if the robot and world URDF files are in different directories
  * All supported joint types can be root joints
1. Clean-up of the Robot class
1. Removed Object class
1. More robust build and installation process on Linux
