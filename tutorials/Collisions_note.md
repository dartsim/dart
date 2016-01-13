# Collision
1. setupRing  --> setup a chain of BodyNodes so it behaves like a closed ring
	+ set spring and damping coefficients
		_skip the root BodyNode when setting spring and damping coefficients_
	+ set rest positions of the joints
		_BallJoint provides a function for converting rotations into a position vector_
	+ set Joint initial positions as rest position

2. MyWindow

     __PUBLIC__:
	+ Mywindow
		_arguments: world, five createobject func_
		_content: initialize all variables and setWorld_
	+ keyboard
		_call corresponding "addObject or addRing" function accourding to keys pressed; self-iteration_
		`addObject(mOriginalBall->clone())`
	+ drawskels
		__?__ _glEnable(gl_lighting)? glpolygonMode(gl_front_and_back, gl_fill)
    + displayTimer
		__?__ _display timer_

   __PROTECTED__:
	+ addObject  --> add object to current world, i.e. toss current object in certain direction
		- positions
			`Eigen::Vector6d::Zero()`
			`position[4] = default_spawrn_range * mDistribution[mMT);`  (mDistribution(mMT) will generate a random value in the range [-1, 1] inclusively)
			`position[5] = default_start_height`
		- set the positions of the root Joint
			`object->getJoint(0)->setPositions(positions)`
		- add object to the world after name it
			`mWorld->addskeleton(object)`
		- compute collisions --> ensure new object is not inside of something else, so use collisition to check it
			`dart::collision::CollisionDetector* detector = mWorld->getConstrainSolver()->getCollisionDetector();`
			`detector->detectorCollision(true, true)`
		- look through list of collisions to check wether any of them are new skeleton
		- remove the new skeleton if it is in collision with something else and output warning
		- __?__ create reference frames  --> Frame Semantics: create refernce frames and use them to get and set data relative to arbitrary frames, two Frames types used in DART are BodyNodes and SimpleFrames
			* create a SimpleFrame at Skeleton's center of mass
			* set ojbect's angular and linear speed
			* directional velocities (factorize the angular and linear speed into world coordinate)
			* set velocity properites of simpleFrame
				1. center.setClassicDerivatives(v,w);  --> setClassicDerivatives allows to set the classic linear and angular velocities and accerlations of a simpleFrame w.r.t its parent frame, i.e. the world frame
				2. __?__ spatial velocity and acceleration vectors
			* __?__ create a new simpleFrame as a child of previous one and set the origin of this new frame to line up with the Root BodyNode of object

	+ addRing  --> create a closed kinematic chain, attach the first and last BodyNode by a BallJoint-style constraint
		+ head, tail
		+ __?__ offset
			`offset = tail->getWorldTransform() * offset` -->the offset will be located half the default height up from the center of tail BodyNode
		+ create constraint and add it to world's constraint solver and add to list of constraints

	+ removeSkeleton
		+ first remove constraint, then remove skeleton
	+ variables: _mRandomize, mRD, mMT, mDistribution, mJointConstraints, mOriginalBall, mOriginalSoftBody, mOriginalHybridBody, mOriginalRigidChain, mOriginalRigidRing, mSkelcount_

3. `template<JointType>  addRigidBody` --> add a rigid body with the specified Joint type to a chain
	+ arguments: _chain, name, shapetype, parent = nullptr_
	+ set joint properties, including name
	+ consider offsetting the new Bodynode from its parent Bodynode
		- __?__ offset__?__ transform matrix__?__ how to compute__?__
		  `Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());`
		  `tf.translation() = Eigen::Vector3d(0, 0, default_shape_height/ 2.0);`
		  `properties.mt_ParentBodyToJoint = tf;`
		  `properties.mt_childbodytoJoint = tf.inverse();`
	+ create JointandBodyNodePair
		`BodyNode* bn = chain->createJointAndBodyNodePair<JointType, BodyNodeType>(parent, Joint_properties, BodyNode::Properties(name)).second`
	+ specify the shape
		`box: Eigen::Vector3d(x,x,x)`
		`cylinder: (x,x)`
		`ellipsoid: default_height*Eigen::Vector3d::Ones()`
	+ addvisualizationShape
	+ addCollisionShape
	+ setup inertia properties of BodyNode according to its geometry, including mass and MomentOfInertia
	+ setup the RestitutionCoefficient for BodyNode
	+ set damping coefficients for each dof of parent of current BodyNode

4. `template<JointType>  addsoftBody`
	+ arguments: _chain, name, SofShapeType, parent = nullptr_
	+ set joint properties, including name
	+ consider offsetting the new Bodynode from its parent Bodynode
		- __?__ offset__?__ transform matrix__?__ how to compute__?__
		  `Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());`
		  `tf.translation() = Eigen::Vector3d(0, 0, default_shape_height/ 2.0);`
		  `properties.mt_ParentBodyToJoint = tf;`
		  `properties.mt_childbodytoJoint = tf.inverse();`
	+ set SoftBodyNode UniqueProperties __?__ why before call createJointAndBodyNodePair
		- specify SoftBodyNode's shape, using SoftBodyNodeHelper()
			`SOFT_BOX`
			`SOFT_CYLINDER`
			`SOFT_ELLIPSOID`
		- soft_properties.mKv, mKe, mDampCoeff
	+ combine unique properties with stardard properties for SoftBodyNode
	+ create JointAndBodyNodePair
		`SoftBodyNode* bn = chain->createJointAndBodyNodePair<JointType, SoftBodyNode>(parent, Joint_properties, BodyNode::Properties(name)).second`
	+ zeroout inertia, including MomentOfInertia and Mass
	+ make the shape transparent for softBodyNode
	+ __?__ addVisualizationShape, addCollisionShape -->(Upon creation, a SoftBodyNode will have exactly one visualization shape: the soft shape visualizer)

5. `enumeration SoftShapeType {SOFT_BOX, SOFT_CYLINDER, SOFT_ELLIPSOID}`

6. setAllColors
	+ set each visualization shape of each Bodynode as the passed color

7. createBall
	+ create skeleton called rigid_ball
	+ use `template<JointType> addRigidBody` to give the ball skeleton a body
	+ set the ball skeleton color as red

8. createRigidChain
	+ create skeleton called rigid_chain
	+ use `template<JointType> addRigidBody` to give the rigid chain skeleton a body
	+ add child BodyNode to parent to have a chain
	+ set chain skeleton color as orange

9. createRigidRing
	+ create skeleton called rigid_ring
	+ use `template<JointType> addRigidBody` to give the rigid ring skeleton a body
	+ add child BodyNode to parent to have a ring
	+ set ring skeleton color as blue

10. createSoftBody
	+ create skeleton called soft
	+ use `template<JointType> addSoftBody` to give the soft skeleton a body
	+ create SoftBody Bone
		- scaled down BodyNode shape from SoftBodyNode
		- addCollisionShape
		- addVisualizationShape
		- set inertia, including mass and MomentOfInertia (inertia computed according to the shape)
	+ set soft skeleton color as Fuschia

11. createHybridBdoy  --> HybridBody is a protruding rigid body attached to a SoftBodyNode using WeldJoint
	+ create a skeleton called hybrid
	+ use `template<JointType> addSoftBody` to give the hybrid skeleton a body
	+ create a new rigid body with WeldJoint
		- specify shape
		- addCollisionShape
		- addVisualizationShape
		- TO MAKE BOX PROTRUDE, shift it away from the center of its parent
			`bn->getParentJoint()->setTransformFromParentBodyNode(tf)`
		- set inertia

12. createGround	
	+ add a skeleton;
	+ createJointandBodyNodePair;
	+ shape for BodyNode; shape color;
	+ add CollisionShape to BodyNode;
	+ add VisualzationShape to BodyNode;

13. createWall	
	+ repeat as creat Ground;
	+ connect the wall with the ground__?__ what's the transform matrix for__?__
		`Eigen::Isometry3d tf();`
		`tf.translation() = Eigen::Vector3d(x,x,x);`

14. main
```
	1 for rigid ball
    2 for soft body
	3 for hybrid soft/rigid body
	4 for rigid chain
	5 for rigid ring
```