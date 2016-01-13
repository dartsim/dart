# Dominoes
-----------------------------------
1. Controller
	__PUBLIC:__
    + Controller
    	- `Controller` class need to be informed of what we want the manipulator's joint angles to be when no operational space
    		`mQDesired = mManipulator->getPositions()`
            `getPosition()`wil return default initial joint configuration in `createManipulator`
    	- setup information needed for an OS controller
    		1. set last BodyNode as endeffector
    			`mEndEffector = mManipulator->getBodyNode(mManipulator->getNumBodyNodes() -1);`
    		2. offset the origin of Operational Space controller from the origin of BodyNode
    			`mOffset = default_endeffector_offset * Eigen::Vector3d::UnitX();`
    		3. set target on the top of the first domino. Here create a reference frame and place it there
    			`mTarget = std::make_shared<SimpleFrame>(Frame::World(),"targe");`
    		4. compute transform needed to get from the center of the domino to the top of the domino and rotate the target's coordinate frame to __make sure that lines up with the end effector's reference frame__
    			`Eigen::Iosmetry3d target_offset(Eigen::Isometry::Identity());`
                `target_offset.translation() - default_domino_height / 2.0 * Eigen::Vector::UnitZ();`
                __?__ `target_offset.linear() = mEndeffector->getTransform(domino->getBodyNode(0)).linear();`
    		5. setup target so that it has a transform of `target_offset` w.r.t the frame of the domino
    			`mTarget->setTransform(target_offset, domino->getBodyNode(0));`
    + setPDForces
    	- current positions and velocities
    		`Eigen::VectorXd q = mManipulator->getPositions();`
            `Eigen::VectorXd dq = mManipulator->getVelocities();`
		- __?__ integrate the position forward by one timestep, for stability consideration
			`q += dq * mManipulator->getTimeStep()`
        - set PD controller and add gravity and coriolis compensation.
            `Eigen::VectorXd q_err = mQDesired -q;`
            `Eigen::VectorXd dq_err = -dq`
            `const Eigen::MatrixXd& M = mManipulator->getMassMatrix();`
            `const Eigen::VectorXd& Cg = mManipulator->getCoriolisAndGravityForces();`
            `mForces = M * (mKpPD* q_err + mKdPD * dq_err) + Cg;`
            `mManipulator->setForces(mForces)`
    + setOPerationalSpaceForces
> _Operational space controllers can be useful for performing tasks. Operational space controllers allow us to __unify geometric tasks__ (like getting the end efector to a particular spot) and __dynamics tasks__ (like applying a certain force with the end effector) while remaining stable and smooth_

    	- Obtain Mass Matrix
    	- __?__ Obtain Jacobian of the tool offset in the end effector
    		`Jacobian J = mEndEffector->getWorldJacobian(mOffset);`
> But operational space control use the Moore-Penrose pseudoinverse of the Jacobian rather than the Jacobian itself
			
            `Eigen::MatrixXd pinv_J = J.transpose() * (J*J.transpose() + 0.0025* Eigen::Matrix::Identity()).inverse();`
		- __?__ Obtain derivative of the Jacobian, as well as its pseudoinverse
> Here we compute the __classic__ derivatives, which means the derivative of Jacobian w.r.t. time in __classical coordinates__ rather than __?__ __spatial coordinates__.

			`Jacobian dJ = mEndEffector->getJacobianClassicDeriv(mOffset);`
            `Eigen::MatrixXd pinv_dJ = dJ.transpose() * (dJ * dJ.transpose() + 0.0025 * Eigen::Matrix6d::Identity()).inverse();`
		- compute linear and angular error, and time derivative of error
			`Eigen::Vector6d e;`
            `e.tail<3> = mTarget->getWorldTransform().translation() - mEndEffector->getWorldTransform() * mOffset;`
            `Eigen::AngleAxisd aa(mTarget->getTransform(mEndEffector).linear());`
            `e.head<3>() = aa.angle() * aa.axis();`
            `Eigen::Vector6d de = -mEndEffector->getSpatialVelocity(mOffset, mTarget.get(), Frame::World());`
		- compensate gravity and coriolis forces
		- convert OS controller gains into matrix form
			`Eigen::Matrix6d Kp = mKpOS * Eigen::Matrix6d::Identity();`
            `size_t dofs = mManipulator->getNumDofs();`
            `Eigen::MatrixXd Kd = mKdOS * Eigen::MatrixXd::Identity(dofs,dofs);`
		- compute the joint forces needed to achieve our desired end effect force.
			`Eigen::Vector6d fDesired = Eigen::Vector6d::Zero();`
            `fDesired[3] =default_push_force;`
            `Eigen::VectorXd f = J.transpose() * fDesired;`
		- __?__ mix everything into a single control law
			`Eigen::VectorXd dq = mManipulator->getVelocities();`
            `mForces = M * (pinv_J * Kp * de + pinv_dJ * Kp * e) - Kd * dq + Kd * pinv_J * Kp * e + Cg + f;`

 __PROTECTED:__
    + arguments: _mManiputlator, mTartget, mEndeffector, mQDesired (Desired Joint position when not applying operational space controller), mOffset,mKpPD, mKdPD, mKpOS, mKDOS, mForces_
2. MyWindow
	__PUBLIC:__
    + MyWindow
    	- arguments: _mTotalAngle = 0.0, mHasEverRun = false, mForceCountDown = 0, mPushCountDown = 0_
    	- content: _setWorld, set mFirstDomino, mFloor, mController_
    + attemptToCreateDomino
    	- clone from existing skeleton and change the skeleton name (clone  will keep the same name)
    		`SkeletonPtr newDomino = mFirstDomino->clone();`
            `newDomino->setName("domino #" + std::to_string(mDominoes.size() + 1));`
		- set positions and angles --> newDomino's root Joint has a FreeJoint which has six degrees of freedom: the first three are for orientation and last three are for translation.
			`Eigen::Vector3d dx = default_distance * Eigen:Vector3d(cos(mTotalAngle), sin(mTotoalAngle), 0.0);`
            `Eigen::Vector6d x = lastDomino-getPositions();`
            `x.tail<3>() += dx`
            `x[2] = mTotalAngle + angle;`
        - ensure no collision
        	_contacting with Floor is okay, i.e. if neither of the colliding BodyNodes belong to the floor, then the new domino is in contact with something it shouldn't be_
            ```if (dominoCollision){mWorld->removeSkeleton(newDomino);}else{mAngles.push_back(angle);mDominoes.push_back(newDomino);mTotalAngle+=angle;}```
    + deleteLastDomino
    	- The `SkeletonPtr` class is a `std::shared_ptr<Skeleton>`, so we don't need to call `delete` to free memory space. Its resources will be freed when `lastDomino` goes out of scope.
    + keyboard
    	- add dominoes in a specified angle, or delete last domino; self-iteration
    + timestepping -->__This function will be visited whenever the user presses 'f'__
		- apply external force to the first domino
			`mFirstDomino->getBodyNode(0)->addExtForce(force, location)`
            `--mforceCountDown;` --> force should be applied through some time. `Ft=mv`
		- run the controller for the manipulator

 __PROTECTED:__
    + arguments: _mFirstDomino, mFloor, mDominoes (History of dominoes that are added), mAngles, mTotalAngle, mHasEverRun, mForceCountDown, mForceDown, mPushCountDown, mController_
    	
3. createDomino
	+ create a skeleton called "domino"
	+ create a body for the domino
		`BodyNodePtr body = domino->createJointAndBodyNodePair<FreeJoint>(nullptr).second`
    + create a shape for the domino
    	_set shape as box, addVisualizationShape, addCollisionShape_
    + set inertia for the domino
    	`dart::dynamics::Inertia inertia;`
        `inertia.setMass(default_domino_mass);`
        `inertia.setMoment(box->computeInertia(default_domino_mass));`
        `body->setInertia(inertia);`
	+ __?__ Joint_pos_z
		`domino->getDof("Joint_pos_z")->setPosition(default_domino_height / 2.0);`
4. createFloor
	+ create a skeleton called "floor"
	+ give the floor a body
		`BodyNodePtr body = floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second`
	+ give the body a shape
		_set shape as BOX, color as Black and addVisulaizationShape and addCollisionShape_
	+ put the body into position
		`Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());`
       `tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);`       
		`body->getParentJoint()->setTransformFromParentBodyNode(tf);` 
5. createManipulator
	+ load a URDF robotic manipulator
		`dart::utils::DartLoader loader;`
        `SkeletonPtr maniputlator = loader.parseSkeleton(DART_DATA_PATH"urdf/KR5/KR5 sixx R650.urdf);`
        `manipulator->setName("manipulator);`
	+ set manipulator positions and configuration (experiments)
		`Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();`
        `tf.translation() = Eigen::Vector(-0.65, 0.0, 0.0);`
        `manipulator->getJoint(0)->setTransformFromParentBodyNode(tf)`
        _inital degree for Dof(1) and Dof(2), so that the endeffector is exactaly towards the first Domino_
        `manipulator->getDof(1)->setPosition(140.0 * M_PI / 180.0);`
        `manipulator->getDof(2)->setPosition(-140.0 * M_PI / 180.0);`
6. int main
	+ create Domino
	+ create Floor
	+ create Manipulator
	+ __?__ `glutInit(&argc, argv);window.initWindow(640, 480, "Dominoes");glutMainLoop();`