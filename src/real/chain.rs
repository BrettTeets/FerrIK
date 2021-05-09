use super::{bone::Bone, joint::Joint, joint::JointType, joint, util};
use cgmath::{Vector3, Rad, InnerSpace, Matrix3};

#[derive(Clone, Copy, PartialEq)]
pub enum BaseboneConstraintType
{
    NONE,         // No constraint - basebone may rotate freely
    GLOBAL_ROTOR, // World-space rotor constraint
    LOCAL_ROTOR,  // Rotor constraint in the coordinate space of (i.e. relative to) the direction of the connected bone
    GLOBAL_HINGE, // World-space hinge constraint
    LOCAL_HINGE   // Hinge constraint in the coordinate space of (i.e. relative to) the direction of the connected bone
}

pub struct Chain{
    mChain: Vec<Bone>,
    mSolveDistanceThreshold: f32,
    mMaxIterationAttempts: usize,
    mMinIterationChange: f32,
    mChainLength: f32,
    mFixedBaseLocation: Vector3<f32>,
    mFixedBaseMode: bool,
    mBaseboneConstraintType: BaseboneConstraintType,
    mBaseboneConstraintUV: Vector3<f32>,
    mBaseboneRelativeConstraintUV: Vector3<f32>,
    mBaseboneRelativeReferenceConstraintUV: Vector3<f32>,
    mLastTargetLocation: Vector3<f32>,
    mConstraintLineWidth: f32,
    mLastBaseLocation: Vector3<f32>,
    mCurrentSolveDistance: f32,
    mConnectedChainNumber: Option<usize>,
    mConnectedBoneNumber: Option<usize>,
    mEmbeddedTarget: Vector3<f32>,
    mUseEmbeddedTarget: bool,
}

impl Chain{
    pub fn new() -> Self{
        Self{
            mChain: Vec::new(),
            mSolveDistanceThreshold: 1.0,
            mMaxIterationAttempts: 20,
            mMinIterationChange: 0.01,
            mChainLength:  0.0,
            mFixedBaseLocation: Vector3::new(0.0,0.0,0.0),
            mFixedBaseMode: true,
            mBaseboneConstraintType: BaseboneConstraintType::NONE,
            mBaseboneConstraintUV: Vector3::new(0.0,0.0,0.0),
            mBaseboneRelativeConstraintUV: Vector3::new(0.0,0.0,0.0),
            mBaseboneRelativeReferenceConstraintUV: Vector3::new(0.0,0.0,0.0),
            mLastTargetLocation: Vector3::new(f32::MAX,f32::MAX,f32::MAX),
            mConstraintLineWidth: 2.0,
            mLastBaseLocation: Vector3::new(f32::MAX,f32::MAX,f32::MAX),
            mCurrentSolveDistance: f32::MAX,
            mConnectedChainNumber: None,
            mConnectedBoneNumber: None,
            mEmbeddedTarget: Vector3::new(0.0,0.0,0.0),
            mUseEmbeddedTarget: false,
        }
    }
}

impl Chain{
    pub fn solveIK(&mut self, target: Vector3<f32>) -> f32
	{	
		// Sanity check that there are bones in the chain
		if self.mChain.is_empty() { 
		  panic!("It makes no sense to solve an IK chain with zero bones."); 
		}
		
		// ---------- Forward pass from end effector to base -----------

		// Loop over all bones in the chain, from the end effector (numBones-1) back to the basebone (0)		
		//for (int loop = mChain.size()-1; loop >= 0; --loop)
        for index in (0..self.mChain.len()).rev()
		{
			// Get the length of the bone we're working on
            //Things I have treid.
			    //let mut thisBone: Bone = self.mChain[index];
                //let thisBone: &mut Bone = self.mChain.get_mut(index).unwrap();
            let mut thisBone: Bone = self.mChain[index];
			let thisBoneLength: f32  = thisBone.length();
			let thisBoneJoint: &Joint = thisBone.getJoint(); //odd, this one asked to be a reference.
			let thisBoneJointType: JointType = thisBone.getJointType();

			// If we are NOT working on the end effector bone
			if index != self.mChain.len() - 1
			{
				// Get the outer-to-inner unit vector of the bone further out
				let outerBoneOuterToInnerUV: Vector3<f32> = -self.mChain[index+1].getDirectionUV();

				// Get the outer-to-inner unit vector of this bone
				let mut thisBoneOuterToInnerUV: Vector3<f32> = -thisBone.getDirectionUV();
				
				// Get the joint type for this bone and handle constraints on thisBoneInnerToOuterUV				
				if thisBoneJointType == JointType::BALL
				{	
					// Constrain to relative angle between this bone and the outer bone if required
					let angleBetweenDegs: Rad<f32>    = util::getAngleBetweenDegs(outerBoneOuterToInnerUV, thisBoneOuterToInnerUV);
					let constraintAngleDegs: Rad<f32> = thisBoneJoint.getBallJointConstraintDegs();
					if angleBetweenDegs > constraintAngleDegs
					{	
						thisBoneOuterToInnerUV = util::getAngleLimitedUnitVectorDegs(thisBoneOuterToInnerUV, outerBoneOuterToInnerUV, constraintAngleDegs);
					}
				}
				else if thisBoneJointType == JointType::GLOBAL_HINGE
				{	
					// Project this bone outer-to-inner direction onto the hinge rotation axis
					// Note: The returned vector is normalised.
					thisBoneOuterToInnerUV = util::projectOntoPlane(thisBoneOuterToInnerUV, thisBoneJoint.getHingeRotationAxis() ); 
					
					// NOTE: Constraining about the hinge reference axis on this forward pass leads to poor solutions... so we won't.
				}
				else if thisBoneJointType == JointType::LOCAL_HINGE
				{	
					// Not a basebone? Then construct a rotation matrix based on the previous bones inner-to-to-inner direction...
					let m: Matrix3<f32>;
					let relativeHingeRotationAxis: Vector3<f32>;
					if index > 0 {
						m = util::createRotationMatrix( self.mChain[index-1].getDirectionUV() );
						relativeHingeRotationAxis = m * ( thisBoneJoint.getHingeRotationAxis() ).normalize();
					}
					else // ...basebone? Need to construct matrix from the relative constraint UV.
					{
						relativeHingeRotationAxis = self.mBaseboneRelativeConstraintUV;
					}
					
					// ...and transform the hinge rotation axis into the previous bones frame of reference.
					//Vec3f 
										
					// Project this bone's outer-to-inner direction onto the plane described by the relative hinge rotation axis
					// Note: The returned vector is normalised.					
					thisBoneOuterToInnerUV = util::projectOntoPlane(thisBoneOuterToInnerUV, relativeHingeRotationAxis);
										
					// NOTE: Constraining about the hinge reference axis on this forward pass leads to poor solutions... so we won't.										
				}
					
				// At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
				// so we can set the new inner joint location to be the end joint location of this bone plus the
				// outer-to-inner direction unit vector multiplied by the length of the bone.
				let newStartLocation: Vector3<f32> = thisBone.getEndLocation() + ( thisBoneOuterToInnerUV * (thisBoneLength) );

				// Set the new start joint location for this bone
				thisBone.setStartLocation(newStartLocation);

				// If we are not working on the basebone, then we also set the end joint location of
				// the previous bone in the chain (i.e. the bone closer to the base) to be the new
				// start joint location of this bone.
				if index > 0
				{
					self.mChain[index-1].setEndLocation(newStartLocation);
				}
			}
			else // If we ARE working on the end effector bone...
			{
				// Snap the end effector's end location to the target
				thisBone.setEndLocation(target);
				
				// Get the UV between the target / end-location (which are now the same) and the start location of this bone
				let thisBoneOuterToInnerUV: Vector3<f32> = -thisBone.getDirectionUV();
				
				// If the end effector is global hinged then we have to snap to it, then keep that
				// resulting outer-to-inner UV in the plane of the hinge rotation axis
				match thisBoneJointType 
				{
					JointType::BALL => ()
						// Ball joints do not get constrained on this forward pass
						,				
                    JointType::GLOBAL_HINGE =>{
						// Global hinges get constrained to the hinge rotation axis, but not the reference axis within the hinge plane
						thisBoneOuterToInnerUV = util::projectOntoPlane(thisBoneOuterToInnerUV, thisBoneJoint.getHingeRotationAxis() );
                        },
                    JointType::LOCAL_HINGE => {
						// Local hinges get constrained to the hinge rotation axis, but not the reference axis within the hinge plane
						
						// Construct a rotation matrix based on the previous bones inner-to-to-inner direction...
						let m: Matrix3<f32> = util::createRotationMatrix( self.mChain[index-1].getDirectionUV() );
						
						// ...and transform the hinge rotation axis into the previous bones frame of reference.
						let relativeHingeRotationAxis: Vector3<f32> = m * ( thisBoneJoint.getHingeRotationAxis() ).normalize();
											
						// Project this bone's outer-to-inner direction onto the plane described by the relative hinge rotation axis
						// Note: The returned vector is normalised.					
						thisBoneOuterToInnerUV = util::projectOntoPlane(thisBoneOuterToInnerUV, relativeHingeRotationAxis);
                        },
				}
												
				// Calculate the new start joint location as the end joint location plus the outer-to-inner direction UV
				// multiplied by the length of the bone.
				let newStartLocation: Vector3<f32> = target + ( thisBoneOuterToInnerUV * (thisBoneLength) );
				
				// Set the new start joint location for this bone to be new start location...
				thisBone.setStartLocation(newStartLocation);

				// ...and set the end joint location of the bone further in to also be at the new start location (if there IS a bone
				// further in - this may be a single bone chain)
				if index > 0
				{
					self.mChain[index-1].setEndLocation(newStartLocation);
				}
			}
			
		} // End of forward pass

		// ---------- Backward pass from base to end effector -----------
 
		//for (int loop = 0; loop < mChain.size(); ++loop)
        for index in 0..self.mChain.len()
		{
			let thisBone: Bone = self.mChain[index];
			let thisBoneLength: f32  = thisBone.length();

			// If we are not working on the basebone
			if index != 0
			{
				// Get the inner-to-outer direction of this bone as well as the previous bone to use as a baseline
				let thisBoneInnerToOuterUV: Vector3<f32> = thisBone.getDirectionUV();
				let prevBoneInnerToOuterUV: Vector3<f32> = self.mChain[index-1].getDirectionUV();
				
				// Dealing with a ball joint?
				let thisBoneJoint: &Joint = thisBone.getJoint();
				let jointType: JointType = thisBoneJoint.getJointType();
				if jointType == JointType::BALL
				{					
					let angleBetweenDegs: Rad<f32>    = util::getAngleBetweenDegs(prevBoneInnerToOuterUV, thisBoneInnerToOuterUV);
					let constraintAngleDegs: Rad<f32> = thisBoneJoint.getBallJointConstraintDegs(); 
					
					// Keep this bone direction constrained within the rotor about the previous bone direction
					if angleBetweenDegs > constraintAngleDegs
					{
						thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, prevBoneInnerToOuterUV, constraintAngleDegs);
					}
				}
				else if jointType == JointType::GLOBAL_HINGE
				{					
					// Get the hinge rotation axis and project our inner-to-outer UV onto it
					let hingeRotationAxis: Vector3<f32>  =  thisBoneJoint.getHingeRotationAxis();
					thisBoneInnerToOuterUV = util::projectOntoPlane(thisBoneInnerToOuterUV, hingeRotationAxis);
					
					// If there are joint constraints, then we must honour them...
					let cwConstraintDegs: Rad<f32>   = -thisBoneJoint.getHingeClockwiseConstraintDegs();
					let acwConstraintDegs: Rad<f32>  =  thisBoneJoint.getHingeAnticlockwiseConstraintDegs();
                    //passing the scalar of the rads in.
					if  !( util::approximatelyEquals(cwConstraintDegs.0, -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.001) ) &&
						 !( util::approximatelyEquals(acwConstraintDegs.0, joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.001) ) 
					{
						let hingeReferenceAxis: Vector3<f32> =  thisBoneJoint.getHingeReferenceAxis();
						
						// Get the signed angle (about the hinge rotation axis) between the hinge reference axis and the hinge-rotation aligned bone UV
						// Note: ACW rotation is positive, CW rotation is negative.
						let signedAngleDegs: f32 = util::getSignedAngleBetweenDegs(hingeReferenceAxis, thisBoneInnerToOuterUV, hingeRotationAxis);
						
						// Make our bone inner-to-outer UV the hinge reference axis rotated by its maximum clockwise or anticlockwise rotation as required
			        	if signedAngleDegs > acwConstraintDegs.0
			        	{	
			        		thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, acwConstraintDegs, hingeRotationAxis).normalize();		        		
			        	}
			        	else if signedAngleDegs < cwConstraintDegs.0
			        	{	
			        		thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, cwConstraintDegs, hingeRotationAxis).normalize();			        		
			        	}
					}
				}
				else if jointType == JointType::LOCAL_HINGE
				{	
					// Transform the hinge rotation axis to be relative to the previous bone in the chain
					let hingeRotationAxis: Vector3<f32>  = thisBoneJoint.getHingeRotationAxis();
					
					// Construct a rotation matrix based on the previous bone's direction
					let m: Matrix3<f32> = util::createRotationMatrix(prevBoneInnerToOuterUV);
					
					// Transform the hinge rotation axis into the previous bone's frame of reference
					let relativeHingeRotationAxis: Vector3<f32>  = m * (hingeRotationAxis).normalize();
					
					
					// Project this bone direction onto the plane described by the hinge rotation axis
					// Note: The returned vector is normalised.
					thisBoneInnerToOuterUV = util::projectOntoPlane(thisBoneInnerToOuterUV, relativeHingeRotationAxis);
					
					// Constrain rotation about reference axis if required
					let cwConstraintDegs: Rad<f32>   = -thisBoneJoint.getHingeClockwiseConstraintDegs();
					let acwConstraintDegs: Rad<f32>  =  thisBoneJoint.getHingeAnticlockwiseConstraintDegs();
					if  !( util::approximatelyEquals(cwConstraintDegs.0, -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.001) ) &&
						 !( util::approximatelyEquals(acwConstraintDegs.0, joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.001) ) 
					{
						// Calc. the reference axis in local space
						let relativeHingeReferenceAxis: Vector3<f32> = m * ( thisBoneJoint.getHingeReferenceAxis() ).normalize();
						
						// Get the signed angle (about the hinge rotation axis) between the hinge reference axis and the hinge-rotation aligned bone UV
						// Note: ACW rotation is positive, CW rotation is negative.
						let signedAngleDegs: f32 = util::getSignedAngleBetweenDegs(relativeHingeReferenceAxis, thisBoneInnerToOuterUV, relativeHingeRotationAxis);
						
						// Make our bone inner-to-outer UV the hinge reference axis rotated by its maximum clockwise or anticlockwise rotation as required
			        	if signedAngleDegs > acwConstraintDegs.0
			        	{	
			        		thisBoneInnerToOuterUV = util::rotateAboutAxis(relativeHingeReferenceAxis, acwConstraintDegs, relativeHingeRotationAxis).normalize();		        		
			        	}
			        	else if signedAngleDegs < cwConstraintDegs.0
			        	{	
			        		thisBoneInnerToOuterUV = util::rotateAboutAxis(relativeHingeReferenceAxis, cwConstraintDegs, relativeHingeRotationAxis).normalize();			        		
			        	}
					}
					
				} // End of local hinge section
				
				// At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
				// so we can set the new inner joint location to be the end joint location of this bone plus the
				// outer-to-inner direction unit vector multiplied by the length of the bone.
				let newEndLocation: Vector3<f32> = thisBone.getStartLocation() + ( thisBoneInnerToOuterUV * (thisBoneLength) );

				// Set the new start joint location for this bone
				thisBone.setEndLocation(newEndLocation);

				// If we are not working on the end effector bone, then we set the start joint location of the next bone in
				// the chain (i.e. the bone closer to the target) to be the new end joint location of this bone.
				if index < self.mChain.len() - 1 { 
				  self.mChain[index+1].setStartLocation(newEndLocation); 
				}
			}
			else // If we ARE working on the basebone...
			{	
				// If the base location is fixed then snap the start location of the basebone back to the fixed base...
				if self.mFixedBaseMode
				{
					thisBone.setStartLocation(self.mFixedBaseLocation);
				}
				else // ...otherwise project it backwards from the end to the start by its length.
				{
					thisBone.setStartLocation( thisBone.getEndLocation() - ( thisBone.getDirectionUV() * (thisBoneLength) ) );
				}
				
				// If the basebone is unconstrained then process it as usual...
				if self.mBaseboneConstraintType == BaseboneConstraintType::NONE
				{
					// Set the new end location of this bone, and if there are more bones,
					// then set the start location of the next bone to be the end location of this bone
					let newEndLocation: Vector3<f32> = thisBone.getStartLocation() + ( thisBone.getDirectionUV() * (thisBoneLength) );
					thisBone.setEndLocation(newEndLocation);	
					
					if self.mChain.len() > 1 { 
					  self.mChain[1].setStartLocation(newEndLocation); 
					}
				}
				else // ...otherwise we must constrain it to the basebone constraint unit vector
				{	
					if self.mBaseboneConstraintType == BaseboneConstraintType::GLOBAL_ROTOR
					{	
						// Get the inner-to-outer direction of this bone
						let thisBoneInnerToOuterUV: Vector3<f32> = thisBone.getDirectionUV();
								
						let angleBetweenDegs: Rad<f32>    = util::getAngleBetweenDegs(self.mBaseboneConstraintUV, thisBoneInnerToOuterUV);
						let constraintAngleDegs: Rad<f32> = thisBone.getBallJointConstraintDegs(); 
					
						if angleBetweenDegs > constraintAngleDegs
						{
							thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, self.mBaseboneConstraintUV, constraintAngleDegs);
						}
						
						let newEndLocation: Vector3<f32> = thisBone.getStartLocation() + ( thisBoneInnerToOuterUV * (thisBoneLength) );
						
						thisBone.setEndLocation( newEndLocation );
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.mChain.len() > 1 { 
						  self.mChain[1].setStartLocation(newEndLocation); 
						}
					}
					else if self.mBaseboneConstraintType == BaseboneConstraintType::LOCAL_ROTOR
					{
						// Note: The mBaseboneRelativeConstraintUV is updated in the FabrikStructure3D.solveForTarget()
						// method BEFORE this FabrikChain3D.solveForTarget() method is called. We no knowledge of the
						// direction of the bone we're connected to in another chain and so cannot calculate this 
						// relative basebone constraint direction on our own, but the FabrikStructure3D does it for
						// us so we are now free to use it here.
						
						// Get the inner-to-outer direction of this bone
						let thisBoneInnerToOuterUV: Vector3<f32> = thisBone.getDirectionUV();
								
						// Constrain about the relative basebone constraint unit vector as neccessary
						let angleBetweenDegs: Rad<f32>    = util::getAngleBetweenDegs(self.mBaseboneRelativeConstraintUV, thisBoneInnerToOuterUV);
						let constraintAngleDegs: Rad<f32> = thisBone.getBallJointConstraintDegs();
						if angleBetweenDegs > constraintAngleDegs
						{
							thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, self.mBaseboneRelativeConstraintUV, constraintAngleDegs);
						}
						
						// Set the end location
						let newEndLocation: Vector3<f32> = thisBone.getStartLocation() + ( thisBoneInnerToOuterUV *(thisBoneLength) );						
						thisBone.setEndLocation( newEndLocation );
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.mChain.len() > 1 { 
						  self.mChain[1].setStartLocation(newEndLocation); 
						}
					}
					else if self.mBaseboneConstraintType == BaseboneConstraintType::GLOBAL_HINGE
					{
						let thisJoint: &Joint  =  thisBone.getJoint();
						let hingeRotationAxis: Vector3<f32>  =  thisJoint.getHingeRotationAxis();
						let cwConstraintDegs: Rad<f32>   = -thisJoint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
						let acwConstraintDegs: Rad<f32>  =  thisJoint.getHingeAnticlockwiseConstraintDegs();
						
						// Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
						let thisBoneInnerToOuterUV: Vector3<f32> = util::projectOntoPlane(thisBone.getDirectionUV(), hingeRotationAxis);
								
						// If we have a global hinge which is not freely rotating then we must constrain about the reference axis
						if  !( util::approximatelyEquals(cwConstraintDegs.0 , -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) &&
							    util::approximatelyEquals(acwConstraintDegs.0,  joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) ) 
						{
							// Grab the hinge reference axis and calculate the current signed angle between it and our bone direction (about the hinge
							// rotation axis). Note: ACW rotation is positive, CW rotation is negative.
							let hingeReferenceAxis: Vector3<f32> = thisJoint.getHingeReferenceAxis();
							let signedAngleDegs: f32    = util::getSignedAngleBetweenDegs(hingeReferenceAxis, thisBoneInnerToOuterUV, hingeRotationAxis);
							
							// Constrain as necessary
				        	if signedAngleDegs > acwConstraintDegs.0
				        	{	
				        		thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, acwConstraintDegs, hingeRotationAxis).normalize();		        		
				        	}
				        	else if signedAngleDegs < cwConstraintDegs.0
				        	{	
				        		thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, cwConstraintDegs, hingeRotationAxis).normalize();			        		
				        	}
						}
						
						// Calc and set the end location of this bone
						let newEndLocation: Vector3<f32> = thisBone.getStartLocation() + ( thisBoneInnerToOuterUV * (thisBoneLength) );						
						thisBone.setEndLocation( newEndLocation );
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.mChain.len() > 1 { 
						  self.mChain[1].setStartLocation(newEndLocation); 
						}
					}
					else if self.mBaseboneConstraintType == BaseboneConstraintType::LOCAL_HINGE
					{
						let thisJoint: &Joint  =  thisBone.getJoint();
						let hingeRotationAxis: Vector3<f32>  =  self.mBaseboneRelativeConstraintUV;                   // Basebone relative constraint is our hinge rotation axis!
						let cwConstraintDegs: Rad<f32>   = -thisJoint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
						let acwConstraintDegs: Rad<f32>  =  thisJoint.getHingeAnticlockwiseConstraintDegs();
						
						// Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
						let thisBoneInnerToOuterUV: Vector3<f32> = util::projectOntoPlane(thisBone.getDirectionUV(), hingeRotationAxis);
						
						// If we have a local hinge which is not freely rotating then we must constrain about the reference axis
						if  !( util::approximatelyEquals(cwConstraintDegs.0 , -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) &&
							    util::approximatelyEquals(acwConstraintDegs.0,  joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) ) 
						{
							// Grab the hinge reference axis and calculate the current signed angle between it and our bone direction (about the hinge
							// rotation axis). Note: ACW rotation is positive, CW rotation is negative.
							let hingeReferenceAxis: Vector3<f32> = self.mBaseboneRelativeReferenceConstraintUV; 
							let signedAngleDegs: f32    = util::getSignedAngleBetweenDegs(hingeReferenceAxis, thisBoneInnerToOuterUV, hingeRotationAxis);
							
							// Constrain as necessary
				        	if signedAngleDegs > acwConstraintDegs.0
				        	{	
				        		thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, acwConstraintDegs, hingeRotationAxis).normalize();		        		
				        	}
				        	else if signedAngleDegs < cwConstraintDegs.0
				        	{	
				        		thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, cwConstraintDegs, hingeRotationAxis).normalize();			        		
				        	}
						}
						
						// Calc and set the end location of this bone
						let newEndLocation: Vector3<f32> = thisBone.getStartLocation()+ ( thisBoneInnerToOuterUV * (thisBoneLength) );						
						thisBone.setEndLocation( newEndLocation );
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.mChain.len() > 1 { 
						  self.mChain[1].setStartLocation(newEndLocation); 
						}
					}
					
				} // End of basebone constraint handling section

			} // End of basebone handling section

		} // End of backward-pass loop over all bones

		// Update our last target location
		self.mLastTargetLocation = target;
				
		// DEBUG - check the live chain length and the originally calculated chain length are the same
		/*
		if (Math.abs( this.getLiveChainLength() - mChainLength) > 0.01f)
		{
			System.out.println("Chain length off by > 0.01f");
		}
		*/
		
		// Finally, calculate and return the distance between the current effector location and the target.
		return util::distanceBetween(self.mChain[self.mChain.len()-1].getEndLocation(), target);
	}
}