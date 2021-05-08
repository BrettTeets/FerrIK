use super::{bone::Bone, joint, util};
use cgmath::{Vector2, Rad};

#[derive(Clone, PartialEq)]
pub enum BaseboneConstraintType
{ 
    NONE, 
    GLOBAL_ABSOLUTE, 
    LOCAL_RELATIVE, 
    LOCAL_ABSOLUTE 
}

pub enum BoneConnectionPoint { START, END }

pub struct Chain{
    mChain: Vec<Bone>,
    mSolveDistanceThreshold: f32,
    mMaxIterationAttempts: usize,
    mMinIterationChange: f32,
    mChainLength: f32,
    mBaseLocation: Vector2<f32>,
    mFixedBaseMode: bool,
    mBaseboneConstraintType: BaseboneConstraintType,
    mBoneConnectionPoint: BoneConnectionPoint,
    mBaseboneConstraintUV: Vector2<f32>,
    mBaseboneRelativeConstraintUV: Vector2<f32>,
    mLastTargetLocation: Vector2<f32>,
    mLastBaseLocation: Vector2<f32>,
    mEmbeddedTarget: Vector2<f32>,
    mUseEmbeddedTarget: bool,
    mCurrentSolveDistance: f32,
    mConnectedChainNumber: Option<usize>,
    mConnectedBoneNumber: Option<usize>,
}

impl Chain{
    pub fn new() -> Self {
        Self {
            mChain: Vec::new(),
            mSolveDistanceThreshold: 1.0,
            mMaxIterationAttempts: 15,
            mMinIterationChange: 0.01,
            mChainLength: 0.0,
            mBaseLocation: Vector2::new(0.0, 0.0),
            mFixedBaseMode: true,
            mBaseboneConstraintType: BaseboneConstraintType::NONE,
            mBoneConnectionPoint: BoneConnectionPoint::END,
            mBaseboneConstraintUV: Vector2::new(0.0, 0.0),
            mBaseboneRelativeConstraintUV: Vector2::new(0.0, 0.0),
            mLastTargetLocation: Vector2::new(f32::MAX, f32::MAX),
            mLastBaseLocation: Vector2::new(f32::MAX, f32::MAX),
            mEmbeddedTarget: Vector2::new(0.0, 0.0),
            mUseEmbeddedTarget: false,
            mCurrentSolveDistance: f32::MAX,
            mConnectedChainNumber: None,
            mConnectedBoneNumber: None,
        }
    }
}

impl Chain{
    pub fn solveIK(&mut self, target: Vector2<f32>) -> f32
	{	
		// ---------- Step 1 of 2 - Forward pass from end-effector to base -----------

		// loope over all bones in the chain, from the end effector (numBones-1) back to the base bone (0)		
        for loope in (0..self.mChain.len()).rev()
		{
			// Get this bone
			let mut thisBone: Bone = self.mChain[loope];
			
			// Get the length of the bone we're working on
			let  boneLength: f32 = thisBone.length();			
			
			// If we are not working on the end effector bone
			if loope != self.mChain.len() - 1
			{
				let outerBone: Bone = self.mChain[loope+1];
				
				// Get the outer-to-inner unit vector of this bone and the bone further out
				let outerBoneOuterToInnerUV: Vector2<f32> = -outerBone.getDirectionUV(); //negated.
				let thisBoneOuterToInnerUV: Vector2<f32> = -thisBone.getDirectionUV(); //negated
				
				// Constrain the angle between the outer bone and this bone.
				// Note: On the forward pass we constrain to the limits imposed by joint of the outer bone.
				let clockwiseConstraintDegs: Rad<f32>     = outerBone.getJoint().getClockwiseConstraintDegs();
				let antiClockwiseConstraintDegs: Rad<f32> = outerBone.getJoint().getAnticlockwiseConstraintDegs();
				
				
				let constrainedUV: Vector2<f32>;
				if self.mChain[loope].getJointConstraintCoordinateSystem() == joint::ConstraintCoordinateSystem::LOCAL
				{
					constrainedUV = util::getConstrainedUV(thisBoneOuterToInnerUV, outerBoneOuterToInnerUV, clockwiseConstraintDegs, antiClockwiseConstraintDegs);
				}
				else // Constraint is in global coordinate system
				{
					constrainedUV = util::getConstrainedUV(thisBoneOuterToInnerUV, -thisBone.getGlobalConstraintUV(), clockwiseConstraintDegs, antiClockwiseConstraintDegs);
				}
				
				
				// At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
				// so we can set the new inner joint location to be the end joint location of this bone plus the
				// outer-to-inner direction unit vector multiplied by the length of the bone.
				let newStartLocation: Vector2<f32> = thisBone.getEndLocation() + ( constrainedUV *(boneLength) );

				// Set the new start joint location for this bone
				thisBone.setStartLocation(newStartLocation);

				// If we are not working on the base bone, then we set the end joint location of
				// the previous bone in the chain (i.e. the bone closer to the base) to be the new
				// start joint location of this bone also.
				if loope > 0
				{
					self.mChain[loope-1].setEndLocation(newStartLocation);
				}
			}
			else // If we are working on the end effector bone ...
			{
				// Snap the end effector's end location to the target
				thisBone.setEndLocation(target);
	        
				// Get the UV between the target / end-location (which are now the same) and the start location of this bone
				let thisBoneOuterToInnerUV: Vector2<f32> = -thisBone.getDirectionUV();
	        
				let constrainedUV: Vector2<f32>;
				if loope > 0 {
					// The end-effector bone is NOT the basebone as well
					// Get the outer-to-inner unit vector of the bone further in
					let innerBoneOuterToInnerUV: Vector2<f32> = -self.mChain[loope-1].getDirectionUV();
	          
					// Constrain the angle between the this bone and the inner bone
					// Note: On the forward pass we constrain to the limits imposed by the first joint of the inner bone.
					let clockwiseConstraintDegs: Rad<f32>     = thisBone.getJoint().getClockwiseConstraintDegs();
					let antiClockwiseConstraintDegs: Rad<f32> = thisBone.getJoint().getAnticlockwiseConstraintDegs();
	          
					// If this bone is locally constrained...
					if thisBone.getJoint().getConstraintCoordinateSystem() == joint::ConstraintCoordinateSystem::LOCAL
					{
						// Params: directionUV, baselineUV, clockwise, anticlockwise
						constrainedUV = util::getConstrainedUV( thisBoneOuterToInnerUV, innerBoneOuterToInnerUV, clockwiseConstraintDegs, antiClockwiseConstraintDegs);
					}
					else // End effector bone is globally constrained
					{
						constrainedUV = util::getConstrainedUV( thisBoneOuterToInnerUV, -thisBone.getGlobalConstraintUV(), clockwiseConstraintDegs, antiClockwiseConstraintDegs);			
					}					
	          
				}
				else // There is only one bone in the chain, and the bone is both the basebone and the end-effector bone.
				{
					// Don't constraint (nothing to constraint against) if constraint is in local coordinate system
					if thisBone.getJointConstraintCoordinateSystem() == joint::ConstraintCoordinateSystem::LOCAL
					{
						constrainedUV = thisBoneOuterToInnerUV;
					}
					else // Can constrain if constraining against global coordinate system
					{	
						constrainedUV = util::getConstrainedUV(thisBoneOuterToInnerUV, -thisBone.getGlobalConstraintUV(), thisBone.getClockwiseConstraintDegs(), thisBone.getAnticlockwiseConstraintDegs());
					}
				}
	                
				// Calculate the new start joint location as the end joint location plus the outer-to-inner direction UV
				// multiplied by the length of the bone.
				let newStartLocation: Vector2<f32> = thisBone.getEndLocation() + ( constrainedUV * (boneLength) );
	        
				// Set the new start joint location for this bone to be new start location...
				thisBone.setStartLocation(newStartLocation);
	
				// ...and set the end joint location of the bone further in to also be at the new start location.
				if loope > 0 { 
					self.mChain[loope-1].setEndLocation(newStartLocation);
				}
			}

		} // End of forward-pass loope over all bones

		// ---------- Step 2 of 2 - Backward pass from base to end effector -----------

		//for (int loope = 0; loope < this.mChain.size(); ++loope)
        for loope in 0..self.mChain.len()
		{
			// Get the length of the bone we're working on
			let boneLength: f32 = self.mChain[loope].length();

			let mut thisBone: Bone = self.mChain[loope];
			
			// If we are not working on the base bone
			if loope != 0
			{
				let previousBone: Bone = self.mChain[loope-1];
				
				// Get the inner-to-outer direction of this bone as well as the previous bone to use as a baseline
				let thisBoneInnerToOuterUV: Vector2<f32> = thisBone.getDirectionUV();
				let prevBoneInnerToOuterUV: Vector2<f32> = previousBone.getDirectionUV();
				
				// Constrain the angle between this bone and the inner bone.
				// Note: On the backward pass we constrain to the limits imposed by the first joint of this bone.
				let clockwiseConstraintDegs: Rad<f32>     = thisBone.getJoint().getClockwiseConstraintDegs();
				let antiClockwiseConstraintDegs: Rad<f32> = thisBone.getJoint().getAnticlockwiseConstraintDegs();
				
				let constrainedUV: Vector2<f32>;
				if thisBone.getJointConstraintCoordinateSystem() == joint::ConstraintCoordinateSystem::LOCAL
				{
					constrainedUV = util::getConstrainedUV(thisBoneInnerToOuterUV, prevBoneInnerToOuterUV, clockwiseConstraintDegs, antiClockwiseConstraintDegs);
				}
				else // Bone is constrained in global coordinate system
				{
					constrainedUV = util::getConstrainedUV(thisBoneInnerToOuterUV, thisBone.getGlobalConstraintUV(), clockwiseConstraintDegs, antiClockwiseConstraintDegs);
				}

				// At this stage we have an inner-to-outer unit vector for this bone which is within our constraints,
				// so we can set the new end location to be the start location of this bone plus the constrained
				// inner-to-outer direction unit vector multiplied by the length of this bone.
				let newEndLocation: Vector2<f32> = thisBone.getStartLocation() + ( constrainedUV * (boneLength) );

				// Set the new end joint location for this bone
				thisBone.setEndLocation(newEndLocation);

				// If we are not working on the end bone, then we set the start joint location of
				// the next bone in the chain (i.e. the bone closer to the end effector) to be the
				// new end joint location of this bone also.
				if loope < self.mChain.len()-1
				{
					self.mChain[loope+1].setStartLocation(newEndLocation);
				}
			}
			else // If we ARE working on the base bone...
			{	
				// If the base location is fixed then snap the start location of the base bone back to the fixed base
				if self.mFixedBaseMode
				{
					self.mChain[0].setStartLocation(self.mBaseLocation);
				}
				else // If the base location is not fixed...
				{
					// ...then set the new base bone start location to be its the end location minus the
					// bone direction multiplied by the length of the bone (i.e. projected backwards).
					//float boneZeroLength = mChain.get(0).length();
					let boneZeroUV: Vector2<f32> = self.mChain[0].getDirectionUV();
					let boneZeroEndLocation: Vector2<f32> = self.mChain[0].getEndLocation();
					let newBoneZeroStartLocation: Vector2<f32> = boneZeroEndLocation - ( boneZeroUV * (boneLength) );
					self.mChain[0].setStartLocation(newBoneZeroStartLocation);
				}
				
				// If the base bone is unconstrained then process it as usual...
				if self.mBaseboneConstraintType == BaseboneConstraintType::NONE
				{
					// Get the inner to outer direction of this bone
					let thisBoneInnerToOuterUV: Vector2<f32> = thisBone.getDirectionUV();
	
					// Calculate the new end location as the start location plus the direction times the length of the bone
					let newEndLocation: Vector2<f32> = thisBone.getStartLocation() + ( thisBoneInnerToOuterUV * (boneLength) );
	
					// Set the new end joint location
					self.mChain[0].setEndLocation(newEndLocation);
	
					// Also, set the start location of the next bone to be the end location of this bone
					if self.mChain.len() > 1 { 
					  self.mChain[1].setStartLocation(newEndLocation);
					}
				}
				else // ...otherwise we must constrain it to the basebone constraint unit vector
				{	
					// Note: The mBaseBoneConstraintUV is either fixed, or it may be dynamically updated from
					// a FabrikStructure2D if this chain is connected to another chain.
					
					// Get the inner-to-outer direction of this bone
					let thisBoneInnerToOuterUV: Vector2<f32> = thisBone.getDirectionUV();

					// Get the constrained direction unit vector between the base bone and the base bone constraint unit vector
					// Note: On the backward pass we constrain to the limits imposed by the first joint of this bone.
					let clockwiseConstraintDegs: Rad<f32>     = thisBone.getJoint().getClockwiseConstraintDegs();
					let antiClockwiseConstraintDegs: Rad<f32> = thisBone.getJoint().getAnticlockwiseConstraintDegs();
					
					// LOCAL_ABSOLUTE? (i.e. local-space directional constraint) - then we must constraint about the relative basebone constraint UV...
					let constrainedUV: Vector2<f32>;
					if self.mBaseboneConstraintType == BaseboneConstraintType::LOCAL_ABSOLUTE
					{
						constrainedUV = util::getConstrainedUV(thisBoneInnerToOuterUV, self.mBaseboneRelativeConstraintUV, clockwiseConstraintDegs, antiClockwiseConstraintDegs);
						
//						System.out.println("----- In LOCAL_ABSOLUTE --------");
//						System.out.println("This bone UV = " + thisBoneInnerToOuterUV);
//						System.out.println("Constraint UV = " + mBaseboneConstraintUV);
//						System.out.println("Relative constraint UV = " + mBaseboneRelativeConstraintUV);
					}
					else // ...otherwise we're free to use the standard basebone constraint UV.
					{
						constrainedUV = util::getConstrainedUV(thisBoneInnerToOuterUV, self.mBaseboneConstraintUV, clockwiseConstraintDegs, antiClockwiseConstraintDegs);
					}
					
					// At this stage we have an inner-to-outer unit vector for this bone which is within our constraints,
					// so we can set the new end location to be the start location of this bone plus the constrained
					// inner-to-outer direction unit vector multiplied by the length of the bone.
					let newEndLocation: Vector2<f32> = self.mChain[loope].getStartLocation() + ( constrainedUV * (boneLength) );

					// Set the new end joint location for this bone
					self.mChain[loope].setEndLocation(newEndLocation);

					// If we are not working on the end bone, then we set the start joint location of
					// the next bone in the chain (i.e. the bone closer to the end effector) to be the
					// new end joint location of this bone.
					if loope < self.mChain.len()-1
					{
						self.mChain[loope+1].setStartLocation(newEndLocation);
					}
					
				} // End of basebone constraint enforcement section			

			} // End of base bone handling section

		} // End of backward-pass loope over all bones

		// Update our last target location
		self.mLastTargetLocation = target;
		
		// Finally, get the current effector location...
		let currentEffectorLocation: Vector2<f32> = self.mChain[self.mChain.len()-1].getEndLocation();
				
		// ...and calculate and return the distance between the current effector location and the target.
		return util::distanceBetween(currentEffectorLocation, target);
	}


}
