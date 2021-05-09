use super::{bone::Bone, joint, util};
use cgmath::{Vector2, Rad, InnerSpace};

#[derive(Clone, Copy, PartialEq)]
pub enum BaseboneConstraintType
{ 
    NONE, 
    GLOBAL_ABSOLUTE, 
    LOCAL_RELATIVE, 
    LOCAL_ABSOLUTE 
}

pub struct Chain{
    mChain: Vec<Bone>,
    mSolveDistanceThreshold: f32,
    mMaxIterationAttempts: usize,
    mMinIterationChange: f32,
    mChainLength: f32,
    mBaseLocation: Vector2<f32>,
    mFixedBaseMode: bool,
    mBaseboneConstraintType: BaseboneConstraintType,
    mBoneConnectionPoint: crate::BoneConnectionPoint,
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
            mBoneConnectionPoint: crate::BoneConnectionPoint::END,
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

    pub fn addBone(&mut self, bone: Bone)
	{
		// Add the new bone to the end of the chain
		

		// If this is the basebone...
		if self.mChain.len()==0
		{
			
			// ...then keep a copy of the fixed start location...
			self.mBaseLocation = bone.getStartLocation();
			
			// ...and set the basebone constraint UV to be around the bone direction
			self.mBaseboneConstraintUV = bone.getDirectionUV();
			self.mChain.push(bone);
			self.updateChainLength();
		}
		else{
			self.mChain.push(bone);
			// ...and update the desired length of the chain (i.e. combined length of all bones)
			self.updateChainLength();
		}

		
	}

    pub fn addConsecutiveBone(&mut self, mut bone: Bone)
	{
		// Validate the direction unit vector - throws an IllegalArgumentException if it has a magnitude of zero
		let dir: Vector2<f32> = bone.getDirectionUV();
		util::validateDirectionUV(dir);
		
		// Validate the length of the bone - throws an IllegalArgumentException if it is not a positive value
		let len: f32 = bone.length();
		util::validateLength(len);
				
		// If we have at least one bone already in the chain...
		if  !self.mChain.is_empty() 
		{		
			// Get the end location of the last bone, which will be used as the start location of the new bone
			let prevBoneEnd: Vector2<f32> = self.mChain[self.mChain.len()-1].getEndLocation();
				
			bone.setStartLocation(prevBoneEnd);
			bone.setEndLocation( prevBoneEnd + (dir * (len)) );
			
			// Add a bone to the end of this IK chain
			self.addBone(bone);
		}
		else // Attempting to add a relative bone when there is no base bone for it to be relative to?
		{
			panic!("You cannot add the base bone to a chain using this method as it does not provide a start location.");
		}
	}	

    pub fn removeBone(&mut self, boneNumber: usize)
	{
		// If the bone number is a bone which exists...
		if boneNumber < self.mChain.len()
		{
			
			// ...then remove the bone, decrease the bone count and update the chain length.
			self.mChain.remove(boneNumber);
			self.updateChainLength();
		}
		else
		{
			panic!("Bone {} does not exist in this chain.", boneNumber);
		}
	}

    fn cloneChainVector(&self) -> Vec<Bone>
	{		
		// Create a new Vector of FabrikBone2D objects large enough to contain all the bones in this chain
		let mut clonedChain: Vec<Bone>  = Vec::new();

		// For each bone in the chain being cloned...		
        for bone in self.mChain.iter()
		{
			// ...use the FabrikBone2D copy constructor to create a new FabrikBone2D with the properties as set from the source bone
			// and add it to the cloned chain
			clonedChain.push( bone.clone() );
		}
		
		return clonedChain;
	}

    pub fn updateChainLength(&mut self)
	{
		self.mChainLength = 0.0;
		for bone in self.mChain.iter()
		{
			self.mChainLength += bone.length();
		}
	}

    pub fn updateEmbeddedTarget(&mut self, newEmbeddedTarget: Vector2<f32>)
	{
		// Using embedded target mode? Overwrite embedded target with provided location
		if self.mUseEmbeddedTarget { 
		  self.mEmbeddedTarget = newEmbeddedTarget; 
		}
		else { 
		  panic!("This chain does not have embedded targets enabled - enable with setEmbeddedTargetMode(true)."); 
		}
	}

    pub fn solveForEmbeddedTarget(&mut self) -> f32
	{
		if self.mUseEmbeddedTarget { 
		  return self.solveForTarget(self.mEmbeddedTarget); 
		}
		else { 
		  panic!("This chain does not have embedded targets enabled - enable with setEmbeddedTargetMode(true)."); 
		}
	}

    pub fn solveForTarget(&mut self, newTarget: Vector2<f32>) -> f32
	{		
		// Same target as before? Abort immediately and save ourselves some cycles
		if  util::approximatelyEquals(self.mLastTargetLocation, newTarget, 0.001) 
            && util::approximatelyEquals(self.mLastBaseLocation, self.mBaseLocation, 0.001) 
		{
			return self.mCurrentSolveDistance;
		}
		
		// Keep starting solutions and distance
		let startingDistance: f32;
		let mut startingSolution: Vec<Bone> = Vec::new();
		
		// If the base location of a chain hasn't moved then we may opt to keep the current solution if our 
		// best new solution is worse...
		if util::approximatelyEquals(self.mLastBaseLocation, self.mBaseLocation, 0.001)
		{			
			startingDistance  = util::distanceBetween(self.mChain[self.mChain.len() - 1].getEndLocation(), newTarget);
			startingSolution = self.cloneChainVector();
		}
		else // Base has changed? Then we have little choice but to recalc the solution and take that new solution.
		{
			startingDistance = f32::MAX;
		}
		
		// Not the same target? Then we must solve the chain for the new target.
		// We'll start by creating a list of bones to store our best solution
		let mut bestSolution: Vec<Bone> = Vec::new();
		
		// We'll keep track of our best solve distance, starting it at a huge value which will be beaten on first attempt			
		let mut bestSolveDistance: f32    = f32::MAX;
		let mut lastPassSolveDistance: f32 = f32::MAX;
		
		// Allow up to our iteration limit attempts at solving the chain
		let mut solveDistance: f32;
		//for (int loop = 0; loop < mMaxIterationAttempts; ++loop)
        for _loope in 0..self.mMaxIterationAttempts
		{	
			// Solve the chain for this target
			solveDistance = self.solveIK(newTarget);
			
			// Did we solve it for distance? If so, update our best distance and best solution, and also
			// update our last pass solve distance. Note: We will ALWAYS beat our last solve distance on the first run. 
			if solveDistance < bestSolveDistance
			{	
				bestSolveDistance = solveDistance;
				bestSolution = self.cloneChainVector();
				
				// Did we solve for distance? Great! Break out of the loop.
				if solveDistance <= self.mSolveDistanceThreshold { 
				  break; 
				}
			}
			else // Did not solve to our satisfaction? Okay...
			{
				// Did we grind to a halt? If so then it's time to break out of the loop.
				if (solveDistance - lastPassSolveDistance).abs() < self.mMinIterationChange { 
				  break; 
				}
			}
			
			// Update the last pass solve distance
			lastPassSolveDistance = solveDistance;
		}
		
		// Did we get a solution that's better than the starting solution's to the new target location?
		if bestSolveDistance < startingDistance
		{
			// If so, set the newly found solve distance and solution as the best found.
			self.mCurrentSolveDistance = bestSolveDistance;
			self.mChain                = bestSolution;
		}
		else // Did we make things worse? Then we keep our starting distance and solution!
		{
			self.mCurrentSolveDistance = startingDistance;
			self.mChain                = startingSolution; 
		}				
		
		// Update our last base and target locations so we know whether we need to solve for this start/end configuration next time
		self.mLastBaseLocation = self.mBaseLocation;
		self.mLastTargetLocation = newTarget;
		
		// Finally, return the solve distance we ended up with
		return self.mCurrentSolveDistance;
	}

    
}

//setters
impl Chain{
    pub fn setBaseboneConstraintType(&mut self, t: BaseboneConstraintType) 
    { 
        self.mBaseboneConstraintType = t; 
    }

    pub fn setBaseboneConstraintUV(&mut self, constraintUV: Vector2<f32>)
	{
		// Sanity checking
		util::validateDirectionUV(constraintUV);
		
		// All good? Then normalise the constraint direction and set it
		self.mBaseboneConstraintUV =  constraintUV.normalize();
	}

    pub fn setBaseLocation(&mut self, baseLocation: Vector2<f32>) 
    { 
        self.mBaseLocation = baseLocation; 
    }

    pub fn setBoneConnectionPoint(&mut self, boneConnectionPoint: crate::BoneConnectionPoint) 
    { 
        self.mBoneConnectionPoint = boneConnectionPoint; 
    }

    pub fn setChain(&mut self, chain: Vec<Bone>)
	{
		// Assign this chain to be a reference to the chain provided as an argument to this method
		self.mChain = chain;
	}

    fn setConnectedBoneNumber(&mut self, boneNumber: usize)	
    {
        self.mConnectedBoneNumber = Some(boneNumber); 
    }

    fn setConnectedChainNumber(&mut self, chainNumber: usize) 
    {	
        self.mConnectedChainNumber = Some(chainNumber); 
    }

    pub fn setFixedBaseMode(&mut self, value: bool)
	{	
		if !value && self.mConnectedChainNumber != None
		{
			panic!("This chain is connected to another chain so must remain in fixed base mode.");
		}
		
		// We cannot have a freely moving base location AND constrain the base bone - so if we have both
		// enabled then we disable the base bone angle constraint here.
		if self.mBaseboneConstraintType == BaseboneConstraintType::GLOBAL_ABSOLUTE && !value
		{
			panic!("Cannot set a non-fixed base mode when the chain's constraint type is BaseBoneConstraintType2D.GLOBAL_ABSOLUTE.");
		}
		
		// Above conditions met? Set the fixedBaseMode
		self.mFixedBaseMode = value;
	}

    pub fn setMaxIterationAttempts(&mut self,  maxIterations: usize)
	{
		// Ensure we have a valid maximum number of iteration attempts
		if maxIterations < 1
		{
			panic!("The maximum number of attempts to solve this IK chain must be at least 1.");
		}
		
		// All good? Set the new maximum iteration attempts property
		self.mMaxIterationAttempts = maxIterations;
	}

    pub fn setMinIterationChange(&mut self, minIterationChange: f32)
	{
		// Ensure we have a valid maximum number of iteration attempts
		if minIterationChange < 0.0
		{
			panic!("The minimum iteration change value must be more than or equal to zero.");
		}
		
		// All good? Set the new minimum iteration change distance
		self.mMinIterationChange = minIterationChange;
	}

    pub fn setSolveDistanceThreshold(&mut self,  solveDistance: f32)
	{
		// Ensure we have a valid solve distance
		if solveDistance < 0.0
		{
			panic!("The solve distance threshold must be greater than or equal to zero.");
		}
		
		// All good? Set the new solve distance threshold
		self.mSolveDistanceThreshold = solveDistance;
	}	

    pub fn setEmbeddedTargetMode(&mut self, value: bool) 
    { 
        self.mUseEmbeddedTarget = value; 
    }

    pub fn setBaseboneRelativeConstraintUV(&mut self, constraintUV: Vector2<f32>) 
    { 
        self.mBaseboneRelativeConstraintUV = constraintUV; 
    }

}

//getters
impl Chain{

    pub fn getSolveDistanceThreshold(&self)-> f32
    {
		return self.mSolveDistanceThreshold;
	}

    pub fn getMinIterationChange(&self) -> f32
    {
		return self.mMinIterationChange;
	}

    pub fn getMaxIterationAttempts(&self) -> usize
    {
		return self.mMaxIterationAttempts;
	}

    pub fn getBaseboneConstraintType(&self) -> BaseboneConstraintType
    { 
        return self.mBaseboneConstraintType; 
    }

    pub fn getBaseboneConstraintUV(&self) -> Vector2<f32>
    { 
        return self.mBaseboneConstraintUV; 
    }

    pub fn getBaseLocation(&self) -> Vector2<f32>
	{
		if !self.mChain.is_empty()
		{
			return self.mChain[0].getStartLocation();
		}
		else
		{
			panic!("Cannot get base location as there are zero bones in the chain.");
		}
	}
    
    pub fn getBone(&self, boneNumber: usize) -> &Bone
	{
		return &self.mChain[boneNumber];
	}

    pub fn getBoneConnectionPoint(&self) -> crate::BoneConnectionPoint
    { 
        return self.mBoneConnectionPoint; 
    }

    pub fn getChain(&self) -> Vec<Bone>
    { 
        //ToDo, might want to fix this.
        return self.mChain.clone(); 
    }

    pub fn getChainLength(&self) -> f32
    { 
        return self.mChainLength; 
    }

    pub fn getConnectedBoneNumber(&self) -> usize
    { 
        return self.mConnectedBoneNumber.expect("There is no connected Bone."); 
    }

    pub fn getConnectedChainNumber(&self) -> Option<usize>
    { 
        return self.mConnectedChainNumber; 
    }

    pub fn getEffectorLocation(&self) -> Vector2<f32>
	{
		if !self.mChain.is_empty()
		{
			return self.mChain[self.mChain.len()-1].getEndLocation();
		}
		else
		{
			panic!("Cannot get effector location as there are zero bones in the chain.");
		}
	}

    pub fn getEmbeddedTargetMode(&self) -> bool
    { 
        return self.mUseEmbeddedTarget; 
    }

    pub fn getEmbeddedTarget(&self) -> Vector2<f32>
    { 
        return self.mEmbeddedTarget; 
    }

    pub fn getLastTargetLocation(&self) -> Vector2<f32>
    { 
        return self.mLastTargetLocation;
    }

    pub fn getNumBones(&self) -> usize
    { 
         return self.mChain.len(); 
    }

    pub fn getBaseboneRelativeConstraintUV(&self) -> Vector2<f32>
    { 
        return self.mBaseboneRelativeConstraintUV; 
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
			//let mut thisBone: &Bone = &self.mChain[loope];
			
			// Get the length of the bone we're working on
			let  boneLength: f32 = self.mChain[loope].length();			
			
			// If we are not working on the end effector bone
			if loope != self.mChain.len() - 1
			{
				let outerBone: &Bone = &self.mChain[loope+1];
				
				// Get the outer-to-inner unit vector of this bone and the bone further out
				let outerBoneOuterToInnerUV: Vector2<f32> = -outerBone.getDirectionUV(); //negated.
				let thisBoneOuterToInnerUV: Vector2<f32> = -self.mChain[loope].getDirectionUV(); //negated
				
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
					constrainedUV = util::getConstrainedUV(thisBoneOuterToInnerUV, -self.mChain[loope].getGlobalConstraintUV(), clockwiseConstraintDegs, antiClockwiseConstraintDegs);
				}
				
				
				// At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
				// so we can set the new inner joint location to be the end joint location of this bone plus the
				// outer-to-inner direction unit vector multiplied by the length of the bone.
				let newStartLocation: Vector2<f32> = self.mChain[loope].getEndLocation() + ( constrainedUV *(boneLength) );

				// Set the new start joint location for this bone
				self.mChain[loope].setStartLocation(newStartLocation);

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
				self.mChain[loope].setEndLocation(target);
	        
				// Get the UV between the target / end-location (which are now the same) and the start location of this bone
				let thisBoneOuterToInnerUV: Vector2<f32> = -self.mChain[loope].getDirectionUV();
	        
				let constrainedUV: Vector2<f32>;
				if loope > 0 {
					// The end-effector bone is NOT the basebone as well
					// Get the outer-to-inner unit vector of the bone further in
					let innerBoneOuterToInnerUV: Vector2<f32> = -self.mChain[loope-1].getDirectionUV();
	          
					// Constrain the angle between the this bone and the inner bone
					// Note: On the forward pass we constrain to the limits imposed by the first joint of the inner bone.
					let clockwiseConstraintDegs: Rad<f32>     = self.mChain[loope].getJoint().getClockwiseConstraintDegs();
					let antiClockwiseConstraintDegs: Rad<f32> = self.mChain[loope].getJoint().getAnticlockwiseConstraintDegs();
	          
					// If this bone is locally constrained...
					if self.mChain[loope].getJoint().getConstraintCoordinateSystem() == joint::ConstraintCoordinateSystem::LOCAL
					{
						// Params: directionUV, baselineUV, clockwise, anticlockwise
						constrainedUV = util::getConstrainedUV( thisBoneOuterToInnerUV, innerBoneOuterToInnerUV, clockwiseConstraintDegs, antiClockwiseConstraintDegs);
					}
					else // End effector bone is globally constrained
					{
						constrainedUV = util::getConstrainedUV( thisBoneOuterToInnerUV, -self.mChain[loope].getGlobalConstraintUV(), clockwiseConstraintDegs, antiClockwiseConstraintDegs);			
					}					
	          
				}
				else // There is only one bone in the chain, and the bone is both the basebone and the end-effector bone.
				{
					// Don't constraint (nothing to constraint against) if constraint is in local coordinate system
					if self.mChain[loope].getJointConstraintCoordinateSystem() == joint::ConstraintCoordinateSystem::LOCAL
					{
						constrainedUV = thisBoneOuterToInnerUV;
					}
					else // Can constrain if constraining against global coordinate system
					{	
						constrainedUV = util::getConstrainedUV(thisBoneOuterToInnerUV, -self.mChain[loope].getGlobalConstraintUV(), self.mChain[loope].getClockwiseConstraintDegs(), self.mChain[loope].getAnticlockwiseConstraintDegs());
					}
				}
	                
				// Calculate the new start joint location as the end joint location plus the outer-to-inner direction UV
				// multiplied by the length of the bone.
				let newStartLocation: Vector2<f32> = self.mChain[loope].getEndLocation() + ( constrainedUV * (boneLength) );
	        
				// Set the new start joint location for this bone to be new start location...
				self.mChain[loope].setStartLocation(newStartLocation);
	
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

			//let mut thisBone: Bone = self.mChain[loope];
			
			// If we are not working on the base bone
			if loope != 0
			{
				//let previousBone: Bone = self.mChain[loope-1];
				
				// Get the inner-to-outer direction of this bone as well as the previous bone to use as a baseline
				let thisBoneInnerToOuterUV: Vector2<f32> = self.mChain[loope].getDirectionUV();
				let prevBoneInnerToOuterUV: Vector2<f32> = self.mChain[loope-1].getDirectionUV();
				
				// Constrain the angle between this bone and the inner bone.
				// Note: On the backward pass we constrain to the limits imposed by the first joint of this bone.
				let clockwiseConstraintDegs: Rad<f32>     = self.mChain[loope].getJoint().getClockwiseConstraintDegs();
				let antiClockwiseConstraintDegs: Rad<f32> = self.mChain[loope].getJoint().getAnticlockwiseConstraintDegs();
				
				let constrainedUV: Vector2<f32>;
				if self.mChain[loope].getJointConstraintCoordinateSystem() == joint::ConstraintCoordinateSystem::LOCAL
				{
					constrainedUV = util::getConstrainedUV(thisBoneInnerToOuterUV, prevBoneInnerToOuterUV, clockwiseConstraintDegs, antiClockwiseConstraintDegs);
				}
				else // Bone is constrained in global coordinate system
				{
					constrainedUV = util::getConstrainedUV(thisBoneInnerToOuterUV, self.mChain[loope].getGlobalConstraintUV(), clockwiseConstraintDegs, antiClockwiseConstraintDegs);
				}

				// At this stage we have an inner-to-outer unit vector for this bone which is within our constraints,
				// so we can set the new end location to be the start location of this bone plus the constrained
				// inner-to-outer direction unit vector multiplied by the length of this bone.
				let newEndLocation: Vector2<f32> = self.mChain[loope].getStartLocation() + ( constrainedUV * (boneLength) );

				// Set the new end joint location for this bone
				self.mChain[loope].setEndLocation(newEndLocation);

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
					let thisBoneInnerToOuterUV: Vector2<f32> = self.mChain[loope].getDirectionUV();
	
					// Calculate the new end location as the start location plus the direction times the length of the bone
					let newEndLocation: Vector2<f32> = self.mChain[loope].getStartLocation() + ( thisBoneInnerToOuterUV * (boneLength) );
	
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
					let thisBoneInnerToOuterUV: Vector2<f32> = self.mChain[loope].getDirectionUV();

					// Get the constrained direction unit vector between the base bone and the base bone constraint unit vector
					// Note: On the backward pass we constrain to the limits imposed by the first joint of this bone.
					let clockwiseConstraintDegs: Rad<f32>     = self.mChain[loope].getJoint().getClockwiseConstraintDegs();
					let antiClockwiseConstraintDegs: Rad<f32> = self.mChain[loope].getJoint().getAnticlockwiseConstraintDegs();
					
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
