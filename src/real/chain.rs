use super::{bone::Bone, joint::Joint, joint::JointType, structure::Structure, joint, util};
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

    pub fn addBone(&mut self, bone: Bone)
	{
		// Add the new bone to the end of the ArrayList of bones
		// If this is the basebone...
		if self.mChain.len() == 1
		{
			// ...then keep a copy of the fixed start location...
			self.mFixedBaseLocation =  bone.getStartLocation();
			
			// ...and set the basebone constraint UV to be around the initial bone direction
			self.mBaseboneConstraintUV = bone.getDirectionUV();

            self.mChain.push(bone);
            self.updateChainLength();
		}
        else{
            self.mChain.push(bone);
            self.updateChainLength();
        }
		
		// Increment the number of bones in the chain and update the chain length
		
	}

    pub fn addConsecutiveBone(&mut self, mut bone: Bone)
	{
		// Validate the direction unit vector - throws an IllegalArgumentException if it has a magnitude of zero
		let dir: Vector3<f32> = bone.getDirectionUV();
		util::validateDirectionUV(dir);
		
		// Validate the length of the bone - throws an IllegalArgumentException if it is not a positive value
		let len: f32 = bone.liveLength();
		util::validateLength(len);
			
		// If we have at least one bone already in the chain...
		if !self.mChain.is_empty()
		{		
			// Get the end location of the last bone, which will be used as the start location of the new bone
			let prevBoneEnd: Vector3<f32> = self.mChain[self.mChain.len()-1].getEndLocation();
						
			bone.setStartLocation(prevBoneEnd);
			bone.setEndLocation( prevBoneEnd + ( dir * (len)) );
					
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
			panic!("Bone {} does not exist to be removed from the chain. Bones are zero indexed.", boneNumber);
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

    pub fn solveForTarget(&mut self, newTarget: Vector3<f32>) -> f32
	{	
		// If we have both the same target and base location as the last run then do not solve
		if  util::v_approximatelyEquals(self.mLastTargetLocation, newTarget, 0.001) &&
			 util::v_approximatelyEquals(self.mLastBaseLocation, self.getBaseLocation(), 0.001) 
		{
			return self.mCurrentSolveDistance;
		}
		
		/***
		 * NOTE: We must allow the best solution of THIS run to be used for a new target or base location - we cannot
		 * just use the last solution (even if it's better) - because that solution was for a different target / base
		 * location combination and NOT for the current setup.
		 */
						
		// Declare a list of bones to use to store our best solution
		let mut bestSolution: Vec<Bone> = Vec::new();
		
		// We start with a best solve distance that can be easily beaten
		let mut bestSolveDistance = f32::MAX;
		
		// We'll also keep track of the solve distance from the last pass
		let mut lastPassSolveDistance = f32::MAX;
		
		// Allow up to our iteration limit attempts at solving the chain
		let mut solveDistance: f32;
		//for (int loop = 0; loop < mMaxIterationAttempts; ++loop)
        for i in 0..self.mMaxIterationAttempts
		{	
			// Solve the chain for this target
			solveDistance = self.solveIK(newTarget);
			
			// Did we solve it for distance? If so, update our best distance and best solution, and also
			// update our last pass solve distance. Note: We will ALWAYS beat our last solve distance on the first run. 
			if solveDistance < bestSolveDistance
			{	
				bestSolveDistance = solveDistance;
				bestSolution = self.cloneIkChain();
				
				// If we are happy that this solution meets our distance requirements then we can exit the loop now
				if solveDistance <= self.mSolveDistanceThreshold
				{				
					break;
				}
			}
			else // Did not solve to our satisfaction? Okay...
			{
				// Did we grind to a halt? If so break out of loop to set the best distance and solution that we have
				if  (solveDistance - lastPassSolveDistance).abs() < self.mMinIterationChange 
				{
					//System.out.println("Ground to halt on iteration: " + loop);
					break;
				}
			}
			
			// Update the last pass solve distance
			lastPassSolveDistance = solveDistance;
			
		} // End of loop
		
		// Update our solve distance and chain configuration to the best solution found
		self.mCurrentSolveDistance = bestSolveDistance;
		self.mChain = bestSolution;
		
		// Update our base and target locations
		self.mLastBaseLocation = self.getBaseLocation();
		self.mLastTargetLocation = newTarget;
		
		return self.mCurrentSolveDistance;
	}

    pub fn updateChainLength(&mut self)
	{
		// We start adding up the length of the bones from an initial length of zero
		self.mChainLength = 0.0;

		// Loop over all the bones in the chain, adding the length of each bone to the mChainLength property
		for bone in self.mChain.iter()
		{
			self.mChainLength += bone.length();
		}
	}

    pub fn updateEmbeddedTarget(&mut self, newEmbeddedTarget: Vector3<f32>)
	{
		// Using embedded target mode? Overwrite embedded target with provided location
		if self.mUseEmbeddedTarget { 
		  self.mEmbeddedTarget = newEmbeddedTarget; 
		}
		else { 
		  panic!("This chain does not have embedded targets enabled - enable with setEmbeddedTargetMode(true)."); 
		}
	}

    fn cloneIkChain(&self) -> Vec<Bone>
	{
		// Create a new Vector of FabrikBone3D objects of that size
		let mut clonedChain: Vec<Bone> = Vec::new();

		// For each bone in the chain being cloned...		
		for bone in self.mChain.iter()
		{
			// Use the copy constructor to create a new FabrikBone3D with the values set from the source FabrikBone3D.
			// and add it to the cloned chain.
			clonedChain.push(bone.clone() );
		}
		
		return clonedChain;
	}

    pub fn connectToStructure(&mut self, structure: Structure, chainNumber:usize,  boneNumber: usize)
	{
		// Sanity check chain exists
        //TODO, Why not ask the strucutre to validate this rather than validating it yourslef here?
		let numChains: usize = structure.getNumChains();
		if chainNumber > numChains { 
		  panic!("Structure does not contain a chain {} - it has {} chains.", chainNumber, numChains); 
		}
		
		// Sanity check bone exists
		let numBones: usize = structure.getChain(chainNumber).getNumBones();
		if boneNumber > numBones { 
		  panic!("Chain does not contain a bone {} - it has {} bones.", boneNumber,  numBones); 
		}
		
		// All good? Set the connection details
		self.mConnectedChainNumber = Some(chainNumber);
		self.mConnectedBoneNumber  = Some(boneNumber);		
	}
}

//setters
impl Chain{
    pub fn setBaseboneRelativeConstraintUV(&mut self, constraintUV: Vector3<f32>) 
    { self.mBaseboneRelativeConstraintUV = constraintUV; }

    pub fn  setBaseboneRelativeReferenceConstraintUV(&mut self, constraintUV: Vector3<f32>) 
    { self.mBaseboneRelativeReferenceConstraintUV = constraintUV; }
	
	pub fn setEmbeddedTargetMode(&mut self, value: bool) 
    { self.mUseEmbeddedTarget = value; }

    pub fn setRotorBaseboneConstraint(&mut self,  rotorType: BaseboneConstraintType,
          constraintAxis: Vector3<f32>, mut angleDegs: Rad<f32>)
	{
		// Sanity checking
		if self.mChain.is_empty()	{ 
		  panic!("Chain must contain a basebone before we can specify the basebone constraint type."); 
		}		
		if constraintAxis.magnitude() <= 0.0 { 
		  panic!("Constraint axis cannot be zero."); 
		}
		if angleDegs < Rad(0.0)  { 
		  angleDegs = Rad(0.0); 
		}
		if angleDegs > Rad(3.1415) { 
		  angleDegs = Rad(3.1415);
		}		
		if  !(rotorType == BaseboneConstraintType::GLOBAL_ROTOR ||
             rotorType == BaseboneConstraintType::LOCAL_ROTOR) 
		{
			panic!("The only valid rotor types for this method are GLOBAL_ROTOR and LOCAL_ROTOR.");
		}
				
		// Set the constraint type, axis and angle
		self.mBaseboneConstraintType = rotorType;
		self.mBaseboneConstraintUV   = constraintAxis.normalize();
		self.mBaseboneRelativeConstraintUV = self.mBaseboneConstraintUV;
		self.getBone(0).getJoint().setAsBallJoint(angleDegs);
	}
    
    pub fn setHingeBaseboneConstraint(&mut self,  hingeType: BaseboneConstraintType,
        hingeRotationAxis: Vector3<f32>, cwConstraintDegs: Rad<f32>, acwConstraintDegs: Rad<f32>, hingeReferenceAxis: Vector3<f32>)
	{
		// Sanity checking
		if self.mChain.is_empty()	{ 
		  panic!("Chain must contain a basebone before we can specify the basebone constraint type."); 
		}		
		if  hingeRotationAxis.magnitude() <= 0.0   { 
		  panic!("Hinge rotation axis cannot be zero.");
		}
		if hingeReferenceAxis.magnitude() <= 0.0  { 
		  panic!("Hinge reference axis cannot be zero.");	
		}
		if  !( util::perpendicular(hingeRotationAxis, hingeReferenceAxis) )  {
			panic!("The hinge reference axis must be in the plane of the hinge rotation axis, that is, they must be perpendicular.");
		}
		if  !(hingeType == BaseboneConstraintType::GLOBAL_HINGE || hingeType == BaseboneConstraintType::LOCAL_HINGE)  {	
			panic!("The only valid hinge types for this method are GLOBAL_HINGE and LOCAL_HINGE.");
		}
		
		// Set the constraint type, axis and angle
		self.mBaseboneConstraintType = hingeType;
		self.mBaseboneConstraintUV = hingeRotationAxis.normalize();
		
		let mut hinge: Joint = Joint::new();
		
		if hingeType == BaseboneConstraintType::GLOBAL_HINGE
		{
			hinge.setHinge(JointType::GLOBAL_HINGE, hingeRotationAxis, cwConstraintDegs, acwConstraintDegs, hingeReferenceAxis);
		}
		else
		{
			hinge.setHinge(JointType::LOCAL_HINGE, hingeRotationAxis, cwConstraintDegs, acwConstraintDegs, hingeReferenceAxis);
		}
		self.getBone(0).setJoint(hinge);
	}

    pub fn setFreelyRotatingGlobalHingedBasebone(&mut self, hingeRotationAxis: Vector3<f32>)
	{
		self.setHingeBaseboneConstraint(BaseboneConstraintType::GLOBAL_HINGE, hingeRotationAxis, Rad(3.1415), Rad(3.1415), util::genPerpendicularVectorQuick(hingeRotationAxis) );
	}

    pub fn setFreelyRotatingLocalHingedBasebone(&mut self, hingeRotationAxis: Vector3<f32>)
	{
		self.setHingeBaseboneConstraint(BaseboneConstraintType::LOCAL_HINGE, hingeRotationAxis, Rad(3.1415), Rad(3.1415), util::genPerpendicularVectorQuick(hingeRotationAxis) );
	}

    pub fn setLocalHingedBasebone(&mut self, hingeRotationAxis: Vector3<f32>, cwDegs: Rad<f32>, acwDegs: Rad<f32>, hingeReferenceAxis: Vector3<f32>)
	{
		self.setHingeBaseboneConstraint(BaseboneConstraintType::LOCAL_HINGE, hingeRotationAxis, cwDegs, acwDegs, hingeReferenceAxis);
	}

    pub fn setGlobalHingedBasebone(&mut self, hingeRotationAxis: Vector3<f32>, cwDegs: Rad<f32>, acwDegs: Rad<f32>, hingeReferenceAxis: Vector3<f32>)
	{
		self.setHingeBaseboneConstraint(BaseboneConstraintType::GLOBAL_HINGE, hingeRotationAxis, cwDegs, acwDegs, hingeReferenceAxis);
	}

    pub fn setBaseboneConstraintUV(&mut self, constraintUV: Vector3<f32>)
	{
		if self.mBaseboneConstraintType == BaseboneConstraintType::NONE
		{
			panic!("Specify the basebone constraint type with setBaseboneConstraintTypeCannot specify a basebone constraint when the current constraint type is BaseboneConstraint.NONE.");
		}
		
		// Validate the constraint direction unit vector
		util::validateDirectionUV(constraintUV);
		
		// All good? Then normalise the constraint direction and set it
		constraintUV.normalize();
		self.mBaseboneConstraintUV = constraintUV;
	}

    pub fn setBaseLocation(&mut self, baseLocation: Vector3<f32>) 
    { self.mFixedBaseLocation = baseLocation; }

    pub fn setFixedBaseMode(&mut self, value: bool)
	{	
		// Enforce that a chain connected to another chain stays in fixed base mode (i.e. it moves with the chain it's connected to instead of independently)
		if !value && self.mConnectedChainNumber != None
		{
			panic!("This chain is connected to another chain so must remain in fixed base mode.");
		}
		
		// We cannot have a freely moving base location AND constrain the basebone to an absolute direction
		if self.mBaseboneConstraintType == BaseboneConstraintType::GLOBAL_ROTOR && !value
		{
			panic!("Cannot set a non-fixed base mode when the chain's constraint type is BaseboneConstraintType3D.GLOBAL_ABSOLUTE_ROTOR.");
		}
		
		// Above conditions met? Set the fixedBaseMode
		self.mFixedBaseMode = value;
	}

    pub fn setMaxIterationAttempts(&mut self, maxIterations: usize)
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
		if self.mMinIterationChange < 0.0
		{
			panic!("The minimum iteration change value must be more than or equal to zero.");
		}
		
		// All good? Set the new minimum iteration change distance
		self.mMinIterationChange = minIterationChange;
	}

    pub fn setSolveDistanceThreshold(&mut self, solveDistance: f32)
	{
		// Ensure we have a valid solve distance
		if solveDistance < 0.0
		{
			panic!("The solve distance threshold must be greater than or equal to zero.");
		}
		
		// All good? Set the new solve distance threshold
		self.mSolveDistanceThreshold = solveDistance;
	}
}

//getters
impl Chain{
	pub fn getMaxIterationAttempts(&self) -> usize {
		return self.mMaxIterationAttempts;
	}
	
	pub fn getMinIterationChange(&self) -> f32 {
		return self.mMinIterationChange;
	}
	
	pub fn getSolveDistanceThreshold(&self) -> f32 {
		return self.mSolveDistanceThreshold;
	}

    pub fn getBaseboneRelativeConstraintUV(&self) -> Vector3<f32>
    { return self.mBaseboneRelativeConstraintUV; }

    pub fn getBaseboneConstraintType(&self) -> BaseboneConstraintType
    { return self.mBaseboneConstraintType; }

    pub fn getBaseboneConstraintUV(&self) -> Vector3<f32>
	{
		if  self.mBaseboneConstraintType != BaseboneConstraintType::NONE 
		{
			return self.mBaseboneConstraintUV;
		}
		else
		{
			panic!("Cannot return the basebone constraint when the basebone constraint type is NONE.");
		}
	}

    pub fn getBaseLocation(&self) -> Vector3<f32> 
    { return self.mChain[0].getStartLocation(); }	

    pub fn getBone(&mut self, boneNumber: usize) -> &mut Bone
    { return &mut self.mChain[boneNumber]; }

    pub fn getChain(&self) -> &Vec<Bone>
    { return &self.mChain; }

    pub fn getChainLength(&self) -> f32
    { return self.mChainLength; }

    pub fn getConnectedBoneNumber(&self) -> Option<usize>
    { return self.mConnectedBoneNumber; }

    pub fn getConnectedChainNumber(&self) -> Option<usize> 
    { return self.mConnectedChainNumber; }

    pub fn getEffectorLocation(&self) -> Vector3<f32> 
    { return self.mChain[self.mChain.len()-1].getEndLocation(); }

    pub fn getEmbeddedTargetMode(&self) -> bool
    { return self.mUseEmbeddedTarget; }

    pub fn getEmbeddedTarget(&self) -> Vector3<f32>  
    { return self.mEmbeddedTarget; }

    pub fn getLastTargetLocation(&self) -> Vector3<f32>   
    { return self.mLastTargetLocation; }

    pub fn getLiveChainLength(&self) -> f32
	{
		let mut length: f32 = 0.0;		
		for bone in self.mChain.iter()
		{  
			length += bone.liveLength();
		}		
		return length;
	}	

    pub fn getNumBones(&self) -> usize 
    { return self.mChain.len(); }

    pub fn getBaseboneRelativeReferenceConstraintUV(&self) -> Vector3<f32> 
    { return self.mBaseboneRelativeReferenceConstraintUV;}

    
	
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
            
            let chain_length = self.mChain.len();
            let thisBoneDirectionUV: Vector3<f32> = self.mChain[index].getDirectionUV();
            let thisBoneLength: f32  = self.mChain[index].length();
			let thisBoneJoint: Joint = self.mChain[index].getJoint(); //odd, this one asked to be a reference.
			let thisBoneJointType: JointType = self.mChain[index].getJointType();
            //let thisBone: &mut Bone = self.mChain.get_mut(index).unwrap();
			

			// If we are NOT working on the end effector bone
			if index != chain_length - 1
			{
				// Get the outer-to-inner unit vector of the bone further out
				let outerBoneOuterToInnerUV: Vector3<f32> = -self.mChain[index+1].getDirectionUV();

				// Get the outer-to-inner unit vector of this bone
				let mut thisBoneOuterToInnerUV: Vector3<f32> = -thisBoneDirectionUV;
				
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
				let newStartLocation: Vector3<f32> = self.mChain[index].getEndLocation() + ( thisBoneOuterToInnerUV * (thisBoneLength) );

				// Set the new start joint location for this bone
				self.mChain[index].setStartLocation(newStartLocation);

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
				self.mChain[index].setEndLocation(target);
				
				// Get the UV between the target / end-location (which are now the same) and the start location of this bone
				let mut thisBoneOuterToInnerUV: Vector3<f32> = -self.mChain[index].getDirectionUV();
				
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
				self.mChain[index].setStartLocation(newStartLocation);

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
			//let thisBone: &mut Bone = self.mChain.get_mut(index).unwrap();
			let thisBoneLength: f32  = self.mChain[index].length();

			// If we are not working on the basebone
			if index != 0
			{
				// Get the inner-to-outer direction of this bone as well as the previous bone to use as a baseline
				let mut thisBoneInnerToOuterUV: Vector3<f32> = self.mChain[index].getDirectionUV();
				let prevBoneInnerToOuterUV: Vector3<f32> = self.mChain[index-1].getDirectionUV();
				
				// Dealing with a ball joint?
				let thisBoneJoint: Joint = self.mChain[index].getJoint();
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
				let newEndLocation: Vector3<f32> = self.mChain[index].getStartLocation() + ( thisBoneInnerToOuterUV * (thisBoneLength) );

				// Set the new start joint location for this bone
				self.mChain[index].setEndLocation(newEndLocation);

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
					self.mChain[index].setStartLocation(self.mFixedBaseLocation);
				}
				else // ...otherwise project it backwards from the end to the start by its length.
				{
                    let temp = self.mChain[index].getEndLocation() - ( self.mChain[index].getDirectionUV() * (thisBoneLength));
					self.mChain[index].setStartLocation( temp  ) ;
				}
				
				// If the basebone is unconstrained then process it as usual...
				if self.mBaseboneConstraintType == BaseboneConstraintType::NONE
				{
					// Set the new end location of this bone, and if there are more bones,
					// then set the start location of the next bone to be the end location of this bone
					let newEndLocation: Vector3<f32> = self.mChain[index].getStartLocation() + ( self.mChain[index].getDirectionUV() * (thisBoneLength) );
					self.mChain[index].setEndLocation(newEndLocation);	
					
					if self.mChain.len() > 1 { 
					  self.mChain[1].setStartLocation(newEndLocation); 
					}
				}
				else // ...otherwise we must constrain it to the basebone constraint unit vector
				{	
					if self.mBaseboneConstraintType == BaseboneConstraintType::GLOBAL_ROTOR
					{	
						// Get the inner-to-outer direction of this bone
						let mut thisBoneInnerToOuterUV: Vector3<f32> = self.mChain[index].getDirectionUV();
								
						let angleBetweenDegs: Rad<f32>    = util::getAngleBetweenDegs(self.mBaseboneConstraintUV, thisBoneInnerToOuterUV);
						let constraintAngleDegs: Rad<f32> = self.mChain[index].getBallJointConstraintDegs(); 
					
						if angleBetweenDegs > constraintAngleDegs
						{
							thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, self.mBaseboneConstraintUV, constraintAngleDegs);
						}
						
						let newEndLocation: Vector3<f32> = self.mChain[index].getStartLocation() + ( thisBoneInnerToOuterUV * (thisBoneLength) );
						
						self.mChain[index].setEndLocation( newEndLocation );
						
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
						let mut thisBoneInnerToOuterUV: Vector3<f32> = self.mChain[index].getDirectionUV();
								
						// Constrain about the relative basebone constraint unit vector as neccessary
						let angleBetweenDegs: Rad<f32>    = util::getAngleBetweenDegs(self.mBaseboneRelativeConstraintUV, thisBoneInnerToOuterUV);
						let constraintAngleDegs: Rad<f32> = self.mChain[index].getBallJointConstraintDegs();
						if angleBetweenDegs > constraintAngleDegs
						{
							thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, self.mBaseboneRelativeConstraintUV, constraintAngleDegs);
						}
						
						// Set the end location
						let newEndLocation: Vector3<f32> = self.mChain[index].getStartLocation() + ( thisBoneInnerToOuterUV *(thisBoneLength) );						
						self.mChain[index].setEndLocation( newEndLocation );
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.mChain.len() > 1 { 
						  self.mChain[1].setStartLocation(newEndLocation); 
						}
					}
					else if self.mBaseboneConstraintType == BaseboneConstraintType::GLOBAL_HINGE
					{
						let thisJoint: Joint  =  self.mChain[index].getJoint();
						let hingeRotationAxis: Vector3<f32>  =  thisJoint.getHingeRotationAxis();
						let cwConstraintDegs: Rad<f32>   = -thisJoint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
						let acwConstraintDegs: Rad<f32>  =  thisJoint.getHingeAnticlockwiseConstraintDegs();
						
						// Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
						let mut thisBoneInnerToOuterUV: Vector3<f32> = util::projectOntoPlane(self.mChain[index].getDirectionUV(), hingeRotationAxis);
								
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
						let newEndLocation: Vector3<f32> = self.mChain[index].getStartLocation() + ( thisBoneInnerToOuterUV * (thisBoneLength) );						
						self.mChain[index].setEndLocation( newEndLocation );
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.mChain.len() > 1 { 
						  self.mChain[1].setStartLocation(newEndLocation); 
						}
					}
					else if self.mBaseboneConstraintType == BaseboneConstraintType::LOCAL_HINGE
					{
						let thisJoint: Joint  =  self.mChain[index].getJoint();
						let hingeRotationAxis: Vector3<f32>  =  self.mBaseboneRelativeConstraintUV;                   // Basebone relative constraint is our hinge rotation axis!
						let cwConstraintDegs: Rad<f32>   = -thisJoint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
						let acwConstraintDegs: Rad<f32>  =  thisJoint.getHingeAnticlockwiseConstraintDegs();
						
						// Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
						let mut thisBoneInnerToOuterUV: Vector3<f32> = util::projectOntoPlane(self.mChain[index].getDirectionUV(), hingeRotationAxis);
						
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
						let newEndLocation: Vector3<f32> = self.mChain[index].getStartLocation()+ ( thisBoneInnerToOuterUV * (thisBoneLength) );						
						self.mChain[index].setEndLocation( newEndLocation );
						
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