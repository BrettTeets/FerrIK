use super::{bone::Bone, joint::Joint, joint::JointType, structure::Structure, joint, util};
use cgmath::{Vector3, Rad, InnerSpace, Matrix3};

#[derive(Clone, Copy, PartialEq)]
pub enum BaseboneConstraintType
{
    None,        // No constraint - basebone may rotate freely
    GlobalRotor, // World-space rotor constraint
    LocalRotor,  // Rotor constraint in the coordinate space of (i.e. relative to) the direction of the connected bone
    GlobalHinge, // World-space hinge constraint
    LocalHinge   // Hinge constraint in the coordinate space of (i.e. relative to) the direction of the connected bone
}

pub struct Chain{
    bones: Vec<Bone>,
    solve_distance_threshold: f32,
    max_iteration_attempts: usize,
    min_iteration_change: f32,
    chain_length: f32,
    fixed_base_location: Vector3<f32>,
    fixed_base_mode: bool,
    basebone_constraint_type: BaseboneConstraintType,
    basebone_constraint_uv: Vector3<f32>,
    basebone_relative_constraint_uv: Vector3<f32>,
    basebone_relative_reference_constraint_uv: Vector3<f32>,
    last_target_location: Vector3<f32>,
    last_base_location: Vector3<f32>,
    current_solve_distance: f32,
    connected_chain_number: Option<usize>,
    connected_bone_number: Option<usize>,
    embedded_target: Vector3<f32>,
    use_embedded_target: bool,
}

impl Chain{
    pub fn new() -> Self{
        Self{
            bones: Vec::new(),
            solve_distance_threshold: 1.0,
            max_iteration_attempts: 20,
            min_iteration_change: 0.01,
            chain_length:  0.0,
            fixed_base_location: Vector3::new(0.0,0.0,0.0),
            fixed_base_mode: true,
            basebone_constraint_type: BaseboneConstraintType::None,
            basebone_constraint_uv: Vector3::new(0.0,0.0,0.0),
            basebone_relative_constraint_uv: Vector3::new(0.0,0.0,0.0),
            basebone_relative_reference_constraint_uv: Vector3::new(0.0,0.0,0.0),
            last_target_location: Vector3::new(f32::MAX,f32::MAX,f32::MAX),
            last_base_location: Vector3::new(f32::MAX,f32::MAX,f32::MAX),
            current_solve_distance: f32::MAX,
            connected_chain_number: None,
            connected_bone_number: None,
            embedded_target: Vector3::new(0.0,0.0,0.0),
            use_embedded_target: false,
        }
    }

    pub fn add_bone(&mut self, bone: Bone)
	{
		// If this is the basebone then set the base location and direction constraint.
		if self.bones.len() == 0
		{	
			self.fixed_base_location =  bone.getStartLocation();
			self.basebone_constraint_uv = bone.getDirectionUV();
		}
		//finish up by adding the bone and updating the length. Rust prevents you from adding the bones
		//first.
		self.bones.push(bone);
		self.update_chain_length();
	}

    pub fn add_consecutive_bone(&mut self, mut bone: Bone)
	{
		//guard against adding a consecutive bone when there is no previous bone to reference.
		if !self.bones.is_empty() { panic!("You cannot add the base bone to a chain using this method as it does not provide a start location."); }

		// Get the end location of the last bone, which will be used as the start location of the new bone
		let prev_bone_end = self.bones.last().unwrap().getEndLocation();

		//set bone start to the previous bones end and set it's end to its current direction * len.
		bone.setStartLocation(prev_bone_end);
		bone.setEndLocation( prev_bone_end + ( bone.get_vector_to_end()));
		self.add_bone(bone);
	}

    pub fn remove_bone(&mut self, bone_number: usize)
	{
		//guard to make sure the bone exist.
		if !bone_number < self.bones.len() {panic!("Bone {} does not exist to be removed from the chain. Bones are zero indexed.", bone_number);}
		
		self.bones.remove(bone_number);
		self.update_chain_length();
	}

    pub fn solve_for_embedded_target(&mut self) -> f32
	{
		//guard against the wrong mode.
		if !self.use_embedded_target {panic!("This chain does not have embedded targets enabled");}
		
		return self.solve_for_target(self.embedded_target); 
	}

	//TODO could probably use some more work.
    pub fn solve_for_target(&mut self, new_target: Vector3<f32>) -> f32
	{	
		// If we have both the same target and base location as the last run then do not solve
		if  util::v_approximatelyEquals(self.last_target_location, new_target, 0.001) &&
			 util::v_approximatelyEquals(self.last_base_location, self.getBaseLocation(), 0.001) 
		{
			return self.current_solve_distance;
		}
			
		// TODO: So the current scheme will make a pass with the solver and if its better than the current solution
		// it will store that here in best_solution. Then on every pass it will check if it is better than the
		// last and if so save over the previous best solution. this Saving is currently making a full blown
		// clone everytime. I think some sort of double buffer looking thing might work better but I will have
		// to think on that one. TODO
		let mut best_solution: Vec<Bone> = Vec::new();
		let mut best_solve_distance = f32::MAX;
		let mut last_pass_solve_distance = f32::MAX;
		let mut solve_distance: f32;

		// Allow up to our iteration limit attempts at solving the chain
        for _ in 0..self.max_iteration_attempts
		{	
			// Solve the chain for this target, rember this is going to modify the actual bones stored
			// in this data structure. Is this really the best way to do this?
			solve_distance = self.solveIK(new_target);
			
			// Did we solve it for distance? If so, update our best distance and best solution, and also
			// update our last pass solve distance. Note: We will ALWAYS beat our last solve distance on the first run. 
			if solve_distance < best_solve_distance
			{	
				best_solve_distance = solve_distance;
				best_solution = self.cloneIkChain();
				
				// If we are happy that this solution meets our distance requirements then we can exit the loop now
				if solve_distance <= self.solve_distance_threshold
				{				
					break;
				}
			}
			else if self.check_for_stalling(solve_distance, last_pass_solve_distance)
			{	
				break;
			}
			
			// Update the last pass solve distance for the next iteration.
			last_pass_solve_distance = solve_distance;
			
		} // End of loop
		
		// Update our solve distance and chain configuration to the best solution found
		self.current_solve_distance = best_solve_distance;
		self.bones = best_solution;
		
		// Update our base and target locations
		self.last_base_location = self.getBaseLocation();
		self.last_target_location = new_target;
		
		return self.current_solve_distance;
	}

	fn check_for_stalling(&self, solve_distance: f32, last_solve_distance: f32) -> bool{
		//TODO, maybe log a warning here?
		return (solve_distance - last_solve_distance).abs() < self.min_iteration_change;
	}

    pub fn update_chain_length(&mut self)
	{
		self.chain_length = 0.0;
		for bone in self.bones.iter()
		{
			self.chain_length += bone.length();
		}
	}

    pub fn update_embedded_target(&mut self, new_target: Vector3<f32>)
	{
		//guard to make sure you are using embedded target mode.
		if !self.use_embedded_target {panic!("This chain does not have embedded targets enabled");};
		
		self.embedded_target = new_target; 
	}

	//TODO come back to this one, solve_for_target uses it and I am unsure about keeping that section.
    fn cloneIkChain(&self) -> Vec<Bone>
	{
		// Create a new Vector of FabrikBone3D objects of that size
		let mut clonedChain: Vec<Bone> = Vec::new();

		// For each bone in the chain being cloned...		
		for bone in self.bones.iter()
		{
			// Use the copy constructor to create a new FabrikBone3D with the values set from the source FabrikBone3D.
			// and add it to the cloned chain.
			clonedChain.push(bone.clone() );
		}
		
		return clonedChain;
	}

	//TODO: This should not be done here,
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
		self.connected_chain_number = Some(chainNumber);
		self.connected_bone_number  = Some(boneNumber);		
	}
}

//setters
impl Chain{
    pub fn set_basebone_relative_constraint_uv(&mut self, constraint_uv: Vector3<f32>) 
    { self.basebone_relative_constraint_uv = constraint_uv; }

    pub fn  set_basebone_relative_reference_constraint_uv(&mut self, constraint_uv: Vector3<f32>) 
    { self.basebone_relative_reference_constraint_uv = constraint_uv; }
	
	pub fn set_embedded_target_mode(&mut self, value: bool) 
    { self.use_embedded_target = value; }

    pub fn set_rotor_basebone_constraint(&mut self,  rotor_type: BaseboneConstraintType,
          constraint_axis: Vector3<f32>, angle: Rad<f32>)
	{
		// Guards.
		if self.bones.is_empty()	{ panic!("Chain must contain a basebone before we can specify the basebone constraint type."); }		
		if constraint_axis.magnitude() <= 0.0 { panic!("Constraint axis cannot be zero."); }
		if angle < Rad(0.0)  { panic!("Constraint angle cannot be less than zero."); }
		if angle > Rad(3.1415) { panic!("Constraint angle cannot be greater than 180 degrees.");}		
		if  !(rotor_type == BaseboneConstraintType::GlobalRotor ||
			rotor_type == BaseboneConstraintType::LocalRotor) { panic!("The only valid rotor types for this method are GLOBAL_ROTOR and LOCAL_ROTOR.");}
				
		// Set the constraint type, axis and angle
		self.basebone_constraint_type = rotor_type;
		self.basebone_constraint_uv   = constraint_axis.normalize();
		self.basebone_relative_constraint_uv = self.basebone_constraint_uv;
		self.getBone(0).getJoint().setAsBallJoint(angle);
	}
    
    pub fn setHingeBaseboneConstraint(&mut self,  hingeType: BaseboneConstraintType,
        hingeRotationAxis: Vector3<f32>, cwConstraintDegs: Rad<f32>, acwConstraintDegs: Rad<f32>, hingeReferenceAxis: Vector3<f32>)
	{
		// Sanity checking
		if self.bones.is_empty()	{ 
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
		if  !(hingeType == BaseboneConstraintType::GlobalHinge || hingeType == BaseboneConstraintType::LocalHinge)  {	
			panic!("The only valid hinge types for this method are GLOBAL_HINGE and LOCAL_HINGE.");
		}
		
		// Set the constraint type, axis and angle
		self.basebone_constraint_type = hingeType;
		self.basebone_constraint_uv = hingeRotationAxis.normalize();
		
		let mut hinge: Joint = Joint::new();
		
		if hingeType == BaseboneConstraintType::GlobalHinge
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
		self.setHingeBaseboneConstraint(BaseboneConstraintType::GlobalHinge, hingeRotationAxis, Rad(3.1415), Rad(3.1415), util::genPerpendicularVectorQuick(hingeRotationAxis) );
	}

    pub fn setFreelyRotatingLocalHingedBasebone(&mut self, hingeRotationAxis: Vector3<f32>)
	{
		self.setHingeBaseboneConstraint(BaseboneConstraintType::LocalHinge, hingeRotationAxis, Rad(3.1415), Rad(3.1415), util::genPerpendicularVectorQuick(hingeRotationAxis) );
	}

    pub fn setLocalHingedBasebone(&mut self, hingeRotationAxis: Vector3<f32>, cwDegs: Rad<f32>, acwDegs: Rad<f32>, hingeReferenceAxis: Vector3<f32>)
	{
		self.setHingeBaseboneConstraint(BaseboneConstraintType::LocalHinge, hingeRotationAxis, cwDegs, acwDegs, hingeReferenceAxis);
	}

    pub fn setGlobalHingedBasebone(&mut self, hingeRotationAxis: Vector3<f32>, cwDegs: Rad<f32>, acwDegs: Rad<f32>, hingeReferenceAxis: Vector3<f32>)
	{
		self.setHingeBaseboneConstraint(BaseboneConstraintType::GlobalHinge, hingeRotationAxis, cwDegs, acwDegs, hingeReferenceAxis);
	}

    pub fn setBaseboneConstraintUV(&mut self, constraintUV: Vector3<f32>)
	{
		if self.basebone_constraint_type == BaseboneConstraintType::None
		{
			panic!("Specify the basebone constraint type with setBaseboneConstraintTypeCannot specify a basebone constraint when the current constraint type is BaseboneConstraint.None.");
		}
		
		// Validate the constraint direction unit vector
		util::validateDirectionUV(constraintUV);
		
		// All good? Then normalise the constraint direction and set it
		constraintUV.normalize();
		self.basebone_constraint_uv = constraintUV;
	}

    pub fn setBaseLocation(&mut self, baseLocation: Vector3<f32>) 
    { self.fixed_base_location = baseLocation; }

    pub fn setFixedBaseMode(&mut self, value: bool)
	{	
		// Enforce that a chain connected to another chain stays in fixed base mode (i.e. it moves with the chain it's connected to instead of independently)
		if !value && self.connected_chain_number != None
		{
			panic!("This chain is connected to another chain so must remain in fixed base mode.");
		}
		
		// We cannot have a freely moving base location AND constrain the basebone to an absolute direction
		if self.basebone_constraint_type == BaseboneConstraintType::GlobalRotor && !value
		{
			panic!("Cannot set a non-fixed base mode when the chain's constraint type is BaseboneConstraintType3D.GLOBAL_ABSOLUTE_ROTOR.");
		}
		
		// Above conditions met? Set the fixedBaseMode
		self.fixed_base_mode = value;
	}

    pub fn setMaxIterationAttempts(&mut self, maxIterations: usize)
	{
		// Ensure we have a valid maximum number of iteration attempts
		if maxIterations < 1
		{
			panic!("The maximum number of attempts to solve this IK chain must be at least 1.");
		}
		
		// All good? Set the new maximum iteration attempts property
		self.max_iteration_attempts = maxIterations;
	}

    pub fn setMinIterationChange(&mut self, minIterationChange: f32)
	{
		// Ensure we have a valid maximum number of iteration attempts
		if self.min_iteration_change < 0.0
		{
			panic!("The minimum iteration change value must be more than or equal to zero.");
		}
		
		// All good? Set the new minimum iteration change distance
		self.min_iteration_change = minIterationChange;
	}

    pub fn setSolveDistanceThreshold(&mut self, solveDistance: f32)
	{
		// Ensure we have a valid solve distance
		if solveDistance < 0.0
		{
			panic!("The solve distance threshold must be greater than or equal to zero.");
		}
		
		// All good? Set the new solve distance threshold
		self.solve_distance_threshold = solveDistance;
	}
}

//getters
impl Chain{
	pub fn getMaxIterationAttempts(&self) -> usize {
		return self.max_iteration_attempts;
	}
	
	pub fn getMinIterationChange(&self) -> f32 {
		return self.min_iteration_change;
	}
	
	pub fn getSolveDistanceThreshold(&self) -> f32 {
		return self.solve_distance_threshold;
	}

    pub fn getBaseboneRelativeConstraintUV(&self) -> Vector3<f32>
    { return self.basebone_relative_constraint_uv; }

    pub fn getBaseboneConstraintType(&self) -> BaseboneConstraintType
    { return self.basebone_constraint_type; }

    pub fn getBaseboneConstraintUV(&self) -> Vector3<f32>
	{
		if  self.basebone_constraint_type != BaseboneConstraintType::None 
		{
			return self.basebone_constraint_uv;
		}
		else
		{
			panic!("Cannot return the basebone constraint when the basebone constraint type is None.");
		}
	}

    pub fn getBaseLocation(&self) -> Vector3<f32> 
    { return self.bones[0].getStartLocation(); }	

    pub fn getBone(&mut self, boneNumber: usize) -> &mut Bone
    { return &mut self.bones[boneNumber]; }

    pub fn getChain(&self) -> &Vec<Bone>
    { return &self.bones; }

    pub fn getChainLength(&self) -> f32
    { return self.chain_length; }

    pub fn getConnectedBoneNumber(&self) -> Option<usize>
    { return self.connected_bone_number; }

    pub fn getConnectedChainNumber(&self) -> Option<usize> 
    { return self.connected_chain_number; }

    pub fn getEffectorLocation(&self) -> Vector3<f32> 
    { return self.bones[self.bones.len()-1].getEndLocation(); }

    pub fn getEmbeddedTargetMode(&self) -> bool
    { return self.use_embedded_target; }

    pub fn getEmbeddedTarget(&self) -> Vector3<f32>  
    { return self.embedded_target; }

    pub fn getLastTargetLocation(&self) -> Vector3<f32>   
    { return self.last_target_location; }

    pub fn getLiveChainLength(&self) -> f32
	{
		let mut length: f32 = 0.0;		
		for bone in self.bones.iter()
		{  
			length += bone.liveLength();
		}		
		return length;
	}	

    pub fn getNumBones(&self) -> usize 
    { return self.bones.len(); }

    pub fn getBaseboneRelativeReferenceConstraintUV(&self) -> Vector3<f32> 
    { return self.basebone_relative_reference_constraint_uv;}

    
	
}

impl Chain{
    pub fn solveIK(&mut self, target: Vector3<f32>) -> f32
	{	
		// Sanity check that there are bones in the chain
		if self.bones.is_empty() { 
		  panic!("It makes no sense to solve an IK chain with zero bones."); 
		}
		
		// ---------- Forward pass from end effector to base -----------

		// Loop over all bones in the chain, from the end effector (numBones-1) back to the basebone (0)		
		//for (int loop = mChain.size()-1; loop >= 0; --loop)
        for index in (0..self.bones.len()).rev()
		{
			// Get the length of the bone we're working on
            
            let chain_length = self.bones.len();
            let thisBoneDirectionUV: Vector3<f32> = self.bones[index].getDirectionUV();
            let thisBoneLength: f32  = self.bones[index].length();
			let thisBoneJoint: Joint = self.bones[index].getJoint(); //odd, this one asked to be a reference.
			let thisBoneJointType: JointType = self.bones[index].getJointType();
            //let thisBone: &mut Bone = self.mChain.get_mut(index).unwrap();
			

			// If we are NOT working on the end effector bone
			if index != chain_length - 1
			{
				// Get the outer-to-inner unit vector of the bone further out
				let outerBoneOuterToInnerUV: Vector3<f32> = -self.bones[index+1].getDirectionUV();

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
						m = util::createRotationMatrix( self.bones[index-1].getDirectionUV() );
						relativeHingeRotationAxis = m * ( thisBoneJoint.getHingeRotationAxis() ).normalize();
					}
					else // ...basebone? Need to construct matrix from the relative constraint UV.
					{
						relativeHingeRotationAxis = self.basebone_relative_constraint_uv;
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
				let newStartLocation: Vector3<f32> = self.bones[index].getEndLocation() + ( thisBoneOuterToInnerUV * (thisBoneLength) );

				// Set the new start joint location for this bone
				self.bones[index].setStartLocation(newStartLocation);

				// If we are not working on the basebone, then we also set the end joint location of
				// the previous bone in the chain (i.e. the bone closer to the base) to be the new
				// start joint location of this bone.
				if index > 0
				{
					self.bones[index-1].setEndLocation(newStartLocation);
				}
			}
			else // If we ARE working on the end effector bone...
			{
				// Snap the end effector's end location to the target
				self.bones[index].setEndLocation(target);
				
				// Get the UV between the target / end-location (which are now the same) and the start location of this bone
				let mut thisBoneOuterToInnerUV: Vector3<f32> = -self.bones[index].getDirectionUV();
				
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
						let m: Matrix3<f32> = util::createRotationMatrix( self.bones[index-1].getDirectionUV() );
						
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
				self.bones[index].setStartLocation(newStartLocation);

				// ...and set the end joint location of the bone further in to also be at the new start location (if there IS a bone
				// further in - this may be a single bone chain)
				if index > 0
				{
					self.bones[index-1].setEndLocation(newStartLocation);
				}
			}
			
		} // End of forward pass

		// ---------- Backward pass from base to end effector -----------
 
		//for (int loop = 0; loop < mChain.size(); ++loop)
        for index in 0..self.bones.len()
		{
			//let thisBone: &mut Bone = self.mChain.get_mut(index).unwrap();
			let thisBoneLength: f32  = self.bones[index].length();

			// If we are not working on the basebone
			if index != 0
			{
				// Get the inner-to-outer direction of this bone as well as the previous bone to use as a baseline
				let mut thisBoneInnerToOuterUV: Vector3<f32> = self.bones[index].getDirectionUV();
				let prevBoneInnerToOuterUV: Vector3<f32> = self.bones[index-1].getDirectionUV();
				
				// Dealing with a ball joint?
				let thisBoneJoint: Joint = self.bones[index].getJoint();
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
				let newEndLocation: Vector3<f32> = self.bones[index].getStartLocation() + ( thisBoneInnerToOuterUV * (thisBoneLength) );

				// Set the new start joint location for this bone
				self.bones[index].setEndLocation(newEndLocation);

				// If we are not working on the end effector bone, then we set the start joint location of the next bone in
				// the chain (i.e. the bone closer to the target) to be the new end joint location of this bone.
				if index < self.bones.len() - 1 { 
				  self.bones[index+1].setStartLocation(newEndLocation); 
				}
			}
			else // If we ARE working on the basebone...
			{	
				// If the base location is fixed then snap the start location of the basebone back to the fixed base...
				if self.fixed_base_mode
				{
					self.bones[index].setStartLocation(self.fixed_base_location);
				}
				else // ...otherwise project it backwards from the end to the start by its length.
				{
                    let temp = self.bones[index].getEndLocation() - ( self.bones[index].getDirectionUV() * (thisBoneLength));
					self.bones[index].setStartLocation( temp  ) ;
				}
				
				// If the basebone is unconstrained then process it as usual...
				if self.basebone_constraint_type == BaseboneConstraintType::None
				{
					// Set the new end location of this bone, and if there are more bones,
					// then set the start location of the next bone to be the end location of this bone
					let newEndLocation: Vector3<f32> = self.bones[index].getStartLocation() + ( self.bones[index].getDirectionUV() * (thisBoneLength) );
					self.bones[index].setEndLocation(newEndLocation);	
					
					if self.bones.len() > 1 { 
					  self.bones[1].setStartLocation(newEndLocation); 
					}
				}
				else // ...otherwise we must constrain it to the basebone constraint unit vector
				{	
					if self.basebone_constraint_type == BaseboneConstraintType::GlobalRotor
					{	
						// Get the inner-to-outer direction of this bone
						let mut thisBoneInnerToOuterUV: Vector3<f32> = self.bones[index].getDirectionUV();
								
						let angleBetweenDegs: Rad<f32>    = util::getAngleBetweenDegs(self.basebone_constraint_uv, thisBoneInnerToOuterUV);
						let constraintAngleDegs: Rad<f32> = self.bones[index].getBallJointConstraintDegs(); 
					
						if angleBetweenDegs > constraintAngleDegs
						{
							thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, self.basebone_constraint_uv, constraintAngleDegs);
						}
						
						let newEndLocation: Vector3<f32> = self.bones[index].getStartLocation() + ( thisBoneInnerToOuterUV * (thisBoneLength) );
						
						self.bones[index].setEndLocation( newEndLocation );
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.bones.len() > 1 { 
						  self.bones[1].setStartLocation(newEndLocation); 
						}
					}
					else if self.basebone_constraint_type == BaseboneConstraintType::LocalRotor
					{
						// Note: The mBaseboneRelativeConstraintUV is updated in the FabrikStructure3D.solveForTarget()
						// method BEFORE this FabrikChain3D.solveForTarget() method is called. We no knowledge of the
						// direction of the bone we're connected to in another chain and so cannot calculate this 
						// relative basebone constraint direction on our own, but the FabrikStructure3D does it for
						// us so we are now free to use it here.
						
						// Get the inner-to-outer direction of this bone
						let mut thisBoneInnerToOuterUV: Vector3<f32> = self.bones[index].getDirectionUV();
								
						// Constrain about the relative basebone constraint unit vector as neccessary
						let angleBetweenDegs: Rad<f32>    = util::getAngleBetweenDegs(self.basebone_relative_constraint_uv, thisBoneInnerToOuterUV);
						let constraintAngleDegs: Rad<f32> = self.bones[index].getBallJointConstraintDegs();
						if angleBetweenDegs > constraintAngleDegs
						{
							thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, self.basebone_relative_constraint_uv, constraintAngleDegs);
						}
						
						// Set the end location
						let newEndLocation: Vector3<f32> = self.bones[index].getStartLocation() + ( thisBoneInnerToOuterUV *(thisBoneLength) );						
						self.bones[index].setEndLocation( newEndLocation );
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.bones.len() > 1 { 
						  self.bones[1].setStartLocation(newEndLocation); 
						}
					}
					else if self.basebone_constraint_type == BaseboneConstraintType::GlobalHinge
					{
						let thisJoint: Joint  =  self.bones[index].getJoint();
						let hingeRotationAxis: Vector3<f32>  =  thisJoint.getHingeRotationAxis();
						let cwConstraintDegs: Rad<f32>   = -thisJoint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
						let acwConstraintDegs: Rad<f32>  =  thisJoint.getHingeAnticlockwiseConstraintDegs();
						
						// Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
						let mut thisBoneInnerToOuterUV: Vector3<f32> = util::projectOntoPlane(self.bones[index].getDirectionUV(), hingeRotationAxis);
								
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
						let newEndLocation: Vector3<f32> = self.bones[index].getStartLocation() + ( thisBoneInnerToOuterUV * (thisBoneLength) );						
						self.bones[index].setEndLocation( newEndLocation );
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.bones.len() > 1 { 
						  self.bones[1].setStartLocation(newEndLocation); 
						}
					}
					else if self.basebone_constraint_type == BaseboneConstraintType::LocalHinge
					{
						let thisJoint: Joint  =  self.bones[index].getJoint();
						let hingeRotationAxis: Vector3<f32>  =  self.basebone_relative_constraint_uv;                   // Basebone relative constraint is our hinge rotation axis!
						let cwConstraintDegs: Rad<f32>   = -thisJoint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
						let acwConstraintDegs: Rad<f32>  =  thisJoint.getHingeAnticlockwiseConstraintDegs();
						
						// Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
						let mut thisBoneInnerToOuterUV: Vector3<f32> = util::projectOntoPlane(self.bones[index].getDirectionUV(), hingeRotationAxis);
						
						// If we have a local hinge which is not freely rotating then we must constrain about the reference axis
						if  !( util::approximatelyEquals(cwConstraintDegs.0 , -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) &&
							    util::approximatelyEquals(acwConstraintDegs.0,  joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) ) 
						{
							// Grab the hinge reference axis and calculate the current signed angle between it and our bone direction (about the hinge
							// rotation axis). Note: ACW rotation is positive, CW rotation is negative.
							let hingeReferenceAxis: Vector3<f32> = self.basebone_relative_reference_constraint_uv; 
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
						let newEndLocation: Vector3<f32> = self.bones[index].getStartLocation()+ ( thisBoneInnerToOuterUV * (thisBoneLength) );						
						self.bones[index].setEndLocation( newEndLocation );
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.bones.len() > 1 { 
						  self.bones[1].setStartLocation(newEndLocation); 
						}
					}
					
				} // End of basebone constraint handling section

			} // End of basebone handling section

		} // End of backward-pass loop over all bones

		// Update our last target location
		self.last_target_location = target;
				
		// DEBUG - check the live chain length and the originally calculated chain length are the same
		/*
		if (Math.abs( this.getLiveChainLength() - mChainLength) > 0.01f)
		{
			System.out.println("Chain length off by > 0.01f");
		}
		*/
		
		// Finally, calculate and return the distance between the current effector location and the target.
		return util::distanceBetween(self.bones[self.bones.len()-1].getEndLocation(), target);
	}
} //line 943