use cgmath::{Vector3, InnerSpace, Deg, Matrix3, Rad};
use crate::{bone3D, joint3D, chain, chain::FerrikErrors, joint};
use log::{error};
use crate::util;

#[derive(Clone)]
pub struct Chain3D{
    id: usize,
    m_chain: Vec<bone3D::Bone3D>,
    solve_distance_threshold: f32, //this was set to 1.0 as the default.
    max_iteration_attempts: usize, //default was 20
    min_iteration_change: f32, //default was 0.01,
    chain_length: f32, //this is recalculated everytime a bone is added or removed
   	pub base_location: Vector3<f32>, //start location of the root bone. This was original fixedbased location, but they both used and both passed vectors directly to it.
    fixed_base_mode: bool, //whether the Base can be moved by the solver.
    pub basebone_constraint_type: BaseboneConstraintType3D,
    mBaseboneConstraintUV: Vector3<f32>, //this is the direction the base bone should be constrained around
    //since it has no prior bone to ask.
    pub basebone_relative_constraint_uv: Vector3<f32>, //used if this bone is attached to another bone in another chain.
    mBaseboneRelativeReferenceConstraintUV: Vector3<f32>, //honestly their just making stuff up now.
    mLastTargetLocation: Vector3<f32>, //this defaulted to a maxed out vector3(f32.max, f32.max f32.max).
    mLastBaseLocation: Vector3<f32>, //aslo set to max by default
    mCurrentSolveDistance: f32, //set to max.
    mConnectedChainNumber: Option<usize>, //index in the structure that this chain was connected to. Might make this an option as it was using -1 to indicate no.
    mConnectedBoneNumber: Option<usize>, //index of the bone in the connected chain. //might make option.
    mEmbeddedTarget: Vector3<f32>,
    mUseEmbeddedTarget: bool, //toggle


}

impl Chain3D{
  
	fn find_live_chain_length(&self) -> f32
	{
		let length = 0.0;		
		for b in self.m_chain
		{
			length += b.live_length();
		}
		return length;
	}
	
	/*fn connect_to_structure(&mut self, structure: &mut structure::Structure3D , 
		chain_number: usize, bone_number:usize) -> Result<(), FerrikErrors>
	{
		// Sanity check chain exists
		let numy_chains = structure.getNumy_chains();
		if chain_number > numy_chains { 
		  	error!("Structure does not contain a chain {} - it has {} chains.", chain_number, numy_chains,); 
			return Err(FerrikErrors::UnsolvableRequirement);
		}
		
		// Sanity check bone exists
		let num_bones = structure.getChain(chain_number).getNumBones();
		if bone_number > num_bones { 
		  	error!("Chain does not contain a bone {}  - it has {} bones.", bone_number, num_bones);
			return Err(FerrikErrors::UnsolvableRequirement); 
		}
		
		// All good? Set the connection details
		self.mConnectedChainNumber = Some(chain_number);
		self.mConnectedBoneNumber  = Some(bone_number);
		return Ok(());		
	}*/
}

impl Chain3D{

	fn solve_for_embedded_target(&mut self) -> Result<f32, FerrikErrors>
	{
		if self.mUseEmbeddedTarget { 
		  	return Ok(self.solve_for_target(self.mEmbeddedTarget)); 
		}
		else { 
		  	error!("This chain does not have embedded targets enabled - enable with setEmbeddedTargetMode(true)."); 
			return Err(FerrikErrors::InvalidMethodCall);
		}
	}

	fn solve_for_target(&mut self, new_target: Vector3<f32>) -> f32
	{	
		// If we have both the same target and base location as the last run then do not solve
		if  util::vector_approximatelyEquals(self.mLastTargetLocation, new_target, 0.001).unwrap() &&
			 util::vector_approximatelyEquals(self.mLastBaseLocation, self.base_location, 0.001).unwrap() 
		{
			return self.mCurrentSolveDistance;
		}
		
		/***
		 * NOTE: We must allow the best solution of THIS run to be used for a new target or base location - we cannot
		 * just use the last solution (even if it's better) - because that solution was for a different target / base
		 * location combination and NOT for the current setup.
		*/
		let best_solution: Vec<bone3D::Bone3D> = Vec::new();
		
		// We start with a best solve distance that can be easily beaten
		let best_solve_distance = f32::MAX;
		
		// We'll also keep track of the solve distance from the last pass
		let last_pass_solve_distance = f32::MAX;
		
		// Allow up to our iteration limit attempts at solving the chain
		let solve_distance: f32;
		for i in 0..self.max_iteration_attempts
		{	
			// Solve the chain for this target
			solve_distance = self.solve_IK(new_target).unwrap();
			
			// Did we solve it for distance? If so, update our best distance and best solution, and also
			// update our last pass solve distance. Note: We will ALWAYS beat our last solve distance on the first run. 
			if solve_distance < best_solve_distance
			{	
				best_solve_distance = solve_distance;
				best_solution = self.m_chain.clone();
				
				// If we are happy that this solution meets our distance requirements then we can exit the loop now
				if solve_distance <= self.solve_distance_threshold
				{				
					break;
				}
			}
			else // Did not solve to our satisfaction? Okay...
			{
				// Did we grind to a halt? If so break out of loop to set the best distance and solution that we have
				if (solve_distance - last_pass_solve_distance).abs() < self.min_iteration_change
				{
					break;
				}
			}
			
			// Update the last pass solve distance
			last_pass_solve_distance = solve_distance;
			
		} // End of loop
		
		// Update our solve distance and chain configuration to the best solution found
		self.mCurrentSolveDistance = best_solve_distance;
		self.m_chain = best_solution;
		
		// Update our base and target locations
		self.mLastBaseLocation = self.base_location;
		self.mLastTargetLocation = new_target;
		
		return self.mCurrentSolveDistance;
	}

	fn add_bone(&mut self, bone: bone3D::Bone3D)
	{
		// Add the new bone to the end of the ArrayList of bones
		self.m_chain.push(bone);
		// If this is the basebone...
		if self.m_chain.len() == 1
		{
			// ...then keep a copy of the fixed start location...
			self.base_location = bone.start.clone();
			// ...and set the basebone constraint UV to be around the initial bone direction
			self.mBaseboneConstraintUV = bone.get_direction_uv();
		}
		
		// Increment the number of bones in the chain and update the chain length
		self.update_chain_length();
	}

	fn set_solve_distance_threshold(&mut self, solve_distance: f32) -> Result<(), FerrikErrors>
	{
		if solve_distance < 0.0 {
			error!("The solve distance threshold must be greater than or equal to zero.");
			return Err(FerrikErrors::UnsolvableRequirement);
		}

		self.solve_distance_threshold = solve_distance;
		return Ok(());
	}

	fn set_max_iteration_attempts(&mut self, max_iterations: usize) -> Result<(), FerrikErrors>
	{
		if max_iterations < 1
		{
			error!("The maximum number of attempts to solve this IK chain must be at least 1.");
			return Err(FerrikErrors::UnsolvableRequirement);
		}

		self.max_iteration_attempts = max_iterations;
		return Ok(());
	}

	fn set_min_iteration_change(&mut self, minIterationChange: f32) -> Result<(), FerrikErrors>
	{
		// Ensure we have a valid maximum number of iteration attempts
		if minIterationChange < 0.0
		{
			error!("The minimum iteration change value must be more than or equal to zero.");
			return Err(FerrikErrors::UnsolvableRequirement);
		}
		
		// All good? Set the new minimum iteration change distance
		self.min_iteration_change = minIterationChange;
		return Ok(());
	}

	fn update_chain_length(&mut self)
	{
		//set it to zero and then loop over the chain adding it up.
		self.chain_length = 0.0;
		for b in self.m_chain
		{
			self.chain_length += b.length();
		}
	}

	fn set_fixed_base_mode(&mut self, value: bool) -> Result<(), FerrikErrors>
	{	
		// Enforce that a chain connected to another chain stays in fixed base mode (i.e. it moves with the chain it's connected to instead of independently)
		if !value && self.mConnectedChainNumber != None
		{
			error!("This chain is connected to another chain so must remain in fixed base mode.");
			return Err(FerrikErrors::MultipleChainIssues)
		}
		// We cannot have a freely moving base location AND constrain the basebone to an absolute direction
		if self.basebone_constraint_type == BaseboneConstraintType3D::GlobalRotor && !value
		{
			error!("Cannot set a non-fixed base mode when the chain's constraint type is BaseboneConstraintType3D.GLOBAL_ABSOLUTE_ROTOR.");
			return Err(FerrikErrors::UnsolvableRequirement)
		}
		
		self.fixed_base_mode = value;
		return Ok(());
	}

	fn find_effector_location(&self) -> Result<Vector3<f32>, FerrikErrors> 
	{ 
		match self.m_chain.last(){
			Some(v) => return Ok(v.end),
			None => {
				error!("When attempting to find the end effector, none was found. This should'nt be possible");
				return Err(FerrikErrors::ImpossibleHappenstance)
			}
		};
	}

	fn set_basebone_constraint_uv(&mut self, constraintUV: Vector3<f32>) -> Result<(), FerrikErrors>
	{
		if self.basebone_constraint_type == BaseboneConstraintType3D::None
		{
			error!("Specify the basebone constraint type with setBaseboneConstraintTypeCannot specify a basebone constraint when the current constraint type is BaseboneConstraint.NONE.");
			return Err(FerrikErrors::UnsolvableRequirement);
		}
		
		// Validate the constraint direction unit vector
		util::validateDirectionUV(constraintUV);
		
		// All good? Then normalise the constraint direction and set it
		self.mBaseboneConstraintUV = constraintUV.normalize();
		return Ok(());
	}

	
}

#[derive(Clone, PartialEq)]
pub enum BaseboneConstraintType3D {
    None,         // No constraint - basebone may rotate freely
    GlobalRotor, // World-space rotor constraint
    LocalRotor,  // Rotor constraint in the coordinate space of (i.e. relative to) the direction of the connected bone
    GlobalHinge, // World-space hinge constraint
    LocalHinge   // Hinge constraint in the coordinate space of (i.e. relative to) the direction of the connected bone
}

//The IK solver gets to live down here. Oh Boy is this a monster.

impl Chain3D{
	fn solve_IK(&mut self, target: Vector3<f32>) -> Result<f32, FerrikErrors>
	{	
		// Sanity check that there are bones in the chain
		if self.m_chain.is_empty() { 
		  	error!("It makes no sense to solve an IK chain with zero bones."); 
			return Err(FerrikErrors::UnsolvableRequirement);
		}
		
		// ---------- Forward pass from end effector to base -----------

		// Loop over all bones in the chain, from the end effector (numBones-1) back to the basebone (0)		
		for index in (0..self.m_chain.len()).rev()
		{
			// Get the length of the bone we're working on
			let mut thisBone = self.m_chain[index];
			let thisBoneLength  = thisBone.length();
			let thisBoneJoint = thisBone.joint;
			let thisBoneJointType = thisBone.joint.joint_type;

			// If we are NOT working on the end effector bone
			if index != self.m_chain.len() - 1
			{
				// Get the outer-to-inner unit vector of the bone further out. dont forget to negate
				let outerBoneOuterToInnerUV = -self.m_chain[index + 1].get_direction_uv();

				// Get the outer-to-inner unit vector of this bone. dont forget to negate.
				let thisBoneOuterToInnerUV = -thisBone.get_direction_uv();
				
				// Get the joint type for this bone and handle constraints on thisBoneInnerToOuterUV				
				if thisBoneJointType == joint::JointType::Ball
				{	
					// Constrain to relative angle between this bone and the outer bone if required
					let angleBetweenDegs: Rad<f32>  = util::getAngleBetweenRads(outerBoneOuterToInnerUV, thisBoneOuterToInnerUV);
					let constraintAngleDegs: Rad<f32> = thisBoneJoint.get_ball_constraint().into();
					if angleBetweenDegs > constraintAngleDegs
					{	
						thisBoneOuterToInnerUV = util::getAngleLimitedUnitVectorDegs(thisBoneOuterToInnerUV, outerBoneOuterToInnerUV, constraintAngleDegs);
					}
				}
				else if thisBoneJointType == joint::JointType::GlobalHinge
				{	
					// Project this bone outer-to-inner direction onto the hinge rotation axis
					// Note: The returned vector is normalised.
					thisBoneOuterToInnerUV = util::projectOntoPlane(&thisBoneOuterToInnerUV, &thisBoneJoint.rotation_axis_uv).unwrap(); 
					
					// NOTE: Constraining about the hinge reference axis on this forward pass leads to poor solutions... so we won't.
				}
				else if thisBoneJointType == joint::JointType::LocalHinge
				{	
					// Not a basebone? Then construct a rotation matrix (Mat3f) based on the previous bones inner-to-to-inner direction...
					let m: Matrix3<f32>;
					let relativeHingeRotationAxis: Vector3<f32>;
					if index > 0 {
						let m: Matrix3<f32> = util::createRotationMatrix( self.m_chain[index-1].get_direction_uv());
						relativeHingeRotationAxis = m * ( thisBoneJoint.rotation_axis_uv ).normalize();
					}
					else // ...basebone? Need to construct matrix from the relative constraint UV.
					{
						relativeHingeRotationAxis = self.basebone_relative_constraint_uv;
					}
					
					// ...and transform the hinge rotation axis into the previous bones frame of reference.
					//Vec3f 
										
					// Project this bone's outer-to-inner direction onto the plane described by the relative hinge rotation axis
					// Note: The returned vector is normalised.					
					thisBoneOuterToInnerUV = util::projectOntoPlane(&thisBoneOuterToInnerUV, &relativeHingeRotationAxis).unwrap();
										
					// NOTE: Constraining about the hinge reference axis on this forward pass leads to poor solutions... so we won't.										
				}
					
				// At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
				// so we can set the new inner joint location to be the end joint location of this bone plus the
				// outer-to-inner direction unit vector multiplied by the length of the bone.
				let newStartLocation: Vector3<f32> = thisBone.end + (thisBoneOuterToInnerUV * thisBoneLength);

				// Set the new start joint location for this bone
				thisBone.start = newStartLocation;

				// If we are not working on the basebone, then we also set the end joint location of
				// the previous bone in the chain (i.e. the bone closer to the base) to be the new
				// start joint location of this bone.
				if index > 0
				{
					self.m_chain[index-1].end = newStartLocation;
				}
			}
			else // If we ARE working on the end effector bone...
			{
				// Snap the end effector's end location to the target
				thisBone.end = target;
				
				// Get the UV between the target / end-location (which are now the same) and the start location of this bone
				let thisBoneOuterToInnerUV: Vector3<f32> = -thisBone.get_direction_uv(); //dont forget to negate
				
				// If the end effector is global hinged then we have to snap to it, then keep that
				// resulting outer-to-inner UV in the plane of the hinge rotation axis
				match  thisBoneJointType 
				{
					joint::JointType::Ball => {},// Ball joints do not get constrained on this forward pass					
					joint::JointType::GlobalHinge => {
						// Global hinges get constrained to the hinge rotation axis, but not the reference axis within the hinge plane
						thisBoneOuterToInnerUV = util::projectOntoPlane(&thisBoneOuterToInnerUV, &thisBoneJoint.rotation_axis_uv ).unwrap();
						break
					},
					joint::JointType::LocalHinge => {
						// Local hinges get constrained to the hinge rotation axis, but not the reference axis within the hinge plane
						
						// Construct a rotation matrix based on the previous bones inner-to-to-inner direction...
						let m: Matrix3<f32> = util::createRotationMatrix( self.m_chain[index-1].get_direction_uv() );
						
						// ...and transform the hinge rotation axis into the previous bones frame of reference.
						let relativeHingeRotationAxis: Vector3<f32> = m * ( thisBoneJoint.rotation_axis_uv).normalize();
											
						// Project this bone's outer-to-inner direction onto the plane described by the relative hinge rotation axis
						// Note: The returned vector is normalised.					
						thisBoneOuterToInnerUV = util::projectOntoPlane(&thisBoneOuterToInnerUV, &relativeHingeRotationAxis).unwrap();
					}
				}
												
				// Calculate the new start joint location as the end joint location plus the outer-to-inner direction UV
				// multiplied by the length of the bone.
				let newStartLocation: Vector3<f32> = target + ( thisBoneOuterToInnerUV * (thisBoneLength) );
				
				// Set the new start joint location for this bone to be new start location...
				thisBone.start  = newStartLocation;

				// ...and set the end joint location of the bone further in to also be at the new start location (if there IS a bone
				// further in - this may be a single bone chain)
				if index > 0
				{
					self.m_chain[index-1].end = newStartLocation;
				}
			}
			
		} // End of forward pass

		// ---------- Backward pass from base to end effector -----------
 
		
		for index in 0..self.m_chain.len()//for (int loop = 0; loop < my_chain.size(); ++loop)
		{
			let thisBone = self.m_chain[index];
			let thisBoneLength: f32  = thisBone.length();

			// If we are not working on the basebone
			if index != 0
			{
				// Get the inner-to-outer direction of this bone as well as the previous bone to use as a baseline
				let thisBoneInnerToOuterUV: Vector3<f32> = thisBone.get_direction_uv();
				let prevBoneInnerToOuterUV: Vector3<f32> = self.m_chain[index-1].get_direction_uv();
				
				// Dealing with a ball joint?
				let thisBoneJoint = thisBone.joint;
				let jointType = thisBoneJoint.joint_type;
				if jointType == joint::JointType::Ball
				{					
					let angleBetweenDegs: Rad<f32> = util::getAngleBetweenRads(prevBoneInnerToOuterUV, thisBoneInnerToOuterUV);
					let constraintAngleDegs: Rad<f32> = thisBoneJoint.get_ball_constraint().into(); 
					
					// Keep this bone direction constrained within the rotor about the previous bone direction
					if angleBetweenDegs > constraintAngleDegs
					{
						thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, prevBoneInnerToOuterUV, constraintAngleDegs);
					}
				}
				else if jointType == joint::JointType::GlobalHinge
				{					
					// Get the hinge rotation axis and project our inner-to-outer UV onto it
					let hingeRotationAxis: Vector3<f32>  =  thisBoneJoint.rotation_axis_uv;
					thisBoneInnerToOuterUV = util::projectOntoPlane(&thisBoneInnerToOuterUV, &hingeRotationAxis).unwrap();
					
					// If there are joint constraints, then we must honour them...
					let cwConstraintDegs: Rad<f32>   = -thisBoneJoint.hinge_clockwise_constraint;
					let acwConstraintDegs: Rad<f32>  =  thisBoneJoint.hinge_anticlockwise_constraint;
					if  !( util::rads_approximatelyEquals(cwConstraintDegs, -joint3D::MAX_CONSTRAINT_ANGLE_DEGS, Deg(0.001).into()) ) &&
						 !( util::rads_approximatelyEquals(acwConstraintDegs, joint3D::MAX_CONSTRAINT_ANGLE_DEGS, Deg(0.001).into()) ) 
					{
						let hingeReferenceAxis: Vector3<f32> =  thisBoneJoint.reference_axis_uv;
						
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
							//TODO looking into the difference between normalize and normalized from the caliko impl.
			        		thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, cwConstraintDegs, hingeRotationAxis).normalize();			        		
			        	}
					}
				}
				else if jointType == joint::JointType::LocalHinge
				{	
					// Transform the hinge rotation axis to be relative to the previous bone in the chain
					let hingeRotationAxis: Vector3<f32>  = thisBoneJoint.rotation_axis_uv;
					
					// Construct a rotation matrix based on the previous bone's direction
					let m: Matrix3<f32> = util::createRotationMatrix(prevBoneInnerToOuterUV);
					
					// Transform the hinge rotation axis into the previous bone's frame of reference
					let relativeHingeRotationAxis: Vector3<f32>  = m * (hingeRotationAxis).normalize();
					
					// Project this bone direction onto the plane described by the hinge rotation axis
					// Note: The returned vector is normalised.
					thisBoneInnerToOuterUV = util::projectOntoPlane(&thisBoneInnerToOuterUV, &relativeHingeRotationAxis).unwrap();
					
					// Constrain rotation about reference axis if required
					let cwConstraintDegs: Rad<f32>   = -thisBoneJoint.hinge_clockwise_constraint;
					let acwConstraintDegs: Rad<f32>  =  thisBoneJoint.hinge_anticlockwise_constraint;
					if  !( util::rads_approximatelyEquals(cwConstraintDegs, -joint3D::MAX_CONSTRAINT_ANGLE_DEGS, Deg(0.001).into()) ) &&
						 !( util::rads_approximatelyEquals(acwConstraintDegs, joint3D::MAX_CONSTRAINT_ANGLE_DEGS, Deg(0.001).into()) ) 
					{
						// Calc. the reference axis in local space
						let relativeHingeReferenceAxis: Vector3<f32> = m * ( thisBoneJoint.reference_axis_uv ).normalize();
						
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
				let newEndLocation: Vector3<f32> = thisBone.start + (thisBoneInnerToOuterUV * thisBoneLength );

				// Set the new start joint location for this bone
				thisBone.end = newEndLocation;

				// If we are not working on the end effector bone, then we set the start joint location of the next bone in
				// the chain (i.e. the bone closer to the target) to be the new end joint location of this bone.
				if index < self.m_chain.len() - 1 { 
				  self.m_chain[index+1].start = newEndLocation; 
				}
			}
			else // If we ARE working on the basebone...
			{	
				// If the base location is fixed then snap the start location of the basebone back to the fixed base...
				if self.fixed_base_mode
				{
					thisBone.start = self.base_location;
				}
				else // ...otherwise project it backwards from the end to the start by its length.
				{
					thisBone.start = thisBone.end - ( thisBone.get_direction_uv() * thisBoneLength ) ;
				}
				
				// If the basebone is unconstrained then process it as usual...
				if self.basebone_constraint_type == BaseboneConstraintType3D::None
				{
					// Set the new end location of this bone, and if there are more bones,
					// then set the start location of the next bone to be the end location of this bone
					let newEndLocation: Vector3<f32> = thisBone.start + ( thisBone.get_direction_uv() * thisBoneLength );
					thisBone.end = newEndLocation;	
					
					if self.m_chain.len() > 1 { 
					  self.m_chain[1].start = newEndLocation; 
					}
				}
				else // ...otherwise we must constrain it to the basebone constraint unit vector
				{	
					if self.basebone_constraint_type == BaseboneConstraintType3D::GlobalRotor
					{	
						// Get the inner-to-outer direction of this bone
						let thisBoneInnerToOuterUV: Vector3<f32> = thisBone.get_direction_uv();
								
						let angleBetweenDegs: Rad<f32>   = util::getAngleBetweenRads(self.mBaseboneConstraintUV, thisBoneInnerToOuterUV);
						let constraintAngleDegs: Rad<f32> = thisBone.joint.get_ball_constraint(); 
					
						if angleBetweenDegs > constraintAngleDegs
						{
							thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, self.mBaseboneConstraintUV, constraintAngleDegs);
						}
						
						let newEndLocation: Vector3<f32> = thisBone.start + ( thisBoneInnerToOuterUV * thisBoneLength);
						
						thisBone.end = newEndLocation;
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.m_chain.len() > 1 { 
						  self.m_chain[1].start = newEndLocation; 
						}
					}
					else if self.basebone_constraint_type == BaseboneConstraintType3D::LocalRotor
					{
						// Note: The mBaseboneRelativeConstraintUV is updated in the FabrikStructure3D.solveForTarget()
						// method BEFORE this FabrikChain3D.solveForTarget() method is called. We no knowledge of the
						// direction of the bone we're connected to in another chain and so cannot calculate this 
						// relative basebone constraint direction on our own, but the FabrikStructure3D does it for
						// us so we are now free to use it here.
						
						// Get the inner-to-outer direction of this bone
						let thisBoneInnerToOuterUV: Vector3<f32> = thisBone.get_direction_uv();
								
						// Constrain about the relative basebone constraint unit vector as neccessary
						let angleBetweenDegs: Rad<f32>   = util::getAngleBetweenRads(self.basebone_relative_constraint_uv, thisBoneInnerToOuterUV);
						let constraintAngleDegs: Rad<f32> = thisBone.joint.get_ball_constraint();
						if angleBetweenDegs > constraintAngleDegs
						{
							thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, self.basebone_relative_constraint_uv, constraintAngleDegs);
						}
						
						// Set the end location
						let newEndLocation: Vector3<f32> = thisBone.start + ( thisBoneInnerToOuterUV * thisBoneLength);						
						thisBone.end = newEndLocation;
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.m_chain.len() > 1 { 
						  self.m_chain[1].start = newEndLocation; 
						}
					}
					else if self.basebone_constraint_type == BaseboneConstraintType3D::GlobalHinge
					{
						let thisJoint  =  thisBone.joint;
						let hingeRotationAxis: Vector3<f32>  =  thisJoint.rotation_axis_uv;
						let cwConstraintDegs: Rad<f32>   = -thisJoint.hinge_clockwise_constraint;     // Clockwise rotation is negative!
						let acwConstraintDegs: Rad<f32>  =  thisJoint.hinge_anticlockwise_constraint;
						
						// Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
						let thisBoneInnerToOuterUV: Vector3<f32> =  util::projectOntoPlane(&thisBone.get_direction_uv(), &hingeRotationAxis).unwrap();
								
						// If we have a global hinge which is not freely rotating then we must constrain about the reference axis
						if  !( util::rads_approximatelyEquals(cwConstraintDegs , -joint3D::MAX_CONSTRAINT_ANGLE_DEGS, Deg(0.01).into()) &&
							    util::rads_approximatelyEquals(acwConstraintDegs,  joint3D::MAX_CONSTRAINT_ANGLE_DEGS, Deg(0.01).into()) ) 
						{
							// Grab the hinge reference axis and calculate the current signed angle between it and our bone direction (about the hinge
							// rotation axis). Note: ACW rotation is positive, CW rotation is negative.
							let hingeReferenceAxis: Vector3<f32> = thisJoint.reference_axis_uv;
							let signedAngleDegs: f32 = util::getSignedAngleBetweenDegs(hingeReferenceAxis, thisBoneInnerToOuterUV, hingeRotationAxis);
							
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
						let newEndLocation: Vector3<f32> = thisBone.start +( thisBoneInnerToOuterUV * thisBoneLength );						
						thisBone.end = newEndLocation;
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.m_chain.len() > 1 { 
						  self.m_chain[1].start = newEndLocation; 
						}
					}
					else if self.basebone_constraint_type == BaseboneConstraintType3D::LocalHinge
					{
						let thisJoint  =  thisBone.joint;
						let  hingeRotationAxis: Vector3<f32>  =  self.basebone_relative_constraint_uv;                   // Basebone relative constraint is our hinge rotation axis!
						let  cwConstraintDegs: Rad<f32>   = -thisJoint.hinge_clockwise_constraint;     // Clockwise rotation is negative!
						let  acwConstraintDegs: Rad<f32>  =  thisJoint.hinge_anticlockwise_constraint;
						
						// Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
						let thisBoneInnerToOuterUV: Vector3<f32> = util::projectOntoPlane(&thisBone.get_direction_uv(), &hingeRotationAxis).unwrap();
						
						// If we have a local hinge which is not freely rotating then we must constrain about the reference axis
						if  !( util::rads_approximatelyEquals(cwConstraintDegs , -joint3D::MAX_CONSTRAINT_ANGLE_DEGS, Deg(0.01).into()) &&
							    util::rads_approximatelyEquals(acwConstraintDegs,  joint3D::MAX_CONSTRAINT_ANGLE_DEGS, Deg(0.01).into()) ) 
						{
							// Grab the hinge reference axis and calculate the current signed angle between it and our bone direction (about the hinge
							// rotation axis). Note: ACW rotation is positive, CW rotation is negative.
							let hingeReferenceAxis: Vector3<f32> = self.mBaseboneRelativeReferenceConstraintUV; 
							let signedAngleDegs: f32  =util::getSignedAngleBetweenDegs(hingeReferenceAxis, thisBoneInnerToOuterUV, hingeRotationAxis);
							
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
						let newEndLocation: Vector3<f32> = thisBone.start + ( thisBoneInnerToOuterUV * thisBoneLength);						
						thisBone.end = newEndLocation;
						
						// Also, set the start location of the next bone to be the end location of this bone
						if self.m_chain.len() > 1 { 
						  self.m_chain[1].start = newEndLocation; 
						}
					}
					
				} // End of basebone constraint handling section

			} // End of basebone handling section

		} // End of backward-pass loop over all bones

		// Update our last target location
		self.mLastTargetLocation = target;
				
		// DEBUG - check the live chain length and the originally calculated chain length are the same
		/*
		if (Math.abs( this.getLiveChainLength() - my_chainLength) > 0.01f)
		{
			System.out.println("Chain length off by > 0.01f");
		}
		*/
		
		// Finally, calculate and return the distance between the current effector location and the target.
		return Ok(util::distanceBetween(self.m_chain[self.m_chain.len()-1].end, target));
	}
}