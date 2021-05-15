use super::{bone::Bone, joint::Joint, joint::JointType, structure::Structure, util, solver};
use cgmath::{Vector3, Rad, InnerSpace};

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
    pub bones: Vec<Bone>,
    pub solve_distance_threshold: f32,
    pub max_iteration_attempts: usize,
    pub min_iteration_change: f32,
    pub chain_length: f32,
    pub fixed_base_location: Vector3<f32>,
    pub fixed_base_mode: bool,
    pub basebone_constraint_type: BaseboneConstraintType,
    pub basebone_constraint_uv: Vector3<f32>,
    pub basebone_relative_constraint_uv: Vector3<f32>,
    pub basebone_relative_reference_constraint_uv: Vector3<f32>,
    pub last_target_location: Vector3<f32>,
    pub last_base_location: Vector3<f32>,
    pub current_solve_distance: f32,
    pub connected_chain_number: Option<usize>,
    pub connected_bone_number: Option<usize>,
    pub embedded_target: Vector3<f32>,
    pub use_embedded_target: bool,
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
			 util::v_approximatelyEquals(self.last_base_location, self.get_base_location(), 0.001) 
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
			solve_distance = solver::solve_ik(self, new_target);
			
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
		self.last_base_location = self.get_base_location();
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
		let numBones: usize = structure.getChain(chainNumber).get_num_bones();
		if boneNumber > numBones { 
		  panic!("Chain does not contain a bone {} - it has {} bones.", boneNumber,  numBones); 
		}
		
		// All good? Set the connection details
		self.connected_chain_number = Some(chainNumber);
		self.connected_bone_number  = Some(boneNumber);		
	}
}

//Absolutely my favorite thing to override.
use std::ops::Index;
impl Index<usize> for Chain {
    type Output = Bone;

    fn index(&self, i: usize) -> &Self::Output {
        &self.bones[i]
    }
}

use std::ops::IndexMut;
impl IndexMut<usize> for Chain{

	fn index_mut(&mut self, i: usize) -> &mut Self::Output{
		&mut self.bones[i]
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
		self.get_bone(0).getJoint().setAsBallJoint(angle);
	}
    
    pub fn set_hinge_basebone_constraint(&mut self,  hinge_type: BaseboneConstraintType,
        hinge_rotation_axis: Vector3<f32>, cw_constraint_rads: Rad<f32>, acw_constraint_rads: Rad<f32>, hinge_reference_axis: Vector3<f32>)
	{
		// Sanity checking
		if self.bones.is_empty()	{ 
		  panic!("Chain must contain a basebone before we can specify the basebone constraint type."); 
		}		
		if  hinge_rotation_axis.magnitude() <= 0.0   { 
		  panic!("Hinge rotation axis cannot be zero.");
		}
		if hinge_reference_axis.magnitude() <= 0.0  { 
		  panic!("Hinge reference axis cannot be zero.");	
		}
		if  !( util::perpendicular(hinge_rotation_axis, hinge_reference_axis) )  {
			panic!("The hinge reference axis must be in the plane of the hinge rotation axis, that is, they must be perpendicular.");
		}
		if  !(hinge_type == BaseboneConstraintType::GlobalHinge || hinge_type == BaseboneConstraintType::LocalHinge)  {	
			panic!("The only valid hinge types for this method are GLOBAL_HINGE and LOCAL_HINGE.");
		}
		
		// Set the constraint type, axis and angle
		self.basebone_constraint_type = hinge_type;
		self.basebone_constraint_uv = hinge_rotation_axis.normalize();
		
		let mut hinge: Joint = Joint::new();
		
		if hinge_type == BaseboneConstraintType::GlobalHinge
		{
			hinge.setHinge(JointType::GLOBAL_HINGE, hinge_rotation_axis, cw_constraint_rads, acw_constraint_rads, hinge_reference_axis);
		}
		else
		{
			hinge.setHinge(JointType::LOCAL_HINGE, hinge_rotation_axis, cw_constraint_rads, acw_constraint_rads, hinge_reference_axis);
		}
		//TODO: Can this be cleaned up it is probably a common operation.
		self.get_bone(0).setJoint(hinge);
	}

    pub fn set_freely_rotating_global_hinged_basebone(&mut self, hinge_rotation_axis: Vector3<f32>)
	{
		self.set_hinge_basebone_constraint(BaseboneConstraintType::GlobalHinge, hinge_rotation_axis, Rad(3.1415), Rad(3.1415), util::genPerpendicularVectorQuick(hinge_rotation_axis) );
	}

    pub fn set_freely_rotating_local_hinged_basebone(&mut self, hinge_rotation_axis: Vector3<f32>)
	{
		self.set_hinge_basebone_constraint(BaseboneConstraintType::LocalHinge, hinge_rotation_axis, Rad(3.1415), Rad(3.1415), util::genPerpendicularVectorQuick(hinge_rotation_axis) );
	}

    pub fn set_local_hinged_basebone(&mut self, hinge_rotation_axis: Vector3<f32>, cw_rads: Rad<f32>, acw_rads: Rad<f32>, hinge_reference_axis: Vector3<f32>)
	{
		self.set_hinge_basebone_constraint(BaseboneConstraintType::LocalHinge, hinge_rotation_axis, cw_rads, acw_rads, hinge_reference_axis);
	}

    pub fn set_global_hinged_basebone(&mut self, hinge_rotation_axis: Vector3<f32>, cw_rads: Rad<f32>, acw_rads: Rad<f32>, hinge_reference_axis: Vector3<f32>)
	{
		self.set_hinge_basebone_constraint(BaseboneConstraintType::GlobalHinge, hinge_rotation_axis, cw_rads, acw_rads, hinge_reference_axis);
	}

    pub fn set_basebone_constraint_uv(&mut self, constraint_uv: Vector3<f32>)
	{
		if self.basebone_constraint_type == BaseboneConstraintType::None
		{
			panic!("Specify the basebone constraint type with setBaseboneConstraintTypeCannot specify a basebone constraint when the current constraint type is BaseboneConstraint.None.");
		}
		
		// Validate the constraint direction unit vector
		util::validateDirectionUV(constraint_uv);
		
		// All good? Then normalise the constraint direction and set it
		constraint_uv.normalize();
		self.basebone_constraint_uv = constraint_uv;
	}

    pub fn set_base_location(&mut self, base_location: Vector3<f32>) 
    { self.fixed_base_location = base_location; }

    pub fn set_fixed_base_mode(&mut self, value: bool)
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

    pub fn set_max_iteration_attempts(&mut self, max_iterations: usize)
	{
		// Ensure we have a valid maximum number of iteration attempts
		if max_iterations < 1
		{
			panic!("The maximum number of attempts to solve this IK chain must be at least 1.");
		}
		
		// All good? Set the new maximum iteration attempts property
		self.max_iteration_attempts = max_iterations;
	}

    pub fn set_min_iteration_change(&mut self, min_iteration_change: f32)
	{
		// Ensure we have a valid maximum number of iteration attempts
		if self.min_iteration_change < 0.0
		{
			panic!("The minimum iteration change value must be more than or equal to zero.");
		}
		
		// All good? Set the new minimum iteration change distance
		self.min_iteration_change = min_iteration_change;
	}

    pub fn set_solve_distance_threshold(&mut self, solve_distance: f32)
	{
		// Ensure we have a valid solve distance
		if solve_distance < 0.0
		{
			panic!("The solve distance threshold must be greater than or equal to zero.");
		}
		
		// All good? Set the new solve distance threshold
		self.solve_distance_threshold = solve_distance;
	}
}

//getters
impl Chain{
	pub fn get_max_iteration_attempts(&self) -> usize {
		return self.max_iteration_attempts;
	}
	
	pub fn get_min_iteration_change(&self) -> f32 {
		return self.min_iteration_change;
	}
	
	pub fn get_solve_distance_threshold(&self) -> f32 {
		return self.solve_distance_threshold;
	}

    pub fn get_basebone_relative_constraint_uv(&self) -> Vector3<f32>
    { return self.basebone_relative_constraint_uv; }

    pub fn get_basebone_constraint_type(&self) -> BaseboneConstraintType
    { return self.basebone_constraint_type; }

    pub fn get_basebone_constraint_uv(&self) -> Vector3<f32>
	{
		if self.basebone_constraint_type == BaseboneConstraintType::None{
			panic!("Cannot return the basebone constraint when the basebone constraint type is None.");
		}

		return self.basebone_constraint_uv;	
	}

    pub fn get_base_location(&self) -> Vector3<f32> 
    { return self.bones[0].getStartLocation(); }	

    pub fn get_bone(&mut self, bone_number: usize) -> &mut Bone
    { return &mut self.bones[bone_number]; }

    pub fn get_chain(&self) -> &Vec<Bone>
    { return &self.bones; }

    pub fn get_chain_length(&self) -> f32
    { return self.chain_length; }

    pub fn get_connected_bone_number(&self) -> Option<usize>
    { return self.connected_bone_number; }

    pub fn get_connected_chain_number(&self) -> Option<usize> 
    { return self.connected_chain_number; }

    pub fn get_effector_location(&self) -> Vector3<f32> 
    { return self.bones[self.bones.len()-1].getEndLocation(); }

    pub fn get_embedded_target_mode(&self) -> bool
    { return self.use_embedded_target; }

    pub fn get_embedded_target(&self) -> Vector3<f32>  
    { return self.embedded_target; }

    pub fn get_last_target_location(&self) -> Vector3<f32>   
    { return self.last_target_location; }

    pub fn get_live_chain_length(&self) -> f32
	{
		let mut length: f32 = 0.0;		
		for bone in self.bones.iter()
		{  
			length += bone.liveLength();
		}		
		return length;
	}	

    pub fn get_num_bones(&self) -> usize 
    { return self.bones.len(); }

    pub fn get_basebone_relative_reference_constraint_uv(&self) -> Vector3<f32> 
    { return self.basebone_relative_reference_constraint_uv;}

    
	
}

//used by solver to query the chain in a readable way.
impl Chain{

	pub fn forward_pass_index_iter(&self) -> std::iter::Rev<std::ops::Range<usize>>{
		return (0..self.bones.len()).rev();
	}

	pub fn is_not_end_effector(&self, index: usize) -> bool{
		return index != self.bones.len() - 1;
	}

	pub fn backwards_pass_index_iter(&self) -> std::ops::Range<usize>{
		return 0..self.bones.len();
	}

	pub fn is_not_basebone(&self, index: usize) -> bool {
		return index != 0;
	}
} //line 943