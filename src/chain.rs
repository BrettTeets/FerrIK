use crate::{bone, bone3D, joint};
use cgmath::Vector3;

pub trait Chain{
    fn add_bone(&mut self, bone: bone3D::Bone3D);

    fn add_consecutive_bone_by_vector(&mut self, direction: cgmath::Vector3<f32>, length: f32);

    fn add_consecutive_bone_with_bone(&mut self, bone: dyn bone::Bone);	

    //I think this wants to return joint type, it was generic in the original.
    fn get_base_bone_constraint_type(&self ) -> joint::JointType;
	
    //This should return the direction of the base bone?
	fn get_base_bone_constraint_vector(&self) -> Vector3<f32>;
	
	//I dont know what this is returning but it has something to do with multiple chains and structures.
	fn get_basebone_relative_constraint_uv(&self) -> Vector3<f32>;	
	
	//Gets the base or root location of the chain
	fn get_base_location(&self) -> Vector3<f32>;
	
	//returns the bone by its index in the chain.
    //what is a dyn?
	fn get_bone(&self, bone_number: usize) -> dyn bone::Bone;
	
	//returns the actual chain as a list of bones. rust doesn't like this one for some reason
    //look into this one. TODO:
    //Thinking about this some more, maybe an iterator would be better?
    //rust didn't like that either.
	//fn getChain() -> std::slice::Iter<bone::Bone>;
	
	//returns the length of this chain.
	fn get_chain_length(&self) -> f32;
	
    //This is what was original said about this one. Take a gander.
    //noted some stuff below.
	    //Return the index of the bone in another chain that this this chain is connected to.
	    //Returns -1 (default) if this chain is not connected to another chain.
	fn get_connected_bone_number(&self) -> usize;
	
    //this one is a little more straight forward, it is the index of another chain that this one
    //is connected to as it sits in the structure. Saying that it probably refers to the parent chain
    //that this chain is attached to. Which means the above is the index of the bone in the parent chain
    //that they are connected to.
	    //Return the index of the chain in a FabrikStructure that this this chain is connected to.
	    //Returns -1 (default) if this chain is not connected to another chain.
	fn get_connected_chain_number(&self) -> usize;
	
    //gets the position end effector this chain is trying to reach for.
	fn find_effector_location(&self) -> Result<Vector3<f32>, FerrikErrors>;
	
    //what is an embedded target mode?
		//fn get_embedded_target_mode(&self) -> bool;	
		//This should be handle elsewhere in the api
	
	//gets the position of an embedded target?
	fn get_embedded_target(&self) -> Vector3<f32>;
	
    //returns the target of the last solve attempt. Is this per pass, or per solution? 
	fn get_last_target_location(&self) -> Vector3<f32>;
	
    //the number of attempts that will be made to solve this.
	fn get_max_iteration_attempts(&self) -> usize;
	
    //This is the minimum amount of change that must happen between passes before the solver aborts.
    //if the distances change by less that this amount the process aborts to prevent wasted time if the
    //solver has stalled.
	fn get_min_iteration_change(&self) -> f32;
	
	//distance that counts as solved.
	fn get_solve_distance_threshold(&self) -> f32;		
	
	//returns the id of this chain, the original caliko version had a string name but I didn't want
    //to mess with rust strings for this.
	fn get_id(&self) -> usize;
	
	//how many bones are in this chain.
	fn get_num_bones(&self) -> usize;
	
    //removes a bone at this index.
		//fn remove_bone(&mut self, bone_number: usize);
		//I would rather you use the rebuild method to just make a new structure.
	
    //sets a direction constraint for the base point. I guess so shoulders dont invert into themselves.
	fn set_basebone_constraint_uv(&mut self, constraint_uv: Vector3<f32>) -> Result<(), FerrikErrors>;
	
    //Sets the base location of this chain? How does that differ from the root bone?
		//fn set_base_location(&mut self, base_location: Vector3<f32>);	
		//removing it it doesn't do anything but directly set the value.
	
    //toggles embedded target mode.
		//fn set_embedded_target_mode(&mut self, value: bool);
	
    //toggles whether the base bone is fixed in position or can be dragged along.
	fn set_fixed_base_mode(&mut self, value: bool) -> Result<(), FerrikErrors>;	
	
	//This is how many attempts will be made. Is this multiples of two to get the root back to start?
    //wait is not being multiples of two how this drags thie point along?
	fn set_max_iteration_attempts(&mut self, max_iterations: usize)  -> Result<(), FerrikErrors>;
	
    //sets the minimum change between the end point and end effector before it gets called a stall.
	fn set_min_iteration_change(&mut self, min_iteration_change: f32) -> Result<(), FerrikErrors>;
	
    //sets the distance that counts as close enough to be solved.
	fn set_solve_distance_threshold(&mut self, solve_distance: f32) -> Result<(), FerrikErrors>;	
	
	//this was set name in the original. I am changing it to set ID.
    // I also dont want people to change the id after creation as that would break the API I
    //have in mind. so this will be commented out for now.
	//fn set_id(&mut self, id: usize);	
	
    //returns the distance after trying to solve between the end end and the target.
	fn solve_for_embedded_target(&mut self) -> Result<f32, FerrikErrors>;
	
    //Will attempt to solve for a given target and return the distance.
	fn solve_for_target(&mut self, new_target: Vector3<f32>) -> f32;
	
    //Updates the position of the embedded target.
	fn update_embedded_target(&mut self, new_embedded_target: Vector3<f32>);	
	
    //this calculates the length of the bone which is accessed by another method if you need it.
	fn update_chain_length(&mut self);	
}

pub enum FerrikErrors{
	UnsolvableRequirement,
	MultipleChainIssues,
	ImpossibleHappenstance,
	InvalidMethodCall,
}