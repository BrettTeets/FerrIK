use cgmath::{Vector3, InnerSpace};
use crate::{bone3D, joint3D, chain, chain::FerrikErrors};
use log::{error};

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
    mBaseboneConstraintType: BaseboneConstraintType3D,
    mBaseboneConstraintUV: Vector3<f32>, //this is the direction the base bone should be constrained around
    //since it has no prior bone to ask.
    mBaseboneRelativeConstraintUV: Vector3<f32>, //used if this bone is attached to another bone in another chain.
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
  

}

impl chain::Chain for Chain3D{

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
		if self.mBaseboneConstraintType == BaseboneConstraintType3D::GlobalRotor && !value
		{
			error!("Cannot set a non-fixed base mode when the chain's constraint type is BaseboneConstraintType3D.GLOBAL_ABSOLUTE_ROTOR.");
			return Err(FerrikErrors::UnsolvableRequirement)
		}
		
		self.fixed_base_mode = value;
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




