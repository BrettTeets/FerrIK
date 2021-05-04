use cgmath::{Vector3, InnerSpace};
use crate::{bone3D, joint3D, chain, chain::FerrikErrors};
use log::{error};

#[derive(Clone)]
pub struct Chain3D{
    id: usize,
    m_chain: Vec<bone3D::Bone3D>,
    solve_distance_threshold: f32, //this was set to 1.0 as the default.
    mMaxIterationAttempts: usize, //default was 20
    mMinIterationChange: f32, //default was 0.01,
    mChainLength: usize, //this is recalculated everytime a bone is added or removed
    mFixedBaseLocation: Vector3<f32>, //start location of the root bone.
    mFixedBaseMode: bool, //whether the Base can be moved by the solver.
    mBaseboneConstraintType: BaseboneConstraintType3D,
    mBaseboneConstraintUV: Vector3<f32>, //this is the direction the base bone should be constrained around
    //since it has no prior bone to ask.
    mBaseboneRelativeConstraintUV: Vector3<f32>, //used if this bone is attached to another bone in another chain.
    mBaseboneRelativeReferenceConstraintUV: Vector3<f32>, //honestly their just making stuff up now.
    mLastTargetLocation: Vector3<f32>, //this defaulted to a maxed out vector3(f32.max, f32.max f32.max).
    mLastBaseLocation: Vector3<f32>, //aslo set to max by default
    mCurrentSolveDistance: f32, //set to max.
    mConnectedChainNumber: usize, //index in the structure that this chain was connected to. Might make this an option as it was using -1 to indicate no.
    mConnectedBoneNumber: usize, //index of the bone in the connected chain. //might make option.
    mEmbeddedTarget: Vector3<f32>,
    mUseEmbeddedTarget: bool, //toggle


}

impl Chain3D{
    pub fn new()->Self{
        
    }

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
}

pub enum BaseboneConstraintType3D {
    NONE,         // No constraint - basebone may rotate freely
    GLOBAL_ROTOR, // World-space rotor constraint
    LOCAL_ROTOR,  // Rotor constraint in the coordinate space of (i.e. relative to) the direction of the connected bone
    GLOBAL_HINGE, // World-space hinge constraint
    LOCAL_HINGE   // Hinge constraint in the coordinate space of (i.e. relative to) the direction of the connected bone
}




