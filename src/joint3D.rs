use cgmath::{Vector3, Deg, InnerSpace};
use crate::{joint};

const MIN_CONSTRAINT_ANGLE_DEGS: cgmath::Deg<f32> = cgmath::Deg(0.0); //fully constrained
const MAX_CONSTRAINT_ANGLE_DEGS: cgmath::Deg<f32> = cgmath::Deg(180.0); //unconstrained.

#[derive(Clone)]
pub struct Joint3D{
    joint_type: joint::JointType,
    rotor_constraint: cgmath::Deg<f32>,
    hinge_clockwise_constraint: cgmath::Deg<f32>,
    hinge_anticlockwise_constraint: cgmath::Deg<f32>,
    //The unit vector axis about which a hinged joint may rotate.
	rotation_axis_uv: cgmath::Vector3<f32>,
	// For a hinged joint, this is the axis used as a point of reference for rotation (it is NOT the axis about which the hinge rotates). */
	reference_axis_uv: cgmath::Vector3<f32>,
}

impl Joint3D{
    pub fn new() -> Self{
        Self{
            joint_type: joint::JointType::Ball,
            rotor_constraint: MAX_CONSTRAINT_ANGLE_DEGS,
            hinge_clockwise_constraint: MAX_CONSTRAINT_ANGLE_DEGS,
            hinge_anticlockwise_constraint: MAX_CONSTRAINT_ANGLE_DEGS,
            rotation_axis_uv: cgmath::Vector3::unit_z(), //spins around the x.
            reference_axis_uv: cgmath::Vector3::unit_x() //is perpandicular to to rotation? I think.
        }
    }

    pub fn copy_existing_joint(&mut self, source: Self){
        //do stuff to do this,
    }

    pub fn set_as_ball_joint(&mut self, constraint_degrees: cgmath::Deg<f32>){

        //TODO Check if constrained degrees are within 180.

        self.rotor_constraint = constraint_degrees;
        self.joint_type = joint::JointType::Ball;
    }

    pub fn set_as_global_hing(&mut self,  rotation_axis: cgmath::Vector3<f32>, reference_axis: cgmath::Vector3<f32>, clockwise_constraint: cgmath::Deg<f32>, anticlockwise_constraint: cgmath::Deg<f32>){
        self.set_hinge(joint::JointType::GlobalHinge, rotation_axis, reference_axis, clockwise_constraint, anticlockwise_constraint);
    }

    pub fn set_as_local_hing(&mut self,  rotation_axis: cgmath::Vector3<f32>, reference_axis: cgmath::Vector3<f32>, clockwise_constraint: cgmath::Deg<f32>, anticlockwise_constraint: cgmath::Deg<f32>){
        self.set_hinge(joint::JointType::LocalHinge, rotation_axis, reference_axis, clockwise_constraint, anticlockwise_constraint);
    }

    pub fn set_hinge(&mut self, joint_type: joint::JointType, rotation_axis: cgmath::Vector3<f32>, reference_axis: cgmath::Vector3<f32>,
        clockwise_constraint: cgmath::Deg<f32>,  anticlockwise_constraint: cgmath::Deg<f32>){
        //Check to make sure that the reference is on the same plane as the rotation axis
        //they would need to be perpendicular so that their dot product was zero. TODO.
		
		// Validate the constraint angles to be within the valid range and the axis isn't zero
        //Check to make sure the constraints are within reason and the axis arent zero. TODO.
        
		
		// Set params
        self.hinge_clockwise_constraint = clockwise_constraint;
        self.hinge_anticlockwise_constraint = anticlockwise_constraint;
        self.joint_type = joint_type;
        self.rotation_axis_uv = rotation_axis;
        self.reference_axis_uv = reference_axis;
	}

    pub fn get_hinge_clockwise_and_anticlockwise_constraints(&self) -> (cgmath::Deg<f32>, cgmath::Deg<f32>){
        //check if this is a hinge joint, cause who ever is calling this is assuming it is.

        return (self.hinge_clockwise_constraint, self.hinge_anticlockwise_constraint);
    }

    pub fn get_ball_constraint(&self) -> cgmath::Deg<f32>{
        //check if this is a ball joint,

        return self.rotor_constraint;
    }

    fn validateConstraintAngleDegs(angleDegs: cgmath::Deg<f32>)
	{
		if angleDegs < MIN_CONSTRAINT_ANGLE_DEGS || angleDegs > MAX_CONSTRAINT_ANGLE_DEGS
		{
			panic!("Constraint angles must be within the range {:?} to {:?} inclusive.", MIN_CONSTRAINT_ANGLE_DEGS, MAX_CONSTRAINT_ANGLE_DEGS);
		}
	}
	
	fn validateAxis(axis: Vector3<f32>)
	{
		if  axis.magnitude() <= 0.0 
		{
			panic!("Provided axis is illegal - it has a magnitude of zero.");
		}
	}

    //so the original had a several constraint setters that also checked that this was the correct
    //type of joint for what was being assumed. I am currently leaving those out. idkw. I feel like
    //rust will get angry at me so I am just going to leave them public to the crate and let the
    //solver proportion work with the directly. I can work them into the developer facing API later.
}

impl joint::Joint for Joint3D{

}

	
	
	
	