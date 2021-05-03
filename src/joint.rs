pub trait Joint {
    //this was all that was in the original, though it was a generic there.
    fn set(&self, sourceJoint: Joint3D);
}

pub struct Joint3D{
    //limits in degrees of how far things can rotate, they were defaulted to 180 in caliko.
    rotor_constraint: cgmath::Deg<f32>,
    hinge_clockwise_constraint: cgmath::Deg<f32>,
    hinge_anticlockwise_constraint: cgmath::Deg<f32>,

    //The unit vector axis about which a hinged joint may rotate.
	rotation_axis_uv: cgmath::Vector3<f32>,
	
	// For a hinged joint, this is the axis used as a point of reference for rotation (it is NOT the axis about which the hinge rotates). */
	reference_axis_uv: cgmath::Vector3<f32>,

    joint_type: JointType,
}

impl Joint3D{
    pub fn SetAsBallJoint(&mut self, constraint_degrees: cgmath::Deg<f32>){

        //TODO Check if constrained degrees are within 180.

        self.rotor_constraint = constraint_degrees;
        self.joint_type = JointType::Ball;
    }

    pub fn set_as_global_hing(&mut self,  rotation_axis: cgmath::Vector3<f32>, reference_axis: cgmath::Vector3<f32>, clockwise_constraint: cgmath::Deg<f32>, anticlockwise_constraint: cgmath::Deg<f32>){
        self.set_hinge(JointType::GlobalHinge, rotation_axis, reference_axis, clockwise_constraint, anticlockwise_constraint);
    }

    pub fn set_as_local_hing(&mut self,  rotation_axis: cgmath::Vector3<f32>, reference_axis: cgmath::Vector3<f32>, clockwise_constraint: cgmath::Deg<f32>, anticlockwise_constraint: cgmath::Deg<f32>){
        self.set_hinge(JointType::LocalHinge, rotation_axis, reference_axis, clockwise_constraint, anticlockwise_constraint);
    }

    pub fn set_hinge(&mut self, joint_type: JointType, rotation_axis: cgmath::Vector3<f32>, reference_axis: cgmath::Vector3<f32>,
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
}


impl Joint for Joint3D{
    //I do not understand the point of this function, but it was in caliko. I will probably move set to 
    //bone when I am done copying everything over. TODO.
    fn set(&self, sourceJoint: Joint3D){

    }
}

//This was pulled over directly from the caliko version.
pub enum JointType {
    Ball,
    GlobalHinge,
    LocalHinge,
}