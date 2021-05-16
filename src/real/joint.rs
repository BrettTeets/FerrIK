use cgmath::{Rad, Vector3, InnerSpace};
use super::{util};

pub const MIN_CONSTRAINT_ANGLE_DEGS: Rad<f32> = Rad(0.0);
pub const MAX_CONSTRAINT_ANGLE_DEGS: Rad<f32> = Rad(3.14159); //This should be 180 degrees in radians.

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum JointType
{
    Ball,
    GlobalHinge,
    LocalHinge
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Joint{
    rotor_constraint: Rad<f32>,
    hinge_clockwise_constraint: Rad<f32>,
    hinge_anticlockwise_constraint: Rad<f32>,
	rotation_axis: Vector3<f32>,
	reference_axis: Vector3<f32>,
    joint_type: JointType, 
}

impl Joint{
    pub fn new() -> Self{
        Self{
            rotor_constraint: MAX_CONSTRAINT_ANGLE_DEGS,
            hinge_clockwise_constraint: MAX_CONSTRAINT_ANGLE_DEGS,  
            hinge_anticlockwise_constraint:  MAX_CONSTRAINT_ANGLE_DEGS,    
            rotation_axis:  Vector3::new(0.0, 0.0, 0.0),    
            reference_axis:  Vector3::new(0.0, 0.0, 0.0),
            joint_type: JointType::Ball,
        }
    }

    pub fn set(&mut self, source: Joint)
	{
            self.rotor_constraint = source.rotor_constraint;
            self.hinge_clockwise_constraint =  source.hinge_clockwise_constraint; 
            self.hinge_anticlockwise_constraint = source.hinge_anticlockwise_constraint;    
            self.rotation_axis =  source.rotation_axis;    
            self.reference_axis =  source.reference_axis;
            self.joint_type = source.joint_type;
	}

    pub fn set_as_ball_joint(&mut self, constraint_angle: Rad<f32>)
	{
		// Throw a RuntimeException if the rotor constraint angle is outside the range 0 to 180 degrees
		Joint::validate_constraint_angle_degs(constraint_angle);
				
		// Set the rotor constraint angle and the joint type to be BALL.
		self.rotor_constraint = constraint_angle;		
		self.joint_type = JointType::Ball;
	}

    pub fn set_hinge(&mut self, joint_type: JointType , rotation_axis: Vector3<f32>, 
        clockwise_constraint_degs: Rad<f32>, anticlockwise_constraint: Rad<f32>, reference_axis: Vector3<f32>)
	{
		// Ensure the reference axis falls within the plane of the rotation axis (i.e. they are perpendicular, so their dot product is zero)		
		if  !util::approximately_equals( rotation_axis.dot(reference_axis), 0.0, 0.01) 
		{
			//float angleDegs = Vec3f.getAngleBetweenDegs(rotationAxis, referenceAxis);
			panic!("The reference axis must be in the plane of the hinge rotation axis - angle between them is currently: " /*+ angleDegs*/);
		}
		
		// Validate the constraint angles to be within the valid range and the axis isn't zero
		Joint::validate_constraint_angle_degs(clockwise_constraint_degs);
		Joint::validate_constraint_angle_degs(anticlockwise_constraint);
		Joint::validate_axis(rotation_axis);
		Joint::validate_axis(reference_axis);
		
		// Set params
		self.hinge_clockwise_constraint     = clockwise_constraint_degs;
		self.hinge_anticlockwise_constraint = anticlockwise_constraint;
		self.joint_type                        = joint_type;
		self.rotation_axis = rotation_axis.normalize();
		self.reference_axis = reference_axis.normalize();
	}

    pub fn set_as_global_hinge(&mut self, global_rotation_axis: Vector3<f32>, 
        cw_constraint: Rad<f32>, acw_constraint: Rad<f32>, global_reference_axis: Vector3<f32>)
	{
		self.set_hinge(JointType::GlobalHinge, global_rotation_axis, cw_constraint, acw_constraint, global_reference_axis);
	}

    pub fn set_as_local_hinge(&mut self, local_rotation_axis: Vector3<f32>, 
        cw_constraint: Rad<f32>, acw_constraint: Rad<f32>, local_reference_axis: Vector3<f32>)
	{
		self.set_hinge(JointType::LocalHinge, local_rotation_axis, cw_constraint, acw_constraint, local_reference_axis);
	}

    pub fn get_hinge_clockwise_constraint_degs(&self) -> Rad<f32>
	{
		if  self.joint_type != JointType::Ball 
		{
			return self.hinge_clockwise_constraint;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have hinge constraint angles.");
		}		
	}

    pub fn get_hinge_anticlockwise_constraint_degs(&self) -> Rad<f32>
	{
		if  self.joint_type != JointType::Ball 
		{
			return self.hinge_anticlockwise_constraint;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have hinge constraint angles.");
		}
	}

    pub fn set_ball_joint_constraint_degs(&mut self, angle: Rad<f32>)
	{
		Joint::validate_constraint_angle_degs(angle);
		
		if self.joint_type == JointType::Ball
		{
			self.rotor_constraint = angle;
		}
		else
		{
			panic!("This joint is of type: {:?} - only joints of type JointType.BALL have a ball joint constraint angle.", self.joint_type);
             
        }
	}

    pub fn get_ball_joint_constraint(&self) -> Rad<f32>
	{
		if self.joint_type == JointType::Ball
		{
			return self.rotor_constraint;
		}
		else
		{
			panic!("This joint is not of type JointType.BALL - it does not have a ball joint constraint angle.");
		}
	}

    pub fn set_hinge_joint_clockwise_constraint(&mut self, angle: Rad<f32>) 
	{
		Joint::validate_constraint_angle_degs(angle);
		
		if self.joint_type != JointType::Ball 
		{
			self.hinge_clockwise_constraint = angle;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have hinge constraint angles.");
		}
	}

    pub fn set_hinge_joint_anticlockwise_constraint(&mut self, angle: Rad<f32>)
	{
		Joint::validate_constraint_angle_degs(angle);
		
		if self.joint_type != JointType::Ball 
		{
			self.hinge_anticlockwise_constraint = angle;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have hinge constraint angles.");
		}
	}

    pub fn  set_hinge_rotation_axis(&mut self, axis: Vector3<f32>)
	{
		Joint::validate_axis(axis);
		
		if self.joint_type != JointType::Ball 
		{
			self.rotation_axis = axis.normalize();
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have a hinge rotation axis.");
		}
	}

    pub fn get_hinge_reference_axis(&self) -> Vector3<f32>
	{	
		if self.joint_type != JointType::Ball
		{
			return self.reference_axis;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have a hinge reference axis.");
		}
	}

   pub fn set_hinge_reference_axis(&mut self, reference_axis: Vector3<f32>)
	{
        Joint::validate_axis(reference_axis);
		
		if self.joint_type != JointType::Ball 
		{
			self.reference_axis = reference_axis.normalize();
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have a hinge reference axis.");
		}
	}

    pub fn get_hinge_rotation_axis(&self) -> Vector3<f32>
	{	
		if self.joint_type != JointType::Ball
		{
			return self.rotation_axis;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have a hinge rotation axis.");
		}
	}

    pub fn get_joint_type(&self) -> JointType
    { 
        return self.joint_type; 
    }

    pub fn validate_constraint_angle_degs(angle: Rad<f32>)
	{
		if angle < MIN_CONSTRAINT_ANGLE_DEGS || angle > MAX_CONSTRAINT_ANGLE_DEGS
		{
			panic!("Constraint angles must be within the range {:?} to {:?} inclusive.", MIN_CONSTRAINT_ANGLE_DEGS, MAX_CONSTRAINT_ANGLE_DEGS); 
            
		}
	}
	
	pub fn validate_axis(axis: Vector3<f32>)
	{
		if axis.magnitude() <= 0.0
		{
			panic!("Provided axis is illegal - it has a magnitude of zero.");
		}
	}


   
}