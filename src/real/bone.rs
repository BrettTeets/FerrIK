use super::{joint::Joint, joint, util};
use cgmath::{Vector3, MetricSpace, Rad};

#[derive(Clone, PartialEq, Debug)]
pub struct Bone{
    bone_connection_point: crate::BoneConnectionPoint,
	pub joint: Joint,
	pub start_location: Vector3<f32>,
	pub end_location: Vector3<f32>,
    length: f32,
}

impl Bone{
    pub fn new() -> Self{
        Self{
            bone_connection_point: crate::BoneConnectionPoint::END,
            joint: Joint::new(),
            start_location: Vector3::new(0.0,0.0,0.0),
            end_location: Vector3::new(0.0,0.0,0.0),
            length: 0.0,
        }
    }

    pub fn new_with_vectors(start_location: Vector3<f32>, end_location: Vector3<f32>) -> Self
	{
        Self{
            bone_connection_point: crate::BoneConnectionPoint::END,
            joint: Joint::new(),
            start_location: start_location,
            end_location: end_location,
            length: start_location.distance(end_location),
        }
	}

    pub fn length(&self) -> f32
    {	return self.length;	}

    pub fn live_length(&self) -> f32
    { return self.start_location.distance(self.end_location);	}

    ///Returns a vector showing the length and direction of the bone, with it's base center
    /// at 0,0 for easy repositioning along the chain. add consecutive bones uses it.
    pub fn get_vector_to_end(&self) -> Vector3<f32>{
        let len = self.start_location.distance(self.end_location);
        let dir = self.get_direction();
        util::validateDirectionUV(dir);
        util::validateLength(len);

        return dir * len;
    }

    pub fn set_bone_connection_point(&mut self, bcp: crate::BoneConnectionPoint) 
    { self.bone_connection_point = bcp; }

    pub fn get_bone_connection_point(&self) -> crate::BoneConnectionPoint
     { return self.bone_connection_point; }

    pub fn get_start_location(&self) -> Vector3<f32>
    { return self.start_location; }

    pub fn set_joint(&mut self, joint: Joint) 
    { self.joint.set(joint); }

    pub fn get_joint_type(&self)	-> joint::JointType
    { return self.joint.get_joint_type(); }

    pub fn set_hinge_joint_clockwise_constraint_degs(&mut self, angle: Rad<f32>) 
    {	self.joint.set_hinge_joint_clockwise_constraint(angle);	}

    pub fn get_hinge_joint_clockwise_constraint_degs(&self)	->Rad<f32>
    { return self.joint.get_hinge_clockwise_constraint_degs(); }
	
    pub fn set_hinge_joint_anticlockwise_constraint_degs(&mut self, angle: Rad<f32>) 
    { self.joint.set_hinge_joint_anticlockwise_constraint(angle); }
	
    pub fn get_hinge_joint_anticlockwise_constraint_degs(&self)->Rad<f32>
    { return self.joint.get_hinge_anticlockwise_constraint_degs(); }

    pub fn set_ball_constraint(&mut self, angle: Rad<f32>)
	{	
		if angle < joint::MIN_CONSTRAINT_ANGLE_DEGS || angle > joint::MAX_CONSTRAINT_ANGLE_DEGS
		{
			panic!("Rotor constraints for ball joints must be in the range 0.0f to 180.0f degrees inclusive.");
		}
		
		self.joint.set_ball_joint_constraint_degs(angle);
	}

    pub fn get_ball_constraint(&self) -> Rad<f32>
    { return self.joint.get_ball_joint_constraint();	}

    pub fn get_direction(&self) -> Vector3<f32>
	{
        return util::getDirectionUV(self.start_location, self.end_location);
	}

    pub fn get_global_pitch_degs(&self) -> Rad<f32>
	{
        let d = util::getDirectionUV(self.start_location, self.end_location);
		return util::getGlobalPitchDegs(d);
	}

    pub fn get_global_yaw_degs(&self) -> Rad<f32>
	{
        let d = util::getDirectionUV(self.start_location, self.end_location);
		return  util::getGlobalYawDegs(d);
	}

    pub fn set_start_location(&mut self, location: Vector3<f32>)
	{
		self.start_location = location;
	}

    pub fn set_end_location(&mut self, location: Vector3<f32>)
	{
		self.end_location = location;               
	}
}