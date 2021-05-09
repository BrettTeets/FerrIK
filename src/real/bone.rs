use super::{joint::Joint, joint, util};
use cgmath::{Vector3, MetricSpace, Rad};

#[derive(Clone, PartialEq, Debug)]
pub struct Bone{
    mBoneConnectionPoint: crate::BoneConnectionPoint,
	mJoint: Joint,
	mStartLocation: Vector3<f32>,
	mEndLocation: Vector3<f32>,
    mLength: f32,
}

impl Bone{
    pub fn new() -> Self{
        Self{
            mBoneConnectionPoint: crate::BoneConnectionPoint::END,
            mJoint: Joint::new(),
            mStartLocation: Vector3::new(0.0,0.0,0.0),
            mEndLocation: Vector3::new(0.0,0.0,0.0),
            mLength: 0.0,
        }
    }

    pub fn new_with_vectors(startLocation: Vector3<f32>, endLocation: Vector3<f32>) -> Self
	{
        Self{
            mBoneConnectionPoint: crate::BoneConnectionPoint::END,
            mJoint: Joint::new(),
            mStartLocation: startLocation,
            mEndLocation: endLocation,
            mLength: startLocation.distance(endLocation),
        }
	}

    pub fn length(&self) -> f32
    {	return self.mLength;	}

    pub fn liveLength(&self) -> f32
    { return self.mStartLocation.distance(self.mEndLocation);	}

    pub fn setBoneConnectionPoint(&mut self, bcp: crate::BoneConnectionPoint) 
    { self.mBoneConnectionPoint = bcp; }

    pub fn getBoneConnectionPoint(&self) -> crate::BoneConnectionPoint
     { return self.mBoneConnectionPoint; }

    pub fn getStartLocation(&self) -> Vector3<f32>
    { return self.mStartLocation; }

    pub fn getEndLocation(&self) -> Vector3<f32>
    { return self.mEndLocation; }

    pub fn setJoint(&mut self, joint: Joint) 
    { self.mJoint.set(joint); }

    //Should joint be ina box as a pointer?
    pub fn getJoint(&self)	-> Joint
    { return self.mJoint; }

    pub fn getJointType(&self)	-> joint::JointType
    { return self.mJoint.getJointType(); }

    pub fn setHingeJointClockwiseConstraintDegs(&mut self, angleDegs: Rad<f32>) 
    {	self.mJoint.setHingeJointClockwiseConstraintDegs(angleDegs);	}

    pub fn getHingeJointClockwiseConstraintDegs(&self)	->Rad<f32>
    { return self.mJoint.getHingeClockwiseConstraintDegs(); }
	
    pub fn setHingeJointAnticlockwiseConstraintDegs(&mut self, angleDegs: Rad<f32>) 
    { self.mJoint.setHingeJointAnticlockwiseConstraintDegs(angleDegs); }
	
    pub fn getHingeJointAnticlockwiseConstraintDegs(&self)->Rad<f32>
    { return self.mJoint.getHingeAnticlockwiseConstraintDegs(); }

    pub fn setBallJointConstraintDegs(&mut self, angleDegs: Rad<f32>)
	{	
		if angleDegs < joint::MIN_CONSTRAINT_ANGLE_DEGS || angleDegs > joint::MAX_CONSTRAINT_ANGLE_DEGS
		{
			panic!("Rotor constraints for ball joints must be in the range 0.0f to 180.0f degrees inclusive.");
		}
		
		self.mJoint.setBallJointConstraintDegs(angleDegs);
	}

    pub fn getBallJointConstraintDegs(&self) -> Rad<f32>
    { return self.mJoint.getBallJointConstraintDegs();	}

    pub fn getDirectionUV(&self) -> Vector3<f32>
	{
        return util::getDirectionUV(self.mStartLocation, self.mEndLocation);
	}

    pub fn getGlobalPitchDegs(&self) -> Rad<f32>
	{
        let d = util::getDirectionUV(self.mStartLocation, self.mEndLocation);
		return util::getGlobalPitchDegs(d);
	}

    pub fn getGlobalYawDegs(&self) -> Rad<f32>
	{
        let d = util::getDirectionUV(self.mStartLocation, self.mEndLocation);
		return  util::getGlobalYawDegs(d);
	}

    pub fn setStartLocation(&mut self, location: Vector3<f32>)
	{
		self.mStartLocation = location;
	}

    pub fn setEndLocation(&mut self, location: Vector3<f32>)
	{
		self.mEndLocation = location;               
	}

    fn setLength(&mut self, length: f32)
	{
		if length > 0.0
		{
			self.mLength = length;
		}
		else
		{
			panic!("Bone length must be a positive value.");
		}
	}
	
}