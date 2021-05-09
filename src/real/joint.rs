use cgmath::{Rad, Vector3, InnerSpace};
use super::{util};

pub const MIN_CONSTRAINT_ANGLE_DEGS: Rad<f32> = Rad(0.0);
pub const MAX_CONSTRAINT_ANGLE_DEGS: Rad<f32> = Rad(3.14159); //This should be 180 degrees in radians.

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum JointType
{
    BALL,
    GLOBAL_HINGE,
    LOCAL_HINGE
}

pub struct Joint{
    mRotorConstraintDegs: Rad<f32>,
    mHingeClockwiseConstraintDegs: Rad<f32>,
    mHingeAnticlockwiseConstraintDegs: Rad<f32>,
	mRotationAxisUV: Vector3<f32>,
	mReferenceAxisUV: Vector3<f32>,
    mJointType: JointType, 
}

impl Joint{
    pub fn new() -> Self{
        Self{
            mRotorConstraintDegs: MAX_CONSTRAINT_ANGLE_DEGS,
            mHingeClockwiseConstraintDegs: MAX_CONSTRAINT_ANGLE_DEGS,  
            mHingeAnticlockwiseConstraintDegs:  MAX_CONSTRAINT_ANGLE_DEGS,    
            mRotationAxisUV:  Vector3::new(0.0, 0.0, 0.0),    
            mReferenceAxisUV:  Vector3::new(0.0, 0.0, 0.0),
            mJointType: JointType::BALL,
        }
    }

    pub fn set(&mut self, source: Joint)
	{
            self.mRotorConstraintDegs = source.mRotorConstraintDegs;
            self.mHingeClockwiseConstraintDegs =  source.mHingeClockwiseConstraintDegs; 
            self.mHingeAnticlockwiseConstraintDegs = source.mHingeAnticlockwiseConstraintDegs;    
            self.mRotationAxisUV =  source.mRotationAxisUV;    
            self.mReferenceAxisUV =  source.mReferenceAxisUV;
            self.mJointType = source.mJointType;
	}

    pub fn setAsBallJoint(&mut self, constraintAngleDegs: Rad<f32>)
	{
		// Throw a RuntimeException if the rotor constraint angle is outside the range 0 to 180 degrees
		Joint::validateConstraintAngleDegs(constraintAngleDegs);
				
		// Set the rotor constraint angle and the joint type to be BALL.
		self.mRotorConstraintDegs = constraintAngleDegs;		
		self.mJointType = JointType::BALL;
	}

    pub fn setHinge(&mut self, jointType: JointType , rotationAxis: Vector3<f32>, 
        clockwiseConstraintDegs: Rad<f32>, anticlockwiseConstraintDegs: Rad<f32>, referenceAxis: Vector3<f32>)
	{
		// Ensure the reference axis falls within the plane of the rotation axis (i.e. they are perpendicular, so their dot product is zero)		
		if  !util::approximatelyEquals( rotationAxis.dot(referenceAxis), 0.0, 0.01) 
		{
			//float angleDegs = Vec3f.getAngleBetweenDegs(rotationAxis, referenceAxis);
			panic!("The reference axis must be in the plane of the hinge rotation axis - angle between them is currently: " /*+ angleDegs*/);
		}
		
		// Validate the constraint angles to be within the valid range and the axis isn't zero
		Joint::validateConstraintAngleDegs(clockwiseConstraintDegs);
		Joint::validateConstraintAngleDegs(anticlockwiseConstraintDegs);
		Joint::validateAxis(rotationAxis);
		Joint::validateAxis(referenceAxis);
		
		// Set params
		self.mHingeClockwiseConstraintDegs     = clockwiseConstraintDegs;
		self.mHingeAnticlockwiseConstraintDegs = anticlockwiseConstraintDegs;
		self.mJointType                        = jointType;
		self.mRotationAxisUV = rotationAxis.normalize();
		self.mReferenceAxisUV = referenceAxis.normalize();
	}

    pub fn setAsGlobalHinge(&mut self, globalRotationAxis: Vector3<f32>, 
        cwConstraintDegs: Rad<f32>, acwConstraintDegs: Rad<f32>, globalReferenceAxis: Vector3<f32>)
	{
		self.setHinge(JointType::GLOBAL_HINGE, globalRotationAxis, cwConstraintDegs, acwConstraintDegs, globalReferenceAxis);
	}

    pub fn setAsLocalHinge(&mut self, localRotationAxis: Vector3<f32>, 
        cwConstraintDegs: Rad<f32>, acwConstraintDegs: Rad<f32>, localReferenceAxis: Vector3<f32>)
	{
		self.setHinge(JointType::LOCAL_HINGE, localRotationAxis, cwConstraintDegs, acwConstraintDegs, localReferenceAxis);
	}

    pub fn getHingeClockwiseConstraintDegs(&self) -> Rad<f32>
	{
		if  self.mJointType != JointType::BALL 
		{
			return self.mHingeClockwiseConstraintDegs;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have hinge constraint angles.");
		}		
	}

    pub fn getHingeAnticlockwiseConstraintDegs(&self) -> Rad<f32>
	{
		if  self.mJointType != JointType::BALL 
		{
			return self.mHingeAnticlockwiseConstraintDegs;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have hinge constraint angles.");
		}
	}

    pub fn setBallJointConstraintDegs(&mut self, angleDegs: Rad<f32>)
	{
		Joint::validateConstraintAngleDegs(angleDegs);
		
		if self.mJointType == JointType::BALL
		{
			self.mRotorConstraintDegs = angleDegs;
		}
		else
		{
			panic!("This joint is of type: {:?} - only joints of type JointType.BALL have a ball joint constraint angle.", self.mJointType);
             
        }
	}

    pub fn getBallJointConstraintDegs(&self) -> Rad<f32>
	{
		if self.mJointType == JointType::BALL
		{
			return self.mRotorConstraintDegs;
		}
		else
		{
			panic!("This joint is not of type JointType.BALL - it does not have a ball joint constraint angle.");
		}
	}

    pub fn setHingeJointClockwiseConstraintDegs(&mut self, angleDegs: Rad<f32>) 
	{
		Joint::validateConstraintAngleDegs(angleDegs);
		
		if self.mJointType != JointType::BALL 
		{
			self.mHingeClockwiseConstraintDegs = angleDegs;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have hinge constraint angles.");
		}
	}

    pub fn setHingeJointAnticlockwiseConstraintDegs(&mut self, angleDegs: Rad<f32>)
	{
		Joint::validateConstraintAngleDegs(angleDegs);
		
		if self.mJointType != JointType::BALL 
		{
			self.mHingeAnticlockwiseConstraintDegs = angleDegs;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have hinge constraint angles.");
		}
	}

    pub fn  setHingeRotationAxis(&mut self, axis: Vector3<f32>)
	{
		Joint::validateAxis(axis);
		
		if self.mJointType != JointType::BALL 
		{
			self.mRotationAxisUV = axis.normalize();
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have a hinge rotation axis.");
		}
	}

    pub fn getHingeReferenceAxis(&self) -> Vector3<f32>
	{	
		if self.mJointType != JointType::BALL
		{
			return self.mReferenceAxisUV;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have a hinge reference axis.");
		}
	}

   pub fn setHingeReferenceAxis(&mut self, referenceAxis: Vector3<f32>)
	{
        Joint::validateAxis(referenceAxis);
		
		if self.mJointType != JointType::BALL 
		{
			self.mReferenceAxisUV = referenceAxis.normalize();
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have a hinge reference axis.");
		}
	}

    pub fn getHingeRotationAxis(&self) -> Vector3<f32>
	{	
		if self.mJointType != JointType::BALL
		{
			return self.mRotationAxisUV;
		}
		else
		{
			panic!("Joint type is JointType.BALL - it does not have a hinge rotation axis.");
		}
	}

    pub fn getJointType(&self) -> JointType
    { 
        return self.mJointType; 
    }

    pub fn validateConstraintAngleDegs(angleDegs: Rad<f32>)
	{
		if angleDegs < MIN_CONSTRAINT_ANGLE_DEGS || angleDegs > MAX_CONSTRAINT_ANGLE_DEGS
		{
			panic!("Constraint angles must be within the range {:?} to {:?} inclusive.", MIN_CONSTRAINT_ANGLE_DEGS, MAX_CONSTRAINT_ANGLE_DEGS); 
            
		}
	}
	
	pub fn validateAxis(axis: Vector3<f32>)
	{
		if axis.magnitude() <= 0.0
		{
			panic!("Provided axis is illegal - it has a magnitude of zero.");
		}
	}


   
}