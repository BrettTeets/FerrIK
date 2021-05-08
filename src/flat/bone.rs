use super::{joint, joint::Joint};
use cgmath::{Vector2, MetricSpace, Rad, InnerSpace};

#[derive(Clone, Copy)]
pub struct Bone{
    mJoint: Joint,
    mStartLocation: Vector2<f32>,
    mEndLocation: Vector2<f32>,
    mLength: f32,
    mGlobalConstraintUV: Vector2<f32>,
}

impl Bone{
    pub fn new() -> Self {
        Self{
            mJoint: Joint::new(),
            //Todo, rather than setting these to zero maybe start them out as None or Some.
            mStartLocation: Vector2::new(0.0, 0.0),
            mEndLocation: Vector2::new(0.0, 0.0),
            mLength: 0.0,
            mGlobalConstraintUV: Vector2::new(1.0, 0.0)
        }
    }

    pub fn new_with_vectors(startLocation: Vector2<f32>, endLocation: Vector2<f32>) -> Self
	{
        Self{
            mJoint: Joint::new(),
            //Todo, rather than setting these to zero maybe start them out as None or Some.
            mStartLocation: startLocation,
            mEndLocation: endLocation,
            mLength: startLocation.distance(endLocation), //Todo double check that this is giving the correct normalized result.
            mGlobalConstraintUV: Vector2::new(1.0, 0.0)
        }
	}

    pub fn new_with_vectors_and_constraints(startLocation: Vector2<f32>, 
        endLocation: Vector2<f32>, cwConstraintDegs: Rad<f32>, acwConstraintDegs: Rad<f32>) -> Self
	{
		// Set up as per previous constructor - IllegalArgumentExceptions will be thrown for invalid directions or lengths
		let mut s = Bone::new_with_vectors(startLocation, endLocation);

		// Set the constraint angles - IllegalArgumentExceptions will be thrown for invalid constraint angles
		s.setClockwiseConstraintDegs(cwConstraintDegs);
		s.setAnticlockwiseConstraintDegs(acwConstraintDegs);

        return s;
	}

    pub fn length(&self) -> f32
    {	return self.mLength; }

    pub fn getStartLocation(&self) -> Vector2<f32>
    { return self.mStartLocation; }

    pub fn getEndLocation(&self) -> Vector2<f32>
    { return self.mEndLocation; }

    pub fn getJoint(&self) -> Joint
    { return self.mJoint; }

    pub fn getClockwiseConstraintDegs(&self) -> Rad<f32>
    { return self.mJoint.getClockwiseConstraintDegs(); }

    pub fn getAnticlockwiseConstraintDegs(&self) -> Rad<f32>
    { return self.mJoint.getAnticlockwiseConstraintDegs(); }

    pub fn getDirectionUV(&self) -> Vector2<f32>
    {	
        return (self.mEndLocation - self.mStartLocation).normalize();
    }

    pub fn getGlobalConstraintUV(&self) -> Vector2<f32>
	{
		return self.mGlobalConstraintUV;
	}

    pub fn getJointConstraintCoordinateSystem(&self) -> joint::ConstraintCoordinateSystem
	{
		return self.mJoint.getConstraintCoordinateSystem();
	}

    //This is setting a joint to the same as the sample.
    pub fn setJoint(&mut self, joint: &Joint) 
    {	self.mJoint.set(joint); }

    pub fn setClockwiseConstraintDegs(&mut self, angleDegs: Rad<f32>) 
    { self.mJoint.setClockwiseConstraintDegs(angleDegs); }

    pub fn setAnticlockwiseConstraintDegs(&mut self, angleDegs: Rad<f32>) 
    { self.mJoint.setAnticlockwiseConstraintDegs(angleDegs); }

    pub fn setGlobalConstraintUV(&mut self, v: Vector2<f32>)
	{
		self.mGlobalConstraintUV = v;
	}

    pub fn setJointConstraintCoordinateSystem(&mut self, coordSystem: joint::ConstraintCoordinateSystem)
	{
		self.mJoint.setConstraintCoordinateSystem(coordSystem);
	}

    pub fn setStartLocation(&mut self, location: Vector2<f32>) 
    { self.mStartLocation = location.clone(); } //The original had a set function here, built into the vector class.

    pub fn setEndLocation(&mut self, location: Vector2<f32>) 
    { self.mEndLocation = location.clone(); } //The original had a set function here, built into the vector class.

    pub fn setLength(&mut self, length: f32)
	{
		if length >= 0.0
		{
			self.mLength = length;
		}
		else
		{
			panic!("Bone length must be a positive value.");
		}
	}

    //wtf getters.
    pub fn getStartLocationAsArray(&self) -> [f32; 2]
    { 
        return [self.mStartLocation.x, self.mStartLocation.y]; 
    }

    pub fn getEndLocationAsArray(&self) -> [f32; 2]
    { 
        return [self.mEndLocation.x, self.mEndLocation.y]; 
    }


}