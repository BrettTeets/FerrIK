use cgmath::{Rad};

pub const MIN_2D_CONSTRAINT_ANGLE_DEGS: Rad<f32> = Rad(0.0);
pub const MAX_2D_CONSTRAINT_ANGLE_DEGS: Rad<f32> = Rad(3.14159); //This should be 180 degrees in radians.

#[derive(Clone, Copy)]
pub struct Joint{
    mClockwiseConstraintDegs: Rad<f32>,
    mAnticlockwiseConstraintDegs: Rad<f32>,
    mConstraintCoordinateSystem: ConstraintCoordinateSystem,	
}

impl Joint{
    pub fn new() -> Self{
        Self{
            mClockwiseConstraintDegs: MAX_2D_CONSTRAINT_ANGLE_DEGS,
            mAnticlockwiseConstraintDegs: MAX_2D_CONSTRAINT_ANGLE_DEGS,
            mConstraintCoordinateSystem: ConstraintCoordinateSystem::LOCAL,
        }
    }

    pub fn new_with_constraints(clockwiseConstraintDegs: Rad<f32>, antiClockwiseConstraintDegs: Rad<f32>,
         constraintCoordSystem: ConstraintCoordinateSystem ) -> Self{
        Self{
            mClockwiseConstraintDegs: clockwiseConstraintDegs,
            mAnticlockwiseConstraintDegs: antiClockwiseConstraintDegs,
            mConstraintCoordinateSystem: constraintCoordSystem,
        }
    }

    //looks to be setting based on another sample.
    pub fn set(&mut self, sourceJoint: &Joint)
	{	
		self.setClockwiseConstraintDegs(sourceJoint.mClockwiseConstraintDegs);
		self.setAnticlockwiseConstraintDegs(sourceJoint.mAnticlockwiseConstraintDegs);
		self.mConstraintCoordinateSystem = sourceJoint.mConstraintCoordinateSystem;
	}

    pub fn setClockwiseConstraintDegs(&mut self, angleDegs: Rad<f32>)
	{		
		if angleDegs < MIN_2D_CONSTRAINT_ANGLE_DEGS { 
		  self.mClockwiseConstraintDegs = MIN_2D_CONSTRAINT_ANGLE_DEGS; 
		} else if angleDegs > MAX_2D_CONSTRAINT_ANGLE_DEGS { 
		  self.mClockwiseConstraintDegs = MAX_2D_CONSTRAINT_ANGLE_DEGS; 
		} else {
	      self.mClockwiseConstraintDegs = angleDegs;
		}
	}

    pub fn setAnticlockwiseConstraintDegs(&mut self, angleDegs: Rad<f32>)
	{		
		if angleDegs < MIN_2D_CONSTRAINT_ANGLE_DEGS { 
		  self.mAnticlockwiseConstraintDegs = MIN_2D_CONSTRAINT_ANGLE_DEGS; 
		}
		else if angleDegs > MAX_2D_CONSTRAINT_ANGLE_DEGS { 
		  self.mAnticlockwiseConstraintDegs = MAX_2D_CONSTRAINT_ANGLE_DEGS; 
		} else {
	      self.mAnticlockwiseConstraintDegs = angleDegs;
		}
	}

    pub fn setConstraintCoordinateSystem(&mut self, coordSystem: ConstraintCoordinateSystem)
	{
		self.mConstraintCoordinateSystem = coordSystem;
	}

    pub fn getClockwiseConstraintDegs(&self) -> Rad<f32>
    {	return self.mClockwiseConstraintDegs; }

    pub fn getAnticlockwiseConstraintDegs(&self) -> Rad<f32>
    {	return self.mAnticlockwiseConstraintDegs; }

    pub fn getConstraintCoordinateSystem(&self) -> ConstraintCoordinateSystem
	{
		return self.mConstraintCoordinateSystem;
	}
}

#[derive(Clone, Copy)]
pub enum ConstraintCoordinateSystem{
    LOCAL,
    GLOBAL,
}