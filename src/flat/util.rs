use cgmath::{Vector2, Rad, InnerSpace, Deg, Angle, MetricSpace};

pub fn getConstrainedUV(directionUV: Vector2<f32>, baselineUV: Vector2<f32>, 
    clockwiseConstraintDegs: Rad<f32>, antiClockwiseConstraintDegs: Rad<f32>) -> Vector2<f32>
	{
		// Get the signed angle from the baseline UV to the direction UV.
		// Note: In our signed angle ranges:
		//       0...180 degrees represents anti-clockwise rotation, and
		//       0..-180 degrees represents clockwise rotation
		let signedAngleDegs: f32 = getSignedAngleDegsTo(baselineUV ,directionUV);

		// If we've exceeded the anti-clockwise (positive) constraint angle...
        //todo the radians were originall in f32 not using a special type this needs to be checked.
        //I think maybe converting the radian to degrees and then grabbing that scalar would would work?
        let tp: Deg<f32> = antiClockwiseConstraintDegs.into();
		if signedAngleDegs > tp.0
		{
			// ...then our constrained unit vector is the baseline rotated by the anti-clockwise constraint angle.
			// Note: We could do this by calculating a correction angle to apply to the directionUV, but it's simpler to work from the baseline.
			return rotateDegs(baselineUV, antiClockwiseConstraintDegs);
		}

		// If we've exceeded the clockwise (negative) constraint angle...
        let tp: Deg<f32> = clockwiseConstraintDegs.into();
		if signedAngleDegs < -tp.0
		{
			// ...then our constrained unit vector is the baseline rotated by the clockwise constraint angle.
			// Note: Again, we could do this by calculating a correction angle to apply to the directionUV, but it's simpler to work from the baseline.
			return rotateDegs(baselineUV, -clockwiseConstraintDegs);
		}

		// If we have not exceeded any constraint then we simply return the original direction unit vector
		return directionUV;
	}

    pub fn getSignedAngleDegsTo(v1: Vector2<f32>, otherVector: Vector2<f32>) -> f32
	{
		// Normalise the vectors that we're going to use
		let thisVectorUV: Vector2<f32>  = v1.normalize();
		let otherVectorUV: Vector2<f32> = otherVector.normalize();

		// Calculate the unsigned angle between the vectors as the arc-cosine of their dot product
		//float unsignedAngleDegs = (float)Math.acos( thisVectorUV.dot(otherVectorUV) ) * RADS_TO_DEGS;
        let unsignedAngleDegs: f32 = thisVectorUV.dot(otherVectorUV).acos();

		// Calculate and return the signed angle between the two vectors using the zcross method
        
		if zcross(thisVectorUV, otherVector) == 1
		{
			return unsignedAngleDegs;
		}
		else
		{
			return -unsignedAngleDegs;
		}
	}

    pub fn zcross(u: Vector2<f32>, v: Vector2<f32>) -> i32
	{
		let p: f32 = u.x * v.y - v.x * u.y;

		if      p > 0.0 { return 1; }
		else if p < 0.0 { return -1;	}
		return 0;
	}

    pub fn rotateDegs(source: Vector2<f32>, angleDegs: Rad<f32>) -> Vector2<f32>
	{
		// Rotation about the z-axis:
		// x' = x*cos q - y*sin q
		// y' = x*sin q + y*cos q
		// z' = z

		// Convert the rotation angle from degrees to radians
		let angleRads = angleDegs;

		// Pre-calc any expensive calculations we use more than once
		let cosTheta: f32 = angleRads.cos();
		let sinTheta: f32 = angleRads.sin();

		// Create a new vector which is the rotated vector
		// Note: This calculation cannot be performed 'inline' because each aspect (x and y) depends on
		// the other aspect to get the correct result. As such, we must create a new rotated vector, and
		// then assign it back to the original vector.
		return Vector2 {x: source.x * cosTheta - source.y * sinTheta,  
            y: source.x * sinTheta + source.y * cosTheta}; // y
	}

    pub fn distanceBetween (v1: Vector2<f32>, v2: Vector2<f32>)	-> f32
    { 
        return v1.distance(v2); 
    }

    pub fn validateDirectionUV(directionUV: Vector2<f32>)
	{
		// Ensure that the magnitude of this direction unit vector is greater than zero
		if  directionUV.magnitude() <= 0.0 
		{
			panic!("Vec2f direction unit vector cannot be zero.");
		}
	}

    pub fn approximatelyEquals(v1: Vector2<f32>, v2: Vector2<f32>, tolerance: f32) -> bool
	{
		if tolerance < 0.0
        {	
            panic!("Equality threshold must be greater than or equal to 0.0f");	
        }
        return (v1 - v2).magnitude() < tolerance;
		//return (v1.x - v2.x).abs() < tolerance && (v1.y - v2.y).abs() < tolerance;
	}

    pub fn validateLength(length: f32)
	{
		// Ensure that the magnitude of this direction unit vector is not zero
		if length < 0.0
		{
			panic!("Length must be a greater than or equal to zero.");
		}
	}
