use cgmath::{Vector3, InnerSpace, Rad, Angle, Matrix3};

pub fn  approximately_equals( a: f32,  b: f32,  tolerance: f32) -> bool
{
    if (a - b).abs() <= tolerance {
        return true;
    }
    else{
        return false;
    } 
}

pub fn get_direction_uv(start: Vector3<f32>, end: Vector3<f32>)-> Vector3<f32>
{
    return (end - start).normalize();
}

pub fn getGlobalPitchDegs(v1: Vector3<f32>) -> Rad<f32>
{
    let xProjected: Vector3<f32> = projectOntoPlane(v1, Vector3::unit_x());
    let pitch: Rad<f32> = get_angle_between(-Vector3::unit_x(), xProjected);
    if xProjected.y < 0.0 {
        return -pitch;
    }
    else{
        return pitch;
    }
}

pub fn getGlobalYawDegs(v1: Vector3<f32>) -> Rad<f32>
	{
		let yProjected: Vector3<f32> = projectOntoPlane(v1, Vector3::unit_y());
		let yaw: Rad<f32> = get_angle_between( -Vector3::unit_y(), yProjected);
		if yProjected.x < 0.0 {
            return -yaw;
        }
        else{
            return yaw;
        }
	}

pub fn projectOntoPlane(original: Vector3<f32>, plane_normal: Vector3<f32>) -> Vector3<f32>
{
    if  !(plane_normal.magnitude() > 0.0)  {	
        panic!("Plane normal cannot be a zero vector."); 
    }
    
    // Projection of vector b onto plane with normal n is defined as: b - ( b.n / ( |n| squared )) * n
    // Note: |n| is length or magnitude of the vector n, NOT its (component-wise) absolute value		
    let b: Vector3<f32> = original.normalize();
    let n: Vector3<f32> = plane_normal.normalize();		
    return b - ( n * ( b.dot(plane_normal) ) ).normalize();
    
}

//Tested.
pub fn get_angle_between(v1: cgmath::Vector3<f32>,v2: cgmath::Vector3<f32>) -> cgmath::Rad<f32>
{
    // Note: a and b are normalised within the dotProduct method.
    let product = v1.dot(v2);
    let rad = product.acos();
    return cgmath::Rad(rad);
}

//Does this need to be tested?
pub fn calc_angle_limited_uv(vec_to_limit: Vector3<f32>,  vec_baseline: Vector3<f32>,
    angle_limit: Rad<f32>) -> cgmath::Vector3<f32>
{
    // Note: This will ALWAYS be a positive value between 0 and 180 degrees. //Will it? Yes. I've added a Test.
    let angle_between_vectors: Rad<f32> = get_angle_between(vec_baseline, vec_to_limit);
    
    if angle_between_vectors > angle_limit
    {        	
        // The axis which we need to rotate around is the one perpendicular to the two vectors - so we're
        // rotating around the vector which is the cross-product of our two vectors.
        // Note: We do not have to worry about both vectors being the same or pointing in opposite directions
        // because if they bones are the same direction they will not have an angle greater than the angle limit,
        // and if they point opposite directions we will approach but not quite reach the precise max angle
        // limit of 180.0f (I believe). - Caliko source.
        let correction_axis: Vector3<f32> = vec_baseline.normalize().cross(vec_to_limit.normalize()).normalize();
        return rotate_about_axis(vec_baseline, angle_limit, correction_axis).normalize();
    }
    else // Angle not greater than limit? Just return a normalised version of the vecToLimit
    {
        // This may already BE normalised, but we have no way of knowing without calcing the length, so best be safe and normalise.
        // TODO: If performance is an issue, then I could get the length, and if it's not approx. 1.0f THEN normalise otherwise just return as is.
        return vec_to_limit.normalize();
    }
}

//Tested
pub fn rotate_about_axis(source: Vector3<f32>,  angle: Rad<f32>, rotation_axis: Vector3<f32>) -> Vector3<f32>
	{
		let sin_theta            = angle.sin();
		let cos_theta            = angle.cos();
		let one_minus_cos_theta  = 1.0 - cos_theta;
		
		// It's quicker to pre-calc these and reuse than calculate x * y, then y * x later (same thing).
		let xy_one = rotation_axis.x * rotation_axis.y * one_minus_cos_theta;
		let xz_one = rotation_axis.x * rotation_axis.z * one_minus_cos_theta;
		let yz_one = rotation_axis.y * rotation_axis.z * one_minus_cos_theta;
		
		// Calculate rotated x-axis
		let m00 = rotation_axis.x * rotation_axis.x * one_minus_cos_theta + cos_theta;
		let m01 = xy_one + rotation_axis.z * sin_theta;
		let m02 = xz_one - rotation_axis.y * sin_theta;

		// Calculate rotated y-axis
		let m10 = xy_one - rotation_axis.z * sin_theta;
		let m11 = rotation_axis.y * rotation_axis.y * one_minus_cos_theta + cos_theta;
		let m12 = yz_one + rotation_axis.x * sin_theta;

		// Calculate rotated z-axis
		let m20 = xz_one + rotation_axis.y * sin_theta;
		let m21 = yz_one - rotation_axis.x * sin_theta;
		let m22 = rotation_axis.z * rotation_axis.z * one_minus_cos_theta + cos_theta;

        let rotation_matrix: Matrix3<f32> = Matrix3::new(m00, m01, m02, m10, m11, m12, m20, m21, m22 );

		// Multiply the source by the rotation matrix we just created to perform the rotation
		return rotation_matrix * source;
	}

    pub fn createRotationMatrix(referenceDirection: Vector3<f32>) -> Matrix3<f32>
	{		
        //todo: add credit to the original author Al Lansley. reminder dont delete this comment.
		// Singularity fix provided by meaten - see: https://github.com/FedUni/caliko/issues/19
		if (referenceDirection.y).abs() > 0.9999
		{
            let y = Vector3::new(1.0, 0.0, 0.0).cross(referenceDirection).normalize();
            return Matrix3::from_cols(Vector3::new(1.0, 0.0, 0.0), y , referenceDirection);
        }
		else
		{
            let x = referenceDirection.cross(Vector3::new(0.0, 1.0, 0.0)).normalize();
            let y = x.cross(referenceDirection).normalize();
            return Matrix3::from_cols(x, y, referenceDirection);
		}
		
	}

    pub fn getSignedAngleBetweenDegs(referenceVector: Vector3<f32>, otherVector: Vector3<f32>,
        normalVector: Vector3<f32>) -> f32
    {
        let unsignedAngle = get_angle_between(referenceVector, otherVector);
        
        //TODO look into this one. somethings up with going from rads to f32. Ends up being used ina greater than less than check
        let sign: f32 = sign(referenceVector.cross(otherVector).dot(normalVector));		
        return unsignedAngle.0 * sign;
    }

    pub fn sign(value: f32) -> f32
{
    if value >= 0.0 { 
      return 1.0; 
    }		
    return -1.0;
}

pub fn distance_between(v1: Vector3<f32>, v2: Vector3<f32>) -> f32
	{
		let dx = v2.x - v1.x;
		let dy = v2.y - v1.y;
		let dz = v2.z - v1.z;
		return (dx * dx + dy * dy + dz * dz).sqrt();
	}

    pub fn perpendicular(a: Vector3<f32>, b: Vector3<f32>) -> bool
	{
		if approximately_equals( a.dot(b), 0.0, 0.01 )
        {
            return true;
        }
        else{
            return false;
        }
	}

    pub fn genPerpendicularVectorQuick(u: Vector3<f32>) -> Vector3<f32>
	{
		let perp: Vector3<f32>;
		
		if (u.y).abs() < 0.99
		{
			perp = Vector3::new(-u.z, 0.0, u.x); // cross(u, UP)
		}
		else
		{
			perp = Vector3::new(0.0, u.z, -u.y); // cross(u, RIGHT)
		}
		
		return perp.normalize();
	}

    pub fn validateDirectionUV(directionUV: Vector3<f32>)
	{
		// Ensure that the magnitude of this direction unit vector is greater than zero
		if  directionUV.magnitude() <= 0.0
		{
			panic!("Vec3f direction unit vector cannot be zero.");
		}
	}

    pub fn v_approximatelyEquals(v1: Vector3<f32>, v2: Vector3<f32>, tolerance: f32) -> bool
	{	
		if tolerance < 0.0
		{
			panic!("Equality threshold must be greater than or equal to 0.0f");
		}
		
		// Get the absolute differences between the components
		let xDiff = (v1.x - v2.x).abs();
		let yDiff = (v1.y - v2.y).abs();
		let zDiff = (v1.z - v2.z).abs();
		
		// Return true or false
		return xDiff < tolerance && yDiff < tolerance && zDiff < tolerance;
	}

    pub fn validateLength(length: f32)
	{
		// Ensure that the magnitude of this direction unit vector is not zero
		if length < 0.0
		{
			panic!("Length must be a greater than or equal to zero.");
		}
	}