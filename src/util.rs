use crate::chain::FerrikErrors;
use log::{error};
use cgmath::InnerSpace;
use cgmath::{Rad, Vector3, Matrix3, One, Angle};

/**
 * Determine the sign of a float value.
 * 
 * @param	value	The value to return the sign of.
 * @return			1.0f if the provided float value is positive, -1.0f otherwise.
 */ 
pub fn sign(value: f32) -> f32
{
    if value >= 0.0 { 
      return 1.0; 
    }		
    return -1.0;
}

/**
 * Validate a direction unit vector (Vec3f) to ensure that it does not have a magnitude of zero.
 * <p>
 * If the direction unit vector has a magnitude of zero then an IllegalArgumentException is thrown.
 * @param	directionUV	The direction unit vector to validate
 */
pub fn validateDirectionUV(directionUV: cgmath::Vector3<f32>) -> Result<(), FerrikErrors>
{
    // Ensure that the magnitude of this direction unit vector is greater than zero
    if  directionUV.magnitude() <= 0.0 {
        error!("Vec3f direction unit vector cannot be zero.");
        return Err(FerrikErrors::UnsolvableRequirement);
    }
    return Ok(());
}

/**
 * Validate the length of a bone to ensure that it's a positive value.
 * <p>
 * If the provided bone length is not greater than zero then an IllegalArgumentException is thrown.
 * @param	length	The length value to validate.
 */
pub fn validateLength(length: f32) -> Result<(), FerrikErrors>
{
    // Ensure that the magnitude of this direction unit vector is not zero
    if length < 0.0
    {
        error!("Length must be a greater than or equal to zero.");
        return Err(FerrikErrors::UnsolvableRequirement);
    }
    return Ok(());
}

/**
 * Convert a value in one range into a value in another range.
 * <p>
 * If the original range is approximately zero then the returned value is the
 * average value of the new range, that is: (newMin + newMax) / 2.0f
 * 
 * @param	origValue	The original value in the original range.
 * @param	origMin		The minimum value in the original range.
 * @param	origMax		The maximum value in the original range.
 * @param	newMin		The new range's minimum value.
 * @param	newMax		The new range's maximum value.
 * @return				The original value converted into the new range.
 */
pub fn convertRange( origValue: f32,  origMin: f32,  origMax: f32,
      newMin: f32,  newMax: f32) -> f32
{	
    let origRange: f32 = origMax - origMin;
    let newRange: f32  = newMax  - newMin;
    
    let newValue: f32;
    if origRange > -0.000001 && origRange < 0.000001
    {
        newValue = (newMin + newMax) / 2.0;
    }
    else
    {
        newValue = (((origValue - origMin) * newRange) / origRange) + newMin;				
    }
    
    return newValue;
}

/**
 * Return a boolean indicating whether a float approximately equals another to within a given tolerance.
 * 
 * @param	a		The first value
 * @param	b		The second value
 * @param	tolerance	The difference within the <strong>a</strong> and <strong>b</strong> values must be within to be considered approximately equal.
 * @return			Whether the a and b values are approximately equal or not.
 */
pub fn rads_approximatelyEquals(a: Rad<f32>,  b: Rad<f32>,  tolerance: Rad<f32>) -> bool
{
    //sp originall this used absolute value, I am going to see if cgmath regular less and greater than
    //can handle this.
    if (a - b) <= tolerance{
        return true;
    }
    else {
        return false;
    }
}

pub fn float_approximatelyEquals(a: f32,  b: f32,  tolerance: f32) -> bool
{
    if (a - b).abs() <= tolerance{
        return true;
    }
    else {
        return false;
    }
}

pub fn  vector_approximatelyEquals(v1: cgmath::Vector3<f32>, v2: cgmath::Vector3<f32>,  tolerance: f32) -> Result<bool, FerrikErrors>
{	
    if tolerance < 0.0
    {
        error!("Equality threshold must be greater than or equal to 0.0f");
        return Err(FerrikErrors::UnsolvableRequirement);
    }
    
    // Get the absolute differences between the components
    let xDiff:f32  = (v1.x - v2.x).abs();
    let yDiff: f32 = (v1.y - v2.y).abs();
    let zDiff: f32 = (v1.z - v2.z).abs();
    
    // Return true or false
    return Ok(xDiff < tolerance && yDiff < tolerance && zDiff < tolerance);
}

pub fn perpendicular(a: cgmath::Vector3<f32>, b: cgmath::Vector3<f32>) -> bool
{
    //Todo: does this need to be cloned before it is used like this is it changing a?
    if float_approximatelyEquals(a.dot(b) , 0.0, 0.01)
    {
        return true;
    }
    else{
        return false;
    }
}

pub fn getAngleBetweenRads(v1: cgmath::Vector3<f32>,v2: cgmath::Vector3<f32>) -> cgmath::Rad<f32>
{
    // Note: a and b are normalised within the dotProduct method.
    let product = v1.dot(v2);
    let rad = product.acos();
    return cgmath::Rad(rad);
}

pub fn getAngleLimitedUnitVectorDegs(vecToLimit: Vector3<f32>,  vecBaseline: Vector3<f32>,
    angleLimitDegs: Rad<f32>) -> cgmath::Vector3<f32>
{
    // Get the angle between the two vectors
    // Note: This will ALWAYS be a positive value between 0 and 180 degrees.
    let angleBetweenVectorsDegs: Rad<f32> = getAngleBetweenRads(vecBaseline, vecToLimit);
    
    if angleBetweenVectorsDegs > angleLimitDegs
    {        	
        // The axis which we need to rotate around is the one perpendicular to the two vectors - so we're
        // rotating around the vector which is the cross-product of our two vectors.
        // Note: We do not have to worry about both vectors being the same or pointing in opposite directions
        // because if they bones are the same direction they will not have an angle greater than the angle limit,
        // and if they point opposite directions we will approach but not quite reach the precise max angle
        // limit of 180.0f (I believe).
        //ToDO, I dont know if this is going to compile correctly might want to break it down into descrte steps.
        let correctionAxis: Vector3<f32> = vecBaseline.normalize().cross(vecToLimit.normalize()).normalize();
        
        // Our new vector is the baseline vector rotated by the max allowable angle about the correction axis
        return rotateAboutAxis(vecBaseline, angleLimitDegs, correctionAxis).normalize();
    }
    else // Angle not greater than limit? Just return a normalised version of the vecToLimit
    {
        // This may already BE normalised, but we have no way of knowing without calcing the length, so best be safe and normalise.
        // TODO: If performance is an issue, then I could get the length, and if it's not approx. 1.0f THEN normalise otherwise just return as is.
        return vecToLimit.normalize();
    }
}

pub fn rotateAboutAxis(source: Vector3<f32>,  angleRads: Rad<f32>, rotationAxis: Vector3<f32>) -> Vector3<f32>
	{
		

		let sinTheta            = angleRads.sin();
		let cosTheta            = angleRads.cos();
		let oneMinusCosTheta  = 1.0 - cosTheta;
		
		// It's quicker to pre-calc these and reuse than calculate x * y, then y * x later (same thing).
		let xyOne = rotationAxis.x * rotationAxis.y * oneMinusCosTheta;
		let xzOne = rotationAxis.x * rotationAxis.z * oneMinusCosTheta;
		let yzOne = rotationAxis.y * rotationAxis.z * oneMinusCosTheta;
		
		// Calculate rotated x-axis
		let m00 = rotationAxis.x * rotationAxis.x * oneMinusCosTheta + cosTheta;
		let m01 = xyOne + rotationAxis.z * sinTheta;
		let m02 = xzOne - rotationAxis.y * sinTheta;

		// Calculate rotated y-axis
		let m10 = xyOne - rotationAxis.z * sinTheta;
		let m11 = rotationAxis.y * rotationAxis.y * oneMinusCosTheta + cosTheta;
		let m12 = yzOne + rotationAxis.x * sinTheta;

		// Calculate rotated z-axis
		let m20 = xzOne + rotationAxis.y * sinTheta;
		let m21 = yzOne - rotationAxis.x * sinTheta;
		let m22 = rotationAxis.z * rotationAxis.z * oneMinusCosTheta + cosTheta;

        let rotationMatrix: Matrix3<f32> = Matrix3::new(m00, m01, m02, m10, m11, m12, m20, m21, m22 );

		// Multiply the source by the rotation matrix we just created to perform the rotation
		return rotationMatrix * source;
	}

    /**
	 * Return a vector which is the result of projecting this vector onto a plane described by the provided surface normal.
	 * <p>
	 * Neither the vector on which this method is called or the provided plane normal vector are modified.
	 * <p>
	 * If the plane surface normal has a magnitude of zero then an IllegalArgumentException is thrown.
	 *  
	 * @param	planeNormal	The normal that describes the plane onto which we will project this vector.
	 * @return				A projected version of this vector.
	 */
pub fn projectOntoPlane(original: &Vector3<f32>, plane_normal: &Vector3<f32>) -> Result<Vector3<f32>, FerrikErrors>
{
    if  !(plane_normal.magnitude() > 0.0)  {	
        error!("Plane normal cannot be a zero vector."); 
        return Err(FerrikErrors::UnsolvableRequirement);
    }
    
    // Projection of vector b onto plane with normal n is defined as: b - ( b.n / ( |n| squared )) * n
    // Note: |n| is length or magnitude of the vector n, NOT its (component-wise) absolute value		
    let b: Vector3<f32> = original.normalize();
    let n: Vector3<f32> = plane_normal.normalize();		
    return Ok(b - ( n * ( b.dot(*plane_normal) ) ).normalize());
    
    /* IMPORTANT: We have to be careful here - even code like the below (where dotProduct uses normalised
     *             versions of 'this' and planeNormal is off by enough to make the IK solutions oscillate:
        *
        *             return this.minus( planeNormal.times( Vec3f.dotProduct(this, planeNormal) ) ).normalised();
        *             
        */
            
    // Note: For non-normalised plane vectors we can use:
    // float planeNormalLength = planeNormal.length();
    // return b.minus( n.times( Vec3f.dotProduct(b, n) / (planeNormalLength * planeNormalLength) ).normalised();
}

pub fn createRotationMatrix(referenceDirection: Vector3<f32>) -> Matrix3<f32>
	{		
		
		let rotMat: Matrix3<f32>;
		
		// Singularity fix provided by meaten - see: https://github.com/FedUni/caliko/issues/19
		if (referenceDirection.y).abs() > 0.9999
		{
            let y = Vector3::new(1.0, 0.0, 0.0).cross(referenceDirection).normalize();
            let rotMat = Matrix3::from_cols(Vector3::new(1.0, 0.0, 0.0), y , referenceDirection);
        }
		else
		{
            let x = referenceDirection.cross(Vector3::new(0.0, 1.0, 0.0)).normalize();
            let y = x.cross(referenceDirection).normalize();
            let rotMat = Matrix3::from_cols(x, y, referenceDirection);
		}

		return rotMat;
		
	}

/*pub fn lengthIsApproximately(float value, float tolerance) -> bool
{
    // Check for a valid tolerance
    if (tolerance < 0.0f)
    {
        throw new IllegalArgumentException("Comparison tolerance cannot be less than zero.");
    }
    
    if ( Math.abs(this.length() - value) < tolerance)
    {
        return true;
    }
    
    return false;
}*/