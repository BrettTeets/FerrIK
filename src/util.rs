use crate::chain::FerrikErrors;
use log::{error};
use cgmath::InnerSpace;
use cgmath::{Rad, Vector3};

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
        return Vec3f.rotateAboutAxisDegs(vecBaseline, angleLimitDegs, correctionAxis).normalised();
    }
    else // Angle not greater than limit? Just return a normalised version of the vecToLimit
    {
        // This may already BE normalised, but we have no way of knowing without calcing the length, so best be safe and normalise.
        // TODO: If performance is an issue, then I could get the length, and if it's not approx. 1.0f THEN normalise otherwise just return as is.
        return vecToLimit.normalised();
    }
}

public static Vec3f rotateAboutAxisRads(Vec3f source, float angleRads, Vec3f rotationAxis)
	{
		Mat3f rotationMatrix = new Mat3f();

		float sinTheta         = (float)Math.sin(angleRads);
		float cosTheta         = (float)Math.cos(angleRads);
		float oneMinusCosTheta = 1.0f - cosTheta;
		
		// It's quicker to pre-calc these and reuse than calculate x * y, then y * x later (same thing).
		float xyOne = rotationAxis.x * rotationAxis.y * oneMinusCosTheta;
		float xzOne = rotationAxis.x * rotationAxis.z * oneMinusCosTheta;
		float yzOne = rotationAxis.y * rotationAxis.z * oneMinusCosTheta;
		
		// Calculate rotated x-axis
		rotationMatrix.m00 = rotationAxis.x * rotationAxis.x * oneMinusCosTheta + cosTheta;
		rotationMatrix.m01 = xyOne + rotationAxis.z * sinTheta;
		rotationMatrix.m02 = xzOne - rotationAxis.y * sinTheta;

		// Calculate rotated y-axis
		rotationMatrix.m10 = xyOne - rotationAxis.z * sinTheta;
		rotationMatrix.m11 = rotationAxis.y * rotationAxis.y * oneMinusCosTheta + cosTheta;
		rotationMatrix.m12 = yzOne + rotationAxis.x * sinTheta;

		// Calculate rotated z-axis
		rotationMatrix.m20 = xzOne + rotationAxis.y * sinTheta;
		rotationMatrix.m21 = yzOne - rotationAxis.x * sinTheta;
		rotationMatrix.m22 = rotationAxis.z * rotationAxis.z * oneMinusCosTheta + cosTheta;

		// Multiply the source by the rotation matrix we just created to perform the rotation
		return rotationMatrix.times(source);
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