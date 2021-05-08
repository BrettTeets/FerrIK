use crate::chain::FerrikErrors;
use log::{error};
use cgmath::InnerSpace;

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
pub fn approximatelyEquals(a: f32,  b: f32,  tolerance: f32) -> bool
{
    if (a - b).abs() <= tolerance{
        return true;
    }
    else {
        return false;
    }
}