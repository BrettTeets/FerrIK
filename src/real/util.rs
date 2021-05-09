use cgmath::{Vector3, InnerSpace, Rad};

pub fn  approximatelyEquals( a: f32,  b: f32,  tolerance: f32) -> bool
{
    if (a - b).abs() <= tolerance {
        return true;
    }
    else{
        return false;
    } 
}

pub fn getDirectionUV(start: Vector3<f32>, end: Vector3<f32>)-> Vector3<f32>
{
    return (end - start).normalize();
}

pub fn getGlobalPitchDegs(v1: Vector3<f32>) -> Rad<f32>
{
    let xProjected: Vector3<f32> = projectOntoPlane(v1, Vector3::unit_x());
    let pitch: Rad<f32> = getAngleBetweenDegs(-Vector3::unit_x(), xProjected);
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
		let yaw: Rad<f32> = getAngleBetweenDegs( -Vector3::unit_y(), yProjected);
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

pub fn getAngleBetweenDegs(v1: cgmath::Vector3<f32>,v2: cgmath::Vector3<f32>) -> cgmath::Rad<f32>
{
    // Note: a and b are normalised within the dotProduct method.
    let product = v1.dot(v2);
    let rad = product.acos();
    return cgmath::Rad(rad);
}
