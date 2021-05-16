// Note this useful idiom: importing names from outer (for mod tests) scope.
use crate::real::util;
use cgmath::{Rad, Vector3, InnerSpace};
use assert_approx_eq::assert_approx_eq;

//Testing get_angle_between
#[test]
fn angle_between_should_be_1_5707964() {
    let v1 = Vector3::unit_x();
    let v2 = Vector3::unit_y();
    assert_approx_eq!(util::get_angle_between(v1, v2).0, Rad(1.5707964).0);
}

#[test]
fn angle_between_should_be_point_785398() {
    let v1 = Vector3{x: 1.0, y: 1.0, z: 0.0}.normalize();
    let v2 = Vector3::unit_y();
    assert_approx_eq!(util::get_angle_between(v1, v2).0, Rad(0.7853982).0);
}

#[test]
fn angle_between_should_always_be_between_zero_and_pi(){
    let v1 = Vector3{x: 1.0, y: 1.0, z: 0.0}.normalize();
    let v2 = Vector3::unit_y();
    let expected_range = 0.0..3.141592;
    assert!(expected_range.contains(&util::get_angle_between(v1, v2).0));
    
    let v1 = Vector3{x: 1.0, y: -1.0, z: 0.0}.normalize();
    let v2 = Vector3::unit_y();
    assert!(expected_range.contains(&util::get_angle_between(v1, v2).0));

    let v1 = Vector3{x: -1.0, y: -1.0, z: 0.0}.normalize();
    let v2 = Vector3::unit_y();
    assert!(expected_range.contains(&util::get_angle_between(v1, v2).0));

    let v1 = Vector3{x: -1.0, y: 1.0, z: 0.0}.normalize();
    let v2 = Vector3::unit_y();
    assert!(expected_range.contains(&util::get_angle_between(v1, v2).0));
}

//I should make more test for this.
#[test]
fn rotate_about_should_be_zerod(){
    let source = Vector3::unit_y();
    let angle: Rad<f32> = cgmath::Deg(90.0).into();
    let rotation_axis =  Vector3::unit_z();

    let result = util::rotate_about_axis(source, -angle, rotation_axis);

    let v1: (f32, f32, f32) = (result.x, result.y, result.z);
    let v2: (f32, f32, f32) = (Vector3::unit_x().x, Vector3::unit_x().y, Vector3::unit_x().z);
    assert_approx_eq!(v1.0, v2.0);
    assert_approx_eq!(v1.1, v2.1);
    assert_approx_eq!(v1.2, v2.2);
}

#[test]
fn angle_limited_should_return_vec_limited(){
    let vec_to_limit = Vector3::unit_y();
    let vec_baseline = Vector3::unit_x(); //what is this? I think it is the direction this bone is point if its rotate was 0
    let angle_limit = Rad(3.141592); //180 degrees.

    let result = util::calc_angle_limited_uv(vec_to_limit, vec_baseline, angle_limit);

    //So the angle between these to is 90 degrees which is less than 180 so the vec_to_limit should
    //be passed back out without any change.
    assert_eq!(vec_to_limit, result);
}

#[test]
fn angle_limited_should_not_return_vec_limited(){
    let vec_to_limit = Vector3::unit_y();
    let vec_baseline = Vector3::unit_x(); //what is this? I think it is the direction this bone is point if its rotate was 0
    let angle_limit = Rad(0.78); //45ish degrees.

    let result = util::calc_angle_limited_uv(vec_to_limit, vec_baseline, angle_limit);

    //So the angle between these to is 90 degrees which is less than 180 so the vec_to_limit should
    //be passed back out without any change.
    assert_ne!(vec_to_limit, result);
}

#[test]
fn angle_limited_should_be_corrected(){
    let vec_to_limit = Vector3::unit_y();
    let vec_baseline = Vector3::unit_x(); //what is this? I think it is the direction this bone is point if its rotate was 0
    let angle_limit = Rad(0.78); //45ish degrees.

    let temp = util::calc_angle_limited_uv(vec_to_limit, vec_baseline, angle_limit);

    let result = util::get_angle_between(temp, vec_baseline); 

    //So if it did correct the vec to limit, then the new angle between the vector and baseline should be less than
    //the limit.
    assert!(result < angle_limit);
}