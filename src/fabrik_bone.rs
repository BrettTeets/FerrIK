use cgmath::{Vector3, InnerSpace};
use crate::fabrik_joint;

//Was going to make this generic but I dont have any idea how to do generics in rust and the compiler
//and cgmath kept yelling at me so everything is going to be f32.
pub trait FabrikBone{
    //So looks like a fabrik bone needs to have a start location, end location, length, and a joint.
    //The original Caliko implimentation mentions each bone only having one joint.
    fn get_start_location(&self) -> &Vector3<f32>;
    fn get_end_location(&self) -> &Vector3<f32>;
    fn set_start_location(&mut self, start: Vector3<f32>);
    fn set_end_location(&mut self, end: Vector3<f32>);
    fn get_length(&self) -> f32;
    fn get_joint(&self) -> &fabrik_joint::Joint;
}

pub struct FabrikBone3D{
    start: Vector3<f32>,
    end: Vector3<f32>,
    length: f32,
    joint: fabrik_joint::Joint3D,
}

impl FabrikBone3D{
    pub fn new(start: Vector3<f32>, end: Vector3<f32>, joint: fabrik_joint::Joint3D) -> Self{
        let length = (end - start).magnitude();
        
        Self{
            start,
            end,
            length,
            joint,
        }
    }
}



impl FabrikBone for FabrikBone3D{
    fn get_start_location(&self) -> &Vector3<f32>{
        return &self.start;
    }

    fn get_end_location(&self) -> &Vector3<f32>{
        return &self.end;
    }

    fn get_length(&self) -> f32{
        return self.length;
    }

    fn get_joint(&self) -> &fabrik_joint::Joint{
        return &self.joint;
    }

    fn set_start_location(&mut self, start: Vector3<f32>){
       self.start = start;
    }

    fn set_end_location(&mut self, end: Vector3<f32>){
        self.end = end;
    }

}


