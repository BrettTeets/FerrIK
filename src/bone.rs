use cgmath::{Vector3, InnerSpace};
use crate::joint;

//Was going to make this generic but I dont have any idea how to do generics in rust and the compiler
//and cgmath kept yelling at me so everything is going to be f32.
pub trait Bone{
    //So looks like a fabrik bone needs to have a start location, end location, length, and a joint.
    //The original Caliko implimentation mentions each bone only having one joint.
    //fn get_start_location(&self) -> &Vector3<f32>;
    //fn get_end_location(&self) -> &Vector3<f32>;
    //fn set_start_location(&mut self, start: Vector3<f32>);
    //fn set_end_location(&mut self, end: Vector3<f32>);
    //fn get_length(&self) -> f32;
    //fn get_joint(&self) -> &joint::Joint;

    //Alright no more silly getters and setters till I take a look at everything.
}

#[derive(Clone)]
pub enum BoneConnectionPoint{
    Start,
    End,
}

/*
pub struct Bone3D{
    start: Vector3<f32>,
    end: Vector3<f32>,
    length: f32,
    joint: joint::Joint3D,
}

impl Bone3D{
    pub fn new(start: Vector3<f32>, end: Vector3<f32>, joint: joint::Joint3D) -> Self{
        let length = (end - start).magnitude();
        
        Self{
            start,
            end,
            length,
            joint,
        }
    }
}



impl Bone for Bone3D{
    fn get_start_location(&self) -> &Vector3<f32>{
        return &self.start;
    }

    fn get_end_location(&self) -> &Vector3<f32>{
        return &self.end;
    }

    fn get_length(&self) -> f32{
        return self.length;
    }

    fn get_joint(&self) -> &joint::Joint{
        return &self.joint;
    }

    fn set_start_location(&mut self, start: Vector3<f32>){
       self.start = start;
    }

    fn set_end_location(&mut self, end: Vector3<f32>){
        self.end = end;
    }

}*/


