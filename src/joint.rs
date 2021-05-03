pub trait Joint {
    //this was all that was in the original, though it was a generic there.
    //fn set(&self, sourceJoint: Joint3D);
}

//This was pulled over directly from the caliko version.
#[derive(Clone)]
pub enum JointType {
    Ball,
    GlobalHinge,
    LocalHinge,
}