use crate::{bone, joint3D};
use cgmath::{Vector3, InnerSpace};

#[derive(Clone)]
pub struct Bone3D{
    bone_connection_point: bone::BoneConnectionPoint, //this is where another bone, in a different chain will attach at.
    //bone connection point sounds like it might be how this bone connects to another in a different chain rather than
    //how that other bone connects to itself. idk. Toss the reading material in useful text.
    joint: joint3D::Joint3D, //type of joint ball, hinge etc. //tons of info in the original on this. stuck it in useful text page.
    pub start: Vector3<f32>,
    pub end: Vector3<f32>,
    id: usize, 
    length: f32, //is calculated in the constructor and used to keep everything in line. 
    //original length had some validation code to ensure it wasn't zero.
}

impl Bone3D{
    //default. the original had many different ways of making a new bone3D, one is good enough for now.
    pub fn new(id: usize, start: Vector3<f32>, end: Vector3<f32>)-> Self{
        let length = (end - start).magnitude();
        return Self{
            bone_connection_point: bone::BoneConnectionPoint::End,
            joint: joint3D::Joint3D::new(),
            start,
            end,
            id, 
            length, 
        }
    }

    //This was some sort of override in the original but for what I dont know, I'll wait and
    //see where it is used.
    pub fn length(&self) -> f32{
        return self.length;
    }

    //so this is the length that the bone currently as, like it would be between a pass to correct
    //it to its normal length.
    pub fn live_length(&self) ->f32{
        return (self.end - self.start).magnitude();
    }

    //the direction. I dont know what uv is referring to.
    pub fn get_direction_uv(&self) -> Vector3<f32>{
        return self.end - self.start;
    }

    pub fn get_global_pitch_degs(&self) -> cgmath::Deg<f32>{
        //This was the original code,
            //return  Vec3f.getDirectionUV(mStartLocation, mEndLocation).getGlobalPitchDegs();
        
        //this is what I think it returned.
        return self.start.angle(self.end).into();
    }

    //Wait... I think I know what it wants now I'll need to get everything converted over to
    //see. Alright this might be where quaternion come in as I cant get the angle of a particlure
    //dimesnions very easily from cgmath.
    pub fn get_global_yaw_degs(&self) -> cgmath::Deg<f32>{
        //This was the original code,
            //return  Vec3f.getDirectionUV(mStartLocation, mEndLocation).getGlobalPitchDegs();
        
        //this is what I think it returned.
        return self.start.angle(self.end).into();
    }



    //Originally there was a bunch of getters and setters for working with the hinge joint
    //but all of them were just a short wrapper around joint calling the same functions with
    //the same names. So I opte to get rid of that to cut down on useless code.
        //cut code
        //cut code
        //cut code.
}

impl bone::Bone for Bone3D{

}
//impl DebugFormat  for Bone3D{
    //the original had this, which is this I think
    /*
/**
     * Return a concise, human readable description of this FabrikBone3D as a String.
     */
    @Override
    public String toString()
    {
        StringBuilder sb = new StringBuilder();
        sb.append("Start joint location : " + mStartLocation  + NEW_LINE);
        sb.append("End   joint location : " + mEndLocation    + NEW_LINE);
        sb.append("Bone length          : " + mLength         + NEW_LINE);
        sb.append("Colour               : " + mColour         + NEW_LINE);
        return sb.toString();
    }
    */
//}

//This is everything that was in the original java bone3d class that I didn't know what it did or
//how to port it over. we've got this serial thing left, a hashing code, and a equals code. I think 
//rust handles this with that [(eq)] looking code. I'll have to look that up.
    //private static final long serialVersionUID = 1L;     

    /*
    @Override
    public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((mBoneConnectionPoint == null) ? 0 : mBoneConnectionPoint.hashCode());
    result = prime * result + ((mColour == null) ? 0 : mColour.hashCode());
    result = prime * result + ((mEndLocation == null) ? 0 : mEndLocation.hashCode());
    result = prime * result + ((mJoint == null) ? 0 : mJoint.hashCode());
    result = prime * result + Float.floatToIntBits(mLength);
    result = prime * result + Float.floatToIntBits(mLineWidth);
    result = prime * result + ((mName == null) ? 0 : mName.hashCode());
    result = prime * result + ((mStartLocation == null) ? 0 : mStartLocation.hashCode());
    return result;
    }*/

    /*
    @Override
    public boolean equals(Object obj) {
    if (this == obj) {
    return true;
    }
    if (obj == null) {
    return false;
    }
    if (getClass() != obj.getClass()) {
    return false;
    }
    FabrikBone3D other = (FabrikBone3D) obj;
    if (mBoneConnectionPoint != other.mBoneConnectionPoint) {
    return false;
    }
    if (mColour == null) {
    if (other.mColour != null) {
        return false;
    }
    } else if (!mColour.equals(other.mColour)) {
    return false;
    }
    if (mEndLocation == null) {
    if (other.mEndLocation != null) {
        return false;
    }
    } else if (!mEndLocation.equals(other.mEndLocation)) {
    return false;
    }
    if (mJoint == null) {
    if (other.mJoint != null) {
        return false;
    }
    } else if (!mJoint.equals(other.mJoint)) {
    return false;
    }
    if (Float.floatToIntBits(mLength) != Float.floatToIntBits(other.mLength)) {
    return false;
    }
    if (Float.floatToIntBits(mLineWidth) != Float.floatToIntBits(other.mLineWidth)) {
    return false;
    }
    if (mName == null) {
    if (other.mName != null) {
        return false;
    }
    } else if (!mName.equals(other.mName)) {
    return false;
    }
    if (mStartLocation == null) {
    if (other.mStartLocation != null) {
        return false;
    }
    } else if (!mStartLocation.equals(other.mStartLocation)) {
    return false;
    }
    return true;
    }*/