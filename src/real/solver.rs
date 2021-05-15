
use crate::real::{bone::Bone, joint::Joint, joint::JointType, structure::Structure, joint, util, chain::Chain};
use crate::real::chain::BaseboneConstraintType;
use cgmath::{Vector3, Rad, InnerSpace, Matrix3};

pub fn solve_ik(chain: &mut Chain, target: Vector3<f32>) -> f32
	{	
		// Sanity check that there are bones in the chain
		if chain.bones.is_empty() { 
		  panic!("It makes no sense to solve an IK chain with zero bones."); 
		}
		
		// Loop over all bones in the chain, from the end effector (numBones-1) back to the basebone (0)		
        forward_pass(chain, target);
        backward_pass(chain);
		

		chain.last_target_location = target;
		return util::distanceBetween(chain[chain.bones.len()-1].getEndLocation(), target);
	}

pub fn forward_pass(chain: &mut Chain, target: Vector3<f32>){
    for index in chain.forward_pass_index_iter()
		{
			if chain.is_not_end_effector(index)
			{
				let new_start_location = non_end_bone_calc(chain, index);
				set_forward_pass_position(chain, index, new_start_location);
			}
			else
			{
				chain[index].setEndLocation(target);
				let new_start_location = end_bone_calc(chain, index, target);
				set_forward_pass_position(chain, index, new_start_location);
			}
		} // End of forward pass
}

pub fn backward_pass(chain: &mut Chain){
    for index in chain.backwards_pass_index_iter()
    {
        if chain.is_not_basebone(index)
        {
            let new_end_location = non_basebone_calc(chain, index);
            set_backwards_position(chain, index, new_end_location);
        }
        else
        {	
            set_start_based_on_fixed_mode(chain, index);
            let new_end_location = basebone_calc(chain, index);
            set_backwards_basebone(chain, index, new_end_location);
        } 
    } 
}

// At this stage we have a outer-to-inner unit vector for this bone which is within our constraints,
// so we can set the new inner joint location to be the end joint location of this bone plus the
// outer-to-inner direction unit vector multiplied by the length of the bone.
pub fn non_end_bone_calc(chain: &Chain, index: usize, )-> Vector3<f32>{
    return chain.bones[index].getEndLocation() + (match chain[index].getJointType(){
        JointType::BALL => calc_ball_joint_constraint(chain, index, -chain[index].getDirectionUV(), -chain[index+1].getDirectionUV()),
        JointType::GLOBAL_HINGE => calc_forward_global_joint_constraint(chain, index, -chain[index].getDirectionUV()),
        JointType::LOCAL_HINGE => calc_forward_local_joint_constraint(chain, index, -chain[index].getDirectionUV()),
    } * (chain[index].length()) );
}

pub fn end_bone_calc(chain: &Chain, index: usize, target: Vector3<f32>) -> Vector3<f32>{
    return target + match chain[index].getJointType(){
        JointType::BALL =>  -chain[index].getDirectionUV(),				
        JointType::GLOBAL_HINGE => // Global hinges get constrained to the hinge rotation axis, but not the reference axis within the hinge plane
            calc_end_effector_global_hinge(chain, index),
        JointType::LOCAL_HINGE => 
            calc_forward_effector_local_hinge(chain, index, -chain[index].getDirectionUV()),
    } * (chain[index].length());
}

pub fn non_basebone_calc(chain: &Chain, index: usize) -> Vector3<f32>{
    return chain.bones[index].getStartLocation() +
        match chain[index].getJointType() {
        JointType::BALL => 
            calc_ball_joint_constraint(chain, index, chain[index].getDirectionUV(), chain[index-1].getDirectionUV()),
        JointType::GLOBAL_HINGE => 
            calc_backwards_global_joint(chain, index, chain[index].getDirectionUV()),
        JointType::LOCAL_HINGE => 
            calc_backwards_local_joint(chain, index,  chain[index].getDirectionUV(), chain[index-1].getDirectionUV()),
    } * chain[index].length();
}

pub fn basebone_calc(chain: &Chain, index: usize) -> Vector3<f32>{
    return chain[index].getStartLocation() + 
        match chain.basebone_constraint_type{
            BaseboneConstraintType::None => // If the basebone is unconstrained then process it as usual...
                chain.bones[index].getDirectionUV(),
            BaseboneConstraintType::GlobalRotor => 	
                calc_backwards_basebone_global_rotor(chain, index),
            BaseboneConstraintType::LocalRotor => 
                calc_backwards_basebone_local_rotor(chain, index),
            BaseboneConstraintType::GlobalHinge => 
                calc_backwards_basebone_global_hinge(chain, index),
            BaseboneConstraintType::LocalHinge => 
                calc_backwards_basebone_local_hinge(chain, index)
    } * chain[index].length(); //TODO: make sure all of the above functions are not multiply by length as well.
}

pub fn set_start_based_on_fixed_mode(chain: &mut Chain, index: usize){
    let temp: Vector3<f32>;
    if chain.fixed_base_mode
    {
        temp = chain.fixed_base_location;
    }
    else 
    {
        temp = chain.bones[index].getEndLocation() - ( chain.bones[index].getDirectionUV() * (chain[index].length())); 
    }
    chain[index].setStartLocation(temp) ;
}


pub fn set_forward_pass_position(chain: &mut Chain, index: usize, new_start_location: Vector3<f32>){
    chain[index].setStartLocation(new_start_location);

    //If we are not working on the basebone set the end position of the lower bone to the start of this
    //bone.
    if index > 0
    {
        chain[index-1].setEndLocation(new_start_location);
    }
}

pub fn set_backwards_position(chain: &mut Chain, index: usize, new_end_location: Vector3<f32>){
    chain[index].setEndLocation(new_end_location);

    // If we are not working on the end effector bone, then we set the start joint location of the next bone in
    // the chain (i.e. the bone closer to the target) to be the new end joint location of this bone.
    if index < chain.bones.len() - 1 { 
        chain[index+1].setStartLocation(new_end_location); 
    }
}

pub fn set_backwards_basebone(chain: &mut Chain, index: usize, new_end_location: Vector3<f32>){
    chain[index].setEndLocation( new_end_location );
                    
    // Also, set the start location of the next bone to be the end location of this bone
    if chain.bones.len() > 1 { 
        chain[1].setStartLocation(new_end_location); 
    }
}

//This function is the exact same in the forward and backwards pass, 
//This one does not precalculating the constraint angle * bone length and that will 
//need to be done before it can become a start or end point.
pub fn calc_ball_joint_constraint(chain: &Chain, this: usize, this_bone_uv: Vector3<f32>, 
    other_bone_uv: Vector3<f32>) -> Vector3<f32>{
    // Constrain to relative angle between this bone and the outer bone if required
    let angle_vetween: Rad<f32>    = util::getAngleBetweenDegs(other_bone_uv, this_bone_uv);
    let constraint_angle: Rad<f32> = chain[this].getJoint().getBallJointConstraintDegs();
    if angle_vetween > constraint_angle
    {	
        return util::getAngleLimitedUnitVectorDegs(this_bone_uv, other_bone_uv, constraint_angle);
    }
    return this_bone_uv; //does not include the length, add that before you figure the new end or start position.
}

pub fn calc_forward_global_joint_constraint(chain: &Chain, index: usize,
    this_bone_outer_to_inner_uv: Vector3<f32>) -> Vector3<f32>{
    // Project this bone outer-to-inner direction onto the hinge rotation axis
    // Note: The returned vector is normalised.
    return util::projectOntoPlane(this_bone_outer_to_inner_uv, chain[index].getJoint().getHingeRotationAxis() ); 
    
    // NOTE: Constraining about the hinge reference axis on this forward pass leads to poor solutions... so we won't.
}

pub fn calc_backwards_global_joint(chain: &Chain, index: usize, 
    mut thisBoneInnerToOuterUV: Vector3<f32>) -> Vector3<f32>{
    // Get the hinge rotation axis and project our inner-to-outer UV onto it
    let hingeRotationAxis: Vector3<f32>  =  chain[index].getJoint().getHingeRotationAxis();
    thisBoneInnerToOuterUV = util::projectOntoPlane(thisBoneInnerToOuterUV, hingeRotationAxis);
    
    let hingeReferenceAxis: Vector3<f32> =  chain[index].getJoint().getHingeReferenceAxis();

    // If there are joint constraints, then we must honour them...
    let cwConstraintDegs: Rad<f32>   = -chain[index].getJoint().getHingeClockwiseConstraintDegs();
    let acwConstraintDegs: Rad<f32>  =  chain[index].getJoint().getHingeAnticlockwiseConstraintDegs();
    return correct_rotation_by_constraints(hingeReferenceAxis, thisBoneInnerToOuterUV,
        hingeReferenceAxis, cwConstraintDegs, acwConstraintDegs);
}

pub fn calc_backwards_local_joint(chain: &Chain, index: usize, mut thisBoneInnerToOuterUV: Vector3<f32>,
    prevBoneInnerToOuterUV: Vector3<f32>) -> Vector3<f32>{
    // Transform the hinge rotation axis to be relative to the previous bone in the chain
    let hingeRotationAxis: Vector3<f32>  = chain[index].getJoint().getHingeRotationAxis();
                
    // Construct a rotation matrix based on the previous bone's direction
    let m: Matrix3<f32> = util::createRotationMatrix(prevBoneInnerToOuterUV);
    
    // Transform the hinge rotation axis into the previous bone's frame of reference
    let relativeHingeRotationAxis: Vector3<f32>  = m * (hingeRotationAxis).normalize();
    
    
    // Project this bone direction onto the plane described by the hinge rotation axis
    // Note: The returned vector is normalised.
    thisBoneInnerToOuterUV = util::projectOntoPlane(thisBoneInnerToOuterUV, relativeHingeRotationAxis);
    
    let relativeHingeReferenceAxis: Vector3<f32> = m * ( chain[index].getJoint().getHingeReferenceAxis() ).normalize();

    // Constrain rotation about reference axis if required
    let cwConstraintDegs: Rad<f32>   = -chain[index].getJoint().getHingeClockwiseConstraintDegs();
    let acwConstraintDegs: Rad<f32>  =  chain[index].getJoint().getHingeAnticlockwiseConstraintDegs();
    return correct_rotation_by_constraints(relativeHingeReferenceAxis, thisBoneInnerToOuterUV,
            relativeHingeRotationAxis, cwConstraintDegs, acwConstraintDegs);
}

fn correct_rotation_by_constraints(relativeHingeReferenceAxis: Vector3<f32>, thisBoneInnerToOuterUV: Vector3<f32>, relativeHingeRotationAxis: Vector3<f32>,
cwConstraintDegs: Rad<f32>, acwConstraintDegs: Rad<f32>) -> Vector3<f32>{
    
    let signedAngleDegs: f32 = util::getSignedAngleBetweenDegs(relativeHingeReferenceAxis, thisBoneInnerToOuterUV, relativeHingeRotationAxis);
    
    if  !( util::approximatelyEquals(cwConstraintDegs.0, -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.001) ) &&
            !( util::approximatelyEquals(acwConstraintDegs.0, joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.001) ) 
    {
        // Make our bone inner-to-outer UV the hinge reference axis rotated by its maximum clockwise or anticlockwise rotation as required
        if signedAngleDegs > acwConstraintDegs.0
        {	
            return util::rotateAboutAxis(relativeHingeReferenceAxis, acwConstraintDegs, relativeHingeRotationAxis).normalize();		        		
        }
        else if signedAngleDegs < cwConstraintDegs.0
        {	
            return util::rotateAboutAxis(relativeHingeReferenceAxis, cwConstraintDegs, relativeHingeRotationAxis).normalize();			        		
        }
    }
    
    return  thisBoneInnerToOuterUV;
}

pub fn calc_forward_local_joint_constraint(chain: &Chain, index: usize,
    this_bone_outer_to_inner_uv: Vector3<f32>) -> Vector3<f32> 
{
    // Not a basebone? Then construct a rotation matrix based on the previous bones inner-to-to-inner direction...
    let m: Matrix3<f32>;
    let relativeHingeRotationAxis: Vector3<f32>;
    if index > 0 {
        m = util::createRotationMatrix( chain[index-1].getDirectionUV() );
        relativeHingeRotationAxis = m * ( chain[index].getJoint().getHingeRotationAxis() ).normalize();
    }
    else // ...basebone? Need to construct matrix from the relative constraint UV.
    {
        relativeHingeRotationAxis = chain.basebone_relative_constraint_uv;
    }
                
    return util::projectOntoPlane(this_bone_outer_to_inner_uv, relativeHingeRotationAxis);
                        
}

pub fn calc_forward_effector_local_hinge(chain: &Chain, this: usize, thisBoneOuterToInnerUV: Vector3<f32>) -> Vector3<f32>{
    // Local hinges get constrained to the hinge rotation axis, but not the reference axis within the hinge plane
                    
    // Construct a rotation matrix based on the previous bones inner-to-to-inner direction...
    let m: Matrix3<f32> = util::createRotationMatrix( chain[this-1].getDirectionUV() );
    
    // ...and transform the hinge rotation axis into the previous bones frame of reference.
    let relativeHingeRotationAxis: Vector3<f32> = m * ( chain[this].getJoint().getHingeRotationAxis() ).normalize();
                        
    // Project this bone's outer-to-inner direction onto the plane described by the relative hinge rotation axis
    // Note: The returned vector is normalised.					
    return util::projectOntoPlane(thisBoneOuterToInnerUV, relativeHingeRotationAxis);
}

pub fn calc_end_effector_global_hinge(chain: &Chain, index: usize) -> Vector3<f32>{
    return util::projectOntoPlane(-chain[index].getDirectionUV(), chain[index].getJoint().getHingeRotationAxis())
}

pub  fn calc_backwards_basebone_global_rotor(chain: &Chain, index: usize) -> Vector3<f32>
	{
		
		let mut thisBoneInnerToOuterUV: Vector3<f32> = chain.bones[index].getDirectionUV();				
		let angleBetweenDegs: Rad<f32>    = util::getAngleBetweenDegs(chain.basebone_constraint_uv, thisBoneInnerToOuterUV);
		let constraintAngleDegs: Rad<f32> = chain.bones[index].getBallJointConstraintDegs(); 
	
		if angleBetweenDegs > constraintAngleDegs
		{
			thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, chain.basebone_constraint_uv, constraintAngleDegs);
		}
		return thisBoneInnerToOuterUV;
	}

pub fn calc_backwards_basebone_local_rotor(chain: &Chain, index: usize) -> Vector3<f32>{
    // Note: The mBaseboneRelativeConstraintUV is updated in the FabrikStructure3D.solveForTarget()
    // method BEFORE this FabrikChain3D.solveForTarget() method is called. We no knowledge of the
    // direction of the bone we're connected to in another chain and so cannot calculate this 
    // relative basebone constraint direction on our own, but the FabrikStructure3D does it for
    // us so we are now free to use it here.
    
    // Get the inner-to-outer direction of this bone
    let mut thisBoneInnerToOuterUV: Vector3<f32> = chain.bones[index].getDirectionUV();
                            
    // Constrain about the relative basebone constraint unit vector as neccessary
    let angleBetweenDegs: Rad<f32>    = util::getAngleBetweenDegs(chain.basebone_relative_constraint_uv, thisBoneInnerToOuterUV);
    let constraintAngleDegs: Rad<f32> = chain.bones[index].getBallJointConstraintDegs();
    if angleBetweenDegs > constraintAngleDegs
    {
        thisBoneInnerToOuterUV = util::getAngleLimitedUnitVectorDegs(thisBoneInnerToOuterUV, chain.basebone_relative_constraint_uv, constraintAngleDegs);
    }
    return thisBoneInnerToOuterUV; //TODO just took off length multiplcation make sure that happens elsewhere
}

pub fn calc_backwards_basebone_global_hinge(chain: &Chain, index: usize) -> Vector3<f32>{
    let thisJoint: Joint  =  chain[index].getJoint();
    let hingeRotationAxis: Vector3<f32>  =  thisJoint.getHingeRotationAxis();
    let cwConstraintDegs: Rad<f32>   = -thisJoint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
    let acwConstraintDegs: Rad<f32>  =  thisJoint.getHingeAnticlockwiseConstraintDegs();
    
    // Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
    let mut thisBoneInnerToOuterUV: Vector3<f32> = util::projectOntoPlane(chain[index].getDirectionUV(), hingeRotationAxis);
            
    // If we have a global hinge which is not freely rotating then we must constrain about the reference axis
    if  !( util::approximatelyEquals(cwConstraintDegs.0 , -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) &&
            util::approximatelyEquals(acwConstraintDegs.0,  joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) ) 
    {
        // Grab the hinge reference axis and calculate the current signed angle between it and our bone direction (about the hinge
        // rotation axis). Note: ACW rotation is positive, CW rotation is negative.
        let hingeReferenceAxis: Vector3<f32> = thisJoint.getHingeReferenceAxis();
        let signedAngleDegs: f32    = util::getSignedAngleBetweenDegs(hingeReferenceAxis, thisBoneInnerToOuterUV, hingeRotationAxis);
        
        // Constrain as necessary
        if signedAngleDegs > acwConstraintDegs.0
        {	
            thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, acwConstraintDegs, hingeRotationAxis).normalize();		        		
        }
        else if signedAngleDegs < cwConstraintDegs.0
        {	
            thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, cwConstraintDegs, hingeRotationAxis).normalize();			        		
        }
    }
    return thisBoneInnerToOuterUV;
}

pub fn calc_backwards_basebone_local_hinge(chain: &Chain, index: usize) -> Vector3<f32> {
    let thisJoint: Joint  =  chain[index].getJoint();
    let hingeRotationAxis: Vector3<f32>  =  chain.basebone_relative_constraint_uv;                   // Basebone relative constraint is our hinge rotation axis!
    let cwConstraintDegs: Rad<f32>   = -thisJoint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
    let acwConstraintDegs: Rad<f32>  =  thisJoint.getHingeAnticlockwiseConstraintDegs();
    
    // Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
    let mut thisBoneInnerToOuterUV: Vector3<f32> = util::projectOntoPlane(chain[index].getDirectionUV(), hingeRotationAxis);
    
    // If we have a local hinge which is not freely rotating then we must constrain about the reference axis
    if  !( util::approximatelyEquals(cwConstraintDegs.0 , -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) &&
            util::approximatelyEquals(acwConstraintDegs.0,  joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) ) 
    {
        // Grab the hinge reference axis and calculate the current signed angle between it and our bone direction (about the hinge
        // rotation axis). Note: ACW rotation is positive, CW rotation is negative.
        let hingeReferenceAxis: Vector3<f32> = chain.basebone_relative_reference_constraint_uv; 
        let signedAngleDegs: f32    = util::getSignedAngleBetweenDegs(hingeReferenceAxis, thisBoneInnerToOuterUV, hingeRotationAxis);
        
        // Constrain as necessary
        if signedAngleDegs > acwConstraintDegs.0
        {	
            thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, acwConstraintDegs, hingeRotationAxis).normalize();		        		
        }
        else if signedAngleDegs < cwConstraintDegs.0
        {	
            thisBoneInnerToOuterUV = util::rotateAboutAxis(hingeReferenceAxis, cwConstraintDegs, hingeRotationAxis).normalize();			        		
        }
    }
    return thisBoneInnerToOuterUV;
}
