use crate::real::{chain::BaseboneConstraintType, joint::Joint, joint::JointType, joint, util, chain::Chain};
use cgmath::{Vector3, Rad, InnerSpace, Matrix3};

pub fn solve_ik(chain: &mut Chain, target: Vector3<f32>) -> f32{	
    // Sanity check that there are bones in the chain
    if chain.bones.is_empty() { 
        panic!("It makes no sense to solve an IK chain with zero bones."); 
    }
    
    // Loop over all bones in the chain, from the end effector (numBones-1) back to the basebone (0)		
    forward_pass(chain, target);
    backward_pass(chain);
    
    chain.last_target_location = target;
    return util::distanceBetween(chain[chain.bones.len()-1].end_location, target);
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
				chain[index].set_end_location(target);
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
    return chain.bones[index].end_location + (match chain[index].get_joint_type(){
        JointType::BALL => calc_ball_joint_constraint(chain, index, -chain[index].get_direction(), -chain[index+1].get_direction()),
        JointType::GLOBAL_HINGE => calc_forward_global_joint_constraint(chain, index, -chain[index].get_direction()),
        JointType::LOCAL_HINGE => calc_forward_local_joint_constraint(chain, index, -chain[index].get_direction()),
    } * (chain[index].length()) );
}

pub fn end_bone_calc(chain: &Chain, index: usize, target: Vector3<f32>) -> Vector3<f32>{
    return target + match chain[index].get_joint_type(){
        JointType::BALL =>  -chain[index].get_direction(),				
        JointType::GLOBAL_HINGE => // Global hinges get constrained to the hinge rotation axis, but not the reference axis within the hinge plane
            calc_end_effector_global_hinge(chain, index),
        JointType::LOCAL_HINGE => 
            calc_forward_effector_local_hinge(chain, index, -chain[index].get_direction()),
    } * (chain[index].length());
}

pub fn non_basebone_calc(chain: &Chain, index: usize) -> Vector3<f32>{
    return chain[index].get_start_location() +
        match chain[index].get_joint_type() {
        JointType::BALL => 
            calc_ball_joint_constraint(chain, index, chain[index].get_direction(), chain[index-1].get_direction()),
        JointType::GLOBAL_HINGE => 
            calc_backwards_global_joint(chain, index, chain[index].get_direction()),
        JointType::LOCAL_HINGE => 
            calc_backwards_local_joint(chain, index,  chain[index].get_direction(), chain[index-1].get_direction()),
    } * chain[index].length();
}

pub fn basebone_calc(chain: &Chain, index: usize) -> Vector3<f32>{
    return chain[index].get_start_location() + 
        match chain.basebone_constraint_type{
            BaseboneConstraintType::None => // If the basebone is unconstrained then process it as usual...
                chain.bones[index].get_direction(),
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
        temp = chain.bones[index].end_location - ( chain.bones[index].get_direction() * (chain[index].length())); 
    }
    chain[index].set_start_location(temp) ;
}


pub fn set_forward_pass_position(chain: &mut Chain, index: usize, new_start_location: Vector3<f32>){
    chain[index].set_start_location(new_start_location);

    //If we are not working on the basebone set the end position of the lower bone to the start of this
    //bone.
    if index > 0
    {
        chain[index-1].set_end_location(new_start_location);
    }
}

pub fn set_backwards_position(chain: &mut Chain, index: usize, new_end_location: Vector3<f32>){
    chain[index].set_end_location(new_end_location);

    // If we are not working on the end effector bone, then we set the start joint location of the next bone in
    // the chain (i.e. the bone closer to the target) to be the new end joint location of this bone.
    if index < chain.bones.len() - 1 { 
        chain[index+1].set_start_location(new_end_location); 
    }
}

pub fn set_backwards_basebone(chain: &mut Chain, index: usize, new_end_location: Vector3<f32>){
    chain[index].set_end_location( new_end_location );
                    
    // Also, set the start location of the next bone to be the end location of this bone
    if chain.bones.len() > 1 { 
        chain[1].set_start_location(new_end_location); 
    }
}

//This function is the exact same in the forward and backwards pass, 
//This one does not precalculating the constraint angle * bone length and that will 
//need to be done before it can become a start or end point.
pub fn calc_ball_joint_constraint(chain: &Chain, this: usize, this_bone_uv: Vector3<f32>, 
    other_bone_uv: Vector3<f32>) -> Vector3<f32>{
    let angle_between: Rad<f32>    = util::getAngleBetweenDegs(other_bone_uv, this_bone_uv);
    let constraint_angle: Rad<f32> = chain[this].joint.getBallJointConstraintDegs();
    return constrain_rotor(this_bone_uv, other_bone_uv, angle_between, constraint_angle)
}

pub fn calc_forward_global_joint_constraint(chain: &Chain, index: usize,
    this_bone_outer_to_inner_uv: Vector3<f32>) -> Vector3<f32>{
    // Project this bone outer-to-inner direction onto the hinge rotation axis
    // Note: The returned vector is normalised.
    return util::projectOntoPlane(this_bone_outer_to_inner_uv, chain[index].joint.getHingeRotationAxis() ); 
    
    // NOTE: Constraining about the hinge reference axis on this forward pass leads to poor solutions... so we won't.
}

pub fn calc_backwards_global_joint(chain: &Chain, index: usize, 
    mut inner_to_outer_uv: Vector3<f32>) -> Vector3<f32>{
    // Get the hinge rotation axis and project our inner-to-outer UV onto it
    let rotation_axis =  chain[index].joint.getHingeRotationAxis();
    inner_to_outer_uv = util::projectOntoPlane(inner_to_outer_uv, rotation_axis);
    
    // If there are joint constraints, then we must honour them...
    let cw_constraint = -chain[index].joint.getHingeClockwiseConstraintDegs();
    let acw_constraint =  chain[index].joint.getHingeAnticlockwiseConstraintDegs();
    if  !( util::approximatelyEquals(cw_constraint.0, -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.001) ) &&
                !( util::approximatelyEquals(acw_constraint.0, joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.001) ) 
    {
        let reference_axis =  chain[index].joint.getHingeReferenceAxis();

        inner_to_outer_uv = constrain_hinge(inner_to_outer_uv, reference_axis, rotation_axis, cw_constraint, acw_constraint);
    }
    return inner_to_outer_uv;
}

pub fn calc_backwards_local_joint(chain: &Chain, index: usize, mut inner_to_outer_uv: Vector3<f32>,
    prev_inner_to_outer_uv: Vector3<f32>) -> Vector3<f32>{
    // Transform the hinge rotation axis to be relative to the previous bone in the chain
    let rotation_axis = chain[index].joint.getHingeRotationAxis();
                
    // Construct a rotation matrix based on the previous bone's direction
    let m = util::createRotationMatrix(prev_inner_to_outer_uv);
    
    // Transform the hinge rotation axis into the previous bone's frame of reference
    let rotation_axis  = m * (rotation_axis).normalize();
    
    
    // Project this bone direction onto the plane described by the hinge rotation axis
    // Note: The returned vector is normalised.
    inner_to_outer_uv = util::projectOntoPlane(inner_to_outer_uv, rotation_axis);
    
    // Constrain rotation about reference axis if required
    let cw_constraint  = -chain[index].joint.getHingeClockwiseConstraintDegs();
    let acw_constraint =  chain[index].joint.getHingeAnticlockwiseConstraintDegs();
    if  !( util::approximatelyEquals(cw_constraint.0, -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.001) ) &&
                !( util::approximatelyEquals(acw_constraint.0, joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.001) ) 
    {
        let reference_axis: Vector3<f32> = m * ( chain[index].joint.getHingeReferenceAxis() ).normalize();

        inner_to_outer_uv = constrain_hinge(inner_to_outer_uv, reference_axis, rotation_axis, cw_constraint, acw_constraint);
    }
    return inner_to_outer_uv;
}

pub fn calc_forward_local_joint_constraint(chain: &Chain, index: usize,
    this_bone_outer_to_inner_uv: Vector3<f32>) -> Vector3<f32> 
{
    // Not a basebone? Then construct a rotation matrix based on the previous bones inner-to-to-inner direction...
    let m: Matrix3<f32>;
    let rotation_axis: Vector3<f32>;
    if index > 0 {
        m = util::createRotationMatrix( chain[index-1].get_direction() );
        rotation_axis = m * ( chain[index].joint.getHingeRotationAxis() ).normalize();
    }
    else // ...basebone? Need to construct matrix from the relative constraint UV.
    {
        rotation_axis = chain.basebone_relative_constraint_uv;
    }
                
    return util::projectOntoPlane(this_bone_outer_to_inner_uv, rotation_axis);
                        
}

pub fn calc_forward_effector_local_hinge(chain: &Chain, this: usize, outer_to_inner_uv: Vector3<f32>) -> Vector3<f32>{
    // Local hinges get constrained to the hinge rotation axis, but not the reference axis within the hinge plane
                    
    // Construct a rotation matrix based on the previous bones inner-to-to-inner direction...
    let m: Matrix3<f32> = util::createRotationMatrix( chain[this-1].get_direction() );
    
    // relative hinge rotation axis.
    let rotation_axis: Vector3<f32> = m * ( chain[this].joint.getHingeRotationAxis() ).normalize();
                        
    // Project this bone's outer-to-inner direction onto the plane described by the relative hinge rotation axis
    // Note: The returned vector is normalised.					
    return util::projectOntoPlane(outer_to_inner_uv, rotation_axis);
}

pub fn calc_end_effector_global_hinge(chain: &Chain, index: usize) -> Vector3<f32>{
    return util::projectOntoPlane(-chain[index].get_direction(), chain[index].joint.getHingeRotationAxis())
}

fn calc_backwards_basebone_global_rotor(chain: &Chain, index: usize) -> Vector3<f32>{
    let inner_to_outer_uv = chain.bones[index].get_direction();				
    let angle_between = util::getAngleBetweenDegs(chain.basebone_constraint_uv, inner_to_outer_uv);
    let constraint_angle = chain.bones[index].get_ball_constraint(); 
    return constrain_rotor(inner_to_outer_uv, chain.basebone_constraint_uv, angle_between, constraint_angle)
}


fn calc_backwards_basebone_local_rotor(chain: &Chain, index: usize) -> Vector3<f32>{
    // Note: The mBaseboneRelativeConstraintUV is updated in the FabrikStructure3D.solveForTarget()
    let inner_to_outer_uv = chain[index].get_direction();
    let angle_between = util::getAngleBetweenDegs(chain.basebone_relative_constraint_uv, inner_to_outer_uv);
    let constraint_angle = chain[index].get_ball_constraint();
    return constrain_rotor(inner_to_outer_uv, chain.basebone_relative_constraint_uv, angle_between, constraint_angle)
}

fn constrain_rotor(inner_to_outer_uv: Vector3<f32>, constraint_uv: Vector3<f32>, 
    angle_between: Rad<f32>, constraint_angle: Rad<f32>) -> Vector3<f32>{
    if angle_between > constraint_angle
    {
        return util::getAngleLimitedUnitVectorDegs(inner_to_outer_uv, constraint_uv, constraint_angle);
    }
    return inner_to_outer_uv;
}

fn calc_backwards_basebone_global_hinge(chain: &Chain, index: usize) -> Vector3<f32>{
    let joint: Joint  =  chain[index].joint;
    let rotation_axis: Vector3<f32>  =  joint.getHingeRotationAxis();
    let cw_constraint: Rad<f32>   = -joint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
    let acw_constraint: Rad<f32>  =  joint.getHingeAnticlockwiseConstraintDegs();
    
    // Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
    let mut inner_to_outer_uv: Vector3<f32> = util::projectOntoPlane(chain[index].get_direction(), rotation_axis);
            
    // If we have a global hinge which is not freely rotating then we must constrain about the reference axis
    if  !( util::approximatelyEquals(cw_constraint.0 , -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) &&
            util::approximatelyEquals(acw_constraint.0,  joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) ) 
    {        
        inner_to_outer_uv = 
            constrain_hinge(inner_to_outer_uv, joint.getHingeReferenceAxis(), rotation_axis, cw_constraint, acw_constraint)
    }
    return inner_to_outer_uv;
}

fn calc_backwards_basebone_local_hinge(chain: &Chain, index: usize) -> Vector3<f32> {
    let joint: Joint  =  chain[index].joint;
    let rotation_axis: Vector3<f32>  =  chain.basebone_relative_constraint_uv;                   // Basebone relative constraint is our hinge rotation axis!
    let cw_constraint: Rad<f32>   = -joint.getHingeClockwiseConstraintDegs();     // Clockwise rotation is negative!
    let acw_constraint: Rad<f32>  =  joint.getHingeAnticlockwiseConstraintDegs();
    
    // Get the inner-to-outer direction of this bone and project it onto the global hinge rotation axis
    let mut inner_to_outer_uv: Vector3<f32> = util::projectOntoPlane(chain[index].get_direction(), rotation_axis);
    
    // If we have a local hinge which is not freely rotating then we must constrain about the reference axis
    if  !( util::approximatelyEquals(cw_constraint.0 , -joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) &&
            util::approximatelyEquals(acw_constraint.0,  joint::MAX_CONSTRAINT_ANGLE_DEGS.0, 0.01) ) 
    {
        let reference_axis: Vector3<f32> = chain.basebone_relative_reference_constraint_uv; 
        
        inner_to_outer_uv = 
            constrain_hinge(inner_to_outer_uv, reference_axis, rotation_axis, cw_constraint, acw_constraint);
    }
    return inner_to_outer_uv;
} 

fn constrain_hinge(inner_to_outer_uv: Vector3<f32>, reference_axis: Vector3<f32>, rotation_axis: Vector3<f32>,
    cw_constraint: Rad<f32> ,acw_constraint: Rad<f32>) -> Vector3<f32>{
    let signed_angle: f32    = util::getSignedAngleBetweenDegs(reference_axis, inner_to_outer_uv, rotation_axis);
    if signed_angle > acw_constraint.0
    {	
        return  util::rotateAboutAxis(reference_axis, acw_constraint, rotation_axis).normalize();		        		
    }
    else if signed_angle < cw_constraint.0
    {	
        return util::rotateAboutAxis(reference_axis, cw_constraint, rotation_axis).normalize();			        		
    }
//if neither of the above two happened return an unmodified inner to outter uv.
return inner_to_outer_uv;
}
//363