use super::{bone::Bone, chain::Chain, chain, joint, util};
use cgmath::{Vector2, Rad, InnerSpace};

pub const UP: Vector2<f32> = Vector2 {x: 0.0, y: 1.0};

pub struct Structure{
    mChains: Vec<Chain>,
    mFixedBaseMode: bool,
}

impl Structure{
    pub fn new() -> Self{
        Self{
            mChains: Vec::new(),
            mFixedBaseMode: true,
        }
    }

    //quiet the conudrum. this data structure wont work in rust.
    pub fn solveForTarget(&mut self, newTargetLocation: Vector2<f32>)
	{
		let numChains: usize = self.mChains.len();
		let mut hostChainNumber: Option<usize>;
		let mut thisChain: &Chain;
		
		//for (int loop = 0; loop < numChains; ++loop)
        for index in 0..numChains
		{
			thisChain = &mut self.mChains[index];
			
			// Is this chain connected to another chain?
			hostChainNumber = thisChain.getConnectedChainNumber();
			
			// Get the basebone constraint type of the chain we're working on
			let constraintType: chain::BaseboneConstraintType = thisChain.getBaseboneConstraintType();
			
			// If this chain is not connected to another chain and the basebone constraint type of this chain is not global absolute
			// then we must update the basebone constraint UV for LOCAL_RELATIVE and the basebone relative constraint UV for LOCAL_ABSOLUTE connection types.
			// Note: For NONE or GLOBAL_ABSOLUTE we don't need to update anything before calling updateTarget().
			if hostChainNumber != None && constraintType != chain::BaseboneConstraintType::GLOBAL_ABSOLUTE
			{	
				// Get the bone which this chain is connected to in the 'host' chain
				let hostBone: Bone = self.mChains[hostChainNumber.unwrap()].getBone( thisChain.getConnectedBoneNumber() );
				
				// If we're connecting this chain to the start location of the bone in the 'host' chain...
				if thisChain.getBoneConnectionPoint() == chain::BoneConnectionPoint::START
				{
					// ...set the base location of this bone to be the start location of the bone it's connected to.
					thisChain.setBaseLocation( hostBone.getStartLocation() );
				}
				else // If the bone connection point is BoneConnectionPoint.END...
				{	
					// ...set the base location of the chain to be the end location of the bone we're connecting to.
					thisChain.setBaseLocation( hostBone.getEndLocation() );
				}
				
				// If the basebone is constrained to the direction of the bone it's connected to...
				let hostBoneUV: Vector2<f32> = hostBone.getDirectionUV();
				if constraintType == chain::BaseboneConstraintType::LOCAL_RELATIVE
				{	
					// ...then set the basebone constraint UV to be the direction of the bone we're connected to.
					thisChain.setBaseboneConstraintUV(hostBoneUV);
				}				
				else if constraintType == chain::BaseboneConstraintType::LOCAL_ABSOLUTE
				{	
					// Note: LOCAL_ABSOLUTE directions are directions which are in the local coordinate system of the host bone.
					// For example, if the baseboneConstraintUV is Vec2f(-1.0f, 0.0f) [i.e. left], then the baseboneConnectionConstraintUV
					// will be updated to be left with regard to the host bone.
					
					// Get the angle between UP and the hostbone direction
					let angleDegs: f32 = util::getSignedAngleDegsTo(UP, hostBoneUV);
					
					// ...then apply that same rotation to this chain's basebone constraint UV to get the relative constraint UV... 
                    //To I dont know if those angle degrees are right.
					let relativeConstraintUV: Vector2<f32> = util::rotateDegs( thisChain.getBaseboneConstraintUV(), Rad(angleDegs));
					
					// ...which we then update.
					thisChain.setBaseboneRelativeConstraintUV(relativeConstraintUV);					
				}
				
				// NOTE: If the basebone constraint type is NONE then we don't do anything with the basebone constraint of the connected chain.
				
			} // End of if chain is connected to another chain section
			
			// Update the target and solve the chain
			if  !thisChain.getEmbeddedTargetMode() 
			{
				thisChain.solveForTarget(newTargetLocation);	
			}
			else
			{
				thisChain.solveForEmbeddedTarget();
			}			
			
		} // End of loop over chains
		
	} // End of updateTarget method
}