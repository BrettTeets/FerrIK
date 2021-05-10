use super::{chain::Chain, chain, bone::Bone, util};
use cgmath::{Vector3, Matrix3, InnerSpace};

pub struct Structure{
    mChains: Vec<Chain>,
}

impl Structure{
    pub fn new() -> Self{
        Self{
            mChains: Vec::new(),
        }
    }

    pub fn addChain(&mut self, chain: Chain)
	{
		self.mChains.push(chain);		
	}

    pub fn removeChain(&mut self, chainIndex: usize)
	{
		self.mChains.remove(chainIndex);		
	}

    //this is creating a circular reference even if temporarily and rust hates it.
    /*
    pub fn connectChain(&mut self, newChain: Chain,  existingChainNumber: usize, existingBoneNumber: usize)
	{	
		// Does this chain exist? If not throw an IllegalArgumentException
		if existingChainNumber > self.mChains.len()
		{
			panic!("Cannot connect to chain {} - no such chain (remember that chains are zero indexed).", existingChainNumber);
		}
		
		// Do we have this bone in the specified chain? If not throw an IllegalArgumentException
		if existingBoneNumber > self.mChains[existingChainNumber].getNumBones()
		{
			panic!("Cannot connect to bone {} of chain {} - no such bone (remember that bones are zero indexed).", existingBoneNumber, existingChainNumber);
		}
			
		// Make a copy of the provided chain so any changes made to the original do not affect this chain
        //chain doesn't impl clone which is what this should do I think, I am not sure
        let relativeChain: Chain = newChain;
		
		// Connect the copy of the provided chain to the specified chain and bone in this structure
		relativeChain.connectToStructure(self, existingChainNumber, existingBoneNumber);
		
		// The chain as we were provided should be centred on the origin, so we must now make it
		// relative to the start location of the given bone in the given chain.
		
		
		// Get the connection point so we know to connect at the start or end location of the bone we're connecting to
		let connectionPoint: crate::BoneConnectionPoint = self.getChain(existingChainNumber).getBone(existingBoneNumber).getBoneConnectionPoint();
		let connectionLocation: Vector3<f32>;
		if connectionPoint == crate::BoneConnectionPoint::START
		{
			connectionLocation = self.mChains[existingChainNumber].getBone(existingBoneNumber).getStartLocation();
		}
		else // If it's BoneConnectionPoint.END then we set the connection point to be the end location of the bone we're connecting to
		{
			connectionLocation = self.mChains[existingChainNumber].getBone(existingBoneNumber).getEndLocation();
		}		
		relativeChain.setBaseLocation(connectionLocation);
		
		// When we have a chain connected to a another 'host' chain, the chain is which is connecting in
		// MUST have a fixed base, even though that means the base location is 'fixed' to the connection
		// point on the host chain, rather than a static location.
		relativeChain.setFixedBaseMode(true);
		
		// Translate the chain we're connecting to the connection point
		//for (int loop = 0; loop < relativeChain.getNumBones(); ++loop)
        for index in 0..relativeChain.getNumBones()
		{
			let origStart: Vector3<f32> = relativeChain.getBone(index).getStartLocation();
			let origEnd: Vector3<f32>   = relativeChain.getBone(index).getEndLocation();
			
			let translatedStart: Vector3<f32> = origStart + (connectionLocation);
			let translatedEnd: Vector3<f32>   = origEnd +(connectionLocation);
			
			relativeChain.getBone(index).setStartLocation(translatedStart);
			relativeChain.getBone(index).setEndLocation(translatedEnd);
		}
		
		self.addChain(relativeChain);
	}
    */

    pub fn getNumChains(&self) -> usize 
    { return self.mChains.len(); }

    pub fn getChain(&self, chainNumber: usize) -> &Chain
    { return &self.mChains[chainNumber]; }

    pub fn setFixedBaseMode(&mut self, fixedBaseMode: bool)
	{
        for index in 0..self.mChains.len(){
			self.mChains[index].setFixedBaseMode(fixedBaseMode);
		}
	}

    pub fn solveForTarget(&mut self, newTargetLocation: Vector3<f32>)
	{
		//let numChains: usize = self.mChains.len();
		let mut connectedChainNumber: Option<usize>;
		
		// Loop over all chains in this structure...
		//for (int loop = 0; loop < numChains; ++loop)
        for index in 0..self.mChains.len()
		{
			// Get this chain, and get the number of the chain in this structure it's connected to (if any)
			//let thisChain: Chain = self.mChains[index];
			connectedChainNumber    = self.mChains[index].getConnectedChainNumber();
			
			// If this chain isn't connected to another chain then update as normal...
			if connectedChainNumber == None
			{	
				self.mChains[index].solve_for_target(newTargetLocation);
			}
			else // ...however, if this chain IS connected to another chain...
			{	
				// ... get the host chain and bone which this chain is connected to
                let host_id = connectedChainNumber.unwrap();
				//let hostChain: Chain = self.mChains[host_id];
                let connect_id = self.mChains[index].getConnectedBoneNumber().unwrap();
				//let hostBone: &mut Bone   = self.mChains[host_id].getBone(connect_id);
                let hostBoneConnection = self.mChains[host_id].getBone(connect_id).getBoneConnectionPoint();
                if hostBoneConnection == crate::BoneConnectionPoint::START 
                {
                    let hostBoneStartLocation = self.mChains[host_id].getBone(connect_id).getStartLocation(); 
                    self.mChains[index].setBaseLocation( hostBoneStartLocation ); 
                }
				else
                { 
                    let hostBoneEndLocation = self.mChains[host_id].getBone(connect_id).getEndLocation();
                    self.mChains[index].setBaseLocation( hostBoneEndLocation   ); 
                }
				
				// Now that we've clamped the base location of this chain to the start or end point of the bone in the chain we are connected to, it's
				// time to deal with any base bone constraints...
				
				// What type of base bone constraint is this (connected to another) chain using? 
				let constraintType: chain::BaseboneConstraintType = self.mChains[index].getBaseboneConstraintType();
				match constraintType
				{
					// None or global basebone constraints? Nothing to do, because these will be handled in FabrikChain3D.solveIK() as we do not
					// need information from another chain to handle them.
					chain::BaseboneConstraintType::None => (),         // Nothing to do because there's no basebone constraint
					chain::BaseboneConstraintType::GlobalRotor => (), // Nothing to do because the basebone constraint is not relative to bones in other chains in this structure
					chain::BaseboneConstraintType::GlobalHinge => (), // Nothing to do because the basebone constraint is not relative to bones in other chains in this structure	
					// If we have a local rotor or hinge constraint then we must calculate the relative basebone constraint before calling updateTarget
					chain::BaseboneConstraintType::LocalRotor => {
                        //TODO: turn these two into a one function.
						// Get the direction of the bone this chain is connected to and create a rotation matrix from it.
						let connectionBoneMatrix: Matrix3<f32> = util::createRotationMatrix( self.mChains[host_id].getBone(connect_id).getDirectionUV() );
						
						// We'll then get the basebone constraint UV and multiply it by the rotation matrix of the connected bone 
						// to make the basebone constraint UV relative to the direction of bone it's connected to.
						let relativeBaseboneConstraintUV: Vector3<f32> = connectionBoneMatrix * (self.mChains[index].getBaseboneConstraintUV() ).normalize();
							
						// Update our basebone relative constraint UV property
						self.mChains[index].set_basebone_relative_constraint_uv(relativeBaseboneConstraintUV);
						
						// Updat the relative reference constraint UV if we hav a local hinge
						if constraintType == chain::BaseboneConstraintType::LocalHinge
						{
                            let referenceAxis = self.mChains[index].getBone(0).getJoint().getHingeReferenceAxis();
							self.mChains[index].set_basebone_relative_reference_constraint_uv( connectionBoneMatrix * ( referenceAxis ) );
						}
					},
					chain::BaseboneConstraintType::LocalHinge => {
						// Get the direction of the bone this chain is connected to and create a rotation matrix from it.
						let connectionBoneMatrix: Matrix3<f32> = util::createRotationMatrix( self.mChains[host_id].getBone(connect_id).getDirectionUV() );
						
						// We'll then get the basebone constraint UV and multiply it by the rotation matrix of the connected bone 
						// to make the basebone constraint UV relative to the direction of bone it's connected to.
						let relativeBaseboneConstraintUV: Vector3<f32> = connectionBoneMatrix * (self.mChains[index].getBaseboneConstraintUV() ).normalize();
							
						// Update our basebone relative constraint UV property
						self.mChains[index].set_basebone_relative_constraint_uv(relativeBaseboneConstraintUV);
						
						// Updat the relative reference constraint UV if we hav a local hinge
						if constraintType == chain::BaseboneConstraintType::LocalHinge
						{
                            let refferenceAxis = self.mChains[index].getBone(0).getJoint().getHingeReferenceAxis();
							self.mChains[index].set_basebone_relative_reference_constraint_uv( connectionBoneMatrix * ( refferenceAxis ) );
						}
					},
					
					// No need for a default - constraint types are enums and we've covered them all.
				}
						
				// NOTE: If the base bone constraint type is NONE then we don't do anything with the base bone constraint of the connected chain.
				
				// Finally, update the target and solve the chain
				// Update the target and solve the chain
				if  !self.mChains[index].getEmbeddedTargetMode() 
				{
					self.mChains[index].solve_for_target(newTargetLocation);	
				}
				else
				{
					self.mChains[index].solve_for_embedded_target();
				}
				
			} // End of if chain is connected to another chain section
			
		} // End of loop over chains

	} // End of updateTarget method
}