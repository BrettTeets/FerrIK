use crate::{chain};
use cgmath::Vector3;

pub trait Structure{
    //Adds a chain to this structure. Does not connect it to anything.
    fn add_chain(&mut self, chain: dyn chain::Chain);
  
    //Will connect a chain to another chain at a given bone. Does this add a chain to the stucture or
    //is this for use my chains in this structure?
    fn connect_chain(&mut self, new_chain: dyn chain::Chain, existing_chain_number: usize, existing_bone_number: usize);
  
    //This is an overloaded method that ask if a chain should go on the end or beginiing of a bone.
    //fn connectChain(T newChain, int existingChainNumber, int existingBoneNumber, BoneConnectionPoint boneConnectionPoint);	
  
    //gets a chain in this structure... This seems wrong this should return a reference maybe?
    fn get_chain(&self, chain_number: usize) -> dyn chain::Chain;
  
    //returns the id for this structure.
    fn get_id(&self) -> usize;
    
    //returns the number of chains in this strucutre.
    fn get_num_chains(&self) -> usize;
  
    //this would brake my api I think, not including it just commenting it out for now.
    //public void setName(String name);
  
    //Will attempt to solve all the chains in this structure towards the given target location.
    //the original did not resolve chains that were in embeed taget mode towards this target.
    //instead they were called but were resolved towards their embedded target.
    fn solve_for_target(&mut self, new_target_location: Vector3<f32>);
}