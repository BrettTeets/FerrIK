pub mod bone;
#[allow(non_snake_case)] 
pub mod bone3D;
pub mod joint;
#[allow(non_snake_case)]
pub mod joint3D;
pub mod chain;
#[allow(non_snake_case)]
pub mod chain3D;
pub mod structure;

pub mod util;
pub mod flat;
pub mod real;

#[derive(Clone, Copy, PartialEq)]
pub enum BoneConnectionPoint { START, END }

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
