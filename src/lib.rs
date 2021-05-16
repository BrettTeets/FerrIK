//pub mod flat;
pub mod real;

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum BoneConnectionPoint { START, END }

#[cfg(test)]
mod util_test;
