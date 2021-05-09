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
