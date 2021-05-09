pub fn  approximatelyEquals( a: f32,  b: f32,  tolerance: f32) -> bool
{
    if (a - b).abs() <= tolerance {
        return true;
    }
    else{
        return false;
    } 
}