use crate::obstacles::SphericalObstacle;
use num_traits::Float;

pub trait DynamicObstacle<F: Float> {
    fn update(&mut self, dt: F);
}

pub trait DynamicSphericalObstacle<F: Float, const N: usize>:
    DynamicObstacle<F> + SphericalObstacle<F, N>
{
}
impl<F: Float, const N: usize, T: DynamicObstacle<F> + SphericalObstacle<F, N>>
    DynamicSphericalObstacle<F, N> for T
{
}
