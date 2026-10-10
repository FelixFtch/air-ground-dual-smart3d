use crate::rrt::state::RealVectorState;
use num_traits::Float;
use rand::distributions::{uniform::SampleUniform, Bernoulli, Distribution, Uniform};

/// A trait for sampling distributions.
pub trait SamplingDistribution<F: Float, const N: usize> {
    /// Samples a state from the distribution.
    fn sample(&mut self) -> RealVectorState<F, N>;
}

/// A uniform distribution for sampling states.
/// Each dimension has a range of values.
pub struct UniformDistribution<F: Float + SampleUniform, const N: usize> {
    uniforms: [Uniform<F>; N],
    rng: rand::rngs::ThreadRng,
}

impl<F: Float + SampleUniform, const N: usize> UniformDistribution<F, N> {
    /// Constructs a new uniform distribution.
    /// Parameters:
    /// - `ranges`: The ranges for each dimension.
    /// Returns:
    /// The uniform distribution.
    pub fn new(ranges: [(F, F); N]) -> Self {
        let uniforms: [Uniform<F>; N] =
            std::array::from_fn(|i| Uniform::new_inclusive(ranges[i].0, ranges[i].1));
        Self {
            uniforms,
            rng: rand::thread_rng(),
        }
    }
}

impl<F: Float + SampleUniform, const N: usize> SamplingDistribution<F, N>
    for UniformDistribution<F, N>
{
    fn sample(&mut self) -> RealVectorState<F, N> {
        let values: [F; N] = std::array::from_fn(|i| self.uniforms[i].sample(&mut self.rng));
        RealVectorState::new(values)
    }
}

/// A uniform distribution that occasionally samples the goal with a given goal_bias probability.
pub struct GoalBiasedUniformDistribution<F: Float + SampleUniform, const N: usize> {
    uniform: UniformDistribution<F, N>, // Uniform distribution for sampling points.
    bernoulli: Bernoulli,               // Bernoulli distribution for goal bias.
    goal: RealVectorState<F, N>,        // The goal point.
    rng: rand::rngs::ThreadRng,
}

impl<F: Float + SampleUniform, const N: usize> GoalBiasedUniformDistribution<F, N> {
    /// Constructs a new goal-biased uniform distribution.
    /// Parameters:
    /// - `ranges`: The ranges for each dimension.
    /// - `goal`: The goal point.
    /// - `goal_bias`: The probability of sampling the goal.
    /// Returns:
    /// The goal-biased uniform distribution.
    pub fn new(
        ranges: [(F, F); N],
        goal: RealVectorState<F, N>,
        goal_bias: f64,
    ) -> Result<Self, &'static str> {
        if goal_bias < 0.0 || goal_bias > 1.0 {
            return Err("goal_bias must be in the range [0, 1]");
        }
        Ok(Self {
            uniform: UniformDistribution::new(ranges),
            bernoulli: Bernoulli::new(goal_bias).unwrap(),
            goal,
            rng: rand::thread_rng(),
        })
    }
}

impl<F: Float + SampleUniform, const N: usize> SamplingDistribution<F, N>
    for GoalBiasedUniformDistribution<F, N>
{
    fn sample(&mut self) -> RealVectorState<F, N> {
        if self.bernoulli.sample(&mut self.rng) {
            self.goal.clone()
        } else {
            self.uniform.sample()
        }
    }
}
