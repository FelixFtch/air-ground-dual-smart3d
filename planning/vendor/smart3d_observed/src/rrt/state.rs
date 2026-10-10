use num_traits::float::Float;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use std::ops::{Add, Div, Index, Mul, Sub};

/// Represents a vector in N-dimensional space.
/// Each dimension is represented by a generic floating point value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RealVectorState<F: Float, const N: usize> {
    values: [F; N],
}

impl<F: Float, const N: usize> RealVectorState<F, N> {
    /// Constructs a new RealVectorState from an array of coordinates.
    ///
    /// Parameters:
    /// - `values`: The coordinates of the RealVectorState.
    ///
    /// Returns:
    /// The RealVectorState.
    pub fn new(values: [F; N]) -> Self {
        Self { values }
    }

    /// Constructs a new RealVectorState from a vector of coordinates.
    ///
    /// Parameters:
    /// - `values`: The coordinates of the RealVectorState.
    ///
    /// Returns:
    /// The RealVectorState.
    pub fn from_vec(values: Vec<F>) -> Result<Self, &'static str> {
        if values.len() != N {
            return Err("Invalid number of values");
        }
        let mut arr = [F::zero(); N];
        for i in 0..N {
            arr[i] = values[i];
        }
        Ok(Self { values: arr })
    }

    /// Returns the coordinates of the RealVectorState.
    pub fn values(&self) -> &[F; N] {
        &self.values
    }

    /// Returns the coordinates of the RealVectorState as a vector.
    pub fn to_vec(&self) -> Vec<F> {
        self.values.to_vec()
    }

    /// Computes the dot product of the RealVectorState with another RealVectorState.
    ///
    /// Parameters:
    /// - `other`: The other RealVectorState.
    ///
    /// Returns:
    /// The dot product of the two RealVectorStates.
    pub fn dot(&self, other: &Self) -> F {
        let mut sum = F::zero();
        for i in 0..N {
            sum = sum + self.values[i] * other.values[i];
        }
        sum
    }

    /// Computes the squared norm of the RealVectorState. Dot product of the RealVectorState with itself.
    pub fn norm_squared(&self) -> F {
        self.dot(self)
    }

    /// Computes the norm of the RealVectorState.
    pub fn norm(&self) -> F {
        self.norm_squared().sqrt()
    }

    /// Compute the squared euclidean distance between two RealVectorStates.
    pub fn euclidean_distance_squared(&self, other: &Self) -> F {
        (self - other).norm_squared()
    }

    /// Compute the euclidean distance between two RealVectorStates.
    pub fn euclidean_distance(&self, other: &Self) -> F {
        self.euclidean_distance_squared(other).sqrt()
    }
}

impl<F: Float, const N: usize> Index<usize> for RealVectorState<F, N> {
    type Output = F;

    fn index(&self, index: usize) -> &Self::Output {
        &self.values[index]
    }
}

impl<F: Float, const N: usize> Add for RealVectorState<F, N> {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        let mut values = [F::zero(); N];
        for i in 0..N {
            values[i] = self.values[i] + other.values[i];
        }
        RealVectorState::new(values)
    }
}

impl<F: Float, const N: usize> Add for &RealVectorState<F, N> {
    type Output = RealVectorState<F, N>;

    fn add(self, other: Self) -> RealVectorState<F, N> {
        let mut values = [F::zero(); N];
        for i in 0..N {
            values[i] = self.values[i] + other.values[i];
        }
        RealVectorState::new(values)
    }
}

impl<F: Float, const N: usize> Sub for RealVectorState<F, N> {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        let mut values = [F::zero(); N];
        for i in 0..N {
            values[i] = self.values[i] - other.values[i];
        }
        RealVectorState::new(values)
    }
}

impl<F: Float, const N: usize> Sub for &RealVectorState<F, N> {
    type Output = RealVectorState<F, N>;

    fn sub(self, other: Self) -> RealVectorState<F, N> {
        let mut values = [F::zero(); N];
        for i in 0..N {
            values[i] = self.values[i] - other.values[i];
        }
        RealVectorState::new(values)
    }
}

impl<F: Float, const N: usize> Mul<F> for RealVectorState<F, N> {
    type Output = Self;

    fn mul(self, scalar: F) -> Self {
        let mut values = [F::zero(); N];
        for i in 0..N {
            values[i] = self.values[i] * scalar;
        }
        RealVectorState::new(values)
    }
}

impl<F: Float, const N: usize> Mul<F> for &RealVectorState<F, N> {
    type Output = RealVectorState<F, N>;

    fn mul(self, scalar: F) -> RealVectorState<F, N> {
        let mut values = [F::zero(); N];
        for i in 0..N {
            values[i] = self.values[i] * scalar;
        }
        RealVectorState::new(values)
    }
}

impl<F: Float, const N: usize> Div<F> for RealVectorState<F, N> {
    type Output = Self;

    fn div(self, scalar: F) -> Self {
        let mut values = [F::zero(); N];
        for i in 0..N {
            values[i] = self.values[i] / scalar;
        }
        RealVectorState::new(values)
    }
}

impl<F: Float, const N: usize> Div<F> for &RealVectorState<F, N> {
    type Output = RealVectorState<F, N>;

    fn div(self, scalar: F) -> RealVectorState<F, N> {
        let mut values = [F::zero(); N];
        for i in 0..N {
            values[i] = self.values[i] / scalar;
        }
        RealVectorState::new(values)
    }
}

// Serialize implementation: converts to Vec and serializes
impl<F, const N: usize> Serialize for RealVectorState<F, N>
where
    F: Float + Serialize,
{
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.to_vec().serialize(serializer)
    }
}

// Deserialize to Vec and convert to RealVectorState
impl<'de, F, const N: usize> Deserialize<'de> for RealVectorState<F, N>
where
    F: Float + Deserialize<'de>,
{
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        // Deserialize directly into a Vec<F>
        let vec: Vec<F> = Vec::deserialize(deserializer)?;

        // Check if the vector has the correct length
        if vec.len() != N {
            return Err(serde::de::Error::invalid_length(
                vec.len(),
                &format!("expected {} elements", N).as_str(),
            ));
        }

        Ok(RealVectorState::from_vec(vec).unwrap())
    }
}
