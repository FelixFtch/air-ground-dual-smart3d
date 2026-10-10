use core::panic;
use num_traits::Float;
use std::cmp::Ordering;

#[derive(Debug, Copy, Clone)]
pub struct OrderedFloat<T: PartialOrd>(pub T);

impl<F: Float> From<F> for OrderedFloat<F> {
    fn from(float: F) -> Self {
        if float.is_nan() {
            panic!("Cannot create OrderedFloat from NaN")
        }
        OrderedFloat(float)
    }
}

impl<T: PartialOrd> PartialEq for OrderedFloat<T> {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl<T: PartialOrd> Eq for OrderedFloat<T> {}

impl<T: PartialOrd> PartialOrd for OrderedFloat<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.partial_cmp(&other.0)
    }
}

impl<T: PartialOrd> Ord for OrderedFloat<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        // This unwrap is safe only if no value is NaN.
        self.partial_cmp(other).expect("Cannot compare NaN values")
    }
}
