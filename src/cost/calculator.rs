mod delivery;

use crate::Solution;
pub use delivery::DeliveryCostCalculator;

pub trait CostCalculator {
    fn calculate(&mut self, solution: &Solution) -> f64;

    fn calculate_lower_bound(&mut self, solution: &Solution) -> f64 {
        self.calculate(solution)
    }
}
