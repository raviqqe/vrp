use super::solver::Solver;
use crate::{
    cost::CostCalculator, hash_map::HashMap, problem::BaseProblem, route::Router, trace, Solution,
};
use itertools::Itertools;
use ordered_float::OrderedFloat;
use rand::{rngs::SmallRng, seq::IteratorRandom, SeedableRng};
use std::ops::Range;

const SEED: [u8; 32] = [0u8; 32];
const MAX_VEHICLE_REGION_SIZE: usize = 2;
const MAX_STOP_REGION_SIZE: usize = 6;

#[derive(Debug)]
struct RouteRegion {
    vehicle_index: usize,
    stop_range: Range<usize>,
}

pub struct RuinAndRecreateSolver<C: CostCalculator, R: Router, S: Solver> {
    initial_solver: S,
    cost_calculator: C,
    router: R,
    iteration_count: usize,
    rng: SmallRng,
}

impl<C: CostCalculator, R: Router, S: Solver> RuinAndRecreateSolver<C, R, S> {
    pub fn new(cost_calculator: C, router: R, initial_solver: S, iteration_count: usize) -> Self {
        Self {
            initial_solver,
            cost_calculator,
            router,
            iteration_count,
            rng: SmallRng::from_seed(SEED),
        }
    }

    fn choose_regions(&mut self, solution: &Solution, closest_stops: &[usize]) -> Vec<RouteRegion> {
        let (first_stop_index, second_stop_index) = closest_stops
            .iter()
            .enumerate()
            .map(|(other, one)| (*one, other))
            .choose(&mut self.rng)
            .expect("stop pair");

        let pairs = [first_stop_index, second_stop_index]
            .into_iter()
            .flat_map(|stop_index| {
                solution
                    .routes()
                    .iter()
                    .enumerate()
                    .find_map(|(vehicle_index, route)| {
                        route
                            .contains(&stop_index)
                            .then_some((vehicle_index, stop_index))
                    })
            })
            .unique_by(|(vehicle_index, _)| vehicle_index)
            .collect::<Vec<_>>();

        pairs
            .iter()
            .map(|(vehicle_index, stop_index)| {
                self.choose_region(
                    solution,
                    vehicle_index,
                    stop_index,
                    MAX_STOP_REGION_SIZE / pairs.len(),
                )
            })
            .collect()
    }

    fn choose_region(
        &mut self,
        solution: &Solution,
        vehicle_index: usize,
        stop_index: usize,
        stop_region_size: usize,
    ) -> Range<usize> {
        let len = solution.routes()[vehicle_index].len();
        let index = (0..len.saturating_sub(stop_region_size))
            .choose(&mut self.rng)
            .unwrap_or(0);

        RouteRegion {
            vehicle_index,
            stop_range: index..(index + stop_region_size).min(len),
        }
    }

    fn optimize_regions(
        &mut self,
        initial_solution: &Solution,
        regions: &[RouteRegion],
    ) -> Solution {
        let mut solution = initial_solution.clone();

        for region in regions {
            solution = solution.ruin_route(region.vehicle_index, region.stop_range.clone())
        }

        let cost = self.cost_calculator.calculate(&solution);

        let mut solutions = HashMap::default();
        solutions.insert(solution.clone(), cost);
        let mut new_solutions = vec![];

        for _ in regions.iter().flat_map(|region| region.stop_range.clone()) {
            new_solutions.clear();

            for solution in solutions.keys() {
                for stop_index in regions
                    .iter()
                    .flat_map(|region| Self::region_stop_indexes(region, initial_solution))
                {
                    if solution.has_stop(stop_index) {
                        continue;
                    }

                    for region in regions {
                        let solution = solution.insert_stop(
                            region.vehicle_index,
                            region.stop_range.start,
                            stop_index,
                        );
                        let cost = self.cost_calculator.calculate(&solution);

                        if cost.is_finite() {
                            new_solutions.push((solution, cost));
                        }
                    }
                }
            }

            solutions.extend(new_solutions.drain(..));
        }

        solutions
            .into_iter()
            .min_by(|(_, one), (_, other)| OrderedFloat(*one).cmp(&OrderedFloat(*other)))
            .expect("at least one solution")
            .0
    }

    fn region_stop_indexes<'a>(
        region: &'a RouteRegion,
        solution: &'a Solution,
    ) -> impl Iterator<Item = usize> + 'a {
        region
            .stop_range
            .clone()
            .map(|index| solution.routes()[region.vehicle_index][index])
    }
}

impl<C: CostCalculator, R: Router, S: Solver> Solver for RuinAndRecreateSolver<C, R, S> {
    fn solve(&mut self, problem: impl BaseProblem) -> Solution {
        if problem.vehicle_count() == 0 {
            return Solution::new(vec![]);
        } else if problem.stop_count() == 0 {
            return Solution::new(
                (0..problem.vehicle_count())
                    .map(|_| vec![].into())
                    .collect(),
            );
        } else if problem.stop_count() == 1 {
            return self.initial_solver.solve(problem);
        }

        let closest_stops = ((0..problem.stop_count()).map(|one| {
            (0..problem.stop_count())
                .filter(|other| one != *other)
                .min_by_key(|other| {
                    OrderedFloat(
                        self.router
                            .route(problem.stop_location(one), problem.stop_location(*other)),
                    )
                })
                .expect("stop index")
        }))
        .collect::<Vec<_>>();

        let mut solution = self.initial_solver.solve(problem);
        let mut cost = self.cost_calculator.calculate(&solution);

        for _ in 0..self.iteration_count {
            let regions = self.choose_regions(&solution, &closest_stops);
            trace!("regions: {:?}", &regions);
            let new_solution = self.optimize_regions(&solution, &regions);
            let new_cost = self.cost_calculator.calculate(&new_solution);

            // TODO Consider a non-greedy strategy like simulated annealing.
            // TODO Save multiple solutions.
            // TODO Decide if a solution is good enough already.
            if new_cost < cost {
                trace!("new solution found!");
                trace!("solution: {:?}", solution);
                trace!("cost: {:?}", cost);

                solution = new_solution;
                cost = new_cost;
            }
        }

        solution
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        cost::{DeliveryCostCalculator, DistanceCostCalculator},
        route::CrowRouter,
        solve::NearestNeighborSolver,
        Location, SimpleProblem, Stop, Vehicle,
    };

    const DISTANCE_COST: f64 = 1.0;
    const MISSED_DELIVERY_COST: f64 = 1e9;
    const ITERATION_COUNT: usize = 100;

    static ROUTER: CrowRouter = CrowRouter::new();

    fn solve(problem: &SimpleProblem) -> Solution {
        RuinAndRecreateSolver::new(
            DeliveryCostCalculator::new(
                DistanceCostCalculator::new(&ROUTER, problem),
                problem.stops().len(),
                MISSED_DELIVERY_COST,
                DISTANCE_COST,
            ),
            &ROUTER,
            NearestNeighborSolver::new(&ROUTER),
            ITERATION_COUNT,
        )
        .solve(problem)
    }

    #[test]
    fn do_nothing() {
        let problem = SimpleProblem::new(vec![Vehicle::new()], vec![]);

        assert_eq!(solve(&problem), Solution::new(vec![vec![].into()]));
    }

    #[test]
    fn keep_one_stop() {
        let problem = SimpleProblem::new(
            vec![Vehicle::new()],
            vec![Stop::new(Location::new(0.0, 0.0))],
        );

        assert_eq!(solve(&problem), Solution::new(vec![vec![0].into()]));
    }

    #[test]
    fn keep_two_stops() {
        let problem = SimpleProblem::new(
            vec![Vehicle::new()],
            vec![
                Stop::new(Location::new(0.0, 0.0)),
                Stop::new(Location::new(1.0, 0.0)),
            ],
        );

        assert_eq!(solve(&problem), Solution::new(vec![vec![0, 1].into()]));
    }

    #[test]
    fn keep_three_stops() {
        let problem = SimpleProblem::new(
            vec![Vehicle::new()],
            vec![
                Stop::new(Location::new(0.0, 0.0)),
                Stop::new(Location::new(1.0, 0.0)),
                Stop::new(Location::new(2.0, 0.0)),
            ],
        );

        assert_eq!(solve(&problem), Solution::new(vec![vec![0, 1, 2].into()]));
    }

    #[test]
    fn even_workload() {
        let problem = SimpleProblem::new(
            vec![Vehicle::new(), Vehicle::new()],
            vec![
                Stop::new(Location::new(0.0, 0.0)),
                Stop::new(Location::new(1.0, 0.0)),
                Stop::new(Location::new(2.0, 0.0)),
            ],
        );

        assert!(solve(&problem).routes().iter().all(|stops| stops.len() < 3));
    }
}
