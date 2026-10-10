pub trait TerminationCondition {
    /// True if the termination condition is satisfied.
    fn evaluate(&mut self) -> bool;

    /// Resets the termination condition.
    fn reset(&mut self);
}

#[derive(Copy, Clone, Debug)]
pub struct MaxIterationsTermination {
    max_iterations: usize,
    current_iteration: usize,
}

impl MaxIterationsTermination {
    pub fn new(max_iterations: usize) -> Self {
        Self {
            max_iterations,
            current_iteration: 0,
        }
    }

    pub fn current_iteration(&self) -> usize {
        self.current_iteration
    }
}

impl TerminationCondition for MaxIterationsTermination {
    fn evaluate(&mut self) -> bool {
        self.current_iteration += 1;
        self.current_iteration > self.max_iterations
    }

    fn reset(&mut self) {
        self.current_iteration = 0;
    }
}

#[derive(Copy, Clone, Debug)]
pub struct MaxTimeTermination {
    max_time: std::time::Duration,
    start_time: std::time::Instant,
}

impl MaxTimeTermination {
    pub fn new(max_time: std::time::Duration) -> Self {
        Self {
            max_time,
            start_time: std::time::Instant::now(),
        }
    }

    pub fn time_elapsed(&self) -> std::time::Duration {
        self.start_time.elapsed()
    }
}

impl TerminationCondition for MaxTimeTermination {
    fn evaluate(&mut self) -> bool {
        self.start_time.elapsed() >= self.max_time
    }

    fn reset(&mut self) {
        self.start_time = std::time::Instant::now();
    }
}

pub struct OrTermination {
    conditions: Vec<Box<dyn TerminationCondition>>,
}

impl OrTermination {
    pub fn new(conditions: Vec<Box<dyn TerminationCondition>>) -> Self {
        Self { conditions }
    }
}

impl TerminationCondition for OrTermination {
    fn evaluate(&mut self) -> bool {
        self.conditions
            .iter_mut()
            .any(|condition| condition.evaluate())
    }

    fn reset(&mut self) {
        self.conditions
            .iter_mut()
            .for_each(|condition| condition.reset());
    }
}

pub struct AndTermination {
    conditions: Vec<Box<dyn TerminationCondition>>,
}

impl AndTermination {
    pub fn new(conditions: Vec<Box<dyn TerminationCondition>>) -> Self {
        Self { conditions }
    }
}

impl TerminationCondition for AndTermination {
    fn evaluate(&mut self) -> bool {
        self.conditions
            .iter_mut()
            .all(|condition| condition.evaluate())
    }

    fn reset(&mut self) {
        self.conditions
            .iter_mut()
            .for_each(|condition| condition.reset());
    }
}

pub struct ConstantTerminationCondition {
    value: bool,
}

impl ConstantTerminationCondition {
    pub fn new(value: bool) -> Self {
        Self { value }
    }
}

impl TerminationCondition for ConstantTerminationCondition {
    fn evaluate(&mut self) -> bool {
        self.value
    }

    fn reset(&mut self) {}
}
