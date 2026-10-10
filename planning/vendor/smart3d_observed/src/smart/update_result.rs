use serde::{Deserialize, Serialize};
use std::fmt;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SmartUpdateResult {
    NoReplanningNeeded,
    ReplanningFailed,
    ReplanningError(String),
    ReplanningSuccessful,
    InCollision,
}

impl SmartUpdateResult {
    pub fn is_success(&self) -> bool {
        match self {
            SmartUpdateResult::ReplanningSuccessful => true,
            SmartUpdateResult::NoReplanningNeeded => true,
            _ => false,
        }
    }

    pub fn is_failure(&self) -> bool {
        !self.is_success()
    }

    pub fn replanned(&self) -> bool {
        match self {
            SmartUpdateResult::ReplanningSuccessful => true,
            SmartUpdateResult::ReplanningFailed => true,
            _ => false,
        }
    }
}

impl fmt::Display for SmartUpdateResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            SmartUpdateResult::NoReplanningNeeded => "NoReplanningNeeded",
            SmartUpdateResult::ReplanningFailed => "ReplanningFailed",
            SmartUpdateResult::ReplanningError(_) => "ReplanningError",
            SmartUpdateResult::ReplanningSuccessful => "ReplanningSuccessful",
            SmartUpdateResult::InCollision => "InCollision",
        };
        write!(f, "{}", s)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ReplanningTriggerCondition {
    PathUnsafe,
    RobotInOHZ,
}

impl fmt::Display for ReplanningTriggerCondition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let s = match self {
            ReplanningTriggerCondition::PathUnsafe => "PathUnsafe",
            ReplanningTriggerCondition::RobotInOHZ => "RobotInOHZ",
        };
        write!(f, "{}", s)
    }
}
