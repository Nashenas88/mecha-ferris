#![no_std]

use nalgebra::{Quaternion, Translation3, Unit, UnitQuaternion, Vector3};

pub struct RobotState {
    pub state_machine: StateMachine,
    pub speed: f32,
    pub angular_velocity: f32,
    pub motion_vector: Vector3<f32>,
    pub body_translation: Translation3<f32>,
    pub body_rotation: UnitQuaternion<f32>,
    pub leg_radius: f32,
    pub battery_level: u32,
    pub battery_update_interval_ms: u32,
}

impl RobotState {
    pub const fn new() -> Self {
        Self {
            state_machine: StateMachine::Paused,
            speed: 0.0,
            angular_velocity: 0.0,
            motion_vector: Vector3::new(0.0, 0.0, 0.0),
            body_translation: Translation3::new(0.0, 0.0, 0.0),
            // Correct: only the unchecked constructor is const, and a scalar of 1 with imaginary
            // units set to 0 represents no rotation.
            body_rotation: Unit::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            leg_radius: 0.0,
            battery_level: 0,
            battery_update_interval_ms: 0,
        }
    }
}

impl Default for RobotState {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Copy, Clone, Debug)]
pub enum StateMachine {
    Paused,
    Homing,
    Calibrating,
    Looping,
    Exploring,
}
