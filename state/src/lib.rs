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
#[repr(u8)]
pub enum StateMachine {
    Paused = 1,
    Homing = 2,
    Calibrating = 3,
    Looping = 4,
    Exploring = 5,
}

impl TryFrom<u8> for StateMachine {
    type Error = ();
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        Ok(match val {
            1 => Self::Paused,
            2 => Self::Homing,
            3 => Self::Calibrating,
            4 => Self::Looping,
            5 => Self::Exploring,
            _ => return Err(()),
        })
    }
}

impl Into<u8> for StateMachine {
    fn into(self) -> u8 {
        match self {
            StateMachine::Paused => 1,
            StateMachine::Homing => 2,
            StateMachine::Calibrating => 3,
            StateMachine::Looping => 4,
            StateMachine::Exploring => 5,
        }
    }
}
