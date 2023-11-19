#![no_std]

pub use nalgebra::{Quaternion, Translation3, Unit, UnitQuaternion, Vector3};

#[derive(Copy, Clone)]
pub struct JointDto {
    pub servo_idx: u8,
    pub cal: [CalibrationDto; 4],
}

#[derive(Copy, Clone)]
pub struct CalibrationDto {
    pub kind: CalibrationKind,
    pub pulse_ms: f32,
}

#[derive(Copy, Clone)]
pub enum CalibrationKind {
    Home,
    Mid,
    Min,
    Max,
}

impl CalibrationKind {
    pub fn to_u8(self) -> u8 {
        match self {
            Self::Home => 0,
            Self::Mid => 1,
            Self::Min => 2,
            Self::Max => 3,
        }
    }
}

pub struct RobotState<J, const NUM_SERVOS_PER_LEG: usize, const NUM_LEGS: usize> {
    /// Track the current state of the robot.
    pub state_machine: StateMachine,
    /// Multiplied by the default duration of the animation. 1.0 is normal speed, 0.5 is half speed,
    /// 2.0 is double speed, etc.
    pub animation_factor: f32,
    /// How fast does the robot turn when it's rotating.
    pub angular_velocity: f32,
    /// What direction/speed is the robot walking in.
    pub motion_vector: Vector3<f32>,
    /// What offset does the robot's base have from the origin.
    pub body_translation: Translation3<f32>,
    /// What rotation does the robot's base have from its translated base.
    pub body_rotation: UnitQuaternion<f32>,
    /// What radius do the feet use as their standing location.
    pub leg_radius: f32,
    /// The current battery level of the robot.
    pub battery_level: u32,
    /// How often to send battery updates to the host.
    pub battery_update_interval_ms: u32,
    /// Joint details,
    pub joints: Option<[[J; NUM_SERVOS_PER_LEG]; NUM_LEGS]>,
}

impl<J, const NUM_SERVOS_PER_LEG: usize, const NUM_LEGS: usize>
    RobotState<J, NUM_SERVOS_PER_LEG, NUM_LEGS>
{
    pub const fn new() -> Self {
        Self {
            state_machine: StateMachine::Paused,
            animation_factor: 1.0,
            angular_velocity: 0.0,
            motion_vector: Vector3::new(0.0, 0.0, 0.0),
            body_translation: Translation3::new(0.0, 0.0, 0.0),
            // Correct: only the unchecked constructor is const, and a scalar of 1 with imaginary
            // units set to 0 represents no rotation.
            body_rotation: Unit::new_unchecked(Quaternion::new(1.0, 0.0, 0.0, 0.0)),
            leg_radius: 0.0,
            battery_level: 0,
            battery_update_interval_ms: 0,
            joints: None,
        }
    }

    pub fn joint(&self, leg: usize, joint: usize) -> Option<&J> {
        self.joints.as_ref()?.get(leg)?.get(joint)
    }

    pub fn joint_mut(&mut self, leg: usize, joint: usize) -> Option<&mut J> {
        self.joints.as_mut()?.get_mut(leg)?.get_mut(joint)
    }
}

impl<J, const NUM_SERVOS_PER_LEG: usize, const NUM_LEGS: usize> Default
    for RobotState<J, NUM_SERVOS_PER_LEG, NUM_LEGS>
{
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum StateMachine {
    /// The robot is not moving.
    Paused = 1,
    /// The robot is homing.
    Homing = 2,
    /// The robot is calibrating.
    Calibrating = 3,
    /// The robot is running a simple walking loop.
    Looping = 4,
    /// The robot is exploring its environment.
    Exploring = 5,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SMConversionErr {
    BadValue(u8),
}

impl TryFrom<u8> for StateMachine {
    type Error = SMConversionErr;
    fn try_from(val: u8) -> Result<Self, Self::Error> {
        Ok(match val {
            1 => Self::Paused,
            2 => Self::Homing,
            3 => Self::Calibrating,
            4 => Self::Looping,
            5 => Self::Exploring,
            _ => return Err(SMConversionErr::BadValue(val)),
        })
    }
}

impl From<StateMachine> for u8 {
    fn from(val: StateMachine) -> Self {
        match val {
            StateMachine::Paused => 1,
            StateMachine::Homing => 2,
            StateMachine::Calibrating => 3,
            StateMachine::Looping => 4,
            StateMachine::Exploring => 5,
        }
    }
}
