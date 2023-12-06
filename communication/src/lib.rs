#![cfg_attr(not(test), no_std)]

#[cfg(feature = "defmt")]
use defmt::{write, Format};
use nalgebra::{Translation3, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use state::StateMachine;

pub const COMMS_ADDR: u16 = 0x69;

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub enum I2cRequest {
    ChangeState(StateMachine),
    GetState,
    SetAnimationFactor(f32),
    GetAnimationFactor,
    SetAngularVelocity(f32),
    GetAngularVelocity,
    SetMotionVector(Vector3<f32>),
    GetMotionVector,
    SetBodyTranslation(Translation3<f32>),
    GetBodyTranslation,
    SetBodyRotation(UnitQuaternion<f32>),
    GetBodyRotation,
    SetLegRadius(f32),
    GetLegRadius,
    SetBatteryUpdateInterval(u32),
    GetBatteryLevel,
    SetCalibration {
        leg: u8,
        joint: u8,
        kind: u8,
        pulse: f32,
    },
    GetCalibration {
        leg: u8,
        joint: u8,
        kind: u8,
    },
}

impl I2cRequest {
    pub fn is_get(&self) -> bool {
        match self {
            I2cRequest::GetState
            | I2cRequest::GetAnimationFactor
            | I2cRequest::GetAngularVelocity
            | I2cRequest::GetMotionVector
            | I2cRequest::GetBodyTranslation
            | I2cRequest::GetBodyRotation
            | I2cRequest::GetLegRadius
            | I2cRequest::GetBatteryLevel
            | I2cRequest::GetCalibration { .. } => true,
            I2cRequest::ChangeState(_)
            | I2cRequest::SetAnimationFactor(_)
            | I2cRequest::SetAngularVelocity(_)
            | I2cRequest::SetMotionVector(..)
            | I2cRequest::SetBodyTranslation(..)
            | I2cRequest::SetBodyRotation(..)
            | I2cRequest::SetLegRadius(_)
            | I2cRequest::SetBatteryUpdateInterval(_)
            | I2cRequest::SetCalibration { .. } => false,
        }
    }

    /// Returns true if self and other are the same variant.
    pub fn is_same_kind(&self, other: &Self) -> bool {
        match (self, other) {
            (I2cRequest::GetState, I2cRequest::GetState)
            | (I2cRequest::ChangeState(_), I2cRequest::ChangeState(_))
            | (I2cRequest::SetAnimationFactor(_), I2cRequest::SetAnimationFactor(_))
            | (I2cRequest::GetAnimationFactor, I2cRequest::GetAnimationFactor)
            | (I2cRequest::SetAngularVelocity(_), I2cRequest::SetAngularVelocity(_))
            | (I2cRequest::GetAngularVelocity, I2cRequest::GetAngularVelocity)
            | (I2cRequest::SetMotionVector(_), I2cRequest::SetMotionVector(_))
            | (I2cRequest::GetMotionVector, I2cRequest::GetMotionVector)
            | (I2cRequest::SetBodyTranslation(_), I2cRequest::SetBodyTranslation(_))
            | (I2cRequest::GetBodyTranslation, I2cRequest::GetBodyTranslation)
            | (I2cRequest::SetBodyRotation(_), I2cRequest::SetBodyRotation(_))
            | (I2cRequest::GetBodyRotation, I2cRequest::GetBodyRotation)
            | (I2cRequest::SetLegRadius(_), I2cRequest::SetLegRadius(_))
            | (I2cRequest::GetLegRadius, I2cRequest::GetLegRadius)
            | (I2cRequest::SetBatteryUpdateInterval(_), I2cRequest::SetBatteryUpdateInterval(_))
            | (I2cRequest::GetBatteryLevel, I2cRequest::GetBatteryLevel)
            | (I2cRequest::SetCalibration { .. }, I2cRequest::SetCalibration { .. })
            | (I2cRequest::GetCalibration { .. }, I2cRequest::GetCalibration { .. }) => true,
            _ => false,
        }
    }
}

#[cfg(feature = "defmt")]
impl Format for I2cRequest {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            I2cRequest::GetState => write!(fmt, "GetState"),
            I2cRequest::ChangeState(sm) => write!(fmt, "ChangeState({})", sm),
            I2cRequest::SetAnimationFactor(f) => write!(fmt, "SetAnimationFactor({})", f),
            I2cRequest::GetAnimationFactor => write!(fmt, "GetAnimationFactor"),
            I2cRequest::SetAngularVelocity(v) => write!(fmt, "SetAngularVelocity({})", v),
            I2cRequest::GetAngularVelocity => write!(fmt, "GetAngularVelocity"),
            I2cRequest::SetMotionVector(v) => {
                write!(fmt, "SetMotionVector(Vector3({}, {}, {}))", v.x, v.y, v.z)
            }
            I2cRequest::GetMotionVector => write!(fmt, "GetMotionVector"),
            I2cRequest::SetBodyTranslation(t) => write!(
                fmt,
                "SetBodyTranslation(Translation3({}, {}, {}))",
                t.x, t.y, t.z
            ),
            I2cRequest::GetBodyTranslation => write!(fmt, "GetBodyTranslation"),
            I2cRequest::SetBodyRotation(r) => write!(
                fmt,
                "SetBodyRotation(UnitQuaternion({}, {}, {}, {}))",
                r.i, r.j, r.k, r.w
            ),
            I2cRequest::GetBodyRotation => write!(fmt, "GetBodyRotation"),
            I2cRequest::SetLegRadius(r) => write!(fmt, "SetLegRadius({})", r),
            I2cRequest::GetLegRadius => write!(fmt, "GetLegRadius"),
            I2cRequest::SetBatteryUpdateInterval(i) => {
                write!(fmt, "SetBatteryUpdateInterval({})", i)
            }
            I2cRequest::GetBatteryLevel => write!(fmt, "GetBatteryLevel"),
            I2cRequest::SetCalibration {
                leg,
                joint,
                kind,
                pulse,
            } => write!(
                fmt,
                "SetCalibration(leg: {}, joint: {}, kind: {}, pulse: {})",
                leg, joint, kind, pulse
            ),
            I2cRequest::GetCalibration { leg, joint, kind } => write!(
                fmt,
                "GetCalibration(leg: {}, joint: {}, kind: {})",
                leg, joint, kind
            ),
        }
    }
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SetRes {
    Ok = 0,
    InvalidIndex = 1,
    InvalidValue = 2,
    InvalidKind = 3,
    InvalidLeg = 4,
    InvalidJoint = 5,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub enum I2cResponse {
    GetState(StateMachine),
    GetAnimationFactor(f32),
    GetAngularVelocity(f32),
    GetMotionVector(Vector3<f32>),
    GetBodyTranslation(Translation3<f32>),
    GetBodyRotation(UnitQuaternion<f32>),
    GetLegRadius(f32),
    #[serde(with = "postcard::fixint::le")]
    GetBatteryLevel(u32),
    SetCalibration {
        leg: u8,
        joint: u8,
        kind: u8,
        pulse: f32,
        res: SetRes,
    },
    GetCalibration {
        leg: u8,
        joint: u8,
        kind: u8,
        pulse: f32,
    },
}

#[cfg(feature = "defmt")]
impl Format for I2cResponse {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            I2cResponse::GetState(sm) => write!(fmt, "GetState({})", sm),
            I2cResponse::GetAnimationFactor(f) => write!(fmt, "GetAnimationFactor({})", f),
            I2cResponse::GetAngularVelocity(v) => write!(fmt, "GetAngularVelocity({})", v),
            I2cResponse::GetMotionVector(v) => {
                write!(fmt, "GetMotionVector(Vector3({}, {}, {}))", v.x, v.y, v.z)
            }
            I2cResponse::GetBodyTranslation(t) => write!(
                fmt,
                "GetBodyTranslation(Translation3({}, {}, {}))",
                t.x, t.y, t.z
            ),
            I2cResponse::GetBodyRotation(r) => write!(
                fmt,
                "GetBodyRotation(UnitQuaternion({}, {}, {}, {}))",
                r.i, r.j, r.k, r.w
            ),
            I2cResponse::GetLegRadius(r) => write!(fmt, "GetLegRadius({})", r),
            I2cResponse::GetBatteryLevel(l) => write!(fmt, "GetBatteryLevel({})", l),
            I2cResponse::SetCalibration {
                leg,
                joint,
                kind,
                pulse,
                res,
            } => write!(
                fmt,
                "SetCalibration((leg: {}, joint: {}, kind: {}, pulse: {}, res: {:?})",
                leg, joint, kind, pulse, res
            ),
            I2cResponse::GetCalibration {
                leg,
                joint,
                kind,
                pulse,
            } => write!(
                fmt,
                "GetCalibration(leg: {}, joint: {}, kind: {}, pulse: {})",
                leg, joint, kind, pulse
            ),
        }
    }
}
