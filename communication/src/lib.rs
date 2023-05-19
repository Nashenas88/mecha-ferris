#![cfg_attr(not(test), no_std)]

use core::convert::Infallible;
use nalgebra::{Quaternion, Translation3, UnitQuaternion, Vector3, Vector4};
use state::StateMachine;

pub const COMMS_ADDR: u16 = 0x69;

#[derive(Copy, Clone, Debug)]
pub enum I2cRequest {
    ChangeState(StateMachine),
    SetAnimationFactor(f32),
    GetAnimationFactor,
    SetAngularVelocity(f32),
    GetAngularVelocity,
    SetMotionVector(f32, f32, f32),
    GetMotionVector,
    SetBodyTranslation(f32, f32, f32),
    GetBodyTranslation,
    SetBodyRotation(f32, f32, f32),
    GetBodyRotation,
    SetLegRadius(f32),
    GetLegRadius,
    SetBatteryUpdateInterval(u32),
    GetBatteryLevel,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2cRequestOp {
    ChangeState,
    Get(I2cRequestField),
    Set(I2cRequestField),
    SetBatteryUpdateInterval,
    GetBatteryLevel,
}

// Only used for satisfying SgSet.
impl Default for I2cRequestOp {
    fn default() -> Self {
        Self::GetBatteryLevel
    }
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum I2cRequestField {
    AnimationFactor,
    AngularVelocity,
    MotionVector,
    BodyTranslation,
    BodyRotation,
    LegRadius,
}

// Only used for satisfying SgSet.
impl Default for I2cRequestField {
    fn default() -> Self {
        Self::AnimationFactor
    }
}

#[cfg_attr(test, derive(Debug))]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeErr {
    BadByte(u8),
}

pub trait Serialize {
    fn serialize(self, buffer: &mut [u8]) -> usize;
}

impl Serialize for f32 {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        assert!(buffer.len() >= 4);
        let bytes = self.to_le_bytes();
        buffer[..4].copy_from_slice(&bytes);
        4
    }
}

impl Serialize for (f32, f32, f32) {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        assert!(buffer.len() >= 12);
        let bytes = self.0.to_le_bytes();
        buffer[..4].copy_from_slice(&bytes);
        let bytes = self.1.to_le_bytes();
        buffer[4..8].copy_from_slice(&bytes);
        let bytes = self.2.to_le_bytes();
        buffer[8..12].copy_from_slice(&bytes);
        12
    }
}

impl Serialize for (f32, f32, f32, f32) {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        assert!(buffer.len() >= 16);
        let bytes = self.0.to_le_bytes();
        buffer[..4].copy_from_slice(&bytes);
        let bytes = self.1.to_le_bytes();
        buffer[4..8].copy_from_slice(&bytes);
        let bytes = self.2.to_le_bytes();
        buffer[8..12].copy_from_slice(&bytes);
        let bytes = self.3.to_le_bytes();
        buffer[12..16].copy_from_slice(&bytes);
        16
    }
}

impl Serialize for u32 {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        assert!(buffer.len() >= 4);
        let bytes = self.to_le_bytes();
        buffer[..4].copy_from_slice(&bytes);
        4
    }
}

impl Serialize for Vector3<f32> {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        (self.x, self.y, self.z).serialize(buffer)
    }
}

impl Serialize for Vector4<f32> {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        (self.x, self.y, self.z, self.w).serialize(buffer)
    }
}

impl Serialize for Translation3<f32> {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        (self.x, self.y, self.z).serialize(buffer)
    }
}

impl Serialize for UnitQuaternion<f32> {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        self.as_vector().serialize(buffer)
    }
}

pub trait Deserialize: Sized {
    type Input<'a>;
    type Error;
    fn deserialize(data: Self::Input<'_>) -> Result<Self, Self::Error>;
}

impl Deserialize for f32 {
    type Input<'a> = &'a [u8];
    type Error = Infallible;

    fn deserialize(data: Self::Input<'_>) -> Result<Self, Self::Error> {
        let mut byte = [0; 4];
        byte.copy_from_slice(&data[..4]);
        Ok(f32::from_le_bytes(byte))
    }
}

impl Deserialize for (f32, f32, f32) {
    type Input<'a> = &'a [u8];
    type Error = Infallible;

    fn deserialize(data: Self::Input<'_>) -> Result<Self, Self::Error> {
        let mut word = [0; 4];
        word.copy_from_slice(&data[..4]);
        let a = f32::from_le_bytes(word);
        word.copy_from_slice(&data[4..8]);
        let b = f32::from_le_bytes(word);
        word.copy_from_slice(&data[8..12]);
        let c = f32::from_le_bytes(word);
        Ok((a, b, c))
    }
}

impl Deserialize for (f32, f32, f32, f32) {
    type Input<'a> = &'a [u8];
    type Error = Infallible;

    fn deserialize(data: Self::Input<'_>) -> Result<Self, Self::Error> {
        let mut word = [0; 4];
        word.copy_from_slice(&data[..4]);
        let a = f32::from_le_bytes(word);
        word.copy_from_slice(&data[4..8]);
        let b = f32::from_le_bytes(word);
        word.copy_from_slice(&data[8..12]);
        let c = f32::from_le_bytes(word);
        word.copy_from_slice(&data[12..16]);
        let d = f32::from_le_bytes(word);
        Ok((a, b, c, d))
    }
}

impl Deserialize for Vector3<f32> {
    type Input<'a> = &'a [u8];
    type Error = Infallible;

    fn deserialize(data: Self::Input<'_>) -> Result<Self, Self::Error> {
        let (x, y, z) = <(f32, f32, f32)>::deserialize(data)?;
        Ok(Vector3::new(x, y, z))
    }
}

impl Deserialize for Vector4<f32> {
    type Input<'a> = &'a [u8];
    type Error = Infallible;

    fn deserialize(data: Self::Input<'_>) -> Result<Self, Self::Error> {
        let (x, y, z, w) = <(f32, f32, f32, f32)>::deserialize(data)?;
        Ok(Vector4::new(x, y, z, w))
    }
}

impl Deserialize for Translation3<f32> {
    type Input<'a> = &'a [u8];
    type Error = Infallible;

    fn deserialize(data: Self::Input<'_>) -> Result<Self, Self::Error> {
        let (x, y, z) = <(f32, f32, f32)>::deserialize(data)?;
        Ok(Translation3::new(x, y, z))
    }
}

impl Deserialize for UnitQuaternion<f32> {
    type Input<'a> = &'a [u8];
    type Error = Infallible;

    fn deserialize(data: Self::Input<'_>) -> Result<Self, Self::Error> {
        let (i, j, k, w) = <(f32, f32, f32, f32)>::deserialize(data)?;
        Ok(UnitQuaternion::new_unchecked(Quaternion::from_vector(
            Vector4::new(i, j, k, w),
        )))
    }
}

impl Deserialize for u32 {
    type Input<'a> = &'a [u8];
    type Error = Infallible;

    fn deserialize(data: Self::Input<'_>) -> Result<Self, Self::Error> {
        let mut byte = [0; 4];
        byte.copy_from_slice(&data[..4]);
        Ok(u32::from_le_bytes(byte))
    }
}

impl Serialize for StateMachine {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        buffer[0] = match self {
            StateMachine::Paused => 0,
            StateMachine::Homing => 1,
            StateMachine::Calibrating => 2,
            StateMachine::Looping => 3,
            StateMachine::Exploring => 4,
        };
        1
    }
}

impl Deserialize for StateMachine {
    type Input<'a> = &'a [u8];
    type Error = DeErr;
    fn deserialize(data: &[u8]) -> Result<Self, DeErr> {
        Ok(match data[0] {
            0 => StateMachine::Paused,
            1 => StateMachine::Homing,
            2 => StateMachine::Calibrating,
            3 => StateMachine::Looping,
            4 => StateMachine::Exploring,
            _ => return Err(DeErr::BadByte(data[0])),
        })
    }
}

impl Serialize for I2cRequest {
    fn serialize(self, buf: &mut [u8]) -> usize {
        match self {
            I2cRequest::ChangeState(sm) => {
                I2cRequestOp::ChangeState.serialize(buf);
                sm.serialize(&mut buf[1..2]) + 1
            }
            I2cRequest::SetAnimationFactor(v) => {
                I2cRequestOp::Set(I2cRequestField::AnimationFactor).serialize(buf);
                buf[1..5].copy_from_slice(&v.to_le_bytes());
                5
            }
            I2cRequest::GetAnimationFactor => {
                I2cRequestOp::Get(I2cRequestField::AnimationFactor).serialize(buf)
            }
            I2cRequest::SetAngularVelocity(v) => {
                I2cRequestOp::Set(I2cRequestField::AngularVelocity).serialize(buf);
                buf[1..5].copy_from_slice(&v.to_le_bytes());
                5
            }
            I2cRequest::GetAngularVelocity => {
                I2cRequestOp::Get(I2cRequestField::AngularVelocity).serialize(buf)
            }
            I2cRequest::SetMotionVector(x, y, z) => {
                I2cRequestOp::Set(I2cRequestField::MotionVector).serialize(buf);
                buf[1..5].copy_from_slice(&x.to_le_bytes());
                buf[5..9].copy_from_slice(&y.to_le_bytes());
                buf[9..13].copy_from_slice(&z.to_le_bytes());
                13
            }
            I2cRequest::GetMotionVector => {
                I2cRequestOp::Get(I2cRequestField::MotionVector).serialize(buf)
            }
            I2cRequest::SetBodyTranslation(x, y, z) => {
                I2cRequestOp::Set(I2cRequestField::BodyTranslation).serialize(buf);
                buf[1..5].copy_from_slice(&x.to_le_bytes());
                buf[5..9].copy_from_slice(&y.to_le_bytes());
                buf[9..13].copy_from_slice(&z.to_le_bytes());
                13
            }
            I2cRequest::GetBodyTranslation => {
                I2cRequestOp::Get(I2cRequestField::BodyTranslation).serialize(buf)
            }
            I2cRequest::SetBodyRotation(y, p, r) => {
                I2cRequestOp::Set(I2cRequestField::BodyRotation).serialize(buf);
                buf[1..5].copy_from_slice(&y.to_le_bytes());
                buf[5..9].copy_from_slice(&p.to_le_bytes());
                buf[9..13].copy_from_slice(&r.to_le_bytes());
                13
            }
            I2cRequest::GetBodyRotation => {
                I2cRequestOp::Get(I2cRequestField::BodyRotation).serialize(buf)
            }
            I2cRequest::SetLegRadius(r) => {
                I2cRequestOp::Set(I2cRequestField::LegRadius).serialize(buf);
                buf[1..5].copy_from_slice(&r.to_le_bytes());
                5
            }
            I2cRequest::GetLegRadius => {
                I2cRequestOp::Get(I2cRequestField::LegRadius).serialize(buf)
            }
            I2cRequest::SetBatteryUpdateInterval(i) => {
                I2cRequestOp::SetBatteryUpdateInterval.serialize(buf);
                buf[1..5].copy_from_slice(&i.to_le_bytes());
                5
            }
            I2cRequest::GetBatteryLevel => I2cRequestOp::GetBatteryLevel.serialize(buf),
        }
    }
}

impl Deserialize for I2cRequest {
    type Input<'a> = (I2cRequestOp, &'a [u8]);
    type Error = DeErr;

    fn deserialize((op, data): Self::Input<'_>) -> Result<Self, Self::Error> {
        let mut buf = [0; 4];
        Ok(match op {
            I2cRequestOp::ChangeState => {
                I2cRequest::ChangeState(<StateMachine as Deserialize>::deserialize(&data[1..])?)
            }
            I2cRequestOp::Set(I2cRequestField::AnimationFactor) => {
                buf.copy_from_slice(&data[1..5]);
                I2cRequest::SetAnimationFactor(f32::from_le_bytes(buf))
            }
            I2cRequestOp::Get(I2cRequestField::AnimationFactor) => I2cRequest::GetAnimationFactor,
            I2cRequestOp::Set(I2cRequestField::AngularVelocity) => {
                buf.copy_from_slice(&data[1..5]);
                I2cRequest::SetAngularVelocity(f32::from_le_bytes(buf))
            }
            I2cRequestOp::Get(I2cRequestField::AngularVelocity) => I2cRequest::GetAngularVelocity,
            I2cRequestOp::Set(I2cRequestField::MotionVector) => {
                buf.copy_from_slice(&data[1..5]);
                let x = f32::from_le_bytes(buf);
                buf.copy_from_slice(&data[5..9]);
                let y = f32::from_le_bytes(buf);
                buf.copy_from_slice(&data[9..13]);
                let z = f32::from_le_bytes(buf);
                I2cRequest::SetMotionVector(x, y, z)
            }
            I2cRequestOp::Get(I2cRequestField::MotionVector) => I2cRequest::GetMotionVector,
            I2cRequestOp::Set(I2cRequestField::BodyTranslation) => {
                buf.copy_from_slice(&data[1..5]);
                let x = f32::from_le_bytes(buf);
                buf.copy_from_slice(&data[5..9]);
                let y = f32::from_le_bytes(buf);
                buf.copy_from_slice(&data[9..13]);
                let z = f32::from_le_bytes(buf);
                I2cRequest::SetBodyTranslation(x, y, z)
            }
            I2cRequestOp::Get(I2cRequestField::BodyTranslation) => I2cRequest::GetBodyTranslation,
            I2cRequestOp::Set(I2cRequestField::BodyRotation) => {
                buf.copy_from_slice(&data[1..5]);
                let p = f32::from_le_bytes(buf);
                buf.copy_from_slice(&data[5..9]);
                let y = f32::from_le_bytes(buf);
                buf.copy_from_slice(&data[9..13]);
                let r = f32::from_le_bytes(buf);
                I2cRequest::SetBodyRotation(p, y, r)
            }
            I2cRequestOp::Get(I2cRequestField::BodyRotation) => I2cRequest::GetBodyRotation,
            I2cRequestOp::SetBatteryUpdateInterval => {
                buf.copy_from_slice(&data[1..5]);
                I2cRequest::SetBatteryUpdateInterval(u32::from_le_bytes(buf))
            }
            I2cRequestOp::Set(I2cRequestField::LegRadius) => {
                buf.copy_from_slice(&data[1..5]);
                I2cRequest::SetLegRadius(f32::from_le_bytes(buf))
            }
            I2cRequestOp::Get(I2cRequestField::LegRadius) => I2cRequest::GetLegRadius,
            I2cRequestOp::GetBatteryLevel => I2cRequest::GetBatteryLevel,
        })
    }
}

impl Serialize for I2cRequestOp {
    fn serialize(self, buffer: &mut [u8]) -> usize {
        buffer[0] = match self {
            I2cRequestOp::ChangeState => 0,
            I2cRequestOp::Set(I2cRequestField::AnimationFactor) => 1,
            I2cRequestOp::Get(I2cRequestField::AnimationFactor) => 2,
            I2cRequestOp::Set(I2cRequestField::AngularVelocity) => 3,
            I2cRequestOp::Get(I2cRequestField::AngularVelocity) => 4,
            I2cRequestOp::Set(I2cRequestField::MotionVector) => 5,
            I2cRequestOp::Get(I2cRequestField::MotionVector) => 6,
            I2cRequestOp::Set(I2cRequestField::BodyTranslation) => 7,
            I2cRequestOp::Get(I2cRequestField::BodyTranslation) => 8,
            I2cRequestOp::Set(I2cRequestField::BodyRotation) => 9,
            I2cRequestOp::Get(I2cRequestField::BodyRotation) => 10,
            I2cRequestOp::Set(I2cRequestField::LegRadius) => 11,
            I2cRequestOp::Get(I2cRequestField::LegRadius) => 12,
            I2cRequestOp::SetBatteryUpdateInterval => 13,
            I2cRequestOp::GetBatteryLevel => 14,
        };
        1
    }
}

impl Deserialize for I2cRequestOp {
    type Input<'a> = &'a [u8];
    type Error = DeErr;

    fn deserialize(data: &[u8]) -> Result<Self, Self::Error> {
        Ok(match data[0] {
            0 => I2cRequestOp::ChangeState,
            1 => I2cRequestOp::Set(I2cRequestField::AnimationFactor),
            2 => I2cRequestOp::Get(I2cRequestField::AnimationFactor),
            3 => I2cRequestOp::Set(I2cRequestField::AngularVelocity),
            4 => I2cRequestOp::Get(I2cRequestField::AngularVelocity),
            5 => I2cRequestOp::Set(I2cRequestField::MotionVector),
            6 => I2cRequestOp::Get(I2cRequestField::MotionVector),
            7 => I2cRequestOp::Set(I2cRequestField::BodyTranslation),
            8 => I2cRequestOp::Get(I2cRequestField::BodyTranslation),
            9 => I2cRequestOp::Set(I2cRequestField::BodyRotation),
            10 => I2cRequestOp::Get(I2cRequestField::BodyRotation),
            11 => I2cRequestOp::Set(I2cRequestField::LegRadius),
            12 => I2cRequestOp::Get(I2cRequestField::LegRadius),
            13 => I2cRequestOp::SetBatteryUpdateInterval,
            14 => I2cRequestOp::GetBatteryLevel,
            _ => return Err(DeErr::BadByte(data[0])),
        })
    }
}

#[cfg(test)]
mod tests;
