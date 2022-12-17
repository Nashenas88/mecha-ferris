use core::marker::PhantomData;

use nalgebra::{ComplexField, Matrix4, Point3, RealField, Rotation3, Translation3, Vector3};

// Used to pull in dbg
#[cfg(test)]
extern crate std;

pub struct DefaultConsts;
impl LegConsts for DefaultConsts {
    const COXA_HEIGHT: f32 = 90.0;
    const COXA_LENGTH: f32 = 54.5;
    const FEMUR_LENGTH: f32 = 59.5;
    const TIBIA_LENGTH: f32 = 145.0;
}

pub struct Leg<T = DefaultConsts> {
    coxa_servo_angle: f32,
    femur_servo_angle: f32,
    tibia_servo_angle: f32,
    _phantom: PhantomData<T>,
}

pub trait LegConsts {
    const COXA_HEIGHT: f32;
    const COXA_LENGTH: f32;
    const FEMUR_LENGTH: f32;
    const TIBIA_LENGTH: f32;
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LegError {
    // (Actual, Max)
    TooFar(f32, f32),
    // (Actual, Min)
    TooClose(f32, f32),
    NonFiniteCalculation,
}

impl<T> Leg<T> {
    pub fn new() -> Self {
        Leg {
            coxa_servo_angle: 0.0,
            femur_servo_angle: 0.0,
            tibia_servo_angle: 0.0,
            _phantom: PhantomData,
        }
    }

    pub fn coxa_servo_angle(&self) -> f32 {
        self.coxa_servo_angle
    }

    pub fn femur_servo_angle(&self) -> f32 {
        self.femur_servo_angle
    }

    pub fn tibia_servo_angle(&self) -> f32 {
        self.tibia_servo_angle
    }
}

impl<T> Leg<T>
where
    T: LegConsts,
{
    const FL2: f32 = <T as LegConsts>::FEMUR_LENGTH * <T as LegConsts>::FEMUR_LENGTH;
    const TL2: f32 = <T as LegConsts>::TIBIA_LENGTH * <T as LegConsts>::TIBIA_LENGTH;
    const TF2: f32 = 2.0 * <T as LegConsts>::FEMUR_LENGTH * <T as LegConsts>::TIBIA_LENGTH;
    const MAX_LEG_LEN: f32 = (<T as LegConsts>::FEMUR_LENGTH + <T as LegConsts>::TIBIA_LENGTH)
        * (<T as LegConsts>::FEMUR_LENGTH + <T as LegConsts>::TIBIA_LENGTH);
    const MIN_LEG_LEN: f32 = (<T as LegConsts>::TIBIA_LENGTH - <T as LegConsts>::FEMUR_LENGTH)
        * (<T as LegConsts>::TIBIA_LENGTH - <T as LegConsts>::FEMUR_LENGTH);

    pub fn go_to(&mut self, target: Point3<f32>) -> Result<(), LegError> {
        let coxa_angle = f32::atan2(target.x, target.z);
        let origin_to_femur_servo = Self::origin_to_coxa() * Self::coxa_to_femur(coxa_angle);
        let femur_point = origin_to_femur_servo
            .try_inverse()
            .unwrap()
            .transform_point(&target);

        let distance_squared = femur_point.z * femur_point.z + femur_point.x * femur_point.x;
        if distance_squared > Self::MAX_LEG_LEN {
            return Err(LegError::TooFar(
                distance_squared.sqrt(),
                Self::MAX_LEG_LEN.sqrt(),
            ));
        } else if distance_squared < Self::MIN_LEG_LEN {
            return Err(LegError::TooClose(
                distance_squared.sqrt(),
                Self::MIN_LEG_LEN.sqrt(),
            ));
        }

        let tibia_angle = f32::acos((distance_squared - Self::FL2 - Self::TL2) / Self::TF2);
        if !tibia_angle.is_finite() {
            return Err(LegError::NonFiniteCalculation);
        }
        let femur_angle = f32::atan2(femur_point.x, femur_point.z)
            - f32::atan2(
                <T as LegConsts>::TIBIA_LENGTH * tibia_angle.sin(),
                <T as LegConsts>::FEMUR_LENGTH + <T as LegConsts>::TIBIA_LENGTH * tibia_angle.cos(),
            );
        if !femur_angle.is_finite() {
            return Err(LegError::NonFiniteCalculation);
        }

        self.coxa_servo_angle = coxa_angle;
        self.femur_servo_angle = femur_angle;
        self.tibia_servo_angle = tibia_angle;
        Ok(())
    }

    pub fn origin_to_coxa() -> Matrix4<f32> {
        Translation3::new(0.0, <T as LegConsts>::COXA_HEIGHT, 0.0).to_homogeneous()
    }

    pub fn coxa_to_femur(coxa_angle: f32) -> Matrix4<f32> {
        Rotation3::from_axis_angle(&Vector3::y_axis(), coxa_angle).to_homogeneous()
            * Translation3::new(0.0, 0.0, <T as LegConsts>::COXA_LENGTH).to_homogeneous()
            * Rotation3::from_axis_angle(&Vector3::z_axis(), -core::f32::consts::FRAC_PI_2)
                .to_homogeneous()
    }

    #[allow(dead_code)]
    pub fn femur_to_tibia(femur_angle: f32) -> Matrix4<f32> {
        Rotation3::from_axis_angle(&Vector3::y_axis(), femur_angle).to_homogeneous()
            * Translation3::new(0.0, 0.0, <T as LegConsts>::FEMUR_LENGTH).to_homogeneous()
    }

    #[allow(dead_code)]
    pub fn tibia_to_foot(tibia_angle: f32) -> Matrix4<f32> {
        Rotation3::from_axis_angle(&Vector3::y_axis(), tibia_angle).to_homogeneous()
            * Translation3::new(0.0, 0.0, <T as LegConsts>::TIBIA_LENGTH).to_homogeneous()
    }
}

#[cfg(test)]
mod leg_tests;
