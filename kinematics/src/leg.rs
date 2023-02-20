use core::marker::PhantomData;
use core::ops::{Add, Mul, Neg, Sub};

use nalgebra::{Matrix4, Point3, RealField, Rotation3, Translation3, Vector3};
use num_traits::FloatConst;

#[allow(dead_code)]
struct Point<F> {
    x: F,
    y: F,
    z: F,
}

pub trait ExpensiveMath<F> {
    fn atan2(l: F, r: F) -> F;
    fn acos(f: F) -> F;
    fn sin(f: F) -> F;
    fn cos(f: F) -> F;
    fn sincos(f: F) -> (F, F);
    fn sqrt(f: F) -> F;
}

// Used to pull in dbg
#[cfg(test)]
extern crate std;

pub trait LegConsts<F> {
    const COXA_LENGTH: F;
    const FEMUR_LENGTH: F;
    const TIBIA_LENGTH: F;
}

#[derive(Copy, Clone)]
pub struct DefaultConsts;
impl LegConsts<f32> for DefaultConsts {
    const COXA_LENGTH: f32 = 54.5;
    const FEMUR_LENGTH: f32 = 59.5;
    const TIBIA_LENGTH: f32 = 175.0;
}

#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LegError<F> {
    // (Actual, Max)
    TooFar(F, F),
    // (Actual, Min)
    TooClose(F, F),
    NonFiniteCalculation,
}

pub struct Leg<F, T, C = DefaultConsts> {
    coxa_servo_angle: F,
    femur_servo_angle: F,
    tibia_servo_angle: F,
    _phantom: PhantomData<(C, T)>,
}

impl<F, T, C> Clone for Leg<F, T, C>
where
    F: Clone,
{
    fn clone(&self) -> Self {
        Self {
            coxa_servo_angle: self.coxa_servo_angle.clone(),
            femur_servo_angle: self.femur_servo_angle.clone(),
            tibia_servo_angle: self.tibia_servo_angle.clone(),
            _phantom: self._phantom,
        }
    }
}

impl<F, T, C> Copy for Leg<F, T, C> where F: Copy {}

impl<F, T, C> Default for Leg<F, T, C>
where
    F: Default,
{
    fn default() -> Self {
        Self {
            coxa_servo_angle: Default::default(),
            femur_servo_angle: Default::default(),
            tibia_servo_angle: Default::default(),
            _phantom: PhantomData,
        }
    }
}

impl<F, T, C> Leg<F, T, C>
where
    F: Default,
{
    pub fn new() -> Self {
        <Self as Default>::default()
    }
}

impl<F, T, C> Leg<F, T, C>
where
    F: Copy,
{
    pub fn coxa_servo_angle(&self) -> F {
        self.coxa_servo_angle
    }

    pub fn femur_servo_angle(&self) -> F {
        self.femur_servo_angle
    }

    pub fn tibia_servo_angle(&self) -> F {
        self.tibia_servo_angle
    }
}

pub trait Two {
    const TWO: Self;
}

impl Two for f32 {
    const TWO: f32 = 2.0;
}

impl<F, T, C> Leg<F, T, C>
where
    F: Copy
        + RealField
        + ~const Mul<Output = F>
        + ~const Add<Output = F>
        + ~const Sub<Output = F>
        + ~const Neg<Output = F>
        + Two
        + FloatConst,
    T: ExpensiveMath<F>,
    C: LegConsts<F>,
{
    const FL2: F = <C as LegConsts<F>>::FEMUR_LENGTH * <C as LegConsts<F>>::FEMUR_LENGTH;
    const TL2: F = <C as LegConsts<F>>::TIBIA_LENGTH * <C as LegConsts<F>>::TIBIA_LENGTH;
    const TF2: F =
        <F as Two>::TWO * <C as LegConsts<F>>::FEMUR_LENGTH * <C as LegConsts<F>>::TIBIA_LENGTH;
    const MAX_LEG_LEN2: F = (<C as LegConsts<F>>::FEMUR_LENGTH + <C as LegConsts<F>>::TIBIA_LENGTH)
        * (<C as LegConsts<F>>::FEMUR_LENGTH + <C as LegConsts<F>>::TIBIA_LENGTH);
    const MIN_LEG_LEN2: F = (<C as LegConsts<F>>::TIBIA_LENGTH - <C as LegConsts<F>>::FEMUR_LENGTH)
        * (<C as LegConsts<F>>::TIBIA_LENGTH - <C as LegConsts<F>>::FEMUR_LENGTH);

    pub fn go_to(&mut self, target: Point3<F>) -> Result<(), LegError<F>> {
        let coxa_angle = T::atan2(target.x, target.z);
        let origin_to_femur_servo = Self::coxa_to_femur(coxa_angle);
        let femur_point = origin_to_femur_servo.transform_point(&target);

        let distance_squared = femur_point.z * femur_point.z + femur_point.x * femur_point.x;
        if distance_squared > Self::MAX_LEG_LEN2 {
            return Err(LegError::TooFar(distance_squared, Self::MAX_LEG_LEN2));
        } else if distance_squared < Self::MIN_LEG_LEN2 {
            return Err(LegError::TooClose(distance_squared, Self::MIN_LEG_LEN2));
        }

        let tibia_angle = T::acos((distance_squared - Self::FL2 - Self::TL2) / Self::TF2);
        if !tibia_angle.is_finite() {
            return Err(LegError::NonFiniteCalculation);
        }
        let (tibia_sin, tibia_cos) = T::sincos(tibia_angle);
        let femur_angle = T::atan2(femur_point.x, femur_point.z)
            - T::atan2(
                <C as LegConsts<F>>::TIBIA_LENGTH * tibia_sin,
                <C as LegConsts<F>>::FEMUR_LENGTH + <C as LegConsts<F>>::TIBIA_LENGTH * tibia_cos,
            );
        if !femur_angle.is_finite() {
            return Err(LegError::NonFiniteCalculation);
        }

        self.coxa_servo_angle = coxa_angle;
        self.femur_servo_angle = femur_angle;
        self.tibia_servo_angle = tibia_angle;
        Ok(())
    }

    /// Returns a matrix that transforms a point in femur space to one in coxa space
    pub fn femur_to_coxa(coxa_angle: F) -> Matrix4<F> {
        Rotation3::from_axis_angle(&Vector3::y_axis(), coxa_angle).to_homogeneous()
            * Translation3::new(F::zero(), F::zero(), <C as LegConsts<F>>::COXA_LENGTH)
                .to_homogeneous()
            * Rotation3::from_axis_angle(&Vector3::z_axis(), -F::FRAC_PI_2()).to_homogeneous()
    }

    /// Returns a matrix that transforms a point in coxa space to one in femur space
    pub fn coxa_to_femur(coxa_angle: F) -> Matrix4<F> {
        Rotation3::from_axis_angle(&Vector3::z_axis(), F::FRAC_PI_2()).to_homogeneous()
            * Translation3::new(F::zero(), F::zero(), -<C as LegConsts<F>>::COXA_LENGTH)
                .to_homogeneous()
            * Rotation3::from_axis_angle(&Vector3::y_axis(), -coxa_angle).to_homogeneous()
    }

    /// Returns a matrix that transforms a point in tibia space to one in femur space
    #[allow(dead_code)]
    pub fn tibia_to_femur(femur_angle: F) -> Matrix4<F> {
        Rotation3::from_axis_angle(&Vector3::y_axis(), femur_angle).to_homogeneous()
            * Translation3::new(F::zero(), F::zero(), <C as LegConsts<F>>::FEMUR_LENGTH)
                .to_homogeneous()
    }

    /// Returns a matrix that transforms a point in foot space to one in tibia space
    #[allow(dead_code)]
    pub fn foot_to_tibia(tibia_angle: F) -> Matrix4<F> {
        Rotation3::from_axis_angle(&Vector3::y_axis(), tibia_angle).to_homogeneous()
            * Translation3::new(F::zero(), F::zero(), <C as LegConsts<F>>::TIBIA_LENGTH)
                .to_homogeneous()
    }
}

#[cfg(test)]
mod leg_tests;
