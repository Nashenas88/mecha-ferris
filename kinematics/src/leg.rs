use crate::lut;

use core::marker::PhantomData;

use nalgebra::{Matrix4, Point3, RealField, Rotation3, Translation3, Vector3};

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

#[derive(Copy, Clone)]
pub struct LutMath;
const LUT_MUL: f32 = 2048.0 / core::f32::consts::PI;

impl ExpensiveMath<f32> for LutMath {
    fn atan2(l: f32, r: f32) -> f32 {
        let y = (l + 256.0).clamp(0.0, 511.0);
        let x = (r + 256.0).clamp(0.0, 511.0);
        lut::ATAN2[y as usize][x as usize]
        // let map = lut::ATAN2[y as usize][x as usize];
        // let left_y = y - 0.5;
        // let right_y = y + 0.5;
        // let left_x = x - 0.5;
        // let right_x = x + 0.5;
        // let ll = lut::ATAN2[left_y as usize][left_x as usize];
        // let lr = lut::ATAN2[left_y as usize][right_x as usize];
        // let rr = lut::ATAN2[right_y as usize][right_x as usize];
        // let rl = lut::ATAN2[right_y as usize][left_x as usize];
        // let mapped_left_x = ll * (left_y - y).abs() + rl * (right_y - y).abs();
        // let mapped_right_x = lr * (left_y - y).abs() + rr * (right_y - y).abs();
        // let lerp = mapped_left_x * (left_x - x).abs() + mapped_right_x * (right_x - x).abs();
        // println!("atan2({l},{r}) = {map} vs {lerp} vs {}", l.atan2(r));
        // lerp
    }

    #[inline(always)]
    fn acos(f: f32) -> f32 {
        let idx = (((f + core::f32::consts::PI) * LUT_MUL) as usize).clamp(0, 4096);
        lut::ACOS[idx]
    }

    #[inline(always)]
    fn sin(f: f32) -> f32 {
        let idx = (((f + core::f32::consts::PI) * LUT_MUL) as usize).clamp(0, 4095);
        lut::SIN[idx]
        // float_funcs::fsin(f)
    }

    #[inline(always)]
    fn cos(f: f32) -> f32 {
        let idx = (((f + core::f32::consts::PI) * LUT_MUL) as usize).clamp(0, 4095);
        lut::COS[idx]
        // float_funcs::fcos(f)
    }

    #[inline(always)]
    fn sincos(f: f32) -> (f32, f32) {
        let idx = (((f + core::f32::consts::PI) * LUT_MUL) as usize).clamp(0, 4095);
        (crate::lut::SIN[idx], lut::COS[idx])
    }

    #[inline(always)]
    fn sqrt(f: f32) -> f32 {
        // let mut tmp: i32 = f.to_bits() as i32;
        // tmp -= 127 << 23; // Remove IEEE bias from exponent (-2^23)
        // // tmp is now an appoximation to logbase2(val)
        // tmp >>= 1; // divide by 2
        // tmp += 127 >> 23; // restore the IEEE bias from the exponent (+2^23)
        // let res = f32::from_bits(tmp as u32);
        let mut y = f;
        let mut i: u32 = y.to_bits();
        i = 0x5F375A86_u32.wrapping_sub(i >> 1);
        y = f32::from_bits(i);
        1.0 / (y * (1.5 - (f * 0.5 * y * y)))
    }
}

// Used to pull in dbg
#[cfg(test)]
extern crate std;

#[derive(Copy, Clone)]
pub struct DefaultConsts;
impl LegConsts for DefaultConsts {
    const COXA_HEIGHT: f32 = 90.0;
    const COXA_LENGTH: f32 = 54.5;
    const FEMUR_LENGTH: f32 = 59.5;
    // const TIBIA_LENGTH: f32 = 175.0;
    const TIBIA_LENGTH: f32 = 185.0;
}

#[derive(Copy, Clone)]
pub struct Leg<F, T, C = DefaultConsts> {
    coxa_servo_angle: F,
    femur_servo_angle: F,
    tibia_servo_angle: F,
    _phantom: PhantomData<(C, T)>,
}

pub trait LegConsts {
    const COXA_HEIGHT: f32;
    const COXA_LENGTH: f32;
    const FEMUR_LENGTH: f32;
    const TIBIA_LENGTH: f32;
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
    F: From<f32> + Copy + RealField,
    T: ExpensiveMath<F>,
    C: LegConsts,
{
    const FL2: f32 = <C as LegConsts>::FEMUR_LENGTH * <C as LegConsts>::FEMUR_LENGTH;
    const TL2: f32 = <C as LegConsts>::TIBIA_LENGTH * <C as LegConsts>::TIBIA_LENGTH;
    const TF2: f32 = 2.0 * <C as LegConsts>::FEMUR_LENGTH * <C as LegConsts>::TIBIA_LENGTH;
    const MAX_LEG_LEN2: f32 = (<C as LegConsts>::FEMUR_LENGTH + <C as LegConsts>::TIBIA_LENGTH)
        * (<C as LegConsts>::FEMUR_LENGTH + <C as LegConsts>::TIBIA_LENGTH);
    const MIN_LEG_LEN2: f32 = (<C as LegConsts>::TIBIA_LENGTH - <C as LegConsts>::FEMUR_LENGTH)
        * (<C as LegConsts>::TIBIA_LENGTH - <C as LegConsts>::FEMUR_LENGTH);

    pub fn go_to(&mut self, target: Point3<F>) -> Result<(), LegError<F>> {
        let coxa_angle = T::atan2(target.x, target.z);
        let origin_to_femur_servo = Self::coxa_to_femur(coxa_angle);
        let femur_point = origin_to_femur_servo.transform_point(&target);

        let distance_squared = femur_point.z * femur_point.z + femur_point.x * femur_point.x;
        if distance_squared > F::from(Self::MAX_LEG_LEN2) {
            return Err(LegError::TooFar(
                distance_squared,
                F::from(Self::MAX_LEG_LEN2),
            ));
        } else if distance_squared < F::from(Self::MIN_LEG_LEN2) {
            return Err(LegError::TooClose(
                distance_squared,
                F::from(Self::MIN_LEG_LEN2),
            ));
        }

        let tibia_angle = T::acos(
            (distance_squared - F::from(Self::FL2) - F::from(Self::TL2)) / F::from(Self::TF2),
        );
        if !tibia_angle.is_finite() {
            return Err(LegError::NonFiniteCalculation);
        }
        let (tibia_sin, tibia_cos) = T::sincos(tibia_angle);
        let femur_angle = T::atan2(femur_point.x, femur_point.z)
            - T::atan2(
                F::from(<C as LegConsts>::TIBIA_LENGTH) * tibia_sin,
                F::from(<C as LegConsts>::FEMUR_LENGTH)
                    + F::from(<C as LegConsts>::TIBIA_LENGTH) * tibia_cos,
            );
        if !femur_angle.is_finite() {
            return Err(LegError::NonFiniteCalculation);
        }

        self.coxa_servo_angle = coxa_angle;
        self.femur_servo_angle = femur_angle;
        self.tibia_servo_angle = tibia_angle;
        Ok(())
    }

    /// Returns a matrix that transforms a point in coxa space to one in body space
    pub fn coxa_to_origin() -> Matrix4<F> {
        Translation3::new(
            F::from(0.0),
            F::from(<C as LegConsts>::COXA_HEIGHT),
            F::from(0.0),
        )
        .to_homogeneous()
    }

    /// Returns a matrix that transforms a point in body space to one in coxa space
    pub fn origin_to_coxa() -> Matrix4<F> {
        Translation3::new(
            F::from(0.0),
            F::from(-<C as LegConsts>::COXA_HEIGHT),
            F::from(0.0),
        )
        .to_homogeneous()
    }

    /// Returns a matrix that transforms a point in femur space to one in coxa space
    pub fn femur_to_coxa(coxa_angle: F) -> Matrix4<F> {
        Rotation3::from_axis_angle(&Vector3::y_axis(), coxa_angle).to_homogeneous()
            * Translation3::new(
                F::from(0.0),
                F::from(0.0),
                F::from(<C as LegConsts>::COXA_LENGTH),
            )
            .to_homogeneous()
            * Rotation3::from_axis_angle(&Vector3::z_axis(), F::from(-core::f32::consts::FRAC_PI_2))
                .to_homogeneous()
    }

    /// Returns a matrix that transforms a point in coxa space to one in femur space
    pub fn coxa_to_femur(coxa_angle: F) -> Matrix4<F> {
        Rotation3::from_axis_angle(&Vector3::z_axis(), F::from(core::f32::consts::FRAC_PI_2))
            .to_homogeneous()
            * Translation3::new(
                F::from(0.0),
                F::from(0.0),
                F::from(-<C as LegConsts>::COXA_LENGTH),
            )
            .to_homogeneous()
            * Rotation3::from_axis_angle(&Vector3::y_axis(), -coxa_angle).to_homogeneous()
    }

    /// Returns a matrix that transforms a point in tibia space to one in femur space
    #[allow(dead_code)]
    pub fn tibia_to_femur(femur_angle: F) -> Matrix4<F> {
        Rotation3::from_axis_angle(&Vector3::y_axis(), femur_angle).to_homogeneous()
            * Translation3::new(
                F::from(0.0),
                F::from(0.0),
                F::from(<C as LegConsts>::FEMUR_LENGTH),
            )
            .to_homogeneous()
    }

    /// Returns a matrix that transforms a point in foot space to one in tibia space
    #[allow(dead_code)]
    pub fn foot_to_tibia(tibia_angle: F) -> Matrix4<F> {
        Rotation3::from_axis_angle(&Vector3::y_axis(), tibia_angle).to_homogeneous()
            * Translation3::new(
                F::from(0.0),
                F::from(0.0),
                F::from(<C as LegConsts>::TIBIA_LENGTH),
            )
            .to_homogeneous()
    }
}

#[cfg(test)]
mod leg_tests;
