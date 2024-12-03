use core::marker::PhantomData;
use nalgebra::{Matrix4, Point3, Rotation3, Vector3};

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

macro_rules! make_const_impls {
    ($($t:ident),*) => {
        $(
            paste::paste! {
                const fn [<coxa_length_translation_ $t>]<T>() -> Matrix4<$t>
                where
                    T: LegConsts<$t>,
                {
                    Matrix4::new(1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, T::COXA_LENGTH,
                        0.0, 0.0, 0.0, 1.0)
                }

                const fn [<femur_length_translation_ $t>]<T>() -> Matrix4<$t>
                where
                    T: LegConsts<$t>,
                {
                    Matrix4::new(1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, T::FEMUR_LENGTH,
                        0.0, 0.0, 0.0, 1.0)
                }

                const fn [<tibia_length_translation_ $t>]<T>() -> Matrix4<$t>
                where
                    T: LegConsts<$t>,
                {
                    Matrix4::new(1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, T::TIBIA_LENGTH,
                        0.0, 0.0, 0.0, 1.0)
                }

                const fn [<fl2_ $t>]<T>() -> $t
                where
                    T: LegConsts<$t>,
                {
                    T::FEMUR_LENGTH * T::FEMUR_LENGTH
                }

                const fn [<tl2_ $t>]<T>() -> $t
                where
                    T: LegConsts<$t>,
                {
                    T::TIBIA_LENGTH * T::TIBIA_LENGTH
                }

                const fn [<tf2_ $t>]<T>() -> $t
                where
                    T: LegConsts<$t>,
                {
                    2.0 * T::FEMUR_LENGTH * T::TIBIA_LENGTH
                }

                const fn [<max_leg_len2_ $t>]<T>() -> $t
                where
                    T: LegConsts<$t>,
                {
                    (T::FEMUR_LENGTH + T::TIBIA_LENGTH) * (T::FEMUR_LENGTH + T::TIBIA_LENGTH)
                }

                const fn [<min_leg_len2_ $t>]<T>() -> $t
                where
                    T: LegConsts<$t>,
                {
                    (T::TIBIA_LENGTH - T::FEMUR_LENGTH) * (T::TIBIA_LENGTH - T::FEMUR_LENGTH)
                }


                impl<T, C> Leg<$t, T, C>
                where
                    T: ExpensiveMath<$t>,
                    C: LegConsts<$t>,
                {
                    pub fn go_to(&mut self, target: Point3<$t>) -> Result<(), LegError<$t>> {
                        // TODO origin to center.
                        let coxa_angle = T::atan2(target.x, target.z);
                        let center_to_femur_servo = Self::coxa_to_femur(coxa_angle);
                        let femur_point = center_to_femur_servo.transform_point(&target);

                        let distance_squared = femur_point.z * femur_point.z + femur_point.x * femur_point.x;
                        if distance_squared > [<max_leg_len2_ $t>]::<C>() {
                            return Err(LegError::TooFar(distance_squared, [<max_leg_len2_ $t>]::<C>()));
                        } else if distance_squared < [<min_leg_len2_ $t>]::<C>() {
                            return Err(LegError::TooClose(
                                distance_squared,
                                [<min_leg_len2_ $t>]::<C>(),
                            ));
                        }

                        let tibia_angle =
                            T::acos((distance_squared - [<fl2_ $t>]::<C>() - [<tl2_ $t>]::<C>()) / [<tf2_ $t>]::<C>());
                        if !tibia_angle.is_finite() {
                            return Err(LegError::NonFiniteCalculation);
                        }
                        let (tibia_sin, tibia_cos) = T::sincos(tibia_angle);
                        let femur_angle = T::atan2(femur_point.x, femur_point.z)
                            - T::atan2(
                                <C as LegConsts<$t>>::TIBIA_LENGTH * tibia_sin,
                                <C as LegConsts<$t>>::FEMUR_LENGTH
                                    + <C as LegConsts<$t>>::TIBIA_LENGTH * tibia_cos,
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
                    pub fn femur_to_coxa(coxa_angle: $t) -> Matrix4<$t> {
                        Rotation3::from_axis_angle(&Vector3::y_axis(), coxa_angle).to_homogeneous()
                            * [<coxa_length_translation_ $t>]::<C>()
                            * Rotation3::from_axis_angle(&Vector3::z_axis(), -::core::$t::consts::FRAC_PI_2)
                                .to_homogeneous()
                    }

                    /// Returns a matrix that transforms a point in coxa space to one in femur space
                    pub fn coxa_to_femur(coxa_angle: $t) -> Matrix4<$t> {
                        Rotation3::from_axis_angle(&Vector3::z_axis(), ::core::$t::consts::FRAC_PI_2).to_homogeneous()
                            * -[<coxa_length_translation_ $t>]::<C>()
                            * Rotation3::from_axis_angle(&Vector3::y_axis(), -coxa_angle).to_homogeneous()
                    }

                    /// Returns a matrix that transforms a point in tibia space to one in femur space
                    #[allow(dead_code)]
                    pub fn tibia_to_femur(femur_angle: $t) -> Matrix4<$t> {
                        Rotation3::from_axis_angle(&Vector3::y_axis(), femur_angle).to_homogeneous()
                            * [<femur_length_translation_ $t>]::<C>()
                    }

                    /// Returns a matrix that transforms a point in foot space to one in tibia space
                    #[allow(dead_code)]
                    pub fn foot_to_tibia(tibia_angle: $t) -> Matrix4<$t> {
                        Rotation3::from_axis_angle(&Vector3::y_axis(), tibia_angle).to_homogeneous()
                            * [<tibia_length_translation_ $t>]::<C>()
                    }
                }
            }
        )*
    };
}

make_const_impls!(f32, f64);

#[cfg(test)]
mod leg_tests;
