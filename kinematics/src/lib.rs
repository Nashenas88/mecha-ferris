#![cfg_attr(not(feature = "std"), no_std)]

pub mod animation;
mod leg;
pub mod lut;
pub mod walking;

pub use leg::{DefaultConsts, ExpensiveMath, Leg, LegConsts, LegError};
pub use nalgebra::{
    self, ComplexField, Matrix4, Point3, RealField, Rotation3, Scalar, Scale3, Translation3, Unit,
    Vector3,
};
