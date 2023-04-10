#![cfg_attr(not(feature = "std"), no_std)]
#![feature(const_ops, const_trait_impl)]

pub mod animation;
mod leg;
pub mod lut;
pub mod walking;

pub use leg::{DefaultConsts, ExpensiveMath, Leg, LegConsts, LegError, Two};
pub use nalgebra::{
    self, ComplexField, Matrix4, Point3, RealField, Rotation3, Scalar, Scale3, Translation3, Unit,
    Vector3,
};
