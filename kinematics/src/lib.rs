#![no_std]

mod leg;
mod lut;
pub use leg::{DefaultConsts, ExpensiveMath, Leg, LegConsts, LutMath};
pub use nalgebra::{
    self, ComplexField, Matrix4, Point3, RealField, Rotation3, Scalar, Scale3, Translation3, Unit,
    Vector3,
};
