use super::*;
use approx::{assert_relative_eq, AbsDiffEq, RelativeEq};
use proptest::proptest;

struct TestConsts;
impl LegConsts for TestConsts {
    const COXA_HEIGHT: f32 = 1.0;
    const COXA_LENGTH: f32 = 2.0;
    const FEMUR_LENGTH: f32 = 3.0;
    const TIBIA_LENGTH: f32 = 4.0;
}

impl AbsDiffEq for LegError {
    type Epsilon = f32;
    fn default_epsilon() -> Self::Epsilon {
        f32::default_epsilon()
    }

    fn abs_diff_eq(&self, rhs: &Self, epsilon: Self::Epsilon) -> bool {
        match (self, rhs) {
            (LegError::TooFar(l1, l2), LegError::TooFar(r1, r2))
            | (LegError::TooClose(l1, l2), LegError::TooClose(r1, r2)) => {
                <f32 as AbsDiffEq>::abs_diff_eq(l1, l2, epsilon)
                    && <f32 as AbsDiffEq>::abs_diff_eq(r1, r2, epsilon)
            }
            (LegError::NonFiniteCalculation, LegError::NonFiniteCalculation) => true,
            _ => false,
        }
    }
}

impl RelativeEq for LegError {
    fn default_max_relative() -> Self::Epsilon {
        f32::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        match (self, other) {
            (LegError::TooFar(l1, l2), LegError::TooFar(r1, r2))
            | (LegError::TooClose(l1, l2), LegError::TooClose(r1, r2)) => {
                <f32 as RelativeEq>::relative_eq(&l1, &r1, epsilon, max_relative)
                    && <f32 as RelativeEq>::relative_eq(&l2, &r2, epsilon, max_relative)
            }
            (LegError::NonFiniteCalculation, LegError::NonFiniteCalculation) => true,
            _ => false,
        }
    }
}

#[ignore]
#[test]
fn angle_too_close() {
    let mut leg = Leg::<TestConsts>::new();
    let result = leg
        .go_to(Point3::new(0.0, 0.0, 0.0))
        .expect_err("should fail");
    assert_relative_eq!(result, LegError::TooClose(0.58578634, 1.0))
}

#[test]
fn angle_too_far() {
    let mut leg = Leg::<TestConsts>::new();
    let result = leg
        .go_to(Point3::new(100.0, 100.0, 100.0))
        .expect_err("should fail");
    assert_relative_eq!(result, LegError::TooFar(170.99509, 7.0))
}

#[test]
fn non_finite_result() {
    // possible?
}

#[test]
fn go_to_point() {
    let mut leg = Leg::<TestConsts>::new();
    let result = leg.go_to(Point3::new(1.0, 1.0, 1.0));
    assert_eq!(result, Ok(()))
}

#[test]
fn test_reachable() {
    let mut runner = TestRunner::default();
}
